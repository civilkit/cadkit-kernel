// Rolling-ball round / chamfer oracle for the rim of an extruded profile, independent of the kernel's B-rep blend builders.
// See docs/fillet-oracle.md for the theory.
//
//   node tools/fillet_oracle.mjs --wasm <ck_wasm.wasm> --out <dir> [--cell 0.1] [--only blob|six] [--chamfer] [--selftest]
//
// Oracle: P = the profile (a polygon), H = the extrude height, flat cap at z = H.
//   D(x,y) = distance to the profile boundary (exact, to the polyline), Pr = {D >= r} (P eroded by r),
//   d(x,y) = distance to Pr (Pr empty: infinity).
//   round r:    z = H - r + sqrt(r^2 - d^2) if d <= r, else H - r
//   chamfer c:  z = H - min(d, c)            (Pc = P eroded by c; a 45 degree cone element)
// The kernel's own base solid is only used to read the profile (cap boundary of a fine tessellation) and to compare.
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

import { rasterise, distToEroded, heightfield, heightSwept } from './fillet_oracle_core.mjs';
export { rasterise, distToEroded, heightfield, heightSwept };

// ------------------------------------------------------------------ kernel side
const TYPES = { f32: Float32Array, i32: Int32Array };
function decode(r) {
  const jl = new DataView(r.bytes.buffer, r.bytes.byteOffset).getUint32(0, true);
  const base = (4 + jl + 7) & ~7, j = JSON.parse(r.json);
  const dec = (v) => {
    if (Array.isArray(v)) return v.map(dec);
    if (v && typeof v === 'object') {
      if ('$buf' in v) { const C = TYPES[v.type]; return new C(r.bytes.buffer.slice(r.bytes.byteOffset + base + v.offset, r.bytes.byteOffset + base + v.offset + v.len * C.BYTES_PER_ELEMENT)); }
      const o = {}; for (const [k, x] of Object.entries(v)) o[k] = dec(x); return o;
    }
    return v;
  };
  return dec(j);
}

export class Host {
  constructor(wasm) { this.k = new Kernel(new WebAssembly.Module(readFileSync(wasm))); }
  req(msg) { const r = this.k.request(JSON.stringify({ id: 1, msg })); if (r.trap) return { ok: false, error: 'trap ' + r.trap }; return decode(r); }
  regen(features) { return this.req({ type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features } }); }
  volume() { const m = this.req({ type: 'massProps', tag: 'main', bodies: true }); return m.result?.volume ?? null; }
  mesh(chord) { const m = this.req({ type: 'exportMesh', tag: 'main', chord_mm: chord }); return m.result; }
}

// cap boundary segments of the top (z = H) or bottom of an exported mesh
export function capBoundary(mesh, H) {
  const P = mesh.positions, nt = mesh.triangles;
  const key = (i) => `${Math.round(P[i] * 1e4)},${Math.round(P[i + 1] * 1e4)}`;
  const cnt = new Map();
  for (let t = 0; t < nt; t++) {
    let ok = true; for (let c = 0; c < 3; c++) if (Math.abs(P[t * 9 + c * 3 + 2] - H) > 1e-4) ok = false;
    if (!ok) continue;
    for (let c = 0; c < 3; c++) {
      const a = t * 9 + c * 3, b = t * 9 + ((c + 1) % 3) * 3, ka = key(a), kb = key(b), k = ka < kb ? ka + '|' + kb : kb + '|' + ka;
      const e = cnt.get(k); if (e) e.n++; else cnt.set(k, { n: 1, s: [P[a], P[a + 1], P[b], P[b + 1]] });
    }
  }
  return [...cnt.values()].filter((e) => e.n === 1).map((e) => e.s);
}

// kernel top surface height on the grid: max z over triangles that face up
export function sampleTop(mesh, g) {
  const P = mesh.positions, nt = mesh.triangles, { x0, y0, nx, ny, cell } = g;
  const z = new Float32Array(nx * ny).fill(-Infinity);
  for (let t = 0; t < nt; t++) {
    const o = t * 9, ax = P[o], ay = P[o + 1], az = P[o + 2], bx = P[o + 3], by = P[o + 4], bz = P[o + 5], cx = P[o + 6], cy = P[o + 7], cz = P[o + 8];
    const area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    if (area <= 1e-12) continue;              // vertical or facing down (counter-clockwise seen from +z means facing up)
    const i0 = Math.max(0, Math.floor((Math.min(ax, bx, cx) - x0) / cell - 0.5)), i1 = Math.min(nx - 1, Math.ceil((Math.max(ax, bx, cx) - x0) / cell - 0.5));
    const j0 = Math.max(0, Math.floor((Math.min(ay, by, cy) - y0) / cell - 0.5)), j1 = Math.min(ny - 1, Math.ceil((Math.max(ay, by, cy) - y0) / cell - 0.5));
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
      const px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
      const w0 = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / area, w1 = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / area, w2 = 1 - w0 - w1;
      if (w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9) continue;
      const zz = w0 * az + w1 * bz + w2 * cz;
      if (zz > z[j * nx + i]) z[j * nx + i] = zz;
    }
  }
  for (let i = 0; i < z.length; i++) if (z[i] === -Infinity) z[i] = NaN;
  return z;
}

// ------------------------------------------------------------------ comparison
export function compare(g, zo, zk, cell) {
  const { nx, ny, inside } = g;
  let n = 0, miss = 0, sum2 = 0, maxe = 0, at = -1; const errs = [];
  let maxFlat = 0, nFlat = 0, sumFlat2 = 0, volO = 0, volK = 0;
  for (let j = 1; j < ny - 1; j++) for (let i = 1; i < nx - 1; i++) {
    const p = j * nx + i; if (!inside[p]) continue;
    volO += zo[p] * cell * cell;
    if (Number.isNaN(zk[p])) { miss++; continue; }
    volK += zk[p] * cell * cell;
    const e = zk[p] - zo[p]; n++; sum2 += e * e; errs.push(Math.abs(e));
    if (Math.abs(e) > maxe) { maxe = Math.abs(e); at = p; }
    // where the oracle surface is not near-vertical (slope < 2): a small horizontal shift cannot make a large height error
    const sl = Math.max(Math.abs(zo[p + 1] - zo[p - 1]), Math.abs(zo[p + nx] - zo[p - nx])) / (2 * cell);
    if (sl < 2) { nFlat++; sumFlat2 += e * e; if (Math.abs(e) > maxFlat) maxFlat = Math.abs(e); }
  }
  errs.sort((a, b) => a - b);
  return { n, miss, maxAbs: maxe, rms: Math.sqrt(sum2 / Math.max(1, n)), p99: errs[Math.floor(errs.length * 0.99)] ?? 0, p999: errs[Math.floor(errs.length * 0.999)] ?? 0,
    maxFlat, rmsFlat: Math.sqrt(sumFlat2 / Math.max(1, nFlat)), nFlat, at: at < 0 ? null : [g.x0 + ((at % nx) + 0.5) * cell, g.y0 + (Math.floor(at / nx) + 0.5) * cell], area: volO };
}

// ------------------------------------------------------------------ cases
const BLOB = [[-12.9, 66.8], [61.2, 51.2], [71.3, -12.2], [21.4, 0.0], [-29.3, -24.8], [-71.8, -16.1], [-114.3, 13.5], [-72.7, 24.6], [-68.2, 59.1]];
const SIX = [[30, 0], [18, 22], [-8, 26], [-30, 6], [-14, -22], [12, -20]];
const CASES = {
  blob: { pts: BLOB, H: 25, radii: [1, 2, 3.7, 5, 7, 10, 15, 20, 24.9] },
  six: { pts: SIX, H: 20, radii: [0.5, 1, 2, 4, 6, 8, 10, 12, 14] },
  // a D: an elliptic arc (interpolating spline through 7 points, ends are the corners) closed by the chord; as blend_spline_corner.rs
  d: { dshape: { w: 60, hgt: 18, n: 8 }, H: 20, radii: [1, 2, 3, 4, 5, 6, 7] },
};

function partFor(c) {
  if (c.dshape) {
    const { w, hgt, n } = c.dshape, a = [];
    for (let i = 0; i <= n; i++) { const t = Math.PI * i / n; a.push([w / 2 * Math.cos(t), hgt * Math.sin(t)]); }
    const inner = a.slice(1, n).map((p, i) => [p[0], p[1] + (i % 2 === 1 ? 0.01 * hgt : 0)]);
    return [
      { type: 'sketch', id: 'F1', name: 'S', plane: { kind: 'std', id: 'front' },
        points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'c0', x: a[0][0], y: a[0][1] }, { id: 'c1', x: a[n][0], y: a[n][1] }, ...inner.map((p, j) => ({ id: `s0_${j}`, x: p[0], y: p[1] }))],
        entities: [{ id: 'e0', type: 'spline', pts: ['c0', ...inner.map((_, j) => `s0_${j}`), 'c1'] }, { id: 'e1', type: 'line', p1: 'c1', p2: 'c0' }], constraints: [] },
      { type: 'extrude', op: 'add', sketch: 'F1', end: 'blind', depth: c.H, id: 'F2' },
    ];
  }
  const pts = c.pts;
  return [
    { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, ...pts.map(([x, y], i) => ({ id: 'p' + i, x, y }))],
      entities: [{ id: 'e1', type: 'spline', pts: pts.map((_, i) => 'p' + i), closed: true }], constraints: [] },
    { type: 'extrude', op: 'add', sketch: 'F1', end: 'blind', depth: c.H, id: 'F2' },
  ];
}

function rimPicks(body, H) {
  return body.edges.filter((e) => e.pts?.length && e.pts.every((p) => Math.abs(p[2] - H) < 1e-6)).map((e) => ({ faces: [...e.faces], at: e.mid ?? e.pts[0] }));
}

export function main(argv) {
  const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
  const wasm = opt('--wasm', fileURLToPath(new URL('../../kit/apps/cad/public/kernel/ck_wasm.wasm', import.meta.url)));
  const out = opt('--out', null), cell = Number(opt('--cell', '0.1')), only = opt('--only', ''), chamfer = argv.includes('--chamfer');
  const radiiOpt = opt('--radii', null);
  const host = new Host(wasm);
  if (out) mkdirSync(out, { recursive: true });
  const rows = [];
  for (const [name, c] of Object.entries(CASES)) {
    if (only && name !== only) continue;
    const feats = partFor(c);
    const base = host.regen(feats);
    if (!base.ok || base.result?.errors?.length) { console.log(name, 'base failed', JSON.stringify(base).slice(0, 200)); continue; }
    const baseVol = host.volume();
    const segs = capBoundary(host.mesh(0.004), c.H);
    const g = rasterise(segs, cell);
    let area = 0; for (let i = 0; i < g.inside.length; i++) area += g.inside[i];
    area *= cell * cell;
    console.log(`${name}: H=${c.H} cell=${cell} grid ${g.nx}x${g.ny} area ${area.toFixed(2)} (kernel base volume/H ${(baseVol / c.H).toFixed(2)})`);
    const picks = rimPicks(base.result.body, c.H);
    for (const r of radiiOpt ? radiiOpt.split(',').map(Number) : c.radii) {
      const { d, prArea } = distToEroded(g, r);
      const zo = heightfield(g, d, c.H, r, chamfer ? 'chamfer' : 'round'), zs = heightSwept(g, c.H, r, chamfer ? 'chamfer' : 'round');
      const feat = chamfer ? { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: r, edges: picks } : { type: 'fillet', id: 'F3', radius: r, edges: picks };
      const res = host.regen([...feats, feat]);
      const errs = res.result?.errors ?? [];
      let oracleRemoved = 0; for (let i = 0; i < zo.length; i++) if (g.inside[i]) oracleRemoved += (c.H - zo[i]) * cell * cell;
      const row = { case: name, kind: chamfer ? 'chamfer' : 'round', r, prArea: +prArea.toFixed(2), oracleRemoved: +oracleRemoved.toFixed(2) };
      { let sw0 = 0; for (let i = 0; i < zs.length; i++) if (g.inside[i]) sw0 += (c.H - zs[i]) * cell * cell; row.sweptOracleRemoved = +sw0.toFixed(2); }
      const dump = (zk) => { if (!out) return; const tag = `${name}_${row.kind}_r${r}`;
        writeFileSync(path.join(out, tag + '_oracle.f32'), Buffer.from(zo.buffer)); writeFileSync(path.join(out, tag + '_kernel.f32'), Buffer.from(zk.buffer)); writeFileSync(path.join(out, tag + '_swept.f32'), Buffer.from(zs.buffer));
        writeFileSync(path.join(out, tag + '.json'), JSON.stringify({ x0: g.x0, y0: g.y0, nx: g.nx, ny: g.ny, cell, H: c.H, r, segs: segs.length })); };
      if (!res.ok || errs.length) { dump(new Float32Array(zo.length).fill(NaN)); row.status = 'kernel refused: ' + JSON.stringify(errs.map((e) => e[1]?.code ?? e)).slice(0, 80); rows.push(row); console.log(JSON.stringify(row)); continue; }
      const vol = host.volume();
      const zk = sampleTop(host.mesh(0.05), g);
      Object.assign(row, { status: 'built', kernelRemoved: +(baseVol - vol).toFixed(2), removedDiff: +(baseVol - vol - oracleRemoved).toFixed(2), removedDiffPct: +(100 * (baseVol - vol - oracleRemoved) / oracleRemoved).toFixed(3), ...compare(g, zo, zk, cell) });
      for (const k of ['maxAbs', 'rms', 'p99', 'p999', 'maxFlat', 'rmsFlat']) row[k] = +row[k].toFixed(4);
      { const cs = compare(g, zs, zk, cell); let sw = 0; for (let i = 0; i < zs.length; i++) if (g.inside[i]) sw += (c.H - zs[i]) * cell * cell;
        Object.assign(row, { sweptRemoved: +sw.toFixed(2), sweptRemovedDiffPct: +(100 * (baseVol - vol - sw) / sw).toFixed(3), sweptMaxAbs: +cs.maxAbs.toFixed(4), sweptRms: +cs.rms.toFixed(4), sweptMaxFlat: +cs.maxFlat.toFixed(4), sweptRmsFlat: +cs.rmsFlat.toFixed(4) }); }
      rows.push(row); console.log(JSON.stringify(row));
      dump(zk);
    }
    if (out) { writeFileSync(path.join(out, `${name}_profile.json`), JSON.stringify(segs)); writeFileSync(path.join(out, `${name}_mask.u8`), Buffer.from(g.inside.buffer)); }
  }
  if (out) writeFileSync(path.join(out, `rows_${chamfer ? 'chamfer' : 'round'}${only ? '_' + only : ''}.json`), JSON.stringify({ wasm, cell, rows }, null, 1));
  return rows;
}

// Analytic self-test: disc of radius R, extrude H, round r: z = H - r + sqrt(r^2 - (R - r - rho... )) -> d = max(0, rho - (R - r)).
function selftest(cell) {
  const R = 20, H = 10, segs = [], N = 4000;
  for (let i = 0; i < N; i++) { const a = 2 * Math.PI * i / N, b = 2 * Math.PI * (i + 1) / N; segs.push([R * Math.cos(a), R * Math.sin(a), R * Math.cos(b), R * Math.sin(b)]); }
  const g = rasterise(segs, cell);
  for (const kind of ['round', 'chamfer']) for (const r of [1, 5, 12, 19.9, 22]) {
    const { d } = distToEroded(g, r), z = heightfield(g, d, H, r, kind);
    let maxe = 0, maxFlat = 0;
    for (let j = 0; j < g.ny; j++) for (let i = 0; i < g.nx; i++) {
      const p = j * g.nx + i; if (!g.inside[p]) continue;
      const rho = Math.hypot(g.x0 + (i + 0.5) * cell, g.y0 + (j + 0.5) * cell), da = r > R ? Infinity : Math.max(0, rho - (R - r));
      const za = kind === 'chamfer' ? H - Math.min(da, r) : (da <= r ? H - r + Math.sqrt(r * r - da * da) : H - r), e = Math.abs(za - z[p]);
      if (e > maxe) maxe = e; if (e > maxFlat && i > 0 && Math.abs(da - r) > 0.3) maxFlat = e;
    }
    console.log(`selftest disc R=${R} ${kind} r=${r}: max |z - analytic| = ${maxe.toFixed(4)} mm (away from the vertical wall: ${maxFlat.toFixed(4)})`);
  }
}

// ------------------------------------------------------------------ kernel layer for the browser viewer
function chainLoop(segs) {
  const key = (x, y) => `${Math.round(x * 1e4)},${Math.round(y * 1e4)}`, adj = new Map();
  for (const [ax, ay, bx, by] of segs) {
    const ka = key(ax, ay), kb = key(bx, by);
    if (!adj.has(ka)) adj.set(ka, { p: [ax, ay], n: [] }); if (!adj.has(kb)) adj.set(kb, { p: [bx, by], n: [] });
    adj.get(ka).n.push(kb); adj.get(kb).n.push(ka);
  }
  const seen = new Set(), loops = [];
  for (const k0 of adj.keys()) {
    if (seen.has(k0)) continue;
    const loop = []; let prev = null, k = k0;
    while (k && !seen.has(k)) { seen.add(k); loop.push(adj.get(k).p); const nx = adj.get(k).n.find((q) => q !== prev && !seen.has(q)); prev = k; k = nx; }
    loops.push(loop);
  }
  loops.sort((a, b) => b.length - a.length);
  return loops[0];
}

export function viewerData(argv) {
  const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
  const wasm = opt('--wasm'), label = opt('--label', 'kernel wasm'), outFile = opt('--viewer-data');
  const cells = { blob: 0.6, six: 0.3, d: 0.3 };
  const host = new Host(wasm), data = { wasm: label, presets: {} };
  for (const [name, c] of Object.entries(CASES)) {
    const feats = partFor(c), base = host.regen(feats);
    const loop = chainLoop(capBoundary(host.mesh(0.004), c.H)).map(([x, y]) => [Math.round(x * 1e3) / 1e3, Math.round(y * 1e3) / 1e3]);
    const segs = loop.map((a, i) => { const b = loop[(i + 1) % loop.length]; return [a[0], a[1], b[0], b[1]]; });
    const g = rasterise(segs, cells[name], 1);
    const picks = rimPicks(base.result.body, c.H);
    const pre = { H: c.H, cell: cells[name], nx: g.nx, ny: g.ny, poly: loop.flat(), kernel: { round: [], chamfer: [] } };
    for (const kind of ['round', 'chamfer']) for (const r of c.radii) {
      const feat = kind === 'chamfer' ? { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: r, edges: picks } : { type: 'fillet', id: 'F3', radius: r, edges: picks };
      const res = host.regen([...feats, feat]), errs = res.result?.errors ?? [];
      if (!res.ok || errs.length) { pre.kernel[kind].push({ r, status: 'refused: ' + (errs[0]?.[1]?.code ?? 'error') }); continue; }
      const zk = sampleTop(host.mesh(0.05), g), u = new Uint16Array(zk.length);
      for (let i = 0; i < zk.length; i++) u[i] = Number.isNaN(zk[i]) ? 65535 : Math.max(0, Math.min(65534, Math.round(zk[i] / c.H * 65534)));
      pre.kernel[kind].push({ r, status: 'built', b64: Buffer.from(u.buffer).toString('base64') });
    }
    data.presets[name] = pre;
    console.log(name, 'viewer data', Object.fromEntries(Object.entries(pre.kernel).map(([k, v]) => [k, v.map((q) => q.r + ':' + q.status[0]).join(' ')])));
  }
  writeFileSync(outFile, JSON.stringify(data));
}

if (process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])) {
  if (process.argv.includes('--viewer-data')) viewerData(process.argv.slice(2));
  else if (process.argv.includes('--selftest')) selftest(Number(process.argv.includes('--cell') ? process.argv[process.argv.indexOf('--cell') + 1] : 0.1));
  else main(process.argv.slice(2));
}
