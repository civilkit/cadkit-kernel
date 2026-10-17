// Oracle vs ours on the loft centre-line corpus (tools/loft_centre_cases.mjs).
// usage: node tools/loft_centre_check.mjs [--ours wasm] [--orig-only] [--json] [--tol 1e-3] [name-substring ...]
// The oracle has no section / probe request, so sections are cut from the regen reply's display mesh by plane
// (the same code for both kernels; area error ~ the tessellation chord error, 1e-3 relative on planar faces).
// Per case: errors, faces/edges, volume, bbox, y-plane section areas / centroids (x, z) at several heights and, for
// cases with a known spine, the sections PERPENDICULAR to the spine at 3 stations (area, centroid offset from the
// spine point in the plane). With --ours also the verdict: AGREE (same error codes, or volume within tol and section
// areas within 2 percent, centroids within 0.25 mm), else DIFF with the reason.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { cases } from './loft_centre_cases.mjs';

const isMain = process.argv[1]?.endsWith('loft_centre_check.mjs');
const args = isMain ? process.argv.slice(2) : [];
const take = (f, v) => { const i = args.indexOf(f); if (i < 0) return null; const x = v ? args[i + 1] : true; args.splice(i, v ? 2 : 1); return x; };
const oursPath = take('--ours', true) ?? new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url);
const origOnly = take('--orig-only'); const asJson = take('--json'); const tol = +(take('--tol', true) ?? 1e-3);
const orig = isMain ? new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url))) : null;
const ours = isMain && !origOnly ? new WebAssembly.Module(readFileSync(oursPath)) : null;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const crs = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// Cut a flat triangle list (9 floats per triangle, outward winding) by the plane (o, n): signed area and centroid
// of the cross-section (sum over all loops, holes negative), via the fan of the cut segments around `o`.
export function sliceMesh(pos, o, n) {
  let A = 0; const C = [0, 0, 0];
  for (let t = 0; t + 8 < pos.length; t += 9) {
    const P = [[pos[t], pos[t + 1], pos[t + 2]], [pos[t + 3], pos[t + 4], pos[t + 5]], [pos[t + 6], pos[t + 7], pos[t + 8]]];
    const d = P.map((p) => dot(sub(p, o), n));
    const pts = [];
    for (let i = 0; i < 3; i++) {
      const j = (i + 1) % 3;
      if ((d[i] < 0) !== (d[j] < 0)) { const u = d[i] / (d[i] - d[j]); pts.push([P[i][0] + (P[j][0] - P[i][0]) * u, P[i][1] + (P[j][1] - P[i][1]) * u, P[i][2] + (P[j][2] - P[i][2]) * u]); }
    }
    if (pts.length !== 2) continue;
    const tn = crs(sub(P[1], P[0]), sub(P[2], P[0]));
    let [p, q] = pts;
    if (dot(crs(tn, n), sub(q, p)) < 0) [p, q] = [q, p];
    const a = dot(crs(sub(p, o), sub(q, o)), n) / 2;
    A += a;
    for (let k = 0; k < 3; k++) C[k] += a * (o[k] + p[k] + q[k]) / 3;
  }
  // outward normals: material is on the +n side of the cut ring when the sign is negative; report |area|
  const s = A < 0 ? -1 : 1;
  return A === 0 ? { area: 0 } : { area: Math.abs(A), c: C.map((x) => x / A), s };
}

export function summarize(mod, part, meta = {}, ys = [3, 9, 15, 21, 27, 33, 45, 57]) {
  const k = new Kernel(mod);
  const call = (msg, id = 2) => JSON.parse(k.request(JSON.stringify({ id, msg: { ...msg, tag: 'main' } })).json ?? 'null');
  const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, opts: { solveSketches: true } } }));
  if (rep.trap) return { trap: rep.trap.slice(0, 100) };
  const r = JSON.parse(rep.json);
  const res = r.result ?? {};
  const out = { errors: (res.errors ?? []).map(([, e]) => `${e.code}${e.params?.detail ? `(${String(e.params.detail).slice(0, 40)})` : ''}`), warn: (res.warnings ?? []).map((w) => w[1]?.code ?? JSON.stringify(w).slice(0, 40)) };
  const jl = new DataView(rep.bytes.buffer).getUint32(0, true), base = (4 + jl + 7) & ~7;
  const bodies = [];
  if (res.body) bodies.push(res.body);
  for (const [, s] of res.surfaces ?? []) bodies.push(s);
  let bb = null; let pos = null;
  for (const b of bodies) {
    const m = b.mesh; const v = m && (m.positions ?? m.pos); if (!v) continue;
    pos = new Float32Array(rep.bytes.buffer.slice(rep.bytes.byteOffset + base + v.offset, rep.bytes.byteOffset + base + v.offset + v.len * 4));
    bb ??= [1e9, 1e9, 1e9, -1e9, -1e9, -1e9];
    for (let i = 0; i < pos.length; i += 3) for (let a = 0; a < 3; a++) { bb[a] = Math.min(bb[a], pos[i + a]); bb[a + 3] = Math.max(bb[a + 3], pos[i + a]); }
  }
  if (bb) out.bbox = bb.map((x) => +x.toFixed(3));
  if (res.body) {
    out.faces = res.body.faces.length; out.edges = res.body.edges.length;
    const m = call({ type: 'massProps' })?.result;
    if (m) { out.vol = +m.volume.toFixed(4); out.area = +m.area.toFixed(3); }
    const v = call({ type: 'validate' })?.result;
    if (v?.issues?.length) out.issues = v.issues.slice(0, 3).map((i) => i.code);
    out.sec = ys.map((y) => { const s = sliceMesh(pos, [0, y, 0], [0, 1, 0]); return s.area ? [y, +s.area.toFixed(2), +s.c[0].toFixed(2), +s.c[2].toFixed(2)] : [y, 0]; });
    if (meta.spine) {
      out.nsec = [0.25, 0.5, 0.75].map((t) => {
        const [p, d] = meta.spine(t); const o = [p[0], p[1], 0], n = [d[0], d[1], 0];
        const s = sliceMesh(pos, o, n);
        return s.area ? [t, +s.area.toFixed(2), +Math.hypot(...sub(s.c, o)).toFixed(2)] : [t, 0];
      });
    }
  }
  return out;
}

export function agree(a, b, tol) {
  if (a.trap || b.trap) return 'trap';
  const code = (e) => e.split('(')[0];
  if (a.errors.map(code).join() !== b.errors.map(code).join()) return `errors differ (${a.errors} vs ${b.errors})`;
  if (a.errors.length) return null;
  if (a.vol != null && b.vol != null) {
    const rv = Math.abs(a.vol - b.vol) / Math.max(1e-9, Math.abs(a.vol));
    if (rv > tol) return `volume ${a.vol} vs ${b.vol} (${(rv * 100).toFixed(2)}%)`;
  } else if ((a.vol == null) !== (b.vol == null)) return 'solid vs no solid';
  for (let i = 0; i < (a.sec ?? []).length; i++) {
    const [sa, sb] = [a.sec[i], b.sec?.[i]];
    if (!sb) return 'no sections';
    if (Math.abs(sa[1] - sb[1]) > 0.02 * Math.max(sa[1], 1)) return `section y=${sa[0]} area ${sa[1]} vs ${sb[1]}`;
    if (sa[1] > 0 && (Math.abs(sa[2] - sb[2]) > 0.25 || Math.abs(sa[3] - sb[3]) > 0.25)) return `section y=${sa[0]} centroid (${sa[2]},${sa[3]}) vs (${sb[2]},${sb[3]})`;
  }
  return null;
}

if (isMain) {
  let n = 0, ok = 0;
  for (const [name, c] of Object.entries(cases)) {
    if (args.length && !args.some((f) => name.includes(f))) continue;
    const a = summarize(orig, c.part, c.meta);
    const b = ours ? summarize(ours, c.part, c.meta) : null;
    if (asJson) { console.log(JSON.stringify({ name, orig: a, ours: b })); continue; }
    const fmt = (x) => x.trap ? `TRAP ${x.trap}` : `${x.errors.length ? 'ERR ' + x.errors.join(',') : `f${x.faces}/e${x.edges} vol ${x.vol} bb ${JSON.stringify(x.bbox)}`}${x.warn?.length ? ' WARN ' + x.warn : ''}${x.issues ? ' ISSUES ' + x.issues : ''}`;
    const secs = (x) => x.sec ? `${''.padEnd(20)}      sec(y:area@x,z) ${x.sec.map((s) => `${s[0]}:${s[1]}@${s[2] ?? '-'},${s[3] ?? '-'}`).join(' ')}${x.nsec ? '\n' + ''.padEnd(26) + 'nsec(t:area,off) ' + x.nsec.map((s) => `${s[0]}:${s[1]},${s[2] ?? '-'}`).join(' ') : ''}` : null;
    console.log(`${name.padEnd(20)} ORIG ${fmt(a)}`);
    if (secs(a)) console.log(secs(a));
    if (b) {
      const d = agree(a, b, tol); n++; if (!d) ok++;
      console.log(`${''.padEnd(20)} ${d ? 'DIFF ' + d : 'AGREE'}  OURS ${fmt(b)}`);
      if (d && secs(b)) console.log(secs(b));
    }
  }
  if (ours) console.log(`\n${ok}/${n} cases agree (volume tol ${tol})`);
}
