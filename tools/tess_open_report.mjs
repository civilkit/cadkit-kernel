// Classify the open display-mesh edges of every gate-corpus part (tools/brep_cases.mjs) and the kit samples.
// usage: node tools/tess_open_report.mjs [--wasm path] [--kit DIR] [--only substr] [--json out.json] [--no-samples]
//
// For each part: regen (ours), decode the display mesh, weld vertices by position (1e-4 mm class), count mesh edges
// used by one triangle and classify each:
//   a  face with zero triangles (no mesh at all)
//   b  boundary edge that coincides geometrically with a boundary edge of ANOTHER face (b1: collinear overlap within
//      1e-6 = T-junction, welded in position; b2: parallel within 1e-3 = hairline crack from different discretisation)
//   c  boundary edge with no partner anywhere: a hole in the face's own triangulation (c1: partner in same face)
//   d  degenerate: edge shorter than 1e-3 or triangle of zero area (pole/seam artefact)
// and cross-checks with `validate {mesh:true}`.
import { readFileSync, readdirSync, existsSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { parts as brepParts } from './brep_cases.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const wasm = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const kit = opt('--kit', '/path/to/work/kit');
const only = opt('--only');
const jsonOut = opt('--json');
const skip = opt('--skip');
const noSamples = flag('--no-samples');
const detail = flag('--detail');
const mod = new WebAssembly.Module(readFileSync(wasm));

const T = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };
function decode(r) {
  const jl = new DataView(r.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const dec = (v) => {
    if (Array.isArray(v)) return v.map(dec);
    if (v && typeof v === 'object') {
      if ('$buf' in v) { const C = T[v.type]; return new C(r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT)); }
      const o = {}; for (const [k, x] of Object.entries(v)) o[k] = dec(x); return o;
    }
    return v;
  };
  return dec(JSON.parse(r.json));
}

function analyse(mesh, nFaces) {
  const pos = mesh.positions, tf = mesh.triFace, nv = pos.length / 3, nt = tf.length;
  let hi = 0; for (const x of pos) hi = Math.max(hi, Math.abs(x));
  const q = Math.max(hi * 1e-5, 1e-4);
  const key = (x, y, z) => `${Math.round(x / q)},${Math.round(y / q)},${Math.round(z / q)}`;
  const vid = new Map(), reps = [], ids = new Int32Array(nv);
  for (let i = 0; i < nv; i++) {
    // weld by position; probe the 8 neighbouring cells so rounding at a cell boundary does not split a vertex
    let found = -1;
    const cx = pos[3 * i] / q, cy = pos[3 * i + 1] / q, cz = pos[3 * i + 2] / q;
    outer: for (const dx of [0, -1, 1]) for (const dy of [0, -1, 1]) for (const dz of [0, -1, 1]) {
      const v = vid.get(`${Math.round(cx) + dx},${Math.round(cy) + dy},${Math.round(cz) + dz}`);
      if (v !== undefined) for (const r of v) {
        const o = reps[r];
        if ((o[0] - pos[3 * i]) ** 2 + (o[1] - pos[3 * i + 1]) ** 2 + (o[2] - pos[3 * i + 2]) ** 2 <= q * q) { found = r; break outer; }
      }
    }
    if (found < 0) { reps.push([pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]]); found = reps.length - 1; const k = key(...reps[found]); if (!vid.has(k)) vid.set(k, []); vid.get(k).push(found); }
    ids[i] = found;
  }
  const uses = new Map(); // "a,b" -> {n, face}
  let degTris = 0;
  for (let t = 0; t < nt; t++) {
    const v = [ids[3 * t], ids[3 * t + 1], ids[3 * t + 2]];
    if (v[0] === v[1] || v[1] === v[2] || v[0] === v[2]) { degTris++; continue; }
    for (let e = 0; e < 3; e++) {
      const a = v[e], b = v[(e + 1) % 3], k = a < b ? `${a},${b}` : `${b},${a}`;
      const s = uses.get(k) ?? { n: 0, face: tf[t], a: Math.min(a, b), b: Math.max(a, b) };
      s.n++; uses.set(k, s);
    }
  }
  const open = [...uses.values()].filter((s) => s.n === 1);
  const hasTris = new Set(tf);
  const noTri = []; for (let f = 0; f < nFaces; f++) if (!hasTris.has(f)) noTri.push(f);
  // face bbox
  const fb = new Map();
  for (let t = 0; t < nt; t++) {
    let b = fb.get(tf[t]); if (!b) { b = [1e300, 1e300, 1e300, -1e300, -1e300, -1e300]; fb.set(tf[t], b); }
    for (let i = 0; i < 9; i++) { const c = i % 3; b[c] = Math.min(b[c], pos[3 * 3 * t + i]); b[c + 3] = Math.max(b[c + 3], pos[3 * 3 * t + i]); }
  }
  // classify open edges
  const segDist = (p, a, b) => { // distance from p to segment ab
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    const L2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2; let u = L2 > 0 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2 : 0; u = Math.max(0, Math.min(1, u));
    return Math.hypot(ap[0] - u * ab[0], ap[1] - u * ab[1], ap[2] - u * ab[2]);
  };
  const crackTol = Math.max(hi * 2e-3, 0.01); let maxGap = 0, maxGapAll = 0, bigGap = 0, holes = 0;
  const cls = { a: noTri.length, b1: 0, b2: 0, c: 0, c1: 0, d: 0 };
  const byFace = new Map(); // face -> {b1,b2,c,c1,d}
  const bump = (f, k) => { const o = byFace.get(f) ?? { b1: 0, b2: 0, c: 0, c1: 0, d: 0 }; o[k]++; byFace.set(f, o); cls[k]++; };
  // spatial bucket of open edges by midpoint, cell = 1 mm-ish scaled
  const cell = Math.max(hi * 1e-2, 0.5), buckets = new Map();
  const ck = (p) => `${Math.floor(p[0] / cell)},${Math.floor(p[1] / cell)},${Math.floor(p[2] / cell)}`;
  open.forEach((e, i) => { const A = reps[e.a], B = reps[e.b]; const m = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2, (A[2] + B[2]) / 2]; e.m = m; const k = ck(m); if (!buckets.has(k)) buckets.set(k, []); buckets.get(k).push(i); });
  const nearby = (m, r) => {
    const out = []; const n = Math.ceil(r / cell);
    const c = [Math.floor(m[0] / cell), Math.floor(m[1] / cell), Math.floor(m[2] / cell)];
    for (let x = -n; x <= n; x++) for (let y = -n; y <= n; y++) for (let z = -n; z <= n; z++) { const v = buckets.get(`${c[0] + x},${c[1] + y},${c[2] + z}`); if (v) out.push(...v); }
    return out;
  };
  for (const e of open) {
    const A = reps[e.a], B = reps[e.b], len = Math.hypot(A[0] - B[0], A[1] - B[1], A[2] - B[2]);
    if (len < 1e-3) { bump(e.face, 'd'); continue; }
    // gap: how far the endpoints of this open edge are from the open edges of ANOTHER face (0 = T-junction welded in
    // position, small = hairline crack from different discretisation of a shared edge, large/none = a real hole)
    const gapTo = (sameFace) => {
      let g = Infinity;
      const ends = [A, B];
      let worst = 0;
      for (const P of ends) {
        let m = Infinity;
        for (const j of nearby(P, len + 2)) { const o = open[j]; if (o === e || (o.face === e.face) !== sameFace) continue; m = Math.min(m, segDist(P, reps[o.a], reps[o.b])); }
        worst = Math.max(worst, m);
      }
      return worst;
    };
    const g = gapTo(false), g1 = gapTo(true);
    if (Number.isFinite(g)) { maxGapAll = Math.max(maxGapAll, g); if (g > crackTol) bigGap++; } else holes++;
    if (g <= 1e-6) bump(e.face, 'b1');
    else if (g <= crackTol) { bump(e.face, 'b2'); maxGap = Math.max(maxGap, g); }
    else if (g1 <= crackTol) bump(e.face, 'c1');
    else bump(e.face, 'c');
  }
  const sample = open.slice(0, 12).map((e) => ({ face: e.face, a: reps[e.a].map((x) => +x.toFixed(3)), b: reps[e.b].map((x) => +x.toFixed(3)) }));
  return { cls, openEdges: open.length, noTri, byFace, fb, degTris, sample, maxGap, maxGapAll, bigGap, holes };
}

function runPart(name, part) {
  const k = new Kernel(mod);
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  if (rr.trap) return { name, trap: rr.trap };
  const j = decode(rr);
  const body = j.result?.body;
  if (!body?.mesh) return { name, nobody: true, errors: j.result?.errors?.length ?? 0 };
  const nFaces = body.faces?.length ?? 0;
  const a = analyse(body.mesh, nFaces);
  const vr = k.request(JSON.stringify({ id: 2, msg: { type: 'validate', tag: 'main', mesh: true } }));
  const v = vr.trap ? null : JSON.parse(vr.json)?.result;
  const meshIssues = (v?.issues ?? []).filter((i) => i.code === 'TESSELLATION_OPEN');
  const otherIssues = (v?.issues ?? []).filter((i) => i.code !== 'TESSELLATION_OPEN').length;
  const faceList = [...a.byFace.entries()].map(([f, c]) => {
    const b = a.fb.get(f); const ctr = b ? [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2].map((x) => +x.toFixed(2)) : null;
    return { face: f, id: body.faces?.[f]?.id ?? body.faces?.[f]?.name, centre: ctr, ...c };
  });
  const noTri = a.noTri.map((f) => ({ face: f, id: body.faces?.[f]?.id ?? body.faces?.[f]?.name, surf: body.faces?.[f]?.surface ?? body.faces?.[f]?.type }));
  return { name, open: a.openEdges, cls: a.cls, maxGap: +a.maxGap.toFixed(4), maxGapAll: +a.maxGapAll.toFixed(4), bigGap: a.bigGap, holes: a.holes, degTris: a.degTris, noTri, faces: faceList, sample: a.sample, check: meshIssues.map((i) => `${i.entity}: ${i.detail}`), otherIssues, failed: j.result?.body?.mesh?.failed };
}

const items = Object.entries(brepParts).map(([n, p]) => ({ name: `brep:${n}`, part: p }));
if (!noSamples) {
  for (const dir of [path.join(kit, 'apps/cad/public/samples'), path.join(kit, 'apps/cad/public/samples/heavy')])
    if (existsSync(dir)) for (const f of readdirSync(dir).filter((x) => x.endsWith('.ckpart')).sort()) items.push({ name: `sample:${f}`, part: JSON.parse(readFileSync(path.join(dir, f), 'utf8')), file: path.join(dir, f) });
}
const results = [];
for (const it of items) {
  if (only && !it.name.includes(only)) continue;
  if (skip && it.name.includes(skip)) continue;
  if (process.env.TOR_PROGRESS) process.stderr.write(`${it.name}\n`);
  let r; try { r = runPart(it.name, it.part); } catch (e) { r = { name: it.name, error: String(e).slice(0, 200) }; }
  if (it.file) r.file = it.file;
  results.push(r);
}
const bad = results.filter((r) => r.open || r.noTri?.length || r.check?.length || r.trap || r.error);
console.log(`parts ${results.length}, with mesh defects ${bad.length}`);
const tot = { a: 0, b1: 0, b2: 0, c: 0, c1: 0, d: 0 };
console.log('part'.padEnd(46), 'open  a  b1  b2   c  c1   d  check');
for (const r of bad) {
  if (r.trap || r.error) { console.log(r.name.padEnd(46), r.trap ?? r.error); continue; }
  for (const k of Object.keys(tot)) tot[k] += r.cls[k];
  console.log(r.name.padEnd(46), String(r.open).padStart(4), ...['a', 'b1', 'b2', 'c', 'c1', 'd'].map((k) => String(r.cls[k]).padStart(3)), r.check.length ? 'FIRES' : '-', `gap<=${r.maxGapAll}`, r.bigGap ? `wide:${r.bigGap}` : '', r.holes ? `NOPARTNER:${r.holes}` : '');
  for (const f of r.noTri) console.log('      no-tri face', JSON.stringify(f));
  for (const f of r.faces) console.log('      face', JSON.stringify(f));
  if (detail) for (const e of r.sample) console.log('      edge', JSON.stringify(e));
}
console.log('totals', JSON.stringify(tot));
if (jsonOut) writeFileSync(jsonOut, JSON.stringify(results, null, 1));
