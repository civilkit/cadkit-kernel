// Mesh statistics (triangle count, mesh area, per-face triangle counts) of every brep case.
// usage: node tools/meshstat.mjs <wasm> <out.json>        compare: node tools/meshstat.mjs --diff a.json b.json
import { readFileSync, writeFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';
if (process.argv[2] === '--diff') {
  const a = JSON.parse(readFileSync(process.argv[3], 'utf8')), b = JSON.parse(readFileSync(process.argv[4], 'utf8'));
  let same = 0; const rows = [];
  for (const k of Object.keys(a)) {
    const x = a[k], y = b[k];
    if (!x || !y || x.tris === undefined || y.tris === undefined) { if (JSON.stringify(x) !== JSON.stringify(y)) rows.push(`${k}: ${JSON.stringify(x)} -> ${JSON.stringify(y)}`); else same++; continue; }
    if (x.tris === y.tris && Math.abs(x.area - y.area) <= 1e-9 * (1 + x.area) && JSON.stringify(x.perFace) === JSON.stringify(y.perFace)) { same++; continue; }
    rows.push(`${k}: tris ${x.tris} -> ${y.tris}, area ${x.area.toFixed(3)} -> ${y.area.toFixed(3)}, ms ${x.ms}->${y.ms}`);
  }
  console.log(`${same} identical, ${rows.length} differ`); for (const r of rows) console.log(' ', r);
  process.exit(0);
}
const mod = new WebAssembly.Module(readFileSync(process.argv[2]));
const out = {};
for (const [name, part] of Object.entries(parts)) {
  const k = new Kernel(mod);
  const t = performance.now();
  const r = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  const ms = Math.round(performance.now() - t);
  if (r.trap) { out[name] = { trap: true }; continue; }
  const jl = new DataView(r.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const j = JSON.parse(r.json);
  const m = j.result?.body?.mesh;
  if (!m) { out[name] = { none: true }; continue; }
  const get = (v) => { const C = { f32: Float32Array, i32: Int32Array }[v.type]; return new C(r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT)); };
  const pos = get(m.positions ?? m.pos), tf = get(m.triFace);
  let area = 0; const pf = {};
  for (let i = 0; i < tf.length; i++) {
    const o = i * 9, ax = pos[o + 3] - pos[o], ay = pos[o + 4] - pos[o + 1], az = pos[o + 5] - pos[o + 2], bx = pos[o + 6] - pos[o], by = pos[o + 7] - pos[o + 1], bz = pos[o + 8] - pos[o + 2];
    area += 0.5 * Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
    pf[tf[i]] = (pf[tf[i]] ?? 0) + 1;
  }
  out[name] = { tris: tf.length, area, perFace: pf, ms };
}
writeFileSync(process.argv[3], JSON.stringify(out));
console.log(Object.keys(out).length, 'cases');
