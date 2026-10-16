// Helper of tools/measure_occt_check.py: regen a part, export STEP, pick random face pairs and measure them.
// usage: node tools/measure_occt_dump.mjs <part.ckpart> <out-dir> [--pairs N] [--seed S] [--wasm path]
// Writes <out-dir>/<name>.step and <out-dir>/<name>.pairs.json:
//   [{a, b, ptA, ptB, ours: distance reply, ms}], ptA / ptB = an interior point of each face (from thicknessAt).
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f, d) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const file = args[0];
const out = args[1];
const n = Number(opt('--pairs', '20'));
let seed = Number(opt('--seed', '1'));
const wasm = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };

const part = JSON.parse(readFileSync(file, 'utf8'));
const name = path.basename(file).replace(/\.ckpart$/, '');
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)), { now: () => Date.now() });
const rq = (msg) => {
  const t = Date.now();
  const r = k.request(JSON.stringify({ id: 1, msg: { tag: 'main', ...msg } }));
  if (!r.json) throw new Error(`trap ${r.trap}`);
  const j = JSON.parse(r.json);
  return { ...j, ms: Date.now() - t };
};
const reg = rq({ type: 'regen', part });
if (!reg.ok || (reg.result?.errors ?? []).length) throw new Error(`regen errors in ${name}`);
const st = rq({ type: 'step', name });
if (!st.ok) throw new Error(`step: ${st.error}`);
mkdirSync(out, { recursive: true });
writeFileSync(path.join(out, `${name}.step`), typeof st.result === 'string' ? st.result : st.result.text ?? JSON.stringify(st.result));

// planar / curved faces with a unique id and a usable interior point
const fids = reg.result.body.faces.map((f) => f.id);
const ids = fids.filter((f, i) => fids.indexOf(f) === i && fids.lastIndexOf(f) === i);
const pool = [];
for (const id of ids) {
  const t = rq({ type: 'thicknessAt', face: id, opts: { samples: 4 } });
  const p = t.ok && t.result?.min?.point;
  if (p) pool.push({ id, p });
}
const pairs = [];
let guard = 0;
while (pairs.length < n && pool.length > 1 && guard++ < 400) {
  const a = pool[Math.floor(rnd() * pool.length)];
  const b = pool[Math.floor(rnd() * pool.length)];
  if (a.id === b.id || pairs.some((x) => x.a === a.id && x.b === b.id)) continue;
  const d = rq({ type: 'distance', a: a.id, b: b.id });
  if (!d.ok) continue;
  pairs.push({ a: a.id, b: b.id, ptA: a.p, ptB: b.p, ours: d.result, ms: d.ms });
}
writeFileSync(path.join(out, `${name}.pairs.json`), JSON.stringify(pairs));
console.log(`${name}: ${ids.length} faces, ${pool.length} usable, ${pairs.length} pairs`);
