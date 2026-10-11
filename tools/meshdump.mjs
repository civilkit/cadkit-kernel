// Run a regen against the ORIGINAL kernel and dump body JSON + decoded $buf arrays.
// usage: node tools/meshdump.mjs part.json [out.json]   (part.json = a .ckpart object or full request msg)
import { readFileSync, writeFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const mod = await WebAssembly.compile(readFileSync(new URL('../../kernel.wasm', import.meta.url)));
const k = new Kernel(mod);
let inp = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const msg = inp.type ? inp : { type: 'regen', tag: 'main', part: inp };
const r = k.request(JSON.stringify({ id: 1, msg }));
if (r.trap) { console.error(r.trap); process.exit(1); }
const jl = new DataView(r.bytes.buffer).getUint32(0, true);
const base = (4 + jl + 7) & ~7;
const T = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };
const dec = (v) => {
  if (Array.isArray(v)) return v.map(dec);
  if (v && typeof v === 'object') {
    if ('$buf' in v) { const C = T[v.type]; const s = r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT); return Array.from(new C(s)); }
    const o = {}; for (const [kk, x] of Object.entries(v)) o[kk] = dec(x); return o;
  }
  return v;
};
const j = JSON.parse(r.json);
const out = dec(j);
if (process.argv[3]) writeFileSync(process.argv[3], JSON.stringify(out));
else console.log(JSON.stringify(out).slice(0, 3000));
