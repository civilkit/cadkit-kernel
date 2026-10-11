// Compare decoded $buf mesh arrays (+ edge pts) of original vs rebuilt wasm per brep case.
// usage: node tools/meshcmp.mjs [--wasm path] [name ...]
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';
const args = process.argv.slice(2);
let wasmB = new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url);
const wi = args.indexOf('--wasm'); if (wi >= 0) { wasmB = args[wi + 1]; args.splice(wi, 2); }
const A = new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url)));
const B = new WebAssembly.Module(readFileSync(wasmB));
const T = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };
function run(mod, part) {
  const r = new Kernel(mod).request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  if (r.trap) return { trap: r.trap };
  const jl = new DataView(r.bytes.buffer).getUint32(0, true), base = (4 + jl + 7) & ~7;
  const dec = (v) => {
    if (Array.isArray(v)) return v.map(dec);
    if (v && typeof v === 'object') {
      if ('$buf' in v) { const C = T[v.type]; return Array.from(new C(r.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT))); }
      const o = {}; for (const [k, x] of Object.entries(v)) o[k] = dec(x); return o;
    }
    return v;
  };
  return dec(JSON.parse(r.json));
}
export { run };
const eq = (a, b) => a.length === b.length && a.every((x, i) => Object.is(x, b[i]));
if (import.meta.url === `file://${process.argv[1]}`) {
  let ok = 0, n = 0;
  for (const [name, part] of Object.entries(parts)) {
    if (args.length && !args.includes(name)) continue;
    const a = run(A, part), b = run(B, part); n++;
    if (a.trap || b.trap) { console.log(name, 'TRAP', !!a.trap, !!b.trap); continue; }
    const ba = a.result?.body ?? {}, bb = b.result?.body ?? {};
    const rep = [];
    const cmp = (k, x, y) => { if (Array.isArray(x) && x.length && typeof x[0] === 'number') { if (!eq(x, y ?? [])) rep.push(`${k}:${x.length}/${(y ?? []).length}`); } };
    const walk = (k, x, y) => {
      if (x && typeof x === 'object' && !Array.isArray(x)) for (const kk of Object.keys(x)) walk(`${k}.${kk}`, x[kk], y?.[kk]);
      else if (Array.isArray(x) && x.length && typeof x[0] === 'object') x.forEach((e, i) => walk(`${k}[${i}]`, e, y?.[i]));
      else cmp(k, x, y);
    };
    walk('body', ba, bb);
    if (!rep.length) ok++; else console.log(name.padEnd(18), rep.slice(0, 8).join(' '), rep.length > 8 ? `+${rep.length - 8}` : '');
  }
  console.log(`mesh-identical ${ok}/${n}`);
}
