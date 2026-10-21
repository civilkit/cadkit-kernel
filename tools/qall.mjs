// Session probe: send several requests to one kernel instance (session state).
// usage: node tools/qall.mjs <f1.json> <f2.json> ...   (FULL=1 prints full)
import { readFileSync } from 'node:fs';
const wasm = new URL('../../kernel.wasm', import.meta.url);
const mod = await WebAssembly.compile(readFileSync(wasm));
let inst, panic = null;
const env = { ck_now: () => 0, ck_unix_ms: () => 0,
  ck_panic: (p, l) => { panic = new TextDecoder().decode(new Uint8Array(inst.memory.buffer, p, l)); } };
inst = new WebAssembly.Instance(mod, { env }).exports;
for (const f of process.argv.slice(2)) {
  const body = readFileSync(f, 'utf8');
  const b = new TextEncoder().encode(body.replace(/"format"\s*:\s*"ck-part"/g, '"format":"ck-part"')); panic = null;   // the original kernel.wasm only accepts the format's old name
  try {
    const ptr = inst.ck_alloc(b.length);
    new Uint8Array(inst.memory.buffer, ptr, b.length).set(b);
    const len = inst.ck_request(ptr, b.length), out = inst.ck_reply();
    const jl = new DataView(inst.memory.buffer, out, len).getUint32(0, true);
    const js = new TextDecoder().decode(new Uint8Array(inst.memory.buffer, out + 4, jl));
    console.log(`# ${f} total=${len} json=${jl}`);
    console.log(!process.env.FULL && js.length > 3000 ? js.slice(0, 3000) + '...' : js);
  } catch (e) { console.log('# TRAP', e.message, 'panic=', panic); inst = new WebAssembly.Instance(mod, { env }).exports; }
}
