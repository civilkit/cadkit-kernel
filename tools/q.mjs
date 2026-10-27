// Probe the ORIGINAL kernel.wasm with raw request bodies.
// usage: node tools/q.mjs '<request body>' ...      (prints reply JSON)
//        node tools/q.mjs --file req.json            (body read from a file)
// env:   FULL=1 prints the whole JSON (default truncates at 4000 chars)
import { readFileSync } from 'node:fs';
const wasm = new URL('../../kernel.wasm', import.meta.url);
const mod = await WebAssembly.compile(readFileSync(wasm));
let inst, panic = null;
const env = { ck_now: () => 0, ck_unix_ms: () => 0,
  ck_panic: (p, l) => { panic = new TextDecoder().decode(new Uint8Array(inst.memory.buffer, p, l)); } };
const boot = () => { inst = new WebAssembly.Instance(mod, { env }).exports; };
boot();
let args = process.argv.slice(2);
if (args[0] === '--file') args = [readFileSync(args[1], 'utf8')];
for (const body of args) {
  const b = new TextEncoder().encode(body.replace(/"format"\s*:\s*"ck-part"/g, '"format":"formwerk-part"')); panic = null;   // the original kernel.wasm only accepts the format's old name
  try {
    const ptr = inst.ck_alloc(b.length);
    new Uint8Array(inst.memory.buffer, ptr, b.length).set(b);
    const len = inst.ck_request(ptr, b.length), out = inst.ck_reply();
    const jl = new DataView(inst.memory.buffer, out, len).getUint32(0, true);
    const js = new TextDecoder().decode(new Uint8Array(inst.memory.buffer, out + 4, jl));
    console.log(`# total=${len} json=${jl}`);
    console.log(!process.env.FULL && js.length > 4000 ? js.slice(0, 4000) + '...' : js);
  } catch (e) { console.log('# TRAP', e.message, 'panic=', panic); boot(); }
}
