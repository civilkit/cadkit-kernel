#!/usr/bin/env node
// Run kernel messages from a JSON file against a built wasm and print the replies.
//   node book/tools/run.mjs <ck_wasm.wasm> <messages.json> [--full]
// messages.json is one message object or an array of them, for example
//   [{"type":"regen","tag":"t","part":{...}}, {"type":"validate","tag":"t"}]
// Messages run in order on one kernel instance, so a later message can use the tag of an earlier regen.
// Binary buffers ({"$buf": ...}) are summarised unless --full is given.
import { readFileSync } from 'node:fs';

const [wasmPath, file, ...flags] = process.argv.slice(2);
if (!wasmPath || !file) { console.error('usage: run.mjs <ck_wasm.wasm> <messages.json> [--full]'); process.exit(64); }
const full = flags.includes('--full');
let inst;
const env = {
  ck_now: () => 0,
  ck_unix_ms: () => Date.now(),
  ck_panic: (p, l) => console.error('kernel panic:', new TextDecoder().decode(new Uint8Array(inst.memory.buffer, p, l))),
};
inst = new WebAssembly.Instance(new WebAssembly.Module(readFileSync(wasmPath)), { env }).exports;

function call(id, msg) {
  const body = new TextEncoder().encode(JSON.stringify({ id, msg }));
  const p = inst.ck_alloc(body.length);
  new Uint8Array(inst.memory.buffer, p, body.length).set(body);
  const total = inst.ck_request(p, body.length);
  const o = inst.ck_reply();
  const bytes = new Uint8Array(inst.memory.buffer.slice(o, o + total));
  const jl = new DataView(bytes.buffer).getUint32(0, true);
  return JSON.parse(new TextDecoder().decode(bytes.subarray(4, 4 + jl)));
}

function summarise(v) {
  if (Array.isArray(v)) return v.length > 12 && !full ? [...v.slice(0, 3).map(summarise), `... ${v.length} items`] : v.map(summarise);
  if (v && typeof v === 'object') {
    if ('$buf' in v && !full) return `<buffer ${v.type} x${v.len}>`;
    return Object.fromEntries(Object.entries(v).map(([k, x]) => [k, summarise(x)]));
  }
  return v;
}

const input = JSON.parse(readFileSync(file, 'utf8'));
(Array.isArray(input) ? input : [input]).forEach((msg, i) => {
  console.log(`// ${msg.type}${msg.tag ? ' (' + msg.tag + ')' : ''}`);
  // arrays of plain numbers or strings stay on one line
  const text = JSON.stringify(summarise(call(i + 1, msg)), null, 2)
    .replace(/\[\s+((?:-?[\d.e+-]+|"[^"\n]*")(?:,\s+(?:-?[\d.e+-]+|"[^"\n]*"))*)\s+\]/g, (_, body) => '[' + body.replace(/,\s+/g, ', ') + ']');
  console.log(text);
});
