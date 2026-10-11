// usage: node tools/probe_case.mjs <brep case name> [--rebuilt wasm]
// dumps faces/edges summary + errors + massProps of the ORIGINAL (or rebuilt) kernel
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const args = process.argv.slice(2);
const wi = args.indexOf('--rebuilt');
let wasm = new URL('../../kernel.wasm', import.meta.url);
if (wi >= 0) { wasm = args[wi + 1]; args.splice(wi, 2); }
const { parts } = await import('./brep_cases.mjs');
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: parts[args[0]], debug: true } })).json);
const b = r.result?.body;
console.log('errors', JSON.stringify(r.result?.errors), 'trace', (r.trace ?? r.result?.trace ?? []).map((t) => t.label).join(' | '));
if (b) {
  for (const f of b.faces) console.log('F', JSON.stringify(f).slice(0, 300));
  for (const e of b.edges) console.log('E', JSON.stringify({ ...e, pts: e.pts.length }).slice(0, 300));
}
const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
console.log('mass', JSON.stringify(m));
