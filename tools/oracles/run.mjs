// usage: node tools/oracles/run.mjs [--wasm path] [caseName ...]
// Sweeps the oracles against the rebuilt wasm and prints every disagreement.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';
import { parts } from '../brep_cases.mjs';
import { oracles } from './index.mjs';
import { checkOracle } from './check.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const wasm = wi >= 0 ? args.splice(wi, 2)[1] : path.join(here, '../../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const M = new WebAssembly.Module(readFileSync(wasm));
let n = 0, bad = 0;
for (const name of args.length ? args : Object.keys(oracles)) {
  const k = new Kernel(M);
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: parts[name] } })).json);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const d = checkOracle(oracles[name], m.result, r.result?.body?.box);
  n++;
  if (d.length) { bad++; console.log(`DISAGREE ${name}: ${d.join(' | ')}`); }
}
console.log(`oracle: ${n - bad}/${n} agree`);
