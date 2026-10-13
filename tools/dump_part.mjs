// usage: node tools/dump_part.mjs <stub-case-name> [--ours [wasm]] [key ...]
// Prints the regen reply's result keys (default: errors curves markers) of a
// tools/stub_cases.mjs part from the ORIGINAL (or --ours) kernel; `body` /
// `surfaces` print one line per face and edge.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './stub_cases.mjs';

const args = process.argv.slice(2);
const oi = args.indexOf('--ours');
let wasm = new URL('../../kernel.wasm', import.meta.url);
if (oi >= 0) {
  const next = args[oi + 1];
  const explicit = next && next.endsWith('.wasm');
  wasm = explicit ? next : new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url);
  args.splice(oi, explicit ? 2 : 1);
}
const name = args.shift();
const keys = args.length ? args : ['errors', 'curves', 'markers'];
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: parts[name], debug: true } })).json);
const res = r.result;
const bodyLines = (b, tag) => {
  for (const f of b.faces) console.log(tag, 'F', JSON.stringify(f).slice(0, 300));
  for (const e of b.edges) console.log(tag, 'E', JSON.stringify({ ...e, pts: e.pts.length }).slice(0, 300));
};
for (const key of keys) {
  if (key === 'body' && res.body) bodyLines(res.body, 'body');
  else if (key === 'surfaces') for (const [id, s] of res.surfaces ?? []) bodyLines(s, `surf ${id}`);
  else console.log(key, JSON.stringify(res[key]).slice(0, 3000));
}
console.log('result keys', Object.keys(res).join());
