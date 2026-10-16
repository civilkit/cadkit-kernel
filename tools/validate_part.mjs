// usage: node tools/validate_part.mjs part.json [--wasm path]   regen + massProps + validate (mesh on); prints issues
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const a = process.argv.slice(2);
const wi = a.indexOf('--wasm');
const wasm = wi >= 0 ? a.splice(wi, 2)[1] : new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url);
export function check(part) {
  const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
  const rr = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, opts: { solveSketches: true } } })).json);
  const call = (msg) => JSON.parse(k.request(JSON.stringify({ id: 2, msg: { ...msg, tag: 'main' } })).json).result;
  return { errors: rr.result?.errors ?? rr, vol: call({ type: 'massProps' })?.volume, issues: call({ type: 'validate' })?.issues ?? [] };
}
if (process.argv[1].endsWith('validate_part.mjs')) {
  const r = check(JSON.parse(readFileSync(a[0], 'utf8')));
  console.log(JSON.stringify(r.errors), 'vol', r.vol);
  for (const i of r.issues) console.log(i.code, i.entity, i.detail ?? '');
  console.log(r.issues.length ? 'ISSUES ' + r.issues.length : 'CLEAN');
}
