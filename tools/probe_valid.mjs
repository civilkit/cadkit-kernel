// usage: node tools/probe_valid.mjs <brep case name | path.ckpart> [upTo] [--orig]
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const a = process.argv.slice(2);
const orig = a.includes('--orig'); const args = a.filter((x) => x !== '--orig');
const wasm = orig ? new URL('../../kernel.wasm', import.meta.url) : 'target/wasm32-unknown-unknown/fast/ck_wasm.wasm';
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
let part, upTo;
if (args[0].endsWith('.ckpart')) { part = JSON.parse(readFileSync(args[0], 'utf8')); upTo = +args[1]; }
else part = (await import('./brep_cases.mjs')).parts[args[0]];
const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(upTo ? { opts: { upTo } } : {}) } })).json);
const b = r.result?.body;
console.log('errors', JSON.stringify(r.result?.errors), 'faces', b?.faces?.length, 'edges', b?.edges?.length);
const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
console.log('mass', JSON.stringify(m));
if (!orig) console.log(JSON.stringify(JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json).result, null, 1));
else for (const e of b?.edges ?? []) if (e.faces?.length !== 2) console.log('nonmanifold edge', JSON.stringify({ ...e, pts: e.pts.length }));
