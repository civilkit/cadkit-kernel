// usage: node tools/probe_part.mjs <case-name|file.json> [--rebuilt wasm] [--full]
// Regens a brep_cases part (or a JSON part file) on the ORIGINAL (default) or a
// rebuilt wasm and prints errors, body/surface face+edge summaries, massProps.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const args = process.argv.slice(2);
const take = (f, v) => { const i = args.indexOf(f); if (i < 0) return null; const x = v ? args[i + 1] : true; args.splice(i, v ? 2 : 1); return x; };
const wasm = take('--rebuilt', true) ?? new URL('../../kernel.wasm', import.meta.url);
const full = take('--full');
let part;
if (args[0].endsWith('.json') || args[0].endsWith('.ckpart')) part = JSON.parse(readFileSync(args[0], 'utf8'));
else part = (await import('./brep_cases.mjs')).parts[args[0]];
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true } }));
if (!rep.json) { console.log('TRAP', rep.trap); process.exit(1); }
const r = JSON.parse(rep.json);
console.log('errors', JSON.stringify(r.result?.errors));
const sum = (b, nm) => {
  if (!b) return;
  const kinds = {};
  for (const f of b.faces) { const t = f.surf?.type ?? f.type ?? '?'; kinds[t] = (kinds[t] ?? 0) + 1; }
  console.log(nm, 'faces', b.faces.length, 'edges', b.edges.length, JSON.stringify(kinds));
  if (full) { for (const f of b.faces) console.log(' F', JSON.stringify(f).slice(0, 260)); }
};
sum(r.result?.body, 'body');
for (const [id, s] of r.result?.surfaces ?? []) sum(s, 'surface ' + id);
console.log('keys', Object.keys(r.result ?? {}).join(','));
const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
console.log('mass', JSON.stringify(m));
