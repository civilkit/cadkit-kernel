// Probe the ORIGINAL kernel with box/cylinder + shell. usage: node tools/shellq.mjs <part> '<faces json>;<thickness>[;outward]' ...
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './brep_cases.mjs';
const [name, ...cases] = process.argv.slice(2);
const wasm = process.env.WASM ?? new URL('../../kernel.wasm', import.meta.url);
for (const c of cases) {
const [faces, th, outward] = c.split(';');
console.log('==', c);
const p = JSON.parse(JSON.stringify(parts[name]));
p.features.push({ type: 'shell', id: 'F9', faces: JSON.parse(faces), thickness: +th, ...(outward ? { outward: true } : {}) });
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const r = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: p } }));
const d = JSON.parse(r.json).result;
console.log('errors', JSON.stringify(d.errors));
const b = d.body;
if (b) {
  console.log('faces', b.faces.length, b.faces.map((f) => f.id ?? f.name ?? JSON.stringify(f).slice(0, 60)).join(' '));
  console.log('edges', b.edges.length, JSON.stringify(b.edges.slice(0, 3)).slice(0, 600));
  if (process.env.V) console.log(JSON.stringify(b.faces).slice(0, +process.env.V));
}
const m = k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } }));
console.log(m.json.slice(0, 400));
}
