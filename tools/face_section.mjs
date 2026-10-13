// usage: node tools/face_section.mjs <stub-case> <face-id-substring> [--ours] [axis=x] [pos]
// Prints the mesh vertices of a face of the regen body whose coordinate `axis` is within 1e-6 of
// the median of the face (a cross-section of a swept / blended face), sorted along the section.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
import { parts } from './stub_cases.mjs';

const args = process.argv.slice(2);
const oi = args.indexOf('--ours');
let wasm = new URL('../../kernel.wasm', import.meta.url);
if (oi >= 0) { wasm = new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url); args.splice(oi, 1); }
const [name, sub, axis = 'x', pos] = args;
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: parts[name] } }));
const r = JSON.parse(rep.json);
const b = r.result.body;
const jl = new DataView(rep.bytes.buffer).getUint32(0, true);
const base = (4 + jl + 7) & ~7;
const m = b.mesh;
const get = (v) => { const C = { f32: Float32Array, i32: Int32Array }[v.type]; return new C(rep.bytes.buffer.slice(rep.bytes.byteOffset + base + v.offset, rep.bytes.byteOffset + base + v.offset + v.len * C.BYTES_PER_ELEMENT)); };
const pos3 = get(m.positions ?? m.pos), tf = get(m.triFace);
const ids = b.faces.map((f) => f.id);
const fi = ids.findIndex((x) => x.includes(sub));
if (fi < 0) { console.log('faces', ids.join(' ')); process.exit(1); }
const ai = { x: 0, y: 1, z: 2 }[axis];
const pts = [];
for (let t = 0; t < tf.length; t++) if (tf[t] === fi) for (let c = 0; c < 3; c++) pts.push([pos3[t * 9 + c * 3], pos3[t * 9 + c * 3 + 1], pos3[t * 9 + c * 3 + 2]]);
const vals = pts.map((p) => p[ai]).sort((a, b2) => a - b2);
const at = pos !== undefined ? Number(pos) : vals[Math.floor(vals.length / 2)];
const sec = [];
const seen = new Set();
for (const p of pts) if (Math.abs(p[ai] - at) < 1e-4) { const key = p.map((x) => x.toFixed(5)).join(); if (!seen.has(key)) { seen.add(key); sec.push(p); } }
const o = [0, 1, 2].filter((i) => i !== ai);
sec.sort((p, q) => p[o[0]] - q[o[0]] || p[o[1]] - q[o[1]]);
console.log(`face ${ids[fi]} section ${axis}=${at.toFixed(4)} (${sec.length} pts)`);
for (const p of sec) console.log(p.map((x) => x.toFixed(6)).join('  '));
