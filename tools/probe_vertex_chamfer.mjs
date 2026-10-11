// Probe the vertex chamfer (chamferType "vertex") on the 40x20x10 box.
// usage: node tools/probe_vertex_chamfer.mjs [rebuilt.wasm]
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { parts } = await import('./brep_cases.mjs');
const wasm = process.argv[2] ?? new URL('../../kernel.wasm', import.meta.url);
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const base = parts.box.features;
const e = (a, b, at) => ({ faces: [a, b], at });
const V = {
  e1mid: { chamferType: 'vertex', distance: 2, edges: [e('F2.C1', 'F2.S.e1', [20, 10, 0])] },
  e1near1: { chamferType: 'vertex', distance: 2, edges: [e('F2.C1', 'F2.S.e1', [39, 10, 0])] },
  e1near0: { chamferType: 'vertex', distance: 2, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  d123: { chamferType: 'vertex', distance: 2, distance2: 3, distance3: 4, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  d2only: { chamferType: 'vertex', distance: 2, distance2: 3, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  two: { chamferType: 'vertex', distance: 2, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0]), e('F2.C1', 'F2.S.e4', [0, 10, -1])] },
  three: { chamferType: 'vertex', distance: 2, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0]), e('F2.C1', 'F2.S.e4', [0, 10, -1]), e('F2.S.e1', 'F2.S.e4', [0, 9, 0])] },
  at: { chamferType: 'vertex', distance: 2, at: [40, 10, 0] },
  atEdges: { chamferType: 'vertex', distance: 2, at: [40, 10, 0], edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  noEdges: { chamferType: 'vertex', distance: 2, edges: [] },
  big: { chamferType: 'vertex', distance: 50, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  zero: { chamferType: 'vertex', distance: 0, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  neg2: { chamferType: 'vertex', distance: 2, distance2: -1, edges: [e('F2.C1', 'F2.S.e1', [1, 10, 0])] },
  vertexEdgesPts: { chamferType: 'vertex', distance: 2, vertexEdges: 'points', points: [[0, 10, 0], [5, 10, 0], [0, 7, 0]] },
};
for (const [n, f] of Object.entries(V)) {
  const part = { format: 'ck-part', version: 1, features: [...base, { type: 'chamfer', id: 'F3', ...f }] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true } })).json);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
  console.log(n.padEnd(14), JSON.stringify(r.result?.errors), 'faces', r.result?.body?.faces?.length, 'removed', (8000 - m.volume).toFixed(6), 'com', m.com.map((x) => +x.toFixed(4)).join(','));
}
