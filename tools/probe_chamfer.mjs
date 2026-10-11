// Probe the chamfer feature variants (errors + trace) on the original or a rebuilt wasm.
// usage: node tools/probe_chamfer.mjs [rebuilt.wasm]
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { parts } = await import('./brep_cases.mjs');
const wasm = process.argv[2] ?? new URL('../../kernel.wasm', import.meta.url);
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const base = parts.box.features;
const e1 = [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }];
const variants = {
  vertexNoEdges: { chamferType: 'vertex', distance: 2, edges: e1 },
  vertexPts: { chamferType: 'vertex', distance: 2, edges: e1, vertexEdges: 'points' },
  flip: { chamferType: 'equal', distance: 2, flip: true, edges: e1 },
  d0: { chamferType: 'equal', distance: 0, edges: e1 },
  dneg: { chamferType: 'equal', distance: -1, edges: e1 },
  noedges: { chamferType: 'equal', distance: 1, edges: [] },
  unknownType: { chamferType: 'foo', distance: 1, edges: e1 },
  notype: { distance: 1, edges: e1 },
  dd_nodist2: { chamferType: 'distanceDistance', distance: 1, edges: e1 },
  ad_noangle: { chamferType: 'angleDistance', distance: 1, edges: e1 },
  vertexNoEdgesAtAll: { chamferType: 'vertex', distance: 1, edges: [] },
  vertexVerts: { chamferType: 'vertex', distance: 1, edges: [], vertices: [[0, 0, 0]] },
  badEdge: { chamferType: 'equal', distance: 1, edges: [{ faces: ['F2.C1', 'F2.S.e9'], at: [20, 10, 0] }] },
};
for (const [n, f] of Object.entries(variants)) {
  const part = { format: 'ck-part', version: 1, features: [...base, { type: 'chamfer', id: 'F3', ...f }] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true } })).json);
  console.log(n.padEnd(14), JSON.stringify(r.result?.errors), (r.trace ?? []).map((t) => t.label).filter((l) => l.startsWith('F3')).join(' | '), r.result?.body?.faces?.length);
}
