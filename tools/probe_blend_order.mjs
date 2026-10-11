// Probe the face / edge order of chamfers & fillets on the 40x20x10 box, for arbitrary
// edge selections: node tools/probe_blend_order.mjs [wasm] ; edit SETS below.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { parts } = await import('./brep_cases.mjs');
const wasm = process.env.WASM ?? new URL('../../kernel.wasm', import.meta.url);
const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)));
const base = parts.box.features;
const E = {
  t1: { faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }, t2: { faces: ['F2.C1', 'F2.S.e2'], at: [40, 10, -10] },
  t3: { faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }, t4: { faces: ['F2.C1', 'F2.S.e4'], at: [0, 10, -10] },
  b1: { faces: ['F2.C0', 'F2.S.e1'], at: [20, 0, 0] }, v12: { faces: ['F2.S.e1', 'F2.S.e2'], at: [40, 5, 0] },
  b3: { faces: ['F2.C0', 'F2.S.e3'], at: [20, 0, -20] }, b2: { faces: ['F2.C0', 'F2.S.e2'], at: [40, 0, -10] },
  v23: { faces: ['F2.S.e2', 'F2.S.e3'], at: [40, 5, -20] }, v34: { faces: ['F2.S.e3', 'F2.S.e4'], at: [0, 5, -20] },
  v41: { faces: ['F2.S.e1', 'F2.S.e4'], at: [0, 5, 0] },
};
const sets = (process.argv[2] ?? 't1,t2;t2,t1;t1,t3;t1,t2,t3;t1,t2,t3,t4;t2,t3,t4,t1;t4,t3,t2,t1').split(';');
const type = process.env.TYPE ?? 'chamfer';
for (const s of sets) {
  const edges = s.split(',').map((n) => E[n]);
  const f = type === 'chamfer' ? { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 2, edges } : { type: 'fillet', id: 'F3', radius: 2, edges };
  const part = { format: 'ck-part', version: 1, features: [...base, f] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true } })).json);
  const b = r.result?.body;
  const short = (id) => id.replace('F2.', '').replace('F3.', '').replace('S.', '');
  console.log(s.padEnd(16), JSON.stringify(r.result?.errors) !== '[]' ? JSON.stringify(r.result?.errors) : '',
    'F:', b?.faces.map((f) => short(f.id)).join(' '));
  if (process.env.EDGES) console.log('   E:', b?.edges.map((e) => short(e.key.split('#')[0].replace('|', '|')) + '#' + e.key.split('#')[1]).join(' '));
}
