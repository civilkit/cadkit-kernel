// Compare oracle vs rebuilt face order + edge keys for many edge selections on the box.
// usage: WASM=<rebuilt.wasm> TYPE=fillet|chamfer node tools/cmp_blend_sets.mjs [set;set;...|all4|all3top|mix]
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { parts } = await import('./brep_cases.mjs');
const A = new Kernel(new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url))));
const B = new Kernel(new WebAssembly.Module(readFileSync(process.env.WASM)));
const base = parts.box.features;
const E = {
  t1: { faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }, t2: { faces: ['F2.C1', 'F2.S.e2'], at: [40, 10, -10] },
  t3: { faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }, t4: { faces: ['F2.C1', 'F2.S.e4'], at: [0, 10, -10] },
  b1: { faces: ['F2.C0', 'F2.S.e1'], at: [20, 0, 0] }, b2: { faces: ['F2.C0', 'F2.S.e2'], at: [40, 0, -10] },
  b3: { faces: ['F2.C0', 'F2.S.e3'], at: [20, 0, -20] }, b4: { faces: ['F2.C0', 'F2.S.e4'], at: [0, 0, -10] },
  v12: { faces: ['F2.S.e1', 'F2.S.e2'], at: [40, 5, 0] }, v23: { faces: ['F2.S.e2', 'F2.S.e3'], at: [40, 5, -20] },
  v34: { faces: ['F2.S.e3', 'F2.S.e4'], at: [0, 5, -20] }, v41: { faces: ['F2.S.e1', 'F2.S.e4'], at: [0, 5, 0] },
};
const perms = (a, k) => (k === 0 ? [[]] : a.flatMap((x, i) => perms([...a.slice(0, i), ...a.slice(i + 1)], k - 1).map((p) => [x, ...p])));
let sets;
const arg = process.argv[2] ?? 'all4';
if (arg === 'all4') sets = [2, 3, 4].flatMap((k) => perms(['t1', 't2', 't3', 't4'], k));
else if (arg === 'mix') sets = [['t1', 'v12'], ['v12', 't1'], ['t1', 'v41'], ['v41', 't1'], ['v12', 'v23'], ['v23', 'v12'], ['t1', 'b2'], ['b2', 't1'], ['t1', 't2', 'v12'], ['v12', 't2', 't1'], ['t1', 'v23'], ['v12', 'b1'], ['b1', 'v12', 't1']];
else sets = arg.split(';').map((s) => s.split(','));
const type = process.env.TYPE ?? 'fillet';
const run = (k, edges) => {
  const f = type === 'chamfer' ? { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 2, edges } : { type: 'fillet', id: 'F3', radius: 2, edges };
  const part = { format: 'ck-part', version: 1, features: [...base, f] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const b = r.result?.body;
  const mp = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
  return { err: JSON.stringify(r.result?.errors ?? []), faces: b?.faces.map((x) => x.id.replace('F2.', '').replace('F3.', '').replace('S.', '')).join(' '), keys: b?.edges.map((x) => x.key.replace(/F2\./g, '').replace(/F3\./g, '').replace(/S\./g, '')).join(' '), vol: mp?.volume };
};
let ok = 0, n = 0;
for (const s of sets) {
  const edges = s.map((x) => E[x]);
  const a = run(A, edges), b = run(B, edges);
  n++;
  const sameF = a.faces === b.faces, sameK = a.keys === b.keys, sameV = Math.abs(a.vol - b.vol) < 1e-6 * Math.abs(a.vol);
  if (sameF && sameK && sameV && a.err === b.err) { ok++; continue; }
  console.log(s.join(',').padEnd(14), sameF ? 'F ok' : 'F DIFF', sameK ? 'K ok' : 'K DIFF', sameV ? 'V ok' : `V DIFF ${b.vol}/${a.vol}`, a.err === b.err ? '' : `ERR ${b.err} / ${a.err}`);
  if (!sameF && process.env.V) console.log('   orig', a.faces, '\n   mine', b.faces);
  if (!sameK && process.env.V) console.log('   orig', a.keys, '\n   mine', b.keys);
}
console.log(`${ok}/${n} identical`);
