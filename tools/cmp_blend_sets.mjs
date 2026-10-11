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
else if (arg === 'corners') {
  // every box vertex with its three edges, in all 6 orders
  const verts = { T1: ['t4', 't1', 'v41'], T2: ['t1', 't2', 'v12'], T3: ['t2', 't3', 'v23'], T4: ['t3', 't4', 'v34'],
    B1: ['b4', 'b1', 'v41'], B2: ['b1', 'b2', 'v12'], B3: ['b2', 'b3', 'v23'], B4: ['b3', 'b4', 'v34'] };
  sets = Object.values(verts).flatMap((tri) => perms(tri, 3));
} else if (arg === 'pairs') {
  const names = Object.keys(E);
  sets = [];
  for (const a of names) for (const b of names) if (a !== b) sets.push([a, b]);
} else if (arg.startsWith('rand')) {
  // rand:N:seed -> N random ordered selections of 2-5 box edges (no three edges at one vertex)
  const [, nn, seed] = arg.split(':');
  let st = +(seed ?? 1);
  const rnd = () => { st = (st * 1664525 + 1013904223) % 4294967296; return st / 4294967296; };
  const names = Object.keys(E);
  const ends = { t1: ['T1', 'T2'], t2: ['T2', 'T3'], t3: ['T3', 'T4'], t4: ['T4', 'T1'], b1: ['B1', 'B2'], b2: ['B2', 'B3'], b3: ['B3', 'B4'], b4: ['B4', 'B1'],
    v12: ['B2', 'T2'], v23: ['B3', 'T3'], v34: ['B4', 'T4'], v41: ['B1', 'T1'] };
  sets = [];
  while (sets.length < +nn) {
    const k = 2 + Math.floor(rnd() * 4);
    const pick = [];
    while (pick.length < k) { const n = names[Math.floor(rnd() * names.length)]; if (!pick.includes(n)) pick.push(n); }
    const deg = {};
    for (const n of pick) for (const v of ends[n]) deg[v] = (deg[v] ?? 0) + 1;
    if (Object.values(deg).some((d) => d >= 3)) continue;
    sets.push(pick);
  }
} else if (arg === 'mix') sets = [['t1', 'v12'], ['v12', 't1'], ['t1', 'v41'], ['v41', 't1'], ['v12', 'v23'], ['v23', 'v12'], ['t1', 'b2'], ['b2', 't1'], ['t1', 't2', 'v12'], ['v12', 't2', 't1'], ['t1', 'v23'], ['v12', 'b1'], ['b1', 'v12', 't1']];
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
