// Compare oracle vs rebuilt vertex chamfers on the box: faces, edge keys, volume, errors.
// usage: WASM=<rebuilt.wasm> node tools/cmp_vertex_chamfer.mjs
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { parts } = await import('./brep_cases.mjs');
const A = new Kernel(new WebAssembly.Module(readFileSync(new URL('../../kernel.wasm', import.meta.url))));
const B = new Kernel(new WebAssembly.Module(readFileSync(process.env.WASM)));
const base = parts.box.features;
const e = (a, b, at) => ({ faces: [a, b], at });
const V = {
  top_p1: e('F2.C1', 'F2.S.e1', [1, 10, 0]), top_p2: e('F2.C1', 'F2.S.e1', [39, 10, 0]),
  top_p3: e('F2.C1', 'F2.S.e3', [39, 10, -20]), top_p4: e('F2.C1', 'F2.S.e3', [1, 10, -20]),
  bot_p1: e('F2.C0', 'F2.S.e1', [1, 0, 0]), bot_p2: e('F2.C0', 'F2.S.e1', [39, 0, 0]),
  bot_p3: e('F2.C0', 'F2.S.e3', [39, 0, -20]), bot_p4: e('F2.C0', 'F2.S.e3', [1, 0, -20]),
};
const cases = [];
for (const [n, ed] of Object.entries(V)) {
  cases.push([n, { distance: 2, distance2: 3, distance3: 4, edges: [ed] }]);
  cases.push([n + '_eq', { distance: 2, edges: [ed] }]);
}
cases.push(['at_only', { distance: 2, distance2: 3, distance3: 4, at: [40, 10, 0] }]);
cases.push(['both', { distance: 2, distance2: 3, distance3: 4, at: [40, 10, 0], edges: [V.top_p1] }]);
for (const d of [9.99, 10]) cases.push(['d=' + d, { distance: d, edges: [V.top_p1] }]);
for (const d of [19, 21]) cases.push(['d2=' + d, { distance: 2, distance2: d, edges: [V.top_p1] }]);
cases.push(['d3zero', { distance: 2, distance2: 2, distance3: 0, edges: [V.top_p1] }]);
cases.push(['noat', { distance: 2 }]);
cases.push(['noedges', { distance: 2, edges: [] }]);
const run = (k, f) => {
  const part = { format: 'ck-part', version: 1, features: [...base, { type: 'chamfer', id: 'F3', chamferType: 'vertex', ...f }] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const b = r.result?.body;
  const mp = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result;
  return { err: JSON.stringify(r.result?.errors ?? []), faces: b?.faces.map((x) => x.id).join(' '), keys: b?.edges.map((x) => x.key).join(' '), vol: mp?.volume };
};
let ok = 0;
for (const [n, f] of cases) {
  const a = run(A, f), b = run(B, f);
  const sameV = Math.abs(a.vol - b.vol) < 1e-9 * Math.abs(a.vol);
  const same = a.faces === b.faces && a.keys === b.keys && sameV && a.err === b.err;
  if (same) ok++;
  else {
    console.log(n.padEnd(12), a.faces === b.faces ? 'F ok' : 'F DIFF', a.keys === b.keys ? 'K ok' : 'K DIFF', sameV ? 'V ok' : `V DIFF ${b.vol}/${a.vol}`, a.err === b.err ? '' : `ERR ${b.err} / ${a.err}`);
    if (process.env.V) console.log('  orig', a.faces, '\n  mine', b.faces, '\n  orig', a.keys, '\n  mine', b.keys);
  }
}
console.log(`${ok}/${cases.length} identical`);
