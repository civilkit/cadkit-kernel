// Probe sketch solver status (dof / conflicts / redundant / solved) on the ORACLE and ours.
// usage: node tools/probe_sketch_status.mjs [--orig | --ours wasm] [--status]
//   --status sends opts.sketchStatus:true (ours only); prints per case the sketch entry keys and status.
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
export const P = (id, x, y) => ({ id, x, y });
const L = (id, a, b) => ({ type: 'line', id, p1: a, p2: b });
const rectPts = [P('p1', 0, 0), P('p2', 10, 1), P('p3', 11, 5), P('p4', 1, 6)];
const rectEnts = [L('e1', 'p1', 'p2'), L('e2', 'p2', 'p3'), L('e3', 'p3', 'p4'), L('e4', 'p4', 'p1')];
const hv = [{ type: 'horizontal', id: 'c1', refs: ['e1'] }, { type: 'vertical', id: 'c2', refs: ['e2'] }, { type: 'horizontal', id: 'c3', refs: ['e3'] }, { type: 'vertical', id: 'c4', refs: ['e4'] }];
const D = (id, a, b, value) => ({ type: 'distance', id, refs: [a, b], value, offset: null });
// `want` = closed-form expectation { dof, conflicts, redundant, solved }
export const cases = {
  freePoint: { points: [P('p1', 1, 2)], entities: [], constraints: [], want: { dof: 2 } },
  rectNone: { points: rectPts, entities: rectEnts, constraints: [], want: { dof: 8 } },
  rectHV: { points: rectPts, entities: rectEnts, constraints: hv, want: { dof: 4 } },
  rectFull: { points: [P('p1', 0, 0), P('p2', 10, 0), P('p3', 10, 5), P('p4', 0, 5)], entities: rectEnts,
    constraints: [...hv, { type: 'fix', id: 'c5', refs: ['p1'], value: [0, 0] }, D('c6', 'p1', 'p2', 10), D('c7', 'p2', 'p3', 5)], want: { dof: 0 } },
  // triangle 3-4-? : all three lengths plus a position pin; the third length contradicts the first two
  overTri: { points: [P('p1', 0, 0), P('p2', 3, 0), P('p3', 0, 4)], entities: [L('e1', 'p1', 'p2'), L('e2', 'p2', 'p3'), L('e3', 'p3', 'p1')],
    constraints: [{ type: 'fix', id: 'c1', refs: ['p1'], value: [0, 0] }, { type: 'horizontal', id: 'c2', refs: ['e1'] }, { type: 'vertical', id: 'c3', refs: ['e3'] },
      D('c4', 'p1', 'p2', 3), D('c5', 'p1', 'p3', 4), D('c6', 'p2', 'p3', 7)], want: { conflicts: ['c6'] } },
  redundantDup: { points: [P('p1', 0, 0), P('p2', 10, 0)], entities: [L('e1', 'p1', 'p2')],
    constraints: [{ type: 'fix', id: 'c1', refs: ['p1'], value: [0, 0] }, { type: 'horizontal', id: 'c2', refs: ['e1'] }, { type: 'horizontal', id: 'c3', refs: ['e1'] }, D('c4', 'p1', 'p2', 10)], want: { dof: 0, redundant: ['c3'] } },
};
export const mkPart = (c) => ({ format: 'ck-part', features: [{ type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: c.points, entities: c.entities, constraints: c.constraints }] });
if (import.meta.url === `file://${process.argv[1]}`) {
  const args = process.argv.slice(2);
  const oi = args.indexOf('--ours');
  const path = oi >= 0 ? args[oi + 1] : args.includes('--orig') ? new URL('../../kernel.wasm', import.meta.url) : new URL('../target/wasm32-unknown-unknown/fast/ck_wasm.wasm', import.meta.url);
  const mod = new WebAssembly.Module(readFileSync(path));
  const opts = args.includes('--status') ? { sketchStatus: true } : undefined;
  for (const [n, c] of Object.entries(cases)) {
    const k = new Kernel(mod);
    const rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: mkPart(c), ...(opts ? { opts } : {}) } }));
    const j = JSON.parse(rep.json);
    const r = j.result ?? j;
    const sk = r.sketches?.[0]?.[1];
    console.log(n, 'errors=', JSON.stringify(r.errors), 'entryKeys=', sk && Object.keys(sk).join(','), 'status=', JSON.stringify(sk?.status), 'want=', JSON.stringify(c.want));
  }
}
