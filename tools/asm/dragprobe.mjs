// Probe the ORIGINAL kernel's `drag` on small assemblies.
// usage: node tools/asm/dragprobe.mjs
import * as L from './lib.mjs';
import { Kernel } from '../kernel.mjs';
const box = L.boxPart(), cyl = L.cylPart();
const mate = (id, kind, a, b, o = {}) => ({ type: 'mate', id, kind, a, b, ...o });
const F = (o = {}) => L.ins('F1', box, { fixed: true, ...o });
const M = (o = {}) => L.ins('F2', box, { dx: 100, dy: 7, dz: 3, ...o });
const asms = {
  free: [F(), M()],
  coinc: [F(), M(), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')],
  dist: [F(), M(), mate('F3', 'distance', 'F2.C0@F2.1', 'F2.C1@F1.1', { distance: 5 })],
  conc: [L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1')],
  chain: [L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1'), L.ins('F4', box, { dx: 50 }), mate('F5', 'coincident', 'F2.C0@F4.1', 'F2.C1@F2.1')],
  two: [L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1'), mate('F4', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')],
};
const drags = {
  free: [['F2.C1@F2.1', [100, 7, 13], [110, 12, 13]], ['F2.C1@F2.1', [100, 7, 13], [100, 7, 13]], ['F1.C1@F1.1', [0, 0, 0], [1, 1, 1]], ['F2.C1@F2.1', [100, 7, 13], [100, 7, 20]]],
  coinc: [['F2.C1@F2.1', [20, 10, -10], [25, 10, -10]], ['F2.C1@F2.1', [20, 10, -10], [20, 20, -10]], ['F2.S.e1@F2.1', [20, 5, 0], [20, 5, 8]]],
  dist: [['F2.C1@F2.1', [20, 15, -10], [25, 15, -10]]],
  conc: [['F2.S.e1@F2.1', [5, 5, 0], [5, 9, 3]], ['F2.S.e1@F2.1', [5, 5, 0], [5, 5, 8]], ['F2.S.e1@F2.1', [5, 5, 0], [9, 5, 0]]],
  chain: [['F2.S.e1@F2.1', [5, 5, 0], [5, 5, 8]], ['F2.C1@F4.1', [60, 10, -10], [70, 10, -10]], ['F2.C0@F4.1', [60, 10, 0], [60, 10, 10]]],
  two: [['F2.S.e1@F2.1', [5, 5, 0], [5, 5, 8]], ['F2.S.e1@F2.1', [5, 5, 0], [5, 9, 3]]],
};
const mod = L.A;
for (const [name, feats] of Object.entries(asms)) {
  const k = new Kernel(mod);
  const reg = JSON.parse(k.request(JSON.stringify({ id: 1, msg: L.regen(L.part(feats)) })).json);
  console.log('==', name, 'errors', JSON.stringify(reg.result?.errors), 'components', JSON.stringify(reg.result?.components));
  for (const [face, point, target] of drags[name] || []) {
    const r = k.request(JSON.stringify({ id: 2, msg: { type: 'drag', tag: 'main', face, point, target } }));
    const j = JSON.parse(r.json).result;
    const c = (m) => m && m.o ? { o: m.o.map((x) => +x.toPrecision(5)), x: m.x.map((x) => +x.toPrecision(4)), z: m.z.map((x) => +x.toPrecision(4)) } : m;
    console.log('  drag', face, JSON.stringify(point), '->', JSON.stringify(target), '\n    ', JSON.stringify({ insert: j.insert, error: j.error, delta: c(j.delta), place: c(j.place), moves: (j.moves || []).map((m) => ({ id: m.id, delta: c(m.delta), place: c(m.place) })) }));
  }
}
