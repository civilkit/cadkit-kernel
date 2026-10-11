// Mate probes (two/three-component assemblies), one per mate kind / role.
import * as L from './lib.mjs';
const box = L.boxPart(), cyl = L.cylPart();
const mate = (id, kind, a, b, o = {}) => ({ type: 'mate', id, kind, a, b, ...o });
const F = (o = {}) => L.ins('F1', box, { fixed: true, ...o });
const M = (o = {}) => L.ins('F2', box, { dx: 100, dy: 7, dz: 3, ...o });
export const cases = {
  coinc: [[F(), M(), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  coincRev: [[F(), M(), mate('F3', 'coincident', 'F2.C1@F1.1', 'F2.C0@F2.1')]],
  coincFlip: [[F(), M(), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1', { flip: true })]],
  coincNoFix: [[L.ins('F1', box), M(), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  coincSame: [[F(), M(), mate('F3', 'coincident', 'F2.C1@F2.1', 'F2.C1@F1.1')]],
  coincTilt: [[F(), M({ axis: { kind: 'axis', axis: 'z' }, angle: 20 }), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  dist: [[F(), M(), mate('F3', 'distance', 'F2.C0@F2.1', 'F2.C1@F1.1', { distance: 5 })]],
  distFlip: [[F(), M(), mate('F3', 'distance', 'F2.C0@F2.1', 'F2.C1@F1.1', { distance: 5, flip: true })]],
  par: [[F(), M({ axis: { kind: 'axis', axis: 'x' }, angle: 30 }), mate('F3', 'parallel', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  conc: [[L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1')]],
  concTilt: [[L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3, axis: { kind: 'axis', axis: 'x' }, angle: 30 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1')]],
  concAnti: [[L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3, axis: { kind: 'axis', axis: 'x' }, angle: 170 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1')]],
  chain: [[L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1'), L.ins('F4', box, { dx: 50 }), mate('F5', 'coincident', 'F2.C0@F4.1', 'F2.C1@F2.1')]],
  two: [[L.ins('F1', cyl, { fixed: true }), L.ins('F2', cyl, { dx: 100, dy: 7, dz: 3 }), mate('F3', 'concentric', 'F2.S.e1@F2.1', 'F2.S.e1@F1.1'), mate('F4', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  same: [[F(), M(), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F2.1')]],
  bothFixed: [[F(), M({ fixed: true }), mate('F3', 'coincident', 'F2.C0@F2.1', 'F2.C1@F1.1')]],
  cylPlane: [[F(), L.ins('F2', cyl, { dx: 100 }), mate('F3', 'coincident', 'F2.S.e1@F2.1', 'F2.C1@F1.1')]],
};
