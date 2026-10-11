import * as L from './lib.mjs';
const box = L.boxPart(), cyl = L.cylPart(), box2 = L.boxPart(5, 5, 5);
const bad3 = L.part([...box.features, { type: 'bogus', id: 'F3' }, L.extrude('F4', 'F9'), { type: 'bogus', id: 'F10' }]);
const cylF = cyl.features.map(f => ({ ...f, id: f.id === 'F1' ? 'F5' : 'F6', ...(f.sketch ? { sketch: 'F5' } : {}) }));
const mate = (id, kind, a, b, o = {}) => ({ type: 'mate', id, kind, a, b, ...o });
export const cases = {
  one: [[L.ins('F1', box)]],
  two: [[L.ins('F1', box), L.ins('F2', cyl, { dx: 100 })]],
  rot: [[L.ins('F1', box, { dx: 1, dy: 2, dz: 3, axis: { kind: 'axis', axis: 'y' }, angle: 90 })]],
  rot150: [[L.ins('F1', box, { dx: -40, dy: 30, dz: 20, axis: { kind: 'axis', axis: 'y' }, angle: 150 })]],
  rot37x: [[L.ins('F1', cyl, { dx: -4, dy: 3, dz: 2, axis: { kind: 'axis', axis: 'x' }, angle: 37 })]],
  place: [[L.ins('F1', box, { place: { o: [1, 2, 3], x: [0, 1, 0], y: [-1, 0, 0], z: [0, 0, 1] } })]],
  fixed: [[L.ins('F1', box, { fixed: true }), L.ins('F2', box, { fixed: true, dz: 50 })]],
  AAB: [[L.ins('F1', box), L.ins('F2', box, { dz: 50 }), L.ins('F3', cyl, { dz: 100 })]],
  three: [[L.ins('F1', box), L.ins('F2', cyl, { dz: 50 }), L.ins('F3', box2, { dz: 90 })]],
  partFirst: [[...cylF, L.ins('F1', box), L.ins('F2', cyl, { dz: 50 })]],
  upTo1: [[L.ins('F1', box), L.ins('F2', cyl, { dz: 50 })], { opts: { upTo: 1 } }],
  bad3: [[L.ins('F1', bad3)]],
  badTwo: [[L.ins('F1', bad3), L.ins('F2', cyl)]],
  supp: [[L.ins('F1', box), { ...L.ins('F2', cyl, { dz: 50 }), suppressed: true }]],
  empty: [[L.ins('F1', L.part([]))]],
  skonly: [[L.ins('F1', L.part([box.features[0]]))]],
  emptyThenBox: [[L.ins('F1', L.part([])), L.ins('F2', box)]],
  rollback: [[L.ins('F1', { ...box, rollback: 1 })]],
  cpat: [[L.ins('F1', box, { dx: 30 }), { type: 'cpattern', id: 'F2', seeds: [], bodies: [{ face: 'F2.C0@F1.1' }], axis: { kind: 'axis', axis: 'z' }, angle: 360, count: 4, equal: true }]],
  cpat2: [[L.ins('F1', box, { dx: 30 }), L.ins('F2', cyl, { dx: -30 }), { type: 'cpattern', id: 'F3', seeds: [], bodies: [{ face: 'F2.S.e1@F2.1' }, { face: 'F2.C0@F1.1' }], axis: { kind: 'axis', axis: 'x' }, angle: 360, count: 3, equal: true }]],
  mateNoBody: [[mate('F1', 'coincident', 'a', 'b')]],
  mateBadRef: [[L.ins('F1', box), mate('F2', 'coincident', 'F2.C0@F1.1', 'Fx')]],
};
