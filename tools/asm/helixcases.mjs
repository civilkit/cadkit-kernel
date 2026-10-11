import * as L from './lib.mjs';
const circ = (id, plane, cx, cy, r) => ({ type: 'sketch', id, plane, points: [{ id: 'c', x: cx, y: cy }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
const hx = (o) => ({ type: 'helix', id: 'F2', sketch: 'F1', definedBy: 'pitchRevs', pitch: 1.25, revolutions: 19, height: 23.75, startAngle: 0, leftHand: false, reverse: false, ...o });
export const cases = {
  bolt: [[{ type: 'plane', id: 'F9', mode: 'offset', base: { kind: 'std', id: 'front' }, offset: 18 }, { ...circ('F1', { kind: 'feature', id: 'F9' }, 0, 0, 4) }, hx({})]],
  top: [[circ('F1', L.STD('top'), 3, 4, 5), hx({ revolutions: 3, startAngle: 30, leftHand: true })]],
  noCircle: [[{ type: 'sketch', id: 'F1', plane: L.STD('top'), points: [{ id: 'a', x: 0, y: 0 }], entities: [], constraints: [] }, hx({})]],
  heightRevs: [[circ('F1', L.STD('right'), 1, 2, 3), hx({ definedBy: 'heightRevs', reverse: true })]],
};
