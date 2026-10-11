// hole feature probes on a 40×20×10 top-plane box (sketch points on its top cap F2.C1)
import * as L from './lib.mjs';
const [P, E] = L.rect(0, 0, 40, 20);
const base = [L.sketch('F1', 'top', P, E), L.extrude('F2', 'F1', { depth: 10 })];
const pts = (id, xy) => ({ type: 'sketch', id, plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, ...xy.map(([x, y], i) => ({ id: 'p' + i, x, y }))], entities: xy.map((_, i) => ({ id: 'e' + (i + 2), type: 'point', p: 'p' + i })), constraints: [] });
const hole = (o) => ({ type: 'hole', id: 'F4', sketch: 'F3', standard: 'iso', size: 'M8', fit: 'normal', kind: 'simple', end: 'through', ...o });
const H = (o, xy = [[10, 10]]) => [[...base, pts('F3', xy), hole(o)]];
export const cases = {
  m8through: H({}),
  m6tapBlind: H({ size: 'M6', fit: 'tap', end: 'blind', depth: 8, tip: false }),
  m6tapTip: H({ size: 'M6', fit: 'tap', end: 'blind', depth: 8, tip: true }),
  m8blindTip: H({ end: 'blind', depth: 6, tip: true }),
  m3cbore: H({ size: 'M3', kind: 'cbore' }),
  m3csink: H({ size: 'M3', kind: 'csink' }),
  m8close: H({ fit: 'close' }),
  m8loose: H({ fit: 'loose' }),
  m3tapThrough: H({ size: 'M3', fit: 'tap' }),
  four: H({ size: 'M3', fit: 'tap' }, [[5, 5], [35, 5], [35, 15], [5, 15]]),
  two: H({}, [[10, 10], [30, 10]]),
  diam: H({ size: undefined, diameter: 7 }),
  noPts: [[...base, pts('F3', []), hole({})]],
  flip: H({ flip: true }),
};
