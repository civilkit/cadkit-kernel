// Test assembly: a base (plate + rod, fixed), a carriage sliding on the rod, an arm hinged on the carriage's pin, and a free ball.
// Sketch planes: front (u, v) -> (x, y), extrude +z; top (u, v) -> (x, -z), extrude +y; right (u, v) -> (-z, y), extrude +x.
import type { AssemblyDoc } from './analyze.js';

type P = { id: string; x: number; y: number };
function sketch(id: string, plane: string, rects: Array<[number, number, number, number]>, circles: Array<[number, number, number]>) {
  const points: P[] = [], entities: Array<Record<string, unknown>> = [];
  let e = 1;
  rects.forEach(([x0, y0, x1, y1], i) => {
    const ids = ['a', 'b', 'c', 'd'].map((s) => `${s}${i}`);
    points.push({ id: ids[0], x: x0, y: y0 }, { id: ids[1], x: x1, y: y0 }, { id: ids[2], x: x1, y: y1 }, { id: ids[3], x: x0, y: y1 });
    for (let k = 0; k < 4; k++) entities.push({ id: `e${e++}`, type: 'line', p1: ids[k], p2: ids[(k + 1) % 4] });
  });
  circles.forEach(([x, y, r], i) => { points.push({ id: `c${i}`, x, y }); entities.push({ id: `e${e++}`, type: 'circle', c: `c${i}`, r }); });
  return { type: 'sketch', id, plane: { kind: 'std', id: plane }, points, entities, constraints: [] };
}
const part = (name: string, material: string, features: unknown[]) => ({ format: 'ck-part', version: 1, name, material, seq: features.length + 1, rollback: null, counters: {}, features });

/** Base: plate x 0..200, y -20..20, z 0..5; rod r 5 on the x axis at z 15, x 0..200 (two solids). */
export const basePart = () => part('Base', 'steel', [
  sketch('F1', 'front', [[0, -20, 200, 20]], []),
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 5 },
  sketch('F3', 'right', [], [[-15, 0, 5]]),
  { type: 'extrude', id: 'F4', op: 'add', sketch: 'F3', end: 'blind', depth: 200 },
]);
/** Carriage: block x -15..15, y -12..12, z 8..35 with a bore r 5.2 at z 15 along x; pin r 3 along y at x 0, z 28, y 0..25. */
export const carriagePart = () => part('Carriage', 'aluminium', [
  sketch('F1', 'right', [[-35, -12, -8, 12]], []),
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'mid', depth: 30 },
  sketch('F3', 'right', [], [[-15, 0, 5.2]]),
  { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth', depth: 30 },
  sketch('F5', 'top', [], [[0, -28, 3]]),
  { type: 'extrude', id: 'F6', op: 'add', sketch: 'F5', end: 'blind', depth: 25 },
]);
/** Arm: plate x -6..60, z -6..6, y 0..4, bore r 3.1 at the origin along y. */
export const armPart = () => part('Arm', 'steel', [
  sketch('F1', 'top', [[-6, -6, 60, 6]], []),
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 4 },
  sketch('F3', 'top', [], [[0, 0, 3.1]]),
  { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth', depth: 4 },
]);
export const ballPart = () => part('Ball', 'pla', [{ type: 'sphere', id: 'F1', op: 'add', center: [0, 0, 0], radius: 10 }]);

export function sliderHingeAssembly(): AssemblyDoc {
  return {
    format: 'ck-part', version: 1, name: 'Slider hinge', material: 'steel', seq: 12, rollback: null, counters: {},
    features: [
      { type: 'insertPart', id: 'F1', name: 'Base', part: basePart(), fixed: true },
      { type: 'insertPart', id: 'F2', name: 'Carriage', part: carriagePart(), dx: 50 },
      { type: 'insertPart', id: 'F3', name: 'Arm', part: armPart(), dx: 50, dy: 14, dz: 28 },
      { type: 'insertPart', id: 'F4', name: 'Ball', part: ballPart(), dx: 100, dz: 100 },
      { type: 'mate', id: 'F5', name: 'Carriage on rod', kind: 'concentric', a: 'F4.S.e1@F2.1', b: 'F4.S.e1@F1.1' },
      { type: 'mate', id: 'F6', name: 'Carriage upright', kind: 'parallel', a: 'F2.S.e2@F2.1', b: 'F2.C1@F1.1' },
      { type: 'mate', id: 'F7', name: 'Travel', kind: 'limits', of: 'distance', min: -170, max: -20, a: 'F2.C0@F2.1', b: 'F2.S.e4@F1.1' },
      { type: 'mate', id: 'F8', name: 'Arm pivot', kind: 'hinge', a: 'F4.S.e1@F3.1', b: 'F6.S.e1@F2.1', a2: 'F2.C0@F3.1', b2: 'F2.S.e3@F2.1', distance: 2 },
      { type: 'mate', id: 'F9', name: 'Swing', kind: 'limits', of: 'angle', min: 0, max: 90, a: 'F2.S.e1@F3.1', b: 'F2.S.e4@F2.1' },
    ],
  };
}
