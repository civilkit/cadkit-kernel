// each: {name, sketch} — a sketch feature object (2D or 3D)
const S = (over) => ({ type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: [], entities: [], constraints: [], seq: 3, visible: true, ...over });
const S3 = (over) => ({ type: 'sketch3d', id: 'F1', points: [], entities: [], constraints: [], seq: 3, visible: true, ...over });
const P = (id, x, y, fixed) => ({ id, x, y, ...(fixed ? { fixed: true } : {}) });
const P3 = (id, x, y, z, fixed) => ({ id, x, y, z, ...(fixed ? { fixed: true } : {}) });
const L = (id, p1, p2) => ({ type: 'line', id, p1, p2 });
const C = (id, c, r) => ({ type: 'circle', id, c, r });
const A = (id, c, p1, p2) => ({ type: 'arc', id, c, p1, p2 });
const SP = (id, pts, closed) => ({ type: 'spline', id, pts, ...(closed ? { closed: true } : {}) });
const CV = (id, pts, degree, closed) => ({ type: 'cvspline', id, pts, degree, ...(closed ? { closed: true } : {}) });
const D2H = (type, id, refs, value, expr) => ({ type, id, refs, value, expr, offset: null });
const K = (type, id, refs, extra) => ({ type, id, refs, ...extra });
const D = (type, id, refs, value, expr) => ({ type, id, refs, value, ...(expr ? { expr } : {}), offset: null });

export const cases = [];
const add = (name, sk) => cases.push({ name, sketch: sk });

// --- g1/g2/g3: cvspline end tangent to a line
add('g1-line-start', S({
  points: [P('O', 0, 0, true), P('a', 2, 0), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('horizontal', 'k0', ['e1']), K('g1', 'k1', ['s1', 'e1'], { at: 'a' }), D('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
add('g1-line-end', S({
  points: [P('O', 0, 0, true), P('a', 2, 0), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4)],
  entities: [L('e1', 'd', 'O'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('horizontal', 'k0', ['e1']), K('g1', 'k1', ['s1', 'e1']), D('distance', 'k2', ['O', 'a'], 0, '8.7')],
}));
add('g2-line', S({
  points: [P('O', 0, 0, true), P('a', 2, 0), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('horizontal', 'k0', ['e1']), K('g2', 'k1', ['s1', 'e1'], { at: 'a' }), D('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
add('g3-line', S({
  points: [P('O', 0, 0, true), P('a', 2, 0), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('horizontal', 'k0', ['e1']), K('g3', 'k1', ['s1', 'e1'], { at: 'a' }), D('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
// g1 to circle (centre fixed)
add('g1-circle', S({
  points: [P('O', 0, 0, true), P('q', 5, 0), P('a', 4, 2), P('b', 6, 4), P('c', 8, 3), P('d', 10, 5)],
  entities: [C('e1', 'O', 4), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [D('radius', 'k0', ['e1'], 4, '4.2'), K('pointOnCircle', 'k1', ['a', 'e1']), K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D('distance', 'k3', ['O', 'd'], 0, '10.6')],
}));
// g1 to arc
add('g1-arc', S({
  points: [P('O', 0, 0, true), P('p1', 4, 0), P('p2', 0, 4), P('a', 3, 1), P('b', 5, 3), P('c', 7, 2), P('d', 9, 5)],
  entities: [A('e1', 'O', 'p1', 'p2'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('coincident', 'k1', ['a', 'e1']), K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D('distance', 'k3', ['O', 'd'], 0, '9.9')],
}));
// g1 to interp spline
add('g1-spline', S({
  points: [P('O', 0, 0, true), P('q1', 0, 4), P('q2', 3, 6), P('q3', 6, 5), P('a', 6, 5), P('b', 8, 8), P('c', 11, 7), P('d', 13, 10)],
  entities: [SP('e1', ['q1', 'q2', 'q3']), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('coincident', 'k1', ['a', 'q3']), K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D('distance', 'k3', ['O', 'd'], 0, '15.4')],
}));
// g1 cvspline to cvspline
add('g1-cvspline', S({
  points: [P('O', 0, 0, true), P('r1', 0, 5), P('r2', 2, 7), P('a', 4, 5), P('b', 6, 8), P('c', 9, 7), P('d', 11, 10)],
  entities: [CV('e1', ['O', 'r1', 'r2', 'a'], 3), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D('distance', 'k3', ['O', 'd'], 0, '14.2')],
}));
// --- onCurve
add('onCurve-line', S({
  points: [P('O', 0, 0, true), P('q', 1, 6), P('a', 4, 1), P('b', 9, 2)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['O', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.5 }), D('distance', 'k2', ['O', 'a'], 0, '4.4')],
}));
add('onCurve-circle', S({
  points: [P('O', 0, 0, true), P('q', 3, 1), P('a', 10, 0, true)],
  entities: [C('e1', 'O', 4), CV('s1', ['q', 'a'], 1)],
  constraints: [D('radius', 'k0', ['e1'], 4, '4.4'), K('onCurve', 'k1', ['q', 'e1'], { value: 0.25 })],
}));
add('onCurve-arc', S({
  points: [P('O', 0, 0, true), P('p1', 4, 0), P('p2', 0, 4), P('q', 1, 5)],
  entities: [A('e1', 'O', 'p1', 'p2'), CV('s1', ['p2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.5 }), D('distance', 'k2', ['O', 'q'], 0, '7.7')],
}));
add('onCurve-ellipse', S({
  points: [P('O', 0, 0, true), P('m', 5, 0), P('q', 2, 4)],
  entities: [{ type: 'ellipse', id: 'e1', c: 'O', p: 'm', r: 2 }, CV('s1', ['m', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.125 }), D('hdist', 'k2', ['O', 'q'], 0, '2.9')],
}));
add('onCurve-spline', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [SP('e1', ['q1', 'q2', 'q3']), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('onCurve-cvspline', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [CV('e1', ['q1', 'q2', 'q3'], 2), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
// --- tension
add('tension', S({
  points: [P('O', 0, 0, true), P('a', 1, 1), P('b', 3, 2), P('c', 6, 2), P('d', 8, 5)],
  entities: [CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('tension', 'k1', ['s1'], { at: 'a', value: 2 }), D('distance', 'k2', ['O', 'd'], 0, '9.1')],
}));
add('tension-end', S({
  points: [P('O', 0, 0, true), P('a', 1, 1), P('b', 3, 2), P('c', 6, 2), P('d', 8, 5)],
  entities: [CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('tension', 'k1', ['s1'], { at: 'd', value: 2 }), D('distance', 'k2', ['O', 'a'], 0, '4.4')],
}));
add('tension-g2', S({
  points: [P('O', 0, 0, true), P('e1a', 4, 0), P('a', 2, 0), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4)],
  entities: [L('e1', 'e1a', 'O'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('horizontal', 'k0', ['e1']), K('coincident', 'kx', ['a', 'e1a']), K('g2', 'k1', ['s1', 'e1'], { at: 'a' }), K('tension', 'k2', ['s1'], { at: 'a', value: 2 }), D('distance', 'k3', ['e1a', 'd'], 0, '11.3')],
}));
// error paths: g1 without shared end leaves the sketch unsolved
add('g1-no-shared', S({
  points: [P('O', 0, 0, true), P('a', 1, 1), P('b', 3, 2), P('c', 6, 2), P('d', 8, 5), P('e', 2, 7), P('f', 4, 9)],
  entities: [L('e1', 'e', 'f'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k1', ['s1', 'e1']), D('distance', 'k2', ['O', 'd'], 0, '9.1')],
}));
// closed cvspline -> error
add('g1-closed', S({
  points: [P('O', 0, 0, true), P('a', 1, 1), P('b', 3, 2), P('c', 6, 2), P('d', 8, 5)],
  entities: [CV('s1', ['a', 'b', 'c', 'd'], 3, true), L('e1', 'd', 'O')],
  constraints: [K('g1', 'k1', ['s1', 'e1']), D('distance', 'k2', ['O', 'a'], 0, '4.4')],
}));
add('onCurve-cv-end0', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [CV('e1', ['q1', 'q2', 'q3'], 2), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0 }), D2H('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('onCurve-cv-end1', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [CV('e1', ['q1', 'q2', 'q3'], 2), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 1 }), D2H('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
// determined g2: a, d fixed; g2 to the horizontal line O-a; distances pin b, c
add('g2-det', S({
  points: [P('O', 0, 0, true), P('a', 2, 0, true), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4, true)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g2', 'k1', ['s1', 'e1'], { at: 'a' }), D('distance', 'k2', ['a', 'b'], 0, '3.3'), D('distance', 'k3', ['c', 'd'], 0, '4.4')],
}));
add('g3-det', S({
  points: [P('O', 0, 0, true), P('a', 2, 0, true), P('b', 4, 2), P('c', 7, 1), P('d', 9, 4, true)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g3', 'k1', ['s1', 'e1'], { at: 'a' }), D('distance', 'k2', ['a', 'b'], 0, '3.3'), D('distance', 'k3', ['c', 'd'], 0, '4.4'), D('hdist', 'k4', ['b', 'c'], 0, '3.7')],
}));

// closed-curve onCurve probes (tools/sketch2_cases.mjs)
add('onCurve-cvspline-closed', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [{ type: 'cvspline', id: 'e1', pts: ['q1', 'q2', 'q3'], degree: 2, closed: true }, CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D2H('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('onCurve-spline-closed', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [{ type: 'spline', id: 'e1', pts: ['q1', 'q2', 'q3'], closed: true }, CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D2H('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('onCurve-cvspline-knots', S({
  points: [P('O', 0, 0, true), P('q1', 2, 3), P('q2', 5, 4), P('q3', 8, 2), P('q', 3, 8)],
  entities: [{ type: 'cvspline', id: 'e1', pts: ['q1', 'q2', 'q3'], degree: 2, knots: [0, 0, 0, 1, 2, 2, 2] }, CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D2H('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
