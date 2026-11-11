// 3D sketch probe cases (each: {name, sketch}) — oracle via sketch3_oracle.mjs
const S3 = (over) => ({ type: 'sketch3d', id: 'F1', points: [], entities: [], constraints: [], seq: 3, visible: true, ...over });
const P3 = (id, x, y, z, fixed) => ({ id, x, y, z, ...(fixed ? { fixed: true } : {}) });
const L = (id, p1, p2) => ({ type: 'line', id, p1, p2 });
const A = (id, c, p1, p2) => ({ type: 'arc', id, c, p1, p2 });
const SP = (id, pts, closed) => ({ type: 'spline', id, pts, ...(closed ? { closed: true } : {}) });
const CV = (id, pts, degree, closed) => ({ type: 'cvspline', id, pts, degree, ...(closed ? { closed: true } : {}) });
const K = (type, id, refs, extra) => ({ type, id, refs, ...(extra ?? {}) });
const D3 = (type, id, refs, value, expr) => ({ type, id, refs, value, ...(expr ? { expr } : {}), offset: null });

export const cases = [];
const add = (name, sk) => cases.push({ name, sketch: sk });

add('alongX', S3({
  points: [P3('p1', 0, 0, 0, true), P3('p2', 10, 1, 2), P3('p3', 20, 0, 0)],
  entities: [L('e1', 'p1', 'p2'), L('e2', 'p2', 'p3')],
  constraints: [K('alongX', 'c1', ['e1']), D3('distance', 'c2', ['p1', 'p3'], 20, '25')],
}));
add('alongY', S3({
  points: [P3('p1', 0, 0, 0, true), P3('p2', 10, 1, 2), P3('p3', 20, 0, 0)],
  entities: [L('e1', 'p1', 'p2'), L('e2', 'p2', 'p3')],
  constraints: [K('alongY', 'c1', ['e1']), D3('distance', 'c2', ['p1', 'p3'], 20, '25')],
}));
add('alongZ', S3({
  points: [P3('p1', 0, 0, 0, true), P3('p2', 10, 1, 2)],
  entities: [L('e1', 'p1', 'p2')],
  constraints: [K('alongZ', 'c1', ['e1']), D3('distance', 'c2', ['p1', 'p2'], 20, '25')],
}));
add('parallel3', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('b1', 2, 1, 1), P3('b2', 9, 4, 3), P3('q', 5, 5, 5)],
  entities: [L('e1', 'a1', 'a2'), L('e2', 'b1', 'b2'), CV('s1', ['b2', 'q'], 1)],
  constraints: [K('parallel', 'c1', ['e1', 'e2']), D3('distance', 'c2', ['a2', 'q'], 20, '25')],
}));
add('perpendicular3', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('b1', 2, 1, 1), P3('b2', 9, 4, 3), P3('q', 5, 5, 5)],
  entities: [L('e1', 'a1', 'a2'), L('e2', 'b1', 'b2'), CV('s1', ['b2', 'q'], 1)],
  constraints: [K('perpendicular', 'c1', ['e1', 'e2']), D3('distance', 'c2', ['a2', 'q'], 20, '25')],
}));
add('equal-lines', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('b1', 2, 1, 1), P3('b2', 9, 4, 3)],
  entities: [L('e1', 'a1', 'a2'), L('e2', 'b1', 'b2')],
  constraints: [D3('distance', 'c0', ['a1', 'b1'], 3, '3.3'), K('equal', 'c1', ['e1', 'e2'])],
}));
add('equal-arcs', S3({
  points: [P3('c1', 0, 0, 0, true), P3('a1', 4, 0, 0), P3('a2', 0, 4, 0), P3('c2', 10, 0, 0, true), P3('b1', 12, 3, 0), P3('b2', 8, 5, 0)],
  entities: [A('e1', 'c1', 'a1', 'a2'), A('e2', 'c2', 'b1', 'b2')],
  constraints: [K('equal', 'c1', ['e1', 'e2'])],
}));
add('collinear3', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('q', 5, 5, 5), P3('r', 6, 1, 2)],
  entities: [L('e1', 'a1', 'a2'), CV('s1', ['q', 'r'], 1)],
  constraints: [K('collinear', 'c1', ['s1', 'e1']), D3('distance', 'c2', ['a2', 'r'], 20, '25')],
}));
add('pointOnLine', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('q', 5, 5, 5)],
  entities: [L('e1', 'a1', 'a2')],
  constraints: [K('pointOnLine', 'c1', ['q', 'e1']), D3('distance', 'c2', ['a2', 'q'], 20, '25')],
}));
add('midpoint3', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 2, 4), P3('q', 5, 5, 5)],
  entities: [L('e1', 'a1', 'a2')],
  constraints: [K('midpoint', 'c1', ['q', 'e1']), D3('distance', 'c2', ['a1', 'q'], 20, '25')],
}));
add('corner', S3({
  points: [P3('v', 0, 0, 0, true), P3('a', 10, 0, 0), P3('b', 1, 9, 2), P3('c1', 3, 4, 5), P3('c2', 7, 5, 1), P3('q', 2, 2, 2)],
  entities: [L('e1', 'v', 'a'), L('e2', 'v', 'b'), L('e3', 'c1', 'c2'), CV('s1', ['c2', 'q'], 1)],
  constraints: [K('corner', 'c1', ['q', 'e1', 'e2']), D3('distance', 'c2', ['c1', 'q'], 20, '25')],
}));
add('fix3', S3({
  points: [P3('a', 1, 2, 3), P3('b', 8, 4, 2)],
  entities: [L('e1', 'a', 'b')],
  constraints: [K('fix', 'c1', ['a'], { value: [2, 3, 4] }), D3('distance', 'c2', ['a', 'b'], 20, '25')],
}));
add('onPlane', S3({
  points: [P3('a', 1, 2, 3), P3('b', 8, 4, 2), P3('c', 3, 3, 3)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['b', 'c'], 1)],
  constraints: [K('onPlane', 'c1', ['c'], { plane: { o: [0, 0, 5], n: [0, 0, 1] } }), D3('distance', 'c2', ['a', 'c'], 20, '25')],
}));
add('onPlane-std', S3({
  points: [P3('a', 1, 2, 3), P3('b', 8, 4, 2), P3('c', 3, 3, 3)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['b', 'c'], 1)],
  constraints: [K('onPlane', 'c1', ['c'], { plane: 'front' }), D3('distance', 'c2', ['a', 'c'], 20, '25')],
}));
add('onPlane-line', S3({
  points: [P3('a', 1, 2, 3), P3('b', 8, 4, 2), P3('c', 3, 3, 3)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['b', 'c'], 1)],
  constraints: [K('onPlane', 'c1', ['e1'], { plane: 'top' }), D3('distance', 'c2', ['a', 'c'], 20, '25')],
}));
add('tangent3-line-arc', S3({
  points: [P3('O', 0, 0, 0, true), P3('m', 4, 0, 0), P3('n', 0, 4, 0), P3('a', 4, 1, 0), P3('b', 6, 2, 0)],
  entities: [A('e1', 'O', 'm', 'n'), L('e2', 'm', 'a'), L('e3', 'a', 'b'), CV('s1', ['b', 'a'], 1)],
  constraints: [K('tangent', 'c1', ['e3', 'e1']), D3('distance', 'c2', ['a', 'b'], 20, '25')],
}));
add('tangent3-line-spline', S3({
  points: [P3('a', 1, 1, 1), P3('b', 5, 2, 0), P3('q1', 0, 0, 0, true), P3('q2', 2, 3, 0)],
  entities: [L('e2', 'a', 'b'), SP('s1', ['a', 'q2'])],
  constraints: [K('tangent', 'c1', ['e2', 's1']), D3('distance', 'c2', ['a', 'b'], 20, '25')],
}));
add('g1-3d-line', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 2, 0, 1), P3('b', 4, 2, 0), P3('c', 7, 1, 3), P3('d', 9, 4, 2)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k1', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
add('g1-3d-line-end', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 2, 0, 1), P3('b', 4, 2, 0), P3('c', 7, 1, 3), P3('d', 9, 4, 2)],
  entities: [L('e1', 'd', 'O'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k1', ['s1', 'e1']), D3('distance', 'k2', ['O', 'a'], 0, '8.7')],
}));
add('g2-3d-line', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 2, 0, 1), P3('b', 4, 2, 0), P3('c', 7, 1, 3), P3('d', 9, 4, 2)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g2', 'k1', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
add('g3-3d-line', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 2, 0, 1), P3('b', 4, 2, 0), P3('c', 7, 1, 3), P3('d', 9, 4, 2)],
  entities: [L('e1', 'O', 'a'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g3', 'k1', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k2', ['O', 'd'], 0, '11.3')],
}));
add('g1-3d-arc', S3({
  points: [P3('O', 0, 0, 0, true), P3('p1', 4, 0, 0), P3('p2', 0, 4, 0), P3('a', 3, 1, 2), P3('b', 5, 3, 0), P3('c', 7, 2, 3), P3('d', 9, 5, 1)],
  entities: [A('e1', 'O', 'p1', 'p2'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('coincident', 'k1', ['a', 'p1']), K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k3', ['O', 'd'], 0, '9.9')],
}));
add('g1-3d-spline', S3({
  points: [P3('O', 0, 0, 0, true), P3('q1', 0, 4, 1), P3('q2', 3, 6, 0), P3('q3', 6, 5, 2), P3('a', 6, 5, 2), P3('b', 8, 8, 1), P3('c', 11, 7, 3), P3('d', 13, 10, 0)],
  entities: [SP('e1', ['q1', 'q2', 'q3']), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('coincident', 'k1', ['a', 'q3']), K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k3', ['O', 'd'], 0, '15.4')],
}));
add('g1-3d-cvspline', S3({
  points: [P3('O', 0, 0, 0, true), P3('r1', 0, 5, 1), P3('r2', 2, 7, 0), P3('a', 4, 5, 2), P3('b', 6, 8, 1), P3('c', 9, 7, 3), P3('d', 11, 10, 0)],
  entities: [CV('e1', ['O', 'r1', 'r2', 'a'], 3), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k2', ['s1', 'e1'], { at: 'a' }), D3('distance', 'k3', ['O', 'd'], 0, '14.2')],
}));
add('onCurve3-line', S3({
  points: [P3('O', 0, 0, 0, true), P3('q', 1, 6, 2), P3('a', 4, 1, 0), P3('b', 9, 2, 1)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['O', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.5 }), D3('distance', 'k2', ['O', 'a'], 0, '4.4')],
}));
add('onCurve3-arc', S3({
  points: [P3('O', 0, 0, 0, true), P3('p1', 4, 0, 0), P3('p2', 0, 4, 0), P3('q', 1, 5, 1)],
  entities: [A('e1', 'O', 'p1', 'p2'), CV('s1', ['p2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.5 }), D3('distance', 'k2', ['O', 'q'], 0, '7.7')],
}));
add('onCurve3-spline', S3({
  points: [P3('O', 0, 0, 0, true), P3('q1', 2, 3, 1), P3('q2', 5, 4, 0), P3('q3', 8, 2, 2), P3('q', 3, 8, 1)],
  entities: [SP('e1', ['q1', 'q2', 'q3']), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D3('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('onCurve3-cvspline', S3({
  points: [P3('O', 0, 0, 0, true), P3('q1', 2, 3, 1), P3('q2', 5, 4, 0), P3('q3', 8, 2, 2), P3('q', 3, 8, 1)],
  entities: [CV('e1', ['q1', 'q2', 'q3'], 2), CV('s1', ['q2', 'q'], 1)],
  constraints: [K('onCurve', 'k1', ['q', 'e1'], { value: 0.6 }), D3('hdist', 'k2', ['O', 'q'], 0, '4.9')],
}));
add('tension3', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 1, 1, 1), P3('b', 3, 2, 0), P3('c', 6, 2, 2), P3('d', 8, 5, 1)],
  entities: [CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('tension', 'k1', ['s1'], { at: 'a', value: 2 }), D3('distance', 'k2', ['O', 'd'], 0, '9.1')],
}));
add('tension3-g2', S3({
  points: [P3('O', 0, 0, 0, true), P3('e1a', 4, 0, 0), P3('a', 2, 0, 1), P3('b', 4, 2, 0), P3('c', 7, 1, 3), P3('d', 9, 4, 2)],
  entities: [L('e1', 'e1a', 'O'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('coincident', 'kx', ['a', 'e1a']), K('g2', 'k1', ['s1', 'e1'], { at: 'a' }), K('tension', 'k2', ['s1'], { at: 'a', value: 2 }), D3('distance', 'k3', ['e1a', 'd'], 0, '11.3')],
}));
add('radius3', S3({
  points: [P3('O', 0, 0, 0, true), P3('m', 4, 0, 0), P3('n', 0, 4, 0), P3('q', 2, 2, 5)],
  entities: [A('e1', 'O', 'm', 'n'), CV('s1', ['n', 'q'], 1)],
  constraints: [D3('radius', 'k1', ['e1'], 4, '4.5'), D3('distance', 'k2', ['O', 'q'], 0, '6.1')],
}));
add('angle3', S3({
  points: [P3('a1', 0, 0, 0, true), P3('a2', 10, 0, 0), P3('b1', 0, 0, 0, true), P3('b2', 3, 4, 5)],
  entities: [L('e1', 'a1', 'a2'), L('e2', 'b1', 'b2')],
  constraints: [D3('angle', 'k1', ['e1', 'e2'], 60, '60')],
}));
add('coincident3', S3({
  points: [P3('a', 1, 2, 3), P3('b', 8, 4, 2), P3('q', 2, 2, 2)],
  entities: [L('e1', 'a', 'b'), CV('s1', ['b', 'q'], 1)],
  constraints: [K('coincident', 'c1', ['q', 'a']), D3('distance', 'c2', ['a', 'b'], 20, '25')],
}));
// error paths: unsolved
add('g1-3d-no-shared', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 1, 1, 1), P3('b', 3, 2, 0), P3('c', 6, 2, 2), P3('d', 8, 5, 1), P3('e', 2, 7, 0), P3('f', 4, 9, 1)],
  entities: [L('e1', 'e', 'f'), CV('s1', ['a', 'b', 'c', 'd'], 3)],
  constraints: [K('g1', 'k1', ['s1', 'e1']), D3('distance', 'k2', ['O', 'd'], 0, '9.1')],
}));
add('g1-3d-closed', S3({
  points: [P3('O', 0, 0, 0, true), P3('a', 1, 1, 1), P3('b', 3, 2, 0), P3('c', 6, 2, 2), P3('d', 8, 5, 1)],
  entities: [CV('s1', ['a', 'b', 'c', 'd'], 3, true), L('e1', 'd', 'O')],
  constraints: [K('g1', 'k1', ['s1', 'e1']), D3('distance', 'k2', ['O', 'a'], 0, '4.4')],
}));
