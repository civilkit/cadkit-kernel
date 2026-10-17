// projectCurve corpus, part 2 (imported by project_curve_cases.mjs): chaining, gaps, holes, clipped closed
// curves, more entity types, and a curved target with a single hit sheet (degree / tolerance behaviour).
export function more({ PCID = 'F20', base_parts, add, mk, bodies, shapes, source, pcf, STD, pt, faceOf }) {
  const L = (P, a, b, extra = {}) => ({ type: 'line', id: `e${a}${b}`, p1: P[a], p2: P[b], ...extra });
  const one = (pts, ent) => ({ points: pts, entities: ent });
  const b = bodies.box;
  const w2 = (d) => mk([...b.features, ...source('spline', b.c, b.z), pcf({ faces: [b.face], direction: 'reverse', degree: d })]);
  const top = (shape, c = b.c, z = b.z, o = {}) => mk([...b.features, ...source(shape, c, z), pcf({ faces: [b.face], direction: 'reverse', ...o })]);

  // gaps between consecutive lines of one sketch (the chain joins or splits depending on the gap)
  for (const g of [0, 1e-5, 1e-4, 1e-3, 1e-2, 0.1, 5]) {
    add(`box-gap-${g}`, top(one([[-3, 0], [0, 0], [g, 0], [g + 3, 0]], (P) => [L(P, 0, 1), L(P, 2, 3)])));
  }
  add('box-chain-out-of-order', top(one([[0, 0], [3, 0], [-3, 0], [0, 0]], (P) => [L(P, 0, 1), L(P, 2, 3)])));
  add('box-chain-reversed-entity', top(one([[-3, 0], [0, 0], [3, 0]], (P) => [L(P, 0, 1), L(P, 2, 1)])));
  add('box-two-separate-curves', top(one([[-3, 0], [3, 0], [-3, 4], [3, 4]], (P) => [L(P, 0, 1), L(P, 2, 3)])));
  add('box-t-junction', top(one([[-3, 0], [3, 0], [0, 0], [0, 3]], (P) => [L(P, 0, 1), L(P, 2, 3)])));

  // a hole through the top face: the curve crosses / ends in / circles the hole
  const hole = [{ type: 'sketch', id: 'F3', plane: STD('front'), points: [pt('hc', 20, 10)], entities: [{ type: 'circle', id: 'e1', c: 'hc', r: 3 }], constraints: [] },
    { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth' }];
  const withHole = (shape, c = b.c) => mk([...b.features, ...hole, ...source(shape, c, b.z), pcf({ faces: [b.face], direction: 'reverse' })]);
  add('box-hole-line-across', withHole(one([[-8, 0], [8, 0]], (P) => [L(P, 0, 1)])));
  add('box-hole-line-into', withHole(one([[-8, 0], [0, 0]], (P) => [L(P, 0, 1)])));
  add('box-hole-circle-around', withHole(one([[0, 0]], (P) => [{ type: 'circle', id: 'e1', c: P[0], r: 6 }])));
  add('box-hole-circle-inside', withHole(one([[0, 0]], (P) => [{ type: 'circle', id: 'e1', c: P[0], r: 1 }])));

  // closed curve clipped by the face boundary: the kept run does / does not wrap around the parameter seam of the circle
  const circ = one([[0, 0]], (P) => [{ type: 'circle', id: 'e1', c: P[0], r: 3 }]);
  add('box-circle-clipped-wrap', top(circ, [-1.5, 10]));
  add('box-circle-clipped-nowrap', top(circ, [41.5, 10]));

  // more entity types
  add('box-arc3', top(one([[-3, 0], [3, 0], [0, 2]], (P) => [{ type: 'arc3', id: 'e1', p1: P[0], p2: P[1], p3: P[2] }])));
  add('box-ellipse', top(one([[0, 0], [4, 0]], (P) => [{ type: 'ellipse', id: 'e1', c: P[0], p: P[1], r: 2 }])));
  add('box-cvspline', top(one([[-4, 0], [-2, 3], [2, -3], [4, 0]], (P) => [{ type: 'cvspline', id: 'e1', pts: P, degree: 3 }])));
  add('box-point-only', top(one([[0, 0]], (P) => [{ type: 'point', id: 'e1', p: P[0] }])));
  add('box-line-plus-construction', top(one([[-3, 0], [3, 0], [0, -4], [0, 4]], (P) => [L(P, 0, 1), L(P, 2, 3, { construction: true })])));
  add('box-zero-length-line', top(one([[0, 0], [0, 0]], (P) => [L(P, 0, 1)])));
  // other source planes
  add('box-source-right-plane-side-face', mk([...b.features, ...source(one([[-3, 2], [3, 6]], (P) => [L(P, 0, 1)]), [0, 0], 0, { plane: STD('right') }), pcf({ faces: ['F2.S.e2'], direction: 'both' })]));
  add('box-source-top-plane', mk([...b.features, ...source(one([[-3, -2], [3, 2]], (P) => [L(P, 0, 1)]), [20, 10], 0, { plane: STD('top') }), pcf({ faces: [b.face], direction: 'both' })]));

  // surface bodies (no solid): the faces of a surface feature as targets, empty face list, no body at all but a surface
  {
    const sbase = (id) => base_parts()[id];
    for (const [nm, part, fc] of [['planar', 'surfPlanar', 'F2.P'], ['extrude-e1', 'surfExtrude', 'F2.S.e1'], ['extrude-e2', 'surfExtrude', 'F2.S.e2'], ['planar-solid', 'surfPlanarSolid', 'F4.P']]) {
      const fs = sbase(part).features;
      for (const faces of [[fc], []])
        add(`surf-${nm}-${faces.length ? 'named' : 'all'}`, mk([...fs, ...source(one([[5, 5], [15, 9]], (P) => [L(P, 0, 1)]), [0, 0], 5, { base: 'top' }), pcf({ faces, direction: 'reverse' })]));
    }
  }
  add('err-empty-nobody', mk([{ type: 'projectCurve', id: PCID }]));
  add('err-curve-missing-nobody', mk([{ type: 'projectCurve', id: PCID, curve: 'F99' }]));
  add('err-badsketch-nobody-empty', mk([{ type: 'projectCurve', id: PCID, sketch: 'F99' }]));
  add('degree-2.6', w2(2.6)); add('degree-2.4', w2(2.4)); add('degree-numeric-string', w2('4')); add('degree-expr', w2('=2+2'));
  // orientation of a planar curve input: its plane normal follows the traversal (counter-clockwise = +z?)
  {
    const ccw = one([[-3, -2], [3, -2], [3, 2], [-3, 2]], (P) => [0, 1, 2, 3].map((i) => L(P, i, (i + 1) % 4)));
    const cw = one([[-3, -2], [-3, 2], [3, 2], [3, -2]], (P) => [0, 1, 2, 3].map((i) => L(P, i, (i + 1) % 4)));
    const open1 = one([[-3, -2], [0, 2], [3, -2]], (P) => [L(P, 0, 1), L(P, 1, 2)]);
    const open2 = one([[3, -2], [0, 2], [-3, -2]], (P) => [L(P, 0, 1), L(P, 1, 2)]);
    const spl = one([[-3, -2], [-1, 2], [1, -2], [3, 2]], (P) => [{ type: 'spline', id: 'e1', pts: P }]);
    for (const [nm, shape] of [['ccw', ccw], ['cw', cw], ['open1', open1], ['open2', open2], ['spline', spl]])
      for (const d of ['forward', 'reverse', 'both'])
        add(`curve-orient-${nm}-${d}`, mk([...b.features, ...source(shape, b.c, b.z), pcf({ faces: [b.face], direction: 'reverse' }),
          { type: 'projectCurve', id: 'F21', curve: 'F20', faces: ['F2.C0'], direction: d }]));
  }
  // consumers of the projected curve (`{kind:'curve', id}` refs)
  {
    const first = pcf({ id: 'F20', faces: [b.face], direction: 'reverse' });
    const pre = [...b.features, ...source('spline', b.c, b.z), first];
    const cref = { kind: 'curve', id: 'F20' };
    add('consumer-editCurve', mk([...pre, { type: 'editCurve', id: 'F21', curves: [cref], method: 'count', degree: 3, count: 8, tolerance: 0.01, keepEnds: true, keepTangents: false, keepCorners: false, entries: [], planar: false, plane: null }]));
    const prof = { type: 'sketch', id: 'F30', plane: STD('right'), points: [pt('c', 0, 0)], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 1 }], constraints: [] };
    add('consumer-sweep-path', mk([...pre, prof, { type: 'sweep', id: 'F31', op: 'add', sketch: 'F30', path: cref }]));
    add('consumer-plane-curve', mk([...pre, { type: 'plane', id: 'F32', mode: 'curve', curves: [cref] }]));
    add('consumer-sweep-guide', mk([...b.features, ...source('line', b.c, b.z), first,
      { type: 'sketch', id: 'F30', plane: STD('right'), points: [pt('c', 0, 5)], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 1 }], constraints: [] },
      { type: 'sketch', id: 'F33', plane: STD('front'), points: [pt('a', 0, 0), pt('b', 0, 30)], entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b' }], constraints: [] },
      { type: 'sweep', id: 'F31', op: 'add', sketch: 'F30', path: { kind: 'sketch', id: 'F33' }, guides: [cref] }]));
  }
  // seeded random cases: shape, target, plane distance, offset of the shape, direction, degree, tolerance
  {
    let seed = 20261017;
    const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    const pick = (a) => a[Math.floor(rnd() * a.length)];
    const shapeNames = Object.keys(shapes);
    for (let i = 0; i < 40; i++) {
      const bn = pick(['box', 'cyl', 'cone', 'sphere', 'torus', 'spline']);
      const t = bodies[bn];
      const sn = pick(shapeNames);
      const c = [t.c[0] + (rnd() - 0.5) * 8, t.c[1] + (rnd() - 0.5) * 8];
      const z = pick([20, 5, 0, -4]);
      const dir = pick(['both', 'forward', 'reverse']);
      const o = { faces: faceOf(t) ?? [], direction: dir, degree: pick([2, 3, 3, 4, 6]), tolerance: pick([0.001, 0.001, 0.01, 0.0001]) };
      add(`fuzz-${String(i).padStart(2, '0')}-${bn}-${sn}-z${z}-${dir}`, mk([...t.features, ...source(sn, c, z), pcf(o)]));
    }
  }
  // where a fold (silhouette) sits decides whether the two sheets are one piece: interior of the entity or its end,
  // on the seam side (+x, u = 0) or the other side (-x) of the revolved cone
  {
    const k = bodies.cone;
    const ln = (a, bb) => one([a, bb], (P) => [L(P, 0, 1)]);
    const run = (name, shape) => add(name, mk([...k.features, ...source(shape, [0, 0], 20), pcf({ faces: [k.face], direction: 'reverse' })]));
    run('fold-cone-end-posx', ln([2, 3], [8, 7]));
    run('fold-cone-end-negx', ln([-2, 3], [-8, 7]));
    run('fold-cone-mid-posx', ln([2, 5], [12, 7]));
    run('fold-cone-mid-negx', ln([-2, 5], [-12, 7]));
    run('fold-cone-mid-posz', ln([0, 3], [0, 3.0001]));
    // the same folds with the sketch plane through the body (z = 0) and the direction both / forward / reverse
    for (const d of ['both', 'forward', 'reverse'])
      add(`fold-cone-mid-posx-z0-${d}`, mk([...k.features, ...source(ln([2, 5], [12, 7]), [0, 0], 0), pcf({ faces: [k.face], direction: d })]));
    const circ = (c, r) => one([c], (P) => [{ type: 'circle', id: 'e1', c: P[0], r }]);
    for (const [nm, z, d] of [['z20-reverse', 20, 'reverse'], ['z0-both', 0, 'both'], ['z0-forward', 0, 'forward']])
      add(`fold-cone-circle-${nm}`, mk([...k.features, ...source(circ([7.12, 8.09], 3), [0, 0], z), pcf({ faces: [k.face], direction: d })]));
    for (const [nm, z, d] of [['z20-reverse', 20, 'reverse'], ['z0-both', 0, 'both']])
      add(`fold-cone-circle-low-${nm}`, mk([...k.features, ...source(circ([7.12, 5], 2), [0, 0], z), pcf({ faces: [k.face], direction: d })]));
    const t = bodies.torus;
    for (const [nm, a, bb] of [['posx', [14, -2], [24, 2]], ['negx', [-14, -2], [-24, 2]]])
      add(`fold-torus-${nm}`, mk([...t.features, ...source(ln(a, bb), [0, 0], 20), pcf({ faces: [t.face], direction: 'reverse' })]));
    const sp = bodies.sphere;
    for (const [nm, a, bb] of [['posx', [2, 0], [14, 0]], ['negx', [-2, 0], [-14, 0]], ['posx-y', [2, 3], [14, 3]], ['negx-y', [-2, 3], [-14, 3]]])
      add(`fold-sphere-${nm}`, mk([...sp.features, ...source(ln(a, bb), [0, 0], 20), pcf({ faces: [sp.face], direction: 'reverse' })]));
    const cy = bodies.cyl; // axis x = 5, z = -3; silhouettes at x = -5 and 15
    for (const [nm, a, bb] of [['posx', [10, 3], [20, 5]], ['negx', [0, 3], [-10, 5]]])
      add(`fold-cyl-${nm}`, mk([...cy.features, ...source(ln(a, bb), [0, 0], 20), pcf({ faces: [cy.face], direction: 'reverse' })]));
  }
  // order of the checks: bad option values against misses / unknown references / missing input
  const missSrc = source('line', [100, 10], b.z);
  const okSrc = source('line', b.c, b.z);
  const w = (src, o, feats = b.features) => mk([...feats, ...src, pcf({ faces: [b.face], direction: 'reverse', ...o })]);
  add('order-degree1-miss', w(missSrc, { degree: 1 }));
  add('order-tol0-miss', w(missSrc, { tolerance: 0 }));
  add('order-dir-miss', w(missSrc, { direction: 'sideways' }));
  add('order-dir-degree1', w(okSrc, { direction: 'sideways', degree: 1 }));
  add('order-dir-tol0', w(okSrc, { direction: 'sideways', tolerance: 0 }));
  add('order-degree1-tol0', w(okSrc, { degree: 1, tolerance: 0 }));
  add('order-degree1-unknown-face', mk([...b.features, ...okSrc, pcf({ faces: ['F77.C1'], degree: 1 })]));
  add('order-degree1-no-body', mk([...source('line', [0, 0], 5), pcf({ faces: ['F2.C1'], degree: 1 })]));
  add('order-degree1-missing-sketch', mk([...b.features, pcf({ sketch: 'F99', faces: [b.face], degree: 1 })]));
  add('order-degree1-no-input', mk([...b.features, { type: 'projectCurve', id: PCID, faces: [b.face], degree: 1 }]));
  add('order-tol0-no-input', mk([...b.features, { type: 'projectCurve', id: PCID, faces: [b.face], tolerance: 0 }]));
  add('order-unknown-face-miss', mk([...b.features, ...missSrc, pcf({ faces: ['F77.C1'] })]));
  add('order-nobody-badsketch', mk([...source('line', [0, 0], 5), pcf({ sketch: 'F99', faces: ['F2.C1'] })]));
  add('order-tol-string', w(okSrc, { tolerance: 'abc' }));
  add('order-tol-expr', w(okSrc, { tolerance: '=0.002' }));
  add('order-degree-null', w(okSrc, { degree: null }));
  add('order-tol-null', w(okSrc, { tolerance: null }));
  add('order-degree-nan', w(okSrc, { degree: '=0/0' }));
  // curve + sketch together; chain of curves with the sketch id wrong
  const first = pcf({ id: 'F20', faces: [b.face], direction: 'reverse' });
  add('order-curve-valid-sketch-missing', mk([...b.features, ...okSrc, first, { type: 'projectCurve', id: 'F21', sketch: 'F99', curve: 'F20', faces: ['F2.C0'], direction: 'reverse' }]));
  add('order-curve-valid-sketch-valid', mk([...b.features, ...okSrc, first, { type: 'projectCurve', id: 'F21', sketch: 'F10', curve: 'F20', faces: ['F2.C0'], direction: 'reverse' }]));
  add('order-curve-null-sketch-valid', mk([...b.features, ...okSrc, { type: 'projectCurve', id: 'F21', sketch: 'F10', curve: null, faces: [b.face], direction: 'reverse' }]));
  // gap versus tolerance
  for (const [g, t] of [[0.01, 0.05], [0.0009, 0.001], [0.0011, 0.002], [0.04, 0.05], [0.06, 0.05], [0.0005, 0.0001]])
    add(`gap-${g}-tol-${t}`, top(one([[-3, 0], [0, 0], [g, 0], [g + 3, 0]], (P) => [L(P, 0, 1), L(P, 2, 3)]), b.c, b.z, { tolerance: t }));

  // sphere, sketch plane through the centre, direction forward (one hit sheet): all shapes, degree, tolerance, extent
  const sp = bodies.sphere;
  const f = (shape, o = {}, c = [0, 0]) => mk([...sp.features, ...source(shape, c, 0, { plane: STD('front') }), pcf({ faces: [sp.face], direction: 'forward', ...o })]);
  const sh = (n) => shapes[n];
  for (const sn of Object.keys(shapes)) add(`sphere-fwd-${sn}`, f(sn));
  for (const d of [2, 3, 5, 9]) add(`sphere-fwd-spline-degree-${d}`, f('spline', { degree: d }));
  for (const t of [1e-9, 1e-7, 1e-6, 0.01, 1, 20]) add(`sphere-fwd-spline-tol-${t}`, f('spline', { tolerance: t }));
  for (const d of [2, 5]) add(`sphere-fwd-polyline-degree-${d}`, f('polyline', { degree: d }));
  for (const t of [1e-6, 1]) add(`sphere-fwd-circle-tol-${t}`, f('circle', { tolerance: t }));
  for (const r of [9, 10, 12]) add(`sphere-fwd-circle-r${r}`, f(one([[0, 0]], (P) => [{ type: 'circle', id: 'e1', c: P[0], r }])));
  void sh;

  // the other curved targets with direction forward and the sketch plane through the middle
  for (const bn of ['cyl', 'cone', 'torus', 'spline']) {
    const t = bodies[bn];
    for (const sn of ['line', 'spline', 'circle', 'polyline'])
      add(`${bn}-mid-fwd-${sn}`, mk([...t.features, ...source(sn, t.c, 0), pcf({ faces: faceOf(t), direction: 'forward' })]));
  }
}

