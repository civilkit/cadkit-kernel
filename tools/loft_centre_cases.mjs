// Corpus for the loft `centreLine` option (internal notes (not published), crates/ck-ops/src/loft/centre.rs).
// Every case is a self-contained .ckpart document: profiles on offset top planes (the loft runs along +y), the
// centre line drawn on the front plane (sketch x -> world x, sketch y -> world y: the loft axis) unless noted.
// Consumers: tools/loft_centre_check.mjs (oracle vs ours, section table) and, through
// `node tools/loft_centre_cases.mjs --dump DIR`, any other harness that wants the .ckpart documents.
const pt = (id, x, y) => ({ id, x, y });
const STD = (id) => ({ kind: 'std', id });
const poly = (P) => ({
  points: P.map(([x, y], i) => pt('p' + (i + 1), x, y)),
  entities: P.map((_, i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + (i + 1), p2: 'p' + (((i + 1) % P.length) + 1) })),
});
const sq = (h, cx = 0, cy = 0) => poly([[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]);
const circ = (r, cx = 0, cy = 0) => ({ points: [pt('c', cx, cy)], entities: [{ type: 'circle', id: 'e1', c: 'c', r }] });
const openPoly = (P) => ({
  points: P.map(([x, y], i) => pt('p' + (i + 1), x, y)),
  entities: P.slice(1).map((_, i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + (i + 1), p2: 'p' + (i + 2) })),
});
const apexPt = (cx = 0, cy = 0) => ({ points: [pt('p', cx, cy)], entities: [{ type: 'point', id: 'e1', p: 'p' }] });

// centre-line sketch bodies (front plane coordinates (x, z))
export const C = {
  line: (a, b) => ({ points: [pt('g1', ...a), pt('g2', ...b)], entities: [{ type: 'line', id: 'e1', p1: 'g1', p2: 'g2' }] }),
  // circular arc through three points (a, m, b): stored as centre + CCW start / end
  arc3: (a, m, b) => {
    const [ax, ay] = a, [bx, by] = m, [cx, cy] = b;
    const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
    const cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax); // > 0: a -> m -> b is counter-clockwise
    const [s, e] = cross > 0 ? [a, b] : [b, a];
    // spine(t): point and unit tangent at the arc fraction t from a to b (for the perpendicular-section probes)
    const a0 = Math.atan2(ay - uy, ax - ux), a1 = Math.atan2(cy - uy, cx - ux), R = Math.hypot(ax - ux, ay - uy);
    const sw = cross > 0 ? ((a1 - a0 + 4 * Math.PI) % (2 * Math.PI)) : -(((a0 - a1) + 4 * Math.PI) % (2 * Math.PI));
    const spine = (t) => { const th = a0 + sw * t, sg = Math.sign(sw); return [[ux + R * Math.cos(th), uy + R * Math.sin(th)], [-sg * Math.sin(th), sg * Math.cos(th)]]; };
    return { points: [pt('g0', ux, uy), pt('g1', ...s), pt('g2', ...e)], entities: [{ type: 'arc', id: 'e1', c: 'g0', p1: 'g1', p2: 'g2' }], spine };
  },
  spline: (P) => ({ points: P.map((p, i) => pt('s' + i, ...p)), entities: [{ type: 'spline', id: 'e1', pts: P.map((_, i) => 's' + i) }] }),
  twoLines: (a, b, c) => ({ points: [pt('g1', ...a), pt('g2', ...b), pt('g3', ...c)], entities: [{ type: 'line', id: 'e1', p1: 'g1', p2: 'g2' }, { type: 'line', id: 'e2', p1: 'g2', p2: 'g3' }] }),
};

// profiles: [z, shape, planeOverride?]; centre: {body, plane?}; extra: loft fields
export function build({ profiles, centre = null, extra = {}, guides = [], pre = [] }) {
  const feats = [...pre]; let n = 0; const sks = [];
  for (const [z, shape, plane] of profiles) {
    let pl = STD('top');
    if (plane) { feats.push(...plane.feats); pl = plane.ref; }
    else if (z !== 0) { const id = `F${++n}`; feats.push({ type: 'plane', id, mode: 'offset', base: STD('top'), offset: z }); pl = { kind: 'feature', id }; }
    const sid = `F${++n}`; feats.push({ type: 'sketch', id: sid, plane: pl, ...shape, constraints: [] }); sks.push(sid);
  }
  const lf = { type: 'loft', id: `F${++n}`, op: 'add', sketches: sks, ...extra };
  if (centre) {
    const cid = `F${++n}`;
    const { spine: _s, ...body } = centre.body;
    feats.push({ type: 'sketch', id: cid, plane: centre.plane ?? STD('front'), ...body, constraints: [] });
    lf.centreLine = { sketch: cid };
  }
  if (guides.length) {
    lf.guides = guides.map((g) => { const gid = `F${++n}`; feats.push({ type: 'sketch', id: gid, plane: STD('front'), ...g, constraints: [] }); return { sketch: gid }; });
  }
  feats.push(lf);
  return { format: 'ck-part', version: 1, features: feats };
}

export const cases = {};
const add = (name, spec, meta = {}) => { cases[name] = { part: build(spec), meta: { spine: spec.centre?.body.spine, ...meta } }; };
const S2 = [[0, sq(10)], [30, sq(5)]];
const S2c = [[0, sq(10)], [30, sq(10)]]; // prism-like
const S3 = [[0, sq(10)], [30, sq(8)], [60, sq(5)]];
const ctr = (body, plane) => ({ body, plane });
const arcPts = (bx, z0 = 0, z1 = 30) => [[0, z0], [bx, (z0 + z1) / 2], [0, z1]];

// ---- two profiles, centred, straight / curved centre lines
add('plain2', { profiles: S2 });
add('straight2', { profiles: S2, centre: ctr(C.line([0, 0], [0, 30])) });
add('bow2_s2', { profiles: S2, centre: ctr(C.arc3(...arcPts(2))) });
add('bow2_s5', { profiles: S2, centre: ctr(C.arc3(...arcPts(5))) });
add('bow2_s10', { profiles: S2, centre: ctr(C.arc3(...arcPts(10))) });
add('bow2_neg5', { profiles: S2, centre: ctr(C.arc3(...arcPts(-5))) });
add('bow2_s15', { profiles: S2, centre: ctr(C.arc3(...arcPts(15))) });
add('bowSpl2_s5', { profiles: S2, centre: ctr(C.spline([[0, 0], [5, 15], [0, 30]])) });
add('sCurve2', { profiles: S2, centre: ctr(C.spline([[0, 0], [4, 10], [-4, 20], [0, 30]])) });
add('bow2_prism_s5', { profiles: S2c, centre: ctr(C.arc3(...arcPts(5))) });
add('bow2_circ', { profiles: [[0, circ(8)], [30, circ(8)]], centre: ctr(C.arc3(...arcPts(5))) });
add('bow2_circSq', { profiles: [[0, circ(8)], [30, sq(5)]], centre: ctr(C.arc3(...arcPts(5))) });
// ---- centre line ends off the profile centres (second profile offset)
add('offsetStraight', { profiles: [[0, sq(10)], [30, sq(5, 8, 0)]], centre: ctr(C.line([0, 0], [8, 30])) });
add('offsetPlain', { profiles: [[0, sq(10)], [30, sq(5, 8, 0)]] });
add('offsetCurved', { profiles: [[0, sq(10)], [30, sq(5, 8, 0)]], centre: ctr(C.spline([[0, 0], [6, 15], [8, 30]])) });
add('offsetArc', { profiles: [[0, sq(10)], [30, sq(5, 8, 0)]], centre: ctr(C.arc3([0, 0], [5, 15], [8, 30])) });
// profiles centred but the centre line is parallel-shifted / arrives elsewhere
add('shiftStraight', { profiles: S2, centre: ctr(C.line([3, 0], [3, 30])) });
add('shiftArc', { profiles: S2, centre: ctr(C.arc3([3, 0], [8, 15], [3, 30])) });
add('endAway', { profiles: S2, centre: ctr(C.arc3([0, 0], [6, 15], [10, 30])) });
add('slantStraight', { profiles: S2, centre: ctr(C.line([0, 0], [8, 30])) });
// ---- length of the centre line against the profile spacing
add('shortLine', { profiles: S2, centre: ctr(C.line([0, 0], [0, 15])) });
add('shortArc', { profiles: S2, centre: ctr(C.arc3([0, 0], [3, 8], [0, 15])) });
add('longLine', { profiles: S2, centre: ctr(C.line([0, -10], [0, 40])) });
add('longArc', { profiles: S2, centre: ctr(C.arc3([0, -10], [5, 15], [0, 40])) });
add('startsAt5', { profiles: S2, centre: ctr(C.arc3([0, 5], [4, 17], [0, 30])) });
// ---- errors
add('reachNear', { profiles: S2, centre: ctr(C.arc3([0, 0], [3, 12.5], [0, 25])) }); // 5 short of the last plane: within its circumradius
add('reachFar', { profiles: S2, centre: ctr(C.arc3([0, 0], [2, 7.5], [0, 15])) });
add('tightWide', { profiles: [[0, sq(20)], [4, sq(20)]], centre: ctr(C.arc3([0, -2], [5, 2], [0, 6])) });
add('kink', { profiles: S2, centre: ctr(C.twoLines([0, 0], [6, 15], [0, 30])) });
add('tight', { profiles: S2, centre: ctr(C.arc3([0, 0], [20, 15], [0, 30])) });
add('wiggle', { profiles: S2, centre: ctr(C.spline([[0, 0], [8, 4], [-8, 10], [8, 18], [-8, 26], [0, 30]])) });
// ---- three profiles
add('plain3', { profiles: S3 });
add('straight3', { profiles: S3, centre: ctr(C.line([0, 0], [0, 60])) });
add('bow3_s5', { profiles: S3, centre: ctr(C.arc3(...arcPts(5, 0, 60))) });
add('bow3_s10', { profiles: S3, centre: ctr(C.arc3(...arcPts(10, 0, 60))) });
add('bow3_s20', { profiles: S3, centre: ctr(C.arc3(...arcPts(20, 0, 60))) });
add('bow3_prism', { profiles: [[0, sq(8)], [30, sq(8)], [60, sq(8)]], centre: ctr(C.arc3(...arcPts(10, 0, 60))) });
add('asym3_plain', { profiles: [[0, sq(10)], [20, sq(8)], [60, sq(5)]] });
add('asym3', { profiles: [[0, sq(10)], [20, sq(8)], [60, sq(5)]], centre: ctr(C.arc3(...arcPts(8, 0, 60))) });
add('sCurve3', { profiles: S3, centre: ctr(C.spline([[0, 0], [6, 20], [-6, 40], [0, 60]])) });
add('mid3_offset', { profiles: [[0, sq(8)], [30, sq(8, 6, 0)], [60, sq(8)]], centre: ctr(C.arc3(...arcPts(6, 0, 60))) });
add('mid3_offsetPlain', { profiles: [[0, sq(8)], [30, sq(8, 6, 0)], [60, sq(8)]] });
add('mid3_offsetStraight', { profiles: [[0, sq(8)], [30, sq(8, 6, 0)], [60, sq(8)]], centre: ctr(C.line([0, 0], [0, 60])) });
// ---- 4 profiles (bottle)
add('bottle4', { profiles: [[0, circ(10)], [20, circ(14)], [40, circ(6)], [60, circ(8)]], centre: ctr(C.arc3(...arcPts(8, 0, 60))) });
add('bottle4Plain', { profiles: [[0, circ(10)], [20, circ(14)], [40, circ(6)], [60, circ(8)]] });
// ---- profiles not perpendicular to the spine: the second profile on a plane tilted 20 degrees about a line along x
// (a sketch line at y = 30 is the axis), so the plane normal leaves the bending plane
const tilt = (z, deg) => ({
  feats: [
    { type: 'plane', id: 'T0', mode: 'offset', base: STD('top'), offset: z },
    { type: 'sketch', id: 'TA', plane: { kind: 'feature', id: 'T0' }, points: [pt('a1', -10, 0), pt('a2', 10, 0)], entities: [{ type: 'line', id: 'e1', p1: 'a1', p2: 'a2', construction: true }], constraints: [] },
    { type: 'plane', id: 'T1', mode: 'angle', base: { kind: 'feature', id: 'T0' }, axis: { kind: 'sketchLine', sketch: 'TA', entity: 'e1' }, angle: deg },
  ],
  ref: { kind: 'feature', id: 'T1' },
});
add('tiltPlain', { profiles: [[0, sq(10)], [30, sq(5), tilt(30, 20)]] });
add('tiltStraight', { profiles: [[0, sq(10)], [30, sq(5), tilt(30, 20)]], centre: ctr(C.line([0, 0], [0, 30])) });
add('tiltBow', { profiles: [[0, sq(10)], [30, sq(5), tilt(30, 20)]], centre: ctr(C.arc3(...arcPts(5))) });
// ---- spine out of the front plane: a planar curve in the right plane, and on a slanted plane (a 3D bow)
add('bowRight2', { profiles: S2, centre: ctr(C.arc3(...arcPts(5)), STD('right')) });
add('bowSlant3d', { profiles: S2, centre: ctr(C.arc3(...arcPts(5)), { kind: 'feature', id: 'TP' }),
  pre: [{ type: 'plane', id: 'TP', mode: 'angle', base: STD('front'), axis: { kind: 'axis', axis: 'z' }, angle: 35 }] });
// ---- surface lofts (open profiles)
const open1 = openPoly([[-10, -5], [0, 5], [10, -5]]), open2 = openPoly([[-6, -3], [0, 3], [6, -3]]);
add('surfPlain', { profiles: [[0, open1], [30, open2]], extra: { surface: true } });
add('surfBow', { profiles: [[0, open1], [30, open2]], extra: { surface: true }, centre: ctr(C.arc3(...arcPts(5))) });
// ---- apex (point) ends
add('apexPlain', { profiles: [[0, sq(10)], [30, apexPt()]] }, { noOracle: 'the oracle has no point (apex) sections: noClosedProfile' });
add('apexStraight', { profiles: [[0, sq(10)], [30, apexPt()]], centre: ctr(C.line([0, 0], [0, 30])) }, { noOracle: 'the oracle has no point (apex) sections: noClosedProfile' });
add('apexBow', { profiles: [[0, sq(10)], [30, apexPt()]], centre: ctr(C.arc3(...arcPts(5))) }, { noOracle: 'the oracle has no point (apex) sections: noClosedProfile' });
add('apexPlain3', { profiles: [[0, sq(10)], [30, sq(8)], [60, apexPt()]] }, { noOracle: 'the oracle has no point (apex) sections: noClosedProfile' });
add('apexBow3', { profiles: [[0, sq(10)], [30, sq(8)], [60, apexPt()]], centre: ctr(C.arc3(...arcPts(5, 0, 60))) }, { noOracle: 'the oracle has no point (apex) sections: noClosedProfile' });
// ---- with guides
add('guidePlain', { profiles: S2, guides: [C.line([10, 0], [5, 30])] });
add('guideBow', { profiles: S2, guides: [C.line([10, 0], [5, 30])], centre: ctr(C.arc3(...arcPts(5))) });
add('guideStraightCl', { profiles: S2, guides: [C.line([10, 0], [5, 30])], centre: ctr(C.line([0, 0], [0, 30])) });
// ---- other loft options alongside
add('bowStartNormal', { profiles: S2, extra: { startNormal: 1 }, centre: ctr(C.arc3(...arcPts(5))) });
add('bowFollow', { profiles: S2, extra: { followNormal: true }, centre: ctr(C.arc3(...arcPts(5))) });
add('bowSections8', { profiles: S2, extra: { sections: 8 }, centre: ctr(C.arc3(...arcPts(5))) });
// ---- closed / empty centre line sketch
add('closedCl', { profiles: S2, centre: ctr({ points: [pt('c', 0, 15)], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 15 }] }) });
add('emptyCl', { profiles: S2, centre: ctr({ points: [], entities: [] }) });

if (process.argv[1]?.endsWith('loft_centre_cases.mjs') && process.argv.includes('--dump')) {
  const { writeFileSync, mkdirSync } = await import('node:fs');
  const dir = process.argv[process.argv.indexOf('--dump') + 1]; mkdirSync(dir, { recursive: true });
  for (const [k, c] of Object.entries(cases)) writeFileSync(`${dir}/${k}.ckpart`, JSON.stringify(c.part));
  console.log(Object.keys(cases).length, 'cases written to', dir);
}
