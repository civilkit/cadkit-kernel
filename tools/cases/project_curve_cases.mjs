// projectCurve corpus: parts built on the single-feature bodies of brep_cases.mjs (plane, cylinder, cone,
// sphere, torus, B-spline faces) plus a source sketch on an offset 'front' plane (normal +z, sketch (x,y) ->
// world (x,y,z0)) and one projectCurve feature. Consumed by tools/project_curve_corpus.mjs, which runs every
// case through the oracle kernel and ours.
//
// Export: `cases` = [{ name, part, note? }].
import { parts as base } from '../brep_cases.mjs';

const pt = (id, x, y) => ({ id, x, y });
const STD = (id) => ({ kind: 'std', id });
const PC = 'F20';

// ---- target bodies: features + the face to project onto + where the body sits in the xy plane of 'front'
const AX = { points: [pt('a1', 0, 0), pt('a2', 0, 10)], ent: { type: 'line', id: 'ax', p1: 'a1', p2: 'a2', construction: true } };
const coneFeatures = [
  { type: 'sketch', id: 'F1', plane: STD('front'),
    points: [pt('p1', 0, 0), pt('p2', 15, 0), pt('p3', 5, 10), pt('p4', 0, 10), ...AX.points],
    entities: [['e1', 'p1', 'p2'], ['e2', 'p2', 'p3'], ['e3', 'p3', 'p4'], ['e4', 'p4', 'p1']]
      .map(([id, p1, p2]) => ({ type: 'line', id, p1, p2 })).concat([AX.ent]), constraints: [] },
  { type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360 },
];
export const bodies = {
  box: { features: base.box.features.map((f) => ({ ...f, plane: f.plane?.id === 'top' ? STD('front') : f.plane })), face: 'F2.C1', c: [20, 10], z: 20 },
  cyl: { features: base.cyl.features, face: 'F2.S.e7', c: [0, 5], z: 20 },
  cone: { features: coneFeatures, face: 'F2.S.e2', c: [5, 5], z: 20 },
  sphere: { features: base.sphere.features, face: 'F2.S.e1', c: [0, 0], z: 20 },
  torus: { features: base.torus.features, face: 'F2.S.e1', c: [20, 0], z: 20 },
  spline: { features: base.loftCircSq.features, face: null, c: [0, 15], z: 20 },
};

// ---- source shapes, local coordinates (about 3 mm around the origin)
export const shapes = {
  line: { points: [[-3, -2], [3, 2]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] },
  polyline: { points: [[-3, -2], [0, 2], [3, -2], [3, 2]],
    entities: (P) => [0, 1, 2].map((i) => ({ type: 'line', id: `e${i + 1}`, p1: P[i], p2: P[i + 1] })) },
  rect: { points: [[-3, -2], [3, -2], [3, 2], [-3, 2]],
    entities: (P) => [0, 1, 2, 3].map((i) => ({ type: 'line', id: `e${i + 1}`, p1: P[i], p2: P[(i + 1) % 4] })) },
  arc: { points: [[0, 0], [3, 0], [0, 3]], entities: (P) => [{ type: 'arc', id: 'e1', c: P[0], p1: P[1], p2: P[2] }] },
  circle: { points: [[0, 0]], r: 3, entities: (P) => [{ type: 'circle', id: 'e1', c: P[0], r: 3 }] },
  spline: { points: [[-3, -2], [-1, 2], [1, -2], [3, 2]], entities: (P) => [{ type: 'spline', id: 'e1', pts: P }] },
};

export function source(shape, c, z, { id = 'F10', planeId = 'F9', plane, base: pbase = 'front' } = {}) {
  const sh = typeof shape === 'string' ? shapes[shape] : shape;
  const P = sh.points.map(([x, y], i) => pt(`s${i + 1}`, x + c[0], y + c[1]));
  const ids = P.map((p) => p.id);
  return [
    ...(plane ? [] : [{ type: 'plane', id: planeId, mode: 'offset', base: STD(pbase), offset: z }]),
    { type: 'sketch', id, plane: plane ?? { kind: 'feature', id: planeId }, points: P, entities: sh.entities(ids), constraints: [] },
  ];
}

const mk = (features) => ({ format: 'ck-part', version: 1, features });
export const cases = [];
const add = (name, part, note) => cases.push({ name, part, note });
const pcf = (o) => ({ type: 'projectCurve', id: PC, sketch: 'F10', degree: 3, tolerance: 0.001, ...o });

// 1. every shape on every target, direction reverse (the sketch sits at z = 20, in front of the body)
const faceOf = (b) => (b.face ? [b.face] : undefined);
for (const [bn, b] of Object.entries(bodies)) {
  for (const sn of Object.keys(shapes)) {
    const feats = [...b.features, ...source(sn, b.c, b.z), pcf({ faces: faceOf(b), direction: 'reverse' })];
    add(`${bn}-${sn}`, mk(feats));
  }
}
// the loft body has four spline faces: each of them alone
for (const i of [1, 2, 3, 4]) {
  const b = bodies.spline;
  add(`spline-line-F4.S.e1~${i}`, mk([...b.features, ...source('line', b.c, b.z), pcf({ faces: [`F4.S.e1~${i}`], direction: 'reverse' })]));
  add(`spline-circle-F4.S.e1~${i}`, mk([...b.features, ...source('circle', b.c, b.z), pcf({ faces: [`F4.S.e1~${i}`], direction: 'reverse' })]));
}

// 2. direction on the box top (z = 10) with the sketch in front (z = 20) / behind (z = -5) / inside the body (z = 5)
{
  const b = bodies.box;
  for (const [z, label] of [[20, 'front'], [-5, 'behind']])
    for (const d of ['both', 'forward', 'reverse', undefined])
      add(`box-dir-${d ?? 'default'}-${label}`, mk([...b.features, ...source('spline', b.c, z), pcf({ faces: [b.face], direction: d })]));
  // sketch plane through the top face itself
  add('box-sketch-on-face-plane', mk([...b.features, ...source('line', b.c, 10), pcf({ faces: [b.face] })]));
}

// 3. degree / tolerance on a spline source projected on the cylinder
{
  const b = bodies.cyl;
  const f = (o) => mk([...b.features, ...source('spline', b.c, b.z), pcf({ faces: [b.face], direction: 'reverse', ...o })]);
  for (const d of [0, 1, 2, 3, 5, 9, 10]) add(`cyl-spline-degree-${d}`, f({ degree: d }));
  for (const t of [1e-6, 1e-4, 0.01, 0.5, 5, 0, -1]) add(`cyl-spline-tol-${t}`, f({ tolerance: t }));
  add('cyl-spline-degree-2.6', f({ degree: 2.6 }));
  add('cyl-spline-no-degree', mk([...b.features, ...source('spline', b.c, b.z), { type: 'projectCurve', id: PC, sketch: 'F10', faces: [b.face], direction: 'reverse' }]));
}

// 4. misses and several faces
{
  const b = bodies.box;
  add('box-miss-x', mk([...b.features, ...source('line', [100, 10], 20), pcf({ faces: [b.face], direction: 'reverse' })]));
  add('box-miss-y', mk([...b.features, ...source('circle', [20, 60], 20), pcf({ faces: [b.face], direction: 'reverse' })]));
  add('box-miss-wrong-face', mk([...b.features, ...source('line', b.c, 20), pcf({ faces: ['F2.S.e1'], direction: 'reverse' })]));
  add('box-faces-all', mk([...b.features, ...source('line', b.c, 20), pcf({ faces: [], direction: 'reverse' })]));
  add('box-faces-all-both', mk([...b.features, ...source('line', b.c, 20), pcf({ faces: [], direction: 'both' })]));
  add('box-faces-omitted', mk([...b.features, ...source('line', b.c, 20), pcf({ direction: 'reverse' })]));
  add('box-faces-top-bottom', mk([...b.features, ...source('line', b.c, 20), pcf({ faces: ['F2.C0', 'F2.C1'], direction: 'reverse' })]));
  add('box-faces-top-bottom-both', mk([...b.features, ...source('line', b.c, 20), pcf({ faces: ['F2.C0', 'F2.C1'], direction: 'both' })]));
  // trimmed boundary: the line runs past the top face edge (x from 30 to 50, the face ends at 40)
  add('box-trimmed-edge-partial', mk([...b.features, ...source({ points: [[30, 10], [50, 10]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] }, [0, 0], 20), pcf({ faces: [b.face], direction: 'reverse' })]));
  add('box-trimmed-edge-circle', mk([...b.features, ...source('circle', [38, 10], 20), pcf({ faces: [b.face], direction: 'reverse' })]));
  // two faces adjacent: top plus the side face x=40 (viewed along z the side face is edge-on)
  add('box-seam-between-faces', mk([...b.features, ...source({ points: [[30, 10], [50, 10]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] }, [0, 0], 20), pcf({ faces: [b.face, 'F2.S.e2'], direction: 'reverse' })]));
}

// 5. cylinder and sphere seams: a line along the revolve seam direction / across it
{
  // 'right' plane sketch (normal +x, local (u,v) -> world (0, v, -u)): projection along x hits the cylinder at z = +-sqrt(r^2 - ..)
  const cylB = bodies.cyl; // axis y, r 10, y 0..10, seam at +x
  const right = { kind: 'std', id: 'right' };
  const cross = { points: [[-4, 2], [4, 8]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] };
  add('cyl-seam-right-fwd', mk([...cylB.features, ...source(cross, [0, 0], 0, { plane: right }), pcf({ faces: [cylB.face], direction: 'forward' })]));
  add('cyl-seam-right-rev', mk([...cylB.features, ...source(cross, [0, 0], 0, { plane: right }), pcf({ faces: [cylB.face], direction: 'reverse' })]));
  add('cyl-seam-right-both', mk([...cylB.features, ...source(cross, [0, 0], 0, { plane: right }), pcf({ faces: [cylB.face], direction: 'both' })]));
  // sphere with the sketch on 'right' crossing the revolve seam
  const sph = bodies.sphere;
  add('sphere-seam-right-fwd', mk([...sph.features, ...source({ points: [[-4, -3], [4, 3]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] }, [0, 0], 0, { plane: right }), pcf({ faces: [sph.face], direction: 'forward' })]));
  add('sphere-seam-right-rev', mk([...sph.features, ...source({ points: [[-4, -3], [4, 3]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] }, [0, 0], 0, { plane: right }), pcf({ faces: [sph.face], direction: 'reverse' })]));
  // sphere through both hemispheres of the projection (both directions, one face): front plane through the centre
  add('sphere-through-both', mk([...sph.features, ...source('arc', [0, 0], 0, { plane: STD('front') }), pcf({ faces: [sph.face], direction: 'both' })]));
  add('sphere-through-fwd', mk([...sph.features, ...source('arc', [0, 0], 0, { plane: STD('front') }), pcf({ faces: [sph.face], direction: 'forward' })]));
  // torus: line over the tube top and over the hole
  const tor = bodies.torus;
  add('torus-over-hole', mk([...tor.features, ...source({ points: [[-8, 0], [8, 0]], entities: (P) => [{ type: 'line', id: 'e1', p1: P[0], p2: P[1] }] }, [0, 0], 20), pcf({ faces: [tor.face], direction: 'reverse' })]));
}

// 6. errors
{
  const b = bodies.box;
  const S = source('line', b.c, b.z);
  add('err-empty-payload', mk([...b.features, { type: 'projectCurve', id: PC }]));
  add('err-empty-with-faces', mk([...b.features, { type: 'projectCurve', id: PC, faces: [b.face] }]));
  add('err-missing-sketch', mk([...b.features, ...S, pcf({ sketch: 'F99', faces: [b.face] })]));
  add('err-sketch-is-extrude', mk([...b.features, ...S, pcf({ sketch: 'F2', faces: [b.face] })]));
  add('err-sketch-is-plane', mk([...b.features, ...S, pcf({ sketch: 'F9', faces: [b.face] })]));
  add('err-missing-curve', mk([...b.features, ...S, { type: 'projectCurve', id: PC, curve: 'F99', faces: [b.face] }]));
  add('err-curve-is-sketch', mk([...b.features, ...S, { type: 'projectCurve', id: PC, curve: 'F10', faces: [b.face] }]));
  add('err-curve-is-extrude', mk([...b.features, ...S, { type: 'projectCurve', id: PC, curve: 'F2', faces: [b.face] }]));
  add('err-unknown-face', mk([...b.features, ...S, pcf({ faces: ['F77.C1'] })]));
  add('err-no-body-sketch', mk([...source('line', [0, 0], 5), pcf({ faces: ['F2.C1'] })]));
  add('err-no-body-nofaces', mk([...source('line', [0, 0], 5), pcf({})]));
  add('err-empty-sketch', mk([...b.features, { type: 'sketch', id: 'F10', plane: STD('front'), points: [], entities: [], constraints: [] }, pcf({ faces: [b.face] })]));
  add('err-construction-only-sketch', mk([...b.features, { type: 'sketch', id: 'F10', plane: STD('front'), points: [pt('a', 0, 0), pt('b', 5, 5)], entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b', construction: true }], constraints: [] }, pcf({ faces: [b.face], direction: 'reverse' })]));
  add('err-faces-not-array', mk([...b.features, ...S, pcf({ faces: b.face, direction: 'reverse' })]));
  add('err-bad-direction', mk([...b.features, ...S, pcf({ faces: [b.face], direction: 'sideways' })]));
  add('err-degree-string', mk([...b.features, ...S, pcf({ faces: [b.face], direction: 'reverse', degree: 'abc' })]));
  add('err-both-sketch-and-curve', mk([...b.features, ...S, pcf({ faces: [b.face], direction: 'reverse', curve: 'F99' })]));
}

// 7. curve inputs (curve feature ids): an earlier projectCurve (planar, on the box top), an editCurve of a sketch, a helix (not planar)
{
  const b = bodies.box;
  const S = source('spline', b.c, b.z);
  const first = pcf({ id: 'F20', faces: [b.face], direction: 'reverse' });
  // chain: the projected curve lies in z = 10, projects further down onto the bottom face
  add('curve-chain-pc-to-bottom', mk([...b.features, ...S, first, { type: 'projectCurve', id: 'F21', curve: 'F20', faces: ['F2.C0'], direction: 'reverse' }]));
  add('curve-chain-pc-to-top-again', mk([...b.features, ...S, first, { type: 'projectCurve', id: 'F21', curve: 'F20', faces: [b.face], direction: 'both' }]));
  const edit = { type: 'editCurve', id: 'F20', curves: [{ kind: 'sketchCurve', sketch: 'F10', entity: 'e1' }], method: 'count', degree: 3, count: 6 };
  add('curve-editCurve-planar', mk([...b.features, ...S, edit, { type: 'projectCurve', id: 'F21', curve: 'F20', faces: [b.face], direction: 'reverse' }]));
  add('curve-editCurve-line', mk([...b.features, ...source('line', b.c, b.z), edit, { type: 'projectCurve', id: 'F21', curve: 'F20', faces: [b.face], direction: 'reverse' }]));
  // non-planar curve: a helix over a circle sketch
  const hx = source('circle', [0, 0], 0, { id: 'F10', plane: STD('front') });
  add('curve-helix-not-planar', mk([...hx, { type: 'helix', id: 'F11', sketch: 'F10', definedBy: 'pitchRevs', pitch: 5, revolutions: 3 },
    ...bodies.box.features.map((f) => ({ ...f, id: f.id === 'F1' ? 'F31' : f.id === 'F2' ? 'F32' : f.id, sketch: f.sketch === 'F1' ? 'F31' : f.sketch })), { type: 'projectCurve', id: 'F40', curve: 'F11', faces: ['F32.C1'], direction: 'both' }]));
}

import { more } from './project_curve_cases2.mjs';
more({ base_parts: () => base, add, mk, bodies, shapes, source, pcf, STD, pt, faceOf });
