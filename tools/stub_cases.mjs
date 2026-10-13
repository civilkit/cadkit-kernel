// Parts for the features that used to be ck-ops stubs (docs/ck-ops-stubs.md).
// Consumers: tools/cmp_stubs.mjs (oracle vs rebuilt, per-part summary) and,
// for the parts that are implemented, tools/brep_cases.mjs (the gate).
import { parts as base } from './brep_cases.mjs';

const box = base.box.features; // F1 sketch (top, 40 x 20), F2 extrude depth 10; top face F2.C1
const withBox = (extra, name) => ({ ...base.box, features: [...box, ...extra] });
const E = (a, b, at) => ({ faces: [a, b], at });
export const TOP_E1 = E('F2.C1', 'F2.S.e1', [20, 10, 0]);
export const TOP_E3 = E('F2.C1', 'F2.S.e3', [20, 10, -20]);
export const TOP_E2 = E('F2.C1', 'F2.S.e2', [40, 10, -10]);
export const TOP_E4 = E('F2.C1', 'F2.S.e4', [0, 10, -10]);

export const parts = {};
const add = (name, features) => { parts[name] = withBox(features); };

// ---- pipeCut: ball of `radius` rolled along edges (and the rims of faces); the
// solid becomes a surface body with everything inside the pipe removed.
const pc = (o) => ({ type: 'pipeCut', id: 'F3', radius: 2, edges: [], faces: [], ...o });
add('pipeCutEdge', [pc({ edges: [TOP_E1] })]);
add('pipeCutEdge2', [pc({ edges: [TOP_E1, TOP_E3] })]);
add('pipeCutRing', [pc({ edges: [TOP_E1, TOP_E2, TOP_E3, TOP_E4] })]);
add('pipeCutFace', [pc({ faces: ['F2.C1'] })]);
add('pipeCutBig', [pc({ radius: 8, edges: [TOP_E1] })]);
add('pipeCutHuge', [pc({ radius: 30, edges: [TOP_E1] })]);
add('pipeCutZero', [pc({ radius: 0, edges: [TOP_E1] })]);
add('pipeCutNone', [pc({})]);
add('pipeCutBadEdge', [pc({ edges: [E('F2.C1', 'F9.S.e1', [20, 10, 0])] })]);

// ---- blend variants (fillet / chamfer fields beyond constant radius)
const fl = (o) => ({ type: 'fillet', id: 'F3', radius: 2, edges: [TOP_E1], ...o });
add('filletConst', [fl({})]);
add('filletVariable', [fl({ filletType: 'variable', radii: [1, 3] })]);
add('filletFullRound', [fl({ filletType: 'fullRound', edges: [], side1: 'F2.S.e1', center: 'F2.C1', side2: 'F2.S.e3' })]);
add('filletConic', [fl({ profile: 'conic', rho: 0.5 })]);
add('filletConic3', [fl({ profile: 'conic', rho: 0.3 })]);
add('filletChord', [fl({ profile: 'chord' })]);
add('filletCurv', [fl({ profile: 'curvature', shape: 0.5 })]);
add('filletCurvG3', [fl({ profile: 'curvature', shape: 0.5, g3: true })]);
add('filletCurvBig', [fl({ profile: 'curvature', shape: 0.5, radius: 30 })]);
add('filletVarConic', [fl({ filletType: 'variable', radii: [1, 3], profile: 'conic', rhos: [0.4, 0.6] })]);
add('filletVarChord', [fl({ filletType: 'variable', radii: [1, 3], profile: 'chord' })]);
add('filletChain', [fl({ edges: [TOP_E1, TOP_E2, TOP_E3, TOP_E4] })]);
add('filletTan', [fl({ edges: [TOP_E1], tangent: true })]);
const ch = (o) => ({ type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 2, edges: [TOP_E1], ...o });
add('chamferEq', [ch({})]);
add('chamferAD', [ch({ chamferType: 'angleDistance', angle: 30 })]);
add('chamferDD', [ch({ chamferType: 'distanceDistance', distance2: 3 })]);
add('chamferChord', [ch({ profile: 'chord' })]);
add('chamferDist', [ch({ chamferType: 'distances', distance: 2, distance2: 3 })]);
add('chamferAngle', [ch({ chamferType: 'angle', distance: 2, angle: 30 })]);

// ---- simplify (internal stage of every boolean): coplanar / cocylindrical faces
// and collinear edges left by an edit are merged.
const pt = (id, x, y) => ({ id, x, y });
const circSk = (id, plane, cx, cy, r) => ({ type: 'sketch', id, plane, points: [pt('c', cx, cy)], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
const ex = (id, sk, o = {}) => ({ type: 'extrude', id, op: 'add', sketch: sk, end: 'blind', depth: 10, ...o });
const STDP = (id) => ({ kind: 'std', id });
const rectSk = (id, plane, w, h) => ({ type: 'sketch', id, plane, points: [pt('p1', 0, 0), pt('p2', w, 0), pt('p3', w, h), pt('p4', 0, h)],
  entities: [['e1', 'p1', 'p2'], ['e2', 'p2', 'p3'], ['e3', 'p3', 'p4'], ['e4', 'p4', 'p1']].map(([i, a, b]) => ({ type: 'line', id: i, p1: a, p2: b })), constraints: [] });
parts.simpCylStack = { ...base.box, features: [circSk('F1', STDP('top'), 5, 3, 10), ex('F2', 'F1'),
  circSk('F3', { kind: 'face', id: 'F2.C1' }, 5, 3, 10), ex('F4', 'F3')] };
parts.simpCylStackCut = { ...base.box, features: [circSk('F1', STDP('top'), 5, 3, 10), ex('F2', 'F1', { depth: 20 }),
  circSk('F3', { kind: 'face', id: 'F2.C1' }, 5, 3, 10), ex('F4', 'F3', { op: 'cut', depth: 10 })] };
parts.simpBoxStack = { ...base.box, features: [...box, rectSk('F3', { kind: 'face', id: 'F2.C1' }, 40, 20), ex('F4', 'F3')] };
parts.simpBoxStackHalf = { ...base.box, features: [...box, rectSk('F3', { kind: 'face', id: 'F2.C1' }, 40, 10), ex('F4', 'F3')] };

// ---- loft: guides, centre line, start/end constraints, connectors
const loftBase = base.loftSq2.features.slice(0, 3); // F1 (z=0 square r10), F2 plane z=30, F3 (square r5)
const lineSk = (id, plane, a, b) => ({ type: 'sketch', id, plane, points: [pt('g1', a[0], a[1]), pt('g2', b[0], b[1])], entities: [{ type: 'line', id: 'e1', p1: 'g1', p2: 'g2' }], constraints: [] });
const loftF = (id, o = {}) => ({ type: 'loft', id, op: 'add', sketches: ['F1', 'F3'], ...o });
const FRONT = STDP('front');
parts.loftGuideLine = { ...base.box, features: [...loftBase, lineSk('F4', FRONT, [10, 0], [5, 30]), loftF('F5', { guides: [{ sketch: 'F4' }] })] };
parts.loftGuideMiss = { ...base.box, features: [...loftBase, lineSk('F4', FRONT, [12, 0], [7, 30]), loftF('F5', { guides: [{ sketch: 'F4' }] })] };
parts.loftGuide2 = { ...base.box, features: [...loftBase, lineSk('F4', FRONT, [10, 0], [5, 30]), lineSk('F6', FRONT, [-10, 0], [-5, 30]), loftF('F5', { guides: [{ sketch: 'F4' }, { sketch: 'F6' }] })] };
parts.loftGuideBulge = { ...base.box, features: [...loftBase,
  { type: 'sketch', id: 'F4', plane: FRONT, points: [pt('g1', 10, 0), pt('g2', 5, 30), pt('g3', 14, 15)], entities: [{ type: 'arc3', id: 'e1', p1: 'g1', p2: 'g2', p3: 'g3' }], constraints: [] },
  loftF('F5', { guides: [{ sketch: 'F4' }] })] };
parts.loftCentre = { ...base.box, features: [...loftBase, lineSk('F4', FRONT, [0, 0], [0, 30]), loftF('F5', { centreLine: { sketch: 'F4' } })] };
parts.loftCentreSlant = { ...base.box, features: [...loftBase, lineSk('F4', FRONT, [0, 0], [8, 30]), loftF('F5', { centreLine: { sketch: 'F4' } })] };
parts.loftStartNormal = { ...base.box, features: [...loftBase, loftF('F5', { startNormal: true })] };
parts.loftEndTangent = { ...base.box, features: [...loftBase, loftF('F5', { endNormal: true })] };

// ---- editCurve: rebuild a curve with n control points, move control points
const splSk = (id) => ({ type: 'sketch', id, plane: STDP('front'), points: [pt('c1', 0, 0), pt('c2', 5, 8), pt('c3', 10, 2), pt('c4', 15, 9), pt('c5', 20, 0)], entities: [{ type: 'spline', id: 'e1', pts: ['c1', 'c2', 'c3', 'c4', 'c5'] }], constraints: [] });
const cvRef = (e = 'e1', sk = 'F1') => ({ kind: 'sketchCurve', sketch: sk, entity: e });
const ec = (o = {}) => ({ type: 'editCurve', id: 'F2', curves: [cvRef()], method: 'count', degree: 3, count: 6, tolerance: 0.01, keepEnds: true, keepTangents: false, keepCorners: false, entries: [], planar: false, plane: null, ...o });
const mkp = (features) => ({ ...base.box, features });
parts.ecSpline = mkp([splSk('F1'), ec()]);
parts.ecSplineCount8 = mkp([splSk('F1'), ec({ count: 8, degree: 5 })]);
parts.ecSplineTol = mkp([splSk('F1'), ec({ method: 'tolerance', tolerance: 0.05 })]);
parts.ecMove = mkp([splSk('F1'), ec({ entries: [{ cv: 2, of: 6, frame: 'world', dx: 0, dy: 3, dz: 0 }] })]);
parts.ecMoveOut = mkp([splSk('F1'), ec({ entries: [{ cv: 9, of: 6, frame: 'world', dx: 0, dy: 3, dz: 0 }] })]);
parts.ecNoInput = mkp([splSk('F1'), ec({ curves: [] })]);
parts.ecMissing = mkp([splSk('F1'), ec({ curves: [cvRef('e9')] })]);
parts.ecDegreeBad = mkp([splSk('F1'), ec({ degree: 1 })]);
parts.ecDegreeBig = mkp([splSk('F1'), ec({ degree: 12, count: 14 })]);
parts.ecPlanarNoPlane = mkp([splSk('F1'), ec({ planar: true })]);
parts.ecEdge = mkp([...box, ec({ curves: [{ kind: 'edge', faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }], id: 'F3' })]);

// ---- sweep guides (guide.rs): the profile follows guide curves while it travels
const swPath = lineSk('F2', FRONT, [0, 0], [0, 20]);
const swCirc = { type: 'sketch', id: 'F3', plane: STDP('top'), points: [pt('c', 0, 0)], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 3 }], constraints: [] };
const swFeat = (o = {}) => ({ type: 'sweep', id: 'F5', op: 'add', sketch: 'F3', path: { kind: 'sketch', id: 'F2' }, ...o });
parts.sweepPlain = mkp([swPath, swCirc, swFeat()]);
parts.sweepGuideScale = mkp([swPath, swCirc, lineSk('F4', FRONT, [3, 0], [6, 20]), swFeat({ guides: [cvRef('e1', 'F4')], guideMode: 'scale' })]);
parts.sweepGuideScale2 = mkp([swPath, swCirc, lineSk('F4', FRONT, [3, 0], [6, 20]), swFeat({ guides: [cvRef('e1', 'F4')], guideStrength: 1, guideMode: 'scale' })]);
parts.sweepGuideTurn = mkp([swPath, swCirc, lineSk('F4', FRONT, [3, 0], [6, 20]), swFeat({ guides: [cvRef('e1', 'F4')], guideMode: 'turn' })]);
parts.sweepGuideStretch1 = mkp([swPath, swCirc, lineSk('F4', FRONT, [3, 0], [6, 20]), swFeat({ guides: [cvRef('e1', 'F4')], guideMode: 'stretch' })]);
parts.sweepGuideStretch2 = mkp([swPath, swCirc, lineSk('F4', FRONT, [3, 0], [6, 20]), lineSk('F6', FRONT, [-3, 0], [-6, 20]), swFeat({ guides: [cvRef('e1', 'F4'), cvRef('e1', 'F6')], guideMode: 'stretch' })]);

// ---- freeform: rebuild a face as a B-spline net and move control points
const ff = (o = {}) => ({ type: 'freeform', id: 'F3', faces: ['F2.C1'], degreeU: 3, degreeV: 3, spansU: 2, spansV: 2, useTolerance: false, tolerance: 0.01, extend: 0, shrink: true, direction: null, edges: [], edits: [], ...o });
const edit = (points, o = {}) => ({ points, of: [5, 5], frame: 'world', dx: 0, dy: 0, dz: 0, ...o });
add('ffPlain', [ff()]);
add('ffEdit', [ff({ edits: [edit([[2, 2]], { dy: 2 })] })]);
add('ffEditTwo', [ff({ edits: [edit([[2, 2]], { dy: 2 }), edit([[1, 1], [3, 3]], { dy: -1 })] })]);
add('ffEditProp', [ff({ edits: [edit([[2, 2]], { dy: 2, proportional: 2 })] })]);
add('ffEditOutside', [ff({ edits: [edit([[9, 9]], { dy: 2 })] })]);
add('ffEditInvalid', [ff({ edits: [{ points: [], of: [5, 5], frame: 'world', dx: 0, dy: 1, dz: 0 }] })]);
add('ffEditSym', [ff({ edits: [edit([[1, 2]], { dy: 2, symmetric: { kind: 'std', id: 'right' } })] })]);
add('ffEditBig', [ff({ edits: [edit([[2, 2]], { dy: -30 })] })]);

// ---- blends on a sheet (blend/sheet.rs): fillet / chamfer the interior edge of an L-shaped surface
const lSheet = [{ type: 'sketch', id: 'F1', plane: STDP('front'), points: [pt('a', 0, 0), pt('b', 20, 0), pt('c', 20, 20)], entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b' }, { type: 'line', id: 'e2', p1: 'b', p2: 'c' }], constraints: [] },
  { type: 'surfaceExtrude', id: 'F2', sketch: 'F1', end: 'blind', depth: 10, flip: false }];
parts.sheetFillet = mkp([...lSheet, { type: 'fillet', id: 'F3', radius: 3, edges: [{ faces: ['F2.S.e1', 'F2.S.e2'], at: [20, 0, 5] }] }]);
parts.sheetChamfer = mkp([...lSheet, { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 3, edges: [{ faces: ['F2.S.e1', 'F2.S.e2'], at: [20, 0, 5] }] }]);
parts.sheetFilletRim = mkp([...lSheet, { type: 'fillet', id: 'F3', radius: 3, edges: [{ faces: ['F2.S.e1', 'F2.C0'], at: [10, 0, 0] }] }]);
parts.sheetBase = mkp(lSheet);

// ---- pipeCut on sheets / mixed bodies / curved edges / no body
parts.pipeCutSheet = mkp([...lSheet, pc({ id: 'F3', edges: [{ faces: ['F2.S.e1', 'F2.S.e2'], at: [20, 0, 5] }] })]);
parts.pipeCutSheetFace = mkp([...lSheet, pc({ id: 'F3', faces: ['F2.S.e1'] })]);
parts.pipeCutNoBody = { ...base.box, features: [pc({ id: 'F1', edges: [TOP_E1] })] };
parts.pipeCutTwoBodies = mkp([...lSheet.map((f) => (f.id === 'F1' ? { ...f, id: 'F7' } : f.id === 'F2' ? { ...f, id: 'F8', sketch: 'F7' } : f)), pc({ id: 'F9', edges: [TOP_E1, { faces: ['F8.S.e1', 'F8.S.e2'], at: [20, 0, 5] }] })]);
parts.pipeCutCyl = { ...base.box, features: [circSk('F1', STDP('top'), 0, 0, 10), ex('F2', 'F1'), pc({ id: 'F3', radius: 2, edges: [{ faces: ['F2.C1', 'F2.S.e1'], at: [10, 10, 0] }] })] };
parts.pipeCutCylTight = { ...base.box, features: [circSk('F1', STDP('top'), 0, 0, 10), ex('F2', 'F1'), pc({ id: 'F3', radius: 11, edges: [{ faces: ['F2.C1', 'F2.S.e1'], at: [10, 10, 0] }] })] };
parts.pipeCutCylBot = { ...base.box, features: [circSk('F1', STDP('top'), 0, 0, 10), ex('F2', 'F1'), pc({ id: 'F3', radius: 3, edges: [{ faces: ['F2.C0', 'F2.S.e1'], at: [10, 0, 0] }] })] };
parts.pipeCut12 = { ...base.box, features: [...box, pc({ id: 'F3', radius: 12, edges: [TOP_E1] })] };

// more simplify probes: stacked cones, a hole refilled by a plug
const frontSk = (id, pts, ents) => ({ type: 'sketch', id, plane: STDP('front'), points: pts, entities: ents, constraints: [] });
const ln = (id, a, b, o = {}) => ({ type: 'line', id, p1: a, p2: b, ...o });
const revF = (id, sk) => ({ type: 'revolve', id, op: 'add', sketch: sk, axis: 'ax', angle: 360 });
const coneSk = (id, r0, y0, r1, y1) => frontSk(id, [pt('a', 0, y0), pt('b', r0, y0), pt('c', r1, y1), pt('d', 0, y1), pt('x1', 0, 0), pt('x2', 0, 30)],
  [ln('e1', 'a', 'b'), ln('e2', 'b', 'c'), ln('e3', 'c', 'd'), ln('e4', 'd', 'a'), ln('ax', 'x1', 'x2', { construction: true })]);
parts.simpConeStack = { ...base.box, features: [coneSk('F1', 10, 0, 6, 10), revF('F2', 'F1'), coneSk('F3', 6, 10, 2, 20), revF('F4', 'F3')] };
parts.simpCylPlug = { ...base.box, features: [circSk('F1', STDP('top'), 0, 0, 10), ex('F2', 'F1'),
  circSk('F3', STDP('top'), 0, 0, 3), ex('F4', 'F3', { op: 'cut', end: 'throughBoth' }),
  circSk('F5', STDP('top'), 0, 0, 3), ex('F6', 'F5')] };
parts.simpCylSideBoss = { ...base.box, features: [circSk('F1', STDP('top'), 0, 0, 10), ex('F2', 'F1'),
  circSk('F3', { kind: 'face', id: 'F2.C1' }, 0, 0, 10), ex('F4', 'F3', { depth: 5 }),
  circSk('F5', { kind: 'face', id: 'F4.C1' }, 0, 0, 10), ex('F6', 'F5', { depth: 5 })] };
