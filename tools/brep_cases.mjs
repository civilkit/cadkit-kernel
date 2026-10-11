// Single-feature B-rep parts (one sketch + extrude/revolve on an empty body):
// the 15 bodies of crates/ck-topo/tests/oracle_bodies.json rebuilt as .ckpart
// documents, plus a few extra end conditions. Consumers:
//   tools/brep_golden.mjs  -> crates/ck-model/tests/golden/brep.json (oracle replies)
//   tools/diff.mjs --cases brep   (massProps / sketch inspect only: the regen
//                                  reply itself carries the mesh, which ck-tess owns)
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const STD = (id) => ({ kind: 'std', id });

function rect(x0, y0, x1, y1, tags = ['e1', 'e2', 'e3', 'e4'], pfx = 'p') {
  const P = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y], i) => pt(`${pfx}${i + 1}`, x, y));
  const E = tags.map((t, i) => line(t, `${pfx}${i + 1}`, `${pfx}${((i + 1) % 4) + 1}`));
  return [P, E];
}
const sketch = (plane, points, entities) => ({ type: 'sketch', id: 'F1', plane: STD(plane), points, entities, constraints: [] });
const extrude = (o = {}) => ({ type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10, ...o });
const revolve = (o = {}) => ({ type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360, ...o });
// construction axis along +Y through the origin
const AX = { points: [pt('a1', 0, 0), pt('a2', 0, 10)], ent: line('ax', 'a1', 'a2', { construction: true }) };
const withAxis = (P, E) => [[...P, ...AX.points], [...E, AX.ent]];
const part = (features) => ({ format: 'ck-part', version: 1, features });

const parts = {};
{
  const [P, E] = rect(0, 0, 40, 20);
  parts.box = part([sketch('top', P, E), extrude()]);
  parts.draft = part([sketch('front', P, E), extrude({ draft: 10 })]);
  parts.flip = part([sketch('front', P, E), extrude({ flip: true })]);
  parts.mid = part([sketch('front', P, E), extrude({ end: 'mid' })]);
  parts.through = part([sketch('front', P, E), extrude({ end: 'through' })]);
  parts.throughBoth = part([sketch('front', P, E), extrude({ end: 'throughBoth' })]);
  parts.throughFlip = part([sketch('front', P, E), extrude({ end: 'through', flip: true })]);
  parts.zeroDepth = part([sketch('front', P, E), extrude({ depth: 0 })]);
  parts.exprDepth = part([sketch('right', P, E), extrude({ depth: '=2*w' })]);
  parts.exprDepth.variables = [{ name: 'w', expr: '7' }];
  parts.draftOut = part([sketch('top', P, E), extrude({ draft: 5, draftOut: true, flip: true })]);
  const [H, HE] = rect(5, 5, 15, 15, ['h1', 'h2', 'h3', 'h4'], 'q');
  parts.box3 = part([sketch('front', [...P, ...H, pt('c5', 30, 10)],
    [...E, { type: 'circle', id: 'e5', c: 'c5', r: 4 }, ...HE]), extrude()]);
  // face-based sketch plane + plane feature on a face
  const [P2, E2] = rect(2, 2, 10, 8, ['k1', 'k2', 'k3', 'k4']);
  parts.faceSketch = part([sketch('top', P, E), extrude(),
    { type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: P2, entities: E2, constraints: [] },
    { type: 'plane', id: 'F4', mode: 'offset', base: { kind: 'face', id: 'F2.S.e2' }, offset: 3 },
    { type: 'sketch', id: 'F5', plane: { kind: 'feature', id: 'F4' }, points: [], entities: [], constraints: [] }]);
}
parts.cyl = part([sketch('top', [pt('c', 5, 3)], [{ type: 'circle', id: 'e7', c: 'c', r: 10 }]), extrude()]);
parts.dext = part([sketch('front', [pt('p1', -10, 0), pt('p2', 10, 0), pt('c', 0, 0)],
  [line('e1', 'p1', 'p2'), { type: 'arc', id: 'e2', c: 'c', p1: 'p2', p2: 'p1' }]), extrude({ depth: 5 })]);
{
  const [A, AE] = rect(0, 0, 10, 10, ['a0', 'a1', 'a2', 'a3'], 'p');
  const [B, BE] = rect(20, 0, 30, 10, ['b0', 'b1', 'b2', 'b3'], 'q');
  parts.two = part([sketch('front', [...A, ...B], [...AE, ...BE]), extrude({ depth: 5 })]);
}
{
  const [P, E] = withAxis(...rect(10, 0, 20, 5));
  parts.rev360 = part([sketch('front', P, E), revolve()]);
  parts.rev90 = part([sketch('front', P, E), revolve({ angle: 90 })]);
  parts.rev90flip = part([sketch('front', P, E), revolve({ angle: 90, flip: true })]);
  parts.rev90mid = part([sketch('front', P, E), revolve({ angle: 90, end: 'mid' })]);
  const [Q, QE] = withAxis(...rect(0, 0, 20, 5));
  parts.revax360 = part([sketch('front', Q, QE), revolve()]);
  parts.revax90 = part([sketch('front', Q, QE), revolve({ angle: 90 })]);
}
parts.sphere = part([sketch('front', [pt('c', 0, 0), pt('s', 0, -10), pt('n', 0, 10), ...AX.points],
  [{ type: 'arc', id: 'e1', c: 'c', p1: 's', p2: 'n' }, line('e2', 'n', 's'), AX.ent]), revolve()]);
parts.torus = part([sketch('front', [pt('c', 20, 0), ...AX.points], [{ type: 'circle', id: 'e1', c: 'c', r: 5 }, AX.ent]), revolve()]);
parts.cone = part([sketch('front', [pt('p1', 5, 0), pt('p2', 15, 0), pt('p3', 5, 10), ...AX.points],
  [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p1'), AX.ent]), revolve({ angle: 120 })]);

// ck-ops-backed features on the 40×20×10 top-plane box F2 (booleans,
// patterns, mirror, fillets). Edge order / closed SSI circle starts after
// booleans are ck-ops's (see crates/ck-model/tests/brep_golden.rs OPS).
{
  const [P, E] = rect(0, 0, 40, 20);
  const base = () => [sketch('top', P, E), extrude()];
  const circ = (id, plane, cx, cy, r) => ({ type: 'sketch', id, plane, points: [pt('c', cx, cy)], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
  const ex2 = (id, sk, o = {}) => ({ type: 'extrude', id, op: 'add', sketch: sk, end: 'blind', depth: 10, ...o });
  const hole = [circ('F3', STD('top'), 5, 5, 2), ex2('F4', 'F3', { op: 'cut', end: 'throughBoth' })];
  parts.cutHole = part([...base(), ...hole]);
  parts.cutThroughFlip = part([...base(), circ('F3', STD('top'), 20, 10, 3), ex2('F4', 'F3', { op: 'cut', end: 'through', flip: true })]);
  parts.cutMiss = part([...base(), circ('F3', STD('top'), 100, 10, 3), ex2('F4', 'F3', { op: 'cut', end: 'throughBoth' })]);
  parts.pocket = part([...base(), circ('F3', { kind: 'face', id: 'F2.C1' }, 10, 10, 3), ex2('F4', 'F3', { op: 'cut', depth: 4 })]);
  parts.boss = part([...base(), circ('F3', { kind: 'face', id: 'F2.C1' }, 10, 10, 3), ex2('F4', 'F3', { depth: 5 })]);
  parts.lpatternCut = part([...base(), ...hole, { type: 'lpattern', id: 'F5', seeds: ['F4'], dir: { kind: 'axis', axis: 'x' }, spacing: 10, count: 3 }]);
  parts.cpatternCut = part([...base(), ...hole, { type: 'cpattern', id: 'F5', seeds: ['F4'], axis: { kind: 'axis', axis: 'y' }, count: 3, angle: 90, equal: true }]);
  parts.cpatternBoss = part([...base(), circ('F3', { kind: 'face', id: 'F2.C1' }, 30, 10, 3), ex2('F4', 'F3', { depth: 5 }),
    { type: 'cpattern', id: 'F5', seeds: ['F4'], axis: { kind: 'axis', axis: 'y' }, count: 4, angle: 360, equal: true }]);
  parts.lpatternRev = part([...base(), ...hole, { type: 'lpattern', id: 'F5', seeds: ['F4'], dir: { kind: 'axis', axis: 'z' }, spacing: 8, count: 2, reverse: true }]);
  parts.mirrorTwo = part([...base(), ...hole, circ('F6', STD('top'), 5, 15, 2), ex2('F7', 'F6', { depth: 15 }),
    { type: 'mirror', id: 'F5', seeds: ['F4', 'F7'], plane: STD('right') }]);
  const below = { type: 'plane', id: 'F5', mode: 'offset', base: STD('top'), offset: -5 };
  const onBelow = circ('F3', { kind: 'feature', id: 'F5' }, 20, 10, 3);
  parts.upToNextAdd = part([...base(), below, onBelow, ex2('F4', 'F3', { end: 'upToNext' })]);
  parts.upToNextOpen = part([...base(), below, onBelow, ex2('F4', 'F3', { end: 'upToNext', flip: true })]);
  parts.upToNextCut = part([...base(), circ('F3', { kind: 'face', id: 'F2.C1' }, 20, 10, 3), ex2('F4', 'F3', { op: 'cut', end: 'upToNext' })]);
  parts.upToNextNone = part([...base(), below, onBelow, ex2('F4', 'F3', { op: 'cut', end: 'upToNext', flip: true })]);
  parts.mirrorBody = part([...base(), ...hole, { type: 'mirror', id: 'F5', seeds: [], plane: STD('right'), body: true }]);
  const fil = (r, edges) => ({ type: 'fillet', id: 'F3', radius: r, edges });
  parts.filletOne = part([...base(), fil(2, [{ faces: ['F2.S.e1', 'F2.C1'], at: [20, 10, 0] }])]);
  parts.filletTwo = part([...base(), fil(2, [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }, { faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }])]);
  parts.filletRail = part([...base(), fil(3, [{ faces: ['F2.S.e2', 'F2.S.e1'], at: [40, 5, 0] }])]);
  parts.filletTooBig = part([...base(), fil(30, [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }])]);
  const ch = (o, edges) => ({ type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 2, edges, ...o });
  const e1 = [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }];
  parts.chamferOne = part([...base(), ch({}, e1)]);
  parts.chamferTwo = part([...base(), ch({}, [...e1, { faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }])]);
  parts.chamferDD = part([...base(), ch({ chamferType: 'distanceDistance', distance2: 3 }, e1)]);
  parts.chamferAD = part([...base(), ch({ chamferType: 'angleDistance', angle: 30 }, e1)]);
  parts.chamferVertex = part([...base(), ch({ chamferType: 'vertex' }, e1)]);
  parts.chamferRail = part([...base(), ch({ distance: 3 }, [{ faces: ['F2.S.e2', 'F2.S.e1'], at: [40, 5, 0] }])]);
  parts.chamferBig = part([...base(), ch({ distance: 30 }, e1)]);
  const cylTop = [{ faces: ['F2.C1', 'F2.S.e7'], at: [15, 10, -3] }];
  parts.chamferCylTop = part([...parts.cyl.features, ch({ distance: 1 }, cylTop)]);
  parts.chamferCylTopDD = part([...parts.cyl.features, ch({ chamferType: 'distanceDistance', distance: 1, distance2: 2 }, cylTop)]);
  parts.filletCylTop = part([...parts.cyl.features, fil(2, cylTop)]);
  parts.filletCylBot = part([...parts.cyl.features, fil(2, [{ faces: ['F2.C0', 'F2.S.e7'], at: [15, 0, -3] }])]);
}

parts.mirrorBodyCone = part([...parts.cone.features, { type: 'plane', id: 'F8', mode: 'offset', base: STD('right'), offset: -30 },
  { type: 'mirror', id: 'F9', seeds: [], plane: { kind: 'feature', id: 'F8' }, body: true }]);

// shell (hollow) cases
{
  const sh = (base, faces, thickness, extra = {}) => part([...base.features, { type: 'shell', id: 'F9', faces, thickness, ...extra }]);
  parts.shellBoxClosed = sh(parts.box, [], 1);
  parts.shellBoxOpen = sh(parts.box, ['F2.C1'], 1);
  parts.shellBoxOpenSide = sh(parts.box, ['F2.S.e1'], 1);
  parts.shellBoxOpenTwo = sh(parts.box, ['F2.S.e1', 'F2.C1'], 1);
  parts.shellBoxOpenThick = sh(parts.box, ['F2.C1'], 4.99);
  parts.shellBoxOutward = sh(parts.box, ['F2.C1'], 1, { outward: true });
  parts.shellBoxTooThick = sh(parts.box, [], 5);
  parts.shellBoxReverses = sh(parts.box, [], 6);
  parts.shellBoxOpenTooThick = sh(parts.box, ['F2.C1'], 20);
  parts.shellCylOpen = sh(parts.cyl, ['F2.C1'], 1);
  parts.shellCylClosed = sh(parts.cyl, [], 1);
  parts.shellCylTooThick = sh(parts.cyl, ['F2.C1'], 10);
}

const m = (msg) => JSON.stringify({ id: 1, msg });
/// Follow-up requests on each part's state (compared byte-for-byte).
export const queries = [
  { type: 'massProps', tag: 'main' },
  { type: 'massProps', tag: 'main', bodies: true },
  { type: 'massProps', tag: 'main', bodies: true, densities: [2.5, 0.5] },
  { type: 'massProps', tag: 'main', bodies: true, densities: [1] },
  { type: 'massProps', tag: 'main', debug: true },
  { type: 'inspect', tag: 'main', ref: { kind: 'sketch', id: 'F1' } },
];

export const regenMsg = (p) => ({ type: 'regen', tag: 'main', part: p });
export { parts };
/// diff.mjs cases: body 0 (the regen) is run but not compared.
export default Object.entries(parts).map(([name, p]) => ({
  name: `brep:${name}`, ignore: [0], bodies: [m(regenMsg(p)), ...queries.map(m)],
}));
