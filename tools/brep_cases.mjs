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

import { readFileSync } from 'node:fs';
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
// twisted extrudes (`twist` degrees over the full depth about the sketch origin;
// not an oracle option -- the original ignores it, so only volume matches it)
{
  const [P, E] = rect(0, 0, 40, 20);
  parts.twistBox = part([sketch('top', P, E), extrude({ twist: 90 })]);
  parts.twistBoxNeg = part([sketch('top', P, E), extrude({ twist: -90, end: 'mid' })]);
  parts.twistBoxCentre = part([sketch('top', P, E), extrude({ twist: 90, twistCentre: [20, 10] })]);
  const ring = [pt('c', 5, 3), pt('h', 8, 5)];
  const rE = [{ type: 'circle', id: 'e7', c: 'c', r: 10 }, { type: 'circle', id: 'e8', c: 'h', r: 3 }];
  parts.twistRing = part([sketch('top', ring, rE), extrude({ twist: 360 })]);
  parts.twistCylAxis = part([sketch('top', [pt('c', 0, 0)], [{ type: 'circle', id: 'e1', c: 'c', r: 8 }]), extrude({ twist: 180 })]);
}
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
  for (const a of [30, 150, 200, 270]) parts[`rev${a}`] = part([sketch('front', P, E), revolve({ angle: a })]);
  parts.rev200flip = part([sketch('front', P, E), revolve({ angle: 200, flip: true })]);
  parts.rev270mid = part([sketch('front', P, E), revolve({ angle: 270, end: 'mid' })]);
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
  // chains / corners on the box (top face C1 at y=10; side faces e1..e4)
  const top4 = [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }, { faces: ['F2.C1', 'F2.S.e2'], at: [40, 10, -10] },
    { faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }, { faces: ['F2.C1', 'F2.S.e4'], at: [0, 10, -10] }];
  parts.filletTop4 = part([...base(), fil(2, top4)]);
  parts.chamferTop4 = part([...base(), ch({}, top4)]);
  parts.filletCorner2 = part([...base(), fil(2, top4.slice(0, 2))]);
  parts.chamferCorner2 = part([...base(), ch({}, top4.slice(0, 2))]);
  parts.filletChain3 = part([...base(), fil(2, top4.slice(0, 3))]);
  parts.filletCorner3 = part([...base(), fil(2, [top4[0], top4[1], { faces: ['F2.S.e1', 'F2.S.e2'], at: [40, 5, 0] }])]);
  parts.chamferCorner3 = part([...base(), ch({}, [top4[0], top4[1], { faces: ['F2.S.e1', 'F2.S.e2'], at: [40, 5, 0] }])]);
  parts.filletTwoParallel = part([...base(), fil(2, [top4[0], top4[2]])]);
  // circular edges of holes / pockets / bosses on the box
  const rim = (f1, f2, at) => [{ faces: [f1, f2], at }];
  parts.holeRimFillet = part([...base(), ...hole, fil(0.5, rim('F2.C1', 'F4.S.e1', [7, 10, -5]))]);
  parts.holeRimChamfer = part([...base(), ...hole, ch({ distance: 0.5 }, rim('F2.C1', 'F4.S.e1', [7, 10, -5]))]);
  parts.holeBotFillet = part([...base(), ...hole, fil(0.5, rim('F2.C0', 'F4.S.e1', [7, 0, -5]))]);
  parts.holeBothChamfer = part([...base(), ...hole, ch({ distance: 0.5 }, [...rim('F2.C1', 'F4.S.e1', [7, 10, -5]), ...rim('F2.C0', 'F4.S.e1', [7, 0, -5])])]);
  parts.pocketFloorFillet = part([...parts.pocket.features, fil(1, rim('F4.C0', 'F4.S.e1', [13, 6, -10]))]);
  parts.pocketRimChamfer = part([...parts.pocket.features, ch({ distance: 0.5 }, rim('F2.C1', 'F4.S.e1', [13, 10, -10]))]);
  parts.bossTopFillet = part([...parts.boss.features, fil(1, rim('F4.C1', 'F4.S.e1', [13, 15, -10]))]);
  parts.bossBaseFillet = part([...parts.boss.features, fil(1, rim('F2.C1', 'F4.S.e1', [13, 10, -10]))]);
  parts.bossBaseChamfer = part([...parts.boss.features, ch({ distance: 1 }, rim('F2.C1', 'F4.S.e1', [13, 10, -10]))]);
  parts.bossTopChamfer = part([...parts.boss.features, ch({ distance: 1 }, rim('F4.C1', 'F4.S.e1', [13, 15, -10]))]);
  // revolved frustum (plane / cone / plane) and a ring with a conical bore
  {
    const [P, E] = withAxis([pt('p1', 0, 0), pt('p2', 10, 0), pt('p3', 6, 10), pt('p4', 0, 10)],
      [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p4'), line('e4', 'p4', 'p1')]);
    const frustum = part([sketch('front', P, E), revolve()]);
    parts.frustum = frustum;
    const [P2, E2] = withAxis([pt('p1', 4, 0), pt('p2', 10, 0), pt('p3', 6, 10), pt('p4', 4, 10)],
      [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p4'), line('e4', 'p4', 'p1')]);
    parts.ring = part([sketch('front', P2, E2), revolve()]);
  }
  parts.frustumBotFillet = part([...parts.frustum.features, fil(1, rim('F2.S.e1', 'F2.S.e2', [-10, 0, 0]))]);
  parts.frustumTopFillet = part([...parts.frustum.features, fil(1, rim('F2.S.e2', 'F2.S.e3', [-6, 10, 0]))]);
  parts.frustumBotChamfer = part([...parts.frustum.features, ch({ distance: 1 }, rim('F2.S.e1', 'F2.S.e2', [-10, 0, 0]))]);
  parts.frustumTopChamfer = part([...parts.frustum.features, ch({ distance: 1 }, rim('F2.S.e2', 'F2.S.e3', [-6, 10, 0]))]);
  parts.frustumBothChamfer = part([...parts.frustum.features, ch({ distance: 1 }, [...rim('F2.S.e1', 'F2.S.e2', [-10, 0, 0]), ...rim('F2.S.e2', 'F2.S.e3', [-6, 10, 0])])]);
  parts.ringInnerFillet = part([...parts.ring.features, fil(1, rim('F2.S.e3', 'F2.S.e4', [-4, 10, 0]))]);
  parts.ringInnerChamfer = part([...parts.ring.features, ch({ distance: 1 }, rim('F2.S.e1', 'F2.S.e4', [-4, 0, 0]))]);
  parts.ringOuterFillet = part([...parts.ring.features, fil(1, rim('F2.S.e2', 'F2.S.e3', [-6, 10, 0]))]);
  parts.chamferVertexD3 = part([...base(), ch({ chamferType: 'vertex', distance: 2, distance2: 3, distance3: 4 }, [{ faces: ['F2.C1', 'F2.S.e1'], at: [39, 10, 0] }])]);
  parts.chamferVertexBot = part([...base(), ch({ chamferType: 'vertex', distance: 1.5 }, [{ faces: ['F2.C0', 'F2.S.e3'], at: [1, 0, -20] }])]);
  // partial arcs: the half-round edge of a D-shaped prism, and a D boss on the box top (concave arc)
  parts.dextArcFillet = part([...parts.dext.features, fil(1, rim('F2.C1', 'F2.S.e2', [0, 10, 5]))]);
  parts.dextArcChamfer = part([...parts.dext.features, ch({ distance: 1 }, rim('F2.C1', 'F2.S.e2', [0, 10, 5]))]);
  parts.dextArcBotFillet = part([...parts.dext.features, fil(1, rim('F2.C0', 'F2.S.e2', [0, 10, 0]))]);
  {
    const dsk = { type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: [pt('p1', 10, 5), pt('p2', 20, 5), pt('c', 15, 5)],
      entities: [line('e1', 'p1', 'p2'), { type: 'arc', id: 'e2', c: 'c', p1: 'p2', p2: 'p1' }], constraints: [] };
    const dboss = [...base(), dsk, ex2('F4', 'F3', { depth: 5 })];
    parts.dboss = part(dboss);
    parts.dbossBaseFillet = part([...dboss, fil(1, rim('F2.C1', 'F4.S.e2', [15, 10, -10]))]);
    parts.dbossBaseChamfer = part([...dboss, ch({ distance: 1 }, rim('F2.C1', 'F4.S.e2', [15, 10, -10]))]);
    parts.dbossTopFillet = part([...dboss, fil(1, rim('F4.C1', 'F4.S.e2', [15, 15, -10]))]);
    parts.dbossTopChamfer = part([...dboss, ch({ distance: 1 }, rim('F4.C1', 'F4.S.e2', [15, 15, -10]))]);
  }
  // vertex blend: three fillets at a box corner give a sphere patch V0
  parts.filletCornerBot = part([...base(), fil(2, [{ faces: ['F2.C0', 'F2.S.e1'], at: [20, 0, 0] }, { faces: ['F2.C0', 'F2.S.e4'], at: [0, 0, -10] }, { faces: ['F2.S.e1', 'F2.S.e4'], at: [0, 5, 0] }])]);
  parts.filletCornerBig = part([...base(), fil(4, [{ faces: ['F2.C1', 'F2.S.e3'], at: [20, 10, -20] }, { faces: ['F2.C1', 'F2.S.e4'], at: [0, 10, -10] }, { faces: ['F2.S.e3', 'F2.S.e4'], at: [0, 5, -20] }])]);
  // non-right dihedral: rails of a triangular prism
  {
    const tri = part([sketch('top', [pt('p1', 0, 0), pt('p2', 30, 0), pt('p3', 10, 20)], [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p1')]), extrude()]);
    parts.triPrism = tri;
    parts.triChamferRail = part([...tri.features, ch({ distance: 1.5 }, rim('F2.S.e1', 'F2.S.e2', [30, 5, 0]))]);
    parts.triChamferRail2 = part([...tri.features, ch({ distance: 1.5 }, rim('F2.S.e2', 'F2.S.e3', [10, 5, -20]))]);
    parts.triFilletRail = part([...tri.features, fil(1.5, rim('F2.S.e1', 'F2.S.e2', [30, 5, 0]))]);
    parts.triChamferTop = part([...tri.features, ch({ distance: 1.5 }, rim('F2.C1', 'F2.S.e2', [20, 10, -10]))]);
  }
  parts.chamferRail = part([...base(), ch({ distance: 3 }, [{ faces: ['F2.S.e2', 'F2.S.e1'], at: [40, 5, 0] }])]);
  parts.chamferBig = part([...base(), ch({ distance: 30 }, e1)]);
  const cylTop = [{ faces: ['F2.C1', 'F2.S.e7'], at: [15, 10, -3] }];
  parts.chamferCylTop = part([...parts.cyl.features, ch({ distance: 1 }, cylTop)]);
  parts.chamferCylTopDD = part([...parts.cyl.features, ch({ chamferType: 'distanceDistance', distance: 1, distance2: 2 }, cylTop)]);
  parts.filletCylTop = part([...parts.cyl.features, fil(2, cylTop)]);
  parts.filletCylBot = part([...parts.cyl.features, fil(2, [{ faces: ['F2.C0', 'F2.S.e7'], at: [15, 0, -3] }])]);
}

// draft feature on the 40x20x10 top-plane box: side faces about the bottom
// cap (F2.C0) / top cap (F2.C1); and on a cylinder.
{
  const [P, E] = rect(0, 0, 40, 20);
  const base = () => [sketch('top', P, E), extrude()];
  const dr = (faces, o = {}) => ({ type: 'draft', id: 'F3', neutral: { kind: 'face', id: 'F2.C0' }, faces, angle: 5, ...o });
  const sides = ['F2.S.e1', 'F2.S.e2', 'F2.S.e3', 'F2.S.e4'];
  parts.draftBox = part([...base(), dr(sides)]);
  parts.draftBoxFlip = part([...base(), dr(sides, { flip: true })]);
  parts.draftBoxTop = part([...base(), dr(sides, { neutral: { kind: 'face', id: 'F2.C1' } })]);
  parts.draftBoxStd = part([...base(), dr(sides, { neutral: STD('top') })]);
  parts.draftBoxFeat = part([...base(), { type: 'plane', id: 'F4', mode: 'offset', base: STD('top'), offset: 10 }, dr(sides, { id: 'F5', neutral: { kind: 'feature', id: 'F4' } })]);
  parts.draftBox10 = part([...base(), dr(sides, { angle: 10 })]);
  parts.draftBox3 = part([...base(), dr(sides, { angle: 3 })]);
  parts.draftOne = part([...base(), dr(['F2.S.e1'])]);
  parts.draftTwo = part([...base(), dr(['F2.S.e1', 'F2.S.e3'], { angle: 8 })]);
  parts.draftCyl = part([sketch('top', [pt('c', 5, 3)], [{ type: 'circle', id: 'e7', c: 'c', r: 10 }]), extrude(),
    dr(['F2.S.e7'], { angle: 3 })]);
  parts.draftParallel = part([...base(), dr(['F2.C1'])]);
  parts.draftZero = part([...base(), dr(sides, { angle: 0 })]);
  parts.draftRight = part([...base(), dr(sides, { angle: 90 })]);
  parts.draftNoFaces = part([...base(), dr([])]);
  parts.draftBadRef = part([...base(), dr(['nope'])]);
  parts.draftBig = part([...base(), dr(sides, { angle: 89 })]);
}

parts.mirrorBodyCone = part([...parts.cone.features, { type: 'plane', id: 'F8', mode: 'offset', base: STD('right'), offset: -30 },
  { type: 'mirror', id: 'F9', seeds: [], plane: { kind: 'feature', id: 'F8' }, body: true }]);

// ---- sweep (circle / square profiles along a line, an arc and a helix)
{
  const sk = (id, plane, points, entities) => ({ type: 'sketch', id, plane: STD(plane), points, entities, constraints: [] });
  const circleSk = (id, plane, cx, cy, r) => sk(id, plane, [pt('c', cx, cy)], [{ type: 'circle', id: 'e1', c: 'c', r }]);
  const squareSk = (id, plane, x0, y0, w) => {
    const [P, E] = rect(x0, y0, x0 + w, y0 + w, ['e5', 'e6', 'e7', 'e8']);
    return sk(id, plane, P, E);
  };
  const trapSk = (id, plane) => sk(id, plane, [pt('p1', 3.3, -0.5), pt('p2', 4.5, -0.3), pt('p3', 4.5, 0.3), pt('p4', 3.3, 0.5)],
    [line('e5', 'p1', 'p2'), line('e6', 'p2', 'p3'), line('e7', 'p3', 'p4'), line('e8', 'p4', 'p1')]);
  const lineSk = (id, a, b) => sk(id, 'front', [pt('a', ...a), pt('b', ...b)], [line('e1', 'a', 'b')]);
  const arcSk = (id) => sk(id, 'front', [pt('c', -10, 0), pt('a', 0, 0), pt('b', -10, 10)], [{ type: 'arc', id: 'e1', c: 'c', p1: 'a', p2: 'b' }]);
  const helixF = (o = {}) => [circleSk('F1', 'front', 0, 0, 4),
    { type: 'helix', id: 'F2', sketch: 'F1', definedBy: 'pitchRevs', pitch: 2, revolutions: 3, height: 6, startAngle: 0, leftHand: false, reverse: false, ...o }];
  const sweep = (path, o = {}) => ({ type: 'sweep', id: 'F4', op: 'add', sketch: 'F3', path, ...o });
  const SK = (id) => ({ kind: 'sketch', id });
  const CV = (id) => ({ kind: 'curve', id });
  parts.sweepLine = part([lineSk('F2', [0, 0], [0, 20]), circleSk('F3', 'top', 0, 0, 3), sweep(SK('F2'))]);
  parts.sweepLineNeg = part([lineSk('F2', [0, 0], [0, -20]), squareSk('F3', 'top', 0, 0, 2), sweep(SK('F2'))]);
  parts.sweepLineFar = part([lineSk('F2', [0, 5], [0, 25]), squareSk('F3', 'top', 0, 0, 2), sweep(SK('F2'))]);
  parts.sweepArc = part([arcSk('F2'), circleSk('F3', 'top', 0, 0, 2), sweep(SK('F2'))]);
  parts.sweepArcSquare = part([arcSk('F2'), squareSk('F3', 'top', 0, 0, 2), sweep(SK('F2'))]);
  parts.sweepNotNormal = part([lineSk('F2', [0, 0], [0, 20]), squareSk('F3', 'front', 0, 0, 2), sweep(SK('F2'))]);
  parts.sweepTwist = part([lineSk('F2', [5, 0], [5, 20]), squareSk('F3', 'top', 1, 1, 2), sweep(SK('F2'), { twist: 90 })]);
  parts.sweepTwistNeg = part([lineSk('F2', [0, 0], [0, -20]), squareSk('F3', 'top', 1, 1, 2), sweep(SK('F2'), { twist: -135 })]);
  parts.sweepNoPath =part([circleSk('F3', 'top', 0, 0, 3), sweep(null)]);
  parts.sweepHelixTrap = part([...helixF(), trapSk('F3', 'top'), sweep(CV('F2'))]);
  parts.sweepHelixRev = part([...helixF({ reverse: true }), trapSk('F3', 'top'), sweep(CV('F2'))]);
  parts.sweepHelixLeft = part([...helixF({ leftHand: true }), trapSk('F3', 'top'), sweep(CV('F2'))]);
  parts.sweepHelixSquare = part([...helixF(), squareSk('F3', 'top', 3.5, -0.5, 1), sweep(CV('F2'))]);
  parts.sweepHelixCircle = part([...helixF(), circleSk('F3', 'top', 4, 0, 0.5), sweep(CV('F2'))]);
  // thread-like cut: a groove swept along a helix into a bored cylinder
  const tube = [sk('F10', 'front', [pt('o', 0, 0)], [{ type: 'circle', id: 'e1', c: 'o', r: 6 }, { type: 'circle', id: 'e2', c: 'o', r: 3.4 }]),
    { type: 'extrude', id: 'F11', op: 'add', sketch: 'F10', end: 'blind', depth: 8 }];
  parts.sweepCut = part([...tube, circleSk('F1', 'front', 0, 0, 4),
    { type: 'helix', id: 'F2', sketch: 'F1', definedBy: 'pitchRevs', pitch: 1.25, revolutions: 4, height: 5, startAngle: 0, leftHand: false, reverse: false },
    trapSk('F3', 'top'), sweep(CV('F2'), { op: 'cut' })]);
}
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

// Surface bodies (surfacePlanar / surfaceRevolve / surfaceExtrude, open chains)
{
  const [P, E] = rect(0, 0, 40, 20);
  const sp = (id, s) => ({ type: 'surfacePlanar', id, sketch: s });
  const se = (id, s, o = {}) => ({ type: 'surfaceExtrude', id, sketch: s, end: 'blind', depth: 10, ...o });
  const sr = (id, s, o = {}) => ({ type: 'surfaceRevolve', id, sketch: s, axis: 'ax', end: 'blind', angle: 360, ...o });
  parts.surfPlanar = part([sketch('top', P, E), sp('F2', 'F1')]);
  parts.surfPlanarSolid = part([sketch('top', P, E), extrude(), { ...sketch('front', P, E), id: 'F3' }, sp('F4', 'F3')]);
  parts.surfPlanarTwo = part([sketch('top', P, E), sp('F2', 'F1'), { ...sketch('front', P, E), id: 'F3' }, sp('F4', 'F3')]);
  parts.surfExtrude = part([sketch('top', P, E), se('F2', 'F1')]);
  parts.surfExtrudeMid = part([sketch('top', P, E), se('F2', 'F1', { end: 'mid' })]);
  parts.surfExtrudeFlip = part([sketch('top', P, E), se('F2', 'F1', { flip: true })]);
  const [Q, QE] = withAxis(...rect(10, 0, 20, 5));
  parts.surfRevolve = part([sketch('front', Q, QE), sr('F2', 'F1', { angle: 180 })]);
  parts.surfRevolve360 = part([sketch('front', Q, QE), sr('F2', 'F1')]);
  // open chains: polyline and spline, closed by a chord internally
  const chain = [pt('c1', 10, 8), pt('c2', 14, 12), pt('c3', 20, 9), pt('a1', 0, 0), pt('a2', 0, 10)];
  const chainE = [line('l1', 'c1', 'c2'), line('l2', 'c2', 'c3'), line('ax', 'a1', 'a2', { construction: true })];
  parts.surfRevolveChain = part([sketch('front', chain, chainE), sr('F2', 'F1', { angle: 180 })]);
  parts.surfExtrudeChain = part([sketch('front', chain, chainE), se('F2', 'F1')]);
  const chain2 = [pt('c1', 20, 9), pt('c2', 14, 12), pt('c3', 10, 8), pt('a1', 0, 0), pt('a2', 0, 10)];
  parts.surfRevolveChainCcw = part([sketch('front', chain2, chainE), sr('F2', 'F1', { angle: 200 })]);
  const sp3 = [pt('c1', 10, 3), pt('c2', 15, 6), pt('c3', 22, 7), pt('a1', 0, 0), pt('a2', 0, 10)];
  parts.surfRevolveSpline = part([sketch('front', sp3, [{ type: 'spline', id: 'e8', pts: ['c1', 'c2', 'c3'] }, line('ax', 'a1', 'a2', { construction: true })]), sr('F2', 'F1', { angle: 180 })]);
  parts.surfExtrudeSpline = part([sketch('front', sp3, [{ type: 'spline', id: 'e8', pts: ['c1', 'c2', 'c3'] }, line('ax', 'a1', 'a2', { construction: true })]), se('F2', 'F1', { depth: 7 })]);
}

// Surface-body operations (knit / fill / trimSurface / thicken / extend / deleteFace / freeform / deleteBody)
{
  const [P, E] = rect(0, 0, 40, 20);
  const sp = (id, s) => ({ type: 'surfacePlanar', id, sketch: s });
  const se = (id, s, o = {}) => ({ type: 'surfaceExtrude', id, sketch: s, end: 'blind', depth: 10, ...o });
  const tube = [sketch('top', P, E), se('F2', 'F1')];
  const cap = [sp('F3', 'F1'), { type: 'plane', id: 'F4', mode: 'offset', base: STD('top'), offset: 10 },
    { type: 'sketch', id: 'F5', plane: { kind: 'feature', id: 'F4' }, points: P, entities: E, constraints: [] }, sp('F6', 'F5')];
  parts.knitBox = part([...tube, ...cap, { type: 'knit', id: 'F7', surfaces: ['F2', 'F3', 'F6'], solid: true }]);
  parts.knitSheet = part([...tube, ...cap, { type: 'knit', id: 'F7', surfaces: ['F2', 'F3'], solid: false }]);
  parts.knitNotClosed = part([...tube, ...cap, { type: 'knit', id: 'F7', surfaces: ['F2', 'F3'], solid: true }]);
  parts.knitMissing = part([...tube, { type: 'knit', id: 'F7', surfaces: ['F2', 'F9'], solid: false }]);
  // half-disc faces (arc + chord share both endpoints): knit must not report knitNearMiss
  parts.knitHalfDisc = part([
    { type: 'sketch', id: 'F1', plane: STD('top'), points: [pt('O', 0, 0), pt('p1', 0, -23), pt('p2', 0, 23)],
      entities: [{ id: 'e3', type: 'arc', c: 'O', p1: 'p1', p2: 'p2' }, line('e4', 'p2', 'p1')], constraints: [] },
    sp('F2', 'F1'),
    { type: 'sketch', id: 'F3', plane: STD('front'), points: [pt('O', 0, 0), pt('p1', 0, -23), pt('p2', 0, 23)],
      entities: [{ id: 'e3', type: 'arc', c: 'O', p1: 'p1', p2: 'p2' }, line('e4', 'p2', 'p1')], constraints: [] },
    sp('F4', 'F3'),
    { type: 'knit', id: 'F5', surfaces: ['F2', 'F4'], solid: false }]);
  const rim = (face, at) => ({ faces: [face], at });
  const top = [rim('F2.S.e1', [20, 10, 0]), rim('F2.S.e2', [40, 10, -10]), rim('F2.S.e3', [20, 10, -20]), rim('F2.S.e4', [0, 10, -10])];
  const bot = [rim('F2.S.e1', [20, 0, 0]), rim('F2.S.e2', [40, 0, -10]), rim('F2.S.e3', [20, 0, -20]), rim('F2.S.e4', [0, 0, -10])];
  const fill = (id, edges, o = {}) => ({ type: 'fill', id, edges, merge: true, continuity: 'G0', ...o });
  parts.fillTop = part([...tube, fill('F3', top)]);
  parts.fillTopLoose = part([...tube, fill('F3', top, { merge: false })]);
  parts.fillBoth = part([...tube, fill('F3', top), fill('F4', bot)]);
  parts.fillSame = part([...tube, fill('F3', [top[0], top[0]])]);
  // two perpendicular sheets for trimSurface
  const [R, RE] = rect(-5, -5, 5, 5);
  const [S, SE] = rect(-6, -6, 6, 6);
  const two = [sketch('right', R, RE), sp('F2', 'F1'), { ...sketch('front', S, SE), id: 'F3' }, sp('F4', 'F3')];
  const trim = (o) => ({ type: 'trimSurface', id: 'F5', surface: 'F2', toolSurface: 'F4', flip: false, mutual: false, ...o });
  parts.trimKeep = part([...two, trim({})]);
  parts.trimFlip = part([...two, trim({ flip: true })]);
  parts.trimMutual = part([...two, trim({ mutual: true })]);
  const stdTrim = (o) => part([{ ...sketch('front', ...rect(-6, -6, 6, 6)), id: 'F1' }, sp('F2', 'F1'), { type: 'trimSurface', id: 'F3', surfaces: ['F2'], plane: STD('right'), flip: false, ...o }]);
  parts.trimStdPlane = stdTrim({});
  parts.trimStdPlaneFlip = stdTrim({ flip: true });
  parts.trimStdPlaneMiss = part([{ ...sketch('front', ...rect(2, 2, 6, 6)), id: 'F1' }, sp('F2', 'F1'), { type: 'trimSurface', id: 'F3', surfaces: ['F2'], plane: STD('right'), flip: true }]);
  parts.trimDeleteTool = part([...two, trim({ deleteTool: true })]);
  parts.thickenFront = part([sketch('top', P, E), sp('F2', 'F1'), { type: 'thicken', id: 'F3', surface: 'F2', thickness: 2, side: 'front' }]);
  parts.thickenBoth = part([sketch('top', P, E), sp('F2', 'F1'), { type: 'thicken', id: 'F3', surface: 'F2', thickness: 2, side: 'both' }]);
  parts.thickenTube = part([...tube, { type: 'thicken', id: 'F3', surface: 'F2', thickness: 1, side: 'back' }]);
  parts.thickenZero = part([...tube, { type: 'thicken', id: 'F3', surface: 'F2', thickness: 0, side: 'front' }]);
  parts.extendPlane = part([sketch('top', P, E), sp('F2', 'F1'), { type: 'extend', id: 'F3', edges: [{ faces: ['F2.P'], at: [20, 0, 0] }], distance: 5 }]);
  parts.deleteFaceBox = part([sketch('top', P, E), extrude(), { type: 'deleteFace', id: 'F3', faces: ['F2.C1'], mode: 'delete', edges: [] }]);
  parts.freeformCap = part([sketch('top', P, E), extrude(), { type: 'freeform', id: 'F3', faces: ['F2.C1'], degreeU: 3, degreeV: 3, spansU: 2, spansV: 2, edits: [], edges: [] }]);
  parts.freeformCapFront = part([sketch('front', P, E), extrude(), { type: 'freeform', id: 'F3', faces: ['F2.C1'], degreeU: 3, degreeV: 3, spansU: 2, spansV: 2, edits: [], edges: [] }]);
  parts.deleteBodySurf = part([...tube, ...cap.slice(0, 1), { type: 'deleteBody', id: 'F4', mode: 'delete', bodies: ['F2'] }]);
  parts.deleteBodySurface = part([...tube, ...cap.slice(0, 1), { type: 'deleteBody', id: 'F4', mode: 'delete', bodies: [{ surface: 'F2' }] }]);
  parts.deleteBodyKeepSurface = part([...tube, ...cap.slice(0, 1), { type: 'deleteBody', id: 'F4', mode: 'keep', bodies: [{ surface: 'F2' }] }]);
  parts.deleteBodyFace = part([...tube, ...cap.slice(0, 1), { type: 'deleteBody', id: 'F4', mode: 'delete', bodies: [{ face: 'F2.S.e1' }] }]);
  parts.deleteBodySolid = part([sketch('top', P, E), extrude(), { ...sketch('front', P, E), id: 'F3' }, sp('F4', 'F3'), { type: 'deleteBody', id: 'F5', mode: 'delete', bodies: [{ face: 'F2.C0' }] }]);
  parts.deleteBodyNone = part([...tube, { type: 'deleteBody', id: 'F3', mode: 'delete', bodies: [] }]);
}

// rib on a shelled 40x20x10 box (sketch plane x runs along world -x: flipped offset plane)
{
  const [P, E] = rect(0, 0, 40, 20);
  const hollow = [sketch('top', P, E), extrude(), { type: 'shell', id: 'F3', faces: ['F2.C1'], thickness: 2 }];
  const plane = { type: 'plane', id: 'F4', mode: 'offset', base: STD('front'), offset: 10, flip: true };
  const onPlane = (pts, ents) => ({ type: 'sketch', id: 'F5', plane: { kind: 'feature', id: 'F4' }, points: pts, entities: ents, constraints: [] });
  const ln = (a, b) => onPlane([pt('a', a[0], a[1]), pt('b', b[0], b[1])], [line('e1', 'a', 'b')]);
  const rib = (t, o = {}) => ({ type: 'rib', id: 'F6', sketch: 'F5', thickness: t, ...o });
  parts.ribHollow = part([...hollow, plane, ln([-5, 5], [-35, 5]), rib(1.2)]);
  parts.ribHollowFlip = part([...hollow, plane, ln([-5, 5], [-35, 5]), rib(1.2, { flip: true })]);
  parts.ribPoly = part([...hollow, plane, onPlane([pt('a', -5, 3), pt('b', -20, 8), pt('c', -35, 3)], [line('e1', 'a', 'b'), line('e2', 'b', 'c')]), rib(1.2)]);
  parts.ribThrough = part([...hollow, plane, ln([5, 5], [-45, 5]), rib(1.2)]);
  parts.ribFar = part([...hollow, plane, ln([100, 5], [130, 5]), rib(1.2)]);
  parts.ribClosed = part([...hollow, plane, onPlane(rect(5, 3, 35, 7, ['r1', 'r2', 'r3', 'r4'], 'q')[0], rect(5, 3, 35, 7, ['r1', 'r2', 'r3', 'r4'], 'q')[1]), rib(1.2)]);
  parts.ribZero = part([...hollow, plane, ln([-5, 5], [-35, 5]), rib(0)]);
  parts.ribNoBody = part([plane, ln([-5, 5], [-35, 5]), rib(1.2)]);
  parts.ribNoSketch = part([...hollow, { type: 'rib', id: 'F6', thickness: 1.2 }]);
}

// lip / groove along the outer rim edges of a shelled box
{
  const [P, E] = rect(0, 0, 40, 20);
  const hollow = [sketch('top', P, E), extrude(), { type: 'shell', id: 'F3', faces: ['F2.C1'], thickness: 2 }];
  const edge = (a, b, at) => ({ faces: [a, b], at });
  const outer = [edge('F2.C1', 'F2.S.e1~1', [20, 10, 0]), edge('F2.C1', 'F2.S.e2~2', [40, 10, -10]), edge('F2.C1', 'F2.S.e3~1', [20, 10, -20]), edge('F2.C1', 'F2.S.e4~1', [0, 10, -10])];
  const lip = (o = {}) => ({ type: 'lip', id: 'F4', kind: 'lip', faces: ['F2.C1'], edges: outer, width: 0.5, height: 1, ...o });
  parts.lipHollow = part([...hollow, lip()]);
  parts.lipGroove = part([...hollow, lip({ kind: 'groove' })]);
  parts.lipOne = part([...hollow, lip({ edges: [outer[0]] })]);
  parts.lipWide = part([...hollow, lip({ width: 3 })]);
  parts.lipNoBody = part([lip()]);
  parts.lipNoFaces = part([...hollow, lip({ faces: [] })]);
  parts.lipNoEdges = part([...hollow, lip({ edges: [] })]);
  parts.lipZeroWidth = part([...hollow, lip({ width: 0 })]);
  parts.lipZeroHeight = part([...hollow, lip({ height: 0 })]);
  parts.lipBadFace = part([...hollow, lip({ faces: ['F9.C0'] })]);
}

// profiles crossing the revolve axis, monotone spline chain
{
  const AX2 = [pt('a1', 0, -20), pt('a2', 0, 20)];
  const axl = line('ax', 'a1', 'a2', { construction: true });
  const sr = (o = {}) => ({ type: 'surfaceRevolve', id: 'F2', sketch: 'F1', axis: 'ax', end: 'blind', angle: 360, ...o });
  const crossRect = [pt('p1', -5, 0), pt('p2', 5, 0), pt('p3', 5, 5), pt('p4', -5, 5)];
  const crossE = [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p4'), line('e4', 'p4', 'p1')];
  parts.srCrossRect = part([sketch('front', [...crossRect, ...AX2], [...crossE, axl]), sr()]);
  parts.rvCrossRect = part([sketch('front', [...crossRect, ...AX2], [...crossE, axl]), revolve({ angle: 360 })]);
  parts.srCrossChain = part([sketch('front', [pt('c1', -5, 3), pt('c2', 6, 8), ...AX2], [line('l1', 'c1', 'c2'), axl]), sr()]);
  const arch = [pt('c1', 10, -10), pt('c2', 14, -3), pt('c3', 15, 5), pt('c4', 12, 14)];
  parts.srSpline = part([sketch('front', [...arch, ...AX2], [{ type: 'spline', id: 'e8', pts: ['c1', 'c2', 'c3', 'c4'] }, axl]), sr({ angle: 180 })]);
}

// extend to a target surface / half-cylinder rim
{
  const [P, E] = rect(0, 0, 40, 20);
  const [T, TE] = rect(-30, -30, 30, 30, ['t1', 't2', 't3', 't4'], 't');
  const sp = (id, sk) => ({ type: 'surfacePlanar', id, sketch: sk });
  const tgt = [{ type: 'plane', id: 'F3', mode: 'offset', base: STD('right'), offset: 60 },
    { type: 'sketch', id: 'F4', plane: { kind: 'feature', id: 'F3' }, points: T, entities: TE, constraints: [] }, sp('F5', 'F4')];
  parts.extendToSurface = part([sketch('top', P, E), sp('F2', 'F1'), ...tgt,
    { type: 'extend', id: 'F6', edges: [{ faces: ['F2.P'], at: [40, 0, -10] }], end: 'surface', target: { kind: 'surface', id: 'F5' } }]);
  // shape linear / soft / reflective add a separate spline face F3.X0 (not reconstructed)
  parts.extendNotBoundary = part([sketch('top', P, E), extrude(),
    { type: 'extend', id: 'F3', edges: [{ faces: ['F2.C0', 'F2.S.e1'], at: [20, 0, 0] }], distance: 7 }]);
  parts.extendNoEdges = part([sketch('top', P, E), sp('F2', 'F1'), { type: 'extend', id: 'F3', edges: [], distance: 7 }]);
}

// loft cases (ck-ops loft): profiles on offset top planes, matched by the
// station rules pinned in crates/ck-ops/src/loft/stations.rs
{
  const ngon = (n, r, a0 = 0, pf = 'p') => {
    const P = [], E = [];
    for (let i = 0; i < n; i++) { const a = a0 + (2 * Math.PI * i) / n; P.push(pt(`${pf}${i + 1}`, r * Math.cos(a), r * Math.sin(a))); }
    for (let i = 0; i < n; i++) E.push(line(`e${i + 1}`, `${pf}${i + 1}`, `${pf}${((i + 1) % n) + 1}`));
    return [P, E];
  };
  const sq = (a) => rect(-a, -a, a, a);
  const circ = (cx, cy, r) => [[pt('c', cx, cy)], [{ type: 'circle', id: 'e1', c: 'c', r }]];
  const loftOf = (profiles, extra = {}) => {
    const fs = []; let n = 0; const sks = [];
    for (const [off, [P, E]] of profiles) {
      let pl = STD('top');
      if (off !== 0) { const id = `F${++n}`; fs.push({ type: 'plane', id, mode: 'offset', base: STD('top'), offset: off }); pl = { kind: 'feature', id }; }
      const sid = `F${++n}`; fs.push({ type: 'sketch', id: sid, plane: pl, points: P, entities: E, constraints: [] }); sks.push(sid);
    }
    fs.push({ type: 'loft', id: `F${++n}`, op: 'add', sketches: sks, ...extra });
    return part(fs);
  };
  parts.loftSq2 = loftOf([[0, sq(10)], [30, sq(5)]]);
  parts.loftSq3 = loftOf([[0, sq(10)], [30, sq(5)], [50, sq(8)]]);
  parts.loftCircSq = loftOf([[0, circ(0, 0, 8)], [30, sq(5)]]);
  parts.loftCircCirc = loftOf([[0, circ(0, 0, 8)], [30, circ(2, 0, 4)]]);
  parts.loftTriSq = loftOf([[0, ngon(3, 10)], [30, sq(5)]]);
  parts.loftSqOct = loftOf([[0, sq(10)], [30, ngon(8, 6)]]);
  parts.loftSqTwist = loftOf([[0, sq(10)], [30, ngon(4, 7, Math.PI / 4)]]);
  parts.loftSurface = loftOf([[0, sq(10)], [30, sq(5)]], { surface: true });
  parts.loftSurfaceCirc = loftOf([[0, circ(0, 0, 8)], [30, sq(5)]], { surface: true });
}

// loft guides / end normals / connectors / centre line (ck-ops loft/guides.rs, smooth::through_ends).
// Straight guides, end tangents and connectors reproduce the oracle's volumes; curved guides and
// curved centre lines are not in the gate (see docs/ck-ops-stubs.md, tools/stub_cases.mjs).
{
  const [s1, p2, s3] = parts.loftSq2.features;   // square r10 at y=0, plane y=30, square r5
  const base3 = [s1, p2, s3];
  const lineSk = (id, plane, a, b) => ({ type: 'sketch', id, plane, points: [pt('g1', ...a), pt('g2', ...b)], entities: [line('e1', 'g1', 'g2')], constraints: [] });
  const loft = (o = {}) => ({ type: 'loft', id: 'F5', op: 'add', sketches: ['F1', 'F3'], ...o });
  parts.loftGuideLine = part([...base3, lineSk('F4', STD('front'), [10, 0], [5, 30]), loft({ guides: [{ sketch: 'F4' }] })]);
  parts.loftGuideRev = part([...base3, lineSk('F4', STD('front'), [5, 30], [10, 0]), loft({ guides: [{ sketch: 'F4' }] })]);
  parts.loftGuide2 = part([...base3, lineSk('F4', STD('front'), [10, 0], [5, 30]), lineSk('F6', STD('front'), [-10, 0], [-5, 30]), loft({ guides: [{ sketch: 'F4' }, { sketch: 'F6' }] })]);
  parts.loftCentreSlant = part([...base3, lineSk('F4', STD('front'), [0, 0], [8, 30]), loft({ centreLine: { sketch: 'F4' } })]);
  parts.loftStartNormal = part([...base3, loft({ startNormal: 1 })]);
  parts.loftStartNormal2 = part([...base3, loft({ startNormal: 2 })]);
  parts.loftBothNormal = part([...base3, loft({ startNormal: 1, endNormal: 1 })]);
  parts.loftConn = part([...base3, loft({ connectors: [null, 0.25] })]);
  const p4 = { type: 'plane', id: 'F4', mode: 'offset', base: STD('top'), offset: 50 };
  const sq8 = { type: 'sketch', id: 'F6', plane: { kind: 'feature', id: 'F4' }, points: [pt('p1', -8, -8), pt('p2', 8, -8), pt('p3', 8, 8), pt('p4', -8, 8)],
    entities: [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p4'), line('e4', 'p4', 'p1')], constraints: [] };
  const loft3 = (o = {}) => ({ type: 'loft', id: 'F7', op: 'add', sketches: ['F1', 'F3', 'F6'], ...o });
  parts.loft3StartNormal = part([...base3, p4, sq8, loft3({ startNormal: 1 })]);
  parts.loft3EndNormal = part([...base3, p4, sq8, loft3({ endNormal: 1.5 })]);
}

// fillet profiles (conic / chord / curvature = Euler spiral - arc - spiral, blend/g2.rs) and chamfer legs
// (`distances`, `angle`): oracle volumes within 1e-6 (curvature 1e-6: the oracle fits a spline to the spiral).
{
  const E = (a, b, at) => ({ faces: [a, b], at });
  const e1 = E('F2.C1', 'F2.S.e1', [20, 10, 0]), e3 = E('F2.C1', 'F2.S.e3', [20, 10, -20]);
  const boxF = (f) => part([sketch('top', ...rect(0, 0, 40, 20)), extrude(), { id: 'F3', ...f }]);
  const fl = (o) => ({ type: 'fillet', radius: 2, edges: [e1], ...o });
  const ch = (o) => ({ type: 'chamfer', chamferType: 'equal', distance: 2, edges: [e1], ...o });
  parts.filletConic = boxF(fl({ profile: 'conic', rho: 0.5 }));
  parts.filletConic3 = boxF(fl({ profile: 'conic', rho: 0.3 }));
  parts.filletConicTwo = boxF(fl({ profile: 'conic', rho: 0.6, edges: [e1, e3] }));
  parts.filletChord = boxF(fl({ profile: 'chord' }));
  parts.filletChordTwo = boxF(fl({ profile: 'chord', radius: 3, edges: [e1, e3] }));
  parts.filletCurv = boxF(fl({ profile: 'curvature', shape: 0.5 }));
  parts.filletCurvTwo = boxF(fl({ profile: 'curvature', shape: 0.5, edges: [e1, e3] }));
  parts.filletCurvShape1 = boxF(fl({ profile: 'curvature', shape: 1 }));
  parts.chamferDist = boxF(ch({ chamferType: 'distances', distance2: 3 }));
  parts.chamferDistRev = boxF(ch({ chamferType: 'distances', distance2: 3, edges: [E('F2.S.e1', 'F2.C1', [20, 10, 0])] }));
  parts.chamferDistTwo = boxF(ch({ chamferType: 'distances', distance2: 3, edges: [e1, e3] }));
  parts.chamferAngle = boxF(ch({ chamferType: 'angle', angle: 30 }));
  parts.chamferAngleRev = boxF(ch({ chamferType: 'angle', angle: 30, edges: [E('F2.S.e1', 'F2.C1', [20, 10, 0])] }));
  // variable radius: radii[0] at the end of the edge nearer the pick
  const nearStart = E('F2.C1', 'F2.S.e1', [4, 10, 0]), nearEnd = E('F2.C1', 'F2.S.e1', [36, 10, 0]);
  const vr = (o) => boxF(fl({ filletType: 'variable', radii: [1, 3], edges: [nearStart], ...o }));
  parts.varNearStart = vr({});
  parts.varNearEnd = vr({ edges: [nearEnd] });
  parts.varBig = vr({ radii: [1, 9] });
  parts.varChord = vr({ profile: 'chord' });
  parts.varConic = vr({ profile: 'conic', rhos: [0.4, 0.6] });
  parts.varConicRho = vr({ profile: 'conic', rho: 0.5 });
  parts.varCurv = vr({ profile: 'curvature', shape: 0.5 });
  parts.varTwoEdges = vr({ edges: [nearStart, E('F2.C1', 'F2.S.e3', [20, 10, -20])] });
  parts.varEdgeE2 = vr({ edges: [E('F2.C1', 'F2.S.e2', [40, 10, -3])] });
  parts.varBottom = vr({ edges: [E('F2.C0', 'F2.S.e1', [4, 0, 0])] });
  const cyl = [sketch('top', [pt('c', 0, 0)], [{ type: 'circle', id: 'e1', c: 'c', r: 10 }]), extrude()];
  const rim = (o) => part([...cyl, { type: 'fillet', id: 'F3', radius: 2, edges: [E('F2.C1', 'F2.S.e1', [10, 10, 0])], ...o }]);
  parts.filletCylConic = rim({ profile: 'conic', rho: 0.5 });
  parts.filletCylCurv = rim({ profile: 'curvature', shape: 0.5 });
  parts.filletCylChord = rim({ profile: 'chord' });
}

// sweep guides (ck-ops guide.rs): circle r=3 swept 20 along a line while a guide scales / turns / stretches it
// (volumes match the oracle to 1e-6: cone frustum, ellipse sections, ...).
{
  const front = (id, points, entities) => ({ type: 'sketch', id, plane: STD('front'), points, entities, constraints: [] });
  const gl = (id, a, b) => front(id, [pt('g1', ...a), pt('g2', ...b)], [line('e1', 'g1', 'g2')]);
  const path = front('F2', [pt('a', 0, 0), pt('b', 0, 20)], [line('e1', 'a', 'b')]);
  const circ = { type: 'sketch', id: 'F3', plane: STD('top'), points: [pt('c', 0, 0)], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 3 }], constraints: [] };
  const sq = { type: 'sketch', id: 'F3', plane: STD('top'), points: [pt('p1', -3, -3), pt('p2', 3, -3), pt('p3', 3, 3), pt('p4', -3, 3)], entities: [line('e1', 'p1', 'p2'), line('e2', 'p2', 'p3'), line('e3', 'p3', 'p4'), line('e4', 'p4', 'p1')], constraints: [] };
  const cr = (sk) => ({ kind: 'sketchCurve', sketch: sk, entity: 'e1' });
  const sw = (o) => ({ type: 'sweep', id: 'F5', op: 'add', sketch: 'F3', path: { kind: 'sketch', id: 'F2' }, ...o });
  const spl = front('F4', [pt('s0', 3, 0), pt('s1', 5.5, 10), pt('s2', 4, 20)], [{ type: 'spline', id: 'e1', pts: ['s0', 's1', 's2'] }]);
  parts.sweepGuideScale = part([path, circ, gl('F4', [3, 0], [6, 20]), sw({ guides: [cr('F4')], guideMode: 'scale' })]);
  parts.sweepGuideTurn = part([path, circ, gl('F4', [3, 0], [6, 20]), sw({ guides: [cr('F4')], guideMode: 'turn' })]);
  parts.sweepGuideShrink = part([path, circ, gl('F4', [3, 0], [1.5, 20]), sw({ guides: [cr('F4')] })]);
  parts.sweepGuideCurved = part([path, circ, spl, sw({ guides: [cr('F4')], guideMode: 'scale' })]);
  parts.sweepGuideStretch = part([path, circ, gl('F4', [3, 0], [6, 20]), gl('F6', [-3, 0], [-4, 20]), sw({ guides: [cr('F4'), cr('F6')], guideMode: 'stretch' })]);
  parts.sweepGuide2Scale = part([path, circ, gl('F4', [3, 0], [6, 20]), gl('F6', [-3, 0], [-5, 20]), sw({ guides: [cr('F4'), cr('F6')], guideMode: 'scale' })]);
  parts.sweepGuideSquare = part([path, sq, gl('F4', [3, 0], [6, 20]), sw({ guides: [cr('F4')], guideMode: 'scale' })]);
}

// editCurve (ck-ops curve_edit): the rebuilt curve goes to `curves[]` / `markers[]`, the solid is untouched
// (the fit is least squares, the oracle's is a different one: deviations differ, see docs/ck-ops-stubs.md).
{
  const spl = { type: 'sketch', id: 'F1', plane: STD('front'), points: [pt('c1', 0, 0), pt('c2', 5, 8), pt('c3', 10, 2), pt('c4', 15, 9), pt('c5', 20, 0)],
    entities: [{ type: 'spline', id: 'e1', pts: ['c1', 'c2', 'c3', 'c4', 'c5'] }], constraints: [] };
  const poly = { type: 'sketch', id: 'F1', plane: STD('front'), points: [pt('q0', 0, 0), pt('q1', 10, 0), pt('q2', 10, 10)],
    entities: [line('l1', 'q0', 'q1'), line('l2', 'q1', 'q2')], constraints: [] };
  const cref = (e) => ({ kind: 'sketchCurve', sketch: 'F1', entity: e });
  const ec = (o = {}) => ({ type: 'editCurve', id: 'F2', curves: [cref('e1')], method: 'count', degree: 3, count: 6, tolerance: 0.01, keepEnds: true, keepTangents: false, keepCorners: false, entries: [], planar: false, plane: null, ...o });
  parts.editCurveSpline = part([spl, ec()]);
  parts.editCurveMove = part([spl, ec({ entries: [{ cv: 2, of: 6, frame: 'world', dx: 0, dy: 3, dz: 0 }] })]);
  parts.editCurveTol = part([spl, ec({ method: 'tolerance', tolerance: 0.05 })]);
  parts.editCurveCorner = part([poly, ec({ curves: [cref('l1'), cref('l2')], keepCorners: true, count: 8 })]);
  const [Pb, Eb] = rect(0, 0, 40, 20);
  parts.editCurveEdge = part([sketch('top', Pb, Eb), extrude(), ec({ id: 'F3', curves: [{ kind: 'edge', faces: ['F2.C1', 'F2.S.e1'], at: [20, 10, 0] }] })]);
}

// pipeCut (ck-ops pipecut): a ball of `radius` rolled along edges / face rims removes
// what it touches; the solid becomes a surface body (docs/ck-ops-stubs.md). Exploration
// cases with error paths live in tools/stub_cases.mjs (node tools/cmp_stubs.mjs pipeCut).
{
  const E = (a, b, at) => ({ faces: [a, b], at });
  const e1 = E('F2.C1', 'F2.S.e1', [20, 10, 0]), e2 = E('F2.C1', 'F2.S.e2', [40, 10, -10]);
  const e3 = E('F2.C1', 'F2.S.e3', [20, 10, -20]), e4 = E('F2.C1', 'F2.S.e4', [0, 10, -10]);
  const pc = (o) => ({ type: 'pipeCut', id: 'F3', radius: 2, edges: [], faces: [], ...o });
  const withBox = (f) => part([sketch('top', ...rect(0, 0, 40, 20)), extrude(), f]);
  parts.pipeCutEdge = withBox(pc({ edges: [e1] }));
  parts.pipeCutEdge2 = withBox(pc({ edges: [e1, e3] }));
  parts.pipeCutRing = withBox(pc({ edges: [e1, e2, e3, e4] }));
  parts.pipeCutFace = withBox(pc({ faces: ['F2.C1'] }));
  parts.pipeCutBig = withBox(pc({ radius: 8, edges: [e1] }));
  parts.pipeCut12 = withBox(pc({ radius: 12, edges: [e1] }));
  const cyl = [sketch('top', [pt('c', 0, 0)], [{ type: 'circle', id: 'e1', c: 'c', r: 10 }]), extrude()];
  parts.pipeCutCyl = part([...cyl, pc({ edges: [E('F2.C1', 'F2.S.e1', [10, 10, 0])] })]);
  parts.pipeCutCylBot = part([...cyl, pc({ radius: 3, edges: [E('F2.C0', 'F2.S.e1', [10, 0, 0])] })]);
  const lSheet = [{ type: 'sketch', id: 'F1', plane: STD('front'), points: [pt('a', 0, 0), pt('b', 20, 0), pt('c', 20, 20)], entities: [line('e1', 'a', 'b'), line('e2', 'b', 'c')], constraints: [] },
    { type: 'surfaceExtrude', id: 'F2', sketch: 'F1', end: 'blind', depth: 10, flip: false }];
  parts.pipeCutSheet = part([...lSheet, pc({ edges: [E('F2.S.e1', 'F2.S.e2', [20, 0, 5])] })]);
}

// simplify stage of the boolean: coplanar / cocylindrical / cocone faces left by an edit are
// merged (docs/ck-ops-stubs.md). simpCylStack used to return volume 2094 (the merged
// cylinder lost its seam) with no error.
{
  const circSk = (id, plane, cx, cy, r) => ({ type: 'sketch', id, plane, points: [pt('c', cx, cy)], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
  const ex = (id, sk, o = {}) => ({ type: 'extrude', id, op: 'add', sketch: sk, end: 'blind', depth: 10, ...o });
  const FACE = (id) => ({ kind: 'face', id });
  parts.simpCylStack = part([circSk('F1', STD('top'), 5, 3, 10), ex('F2', 'F1'), circSk('F3', FACE('F2.C1'), 5, 3, 10), ex('F4', 'F3')]);
  parts.simpCylStackCut = part([circSk('F1', STD('top'), 5, 3, 10), ex('F2', 'F1', { depth: 20 }), circSk('F3', FACE('F2.C1'), 5, 3, 10), ex('F4', 'F3', { op: 'cut' })]);
  parts.simpCylPlug = part([circSk('F1', STD('top'), 0, 0, 10), ex('F2', 'F1'), circSk('F3', STD('top'), 0, 0, 3), ex('F4', 'F3', { op: 'cut', end: 'throughBoth' }),
    circSk('F5', STD('top'), 0, 0, 3), ex('F6', 'F5')]);
  parts.simpCylTriple = part([circSk('F1', STD('top'), 0, 0, 10), ex('F2', 'F1'), circSk('F3', FACE('F2.C1'), 0, 0, 10), ex('F4', 'F3', { depth: 5 }),
    circSk('F5', FACE('F4.C1'), 0, 0, 10), ex('F6', 'F5', { depth: 5 })]);
  const coneSk = (id, r0, y0, r1, y1) => ({ type: 'sketch', id, plane: STD('front'), points: [pt('a', 0, y0), pt('b', r0, y0), pt('c', r1, y1), pt('d', 0, y1), pt('x1', 0, 0), pt('x2', 0, 30)],
    entities: [line('e1', 'a', 'b'), line('e2', 'b', 'c'), line('e3', 'c', 'd'), line('e4', 'd', 'a'), line('ax', 'x1', 'x2', { construction: true })], constraints: [] });
  const rev = (id, sk) => ({ type: 'revolve', id, op: 'add', sketch: sk, axis: 'ax', angle: 360 });
  parts.simpConeStack = part([coneSk('F1', 10, 0, 6, 10), rev('F2', 'F1'), coneSk('F3', 6, 10, 2, 20), rev('F4', 'F3')]);
}

const m = (msg) => JSON.stringify({ id: 1, msg });
/// Follow-up requests on each part's state (compared byte-for-byte).
// nested parts of the flange coupling assembly's insertParts (tools/cases/*.ckpart);
// name@N = the first N features
{
  const dir = new URL('./cases/', import.meta.url);
  const load = (n) => JSON.parse(readFileSync(new URL(`${n}.ckpart`, dir), 'utf8'));
  const cut = (n, ks) => { const p = load(n); for (const k of ks) parts[`${n}@${k}`] = { ...p, features: p.features.slice(0, k) }; };
  cut('flangeHalf', [8, 9, 10, 11]);
  cut('flangeBolt', [6, 8, 12, 13]);
  cut('flangeNut', [4, 6, 11, 12]);
}

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
