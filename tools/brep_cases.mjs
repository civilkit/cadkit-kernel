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
