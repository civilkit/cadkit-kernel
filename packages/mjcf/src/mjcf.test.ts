import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { axisAngle, compose, fromQuat, quat, type Pose, type V3 } from './math.js';
import { parse, serialize, el, findAll, nums } from './xml.js';
import { readStl, writeStl } from './stl.js';
import { fitPrimitive, isConvex, meshVolume } from './geom.js';
import { bodyPoses, toMjcf } from './mjcf.js';
import { componentOfFace, inferJoints, safeName, type Geo } from './analyze.js';
import { readSimFile } from './options.js';
import { motionStudy, posesAt, readTrajectory, sampleAt, withMotionStudy } from './trajectory.js';
import type { MjPart, MjPlan } from './plan.js';

const close = (a: number, b: number, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} != ${b}`);
const closeV = (a: ArrayLike<number>, b: ArrayLike<number>, tol = 1e-9, msg = '') => Array.from(a).forEach((x, i) => close(x, b[i], tol, `${msg}[${i}]`));

/** Box mesh (12 triangles, outward winding) from lo to hi. */
function boxMesh(lo: V3, hi: V3): Float32Array {
  const c = (i: number): V3 => [i & 1 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 4 ? hi[2] : lo[2]];
  const quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
  const out: number[] = [];
  for (const [a, b, cc, d] of quads) out.push(...c(a), ...c(b), ...c(cc), ...c(a), ...c(cc), ...c(d));
  return Float32Array.from(out);
}

function part(key: string, name: string, lo: V3, hi: V3, density = 1): MjPart {
  const e = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
  const V = e[0] * e[1] * e[2];
  const mass = V * density * 1e-6;
  const I = (a: number, b: number) => (mass * (a * a + b * b)) / 12 * 1e-6;
  return {
    key, name, material: 'x', density, volume: V, mass, com: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2],
    inertia: [[I(e[1], e[2]), 0, 0], [0, I(e[0], e[2]), 0], [0, 0, I(e[0], e[1])]],
    positions: boxMesh(lo, hi), lumpOfTri: new Int32Array(12), lumps: 1, faceKinds: ['plane', 'plane', 'plane', 'plane', 'plane', 'plane'],
  };
}

/** A fixed base, a door hinged on the base's z axis at x=10 (turned 30 degrees about z), a slider on the door along its x. */
function plan(): MjPlan {
  const turn: Pose = { ...axisAngle([0, 0, 1], Math.PI / 6), o: [10, 0, 0] };
  const slid: Pose = compose(turn, { o: [20, 0, 5], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] });
  return {
    model: 'm',
    parts: { P0: part('P0', 'base', [0, 0, 0], [100, 50, 10], 7.85), P1: part('P1', 'door', [0, 0, 0], [40, 4, 30], 2.7), P2: part('P2', 'slider', [-5, -5, 0], [5, 5, 5], 1) },
    bodies: [
      { id: 'F1', name: 'Base', parent: null, place: { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] }, part: 'P0', joints: [], fixed: true },
      { id: 'F2', name: 'Door', parent: 'F1', place: turn, part: 'P1', joints: [{ name: 'Pivot', type: 'hinge', axis: [0, 0, 1], pos: [10, 0, 0], range: [-1, 2], mate: 'F5' }], fixed: false },
      { id: 'F3', name: 'Slider', parent: 'F2', place: slid, part: 'P2', joints: [{ name: 'Rail', type: 'slide', axis: turn.x, pos: slid.o, range: [-10, 15], mate: 'F6' }], fixed: false },
    ],
    couplings: [{ name: 'c', joint1: 'Rail', joint2: 'Pivot', c0: 0, c1: 3, mate: 'F7' }],
    warnings: [],
  };
}

describe('math, xml, stl', () => {
  it('quaternion round trip and composition', () => {
    const p = compose(axisAngle([1, 2, 3], 0.7), axisAngle([0, 1, 0], -2.1));
    const q = quat(p);
    const r = fromQuat(q);
    for (const k of ['x', 'y', 'z'] as const) closeV(r[k], p[k], 1e-12);
    assert.ok(q[0] >= 0);
  });
  it('xml escapes and parses back', () => {
    const x = serialize(el('a', { n: 'x<"&>y' }, [el('b', { v: '1 2 3' }), el('c')]));
    const t = parse(x);
    assert.equal(t.attrs.n, 'x<"&>y');
    assert.deepEqual(nums(findAll(t, 'b')[0].attrs.v), [1, 2, 3]);
    assert.throws(() => parse('<a><b></a>'));
  });
  it('binary stl round trip', () => {
    const m = boxMesh([0, 0, 0], [1, 2, 3]);
    const back = readStl(writeStl(m));
    closeV(back, m, 1e-6);
    close(meshVolume(back), 6, 1e-5);
  });
});

describe('geometry', () => {
  it('convexity: a box is convex, an L made of two boxes is not', () => {
    assert.equal(isConvex(boxMesh([0, 0, 0], [10, 10, 10])), true);
    const l = new Float32Array([...boxMesh([0, 0, 0], [30, 10, 10]), ...boxMesh([0, 10, 0], [10, 30, 10])]);
    assert.equal(isConvex(l), false);
  });
  it('fits a box from exact moments, refuses wrong face kinds', () => {
    const V = 10 * 20 * 30;
    const principal = [V * (20 * 20 + 30 * 30) / 12, V * (10 * 10 + 30 * 30) / 12, V * (10 * 10 + 20 * 20) / 12];
    const f = { faceKinds: Array(6).fill('plane'), lumps: 1, positions: boxMesh([0, 0, 0], [10, 20, 30]), volume: V, com: [5, 10, 15] as V3, principal, axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] };
    const b = fitPrimitive(f)!;
    assert.equal(b.type, 'box');
    closeV(b.size, [5, 10, 15], 1e-9);
    closeV(b.pose.o, [5, 10, 15], 1e-9);
    assert.equal(fitPrimitive({ ...f, faceKinds: [...f.faceKinds, 'cylinder'] }), null);
    assert.equal(fitPrimitive({ ...f, volume: V * 0.9 }), null);
  });
});

describe('toMjcf', () => {
  it('nests bodies with poses relative to the parent, joints in the body frame, metres, explicit inertials', () => {
    const p = plan();
    const r = toMjcf(p);
    const x = parse(r.xml);
    const bodies = findAll(x, 'body');
    assert.deepEqual(bodies.map((b) => b.attrs.name), ['Base', 'Door', 'Slider']);
    const door = bodies[1];
    closeV(nums(door.attrs.pos), [0.01, 0, 0], 1e-12);
    closeV(nums(door.attrs.quat), [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)], 1e-8);
    const slider = bodies[2];
    closeV(nums(slider.attrs.pos), [0.02, 0, 0.005], 1e-12);
    const jd = findAll(door, 'joint')[0];
    closeV(nums(jd.attrs.axis), [0, 0, 1], 1e-12);
    closeV(nums(jd.attrs.pos), [0, 0, 0], 1e-12);
    closeV(nums(jd.attrs.range), [-1, 2]);
    const js = findAll(slider, 'joint')[0];
    // the slide runs along the door's x: (1, 0, 0) in the slider's frame, range in metres
    closeV(nums(js.attrs.axis), [1, 0, 0], 1e-9);
    closeV(nums(js.attrs.range), [-0.01, 0.015]);
    const inertial = door.children.find((c) => c.tag === 'inertial')!;
    close(Number(inertial.attrs.mass), p.parts.P1.mass, 1e-9);
    closeV(nums(inertial.attrs.pos), [0.02, 0.002, 0.015], 1e-12);
    const full = nums(inertial.attrs.fullinertia);
    close(full[0], p.parts.P1.inertia[0][0], 1e-15);
    // coupling: slide (m) = 3 mm/rad * 1e-3
    const eq = findAll(x, 'equality')[0].children[0];
    assert.equal(eq.attrs.joint1, 'Rail');
    closeV(nums(eq.attrs.polycoef), [0, 0.003, 0, 0, 0], 1e-12);
    assert.ok(r.files['meshes/door.stl'] && r.files['meshes/base.stl']);
    assert.equal(r.meta.joints.Pivot.mate, 'F5');
    assert.equal(r.meta.bodies.F3.parent, 'F2');
  });
  it('collision modes, visual-only parts, sim block and includes', () => {
    const p = plan();
    p.parts.P2.primitive = { type: 'box', size: [5, 5, 2.5], pose: { o: [0, 0, 2.5], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] } };
    const r = toMjcf(p, {
      collision: 'solids', collisionByPart: { slider: 'primitive' }, visualOnly: ['Door'],
      sim: {
        option: { timestep: 0.001, gravity: [0, 0, -9.81] }, contact: { default: { friction: [1, 0.005, 0.0001] }, bodies: { Slider: { condim: 4 } }, exclude: [['Base', 'Slider']] },
        joints: { '*': { damping: 0.1 }, Pivot: { armature: 0.01 } }, actuators: [{ type: 'position', name: 'drive', joint: 'Pivot', kp: 5, ctrlrange: [-1, 1] }],
        sensors: [{ type: 'jointpos', name: 'q', joint: 'Rail' }], worldbody: '<body name="ball" pos="0 0 1"><freejoint/><geom type="sphere" size="0.02"/></body>', include: ['extra.xml'],
      },
    });
    const x = parse(r.xml);
    const geoms = findAll(x, 'geom');
    assert.ok(!geoms.some((g) => g.attrs.name?.startsWith('Door_collision')), 'visual only');
    const box = geoms.find((g) => g.attrs.name === 'Slider_collision')!;
    assert.equal(box.attrs.type, 'box');
    closeV(nums(box.attrs.size), [0.005, 0.005, 0.0025], 1e-12);
    assert.equal(box.attrs.condim, '4');
    assert.ok(geoms.some((g) => g.attrs.name === 'Base_collision_s0'));
    assert.equal(r.hints.length, 1);
    assert.equal(r.hints[0].convex, true);
    assert.equal(findAll(x, 'option')[0].attrs.timestep, '0.001');
    const coll = findAll(x, 'default').find((d) => d.attrs.class === 'collision')!.children[0];
    assert.equal(coll.attrs.friction, '1 0.005 0.0001');
    const pivot = findAll(x, 'joint').find((j) => j.attrs.name === 'Pivot')!;
    assert.equal(pivot.attrs.damping, '0.1');
    assert.equal(pivot.attrs.armature, '0.01');
    assert.equal(findAll(x, 'position')[0].attrs.kp, '5');
    assert.equal(findAll(x, 'jointpos')[0].attrs.joint, 'Rail');
    assert.ok(findAll(x, 'body').some((b) => b.attrs.name === 'ball'));
    assert.equal(findAll(x, 'include')[0].attrs.file, 'extra.xml');
    assert.equal(findAll(x, 'exclude')[0].attrs.body2, 'Slider');
  });
  it('up y turns the world so CAD +y points up', () => {
    const p = plan();
    const x = parse(toMjcf(p, { up: 'y' }).xml);
    const q = nums(findAll(x, 'body')[0].attrs.quat);
    const r = fromQuat(q);
    closeV(r.y, [0, 0, 1], 1e-12);
  });
  it('unknown sim keys are refused', () => {
    assert.throws(() => readSimFile('{"actuator": []}'), /unknown key "actuator"/);
    assert.deepEqual(readSimFile('{"option": {"timestep": 0.002}}'), { option: { timestep: 0.002 } });
  });
});

describe('joint inference', () => {
  const g = new Map<string, Geo>([
    ['pinA', { kind: 'cylinder', o: [0, 0, 0], d: [0, 0, 1] }], ['pinB', { kind: 'cylinder', o: [5, 5, 0], d: [0, 0, -1] }],
    ['floorA', { kind: 'plane', o: [0, 0, 0], n: [0, 0, -1] }], ['floorB', { kind: 'plane', o: [0, 0, 0], n: [0, 0, 1] }],
    ['wallA', { kind: 'plane', o: [0, 0, 0], n: [0, 1, 0] }], ['wallB', { kind: 'plane', o: [0, 0, 0], n: [0, -1, 0] }],
    ['ballA', { kind: 'sphere', c: [1, 2, 3], r: 4 }], ['ballB', { kind: 'sphere', c: [1, 2, 3], r: 4 }],
  ]);
  let k = 0;
  const name = (s: string) => `${s}${k++}`;
  const m = (kind: string, a: string, b: string, extra: Record<string, unknown> = {}) => ({ id: `M${k}`, type: 'mate', kind, a, b, ...extra });
  it('hinge with a stop -> one hinge on the target axis through the axis point', () => {
    const w: string[] = [];
    const j = inferJoints('b', [m('hinge', 'pinA', 'pinB', { a2: 'floorA', b2: 'floorB' })], g, [9, 9, 9], name, w);
    assert.deepEqual(j.map((x) => x.type), ['hinge']);
    closeV(j[0].axis, [0, 0, -1]);
    closeV(j[0].pos, [5, 5, 9]);
    assert.deepEqual(w, []);
  });
  it('concentric -> cylindrical (slide + hinge on one axis); + a parallel plane -> slide', () => {
    const j = inferJoints('b', [m('concentric', 'pinA', 'pinB')], g, [0, 0, 0], name, []);
    assert.deepEqual(j.map((x) => x.type), ['slide', 'hinge']);
    const j2 = inferJoints('b', [m('concentric', 'pinA', 'pinB'), m('parallel', 'wallA', 'wallB')], g, [0, 0, 0], name, []);
    assert.deepEqual(j2.map((x) => x.type), ['slide']);
  });
  it('two coincident planes -> slide along their intersection, one -> planar (2 slides + hinge); sphere coincident -> ball; lock -> weld', () => {
    const j = inferJoints('b', [m('coincident', 'floorA', 'floorB'), m('coincident', 'wallA', 'wallB')], g, [0, 0, 0], name, []);
    assert.deepEqual(j.map((x) => x.type), ['slide']);
    closeV(j[0].axis, [1, 0, 0]);
    assert.deepEqual(inferJoints('b', [m('coincident', 'floorA', 'floorB')], g, [0, 0, 0], name, []).map((x) => x.type), ['slide', 'slide', 'hinge']);
    const ball = inferJoints('b', [m('coincident', 'ballA', 'ballB')], g, [0, 0, 0], name, []);
    assert.deepEqual(ball.map((x) => x.type), ['ball']);
    closeV(ball[0].pos, [1, 2, 3]);
    assert.deepEqual(inferJoints('b', [m('lock', 'floorA', 'floorB')], g, [0, 0, 0], name, []), []);
  });
  it('face names map to components; names are sanitised', () => {
    const ins = new Set(['F2', 'F3']);
    assert.equal(componentOfFace('F4.S.e1@F2.1', ins), 'F2');
    assert.equal(componentOfFace('F8.S.h1@F9.1@F3.1', ins), 'F3');
    assert.equal(componentOfFace('F4.C0', ins), null);
    assert.equal(safeName(' Door / left #2 '), 'Door_left_2');
  });
});

describe('trajectory import', () => {
  it('reads CSV and the JSON shapes, converts rad -> deg and m -> mm, reports unknown columns', () => {
    const csv = 'time,Pivot,qpos.Rail,mystery\n0,0,0,1\n0.5,0.5,0.01,1\n1,1,0.02,1\n';
    const t = readTrajectory(csv);
    assert.deepEqual(t.time, [0, 0.5, 1]);
    const { study, unknown } = motionStudy(plan(), t, { name: 'run 1' });
    assert.deepEqual(unknown, ['mystery']);
    const piv = study.tracks.find((x) => x.joint === 'Pivot')!;
    assert.equal(piv.unit, 'deg');
    close(piv.keys[2][1], 180 / Math.PI, 1e-9);
    assert.equal(piv.mate, 'F5');
    const rail = study.tracks.find((x) => x.joint === 'Rail')!;
    close(rail.keys[1][1], 10, 1e-9);
    assert.equal(rail.component, 'F3');
    for (const j of [
      '{"time":[0,0.5,1],"qpos":{"Pivot":[0,0.5,1],"Rail":[0,0.01,0.02]}}',
      '{"time":[0,0.5,1],"joints":["Pivot","Rail"],"qpos":[[0,0],[0.5,0.01],[1,0.02]]}',
      '[{"time":0,"qpos":{"Pivot":0,"Rail":0}},{"t":0.5,"Pivot":0.5,"Rail":0.01},{"time":1,"qpos":{"Pivot":1,"Rail":0.02}}]',
    ]) {
      const s = motionStudy(plan(), readTrajectory(j)).study;
      close(sampleAt(s, 0.75).Pivot, 0.75 * 180 / Math.PI, 1e-9);
      close(sampleAt(s, 0.75).Rail, 15, 1e-9);
    }
    // a mate id works as a column name when it maps to one joint
    assert.deepEqual(motionStudy(plan(), readTrajectory('t,F5\n0,1\n')).unknown, []);
    assert.throws(() => readTrajectory('time,a\n1,0\n0,0\n'), /time goes back/);
  });
  it('stores the study in the document and gives the component poses MuJoCo would', () => {
    const p = plan();
    const { study } = motionStudy(p, readTrajectory('time,Pivot,Rail\n0,0,0\n1,0.4,0.012\n'));
    const doc = withMotionStudy({ features: [], motion: { studies: [{ ...study, tracks: [] }] } }, study);
    assert.equal(doc.motion.studies.length, 1);
    assert.equal(doc.motion.studies[0].tracks.length, 2);
    const poses = posesAt(p, study, 1);
    // door: turned by 30 deg + 0.4 rad about z through (10, 0, 0)
    const door = poses.get('F2')!;
    const want = axisAngle([0, 0, 1], Math.PI / 6 + 0.4);
    closeV(door.x, want.x, 1e-9);
    closeV(door.o, [10, 0, 0], 1e-9);
    // slider: 12 mm along the door's x, carried by the door
    const sl = poses.get('F3')!;
    const exp = compose(door, { o: [32, 0, 5], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] });
    closeV(sl.o, exp.o, 1e-9);
    const p0 = bodyPoses(p, {});
    closeV(p0.get('F3')!.o, p.bodies[2].place.o, 1e-9);
  });
});
