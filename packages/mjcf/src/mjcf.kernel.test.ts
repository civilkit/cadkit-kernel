// Against the kernel wasm (skipped when none is built): export the slider + hinge fixture and check poses, axes, ranges and masses
// against the CAD. With MUJOCO_PYTHON set (a python that has the `mujoco` package) the model is also loaded and stepped in MuJoCo.
import { after, before, describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { existsSync, mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createWorkerKernel, defaultWasmPath, type WorkerKernel } from '../../kernel-client/src/worker.js';
import { meshFromBody, bboxOf } from '../../render/src/mesh.js';
import { apply, clean, quat } from './math.js';
import { findAll, nums, parse } from './xml.js';
import { readStl } from './stl.js';
import { sliderHingeAssembly } from './fixtures.js';
import { exportMjcf, writeMjcf, motionStudy, posesAt, readTrajectory } from './index.js';
import type { MjPlan } from './plan.js';

const close = (a: number, b: number, tol: number, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
const closeV = (a: readonly number[], b: readonly number[], tol: number, msg = '') => a.forEach((x, i) => close(x, b[i], tol, `${msg}[${i}]`));
const sameRot = (q1: number[], q2: number[], tol: number, msg: string) => close(Math.abs(q1.reduce((s, x, i) => s + x * q2[i], 0)), 1, tol, msg);

const PY = process.env.MUJOCO_PYTHON;
const CHECK = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../../tools/mjcf_check.py');
type MjOut = { bodies: Record<string, { pos: number[]; quat: number[]; mass: number }>; joints: Record<string, { type: string; axis: number[]; anchor: number[]; range: number[]; limited: boolean }>; qpos: Record<string, number | number[]> };
const mujoco = (model: string, args: string[] = []): MjOut => JSON.parse(execFileSync(PY!, ['-I', CHECK, model, ...args], { encoding: 'utf8' })) as MjOut;

describe('mjcf export against the kernel', { skip: !existsSync(defaultWasmPath()) && 'no wasm built' }, () => {
  let k: WorkerKernel;
  let dir: string;
  before(async () => { k = await createWorkerKernel(); dir = mkdtempSync(path.join(tmpdir(), 'mjcf-')); });
  after(async () => { await k?.close(); rmSync(dir, { recursive: true, force: true }); });

  it('slider + hinge: tree, poses, axes, ranges and masses match the CAD', async () => {
    const doc = sliderHingeAssembly();
    const r = await exportMjcf(k, doc, { collision: 'solids' });
    assert.deepEqual(r.plan.warnings, []);
    const reg = (await k.request({ type: 'regen', tag: 'ref', part: doc, opts: { dofReport: true } })).result as { components: Array<{ id: string; place: never }>; body: never };
    const place = new Map(reg.components.map((c) => [c.id, clean(c.place)]));
    const by = new Map(r.plan.bodies.map((b) => [b.name, b]));
    assert.deepEqual([...by.keys()], ['Base', 'Carriage', 'Arm', 'Ball']);
    assert.equal(by.get('Carriage')!.parent, 'F1');
    assert.equal(by.get('Arm')!.parent, 'F2');
    for (const b of r.plan.bodies) closeV(b.place.o, place.get(b.id)!.o, 1e-9, b.name);
    const slide = by.get('Carriage')!.joints;
    assert.deepEqual(slide.map((j) => [j.type, j.name, j.mate]), [['slide', 'Carriage_on_rod', 'F5']]);
    assert.ok(Math.abs(slide[0].axis[0]) > 1 - 1e-9, 'slides along x');
    // limits mate Travel: carriage x- face 35 mm from the plate end, allowed -170..-20 (signed, plate normal -x)
    const sx = Math.sign(slide[0].axis[0]);
    closeV([...slide[0].range!].sort((a, b) => a - b), [sx > 0 ? -15 : -135, sx > 0 ? 135 : 15], 1e-6, 'slide range');
    const hinge = by.get('Arm')!.joints;
    assert.deepEqual(hinge.map((j) => [j.type, j.name, j.mate, j.limit]), [['hinge', 'Arm_pivot', 'F8', 'F9']]);
    assert.ok(Math.abs(hinge[0].axis[1]) > 1 - 1e-9, 'hinge about y');
    closeV(hinge[0].pos.filter((_, i) => i !== 1), [50, 28], 1e-6, 'hinge on the pin axis');
    closeV(hinge[0].range!, [-Math.PI / 2, Math.PI / 2], 1e-6, 'swing 0..90 unsigned -> +-90');
    assert.deepEqual(by.get('Ball')!.joints.map((j) => j.type), ['free']);
    // masses: massProps of the part alone x density
    for (const p of Object.values(r.plan.parts)) assert.ok(p.mass > 0, p.name);
    const carriage = r.plan.parts[by.get('Carriage')!.part];
    close(carriage.mass, carriage.volume * 2.7e-6, 1e-12, 'aluminium');
    close(r.plan.parts[by.get('Ball')!.part].volume, (4 / 3) * Math.PI * 1000, 1e-6, 'sphere volume');
    // the part-local mesh placed by the component placement covers the component's faces in the assembly (to the tessellation)
    const all = meshFromBody(reg.body);
    for (const b of r.plan.bodies) {
      const tris: number[] = [];
      for (let t = 0; t < all.triFace.length; t++) if (all.faceIds[all.triFace[t]]?.endsWith(`@${b.id}.1`)) tris.push(...all.positions.subarray(t * 9, t * 9 + 9));
      const want = bboxOf(Float32Array.from(tris));
      const p = r.plan.parts[b.part].positions;
      const moved: number[] = [];
      for (let i = 0; i < p.length; i += 3) moved.push(...apply(b.place, [p[i], p[i + 1], p[i + 2]]));
      const got = bboxOf(Float32Array.from(moved));
      closeV(got.min, want.min, 0.2, `${b.name} min`);
      closeV(got.max, want.max, 0.2, `${b.name} max`);
    }
    // XML: relative poses in metres, meshes in metres
    const x = parse(r.xml);
    const arm = findAll(x, 'body').find((e) => e.attrs.name === 'Arm')!;
    closeV(nums(arm.attrs.pos), [0, 0.014, 0.028], 1e-9, 'arm relative to the carriage');
    const stl = readStl(r.files['meshes/Arm.stl']);
    const bb = bboxOf(stl);
    closeV(bb.max, [0.06, 0.004, 0.006], 1e-6, 'arm mesh in metres');
    // the carriage's bore makes it non-convex: a decomposition hint; the base's two solids are convex
    assert.deepEqual(r.hints.map((h) => [h.mesh, h.convex]), [['Base_s0', true], ['Base_s1', true], ['Carriage_s0', false], ['Arm_s0', false], ['Ball_s0', true]]);
  });

  it('primitive collision: the ball becomes a sphere, parts without a fit fall back to the hull with a warning', async () => {
    const r = await exportMjcf(k, sliderHingeAssembly(), { collision: 'primitive', tag: 'mjcf-prim' });
    const g = findAll(parse(r.xml), 'geom').find((e) => e.attrs.name === 'Ball_collision')!;
    assert.equal(g.attrs.type, 'sphere');
    close(nums(g.attrs.size)[0], 0.01, 1e-6, 'radius');
    assert.ok(r.meta.warnings.some((w) => w.includes('Carriage') && w.includes('convex hull')));
  });

  it('loads in MuJoCo: compiled poses, axes, ranges and masses equal the CAD; joint values give the poses posesAt computes', { skip: !PY && 'MUJOCO_PYTHON not set' }, async () => {
    const doc = sliderHingeAssembly();
    const r = await exportMjcf(k, doc, { collision: 'hull', tag: 'mjcf-mj', sim: { option: { timestep: 0.001 }, actuators: [{ type: 'position', name: 'arm', joint: 'Arm_pivot', kp: 2 }] } });
    const file = await writeMjcf(dir, r);
    const m = mujoco(file);
    const plan: MjPlan = r.plan;
    for (const b of plan.bodies) {
      const got = m.bodies[b.name];
      closeV(got.pos, b.place.o.map((v) => v / 1000), 1e-9, `${b.name} pos`);
      sameRot(got.quat, quat(b.place), 1e-9, `${b.name} quat`);
      close(got.mass, plan.parts[b.part].mass, 1e-9 * Math.max(1, plan.parts[b.part].mass), `${b.name} mass`);
      for (const j of b.joints) {
        if (j.type === 'free') continue;
        const mj = m.joints[j.name];
        assert.equal(mj.type, j.type);
        closeV(mj.axis, j.axis, 1e-9, `${j.name} axis`);
        if (j.range) { assert.equal(mj.limited, true); closeV(mj.range, j.type === 'slide' ? j.range.map((v) => v / 1000) : j.range, 1e-9, `${j.name} range`); }
      }
    }
    // a logged trajectory (MuJoCo units) scrubbed on the CAD components gives MuJoCo's body poses
    const traj = readTrajectory('time,Carriage_on_rod,Arm_pivot\n0,0,0\n1,0.06,0.8\n2,0.1,-1.2\n');
    const { study, unknown } = motionStudy(plan, traj);
    assert.deepEqual(unknown, []);
    for (const t of [0.5, 1.5]) {
      const q: Record<string, number> = {};
      for (const tr of study.tracks) {
        const ks = tr.keys, i = ks.findIndex((kk) => kk[0] > t);
        const v = ks[i - 1][1] + ((ks[i][1] - ks[i - 1][1]) * (t - ks[i - 1][0])) / (ks[i][0] - ks[i - 1][0]);
        q[tr.joint] = tr.unit === 'deg' ? (v * Math.PI) / 180 : v / 1000;
      }
      const mm = mujoco(file, ['--qpos', JSON.stringify(q)]);
      const ours = posesAt(plan, study, t);
      for (const b of plan.bodies) {
        if (b.name === 'Ball') continue;
        const p = ours.get(b.id)!;
        closeV(mm.bodies[b.name].pos, p.o.map((v) => v / 1000), 1e-9, `t=${t} ${b.name} pos`);
        sameRot(mm.bodies[b.name].quat, quat(p), 1e-9, `t=${t} ${b.name} quat`);
      }
    }
    // a short simulation stays inside the ranges and the slide moves only along the rod
    const sim = mujoco(file, ['--steps', '400']);
    const s = sim.qpos.Carriage_on_rod as number, a = sim.qpos.Arm_pivot as number;
    const rs = m.joints.Carriage_on_rod.range, ra = m.joints.Arm_pivot.range;
    assert.ok(s >= rs[0] - 1e-3 && s <= rs[1] + 1e-3, `slide ${s} in ${rs}`);
    assert.ok(a >= ra[0] - 0.05 && a <= ra[1] + 0.05, `hinge ${a} in ${ra}`);
    const c = sim.bodies.Carriage.pos;
    closeV([c[1], c[2]], [0, 0], 1e-9, 'carriage stays on the rod');
  });
});
