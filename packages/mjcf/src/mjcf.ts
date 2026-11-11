// MjPlan -> MJCF text + mesh files. Pure: no kernel, no file system.
import { type Pose, type V3, apply, applyInv, axisAngle, compose, fmt, fmtv, fromQuat, IDENTITY, quat, relative, rotInv, scale, sub } from './math.js';
import { el, serialize, type XmlNode } from './xml.js';
import { writeStl } from './stl.js';
import { isConvex, lumpPositions, meshVolume } from './geom.js';
import type { MjBody, MjPart, MjPlan } from './plan.js';
import type { Attrs, CollisionMode, ContactParams, MjcfOptions } from './options.js';

/** A solid whose collision mesh is not convex: MuJoCo collides its hull, so decompose it externally (CoACD / V-HACD) for contact detail. */
export interface DecompositionHint { part: string; mesh: string; file: string; solid: number; convex: boolean; volume: number; suggestion: string }

export interface MjcfResult {
  xml: string;
  /** path relative to the model file -> bytes */
  files: Record<string, Uint8Array>;
  hints: DecompositionHint[];
  /** component id -> body name, mate id -> joint names, part key -> mesh name, plus masses and warnings (written as `<model>.meta.json`) */
  meta: MjcfMeta;
}

export interface MjcfMeta {
  model: string;
  units: 'm' | 'mm';
  bodies: Record<string, { name: string; parent: string | null; part: string; mass: number; joints: Array<{ name: string; type: string; mate?: string }> }>;
  joints: Record<string, { body: string; component: string; type: string; mate?: string; range?: [number, number] }>;
  parts: Record<string, { name: string; material: string; density: number; mass: number; volume: number; collision: CollisionMode | 'primitive-fallback'; primitive?: string }>;
  hints: DecompositionHint[];
  warnings: string[];
}

const attrs = (a: Attrs | ContactParams | undefined): Record<string, string> => {
  const out: Record<string, string> = {};
  for (const [k, v] of Object.entries(a ?? {})) {
    if (v === undefined || v === null) continue;
    out[k] = Array.isArray(v) ? fmtv(v as number[]) : typeof v === 'number' ? fmt(v) : String(v);
  }
  return out;
};
const raw = (text: string | undefined): XmlNode[] => (text && text.trim() ? [{ tag: '#raw', attrs: { text }, children: [] }] : []);
const pick = (names: readonly string[] | undefined, ...keys: string[]): boolean => !!names && keys.some((k) => names.includes(k));

export function toMjcf(plan: MjPlan, opts: MjcfOptions = {}): MjcfResult {
  const units = opts.units ?? 'm';
  const s = units === 'm' ? 1e-3 : 1;
  const L = (v: V3): V3 => scale(v, s);
  const meshDir = (opts.meshDir ?? 'meshes').replace(/\/+$/, '');
  const warnings = [...plan.warnings];
  const files: Record<string, Uint8Array> = {};
  const hints: DecompositionHint[] = [];
  const sim = opts.sim ?? {};
  const model = opts.model ?? plan.model;
  const byId = new Map(plan.bodies.map((b) => [b.id, b]));
  // CAD +y up: turn the world +90 degrees about x (y -> z, z -> -y)
  const upPose: Pose = opts.up === 'y' ? { o: [0, 0, 0], x: [1, 0, 0], y: [0, 0, 1], z: [0, -1, 0] } : IDENTITY;

  const asset: XmlNode[] = [];
  const meshAssets = new Set<string>();
  const addMesh = (name: string, file: string, positions: ArrayLike<number>): void => {
    if (meshAssets.has(name)) return;
    meshAssets.add(name);
    const p = new Float32Array(positions.length);
    for (let i = 0; i < p.length; i++) p[i] = positions[i] * s;
    files[`${meshDir}/${file}`] = writeStl(p, `ck-cad ${name}`);
    asset.push(el('mesh', { name, file }));
  };

  const meta: MjcfMeta = { model, units, bodies: {}, joints: {}, parts: {}, hints, warnings };
  const modeOf = (b: MjBody, part: MjPart): CollisionMode => {
    if (pick(opts.visualOnly, b.name, b.id, part.name)) return 'none';
    return opts.collisionByPart?.[b.name] ?? opts.collisionByPart?.[b.id] ?? opts.collisionByPart?.[part.name] ?? opts.collision ?? 'hull';
  };
  const contactOf = (b: MjBody, part: MjPart): Record<string, string> =>
    attrs({ ...(sim.contact?.bodies?.[part.name] ?? {}), ...(sim.contact?.bodies?.[b.name] ?? {}) });

  const materials = new Map<string, string>();
  const colorOf = (b: MjBody, part: MjPart): string => {
    const rgba = opts.colors?.[b.name] ?? opts.colors?.[part.name] ?? [0.7, 0.72, 0.75, 1];
    const key = fmtv(rgba);
    if (!materials.has(key)) {
      const name = `mat${materials.size}`;
      materials.set(key, name);
      asset.push(el('material', { name, rgba: key }));
    }
    return materials.get(key)!;
  };

  // geoms of one body (part frame = body frame)
  const geoms = (b: MjBody, part: MjPart): XmlNode[] => {
    const out: XmlNode[] = [];
    const vis = `${part.name}`;
    addMesh(vis, `${part.name}.stl`, part.positions);
    out.push(el('geom', { name: `${b.name}_visual`, class: 'visual', type: 'mesh', mesh: vis, material: colorOf(b, part) }));
    let mode = modeOf(b, part);
    const cp = contactOf(b, part);
    let used: CollisionMode | 'primitive-fallback' = mode;
    if (mode === 'primitive') {
      const pr = part.primitive;
      if (pr) {
        out.push(el('geom', { name: `${b.name}_collision`, class: 'collision', type: pr.type, size: fmtv(pr.type === 'box' || pr.type === 'sphere' ? pr.size.map((x) => x * s) : [pr.size[0] * s, pr.size[1] * s]), pos: fmtv(L(pr.pose.o)), quat: fmtv(quat(pr.pose)), ...cp }));
      } else {
        used = 'primitive-fallback';
        mode = 'hull';
        warnings.push(`part ${part.name}: no primitive fits, collides as one convex hull`);
      }
    }
    if (mode === 'hull') out.push(el('geom', { name: `${b.name}_collision`, class: 'collision', type: 'mesh', mesh: vis, ...cp }));
    if (mode === 'solids') {
      for (let k = 0; k < part.lumps; k++) {
        const pos = part.lumps === 1 ? part.positions : lumpPositions(part.positions, part.lumpOfTri, k);
        const mesh = `${part.name}_s${k}`;
        const file = `${mesh}.stl`;
        addMesh(mesh, file, pos);
        out.push(el('geom', { name: `${b.name}_collision_s${k}`, class: 'collision', type: 'mesh', mesh, ...cp }));
        if (!hints.some((h) => h.mesh === mesh)) {
          const convex = part.convexSolids?.[k] ?? isConvex(pos);
          hints.push({ part: part.name, mesh, file: `${meshDir}/${file}`, solid: k, convex, volume: meshVolume(pos), suggestion: convex ? 'convex: use as is' : 'not convex: MuJoCo collides its hull; decompose with CoACD or V-HACD and replace the geom by the pieces' });
        }
      }
    }
    meta.parts[part.key] = { name: part.name, material: part.material, density: part.density, mass: part.mass, volume: part.volume, collision: used, ...(part.primitive ? { primitive: part.primitive.type } : {}) };
    return out;
  };

  const jointAttrs = (name: string): Record<string, string> => attrs({ ...(sim.joints?.['*'] ?? {}), ...(sim.joints?.[name] ?? {}) });

  const bodyNode = (b: MjBody): XmlNode => {
    const part = plan.parts[b.part];
    const parentPlace: Pose = b.parent ? byId.get(b.parent)!.place : IDENTITY;
    const rel = b.parent ? relative(parentPlace, b.place) : compose(upPose, b.place);
    const kids: XmlNode[] = [];
    // inertial (explicit on every body)
    if (part && part.mass > 0) {
      const I = part.inertia, k = units === 'm' ? 1 : 1e6; // kg m² -> kg mm² when units are mm
      kids.push(el('inertial', { pos: fmtv(L(part.com)), mass: fmt(part.mass), fullinertia: fmtv([I[0][0], I[1][1], I[2][2], I[0][1], I[0][2], I[1][2]].map((x) => x * k)) }));
    } else {
      warnings.push(`body ${b.name}: no mass (empty part); given a tiny placeholder inertial`);
      kids.push(el('inertial', { pos: '0 0 0', mass: '1e-6', diaginertia: '1e-12 1e-12 1e-12' }));
    }
    for (const j of b.joints) {
      if (j.type === 'free') { kids.push(el('freejoint', { name: j.name })); }
      else {
        const a: Record<string, string> = { name: j.name, type: j.type, pos: fmtv(L(applyInv(b.place, j.pos))) };
        if (j.type !== 'ball') a.axis = fmtv(rotInv(b.place, j.axis));
        if (j.range) a.range = fmtv(j.type === 'slide' ? j.range.map((x) => x * s) : j.range);
        kids.push(el('joint', { ...a, ...jointAttrs(j.name) }));
      }
      meta.joints[j.name] = { body: b.name, component: b.id, type: j.type, ...(j.mate ? { mate: j.mate } : {}), ...(j.range ? { range: j.range } : {}) };
    }
    if (part) kids.push(...geoms(b, part));
    for (const c of plan.bodies) if (c.parent === b.id) kids.push(bodyNode(c));
    meta.bodies[b.id] = { name: b.name, parent: b.parent, part: b.part, mass: part?.mass ?? 0, joints: b.joints.map((j) => ({ name: j.name, type: j.type, ...(j.mate ? { mate: j.mate } : {}) })) };
    return el('body', { name: b.name, pos: fmtv(L(rel.o)), quat: fmtv(quat(rel)) }, kids);
  };

  const world = plan.bodies.filter((b) => b.parent === null).map(bodyNode);
  // a body whose parent is missing from the plan would be lost: report it
  for (const b of plan.bodies) if (b.parent !== null && !byId.has(b.parent)) warnings.push(`body ${b.name}: parent ${b.parent} is not exported`);

  const root: XmlNode[] = [];
  root.push(el('compiler', { angle: 'radian', meshdir: meshDir, autolimits: 'true', inertiafromgeom: 'false', ...attrs(sim.compiler) }));
  if (sim.option) root.push(el('option', attrs(sim.option)));
  root.push(el('default', {}, [
    el('default', { class: 'visual' }, [el('geom', { contype: '0', conaffinity: '0', group: '2', density: '0' })]),
    el('default', { class: 'collision' }, [el('geom', { group: '3', density: '0', rgba: '0.9 0.4 0.3 0.5', ...attrs(sim.contact?.default) })]),
  ]));
  for (const f of sim.include ?? []) root.push(el('include', { file: f }));
  root.push(el('asset', {}, [...asset, ...raw(sim.asset)]));
  root.push(el('worldbody', {}, [...world, ...raw(sim.worldbody)]));
  const eq = plan.couplings.map((c) => {
    const t1 = jointType(plan, c.joint1), t2 = jointType(plan, c.joint2);
    const k1 = t1 === 'slide' ? s : 1, k2 = t2 === 'slide' ? s : 1;
    return el('joint', { name: c.name, joint1: c.joint1, joint2: c.joint2, polycoef: fmtv([c.c0 * k1, (c.c1 * k1) / k2, 0, 0, 0]) });
  });
  if (eq.length) root.push(el('equality', {}, eq));
  const ex = (sim.contact?.exclude ?? []).map(([a, b]) => el('exclude', { body1: a, body2: b }));
  if (ex.length) root.push(el('contact', {}, ex));
  const act = (sim.actuators ?? []).map(({ type, ...a }) => el(type ?? 'motor', attrs(a)));
  if (act.length) root.push(el('actuator', {}, act));
  const sen = (sim.sensors ?? []).map(({ type, ...a }) => el(type, attrs(a)));
  if (sen.length) root.push(el('sensor', {}, sen));
  root.push(...raw(sim.raw));
  for (const a of sim.actuators ?? []) if (typeof a.joint === 'string' && !meta.joints[a.joint]) warnings.push(`actuator ${String(a.name ?? '')}: no exported joint "${a.joint}"`);

  const xml = '<?xml version="1.0" encoding="utf-8"?>\n' + serialize(el('mujoco', { model }, root));
  return { xml, files, hints, meta };
}

function jointType(plan: MjPlan, name: string): string | undefined {
  for (const b of plan.bodies) for (const j of b.joints) if (j.name === name) return j.type;
  return undefined;
}

/** World pose of every body for given joint values (CAD units: rad, mm); missing joints stay at 0. Same composition MuJoCo uses
 *  (joints applied in order, each about its axis through its anchor, in the body frame). */
export function bodyPoses(plan: MjPlan, q: Record<string, number | number[]>): Map<string, Pose> {
  const out = new Map<string, Pose>();
  const byId = new Map(plan.bodies.map((b) => [b.id, b]));
  const get = (b: MjBody): Pose => {
    const hit = out.get(b.id);
    if (hit) return hit;
    const parentNow = b.parent ? get(byId.get(b.parent)!) : IDENTITY;
    const parent0 = b.parent ? byId.get(b.parent)!.place : IDENTITY;
    // body pose relative to its parent at the CAD pose, then the joint motion in the body frame
    let local = relative(parent0, b.place);
    let motion: Pose = IDENTITY;
    for (const j of b.joints) {
      const v = q[j.name];
      if (v === undefined) continue;
      const axisL = rotInv(b.place, j.axis), anchor = applyInv(b.place, j.pos);
      if (j.type === 'slide' && typeof v === 'number') motion = compose(motion, { ...IDENTITY, o: scale(axisL, v) });
      else if (j.type === 'hinge' && typeof v === 'number') motion = compose(motion, about(anchor, axisL, v));
      else if (j.type === 'ball' && Array.isArray(v)) motion = compose(motion, aboutQuat(anchor, v));
      // a freejoint value is the world pose (mm, quaternion w x y z); free bodies sit under the world
      else if (j.type === 'free' && Array.isArray(v) && v.length >= 7) local = fromQuat(v.slice(3, 7), [v[0], v[1], v[2]]);
    }
    const p = compose(compose(parentNow, local), motion);
    out.set(b.id, p);
    return p;
  };
  for (const b of plan.bodies) get(b);
  return out;
}

function about(anchor: V3, axis: V3, ang: number): Pose {
  const r = axisAngle(axis, ang);
  return { ...r, o: sub(anchor, apply(r, anchor)) };
}
function aboutQuat(anchor: V3, q: number[]): Pose {
  const r = fromQuat(q);
  return { ...r, o: sub(anchor, apply(r, anchor)) };
}
