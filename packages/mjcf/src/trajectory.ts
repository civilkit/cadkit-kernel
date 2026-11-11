// Joint trajectories logged from a MuJoCo run -> a motion study on the CAD components.
// The log is in MuJoCo units (rad for hinges, m for slides); the study is in CAD units (degrees, mm), keyed by joint name with the
// component and the mate each joint came from, so a viewer can scrub it on the real parts.
import type { Pose } from './math.js';
import { bodyPoses, type MjcfMeta } from './mjcf.js';
import type { MjPlan } from './plan.js';

export interface Trajectory { time: number[]; joints: Record<string, number[]> }

export interface MotionTrack {
  joint: string;
  component: string;
  mate?: string;
  type: 'hinge' | 'slide';
  unit: 'deg' | 'mm';
  /** [time s, value in `unit`], time ascending */
  keys: Array<[number, number]>;
}
export interface MotionStudy { name: string; source: string; duration: number; tracks: MotionTrack[] }

const isNum = (x: unknown): x is number => typeof x === 'number' && Number.isFinite(x);
const strip = (s: string) => s.trim().replace(/^"|"$/g, '').replace(/^qpos[.:/_\s]+/i, '');
const TIME = /^(time|t|sim_?time|timestamp)$/i;

function csvRows(text: string): string[][] {
  return text.split(/\r?\n/).filter((l) => l.trim() && !l.trimStart().startsWith('#')).map((l) => l.split(/[,;\t]/).map((c) => c.trim()));
}

/** Read a joint log: CSV with a header (`time,<joint>,...`; the first column is the time unless one is named time / t), or JSON as
 *  `{time: [...], qpos: {joint: [...]}}`, `{time: [...], joints: [names], qpos: [[...], ...]}` or `[{time, qpos: {joint: v}}, ...]`
 *  (`t` for `time`, and joint values directly on the row, are accepted too). */
export function readTrajectory(text: string): Trajectory {
  const s = text.trim();
  if (s.startsWith('{') || s.startsWith('[')) return fromJson(JSON.parse(s) as unknown);
  const rows = csvRows(s);
  if (rows.length < 2) throw new Error('trajectory: a header and at least one row are needed');
  const head = rows[0].map(strip);
  const ti = Math.max(0, head.findIndex((h) => TIME.test(h)));
  const joints: Record<string, number[]> = {};
  const time: number[] = [];
  head.forEach((h, i) => { if (i !== ti) joints[h] = []; });
  rows.slice(1).forEach((r, k) => {
    const t = Number(r[ti]);
    if (!Number.isFinite(t)) throw new Error(`trajectory: row ${k + 2}: time "${r[ti]}" is not a number`);
    time.push(t);
    head.forEach((h, i) => { if (i !== ti) joints[h].push(Number(r[i])); });
  });
  return check({ time, joints });
}

function fromJson(o: unknown): Trajectory {
  if (Array.isArray(o)) {
    const time: number[] = [];
    const joints: Record<string, number[]> = {};
    o.forEach((row, k) => {
      const r = row as Record<string, unknown>;
      const t = r.time ?? r.t;
      if (!isNum(t)) throw new Error(`trajectory: row ${k}: no time`);
      time.push(t);
      const vals = (r.qpos && typeof r.qpos === 'object' && !Array.isArray(r.qpos) ? r.qpos : r) as Record<string, unknown>;
      for (const [name, v] of Object.entries(vals)) {
        if (name === 'time' || name === 't' || !isNum(v)) continue;
        (joints[strip(name)] ??= new Array(k).fill(NaN)).push(v);
      }
      for (const a of Object.values(joints)) while (a.length < time.length) a.push(NaN);
    });
    return check({ time, joints });
  }
  const r = (o ?? {}) as Record<string, unknown>;
  const time = (r.time ?? r.t) as unknown;
  if (!Array.isArray(time)) throw new Error('trajectory: no time array');
  if (Array.isArray(r.joints) && Array.isArray(r.qpos)) {
    const names = (r.joints as unknown[]).map((x) => strip(String(x)));
    const joints: Record<string, number[]> = Object.fromEntries(names.map((n) => [n, [] as number[]]));
    (r.qpos as unknown[][]).forEach((row) => names.forEach((n, i) => joints[n].push(Number(row[i]))));
    return check({ time: time.map(Number), joints });
  }
  if (r.qpos && typeof r.qpos === 'object') {
    const joints: Record<string, number[]> = {};
    for (const [n, v] of Object.entries(r.qpos as Record<string, unknown>)) if (Array.isArray(v)) joints[strip(n)] = v.map(Number);
    return check({ time: time.map(Number), joints });
  }
  throw new Error('trajectory: expected qpos (an object of arrays, or rows with joints[])');
}

function check(t: Trajectory): Trajectory {
  for (const [n, v] of Object.entries(t.joints)) if (v.length !== t.time.length) throw new Error(`trajectory: ${n} has ${v.length} values for ${t.time.length} times`);
  for (let i = 1; i < t.time.length; i++) if (!(t.time[i] >= t.time[i - 1])) throw new Error(`trajectory: time goes back at row ${i}`);
  return t;
}

type JointIndex = Record<string, { component: string; type: string; mate?: string }>;
function index(m: MjPlan | MjcfMeta): JointIndex {
  if ('bodies' in m && Array.isArray(m.bodies)) {
    const out: JointIndex = {};
    for (const b of (m as MjPlan).bodies) for (const j of b.joints) out[j.name] = { component: b.id, type: j.type, ...(j.mate ? { mate: j.mate } : {}) };
    return out;
  }
  return (m as MjcfMeta).joints;
}

export interface StudyOptions {
  name?: string;
  /** length unit of the log (default 'm', MuJoCo's) */
  units?: 'm' | 'mm';
  /** keep at most this many keys per track (evenly spaced, first and last kept); default all */
  maxKeys?: number;
}

/** Turn a trajectory into a motion study of the exported joints. Columns that name no exported hinge / slide (or a mate id / name
 *  that maps to exactly one joint) are listed in `unknown`. */
export function motionStudy(model: MjPlan | MjcfMeta, traj: Trajectory, opts: StudyOptions = {}): { study: MotionStudy; unknown: string[] } {
  const idx = index(model);
  const k = (opts.units ?? 'm') === 'm' ? 1000 : 1;
  const tracks: MotionTrack[] = [];
  const unknown: string[] = [];
  const n = traj.time.length;
  const keep = (opts.maxKeys && opts.maxKeys >= 2 && n > opts.maxKeys)
    ? [...new Set(Array.from({ length: opts.maxKeys }, (_, i) => Math.round((i * (n - 1)) / (opts.maxKeys! - 1))))]
    : Array.from({ length: n }, (_, i) => i);
  for (const [col, vals] of Object.entries(traj.joints)) {
    let name = idx[col] ? col : undefined;
    if (!name) {
      const byMate = Object.entries(idx).filter(([, j]) => j.mate === col);
      if (byMate.length === 1) name = byMate[0][0];
    }
    const j = name ? idx[name] : undefined;
    if (!name || !j || (j.type !== 'hinge' && j.type !== 'slide')) { unknown.push(col); continue; }
    const conv = j.type === 'hinge' ? 180 / Math.PI : k;
    tracks.push({
      joint: name, component: j.component, ...(j.mate ? { mate: j.mate } : {}), type: j.type, unit: j.type === 'hinge' ? 'deg' : 'mm',
      keys: keep.filter((i) => Number.isFinite(vals[i])).map((i) => [traj.time[i], vals[i] * conv] as [number, number]),
    });
  }
  const t0 = traj.time[0] ?? 0, t1 = traj.time[n - 1] ?? 0;
  return { study: { name: opts.name ?? 'MuJoCo run', source: 'mujoco', duration: t1 - t0, tracks }, unknown };
}

/** The document with the study stored under `motion.studies` (replacing a study of the same name). Not a feature: the kernel
 *  ignores it, like the GUI's exploded view. */
export function withMotionStudy<D extends Record<string, unknown>>(doc: D, study: MotionStudy): D & { motion: { studies: MotionStudy[] } } {
  const old = ((doc.motion as { studies?: MotionStudy[] } | undefined)?.studies ?? []).filter((s) => s.name !== study.name);
  return { ...doc, motion: { ...(doc.motion as object | undefined), studies: [...old, study] } };
}

/** Track values at time t (linear between keys, held outside), CAD units. */
export function sampleAt(study: MotionStudy, t: number): Record<string, number> {
  const out: Record<string, number> = {};
  for (const tr of study.tracks) {
    const ks = tr.keys;
    if (!ks.length) continue;
    if (t <= ks[0][0]) { out[tr.joint] = ks[0][1]; continue; }
    if (t >= ks[ks.length - 1][0]) { out[tr.joint] = ks[ks.length - 1][1]; continue; }
    let lo = 0, hi = ks.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ks[m][0] <= t) lo = m; else hi = m; }
    const [ta, va] = ks[lo], [tb, vb] = ks[hi];
    out[tr.joint] = tb > ta ? va + ((vb - va) * (t - ta)) / (tb - ta) : vb;
  }
  return out;
}

/** World placement (mm) of every component at time t of the study. */
export function posesAt(plan: MjPlan, study: MotionStudy, t: number): Map<string, Pose> {
  const v = sampleAt(study, t);
  const q: Record<string, number> = {};
  for (const tr of study.tracks) if (tr.joint in v) q[tr.joint] = tr.unit === 'deg' ? (v[tr.joint] * Math.PI) / 180 : v[tr.joint];
  return bodyPoses(plan, q);
}
