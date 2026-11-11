// Rigid poses for the MJCF exporter: a rotation (columns x, y, z of the body frame in the parent frame) plus a translation.
// The kernel's component placement `{o, x, y, z}` is such a pose (part-local -> world, millimetres).

export type V3 = [number, number, number];
/** Rotation as its three columns (the frame's axes) and the origin. */
export interface Pose { o: V3; x: V3; y: V3; z: V3 }

export const IDENTITY: Pose = { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };

export const add = (a: V3, b: V3): V3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a: V3, k: number): V3 => [a[0] * k, a[1] * k, a[2] * k];
export const dot = (a: V3, b: V3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: V3, b: V3): V3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (a: V3): number => Math.hypot(a[0], a[1], a[2]);
export const unit = (a: V3): V3 => { const n = norm(a); return n > 0 ? scale(a, 1 / n) : [0, 0, 0]; };
export const v3 = (a: unknown, d: V3 = [0, 0, 0]): V3 => (Array.isArray(a) && a.length >= 3 && a.slice(0, 3).every((x) => typeof x === 'number' && Number.isFinite(x)) ? [a[0], a[1], a[2]] : d);

/** Re-orthonormalise a pose read from the kernel (Gram-Schmidt on x, y; z = x cross y). */
export function clean(p: { o?: unknown; x?: unknown; y?: unknown; z?: unknown } | undefined): Pose {
  if (!p) return IDENTITY;
  const x = unit(v3(p.x, [1, 0, 0]));
  let y = v3(p.y, [0, 1, 0]);
  y = unit(sub(y, scale(x, dot(x, y))));
  return { o: v3(p.o), x, y, z: cross(x, y) };
}

/** Direction from the pose's frame to the parent frame. */
export const rot = (p: Pose, v: V3): V3 => add(add(scale(p.x, v[0]), scale(p.y, v[1])), scale(p.z, v[2]));
/** Point from the pose's frame to the parent frame. */
export const apply = (p: Pose, v: V3): V3 => add(p.o, rot(p, v));
/** Direction from the parent frame into the pose's frame. */
export const rotInv = (p: Pose, v: V3): V3 => [dot(p.x, v), dot(p.y, v), dot(p.z, v)];
/** Point from the parent frame into the pose's frame. */
export const applyInv = (p: Pose, v: V3): V3 => rotInv(p, sub(v, p.o));

/** a then b: the pose of b's frame expressed in a's parent (a ∘ b). */
export const compose = (a: Pose, b: Pose): Pose => ({ o: apply(a, b.o), x: rot(a, b.x), y: rot(a, b.y), z: rot(a, b.z) });
export function inverse(p: Pose): Pose {
  const x: V3 = [p.x[0], p.y[0], p.z[0]], y: V3 = [p.x[1], p.y[1], p.z[1]], z: V3 = [p.x[2], p.y[2], p.z[2]];
  const r = { o: [0, 0, 0] as V3, x, y, z };
  return { ...r, o: scale(rot(r, p.o), -1) };
}
/** b relative to a: inverse(a) ∘ b. */
export const relative = (a: Pose, b: Pose): Pose => compose(inverse(a), b);

/** Rotation about a unit axis through the origin (Rodrigues). */
export function axisAngle(axis: V3, ang: number): Pose {
  const [x, y, z] = unit(axis), c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  return {
    o: [0, 0, 0],
    x: [t * x * x + c, t * x * y + s * z, t * x * z - s * y],
    y: [t * x * y - s * z, t * y * y + c, t * y * z + s * x],
    z: [t * x * z + s * y, t * y * z - s * x, t * z * z + c],
  };
}

/** Quaternion (w, x, y, z), w >= 0, of the pose's rotation. */
export function quat(p: Pose): [number, number, number, number] {
  const m00 = p.x[0], m10 = p.x[1], m20 = p.x[2], m01 = p.y[0], m11 = p.y[1], m21 = p.y[2], m02 = p.z[0], m12 = p.z[1], m22 = p.z[2];
  const tr = m00 + m11 + m22;
  let w: number, x: number, y: number, z: number;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
  }
  const n = Math.hypot(w, x, y, z) * (w < 0 ? -1 : 1);
  return [w / n, x / n, y / n, z / n];
}

/** Pose of a quaternion (w, x, y, z) and a position. */
export function fromQuat(q: readonly number[], o: V3 = [0, 0, 0]): Pose {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  const [w, x, y, z] = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
  return {
    o,
    x: [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)],
    y: [2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)],
    z: [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)],
  };
}

/** Number for XML: 12 significant digits, tiny values (|x| < 1e-12) and -0 written as 0. */
export function fmt(x: number): string {
  if (!Number.isFinite(x)) return '0';
  if (Math.abs(x) < 1e-12) return '0';
  return String(+x.toPrecision(12));
}
export const fmtv = (v: readonly number[]): string => v.map(fmt).join(' ');

/** R M Rᵀ for a 3x3 row-major matrix M and the pose's rotation R. */
export function rotTensor(p: Pose, m: number[][]): number[][] {
  const R = [[p.x[0], p.y[0], p.z[0]], [p.x[1], p.y[1], p.z[1]], [p.x[2], p.y[2], p.z[2]]];
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0;
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) s += R[i][a] * m[a][b] * R[j][b];
    out[i][j] = s;
  }
  return out;
}
