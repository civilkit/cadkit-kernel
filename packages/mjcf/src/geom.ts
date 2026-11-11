// Mesh helpers for the collision geometry: volume, a convexity test, per-solid split, and fitting primitives
// (box / cylinder / sphere / capsule) to a part from its face kinds, mesh and exact mass properties.
import { type Pose, type V3, dot, sub, unit, cross } from './math.js';
import type { MjPrimitive } from './plan.js';

/** Signed volume of a closed triangle soup (divergence theorem). */
export function meshVolume(p: ArrayLike<number>): number {
  let v = 0;
  for (let i = 0; i + 8 < p.length; i += 9) {
    v += p[i] * (p[i + 4] * p[i + 8] - p[i + 5] * p[i + 7]) - p[i + 1] * (p[i + 3] * p[i + 8] - p[i + 5] * p[i + 6]) + p[i + 2] * (p[i + 3] * p[i + 7] - p[i + 4] * p[i + 6]);
  }
  return v / 6;
}

/** Triangles of one solid. */
export function lumpPositions(p: Float32Array, lumpOfTri: Int32Array, lump: number): Float32Array {
  let n = 0;
  for (let t = 0; t < lumpOfTri.length; t++) if (lumpOfTri[t] === lump) n++;
  const out = new Float32Array(n * 9);
  let k = 0;
  for (let t = 0; t < lumpOfTri.length; t++) if (lumpOfTri[t] === lump) { out.set(p.subarray(t * 9, t * 9 + 9), k); k += 9; }
  return out;
}

function uniqueVerts(p: ArrayLike<number>): V3[] {
  const seen = new Map<string, V3>();
  for (let i = 0; i + 2 < p.length; i += 3) {
    const v: V3 = [p[i], p[i + 1], p[i + 2]];
    seen.set(v.map((x) => x.toFixed(5)).join(','), v);
  }
  return [...seen.values()];
}

/** True when every vertex lies on the inner side of every triangle's plane (within `tol` mm, scaled by the mesh size).
 *  At most `maxPlanes` triangles are tested (evenly spread), so a huge mesh costs a bounded time. */
export function isConvex(p: ArrayLike<number>, tol = 5e-3, maxPlanes = 4000): boolean {
  const verts = uniqueVerts(p);
  const nt = Math.floor(p.length / 9);
  if (nt < 4) return true;
  let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const v of verts) for (let a = 0; a < 3; a++) { lo[a] = Math.min(lo[a], v[a]); hi[a] = Math.max(hi[a], v[a]); }
  const eps = tol * Math.max(1, Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]));
  const step = Math.max(1, Math.floor(nt / maxPlanes));
  for (let t = 0; t < nt; t += step) {
    const a: V3 = [p[t * 9], p[t * 9 + 1], p[t * 9 + 2]];
    const b: V3 = [p[t * 9 + 3], p[t * 9 + 4], p[t * 9 + 5]], c: V3 = [p[t * 9 + 6], p[t * 9 + 7], p[t * 9 + 8]];
    const n = cross(sub(b, a), sub(c, a));
    const l = Math.hypot(n[0], n[1], n[2]);
    // slivers (near a sphere's pole, say) have no reliable plane in float32: skip them
    const e2 = Math.max(dot(sub(b, a), sub(b, a)), dot(sub(c, a), sub(c, a)), dot(sub(c, b), sub(c, b)));
    if (l < 1e-12 || l < 1e-2 * e2) continue;
    const u: V3 = [n[0] / l, n[1] / l, n[2] / l];
    // every vertex on one side of the plane (either winding)
    let above = false, below = false;
    for (const v of verts) {
      const d = dot(u, sub(v, a));
      if (d > eps) above = true; else if (d < -eps) below = true;
      if (above && below) return false;
    }
  }
  return true;
}

/** Extents of the vertices along three unit axes: [min, max] per axis. */
function extents(p: ArrayLike<number>, axes: V3[]): Array<[number, number]> {
  const out: Array<[number, number]> = axes.map(() => [Infinity, -Infinity]);
  for (let i = 0; i + 2 < p.length; i += 3) {
    const v: V3 = [p[i], p[i + 1], p[i + 2]];
    axes.forEach((a, k) => { const d = dot(a, v); if (d < out[k][0]) out[k][0] = d; if (d > out[k][1]) out[k][1] = d; });
  }
  return out;
}

export interface FitInput {
  faceKinds: string[];
  lumps: number;
  positions: ArrayLike<number>;
  volume: number;
  com: V3;
  /** principal moments for unit density (mm⁵) and their axes */
  principal: number[];
  axes: number[][];
}

const near = (a: number, b: number, rel: number): boolean => Math.abs(a - b) <= rel * Math.max(Math.abs(a), Math.abs(b), 1e-12);

/** A primitive that matches the part, or null. The face kinds must fit (planes only for a box, one sphere face for a sphere,
 *  cylinder + 2 planes for a cylinder, cylinder + 2 spheres for a capsule) and the exact volume and principal moments must agree
 *  with the primitive's to `rel` (default 1e-4). */
export function fitPrimitive(f: FitInput, rel = 1e-4): MjPrimitive | null {
  if (f.lumps !== 1 || !(f.volume > 0) || f.principal.length !== 3 || f.axes.length !== 3) return null;
  const kinds = new Map<string, number>();
  for (const k of f.faceKinds) kinds.set(k, (kinds.get(k) ?? 0) + 1);
  const only = (...ks: string[]) => [...kinds.keys()].every((k) => ks.includes(k));
  const V = f.volume, I = f.principal;
  const ax = f.axes.map((a) => unit([a[0], a[1], a[2]])) as V3[];
  const frame = (k: number): Pose => {
    // z along axis k, right-handed
    const z = ax[k], x = ax[(k + 1) % 3], y = cross(z, x);
    return { o: f.com, x, y, z };
  };
  // sphere: I = 2/5 V r²
  if (only('sphere')) {
    const r = Math.cbrt((3 * V) / (4 * Math.PI));
    if (I.every((m) => near(m, 0.4 * V * r * r, rel * 10))) return { type: 'sphere', size: [r], pose: { o: f.com, x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] } };
    return null;
  }
  // box: I_a = V (b² + c²) / 12 -> a² = 6 (I_b + I_c - I_a) / V
  if (only('plane') && f.faceKinds.length === 6) {
    const e = [0, 1, 2].map((k) => Math.sqrt(Math.max(0, (6 * (I[(k + 1) % 3] + I[(k + 2) % 3] - I[k])) / V)));
    if (!near(e[0] * e[1] * e[2], V, rel)) return null;
    // the mesh must fill the box: its extents along the principal axes equal e
    const ex = extents(f.positions, ax);
    if (!ex.every(([lo, hi], k) => near(hi - lo, e[k], rel * 10))) return null;
    const o: V3 = [0, 0, 0];
    ex.forEach(([lo, hi], k) => { for (let a = 0; a < 3; a++) o[a] += ax[k][a] * (lo + hi) / 2; });
    return { type: 'box', size: e.map((x) => x / 2), pose: { o, x: ax[0], y: ax[1], z: ax[2] } };
  }
  if (only('cylinder', 'plane') && (kinds.get('plane') ?? 0) === 2 && (kinds.get('cylinder') ?? 0) >= 1) {
    for (let k = 0; k < 3; k++) {
      const [lo, hi] = extents(f.positions, [ax[k]])[0];
      const L = hi - lo;
      if (!(L > 0)) continue;
      const r = Math.sqrt(V / (Math.PI * L));
      const ia = 0.5 * V * r * r, ip = V * (3 * r * r + L * L) / 12;
      if (near(I[k], ia, rel * 10) && near(I[(k + 1) % 3], ip, rel * 10) && near(I[(k + 2) % 3], ip, rel * 10)) {
        return { type: 'cylinder', size: [r, L / 2], pose: frame(k) };
      }
    }
    return null;
  }
  if (only('cylinder', 'sphere') && (kinds.get('sphere') ?? 0) === 2) {
    for (let k = 0; k < 3; k++) {
      const [lo, hi] = extents(f.positions, [ax[k]])[0];
      const T = hi - lo;
      // V = π r² (T - 2r) + 4/3 π r³ -> solve for r by bisection on (0, T/2)
      let a = 0, b = T / 2;
      for (let it = 0; it < 80; it++) { const m = (a + b) / 2; const v = Math.PI * m * m * (T - 2 * m) + (4 / 3) * Math.PI * m ** 3; if (v < V) a = m; else b = m; }
      const r = (a + b) / 2, L = T - 2 * r;
      if (!(L > 0)) continue;
      // moments of a capsule about its axis: cylinder ½ m r² + hemispheres 2/5 m r²
      const mc = Math.PI * r * r * L, ms = (4 / 3) * Math.PI * r ** 3;
      const ia = 0.5 * mc * r * r + 0.4 * ms * r * r;
      if (!near(I[k], ia, rel * 100)) continue;
      return { type: 'capsule', size: [r, L / 2], pose: frame(k) };
    }
    return null;
  }
  return null;
}
