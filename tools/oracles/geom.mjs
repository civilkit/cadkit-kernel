// Closed-form building blocks for the independent geometry oracles
// (tools/oracles/index.mjs). Nothing here touches the kernel or its mesh:
// every number comes from textbook formulas (Green's theorem on polygons,
// Pappus for revolves, box/cylinder primitives, exact moment bookkeeping).
//
// A solid is { V, M:[mx,my,mz] (first moment, V*com), A (surface area|null),
// bbox:{min,max}, exact:bool (bbox tight for the true B-rep) }.
export const PI = Math.PI;

// Sketch plane -> world maps used by the .ckpart planes (verified against the
// plane definitions, not the kernel output): point = u*x + v*y + n*t.
export const PLANES = {
  top: { u: [1, 0, 0], v: [0, 0, -1], n: [0, 1, 0] },
  front: { u: [1, 0, 0], v: [0, 1, 0], n: [0, 0, 1] },
  right: { u: [0, 0, -1], v: [0, 1, 0], n: [1, 0, 0] },
};
const mapPt = (pl, x, y, t) => [0, 1, 2].map((i) => pl.u[i] * x + pl.v[i] * y + pl.n[i] * t);

// ---- 2D loops: { A, cx, cy, P, bb:[x0,y0,x1,y1] }
export function poly(pts) {
  let A = 0, Mx = 0, My = 0, P = 0;
  for (let i = 0; i < pts.length; i++) {
    const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % pts.length];
    const c = x1 * y2 - x2 * y1;
    A += c / 2; Mx += ((x1 + x2) * c) / 6; My += ((y1 + y2) * c) / 6; P += Math.hypot(x2 - x1, y2 - y1);
  }
  const s = A < 0 ? -1 : 1;
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  return { A: s * A, cx: Mx / A, cy: My / A, P, bb: [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)] };
}
export const rectL = (x0, y0, x1, y1) => poly([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]);
export const circL = (cx, cy, r) => ({ A: PI * r * r, cx, cy, P: 2 * PI * r, bb: [cx - r, cy - r, cx + r, cy + r], curved: true });
// upper half disc of radius r centred at (cx,cy) on its diameter
export const halfDiscL = (cx, cy, r) => ({ A: (PI * r * r) / 2, cx, cy: cy + (4 * r) / (3 * PI), P: PI * r + 2 * r, bb: [cx - r, cy, cx + r, cy + r], curved: true });

// ---- prism: loops = [[loop, +1|-1], ...] extruded along the plane normal t0..t1
export function prism(plane, loops, t0, t1) {
  const pl = PLANES[plane];
  let A = 0, Mx = 0, My = 0, P = 0, curved = false;
  for (const [l, s] of loops) { A += s * l.A; Mx += s * l.A * l.cx; My += s * l.A * l.cy; P += l.P; curved ||= !!l.curved; }
  const d = t1 - t0, V = A * d;
  const com = mapPt(pl, Mx / A, My / A, (t0 + t1) / 2);
  const bbs = loops.filter(([, sg]) => sg > 0).map(([l]) => l.bb);
  const x0 = Math.min(...bbs.map((b) => b[0])), y0 = Math.min(...bbs.map((b) => b[1]));
  const x1 = Math.max(...bbs.map((b) => b[2])), y1 = Math.max(...bbs.map((b) => b[3]));
  const cs = [[x0, y0, t0], [x1, y1, t1]].map((c) => mapPt(pl, ...c));
  return {
    V, M: com.map((c) => c * V), A: 2 * A + P * Math.abs(d),
    bbox: { min: [0, 1, 2].map((i) => Math.min(cs[0][i], cs[1][i])), max: [0, 1, 2].map((i) => Math.max(cs[0][i], cs[1][i])) },
    exact: !loops.some(([l]) => l.curved),
  };
}

// ---- axis-aligned box and Y-axis cylinder primitives (world coordinates)
export function boxS(min, max) {
  const d = [0, 1, 2].map((i) => max[i] - min[i]);
  const V = d[0] * d[1] * d[2];
  return { V, M: [0, 1, 2].map((i) => (V * (min[i] + max[i])) / 2), A: 2 * (d[0] * d[1] + d[1] * d[2] + d[0] * d[2]), bbox: { min, max }, exact: true };
}
export function cylY(cx, cz, r, y0, y1) {
  const h = y1 - y0, V = PI * r * r * h;
  return { V, M: [V * cx, (V * (y0 + y1)) / 2, V * cz], A: 2 * PI * r * h + 2 * PI * r * r, bbox: { min: [cx - r, y0, cz - r], max: [cx + r, y1, cz + r] }, exact: false };
}

// ---- bookkeeping: base minus/plus pieces, with an explicit area delta
export function csg(base, ops, dA = 0) {
  let V = base.V, M = [...base.M];
  for (const [sign, p] of ops) { V += sign * p.V; for (let i = 0; i < 3; i++) M[i] += sign * p.M[i]; }
  // bbox grows to hull any added piece (cuts never shrink it in our cases)
  const bbox = { min: [...base.bbox.min], max: [...base.bbox.max] };
  for (const [sign, p] of ops) if (sign > 0 && p.bbox) for (let i = 0; i < 3; i++) { bbox.min[i] = Math.min(bbox.min[i], p.bbox.min[i]); bbox.max[i] = Math.max(bbox.max[i], p.bbox.max[i]); }
  return { ...base, V, M, A: base.A == null ? null : base.A + dA, bbox };
}
// disjoint union (separate lumps): bbox is the hull, area adds
export function lump(a, b) {
  return {
    V: a.V + b.V, M: a.M.map((m, i) => m + b.M[i]), A: a.A + b.A,
    bbox: { min: a.bbox.min.map((x, i) => Math.min(x, b.bbox.min[i])), max: a.bbox.max.map((x, i) => Math.max(x, b.bbox.max[i])) },
    exact: a.exact && b.exact,
  };
}

// ---- revolve of a polygon profile (r,y) about the Y axis. Sweep from +x
// toward -z: point(phi) = (r cos phi, y, -r sin phi), phi in [phi0, phi0+theta].
export function revolvePoly(pts, theta, mode = 'start') {
  const phi0 = mode === 'flip' ? -theta : mode === 'mid' ? -theta / 2 : 0, phi1 = phi0 + theta;
  let A = 0, I1 = 0, I2 = 0, Iy = 0, lat = 0;
  const n = pts.length;
  const s = pts.reduce((a, p, i) => a + (p[0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * p[1]), 0) < 0 ? -1 : 1;
  for (let i = 0; i < n; i++) {
    const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % n], c = s * (x1 * y2 - x2 * y1);
    A += c / 2; I1 += ((x1 + x2) * c) / 6; I2 += ((x1 * x1 + x1 * x2 + x2 * x2) * c) / 12;
    Iy += ((x1 * y2 + 2 * x1 * y1 + 2 * x2 * y2 + x2 * y1) * c) / 24;
    lat += theta * Math.hypot(x2 - x1, y2 - y1) * ((x1 + x2) / 2);
  }
  const V = theta * I1;
  const full = Math.abs(theta - 2 * PI) < 1e-12;
  const com = [(I2 * (Math.sin(phi1) - Math.sin(phi0))) / V, Iy / I1, (I2 * (Math.cos(phi1) - Math.cos(phi0))) / V];
  // exact bbox: extremes at profile vertices, at the sweep ends and at multiples of pi/2 inside the sweep
  const phis = [phi0, phi1];
  for (let k = Math.ceil(phi0 / (PI / 2)); k * (PI / 2) <= phi1 + 1e-12; k++) phis.push(k * (PI / 2));
  const rs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const xs = [], zs = [];
  for (const r of rs) for (const f of phis) { xs.push(r * Math.cos(f)); zs.push(-r * Math.sin(f)); }
  if (full) { /* closed ring: same candidate set covers it */ }
  return {
    V, M: com.map((c) => c * V), A: lat + (full ? 0 : 2 * A),
    bbox: { min: [Math.min(...xs), Math.min(...ys), Math.min(...zs)], max: [Math.max(...xs), Math.max(...ys), Math.max(...zs)] },
    exact: false, // curved surfaces: kernel box is mesh-sampled
  };
}

// ---- edge treatments on a straight convex 90-degree box edge.
// mid: edge midpoint; a,b: unit vectors from the edge into the two faces.
export function edgeCut(kind, r, len, mid, a, b) {
  const c = kind === 'fillet' ? (1 - PI / 4) * r * r : (r * r) / 2;
  const f = kind === 'fillet' ? (5 / 6 - PI / 4) / (1 - PI / 4) : 1 / 3;
  const V = c * len, com = [0, 1, 2].map((i) => mid[i] + (a[i] + b[i]) * f * r);
  // two long faces lose r strips, the new face is added, both end caps lose the removed cross-section
  const dA = len * (kind === 'fillet' ? (PI / 2) * r - 2 * r : Math.SQRT2 * r - 2 * r) - 2 * c;
  return { piece: { V, M: com.map((x) => x * V) }, dA };
}

// Simpson on [0,h] (exact up to cubics) for loft/draft sections.
export function simpson(f, h, n = 4) {
  let s = f(0) + f(h);
  for (let i = 1; i < n; i++) s += (i % 2 ? 4 : 2) * f((i * h) / n);
  return (s * h) / (3 * n);
}
