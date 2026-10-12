// Shared sketch/feature helpers for the BenchCAD translators (gen.mjs, gen2.mjs).
export class Sk {
  constructor(id, plane) { this.id = id; this.plane = plane; this.points = []; this.entities = []; this.np = 0; this.ne = 0; }
  p(x, y) { const id = `p${++this.np}`; this.points.push({ id, x, y }); return id; }
  line(a, b, o = {}) { const id = `e${++this.ne}`; this.entities.push({ type: 'line', id, p1: a, p2: b, ...o }); return id; }
  circle(cx, cy, r) { const id = `e${++this.ne}`; this.entities.push({ type: 'circle', id, c: this.p(cx, cy), r }); return id; }
  arc(c, a, b) { const id = `e${++this.ne}`; this.entities.push({ type: 'arc', id, c, p1: a, p2: b }); return id; }
  poly(pts) { const ids = pts.map(([x, y]) => this.p(x, y)); ids.forEach((a, i) => this.line(a, ids[(i + 1) % ids.length])); return this; }
  rect(x0, y0, x1, y1) { return this.poly([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]); }
  axis(x0, y0, x1, y1) { this.line(this.p(x0, y0), this.p(x1, y1), { construction: true }); return `e${this.ne}`; }
  // cached point: the same coordinates give the same point id (closed arc/line chains)
  P(x, y) { this.cache ??= new Map(); const k = `${x.toFixed(7)},${y.toFixed(7)}`; if (!this.cache.has(k)) this.cache.set(k, this.p(x, y)); return this.cache.get(k); }
  // CadQuery-style threePointArc a -> m -> b as one of our ccw (centre, p1, p2) arcs
  arc3(A, M, B) {
    const [ax, ay] = A, [bx, by] = M, [cx, cy] = B;
    const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
    const ccw = (bx - ax) * (cy - by) - (by - ay) * (cx - bx) > 0; // A->M->B counter-clockwise
    const c = this.p(ux, uy), a = this.P(...A), b = this.P(...B);
    return ccw ? this.arc(c, a, b) : this.arc(c, b, a);
  }
  // closed polyline over cached points, dropping a repeated closing vertex
  loop(pts) { if (pts.length > 1 && pts[0][0] === pts.at(-1)[0] && pts[0][1] === pts.at(-1)[1]) pts = pts.slice(0, -1); const ids = pts.map(([x, y]) => this.P(x, y)); ids.forEach((a, i) => this.line(a, ids[(i + 1) % ids.length])); return this; }
  json() { return { type: 'sketch', id: this.id, plane: this.plane, points: this.points, entities: this.entities, constraints: [] }; }
}
export const STD = (id) => ({ kind: 'std', id });
export const FEAT = (id) => ({ kind: 'feature', id });
export const FACE = (id) => ({ kind: 'face', id });
export const plane = (id, base, offset) => ({ type: 'plane', id, mode: 'offset', base: STD(base), offset });
export const ext = (id, sketch, o = {}) => ({ type: 'extrude', id, op: 'add', sketch, end: 'blind', depth: 10, ...o });
export const edge = (f1, f2, at) => ({ faces: [f1, f2], at });
export const part = (name, features) => ({ format: 'ck-part', version: 1, name, features });

