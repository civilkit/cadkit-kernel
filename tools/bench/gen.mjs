// Authoring helper for tools/bench/benchcad/*.ckpart.json: the 10 BenchCAD
// parts translated to our feature tree by hand (parameters paraphrased from
// the dataset's CadQuery programs, no source copied).
//
// Frame: BenchCAD is Z-up CadQuery; our world is Y-up. Ground truth is
// rotated (x,y,z)_cq -> (x,z,-y) when compared, which maps CQ XY/XZ/YZ
// workplanes exactly onto our top/front/right planes with the same local
// (x,y) and the same normal. So sketch coordinates below are the CQ local ones.
//
// usage: node tools/bench/gen.mjs   (rewrites tools/bench/benchcad/*.ckpart.json)
import { writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.join(here, 'benchcad');
mkdirSync(out, { recursive: true });

class Sk {
  constructor(id, plane) { this.id = id; this.plane = plane; this.points = []; this.entities = []; this.np = 0; this.ne = 0; }
  p(x, y) { const id = `p${++this.np}`; this.points.push({ id, x, y }); return id; }
  line(a, b, o = {}) { const id = `e${++this.ne}`; this.entities.push({ type: 'line', id, p1: a, p2: b, ...o }); return id; }
  circle(cx, cy, r) { const id = `e${++this.ne}`; this.entities.push({ type: 'circle', id, c: this.p(cx, cy), r }); return id; }
  arc(c, a, b) { const id = `e${++this.ne}`; this.entities.push({ type: 'arc', id, c, p1: a, p2: b }); return id; }
  poly(pts) { const ids = pts.map(([x, y]) => this.p(x, y)); ids.forEach((a, i) => this.line(a, ids[(i + 1) % ids.length])); return this; }
  rect(x0, y0, x1, y1) { return this.poly([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]); }
  axis(x0, y0, x1, y1) { this.line(this.p(x0, y0), this.p(x1, y1), { construction: true }); return `e${this.ne}`; }
  json() { return { type: 'sketch', id: this.id, plane: this.plane, points: this.points, entities: this.entities, constraints: [] }; }
}
const STD = (id) => ({ kind: 'std', id });
const FEAT = (id) => ({ kind: 'feature', id });
const FACE = (id) => ({ kind: 'face', id });
const plane = (id, base, offset) => ({ type: 'plane', id, mode: 'offset', base: STD(base), offset });
const ext = (id, sketch, o = {}) => ({ type: 'extrude', id, op: 'add', sketch, end: 'blind', depth: 10, ...o });
const edge = (f1, f2, at) => ({ faces: [f1, f2], at });
const part = (name, features) => ({ format: 'ck-part', version: 1, name, features });

const parts = {};

// 1. round_flange: centred disc + through bore
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 102.63);
  const b = new Sk('F3', STD('top')); b.circle(0, 0, 17.54);
  parts.round_flange_000341 = part('round_flange', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 25.66 }),
    b.json(), ext('F4', 'F3', { op: 'cut', end: 'throughBoth' })]);
}
// 2. pulley: closed polyline revolved about the local y axis
{
  const a = new Sk('F1', STD('right'));
  a.poly([[17.5, -12.6], [31.5, -12.6], [62.4, -12.6], [62.4, -5], [70, -5], [70, 5], [62.4, 5], [62.4, 12.6], [31.5, 12.6], [17.5, 12.6]]);
  a.axis(0, 0, 0, 10);
  parts.pulley_000030 = part('pulley', [a.json(), { type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: `e${a.ne}`, angle: 360 }]);
}
// 3. washer: disc, bore, chamfer on both rims of the start face
{
  const a = new Sk('F1', STD('front')); a.circle(0, 0, 32.4);
  const b = new Sk('F3', STD('front')); b.circle(0, 0, 18.5);
  parts.washer_000001 = part('washer', [a.json(), ext('F2', 'F1', { depth: 5 }), b.json(), ext('F4', 'F3', { op: 'cut', end: 'throughBoth' }),
    { type: 'chamfer', id: 'F5', chamferType: 'equal', distance: 0.4, edges: [edge('F2.C0', 'F2.S.e1', [32.4, 0, 0]), edge('F2.C0', 'F4.S.e1', [18.5, 0, 0])] }]);
}
// 4. fan_shroud: square plate, big round cut-out, four corner holes
{
  const a = new Sk('F1', STD('front')); a.rect(-79.9, -79.9, 79.9, 79.9);
  const b = new Sk('F3', STD('front')); b.circle(0, 0, 57.7);
  const c = new Sk('F5', STD('front'));
  for (const [x, y] of [[68.8, 68.8], [-68.8, 68.8], [68.8, -68.8], [-68.8, -68.8]]) c.circle(x, y, 1.615);
  parts.fan_shroud_000001 = part('fan_shroud', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 9.69 }),
    b.json(), ext('F4', 'F3', { op: 'cut', end: 'throughBoth' }), c.json(), ext('F6', 'F5', { op: 'cut', end: 'throughBoth' })]);
}
// 5. enclosure: box, shell open on one face, fillet the closed-face rim
{
  const a = new Sk('F1', STD('front')); a.rect(-96.435, -77.635, 96.435, 77.635);
  const z = 19.82;
  parts.enclosure_000042 = part('enclosure', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 39.64 }),
    { type: 'shell', id: 'F3', faces: ['F2.C0'], thickness: 1.8 },
    { type: 'fillet', id: 'F4', radius: 0.68, edges: [edge('F2.C1~2', 'F2.S.e1~1', [0, -77.635, z]), edge('F2.C1~2', 'F2.S.e2~2', [96.435, 0, z]),
      edge('F2.C1~2', 'F2.S.e3~2', [0, 77.635, z]), edge('F2.C1~2', 'F2.S.e4~1', [-96.435, 0, z])] }]);
}
// 6. heat_sink: plate, 7 fins (one sketch), chamfer fin tops, 4 holes
{
  const a = new Sk('F1', STD('right')); a.rect(-29.42, -42.655, 29.42, 42.655);
  const pl = plane('F3', 'right', 1.85);
  const b = new Sk('F4', FEAT('F3'));
  for (let i = -3; i <= 3; i++) b.rect(-28.42, 12.95 * i - 1.85, 28.42, 12.95 * i + 1.85);
  // fin k (1..7): cap F5.C1~k, side faces F5.S.e(4k-3..4k); rim edge mids (our frame: sketch x -> -z)
  const finTop = [];
  for (let k = 1; k <= 7; k++) {
    const yc = 12.95 * (k - 4), c = `F5.C1~${k}`, s = (j) => `F5.S.e${4 * k - 4 + j}`;
    finTop.push(edge(c, s(1), [41.21, yc - 1.85, 0]), edge(c, s(2), [41.21, yc, -28.42]), edge(c, s(3), [41.21, yc + 1.85, 0]), edge(c, s(4), [41.21, yc, 28.42]));
  }
  const c = new Sk('F7', STD('right'));
  for (const [x, y] of [[-5.12, -36.455], [-5.12, 36.455], [5.12, -36.455], [5.12, 36.455]]) c.circle(x, y, 1.35);
  parts.heat_sink_000029 = part('heat_sink', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 3.7 }), pl, b.json(), ext('F5', 'F4', { depth: 39.36 }),
    { type: 'chamfer', id: 'F6', chamferType: 'equal', distance: 0.8, edges: finTop },
    c.json(), ext('F8', 'F7', { op: 'cut', end: 'throughBoth' })]);
}
// 7. nozzle: revolved cone tube + flange + bore + 6-hole circular pattern
{
  const a = new Sk('F1', STD('top')); a.poly([[0, 16.19], [60.53, 5.18], [60.53, 8.05], [0, 19.06]]); a.axis(0, 0, 10, 0);
  const pl1 = plane('F2', 'right', -7.09);
  const b = new Sk('F3', FEAT('F2')); b.circle(0, 0, 22.99);
  const pl2 = plane('F5', 'right', -7.59);
  const c = new Sk('F6', FEAT('F5')); c.circle(0, 0, 16.19);
  const pl3 = plane('F8', 'right', -8.09);
  const d = new Sk('F9', FEAT('F8')); d.circle(21.025, 0, 1.3);
  parts.nozzle_000069 = part('nozzle', [a.json(), { type: 'revolve', id: 'F10', op: 'add', sketch: 'F1', axis: `e${a.ne}`, angle: 360 },
    pl1, b.json(), ext('F4', 'F3', { depth: 7.09 }), pl2, c.json(), ext('F7', 'F6', { op: 'cut', depth: 8.09, flip: true }),
    pl3, d.json(), ext('F11', 'F9', { op: 'cut', depth: 8.09, flip: true }),
    { type: 'cpattern', id: 'F12', seeds: ['F11'], axis: { kind: 'axis', axis: 'x' }, count: 6, angle: 360, equal: true }]);
}
// 8. worm_screw: shaft + trapezoid thread swept along a helix
{
  const pl = plane('F1', 'top', -18.01);
  const a = new Sk('F2', FEAT('F1')); a.circle(0, 0, 8.4);
  const h = new Sk('F4', STD('top')); h.circle(0, 0, 8.4);
  const t = new Sk('F6', STD('front')); t.poly([[8.4, -3.667], [15.0, -1.264], [15.0, 1.264], [8.4, 3.667]]);
  parts.worm_screw_000021 = part('worm_screw', [pl, a.json(), ext('F3', 'F2', { depth: 72.04 }),
    h.json(), { type: 'helix', id: 'F5', sketch: 'F4', definedBy: 'heightPitch', pitch: 9.425, height: 36.02, revolutions: 3.8, startAngle: 0 },
    t.json(), { type: 'sweep', id: 'F7', op: 'add', sketch: 'F6', path: { kind: 'curve', id: 'F5' } }]);
}
// 9. tapered_boss: loft between two circles, round base, bore, rim chamfers
{
  const pl = plane('F2', 'top', 38.45);
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 34.695);
  const b = new Sk('F3', FEAT('F2')); b.circle(0, 0, 14.5);
  const c = new Sk('F5', STD('top')); c.circle(0, 0, 50.855);
  const d = new Sk('F7', STD('top')); d.circle(0, 0, 6.34);
  parts.tapered_boss_000256 = part('tapered_boss', [a.json(), pl, b.json(), { type: 'loft', id: 'F4', op: 'add', sketches: ['F1', 'F3'] },
    c.json(), ext('F6', 'F5', { depth: 8.64, flip: true }), d.json(), ext('F8', 'F7', { op: 'cut', end: 'throughBoth' }),
    { type: 'chamfer', id: 'F9', chamferType: 'equal', distance: 1.45, edges: [
      edge('F4.C1', 'F4.S.e1~1', [0, 38.45, -14.5]), edge('F4.C1', 'F4.S.e1~2', [0, 38.45, 14.5]), edge('F4.C1', 'F8.S.e1', [4.622, 38.45, -4.34])] }]);
}
// 10. twisted_drill: fluted section, twist-extruded 471.6 deg (UNSUPPORTED:
// extruded straight here), conical tip cut
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 2.4156);
  // flute loop: three 3-point arcs. [p1, mid, p2] -> centre + ccw endpoints
  const arc3 = (s, A, M, Bp) => {
    const [ax, ay] = A, [bx, by] = M, [cx, cy] = Bp;
    const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
    const cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx); // >0: A->M->B is ccw
    return { c: [ux, uy], ccw: cross > 0 };
  };
  const loop = (s, rot) => {
    const R = ([x, y]) => { const t = (rot * Math.PI) / 180; return [x * Math.cos(t) - y * Math.sin(t), x * Math.sin(t) + y * Math.cos(t)]; };
    const P = [[-1.67498, 1.77427], [-1.08635, 0.78112], [0, 0.3904], [1.12411, 0.6414], [2.03468, 1.34673], [0.27936, 2.42395]].map(R);
    const ids = { s0: s.p(...P[0]), s2: s.p(...P[2]), s4: s.p(...P[4]) };
    const segs = [[P[0], P[1], P[2], ids.s0, ids.s2], [P[2], P[3], P[4], ids.s2, ids.s4], [P[4], P[5], P[0], ids.s4, ids.s0]];
    for (const [A, M, Bp, ia, ib] of segs) {
      const { c, ccw } = arc3(s, A, M, Bp);
      const cid = s.p(...c);
      ccw ? s.arc(cid, ia, ib) : s.arc(cid, ib, ia);
    }
  };
  const f = new Sk('F3', STD('top')); loop(f, 0); loop(f, 180);
  const r = new Sk('F5', STD('front'));
  r.poly([[2.44, 33.734], [2.44, 35.3], [0, 35.3], [0, 35.2], [0.05, 35.2]]); r.axis(0, 0, 0, 10);
  parts.twisted_drill_000546 = part('twisted_drill', [a.json(), ext('F2', 'F1', { depth: 35.2 }), f.json(), ext('F4', 'F3', { op: 'cut', end: 'throughBoth' }),
    r.json(), { type: 'revolve', id: 'F6', op: 'cut', sketch: 'F5', axis: `e${r.ne}`, angle: 360 }]);
}

for (const [id, p] of Object.entries(parts)) writeFileSync(path.join(out, `${id}.ckpart.json`), JSON.stringify(p, null, 1) + '\n');
console.log(`wrote ${Object.keys(parts).length} parts`);
