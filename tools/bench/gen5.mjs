// Authoring helper for the fourth BenchCAD batch (10 parts, families n-z): tools/bench/benchcad/*.ckpart.json
// translated by hand into our feature tree from the dataset's CadQuery programs. Parameters are paraphrased;
// no CadQuery source is copied. The reference figures in manifest.json come from running the dataset programs in
// CadQuery (see FINDINGS.md "Batch 4"), never from this script or from our kernel.
//
// Frame (see gen.mjs / gen3.mjs): BenchCAD is Z-up CadQuery, ground truth is rotated (x,y,z)->(x,z,-y), which maps
// CQ XY / XZ / YZ workplanes onto our top / front / right planes with the same local (x,y) and the same normal
// (top normal = +Z_cq, front normal = -Y_cq, right normal = +X_cq). Sketch coordinates below are the CQ local
// ones; `W` maps a CQ point to our world for edge `at` hints.
//
// usage: node tools/bench/gen5.mjs
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Sk, STD, FEAT, plane, ext, edge, part } from './lib.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.join(here, 'benchcad');
const rad = (d) => (d * Math.PI) / 180;
const revolve = (id, sketch, ax, o = {}) => ({ type: 'revolve', id, op: 'add', sketch, axis: ax, angle: 360, ...o });
const cut = (id, sketch, o = {}) => ext(id, sketch, { op: 'cut', ...o });
const thru = { end: 'throughBoth' };

const parts = {};

// 1. ratchet_sector: sector outline (two straight sides and a three-point arc) extruded, centre bore, four small
//    holes along an arc, three rotated rectangular slots cut half way into the front face
{
  const A = [0, 0], B = [99.61, 0], M = [59.5294, 79.8649], C = [-28.4574, 95.4585];
  const a = new Sk('F1', STD('right'));
  a.line(a.P(...A), a.P(...B)); a.arc3(B, M, C); a.line(a.P(...C), a.P(...A));
  const h = new Sk('F3', STD('right')); h.circle(0, 0, 15.29);
  const g = new Sk('F5', STD('right'));
  for (const [x, y] of [[63.0993, 24.6268], [49.8274, 45.8829], [29.7355, 60.8589], [5.5737, 67.5051]]) g.circle(x, y, 1.97);
  const slot = (id, cx, cy, deg) => {
    const s = new Sk(id, STD('right')), t = rad(deg), hx = 22.41225, hy = 4.06;
    const R = ([x, y]) => [cx + x * Math.cos(t) - y * Math.sin(t), cy + x * Math.sin(t) + y * Math.cos(t)];
    s.poly([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]].map(R));
    return s;
  };
  const s1 = slot('F7', 40.0625, 20.1055, 26.65), s2 = slot('F9', 26.7882, 35.9392, 53.3), s3 = slot('F11', 7.8222, 44.1367, 79.95);
  parts.ratchet_sector_001018 = part('ratchet_sector', [a.json(), ext('F2', 'F1', { depth: 17.44 }),
    h.json(), cut('F4', 'F3', thru), g.json(), cut('F6', 'F5', thru),
    s1.json(), cut('F8', 'F7', { end: 'mid', depth: 18.44 }), s2.json(), cut('F10', 'F9', { end: 'mid', depth: 18.44 }),
    s3.json(), cut('F12', 'F11', { end: 'mid', depth: 18.44 })]);
}

// 2. shaft_collar: short tube on the x axis (the dataset's inner step cylinder lies fully inside the wall, so it
//    changes nothing), rim chamfers on the start face, one cross hole through the wall
{
  const a = new Sk('F1', STD('right')); a.circle(0, 0, 11.57);
  const b = new Sk('F3', STD('right')); b.circle(0, 0, 6.61);
  const c = new Sk('F6', STD('front')); c.circle(4.96, 0, 0.87);
  parts.shaft_collar_000553 = part('shaft_collar', [a.json(), ext('F2', 'F1', { depth: 9.92 }), b.json(), cut('F4', 'F3', thru),
    // F5: chamfer, see gen5_blends.json
    c.json(), cut('F7', 'F6', thru)]);
}

// 3. pipe_flange: rectangular plate with rounded corners, four small holes that graze the bore, a round boss on
//    top, centre bore through both, chamfer on the boss rims
{
  const a = new Sk('F1', STD('top')); a.rect(-61.75, -56.83, 61.75, 56.83);
  const w = new Sk('F4', STD('top'));
  for (const [x, y] of [[-17.85, -12.93], [-17.85, 12.93], [17.85, -12.93], [17.85, 12.93]]) w.circle(x, y, 2.765);
  const pl = plane('F6', 'top', 9);
  const bs = new Sk('F7', FEAT('F6')); bs.circle(0, 0, 29.73);
  const bo = new Sk('F9', STD('top')); bo.circle(0, 0, 22.46);
  parts.pipe_flange_000214 = part('pipe_flange', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 18 }),
    // F3: fillet, see gen5_blends.json
    w.json(), cut('F5', 'F4', thru), pl, bs.json(), ext('F8', 'F7', { depth: 2.81 }), bo.json(), cut('F10', 'F9', thru)
    // F11: chamfer, see gen5_blends.json
  ]);
}

// 4. piston: stepped revolved profile with three ring grooves, a cross hole through the whole piston and a
//    chamfer on the crown rim
{
  const a = new Sk('F1', STD('top'));
  a.poly([[0, 0], [45.3, 0], [45.3, 86.9975], [43.51, 86.9975], [43.51, 89.2375], [45.3, 89.2375], [45.3, 99.555], [43.51, 99.555], [43.51, 101.795],
    [45.3, 101.795], [45.3, 112.1125], [43.51, 112.1125], [43.51, 114.3525], [45.3, 114.3525], [45.3, 125.79], [0, 125.79]]);
  const ax = a.axis(0, 0, 0, 10);
  const h = new Sk('F3', STD('top')); h.circle(0, 22.09, 12.39);
  parts.piston_002697 = part('piston', [a.json(), revolve('F2', 'F1', ax), h.json(), cut('F4', 'F3', thru)
    // F5: chamfer, see gen5_blends.json
  ]);
}

// 5. pull_handle: two legs and a cross bar (cylinders), two square foot plates with a counterbore-like blind hole
//    that also bites into the leg end
{
  const l1 = new Sk('F1', STD('right')); l1.circle(-100, 0, 10.45);
  const l2 = new Sk('F3', STD('right')); l2.circle(100, 0, 10.45);
  const br = new Sk('F5', STD('front')); br.circle(76.15, 0, 10.45);
  const p1 = new Sk('F7', STD('right')); p1.rect(-126.1, -26.1, -73.9, 26.1);
  const h1 = new Sk('F10', FEAT('F9')); h1.circle(-100, 0, 6.25);
  const p2 = new Sk('F11', STD('right')); p2.rect(73.9, -26.1, 126.1, 26.1);
  const h2 = new Sk('F14', FEAT('F13')); h2.circle(100, 0, 6.25);
  parts.pull_handle_007593 = part('pull_handle', [l1.json(), ext('F2', 'F1', { depth: 76.15 }), l2.json(), ext('F4', 'F3', { depth: 76.15 }),
    br.json(), ext('F6', 'F5', { end: 'mid', depth: 220.9 }),
    p1.json(), ext('F8', 'F7', { depth: 12.5 }), plane('F9', 'right', 6.25), h1.json(), cut('F12', 'F10', { end: 'mid', depth: 14.5 }),
    p2.json(), ext('F15', 'F11', { depth: 12.5 }), plane('F13', 'right', 6.25), h2.json(), cut('F16', 'F14', { end: 'mid', depth: 14.5 })]);
}

// 6. vented_panel: slab with a chamfered top rim and a 5 x 5 grid of round vents
{
  const a = new Sk('F1', STD('top')); a.rect(-63.805, -48.225, 63.805, 48.225);
  const v = new Sk('F4', STD('top'));
  for (let i = -2; i <= 2; i++) for (let j = -2; j <= 2; j++) v.circle(25.38 * i, 18.09 * j, 4.535);
  parts.vented_panel_003475 = part('vented_panel', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 10.35 }),
    // F3: chamfer, see gen5_blends.json
    v.json(), cut('F5', 'F4', thru)]);
}

// 7. waffle_plate: slab, chamfered rim on the back face, a 4 x 2 grid of rectangular pockets from that face,
//    four corner holes
{
  const a = new Sk('F1', STD('front')); a.rect(-39.445, -26.045, 39.445, 26.045);
  const pl = plane('F4', 'front', -10.275);
  const p = new Sk('F5', FEAT('F4'));
  for (const x of [-27.825, -9.275, 9.275, 27.825]) for (const y of [-12.385, 12.385]) p.rect(x - 5.04, y - 7.255, x + 5.04, y + 7.255);
  const h = new Sk('F7', STD('front'));
  for (const [x, y] of [[33.645, 20.245], [33.645, -20.245], [-33.645, 20.245], [-33.645, -20.245]]) h.circle(x, y, 1.51);
  parts.waffle_plate_010153 = part('waffle_plate', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 20.55 }),
    // F3: chamfer, see gen5_blends.json
    pl, p.json(), cut('F6', 'F5', { depth: 12.39, flip: true }), h.json(), cut('F8', 'F7', thru)]);
}

// 8. rect_frame: flat frame (rectangle with a window), four corner holes, fillet on every edge of the back face
{
  const a = new Sk('F1', STD('front')); a.rect(-111.89, -53.71, 111.89, 53.71);
  const w = new Sk('F3', STD('front')); w.rect(-90.41, -32.23, 90.41, 32.23);
  const h = new Sk('F5', STD('front'));
  for (const [x, y] of [[101.15, 42.97], [-101.15, 42.97], [101.15, -42.97], [-101.15, -42.97]]) h.circle(x, y, 3.58);
  parts.rect_frame_008694 = part('rect_frame', [a.json(), ext('F2', 'F1', { depth: 3.58 }), w.json(), cut('F4', 'F3', thru), h.json(), cut('F6', 'F5', thru)
    // F7: fillet, see gen5_blends.json
  ]);
}

// 9. snap_clip: thin C-shaped clip (two concentric-ish arcs closed by short ends), two small holes centred on the
//    open ends, chamfer on the outer face rim
{
  const a = new Sk('F1', STD('right'));
  const A = [2.0982, 0.6616], M1 = [-2.2, 0], B = [2.0982, -0.6616], C = [0.8583, -0.2706], M2 = [-0.9, 0], D = [0.8583, 0.2706];
  a.arc3(A, M1, B); a.line(a.P(...B), a.P(...C)); a.arc3(C, M2, D); a.line(a.P(...D), a.P(...A));
  const h = new Sk('F3', STD('right')); h.circle(1.4783, 0.4661, 0.25); h.circle(1.4783, -0.4661, 0.25);
  parts.snap_clip_000052 = part('snap_clip', [a.json(), ext('F2', 'F1', { depth: 0.39 }), h.json(), cut('F4', 'F3', thru)
    // F5: chamfer, see gen5_blends.json
  ]);
}

// 10. propeller: hub (a revolved disc + cone around the x axis, axial bore through both) and four thin blades, each a
//     loft between two ellipses on tilted planes (root on the axis, tip 59 away), spun about the axis
{
  const R0 = 20.71, hw = 6.29, rc = 18.639, rt = 1.5, ch = 8.91, bore = 5.29;
  const xb = hw + ((rc - bore) / (rc - rt)) * ch; // the cone meets the bore wall here
  const h = new Sk('F1', STD('front'));
  h.poly([[-hw, bore], [-hw, R0], [hw, R0], [hw, rc], [xb, bore]]);
  const ax = h.axis(-10, 0, 20, 0);
  const t12 = rad(12.9), t98 = rad(12.9 - 3.1), L = 59.03, xr = 14.497;
  const C = [xr, 0], T = [xr - L * Math.sin(t12), L * Math.cos(t12)];
  const g = new Sk('F3', STD('top'));
  g.line(g.p(...C), g.p(xr - 5 * Math.sin(t12), 5 * Math.cos(t12)));
  g.line(g.p(T[0] + 5 * Math.sin(t98), T[1] - 5 * Math.cos(t98)), g.p(...T));
  const root = { type: 'plane', id: 'F4', mode: 'normalCurve', curves: [{ kind: 'sketchCurve', sketch: 'F3', entity: 'e1' }] };
  const tip = { type: 'plane', id: 'F6', mode: 'normalCurve', atEnd: true, curves: [{ kind: 'sketchCurve', sketch: 'F3', entity: 'e2' }] };
  const e1 = new Sk('F5', FEAT('F4')); { const c = e1.p(0, 0); e1.entities.push({ type: 'ellipse', id: `e${++e1.ne}`, c, p: e1.p(0, 8.4), r: 1.72 }); }
  const e2 = new Sk('F7', FEAT('F6')); { const c = e2.p(0, 0); e2.entities.push({ type: 'ellipse', id: `e${++e2.ne}`, c, p: e2.p(0, 4.45), r: 0.635 }); }
  parts.propeller_000192 = part('propeller', [h.json(), revolve('F2', 'F1', ax), g.json(), root, e1.json(), tip, e2.json(),
    { type: 'loft', id: 'F8', op: 'add', sketches: ['F5', 'F7'] },
    { type: 'cpattern', id: 'F9', seeds: ['F8'], axis: { kind: 'axis', axis: 'x' }, count: 4, angle: 360, equal: true }]);
}

// ---- blend references found by probing (tools/bench/probe.mjs --edges) ----
let extra = {};
try { extra = JSON.parse(readFileSync(path.join(here, 'gen5_blends.json'), 'utf8')); } catch {}
for (const [id, list] of Object.entries(extra)) {
  const p = parts[id];
  for (const f of list) {
    const idx = f.after ? p.features.findIndex((x) => x.id === f.after) + 1 : p.features.length;
    const { after, ...feat } = f;
    p.features.splice(idx, 0, feat);
  }
}

for (const [id, p] of Object.entries(parts)) writeFileSync(path.join(out, `${id}.ckpart.json`), JSON.stringify(p, null, 1) + '\n');
console.log(`wrote ${Object.keys(parts).length} parts`);
