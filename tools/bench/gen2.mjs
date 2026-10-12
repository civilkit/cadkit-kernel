// Authoring helper for the second BenchCAD batch (20 parts): tools/bench/benchcad/*.ckpart.json
// translated by hand into our feature tree from the dataset's CadQuery programs. Parameters are
// paraphrased; no CadQuery source is copied. The long point lists of the gear / bellows / venturi
// profiles (plain coordinates) are kept in gen2_data.json.
//
// Frame (see gen.mjs): BenchCAD is Z-up CadQuery, ground truth is rotated (x,y,z)->(x,z,-y), which
// maps CQ XY / XZ / YZ workplanes onto our top / front / right planes with the same local (x,y) and
// the same normal (front normal = -Y_cq, right normal = +X_cq, top normal = +Z_cq).
//
// usage: node tools/bench/gen2.mjs
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Sk, STD, FEAT, plane, ext, edge, part } from './lib.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.join(here, 'benchcad');
const D = JSON.parse(readFileSync(path.join(here, 'gen2_data.json'), 'utf8'));
const W = (x, y, z) => [x, z, -y]; // CQ world -> our world (for edge `at` hints)
const rad = (d) => (d * Math.PI) / 180;
const hex = (R) => [0, 1, 2, 3, 4, 5].map((k) => [R * Math.cos(rad(60 * k)), R * Math.sin(rad(60 * k))]);
const revolve = (id, sketch, ax, o = {}) => ({ type: 'revolve', id, op: 'add', sketch, axis: ax, angle: 360, ...o });
const cut = (id, sketch, o = {}) => ext(id, sketch, { op: 'cut', ...o });
const thru = { end: 'throughBoth' };
// NOTE extrude `flip`: add extrudes along the plane normal (flip = against it); a CUT runs against the
// normal by default (into the material behind the plane) and flip sends it along the normal.
const ring = (R, n, a0 = 0) => Array.from({ length: n }, (_, i) => [R * Math.cos(rad(a0 + (360 * i) / n)), R * Math.sin(rad(a0 + (360 * i) / n))]);

const parts = {};

// 1. hex_nut: hexagon prism (6 mm across corners 6.35) + through bore
{
  const a = new Sk('F1', STD('top')).poly(hex(3.175));
  const b = new Sk('F3', STD('top')); b.circle(0, 0, 1.5);
  parts.hex_nut_000006 = part('hex_nut', [a.json(), ext('F2', 'F1', { depth: 2.4 }), b.json(), cut('F4', 'F3', thru)]);
}

// 2. bolt: round shank 110 + hex head 10, chamfered top rim
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 8);
  const pl = plane('F3', 'top', 110);
  const h = new Sk('F4', FEAT('F3')).poly(hex(13.855));
  const hv = hex(13.855);
  const ed = hv.map(([x, y], k) => { const [x2, y2] = hv[(k + 1) % 6]; return edge('F5.C1', `F5.S.e${k + 1}`, W((x + x2) / 2, (y + y2) / 2, 120)); });
  parts.bolt_000008 = part('bolt', [a.json(), ext('F2', 'F1', { depth: 110 }), pl, h.json(), ext('F5', 'F4', { depth: 10 }),
    { type: 'chamfer', id: 'F6', chamferType: 'equal', distance: 1.5, edges: ed }]);
}

// 3. coil_spring: three constant-pitch helix sweeps (closed ends pitch 6 x2 turns, active pitch 20.5 x3 turns)
//    standing in for the dataset's piecewise spline path; flat-ground ends by two box cuts.
{
  const R = 55.59, r = 3.47;
  const segs = [[0, 6, 12], [12, 20.5, 61.5], [73.5, 6, 12]];
  const feats = [];
  let n = 1;
  segs.forEach(([z0, pitch, height], i) => {
    const base = z0 === 0 ? STD('top') : FEAT(`P${i}`);
    if (z0 !== 0) feats.push(plane(`P${i}`, 'top', z0));
    const hs = new Sk(`H${i}`, base); hs.circle(0, 0, R);
    const ps = new Sk(`S${i}`, STD('front')); ps.circle(R, z0, r);
    feats.push(hs.json(), { type: 'helix', id: `X${i}`, sketch: `H${i}`, definedBy: 'heightPitch', pitch, height, revolutions: height / pitch, startAngle: 0 },
      ps.json(), { type: 'sweep', id: `W${i}`, op: 'add', sketch: `S${i}`, path: { kind: 'curve', id: `X${i}` } });
  });
  const c1 = new Sk('C1', FEAT('PT')); c1.rect(-73.825, -73.825, 73.825, 73.825);
  const c2 = new Sk('C2', FEAT('PB')); c2.rect(-73.825, -73.825, 73.825, 73.825);
  feats.push(plane('PT', 'top', 85.5), c1.json(), cut('K1', 'C1', { depth: 7.88, flip: true }), plane('PB', 'top', -7.88), c2.json(), cut('K2', 'C2', { depth: 7.88, flip: true }));
  parts.coil_spring_000026 = part('coil_spring', feats);
}

// 4. torsion_spring: 12-turn helix sweep + two straight legs. The dataset tilts the leg workplanes by
//    2.8 deg about X (plane mode "angle" through the X axis); leg 1 starts at the helix end, so its plane is the
//    tilted one offset along its own normal to pass through that point.
{
  const R = 56.25, r = 3.52;
  const hs = new Sk('H', STD('top')); hs.circle(0, 0, R);
  const ps = new Sk('S', STD('front')); ps.circle(R, 0, r);
  const th = 2.8 * Math.PI / 180;
  const tilt = { type: 'plane', id: 'PT', mode: 'angle', base: STD('front'), axis: { kind: 'axis', axis: 'x' }, angle: 2.8 };
  const up = { type: 'plane', id: 'PU', mode: 'offset', base: FEAT('PT'), offset: -206.84 * Math.sin(th) };
  const l1 = new Sk('L1', FEAT('PU')); l1.circle(R, 206.84 * Math.cos(th), r);
  const l2 = new Sk('L2', FEAT('PT')); l2.circle(R, 0, r);
  parts.torsion_spring_000054 = part('torsion_spring', [hs.json(), { type: 'helix', id: 'X', sketch: 'H', definedBy: 'heightPitch', pitch: 17.24, height: 206.84, revolutions: 12, startAngle: 0 },
    ps.json(), { type: 'sweep', id: 'W', op: 'add', sketch: 'S', path: { kind: 'curve', id: 'X' } },
    tilt, up, l1.json(), ext('E1', 'L1', { depth: 220.5, flip: true }), l2.json(), ext('E2', 'L2', { depth: 162.87 })]);
}

// path in the XZ (front) plane: up L1, quarter arc radius ra, then along +X
function elbowPath(id, L1, ra, ex) {
  const s = new Sk(id, STD('front'));
  const a = s.p(0, 0), b = s.p(0, L1), c = s.p(ra, L1 + ra), d = s.p(ex, L1 + ra), cen = s.p(ra, L1);
  s.line(a, b); s.arc(cen, c, b); s.line(c, d);
  return s;
}
// 5. pipe_elbow: circular profile swept along line-arc-line, minus a smaller swept bore
{
  const p = elbowPath('F1', 37.16, 42.38, 79.54);
  const a = new Sk('F2', STD('top')); a.circle(0, 0, 18.58);
  const b = new Sk('F4', STD('top')); b.circle(0, 0, 14.82);
  parts.pipe_elbow_000012 = part('pipe_elbow', [p.json(), a.json(), { type: 'sweep', id: 'F3', op: 'add', sketch: 'F2', path: { kind: 'sketch', id: 'F1' } },
    b.json(), { type: 'sweep', id: 'F5', op: 'cut', sketch: 'F4', path: { kind: 'sketch', id: 'F1' } }]);
}

// 6. duct_elbow: rectangular tube sweep + inner cut + a small block fused on the outside wall
{
  const p = elbowPath('F1', 38.94, 108.47, 145.13);
  const a = new Sk('F2', STD('top')); a.rect(-23.05, -25.055, 23.05, 25.055);
  const b = new Sk('F4', STD('top')); b.rect(-19.9, -21.855, 19.9, 21.855);
  const t = new Sk('F7', FEAT('F6')); t.rect(23.05, -17.5385, 27.71, 17.5385);
  parts.duct_elbow_000181 = part('duct_elbow', [p.json(), a.json(), { type: 'sweep', id: 'F3', op: 'add', sketch: 'F2', path: { kind: 'sketch', id: 'F1' } },
    b.json(), { type: 'sweep', id: 'F5', op: 'cut', sketch: 'F4', path: { kind: 'sketch', id: 'F1' } },
    plane('F6', 'top', 11.445), t.json(), ext('F8', 'F7', { depth: 16.05 })]);
}

// 7. i_beam: I profile extruded 483.36, chamfer top rim, fillet bottom rim, two vertical holes
{
  const a = new Sk('F1', STD('top'));
  const half = [[-39.635, 0], [39.635, 0], [39.635, 7.15], [2.415, 7.15], [2.415, 147.53], [39.635, 147.53], [39.635, 154.68], [-39.635, 154.68],
    [-39.635, 147.53], [-2.415, 147.53], [-2.415, 7.15], [-39.635, 7.15]];
  a.poly(half);
  const L = 483.36;
  // e1 bottom flange face (y=0), e7 top flange face (y=154.68), e2/e12 its side neighbours
  const top = [edge('F2.C0', 'F2.S.e7', W(0, 154.68, 0)), edge('F2.C1', 'F2.S.e7', W(0, 154.68, L)), edge('F2.S.e6', 'F2.S.e7', W(39.635, 154.68, L / 2)), edge('F2.S.e7', 'F2.S.e8', W(-39.635, 154.68, L / 2))];
  const bot = [edge('F2.C0', 'F2.S.e1', W(0, 0, 0)), edge('F2.C1', 'F2.S.e1', W(0, 0, L)), edge('F2.S.e12', 'F2.S.e1', W(-39.635, 0, L / 2)), edge('F2.S.e1', 'F2.S.e2', W(39.635, 0, L / 2))];
  const c = new Sk('F5', STD('front'));
  c.circle(0, 0, 2.175); c.circle(0, 120.84, 2.175);
  parts.i_beam_000007 = part('i_beam', [a.json(), ext('F2', 'F1', { depth: L }),
    { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 1.06, edges: top },
    { type: 'fillet', id: 'F4', radius: 1.45, edges: bot }, c.json(), cut('F6', 'F5', thru)]);
}

// 8. spur_gear: 448-vertex tooth outline, bored to a rim, hub + 4 spokes, centre bore + keyway
{
  const g = new Sk('F1', STD('top')).loop(D.spur_gear_000098[0]);
  const b = new Sk('F3', STD('top')); b.circle(0, 0, 23.25);
  const h = new Sk('F5', STD('top')); h.circle(0, 0, 10.88);
  const s = new Sk('F7', STD('top'));
  for (let k = 0; k < 4; k++) {
    const t = rad(90 * k), c = Math.cos(t), sn = Math.sin(t);
    const P = [[10.38, -2.27], [23.75, -2.27], [23.75, 2.27], [10.38, 2.27]].map(([x, y]) => [x * c - y * sn, x * sn + y * c]);
    s.poly(P);
  }
  const bore = new Sk('F9', STD('top')); bore.circle(0, 0, 8.4);
  const key = new Sk('F11', STD('top')); key.rect(-2.64, 0, 2.64, 11.04);
  parts.spur_gear_000098 = part('spur_gear', [g.json(), ext('F2', 'F1', { depth: 28.53 }), b.json(), cut('F4', 'F3', thru),
    h.json(), ext('F6', 'F5', { depth: 28.53 }), s.json(), ext('F8', 'F7', { depth: 28.53 }),
    bore.json(), cut('F10', 'F9', thru), key.json(), cut('F12', 'F11', thru)]);
}

// 9. bevel_gear: loft between two 156-vertex tooth outlines 12.85 apart, bore + keyway
{
  const pl = plane('F2', 'right', 12.85);
  const a = new Sk('F1', STD('right')).loop(D.bevel_gear_000437[0]);
  const b = new Sk('F3', FEAT('F2')).loop(D.bevel_gear_000437[1]);
  const c = new Sk('F5', STD('right')); c.circle(0, 0, 2.875);
  const k = new Sk('F7', STD('right')); k.rect(-0.565, 0, 0.565, 3.435);
  parts.bevel_gear_000437 = part('bevel_gear', [a.json(), pl, b.json(), { type: 'loft', id: 'F4', op: 'add', sketches: ['F1', 'F3'] },
    c.json(), cut('F6', 'F5', thru), k.json(), cut('F8', 'F7', thru)]);
}

// 10. spline_hub: round hub, 100-arc involute-ish spline pocket, internal relief groove
{
  const [start, arcs] = D.spline_hub_000030;
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 57.15);
  const sp = new Sk('F4', FEAT('F3'));
  let cur = start;
  for (const [m, e] of arcs) { sp.arc3(cur, m, e); cur = e; }
  const g = new Sk('F7', FEAT('F6')); g.circle(0, 0, 40.15);
  parts.spline_hub_000030 = part('spline_hub', [a.json(), ext('F2', 'F1', { depth: 57.92 }),
    plane('F3', 'top', 15.48), sp.json(), cut('F5', 'F4', { depth: 43.44, flip: true }),
    plane('F6', 'top', 19.48), g.json(), cut('F8', 'F7', { depth: 2, flip: true })]);
}

// 11. torus_link: torus + three radial lugs, each with a blind cross hole that stays internal
{
  const t = new Sk('F1', STD('top')); t.circle(53.03, 0, 9.82); const ax = t.axis(0, 0, 0, 10);
  const lug = new Sk('F3', STD('front')), hole = new Sk('F5', STD('front'));
  for (const deg of [0, 120, 240]) {
    const cx = 66.47 * Math.cos(rad(deg)), cz = 66.47 * Math.sin(rad(deg));
    const xr = [Math.cos(rad(deg)), Math.sin(rad(deg))], zr = [-Math.sin(rad(deg)), Math.cos(rad(deg))]; // radial / tangential in the XZ plane
    lug.poly([[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([i, j]) => [cx + i * 3.62 * xr[0] + j * 4.245 * zr[0], cz + i * 3.62 * xr[1] + j * 4.245 * zr[1]]));
    hole.circle(cx, cz, 2.45);
  }
  parts.torus_link_000095 = part('torus_link', [t.json(), revolve('F2', 'F1', ax), lug.json(), ext('F4', 'F3', { end: 'mid', depth: 13.82 }),
    hole.json(), cut('F6', 'F5', { end: 'mid', depth: 9.49 })]);
}

// 12. dome_cap: revolved cylinder + spherical-ish dome, bottom pocket, four holes, rim fillets
{
  const a = new Sk('F1', STD('top'));
  const p0 = a.P(0, 0), p1 = a.P(42.88, 0), p2 = a.P(42.88, 38.86);
  a.line(p0, p1); a.line(p1, p2); a.arc3([42.88, 38.86], [30.3207, 69.1807], [0, 81.74]); a.line(a.P(0, 81.74), p0);
  const ax = a.axis(0, 0, 0, 10);
  const pk = new Sk('F3', STD('front')); pk.circle(0, 0, 38.58);
  const hs = new Sk('F5', STD('front'));
  for (const [x, y] of ring(29.5, 4)) hs.circle(x, y, 2.12);
  parts.dome_cap_000212 = part('dome_cap', [a.json(), revolve('F2', 'F1', ax), pk.json(), cut('F4', 'F3', { depth: 12.28 }),
    hs.json(), cut('F6', 'F5', thru)]); // + rim fillet added below
}

// 13. venturi_tube: revolved hollow venturi profile
{
  const a = new Sk('F1', STD('top')).loop(D.venturi_tube_000129[0]);
  const ax = a.axis(0, 0, 0, 10);
  parts.venturi_tube_000129 = part('venturi_tube', [a.json(), revolve('F2', 'F1', ax)]);
}

// 14. bellows: revolved zig-zag convolution profile + 6-hole bolt circle at both flanges
{
  const a = new Sk('F1', STD('top')).loop(D.bellows_000367[0]);
  const ax = a.axis(0, 0, 0, 10);
  const h1 = new Sk('F4', FEAT('F3')), h2 = new Sk('F6', STD('front'));
  for (const [x, y] of ring(33.759, 6)) { h1.circle(x, y, 1.915); h2.circle(x, y, 1.915); }
  parts.bellows_000367 = part('bellows', [a.json(), revolve('F2', 'F1', ax), plane('F3', 'front', -236.3), h1.json(), cut('F5', 'F4', { depth: 4.15, flip: true }),
    h2.json(), cut('F7', 'F6', { depth: 4.15 })]);
}

// 15. t_pipe_fitting: run + branch cylinders, branch bore, run bore
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 26.75);
  const b = new Sk('F3', STD('front')); b.circle(0, 0, 13.35);
  const c = new Sk('F5', STD('front')); c.circle(0, 0, 9.27);
  const d = new Sk('F7', STD('top')); d.circle(0, 0, 22.67);
  parts.t_pipe_fitting_000075 = part('t_pipe_fitting', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 156.73 }),
    b.json(), ext('F4', 'F3', { depth: 46.63, flip: true }), c.json(), cut('F6', 'F5', { depth: 47.63 }),
    d.json(), cut('F8', 'F7', { end: 'mid', depth: 158.73 })]);
}

// 16. dovetail_slide: dovetail profile extruded 95.5, chamfered land rims, c-bore hole, internal slot
{
  const L = 95.5;
  const a = new Sk('F1', STD('top'));
  a.poly([[-25.85, 0], [25.85, 0], [25.85, 9.77], [3.6, 9.77], [16.1, 2.57], [-16.1, 2.57], [-3.6, 9.77], [-25.85, 9.77]]);
  const y = 9.77;
  const ed = [edge('F2.C0', 'F2.S.e3', W(14.725, y, 0)), edge('F2.C1', 'F2.S.e3', W(14.725, y, L)), edge('F2.S.e2', 'F2.S.e3', W(25.85, y, L / 2)), edge('F2.S.e3', 'F2.S.e4', W(3.6, y, L / 2)),
    edge('F2.C0', 'F2.S.e7', W(-14.725, y, 0)), edge('F2.C1', 'F2.S.e7', W(-14.725, y, L)), edge('F2.S.e6', 'F2.S.e7', W(-3.6, y, L / 2)), edge('F2.S.e7', 'F2.S.e8', W(-25.85, y, L / 2))];
  const hl = new Sk('F4', STD('front')); hl.circle(0, 15.916, 2.045);
  const cb = new Sk('F6', STD('front')); cb.circle(0, 15.916, 3.7);
  const sl = new Sk('F9', FEAT('F8')); sl.rect(-1.02, -38.2, 1.02, 38.2);
  parts.dovetail_slide_000038 = part('dovetail_slide', [a.json(), ext('F2', 'F1', { depth: L }),
    { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 0.53, edges: ed },
    hl.json(), cut('F5', 'F4', thru), cb.json(), cut('F7', 'F6', { depth: 2.88 }),
    plane('F8', 'front', -2.35), sl.json(), cut('F10', 'F9', { depth: 0.88, flip: true })]);
}

// 17. sheet_metal_tray: box, chamfered open rim, shelled open at the top, two floor holes
{
  const a = new Sk('F1', STD('front')); a.rect(-117.985, -20.77, 117.985, 20.77);
  const y = 19.515;
  const ed = [edge('F2.C0', 'F2.S.e1', W(0, y, -20.77)), edge('F2.C0', 'F2.S.e2', W(117.985, y, 0)), edge('F2.C0', 'F2.S.e3', W(0, y, 20.77)), edge('F2.C0', 'F2.S.e4', W(-117.985, y, 0))];
  const h = new Sk('F5', STD('front')); h.circle(0, -13.045, 2.395); h.circle(0, 13.045, 2.395);
  parts.sheet_metal_tray_000130 = part('sheet_metal_tray', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 39.03 }),
    { type: 'chamfer', id: 'F3', chamferType: 'equal', distance: 0.96, edges: ed },
    { type: 'shell', id: 'F4', faces: ['F2.C0'], thickness: 3.83 }, h.json(), cut('F6', 'F5', thru)]);
}

// 18. u_channel: box with a deep slot (U section), vertical-edge fillets, top-rim chamfer
{
  const a = new Sk('F1', STD('right')); a.rect(-38.645, -1085.085, 38.645, 1085.085);
  const b = new Sk('F4', FEAT('F3')); b.rect(-26.155, -1085.085, 26.155, 1085.085);
  parts.u_channel_000058 = part('u_channel', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 190.26 }),
    plane('F3', 'right', 95.13), b.json(), cut('F5', 'F4', { depth: 177.77 })]); // + fillet / chamfer below
}

// 19. eyebolt: disc collar, tapered neck loft, threaded shank, torus eye
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 35);
  const l1 = new Sk('F4', FEAT('F3')); l1.circle(0, 0, 19.39);
  const l2 = new Sk('F6', FEAT('F5')); l2.circle(0, 0, 16);
  const sh = new Sk('F9', FEAT('F8')); sh.circle(0, 0, 18);
  const t = new Sk('F11', STD('right')); t.circle(0, 108.67, 14); const ax = t.axis(0, 59.67, 10, 59.67);
  parts.eyebolt_000265 = part('eyebolt', [a.json(), ext('F2', 'F1', { depth: 26 }),
    plane('F3', 'top', 26), l1.json(), plane('F5', 'top', 31.67), l2.json(), { type: 'loft', id: 'F7', op: 'add', sketches: ['F4', 'F6'] },
    plane('F8', 'top', -54), sh.json(), ext('F10', 'F9', { depth: 54 }),
    t.json(), revolve('F12', 'F11', ax)]);
}

// 20. pan_head_screw: shank, domed head with rim fillet, cross slot, V thread groove swept on a helix
{
  const a = new Sk('F1', STD('top')); a.circle(0, 0, 3);
  const hd = new Sk('F4', FEAT('F3')); hd.circle(0, 0, 6);
  const sl = new Sk('F7', FEAT('F6')); sl.rect(-6.2, -0.8, 6.2, 0.8);
  const hs = new Sk('F9', STD('top')); hs.circle(0, 0, 3);
  const tr = new Sk('F11', STD('front')); tr.poly([[3, 0.2309], [2.6, 0], [3, -0.2309]]);
  parts.pan_head_screw_000035 = part('pan_head_screw', [a.json(), ext('F2', 'F1', { depth: 40 }), plane('F3', 'top', 40), hd.json(), ext('F5', 'F4', { depth: 3.6 }),
    plane('F6', 'top', 41.4), sl.json(), cut('F8', 'F7', { depth: 3 }),
    hs.json(), { type: 'helix', id: 'F10', sketch: 'F9', definedBy: 'heightPitch', pitch: 1, height: 14.51, revolutions: 14.51, startAngle: 0 },
    tr.json(), { type: 'sweep', id: 'F12', op: 'cut', sketch: 'F11', path: { kind: 'curve', id: 'F10' } }]); // + head fillet below
}

// ---- blend references found by probing (tools/bench/probe.mjs --edges) ----
const extra = JSON.parse(readFileSync(path.join(here, 'gen2_blends.json'), 'utf8'));
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
