// Authoring helper for the third BenchCAD batch (10 parts, families a-m): tools/bench/benchcad/*.ckpart.json
// translated by hand into our feature tree from the dataset's CadQuery programs. Parameters are
// paraphrased; no CadQuery source is copied. The long point lists of the cam and helical_gear profiles
// (plain coordinates) are kept in gen3_data.json.
//
// Frame (see gen.mjs / gen2.mjs): BenchCAD is Z-up CadQuery, ground truth is rotated (x,y,z)->(x,z,-y),
// which maps CQ XY / XZ / YZ workplanes onto our top / front / right planes with the same local (x,y)
// and the same normal (top normal = +Z_cq, front normal = -Y_cq, right normal = +X_cq).
// Sketch coordinates below are the CQ ones; `W` maps a CQ point to our world for edge `at` hints.
//
// usage: node tools/bench/gen3.mjs
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Sk, STD, FEAT, plane, ext, edge, part } from './lib.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.join(here, 'benchcad');
const D = JSON.parse(readFileSync(path.join(here, 'gen3_data.json'), 'utf8'));
const W = (x, y, z) => [x, z, -y];
const rad = (d) => (d * Math.PI) / 180;
const revolve = (id, sketch, ax, o = {}) => ({ type: 'revolve', id, op: 'add', sketch, axis: ax, angle: 360, ...o });
const cut = (id, sketch, o = {}) => ext(id, sketch, { op: 'cut', ...o });
const thru = { end: 'throughBoth' };
const ring = (R, n, a0 = 0) => Array.from({ length: n }, (_, i) => [R * Math.cos(rad(a0 + (360 * i) / n)), R * Math.sin(rad(a0 + (360 * i) / n))]);

const parts = {};

// 1. bucket: closed 6-point wall profile (flared pot, thin wall, floor) revolved about the local y axis
{
  const a = new Sk('F1', STD('top'));
  a.poly([[0, 0], [41.2, 0], [54.5, 112.09], [51.56, 112.09], [38.26, 3.02], [0, 3.02]]);
  a.axis(0, 0, 0, 10);
  parts.bucket_003137 = part('bucket', [a.json(), revolve('F2', 'F1', `e${a.ne}`)]);
}

// 2. ball_knob: sphere (revolved half disc) + stem (touches the sphere at one point only) minus an axial bore
//    CQ axis is Y; the stem and bore are circles on the CQ XZ (our front) plane.
{
  const R = 14.32;
  const a = new Sk('F1', STD('front'));
  const c = a.p(0, 0), t = a.p(0, R), b = a.p(0, -R);
  a.arc(c, b, t); a.line(t, b); a.axis(0, 0, 0, 10);
  const pl1 = plane('F3', 'front', -42.95);
  const s = new Sk('F4', FEAT('F3')); s.circle(0, 0, 5.15);
  const pl2 = plane('F6', 'front', -71.58);
  const h = new Sk('F7', FEAT('F6')); h.circle(0, 0, 3.435);
  parts.ball_knob_007575 = part('ball_knob', [a.json(), revolve('F2', 'F1', `e${a.ne}`),
    pl1, s.json(), ext('F5', 'F4', { end: 'mid', depth: 57.26 }),
    pl2, h.json(), cut('F8', 'F7', { end: 'mid', depth: 85.9 })]);
}

// 3. clevis: block with a deep fork slot, chamfered prong tops, a cross hole whose axis runs past the block
//    edge, a stem cylinder on top and an axial bore
{
  const a = new Sk('F1', STD('front')); a.rect(-16.96, -17.725, 16.96, 17.725);
  const pl = plane('F3', 'front', -33.375);
  const s = new Sk('F4', FEAT('F3')); s.rect(-6.02, -18.225, 6.02, 18.225);
  const h = new Sk('F7', STD('right')); h.circle(0, 22.435, 5.47);
  const pl2 = plane('F9', 'front', -49.79);
  const st = new Sk('F10', FEAT('F9')); st.circle(0, 0, 9.85);
  const pl3 = plane('F12', 'front', -50.79);
  const bo = new Sk('F13', FEAT('F12')); bo.circle(0, 0, 4.65);
  parts.clevis_014687 = part('clevis', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 66.75 }),
    pl, s.json(), cut('F5', 'F4', { depth: 49.02, flip: true }),
    // F6: chamfer, see gen3_blends.json
    h.json(), cut('F8', 'F7', thru),
    pl2, st.json(), ext('F11', 'F10', { end: 'mid', depth: 32.83 }),
    pl3, bo.json(), cut('F14', 'F13', { end: 'mid', depth: 34.83 })]);
}

// 4. hinge: thin leaf plate with two holes + two knuckle barrels (tangent to the leaf edge) with bores
{
  const a = new Sk('F1', STD('right')); a.rect(-14.545, -23.275, 14.545, 23.275);
  const h = new Sk('F3', STD('right')); h.circle(-7.272, -7.758, 1.455); h.circle(-7.272, 7.758, 1.455);
  const mk = (id, pid, z, r, f) => { const s = new Sk(id, FEAT(pid)); s.circle(0, 18.035, r); return s; };
  const k1 = mk('F6', 'F5', -10.862, 3.49), k2 = mk('F9', 'F8', 10.861, 3.49);
  const b1 = mk('F12', 'F11', -10.862, 1.745), b2 = mk('F15', 'F14', 10.861, 1.745);
  parts.hinge_000079 = part('hinge', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 2.33 }), h.json(), cut('F4', 'F3', thru),
    plane('F5', 'top', -10.862), k1.json(), ext('F7', 'F6', { end: 'mid', depth: 18.62 }),
    plane('F8', 'top', 10.861), k2.json(), ext('F10', 'F9', { end: 'mid', depth: 18.62 }),
    plane('F11', 'top', -10.862), b1.json(), cut('F13', 'F12', { end: 'mid', depth: 20.62 }),
    plane('F14', 'top', 10.861), b2.json(), cut('F16', 'F15', { end: 'mid', depth: 20.62 })]);
}

// 5. j_hook: straight rod, 180 degree handle (revolved circle), square base plate and a pentagonal boss below it
{
  const a = new Sk('S1', FEAT('P1')); a.circle(31, 0, 6);
  const r = new Sk('S2', FEAT('P2')); r.circle(31, 0, 6); const ax = r.axis(0, 0, 0, -10);
  const b = new Sk('S3', FEAT('P3')); b.rect(10, -21, 52, 21);
  const hx = new Sk('S4', FEAT('P4')); hx.poly([[0, 9], [-7.7942, 4.5], [-7.7942, -4.5], [0, -9], [7.7942, -4.5]].map(([x, y]) => [31 + x, y])); // pentagon: the dataset's polyline drops the moveTo start vertex
  parts.j_hook_001383 = part('j_hook', [plane('P1', 'top', 48.72), a.json(), ext('E1', 'S1', { end: 'mid', depth: 107.04 }),
    plane('P2', 'top', 99.24), r.json(), revolve('R1', 'S2', ax, { angle: 180 }),
    plane('P3', 'top', -2.4), b.json(), ext('E2', 'S3', { end: 'mid', depth: 4.8 }),
    plane('P4', 'top', -9.1), hx.json(), ext('E3', 'S4', { end: 'mid', depth: 9.6 })]);
}

// 6. gridfinity_bin: 84 x 84 x 42 block, bottom rim filleted, big pocket open at the top, a ledge on one wall
{
  const a = new Sk('F1', STD('top')); a.rect(-42, -42, 42, 42);
  const pl = plane('F4', 'top', 1.115);
  const p = new Sk('F5', FEAT('F4')); p.rect(-39.77, -39.77, 39.77, 39.77);
  const pl2 = plane('F7', 'top', 12.25);
  const l = new Sk('F8', FEAT('F7')); l.rect(-38.77, -39.77, 38.77, -24.57);
  parts.gridfinity_bin_000166 = part('gridfinity_bin', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 42 }),
    // F3: fillet, see gen3_blends.json
    pl, p.json(), cut('F6', 'F5', { end: 'mid', depth: 39.97 }),
    pl2, l.json(), ext('F9', 'F8', { depth: 3.5 })]);
}

// 7. cam: 64-point lobed outline extruded 27.87, hub boss, bore + key slot, cross hole through the lobe
{
  const a = new Sk('F1', STD('top')).loop(D.cam_012392[0]);
  const hb = new Sk('F4', FEAT('F3')); hb.circle(0, 0, 5.91);
  const bo = new Sk('F6', STD('top')); bo.circle(0, 0, 2.755);
  const k = new Sk('F8', STD('top')); k.rect(-1.105, 2.09, 1.105, 3.42);
  const x = new Sk('F10', STD('top')); x.circle(6.915, 0, 1.05);
  parts.cam_012392 = part('cam', [a.json(), ext('F2', 'F1', { depth: 27.87 }),
    plane('F3', 'top', 27.87), hb.json(), ext('F5', 'F4', { depth: 12.53 }),
    bo.json(), cut('F7', 'F6', thru), k.json(), cut('F9', 'F8', thru), x.json(), cut('F11', 'F10', thru)]);
}

// 8. motor_end_cap: 140 dia flange disc, bore, round boss on the +x face (closes the bore), boss chamfer,
//    fillets on the -x edges, two 8-hole bolt circles through the flange
{
  const a = new Sk('F1', STD('right')); a.circle(0, 0, 69.86);
  const b = new Sk('F3', STD('right')); b.circle(0, 0, 20.62);
  const pl = plane('F5', 'right', 8.085);
  const bs = new Sk('F6', FEAT('F5')); bs.circle(0, 0, 36.58);
  const h1 = new Sk('F10', STD('right')); h1.circle(44.32, 0, 1.65);
  const h2 = new Sk('F12', STD('right')); h2.circle(64.75, 0, 1.73);
  parts.motor_end_cap_000242 = part('motor_end_cap', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 16.17 }), b.json(), cut('F4', 'F3', thru),
    pl, bs.json(), ext('F7', 'F6', { depth: 5.28 }),
    // F8: chamfer, F9: fillet, see gen3_blends.json
    h1.json(), cut('F11', 'F10', thru), { type: 'cpattern', id: 'F11p', seeds: ['F11'], axis: { kind: 'axis', axis: 'x' }, count: 8, angle: 360, equal: true },
    h2.json(), cut('F13', 'F12', thru), { type: 'cpattern', id: 'F13p', seeds: ['F13'], axis: { kind: 'axis', axis: 'x' }, count: 8, angle: 360, equal: true }]);
}

// 9. cable_routing_panel: plate with chamfered top rim, filleted corners, two rows of four slots,
//    four bigger holes and four corner holes
{
  const a = new Sk('F1', STD('top')); a.rect(-105.135, -46.08, 105.135, 46.08);
  const s = new Sk('F5', STD('top'));
  for (const y of [22.2, -22.2]) for (let i = 0; i < 4; i++) { const x = (i - 1.5) * 43.89; s.rect(x - 17.65, y - 4.03, x + 17.65, y + 4.03); }
  const h = new Sk('F7', STD('top'));
  for (let i = 0; i < 4; i++) h.circle((i - 1.5) * 22.14, 0, 5.035);
  const c = new Sk('F9', STD('top'));
  for (const [x, y] of [[97.135, 38.08], [97.135, -38.08], [-97.135, 38.08], [-97.135, -38.08]]) c.circle(x, y, 1.835);
  parts.cable_routing_panel_004961 = part('cable_routing_panel', [a.json(), ext('F2', 'F1', { end: 'mid', depth: 3.79 }),
    // F3: chamfer, F4: fillet, see gen3_blends.json
    s.json(), cut('F6', 'F5', thru), h.json(), cut('F8', 'F7', thru), c.json(), cut('F10', 'F9', thru)]);
}

// 10. helical_gear: loft between two 252-vertex tooth outlines 20.55 apart (second one turned), centre bore
{
  const pl = plane('F2', 'right', 20.55);
  const a = new Sk('F1', STD('right')).loop(D.helical_gear_015542[0]);
  const b = new Sk('F3', FEAT('F2')).loop(D.helical_gear_015542[1]);
  const c = new Sk('F5', STD('right')); c.circle(0, 0, 2.26);
  parts.helical_gear_015542 = part('helical_gear', [a.json(), pl, b.json(), { type: 'loft', id: 'F4', op: 'add', sketches: ['F1', 'F3'] },
    c.json(), cut('F6', 'F5', thru)]);
}

// ---- blend references found by probing (tools/bench/probe.mjs --edges) ----
let extra = {};
try { extra = JSON.parse(readFileSync(path.join(here, 'gen3_blends.json'), 'utf8')); } catch {}
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
