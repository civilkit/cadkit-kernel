// Fillet / chamfer corpus against OCCT reference volumes.
//
//   node tools/blend_corpus.mjs [--wasm path] [--only substr] [--json out.json]   run every case, exit 1 on any mismatch
//   node tools/blend_corpus.mjs --emit DIR [--wasm path]                          write DIR/cases.json + DIR/<case>.step (the base bodies)
//   node tools/blend_corpus.mjs --edges base [--wasm path]                        list the edges of one base body (for authoring picks)
//   python3.12 tools/blend_reference.py DIR                                       build tools/blend_reference.json (OCCT volumes) from an emit
//
// A case = a base part (kernel features) + one blend op + picks. A pick is a POINT on the edge (the edge of the base body
// nearest to it is used, in both the kernel and the OCCT reference), so the same cases drive both engines.
// Per case the check is: no feature error, `validate` ok, volume equal to OCCT's within `tol` (relative; 1e-5 planar, 1e-4 curved).
// `expect: "<code>"` marks a case whose right answer is a specific error code (the kernel must report that code).
// Known-gap cases keep running; `gap` names the roadmap item they belong to (reported, still counted as failures).
import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const mod = new WebAssembly.Module(readFileSync(wasmPath));

// ---------------------------------------------------------------- bases
const STD = (id) => ({ kind: 'std', id });
const FACE = (id) => ({ kind: 'face', id });
const poly = (id, plane, pts) => ({
  type: 'sketch', id, plane,
  points: pts.map(([x, y], i) => ({ id: `p${i + 1}`, x, y })),
  entities: pts.map((_, i) => ({ type: 'line', id: `e${i + 1}`, p1: `p${i + 1}`, p2: `p${((i + 1) % pts.length) + 1}` })),
  constraints: [],
});
const rect = (id, plane, x0, y0, x1, y1) => poly(id, plane, [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]);
const circ = (id, plane, cx, cy, r) => ({ type: 'sketch', id, plane, points: [{ id: 'c', x: cx, y: cy }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [] });
const ext = (id, sketch, depth, op = 'add') => ({ type: 'extrude', id, op, sketch, end: 'blind', depth });
const TOP = STD('top');

// rounded rectangle x 0..w, sketch y 0..h (z = -y), corner radius R: four lines and four arcs
const rrectSketch = (id, plane, w, h, R) => {
  const P = { a: [R, 0], b: [w - R, 0], c: [w, R], d: [w, h - R], e: [w - R, h], f: [R, h], g: [0, h - R], i: [0, R] };
  const C = { c1: [w - R, R], c2: [w - R, h - R], c3: [R, h - R], c4: [R, R] };
  const points = [...Object.entries(P), ...Object.entries(C)].map(([k, [x, y]]) => ({ id: k, x, y }));
  const L = (n, a, b) => ({ type: 'line', id: n, p1: a, p2: b });
  const A = (n, c, a, b) => ({ type: 'arc', id: n, c, p1: a, p2: b });
  return { type: 'sketch', id, plane, points, constraints: [], entities: [L('e1', 'a', 'b'), A('e2', 'c1', 'b', 'c'), L('e3', 'c', 'd'), A('e4', 'c2', 'd', 'e'), L('e5', 'e', 'f'), A('e6', 'c3', 'f', 'g'), L('e7', 'g', 'i'), A('e8', 'c4', 'i', 'a')] };
};

const BASES = {
  // 40 x 20 x 10 block: x 0..40, y 0..10, z -20..0
  box: [rect('F1', TOP, 0, 0, 40, 20), ext('F2', 'F1', 10)],
  // L bracket: legs 40 x 10 and 10 x 30 (outline in x/-z), 10 high; concave vertical edge at x=10, z=-10
  lbr: [poly('F1', TOP, [[0, 0], [40, 0], [40, 10], [10, 10], [10, 30], [0, 30]]), ext('F2', 'F1', 10)],
  // 40 x 40 x 20 block with a 20 x 20 x 8 pocket in the top face (z -10..-30, x 10..30)
  pocket: [rect('F1', TOP, 0, 0, 40, 40), ext('F2', 'F1', 20), rect('F3', FACE('F2.C1'), 10, 10, 30, 30), ext('F4', 'F3', 8, 'cut')],
  // 40 x 40 x 10 plate with a boss, radius 8, 15 high, at x=20, z=-20
  boss: [rect('F1', TOP, 0, 0, 40, 40), ext('F2', 'F1', 10), circ('F3', FACE('F2.C1'), 20, 20, 8), ext('F4', 'F3', 15)],
  // 40 x 40 x 10 plate with a through hole, radius 6, at x=20, z=-20
  holed: [rect('F1', TOP, 0, 0, 40, 40), ext('F2', 'F1', 10), circ('F3', FACE('F2.C1'), 20, 20, 6), ext('F4', 'F3', 10, 'cut')],
  // 40 x 30 x 10 plate with its vertical edges rounded (r 6): the top outline is a tangent chain of 4 lines and 4 arcs
  rounded: [rect('F1', TOP, 0, 0, 40, 30), ext('F2', 'F1', 10), { type: 'fillet', id: 'F3', radius: 6, edges: [
    { faces: ['F2.S.e1', 'F2.S.e4'], at: [0, 5, 0] }, { faces: ['F2.S.e1', 'F2.S.e2'], at: [40, 5, 0] },
    { faces: ['F2.S.e2', 'F2.S.e3'], at: [40, 5, -30] }, { faces: ['F2.S.e3', 'F2.S.e4'], at: [0, 5, -30] }] }],
  // hollow frustum: outer cone radius 15 (y 0) to 10 (y 20), bore radius 8, axis +y through the origin
  cone: [{ ...poly('F1', STD('front'), [[8, 0], [15, 0], [10, 20], [8, 20]]),
    points: [{ id: 'p1', x: 8, y: 0 }, { id: 'p2', x: 15, y: 0 }, { id: 'p3', x: 10, y: 20 }, { id: 'p4', x: 8, y: 20 }, { id: 'a1', x: 0, y: 0 }, { id: 'a2', x: 0, y: 20 }],
    entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })).concat([{ type: 'line', id: 'ax', p1: 'a1', p2: 'a2', construction: true }]) },
    { type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360 }],
  // rounded rectangle prisms 40 x 25 x 10 (corner radius 5, 2, 6) and one with an arm unioned on top (x 5..15, z -5..-20, 8 high)
  rrect5: [rrectSketch('F1', TOP, 40, 25, 5), ext('F2', 'F1', 10)],
  rrect2: [rrectSketch('F1', TOP, 40, 25, 2), ext('F2', 'F1', 10)],
  rrect6: [rrectSketch('F1', TOP, 40, 25, 6), ext('F2', 'F1', 10)],
  rrectarm: [rrectSketch('F1', TOP, 40, 25, 5), ext('F2', 'F1', 10), rect('F3', FACE('F2.C1'), 5, 5, 15, 20), ext('F4', 'F3', 8)],
  // thin wall 80 x 4 x 60 (x 0..80, z -4..0, y 0..60) with a tower 20 x 12 x 40 on its front face (x 30..50, z 0..12, y 0..40)
  tower: [rect('F1', TOP, 0, 0, 80, 4), ext('F2', 'F1', 60), rect('F3', TOP, 30, -12, 50, 0), ext('F4', 'F3', 40)],
  // 60 x 60 x 5 plate with a 20 x 20 x 30 post (x 20..40, z -20..-40) on top; `post2` / `post4`: its vertical edges rounded R2 / R4
  post0: [rect('F1', TOP, 0, 0, 60, 60), ext('F2', 'F1', 5), rect('F3', FACE('F2.C1'), 20, 20, 40, 40), ext('F4', 'F3', 30)],
  post2: [rect('F1', TOP, 0, 0, 60, 60), ext('F2', 'F1', 5), rect('F3', FACE('F2.C1'), 20, 20, 40, 40), ext('F4', 'F3', 30), { type: 'fillet', id: 'F5', radius: 2, edges: [
    { faces: ['F4.S.e1', 'F4.S.e2'], at: [40, 20, -20] }, { faces: ['F4.S.e2', 'F4.S.e3'], at: [40, 20, -40] },
    { faces: ['F4.S.e3', 'F4.S.e4'], at: [20, 20, -40] }, { faces: ['F4.S.e4', 'F4.S.e1'], at: [20, 20, -20] }] }],
  post4: [rect('F1', TOP, 0, 0, 60, 60), ext('F2', 'F1', 5), rect('F3', FACE('F2.C1'), 20, 20, 40, 40), ext('F4', 'F3', 30), { type: 'fillet', id: 'F5', radius: 4, edges: [
    { faces: ['F4.S.e1', 'F4.S.e2'], at: [40, 20, -20] }, { faces: ['F4.S.e2', 'F4.S.e3'], at: [40, 20, -40] },
    { faces: ['F4.S.e3', 'F4.S.e4'], at: [20, 20, -40] }, { faces: ['F4.S.e4', 'F4.S.e1'], at: [20, 20, -20] }] }],
  // 60 x 60 x 5 floor with an L-shaped wall, 6 thick, 30 high: the inside corner of two walls and the floor is at x 16, z -16
  lwall: [rect('F1', TOP, 0, 0, 60, 60), ext('F2', 'F1', 5), poly('F3', FACE('F2.C1'), [[10, 10], [50, 10], [50, 16], [16, 16], [16, 50], [10, 50]]), ext('F4', 'F3', 30)],
  // 30 cube
  cube: [rect('F1', TOP, 0, 0, 30, 30), ext('F2', 'F1', 30)],
  // tall block 20 x 20 x 40 (long vertical edges)
  pillar: [rect('F1', TOP, 0, 0, 20, 20), ext('F2', 'F1', 40)],
};

// ---------------------------------------------------------------- cases
// op: {k:'fillet', r} | {k:'varfillet', radii:[near, far]} | {k:'chamfer', d} | {k:'chamfer2', d1, d2} | {k:'chamferA', d, a}
// picks: [{at:[x,y,z], radii?:[near, far], r?:number}]; chamfer2 / chamferA take `face: [nx,ny,nz]` = normal of the face the first distance is on.
const cases = [];
const add = (name, base, op, picks, o = {}) => cases.push({ name, base, op, picks: picks.map((p) => (Array.isArray(p) ? { at: p } : p)), ...o });
const TOPE = { f: [20, 10, 0], r: [40, 10, -10], b: [20, 10, -20], l: [0, 10, -10] }; // box top edges: front, right, back, left
const BOTE = { f: [20, 0, 0], r: [40, 0, -10], b: [20, 0, -20], l: [0, 0, -10] };
const VERT = { fl: [0, 5, 0], fr: [40, 5, 0], br: [40, 5, -20], bl: [0, 5, -20] };
const UP = [0, 1, 0], DN = [0, -1, 0], FRONT = [0, 0, 1], RIGHT = [1, 0, 0];

// 1. chamfers with two distances / distance and angle
add('box_ch_equal_edge', 'box', { k: 'chamfer', d: 2 }, [TOPE.f]);
add('box_ch2_top_edge', 'box', { k: 'chamfer2', d1: 2, d2: 3, face: UP }, [TOPE.f]);
add('box_ch2_side_edge', 'box', { k: 'chamfer2', d1: 2, d2: 3, face: FRONT }, [TOPE.f]);
add('box_ch2_vertical_edge', 'box', { k: 'chamfer2', d1: 1.5, d2: 4, face: RIGHT }, [VERT.fr]);
add('box_ch2_ring_top', 'box', { k: 'chamfer2', d1: 2, d2: 3, face: UP }, [TOPE.f, TOPE.r, TOPE.b, TOPE.l]);
add('box_ch2_ring_bottom', 'box', { k: 'chamfer2', d1: 1, d2: 4, face: DN }, [BOTE.f, BOTE.r, BOTE.b, BOTE.l]);
add('box_ch2_two_adjacent', 'box', { k: 'chamfer2', d1: 2, d2: 3, face: UP }, [TOPE.f, TOPE.r]);
add('box_ch2_two_opposite', 'box', { k: 'chamfer2', d1: 3, d2: 1, face: UP }, [TOPE.f, TOPE.b]);
add('box_ch2_big', 'box', { k: 'chamfer2', d1: 8, d2: 2, face: UP }, [TOPE.f]);
add('box_chA_top_edge', 'box', { k: 'chamferA', d: 2, a: 30, face: UP }, [TOPE.f]);
add('box_chA_side_edge', 'box', { k: 'chamferA', d: 3, a: 60, face: FRONT }, [TOPE.f]);
add('box_chA_ring_top', 'box', { k: 'chamferA', d: 3, a: 50, face: UP }, [TOPE.f, TOPE.r, TOPE.b, TOPE.l]);
add('box_chA_vertical', 'box', { k: 'chamferA', d: 2, a: 45, face: RIGHT }, [VERT.fr]);
add('lbr_ch2_concave_edge', 'lbr', { k: 'chamfer2', d1: 2, d2: 3, face: [0, 0, -1] }, [[10, 5, -10]]);
add('lbr_ch2_top_outline', 'lbr', { k: 'chamfer2', d1: 1, d2: 2, face: UP }, [[20, 10, 0], [40, 10, -5], [25, 10, -10], [10, 10, -20], [5, 10, -30], [0, 10, -15]]);
const LTOP = [[20, 10, 0], [40, 10, -5], [25, 10, -10], [10, 10, -20], [5, 10, -30], [0, 10, -15]];
add('lbr_ch_equal_top_outline', 'lbr', { k: 'chamfer', d: 1.5 }, LTOP);
add('lbr_fillet_top_outline', 'lbr', { k: 'fillet', r: 2 }, LTOP);
add('lbr_ch2_two_edges_reflex', 'lbr', { k: 'chamfer2', d1: 1, d2: 2, face: UP }, [[25, 10, -10], [10, 10, -20]]);
add('cube_ch_equal_corner3', 'cube', { k: 'chamfer', d: 4 }, [[15, 30, 0], [30, 30, -15], [30, 15, 0]]);
// plane / cylinder neighbours
add('boss_ch2_top_rim', 'boss', { k: 'chamfer2', d1: 2, d2: 1, face: UP }, [[28, 25, -20]]);
add('boss_ch2_top_rim_cyl_first', 'boss', { k: 'chamfer2', d1: 2, d2: 1, face: 'cyl' }, [[28, 25, -20]]);
add('boss_chA_top_rim', 'boss', { k: 'chamferA', d: 2, a: 30, face: UP }, [[28, 25, -20]]);
add('boss_ch2_base_rim', 'boss', { k: 'chamfer2', d1: 1.5, d2: 2.5, face: UP }, [[28, 10, -20]]);
add('holed_ch2_rim', 'holed', { k: 'chamfer2', d1: 1.5, d2: 2.5, face: UP }, [[26, 10, -20]]);
add('holed_chA_rim', 'holed', { k: 'chamferA', d: 2, a: 40, face: UP }, [[26, 10, -20]]);
add('cone_ch2_top_rim', 'cone', { k: 'chamfer2', d1: 1, d2: 1.5, face: UP }, [[10, 20, 0]]);
add('cone_ch2_top_rim_cone_first', 'cone', { k: 'chamfer2', d1: 1.5, d2: 1, face: 'cone' }, [[10, 20, 0]]);
add('cone_chA_bottom_rim', 'cone', { k: 'chamferA', d: 3, a: 30, face: DN }, [[15, 0, 0]]);
add('cone_ch2_bore_rim', 'cone', { k: 'chamfer2', d1: 1, d2: 2, face: UP }, [[8, 20, 0]]);
add('cone_fillet_top_rim', 'cone', { k: 'fillet', r: 1.5 }, [[10, 20, 0]]);
add('holed_ch2_bottom_rim', 'holed', { k: 'chamfer2', d1: 1, d2: 2, face: DN }, [[26, 0, -20]]);

// chains of tangent edges: one pick, `propagate: true` (OCCT always propagates along tangent edges)
add('rounded_ch_propagate', 'rounded', { k: 'chamfer', d: 1.5 }, [[20, 10, 0]], { propagate: true });
add('rounded_ch2_propagate', 'rounded', { k: 'chamfer2', d1: 2, d2: 1, face: UP }, [[20, 10, 0]], { propagate: true });
add('rounded_chA_propagate', 'rounded', { k: 'chamferA', d: 2, a: 35, face: UP }, [[20, 10, 0]], { propagate: true });
add('rounded_fillet_propagate', 'rounded', { k: 'fillet', r: 1.5 }, [[20, 10, 0]], { propagate: true });

// rim loop of a rounded-rectangle prism (four lines and four tangent arcs): all eight edges picked, or one pick with `propagate`
const rloop = (w, h, R, y) => {
  const m = R * (1 - Math.SQRT1_2);
  return [[w / 2, y, 0], [w, y, -h / 2], [w / 2, y, -h], [0, y, -h / 2], [w - m, y, -m], [w - m, y, -(h - m)], [m, y, -(h - m)], [m, y, -m]];
};
for (const [base, R] of [['rrect5', 5], ['rrect2', 2], ['rrect6', 6]]) {
  add(`${base}_fillet_top_loop_r1_5`, base, { k: 'fillet', r: 1.5 }, rloop(40, 25, R, 10));
  add(`${base}_fillet_bottom_loop_r1_5`, base, { k: 'fillet', r: 1.5 }, rloop(40, 25, R, 0));
  add(`${base}_chamfer_top_loop_d1_5`, base, { k: 'chamfer', d: 1.5 }, rloop(40, 25, R, 10));
}
add('rrect5_fillet_top_loop_r2', 'rrect5', { k: 'fillet', r: 2 }, rloop(40, 25, 5, 10));
add('rrect6_fillet_top_loop_r2', 'rrect6', { k: 'fillet', r: 2 }, rloop(40, 25, 6, 10));
add('rrect5_fillet_propagate_r2', 'rrect5', { k: 'fillet', r: 2 }, [[20, 10, 0]], { propagate: true });
add('rrect6_fillet_propagate_r2', 'rrect6', { k: 'fillet', r: 2 }, [[20, 10, 0]], { propagate: true });
add('rrectarm_fillet_top_loop_r2', 'rrectarm', { k: 'fillet', r: 2 }, rloop(40, 25, 5, 10));
add('rrectarm_fillet_top_loop_r1', 'rrectarm', { k: 'fillet', r: 1 }, rloop(40, 25, 5, 10));

// 2. constant fillets (baseline of what works)
add('box_fillet_edge', 'box', { k: 'fillet', r: 2 }, [TOPE.f]);
add('box_fillet_corner3', 'cube', { k: 'fillet', r: 4 }, [[15, 30, 0], [30, 30, -15], [30, 15, 0]]);
add('box_fillet_ring_top', 'box', { k: 'fillet', r: 2 }, [TOPE.f, TOPE.r, TOPE.b, TOPE.l]);
add('lbr_fillet_concave', 'lbr', { k: 'fillet', r: 2 }, [[10, 5, -10]]);
add('boss_fillet_base', 'boss', { k: 'fillet', r: 2 }, [[28, 10, -20]]);
add('boss_fillet_top', 'boss', { k: 'fillet', r: 2 }, [[28, 25, -20]]);
add('holed_fillet_rim', 'holed', { k: 'fillet', r: 1.5 }, [[26, 10, -20]]);

// 3. variable radius, isolated and at corners
add('var_isolated', 'box', { k: 'varfillet', radii: [1, 3] }, [{ at: [4, 10, 0] }]);
add('var_concave_isolated', 'lbr', { k: 'varfillet', radii: [1, 3] }, [{ at: [10, 1, -10] }]);
// two edges at the corner (40,10,0): radius 3 there
add('var_corner2', 'box', { k: 'varfillet', radii: [3, 1] }, [{ at: [38, 10, 0] }, { at: [40, 10, -2] }]);
add('var_corner2_equal_ends', 'box', { k: 'varfillet', radii: [2, 2] }, [{ at: [38, 10, 0] }, { at: [40, 10, -2] }]);
add('var_corner3', 'box', { k: 'varfillet', radii: [3, 1] }, [{ at: [38, 10, 0] }, { at: [40, 10, -2] }, { at: [40, 8, 0] }]);
add('var_corner3_equal_ends', 'cube', { k: 'varfillet', radii: [3, 3] }, [{ at: [28, 30, 0] }, { at: [30, 30, -2] }, { at: [30, 28, 0] }]);
// ring of four top edges: radius 3 at two opposite corners, 1 at the others
add('var_ring_top', 'box', { k: 'varfillet', radii: [3, 1] }, [{ at: [2, 10, 0] }, { at: [40, 10, -18] }, { at: [38, 10, -20] }, { at: [0, 10, -2] }]);
add('var_two_vertical_far', 'box', { k: 'varfillet', radii: [3, 1] }, [{ at: [40, 8, 0] }, { at: [0, 8, -20] }]);
// conflicting radii at the shared vertex of two edges: must be refused by name
add('var_corner2_mismatch', 'box', { k: 'varfillet', radii: [3, 1] }, [{ at: [38, 10, 0] }, { at: [40, 10, -18] }], { expect: 'variableRadiusMismatch' });

// 4. mixed vertices: pocket rim (convex) next to the concave wall edge
const PR1 = [10, 20, -20], PR2 = [20, 20, -10]; // rim edges of the pocket on the left / front wall
const PW = [10, 16, -10]; // wall-wall concave edge at the pocket corner (x=10, z=-10)
add('pocket_mixed_r2', 'pocket', { k: 'fillet', r: 2 }, [PR1, PR2, PW]);
add('pocket_mixed_r1', 'pocket', { k: 'fillet', r: 1 }, [PR1, PR2, PW]);
add('pocket_mixed_r3', 'pocket', { k: 'fillet', r: 3 }, [PR1, PR2, PW]);
add('pocket_mixed_r2_reversed_order', 'pocket', { k: 'fillet', r: 2 }, [PW, PR2, PR1]);
add('pocket_mixed_wall_first', 'pocket', { k: 'fillet', r: 2 }, [PW, PR1, PR2]);
add('pocket_rim_ring_r2', 'pocket', { k: 'fillet', r: 2 }, [[10, 20, -20], [20, 20, -10], [30, 20, -20], [20, 20, -30]]);
add('pocket_walls_concave3_r2', 'pocket', { k: 'fillet', r: 2 }, [[10, 12, -20], [20, 12, -10], PW]);

// 5. unequal radii at a 3-edge vertex
add('cube_unequal_corner3', 'cube', { k: 'fillet', r: 0 }, [{ at: [15, 30, 0], r: 3 }, { at: [30, 30, -15], r: 4 }, { at: [30, 15, 0], r: 5 }]);
add('pocket_unequal_concave3', 'pocket', { k: 'fillet', r: 0 }, [{ at: [10, 12, -20], r: 1 }, { at: [20, 12, -10], r: 2 }, { at: PW, r: 3 }]);
add('pocket_unequal_mixed3', 'pocket', { k: 'fillet', r: 0 }, [{ at: PR1, r: 1 }, { at: PR2, r: 2 }, { at: PW, r: 3 }]);
add('cube_unequal_two_edges', 'cube', { k: 'fillet', r: 0 }, [{ at: [15, 30, 0], r: 3 }, { at: [30, 30, -15], r: 5 }]);

// 8. coves (concave edges): the inside corner of two floor/wall edges with and without the vertical edge, a cove ring, and the base loop of
// a post (4 lines, plus 4 tangent arcs when its vertical edges are rounded: a closed concave tangent chain)
const CV1 = [10, 12, -20], CV2 = [20, 12, -10], CV3 = [30, 12, -20], CV4 = [20, 12, -30];
const VS = [[10, 16, -10], [30, 16, -10], [30, 16, -30], [10, 16, -30]];
for (const r of [1, 2, 3]) {
  add(`pocket_cove2_r${r}`, 'pocket', { k: 'fillet', r }, [CV1, CV2], { g1: true, mitre: true });
  add(`pocket_cove_ring_r${r}`, 'pocket', { k: 'fillet', r }, [CV1, CV2, CV3, CV4], { g1: true, mitre: true });
  add(`pocket_cove_ring_verts_r${r}`, 'pocket', { k: 'fillet', r }, [CV1, CV2, CV3, CV4, ...VS], { g1: true });
  add(`pocket_cove2_vert_r${r}`, 'pocket', { k: 'fillet', r }, [CV1, CV2, VS[0]], { g1: true });
}
// the inside corner of an L-shaped wall on a floor (floor edges e3, e4 of the wall, vertical edge between them) and the whole floor loop
const LI = [[33, 5, -16], [16, 5, -33]], LV = [16, 20, -16];
const LLOOP = [[30, 5, -10], [10, 5, -30], [13, 5, -50], [16, 5, -33], [33, 5, -16], [50, 5, -13]];
for (const r of [1, 2, 3]) {
  add(`lwall_inside_r${r}`, 'lwall', { k: 'fillet', r }, LI, { g1: true, mitre: true });
  add(`lwall_inside_vert_r${r}`, 'lwall', { k: 'fillet', r }, [...LI, LV], { g1: true });
  add(`lwall_floor_loop_r${r}`, 'lwall', { k: 'fillet', r }, LLOOP, { g1: true, mitre: true });
}
const ploop = (R) => {
  const m = R * (1 - Math.SQRT1_2);
  const lines = [[30, 5, -20], [40, 5, -30], [30, 5, -40], [20, 5, -30]];
  return R ? [...lines, [40 - m, 5, -(20 + m)], [40 - m, 5, -(40 - m)], [20 + m, 5, -(40 - m)], [20 + m, 5, -(20 + m)]] : lines;
};
for (const [base, R] of [['post0', 0], ['post2', 2], ['post4', 4]]) {
  for (const r of [1, 2, 3]) {
    add(`${base}_cove_r${r}`, base, { k: 'fillet', r }, ploop(R), { g1: true, mitre: R === 0 });
    if (R) add(`${base}_cove_r${r}_propagate`, base, { k: 'fillet', r }, [ploop(R)[0]], { propagate: true, g1: true });
  }
}

// 7. a tower on a thin wall (Onshape comparison): every tower edge in one fillet. Vertices at the top: front corners 3 convex, wall corners
// concave-concave-convex (side-wall, top-wall, side-top). `g1` cases also assert normal continuity across the corner patches.
const TW_CCV = [{ at: [30, 20, 0] }, { at: [50, 20, 0] }, { at: [40, 40, 0] }]; // concave: two vertical, one along the top
const TW_CVX = [{ at: [30, 20, 12] }, { at: [50, 20, 12] }, { at: [40, 40, 12] }, { at: [50, 40, 6] }, { at: [30, 40, 6] }]; // convex
const TW_ALL = [...TW_CVX, ...TW_CCV];
for (const r of [1, 1.5, 2, 3]) {
  const nm = String(r).replace('.', '_');
  add(`tower_all_r${nm}`, 'tower', { k: 'fillet', r }, TW_ALL, { g1: true });
  add(`tower_all_r${nm}_reversed`, 'tower', { k: 'fillet', r }, [...TW_ALL].reverse(), { g1: true });
  add(`tower_two_step_r${nm}`, 'tower', { k: 'fillet', r }, TW_CCV, { step2: TW_CVX, g1: true });
}
add('tower_all_r2_propagate', 'tower', { k: 'fillet', r: 2 }, TW_ALL, { propagate: true, g1: true });
add('tower_concave_only_r2', 'tower', { k: 'fillet', r: 2 }, TW_CCV, { g1: true, mitre: true });
add('tower_convex_only_r2', 'tower', { k: 'fillet', r: 2 }, TW_CVX, { g1: true });
add('tower_top_rim_r2', 'tower', { k: 'fillet', r: 2 }, [TW_CVX[2], TW_CVX[3], TW_CVX[4], TW_CCV[2]], { g1: true });

// 6. an inner (concave) vertical fillet meeting the outer (convex) fillets of the rim: L bracket (inner corner at x=10, z=-10),
// the vertex at the top has two convex rim edges and the concave edge. Same and different radii, 1 to 4 mm. Every case is also
// checked for open display-mesh edges (a face missing while `validate` passes).
const LW = [10, 5, -10], LR1 = [25, 10, -10], LR2 = [10, 10, -20];
for (const r of [1, 2, 3, 4]) add(`lbr_mixed_r${r}`, 'lbr', { k: 'fillet', r }, [LR1, LR2, LW]);
add('lbr_mixed_outline_r2', 'lbr', { k: 'fillet', r: 2 }, [...LTOP, LW]);
for (const [rim, wall] of [[1, 3], [3, 1], [2, 4], [4, 2], [1, 4]]) {
  add(`lbr_mixed_rim${rim}_wall${wall}`, 'lbr', { k: 'fillet', r: 0 }, [{ at: LR1, r: rim }, { at: LR2, r: rim }, { at: LW, r: wall }]);
}
add('lbr_mixed_rims_1_2_wall_3', 'lbr', { k: 'fillet', r: 0 }, [{ at: LR1, r: 1 }, { at: LR2, r: 2 }, { at: LW, r: 3 }]);
for (const [rim, wall] of [[1, 3], [2, 4], [4, 2]]) {
  add(`pocket_mixed_rim${rim}_wall${wall}`, 'pocket', { k: 'fillet', r: 0 }, [{ at: PR1, r: rim }, { at: PR2, r: rim }, { at: PW, r: wall }]);
}

// ---------------------------------------------------------------- engine
function req(k, id, msg) {
  const r = k.request(JSON.stringify({ id, msg }));
  return r.json ? JSON.parse(r.json) : { ok: false, trap: r.trap };
}
function baseBody(k, base) {
  const res = req(k, 1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: BASES[base] } });
  if (res.result?.errors?.length) throw new Error(`base ${base}: ${JSON.stringify(res.result.errors)}`);
  return res.result.body;
}
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function distSeg(p, a, b) {
  const ab = sub(b, a), ap = sub(p, a), t = Math.max(0, Math.min(1, dot(ap, ab) / (dot(ab, ab) || 1)));
  const q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
  const d = sub(p, q);
  return Math.hypot(d[0], d[1], d[2]);
}
function findEdge(body, at) {
  let best = null;
  for (const e of body.edges) {
    const pts = e.pts?.length >= 2 ? e.pts : [e.mid, e.mid];
    let d = Math.hypot(...sub(at, e.mid));
    for (let i = 0; i + 1 < pts.length; i++) d = Math.min(d, distSeg(at, pts[i], pts[i + 1]));
    if (!best || d < best.d) best = { d, e };
  }
  return best && best.d < 1e-3 ? best.e : null;
}
// the base face of an edge whose normal is closest to `n` (planar faces), or the cylinder face
function faceByNormal(body, edge, n) {
  if (n === 'cyl' || n === 'cone') return edge.faces.find((f) => body.faces.find((x) => x.id === f)?.kind === (n === 'cyl' ? 'cylinder' : 'cone'));
  let best = null;
  for (const id of edge.faces) {
    const f = body.faces.find((x) => x.id === id);
    if (!f?.normal) continue;
    const d = dot(f.normal, n);
    if (!best || d > best.d) best = { d, id };
  }
  return best && best.d > 0.99 ? best.id : null;
}
function feature(k, c, body, picks = c.picks, id = 'F9') {
  const edges = picks.map((p) => {
    const e = findEdge(body, p.at);
    if (!e) throw new Error(`${c.name}: no edge near ${p.at}`);
    const ref = { faces: [...e.faces], at: p.at };
    if (p.radii) ref.radii = p.radii;
    if (p.r) ref.radius = p.r;
    return ref;
  });
  const o = c.op;
  const pr = c.propagate ? { propagate: true } : {};
  switch (o.k) {
    case 'fillet': return o.r > 0 ? { type: 'fillet', id, radius: o.r, edges, ...pr } : { type: 'fillet', id, radius: Math.max(...picks.map((p) => p.r)), edges };
    case 'varfillet': return { type: 'fillet', id, filletType: 'variable', radii: o.radii, edges };
    case 'chamfer': return { type: 'chamfer', id, chamferType: 'equal', distance: o.d, edges, ...pr };
    case 'chamfer2':
    case 'chamferA': {
      const e0 = findEdge(body, picks[0].at);
      const face = faceByNormal(body, e0, o.face);
      if (!face) throw new Error(`${c.name}: no face with normal ${o.face}`);
      return o.k === 'chamfer2'
        ? { type: 'chamfer', id, chamferType: 'distanceDistance', distance: o.d1, distance2: o.d2, face, edges, ...pr }
        : { type: 'chamfer', id, chamferType: 'angleDistance', distance: o.d, angle: o.a, face, edges, ...pr };
    }
  }
  throw new Error('op');
}
// Mesh edges used by exactly one triangle (a missing face the validator does not see: the black hole in the viewport).
// The display mesh is per face, so a face edge may be split differently on its neighbour (T-junction): an unmatched edge only
// counts as open when sample points along it are not covered by other unmatched edges.
function openMeshEdges(raw, res) {
  const m = res.result?.body?.mesh;
  if (!m || !raw.bytes) return null;
  const jl = new DataView(raw.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const pos = new Float32Array(raw.bytes.buffer.slice(base + m.positions.offset, base + m.positions.offset + m.positions.len * 4));
  const q = (i) => `${Math.round(pos[i] * 2000)},${Math.round(pos[i + 1] * 2000)},${Math.round(pos[i + 2] * 2000)}`;
  const count = new Map();
  const segs = [];
  for (let t = 0; t + 8 < pos.length; t += 9) {
    for (let e = 0; e < 3; e++) {
      const a = t + 3 * e, b = t + 3 * ((e + 1) % 3);
      const ka = q(a), kb = q(b);
      if (ka === kb) continue;
      const key = ka < kb ? `${ka}|${kb}` : `${kb}|${ka}`;
      const c = count.get(key);
      if (c) c.n++; else count.set(key, { n: 1, a: [pos[a], pos[a + 1], pos[a + 2]], b: [pos[b], pos[b + 1], pos[b + 2]] });
    }
  }
  const bx = res.result.body.box;
  const covTol = 1e-3 * Math.hypot(bx.max[0] - bx.min[0], bx.max[1] - bx.min[1], bx.max[2] - bx.min[2]);
  const un = [...count.values()].filter((c) => c.n === 1);
  for (const c of un) segs.push(c);
  let open = 0;
  for (const c of un) {
    let ok = true;
    for (const f of [0.2, 0.4, 0.6, 0.8]) {
      const p = [0, 1, 2].map((i) => c.a[i] + (c.b[i] - c.a[i]) * f);
      let hit = false;
      for (const o of segs) {
        if (o === c) continue;
        if (distSeg(p, o.a, o.b) < covTol) { hit = true; break; }
      }
      if (!hit) { ok = false; break; }
    }
    if (!ok) { open++; if (process.env.OPENDBG) console.log("  open", c.a.map((x) => x.toFixed(2)).join(","), "->", c.b.map((x) => x.toFixed(2)).join(",")); }
  }
  return open;
}

// G1 across blend faces: for every edge of the result that borders a face named `F*.B*` / `F*.V*` (a fillet, chamfer-free), the display-mesh
// normals of the two faces at the edge points must agree (fillets are tangent to the faces they join; corner patches to the bands).
// Chamfer faces are planar facets and meet their neighbours at an angle by design, so cases with chamfers skip the check.
function g1Defect(raw, res, mitre = false) {
  const body = res.result?.body, m = body?.mesh;
  if (!m || !raw.bytes) return null;
  const jl = new DataView(raw.bytes.buffer).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const get = (v, C) => new C(raw.bytes.buffer.slice(base + v.offset, base + v.offset + v.len * C.BYTES_PER_ELEMENT));
  const pos = get(m.positions, Float32Array), nor = get(m.normals, Float32Array), tf = get(m.triFace, Int32Array);
  const byFace = new Map();
  for (let t = 0; t < tf.length; t++) {
    const l = byFace.get(tf[t]) ?? [];
    for (let v = 0; v < 3; v++) { const o = t * 9 + v * 3; l.push([pos[o], pos[o + 1], pos[o + 2], nor[o], nor[o + 1], nor[o + 2]]); }
    byFace.set(tf[t], l);
  }
  const idx = new Map(body.faces.map((f, i) => [f.id, i]));
  const isBlend = (id) => /\.(B|V)\d+/.test(id);
  const nearest = (fi, p) => {
    let best = null, bd = 2e-3;
    for (const v of byFace.get(fi) ?? []) { const d = Math.hypot(v[0] - p[0], v[1] - p[1], v[2] - p[2]); if (d < bd) { bd = d; best = v; } }
    return best;
  };
  let max = 0, where = '';
  for (const e of body.edges) {
    if (mitre || e.faces.length !== 2 || !e.faces.every(isBlend)) continue;
    const [a, b] = e.faces.map((f) => idx.get(f));
    for (const p of e.pts ?? []) {
      const va = nearest(a, p), vb = nearest(b, p);
      if (!va || !vb) continue;
      const d = va[3] * vb[3] + va[4] * vb[4] + va[5] * vb[5];
      const ang = Math.acos(Math.max(-1, Math.min(1, d)));
      if (ang > max) { max = ang; where = `${e.faces.join('|')} at ${p.map((x) => x.toFixed(2)).join(',')}`; }
    }
  }
  // faces of the tool left in the result (`F9.S.l1`: a flat cut where a band ends instead of blending)
  const left = body.faces.find((f) => /^F\d+\.S\.l\d+/.test(f.id));
  if (left) { max = Math.PI; where = `tool face ${left.id} left in the result`; }
  return { max, where };
}

function run(c) {
  const k = new Kernel(mod);
  const body = baseBody(k, c.base);
  const feats = [...BASES[c.base], feature(k, c, body)];
  if (c.step2) {
    // second feature on the body the first one made (the app's flow: concave edges first, then the convex ones)
    const r1 = req(k, 9, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: feats } });
    if (r1.result?.errors?.length || !r1.result?.body) return { errs: (r1.result?.errors ?? []).map(([, e]) => e.code), ms: 0 };
    feats.push(feature(k, c, r1.result.body, c.step2, 'F10'));
  }
  const part = { format: 'ck-part', version: 1, features: feats };
  const t0 = Date.now();
  const raw = k.request(JSON.stringify({ id: 2, msg: { type: 'regen', tag: 'main', part } }));
  const res = raw.json ? JSON.parse(raw.json) : { ok: false, trap: raw.trap };
  const errs = (res.result?.errors ?? []).map(([id, e]) => e.code + (e.params?.detail ? `(${String(e.params.detail).slice(0, 60)})` : ''));
  const ms = Date.now() - t0;
  if (res.trap || (!res.ok && !res.result)) return { errs: [`trap ${res.trap ?? ''}`], ms };
  const m = req(k, 3, { type: 'massProps', tag: 'main' });
  const v = req(k, 4, { type: 'validate', tag: 'main' });
  return { errs, vol: m.ok ? m.result?.volume : null, valid: !!(v.ok && v.result?.ok), ms, faces: res.result?.body?.faces?.length, open: openMeshEdges(raw, res), g1: g1Defect(raw, res, c.mitre) };
}

if (argv.includes('--edges')) {
  const k = new Kernel(mod);
  const b = baseBody(k, opt('--edges'));
  for (const e of b.edges) console.log(e.key.padEnd(24), JSON.stringify(e.mid), e.line ? 'line' : 'curve', JSON.stringify(e.pts?.[0]));
  process.exit(0);
}

if (argv.includes('--part')) {
  // write the part of one case (base + blend feature(s)) as a .ckpart for the kit snapshot renderer
  const c = cases.find((x) => x.name === opt('--part'));
  const k = new Kernel(mod);
  const feats = [...BASES[c.base], feature(k, c, baseBody(k, c.base))];
  if (c.step2) {
    const r1 = req(k, 9, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: feats } });
    feats.push(feature(k, c, r1.result.body, c.step2, 'F10'));
  }
  writeFileSync(opt('--out'), JSON.stringify({ format: 'ck-part', version: 1, features: feats }));
  process.exit(0);
}

if (argv.includes('--debug')) {
  // one case with regen debug on: the feature, errors, warnings and the trace labels
  const c = cases.find((x) => x.name === opt('--debug'));
  const k = new Kernel(mod);
  const f = feature(k, c, baseBody(k, c.base));
  console.log(JSON.stringify(f));
  const r = req(k, 2, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: [...BASES[c.base], f] }, debug: true });
  console.log('errors', JSON.stringify(r.result?.errors), 'warnings', JSON.stringify(r.result?.warnings));
  console.log(JSON.stringify(r.trace ?? r.debug ?? Object.keys(r)).slice(0, 4000));
  process.exit(0);
}

if (argv.includes('--emit')) {
  const dir = opt('--emit');
  mkdirSync(dir, { recursive: true });
  const out = [];
  const seen = new Set();
  for (const c of cases) {
    const k = new Kernel(mod);
    const body = baseBody(k, c.base);
    if (!seen.has(c.base)) {
      seen.add(c.base);
      const s = req(k, 5, { type: 'step', name: 'part' });
      writeFileSync(path.join(dir, `${c.base}.step`), s.result);
    }
    const f = feature(k, c, body);
    out.push({ name: c.name, base: c.base, op: c.op, picks: c.picks, faces: f.edges.map((e) => e.faces), face: f.face ?? null, step2: c.step2 ?? null, tolKind: c.curved ? 'curved' : null });
  }
  writeFileSync(path.join(dir, 'cases.json'), JSON.stringify(out, null, 1));
  console.log(`wrote ${out.length} cases to ${dir}`);
  process.exit(0);
}

// ---------------------------------------------------------------- run
const refFile = path.join(here, 'blend_reference.json');
const ref = existsSync(refFile) ? JSON.parse(readFileSync(refFile, 'utf8')) : {};
const only = opt('--only', '');
const rows = [];
let bad = 0, n = 0;
for (const c of cases) {
  if (only && !c.name.includes(only)) continue;
  n++;
  const rf = ref[c.name];
  let r;
  try { r = run(c); } catch (e) { r = { errs: [`script: ${e.message}`], ms: 0 }; }
  const curved = ['boss', 'holed', 'rounded', 'post2', 'post4', 'rrect5', 'rrect2', 'rrect6', 'rrectarm'].includes(c.base) || c.name.startsWith('var_') || c.curved;
  // variable radius: the radius law differs from OCCT's by about 1e-4 on an isolated edge (docs/blend-corpus.md), corners add a mitre
  // approximation; unequal radii at one vertex end in a mitre where OCCT blends a patch
  const tol = c.name.startsWith('var_corner') || c.name.startsWith('var_ring') ? 5e-4 : c.name.includes('unequal') ? 3e-4 : curved ? 1e-4 : 1e-5;
  let status, note = '';
  if (c.expect) {
    status = r.errs.some((e) => e.startsWith(c.expect)) ? 'ok' : 'FAIL';
    note = `expect ${c.expect}, got ${r.errs.join() || 'no error'}`;
  } else if (r.errs.length) {
    status = 'FAIL'; note = r.errs.join();
  } else if (!r.valid) {
    status = 'FAIL'; note = `invalid solid, vol ${r.vol?.toFixed(4)}`;
  } else if (r.g1 && c.g1 && r.g1.max > 5e-3) {
    status = 'FAIL'; note = `normal jump ${r.g1.max.toFixed(4)} rad across ${r.g1.where}, vol ${r.vol?.toFixed(4)}`;
  } else if (r.open) {
    status = 'FAIL'; note = `${r.open} open mesh edge(s) (a face is missing in the display mesh), vol ${r.vol?.toFixed(4)}`;
  } else if (!rf || rf.volume == null) {
    status = 'NOREF'; note = `vol ${r.vol?.toFixed(4)} (no OCCT reference${rf?.error ? ': ' + rf.error : ''})`;
  } else {
    const rel = Math.abs(r.vol - rf.volume) / rf.volume;
    status = rel <= tol ? 'ok' : 'FAIL';
    note = `vol ${r.vol.toFixed(4)} occt ${rf.volume.toFixed(4)} diff ${(r.vol - rf.volume).toFixed(4)} (rel ${rel.toExponential(1)})`;
  }
  if (status === 'FAIL') bad++;
  rows.push({ name: c.name, status, note, ms: r.ms });
  console.log(`${status.padEnd(5)} ${c.name.padEnd(34)} ${note}  [${r.ms} ms]`);
}
const okN = rows.filter((r) => r.status === 'ok').length;
console.log(`\n${okN}/${n} ok, ${bad} failed, ${rows.filter((r) => r.status === 'NOREF').length} without reference`);
const jo = opt('--json');
if (jo) writeFileSync(jo, JSON.stringify(rows, null, 1));
process.exit(bad ? 1 : 0);
