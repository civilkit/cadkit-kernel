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
function feature(k, c, body) {
  const edges = c.picks.map((p) => {
    const e = findEdge(body, p.at);
    if (!e) throw new Error(`${c.name}: no edge near ${p.at}`);
    const ref = { faces: [...e.faces], at: p.at };
    if (p.radii) ref.radii = p.radii;
    if (p.r) ref.radius = p.r;
    return ref;
  });
  const id = 'F9';
  const o = c.op;
  const pr = c.propagate ? { propagate: true } : {};
  switch (o.k) {
    case 'fillet': return o.r > 0 ? { type: 'fillet', id, radius: o.r, edges, ...pr } : { type: 'fillet', id, radius: Math.max(...c.picks.map((p) => p.r)), edges };
    case 'varfillet': return { type: 'fillet', id, filletType: 'variable', radii: o.radii, edges };
    case 'chamfer': return { type: 'chamfer', id, chamferType: 'equal', distance: o.d, edges, ...pr };
    case 'chamfer2':
    case 'chamferA': {
      const e0 = findEdge(body, c.picks[0].at);
      const face = faceByNormal(body, e0, o.face);
      if (!face) throw new Error(`${c.name}: no face with normal ${o.face}`);
      return o.k === 'chamfer2'
        ? { type: 'chamfer', id, chamferType: 'distanceDistance', distance: o.d1, distance2: o.d2, face, edges, ...pr }
        : { type: 'chamfer', id, chamferType: 'angleDistance', distance: o.d, angle: o.a, face, edges, ...pr };
    }
  }
  throw new Error('op');
}
function run(c) {
  const k = new Kernel(mod);
  const body = baseBody(k, c.base);
  const part = { format: 'ck-part', version: 1, features: [...BASES[c.base], feature(k, c, body)] };
  const t0 = Date.now();
  const res = req(k, 2, { type: 'regen', tag: 'main', part });
  const errs = (res.result?.errors ?? []).map(([id, e]) => e.code + (e.params?.detail ? `(${String(e.params.detail).slice(0, 60)})` : ''));
  const ms = Date.now() - t0;
  if (res.trap || (!res.ok && !res.result)) return { errs: [`trap ${res.trap ?? ''}`], ms };
  const m = req(k, 3, { type: 'massProps', tag: 'main' });
  const v = req(k, 4, { type: 'validate', tag: 'main' });
  return { errs, vol: m.ok ? m.result?.volume : null, valid: !!(v.ok && v.result?.ok), ms, faces: res.result?.body?.faces?.length };
}

if (argv.includes('--edges')) {
  const k = new Kernel(mod);
  const b = baseBody(k, opt('--edges'));
  for (const e of b.edges) console.log(e.key.padEnd(24), JSON.stringify(e.mid), e.line ? 'line' : 'curve', JSON.stringify(e.pts?.[0]));
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
    out.push({ name: c.name, base: c.base, op: c.op, picks: c.picks, faces: f.edges.map((e) => e.faces), face: f.face ?? null, tolKind: c.curved ? 'curved' : null });
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
  const curved = ['boss', 'holed', 'rounded'].includes(c.base) || c.name.startsWith('var_') || c.curved;
  const tol = curved ? 1e-4 : 1e-5;
  let status, note = '';
  if (c.expect) {
    status = r.errs.some((e) => e.startsWith(c.expect)) ? 'ok' : 'FAIL';
    note = `expect ${c.expect}, got ${r.errs.join() || 'no error'}`;
  } else if (r.errs.length) {
    status = 'FAIL'; note = r.errs.join();
  } else if (!r.valid) {
    status = 'FAIL'; note = `invalid solid, vol ${r.vol?.toFixed(4)}`;
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
