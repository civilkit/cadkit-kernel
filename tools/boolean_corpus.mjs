// Boolean coincidence corpus: ~70 cases built through the wasm (extrude / revolve add and cut, and the
// body-level `combine` feature), each with an analytic expected volume (and lump count where it is defined).
// Result classes per case: ok | okNoEffect (cutNoEffect where nothing should change) | booleanFailed | error:<code>
// | invalid (validate issues) | wrongVolume | wrongLumps | trap | unsupported (feature not in this wasm).
// usage: node tools/boolean_corpus.mjs [--wasm path] [--only substr] [--json out.json] [--md out.md] [--quiet]
// exit 1 when any case is not ok.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (n) => { const i = process.argv.indexOf(n); return i > 0 ? process.argv[i + 1] : null; };
const wasm = arg('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const only = arg('--only');
const mod = new WebAssembly.Module(readFileSync(wasm));
const PI = Math.PI;

// ---------------------------------------------------------------- feature builders
// Everything is built on the std planes. 'front': u=X, v=Y, w=Z (extrude +Z); 'top': u=X, v=-Z, w=Y.
let n = 0;
const bx = (u0, v0, u1, v1, w0, w1, pl = 'front') => ({ k: 'box', u0, v0, u1, v1, w0, w1, pl });
const cy = (cu, cv, r, w0, w1, pl = 'front') => ({ k: 'cyl', cu, cv, r, w0, w1, pl });
const rev = (pts, cz = 0) => ({ k: 'rev', pts, cz });
const sp = (cx, cyy, cz, r) => ({ k: 'sph', cx, cy: cyy, cz, r });
const add = (p, tag = null) => ({ op: 'add', p, tag });
const cut = (p) => ({ op: 'cut', p });
const comb = (op, target, tools, keepTools = false, extra = {}) => ({ combine: { op, target, tools, keepTools, ...extra } });
const mv = (tag, t) => ({ mv: { tag, t } });
const TOL = arg('--tol') ? Number(arg('--tol')) : 0;
const AUTO = process.argv.includes('--auto');

function features(steps) {
  const f = [];
  const ids = {};
  const face = (tag) => (/^L\d+$/.test(tag) ? { lump: Number(tag.slice(1)) } : { face: `${ids[tag]}.C1` });
  for (const s of steps) {
    n++;
    if (s.combine) {
      const c = s.combine;
      f.push({ type: 'combine', id: `C${n}`, op: c.op, target: face(c.target), tools: c.tools.map(face), keepTools: c.keepTools, ...(c.tolerance ? { tolerance: c.tolerance } : {}) });
      continue;
    }
    if (s.mv) {
      f.push({ type: 'moveBody', id: `M${n}`, bodies: [face(s.mv.tag)], translate: s.mv.t });
      continue;
    }
    if (s.tag) ids[s.tag] = `F${n}`;
    const p = s.p;
    const w0 = p.k === 'sph' || p.k === 'rev' ? p.cz : p.w0;
    const pl = p.k === 'sph' || p.k === 'rev' ? 'front' : p.pl;
    let plane = { kind: 'std', id: pl };
    if (w0 !== 0) {
      f.push({ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: pl }, offset: w0 });
      plane = { kind: 'feature', id: `P${n}` };
    }
    const sid = `S${n}`;
    if (p.k === 'box') {
      const pt = [[p.u0, p.v0], [p.u1, p.v0], [p.u1, p.v1], [p.u0, p.v1]];
      const points = pt.map(([x, y], i) => ({ id: `p${i}`, x, y }));
      const entities = pt.map((_, i) => ({ type: 'line', id: `e${i + 1}`, p1: `p${i}`, p2: `p${(i + 1) % 4}` }));
      f.push({ type: 'sketch', id: sid, plane, points, entities, constraints: [] });
      f.push({ type: 'extrude', id: `F${n}`, op: s.op, sketch: sid, end: 'blind', depth: p.w1 - p.w0, ...(s.op === 'cut' ? { flip: true } : {}), ...(TOL ? { tolerance: TOL } : {}), ...(AUTO ? { autoFuzzy: true } : {}) });
    } else if (p.k === 'cyl') {
      f.push({ type: 'sketch', id: sid, plane, points: [{ id: 'c', x: p.cu, y: p.cv }], entities: [{ type: 'circle', id: 'e1', c: 'c', r: p.r }], constraints: [] });
      f.push({ type: 'extrude', id: `F${n}`, op: s.op, sketch: sid, end: 'blind', depth: p.w1 - p.w0, ...(s.op === 'cut' ? { flip: true } : {}), ...(TOL ? { tolerance: TOL } : {}), ...(AUTO ? { autoFuzzy: true } : {}) });
    } else if (p.k === 'rev') {
      const points = p.pts.map(([x, y], i) => ({ id: `p${i}`, x, y }));
      const entities = p.pts.map((_, i) => ({ type: 'line', id: `e${i + 1}`, p1: `p${i}`, p2: `p${(i + 1) % p.pts.length}` }));
      f.push({ type: 'sketch', id: sid, plane, points, entities, constraints: [] });
      f.push({ type: 'revolve', id: `F${n}`, op: s.op, sketch: sid, axis: `e${p.pts.length}`, angle: 360, ...(TOL ? { tolerance: TOL } : {}), ...(AUTO ? { autoFuzzy: true } : {}) });
    } else {
      f.push({ type: 'sketch', id: sid, plane,
        points: [{ id: 'c', x: p.cx, y: p.cy }, { id: 'a', x: p.cx, y: p.cy - p.r }, { id: 'b', x: p.cx, y: p.cy + p.r }],
        entities: [{ type: 'arc', id: 'e1', c: 'c', p1: 'a', p2: 'b' }, { type: 'line', id: 'ax', p1: 'a', p2: 'b' }], constraints: [] });
      f.push({ type: 'revolve', id: `F${n}`, op: s.op, sketch: sid, axis: 'ax', angle: 360, ...(TOL ? { tolerance: TOL } : {}), ...(AUTO ? { autoFuzzy: true } : {}) });
    }
  }
  return f;
}

// ---------------------------------------------------------------- the cases
const cases = [];
const C = (name, group, steps, want, o = {}) => cases.push({ name, group, steps, want, ...o });
const cylV = (r, h) => PI * r * r * h;
const sphV = (r) => 4 / 3 * PI * r ** 3;

// --- coincident faces: boxes
C('box+box share a face, fuse', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10, 0, 20, 10, 0, 10))], 2000, { lumps: 1 });
C('box+box share a face, cut (no overlap)', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(10, 0, 20, 10, 0, 10))], 1000, { noEffectOk: true });
C('box+box share half a face, fuse', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10, 5, 20, 15, 0, 10))], 2000, { lumps: 1 });
C('small box on a big one (stack), fuse', 'coincident', [add(bx(0, 0, 20, 20, 0, 10)), add(bx(5, 5, 15, 15, 10, 20))], 4000 + 1000, { lumps: 1 });
C('box in box flush on 1 face, fuse', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), add(bx(5, 5, 15, 15, 0, 10))], 8000, { lumps: 1 });
C('box in box flush on 2 faces, fuse', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), add(bx(0, 5, 10, 15, 0, 10))], 8000, { lumps: 1 });
C('box in box flush on 3 faces, fuse', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), add(bx(0, 0, 10, 10, 0, 10))], 8000, { lumps: 1 });
C('box in box flush on 1 face, cut', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(5, 5, 15, 15, 0, 10))], 7000, { lumps: 1 });
C('box in box flush on 2 faces, cut', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(0, 5, 10, 15, 0, 10))], 7000, { lumps: 1 });
C('box in box flush on 3 faces, cut', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(0, 0, 10, 10, 0, 10))], 7000, { lumps: 1 });
C('cut slab, tool sides on all 4 walls', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(0, 0, 20, 20, 10, 30))], 4000, { lumps: 1 });
C('cut slab, tool sides on 2 walls', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(0, -5, 10, 25, -5, 30))], 4000, { lumps: 1 });
C('cut box with same top plane (flush notch)', 'coincident', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(5, 5, 15, 15, 10, 20))], 7000, { lumps: 1 });
C('fuse box with same top plane overlapping', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(5, 5, 15, 15, 0, 10))], 1000 + 1000 - 250, { lumps: 1 });
C('cut box by box coplanar top and bottom', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(5, 5, 15, 15, 0, 10))], 750, { lumps: 1 });
C('A union A (same box twice)', 'duplicate', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(0, 0, 10, 10, 0, 10))], 1000, { lumps: 1, noEffectOk: true });
C('A cut A (same box)', 'duplicate', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(0, 0, 10, 10, 0, 10))], 0, { emptyOk: true });
C('A cut A (same cylinder)', 'duplicate', [add(cy(0, 0, 5, 0, 10)), cut(cy(0, 0, 5, 0, 10))], 0, { emptyOk: true });
C('A union A (same cylinder)', 'duplicate', [add(cy(0, 0, 5, 0, 10)), add(cy(0, 0, 5, 0, 10))], cylV(5, 10), { lumps: 1, noEffectOk: true });
// --- contained
C('tiny box strictly inside, fuse', 'contained', [add(bx(0, 0, 20, 20, 0, 20)), add(bx(8, 8, 12, 12, 8, 12))], 8000, { lumps: 1, noEffectOk: true });
C('tiny box strictly inside, cut (void)', 'contained', [add(bx(0, 0, 20, 20, 0, 20)), cut(bx(8, 8, 12, 12, 8, 12))], 8000 - 64, { lumps: 1 });
C('cylinder strictly inside box, fuse', 'contained', [add(bx(0, 0, 20, 20, 0, 20)), add(cy(10, 10, 3, 5, 15))], 8000, { lumps: 1, noEffectOk: true });
// --- edge / vertex contact
C('two boxes sharing only an edge, fuse', 'contact', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10, 10, 20, 20, 0, 10))], 2000, { lumps: 2 });
C('two boxes sharing only a vertex, fuse', 'contact', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10, 10, 20, 20, 10, 20))], 2000, { lumps: 2 });
C('two boxes sharing only an edge, cut', 'contact', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(10, 10, 20, 20, 0, 10))], 1000, { noEffectOk: true });
// --- coaxial cylinders
C('cylinder + equal cylinder end to end, fuse', 'coaxial', [add(cy(0, 0, 5, 0, 10)), add(cy(0, 0, 5, 10, 20))], cylV(5, 20), { lumps: 1 });
C('cylinder + equal cylinder overlapping, fuse', 'coaxial', [add(cy(0, 0, 5, 0, 10)), add(cy(0, 0, 5, 5, 15))], cylV(5, 15), { lumps: 1 });
C('cylinder cut by equal cylinder overlapping', 'coaxial', [add(cy(0, 0, 5, 0, 10)), cut(cy(0, 0, 5, 5, 15))], cylV(5, 5), { lumps: 1 });
C('cylinder cut by equal cylinder past both ends', 'coaxial', [add(cy(0, 0, 5, 0, 10)), cut(cy(0, 0, 5, -5, 15))], 0, { emptyOk: true });
C('tube + core (equal radius), fuse', 'coaxial', [add(cy(0, 0, 10, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(0, 0, 5, 0, 10))], cylV(10, 10), { lumps: 1 });
C('plate with hole + equal plug longer, fuse', 'coaxial', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(0, 0, 5, -3, 13))], 40 * 40 * 10 + cylV(5, 6), { lumps: 1 });
C('big cylinder, blind hole of smaller radius', 'coaxial', [add(cy(0, 0, 10, 0, 10)), cut(cy(0, 0, 5, 5, 11))], cylV(10, 10) - cylV(5, 5), { lumps: 1 });
C('big cylinder + inner equal-length smaller, fuse', 'coaxial', [add(cy(0, 0, 10, 0, 10)), add(cy(0, 0, 5, 0, 10))], cylV(10, 10), { lumps: 1, noEffectOk: true });
C('stepped shaft, fuse', 'coaxial', [add(cy(0, 0, 10, 0, 10)), add(cy(0, 0, 5, 10, 20))], cylV(10, 10) + cylV(5, 10), { lumps: 1 });
C('cylinder with coaxial counterbore at the same radius as the bore below', 'coaxial', [add(cy(0, 0, 10, 0, 20)), cut(cy(0, 0, 5, 0, 20)), cut(cy(0, 0, 5, 5, 25))], cylV(10, 20) - cylV(5, 20), { lumps: 1, noEffectOk: true });
C('cylinder lying on another, same radius, cut', 'coaxial', [add(cy(0, 0, 10, 0, 20, 'top')), cut(cy(0, 0, 10, 5, 15, 'top'))], cylV(10, 10), { lumps: 2 });
// --- tangent bodies
C('sphere touching a plane, fuse', 'tangent', [add(bx(-10, -10, 10, 10, -10, 0)), add(sp(0, 0, 5, 5))], 4000 + sphV(5), { lumps: 2 });
C('cylinder lying on a plane (line contact), fuse', 'tangent', [add(bx(-10, -10, 10, 10, -10, 0)), add(cy(0, -5, 5, -8, 8, 'top'))], 4000 + cylV(5, 16), { lumps: 2 });
C('two spheres touching, fuse', 'tangent', [add(sp(0, 0, 0, 5)), add(sp(10, 0, 0, 5))], 2 * sphV(5), { lumps: 2 });
C('cylinder tangent to a box wall (line contact), fuse', 'tangent', [add(bx(0, 0, 10, 10, 0, 10)), add(cy(-5, 5, 5, 0, 10))], 1000 + cylV(5, 10), { lumps: 2 });
C('cylinder inside a hole of larger radius, tangent along a line, fuse', 'tangent', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 10, -1, 11)), add(cy(5, 0, 5, 0, 10))], 1600 * 10 - cylV(10, 10) + cylV(5, 10), { lumps: 2 });
C('sphere inscribed in a box (6 point contacts), fuse', 'tangent', [add(bx(0, 0, 10, 10, 0, 10)), add(sp(5, 5, 5, 5))], 1000, { lumps: 1, noEffectOk: true });
C('cylinder inscribed in a box (4 line contacts), cut', 'tangent', [add(bx(0, 0, 10, 10, 0, 10)), cut(cy(5, 5, 5, -1, 11))], 1000 - cylV(5, 10), {});
C('cylinder inscribed in a box, fuse', 'tangent', [add(bx(0, 0, 10, 10, 0, 10)), add(cy(5, 5, 5, 0, 10))], 1000, { lumps: 1, noEffectOk: true });
C('BJ-8 hole tangent to a wall (r3 at x=3), cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(3, 10, 3, -1, 11))], 4000 - cylV(3, 10), {});
C('hole tangent to two walls (corner, r3 at 3,3), cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(3, 3, 3, -1, 11))], 4000 - cylV(3, 10), {});
C('blind hole tangent to a wall, cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(3, 10, 3, 4, 11))], 4000 - cylV(3, 6), {});
// --- near-coincident offsets
for (const d of [1e-9, 1e-7, 1e-5, 1e-3]) {
  const t = `d=${d.toExponential(0)}`;
  C(`fuse boxes overlapping by thin ${t}`, 'near', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10 - d, 0, 20, 10, 0, 10))], 2000, {});
  C(`fuse boxes with gap ${t}`, 'near', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(10 + d, 0, 20, 10, 0, 10))], 1000 + 100 * (10 - d), { gap: d });
  C(`cut tool 4 walls flush, top plane off by ${t}`, 'near', [add(bx(0, 0, 20, 20, 0, 10)), cut(bx(0, 0, 20, 20, 10 - d, 30))], 400 * (10 - d), { gap: d, noEffectOk: d <= 1e-5 });
  C(`cut tool with wall off by ${t} (sliver wall)`, 'near', [add(bx(0, 0, 20, 20, 0, 10)), cut(bx(0, 0, 20 - d, 20, 5, 30))], 400 * 5 + (20 - d) * 20 * 0 + 20 * d * 5, {});
  C(`stack cylinders r5 and r5+${t}`, 'near', [add(cy(0, 0, 5, 0, 10)), add(cy(0, 0, 5 + d, 10, 20))], cylV(5, 10) + cylV(5 + d, 10), {});
  C(`plug r5+${t} in hole r5, fuse`, 'near', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(0, 0, 5 + d, 0, 10))], 16000, {});
  C(`plug r5-${t} in hole r5, fuse`, 'near', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(0, 0, 5 - d, 0, 10))], 16000 - (cylV(5, 10) - cylV(5 - d, 10)), { gap: d });
}
// --- zero-thickness slivers
for (const d of [1e-9, 1e-7, 1e-5, 1e-3]) {
  C(`cut leaves a sliver plate ${d.toExponential(0)} thick`, 'sliver', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(-1, -1, 11, 11, d, 11))], 100 * d, { gap: d, emptyOk: d <= 1e-7 });
}

// --- harder analytic families
C('perpendicular equal cylinders (Steinmetz), fuse', 'coaxial', [add(cy(0, 0, 5, -10, 10)), add(cy(0, 0, 5, -10, 10, 'top'))], 2 * cylV(5, 20) - 16 / 3 * 125, { lumps: 1 });
C('perpendicular equal cylinders (Steinmetz), cut', 'coaxial', [add(cy(0, 0, 5, -10, 10)), cut(cy(0, 0, 5, -10, 10, 'top'))], cylV(5, 20) - 16 / 3 * 125, { lumps: 2 });
C('sphere union itself', 'duplicate', [add(sp(0, 0, 0, 5)), add(sp(0, 0, 0, 5))], sphV(5), { lumps: 1, noEffectOk: true });
C('sphere cut by box through its centre (hemisphere)', 'tangent', [add(sp(0, 0, 0, 5)), cut(bx(-10, -10, 10, 10, -10, 0))], sphV(5) / 2, { lumps: 1 });
C('cylinder cut in half by a plane through its axis', 'coincident', [add(cy(0, 0, 5, 0, 10)), cut(bx(-10, -10, 0, 10, -1, 11))], cylV(5, 10) / 2, { lumps: 1 });
C('box fuse on a cylinder tangent plane, caps flush', 'tangent', [add(cy(0, 0, 5, 0, 10)), add(bx(5, -5, 15, 5, 0, 10))], cylV(5, 10) + 1000, { lumps: 2 });
C('cap box over a plate with a hole (closes it), fuse', 'coincident', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(bx(-20, -20, 20, 20, 10, 15))], 40 * 40 * 15 - cylV(5, 10), { lumps: 1 });
C('three boxes in a row sharing faces, fuse', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), add(bx(20, 0, 30, 10, 0, 10)), add(bx(10, 0, 20, 10, 0, 10))], 3000, { lumps: 1 });
C('hole centred on a box edge (half hole), cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(0, 10, 3, -1, 11))], 4000 - cylV(3, 10) / 2, { lumps: 1 });
C('hole centred on a box corner (quarter hole), cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(0, 0, 3, -1, 11))], 4000 - cylV(3, 10) / 4, { lumps: 1 });
C('nested tubes sharing the r5 surface (opposite normals), fuse', 'coaxial', [add(cy(0, 0, 10, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(0, 0, 5, 0, 10)), cut(cy(0, 0, 2, -1, 11))], cylV(10, 10) - cylV(2, 10), { lumps: 1 });
C('rib of equal height on a plate, flush ends', 'coincident', [add(bx(0, 0, 40, 40, 0, 5)), add(bx(0, 18, 40, 22, 5, 25))], 8000 + 40 * 4 * 20, { lumps: 1 });
C('bar cut by equal width bar offset along (shared side walls)', 'coincident', [add(bx(0, 0, 40, 10, 0, 10)), cut(bx(-5, 0, 20, 10, 0, 10))], 20 * 100, { lumps: 1 });
// --- more adversarial families
C('lying cylinder groove centred on the top face (half cylinder), cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(10, -10, 3, -1, 21, 'top'))], 4000 - cylV(3, 20) / 2, { lumps: 1 });
C('lying cylinder tangent to top and bottom faces, cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(cy(10, -5, 5, -1, 21, 'top'))], 4000 - cylV(5, 20), {});
C('hemisphere dome on a box top, fuse', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), add(sp(10, 10, 10, 5))], 4000 + sphV(5) / 2, { lumps: 1 });
C('spherical dimple centred on the top face, cut', 'tangent', [add(bx(0, 0, 20, 20, 0, 10)), cut(sp(10, 10, 10, 5))], 4000 - sphV(5) / 2, { lumps: 1 });
C('frustum on an equal-radius cylinder, fuse', 'coaxial', [add(cy(0, 0, 5, -10, 0, 'top')), add(rev([[0, 0], [5, 0], [2, 10], [0, 10]]))], cylV(5, 10) + PI * 10 / 3 * (25 + 10 + 4), { lumps: 1 });
C('cylinder tangent to a cylinder at a point (crossed axes), fuse', 'tangent', [add(cy(0, 0, 5, -10, 10)), add(cy(8, 0, 3, -10, 10, 'top'))], cylV(5, 20) + cylV(3, 20), { lumps: 2 });
C('six stacked boxes, fuse one after the other', 'coincident', [0, 1, 2, 3, 4, 5].map((i) => add(bx(0, 0, 10, 10, 2 * i, 2 * i + 2))), 1200, { lumps: 1 });
C('cut contained in an existing pocket (floor coplanar)', 'coincident', [add(bx(0, 0, 10, 10, 0, 10)), cut(bx(2, 2, 8, 8, 5, 10)), cut(bx(4, 4, 6, 6, 5, 10))], 1000 - 36 * 5, { noEffectOk: true, lumps: 1 });
C('boss half over a hole of equal radius, fuse', 'coaxial', [add(bx(-20, -20, 20, 20, 0, 10)), cut(cy(0, 0, 5, -1, 11)), add(cy(5, 0, 5, 10, 20))], 16000 - cylV(5, 10) + cylV(5, 10), { lumps: 1 });
C('concentric spheres, cut (hollow)', 'coaxial', [add(sp(0, 0, 0, 5)), cut(sp(0, 0, 0, 3))], sphV(5) - sphV(3), {});
C('concentric spheres, fuse', 'coaxial', [add(sp(0, 0, 0, 5)), add(sp(0, 0, 0, 3))], sphV(5), { lumps: 1, noEffectOk: true });
C('sphere cut by a cylinder of its own radius (tangent along the equator)', 'tangent', [add(sp(0, 0, 0, 5)), cut(cy(0, 0, 5, -10, 10))], 0, { emptyOk: true });
C('sphere fused with an equal-radius cylinder across its middle', 'tangent', [add(sp(0, 0, 0, 5)), add(cy(0, 0, 5, -3, 3))], sphV(5) + cylV(5, 6) - 132 * PI, { lumps: 1 });
C('spherical void strictly inside a box, cut', 'contained', [add(bx(0, 0, 20, 20, 0, 20)), cut(sp(10, 10, 10, 4))], 8000 - sphV(4), { lumps: 1 });
C('cylindrical void strictly inside a box, cut', 'contained', [add(bx(0, 0, 20, 20, 0, 20)), cut(cy(10, 10, 3, 5, 15))], 8000 - cylV(3, 10), { lumps: 1 });
C('two overlapping spheres, fuse', 'tangent', [add(sp(0, 0, 0, 5)), add(sp(5, 0, 0, 5))], 2 * sphV(5) - PI * 25 * 25 / 12, { lumps: 1 });
C('two overlapping spheres, cut', 'tangent', [add(sp(0, 0, 0, 5)), cut(sp(5, 0, 0, 5))], sphV(5) - PI * 25 * 25 / 12, { lumps: 1 });
C('napkin ring: sphere bored by a cylinder', 'tangent', [add(sp(0, 0, 0, 5)), cut(cy(0, 0, 3, -10, 10))], PI * 512 / 6, { lumps: 1 });
// bicylinder volume of radii R >= r (axes crossing at right angles): 8 * integral_0^r sqrt(R^2-x^2) sqrt(r^2-x^2) dx
const bicyl = (R, r) => { let t = 0; const n = 200000; for (let i = 0; i < n; i++) { const x = r * (i + 0.5) / n; t += Math.sqrt(R * R - x * x) * Math.sqrt(r * r - x * x); } return 8 * t * r / n; };
C('tube with a cross hole of the bore radius', 'coaxial', [add(cy(0, 0, 10, 0, 20)), cut(cy(0, 0, 6, -1, 21)), cut(cy(0, -10, 6, -15, 15, 'top'))], cylV(10, 20) - cylV(6, 20) - (bicyl(10, 6) - 16 / 3 * 216), { lumps: 1, relTol: 2e-4 });
C('countersunk hole (cone ends on the top face)', 'coaxial', [add(bx(-20, -20, 20, 20, 0, 10, 'top')), cut(rev([[0, -1], [3, -1], [3, 7], [6, 10], [6, 11], [0, 11]]))], 40 * 40 * 10 - cylV(3, 7) - PI * 3 / 3 * (36 + 18 + 9), { lumps: 1 });
C('three holes tangent to each other in a row, cut', 'tangent', [add(bx(0, 0, 18, 12, 0, 10)), cut(cy(3, 6, 3, -1, 11)), cut(cy(9, 6, 3, -1, 11)), cut(cy(15, 6, 3, -1, 11))], 18 * 12 * 10 - 3 * cylV(3, 10), {});
C('slot whose side is tangent to a hole, cut', 'tangent', [add(bx(0, 0, 30, 20, 0, 10)), cut(cy(10, 10, 3, -1, 11)), cut(bx(13, 0, 20, 20, -1, 11))], 30 * 20 * 10 - cylV(3, 10) - 7 * 20 * 10, {});

// --- re-drilled hole after a pad was fused under it (matx-corner-bug): the second cut coincides with the first hole's wall
const rd = (r, name, steps, want, o = {}) => C(name, 'redrill', steps, want, { meshClosed: true, ...o });
const plate = add(bx(0, 0, 40, 40, 0, 4)), pad = add(bx(5, 5, 30, 30, -3, 0));
rd(3.15, 'redrill A: plate, hole, pad under it, same drill again (ends on plate top)', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad, cut(cy(10, 10, 3.15, -6, 4))], 1600 * 4 + 625 * 3 - cylV(3.15, 7));
rd(3.15, 'redrill B: plate, pad, one drill (control)', [plate, pad, cut(cy(10, 10, 3.15, -6, 4))], 1600 * 4 + 625 * 3 - cylV(3.15, 7));
rd(3.15, 'redrill C: plate, hole, pad, no second drill (control)', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad], 1600 * 4 + 625 * 3 - cylV(3.15, 4));
rd(3.1, 'redrill D: as A, second drill r=3.1', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad, cut(cy(10, 10, 3.1, -6, 4))], 1600 * 4 + 625 * 3 - cylV(3.15, 4) - cylV(3.1, 3));
rd(0, 'redrill: second drill passes the plate top (-6..6)', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad, cut(cy(10, 10, 3.15, -6, 6))], 1600 * 4 + 625 * 3 - cylV(3.15, 7));
rd(0, 'redrill: second drill of a larger radius', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad, cut(cy(10, 10, 4, -6, 6))], 1600 * 4 + 625 * 3 - cylV(4, 7));
rd(0, 'redrill: second drill of a smaller radius (0.5)', [plate, cut(cy(10, 10, 3.15, -1, 5)), pad, cut(cy(10, 10, 2.65, -6, 6))], 1600 * 4 + 625 * 3 - cylV(3.15, 4) - cylV(2.65, 3));
rd(0, 'redrill: coaxial blind hole re-drilled after the pad (pad only)', [plate, cut(cy(10, 10, 3.15, 1, 5)), pad, cut(cy(10, 10, 3.15, -2, 4))], 1600 * 4 + 625 * 3 - cylV(3.15, 6));
rd(0, 'redrill: blind hole re-drilled, same blind depth', [plate, cut(cy(10, 10, 3.15, 1, 5)), pad, cut(cy(10, 10, 3.15, 1, 4))], 1600 * 4 + 625 * 3 - cylV(3.15, 3), { noEffectOk: true });
rd(0, 'redrill: boss re-cut (cylinder fused onto the plate, then cut by the same cylinder)', [plate, add(cy(30, 30, 3.15, 4, 9)), cut(cy(30, 30, 3.15, 3, 9))], 1600 * 4 - cylV(3.15, 1));
rd(0, 'redrill: boss on a plate re-cut after a pad', [plate, add(cy(30, 30, 3.15, 4, 9)), pad, cut(cy(30, 30, 3.15, 4, 9))], 1600 * 4 + 625 * 3);

// --- body-level (combine) cases; unsupported (and not counted) on a wasm without the feature


// --- body-level (combine) cases: two lumps overlapping inside one document (second box built apart, then moved)
const two = (dx, extra = []) => [add(bx(0, 0, 10, 10, 0, 10), 'A'), add(bx(30, 0, 40, 10, 0, 10), 'B'), mv('B', [dx, 0, 0]), ...extra];
C('combine union of two overlapping lumps', 'combine', two(-25, [comb('union', 'A', ['B'])]), 1500, { lumps: 1 });
C('combine cut of overlapping lumps', 'combine', two(-25, [comb('cut', 'A', ['B'])]), 500, { lumps: 1 });
C('combine intersect of overlapping lumps', 'combine', two(-25, [comb('intersect', 'A', ['B'])]), 500, { lumps: 1 });
C('combine split by an overlapping lump', 'combine', two(-25, [comb('split', 'A', ['B'])]), 1000, { lumps: 2 });
C('combine cut, keepTools', 'combine', two(-25, [comb('cut', 'A', ['B'], true)]), 500 + 1000, { lumps: 2 });
C('combine union of lumps sharing a face', 'combine', two(-20, [comb('union', 'A', ['B'])]), 2000, { lumps: 1 });
C('combine cut of lumps sharing a face (no effect)', 'combine', two(-20, [comb('cut', 'A', ['B'])]), 2000, { noEffectOk: true });
C('combine union of identical lumps', 'combine', two(-30, [comb('union', 'A', ['B'])]), 1000, { lumps: 1, noEffectOk: true });
C('combine intersect of identical lumps', 'combine', two(-30, [comb('intersect', 'A', ['B'])]), 1000, { lumps: 1, noEffectOk: true });
C('combine cut of identical lumps (empty)', 'combine', two(-30, [comb('cut', 'A', ['B'])]), 0, { emptyOk: true });
C('combine intersect of disjoint lumps', 'combine', two(0, [comb('intersect', 'A', ['B'])]), 2000, { wantErr: 'combineEmpty' });
C('combine union of lumps touching along an edge', 'combine', [add(bx(0, 0, 10, 10, 0, 10), 'A'), add(bx(30, 30, 40, 40, 0, 10), 'B'), mv('B', [-20, -20, 0]), comb('union', 'A', ['B'])], 2000, { lumps: 2 });
C('combine n-ary union of three overlapping lumps', 'combine', [add(bx(0, 0, 10, 10, 0, 10), 'A'), add(bx(30, 0, 40, 10, 0, 10), 'B'), add(bx(60, 0, 70, 10, 0, 10), 'D'), mv('B', [-22, 0, 0]), mv('D', [-44, 0, 0]), comb('union', 'A', ['B', 'D'])], 2600, { lumps: 1 });
C('combine union leaves a third lump alone', 'combine', [add(bx(0, 0, 10, 10, 0, 10), 'A'), add(bx(30, 0, 40, 10, 0, 10), 'B'), add(bx(100, 0, 110, 10, 0, 10), 'D'), mv('B', [-25, 0, 0]), comb('union', 'A', ['B'])], 2500, { lumps: 2 });
C('combine coaxial equal cylinders, union', 'combine', [add(cy(0, 0, 5, 0, 10), 'A'), add(cy(30, 0, 5, 0, 10), 'B'), mv('B', [-30, 0, 5]), comb('union', 'A', ['B'])], cylV(5, 15), { lumps: 1 });
C('combine coaxial equal cylinders, cut', 'combine', [add(cy(0, 0, 5, 0, 10), 'A'), add(cy(30, 0, 5, 0, 10), 'B'), mv('B', [-30, 0, 5]), comb('cut', 'A', ['B'])], cylV(5, 5), { lumps: 1 });
C('combine coaxial equal cylinders, intersect', 'combine', [add(cy(0, 0, 5, 0, 10), 'A'), add(cy(30, 0, 5, 0, 10), 'B'), mv('B', [-30, 0, 5]), comb('intersect', 'A', ['B'])], cylV(5, 5), { lumps: 1 });
C('combine sphere cut by a lump through its centre', 'combine', [add(sp(0, 0, 0, 5)), add(bx(30, -10, 50, 10, -10, 0), 'B'), mv('B', [-40, 0, 0]), comb('cut', 'L0', ['B'])], sphV(5) / 2, { lumps: 1 });
C('combine near-coincident lumps (overlap 1e-4), tolerance 1e-3', 'combine', two(-20 - 1e-4, [comb('union', 'A', ['B'], false, { tolerance: 1e-3 })]), 2000, { lumps: 1, gap: 1e-4 });

// --- known parts
const hook = path.join(here, 'bench/benchcad/j_hook_001383.ckpart.json');
const frame = path.join(here, 'bench/benchcad/rect_frame_008694.ckpart.json');
if (existsSync(hook)) cases.push({ name: 'j_hook (BenchCAD)', group: 'known', part: JSON.parse(readFileSync(hook, 'utf8')), want: 32302.8364, relTol: 1e-3 });
if (existsSync(frame)) cases.push({ name: 'rect_frame (BenchCAD)', group: 'known', part: JSON.parse(readFileSync(frame, 'utf8')), want: 43446.9484, relTol: 1e-3 });

// ---------------------------------------------------------------- runner
function rq(k, msg) {
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) return { trap: r.trap };
  try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; }
}

// mesh edges used by exactly one triangle in the regen display mesh (0 for a watertight tessellation); -1 when there is no mesh
function openMeshEdges(rr, r) {
  const mesh = r.result?.body?.mesh;
  if (!mesh || !rr.bytes) return -1;
  const jl = new DataView(rr.bytes.buffer, rr.bytes.byteOffset).getUint32(0, true);
  const base = (4 + jl + 7) & ~7;
  const v = mesh.positions ?? mesh.pos;
  const pos = new Float32Array(rr.bytes.buffer.slice(rr.bytes.byteOffset + base + v.offset, rr.bytes.byteOffset + base + v.offset + v.len * 4));
  const key = (i) => [0, 1, 2].map((q) => Math.round(pos[i * 3 + q] / 1e-4)).join(',');
  const uses = new Map();
  for (let t = 0; t < pos.length / 9; t++) {
    const vs = [3 * t, 3 * t + 1, 3 * t + 2].map(key);
    for (let e = 0; e < 3; e++) { const a = vs[e], b = vs[(e + 1) % 3]; if (a === b) continue; const kk = a < b ? a + '|' + b : b + '|' + a; uses.set(kk, (uses.get(kk) ?? 0) + 1); }
  }
  return [...uses.values()].filter((x) => x === 1).length;
}

function run(c) {
  if (TOL && c.gap && c.gap <= TOL * 1.0001) c = { ...c, noEffectOk: true, emptyOk: true }; // a gap within the fuzzy value is coincident by definition
  const k = new Kernel(mod);
  const part = c.part ?? { format: 'ck-part', version: 1, features: features(c.steps) };
  if (arg('--dump') && c.name.includes(arg('--dump'))) writeFileSync(arg('--dumpto') ?? '/dev/stdout', JSON.stringify(part));
  const t0 = Date.now();
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  let r; if (rr.trap) r = { trap: rr.trap }; else { try { r = JSON.parse(rr.json); } catch { r = { trap: 'unparsable reply' }; } }
  const ms = Date.now() - t0;
  if (r.trap) return { cls: 'trap', detail: r.trap, ms };
  const errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, ...e }));
  if (c.wantErr && errs.some((e) => e.code === 'unknownFeature')) return { cls: 'unsupported', detail: 'unknownFeature', ms };
  if (c.wantErr) return errs.some((e) => e.code === c.wantErr) ? { cls: 'ok', detail: `error ${c.wantErr}`, ms } : { cls: 'wrongVolume', detail: `expected error ${c.wantErr}, got ${errs.map((e) => e.code).join(',') || 'none'}`, ms };
  const bad = errs.filter((e) => !(c.noEffectOk && e.code === 'cutNoEffect'));
  const sawNoEffect = errs.some((e) => e.code === 'cutNoEffect');
  const warns = (r.result?.warnings ?? []).map(([, w]) => w.code);
  if (c.group === 'combine' && errs.some((e) => /todo|unsupported|unknown/i.test(e.code))) return { cls: 'unsupported', detail: errs[0].code, ms };
  const m = rq(k, { type: 'massProps', tag: 'main', bodies: true });
  const vol = m.ok ? m.result?.volume ?? null : null;
  const fail = bad.find((e) => e.code === 'booleanFailed');
  if (fail) return { cls: 'booleanFailed', detail: `${fail.id} ${JSON.stringify(Object.fromEntries(Object.entries(fail).filter(([k]) => !['id', 'code'].includes(k))))}`, ms };
  if (bad.length) {
    if (c.emptyOk && bad.every((e) => ['nothingToCut', 'emptyResult', 'cutEmpty', 'booleanEmpty'].includes(e.code))) return { cls: 'ok', detail: `empty (${bad[0].code})`, ms };
    return { cls: `error:${bad.map((e) => e.code).join(',')}`, detail: bad.map((e) => `${e.id}:${e.code}${e.params && e.code === 'booleanVolumeMismatch' ? ' ' + JSON.stringify({ what: e.params.what, min: e.params.expected_min, max: e.params.expected_max, got: e.params.got }) : ''}`).join(' '), ms };
  }
  if (vol == null || vol === 0) {
    if (c.emptyOk || c.want === 0) return { cls: 'ok', detail: 'empty', ms };
    return { cls: 'wrongVolume', detail: `no body, want ${c.want}`, ms };
  }
  const v = rq(k, { type: 'validate', tag: 'main' });
  const issues = v.result?.issues ?? [];
  const relTol = c.relTol ?? 1e-4;
  const tolV = Math.max(relTol * Math.abs(c.want), c.want < 1 ? 1e-4 : 0);
  // a gap or thin overlap below the welding tolerance may legitimately be treated as touching (extra volume <= d*area)
  const slack = (c.gap ? c.gap * 400 : 0) + ((TOL || AUTO) ? (TOL || 1e-3) * 500 : 0);
  if (Math.abs(vol - c.want) > tolV + slack) return { cls: 'wrongVolume', detail: `${vol.toPrecision(9)} want ${c.want.toPrecision(9)} (${((vol - c.want) / c.want).toExponential(1)})`, ms };
  if (issues.length) return { cls: 'invalid', detail: [...new Set(issues.map((i) => i.code))].join(','), ms };
  if (c.meshClosed) { const o = openMeshEdges(rr, r); if (o !== 0) return { cls: 'openMesh', detail: `${o} open mesh edges (vol ${vol.toPrecision(9)})`, ms }; }
  const lumps = m.result?.bodies?.length ?? null;
  if (c.lumps != null && lumps != null && lumps !== c.lumps && !(c.gap && lumps >= 1)) return { cls: 'wrongLumps', detail: `${lumps} want ${c.lumps}`, ms };
  return { cls: sawNoEffect ? 'okNoEffect' : 'ok', detail: vol.toPrecision(9) + (warns.includes('booleanFuzzy') ? ' fuzzy' : ''), ms };
}


// ---------------------------------------------------------------- grid fuzz (--fuzz N [--seed S] [--ops K] [--grid G])
// Random add / cut sequences of integer-grid boxes: planes coincide constantly, and the exact volume is the count of unit cells.
if (arg('--fuzz')) {
  let seed = Number(arg('--seed') ?? 1);
  const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const G = Number(arg('--grid') ?? 5), K = Number(arg('--ops') ?? 4), N = Number(arg('--fuzz'));
  let bad = 0;
  for (let t = 0; t < N; t++) {
    const cells = new Set();
    const steps = [];
    const ri = (lo, hi) => lo + Math.floor(rnd() * (hi - lo + 1));
    for (let k = 0; k < K; k++) {
      const x0 = ri(0, G - 1), y0 = ri(0, G - 1), z0 = ri(0, G - 1);
      const x1 = ri(x0 + 1, G), y1 = ri(y0 + 1, G), z1 = ri(z0 + 1, G);
      const isCut = k > 0 && rnd() < 0.45;
      steps.push(isCut ? cut(bx(x0, y0, x1, y1, z0, z1)) : add(bx(x0, y0, x1, y1, z0, z1)));
      for (let x = x0; x < x1; x++) for (let y = y0; y < y1; y++) for (let z = z0; z < z1; z++) { const key = `${x},${y},${z}`; if (isCut) cells.delete(key); else cells.add(key); }
    }
    const c = { name: `fuzz ${t}`, group: 'fuzz', steps, want: cells.size, noEffectOk: true, emptyOk: cells.size === 0 };
    const r = run(c);
    if (!r.cls.startsWith('ok')) { bad++; console.log(`FAIL fuzz ${t}: ${r.cls} ${r.detail}\n   ${JSON.stringify(steps.map((s) => [s.op, s.p.u0, s.p.v0, s.p.w0, s.p.u1, s.p.v1, s.p.w1]))}`); }
  }
  console.log(`boolean_corpus fuzz: ${N - bad}/${N} ok`);
  process.exit(bad ? 1 : 0);
}

const rows = [];
for (const c of cases) {
  if (only && !c.name.includes(only) && !c.group.includes(only)) continue;
  const res = run(c);
  rows.push({ name: c.name, group: c.group, ...res });
  if (!process.argv.includes('--quiet')) console.log(`${res.cls.startsWith('ok') ? 'ok  ' : res.cls === 'unsupported' ? 'n/a ' : 'FAIL'} [${c.group}] ${c.name}: ${res.cls} ${res.detail} (${res.ms} ms)`);
}
// bodyBoolean request: preview of a lump operation, nothing committed
if (!only || 'bodyBoolean'.includes(only)) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: features(two(-25)) };
  rq(k, { type: 'regen', tag: 'main', part });
  const q = (msg) => rq(k, { ...msg, tag: 'main' });
  const A = `${part.features.find((f) => f.type === 'extrude').id}.C1`;
  const B = `${part.features.filter((f) => f.type === 'extrude')[1].id}.C1`;
  const r1 = q({ type: 'bodyBoolean', op: 'intersect', target: A, tools: [B] });
  const after = q({ type: 'massProps' });
  const ok = r1.ok && Math.abs(r1.result?.volume - 500) < 1e-6 && r1.result?.ok && Math.abs(after.result?.volume - 2000) < 1e-6;
  rows.push({ name: 'bodyBoolean request previews an intersect without committing', group: 'combine', cls: ok ? 'ok' : (r1.result?.ok === undefined && !r1.ok ? 'unsupported' : 'wrongVolume'), detail: JSON.stringify(r1.result ?? r1.error).slice(0, 120), ms: 0 });
  console.log(`${ok ? 'ok  ' : 'FAIL'} [combine] bodyBoolean request: ${JSON.stringify(r1.result ?? r1.error).slice(0, 160)}`);
}
const counted = rows.filter((r) => r.cls !== 'unsupported');
const okN = counted.filter((r) => r.cls.startsWith('ok')).length;
const by = {};
for (const r of counted) by[r.cls.replace(/^ok.*/, 'ok')] = (by[r.cls.replace(/^ok.*/, 'ok')] ?? 0) + 1;
console.log(`boolean_corpus: ${okN}/${counted.length} ok  ${JSON.stringify(by)}  (${rows.length - counted.length} unsupported)`);
const jo = arg('--json');
if (jo) writeFileSync(jo, JSON.stringify(rows, null, 1));
const mo = arg('--md');
if (mo) {
  const out = ['| group | case | result | detail | ms |', '|---|---|---|---|---|', ...rows.map((r) => `| ${r.group} | ${r.name} | ${r.cls} | ${r.detail} | ${r.ms} |`), '', `ok ${okN}/${counted.length} ${JSON.stringify(by)}`];
  writeFileSync(mo, out.join('\n') + '\n');
}
process.exit(okN === counted.length ? 0 : 1);
