// Wrap corpus: sketch text and closed profiles laid on cylindrical faces (feature `wrap`, docs/proposals/wrap-feature.md), built through the wasm.
// Cases: text cut and add on three radii x three heights, bores, a rectangle label (exact planar walls), offsets, a taper, and the refusals.
// Checks per case: no error / the expected error code, validate (mesh:true) clean (the display mesh may carry forgiven hairline notes where an exact circular wall meets the host), and for the text cases the
// volume change = glyph area x depth x (1 +- depth / 2 r) (the area comes from the `textOutline` request, a shoelace over the loops) within 1.5 %.
// Phase 2: `curve ...` cases lay open curves on a cylinder with projectCurve method:"wrap" (length = sketch length, every point at radius r, helix, sweep consumer, refusals).
// usage: node tools/wrap_corpus.mjs [--wasm path] [--only substr] [--quiet]
// exit 1 when any case fails.
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (n) => { const i = process.argv.indexOf(n); return i > 0 ? process.argv[i + 1] : null; };
const wasm = arg('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const only = arg('--only');
const quiet = process.argv.includes('--quiet');
const mod = new WebAssembly.Module(readFileSync(wasm));
const rqk = (k, msg) => { const r = k.request(JSON.stringify({ id: 1, msg })); if (r.trap) return { trap: r.trap }; try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; } };

// a tube about z from 0 to h: outer radius ro (face F1.S.e1), bore ri (face F1.S.e2; 0 = solid)
const tube = (ro, ri, h) => [
  { type: 'sketch', id: 'S1', plane: { kind: 'std', id: 'front' }, points: [{ id: 'p1', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'p1', r: ro }, ...(ri ? [{ type: 'circle', id: 'e2', c: 'p1', r: ri }] : [])], constraints: [] },
  { type: 'extrude', id: 'F1', op: 'add', sketch: 'S1', end: 'blind', depth: h },
];
// sketches on the plane through the axis seen from +x (sketch x = +y, sketch y = +z)
const text = (string, o = {}) => ({ type: 'text', id: 't1', string, font: 'gauge', height: 6, at: [0, 45], anchor: ['center', 'middle'], ...o });
const textSketch = (t) => ({ type: 'sketch', id: 'S2', plane: { kind: 'std', id: 'yz' }, points: [], entities: [t], constraints: [] });
const rectSketch = (u0, u1, w0, w1) => {
  const pts = [[u0, w0], [u1, w0], [u1, w1], [u0, w1]];
  return { type: 'sketch', id: 'S2', plane: { kind: 'std', id: 'yz' }, points: pts.map(([x, y], i) => ({ id: 'p' + i, x, y })), entities: pts.map((_, i) => ({ type: 'line', id: 'e' + i, p1: 'p' + i, p2: 'p' + ((i + 1) % 4) })), constraints: [] };
};
const wrap = (o = {}) => ({ type: 'wrap', id: 'F3', sketch: 'S2', face: 'F1.S.e1', op: 'cut', depth: 0.4, ...o });

const cases = [];
const C = (name, o) => cases.push({ name, ...o });
for (const r of [10, 40, 150]) for (const h of [2, 6, 20]) {
  if (r < 20 && h > 6) continue; // a string of this height would pass one turn
  const s = r < 20 ? 'CAD' : 'CADKIT 12';
  for (const [op, d] of [['cut', 0.4], ['add', 0.5]])
    C(`text r${r} h${h} ${op}`, { feats: [...tube(r, 0, 120), textSketch(text(s, { height: h, at: [0, 60] })), wrap({ op, depth: d })], text: text(s, { height: h }), r, op, d, ro: r });
}
for (const font of ['gauge', 'engrave'])
  for (const [op, d] of [['cut', 0.4], ['add', 0.5]])
    C(`mug ${font} ${op}`, { feats: [...tube(40, 37, 90), textSketch(text('CADKIT 1/2', { font })), wrap({ op, depth: d })], text: text('CADKIT 1/2', { font }), r: 40, op, d });
for (const [op, d] of [['cut', 0.4], ['add', 0.4]])
  C(`bore ${op}`, { feats: [...tube(40, 37, 90), textSketch(text('CADKIT 1/2')), wrap({ face: 'F1.S.e2', op, depth: d })], text: text('CADKIT 1/2'), r: 37, op, d, bore: true });
C('rectangle label cut', { feats: [...tube(40, 0, 90), rectSketch(-10, 10, 40, 50), wrap({ depth: 0.8 })] });
C('rectangle label add', { feats: [...tube(40, 0, 90), rectSketch(-10, 10, 40, 50), wrap({ op: 'add', depth: 0.8 })] });
C('rectangle taper 10', { feats: [...tube(40, 0, 90), rectSketch(-10, 10, 39, 51), wrap({ depth: 2, taper: 10 })] });
C('offsets', { feats: [...tube(40, 0, 90), rectSketch(-8, 8, 40, 50), wrap({ alongAxis: 7, around: 30, rotation: 35, flip: true })] });
C('0.999 turn', { feats: [...tube(10, 0, 40), rectSketch(-0.999 * Math.PI * 10, 0.999 * Math.PI * 10, 10, 15), wrap({ depth: 0.3 })] });
C('through a thin wall', { feats: [...tube(40, 37, 90), rectSketch(-6, 6, 40, 50), wrap({ depth: 4 })], warn: 'wrapCutThrough' });
C('too wide', { feats: [...tube(10, 0, 40), textSketch(text('CADKIT CADKIT CADKIT', { at: [0, 20] })), wrap({ depth: 0.3 })], err: 'wrapProfileTooWide' });
C('past the end', { feats: [...tube(40, 0, 90), rectSketch(-5, 5, 85, 95), wrap({ depth: 0.5 })], err: 'wrapProfileOffFace' });
C('plane face', { feats: [...tube(40, 0, 90), rectSketch(-5, 5, 40, 50), wrap({ face: 'F1.C1' })], err: 'wrapFaceUnsupported' });
C('bore add too deep', { feats: [...tube(40, 37, 90), rectSketch(-5, 5, 40, 50), wrap({ face: 'F1.S.e2', op: 'add', depth: 40 })], err: 'wrapDepthTooLarge' });
C('no sketch', { feats: [...tube(40, 0, 90), { type: 'wrap', id: 'F3', face: 'F1.S.e1', op: 'cut', depth: 1 }], err: 'wrapNoInput' });

// phase 2: open curves (projectCurve method wrap). `curve`: expected marker length (null: only the radius is checked), r the cylinder radius
const curveSketch = (pts, ents) => ({ type: 'sketch', id: 'S2', plane: { kind: 'std', id: 'yz' }, points: pts.map(([x, y], i) => ({ id: 'p' + i, x, y })), entities: ents, constraints: [] });
const lineS = (a, b) => curveSketch([a, b], [{ type: 'line', id: 'e0', p1: 'p0', p2: 'p1' }]);
const wcurve = (o = {}) => ({ type: 'projectCurve', id: 'F5', sketch: 'S2', faces: ['F1.S.e1'], method: 'wrap', ...o });
for (const r of [10, 40, 150]) {
  C(`curve spline r${r}`, { feats: [...tube(r, 0, 120), curveSketch([[0, 40], [8, 46], [16, 40], [24, 46], [32, 42]], [{ type: 'spline', id: 'e0', pts: ['p0', 'p1', 'p2', 'p3', 'p4'] }]), wcurve()], curve: { r } });
  C(`curve line r${r}`, { feats: [...tube(r, 0, 120), lineS([0, 30], [40, 55]), wcurve()], curve: { r, length: Math.hypot(40, 25) } });
}
C('curve 3-turn helix', { feats: [...tube(20, 0, 100), lineS([0, 10], [3 * 2 * Math.PI * 20, 25]), wcurve()], curve: { r: 20, length: 3 * Math.hypot(2 * Math.PI * 20, 5), turns: 3 } });
C('curve in a bore', { feats: [...tube(40, 37, 90), lineS([10, 40], [20, 40]), wcurve({ faces: ['F1.S.e2'] })], curve: { r: 37, length: 10 } });
C('curve offsets', { feats: [...tube(40, 0, 90), lineS([0, 40], [0, 50]), wcurve({ alongAxis: 5, around: 30, rotation: 90, flip: true })], curve: { r: 40, length: 10 } });
C('curve sweep consumer', { feats: [...tube(20, 0, 100), lineS([0, 45], [10, 45]), wcurve(), { type: 'sketch', id: 'S3', plane: { kind: 'std', id: 'xz' }, points: [{ id: 'c', x: 20, y: 45 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 1 }], constraints: [] }, { type: 'sweep', id: 'F6', op: 'add', sketch: 'S3', path: { kind: 'curve', id: 'F5' } }], curve: { r: 20, length: 10 }, brepOnly: true }); // B-rep valid; the display mesh of a pipe half inside the cylinder is open at a seam (sweep tessellation, not the curve): deferred
C('curve past the end', { feats: [...tube(40, 0, 90), lineS([0, 80], [10, 100]), wcurve()], err: 'wrapProfileOffFace' });
C('curve plane face', { feats: [...tube(40, 0, 90), lineS([0, 40], [10, 40]), wcurve({ faces: ['F1.C1'] })], err: 'wrapFaceUnsupported' });
C('curve too long', { feats: [...tube(5, 0, 100), lineS([0, 5], [60 * 2 * Math.PI * 5, 95]), wcurve()], err: 'wrapCurveTooLong' });

const shoelace = (loop) => { let s = 0; for (let i = 0; i < loop.length; i++) { const a = loop[i], b = loop[(i + 1) % loop.length]; s += a[0] * b[1] - b[0] * a[1]; } return s / 2; };
function run(c) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: c.feats };
  const t0 = Date.now();
  const rr = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  const ms = Date.now() - t0;
  if (rr.trap) return { ok: false, detail: 'trap ' + rr.trap, ms };
  const r = JSON.parse(rr.json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code }));
  const warns = (r.result?.warnings ?? []).map(([id, w]) => ({ id, code: w.code }));
  if (c.err) return errs.some((e) => e.code === c.err) ? { ok: true, detail: `error ${c.err}`, ms } : { ok: false, detail: `want error ${c.err}, got ${JSON.stringify(errs)}`, ms };
  if (errs.length) return { ok: false, detail: 'errors ' + JSON.stringify(errs), ms };
  if (c.warn && !warns.some((w) => w.code === c.warn)) return { ok: false, detail: `want warning ${c.warn}, got ${JSON.stringify(warns)}`, ms };
  const v = rqk(k, { type: 'validate', tag: 'main', mesh: !c.brepOnly });
  const issues = v.result?.issues ?? [];
  if (issues.length) return { ok: false, detail: 'validate ' + [...new Set(issues.map((i) => i.code))].join(','), ms };
  let detail = 'valid';
  if (c.curve) {
    const cu = (r.result?.curves ?? []).find((x) => x[0] === 'F5')?.[1];
    const mk = (r.result?.markers ?? []).find((x) => x[0] === 'F5')?.[1];
    if (!cu || !mk) return { ok: false, detail: 'no curve or marker', ms };
    const rad = Math.max(...cu.pts.map((p) => Math.abs(Math.hypot(p[0], p[1]) - c.curve.r)));
    detail = `radial ${rad.toExponential(2)} length ${mk.length.toFixed(4)} turns ${mk.turns?.toFixed(3)}`;
    if (rad > 1.5e-3) return { ok: false, detail, ms };
    if (c.curve.length != null && Math.abs(mk.length - c.curve.length) > 1e-4 * c.curve.length) return { ok: false, detail: detail + ` want ${c.curve.length}`, ms };
    if (c.curve.turns != null && Math.abs(mk.turns - c.curve.turns) > 1e-6) return { ok: false, detail, ms };
  }
  if (c.text) {
    const o = rqk(k, { ...c.text, type: 'textOutline' });
    if (!o.ok) return { ok: false, detail: 'no outline', ms };
    let area = 0;
    for (const reg of o.result.regions) area += shoelace(reg.outer) + reg.holes.reduce((s, h) => s + shoelace(h), 0);
    // base body without the wrap
    const kb = new Kernel(mod);
    const rb = rqk(kb, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: c.feats.filter((f) => f.type === 'extrude' && f.id === 'F1' || f.id === 'S1') } });
    const vb = rqk(kb, { type: 'massProps', tag: 'main' }).result?.volume;
    const va = rqk(k, { type: 'massProps', tag: 'main' }).result?.volume;
    if (!rb.ok || !(vb > 0) || !(va > 0)) return { ok: false, detail: 'no volumes', ms };
    // outward when the tool goes away from the axis: an add outside, a cut in a bore
    const outward = (c.op === 'add') !== !!c.bore;
    const want = (c.op === 'add' ? 1 : -1) * area * c.d * (1 + (outward ? 1 : -1) * c.d / (2 * c.r));
    const got = va - vb;
    detail = `change ${got.toFixed(4)} want ${want.toFixed(4)}`;
    if (Math.abs(got - want) > 0.015 * Math.abs(want)) return { ok: false, detail, ms };
  }
  return { ok: true, detail, ms };
}

let fails = 0, n = 0;
for (const c of cases) {
  if (only && !c.name.includes(only)) continue;
  n++;
  let res;
  try { res = run(c); } catch (e) { res = { ok: false, detail: 'exception ' + e.message, ms: 0 }; }
  if (!res.ok) fails++;
  if (!quiet || !res.ok) console.log(`${res.ok ? 'ok  ' : 'FAIL'} ${c.name} (${res.ms} ms) ${res.detail}`);
}
console.log(`wrap corpus: ${n - fails}/${n} ok`);
process.exit(fails ? 1 : 0);
