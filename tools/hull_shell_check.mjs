// Open-top hollow hulls through the wasm: a loft through closed free-form spline stations, the deck
// cut off (face F92.C0), shell of that face. Wall volumes against an independent construction (inner
// offset of the closed hull, cut at the deck line, subtracted from the open hull), validity, and the
// refusal (with a hint) when the apex of a point-ended hull stays in the body.
// usage: node tools/hull_shell_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};

const BLUNT = [[-60, 4, 4, 3], [-30, 14, 9, 4], [0, 18, 12, 5], [30, 14, 9, 4], [60, 2, 3, 7]];
const BOAT = [[-60, 6, 5, 3], [-35, 15, 9, 4], [0, 18, 12, 5], [30, 13, 9, 5.5]];

function hull(stations, tip, thick) {
  const feats = []; const sk = [];
  stations.forEach(([x, w, d, h], i) => {
    feats.push({ type: 'plane', id: `P${i}`, mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x });
    const P = [[0, h], [0.55 * w, h + 0.4], [w, h - 1], [0.97 * w, -0.3 * d], [0.6 * w, -0.8 * d], [0, -d], [-0.6 * w, -0.8 * d], [-0.97 * w, -0.3 * d], [-w, h - 1], [-0.55 * w, h + 0.4]];
    feats.push({ type: 'sketch', id: `S${i}`, plane: { kind: 'feature', id: `P${i}` }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, ...P.map(([u, v], j) => ({ id: 'p' + (j + 1), x: u, y: v }))],
      entities: [{ id: 'e1', type: 'spline', pts: P.map((_, j) => 'p' + (j + 1)), closed: true }], constraints: [] });
    sk.push(`S${i}`);
  });
  if (tip != null) {
    feats.push({ type: 'plane', id: 'PT', mode: 'offset', base: { kind: 'std', id: 'right' }, offset: tip });
    feats.push({ type: 'sketch', id: 'ST', plane: { kind: 'feature', id: 'PT' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 0, y: 2 }], entities: [{ id: 'e1', type: 'point', p: 'p1' }], constraints: [] });
    sk.push('ST');
  }
  feats.push({ type: 'loft', id: 'F50', op: 'add', sketches: sk });
  feats.push({ type: 'plane', id: 'F90', mode: 'offset', base: { kind: 'std', id: 'top' }, offset: 1.5 });
  feats.push({ type: 'sketch', id: 'F91', plane: { kind: 'feature', id: 'F90' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'a', x: -90, y: -40 }, { id: 'b', x: 90, y: -40 }, { id: 'c', x: 90, y: 40 }, { id: 'd', x: -90, y: 40 }],
    entities: [{ id: 'e1', type: 'line', p1: 'a', p2: 'b' }, { id: 'e2', type: 'line', p1: 'b', p2: 'c' }, { id: 'e3', type: 'line', p1: 'c', p2: 'd' }, { id: 'e4', type: 'line', p1: 'd', p2: 'a' }], constraints: [] });
  feats.push({ type: 'extrude', id: 'F92', op: 'cut', sketch: 'F91', end: 'blind', depth: 30, flip: true });
  if (thick) feats.push({ type: 'shell', id: 'F93', faces: ['F92.C0'], thickness: thick });
  return feats;
}

function regen(features) {
  const k = new Kernel(mod);
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  const t0 = Date.now();
  const r = ask(1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features }, opts: { solveSketches: true } });
  const ms = Date.now() - t0;
  const m = ask(2, { type: 'massProps', tag: 'main' });
  const v = ask(3, { type: 'validate', tag: 'main' });
  const errors = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, params: e.params ?? e }));
  return { errors, ms, vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, issues: v.result?.issues ?? [] };
}

// reference wall volumes (independent construction from the same sections)
for (const [t, want] of [[1.0, 4383.0], [0.6, 2683.5]]) {
  const r = regen(hull(BLUNT, null, t));
  check(`blunt hull shell ${t}`, r.errors.length === 0 && r.valid === true && r.vol != null && Math.abs(r.vol - want) < 0.04 * want,
    `errors ${JSON.stringify(r.errors).slice(0, 160)} vol ${r.vol?.toFixed(1)} want ${want} valid ${r.valid} ${r.ms} ms ${JSON.stringify(r.issues).slice(0, 160)}`);
}
// the point-ended hull (reference wall volumes: inner offset of the closed hull cut at the deck line,
// subtracted from the open hull, made from the same sections)
for (const [t, want] of [[1.0, 4393.9], [0.8, 3684.9]]) {
  const r = regen(hull(BOAT, 62, t));
  check(`pointy hull shell ${t}`, r.errors.length === 0 && r.valid === true && r.vol != null && Math.abs(r.vol - want) < 0.01 * want,
    `errors ${JSON.stringify(r.errors).slice(0, 160)} vol ${r.vol?.toFixed(1)} want ${want} valid ${r.valid} ${r.ms} ms ${JSON.stringify(r.issues).slice(0, 160)}`);
}
{
  // the apex stays in the body (stern cap shelled away): valid, plausible wall
  const f = hull(BOAT, 62, 0).filter((x) => !['F90', 'F91', 'F92'].includes(x.id));
  const solid = regen(f).vol;
  f.push({ type: 'shell', id: 'F93', faces: ['F50.C0'], thickness: 1.0 });
  const r = regen(f);
  check('pointy hull with its apex shells', r.errors.length === 0 && r.valid === true && r.vol > 0.05 * solid && r.vol < 0.25 * solid, `errors ${JSON.stringify(r.errors).slice(0, 160)} vol ${r.vol?.toFixed(1)} of ${solid?.toFixed(1)} valid ${r.valid} ${r.ms} ms`);
}
{
  // cone loft (circle to point) with its base opened: analytic wall volume
  const R = 8, H = 20;
  for (const t of [1.0, 0.6]) {
    const f = [
      { type: 'plane', id: 'P0', mode: 'offset', base: { kind: 'std', id: 'right' }, offset: 0 },
      { type: 'sketch', id: 'S0', plane: { kind: 'feature', id: 'P0' }, points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r: R }], constraints: [] },
      { type: 'plane', id: 'PT', mode: 'offset', base: { kind: 'std', id: 'right' }, offset: H },
      { type: 'sketch', id: 'ST', plane: { kind: 'feature', id: 'PT' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 0, y: 0 }], entities: [{ id: 'e1', type: 'point', p: 'p1' }], constraints: [] },
      { type: 'loft', id: 'F50', op: 'add', sketches: ['S0', 'ST'] },
      { type: 'shell', id: 'F93', faces: ['F50.C0'], thickness: t },
    ];
    const a = Math.atan(R / H);
    const want = (Math.PI / 3) * (R * R * H - (R - t / Math.cos(a)) ** 2 * (H - t / Math.sin(a)));
    const r = regen(f);
    check(`cone loft shell ${t}`, r.errors.length === 0 && r.valid === true && r.vol != null && Math.abs(r.vol - want) < 0.01 * want,
      `errors ${JSON.stringify(r.errors).slice(0, 160)} vol ${r.vol?.toFixed(2)} want ${want.toFixed(2)} valid ${r.valid} ${r.ms} ms`);
  }
}
{
  // a wall too thick for the shape: refused with the thickness that would work, and that thickness works
  const r = regen(hull(BLUNT, null, 6.0));
  const p = r.errors[0]?.params ?? {};
  check('too thick shell carries maxThickness', r.errors.length === 1 && typeof p.maxThickness === 'number' && p.maxThickness > 0 && p.maxThickness < 6, JSON.stringify(r.errors).slice(0, 240));
  if (typeof p.maxThickness === 'number') {
    const ok = regen(hull(BLUNT, null, p.maxThickness));
    check(`suggested thickness ${p.maxThickness} works`, ok.errors.length === 0 && ok.valid === true, JSON.stringify(ok.errors).slice(0, 200));
  }
}
// thicken of free-form sheets: a loft of open U sections, both sides (volume = area x thickness to second order)
function skin(stations, thickness, side) {
  const feats = []; const sk = [];
  stations.forEach(([x, w, d, h], i) => {
    feats.push({ type: 'plane', id: `P${i}`, mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x });
    const P = [[-w, h - 1], [-0.97 * w, -0.3 * d], [-0.6 * w, -0.8 * d], [0, -d], [0.6 * w, -0.8 * d], [0.97 * w, -0.3 * d], [w, h - 1]];
    feats.push({ type: 'sketch', id: `S${i}`, plane: { kind: 'feature', id: `P${i}` }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, ...P.map(([u, v], j) => ({ id: 'p' + (j + 1), x: u, y: v }))],
      entities: [{ id: 'e1', type: 'spline', pts: P.map((_, j) => 'p' + (j + 1)) }], constraints: [] });
    sk.push(`S${i}`);
  });
  feats.push({ type: 'loft', id: 'L1', name: 'Hull surface', op: 'add', surface: true, sketches: sk });
  if (thickness) feats.push({ type: 'thicken', id: 'T1', surface: 'L1', thickness, side });
  return feats;
}
{
  const k = new Kernel(mod);
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  ask(1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: skin(BLUNT, 0) }, opts: { solveSketches: true } });
  const area = ask(2, { type: 'massProps', tag: 'main' }).result?.area;
  check('hull skin sheet has an area', area > 1000, `area ${area}`);
  for (const [t, side] of [[0.8, 'both'], [1.0, 'both'], [0.8, 'front'], [0.8, 'back']]) {
    const r = regen(skin(BLUNT, t, side));
    // front / back differ from area x t by the first order curvature term; both sides cancel it
    const tol = side === 'both' ? 0.01 : 0.06;
    check(`hull skin thicken ${t} ${side}`, r.errors.length === 0 && r.valid === true && r.vol != null && Math.abs(r.vol - area * t) < tol * area * t,
      `errors ${JSON.stringify(r.errors).slice(0, 200)} vol ${r.vol?.toFixed(1)} area*t ${(area * t).toFixed(1)} valid ${r.valid} ${r.ms} ms ${JSON.stringify(r.issues).slice(0, 160)}`);
  }
  // beyond the tightest radius: structured error with the thickness that would work
  const r = regen(skin(BLUNT, 3.0, 'front'));
  const p = r.errors[0]?.params ?? {};
  check('hull skin thicken too thick carries maxThickness', r.errors.length === 1 && typeof p.maxThickness === 'number' && p.maxThickness > 0 && p.maxThickness < 3, JSON.stringify(r.errors).slice(0, 300));
}
if (bad) { console.log(`${bad} failed`); process.exit(1); }
