// Point-ended (apex) loft shape check: the boat hull of the loft film (sections at -60,-30,0,30, bow point at 62) and variants.
// Indicators: the cross-section area is monotone along the last span, the tip is the axis extreme, rays along the axis enter and
// leave exactly once (no fold / self crossing near the tip), a valid solid.
// A far apex must not dent the half-width below the chord to the tip (the natural end condition of the rails did), `apexTangent` (0..2) sharpens / blunts the tip,
// a 3-station cone is exact, the tip is the axis extreme and convex.
// usage: node tools/loft_apex_check.mjs [--wasm path] [--dump dir]    exit 1 on a failed assertion
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (k) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : null; };
const mod = new WebAssembly.Module(readFileSync(arg('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm')));
const dumpDir = arg('--dump');

const section = (w, d, h) => [[0, h], [.55 * w, h + .4], [w, h - 1], [.97 * w, -.3 * d], [.6 * w, -.8 * d], [0, -d], [-.6 * w, -.8 * d], [-.97 * w, -.3 * d], [-w, h - 1], [-.55 * w, h + .4]].map(([u, v]) => [Math.round(u), Math.round(v)]);
const STATIONS = [[-60, 6, 5, 3], [-30, 15, 9, 4], [0, 18, 12, 5], [30, 13, 9, 5.5]];

export function build(bowX, bowPt, extra = {}, stations = STATIONS) {
  const feats = [];
  let n = 0;
  const id = (p = 'F') => p + ++n;
  const ids = [];
  const add = (x, sketch) => {
    const pl = { type: 'plane', id: id(), mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x };
    const sk = { type: 'sketch', id: id(), plane: { kind: 'feature', id: pl.id }, constraints: [], ...sketch };
    feats.push(pl, sk); ids.push(sk.id);
  };
  for (const [x, w, d, h] of stations) {
    const P = section(w, d, h);
    add(x, { points: P.map(([u, v], i) => ({ id: 'p' + (i + 1), x: u, y: v })), entities: [{ id: 'e1', type: 'spline', pts: P.map((_, i) => 'p' + (i + 1)), closed: true }] });
  }
  add(bowX, { points: [{ id: 'p', x: bowPt[0], y: bowPt[1] }], entities: [{ type: 'point', id: 'e1', p: 'p' }] });
  feats.push({ type: 'loft', id: id(), op: 'add', sketches: ids, ...extra });
  return { format: 'ck-part', version: 1, features: feats };
}

const PI = Math.PI;
const open = (part) => {
  const k = new Kernel(mod);
  const raw = (msg, id = 2) => JSON.parse(k.request(JSON.stringify({ id, msg: { ...msg, tag: 'main' } })).json);
  const r = raw({ type: 'regen', part, opts: { solveSketches: true } }, 1);
  return { call: raw, r, errs: (r.result?.errors ?? []).map(([i, e]) => `${i}:${e.code}`) };
};
// extent of the section at x: [uMin, uMax, vMin, vMax] of the loop points (u = hull height, v = half beam), or null when empty
function extent(call, x) {
  const s = call({ type: 'section', plane: { origin: [x, 0, 0], normal: [1, 0, 0] } }).result;
  let e = null;
  for (const l of s?.loops ?? []) for (const ed of l.edges ?? []) for (const p of ed.pts ?? []) {
    e ??= [1e9, -1e9, 1e9, -1e9];
    e = [Math.min(e[0], p[0]), Math.max(e[1], p[0]), Math.min(e[2], p[1]), Math.max(e[3], p[1])];
  }
  return { e, area: s?.area ?? 0 };
}

let bad = 0;
const check = (name, ok, info) => { if (!ok) bad++; console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${info}`); };

function hullCase(bowX, h, extra = {}, stations = STATIONS, label = 'hull', strict = true) {
  const name = `${label} bow x=${bowX} h=${h}${extra.apexTangent != null ? ` apexTangent=${extra.apexTangent}` : ''}`;
  const part = build(bowX, [0, h], extra, stations);
  const L = stations[stations.length - 1];
  if (dumpDir) writeFileSync(path.join(dumpDir, name.replace(/[^\w.]+/g, '_') + '.ckpart'), JSON.stringify(part));
  const o = open(part);
  if (o.errs.length) { check(name, false, `errors ${o.errs}`); return null; }
  const issues = o.call({ type: 'validate' }).result?.issues ?? [];
  const vol = o.call({ type: 'massProps' }).result?.volume;
  const span = bowX - L[0];
  let prev = Infinity, mono = true, dentW = 0, dentT = 0, dentB = 0;
  const areas = [];
  for (const f of [0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.92, 0.97]) {
    const { e, area } = extent(o.call, L[0] + f * span);
    areas.push(area);
    if (area > prev * (1 + 1e-6) + 1e-9) mono = false;
    prev = area;
    if (!e) continue;
    // a convex tip lies on or outside the chord from the last section to the apex: any shortfall is an indentation
    dentW = Math.max(dentW, L[1] * (1 - f) - e[3]);
    if (h <= L[3] && h >= -L[2]) { dentT = Math.max(dentT, (L[3] + (h - L[3]) * f) - e[1]); dentB = Math.max(dentB, e[0] - (-L[2] + (h + L[2]) * f)); }
  }
  // a ray along the axis direction through the tip region: exactly one entry and one exit (a fold would add hits)
  const rays = [];
  for (const dy of [-6, -3, 0, 3, 6]) for (const dz of [-6, -3, 0, 3, 6]) rays.push({ origin: [L[0] - 80, dy, dz + 2], dir: [1, 0, 0] });
  const hits = (o.call({ type: 'probe', rays }).result?.rays ?? []).map((q) => q.hits.length);
  const multi = hits.filter((n) => n > 2).length;
  const tip = o.r.result?.body?.box?.max?.[0];
  const H = L[3] + L[2];
  const ok = issues.length === 0 && mono && multi === 0 && (!strict || (dentW <= 0.02 * L[1] && dentT <= 0.05 * H && dentB <= 0.05 * H)) && Math.abs(tip - bowX) < 1e-6;
  check(name, ok, `vol ${vol?.toFixed(1)} issues ${issues.length} monoArea ${mono} multiHitRays ${multi} dentW ${dentW.toFixed(2)} dentTop ${dentT.toFixed(2)} dentBottom ${dentB.toFixed(2)} tipX ${tip}`);
  return vol;
}
for (const bowX of [62, 70, 90]) for (const h of [2, 0, -2, -4]) hullCase(bowX, h);
hullCase(120, 2);
// sections that shrink fast towards a far apex: the natural rail end made the cross-section area GROW again (a fold); not strictly convex (the data are not), but monotone
const TAPER = [[-60, 18, 12, 5], [-30, 18, 12, 5], [0, 14, 10, 5], [30, 7, 6, 4]];
for (const h of [2, -6]) hullCase(150, h, {}, TAPER, 'taper hull', false);
// apexTangent: valid over the range
{
  const v = [0, 0.5, 1, 2].map((t) => hullCase(62, 2, { apexTangent: t }));
  check('apexTangent: a larger factor gives a wider tip angle and a slimmer body (volume falls)', v.every((x) => x != null) && v[1] < v[0] && v[2] < v[1] && v[3] < v[2], v.map((x) => x?.toFixed(1)).join(' > '));
}

// point sections of simple shapes
const circleSk = (r) => ({ points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }] });
function shapeCase(name, stations, apexX, volWant, tol, rOf, areaTol) {
  const feats = [];
  const ids = [];
  let n = 0;
  const add = (x, sk) => {
    const pl = { type: 'plane', id: 'P' + ++n, mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x };
    const s = { type: 'sketch', id: 'S' + n, plane: { kind: 'feature', id: pl.id }, constraints: [], ...sk };
    feats.push(pl, s); ids.push(s.id);
  };
  for (const [x, r] of stations) add(x, circleSk(r));
  add(apexX, { points: [{ id: 'p', x: 0, y: 0 }], entities: [{ type: 'point', id: 'e1', p: 'p' }] });
  feats.push({ type: 'loft', id: 'L', op: 'add', sketches: ids });
  const o = open({ format: 'ck-part', version: 1, features: feats });
  if (o.errs.length) { check(name, false, `errors ${o.errs}`); return; }
  const issues = o.call({ type: 'validate' }).result?.issues ?? [];
  const vol = o.call({ type: 'massProps' }).result?.volume;
  const tip = o.r.result?.body?.box?.max?.[0];
  const x0 = stations[stations.length - 1][0];
  // the cross-section at distance d behind the tip against the ideal shape: area pi r(d)^2 (a dented tip is thinner than the ideal, a
  // folded one is not monotone); sections are used rather than point probes, which misclassify points next to a pole
  const ds = [0.01, 0.03, 0.1, 0.2].map((f) => f * (apexX - stations[0][0]));
  const areas = ds.map((d) => o.call({ type: 'section', plane: { origin: [apexX - d, 0, 0], normal: [1, 0, 0] } }).result?.area ?? 0);
  const ideal = ds.map((d) => PI * rOf(d) ** 2);
  const rel = areas.map((a, i) => Math.abs(a - ideal[i]) / ideal[i]);
  const mono = areas.every((a, i) => i === 0 || a > areas[i - 1]);
  const ok = issues.length === 0 && Math.abs(vol - volWant) <= tol * volWant && Math.abs(tip - apexX) < 1e-6 && mono && Math.max(...rel) <= areaTol;
  check(name, ok, `vol ${vol?.toFixed(3)} want ${volWant.toFixed(3)} issues ${issues.length} tipX ${tip} tip section areas ${areas.map((x) => x.toFixed(4)).join(' ')} ideal ${ideal.map((x) => x.toFixed(4)).join(' ')} (monotone ${mono}, max rel error ${Math.max(...rel).toFixed(4)})`);
}
const cone = (d) => 8 * d / 30; // radius of the cone at distance d from the tip
shapeCase('cone through a middle section is exact', [[0, 8], [15, 4]], 30, PI * 64 * 30 / 3, 1e-6, cone, 1e-6);
shapeCase('cone through two middle sections is exact', [[0, 8], [10, 16 / 3], [20, 8 / 3]], 30, PI * 64 * 30 / 3, 1e-6, cone, 1e-6);
// a bulged cone (radius 1.2 d - 0.04 d^2 at distance d from the tip): the stations carry it, the tip is the straight limit of the rails
shapeCase('bulged cone stations to a pole (convex tip)', [[0, 8], [3, 6.44], [6, 4.16], [8, 2.24]], 10, PI * 272, 0.01, (d) => 1.2 * d - 0.04 * d * d, 0.15); // the tip slope is the chord slope (1.12, not 1.2): 6 percent in radius
process.exit(bad ? 1 : 0);
