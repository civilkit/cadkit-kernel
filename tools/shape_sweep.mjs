// Seeded sweep/loft/revolve sweep against OCCT.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
const wasm = opt('--wasm', '/path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm');
const seed = Number(opt('--seed', '1'));
const samples = Number(opt('--samples', '30'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'shape_sweep_baseline.json')));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });

let n = 0;
const plane = (z) => z === 0 ? [{ kind: 'std', id: 'front' }, []] : [{ kind: 'feature', id: `P${++n}` }, [{ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }]];
function circleSketch(id, r, z, cx = 0, cy = 0) {
  const [pl, pre] = plane(z);
  return [...pre, { type: 'sketch', id, plane: pl, points: [pt('c', cx, cy)], entities: [circ('e1', 'c', r)], constraints: [] }];
}
function ellipseSketch(id, a, b, z) {
  const [pl, pre] = plane(z);
  return [...pre, { type: 'sketch', id, plane: pl, points: [pt('c', 0, 0), pt('m', a, 0)],
    entities: [{ type: 'ellipse', id: 'e1', c: 'c', p: 'm', r: b }], constraints: [] }];
}
function rectSketch(id, w, d, z, rot = 0) {
  const c = Math.cos(rot * Math.PI / 180), s = Math.sin(rot * Math.PI / 180);
  const P = [[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]].map(([x, y], i) => pt(`p${i}`, x * c - y * s, x * s + y * c));
  const E = [0, 1, 2, 3].map((i) => line(`e${i + 1}`, `p${i}`, `p${(i + 1) % 4}`));
  const [pl, pre] = plane(z);
  return [...pre, { type: 'sketch', id, plane: pl, points: P, entities: E, constraints: [] }];
}
function topCircleSketch(id, r) {
  return [{ type: 'sketch', id, plane: { kind: 'std', id: 'top' }, points: [pt('c', 0, 0)], entities: [circ('e1', 'c', r)], constraints: [] }];
}
function bottomCircleSketch(id, r) {
  return [{ type: 'sketch', id, plane: { kind: 'std', id: 'bottom' }, points: [pt('c', 0, 0)], entities: [circ('e1', 'c', r)], constraints: [] }];
}
function topRectSketch(id, w, d) {
  const P = [[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]].map(([x, y], i) => pt(`p${i}`, x, y));
  const E = [0, 1, 2, 3].map((i) => line(`e${i + 1}`, `p${i}`, `p${(i + 1) % 4}`));
  return [{ type: 'sketch', id, plane: { kind: 'std', id: 'top' }, points: P, entities: E, constraints: [] }];
}
function pointSketch(id, z) {
  const [pl, pre] = plane(z);
  return [...pre, { type: 'sketch', id, plane: pl, points: [{ id: 'p', x: 0, y: 0 }], entities: [{ type: 'point', p: 'p' }], constraints: [] }];
}

const cases = [];
const add = (id, template, features, ref) => cases.push({ id, template, features, ref });
function gen() {
  for (let i = 0; i < samples; i++) {
    // loft circles
    let f = [], ids = [], specs = [];
    const N = 2 + Math.floor(rnd() * 2);
    let z = 0;
    for (let j = 0; j < N; j++) { const id = `S${j}`; const r = rr(5, 20); f.push(...circleSketch(id, r, z)); ids.push(id); specs.push({ kind: 'circle', r, z }); z += rr(10, 30); }
    f.push({ type: 'loft', id: 'L1', op: 'add', sketches: ids });
    add(`loft_circle_${i}`, 'loft_circle', f, { kind: 'loft', sketches: specs });

    // loft ellipses
    f = []; ids = []; z = 0; specs = [];
    for (let j = 0; j < 2; j++) { const id = `S${j}`; const a = rr(8, 20), b = rr(4, 12); f.push(...ellipseSketch(id, a, b, z)); ids.push(id); specs.push({ kind: 'ellipse', a, b, z }); z += rr(10, 30); }
    f.push({ type: 'loft', id: 'L1', op: 'add', sketches: ids });
    add(`loft_ellipse_${i}`, 'loft_ellipse', f, { kind: 'loft', sketches: specs });

    // loft rectangles, rotated
    f = []; ids = []; z = 0; specs = [];
    for (let j = 0; j < 2; j++) { const id = `S${j}`; const w = rr(10, 25), d = rr(8, 20), rot = j ? rr(-30, 30) : 0; f.push(...rectSketch(id, w, d, z, rot)); ids.push(id); specs.push({ kind: 'rect', w, d, z, rot }); z += rr(10, 30); }
    f.push({ type: 'loft', id: 'L1', op: 'add', sketches: ids });
    add(`loft_rect_${i}`, 'loft_rect', f, { kind: 'loft', sketches: specs });

    // loft to a point
    const lr = rr(5, 18), lz = rr(15, 40);
    f = circleSketch('S0', lr, 0); f.push(...pointSketch('S1', lz)); f.push({ type: 'loft', id: 'L1', op: 'add', sketches: ['S0', 'S1'] });
    add(`loft_point_${i}`, 'loft_point', f, { kind: 'loft_point', r: lr, z: lz });

    // revolve a random closed profile around x=0
    const h = rr(10, 40), w0 = rr(2, 15), w1 = rr(2, 15);
    f = [{ type: 'sketch', id: 'S1', plane: { kind: 'std', id: 'front' },
      points: [pt('p0', w0, 0), pt('p1', w1, h), pt('p2', 0, h), pt('p3', 0, 0), pt('a1', 0, 0), pt('a2', 0, h)],
      entities: [line('e1', 'p0', 'p1'), line('e2', 'p1', 'p2'), line('e3', 'p2', 'p3'), line('e4', 'p3', 'p0'), line('ax', 'a1', 'a2', { construction: true })], constraints: [] }];
    const ang = rr(90, 360);
    f.push({ type: 'revolve', id: 'R1', op: 'add', sketch: 'S1', axis: 'ax', angle: ang });
    add(`revolve_${i}`, 'revolve', f, { kind: 'revolve', profile: [[w0, 0], [w1, h], [0, h], [0, 0]], angle: ang });

    // sweep a circle along a line
    const len = rr(15, 50), pr = rr(2, 8);
    f = [{ type: 'sketch', id: 'P1', plane: { kind: 'std', id: 'front' }, points: [pt('a', 0, 0), pt('b', 0, len)], entities: [line('e1', 'a', 'b')], constraints: [] },
      ...topCircleSketch('S1', pr)];
    f.push({ type: 'sweep', id: 'W1', op: 'add', sketch: 'S1', path: { kind: 'sketch', id: 'P1' } });
    add(`sweep_line_${i}`, 'sweep_line', f, { kind: 'sweep_line', r: pr, len });

    // sweep a circle along an arc
    const R = rr(10, 30), ar = rr(2, 7), a0 = 0, a1 = rr(45, 180);
    f = [{ type: 'sketch', id: 'P1', plane: { kind: 'std', id: 'front' },
      points: [pt('c', -R, 0), pt('a', 0, 0), pt('b', -R + R * Math.cos(a1 * Math.PI / 180), R * Math.sin(a1 * Math.PI / 180))],
      entities: [{ type: 'arc', id: 'e1', c: 'c', p1: 'a', p2: 'b' }], constraints: [] }, ...topCircleSketch('S1', ar)];
    f.push({ type: 'sweep', id: 'W1', op: 'add', sketch: 'S1', path: { kind: 'sketch', id: 'P1' } });
    add(`sweep_arc_${i}`, 'sweep_arc', f, { kind: 'sweep_arc', R, r: ar, a1 });

    // sweep a rectangle along an L path
    const l1 = rr(10, 25), l2 = rr(10, 25), rw = rr(3, 8), rd = rr(3, 8);
    f = [{ type: 'sketch', id: 'P1', plane: { kind: 'std', id: 'front' },
      points: [pt('p0', 0, 0), pt('p1', 0, l1), pt('p2', l2, l1)], entities: [line('e1', 'p0', 'p1'), line('e2', 'p1', 'p2')], constraints: [] },
      ...topRectSketch('S1', rw, rd)];
    f.push({ type: 'sweep', id: 'W1', op: 'add', sketch: 'S1', path: { kind: 'sketch', id: 'P1' } });
    add(`sweep_L_${i}`, 'sweep_L', f, { kind: 'sweep_L', rw, rd, l1, l2 });
  }
}
gen();
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;

const rows = [], refCases = {};
for (const c of selected) {
  refCases[c.id] = { template: c.template, ref: c.ref };
  let r;
  try { r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: c.features }, opts: { solveSketches: true } } })).json); } catch { r = null; }
  if (!r || !r.ok) { rows.push({ id: c.id, template: c.template, cls: 'regen-fail', detail: JSON.stringify(r?.error ?? r).slice(0, 140) }); continue; }
  const errs = (r.result?.errors ?? []).map(([, e]) => e.code);
  if (errs.length) { rows.push({ id: c.id, template: c.template, cls: errs[0], detail: errs.join(',') }); continue; }
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  rows.push({ id: c.id, template: c.template, cls: (v.result?.issues ?? []).length ? 'invalid' : 'ok', detail: (v.result?.issues ?? []).map((x) => x.code).join(','), vol: m.result?.volume ?? null });
}

const rp = '/tmp/shape_sweep_cases.json', ro = '/tmp/shape_sweep_ref.json';
writeFileSync(rp, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here, 'shape_sweep_ref.py'), '--cases', rp, '--out', ro], { stdio: 'pipe', timeout: 180000 }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(ro) ? JSON.parse(readFileSync(ro, 'utf8')) : {};
for (const row of rows) {
  const rf = ref[row.id];
  if (!rf) { if (row.cls === 'ok') row.cls = 'no-ref'; continue; }
  if (rf.error) { row.cls = 'ref-fails'; row.detail = rf.error; continue; }
  if (row.cls === 'ok' && row.vol != null && rf.volume != null) {
    const rel = Math.abs(row.vol - rf.volume) / Math.max(Math.abs(rf.volume), 1e-9);
    if (rel > 1e-3) { row.cls = 'wrongVolume'; row.detail = `rel ${rel.toExponential(2)} vol ${row.vol.toFixed(3)} ref ${rf.volume.toFixed(3)}`; }
  }
}
const g = {}; for (const r of rows) { g[r.template] ??= {}; g[r.template][r.cls] = (g[r.template][r.cls] ?? 0) + 1; }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, seed, samples, wasmHash, ...tot }));
for (const [t, x] of Object.entries(g)) console.log(`${t}: ${JSON.stringify(x)}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: opt('--wasm', 'kit'), wasmHash, seed, samples, cases: Object.fromEntries(rows.map((r) => [r.id, r.cls])) }, null, 1) + '\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {}; const reg = rows.filter((r) => base[r.id] === 'ok' && r.cls !== 'ok'); console.log(`shape_sweep check: ${reg.length} regressions`); if (reg.length) process.exit(1); }
}
