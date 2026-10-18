// Seeded boolean sweep beyond the fixed corpus: random primitives, hard relations.
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
const per = Number(opt('--samples', '20'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'boolean_sweep_baseline.json')));
let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pick = (a) => a[Math.floor(rnd() * a.length)];
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });

let n = 0;
function prism(shape, z = 0, park = 0) {
  const w = shape.w, d = shape.d;
  const P = [[0, 0], [w, 0], [w, d], [0, d]].map(([x, y], i) => pt(`p${i}`, x + park, y));
  const E = [0, 1, 2, 3].map((i) => line(`e${i + 1}`, `p${i}`, `p${(i + 1) % 4}`));
  const plane = z === 0 ? { kind: 'std', id: 'front' } : { kind: 'feature', id: `P${++n}` };
  const pre = z === 0 ? [] : [{ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }];
  return [...pre, { type: 'sketch', id: `S${++n}`, plane, points: P, entities: E, constraints: [] }];
}
function features(shape, base = 0, park = 0) {
  n = base;
  if (shape.k === 'box') {
    const f = prism(shape, 0, park); return [...f, { type: 'extrude', id: `F${++n}`, op: 'add', sketch: f[f.length - 1].id, end: 'blind', depth: shape.h }];
  }
  if (shape.k === 'cyl') {
    const sid = `S${++n}`;
    return [{ type: 'sketch', id: sid, plane: { kind: 'std', id: 'front' }, points: [pt('c', park, 0)], entities: [circ('e1', 'c', shape.r)], constraints: [] },
      { type: 'extrude', id: `F${++n}`, op: 'add', sketch: sid, end: 'blind', depth: shape.h }];
  }
  if (shape.k === 'cone') {
    const sid = `S${++n}`;
    return [{ type: 'sketch', id: sid, plane: { kind: 'std', id: 'front' }, points: [pt('c', park, 0)], entities: [circ('e1', 'c', shape.r1)], constraints: [] },
      { type: 'extrude', id: `F${++n}`, op: 'add', sketch: sid, end: 'blind', depth: shape.h, draft: shape.draft }];
  }
  if (shape.k === 'sphere') {
    const sid = `S${++n}`;
    return [{ type: 'sketch', id: sid, plane: { kind: 'std', id: 'front' },
      points: [pt('c', park, 0), pt('s', park, -shape.r), pt('t', park, shape.r), pt('a1', park, 0), pt('a2', park, shape.r)],
      entities: [{ type: 'arc', id: 'e1', c: 'c', p1: 's', p2: 't' }, line('e2', 't', 's'), line('ax', 'a1', 'a2', { construction: true })], constraints: [] },
      { type: 'revolve', id: `F${++n}`, op: 'add', sketch: sid, axis: 'ax', angle: 360 }];
  }
  if (shape.k === 'torus') {
    const sid = `S${++n}`;
    return [{ type: 'sketch', id: sid, plane: { kind: 'std', id: 'front' },
      points: [pt('c', park + shape.R, 0), pt('a1', park, 0), pt('a2', park, 2 * shape.r + 1)],
      entities: [circ('e1', 'c', shape.r), line('ax', 'a1', 'a2', { construction: true })], constraints: [] },
      { type: 'revolve', id: `F${++n}`, op: 'add', sketch: sid, axis: 'ax', angle: 360 }];
  }
  throw new Error(shape.k);
}

function bodyFace(shape, base) {
  const ext = shape.k === 'box' || shape.k === 'cyl' || shape.k === 'cone';
  return ext ? `F${base + 2}.C1` : `F${base + 2}.S.e1`;
}
const cases = [];
const add = (id, group, A, B, op, move, extra = {}) => cases.push({ id, group, A, B, op, move, ...extra });
const prim = () => pick(['box', 'cyl', 'sphere', 'cone', 'torus']);
function shape(k) {
  if (k === 'box') { const w = rr(10, 50), d = rr(10, 50), h = rr(10, 50); return { k, w, d, h }; }
  if (k === 'cyl') return { k, r: rr(4, 25), h: rr(10, 60) };
  if (k === 'sphere') return { k, r: rr(5, 25) };
  if (k === 'cone') { const r1 = rr(8, 25), h = rr(10, 50), maxd = Math.atan((r1 - 0.5) / h) * 180 / Math.PI; return { k, r1, r2: rr(1, 8), h, draft: rr(0, Math.min(20, maxd * 0.8)) }; }
  const R = rr(10, 30), r = rr(2, Math.min(8, R * 0.5)); return { k, R, r };
}
function gen() {
  for (let i = 0; i < per; i++) {
    const op = pick(['union', 'cut', 'intersect']);
    let A = shape(prim()), B = shape(prim());
    // overlap
    add(`overlap_${i}`, 'overlap', A, B, op, { translate: [rr(-15, 15), rr(-15, 15), rr(-15, 15)] });
    // coincident: two boxes share a face, or coaxial cylinders end to end
    if (i % 2) { A = { k: 'box', w: 30, d: 30, h: 20 }; B = { k: 'box', w: 30, d: 30, h: 20 }; add(`coincident_${i}`, 'coincident', A, B, op, { translate: [30, 0, 0] }); }
    else { A = { k: 'cyl', r: 10, h: 20 }; B = { k: 'cyl', r: 10, h: 20 }; add(`coincident_${i}`, 'coincident', A, B, op, { translate: [0, 0, 20] }); }
    // tangent: sphere sits on a box top
    A = { k: 'box', w: 40, d: 40, h: 20 }; B = { k: 'sphere', r: rr(5, 15) };
    add(`tangent_${i}`, 'tangent', A, B, op, { translate: [20, 20, 20 + B.r] });
    // contained: small sphere inside a box
    A = { k: 'box', w: 50, d: 50, h: 50 }; B = { k: 'sphere', r: rr(3, 12) };
    add(`contained_${i}`, 'contained', A, B, op, { translate: [25, 25, 25] });
    // sliver gap 1e-3 .. 1e-1 between two boxes
    const gap = rr(1e-3, 1e-1);
    A = { k: 'box', w: 30, d: 30, h: 20 }; B = { k: 'box', w: 30, d: 30, h: 20 };
    add(`sliver_${i}`, 'sliver', A, B, op, { translate: [30 + gap, 0, 0] }, { gap });
    // angled cylinders, axes crossing at 5 .. 30 degrees
    A = { k: 'cyl', r: 8, h: 40 }; B = { k: 'cyl', r: 6, h: 40 };
    add(`angled_${i}`, 'angled', A, B, op, { translate: [0, 0, 20], rotate: { axis: [0, 1, 0], origin: [0, 0, 0], angle: rr(5, 30) } });
    // wide-base cone fused onto a plate
    A = { k: 'box', w: 60, d: 60, h: 8 }; B = { k: 'cone', r1: rr(15, 26), r2: rr(2, 8), h: rr(10, 30), draft: rr(0, 15) };
    add(`cone_plate_${i}`, 'cone_plate', A, B, op, { translate: [30, 30, 8] });
    // chain of three: A op1 B then result op2 C (n-ary combine with two tools)
    const C = shape(prim());
    add(`chain3_${i}`, 'chain3', A, B, op, { translate: [30, 30, 8] }, { C, op2: pick(['union', 'cut', 'intersect']), Cmove: { translate: [0, 0, 40] } });
  }
}
gen();
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;

function moveFeature(shape, move, id = 'M1', park = 0, base = 0) {
  const m = { type: 'moveBody', id, bodies: [{ face: bodyFace(shape, base) }], translate: [move.translate[0] - park, move.translate[1], move.translate[2]] };
  if (move.rotate) m.rotate = { ...move.rotate, origin: [park, 0, 0] };
  return m;
}
function build(c) {
  const fa = features(c.A, 0, 0), fb = features(c.B, 1000, 1000);
  const f = [...fa, ...fb, moveFeature(c.B, c.move, 'M1', 1000, 1000)];
  if (c.C) {
    const fc = features(c.C, 4000, 4000);
    f.push(...fc, moveFeature(c.C, c.Cmove, 'M2', 4000, 4000));
    f.push({ type: 'combine', id: 'C1', op: c.op, target: { face: bodyFace(c.A, 0) }, tools: [{ face: bodyFace(c.B, 1000) }] });
    f.push({ type: 'combine', id: 'C2', op: c.op2, target: { face: bodyFace(c.A, 0) }, tools: [{ face: bodyFace(c.C, 4000) }] });
  } else {
    f.push({ type: 'combine', id: 'C1', op: c.op, target: { face: bodyFace(c.A, 0) }, tools: [{ face: bodyFace(c.B, 1000) }] });
  }
  return f;
}

const rows = [], refCases = {};
for (const c of selected) {
  refCases[c.id] = { A: c.A, B: c.B, op: c.op, move: c.move, C: c.C, op2: c.op2, Cmove: c.Cmove };
  let r;
  try { r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: build(c) }, opts: { solveSketches: true } } })).json); } catch (e) { r = null; }
  if (!r || !r.ok) { rows.push({ id: c.id, group: c.group, cls: 'regen-fail', detail: JSON.stringify(r?.error ?? r).slice(0, 140) }); continue; }
  const errs = (r.result?.errors ?? []).map(([, e]) => e.code);
  const real = errs.filter((e) => e !== 'combineEmpty' && e !== 'nothingToCut' && e !== 'cutNoEffect');
  if (real.length) { rows.push({ id: c.id, group: c.group, cls: real[0], detail: errs.join(','), params: { op: c.op, op2: c.op2, A: c.A, B: c.B, C: c.C, move: c.move, Cmove: c.Cmove } }); continue; }
  const benign = errs.includes('combineEmpty') ? 'okEmpty' : (errs.includes('nothingToCut') || errs.includes('cutNoEffect')) ? 'okNoEffect' : 'ok';
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  rows.push({ id: c.id, group: c.group, cls: (v.result?.issues ?? []).length ? 'invalid' : benign, detail: (v.result?.issues ?? []).map((x) => x.code).join(','), vol: m.result?.volume ?? null, params: { op: c.op, op2: c.op2, A: c.A, B: c.B, C: c.C, move: c.move, Cmove: c.Cmove } });
}

const rp = '/tmp/boolean_sweep_cases.json', ro = '/tmp/boolean_sweep_ref.json';
writeFileSync(rp, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here, 'boolean_sweep_ref.py'), '--cases', rp, '--out', ro], { stdio: 'pipe', timeout: 180000 }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(ro) ? JSON.parse(readFileSync(ro, 'utf8')) : {};
for (const row of rows) {
  const rf = ref[row.id];
  if (!rf) { if (row.cls === 'ok') row.cls = 'no-ref'; continue; }
  if (rf.error) { row.cls = 'ref-fails'; row.detail = rf.error; continue; }
  if (row.cls === 'ok' && row.vol != null && rf.volume != null) {
    const rel = Math.abs(row.vol - rf.volume) / Math.max(Math.abs(rf.volume), 1e-9);
    if (rel > 1e-3) { row.cls = 'wrongVolume'; row.detail = `rel ${rel.toExponential(2)} vol ${row.vol.toFixed(3)} ref ${rf.volume.toFixed(3)}`; }
    else if (Math.abs(row.vol) < 1e-9 && rf.volume > 1e-9) row.cls = 'empty';
  }
}

const g = {}; for (const r of rows) { g[r.group] ??= {}; g[r.group][r.cls] = (g[r.group][r.cls] ?? 0) + 1; }
const tot = {}; for (const r of rows) tot[r.cls] = (tot[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, seed, samples: per, wasmHash, ...tot }));
for (const [t, x] of Object.entries(g)) console.log(`${t}: ${JSON.stringify(x)}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: opt('--wasm', 'kit'), wasmHash, seed, samples: per, cases: Object.fromEntries(rows.map((r) => [r.id, r.cls])) }, null, 1) + '\n'); console.log(`wrote ${baselineFile}`); }
  else { const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {}; const reg = rows.filter((r) => base[r.id] === 'ok' && r.cls !== 'ok'); console.log(`boolean_sweep check: ${reg.length} regressions`); if (reg.length) process.exit(1); }
}
