// Parametric shell / thicken sweep against OCCT. Seeded, reproducible.
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
const samples = Number(opt('--samples', '40'));
const only = opt('--only', '');
const mod = new WebAssembly.Module(readFileSync(wasm));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const k = new Kernel(mod);
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'shell_sweep_baseline.json')));

let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const pt = (id, x, y) => ({ id, x, y });
const line = (id, p1, p2, extra = {}) => ({ type: 'line', id, p1, p2, ...extra });
const circ = (id, c, r) => ({ type: 'circle', id, c, r });
function rect(x0, y0, x1, y1) {
  const P = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y], i) => pt(`p${i}`, x, y));
  const E = [0, 1, 2, 3].map((i) => line(`e${i + 1}`, `p${i}`, `p${(i + 1) % 4}`));
  return [P, E];
}
let n = 0;
const plane = (z) => (z === 0 ? [{ kind: 'std', id: 'front' }, []] : [{ kind: 'feature', id: `P${++n}` }, [{ type: 'plane', id: `P${n}`, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z }]]);
const sk = (x0, y0, x1, y1, z = 0) => {
  const [pl, pre] = plane(z); const [P, E] = rect(x0, y0, x1, y1);
  return [...pre, { type: 'sketch', id: `S${++n}`, plane: pl, points: P, entities: E, constraints: [] }];
};
const circSk = (r, z = 0) => { const [pl, pre] = plane(z); return [...pre, { type: 'sketch', id: `S${++n}`, plane: pl, points: [pt('c', 0, 0)], entities: [circ('e1', 'c', r)], constraints: [] }]; };
const ex = (id, sid, depth, extra = {}) => ({ type: 'extrude', id, op: 'add', sketch: sid, end: 'blind', depth, ...extra });
const box = (w, d, h, z = 0) => { const f = sk(0, 0, w, d, z); return [...f, ex(`F${++n}`, f[f.length - 1].id, h)]; };
const cyl = (r, h, z = 0) => { const f = circSk(r, z); return [...f, ex(`F${++n}`, f[f.length - 1].id, h)]; };
const tube = (ro, ri, h) => { const f = [...circSk(ro, 0)]; f[f.length - 1].entities.push(circ('e2', 'c', ri)); return [...f, ex(`F${++n}`, f[f.length - 1].id, h)]; };
const cone = (r, h, draft) => { const f = circSk(r, 0); return [...f, ex(`F${++n}`, f[f.length - 1].id, h, { draft })]; };

const cases = [];
const add = (id, template, base, op, ref) => cases.push({ id, template, base, op, ref });
function gen() {
  for (let i = 0; i < samples; i++) {
    const w = rr(20, 70), d = rr(20, 70), h = rr(10, 60), min = Math.min(w, d, h);
    const tr = [0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.6][Math.floor(rnd() * 7)];
    const t = tr * min;
    add(`box_top_${i}`, 'box_top', { kind: 'box', w, d, h }, { thickness: t, remove: ['top'] }, { kind: 'shell' });
    add(`box_side_${i}`, 'box_side', { kind: 'box', w, d, h }, { thickness: t, remove: ['side_ymin'] }, { kind: 'shell' });
    add(`box_two_${i}`, 'box_two', { kind: 'box', w, d, h }, { thickness: t, remove: ['top', 'side_ymin'] }, { kind: 'shell' });
    add(`box_out_${i}`, 'box_out', { kind: 'box', w, d, h }, { thickness: t, outward: true, remove: ['top'] }, { kind: 'shell' });

    const r = rr(8, 30), ch = rr(10, 60);
    add(`cyl_top_${i}`, 'cyl_top', { kind: 'cyl', r, h: ch }, { thickness: Math.min(t, r * 0.6), remove: ['top'] }, { kind: 'shell' });

    const ro = rr(12, 40), ri = ro * rr(0.3, 0.8), th = rr(10, 50);
    add(`tube_top_${i}`, 'tube_top', { kind: 'tube', ro, ri, h: th }, { thickness: Math.min(t, (ro - ri) * 0.6), remove: ['top'] }, { kind: 'shell' });

    const cr = rr(10, 30), cd = rr(5, 25), chh = rr(10, 50);
    add(`cone_top_${i}`, 'cone_top', { kind: 'cone', r: cr, h: chh, draft: cd }, { thickness: Math.min(t, cr * 0.3), remove: ['top'] }, { kind: 'shell' });

    const lw = rr(30, 80), ld = rr(20, 50), lh = rr(20, 60), lt = rr(2, 8);
    add(`l_closed_${i}`, 'l_closed', { kind: 'l', w: lw, d: ld, h: lh, t: lt }, { thickness: Math.min(t, 0.2 * Math.min(lw, ld, lh)), remove: [] }, { kind: 'shell' });

    const fw = rr(20, 60), fd = rr(20, 60), fh = rr(20, 60), fr = rr(0.02, 0.2) * Math.min(fw, fd, fh), ft = Math.min(t, fr * 0.8);
    add(`fillet_shell_${i}`, 'fillet_shell', { kind: 'filletbox', w: fw, d: fd, h: fh, r: fr }, { thickness: ft, remove: ['top'] }, { kind: 'shell' });

    const pw = rr(20, 60), pd = rr(20, 60), pt = rr(0.2, 5);
    add(`thicken_planar_${i}`, 'thicken_planar', { kind: 'sheet_planar', w: pw, d: pd }, { thickness: pt, side: 'both' }, { kind: 'thicken' });

    const sr = rr(5, 25), sh = rr(10, 50), st = rr(0.2, Math.min(3, sr * 0.5));
    add(`thicken_cyl_${i}`, 'thicken_cyl', { kind: 'sheet_cyl', r: sr, h: sh }, { thickness: st, side: 'both' }, { kind: 'thicken' });

    const r1 = rr(5, 25), r2 = rr(3, 20), ch2 = rr(10, 50), ct = rr(0.2, 2);
    add(`thicken_cone_${i}`, 'thicken_cone', { kind: 'sheet_cone', r1, r2, h: ch2 }, { thickness: ct, side: 'both' }, { kind: 'thicken' });
  }
}
gen();
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;

function featFor(c) {
  n = 0;
  const b = c.base;
  let f, faces;
  if (b.kind === 'box') {
    f = box(b.w, b.d, b.h); faces = { top: 'F2.C1', side_ymin: 'F2.S.e1' };
  } else if (b.kind === 'cyl') {
    f = cyl(b.r, b.h); faces = { top: 'F2.C1' };
  } else if (b.kind === 'tube') {
    f = tube(b.ro, b.ri, b.h); faces = { top: 'F2.C1' };
  } else if (b.kind === 'cone') {
    f = cone(b.r, b.h, b.draft); faces = { top: 'F2.C1' };
  } else if (b.kind === 'l') {
    const base = box(b.w, b.d, b.t); const wall = box(b.t, b.d, b.h, b.t);
    f = [...base, ...wall]; faces = {};
  } else if (b.kind === 'filletbox') {
    f = box(b.w, b.d, b.h);
    f.push({ type: 'fillet', id: 'B1', radius: b.r, edges: [{ faces: ['F2.S.e1', 'F2.S.e2'], at: [b.w, 0, b.h / 2] }] });
    faces = { top: 'F2.C1' };
  } else if (b.kind === 'sheet_planar') {
    f = sk(0, 0, b.w, b.d, 0);
    const sf = `F${++n}`;
    f.push({ type: 'surfacePlanar', id: sf, sketch: f[f.length - 1].id });
    f.push({ type: 'thicken', id: `T${++n}`, surface: sf, thickness: c.op.thickness, side: c.op.side });
    return f;
  } else if (b.kind === 'sheet_cyl') {
    const p1 = plane(0), p2 = plane(b.h), s1 = `S${++n}`, s2 = `S${++n}`, lf = `L${++n}`;
    f = [...p1[1], { type: 'sketch', id: s1, plane: p1[0], points: [pt('c', 0, 0)], entities: [circ('e1', 'c', b.r)], constraints: [] },
      ...p2[1], { type: 'sketch', id: s2, plane: p2[0], points: [pt('c', 0, 0)], entities: [circ('e1', 'c', b.r)], constraints: [] },
      { type: 'loft', id: lf, op: 'add', surface: true, sketches: [s1, s2] },
      { type: 'thicken', id: `T${++n}`, surface: lf, thickness: c.op.thickness, side: c.op.side }];
    return f;
  } else if (b.kind === 'sheet_cone') {
    const p1 = plane(0), p2 = plane(b.h);
    const s1 = `S${++n}`, s2 = `S${++n}`;
    f = [...p1[1], { type: 'sketch', id: s1, plane: p1[0], points: [pt('c', 0, 0)], entities: [circ('e1', 'c', b.r1)], constraints: [] },
      ...p2[1], { type: 'sketch', id: s2, plane: p2[0], points: [pt('c', 0, 0)], entities: [circ('e1', 'c', b.r2)], constraints: [] },
      ...((() => { const lf = `L${++n}`; return [{ type: 'loft', id: lf, op: 'add', surface: true, sketches: [s1, s2] }, { type: 'thicken', id: `T${++n}`, surface: lf, thickness: c.op.thickness, side: c.op.side }]; })())];
    return f;
  }
  const remove = c.op.remove ?? [];
  const fs = remove.map((x) => faces[x]).filter(Boolean);
  f.push({ type: 'shell', id: `H${++n}`, faces: fs, thickness: c.op.thickness, ...(c.op.outward ? { outward: true } : {}) });
  return f;
}

const rows = [], refCases = {};
for (const c of selected) {
  refCases[c.id] = { base: c.base, op: c.op, ref: c.ref };
  const f = featFor(c);
  refCases[c.id] = { base: c.base, op: c.op, ref: c.ref };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features: f }, opts: { solveSketches: true } } })).json);
  if (!r || !r.ok) { rows.push({ id: c.id, template: c.template, cls: 'regen-fail', detail: JSON.stringify(r?.error ?? r).slice(0, 150) }); continue; }
  const errs = (r.result?.errors ?? []).map(([, e]) => e.code);
  if (errs.length) {
    const refused = errs.some((e) => /thick|maxThickness|reach|apex|tip/i.test(e));
    rows.push({ id: c.id, template: c.template, cls: refused ? 'refused' : errs[0], detail: errs.join(','), params: c.base, op: c.op });
    continue;
  }
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  const vol = m.result?.volume ?? null;
  rows.push({ id: c.id, template: c.template, cls: (v.result?.issues ?? []).length ? 'invalid' : 'ok', detail: (v.result?.issues ?? []).map((x) => x.code).join(','), vol, params: c.base, op: c.op });
}

const refPath = '/tmp/shell_sweep_cases.json', refOut = '/tmp/shell_sweep_ref.json';
writeFileSync(refPath, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here, 'shell_sweep_ref.py'), '--cases', refPath, '--out', refOut], { stdio: 'pipe' }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(refOut) ? JSON.parse(readFileSync(refOut, 'utf8')) : {};
for (const row of rows) {
  const rf = ref[row.id];
  if (!rf) { if (row.cls === 'ok' || row.cls === 'refused') row.cls = 'no-ref'; continue; }
  if (rf.error) { row.cls = 'ref-fails'; row.detail = rf.error; continue; }
  if (rf.area != null) {
    const want = rf.area * (row.op?.thickness ?? 0);
    if (row.cls === 'ok' && row.vol != null && Math.abs(row.vol - want) > Math.max(0.03 * Math.abs(want), 1e-6)) { row.cls = 'wrongVolume'; row.detail = `vol ${row.vol.toFixed(3)} want ${want.toFixed(3)}`; }
    continue;
  }
  if (rf.volume != null && row.cls === 'ok' && row.vol != null) {
    const rel = Math.abs(row.vol - rf.volume) / Math.max(Math.abs(rf.volume), 1e-9);
    if (rel > 1e-3) { row.cls = 'wrongVolume'; row.detail = `rel ${rel.toExponential(2)} vol ${row.vol.toFixed(3)} ref ${rf.volume.toFixed(3)}`; }
  }
}

const groups = {};
for (const r of rows) { groups[r.template] ??= {}; groups[r.template][r.cls] = (groups[r.template][r.cls] ?? 0) + 1; }
const totals = {}; for (const r of rows) totals[r.cls] = (totals[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, seed, samples, wasmHash, ...totals }));
for (const [t, g] of Object.entries(groups)) if (Object.keys(g).some((x) => x !== 'ok')) console.log(`${t}: ${JSON.stringify(g)}`);
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));
if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: opt('--wasm', 'kit'), wasmHash, seed, samples, cases: Object.fromEntries(rows.map((r) => [r.id, r.cls])) }, null, 1) + '\n'); console.log(`wrote ${baselineFile}`); }
  else {
    const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {};
    const reg = rows.filter((r) => base[r.id] === 'ok' && r.cls !== 'ok');
    console.log(`shell_sweep check: ${reg.length} regressions`);
    for (const r of reg) console.log(`  ${r.id}: ${r.cls} ${r.detail ?? ''}`);
    if (reg.length) process.exit(1);
  }
}
