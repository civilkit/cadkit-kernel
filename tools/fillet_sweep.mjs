// Parametric fillet/chamfer sweep against OCCT. Seeded, reproducible.
// Templates: box, cylinder rim, tube rim, boss foot/top, hole rim, chamfer,
// variable-radius edge. Edge picks are geometric (point + direction).
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
const baselineFile = opt('--update-baseline', opt('--check', path.join(here, 'fillet_sweep_baseline.json')));

let s = seed >>> 0;
const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();

// ---------------------------------------------------------------- features
let n = 0;
const plane = (z) => {
  if (z === 0) return [{ kind: 'std', id: 'front' }, []];
  const id = `P${++n}`;
  return [{ kind: 'feature', id }, [{ type: 'plane', id, mode: 'offset', base: { kind: 'std', id: 'front' }, offset: z, visible: false }]];
};
const rectSketch = (id, pl, x0, y0, x1, y1) => ({
  type: 'sketch', id, plane: pl,
  points: [{ id: 'a', x: x0, y: y0 }, { id: 'b', x: x1, y: y0 }, { id: 'c', x: x1, y: y1 }, { id: 'd', x: x0, y: y1 }],
  entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b' }, { type: 'line', id: 'e2', p1: 'b', p2: 'c' },
    { type: 'line', id: 'e3', p1: 'c', p2: 'd' }, { type: 'line', id: 'e4', p1: 'd', p2: 'a' }], constraints: [],
});
const circSketch = (id, pl, cx, cy, r) => ({
  type: 'sketch', id, plane: pl, points: [{ id: 'c', x: cx, y: cy }],
  entities: [{ type: 'circle', id: 'e1', c: 'c', r }], constraints: [],
});
function box(w, d, h, x0 = 0, y0 = 0, z0 = 0, op = 'add') {
  const [pl, pre] = plane(z0);
  const sid = `S${++n}`;
  return [...pre, { type: 'sketch', id: sid, ...rectSketch(sid, pl, x0, y0, x0 + w, y0 + d) }, { type: 'extrude', id: `F${++n}`, op, sketch: sid, end: 'blind', depth: h }];
}
function cyl(r, h, cx = 0, cy = 0, z0 = 0, op = 'add') {
  const [pl, pre] = plane(z0);
  const sid = `S${++n}`;
  return [...pre, { type: 'sketch', id: sid, ...circSketch(sid, pl, cx, cy, r) }, { type: 'extrude', id: `F${++n}`, op, sketch: sid, end: 'blind', depth: h, ...(op === 'cut' ? { flip: true } : {}) }];
}
function rrectBase(p) {
  const { w, d, cr, h } = p;
  const pts = [
    ['p1', cr, 0], ['p2', w - cr, 0], ['p3', w, cr], ['p4', w, d - cr],
    ['p5', w - cr, d], ['p6', cr, d], ['p7', 0, d - cr], ['p8', 0, cr],
    ['c1', w - cr, cr], ['c2', w - cr, d - cr], ['c3', cr, d - cr], ['c4', cr, cr],
  ];
  const ents = [
    { type: 'line', id: 'e1', p1: 'p1', p2: 'p2' },
    { type: 'arc', id: 'e2', c: 'c1', p1: 'p2', p2: 'p3' },
    { type: 'line', id: 'e3', p1: 'p3', p2: 'p4' },
    { type: 'arc', id: 'e4', c: 'c2', p1: 'p4', p2: 'p5' },
    { type: 'line', id: 'e5', p1: 'p5', p2: 'p6' },
    { type: 'arc', id: 'e6', c: 'c3', p1: 'p6', p2: 'p7' },
    { type: 'line', id: 'e7', p1: 'p7', p2: 'p8' },
    { type: 'arc', id: 'e8', c: 'c4', p1: 'p8', p2: 'p1' },
  ];
  const sid = `S${++n}`;
  return [
    { type: 'sketch', id: sid, plane: { kind: 'std', id: 'front' }, points: pts.map(([id, x, y]) => ({ id, x, y })), entities: ents, constraints: [] },
    { type: 'extrude', id: `F${++n}`, op: 'add', sketch: sid, end: 'blind', depth: h },
  ];
}

const baseFns = {
  box: (p) => box(p.w, p.d, p.h),
  cyl: (p) => cyl(p.r, p.h),
  tube: (p) => [...cyl(p.ro, p.h), ...cyl(p.ri, p.h + 2, 0, 0, -1, 'cut')],
  boss: (p) => [...box(p.w, p.d, p.pt), ...cyl(p.br, p.bh, p.bx, p.by, p.pt)],
  hole: (p) => [...box(p.w, p.d, p.h), ...cyl(p.r, p.h + 2, p.cx, p.cy, -1, 'cut')],
  rrect: (p) => rrectBase(p),

};

// ---------------------------------------------------------------- edge selection
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function distSeg(p, a, b) {
  const ab = sub(b, a), ap = sub(p, a), t = Math.max(0, Math.min(1, dot(ap, ab) / (dot(ab, ab) || 1)));
  const q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t], d = sub(p, q);
  return Math.hypot(d[0], d[1], d[2]);
}
function findEdge(body, at, dir) {
  let best = null;
  for (const e of body.edges ?? []) {
    const pts = e.pts?.length >= 2 ? e.pts : [e.mid, e.mid];
    let d = Math.hypot(...sub(at, e.mid)), de = null;
    for (let i = 0; i + 1 < pts.length; i++) { const dd = distSeg(at, pts[i], pts[i + 1]); if (dd <= d) { d = dd; de = sub(pts[i + 1], pts[i]); } }
    let pen = 0;
    if (dir && de) { const l = Math.hypot(...de); if (l) pen = (1 - Math.abs(dot(dir, [de[0] / l, de[1] / l, de[2] / l]))) * 0.05; }
    const score = d + pen;
    if (!best || score < best.s) best = { s: score, d, e };
  }
  return best && best.d < 0.05 ? best.e : null;
}
function req(msg) { const r = k.request(JSON.stringify({ id: 1, msg })); try { return JSON.parse(r.json); } catch { return { trap: r.trap }; } }
function regen(features) { return req({ type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features } }); }
function mass() { return req({ type: 'massProps', tag: 'main', bodies: true }); }

// ---------------------------------------------------------------- cases
const cases = [];
const addCase = (id, template, base, op, picks) => cases.push({ id, template, base, op, picks });
function boxPicks(p, kind) {
  const { w, d, h } = p;
  if (kind === 'edge1') return [{ at: [0, 0, h / 2], dir: [0, 0, 1] }];
  if (kind === 'parallel4') return [[0, 0, h / 2], [w, 0, h / 2], [w, d, h / 2], [0, d, h / 2]].map((at) => ({ at, dir: [0, 0, 1] }));
  if (kind === 'corner3') return [{ at: [w / 2, d, h], dir: [1, 0, 0] }, { at: [w, d / 2, h], dir: [0, 1, 0] }, { at: [w, d, h / 2], dir: [0, 0, 1] }];
  // all 12: 4 vertical + 4 top + 4 bottom
  const out = [];
  for (const [x, y] of [[0, 0], [w, 0], [w, d], [0, d]]) out.push({ at: [x, y, h / 2], dir: [0, 0, 1] });
  for (const [x, y] of [[w / 2, 0], [w, d / 2], [w / 2, d], [0, d / 2]]) { out.push({ at: [x, y, h], dir: [1, 0, 0] }); out.push({ at: [x, y, 0], dir: [1, 0, 0] }); }
  return out;
}
function generate() {
  const N = samples;
  for (let i = 0; i < N; i++) {
    const w = rr(10, 60), d = rr(10, 60), h = rr(10, 60), r = rr(0.02, 0.6) * Math.min(w, d, h);
    const p = { w, d, h };
    for (const k2 of ['edge1', 'parallel4', 'corner3', 'all12']) addCase(`box_${k2}_${i}`, `box_${k2}`, p, { kind: 'fillet', radius: r }, boxPicks(p, k2));
    const rc = rr(5, 40), hc = rr(5, 50), fr = rr(0.02, 0.8) * rc;
    const cp = { r: rc, h: hc };
    addCase(`cyl_top_${i}`, 'cyl_top', cp, { kind: 'fillet', radius: fr }, [{ at: [rc, 0, hc], dir: [0, 1, 0] }]);
    addCase(`cyl_bot_${i}`, 'cyl_bot', cp, { kind: 'fillet', radius: fr }, [{ at: [rc, 0, 0], dir: [0, 1, 0] }]);
    const ro = rr(10, 40), ri = rr(0.3 * ro, 0.8 * ro), ht = rr(10, 50), tr = rr(0.02, 0.8) * (ro - ri);
    const tp = { ro, ri, h: ht };
    addCase(`tube_outer_${i}`, 'tube_outer', tp, { kind: 'fillet', radius: tr }, [{ at: [ro, 0, ht], dir: [0, 1, 0] }]);
    addCase(`tube_inner_${i}`, 'tube_inner', tp, { kind: 'fillet', radius: tr }, [{ at: [ri * 0.7071, ri * 0.7071, ht], dir: [-0.7071, 0.7071, 0] }]);
    const pw = rr(20, 60), pd = rr(20, 60), pt = rr(5, 20), br = rr(2, Math.min(pw, pd) * 0.3), bh = rr(5, 30), bfr = rr(0.05, 0.5) * Math.min(br, bh);
    const bp = { w: pw, d: pd, pt, bx: pw / 2, by: pd / 2, br, bh };
    addCase(`boss_foot_${i}`, 'boss_foot', bp, { kind: 'fillet', radius: bfr }, [{ at: [pw / 2 + br, pd / 2, pt], dir: [0, 1, 0] }]);
    addCase(`boss_top_${i}`, 'boss_top', bp, { kind: 'fillet', radius: bfr }, [{ at: [pw / 2 + br, pd / 2, pt + bh], dir: [0, 1, 0] }]);
    const hw = rr(20, 60), hd = rr(20, 60), hh = rr(5, 20), hr = rr(1, Math.min(hw, hd) * 0.3), hfr = rr(0.05, 0.5) * hr;
    const hp = { w: hw, d: hd, h: hh, cx: hw / 2, cy: hd / 2, r: hr };
    addCase(`hole_rim_${i}`, 'hole_rim', hp, { kind: 'fillet', radius: hfr }, [{ at: [hw / 2 + hr * 0.7071, hd / 2 + hr * 0.7071, hh], dir: [-0.7071, 0.7071, 0] }]);
    const cd = rr(0.02, 0.3) * Math.min(w, d, h);
    addCase(`chamfer_box_${i}`, 'chamfer_box', p, { kind: 'chamfer', d: cd }, boxPicks(p, 'edge1'));
    addCase(`chamfer2_box_${i}`, 'chamfer2_box', p, { kind: 'chamfer', type: 'two', d1: cd, d2: rr(0.02, 0.3) * Math.min(w, d, h), face: [0, 0, 1] }, [{ at: [w / 2, 0, h], dir: [1, 0, 0] }]);
    addCase(`chamferA_box_${i}`, 'chamferA_box', p, { kind: 'chamfer', type: 'angle', d: cd, a: rr(10, 80), face: [0, 0, 1] }, [{ at: [w / 2, 0, h], dir: [1, 0, 0] }]);
    const r1 = rr(0.02, 0.2) * Math.min(w, d, h), r2 = rr(0.25, 0.6) * Math.min(w, d, h);
    addCase(`var_box_${i}`, 'var_box', p, { kind: 'varfillet', r1, r2 }, boxPicks(p, 'edge1'));
  }
  for (let i = 0; i < N; i++) {
    const rw = rr(20, 60), rd = rr(20, 60), rcr = rr(2, Math.min(rw, rd) * 0.3), rh = rr(10, 40), rf = rr(0.05, 0.5) * Math.min(rcr, rh);
    const rp = { w: rw, d: rd, cr: rcr, h: rh };
    addCase(`rrect_loop_${i}`, 'rrect_loop', rp, { kind: 'fillet', radius: rf, propagate: true }, [{ at: [rw / 2, 0, rh], dir: [1, 0, 0] }]);
  }
}
generate();
const selected = only ? cases.filter((c) => c.id.includes(only)) : cases;

// ---------------------------------------------------------------- run
function featureFor(body, c) {
  const edges = c.picks.map((p) => {
    const e = findEdge(body, p.at, p.dir);
    if (!e) throw new Error(`no edge near ${p.at}`);
    return { faces: [...e.faces], at: p.at };
  });
  const o = c.op;
  if (o.kind === 'fillet') return { type: 'fillet', id: `B${++n}`, radius: o.radius, edges, ...(o.propagate ? { propagate: true } : {}) };
  if (o.kind === 'varfillet') return { type: 'fillet', id: `B${++n}`, filletType: 'variable', radii: [o.r1, o.r2], edges };
  if (o.kind === 'chamfer') {
    const f = { type: 'chamfer', id: `B${++n}`, chamferType: o.type === 'two' ? 'distanceDistance' : o.type === 'angle' ? 'angleDistance' : 'equal', distance: o.d ?? o.d1, edges };
    if (o.type === 'two') f.distance2 = o.d2;
    if (o.type === 'angle') f.angle = o.a;
    if (o.face) f.face = { kind: 'face', id: '' }; // replaced below by faceByNormal
    return f;
  }
  throw new Error('op');
}

const rows = [];
const refCases = {};
for (const c of selected) {
  const kind = c.template.startsWith('box') || c.template.includes('_box') ? 'box'
    : c.template.startsWith('cyl') ? 'cyl'
    : c.template.startsWith('tube') ? 'tube'
    : c.template.startsWith('boss') ? 'boss'
    : c.template.startsWith('hole') ? 'hole'
    : c.template.startsWith('rrect') ? 'rrect' : 'box';
  const baseFeats = baseFns[kind](c.base);
  const r = regen(baseFeats);
  if (!r.ok || r.result?.errors?.length) { rows.push({ id: c.id, template: c.template, cls: 'base-error', detail: JSON.stringify(r.result?.errors ?? r.error).slice(0, 120) }); continue; }
  let feat;
  try { feat = featureFor(r.result.body, c); } catch (e) { rows.push({ id: c.id, template: c.template, cls: 'pick-error', detail: e.message }); continue; }
  if (feat.type === 'chamfer' && c.op.face) {
    // find the planar face whose normal is closest to the requested direction
    let best = null;
    for (const f of r.result.body.faces) {
      if (!f.normal) continue;
      const d = dot(f.normal, c.op.face);
      if (!best || d > best.d) best = { d, id: f.id };
    }
    if (best && best.d > 0.99) feat.face = { kind: 'face', id: best.id }; else { rows.push({ id: c.id, template: c.template, cls: 'pick-error', detail: 'no face normal' }); continue; }
  }
  const r2 = regen([...baseFeats, feat]);
  if (!r2) { rows.push({ id: c.id, template: c.template, cls: 'kernel-null', detail: 'no response' }); continue; }
  const errs = (r2.result?.errors ?? []).map(([, e]) => e.code);
  if (errs.length) { rows.push({ id: c.id, template: c.template, cls: errs[0], detail: errs.join(',') }); continue; }
  const m = mass();
  const vol = m.result?.volume ?? null;
  const v = req({ type: 'validate', tag: 'main' });
  const issues = v.result?.issues ?? [];
  refCases[c.id] = { base: { kind, ...c.base }, op: c.op, picks: c.picks };
  rows.push({ id: c.id, template: c.template, cls: issues.length ? 'invalid' : 'ok', detail: issues.length ? [...new Set(issues.map((i) => i.code))].join(',') : '', vol, params: c.base, op: c.op });
}

// OCP reference
const refPath = '/tmp/fillet_sweep_cases.json', refOut = '/tmp/fillet_sweep_ref.json';
writeFileSync(refPath, JSON.stringify(refCases, null, 1));
try { execFileSync('python3.12', [path.join(here, 'fillet_sweep_ref.py'), '--cases', refPath, '--out', refOut], { stdio: 'pipe' }); } catch (e) { console.error('ref failed', e.message); }
const ref = existsSync(refOut) ? JSON.parse(readFileSync(refOut, 'utf8')) : {};
for (const row of rows) {
  const rf = ref[row.id];
  if (!rf) { if (row.cls === 'ok') row.cls = 'no-ref'; continue; }
  if (rf.error) { row.cls = 'ref-fails'; row.detail = rf.error; continue; }
  if (row.cls !== 'ok') continue;
  const rel = Math.abs(row.vol - rf.volume) / Math.max(Math.abs(rf.volume), 1e-12);
  row.ref = rf.volume; row.rel = rel;
  if (rel > 1e-4) row.cls = 'wrongVolume';
}

const byId = new Map(cases.map((c) => [c.id, c]));
for (const r of rows) { const c = byId.get(r.id); if (c) { r.params = c.base; r.op = c.op; } }
const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, seed, samples, ...by }));
const templates = [...new Set(rows.map((r) => r.template))];
for (const t of templates) {
  const rs = rows.filter((r) => r.template === t); const c = {};
  for (const r of rs) c[r.cls] = (c[r.cls] ?? 0) + 1;
  console.log(`${t}: ${JSON.stringify(c)}`);
}
const out = opt('--json'); if (out) writeFileSync(out, JSON.stringify(rows, null, 1));

if (baselineFile && (opt('--update-baseline') || opt('--check'))) {
  const casesOut = Object.fromEntries(rows.map((r) => [r.id, r.cls]));
  if (opt('--update-baseline')) { writeFileSync(baselineFile, JSON.stringify({ version: 1, wasm: opt('--wasm', 'kit'), wasmHash, seed, samples, cases: casesOut }, null, 1) + '\n'); console.log(`wrote ${baselineFile}`); }
  else {
    const base = JSON.parse(readFileSync(baselineFile, 'utf8')).cases ?? {};
    const pass = new Set(['ok', 'okNoEffect', 'ok-loose']);
    const reg = [];
    for (const r of rows) if (pass.has(base[r.id]) && !pass.has(r.cls)) reg.push(r);
    console.log(`fillet_sweep check: ${reg.length} regressions`);
    for (const r of reg) console.log(`REGRESSION ${r.id}: ${r.cls} ${r.detail}`);
    if (reg.length) process.exit(1);
  }
}
