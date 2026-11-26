// Cylinder-into-box timing through the wasm kernel (the app's path): plain cylinder cut / boss / hole feature on a
// 100 x 60 x 40 box, and 1 / 10 / 50 holes in a plate (hole feature + linear pattern, and a pattern of sketch circles in one cut).
// Each case: a cold regen of the full part (fresh kernel instance each run, node/JIT warm), after WARM warm-up runs, median of RUNS runs. The "box only"
// row is the same regen without the cylinder step, so the cost of the cylinder step is the difference.
// usage: node tools/bench_boolean_cyl.mjs [--wasm path] [--runs 20] [--warm 3] [--json out.json] [--quiet]
// Native counterpart (ck-ops boolean call and ck-model regen): cargo test --profile fast -p ck-model --test bench_boolean_cyl -- --ignored --nocapture
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const arg = (n, d) => { const i = process.argv.indexOf(n); return i > 0 ? process.argv[i + 1] : d; };
const wasm = arg('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const RUNS = Number(arg('--runs', 20)), WARM = Number(arg('--warm', 3));
const quiet = process.argv.includes('--quiet');
const mod = new WebAssembly.Module(readFileSync(wasm));
const newKernel = () => new Kernel(mod, { now: () => performance.now(), unixMs: () => Date.now() });
const rq = (k, msg) => { const r = k.request(JSON.stringify({ id: 1, msg })); if (r.trap) return { trap: r.trap }; try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; } };

const W = 100, D = 60, H = 40, R = 10;
const rect = (id, w, d) => {
  const p = [[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]];
  return { type: 'sketch', id, plane: { kind: 'std', id: 'front' }, points: p.map(([x, y], i) => ({ id: 'p' + i, x, y })),
    entities: p.map((_, i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + i, p2: 'p' + ((i + 1) % 4) })), constraints: [] };
};
const box = (w, d, h) => [rect('S1', w, d), { type: 'extrude', id: 'E1', op: 'add', sketch: 'S1', end: 'blind', depth: h }];
const top = (h) => ({ type: 'plane', id: 'PT', mode: 'offset', base: { kind: 'std', id: 'front' }, offset: h, name: 'top' });
const circles = (pts, r) => ({ type: 'sketch', id: 'S2', plane: { kind: 'feature', id: 'PT' },
  points: pts.map(([x, y], i) => ({ id: 'c' + i, x, y })), entities: pts.map((_, i) => ({ type: 'circle', id: 'k' + i, c: 'c' + i, r })), constraints: [] });
const holeSk = (x, y) => ({ type: 'sketch', id: 'S2', plane: { kind: 'feature', id: 'PT' }, points: [{ id: 'p1', x, y }], entities: [{ type: 'point', id: 'e1', p: 'p1' }], constraints: [] });
const part = (features) => ({ format: 'ck-part', version: 1, features });
const grid = (n, w, d) => { // n hole centres on a grid inside w x d
  const cols = Math.ceil(Math.sqrt(n * w / d)), rows = Math.ceil(n / cols), pts = [];
  for (let i = 0; i < n; i++) pts.push([-w / 2 + (w / cols) * (0.5 + i % cols), -d / 2 + (d / rows) * (0.5 + Math.floor(i / cols))]);
  return pts;
};

const cases = [];
const add = (name, features, expect) => cases.push({ name, part: part(features), expect });
const base = box(W, D, H);
add('box only (extrude 100x60x40)', base);
add('cut: extrude cut r10 through', [...base, top(H), circles([[0, 0]], R), { type: 'extrude', id: 'F3', op: 'cut', sketch: 'S2', end: 'through' }], { dv: -Math.PI * R * R * H });
add('cut: extrude cut r10 blind 20', [...base, top(H), circles([[0, 0]], R), { type: 'extrude', id: 'F3', op: 'cut', sketch: 'S2', end: 'blind', depth: 20 }], { dv: -Math.PI * R * R * 20 });
add('union: extrude boss r10 h20 on top', [...base, top(H), circles([[0, 0]], R), { type: 'extrude', id: 'F3', op: 'add', sketch: 'S2', end: 'blind', depth: 20 }], { dv: Math.PI * R * R * 20 });
add('hole feature d20 through', [...base, top(H), holeSk(0, 0), { type: 'hole', id: 'F3', sketch: 'S2', kind: 'simple', diameter: 2 * R, end: 'through' }], { dv: -Math.PI * R * R * H });
add('hole feature d20 blind 20', [...base, top(H), holeSk(0, 0), { type: 'hole', id: 'F3', sketch: 'S2', kind: 'simple', diameter: 2 * R, end: 'blind', depth: 20 }], { dv: -Math.PI * R * R * 20 });
// scale: plate 300 x 200 x 10, holes d10 through
const PW = 300, PD = 200, PH = 10, HR = 5;
const plate = box(PW, PD, PH);
add('plate only (300x200x10)', plate);
for (const n of [1, 10, 50]) {
  add(`plate, ${n} circles in ONE extrude cut`, [...plate, top(PH), circles(grid(n, PW - 40, PD - 40), HR), { type: 'extrude', id: 'F3', op: 'cut', sketch: 'S2', end: 'through' }], { dv: -n * Math.PI * HR * HR * PH });
}
for (const n of [10, 50]) {
  const c = Math.min(n, 10), r = Math.ceil(n / c);
  add(`plate, ${n} holes: 1 hole feature + linear pattern${r > 1 ? ' x2 dirs' : ''}`, [...plate, top(PH), holeSk(-130, -80),
    { type: 'hole', id: 'F3', sketch: 'S2', kind: 'simple', diameter: 2 * HR, end: 'through' },
    r > 1 ? { type: 'lpattern', id: 'F4', seeds: ['F3'], dir: { kind: 'axis', axis: 'x' }, spacing: 26, count: c, dir2: { kind: 'axis', axis: 'y' }, spacing2: 25, count2: r }
          : { type: 'lpattern', id: 'F4', seeds: ['F3'], dir: { kind: 'axis', axis: 'x' }, spacing: 26, count: c }], { dv: -n * Math.PI * HR * HR * PH });
}
add('plate, 50 separate hole features (50 sequential cuts)', [...plate, top(PH), ...grid(50, PW - 40, PD - 40).flatMap(([x, y], i) => [
  { type: 'sketch', id: `H${i}s`, plane: { kind: 'feature', id: 'PT' }, points: [{ id: 'p1', x, y }], entities: [{ type: 'point', id: 'e1', p: 'p1' }], constraints: [] },
  { type: 'hole', id: `H${i}`, sketch: `H${i}s`, kind: 'simple', diameter: 2 * HR, end: 'through' }])], { dv: -50 * Math.PI * HR * HR * PH });

const med = (a) => { const s = [...a].sort((x, y) => x - y); return s.length % 2 ? s[(s.length - 1) / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2; };
const volume = (k) => rq(k, { type: 'massProps', tag: 'main' }).result?.volume;
const out = { machine: `${os.cpus()[0].model.trim()} x${os.cpus().length}`, node: process.version, runs: RUNS, warm: WARM, rows: [] };
let v0 = null, vBox = null;
const baseVol = {};
for (const c of cases) {
  let k = newKernel();
  const times = [];
  let res, err = null;
  for (let i = 0; i < WARM + RUNS; i++) {
    k = newKernel(); // a fresh kernel per run: an identical regen on a warm one is served from the regen cache
    const t0 = performance.now();
    res = rq(k, { type: 'regen', tag: 'main', part: c.part });
    const ms = performance.now() - t0;
    if (res.trap) { err = 'trap ' + res.trap; break; }
    const e = (res.result?.errors ?? []).map(([id, x]) => `${id}:${x.code}`);
    if (e.length) { err = 'errors ' + e.join(' '); break; }
    if (i >= WARM) times.push(ms);
  }
  const row = { name: c.name, medianMs: err ? null : Number(med(times).toFixed(2)), minMs: err ? null : Number(Math.min(...times).toFixed(2)), maxMs: err ? null : Number(Math.max(...times).toFixed(2)) };
  if (err) row.error = err;
  else {
    const v = volume(k);
    row.volume = Number(v.toFixed(3));
    if (c.name.startsWith('box only')) baseVol.box = v; else if (c.name.startsWith('plate only')) baseVol.plate = v;
    const b = c.name.startsWith('plate') ? baseVol.plate : baseVol.box;
    if (c.expect && b != null) { row.volumeOk = Math.abs(v - b - c.expect.dv) < 1e-3 * Math.abs(c.expect.dv); }
    const issues = rq(k, { type: 'validate', tag: 'main', mesh: true }).result?.issues ?? ['no validate reply'];
    row.validateIssues = issues.length;
  }
  out.rows.push(row);
  if (!quiet) console.log(`${row.name.padEnd(62)} ${row.error ?? `${String(row.medianMs).padStart(8)} ms  (min ${row.minMs}, max ${row.maxMs})  vol ${row.volume}${row.volumeOk === false ? '  VOLUME MISMATCH' : ''}${row.validateIssues ? '  validate issues ' + row.validateIssues : ''}`}`);
}
if (arg('--json')) writeFileSync(arg('--json'), JSON.stringify(out, null, 2) + '\n');
const bad = out.rows.some((r) => r.error || r.volumeOk === false || r.validateIssues);
console.log(`bench_boolean_cyl: ${bad ? 'CHECK ROWS ABOVE' : 'ok'} (${out.machine}, ${RUNS} runs after ${WARM} warm-up)`);
process.exit(bad ? 1 : 0);
