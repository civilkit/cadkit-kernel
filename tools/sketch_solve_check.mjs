// opts.solveSketches through the wasm regen reply (docs/sketch-status.md).
// usage: node tools/sketch_solve_check.mjs [--wasm path]      exit 1 on any failure
//        node tools/sketch_solve_check.mjs --make-fixture <baseline.wasm>
//          writes tools/fixtures/sketch_solve_off.json: sha256 of the whole option-OFF reply for the
//          cases below and every BenchCAD part, from a build that predates the option (check 6).
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import { cases, mkPart, P } from './probe_sketch_status.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); return i < 0 ? null : args[i + 1]; };
const mk = args.includes('--make-fixture');
const wasm = mk ? opt('--make-fixture') : opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/fast/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));
const raw = (part, opts) => {
  const r = new Kernel(mod).request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(opts ? { opts } : {}) } }));
  if (r.trap) throw new Error(r.trap);
  return r.json;
};
const regen = (part, opts) => JSON.parse(raw(part, opts)).result;
const h = (s) => createHash('sha256').update(s).digest('hex');
const L = (id, a, b) => ({ type: 'line', id, p1: a, p2: b });
const D = (id, a, b, value) => ({ type: 'distance', id, refs: [a, b], value, offset: null });
const hv = cases.rectHV.constraints;
const rectEnts = cases.rectHV.entities;
const skewRect = cases.rectHV.points; // (0,0) (10,1) (11,5) (1,6)
const fix = (id, ref, value) => ({ type: 'fix', id, refs: [ref], value });
const SOLVE = { solveSketches: true };
const part = (points, entities, constraints) => mkPart({ points, entities, constraints });
const pts = (r) => r.sketches[0][1].sketch.points.map((p) => [p.x, p.y]);
const near = (a, b, t = 1e-6) => Math.abs(a - b) <= t;
const benchDir = path.join(here, 'bench/benchcad');
const benchPart = (id) => JSON.parse(readFileSync(path.join(benchDir, `${id}.ckpart.json`), 'utf8'));

const scenarios = {
  perp: part([P('a', 0, 0), P('b', 10, 1), P('c', 9, 8)], [L('e1', 'a', 'b'), L('e2', 'b', 'c')], [{ type: 'perpendicular', id: 'k1', refs: ['e1', 'e2'] }]),
  rectDist: part(skewRect, rectEnts, [...hv, D('c5', 'p1', 'p2', 25)]),
  freePin: part([P('p', 1, 2)], [], [fix('c1', 'p', [7, -3])]),
  dragQuad: part(skewRect, rectEnts, [...hv, fix('pin', 'p3', [14, 9])]),
  conflictPin: part(cases.rectFull.points, cases.rectFull.entities, [...cases.rectFull.constraints, fix('pin', 'p3', [30, 30])]),
};

if (mk) {
  const fx = { cases: {}, bench: {} };
  for (const [n, p] of Object.entries(scenarios)) fx.cases[n] = h(raw(p));
  for (const [n, c] of Object.entries(cases)) fx.cases['probe_' + n] = h(raw(mkPart(c)));
  const manifest = JSON.parse(readFileSync(path.join(benchDir, 'manifest.json'), 'utf8'));
  for (const m of manifest.parts) {
    if (manifest.known?.[m.id]?.skip) continue;
    fx.bench[m.id] = h(raw(benchPart(m.id)));
  }
  writeFileSync(path.join(here, 'fixtures/sketch_solve_off.json'), JSON.stringify(fx, null, 1) + '\n');
  console.log('wrote fixture', Object.keys(fx.cases).length, 'cases', Object.keys(fx.bench).length, 'parts');
  process.exit(0);
}

let bad = 0;
const check = (name, ok, extra = '') => { if (!ok) { bad++; console.log('FAIL', name, extra); } else console.log('ok  ', name); };

// 1. skewed perpendicular pair
{
  const before = pts(regen(scenarios.perp));
  const r = regen(scenarios.perp, SOLVE);
  const a = pts(r);
  const dot = (a[1][0] - a[0][0]) * (a[2][0] - a[1][0]) + (a[1][1] - a[0][1]) * (a[2][1] - a[1][1]);
  check('perpendicular: exactly perpendicular', Math.abs(dot) < 1e-9, `dot=${dot}`);
  check('perpendicular: coordinates changed', JSON.stringify(a) !== JSON.stringify(before));
  check('perpendicular: solved, no error', r.sketches[0][1].status.solved === true && r.errors.length === 0);
}
// 2. rectangle H/V + distance
{
  const a = pts(regen(scenarios.rectDist, SOLVE));
  check('rect distance: exact size, axis-aligned', near(a[1][0] - a[0][0], 25, 1e-9) && near(a[1][1], a[0][1], 1e-9) && near(a[2][0], a[1][0], 1e-9) && near(a[3][1], a[2][1], 1e-9) && near(a[3][0], a[0][0], 1e-9), JSON.stringify(a));
}
// 3. free point pinned
{
  const a = pts(regen(scenarios.freePin, SOLVE));
  check('fix pin: free point lands on pin', near(a[0][0], 7, 1e-9) && near(a[0][1], -3, 1e-9), JSON.stringify(a));
}
// 4. drag: quad with a pinned corner
{
  const r = regen(scenarios.dragQuad, SOLVE);
  const a = pts(r);
  check('drag: pinned corner on pin', near(a[2][0], 14, 1e-9) && near(a[2][1], 9, 1e-9), JSON.stringify(a));
  check('drag: connected corners follow', near(a[1][0], 14, 1e-9) && near(a[3][1], 9, 1e-9) && near(a[0][1], a[1][1], 1e-9) && near(a[0][0], a[3][0], 1e-9), JSON.stringify(a));
  check('drag: solved', r.sketches[0][1].status.solved === true);
}
// 5. fully constrained + conflicting pin
{
  const off = pts(regen(scenarios.conflictPin));
  const r = regen(scenarios.conflictPin, SOLVE);
  const st = r.sketches[0][1].status;
  check('conflict: not solved', st.solved === false, JSON.stringify(st));
  check('conflict: original coordinates', JSON.stringify(pts(r)) === JSON.stringify(off));
  check('conflict: names the pin', st.conflicts.includes('pin'), JSON.stringify(st));
  check('conflict: no regen error', r.errors.length === 0);
}
// downstream: extrude consumes the solved profile
{
  const sk = part(skewRect, rectEnts, [...hv, D('c5', 'p1', 'p2', 25)]);
  sk.features.push({ type: 'extrude', id: 'F2', sketch: 'F1', op: 'add', end: 'blind', depth: 2 });
  const width = (o) => { const b = regen(sk, o).body; return b && b.box ? b.box.max[0] - b.box.min[0] : null; };
  const w = width(SOLVE), w0 = width(null);
  check('downstream extrude uses solved profile', w !== null && near(w, 25, 1e-6) && w0 !== null && !near(w0, 25, 1e-3), `${w} vs ${w0}`);
}
// 6. option OFF byte-identical to a pre-option build (fixture), plus literal-true semantics
{
  const fxp = path.join(here, 'fixtures/sketch_solve_off.json');
  if (!existsSync(fxp)) check('off: fixture present', false, fxp);
  else {
    const fx = JSON.parse(readFileSync(fxp, 'utf8'));
    let n = 0;
    const mism = [];
    for (const [k, p] of Object.entries(scenarios)) { n++; if (h(raw(p)) !== fx.cases[k]) mism.push(k); }
    for (const [k, c] of Object.entries(cases)) { n++; if (h(raw(mkPart(c))) !== fx.cases['probe_' + k]) mism.push('probe_' + k); }
    for (const [id, want] of Object.entries(fx.bench)) { n++; if (h(raw(benchPart(id))) !== want) mism.push(id); }
    check(`off: ${n} replies byte-identical to the pre-option build`, mism.length === 0, mism.join(','));
  }
  check('off: solveSketches:1 is ignored', raw(scenarios.perp) === raw(scenarios.perp, { solveSketches: 1 }));
  check('on: status included without sketchStatus', regen(scenarios.perp, SOLVE).sketches[0][1].status !== undefined);
  check('on: sketchStatus alongside is identical', raw(scenarios.perp, { solveSketches: true, sketchStatus: true }) === raw(scenarios.perp, SOLVE));
}
process.exit(bad ? 1 : 0);
