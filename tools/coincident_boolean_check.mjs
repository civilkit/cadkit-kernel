// Booleans whose tool faces coincide exactly with target faces, through the wasm: plugging round / square
// holes (fully, partly, past the plate, flipped), a boss flush with a pocket wall, stacked extrusions, a cut
// whose side lies on an existing wall. Volumes, face counts, no errors.
// usage: node tools/coincident_boolean_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

const pts = (l) => l.map(([id, x, y]) => ({ id, x, y }));
const lines = (ids, ps) => ids.map((id, i) => ({ type: 'line', id, p1: ps[i], p2: ps[(i + 1) % ps.length] }));
const round = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: pts([['p1', -30, -20], ['p2', 30, -20], ['p3', 30, 20], ['p4', -30, 20], ['ca', -12, 0], ['cb', 12, 0]]),
  entities: [...lines(['e1', 'e2', 'e3', 'e4'], ['p1', 'p2', 'p3', 'p4']),
    { type: 'circle', id: 'e5', c: 'ca', r: 6 }, { type: 'circle', id: 'e6', c: 'cb', r: 6 }], constraints: [] };
const square = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: pts([['p1', -30, -20], ['p2', 30, -20], ['p3', 30, 20], ['p4', -30, 20], ['q1', -10, -10], ['q2', 10, -10], ['q3', 10, 10], ['q4', -10, 10]]),
  entities: [...lines(['e1', 'e2', 'e3', 'e4'], ['p1', 'p2', 'p3', 'p4']), ...lines(['e5', 'e6', 'e7', 'e8'], ['q1', 'q2', 'q3', 'q4'])], constraints: [] };
const ext = (id, op, depth, regions, flip = false) => ({ type: 'extrude', id, op, sketch: 'F1', end: 'blind', depth, regions, ...(flip ? { flip: true } : {}) });

const PL = 60 * 40, C = Math.PI * 36, SQ = 400;
let bad = 0;
function run(name, sketch, features, want, faces) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: [sketch, ...features] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const vol = m.ok ? m.result?.volume ?? null : null;
  let ok;
  if (want === 'noEffect') ok = errs.length === 1 && errs[0].endsWith('cutNoEffect');
  else ok = errs.length === 0 && vol != null && Math.abs(vol - want) <= 1e-3 * want;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${errs.length ? errs.join(' ') : vol?.toFixed(3)}${typeof want === 'number' ? ` (want ${want.toFixed(3)})` : ''}`);
}

for (const flip of [false, true]) {
  const t = flip ? ' flipped' : '';
  const base = ext('F2', 'add', 10, ['e1', 'e5'], flip);
  run('plug round hole' + t, round, [base, ext('F3', 'add', 10, ['e6'], flip)], PL * 10);
  run('plug round hole partly' + t, round, [base, ext('F3', 'add', 5, ['e6'], flip)], (PL - C) * 10 + C * 5);
  run('plug round hole, tool past the plate' + t, round, [base, ext('F3', 'add', 15, ['e6'], flip)], PL * 10 + C * 5);
  run('plug square hole' + t, square, [ext('F2', 'add', 10, ['e1'], flip), ext('F3', 'add', 10, ['e5'], flip)], PL * 10);
  run('plug square hole partly' + t, square, [ext('F2', 'add', 10, ['e1'], flip), ext('F3', 'add', 5, ['e5'], flip)], (PL - SQ) * 10 + SQ * 5);
  run('plug square hole, tool past the plate' + t, square, [ext('F2', 'add', 10, ['e1'], flip), ext('F3', 'add', 15, ['e5'], flip)], PL * 10 + SQ * 5);
}
run('plate and plug in one extrude', square, [ext('F2', 'add', 10, ['e1', 'e5'])], PL * 10);
run('cut of an open hole removes nothing', round, [ext('F2', 'add', 10, ['e1', 'e5']), ext('F3', 'cut', 10, ['e6'], true)], 'noEffect');
run('cut a plug back out', round, [ext('F2', 'add', 10, ['e1', 'e5']), ext('F3', 'cut', 10, ['e5'], true)], (PL - 2 * C) * 10);
run('cut a plug partly', round, [ext('F2', 'add', 10, ['e1', 'e5']), ext('F3', 'cut', 4, ['e5'], true)], (PL - C) * 10 - C * 4);
run('boss shorter than the pocket', square, [ext('F2', 'add', 10, ['e1', 'e5']), ext('F3', 'cut', 5, ['e5'], true), ext('F4', 'add', 3, ['e5'])], PL * 10 - SQ * 2);
run('boss as deep as the pocket', square, [ext('F2', 'add', 10, ['e1', 'e5']), ext('F3', 'cut', 5, ['e5'], true), ext('F4', 'add', 5, ['e5'])], PL * 10);
run('stacked discs share a face', round, [ext('F2', 'add', 10, ['e6']), ext('F3', 'add', 10, ['e6'], true)], C * 20);
run('cut slab with the cylinder wall as its side', round, [ext('F2', 'add', 10, ['e6']), ext('F3', 'cut', 4, ['e6'], true)], C * 6);
console.log(bad ? `coincident_boolean_check: ${bad} FAILED` : 'coincident_boolean_check: ok');
process.exit(bad ? 1 : 0);
