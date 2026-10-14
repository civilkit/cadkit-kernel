// Custom counterbore / countersink dimensions of `hole` through the wasm (docs/spec-agent-interface.md, "Hole").
// Box 40x20x10 with one M8 normal hole (d 9); removed volume checked against the analytic cylinder / frustum.
// usage: node tools/hole_dims_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

const sk1 = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' },
  points: [[0, 0], [40, 0], [40, 20], [0, 20]].map(([x, y], i) => ({ id: 'p' + i, x, y })),
  entities: [0, 1, 2, 3].map((i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + i, p2: 'p' + ((i + 1) % 4) })), constraints: [] };
const ext = { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 };
const sk3 = { type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p0', x: 10, y: 10 }],
  entities: [{ type: 'point', id: 'e2', p: 'p0' }], constraints: [] };
const hole = (o) => ({ type: 'hole', id: 'F4', sketch: 'F3', standard: 'iso', size: 'M8', fit: 'normal', end: 'through', ...o });

const R = 4.5, BOX = 8000, P = Math.PI;
const frustum = (h, a, b) => P * h / 3 * (a * a + a * b + b * b);
let bad = 0;
function run(name, o, want) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: [sk1, ext, sk3, hole(o)] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}${e.params?.hint ? ' (hint)' : ''}`);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const vol = m.ok ? m.result?.volume ?? null : null;
  let ok;
  if (typeof want === 'string') ok = errs.length === 1 && errs[0].startsWith(want) && errs[0].endsWith('(hint)');
  else ok = errs.length === 0 && vol != null && Math.abs((BOX - vol) - want) <= 1e-3 * want;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${errs.length ? errs.join(' ') : (BOX - vol).toFixed(3)}${typeof want === 'number' ? ` (want ${want.toFixed(3)})` : ''}`);
}
run('cbore table', { kind: 'cbore' }, P * (7.5 * 7.5 * 8.6 + R * R * 1.4));
run('cbore custom', { kind: 'cbore', cboreDiameter: 14, cboreDepth: 4 }, P * (49 * 4 + R * R * 6));
run('cbore diameter only', { kind: 'cbore', cboreDiameter: 14 }, P * (49 * 8.6 + R * R * 1.4));
run('csink legacy (plain)', { kind: 'csink' }, P * R * R * 10);
run('csink + cskDiameter', { kind: 'csink', cskDiameter: 16 }, frustum(3.5, 8, R) + P * R * R * 6.5);
const h82 = 3.5 / Math.tan(41 * P / 180);
run('csk 82 degrees', { kind: 'csk', cskDiameter: 16, cskAngle: 82 }, frustum(h82, 8, R) + P * R * R * (10 - h82));
run('countersink table', { kind: 'countersink' }, frustum(3.7, 8.2, R) + P * R * R * 6.3);
run('blind countersink + tip', { kind: 'countersink', cskDiameter: 16, end: 'blind', depth: 6, tip: true },
  frustum(3.5, 8, R) + P * R * R * 2.5 + P * R * R * (R / Math.tan(59 * P / 180)) / 3);
run('cbore diameter <= hole', { kind: 'cbore', cboreDiameter: 9 }, 'F4:holeCboreInvalid');
run('cbore deeper than blind hole', { kind: 'cbore', end: 'blind', depth: 5, cboreDepth: 5 }, 'F4:holeCboreInvalid');
run('csk head <= hole', { kind: 'csink', cskDiameter: 9 }, 'F4:holeCskDiameter');
run('csk cone deeper than blind hole', { kind: 'countersink', cskDiameter: 16, end: 'blind', depth: 3 }, 'F4:holeCskDiameter');
console.log(bad ? `hole_dims_check: ${bad} FAILED` : 'hole_dims_check: ok');
process.exit(bad ? 1 : 0);
