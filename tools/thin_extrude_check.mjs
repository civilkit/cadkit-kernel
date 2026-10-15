// Thin-wall extrude / revolve through the wasm (`thin` option, docs/spec-agent-interface.md "Thin wall").
// usage: node tools/thin_extrude_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

const arc = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: [{ id: 'c', x: 0, y: 0 }, { id: 'p1', x: 10, y: 0 }, { id: 'p2', x: -10, y: 0 }],
  entities: [{ type: 'arc', id: 'e1', c: 'c', p1: 'p1', p2: 'p2' }], constraints: [] };
const circle = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' }, points: [{ id: 'c', x: 0, y: 0 }],
  entities: [{ type: 'circle', id: 'e1', c: 'c', r: 10 }], constraints: [] };
const cpts = [[20, 0], [14, 14], [0, 20], [-14, 14], [-20, 0], [-14, -14]];
const spline = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: cpts.map(([x, y], i) => ({ id: `p${i}`, x, y })),
  entities: [{ type: 'spline', id: 'e1', pts: cpts.map((_, i) => `p${i}`), closed: false }], constraints: [] };
const tube = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' },
  points: [{ id: 'a', x: 10, y: 0 }, { id: 'b', x: 10, y: 20 }, { id: 'o', x: 0, y: 0 }, { id: 'q', x: 0, y: 20 }],
  entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b' }, { type: 'line', id: 'ax', p1: 'o', p2: 'q', construction: true }], constraints: [] };
const ext = (thin, o = {}) => ({ type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 5, thin, ...o });
const rev = (thin, o = {}) => ({ type: 'revolve', id: 'F2', op: 'add', sketch: 'F1', axis: 'ax', angle: 360, thin, ...o });
const PI = Math.PI;

let bad = 0;
function run(name, sk, feature, want, tol = 1e-5) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: [sk, feature] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, params: e.params ?? {} }));
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const vol = m.ok ? m.result?.volume ?? null : null;
  let ok, shown;
  if (typeof want === 'string') {
    ok = errs.length === 1 && errs[0].code === want && (want !== 'thinTooThick' || (errs[0].params.minRadius > 0 && errs[0].params.maxThickness > 0));
    shown = errs.map((e) => `${e.code}${JSON.stringify(e.params)}`).join(' ');
  } else if (want === null) {
    ok = errs.length === 0 && vol > 0;
    shown = errs.length ? errs.map((e) => e.code).join(' ') : vol?.toFixed(3);
  } else {
    ok = errs.length === 0 && vol != null && Math.abs(vol - want) <= tol * want;
    shown = errs.length ? errs.map((e) => e.code).join(' ') : vol?.toFixed(3);
  }
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${shown}${typeof want === 'number' ? ` (want ${want.toFixed(3)})` : ''}`);
  return vol;
}

const ring = (ro, ri) => (PI / 2) * (ro * ro - ri * ri) * 5;
run('half circle, inside 2', arc, ext({ thickness: 2, side: 'inside' }), ring(10, 8));
run('half circle, outside 2', arc, ext({ thickness: 2, side: 'outside' }), ring(12, 10));
run('half circle, mid 2', arc, ext({ thickness: 2, side: 'mid' }), ring(11, 9));
run('half circle, both 2 + 1', arc, ext({ thickness: 2, thickness2: 1, side: 'both' }), ring(11, 8));
run('half circle, mid 2, round ends', arc, ext({ thickness: 2, side: 'mid', ends: 'round' }), ring(11, 9) + PI * 5);
run('closed circle hollow wall', circle, ext({ thickness: 2 }), PI * (100 - 64) * 5);
run('thickness as an expression', arc, ext({ thickness: '=1+1', side: 'inside' }), ring(10, 8));
const vi = run('spline arc, inside 3', spline, ext({ thickness: 3, side: 'inside' }), null);
const vo = run('spline arc, outside 3', spline, ext({ thickness: 3, side: 'outside' }), null);
const vm = run('spline arc, mid 3', spline, ext({ thickness: 3, side: 'mid' }), null);
{
  const ok = vi > 0 && Math.abs(vm - 0.5 * (vi + vo)) < 5e-3 * vm;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} spline arc: mid band is the mean of inside and outside (${vm?.toFixed(2)} vs ${(0.5 * (vi + vo)).toFixed(2)})`);
}
run('too thick: half circle inside 15', arc, ext({ thickness: 15, side: 'inside' }), 'thinTooThick');
run('bad side', arc, ext({ thickness: 2, side: 'sideways' }), 'invalidValue');
run('thin cut needs a body', arc, ext({ thickness: 2 }, { op: 'cut' }), 'nothingToCut');
run('thin revolve tube', tube, rev({ thickness: 2, side: 'mid' }), PI * 40 * 20);
console.log(bad ? `thin_extrude_check: ${bad} FAILED` : 'thin_extrude_check: ok');
process.exit(bad ? 1 : 0);
