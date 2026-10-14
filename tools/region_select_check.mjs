// Region selection through the wasm (`regions` on extrude / revolve, docs/spec-agent-interface.md).
// Two features from ONE sketch: extrude add of circle A, then extrude cut of circle B, etc.
// usage: node tools/region_select_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

// concentric circles: ring (e5, r8, hole r3) and disc (e6, r3); the cut disc goes through the ring's hole
const sketch = { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [{ id: 'c', x: 0, y: 0 }],
  entities: [{ type: 'circle', id: 'e5', c: 'c', r: 8 }, { type: 'circle', id: 'e6', c: 'c', r: 3 }], constraints: [] };
const ext = (id, op, depth, o = {}) => ({ type: 'extrude', id, op, sketch: 'F1', end: 'blind', depth, ...o });
const ring = Math.PI * 55, disc = Math.PI * 9;

let bad = 0;
function run(name, features, want) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: [sketch, ...features] };
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const errs = (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}${e.params ? JSON.stringify(e.params) : ''}`);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const vol = m.ok ? m.result?.volume ?? null : null;
  let ok;
  if (typeof want === 'string') ok = errs.length === 1 && errs[0].startsWith(want);
  else ok = errs.length === 0 && vol != null && Math.abs(vol - want) <= 1e-3 * want;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${errs.length ? errs.join(' ') : vol?.toFixed(3)}${typeof want === 'number' ? ` (want ${want.toFixed(3)})` : ''}`);
}

run('all regions (no field)', [ext('F2', 'add', 10)], ring * 10);
run('ring only', [ext('F2', 'add', 10, { regions: ['e5'] })], ring * 10);
run('disc only', [ext('F2', 'add', 10, { regions: ['e6'] })], disc * 10);
run('add ring, add disc taller', [ext('F2', 'add', 10, { regions: ['e5'] }), ext('F3', 'add', 20, { regions: ['e6'] })], ring * 10 + disc * 20);
run('add ring + disc, cut disc (flip)', [ext('F2', 'add', 10, { regions: ['e5'] }), ext('F3', 'add', 20, { regions: ['e6'] }),
  ext('F4', 'cut', 15, { regions: ['e6'], flip: true })], ring * 10 + disc * 5);
run('through-all cut of the disc', [ext('F2', 'add', 10, { regions: ['e5'] }), ext('F3', 'add', 20, { regions: ['e6'] }),
  ext('F4', 'cut', 0, { regions: ['e6'], end: 'throughBoth' })], ring * 10);
run('unknown id', [ext('F2', 'add', 10, { regions: ['nope'] })], 'F2:extrudeRegionUnknown');
run('empty selection', [ext('F2', 'add', 10, { regions: [] })], 'F2:extrudeRegionNone');
// the regen cache keys on the document: a changed selection must not reuse the old result
{
  const k = new Kernel(mod);
  const vol = (regions) => {
    const part = { format: 'ck-part', version: 1, features: [sketch, ext('F2', 'add', 10, regions ? { regions } : {})] };
    k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
    return JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json).result.volume;
  };
  const a = vol(['e5']), b = vol(['e6']), c = vol(['e5']);
  const ok = Math.abs(a - ring * 10) < 1 && Math.abs(b - disc * 10) < 1 && Math.abs(c - a) < 1e-9;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} cache: ${a.toFixed(2)} ${b.toFixed(2)} ${c.toFixed(2)}`);
}
console.log(bad ? `region_select_check: ${bad} FAILED` : 'region_select_check: ok');
process.exit(bad ? 1 : 0);
