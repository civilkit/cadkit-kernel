// Loft with a point (apex) end section through the wasm (docs/spec-agent-interface.md, loft "Point sections").
// A sketch with exactly ONE point entity is a point section (first and/or last).
// usage: node tools/loft_point_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));
const PI = Math.PI;

let n = 0;
const id = (p = 'F') => p + ++n;
// a station plane at `x` along the hull axis (right plane normal) and a sketch on it
function station(feats, x, sketch) {
  const pl = { type: 'plane', id: id(), mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x };
  const sk = { type: 'sketch', id: id(), plane: { kind: 'feature', id: pl.id }, constraints: [], ...sketch };
  feats.push(pl, sk);
  return sk.id;
}
const circle = (r) => ({ points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r }] });
const square = (a) => ({
  points: [[-a, -a], [a, -a], [a, a], [-a, a]].map(([x, y], i) => ({ id: 'p' + i, x, y })),
  entities: [0, 1, 2, 3].map((i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + i, p2: 'p' + ((i + 1) % 4) })),
});
const point = (x = 0, y = 0) => ({ points: [{ id: 'p', x, y }], entities: [{ type: 'point', id: 'e1', p: 'p' }] });
function hullSection(w, d, h) {
  const P = [[0, h], [0.55 * w, h + 0.4], [w, h - 1], [0.97 * w, -0.3 * d], [0.6 * w, -0.8 * d], [0, -d], [-0.6 * w, -0.8 * d], [-0.97 * w, -0.3 * d], [-w, h - 1], [-0.55 * w, h + 0.4]];
  return { points: P.map(([x, y], i) => ({ id: 'p' + (i + 1), x, y })), entities: [{ id: 'e1', type: 'spline', pts: P.map((_, i) => 'p' + (i + 1)), closed: true }] };
}

let bad = 0;
function run(name, build, want) {
  n = 0;
  const feats = [];
  build(feats);
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: feats };
  if (process.env.LOFT_POINT_DUMP && name.includes(process.env.LOFT_POINT_DUMP)) writeFileSync(path.join(process.env.LOFT_POINT_OUT ?? '.', 'dump.json'), JSON.stringify(part));
  const t0 = Date.now();
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, opts: { solveSketches: true } } })).json);
  const ms = Date.now() - t0;
  const errs = (r.result?.errors ?? []).map(([i, e]) => `${i}:${e.code}`);
  const call = (msg, tag = 'main') => JSON.parse(k.request(JSON.stringify({ id: 2, msg: { ...msg, tag } })).json);
  const m = call({ type: 'massProps' });
  const v = call({ type: 'validate' });
  const vol = m.ok ? m.result?.volume : null;
  const issues = v.result?.issues ?? [];
  let ok = true, shown = '';
  if (want.error) {
    ok = errs.length > 0 && errs.some((e) => e.includes(want.error));
    shown = errs.join(' ') || `no error (vol ${vol})`;
  } else if (want.cleanOrError) {
    // valid solid, or a structured error: never ok:true with an invalid body
    ok = errs.length > 0 ? errs.every((e) => e.includes('loft')) : issues.length === 0 && vol > 0;
    shown = errs.length ? errs.join(' ') : `vol ${vol?.toFixed(4)} issues ${issues.length} ${issues.slice(0, 2).map((i) => i.code).join(',')}`;
  } else {
    ok = errs.length === 0 && issues.length === 0 && vol > 0;
    if (want.volume != null) ok = ok && Math.abs(vol - want.volume) <= (want.tol ?? 1e-6) * want.volume;
    if (want.faces != null) ok = ok && r.result?.body?.faces?.length === want.faces;
    shown = errs.length ? errs.join(' ') : `vol ${vol?.toFixed(4)} faces ${r.result?.body?.faces?.length} issues ${issues.length}${want.volume != null ? ` (want ${want.volume.toFixed(4)})` : ''}`;
    if (ok && want.step) {
      const st = call({ type: 'step', name: 'rt' });
      const imp = call({ type: 'importStep', name: 'rt', data: st.result }, 'imp');
      const mi = call({ type: 'massProps' }, 'imp');
      const vi = call({ type: 'validate' }, 'imp');
      const rt = Math.abs(mi.result?.volume - vol) / vol;
      const iok = imp.result?.body && rt <= 1e-6 && (vi.result?.issues ?? []).length === 0;
      if (!iok) { ok = false; shown += ` STEP roundtrip: ${JSON.stringify(imp.result?.errors ?? '')} rel ${rt} ${JSON.stringify(vi.result?.issues?.slice(0, 2))}`; }
      else shown += ' step-roundtrip ok';
    }
  }
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${shown} [${ms} ms]`);
}

const loft = (feats, ids, o = {}) => { const f = { type: 'loft', id: id(), op: 'add', sketches: ids, ...o }; feats.push(f); return f; };
const h = 30;
run('circle r8 -> point = cone', (f) => loft(f, [station(f, 0, circle(8)), station(f, h, point())]), { volume: PI * 64 * h / 3, faces: 3, step: true });
run('point -> circle r8 = cone (reversed)', (f) => loft(f, [station(f, 0, point()), station(f, h, circle(8))]), { volume: PI * 64 * h / 3, step: true });
run('square 10 -> point = pyramid', (f) => loft(f, [station(f, 0, square(5)), station(f, h, point())]), { volume: 100 * h / 3, faces: 5, step: true });
run('oblique cone (apex off axis)', (f) => loft(f, [station(f, 0, circle(8)), station(f, h, point(5, 3))]), { volume: PI * 64 * h / 3, step: true });
run('point -> circle -> point', (f) => loft(f, [station(f, 0, point()), station(f, 10, circle(6)), station(f, 20, point())]), {});
const hull = (w1, w2, extra = {}) => (f) => loft(f, [
  station(f, -60, w1 ? hullSection(...w1) : point(0, 3)), station(f, -30, hullSection(14, 9, 4)), station(f, 0, hullSection(18, 12, 5)),
  station(f, 30, hullSection(14, 9, 4)), station(f, 60, w2 ? hullSection(...w2) : point(0, 5)),
], extra);
run('hull: spline stations, point at both ends', hull(null, null), { step: true });
run('hull: point bow, spline stern', hull(null, [2, 3, 7]), { step: true });
run('hull: 0.3 mm stern station (near degenerate): valid or a clean error', hull([4, 4, 3], [0.0003, 0.0003, 0.0003]), { cleanOrError: true });
run('point in the middle is refused', (f) => loft(f, [station(f, 0, circle(5)), station(f, 10, point()), station(f, 20, circle(5))]), { error: 'loftFailed' });
run('guide with a point end is refused', (f) => {
  const g = id();
  f.push({ type: 'sketch', id: g, plane: { kind: 'std', id: 'top' }, points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: 30, y: 4 }], entities: [{ type: 'line', id: 'e1', p1: 'a', p2: 'b' }], constraints: [] });
  loft(f, [station(f, 0, circle(8)), station(f, h, point())], { guides: [g] });
}, { error: 'loftFailed' });
// a pocket cut away from the tip
run('cone with a pocket away from the apex', (f) => {
  loft(f, [station(f, 0, circle(8)), station(f, h, point())]);
  const pl = { type: 'plane', id: id(), mode: 'offset', base: { kind: 'std', id: 'right' }, offset: 6 };
  const sk = { type: 'sketch', id: id(), plane: { kind: 'feature', id: pl.id }, points: [{ id: 'c', x: 0, y: 0 }], entities: [{ type: 'circle', id: 'e1', c: 'c', r: 2 }], constraints: [] };
  f.push(pl, sk, { type: 'extrude', id: id(), op: 'cut', sketch: sk.id, end: 'blind', depth: 5 });
}, {});
process.exit(bad ? 1 : 0);
