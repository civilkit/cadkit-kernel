// Arc unit check without the DeepCAD dataset: a circular-segment loop
// (one chord + one arc) extruded by 1. Tests {c,p1,p2} configurations:
// CCW/CW, <180, =180, >180, tiny, and the centre on the opposite side.
import { fileURLToPath } from 'node:url';
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';

const wasm = process.argv.includes('--wasm')
  ? process.argv[process.argv.indexOf('--wasm') + 1]
  : fileURLToPath(new URL('../../kit/apps/cad/public/kernel/ck_wasm.wasm', import.meta.url));
const mod = new WebAssembly.Module(readFileSync(wasm));
const k = new Kernel(mod);
const PI = Math.PI;

function pt(c, r, a) { return { x: c[0] + r * Math.cos(a), y: c[1] + r * Math.sin(a) }; }
function partFor(c, r, a0, a1) {
  const A = pt(c, r, a0), B = pt(c, r, a1);
  return {
    format: 'ck-part', version: 1,
    features: [
      { type: 'sketch', id: 'S1', plane: { kind: 'std', id: 'front' },
        points: [{ id: 'c', x: c[0], y: c[1] }, { id: 'a', x: A.x, y: A.y }, { id: 'b', x: B.x, y: B.y }],
        entities: [
          { type: 'line', id: 'e1', p1: 'b', p2: 'a' },
          { type: 'arc', id: 'e2', c: 'c', p1: 'a', p2: 'b' },
        ], constraints: [] },
      { type: 'extrude', id: 'F1', op: 'add', sketch: 'S1', end: 'blind', depth: 1 },
    ],
  };
}

const cases = [];
for (const [name, sweep] of [['tiny', 1e-3], ['90', PI / 2], ['180', PI], ['270', 3 * PI / 2], ['342', 1.9 * PI]]) {
  for (const cw of [false, true]) {
    for (const cy of [0, 1]) {
      // CCW from a0 to a0+sweep; CW uses the same point set by swapping p1/p2.
      const a0 = 0, a1 = sweep;
      const area = cw
        ? 0.5 * ((2 * PI - sweep) - Math.sin(2 * PI - sweep))
        : 0.5 * (sweep - Math.sin(sweep));
      const c = [0, cy];
      const part = partFor(c, 1, a0, a1);
      if (cw) { const e = part.features[0].entities[1]; [e.p1, e.p2] = [e.p2, e.p1]; }
      cases.push({ name: `${name}${cw ? '-cw' : '-ccw'}-cy${cy}`, part, want: area });
    }
  }
}
const rows = [];
for (const c of cases) {
  let cls = 'ok', detail = '';
  try {
    const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'm', part: c.part } })).json);
    if (r.result?.errors?.length) { cls = 'feature-error'; detail = JSON.stringify(r.result.errors).slice(0, 120); }
    else {
      const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'm', bodies: true } })).json);
      const v = m.result?.volume;
      if (v == null) { cls = 'no-body'; }
      else {
        const rel = Math.abs(v - c.want) / Math.max(Math.abs(c.want), 1e-12);
        detail = `vol ${v.toExponential(6)} want ${c.want.toExponential(6)} (${rel.toExponential(1)})`;
        if (rel > 1e-4) cls = 'wrong';
      }
    }
  } catch (e) { cls = 'trap'; detail = String(e.message ?? e); }
  rows.push({ name: c.name, cls, detail });
}
const by = {};
for (const r of rows) by[r.cls] = (by[r.cls] ?? 0) + 1;
console.log(JSON.stringify({ n: rows.length, ...by }));
for (const r of rows) console.log(`${r.cls === 'ok' ? 'ok  ' : 'FAIL'} ${r.name}: ${r.detail}`);
