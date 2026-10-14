// Booleans on free-form walls through the wasm: a spline pocket / boss / ring cut on a closed-spline (kidney)
// extrusion, and a sphere centred on a block face (added and cut) keeping its hemisphere face.
// usage: node tools/spline_boolean_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};
const near = (a, b, tol) => Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));

function regen(features, measure = []) {
  const k = new Kernel(mod);
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  const r = ask(1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features } });
  const m = ask(2, { type: 'massProps', tag: 'main' });
  const v = ask(3, { type: 'validate', tag: 'main' });
  const errors = (r.result?.errors ?? []).map(([, e]) => e.code);
  const areas = measure.map((id) => {
    const q = ask(4, { type: 'measure', tag: 'main', refs: [{ kind: 'face', id }] });
    return q.ok ? q.result?.items?.[0]?.area ?? null : null;
  });
  return { errors, vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, areas };
}

// ---- spline pocket / boss / ring on a spline wall
const KIDNEY = [[-30, 0], [-20, 15], [0, 9], [20, 15], [30, 0], [15, -15], [-15, -15]];
const INNER = KIDNEY.map(([x, y]) => [0.9 * x, 0.9 * y]);
const spl = (id, plane, ...loops) => ({
  type: 'sketch', id, plane,
  points: loops.flatMap((l, k) => l.map(([x, y], i) => ({ id: `p${k}_${i}`, x, y }))),
  entities: loops.map((l, k) => ({ type: 'spline', id: `e${k}`, pts: l.map((_, i) => `p${k}_${i}`), closed: true })),
  constraints: [],
});
const FRONT = { kind: 'std', id: 'front' };
const TOP = { kind: 'face', id: 'F2.C1' };
const base = [spl('F1', FRONT, KIDNEY), { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 4 }];
const b0 = regen(base);
const solo = regen([spl('F1', FRONT, INNER), { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 1 }]);
check('kidney and inner outline extrude', b0.errors.length === 0 && solo.errors.length === 0 && b0.vol > 0 && solo.vol > 0);
const A0 = b0.vol / 4;
const Ai = solo.vol;
const cases = [
  ['ring cut 1 mm', spl('F3', TOP, KIDNEY, INNER), 'cut', 1, b0.vol - (A0 - Ai)],
  ['inner pocket 2 mm', spl('F3', TOP, INNER), 'cut', 2, b0.vol - Ai * 2],
  ['inner boss 2 mm', spl('F3', TOP, INNER), 'add', 2, b0.vol + Ai * 2],
];
for (const [name, sk, op, depth, want] of cases) {
  const r = regen([...base, sk, { type: 'extrude', id: 'F4', op, sketch: 'F3', end: 'blind', depth }]);
  check(name, r.errors.length === 0 && r.valid === true && near(r.vol, want, 5e-3), `${r.errors} valid ${r.valid} vol ${r.vol} want ${want}`);
}

// ---- sphere centred on a block face
const R = 6;
const block = [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [[-20, -20], [20, -20], [20, 20], [-20, 20]].map(([x, y], i) => ({ id: `b${i}`, x, y })),
    entities: [0, 1, 2, 3].map((i) => ({ type: 'line', id: `l${i}`, p1: `b${i}`, p2: `b${(i + 1) % 4}` })), constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 40 },
];
const sphere = (plane, u, v, alongX, op) => {
  const [p1, p2] = alongX ? [[u + R, v], [u - R, v]] : [[u, v - R], [u, v + R]];
  return [
    { type: 'sketch', id: 'F3', plane: { kind: 'std', id: plane },
      points: [{ id: 'c', x: u, y: v }, { id: 'p1', x: p1[0], y: p1[1] }, { id: 'p2', x: p2[0], y: p2[1] }],
      entities: [{ type: 'arc', id: 'arc', c: 'c', p1: 'p1', p2: 'p2' }, { type: 'line', id: 'cl', p1: 'p2', p2: 'p1' },
        { type: 'line', id: 'ax', p1: 'p1', p2: 'p2', construction: true }], constraints: [] },
    { type: 'revolve', id: 'F4', op, sketch: 'F3', axis: 'ax', angle: 360 },
  ];
};
const V0 = 40 ** 3;
const dv = (2 / 3) * Math.PI * R ** 3;
const hemi = 2 * Math.PI * R * R;
for (const [plane, u, v, label] of [['front', 0, 40, 'top'], ['front', 0, 0, 'bottom'], ['front', 20, 20, '+x'], ['right', -20, 20, '-z']]) {
  for (const alongX of [false, true]) {
    for (const op of ['add', 'cut']) {
      const r = regen([...block, ...sphere(plane, u, v, alongX, op)], ['F4.S.arc']);
      const want = op === 'add' ? V0 + dv : V0 - dv;
      check(`sphere ${op} on ${label} face, axis ${alongX ? 'x' : 'y'}`, r.errors.length === 0 && r.valid === true && near(r.vol, want, 1e-3) && r.areas[0] !== null && near(r.areas[0], hemi, 2e-3),
        `${r.errors} valid ${r.valid} vol ${r.vol} want ${want} area ${r.areas[0]} want ${hemi}`);
    }
  }
}
console.log(bad ? `spline_boolean_check: ${bad} FAILED` : 'spline_boolean_check: ok');
process.exit(bad ? 1 : 0);
