// Patterns and mirrors carry the threads of their seeds, through the wasm
// (docs/spec-agent-interface.md, "Threads" -> "Patterns and mirrors").
// Plate 80x80x10 with one M5 `hole` feature and a `thread` on its wall; a circular pattern of 6, a linear
// pattern and a mirror must give one thread entry per instance, flagged `instanceOf`, on the instance face;
// modeled threads remove N grooves; `carryThreads:false` and an unreachable instance wall.
// usage: node tools/thread_pattern_check.mjs [--wasm path]     exit 1 on any mismatch
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
const near = (a, b, tol) => Math.abs(a - b) <= tol;

const PLATE = [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [[-40, -40], [40, -40], [40, 40], [-40, 40]].map(([x, y], i) => ({ id: 'p' + i, x, y })),
    entities: [0, 1, 2, 3].map((i) => ({ type: 'line', id: 'e' + (i + 1), p1: 'p' + i, p2: 'p' + ((i + 1) % 4) })), constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 },
  { type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p0', x: 20, y: 0 }],
    entities: [{ type: 'point', id: 'e2', p: 'p0' }], constraints: [] },
  { type: 'hole', id: 'F4', sketch: 'F3', standard: 'iso', size: 'M5', fit: 'tap', end: 'through' },
];
// the same plate with a plain circle cut (a `hole` feature has no mirror recipe yet)
const PLATE_CUT = [PLATE[0], PLATE[1],
  { type: 'sketch', id: 'F3', plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'c', x: 20, y: 0 }], entities: [{ type: 'circle', id: 'e2', c: 'c', r: 2.1 }], constraints: [] },
  { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth', depth: 0 }];
const THREAD = (o = {}) => ({ type: 'thread', id: 'F5', face: 'F4.S.h1', standard: 'metric', size: 'M5', ...o });
const CPAT = (o = {}) => ({ type: 'cpattern', id: 'F7', seeds: ['F4'], axis: { kind: 'axis', axis: 'y' }, count: 6, angle: 360, equal: true, ...o });
const LPAT = (o = {}) => ({ type: 'lpattern', id: 'F7', seeds: ['F4'], dir: { kind: 'axis', axis: 'x' }, spacing: 8, count: 3, ...o });
const MIR = (o = {}) => ({ type: 'mirror', id: 'F7', seeds: ['F4'], plane: { kind: 'std', id: 'right' }, ...o });

function regen(features) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features };
  const t0 = performance.now();
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const ms = performance.now() - t0;
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  return { res: r.result, errs: (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`), threads: r.result?.threads ?? [],
    warns: (r.result?.warnings ?? []).map(([id, e]) => `${id}:${e.code}`), vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, ms };
}
const ids = (r) => r.threads.map((t) => t.id).join(',');

{
  const plain = regen([...PLATE, CPAT()]);
  const r = regen([...PLATE, THREAD(), CPAT()]);
  check('6x circular: six entries', r.errs.length === 0 && r.threads.length === 6 && ids(r) === 'F5,F5@F7.1,F5@F7.2,F5@F7.3,F5@F7.4,F5@F7.5', `${r.errs.join()} ${ids(r)}`);
  check('6x circular: cosmetic = no geometry change', r.vol === plain.vol);
  const dirs = r.threads.every((t) => near(Math.abs(t.dir[1]), 1, 1e-9));
  const radii = r.threads.every((t) => near(Math.hypot(t.start[0], t.start[2]), 20, 1e-5));
  // consecutive instances are 60 degrees apart
  const ang = r.threads.map((t) => Math.atan2(t.start[2], t.start[0]) * 180 / Math.PI);
  const steps = ang.slice(1).map((a, i) => ((((a - ang[i]) % 360) + 540) % 360) - 180);
  check('6x circular: axes along the pattern axis, 20 mm out, 60 degrees apart', dirs && radii && steps.every((s) => near(Math.abs(s), 60, 1e-4)) && steps.every((s) => Math.sign(s) === Math.sign(steps[0])), JSON.stringify(steps));
  const t3 = r.threads[3];
  check('instance entry: face, instanceOf, same spec', t3.face === 'F4.S.h1@F7.3' && t3.instanceOf === 'F5' && t3.size === 'M5' && t3.class === '6H' && t3.hand === 'right' && t3.internal === true && near(t3.length, 10, 1e-6), JSON.stringify(t3));
}
{
  const r = regen([...PLATE, THREAD(), CPAT({ carryThreads: false })]);
  check('carryThreads:false', r.errs.length === 0 && ids(r) === 'F5', ids(r));
  const e = regen([...PLATE, THREAD(), CPAT({ seeds: ['F4', 'F5'], carryThreads: false })]);
  check('explicit thread seed beats carryThreads:false', e.errs.length === 0 && e.threads.length === 6, ids(e));
}
{
  const r = regen([...PLATE, THREAD({ hand: 'left', length: 6 }), LPAT()]);
  check('linear pattern', r.errs.length === 0 && ids(r) === 'F5,F5@F7.1,F5@F7.2' && r.threads.every((t) => t.hand === 'left' && near(t.length, 6, 1e-6)) && near(Math.abs(r.threads[2].start[0] - r.threads[0].start[0]), 16, 1e-5), `${r.errs.join()} ${ids(r)}`);
  const m = regen([...PLATE_CUT, THREAD({ face: 'F4.S.e2' }), MIR()]);
  check('mirror', m.errs.length === 0 && ids(m) === 'F5,F5@F7.1' && near(m.threads[1].start[0], -m.threads[0].start[0], 1e-5) && m.threads[1].face === 'F4.S.e2@F7.1' && m.threads[1].hand === 'right', `${m.errs.join()} ${ids(m)}`);
}
{
  // modeled: 4 grooves remove 4x the volume of one
  const one = regen([...PLATE, THREAD({ modeled: true })]);
  const base = regen(PLATE);
  const plain4 = regen([...PLATE, CPAT({ count: 4 })]);
  const r4 = regen([...PLATE, THREAD({ modeled: true }), CPAT({ count: 4 })]);
  const g1 = base.vol - one.vol, g4 = plain4.vol - r4.vol;
  console.log(`time modeled 4x: ${r4.ms.toFixed(0)} ms (one ${one.ms.toFixed(0)} ms)`);
  check('modeled 4x: four modeled entries, valid solid', r4.errs.length === 0 && r4.threads.length === 4 && r4.threads.every((t) => t.modeled === true) && r4.valid === true, `${r4.errs.join()} ${r4.warns.join()}`);
  check('modeled 4x: removed volume = 4 x one groove (1e-3)', g1 > 1 && near(g4, 4 * g1, 1e-3 * 4 * g1), `${g4.toFixed(4)} vs 4 x ${g1.toFixed(4)}`);
}
{
  // an instance whose hole falls inside a pocket has no wall to carry the thread
  const pocket = [
    { type: 'sketch', id: 'F6', plane: { kind: 'face', id: 'F2.C1' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'c', x: 20, y: 0 }],
      entities: [{ type: 'circle', id: 'e2', c: 'c', r: 10 }], constraints: [] },
    { type: 'extrude', id: 'F8', op: 'cut', sketch: 'F6', end: 'throughBoth', depth: 0 },
  ];
  // pocket on the source hole itself would remove the thread: put the pocket where instance 3 of 6 (180 degrees) lands
  const r = regen([...PLATE, THREAD(), ...pocket.map((f) => (f.type === 'sketch' ? { ...f, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'c', x: -20, y: 0 }] } : f)), CPAT()]);
  check('unreachable instance wall -> threadNotCarried warning, others carried', r.errs.length === 0 && r.warns.includes('F7:threadNotCarried') && r.threads.length === 5, `${r.errs.join()} ${r.warns.join()} ${ids(r)}`);
}

process.exit(bad ? 1 : 0);
