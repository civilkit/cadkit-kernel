// Wire cases for the design-for-manufacture requests (dfmMinWall, dfmMinInternalRadius, dfmPocketRatio, dfmOverhang,
// dfmConstantSection) through the wasm kernel, on small parts with hand-computed answers (the same fixtures as
// crates/ck-wasm/tests/dfm.rs). These requests have no counterpart in the original kernel, so they live here and not in
// tools/cases.mjs (whose replies are compared byte-for-byte with the original).
//
// usage: node tools/dfm_check.mjs [--wasm path] [--quiet]     exit 1 on any failed case
// "top" plane: sketch (x, y) -> world (x, ., -y); `add` extrudes along +Y.
import { readFileSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const wi = args.indexOf('--wasm');
const wasm = wi >= 0 ? args[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const quiet = args.includes('--quiet');

const TOP = { kind: 'std', id: 'top' };
const poly = (id, plane, pts) => ({
  type: 'sketch', id, plane, constraints: [],
  points: pts.map(([x, y], i) => ({ id: `p${i + 1}`, x, y })),
  entities: pts.map((_, i) => ({ type: 'line', id: `e${i + 1}`, p1: `p${i + 1}`, p2: `p${(i + 1) % pts.length + 1}` })),
});
const ext = (id, sketch, op, depth) => ({ type: 'extrude', id, op, sketch, end: 'blind', depth });
const prism = (pts, depth) => [poly('F1', TOP, pts), ext('F2', 'F1', 'add', depth)];
const part = (features) => ({ format: 'ck-part', version: 1, features });
const rectPts = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];

const parts = {
  stepped: part(prism([[0, 0], [40, 0], [40, 5], [20, 5], [20, 10], [0, 10]], 10)),
  bracket: part(prism([[0, 0], [40, 0], [40, 3], [3, 3], [3, 30], [0, 30]], 10)),
  tee: part(prism([[10, 0], [30, 0], [30, 5], [40, 5], [40, 10], [0, 10], [0, 5], [10, 5]], 10)),
  pocketed: part([poly('F1', TOP, rectPts(0, 0, 40, 40)), ext('F2', 'F1', 'add', 20),
    poly('F3', { kind: 'face', id: 'F2.C1' }, rectPts(10, 15, 30, 25)), ext('F4', 'F3', 'cut', 12)]),
  shell: part([...prism(rectPts(0, 0, 40, 30), 20), { type: 'shell', id: 'F3', faces: ['F2.C1'], thickness: 0.8 }]),
  empty: part([]),
};

const near = (a, b, tol = 1e-3) => typeof a === 'number' && Math.abs(a - b) <= tol;
// [name, part, request, check(result) -> true | message]
const cases = [
  ['dfmMinWall fail', 'bracket', { type: 'dfmMinWall', threshold: 4 },
    (r) => (r.pass === false && near(r.minimum.thickness, 3) && r.violations.length > 0 && r.violations[0].faces.length === 2) || 'expected fail at 3'],
  ['dfmMinWall pass', 'bracket', { type: 'dfmMinWall', threshold: 3 }, (r) => r.pass === true || 'expected pass'],
  ['dfmMinWall thin shell', 'shell', { type: 'dfmMinWall', threshold: 1.2 },
    (r) => (r.pass === false && near(r.minimum.thickness, 0.8)) || 'expected fail at 0.8'],
  ['dfmMinInternalRadius sharp', 'bracket', { type: 'dfmMinInternalRadius', toolRadius: 1, direction: [0, 1, 0] },
    (r) => (r.pass === false && r.minimum.kind === 'sharp' && r.minimum.radius === 0 && near(r.minimum.point[0], 3) && near(r.minimum.point[2], -3)) || 'expected the inside corner'],
  ['dfmPocketRatio', 'pocketed', { type: 'dfmPocketRatio', maxRatio: 1 },
    (r) => (r.pass === false && r.pockets.length === 1 && near(r.pockets[0].ratio, 1.2) && r.pockets[0].floor === 'F4.C0') || 'expected one pocket, ratio 1.2'],
  ['dfmPocketRatio none', 'stepped', { type: 'dfmPocketRatio', maxRatio: 1 }, (r) => (r.pass === true && r.pockets.length === 0) || 'a step is not a pocket'],
  ['dfmOverhang', 'tee', { type: 'dfmOverhang', direction: [0, 0, -1], angle: 45 },
    (r) => (r.pass === false && near(r.overhangArea, 200) && near(r.supportVolume, 1000) && r.faces.length === 2) || 'expected two wing undersides'],
  ['dfmConstantSection yes', 'stepped', { type: 'dfmConstantSection', axis: [0, 1, 0] }, (r) => r.constant === true || 'expected constant'],
  ['dfmConstantSection station', 'stepped', { type: 'dfmConstantSection', axis: [0, 0, 1] },
    (r) => (r.constant === false && near(r.station.position, -5) && near(r.station.sectionAreaBefore, 200) && near(r.station.sectionAreaAfter, 400)) || 'expected the ledge at z=-5'],
  ['no body -> null', 'empty', { type: 'dfmMinWall', threshold: 1 }, (r) => r === null || 'expected null'],
];

const mod = new WebAssembly.Module(readFileSync(wasm));
const fails = [];
const send = (k, msg) => {
  const r = k.request(JSON.stringify({ id: 1, msg }));
  if (r.trap) return { trap: r.trap };
  return JSON.parse(r.json);
};
for (const [name, p, req, check] of cases) {
  const k = new Kernel(mod);
  const rg = send(k, { type: 'regen', tag: 't', part: parts[p] });
  if (!rg.ok || (rg.result.errors ?? []).length) { fails.push(`${name}: setup regen failed ${JSON.stringify(rg).slice(0, 200)}`); continue; }
  const a = send(k, { ...req, tag: 't' });
  if (a.trap || !a.ok) { fails.push(`${name}: ${a.trap ?? a.error}`); continue; }
  const msg = check(a.result);
  if (msg !== true) { fails.push(`${name}: ${msg} -- ${JSON.stringify(a.result).slice(0, 300)}`); continue; }
  const b = send(k, { ...req, tag: 't' });
  if (JSON.stringify(a.result) !== JSON.stringify(b.result)) fails.push(`${name}: reply is not deterministic`);
  if (!quiet) console.log(`ok   ${name}`);
}
for (const f of fails) console.log(`FAIL ${f}`);
console.log(`dfm_check: ${cases.length - fails.length}/${cases.length} cases`);
process.exit(fails.length ? 1 : 0);
