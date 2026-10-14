// Extrude / cut end references through the wasm (upToFace, offsetFromFace, upToPoint;
// docs/spec-agent-interface.md, "End references").
// usage: node tools/end_reference_check.mjs [--wasm path] [--time]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));
const showTime = process.argv.includes('--time');

let bad = 0;
const check = (name, ok, detail = '') => {
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `: ${detail}` : ''}`);
};
const near = (a, b, tol) => Math.abs(a - b) <= tol;

const TOP = { kind: 'std', id: 'top' }; // the xz plane, sketch y runs along -z, extrudes along +y
const rect = (id, plane, x0, y0, x1, y1) => ({
  type: 'sketch', id, plane, points: [{ id: 'p1', x: x0, y: y0 }, { id: 'p2', x: x1, y: y0 }, { id: 'p3', x: x1, y: y1 }, { id: 'p4', x: x0, y: y1 }],
  entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })), constraints: [],
});
const block = (h) => [rect('F1', TOP, 0, 0, 40, 40), { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: h }];
const boss = (end) => [...block(20), rect('F3', TOP, 35, 10, 45, 20), { type: 'extrude', id: 'F4', op: 'add', sketch: 'F3', ...end }];

function regen(features) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features };
  const t0 = performance.now();
  const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const ms = performance.now() - t0;
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  return { res: r.result, errs: (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`), warns: r.result?.warnings ?? [], vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, ms };
}
const base = regen(block(20));
const dv = (r) => r.vol - base.vol;

{
  const top = 'F2.C1';
  const r = regen(boss({ end: 'offsetFromFace', endRef: { kind: 'face', id: top }, offset: 5 }));
  check('offsetFromFace +5', r.errs.length === 0 && r.valid && near(dv(r), 1500, 1e-3) && r.warns.length === 0, `${dv(r)} ${r.errs}`);
  const n = regen(boss({ end: 'offsetFromFace', endRef: top, offset: -5 }));
  check('offsetFromFace -5 (bare id)', n.errs.length === 0 && near(dv(n), 750, 1e-3), `${dv(n)}`);
  const u = regen(boss({ end: 'upToFace', endRef: top, depth: 3 }));
  const w = u.warns[0]?.[1];
  check('upToFace not covering falls back with a warning', u.errs.length === 0 && w?.code === 'endConditionFallback' && w.params.reason === 'upToFaceOutside' && near(dv(u), 150, 1e-3), JSON.stringify(u.warns));
  const s = regen(boss({ end: 'upToFace', endRef: top, depth: 3, strictEnd: true }));
  check('strictEnd gives the oracle error', s.errs.join() === 'F4:upToFaceOutside', s.errs.join());
}
{
  const r = regen(boss({ end: 'upToPoint', endRef: [0, 12, 0] }));
  check('upToPoint array', r.errs.length === 0 && r.valid && near(dv(r), 600, 1e-3), `${dv(r)} ${r.errs}`);
  const v = regen(boss({ end: 'upToPoint', endRef: { kind: 'vertex', faces: ['F2.C1', 'F2.S.e4', 'F2.S.e1'], at: [0, 20, 0] } }));
  check('upToPoint vertex by faces', v.errs.length === 0 && near(dv(v), 1000, 1e-3), `${dv(v)} ${v.errs}`);
  const b = regen(boss({ end: 'upToPoint', endRef: [0, -5, 0], depth: 3 }));
  check('upToPoint behind falls back', b.warns[0]?.[1]?.params?.reason === 'upToPointBehind' && near(dv(b), 150, 1e-3), JSON.stringify(b.warns));
}
{
  const m = regen(boss({ end: 'upToFace' }));
  check('missing endRef', m.errs.join() === 'F4:missingReference', m.errs.join());
  const m2 = regen(boss({ end: 'upToFace', endRef: 'F9.C9' }));
  check('unknown face', m2.errs.join() === 'F4:missingReference', m2.errs.join());
}
{
  // persistent names: the end follows the face when the block is edited
  const part = (h) => [...block(h), rect('F3', TOP, 35, 10, 45, 20), { type: 'extrude', id: 'F4', op: 'add', sketch: 'F3', end: 'offsetFromFace', endRef: 'F2.C1', offset: 4 }];
  const a = regen(part(20)), b = regen(part(30));
  check('end follows an edited target', near(a.vol - regen(block(20)).vol, 1400, 1e-3) && near(b.vol - regen(block(30)).vol, 5 * 10 * 34 + 200, 1e-3), `${a.vol} ${b.vol}`);
}
{
  // slanted target (wedge), cut up to its sloped top
  const front = { kind: 'std', id: 'front' };
  const wedge = [
    { type: 'sketch', id: 'F1', plane: front, points: [{ id: 'a', x: 0, y: 0 }, { id: 'b', x: 40, y: 0 }, { id: 'c', x: 40, y: 10 }, { id: 'd', x: 0, y: 20 }],
      entities: [['a', 'b'], ['b', 'c'], ['c', 'd'], ['d', 'a']].map(([p, q], i) => ({ type: 'line', id: `e${i + 1}`, p1: p, p2: q })), constraints: [] },
    { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 40 },
  ];
  const w0 = regen(wedge);
  const slope = 'F2.S.e3';
  const r = regen([...wedge, rect('F3', TOP, 5, -10, 10, -5), { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', flip: true, end: 'upToFace', endRef: slope }]);
  const expect = 25 * (20 - 7.5 / 4);
  check('slanted upToFace', r.errs.length === 0 && r.valid && near(w0.vol - r.vol, expect, 1e-3) && r.warns.length === 0, `${w0.vol - r.vol} vs ${expect} ${r.errs}`);
  if (showTime) console.log(`slanted upToFace ${r.ms.toFixed(0)} ms (wedge alone ${w0.ms.toFixed(0)} ms)`);
}
process.exit(bad ? 1 : 0);
