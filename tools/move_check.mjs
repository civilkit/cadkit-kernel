// moveFace / moveBody through the wasm (docs/spec-agent-interface.md, "Direct manipulation").
// usage: node tools/move_check.mjs [--wasm path]     exit 1 on any mismatch
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
const near = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));

const BLOCK = [
  { type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'top' }, points: [{ id: 'p1', x: 0, y: 0 }, { id: 'p2', x: 40, y: 0 }, { id: 'p3', x: 40, y: 20 }, { id: 'p4', x: 0, y: 20 }],
    entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })), constraints: [] },
  { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 },
];
const HOLE = [...BLOCK,
  { type: 'sketch', id: 'F3', plane: { kind: 'std', id: 'top' }, points: [{ id: 'c', x: 20, y: 10 }], entities: [{ type: 'circle', id: 'c1', c: 'c', r: 3 }], constraints: [] },
  { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'throughBoth', depth: 0 }];

function regen(features) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features };
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  const r = ask(1, { type: 'regen', tag: 'main', part });
  const m = ask(2, { type: 'massProps', tag: 'main' });
  const v = ask(3, { type: 'validate', tag: 'main' });
  const faces = (r.result?.body?.faces ?? []).map((f) => f.id).sort();
  return { res: r.result, errs: (r.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`), vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, faces };
}
const mf = (o) => ({ type: 'moveFace', id: 'M1', ...o });
const mb = (o) => ({ type: 'moveBody', id: 'M1', ...o });

const base = regen(BLOCK);
{
  const r = regen([...BLOCK, mf({ faces: ['F2.C1'], distance: 3 })]);
  check('moveFace offset', r.errs.length === 0 && near(r.vol, 40 * 20 * 13, 1e-6) && r.valid === true && r.faces.join() === base.faces.join(), `${r.errs} vol ${r.vol}`);
}
{
  const r = regen([...BLOCK, mf({ faces: ['F2.S.e2'], mode: 'translate', translate: [4, 9, 9] })]);
  check('moveFace translate', r.errs.length === 0 && r.valid === true && Math.abs(r.vol - 8000) > 1, `${r.errs} vol ${r.vol}`);
}
{
  const r = regen([...HOLE, mf({ faces: ['F4.S.c1'], distance: 1 })]);
  const want = 40 * 20 * 10 - Math.PI * 4 * 10;
  check('moveFace hole wall shrinks the hole', r.errs.length === 0 && near(r.vol, want, 1e-4) && r.valid === true, `${r.errs} vol ${r.vol} want ${want}`);
}
{
  const r = regen([...BLOCK, mf({ faces: ['F2.C1'], distance: -12 })]);
  check('moveFace collapse is structured', r.errs.length === 1 && r.errs[0].startsWith('M1:moveFaceFailed') && near(r.vol, 8000), r.errs.join());
  const r2 = regen([...BLOCK, mf({ faces: [] })]);
  check('moveFace noFaces', r2.errs.join() === 'M1:noFaces', r2.errs.join());
  const r3 = regen([...BLOCK, mf({ faces: ['nope'], distance: 1 })]);
  check('moveFace missingReference', r3.errs.length === 1 && r3.errs[0].startsWith('M1:missingReference'), r3.errs.join());
}
{
  const r = regen([...BLOCK, mb({ translate: [5, 0, 0], rotate: { axis: [0, 0, 1], origin: [0, 0, 0], angle: 90 } })]);
  check('moveBody translate + rotate', r.errs.length === 0 && near(r.vol, 8000) && r.valid === true && r.faces.join() === base.faces.join(), `${r.errs} vol ${r.vol}`);
  const c = regen([...BLOCK, mb({ copy: true, translate: [60, 0, 0] })]);
  check('moveBody copy', c.errs.length === 0 && near(c.vol, 16000) && c.faces.some((f) => f.endsWith('@M1.1')), `${c.errs} vol ${c.vol}`);
  const e = regen([mb({ translate: [1, 0, 0] })]);
  check('moveBody without a body', e.errs.join() === 'M1:nothingToCut', e.errs.join());
}
console.log(bad ? `move_check: ${bad} FAILED` : 'move_check: ok');
process.exit(bad ? 1 : 0);
