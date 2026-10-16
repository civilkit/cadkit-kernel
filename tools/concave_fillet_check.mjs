// Fillets at the corners of a pocket through the wasm: three equal concave fillets end in a sphere patch,
// two mitre without a flat cap, the full floor + corner ring gets four patches (volumes from OCCT
// BRepFilletAPI_MakeFillet on the same 40x20x40 block with a 20x8x20 pocket, base volume 28800).
// usage: node tools/concave_fillet_check.mjs [--wasm path]     exit 1 on any mismatch
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const wi = process.argv.indexOf('--wasm');
const wasm = wi > 0 ? process.argv[wi + 1] : path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const mod = new WebAssembly.Module(readFileSync(wasm));

const rect = (id, plane, x0, y0, x1, y1) => ({
  type: 'sketch', id, plane, points: [{ id: 'p1', x: x0, y: y0 }, { id: 'p2', x: x1, y: y0 }, { id: 'p3', x: x1, y: y1 }, { id: 'p4', x: x0, y: y1 }],
  entities: [['p1', 'p2'], ['p2', 'p3'], ['p3', 'p4'], ['p4', 'p1']].map(([a, b], i) => ({ type: 'line', id: `e${i + 1}`, p1: a, p2: b })), constraints: [],
});
const pocket = [
  rect('F1', { kind: 'std', id: 'top' }, 0, 0, 40, 40), { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 20 },
  rect('F3', { kind: 'face', id: 'F2.C1' }, 10, 10, 30, 30), { type: 'extrude', id: 'F4', op: 'cut', sketch: 'F3', end: 'blind', depth: 8 },
];
const E = (a, b, at) => ({ faces: [a, b], at });
const corner = [E('F4.C0', 'F4.S.e4', [10, 12, -20]), E('F4.C0', 'F4.S.e1', [20, 12, -10]), E('F4.S.e1', 'F4.S.e4', [10, 16, -10])];
const ring = [
  E('F4.C0', 'F4.S.e4', [10, 12, -20]), E('F4.C0', 'F4.S.e1', [20, 12, -10]), E('F4.C0', 'F4.S.e2', [30, 12, -20]), E('F4.C0', 'F4.S.e3', [20, 12, -30]),
  E('F4.S.e1', 'F4.S.e4', [10, 16, -10]), E('F4.S.e1', 'F4.S.e2', [30, 16, -10]), E('F4.S.e2', 'F4.S.e3', [30, 16, -30]), E('F4.S.e3', 'F4.S.e4', [10, 16, -30]),
];

let bad = 0;
function run(name, r, edges, want, spheres, tol = 1e-6) {
  const k = new Kernel(mod);
  const part = { format: 'ck-part', version: 1, features: [...pocket, { type: 'fillet', id: 'F5', radius: r, edges }] };
  const res = JSON.parse(k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } })).json);
  const errs = (res.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`);
  const m = JSON.parse(k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } })).json);
  const v = JSON.parse(k.request(JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } })).json);
  const vol = m.ok ? m.result?.volume : null;
  const faces = res.result?.body?.faces ?? [];
  const nSphere = faces.filter((f) => f.id.startsWith('F5.') && f.kind === 'sphere').length;
  const nCap = faces.filter((f) => f.id.startsWith('F5.') && f.kind === 'plane').length;
  const ok = errs.length === 0 && v.ok && v.result?.ok && vol != null && Math.abs(vol - want) <= tol * want && nSphere === spheres && nCap === 0;
  if (!ok) bad++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: ${errs.join() || vol?.toFixed(6)} (want ${want.toFixed(6)}) spheres ${nSphere}/${spheres} caps ${nCap} valid ${v.result?.ok}`);
}

run('3 concave edges r=1', 1, corner, 28810.133483871516, 1);
run('3 concave edges r=2', 2, corner, 28839.86431834444, 1);
run('3 concave edges r=3', 3, corner, 28888.188077706345, 1);
run('2 concave edges r=1 (mitre)', 1, corner.slice(0, 2), 28808.488203128036, 0);
run('2 concave edges r=2 (mitre)', 2, corner.slice(0, 2), 28833.56933115844, 0);
run('2 concave edges r=3 (mitre)', 3, corner.slice(0, 2), 28874.668162060, 0);
run('floor ring + 4 corners r=1', 1, ring, 28823.365788557858, 4);
run('floor ring + 4 corners r=2', 2, ring, 28890.78468566495, 4);
run('floor ring + 4 corners r=3', 3, ring, 28998.23898847153, 4);
process.exit(bad ? 1 : 0);
