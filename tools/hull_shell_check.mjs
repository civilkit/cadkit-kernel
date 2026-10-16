// Open-top hollow hulls through the wasm: a loft through closed free-form spline stations, the deck
// cut off (face F92.C0), shell of that face. Wall volumes against an independent construction (inner
// offset of the closed hull, cut at the deck line, subtracted from the open hull), validity, and the
// refusal (with a hint) when the apex of a point-ended hull stays in the body.
// usage: node tools/hull_shell_check.mjs [--wasm path]     exit 1 on any mismatch
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

const BLUNT = [[-60, 4, 4, 3], [-30, 14, 9, 4], [0, 18, 12, 5], [30, 14, 9, 4], [60, 2, 3, 7]];
const BOAT = [[-60, 6, 5, 3], [-35, 15, 9, 4], [0, 18, 12, 5], [30, 13, 9, 5.5]];

function hull(stations, tip, thick) {
  const feats = []; const sk = [];
  stations.forEach(([x, w, d, h], i) => {
    feats.push({ type: 'plane', id: `P${i}`, mode: 'offset', base: { kind: 'std', id: 'right' }, offset: x });
    const P = [[0, h], [0.55 * w, h + 0.4], [w, h - 1], [0.97 * w, -0.3 * d], [0.6 * w, -0.8 * d], [0, -d], [-0.6 * w, -0.8 * d], [-0.97 * w, -0.3 * d], [-w, h - 1], [-0.55 * w, h + 0.4]];
    feats.push({ type: 'sketch', id: `S${i}`, plane: { kind: 'feature', id: `P${i}` }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, ...P.map(([u, v], j) => ({ id: 'p' + (j + 1), x: u, y: v }))],
      entities: [{ id: 'e1', type: 'spline', pts: P.map((_, j) => 'p' + (j + 1)), closed: true }], constraints: [] });
    sk.push(`S${i}`);
  });
  if (tip != null) {
    feats.push({ type: 'plane', id: 'PT', mode: 'offset', base: { kind: 'std', id: 'right' }, offset: tip });
    feats.push({ type: 'sketch', id: 'ST', plane: { kind: 'feature', id: 'PT' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 0, y: 2 }], entities: [{ id: 'e1', type: 'point', p: 'p1' }], constraints: [] });
    sk.push('ST');
  }
  feats.push({ type: 'loft', id: 'F50', op: 'add', sketches: sk });
  feats.push({ type: 'plane', id: 'F90', mode: 'offset', base: { kind: 'std', id: 'top' }, offset: 1.5 });
  feats.push({ type: 'sketch', id: 'F91', plane: { kind: 'feature', id: 'F90' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'a', x: -90, y: -40 }, { id: 'b', x: 90, y: -40 }, { id: 'c', x: 90, y: 40 }, { id: 'd', x: -90, y: 40 }],
    entities: [{ id: 'e1', type: 'line', p1: 'a', p2: 'b' }, { id: 'e2', type: 'line', p1: 'b', p2: 'c' }, { id: 'e3', type: 'line', p1: 'c', p2: 'd' }, { id: 'e4', type: 'line', p1: 'd', p2: 'a' }], constraints: [] });
  feats.push({ type: 'extrude', id: 'F92', op: 'cut', sketch: 'F91', end: 'blind', depth: 30, flip: true });
  if (thick) feats.push({ type: 'shell', id: 'F93', faces: ['F92.C0'], thickness: thick });
  return feats;
}

function regen(features) {
  const k = new Kernel(mod);
  const ask = (id, msg) => JSON.parse(k.request(JSON.stringify({ id, msg })).json);
  const t0 = Date.now();
  const r = ask(1, { type: 'regen', tag: 'main', part: { format: 'ck-part', version: 1, features }, opts: { solveSketches: true } });
  const ms = Date.now() - t0;
  const m = ask(2, { type: 'massProps', tag: 'main' });
  const v = ask(3, { type: 'validate', tag: 'main' });
  const errors = (r.result?.errors ?? []).map(([id, e]) => ({ id, code: e.code, params: e.params ?? e }));
  return { errors, ms, vol: m.ok ? m.result?.volume : null, valid: v.ok ? v.result?.ok : null, issues: v.result?.issues ?? [] };
}

// reference wall volumes (independent construction from the same sections)
for (const [t, want] of [[1.0, 4383.0], [0.6, 2683.5]]) {
  const r = regen(hull(BLUNT, null, t));
  check(`blunt hull shell ${t}`, r.errors.length === 0 && r.valid === true && r.vol != null && Math.abs(r.vol - want) < 0.04 * want,
    `errors ${JSON.stringify(r.errors).slice(0, 160)} vol ${r.vol?.toFixed(1)} want ${want} valid ${r.valid} ${r.ms} ms ${JSON.stringify(r.issues).slice(0, 160)}`);
}
{
  // a point-ended hull: the deck cut leaves the rim 1.25 mm short of the apex: refused with a hint
  const r = regen(hull(BOAT, 62, 1.0));
  const d = JSON.stringify(r.errors);
  check('pointy hull (tip cut short of the apex) refused with a hint', r.errors.length === 1 && r.errors[0].code === 'offsetUnsupported' && /apex/.test(d), d.slice(0, 200));
}
{
  // the apex stays in the body: refused with a hint
  const f = hull(BOAT, 62, 0).filter((x) => !['F90', 'F91', 'F92'].includes(x.id));
  f.push({ type: 'shell', id: 'F93', faces: ['F50.C0'], thickness: 1.0 });
  const r = regen(f);
  const d = JSON.stringify(r.errors);
  check('pointy hull with its apex refused with a hint', r.errors.length === 1 && r.errors[0].code === 'offsetUnsupported' && /apex/.test(d), d.slice(0, 200));
}
if (bad) { console.log(`${bad} failed`); process.exit(1); }
console.log('all ok');
