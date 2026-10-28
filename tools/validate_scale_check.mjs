// Time `validate` against face count on generated parts (issue #34).
//   node tools/validate_scale_check.mjs [--wasm path] [--sizes 500,1000,2000,4000] [--modes holes,pads,lumps] [--quick] [--max-s 600] [--budget-ms 0]
// holes: a plate with N holes (N+6 faces, one loop per hole on the two big faces);
// pads: a plate with N small square pads joined to it (5N+6 faces, one body); lumps: N/6 disjoint boxes (N faces, N/6 lumps).
// Prints one line per (mode, size): faces, regen ms, validate ms with and without the mesh, and the per-check `stats` of the reply.
// --budget-ms is opts.validateTimeMs (default 0 = unlimited, so the table shows the true cost; 60000 is the kernel's default).
// A size is skipped once the previous one of its mode took longer than --max-s / 4 (the growth is at least linear).
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const a = process.argv.slice(2);
const opt = (n, d) => { const i = a.indexOf(n); return i >= 0 ? a[i + 1] : d; };
const wasm = opt('--wasm', new URL('../target/wasm32-unknown-unknown/release/ck_wasm.wasm', import.meta.url));
const sizes = opt('--sizes', '500,1000,2000,4000').split(',').map(Number);
const modes = opt('--modes', 'holes,pads,lumps').split(',');
const maxS = +opt('--max-s', '600');
const quickOnly = a.includes('--quick');
const budget = +opt('--budget-ms', '0');

const sq = (id, pid, x, y, s, pts, ents) => {
  const c = [[x, y], [x + s, y], [x + s, y + s], [x, y + s]].map((p, i) => ({ id: `${pid}${i}`, x: p[0], y: p[1] }));
  pts.push(...c);
  for (let i = 0; i < 4; i++) ents.push({ id: `${id}${i}`, type: 'line', p1: c[i].id, p2: c[(i + 1) % 4].id });
};
export function makePart(mode, n) {
  const pitch = 6, count = mode === 'lumps' ? Math.max(1, Math.round(n / 6)) : mode === 'pads' ? Math.max(1, Math.round(n / 5)) : n;
  const cols = Math.ceil(Math.sqrt(count)), W = cols * pitch + 4, H = Math.ceil(count / cols) * pitch + 4;
  const at = (i) => [2 + (i % cols) * pitch, 2 + Math.floor(i / cols) * pitch];
  const pts = [], ents = [], fs = [];
  if (mode === 'lumps') {
    for (let i = 0; i < count; i++) sq(`k${i}_`, `q${i}_`, ...at(i), 3, pts, ents);
    fs.push({ type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' }, points: pts, entities: ents, constraints: [] },
      { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 5 });
  } else {
    sq('e', 'p', 0, 0, 1, pts, ents); pts.length = 0; ents.length = 0;
    pts.push({ id: 'p0', x: 0, y: 0 }, { id: 'p1', x: W, y: 0 }, { id: 'p2', x: W, y: H }, { id: 'p3', x: 0, y: H });
    for (let i = 0; i < 4; i++) ents.push({ id: 'e' + i, type: 'line', p1: 'p' + i, p2: 'p' + ((i + 1) % 4) });
    if (mode === 'holes') {
      for (let i = 0; i < count; i++) { const [x, y] = at(i); pts.push({ id: 'c' + i, x: x + 1.5, y: y + 1.5 }); ents.push({ id: 'k' + i, type: 'circle', c: 'c' + i, r: 1.2 }); }
    }
    fs.push({ type: 'sketch', id: 'F1', plane: { kind: 'std', id: 'front' }, points: pts, entities: ents, constraints: [] },
      { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 5 });
    if (mode === 'pads') {
      const p2 = [], e2 = [];
      for (let i = 0; i < count; i++) sq(`k${i}_`, `q${i}_`, ...at(i), 3, p2, e2);
      fs.push({ type: 'plane', id: 'P1', mode: 'offset', base: { kind: 'std', id: 'front' }, offset: 5 },
        { type: 'sketch', id: 'F3', plane: { kind: 'feature', id: 'P1' }, points: p2, entities: e2, constraints: [] },
        { type: 'extrude', id: 'F4', op: 'add', sketch: 'F3', end: 'blind', depth: 2 });
    }
  }
  return { format: 'ck-part', version: 1, name: mode + n, features: fs };
}

if (process.argv[1].endsWith('validate_scale_check.mjs')) {
  const k = new Kernel(new WebAssembly.Module(readFileSync(wasm)), { now: () => performance.now(), unixMs: () => Date.now() });
  const req = (msg) => { const t = performance.now(); const r = JSON.parse(k.request(JSON.stringify({ id: 1, msg })).json); return [r.result, performance.now() - t]; };
  console.log('mode faces lumps regen_ms validate_ms validate_nomesh_ms quick_ms | stats');
  for (const mode of modes) {
    let last = 0;
    for (const n of sizes) {
      if (last > maxS * 250) { console.log(mode, n, 'skipped (previous size took', Math.round(last / 1000), 's)'); continue; }
      const [rr, regen] = req({ type: 'regen', tag: mode + n, part: makePart(mode, n), opts: { solveSketches: false } });
      const faces = rr?.body?.faces?.length ?? -1, lumps = rr?.body?.lumps?.length ?? '?';
      if (rr?.errors?.length) console.log('  regen errors', JSON.stringify(rr.errors).slice(0, 300));
      const [qv, tq] = req({ type: 'validate', tag: mode + n, opts: { quick: true, validateTimeMs: budget } });
      const [nm, tn] = quickOnly ? [null, 0] : req({ type: 'validate', tag: mode + n, mesh: false, opts: { validateTimeMs: budget } });
      const [fv, tf] = quickOnly ? [null, 0] : req({ type: 'validate', tag: mode + n, opts: { validateTimeMs: budget } });
      last = tf || tn || tq;
      const iss = (r) => r ? (r.issues || []).length : '-';
      console.log(mode, faces, lumps, Math.round(regen), Math.round(tf), Math.round(tn), Math.round(tq), '| issues', iss(fv), iss(nm), iss(qv), '|', JSON.stringify((fv || qv)?.stats ?? null));
    }
  }
}
