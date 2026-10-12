// BenchCAD adapter scorer: regen each translated part on the rebuilt wasm and
// compare volume / area / centre of mass / bbox with the CadQuery ground truth
// (tools/bench/benchcad/manifest.json), run `validate`, and, when the ground
// truth STEP exists, run it through `importStep` and compare that too.
//
// usage: node tools/bench/run.mjs [--wasm path] [--only id] [-v]
//   exit code 0 always; the gate reads runBench() (metric bench.benchcad).
//
// Frame: the dataset is Z-up, we are Y-up; ground truth is rotated
// (x,y,z) -> (x,z,-y), which maps the CQ XY/XZ/YZ workplanes onto our
// top/front/right planes (see gen.mjs). Tolerances: volume/area 1e-3
// relative; com and bbox 1e-3 of the ground-truth bbox diagonal (our bbox is
// mesh-derived, so curved extents are slightly short).
//
// Failure labels (manifest `known[id]`, set by a human after investigating):
//   translation-mistake | kernel-gap:<what> | kernel-bug:<what> | ground-truth-mismatch
// A failing part with no recorded label prints UNLABELLED.
import { readFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const dir = path.join(here, 'benchcad');
const TOL = 1e-3;

const rot = (v) => [v[0], v[2], -v[1]];
const rotBox = (b) => { const lo = [b[0], b[1], b[2]], hi = [b[3], b[4], b[5]]; const a = rot(lo), c = rot(hi); return { min: a.map((x, i) => Math.min(x, c[i])), max: a.map((x, i) => Math.max(x, c[i])) }; };
const rel = (x, y) => Math.abs(x - y) / Math.max(Math.abs(y), 1e-12);

function compare(got, want, diag) {
  const bad = [];
  if (!(rel(got.volume, want.volume) <= TOL)) bad.push(`volume ${got.volume?.toPrecision(6)} vs ${want.volume.toPrecision(6)} (${(rel(got.volume, want.volume) * 100).toFixed(2)}%)`);
  if (!(rel(got.area, want.area) <= TOL)) bad.push(`area ${got.area?.toPrecision(6)} vs ${want.area.toPrecision(6)} (${(rel(got.area, want.area) * 100).toFixed(2)}%)`);
  const dc = Math.max(...want.com.map((c, i) => Math.abs(got.com[i] - c))) / diag;
  if (!(dc <= TOL)) bad.push(`com ${got.com.map((x) => +x.toFixed(3))} vs ${want.com.map((x) => +x.toFixed(3))}`);
  const db = Math.max(...[0, 1, 2].flatMap((i) => [Math.abs(got.box.min[i] - want.box.min[i]), Math.abs(got.box.max[i] - want.box.max[i])])) / diag;
  if (!(db <= TOL)) bad.push(`bbox ${JSON.stringify(got.box.min.map((x) => +x.toFixed(3)))}..${JSON.stringify(got.box.max.map((x) => +x.toFixed(3)))} vs ${JSON.stringify(want.box.min.map((x) => +x.toFixed(3)))}..${JSON.stringify(want.box.max.map((x) => +x.toFixed(3)))}`);
  return bad;
}

export function runBench({ wasm, only = null, log = () => {} } = {}) {
  const B = new WebAssembly.Module(readFileSync(wasm ?? path.join(here, '../../target/wasm32-unknown-unknown/release/ck_wasm.wasm')));
  const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
  const rows = [];
  for (const m of manifest.parts) {
    if (only && m.id !== only) continue;
    const row = { id: m.id, family: m.family, tier: m.tier, bad: [], import: null, importBad: [], notes: [] };
    rows.push(row);
    // known[id].skip: the translation runs far past any sane time budget (a kernel hang / perf cliff),
    // so the part is reported as a failure without being regenerated (rerun with --force-skipped).
    if (manifest.known?.[m.id]?.skip && !process.argv.includes('--force-skipped')) { row.bad.push(`skipped (not run): ${manifest.known[m.id].skip}`); row.skipped = true; }
    const gt = m.ground_truth;
    const want = { volume: gt.volume, area: gt.area, com: rot(gt.com), box: rotBox(gt.bbox) };
    const diag = Math.hypot(gt.bbox[3] - gt.bbox[0], gt.bbox[4] - gt.bbox[1], gt.bbox[5] - gt.bbox[2]);
    const k = new Kernel(B);
    const q = (id, msg) => { const r = k.request(JSON.stringify({ id, msg })); return r.trap ? { trap: r.trap } : JSON.parse(r.json); };
    regen: {
    if (row.skipped) break regen;
    const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
    const reg = q(1, { type: 'regen', tag: 'main', part });
    if (reg.trap) { row.bad.push(`TRAP ${reg.trap.slice(0, 120)}`); row.errors = 'trap'; break regen; }
    const errs = reg.result?.errors ?? [];
    if (errs.length) { row.errors = errs.map(([id, e]) => `${id}:${e.code}`).join(','); row.bad.push(`feature errors ${row.errors}`); }
    const body = reg.result?.body;
    if (!body) { row.bad.push('no body'); break regen; }
    const mp = q(2, { type: 'massProps', tag: 'main' }).result;
    const got = { volume: mp.volume, area: mp.area, com: mp.com, box: body.box };
    row.got = got; row.want = want;
    if (!errs.length) row.bad.push(...compare(got, want, diag));
    else row.bad.push(...compare(got, want, diag).map((s) => `(after errors) ${s}`));
    const v = q(3, { type: 'validate', tag: 'main' }).result;
    row.validate = v?.issues?.length ?? -1;
    if (row.validate !== 0) row.bad.push(`validate ${row.validate} issue(s): ${(v?.issues ?? []).slice(0, 3).map((i) => `${i.code} ${i.entity}`).join('; ')}`);
    }
    // ground-truth STEP through the importer (a second check of ck-io)
    const sp = path.join(dir, 'gt', `${m.id}.step`);
    if (existsSync(sp) && manifest.known?.[m.id]?.skipStep && !process.argv.includes('--force-skipped')) { row.import = false; row.importBad.push(`skipped (not run): ${manifest.known[m.id].skipStep}`); }
    else if (existsSync(sp)) {
      const imp = q(4, { type: 'importStep', tag: 'imp', name: m.id, data: readFileSync(sp, 'utf8') });
      const ib = imp.result?.body;
      if (!ib) { row.import = false; row.importBad.push(`import: ${JSON.stringify(imp.result?.errors ?? imp.error ?? imp.trap).slice(0, 160)}`); }
      else {
        const im = q(5, { type: 'massProps', tag: 'imp' }).result;
        const iv = q(6, { type: 'validate', tag: 'imp' }).result;
        const w2 = { volume: gt.volume, area: gt.area, com: gt.com, box: { min: gt.bbox.slice(0, 3), max: gt.bbox.slice(3) } };
        row.importBad.push(...compare({ volume: im.volume, area: im.area, com: im.com, box: ib.box }, w2, diag));
        if (iv?.issues?.length) row.importBad.push(`validate ${iv.issues.length} issue(s): ${iv.issues.slice(0, 2).map((i) => i.code).join(',')}`);
        row.import = row.importBad.length === 0;
      }
    }
  }
  for (const r of rows) {
    r.pass = r.bad.length === 0;
    r.known = manifest.known?.[r.id] ?? null;
    r.knownStep = manifest.knownStep?.[r.id] ?? null;
  }
  return rows;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2);
  const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
  const wasm = opt('--wasm'), only = opt('--only'), verbose = args.includes('-v');
  const rows = runBench({ wasm, only });
  const pad = (s, n) => String(s).padEnd(n);
  console.log(`${pad('id', 22)} ${pad('tier', 6)} ${pad('regen', 6)} ${pad('step-in', 8)} cause`);
  for (const r of rows) {
    const imp = r.import === null ? '-' : r.import ? 'PASS' : 'FAIL';
    console.log(`${pad(r.id, 22)} ${pad(r.tier, 6)} ${pad(r.pass ? 'PASS' : 'FAIL', 6)} ${pad(imp, 8)} ${r.pass ? '' : (r.known ? `${r.known.label}: ${r.known.note}` : 'UNLABELLED')}`);
    if (r.pass && r.known) console.log(`    note: recorded as ${r.known.label} but now passes - update manifest`);
    if (!r.pass || verbose) for (const b of r.bad) console.log(`    - ${b}`);
    if (r.import === false || verbose) for (const b of r.importBad) console.log(`    - step-in: ${b}`);
    if (r.import === false) console.log(`    step-in cause: ${r.knownStep ? `${r.knownStep.label}: ${r.knownStep.note}` : r.known && !r.pass ? '(see regen cause)' : 'UNLABELLED'}`);
  }
  console.log(`bench.benchcad ${rows.filter((r) => r.pass).length}/${rows.length} regen, ${rows.filter((r) => r.import === true).length}/${rows.filter((r) => r.import !== null).length} step import`);
}
