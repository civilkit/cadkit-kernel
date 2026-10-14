// Edit robustness: a small parameter edit must not break a model that the
// ORIGINAL kernel still builds (docs/edit-robustness.md).
//
// For each part (tools/cases/*.ckpart, tools/bench/benchcad/*.ckpart.json,
// optionally the large sample) a series of single-parameter edits is applied:
// a sample of features, the first numeric leaves of each (preferring
// depth/offset/angle/radius/distance/count/height/width/x/y, as `tweak` in
// regen_cache_check.mjs), each multiplied by 0.8, 1.25 and 1.5. Both kernels
// regen the edited document. A case counts when the ORIGINAL regens with no
// feature errors; it FAILS when ours then has a feature error / trap, or a
// volume more than --vol-tol (relative) away from the original's.
//
// usage: node tools/edit_robustness_check.mjs [--wasm ours.wasm] [--orig kernel.wasm]
//          [--only substr] [--features N] [--leaves N] [--factors 0.8,1.25,1.5]
//          [--vol-tol 1e-3] [--hair] [--jobs N] [--timeout s] [--quick] [--json out.json] [-v]
//   --features N  edited features per part (spread over the part; default 5)
//   --leaves N    numeric leaves per feature (default 1)
//   --hair        also the 44-feature large sample (features 18..43 sampled, minutes)
//   --quick       the gate subset: 12 small parts, 2 features, factors 1.25 only
//   --jobs N      parts in parallel worker processes (default 4)
//   --timeout s   per-regen wall limit in a worker (default 120); a hang is a failure of ours
// exit 1 when any case fails on ours. Wherever the original also errors the case is skipped
// (counted as "orig-error"), so the table lists only regressions of ours against the original.
import { readFileSync, readdirSync, writeFileSync, existsSync } from 'node:fs';
import { spawn } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f, d = null) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const worker = opt('--worker');
const workerFrom = +opt('--from', 0);
const quick = flag('--quick'), hair = flag('--hair'), verbose = flag('-v');
const wasmPath = opt('--wasm', path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm'));
const origPath = opt('--orig', path.join(here, '../../kernel.wasm'));
const only = opt('--only');
const nFeatures = +opt('--features', quick ? 2 : 5);
const nLeaves = +opt('--leaves', 1);
const factors = opt('--factors', quick ? '1.25' : '0.8,1.25,1.5').split(',').map(Number);
const volTol = +opt('--vol-tol', 1e-3);
const jobs = +opt('--jobs', 4);
const timeoutS = +opt('--timeout', 120);
const jsonOut = opt('--json');

// ------------------------------------------------------------ documents
const QUICK = new Set(['washer_000001', 'round_flange_000341', 'hex_nut_000006', 'bolt_000008', 'i_beam_000007', 'enclosure_000042', 'u_channel_000058',
  'sheet_metal_tray_000130', 'dome_cap_000212', 'shaft_collar_000553', 'snap_clip_000052', 'piston_002697']);
function documents() {
  const out = [];
  const cdir = path.join(here, 'cases');
  for (const f of readdirSync(cdir).filter((f) => f.endsWith('.ckpart')).sort()) {
    const name = f.replace('.ckpart', '');
    if (name === 'largeSampleFull' && !hair) continue;
    if (quick) continue;
    out.push({ name, file: path.join(cdir, f), heavy: name === 'largeSampleFull' });
  }
  const bdir = path.join(here, 'bench/benchcad');
  for (const f of readdirSync(bdir).filter((f) => f.endsWith('.ckpart.json')).sort()) {
    const name = f.replace('.ckpart.json', '');
    if (quick && !QUICK.has(name)) continue;
    out.push({ name, file: path.join(bdir, f), heavy: false });
  }
  return out.filter((d) => !only || d.name.includes(only));
}

// ------------------------------------------------------------ edits
const clone = (x) => JSON.parse(JSON.stringify(x));
const SKIP_KEYS = new Set(['id', 'num', 'seq', 'order', 'version']);
const PREFER = ['depth', 'offset', 'angle', 'r', 'radius', 'distance', 'count', 'height', 'width', 'thickness', 'x', 'y'];
function numLeaves(o, p = [], out = []) {
  if (typeof o === 'number') out.push({ path: p, v: o });
  else if (Array.isArray(o)) o.forEach((v, i) => numLeaves(v, [...p, i], out));
  else if (o && typeof o === 'object') for (const [k, v] of Object.entries(o)) if (!SKIP_KEYS.has(k)) numLeaves(v, [...p, k], out);
  return out;
}
const rank = (p) => { const i = PREFER.indexOf(String(p[p.length - 1])); return i < 0 ? 99 : i; };
/** the leaves of a feature worth editing: non-zero numbers, preferred keys first (stable) */
function pickLeaves(f, n) {
  const ls = numLeaves(f).filter((l) => l.v !== 0 && Number.isFinite(l.v));
  return ls.map((l, i) => ({ ...l, i })).sort((a, b) => rank(a.path) - rank(b.path) || a.i - b.i).slice(0, n);
}
/** evenly spread sample of k indices out of the candidates (always the first and last) */
function spread(c, k) {
  if (c.length <= k) return c;
  const out = [];
  for (let j = 0; j < k; j++) out.push(c[Math.round((j * (c.length - 1)) / (k - 1 || 1))]);
  return [...new Set(out)];
}
let skippedInvalid = 0;
function cases(doc, part) {
  let cand = part.features.map((f, i) => i).filter((i) => pickLeaves(part.features[i], 1).length);
  if (doc.heavy) cand = cand.filter((i) => i >= 18); // the early large sample features are cheap sketches / lofts
  const out = [];
  for (const i of spread(cand, nFeatures)) {
    const before = sketchProblems(part.features[i]);
    for (const l of pickLeaves(part.features[i], nLeaves)) {
      for (const k of factors) {
        const c = { at: i, leaf: l.path, base: l.v, factor: k };
        if (sketchProblems(apply(part, c).features[i]) > before) { skippedInvalid++; continue; }
        out.push(c);
      }
    }
  }
  return out;
}
function apply(part, c) {
  const d = clone(part);
  let o = d.features[c.at];
  for (const k of c.leaf.slice(0, -1)) o = o[k];
  o[c.leaf[c.leaf.length - 1]] = c.base * c.factor;
  return d;
}

/** Sketch problems an edit can introduce that make the INPUT nonsense (not a kernel robustness question):
 *  an arc whose endpoints are no longer equidistant from its centre, and crossing line segments in a
 *  loop. Counted before and after the edit; an edit that adds problems is skipped. */
function sketchProblems(f) {
  if (f.type !== 'sketch' || !Array.isArray(f.points)) return 0;
  const P = new Map(f.points.map((p) => [p.id, p]));
  let n = 0;
  const segs = [];
  for (const e of f.entities ?? []) {
    if (e.type === 'arc') {
      const [c, a, b] = [P.get(e.c), P.get(e.p1), P.get(e.p2)];
      if (c && a && b) {
        const r1 = Math.hypot(a.x - c.x, a.y - c.y), r2 = Math.hypot(b.x - c.x, b.y - c.y);
        if (Math.abs(r1 - r2) > 1e-4 * Math.max(r1, r2, 1e-9)) n++;
      }
    } else if (e.type === 'line' && !e.construction) {
      const [a, b] = [P.get(e.p1), P.get(e.p2)];
      if (a && b) segs.push([a, b, e.p1, e.p2]);
    }
  }
  const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  for (let i = 0; i < segs.length; i++) {
    for (let j = i + 1; j < segs.length; j++) {
      const [a, b, ia, ib] = segs[i], [c, d, ic, id] = segs[j];
      if (ia === ic || ia === id || ib === ic || ib === id) continue; // share an end point
      if (cross(a, b, c) * cross(a, b, d) < 0 && cross(c, d, a) * cross(c, d, b) < 0) n++;
    }
  }
  return n;
}

// ------------------------------------------------------------ kernels
function run(k, part) {
  const t = performance.now();
  const r = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part } }));
  if (r.trap) return { trap: r.trap, ms: performance.now() - t };
  const j = JSON.parse(r.json);
  const errors = (j.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}${e.params?.detail ? '(' + String(e.params.detail).slice(0, 60) + ')' : ''}`);
  let vol = null;
  const m = k.request(JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } }));
  if (!m.trap) vol = JSON.parse(m.json)?.result?.volume ?? null;
  return { errors, vol, ms: performance.now() - t };
}

// ------------------------------------------------------------ worker: one part, NDJSON on stdout
function workerMain(name) {
  const doc = documents().find((d) => d.name === name);
  const part = JSON.parse(readFileSync(doc.file, 'utf8'));
  const ours = new Kernel(new WebAssembly.Module(readFileSync(wasmPath)));
  const orig = new Kernel(new WebAssembly.Module(readFileSync(origPath)));
  const list = cases(doc, part);
  const emit = (o) => process.stdout.write(JSON.stringify(o) + '\n');
  emit({ part: name, total: list.length, skippedInvalid });
  const base = { o: run(orig, part), u: run(ours, part) };
  const bvol = base.o.vol && base.u.vol ? Math.abs(base.u.vol - base.o.vol) / Math.abs(base.o.vol) : null;
  emit({ part: name, baseline: { orig: base.o.errors?.length ?? 'trap', ours: base.u.errors?.length ?? 'trap', volDiff: bvol } });
  list.forEach((c, n) => {
    if (n < workerFrom) return;
    emit({ part: name, n, start: true });
    const ed = apply(part, c);
    const o = run(orig, ed);
    let u = null;
    if (!o.trap && !o.errors.length) u = run(ours, ed);
    const feat = part.features[c.at];
    emit({ part: name, n, at: c.at, id: feat.id, type: feat.type, leaf: c.leaf.join('.'), from: c.base, to: c.base * c.factor, factor: c.factor, orig: o, ours: u });
  });
  emit({ part: name, done: true });
}
if (worker) { workerMain(worker); process.exit(0); }

// ------------------------------------------------------------ driver
const docs = documents();
if (!existsSync(origPath)) { console.error('original kernel not found: ' + origPath); process.exit(2); }
const results = [];
const baselines = {};
let skippedTotal = 0;
const known = existsSync(path.join(here, 'edit_robustness_known.json')) ? JSON.parse(readFileSync(path.join(here, 'edit_robustness_known.json'), 'utf8')) : {};
const keyOf = (r) => `${r.part}#${r.id}.${r.leaf}x${r.factor}`;
function runPart(doc) {
  return new Promise((resolve) => {
    let from = 0, total = null;
    const launch = () => {
      const child = spawn(process.execPath, [fileURLToPath(import.meta.url), '--worker', doc.name, '--from', String(from), '--wasm', wasmPath, '--orig', origPath,
        '--features', String(nFeatures), '--leaves', String(nLeaves), '--factors', factors.join(','), ...(hair ? ['--hair'] : []), ...(quick ? ['--quick'] : [])], { stdio: ['ignore', 'pipe', 'inherit'] });
      let buf = '', current = null, timer = null;
      const arm = () => { clearTimeout(timer); timer = setTimeout(() => child.kill('SIGKILL'), timeoutS * 1000 * (doc.heavy ? 4 : 1)); };
      arm();
      child.stdout.on('data', (d) => {
        buf += d;
        let i;
        while ((i = buf.indexOf('\n')) >= 0) {
          const line = JSON.parse(buf.slice(0, i)); buf = buf.slice(i + 1); arm();
          if (line.total !== undefined) { total = line.total; skippedTotal += line.skippedInvalid; }
          else if (line.baseline) baselines[doc.name] = line.baseline;
          else if (line.start) current = line.n;
          else if (line.at !== undefined) { results.push(line); from = line.n + 1; current = null; if (verbose) console.log(`  ${doc.name} #${line.n} ${line.id} ${line.leaf} x${line.factor}`); }
        }
      });
      child.on('exit', () => {
        clearTimeout(timer);
        if (current !== null) {
          // the worker died (hang killed, or wasm abort) in case `current`: record it as a failure of whichever kernel was running
          results.push({ part: doc.name, n: current, hang: true, id: '?', type: '?', leaf: '?', factor: 0 });
          from = current + 1;
        }
        if (total !== null && from < total) launch(); else resolve();
      });
    };
    launch();
  });
}
let next = 0;
await Promise.all(Array.from({ length: Math.min(jobs, docs.length) }, async () => { while (next < docs.length) await runPart(docs[next++]); }));

// ------------------------------------------------------------ report
const rows = results.sort((a, b) => a.part.localeCompare(b.part) || a.n - b.n);
let comparable = 0, oursErr = 0, volBad = 0, hangs = 0;
const fails = [], knownHit = new Set();
for (const r of rows) {
  const note = (why, kind) => {
    const k = keyOf(r);
    if (known[k]) { knownHit.add(k); fails.push({ ...r, why, known: known[k] }); }
    else fails.push({ ...r, why });
    return kind;
  };
  if (r.hang) { hangs++; note('hang/abort (worker killed)'); continue; }
  const o = r.orig;
  if (o.trap || o.errors.length) continue;
  comparable++;
  const u = r.ours;
  if (u.trap) { oursErr++; note('trap: ' + u.trap.slice(0, 80)); continue; }
  if (u.errors.length) { oursErr++; note(u.errors.join(' ')); continue; }
  // a part whose UNEDITED volume already differs from the original's is a baseline gap, not an edit failure
  if ((baselines[r.part]?.volDiff ?? 0) > volTol) continue;
  const rel = Math.abs(u.vol - o.vol) / Math.max(Math.abs(o.vol), 1e-12);
  if (!(rel <= volTol)) { volBad++; note(`volume ${u.vol?.toPrecision(7)} vs ${o.vol?.toPrecision(7)} (${(rel * 100).toFixed(2)}%)`); }
}
const failOrig = rows.filter((r) => !r.hang && (r.orig.trap || r.orig.errors.length)).length;
const unknown = fails.filter((f) => !f.known);
console.log(`edit robustness: ${docs.length} parts, ${rows.length} edit cases (factors ${factors.join('/')}, ${nFeatures} features x ${nLeaves} leaves per part; ${skippedTotal} edits skipped as invalid sketches)`);
console.log(`  original errors on the edit (case skipped): ${failOrig}`);
console.log(`  comparable cases (original builds clean): ${comparable}`);
console.log(`  FAIL ours: ${oursErr} feature error / trap, ${volBad} volume > ${volTol} off, ${hangs} hang  (${fails.length - unknown.length} of them known, see tools/edit_robustness_known.json)`);
if (fails.length) {
  console.log('\npart                         feature      edit                          result');
  for (const f of fails) console.log(`${f.known ? 'known ' : ''}${f.part.padEnd(28)} ${(f.id + ' ' + f.type).padEnd(12)} ${(f.leaf + ' ' + f.from + '->' + f.to).slice(0, 30).padEnd(30)} ${f.why}${f.known ? '  [' + f.known + ']' : ''}`);
}
for (const k of Object.keys(known)) if (!knownHit.has(k) && rows.some((r) => keyOf(r) === k)) console.log(`NOTE ${k} is listed as known but passes now: remove it from tools/edit_robustness_known.json`);
const bad = Object.entries(baselines).filter(([, b]) => b.ours !== 0 && b.orig === 0);
for (const [n, b] of bad) console.log(`NOTE ${n}: unedited baseline has ${b.ours} errors on ours (original 0)`);
if (jsonOut) writeFileSync(jsonOut, JSON.stringify({ rows, fails, baselines, comparable }, null, 1));
process.exit(unknown.length || bad.length ? 1 : 0);
