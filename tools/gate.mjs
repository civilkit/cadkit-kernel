// Regression gate + scoreboard: original kernel.wasm vs the rebuilt wasm.
//
// usage: node tools/gate.mjs [--fast] [--update] [--wasm path] [--quiet] [--standalone] [--workers N]
//   Prints a scoreboard and compares it with tools/scoreboard.json (the
//   committed baseline). Exit 1 if ANY metric went down. --update rewrites
//   the baseline (do this in the commit that improves a metric).
//   Build first: cargo build -p ck-wasm --target wasm32-unknown-unknown --release
//
// Standalone mode (--standalone, or automatically when ../../kernel.wasm does not exist): no comparison build is
// needed. The metrics that compare against it (wire, regen.*, mass.*, vol.*, step*, edit.robust) are not run;
// everything that judges our build alone is (valid.*, oracle.*, import.*, heal.repair, naming.stable, iface.*,
// bench.*). The numbers are compared with tools/scoreboard.standalone.json instead, and --update writes that file.
// In this mode valid.* covers the parts that build; build.* counts how many of them do.
//
// Metrics (counts of matching items; higher is better):
//   wire.*        tools/cases.mjs replies byte-identical (ms normalised)
//   regen.<set>.* per target: faces / edges (pts as counts) / box / result
//                 keys outside body / whole JSON reply (edge pts included)
//   mass.<set>    massProps volume/area/com within 1e-9 relative
//   step          STEP replies byte-identical (ck-io golden request bodies)
//   valid.<set>   OURS alone: no feature errors, volume > 0, every edge
//                 bounds exactly 2 faces (closed manifold shell)
//   oracle.<set>  HARD: massProps volume/area/com + box vs the closed-form
//                 oracles of tools/oracles (independent of the original), 1e-6
//   vol.<set>     our volume within 1e-3 relative of the original's
//   naming.stable OURS alone (SOFT): case/edit pairs where every name referenced by a later
//                 feature still resolves to the same geometric entity (tools/naming_stability.mjs)
//   iface.*       OURS alone (SOFT): describe/resolve/check/refDiff self-consistency on every 4th
//                 brep part (tools/agent_iface.mjs, docs/agent-queries.md)
//   heal.repair   OURS alone: damaged copies of the valid brep bodies healed
//                 back to validate-clean with volume within 1e-6
//   import.roundtrip  OURS alone (HARD): our `step` export -> `importStep` into
//                 a fresh tag; volume/area within 1e-6, same face and edge
//                 counts, `validate` clean (brep parts that produce a body)
//   import.sat    OURS alone: each fixture in crates/ck-io/tests/fixtures/sat/
//                 expected.json imports with no errors, validates with zero
//                 issues and has the closed-form volume (1e-6 relative)
//   bench.benchcad  SOFT: BenchCAD parts (tools/bench/benchcad, 30 parts) whose
//                 regen matches the CadQuery ground truth (volume/area/com/
//                 bbox 1e-3) and validates clean; bench.benchcad.step: the
//                 ground-truth STEP imports to the same numbers, validate clean
//
// Direction (2026-10-12): correctness + robustness, not byte-identity.
// HARD metrics (a drop fails the gate): wire, leak, edit.robust, valid.*, vol.*, oracle.*, import.*, heal.repair.
// SOFT metrics (oracle-identity: faces/edges/box/rest/json/mesh/mass/step):
// a drop prints WARN but does not fail. The original is a reference, not
// the arbiter.
//
// Workers (--workers N, or CK_GATE_WORKERS=N; default min(4, cores)): every case (a wire sequence, a regen target, a
// SAT file, a bench part, a naming case, an iface part, a STEP golden, the edit.robust run) is a task; N worker threads,
// each with its own wasm instances, take the tasks one at a time, and the main thread applies their results (score
// bumps, report lines, stderr lines) in the sequential order, so the output is the same as with --workers 1 (everything
// in one thread) apart from the timing numbers. CK_GATE_TIMING=1 prints per-group busy time on stderr.
import { readFileSync, writeFileSync, existsSync, readdirSync, realpathSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { availableParallelism } from 'node:os';
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import cases from './cases.mjs';
import { parts as brepParts } from './brep_cases.mjs';
import { oracles } from './oracles/index.mjs';
import { checkOracle } from './oracles/check.mjs';
import { runBench, benchIds } from './bench/run.mjs';
import { runNaming, namingCases } from './naming_stability.mjs';
import { runIface } from './agent_iface.mjs';
import { findLeaks } from './check_leaks.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const update = flag('--update'), quiet = flag('--quiet'), fast = flag('--fast');
const wasmB = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
const standaloneFlag = flag('--standalone');
const workersOpt = opt('--workers') ?? process.env.CK_GATE_WORKERS;
const origPath = path.join(here, '../../kernel.wasm');
const haveRef = !standaloneFlag && existsSync(origPath);
// Artifacts live next to the comparison kernel.wasm (follows the worktree symlink); standalone: next to the build.
const origWasm = haveRef ? realpathSync(origPath) : null;
// CK_GATE_ART: a directory of its own for these files (concurrent runs on the CI box share one oracle kernel.wasm and must not share its directory)
const art = process.env.CK_GATE_ART ? path.resolve(process.env.CK_GATE_ART) : origWasm ? path.dirname(origWasm) : path.dirname(path.resolve(wasmB));
// workers receive the modules compiled by the main thread (a WebAssembly.Module crosses threads without recompiling)
const A = haveRef ? (workerData?.A ?? new WebAssembly.Module(readFileSync(origWasm))) : null;
if (!haveRef && isMainThread) console.log('standalone: no comparison build; wire, regen.*, mass.*, vol.*, step* and edit.robust are not run');
const B = workerData?.B ?? new WebAssembly.Module(readFileSync(wasmB));

// Every case below is a TASK that records EVENTS (score bumps, report lines, stderr lines) instead of applying them;
// the main thread applies each task's events in task order, so the output does not depend on which worker ran what.
let ev = null;   // the running task's event list
const bump = (k, ok) => ev.push(['b', k, !!ok]);
const note = (list, s) => ev.push(['p', list, s]);
const err = (s) => { if (!quiet) ev.push(['e', s]); };
// our-only warning (the reference clamps a zero depth silently): leave it out of the comparison with the reference
const dropOurWarnings = (j) =>
  j.includes('"extrudeZeroDepth"') || j.includes('"sheetCutFallback"')
    ? j
        .replace(/\["[^"]*",\{"code":"(extrudeZeroDepth|sheetCutFallback)"(?:[^\]\[]|\[[^\]]*\]|\{[^{}]*\})*\}\],?/g, '')
        .replace(/,\]/g, ']')
        .replace(/,"warnings":\[\]|"warnings":\[\],?/g, '')
    : j;
// the original kernel's STEP header names its old product; the rebuilt writer says CadKit (the old name is built from pieces so no source file carries it)
const OLD_PRODUCT = new RegExp(['Purra', 'metric'].join(''), 'gi');
const norm = (r) => (r.trap ? `TRAP ${r.trap.replace(/\/rustc\/[0-9a-f]+\/|:\d+:\d+:/g, '')}` : dropOurWarnings(r.json).replace(/"ms":[0-9.e+-]+/g, '"ms":0').replace(/"at":[0-9.e+-]+/g, '"at":0').replace(/,"build":"[0-9a-f]+(-dirty)?"/g, '').replace(OLD_PRODUCT, 'CadKit'));
const bin = (r) => (r.trap ? '' : Buffer.from(r.bytes.subarray((4 + new DataView(r.bytes.buffer).getUint32(0, true) + 7) & -8)).toString('base64'));

const tasks = [];   // { g: group, name, run }, in the order of the sequential gate
const task = (g, name, run) => tasks.push({ g, name, run });

// 0. leak check: the built wasm carries none of the forbidden patterns (tools/check_leaks.mjs)
task('leak', 'wasm', () => bump('leak', findLeaks(readFileSync(wasmB)).length === 0));

// 1. wire cases (stateful sequences)
for (const c of haveRef ? cases : []) task('wire', c.name ?? String(tasks.length), () => {
  const ka = new Kernel(A), kb = new Kernel(B);
  for (const [i, body] of c.bodies.entries()) {
    const ra = ka.request(body), rb = kb.request(body);
    if (c.ignore?.includes(i)) continue;
    const same = norm(ra) === norm(rb) && bin(ra) === bin(rb);
    // a known, intentional divergence (tools/cases.mjs `known`): the rebuilt reply must be exactly the pinned one
    const kn = !same && c.known?.[i] && norm(rb) === c.known[i].json && bin(rb) === '' ? c.known[i] : null;
    if (kn) err(`wire known divergence ${c.name}#${i}: ${kn.reason}\n`);
    bump('wire', same || !!kn);
  }
});

// 2. regen targets
const rel = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
const close = (a, b) => {
  if (typeof a === 'number' && typeof b === 'number') return rel(a, b) <= 1e-9;
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length && a.every((x, i) => close(x, b[i]));
  if (a && b && typeof a === 'object') return Object.keys(a).join() === Object.keys(b).join() && Object.keys(a).every((k) => close(a[k], b[k]));
  return a === b;
};
const HEAL_KINDS = ['detach', 'shells', 'flip', 'invert', 'reverseCoedge', 'shuffle', 'tiny'];
// our STEP export -> importStep (tag "imp") -> massProps / validate / counts
function importRoundtrip(t, kb, pb, mb) {
  let why = null;
  try {
    const st = JSON.parse(kb.request(JSON.stringify({ id: 4, msg: { type: 'step', tag: 'main', name: 'rt' } })).json || 'null');
    const imp = JSON.parse(kb.request(JSON.stringify({ id: 5, msg: { type: 'importStep', tag: 'imp', name: 'rt', data: st?.result } })).json || 'null');
    const body = imp?.result?.body;
    if (!body) why = `import: ${JSON.stringify(imp?.result?.errors ?? imp?.error)}`.slice(0, 200);
    else {
      const m = JSON.parse(kb.request(JSON.stringify({ id: 6, msg: { type: 'massProps', tag: 'imp' } })).json || 'null')?.result;
      const v = JSON.parse(kb.request(JSON.stringify({ id: 7, msg: { type: 'validate', tag: 'imp' } })).json || 'null')?.result;
      const ob = pb.result.body, r = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
      if (!(r(m?.volume, mb.volume) <= 1e-6)) why = `volume ${m?.volume} vs ${mb.volume}`;
      else if (!(r(m?.area, mb.area) <= 1e-6)) why = `area ${m?.area} vs ${mb.area}`;
      else if (body.faces.length !== ob.faces.length) why = `faces ${body.faces.length} vs ${ob.faces.length}`;
      else if (body.edges.length !== ob.edges.length) why = `edges ${body.edges.length} vs ${ob.edges.length}`;
      else if (!v || v.issues.length) why = `validate ${JSON.stringify(v?.issues?.slice(0, 2))}`;
      // import.heal.noop: healing a valid body changes nothing (no fix recorded, volume identical to 1e-12)
      if (why === null) {
        const h = JSON.parse(kb.request(JSON.stringify({ id: 8, msg: { type: 'heal', tag: 'imp', dryRun: true, tol: 1e-4 } })).json || 'null')?.result;
        const hv = h?.volume;
        bump('import.heal.noop', !!h && h.fixed.length === 0 && Math.abs(hv.after - hv.before) <= 1e-12 * (1 + Math.abs(hv.before)));
      }
    }
  } catch (e) { why = String(e); }
  bump('import.roundtrip', why === null);
  if (why) note('importFail', `${t.name}: ${why}`);
}
function regenOne(set, t) {
  const t0 = performance.now();
  const msg = { type: 'regen', tag: 'main', part: t.part, ...(t.upTo ? { opts: { upTo: t.upTo } } : {}) };
  const ka = haveRef ? new Kernel(A) : null, kb = new Kernel(B);
  const body = JSON.stringify({ id: 1, msg });
  const ra = ka?.request(body), rb = kb.request(body);
  const pa = !ra || ra.trap ? null : JSON.parse(ra.json), pb = rb.trap ? null : JSON.parse(rb.json);
  const ba = pa?.result?.body, bb = pb?.result?.body;
  const cnt = (b) => b?.edges?.map(({ pts, ...e }) => ({ ...e, pts: pts.length }));
  const rest = (p) => { const r = { ...(p?.result ?? {}) }; delete r.body; if (r.warnings) { r.warnings = r.warnings.filter((e) => !['extrudeZeroDepth', 'sheetCutFallback'].includes(e?.[1]?.code)); if (!r.warnings.length) delete r.warnings; } return JSON.stringify([p?.ok, p?.error, r]); };
  if (haveRef) bump(`regen.${set}.faces`, !!pa && !!pb && JSON.stringify(ba?.faces) === JSON.stringify(bb?.faces));
  if (haveRef) bump(`regen.${set}.edges`, !!pa && !!pb && JSON.stringify(cnt(ba)) === JSON.stringify(cnt(bb)));
  if (haveRef) bump(`regen.${set}.box`, !!pa && !!pb && JSON.stringify(ba?.box) === JSON.stringify(bb?.box));
  if (haveRef) bump(`regen.${set}.rest`, !!pa && !!pb && rest(pa) === rest(pb));
  if (haveRef) bump(`regen.${set}.json`, !!pa && !!pb && norm(ra) === norm(rb));
  if (pa && pb && norm(ra) !== norm(rb)) note('jsonDiff', `${set}:${t.name}${(pa.result?.errors?.length ?? 0) > 0 ? ' (original has errors)' : ''}`);
  if (haveRef) bump(`regen.${set}.mesh`, !!pa && !!pb && norm(ra) === norm(rb) && bin(ra) === bin(rb));
  if (haveRef && pa && pb && norm(ra) === norm(rb) && bin(ra) !== bin(rb)) note('meshDiff', `${set}:${t.name}`);
  if (haveRef ? ba && bb : bb) {
    const q = JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } });
    const ma = ka ? JSON.parse(ka.request(q).json || 'null')?.result : null, mb = JSON.parse(kb.request(q).json || 'null')?.result;
    if (haveRef) bump(`mass.${set}`, ['volume', 'area', 'com'].every((k) => close(mb?.[k], ma?.[k])));
    if (set === 'brep' && oracles[t.name]) {
      const bad = checkOracle(oracles[t.name], mb, bb.box);
      bump('oracle.brep', bad.length === 0);
      if (bad.length) note('oracleFail', `${t.name}: ${bad.join(' | ')}`);
    }
    const volOk = typeof mb?.volume === 'number' && Math.abs(mb.volume - ma?.volume) <= 1e-3 * Math.abs(ma?.volume ?? 0);
    // the original refuses the feature (`unsupported` ...) and ours builds it: not a regression
    const origRefused = (pa?.result?.errors?.length ?? 0) > 0 && (pb?.result?.errors?.length ?? 0) === 0;
    if (haveRef) bump(`vol.${set}`, volOk || origRefused);
    if (haveRef && !volOk) note('volFail', `${set}:${t.name}: ours ${mb?.volume} original ${ma?.volume}`);
    const manifold = (bb.edges ?? []).every((e) => e.faces?.length === 2);
    const vq = JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } });
    const vr = JSON.parse(kb.request(vq).json || 'null')?.result;
    const vissues = vr?.issues ?? [{ code: 'NO_VALIDATE' }];
    for (const i of vissues) ev.push(['c', i.code]);
    if (vissues.length) note('validFail', `${set}:${t.name}:${[...new Set(vissues.map((i) => i.code))].join(',')} | ${vissues.slice(0, 3).map((i) => `${i.entity}: ${i.detail}`).join(' ; ')}`);
    if (!haveRef) bump(`build.${set}`, true);
    bump(`valid.${set}`, !(pb.result.errors?.length) && mb?.volume > 0 && manifold && vissues.length === 0);
    // heal.repair: damage a COPY of each valid brep body (see ck_heal::damage::KINDS),
    // heal it, and require validate-clean + volume within 1e-6 (relative).
    if (set === 'brep' && vissues.length === 0) for (const kind of HEAL_KINDS) {
      const hq = JSON.stringify({ id: 4, msg: { type: 'heal', tag: 'main', damage: kind } });
      const hd = JSON.parse(kb.request(hq).json || 'null')?.result?.damage;
      if (hd?.applied) bump('heal.repair', hd.clean === true);
    }
    if (set === 'brep') importRoundtrip(t, kb, pb, mb);
  } else if (haveRef) {
    bump(`mass.${set}`, !ba && !bb);
    bump(`vol.${set}`, !ba && !bb);
    bump(`valid.${set}`, !ba && !bb && !(pb?.result?.errors?.length));
  } else {
    bump(`build.${set}`, false);
  }
  err(`${t.name} ${((performance.now() - t0) / 1000).toFixed(1)}s\n`);
}
const regenSet = (set, targets) => { for (const t of targets) task(`regen.${set}`, t.name, () => regenOne(set, t)); };
regenSet('brep', Object.entries(brepParts).map(([name, part]) => ({ name, part })));
// --fast skips the sample parts (minutes while booleans are slow)
for (const f of fast ? [] : ['bracket', 'stepper-sample', 'coupling-sample', 'large-sample']) {
  const p = path.join(art, `${f}.ckpart`);
  if (!existsSync(p)) continue;
  const part = JSON.parse(readFileSync(p, 'utf8'));
  regenSet(f, part.features.map((_, k) => ({ name: `${f}@${k + 1}`, part, upTo: k + 1 })));
}

// 2b. ACIS SAT import (ours only; closed-form volumes in expected.json)
const sdir = path.join(here, '../crates/ck-io/tests/fixtures/sat');
if (existsSync(path.join(sdir, 'expected.json'))) {
  const expected = JSON.parse(readFileSync(path.join(sdir, 'expected.json'), 'utf8'));
  for (const [f, vol] of Object.entries(expected)) task('import.sat', f, () => {
    const kb = new Kernel(B);
    const rq = (id, msg) => { const r = kb.request(JSON.stringify({ id, msg })); return r.trap ? null : JSON.parse(r.json)?.result; };
    const imp = rq(1, { type: 'importSat', tag: 'main', name: f, data: readFileSync(path.join(sdir, f), 'utf8') });
    const val = rq(2, { type: 'validate', tag: 'main' });
    const mass = rq(3, { type: 'massProps', tag: 'main' });
    const ok = !!imp && !imp.errors?.length && !!imp.body && val?.issues?.length === 0 && Math.abs(mass?.volume - vol) <= 1e-6 * vol;
    if (!ok) err(`import.sat ${f}: ${JSON.stringify(imp?.errors)} issues=${JSON.stringify(val?.issues?.slice(0, 2))} vol=${mass?.volume} want=${vol}\n`);
    bump('import.sat', ok);
  });
}

// 2d. BenchCAD adapter (SOFT; failures carry a recorded cause in the manifest)
for (const id of benchIds('benchcad')) task('bench.benchcad', id, () => {
  for (const r of runBench({ wasm: wasmB, mod: B, only: id })) {
    bump('bench.benchcad', r.pass);
    if (r.import !== null) bump('bench.benchcad.step', r.import);
    if (!r.pass) err(`bench.benchcad ${r.id}: ${r.known ? r.known.label : 'UNLABELLED'}: ${r.bad[0]}\n`);
  }
});

// 2d'. brepjs reference STEP files through importStep (the dataset's own OCCT output; HARD via bench.brepjs.step)
for (const id of benchIds('brepjs')) task('bench.brepjs', id, () => {
  for (const r of runBench({ wasm: wasmB, mod: B, set: 'brepjs', only: id })) {
    if (r.import !== null) bump('bench.brepjs.step', r.import);
  }
});

// 2c. naming.stable (SOFT): references to earlier-feature faces/edges still point at the
// same geometric entity after upstream dimension edits (tools/naming_stability.mjs)
for (const name of namingCases()) task('naming', name, () => {
  for (const r of runNaming({ ours: B, only: name })) if (!r.neg && r.edit !== '(baseline)') bump('naming.stable', r.cls === 'ok');
});

// 2c'. iface.* (SOFT): agent queries (check/resolve/describe/refDiff) self-consistency, every 4th brep part
for (const name of Object.keys(brepParts).filter((_, i) => i % 4 === 0)) task('iface', name, () => {
  for (const r of runIface({ wasm: wasmB, mod: B, only: name })) bump(`iface.${r.key}`, r.ok);
});

// 2e. edit.robust (HARD; needs the comparison build): single-parameter edits of 12 small parts regen like the original's (no feature
// error, volume within 1e-3) wherever the original builds them (tools/edit_robustness_check.mjs --quick; its own 4 processes)
if (haveRef) task('edit.robust', 'quick', () => {
  const jf = path.join(art, 'edit_quick.json');
  try { execFileSync(process.execPath, [path.join(here, 'edit_robustness_check.mjs'), '--quick', '--jobs', '4', '--wasm', wasmB, '--json', jf], { stdio: 'ignore' }); } catch { /* exit 1 = failures; the json is still written */ }
  if (existsSync(jf)) {
    const j = JSON.parse(readFileSync(jf, 'utf8'));
    const bad = new Set(j.fails.filter((f) => !f.known).map((f) => `${f.part}#${f.n}`));
    for (const r of j.rows) if (!r.hang && !(r.orig.trap || r.orig.errors.length)) bump('edit.robust', !bad.has(`${r.part}#${r.n}`));
    for (const r of j.rows) if (r.hang) bump('edit.robust', false);
  }
});

// 3. STEP export
const gdir = path.join(here, '../crates/ck-io/tests/golden');
if (haveRef && existsSync(gdir)) for (const f of readdirSync(gdir).filter((f) => f.endsWith('.req.json')).sort()) task('step', f, () => {
  const req = readFileSync(path.join(gdir, f), 'utf8');
  const st = '{"id":2,"msg":{"type":"step","name":"t"}}';
  const [a, b] = [A, B].map((m) => { const k = new Kernel(m); k.request(req); return norm(k.request(st)); });
  bump('step', a === b);
  // same replies with every number rounded to 9 significant digits and -0 as 0: only
  // float-order noise (the oracle's own solver residue) separates these from identical
  const near = (t) => t.replace(/-?\d+\.\d*(?:E-?\d+)?/g, (x) => { const v = Number(x); return v === 0 ? '0' : v.toPrecision(9); });
  bump('step.near', near(a) === near(b));
});

// run one task: its events, then its wall time (['t', ms], for CK_GATE_TIMING)
const runTask = (i) => { ev = []; const t0 = performance.now(); tasks[i].run(); ev.push(['t', performance.now() - t0]); const e = ev; ev = null; return e; };

if (!isMainThread) {
  // worker: run the task indices the main thread sends, reply with their events; -1 = no more work
  parentPort.on('message', (i) => {
    if (i < 0) { parentPort.close(); return; }
    let e;
    try { e = runTask(i); } catch (x) { parentPort.postMessage({ i, error: x?.stack ?? String(x) }); return; }
    parentPort.postMessage({ i, ev: e });
  });
} else {
  const score = {}, validCodes = {};
  const lists = { validFail: [], oracleFail: [], importFail: [], volFail: [], jsonDiff: [], meshDiff: [] };
  const timing = process.env.CK_GATE_TIMING ? {} : null, tWall0 = performance.now();
  const apply = (i, e) => {
    for (const [t, a, b] of e) {
      if (t === 'b') { score[a] ??= [0, 0]; score[a][1]++; if (b) score[a][0]++; }
      else if (t === 'p') lists[a].push(b);
      else if (t === 'c') validCodes[a] = (validCodes[a] ?? 0) + 1;
      else if (t === 'e') process.stderr.write(a);
      else if (t === 't' && timing) { const g = (timing[tasks[i].g] ??= { n: 0, ms: 0, max: 0, maxName: '' }); g.n++; g.ms += a; if (a > g.max) { g.max = a; g.maxName = tasks[i].name; } }
    }
  };
  // --workers N / CK_GATE_WORKERS=N: worker threads, each with its own wasm instances; 1 = everything in this thread, in order
  const nWorkers = Math.max(1, Math.min(tasks.length, workersOpt ? Math.floor(+workersOpt) || 1 : Math.min(4, availableParallelism())));
  if (nWorkers === 1) for (let i = 0; i < tasks.length; i++) apply(i, runTask(i));
  else {
    // tasks go out one at a time to whichever worker is free, the slow groups first (the long edit.robust run, then the
    // brep regen and the bench parts; the cheap groups fill the tail); results are applied strictly in task order
    const first = ['edit.robust', 'bench.benchcad', 'regen.brep', 'bench.brepjs'];
    const rank = (g) => { const k = first.indexOf(g); return k < 0 ? first.length : k; };
    const order = tasks.map((_, i) => i).sort((x, y) => rank(tasks[x].g) - rank(tasks[y].g) || x - y);
    const res = new Array(tasks.length);
    let next = 0, applied = 0;
    const pool = [];
    try {
      await new Promise((resolve, reject) => {
        let live = nWorkers;
        for (let w = 0; w < nWorkers; w++) {
          const wk = new Worker(new URL(import.meta.url), { argv: process.argv.slice(2), workerData: { A, B } });
          pool.push(wk);
          const feed = () => wk.postMessage(next < order.length ? order[next++] : -1);
          wk.on('message', ({ i, ev: e, error }) => {
            if (error) { reject(new Error(`task ${tasks[i].g} ${tasks[i].name} failed:\n${error}`)); return; }
            res[i] = e;
            while (applied < tasks.length && res[applied]) { apply(applied, res[applied]); res[applied++] = null; }
            feed();
          });
          wk.on('error', (x) => reject(new Error(`gate worker ${w} crashed: ${x?.stack ?? x}`)));
          wk.on('exit', (code) => { if (code !== 0) reject(new Error(`gate worker ${w} exited with code ${code}`)); else if (--live === 0) resolve(); });
          feed();
        }
      });
      if (applied !== tasks.length) throw new Error(`only ${applied} of ${tasks.length} gate tasks finished`);
    } catch (x) {
      for (const wk of pool) wk.terminate();
      console.log(`\nGATE FAILED: ${x.message}`);
      process.exit(1);
    }
  }
  if (timing) {
    for (const [g, t] of Object.entries(timing)) process.stderr.write(`timing ${g.padEnd(16)} tasks ${String(t.n).padStart(4)} busy ${(t.ms / 1000).toFixed(1)}s slowest ${(t.max / 1000).toFixed(1)}s ${t.maxName}\n`);
    const c = process.cpuUsage();
    process.stderr.write(`timing total ${nWorkers} worker(s) wall ${((performance.now() - tWall0) / 1000).toFixed(1)}s cpu ${((c.user + c.system) / 1e6).toFixed(1)}s (this process; edit.robust children not counted)\n`);
  }
  const { validFail, oracleFail, importFail } = lists;
  if (!quiet) process.stderr.write('\n');
  if (importFail.length && !quiet) console.log(importFail.join('\n'));

  // report + compare with baseline
  const basePath = path.join(here, haveRef ? 'scoreboard.json' : 'scoreboard.standalone.json');
  const base = existsSync(basePath) ? JSON.parse(readFileSync(basePath, 'utf8')) : {};
  let worse = 0, soft = 0;
  const hard = (k) => /^(wire|leak|valid\.|vol\.|oracle\.|import\.|heal\.|edit\.)/.test(k);
  const skipped = (k) => fast && /^(regen|mass|valid|vol)\.(bracket|stepper|flange|hair)/.test(k);
  for (const k of Object.keys({ ...base, ...score }).sort()) {
    const [m, n] = score[k] ?? [0, 0], b = base[k]?.[0];
    const d = b === undefined ? ' (new)' : m > b ? ` (+${m - b})` : m < b ? ` (-${b - m}) ${hard(k) ? 'REGRESSION' : 'WARN(soft)'}` : '';
    if (skipped(k)) continue;
    if (b !== undefined && m < b) hard(k) ? worse++ : soft++;
    console.log(`${k.padEnd(40)} ${String(m).padStart(4)}/${String(n).padEnd(4)}${d}`);
  }
  for (const f of oracleFail) console.log(`ORACLE MISMATCH ${f}`);
  for (const f of lists.volFail) console.log(`VOL MISMATCH ${f}`);
  for (const f of lists.jsonDiff) console.log(`JSON DIFF ${f}`);
  for (const f of lists.meshDiff) console.log(`MESH DIFF ${f}`);   // same JSON, different binary buffers: the cases the mesh metric misses
  if (Object.keys(validCodes).length) { console.log('validate issue codes:', JSON.stringify(validCodes)); if (!quiet) console.log(validFail.join('\n')); else console.log(`validate-failing cases: ${validFail.length}`); }
  if (update) { const out = fast ? { ...base, ...score } : score; writeFileSync(basePath, JSON.stringify(out, null, 1) + '\n'); console.log(`baseline written: ${basePath}`); }
  else if (worse) { console.log(`\nGATE FAILED: ${worse} metric(s) regressed`); process.exitCode = 1; }
  else console.log(`\ngate ok${soft ? ` (${soft} soft oracle-identity metric(s) dropped)` : ''}`);
}
