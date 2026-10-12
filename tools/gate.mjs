// Regression gate + scoreboard: original kernel.wasm vs the rebuilt wasm.
//
// usage: node tools/gate.mjs [--fast] [--update] [--wasm path] [--quiet]
//   Prints a scoreboard and compares it with tools/scoreboard.json (the
//   committed baseline). Exit 1 if ANY metric went down. --update rewrites
//   the baseline (do this in the commit that improves a metric).
//   Build first: cargo build -p ck-wasm --target wasm32-unknown-unknown --release
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
// HARD metrics (a drop fails the gate): wire, valid.*, vol.*, oracle.*, import.*, heal.repair.
// SOFT metrics (oracle-identity: faces/edges/box/rest/json/mesh/mass/step):
// a drop prints WARN but does not fail. The original is a reference, not
// the arbiter.
import { readFileSync, writeFileSync, existsSync, readdirSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';
import cases from './cases.mjs';
import { parts as brepParts } from './brep_cases.mjs';
import { oracles } from './oracles/index.mjs';
import { checkOracle } from './oracles/check.mjs';
import { runBench } from './bench/run.mjs';
import { runNaming } from './naming_stability.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f) => { const i = args.indexOf(f); if (i < 0) return false; args.splice(i, 1); return true; };
const opt = (f) => { const i = args.indexOf(f); if (i < 0) return null; const v = args[i + 1]; args.splice(i, 2); return v; };
const update = flag('--update'), quiet = flag('--quiet'), fast = flag('--fast');
const wasmB = opt('--wasm') ?? path.join(here, '../target/wasm32-unknown-unknown/release/ck_wasm.wasm');
// Artifacts live next to the ORIGINAL kernel.wasm (follows the worktree symlink).
const origWasm = realpathSync(path.join(here, '../../kernel.wasm'));
const art = path.dirname(origWasm);
const A = new WebAssembly.Module(readFileSync(origWasm));
const B = new WebAssembly.Module(readFileSync(wasmB));

const score = {};
const bump = (k, ok) => { score[k] ??= [0, 0]; score[k][1]++; if (ok) score[k][0]++; };
const norm = (r) => (r.trap ? `TRAP ${r.trap.replace(/\/rustc\/[0-9a-f]+\/|:\d+:\d+:/g, '')}` : r.json.replace(/"ms":[0-9.e+-]+/g, '"ms":0').replace(/"at":[0-9.e+-]+/g, '"at":0'));
const bin = (r) => (r.trap ? '' : Buffer.from(r.bytes.subarray((4 + new DataView(r.bytes.buffer).getUint32(0, true) + 7) & -8)).toString('base64'));

// 1. wire cases (stateful sequences)
for (const c of cases) {
  const ka = new Kernel(A), kb = new Kernel(B);
  for (const [i, body] of c.bodies.entries()) {
    const ra = ka.request(body), rb = kb.request(body);
    if (c.ignore?.includes(i)) continue;
    bump('wire', norm(ra) === norm(rb) && bin(ra) === bin(rb));
  }
}

// 2. regen targets
const rel = (x, y) => Math.abs(x - y) / (1 + Math.abs(y));
const close = (a, b) => {
  if (typeof a === 'number' && typeof b === 'number') return rel(a, b) <= 1e-9;
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length && a.every((x, i) => close(x, b[i]));
  if (a && b && typeof a === 'object') return Object.keys(a).join() === Object.keys(b).join() && Object.keys(a).every((k) => close(a[k], b[k]));
  return a === b;
};
const HEAL_KINDS = ['detach', 'shells', 'flip', 'invert', 'reverseCoedge', 'shuffle', 'tiny'];
const validCodes = {}, validFail = [], oracleFail = [], importFail = [];
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
    }
  } catch (e) { why = String(e); }
  bump('import.roundtrip', why === null);
  if (why) importFail.push(`${t.name}: ${why}`);
}
function regenSet(set, targets) {
  for (const t of targets) {
    const t0 = performance.now();
    const msg = { type: 'regen', tag: 'main', part: t.part, ...(t.upTo ? { opts: { upTo: t.upTo } } : {}) };
    const ka = new Kernel(A), kb = new Kernel(B);
    const body = JSON.stringify({ id: 1, msg });
    const ra = ka.request(body), rb = kb.request(body);
    const pa = ra.trap ? null : JSON.parse(ra.json), pb = rb.trap ? null : JSON.parse(rb.json);
    const ba = pa?.result?.body, bb = pb?.result?.body;
    const cnt = (b) => b?.edges?.map(({ pts, ...e }) => ({ ...e, pts: pts.length }));
    const rest = (p) => { const r = { ...(p?.result ?? {}) }; delete r.body; return JSON.stringify([p?.ok, p?.error, r]); };
    bump(`regen.${set}.faces`, !!pa && !!pb && JSON.stringify(ba?.faces) === JSON.stringify(bb?.faces));
    bump(`regen.${set}.edges`, !!pa && !!pb && JSON.stringify(cnt(ba)) === JSON.stringify(cnt(bb)));
    bump(`regen.${set}.box`, !!pa && !!pb && JSON.stringify(ba?.box) === JSON.stringify(bb?.box));
    bump(`regen.${set}.rest`, !!pa && !!pb && rest(pa) === rest(pb));
    bump(`regen.${set}.json`, !!pa && !!pb && norm(ra) === norm(rb));
    bump(`regen.${set}.mesh`, !!pa && !!pb && norm(ra) === norm(rb) && bin(ra) === bin(rb));
    if (ba && bb) {
      const q = JSON.stringify({ id: 2, msg: { type: 'massProps', tag: 'main' } });
      const ma = JSON.parse(ka.request(q).json || 'null')?.result, mb = JSON.parse(kb.request(q).json || 'null')?.result;
      bump(`mass.${set}`, ['volume', 'area', 'com'].every((k) => close(mb?.[k], ma?.[k])));
      if (set === 'brep' && oracles[t.name]) {
        const bad = checkOracle(oracles[t.name], mb, bb.box);
        bump('oracle.brep', bad.length === 0);
        if (bad.length) oracleFail.push(`${t.name}: ${bad.join(' | ')}`);
      }
      bump(`vol.${set}`, typeof mb?.volume === 'number' && Math.abs(mb.volume - ma?.volume) <= 1e-3 * Math.abs(ma?.volume ?? 0));
      const manifold = (bb.edges ?? []).every((e) => e.faces?.length === 2);
      const vq = JSON.stringify({ id: 3, msg: { type: 'validate', tag: 'main' } });
      const vr = JSON.parse(kb.request(vq).json || 'null')?.result;
      const vissues = vr?.issues ?? [{ code: 'NO_VALIDATE' }];
      for (const i of vissues) validCodes[i.code] = (validCodes[i.code] ?? 0) + 1;
      if (vissues.length) validFail.push(`${set}:${t.name}:${[...new Set(vissues.map((i) => i.code))].join(',')} | ${vissues.slice(0, 3).map((i) => `${i.entity}: ${i.detail}`).join(' ; ')}`);
      bump(`valid.${set}`, !(pb.result.errors?.length) && mb?.volume > 0 && manifold && vissues.length === 0);
      // heal.repair: damage a COPY of each valid brep body (see ck_heal::damage::KINDS),
      // heal it, and require validate-clean + volume within 1e-6 (relative).
      if (set === 'brep' && vissues.length === 0) for (const kind of HEAL_KINDS) {
        const hq = JSON.stringify({ id: 4, msg: { type: 'heal', tag: 'main', damage: kind } });
        const hd = JSON.parse(kb.request(hq).json || 'null')?.result?.damage;
        if (hd?.applied) bump('heal.repair', hd.clean === true);
      }
      if (set === 'brep') importRoundtrip(t, kb, pb, mb);
    } else {
      bump(`mass.${set}`, !ba && !bb);
      bump(`vol.${set}`, !ba && !bb);
      bump(`valid.${set}`, !ba && !bb && !(pb?.result?.errors?.length));
    }
    if (!quiet) process.stderr.write(`${t.name} ${((performance.now() - t0) / 1000).toFixed(1)}s\n`);
  }
}
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
  for (const [f, vol] of Object.entries(expected)) {
    const kb = new Kernel(B);
    const rq = (id, msg) => { const r = kb.request(JSON.stringify({ id, msg })); return r.trap ? null : JSON.parse(r.json)?.result; };
    const imp = rq(1, { type: 'importSat', tag: 'main', name: f, data: readFileSync(path.join(sdir, f), 'utf8') });
    const val = rq(2, { type: 'validate', tag: 'main' });
    const mass = rq(3, { type: 'massProps', tag: 'main' });
    const ok = !!imp && !imp.errors?.length && !!imp.body && val?.issues?.length === 0 && Math.abs(mass?.volume - vol) <= 1e-6 * vol;
    if (!ok && !quiet) process.stderr.write(`import.sat ${f}: ${JSON.stringify(imp?.errors)} issues=${JSON.stringify(val?.issues?.slice(0, 2))} vol=${mass?.volume} want=${vol}\n`);
    bump('import.sat', ok);
  }
}

// 2d. BenchCAD adapter (SOFT; failures carry a recorded cause in the manifest)
for (const r of runBench({ wasm: wasmB })) {
  bump('bench.benchcad', r.pass);
  if (r.import !== null) bump('bench.benchcad.step', r.import);
  if (!r.pass && !quiet) process.stderr.write(`bench.benchcad ${r.id}: ${r.known ? r.known.label : 'UNLABELLED'}: ${r.bad[0]}\n`);
}

// 2c. naming.stable (SOFT): references to earlier-feature faces/edges still point at the
// same geometric entity after upstream dimension edits (tools/naming_stability.mjs)
for (const r of runNaming({ ours: wasmB })) if (!r.neg && r.edit !== '(baseline)') bump('naming.stable', r.cls === 'ok');

// 3. STEP export
const gdir = path.join(here, '../crates/ck-io/tests/golden');
if (existsSync(gdir)) for (const f of readdirSync(gdir).filter((f) => f.endsWith('.req.json')).sort()) {
  const req = readFileSync(path.join(gdir, f), 'utf8');
  const st = '{"id":2,"msg":{"type":"step","name":"t"}}';
  const [a, b] = [A, B].map((m) => { const k = new Kernel(m); k.request(req); return norm(k.request(st)); });
  bump('step', a === b);
}
if (!quiet) process.stderr.write('\n');
if (importFail.length && !quiet) console.log(importFail.join('\n'));

// report + compare with baseline
const basePath = path.join(here, 'scoreboard.json');
const base = existsSync(basePath) ? JSON.parse(readFileSync(basePath, 'utf8')) : {};
let worse = 0, soft = 0;
const hard = (k) => /^(wire|valid\.|vol\.|oracle\.|import\.|heal\.)/.test(k);
const skipped = (k) => fast && /^(regen|mass|valid|vol)\.(bracket|stepper|flange|hair)/.test(k);
for (const k of Object.keys({ ...base, ...score }).sort()) {
  const [m, n] = score[k] ?? [0, 0], b = base[k]?.[0];
  const d = b === undefined ? ' (new)' : m > b ? ` (+${m - b})` : m < b ? ` (-${b - m}) ${hard(k) ? 'REGRESSION' : 'WARN(soft)'}` : '';
  if (skipped(k)) continue;
  if (b !== undefined && m < b) hard(k) ? worse++ : soft++;
  console.log(`${k.padEnd(40)} ${String(m).padStart(4)}/${String(n).padEnd(4)}${d}`);
}
for (const f of oracleFail) console.log(`ORACLE MISMATCH ${f}`);
if (Object.keys(validCodes).length) { console.log('validate issue codes:', JSON.stringify(validCodes)); if (!quiet) console.log(validFail.join('\n')); else console.log(`validate-failing cases: ${validFail.length}`); }
if (update) { const out = fast ? { ...base, ...score } : score; writeFileSync(basePath, JSON.stringify(out, null, 1) + '\n'); console.log(`baseline written: ${basePath}`); }
else if (worse) { console.log(`\nGATE FAILED: ${worse} metric(s) regressed`); process.exitCode = 1; }
else console.log(`\ngate ok${soft ? ` (${soft} soft oracle-identity metric(s) dropped)` : ''}`);
