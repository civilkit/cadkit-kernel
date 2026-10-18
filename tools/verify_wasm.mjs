// One-command verifier for every sweep and corpus check.
import { readFileSync, writeFileSync, existsSync, mkdtempSync, rmSync } from 'node:fs';
import { spawn } from 'node:child_process';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';
import crypto from 'node:crypto';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.join(here, '..');
const argv = process.argv.slice(2);
const wasm = argv.find((a) => !a.startsWith('--'));
const quick = argv.includes('--quick');
const accept = argv.includes('--accept');
if (accept && quick) { console.error('--accept requires a full run'); process.exit(2); }
const val = (f, d) => (argv.includes(f) ? argv[argv.indexOf(f) + 1] : d);
if (!wasm) { console.error('usage: tools/verify_wasm.sh <wasm> [--quick]'); process.exit(2); }
const tmp = mkdtempSync(path.join(os.tmpdir(), 'ck-verify-'));
const wasmHash = crypto.createHash('sha256').update(readFileSync(wasm)).digest('hex').slice(0, 16);
const t0 = Date.now();
const PASS = new Set(['ok', 'okNoEffect', 'okEmpty', 'refused', 'structured-error', 'surface-only', 'degenerate']);
const UNRESOLVED = new Set(['no-ref', 'ref-fails', 'missingMass', 'ocp-missing', 'no-mesh']);
const results = [];

function run(cmd, args, timeout) {
  return new Promise((resolve) => {
    const start = Date.now();
    const p = spawn('perl', ['-e', `alarm ${Math.ceil(timeout / 1000)}; exec @ARGV`, cmd, ...args]);
    let out = '', err = '';
    p.stdout.on('data', (d) => { out += d; });
    p.stderr.on('data', (d) => { err += d; });
    const timer = setTimeout(() => p.kill('SIGKILL'), timeout + 5000);
    p.on('close', (code, signal) => {
      clearTimeout(timer);
      const sec = (Date.now() - start) / 1000;
      if (signal) return resolve({ status: 'hang', sec });
      if (code !== 0) return resolve({ status: 'error', code, sec, detail: (err || out).slice(-300) });
      resolve({ status: 'ok', sec });
    });
  });
}

async function mapLimit(items, limit, fn) {
  const out = new Array(items.length);
  let next = 0;
  async function worker() {
    while (true) {
      const i = next++;
      if (i >= items.length) return;
      out[i] = await fn(items[i], i);
    }
  }
  await Promise.all(Array.from({ length: Math.min(limit, items.length) }, worker));
  return out;
}

function baseline(file) { return JSON.parse(readFileSync(path.join(here, '..', file), 'utf8')); }
function readJson(file) { try { return JSON.parse(readFileSync(file, 'utf8')); } catch { return null; } }
function classOf(v) { return typeof v === 'string' ? v : v.cls; }

function compare(rows, base, idKey = 'id') {
  const map = base.cases ?? {};
  const classes = {};
  const failures = [], passes = [], unresolved = [];
  for (const r of rows) {
    classes[r.cls] = (classes[r.cls] ?? 0) + 1;
    const id = r[idKey];
    const b = map[id];
    if (b == null) continue;
    const was = PASS.has(classOf(b)), now = PASS.has(r.cls);
    if (was && !now) (UNRESOLVED.has(r.cls) ? unresolved : failures).push({ id, from: classOf(b), to: r.cls, detail: r.detail });
    else if (!was && now) passes.push({ id, from: classOf(b), to: r.cls });
  }
  const passN = rows.filter((r) => PASS.has(r.cls)).length;
  return { classes, failures, passes, unresolved, n: rows.length, passN, failN: rows.length - passN };
}

const SWEEPS = [
  { name: 'fillet', file: 'tools/fillet_sweep.mjs', base: 'tools/fillet_sweep_baseline.json', timeout: 180 },
  { name: 'shell', file: 'tools/shell_sweep.mjs', base: 'tools/shell_sweep_baseline.json', timeout: 120 },
  { name: 'boolean', file: 'tools/boolean_sweep.mjs', base: 'tools/boolean_sweep_baseline.json', timeout: 120 },
  { name: 'shape', file: 'tools/shape_sweep.mjs', base: 'tools/shape_sweep_baseline.json', timeout: 120 },
  { name: 'massprops', file: 'tools/massprops_sweep.mjs', base: 'tools/massprops_sweep_baseline.json', timeout: 120 },
  { name: 'step', file: 'tools/step_roundtrip_sweep.mjs', base: 'tools/step_roundtrip_baseline.json', timeout: 120 },
  { name: 'tess', file: 'tools/tess_sweep.mjs', base: 'tools/tess_sweep_baseline.json', timeout: 120 },
  { name: 'features', file: 'tools/feature_sweep.mjs', base: 'tools/feature_sweep_baseline.json', timeout: 120 },
];

async function runSweep(s) {
  const base = baseline(s.base);
  const templates = [...new Set(Object.keys(base.cases).map((id) => id.replace(/_\d+$/, '')))];
  const runList = quick ? templates.slice(0, 4) : templates;
  const samples = quick ? 5 : (base.samples ?? 40);
  const all = [];
  let hang = false, sec = 0;
  for (const t of runList) {
    const out = path.join(tmp, `${s.name}_${t}.json`);
    const r = await run('node', [path.join(root, s.file), '--wasm', wasm, '--samples', String(samples), '--seed', String(base.seed ?? 7), '--only', t, '--json', out], s.timeout * 1000);
    sec += r.sec;
    if (r.status !== 'ok') { return { name: s.name, status: 'error', sec, reason: `${t}: ${r.status} ${r.detail ?? ''}`.trim() }; }
    all.push(...JSON.parse(readFileSync(out, 'utf8')));
  }
  if (hang) return { name: s.name, status: 'error', sec, reason: 'hang' };
  return { name: s.name, status: 'ok', sec, rows: all, samples, seed: base.seed ?? 7, ...compare(all, base) };
}
results.push(...(await mapLimit(SWEEPS, quick ? 1 : 2, runSweep)));

// Crash / hang fuzz
{
  const out = path.join(tmp, 'crashfuzz.json');
  const cases = quick ? 200 : 5000;
  const r = await run('node', [path.join(root, 'tools/crash_fuzz.mjs'), '--wasm', wasm, '--cases', String(cases), '--seed', '7', '--no-shrink', '--workers', '8', '--json', out], (quick ? 90 : 360) * 1000);
  const d = readJson(out);
  if (d?.results) results.push({ name: 'crashfuzz', status: 'ok', sec: r.sec, rows: d.results, ...compare(d.results, baseline('tools/crash_fuzz_baseline.json'), 'id') });
  else results.push({ name: 'crashfuzz', status: 'error', sec: r.sec, reason: (r.detail || r.status) });
}

// OCCT corpus
const occt = '/tmp/occt';
if (existsSync(occt)) {
  const out = path.join(tmp, 'occt.json');
  const args = [path.join(root, 'tools/occt_corpus.mjs'), '--occt', occt, '--wasm', wasm, '--quiet', '--json', out];
  if (quick) args.push('--only', 'bopcut_simple');
  const r = await run('node', args, (quick ? 120 : Number(val('--occt-timeout', '360'))) * 1000);
  const rows = readJson(out);
  if (rows) results.push({ name: 'occt', status: 'ok', sec: r.sec, rows, ...compare(rows, baseline('tools/occt_baseline.json'), 'name') });
  else results.push({ name: 'occt', status: 'error', sec: r.sec, reason: (r.detail || r.status) });
} else results.push({ name: 'occt', status: 'skip', reason: '/tmp/occt absent' });

// Fusion import (opt-in cache)
if (!quick) {
  const sets = [
    ['1234', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample'), 'tools/fusion_sample_1234.json', 'tools/import_baseline_1234.json'],
    ['5678', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample2'), 'tools/fusion_sample_5678.json', 'tools/import_baseline_5678.json'],
    ['4242', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample4242'), 'tools/fusion_sample_4242.json', 'tools/import_baseline_4242.json'],
    ['9001', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample9001'), 'tools/fusion_sample_9001.json', 'tools/import_baseline_9001.json'],
  ];
  for (const [tag, dir, manifest, bl] of sets) {
    if (!existsSync(dir)) { results.push({ name: `import${tag}`, status: 'skip', reason: 'cache absent' }); continue; }
    const out = path.join(tmp, `import${tag}.json`);
    const r = await run('node', [path.join(root, 'tools/import_sample_check.mjs'), dir, '--wasm', wasm, '--manifest', path.join(root, manifest), '--json', out], 180000);
    const baseObj = baseline(bl);
    const rows = readJson(out);
    if (rows) results.push({ name: `import${tag}`, status: 'ok', sec: r.sec, rows, ...compare(rows, baseObj, 'file') });
    else if (baseObj.knownHang) results.push({ name: `import${tag}`, status: 'known', sec: r.sec, reason: baseObj.knownHang });
    else results.push({ name: `import${tag}`, status: 'error', sec: r.sec, reason: (r.detail || r.status) });
  }
} else results.push({ name: 'import', status: 'skip', reason: 'quick' });

// DeepCAD
const dc = path.join(os.homedir(), '.cache/b-rep-corpora/deepcad/cases1000.json');
if (!quick && existsSync(dc)) {
  const out = path.join(tmp, 'deepcad.json');
  const r = await run('node', [path.join(root, 'tools/deepcad_check.mjs'), '--cases', dc, '--wasm', wasm, '--scale', '100', '--json', out], 240000);
  const rows = readJson(out);
  if (rows) results.push({ name: 'deepcad', status: 'ok', sec: r.sec, rows, ...compare(rows, baseline('tools/deepcad_baseline.json'), 'name') });
  else results.push({ name: 'deepcad', status: 'error', sec: r.sec, reason: (r.detail || r.status) });
} else results.push({ name: 'deepcad', status: 'skip', reason: quick ? 'quick' : 'cases absent' });

if (accept) {
  const sweepBase = {
    fillet: 'tools/fillet_sweep_baseline.json', shell: 'tools/shell_sweep_baseline.json',
    boolean: 'tools/boolean_sweep_baseline.json', shape: 'tools/shape_sweep_baseline.json',
    massprops: 'tools/massprops_sweep_baseline.json', step: 'tools/step_roundtrip_baseline.json',
    tess: 'tools/tess_sweep_baseline.json', features: 'tools/feature_sweep_baseline.json',
  };
  for (const r of results) {
    if (r.status !== 'ok' || !r.rows) continue;
    if (sweepBase[r.name]) {
      const cases = Object.fromEntries(r.rows.map((x) => [x.id, x.cls]));
      writeFileSync(path.join(root, sweepBase[r.name]), JSON.stringify({ version: 1, wasm, wasmHash, seed: r.seed, samples: r.samples, cases }, null, 1) + '\n');
    }
  }
  const occt = results.find((r) => r.name === 'occt' && r.rows);
  if (occt) {
    const cases = Object.fromEntries(occt.rows.map((x) => [x.name, { cls: x.cls, rel: x.rel ?? null, relVol: x.relVol ?? null }]));
    writeFileSync(path.join(root, 'tools/occt_baseline.json'), JSON.stringify({ version: 1, wasm, wasmHash, cases }, null, 1) + '\n');
  }
  const importFiles = { import1234: ['tools/import_baseline_1234.json'], import5678: ['tools/import_baseline_5678.json'], import4242: ['tools/import_baseline_4242.json'], import9001: ['tools/import_baseline_9001.json'] };
  for (const [name, [file]] of Object.entries(importFiles)) {
    const r = results.find((x) => x.name === name && x.rows);
    if (!r) continue;
    const old = baseline(file);
    const cases = Object.fromEntries(r.rows.map((x) => [x.file, { cls: x.cls, detail: x.detail }]));
    writeFileSync(path.join(root, file), JSON.stringify({ version: 1, wasm, wasmHash, date: new Date().toISOString().slice(0, 10), ...(old.knownHang ? { knownHang: old.knownHang } : {}), cases }, null, 1) + '\n');
  }
  const dc = results.find((r) => r.name === 'deepcad' && r.rows);
  if (dc) writeFileSync(path.join(root, 'tools/deepcad_baseline.json'), JSON.stringify({ version: 1, scale: 100, wasmHash, cases: Object.fromEntries(dc.rows.map((x) => [x.name, x.cls])) }, null, 1) + '\n');
  const cf = results.find((r) => r.name === 'crashfuzz' && r.rows);
  if (cf) {
    const old = baseline('tools/crash_fuzz_baseline.json');
    writeFileSync(path.join(root, 'tools/crash_fuzz_baseline.json'), JSON.stringify({ version: 1, wasm, wasmHash, seed: 7, n: cf.rows.length, cases: Object.fromEntries(cf.rows.map((x) => [x.id, x.cls])), minimal: old.minimal ?? [] }, null, 1) + '\n');
  }
  // fillet second seed (not part of the normal page)
  const f11 = await runSweep({ name: 'fillet11', file: 'tools/fillet_sweep.mjs', base: 'tools/fillet_sweep_baseline_seed11.json', timeout: 180 });
  if (f11.status === 'ok') writeFileSync(path.join(root, 'tools/fillet_sweep_baseline_seed11.json'), JSON.stringify({ version: 1, wasm, wasmHash, seed: 11, samples: f11.samples, cases: Object.fromEntries(f11.rows.map((x) => [x.id, x.cls])) }, null, 1) + '\n');
  console.log(`accepted baselines for wasm ${wasmHash}`);
  bad = 0;
}

let bad = 0;
console.log(`verify_wasm ${quick ? 'quick' : 'full'}  ${(Date.now() - t0) / 1000}s`);
console.log('check       cases pass fail  changed  classes');
for (const r of results) {
  if (r.status === 'skip') { console.log(`${r.name.padEnd(10)} skip: ${r.reason}`); continue; }
  if (r.status === 'known') { console.log(`${r.name.padEnd(10)} KNOWN: ${r.reason}`); continue; }
  if (r.status !== 'ok') { console.log(`${r.name.padEnd(10)} ERROR: ${r.reason ?? r.status}`); continue; }
  const fails = r.failures.length;
  const classes = Object.entries(r.classes).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k}=${v}`).join(' ');
  console.log(`${r.name.padEnd(10)} ${String(r.n).padStart(5)} ${String(r.passN).padStart(4)} ${String(r.failN).padStart(4)}  ${String(fails).padStart(5)}+${r.passes.length}  ${classes}${r.note ? '  (' + r.note + ')' : ''}`);
  for (const f of r.failures.slice(0, 10)) console.log(`  FAIL ${f.id}: ${f.from} -> ${f.to}${f.detail ? ' ' + String(f.detail).slice(0, 80) : ''}`);
  for (const f of (r.unresolved ?? []).slice(0, 5)) console.log(`  UNRES ${f.id}: ${f.from} -> ${f.to}`);
  if (r.failures.length > 10) console.log(`  ... ${r.failures.length - 10} more`);
  for (const p of r.passes.slice(0, 6)) console.log(`  NEW  ${p.id}: ${p.from} -> ${p.to}`);
  if (r.passes.length > 6) console.log(`  ... ${r.passes.length - 6} more new passes`);
  bad += fails;
}
console.log(bad ? `VERDICT: FAIL (${bad} regressions)` : 'VERDICT: OK');
rmSync(tmp, { recursive: true, force: true });
process.exit(bad ? 1 : 0);
