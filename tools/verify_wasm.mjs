// One-command verifier for every sweep and corpus check.
import { readFileSync, writeFileSync, existsSync, mkdtempSync, rmSync } from 'node:fs';
import { spawn } from 'node:child_process';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.join(here, '..');
const argv = process.argv.slice(2);
const wasm = argv.find((a) => !a.startsWith('--'));
const quick = argv.includes('--quick');
if (!wasm) { console.error('usage: tools/verify_wasm.sh <wasm> [--quick]'); process.exit(2); }
const tmp = mkdtempSync(path.join(os.tmpdir(), 'ck-verify-'));
const t0 = Date.now();
const PASS = new Set(['ok', 'okNoEffect', 'okEmpty', 'refused']);
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
      if (code !== 0) return resolve({ status: 'error', sec, detail: (err || out).slice(-300) });
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
function classOf(v) { return typeof v === 'string' ? v : v.cls; }

function compare(rows, base, idKey = 'id') {
  const map = base.cases ?? {};
  const classes = {};
  const failures = [], passes = [];
  for (const r of rows) {
    classes[r.cls] = (classes[r.cls] ?? 0) + 1;
    const id = r[idKey];
    const b = map[id];
    if (b == null) continue;
    const was = PASS.has(classOf(b)), now = PASS.has(r.cls);
    if (was && !now) failures.push({ id, from: classOf(b), to: r.cls, detail: r.detail });
    else if (!was && now) passes.push({ id, from: classOf(b), to: r.cls });
  }
  const passN = rows.filter((r) => PASS.has(r.cls)).length;
  return { classes, failures, passes, n: rows.length, passN, failN: rows.length - passN };
}

const SWEEPS = [
  { name: 'fillet', file: 'tools/fillet_sweep.mjs', base: 'tools/fillet_sweep_baseline.json', timeout: 180 },
  { name: 'shell', file: 'tools/shell_sweep.mjs', base: 'tools/shell_sweep_baseline.json', timeout: 120 },
  { name: 'boolean', file: 'tools/boolean_sweep.mjs', base: 'tools/boolean_sweep_baseline.json', timeout: 120 },
  { name: 'shape', file: 'tools/shape_sweep.mjs', base: 'tools/shape_sweep_baseline.json', timeout: 120 },
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
    if (r.status !== 'ok') { hang = true; break; }
    all.push(...JSON.parse(readFileSync(out, 'utf8')));
  }
  if (hang) return { name: s.name, status: 'hang', sec };
  return { name: s.name, status: 'ok', sec, ...compare(all, base) };
}
results.push(...(await mapLimit(SWEEPS, 4, runSweep)));

// OCCT corpus
const occt = '/tmp/occt';
if (existsSync(occt)) {
  const out = path.join(tmp, 'occt.json');
  const args = [path.join(root, 'tools/occt_corpus.mjs'), '--occt', occt, '--wasm', wasm, '--quiet', '--json', out];
  if (quick) args.push('--only', 'bopcut_simple');
  const r = await run('node', args, (quick ? 90 : 300) * 1000);
  if (r.status === 'ok') results.push({ name: 'occt', status: 'ok', sec: r.sec, ...compare(JSON.parse(readFileSync(out, 'utf8')), baseline('tools/occt_baseline.json'), 'name') });
  else results.push({ name: 'occt', status: r.status, sec: r.sec });
} else results.push({ name: 'occt', status: 'skip', reason: '/tmp/occt absent' });

// Fusion import (opt-in cache)
if (!quick) {
  const sets = [
    ['1234', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample'), 'tools/fusion_sample_1234.json', 'tools/import_baseline_1234.json'],
    ['5678', path.join(os.homedir(), '.cache/b-rep-corpora/fusion360/sample2'), 'tools/fusion_sample_5678.json', 'tools/import_baseline_5678.json'],
  ];
  for (const [tag, dir, manifest, bl] of sets) {
    if (!existsSync(dir)) { results.push({ name: `import${tag}`, status: 'skip', reason: 'cache absent' }); continue; }
    const out = path.join(tmp, `import${tag}.json`);
    const r = await run('node', [path.join(root, 'tools/import_sample_check.mjs'), dir, '--wasm', wasm, '--manifest', path.join(root, manifest), '--json', out], 180000);
    if (r.status === 'ok') results.push({ name: `import${tag}`, status: 'ok', sec: r.sec, ...compare(JSON.parse(readFileSync(out, 'utf8')), baseline(bl), 'file') });
    else results.push({ name: `import${tag}`, status: r.status, sec: r.sec });
  }
} else results.push({ name: 'import', status: 'skip', reason: 'quick' });

// DeepCAD
const dc = path.join(os.homedir(), '.cache/b-rep-corpora/deepcad/cases1000.json');
if (!quick && existsSync(dc)) {
  const out = path.join(tmp, 'deepcad.json');
  const r = await run('node', [path.join(root, 'tools/deepcad_check.mjs'), '--cases', dc, '--wasm', wasm, '--scale', '100', '--json', out], 240000);
  if (r.status === 'ok') results.push({ name: 'deepcad', status: 'ok', sec: r.sec, ...compare(JSON.parse(readFileSync(out, 'utf8')), baseline('tools/deepcad_baseline.json'), 'name') });
  else results.push({ name: 'deepcad', status: r.status, sec: r.sec });
} else results.push({ name: 'deepcad', status: 'skip', reason: quick ? 'quick' : 'cases absent' });

let bad = 0;
console.log(`verify_wasm ${quick ? 'quick' : 'full'}  ${(Date.now() - t0) / 1000}s`);
console.log('check       cases pass fail  changed  classes');
for (const r of results) {
  if (r.status === 'skip') { console.log(`${r.name.padEnd(10)} skip: ${r.reason}`); continue; }
  if (r.status !== 'ok') { console.log(`${r.name.padEnd(10)} ${r.status.toUpperCase()}`); bad++; continue; }
  const fails = r.failures.length;
  const classes = Object.entries(r.classes).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k}=${v}`).join(' ');
  console.log(`${r.name.padEnd(10)} ${String(r.n).padStart(5)} ${String(r.passN).padStart(4)} ${String(r.failN).padStart(4)}  ${String(fails).padStart(5)}+${r.passes.length}  ${classes}`);
  for (const f of r.failures.slice(0, 10)) console.log(`  FAIL ${f.id}: ${f.from} -> ${f.to}${f.detail ? ' ' + String(f.detail).slice(0, 80) : ''}`);
  if (r.failures.length > 10) console.log(`  ... ${r.failures.length - 10} more`);
  for (const p of r.passes.slice(0, 6)) console.log(`  NEW  ${p.id}: ${p.from} -> ${p.to}`);
  if (r.passes.length > 6) console.log(`  ... ${r.passes.length - 6} more new passes`);
  bad += fails;
}
console.log(bad ? `VERDICT: FAIL (${bad} regressions)` : 'VERDICT: OK');
rmSync(tmp, { recursive: true, force: true });
process.exit(bad ? 1 : 0);
