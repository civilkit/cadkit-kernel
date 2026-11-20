// Regenerate every sample part of the kit (or any directory of .ckpart files) through a kernel wasm and record
// time, feature errors, volume and validity. Run it before and after a kernel change and compare: a regression
// that makes one sample error, change volume or run much slower is caught here (a past change made swept-handle 79 s).
//
//   node tools/sample_sweep.mjs --wasm <wasm> --dir <samples dir> --out run.json [--only name,name]
//   node tools/sample_sweep.mjs --compare before.json after.json [--slower 3] [--min-slower 5] [--vol 1e-6]
//                                [--max-load 8] [--accepted tools/ci/sweep_volume_accepted.json]
//
// Times are CPU seconds of the process. Compare exit code 1 when a sample that is in both files
//  - errors (and did not before), or turns invalid,
//  - changes volume by more than --vol (relative) and the change is not listed in the accepted file, or
//  - is more than --slower times slower (default 3) AND at least --min-slower seconds slower (default 5).
// Samples only in the second file ("new") are listed and never count. Timing verdicts are skipped when the
// one-minute load average recorded in the second file (key "_meta") was above --max-load (default 8): a box that
// is busy with other jobs inflates CPU time too (cache and SMT contention).
// The accepted file maps sample -> {from, to, reason}: an audited volume change from -> to is not a regression.
import { readFileSync, readdirSync, writeFileSync, existsSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const here = path.dirname(fileURLToPath(import.meta.url));
const near = (x, y, tol = 1e-6) => Math.abs(x - y) <= tol * Math.max(Math.abs(y), 1e-12);

if (args.includes('--compare')) {
  const [fa, fb] = args.slice(args.indexOf('--compare') + 1, args.indexOf('--compare') + 3);
  const A = JSON.parse(readFileSync(fa, 'utf8')), B = JSON.parse(readFileSync(fb, 'utf8'));
  const slower = +opt('--slower', 3), minSlower = +opt('--min-slower', 5), volTol = +opt('--vol', 1e-6), maxLoad = +opt('--max-load', 8);
  const accFile = opt('--accepted', path.join(here, 'ci/sweep_volume_accepted.json'));
  const accepted = existsSync(accFile) ? JSON.parse(readFileSync(accFile, 'utf8')) : {};
  const load = B._meta?.load1;
  const skipTiming = load != null && load > maxLoad;
  if (skipTiming) console.log(`timing skipped: box load ${(+load).toFixed(1)} (limit ${maxLoad})`);
  let bad = 0, n = 0, nNew = 0, nAcc = 0, nSlow = 0;
  for (const name of Object.keys(B).filter((k) => !k.startsWith('_'))) {
    const a = A[name], b = B[name];
    if (!a) { nNew++; console.log(`${name}: new (not counted)`); continue; }
    n++;
    const notes = [];
    if (b.errors.length && JSON.stringify(b.errors) !== JSON.stringify(a.errors)) notes.push(`ERRORS ${JSON.stringify(b.errors)} (was ${JSON.stringify(a.errors)})`);
    const dv = Math.abs(b.volume - a.volume) / Math.max(Math.abs(a.volume), 1e-12);
    if (!(dv <= volTol)) {
      const ok = accepted[name];
      if (ok && near(a.volume, ok.from) && near(b.volume, ok.to)) { nAcc++; console.log(`${name}: volume change accepted (${a.volume} -> ${b.volume}): ${ok.reason}`); }
      else notes.push(`VOLUME ${a.volume} -> ${b.volume} (${dv.toExponential(2)})`);
    }
    if (b.valid !== a.valid) notes.push(`VALID ${a.valid} -> ${b.valid}`);
    if (b.seconds > a.seconds * slower && b.seconds - a.seconds > minSlower) {
      if (skipTiming) nSlow++; else notes.push(`SLOWER ${a.seconds.toFixed(2)}s -> ${b.seconds.toFixed(2)}s cpu`);
    }
    if (notes.length) { bad++; console.log(`${name}: ${notes.join('; ')}`); }
  }
  const sum = (X) => Object.entries(X).filter(([k]) => !k.startsWith('_')).reduce((t, [, r]) => t + r.seconds, 0);
  console.log(`${bad ? 'FAIL' : 'ok'}: ${n} compared, ${bad} regressions, ${nAcc} accepted volume changes, ${nNew} new (not counted)` +
    `${skipTiming ? `, ${nSlow} slower samples ignored (box load)` : ''}; total cpu ${sum(A).toFixed(1)}s -> ${sum(B).toFixed(1)}s`);
  process.exit(bad ? 1 : 0);
}

const wasm = opt('--wasm'), dir = opt('--dir'), out = opt('--out'), only = opt('--only')?.split(',');
const mod = new WebAssembly.Module(readFileSync(wasm));
const res = {};
let loadMax = os.loadavg()[0];
for (const f of readdirSync(dir).filter((x) => x.endsWith('.ckpart')).sort()) {
  const name = f.replace(/\.ckpart$/, '');
  if (only && !only.includes(name)) continue;
  let part;
  try { part = JSON.parse(readFileSync(path.join(dir, f), 'utf8')); } catch { continue; }
  const k = new Kernel(mod);
  const q = (id, msg) => { const r = k.request(JSON.stringify({ id, msg })); return r.trap ? { trap: r.trap } : JSON.parse(r.json); };
  // CPU time of this process (the wasm is single threaded): immune to a loaded machine, wall time is kept beside it
  const c0 = process.cpuUsage(), t0 = process.hrtime.bigint();
  const reg = q(1, { type: 'regen', tag: 'main', part });
  const c1 = process.cpuUsage(c0);
  const seconds = (c1.user + c1.system) / 1e6, wall = Number(process.hrtime.bigint() - t0) / 1e9;
  const row = { seconds, wall, errors: [], volume: NaN, valid: null };
  if (reg.trap) row.errors = [`TRAP ${reg.trap.slice(0, 100)}`];
  else {
    row.errors = (reg.result?.errors ?? []).map(([id, e]) => `${id}:${e.code}`);
    if (reg.result?.body) {
      const mp = q(2, { type: 'massProps', tag: 'main' }).result;
      row.volume = mp?.volume ?? NaN;
      row.valid = (q(3, { type: 'validate', tag: 'main' }).result?.issues?.length ?? -1) === 0;
    }
  }
  res[name] = row;
  loadMax = Math.max(loadMax, os.loadavg()[0]);
  console.log(`${name.padEnd(40)} ${seconds.toFixed(2).padStart(7)}s cpu (${wall.toFixed(1)}s wall) vol ${row.volume} ${row.valid === false ? 'INVALID ' : ''}${row.errors.join(',')}`);
}
res._meta = { load1: loadMax, cpus: os.cpus().length, kernel: path.basename(wasm) };
if (out) writeFileSync(out, JSON.stringify(res, null, 1));
