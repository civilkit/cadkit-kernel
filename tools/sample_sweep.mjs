// Regenerate every sample part of the kit (or any directory of .ckpart files) through a kernel wasm and record
// time, feature errors, volume and validity. Run it before and after a kernel change and compare: a regression
// that makes one sample error, change volume or run much slower is caught here (a past change made swept-handle 79 s).
//
//   node tools/sample_sweep.mjs --wasm <wasm> --dir <samples dir> --out run.json [--only name,name]
//   node tools/sample_sweep.mjs --compare before.json after.json [--slower 1.2] [--vol 1e-6]
//
// Compare exit code 1 when a sample errors (that did not before), its volume changes by more than --vol (relative),
// or it is more than --slower times slower (default 1.2) and slower by at least 0.3 s.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };

if (args.includes('--compare')) {
  const [fa, fb] = args.slice(args.indexOf('--compare') + 1, args.indexOf('--compare') + 3);
  const A = JSON.parse(readFileSync(fa, 'utf8')), B = JSON.parse(readFileSync(fb, 'utf8'));
  const slower = +opt('--slower', 1.2), volTol = +opt('--vol', 1e-6);
  let bad = 0;
  for (const name of Object.keys(B)) {
    const a = A[name], b = B[name];
    if (!a) { console.log(`${name}: new`); continue; }
    const notes = [];
    if (b.errors.length && JSON.stringify(b.errors) !== JSON.stringify(a.errors)) notes.push(`ERRORS ${JSON.stringify(b.errors)} (was ${JSON.stringify(a.errors)})`);
    const dv = Math.abs(b.volume - a.volume) / Math.max(Math.abs(a.volume), 1e-12);
    if (!(dv <= volTol)) notes.push(`VOLUME ${a.volume} -> ${b.volume} (${dv.toExponential(2)})`);
    if (b.valid !== a.valid) notes.push(`VALID ${a.valid} -> ${b.valid}`);
    if (b.seconds > a.seconds * slower && b.seconds - a.seconds > 0.3) notes.push(`SLOWER ${a.seconds.toFixed(2)}s -> ${b.seconds.toFixed(2)}s`);
    if (notes.length) { bad++; console.log(`${name}: ${notes.join('; ')}`); }
  }
  const ta = Object.values(A).reduce((s, r) => s + r.seconds, 0), tb = Object.values(B).reduce((s, r) => s + r.seconds, 0);
  console.log(`${Object.keys(B).length} samples, ${bad} regressions; total ${ta.toFixed(1)}s -> ${tb.toFixed(1)}s`);
  process.exit(bad ? 1 : 0);
}

const wasm = opt('--wasm'), dir = opt('--dir'), out = opt('--out'), only = opt('--only')?.split(',');
const mod = new WebAssembly.Module(readFileSync(wasm));
const res = {};
for (const f of readdirSync(dir).filter((x) => x.endsWith('.ckpart')).sort()) {
  const name = f.replace(/\.ckpart$/, '');
  if (only && !only.includes(name)) continue;
  let part;
  try { part = JSON.parse(readFileSync(path.join(dir, f), 'utf8')); } catch { continue; }
  const k = new Kernel(mod);
  const q = (id, msg) => { const r = k.request(JSON.stringify({ id, msg })); return r.trap ? { trap: r.trap } : JSON.parse(r.json); };
  const t0 = process.hrtime.bigint();
  const reg = q(1, { type: 'regen', tag: 'main', part });
  const seconds = Number(process.hrtime.bigint() - t0) / 1e9;
  const row = { seconds, errors: [], volume: NaN, valid: null };
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
  console.log(`${name.padEnd(40)} ${seconds.toFixed(2).padStart(7)}s vol ${row.volume} ${row.valid === false ? 'INVALID ' : ''}${row.errors.join(',')}`);
}
if (out) writeFileSync(out, JSON.stringify(res, null, 1));
