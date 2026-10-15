// Regenerate every kit sample through a wasm build and compare two runs (regression sweep for speed work).
//   node tools/sample_sweep_pair.mjs run --wasm w.wasm --out a.json [--samples <dir>] [--only id,id] [--timeout 240]
//   node tools/sample_sweep_pair.mjs cmp old.json new.json     # volumes within 1e-6, no error, no sample >20 % slower
// Each sample runs in a child process on a fresh instance (noCache, cold build), so a hang cannot stall the sweep.
import { readFileSync, writeFileSync, readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from './kernel.mjs';

const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const DEF = '/path/to/work/kit/apps/cad/public/samples';

if (args[0] === 'child') {
  const [, wasm, file, repsArg] = args;
  const B = new WebAssembly.Module(readFileSync(wasm));
  const part = JSON.parse(readFileSync(file, 'utf8'));
  // `reps` cold builds, each on a fresh instance of the compiled module; the minimum is reported (the
  // dev machine is shared, so one sample is noisy); the last instance answers massProps / validate
  let k, rep, o, build = Infinity, wall = Infinity;
  for (let r = 0; r < +(repsArg ?? 1); r++) {
    k = new Kernel(B, { now: () => performance.now(), unixMs: () => Date.now() });
    const t0 = performance.now();
    rep = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, debug: true, opts: { noCache: true } } }));
    const w = performance.now() - t0;
    if (rep.trap) { console.log(JSON.stringify({ trap: rep.trap.slice(0, 160), wall: w })); process.exit(0); }
    o = JSON.parse(rep.json);
    wall = Math.min(wall, w);
    for (const s of o.trace ?? []) if (s.depth === 0 && s.label.startsWith('regenerate')) build = Math.min(build, s.ms);
    if (w > 4000) break; // slow parts: fewer repeats
  }
  const q = (id, msg) => { const r = k.request(JSON.stringify({ id, msg })); return r.trap ? null : JSON.parse(r.json).result; };
  const mp = q(2, { type: 'massProps', tag: 'main' });
  const v = q(3, { type: 'validate', tag: 'main' });
  console.log(JSON.stringify({ wall, build, errors: (o.result?.errors ?? []).map(([f, e]) => `${f}:${e.code}`).join(','),
    body: !!o.result?.body, volume: mp?.volume ?? null, area: mp?.area ?? null, issues: v?.issues?.length ?? null }));
  process.exit(0);
}

if (args[0] === 'run') {
  args.shift();
  const wasm = path.resolve(opt('--wasm')); const out = opt('--out'); const dir = opt('--samples', DEF);
  const only = opt('--only'); const reps = opt('--reps', '3'); const timeout = +opt('--timeout', 240) * 1000;
  const here = fileURLToPath(import.meta.url);
  let files = readdirSync(dir).filter((f) => f.endsWith('.ckpart')).sort();
  if (only) files = files.filter((f) => only.split(',').includes(f.replace(/\.ckpart$/, '')));
  // `--wasm2 w2 --out2 o2` times a second build in the same pass, alternating per sample (load drift hits both)
  const w2 = opt('--wasm2');
  const wasm2 = w2 ? path.resolve(w2) : null;
  const out2 = opt('--out2');
  const one = (w, f) => {
    const r = spawnSync('node', [here, 'child', w, path.join(dir, f), reps], { encoding: 'utf8', timeout, maxBuffer: 1 << 28 });
    try { return JSON.parse(r.stdout.trim().split('\n').pop()); } catch { return { trap: r.error ? 'timeout' : 'crash ' + (r.stderr || '').slice(0, 100) }; }
  };
  const res = {}, res2 = {};
  for (const f of files) {
    const id = f.replace(/\.ckpart$/, '');
    const x = (res[id] = one(wasm, f));
    const y = wasm2 ? (res2[id] = one(wasm2, f)) : null;
    const fm = (z) => (z.trap ? 'TRAP ' + z.trap : `${(z.build ?? z.wall).toFixed(0).padStart(7)} ms vol ${z.volume} ${z.errors || ''}`);
    console.error(id.padEnd(40), fm(x), y ? ' | ' + fm(y) : '');
  }
  if (wasm2) writeFileSync(out2, JSON.stringify(res2, null, 1));
  writeFileSync(out, JSON.stringify(res, null, 1));
  process.exit(0);
}

if (args[0] === 'cmp') {
  const a = JSON.parse(readFileSync(args[1], 'utf8')); const b = JSON.parse(readFileSync(args[2], 'utf8'));
  let bad = 0, sa = 0, sb = 0;
  for (const id of Object.keys(a)) {
    const x = a[id], y = b[id]; if (!y) continue;
    const msgs = [];
    if (y.trap && !x.trap) msgs.push('NEW TRAP ' + y.trap);
    if ((y.errors || '') !== (x.errors || '')) msgs.push(`errors ${x.errors || '-'} -> ${y.errors || '-'}`);
    if (x.volume != null && y.volume != null) {
      const rel = Math.abs(x.volume - y.volume) / Math.max(1e-9, Math.abs(x.volume));
      if (rel > 1e-6) msgs.push(`volume ${x.volume} -> ${y.volume} (${rel.toExponential(2)})`);
    } else if ((x.volume == null) !== (y.volume == null)) msgs.push('volume presence changed');
    if ((y.issues ?? 0) > (x.issues ?? 0)) msgs.push(`issues ${x.issues} -> ${y.issues}`);
    const ta = x.build ?? x.wall, tb = y.build ?? y.wall;
    sa += ta; sb += tb;
    const slow = tb > ta * 1.2 && tb - ta > 15;
    if (slow) msgs.push('SLOWER');
    console.log(`${id.padEnd(40)} ${ta.toFixed(0).padStart(7)} -> ${tb.toFixed(0).padStart(7)} ms  ${(tb / ta).toFixed(2)}x  ${msgs.join('; ')}`);
    if (msgs.length) bad++;
  }
  console.log(`total ${sa.toFixed(0)} -> ${sb.toFixed(0)} ms; ${bad} flagged`);
  process.exit(bad ? 1 : 0);
}
console.error('usage: run|cmp');
process.exit(2);
