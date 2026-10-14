// OCCT-in-wasm timing of the brepjs playground "mechanical" examples (the kit's brepjs-* samples).
// usage: node tools/bench/brepjs_times.mjs <run-dir> <out.json> [--runs 5] [--only id,id]
// <run-dir> is the scratch layout described in tools/bench/brepjs_ref/README.txt (brepjs + occt-wasm installed,
// ex/<id>.mjs = the unmodified examples). Each example module builds its shape at import time, so a build is
// one dynamic import of a cache-busted URL (module evaluation re-runs; the wasm module stays initialised).
// 1 warmup, N timed runs; output has the same shape as occt_times.py (ms, min_ms, cold_ms, volume, valid).
import { writeFileSync, readFileSync, readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const opt = (f, d = null) => { const i = args.indexOf(f); if (i < 0) return d; const v = args[i + 1]; args.splice(i, 2); return v; };
const runs = +opt('--runs', 5);
const only = opt('--only')?.split(',');
const timeout = +opt('--timeout', 240) * 1000;

if (process.env.BREPJS_CHILD) {
  // child, run inside <run-dir> so that 'brepjs/quick' resolves
  const id = process.env.BREPJS_CHILD;
  const B = await import('brepjs/quick');
  const t0 = performance.now();
  let mod = await import(`./ex/${id}.mjs?r=0`);
  const cold = performance.now() - t0;
  const ts = [];
  for (let r = 1; r <= runs + 1; r++) {
    const t = performance.now();
    mod = await import(`./ex/${id}.mjs?r=${r}`);
    ts.push(performance.now() - t);
  }
  const timed = ts.slice(1).sort((a, b) => a - b);
  let items = Array.isArray(mod.default) ? mod.default : [mod.default];
  items = items.map((x) => (x && x.__color !== undefined ? x.shape : x));
  let volume = 0, valid = true, faces = 0;
  for (const s of items) { volume += B.unwrap(B.measureVolumeProps(s)).volume; valid = valid && B.isValid(s); faces += B.getFaces(s).length; }
  const n = timed.length;
  console.log('RESULT ' + JSON.stringify({ id, cold_ms: cold, ms: n % 2 ? timed[(n - 1) / 2] : (timed[n / 2 - 1] + timed[n / 2]) / 2, min_ms: timed[0], runs_ms: ts.slice(1), volume, valid, faces }));
  process.exit(0);
}

const runDir = args[0], out = args[1];
const ids = readdirSync(path.join(runDir, 'ex')).filter((f) => f.endsWith('.mjs')).map((f) => f.slice(0, -4)).sort().filter((i) => !only || only.includes(i));
const man = new Set(JSON.parse(readFileSync(path.join(path.dirname(fileURLToPath(import.meta.url)), 'brepjs/manifest.json'), 'utf8')).parts.map((p) => p.id));
const res = {};
// the child must live in <run-dir> (module resolution): copy this very file there
const self = path.join(runDir, '_brepjs_child.mjs');
writeFileSync(self, readFileSync(fileURLToPath(import.meta.url)));
for (const id of ids.filter((i) => man.has(i))) {
  const r = spawnSync(process.execPath, [self, '--runs', String(runs)], { cwd: runDir, env: { ...process.env, BREPJS_CHILD: id }, encoding: 'utf8', timeout, maxBuffer: 1 << 26 });
  const line = (r.stdout ?? '').split('\n').find((l) => l.startsWith('RESULT '));
  res[id] = line ? JSON.parse(line.slice(7)) : { id, error: `child failed ${r.status ?? r.error?.code} ${(r.stderr ?? '').split('\n').filter((l) => /rror/.test(l))[0] ?? ''}`.slice(0, 200) };
  console.log(id, res[id].ms?.toFixed(1) ?? res[id].error, res[id].volume);
  writeFileSync(out, JSON.stringify(res, null, 1));
}
