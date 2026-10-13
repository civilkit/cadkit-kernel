// Cost of opts.sketchStatus on regen: BenchCAD parts, regen with vs without the option
// (interleaved, median of N runs each). Budget: < 10% added over the whole set.
// usage: node tools/bench/sketch_status_timing.mjs [--wasm path] [--runs N] [--option sketchStatus|solveSketches]
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kernel } from '../kernel.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const dir = path.join(here, 'benchcad');
const args = process.argv.slice(2);
const opt = (f, d) => { const i = args.indexOf(f); return i < 0 ? d : args[i + 1]; };
const mod = new WebAssembly.Module(readFileSync(opt('--wasm', path.join(here, '../../target/wasm32-unknown-unknown/fast/ck_wasm.wasm'))));
const runs = Number(opt('--runs', 5));
const optName = opt('--option', 'sketchStatus'); // or solveSketches
const median = (a) => [...a].sort((x, y) => x - y)[Math.floor(a.length / 2)];
const manifest = JSON.parse(readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
let totA = 0, totB = 0, n = 0;
console.log(`${'part'.padEnd(26)} ${'sketches'.padStart(8)} ${'base_ms'.padStart(8)} ${'status_ms'.padStart(9)} ${'delta'.padStart(7)}`);
for (const m of manifest.parts) {
  if (manifest.known?.[m.id]?.skip) continue;
  const part = JSON.parse(readFileSync(path.join(dir, `${m.id}.ckpart.json`), 'utf8'));
  const one = (opts) => {
    const k = new Kernel(mod);
    const t0 = performance.now();
    const r = k.request(JSON.stringify({ id: 1, msg: { type: 'regen', tag: 'main', part, ...(opts ? { opts } : {}) } }));
    return [performance.now() - t0, r.trap ? null : JSON.parse(r.json).result];
  };
  const a = [], b = [];
  let sk = 0, trap = false;
  for (let i = 0; i < runs; i++) {
    const [ta, ra] = one(null), [tb, rb] = one({ [optName]: true });
    if (!ra || !rb) { trap = true; break; }
    a.push(ta); b.push(tb); sk = rb.sketches.length;
  }
  if (trap) continue;
  const ma = median(a), mb = median(b);
  totA += ma; totB += mb; n++;
  console.log(`${m.id.padEnd(26)} ${String(sk).padStart(8)} ${ma.toFixed(0).padStart(8)} ${mb.toFixed(0).padStart(9)} ${(((mb - ma) / ma) * 100).toFixed(1).padStart(6)}%`);
}
console.log(`\n${n} parts: base ${totA.toFixed(0)} ms, with status ${totB.toFixed(0)} ms, added ${(((totB - totA) / totA) * 100).toFixed(2)}%`);
