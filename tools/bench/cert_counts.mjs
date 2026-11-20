// The gate's bench counts (benchcad, benchcad.step, brepjs.step) per wasm, and the parts whose verdict differs from the first wasm.
// usage: node tools/bench/cert_counts.mjs name=path [name=path ...]
import { runBench } from './run.mjs';
const ws = process.argv.slice(2).map((a) => a.split('='));
let base = null;
for (const [name, wasm] of ws) {
  const t0 = performance.now();
  const bc = [...runBench({ wasm })], bj = [...runBench({ wasm, set: 'brepjs' })];
  const cnt = (rows, f) => { const r = rows.filter((x) => f(x) !== null); return `${r.filter(f).length}/${r.length}`; };
  const verdict = new Map([...bc.map((r) => [`benchcad ${r.id}`, `${r.pass}/${r.import}`]), ...bj.map((r) => [`brepjs ${r.id}`, `${r.import}`])]);
  console.log(`${name}: bench.benchcad ${cnt(bc, (r) => r.pass)}  benchcad.step ${cnt(bc, (r) => r.import)}  brepjs.step ${cnt(bj, (r) => r.import)}  (${((performance.now() - t0) / 1000).toFixed(0)} s)`);
  if (base) for (const [k, v] of verdict) if (base.get(k) !== v) console.log(`  differs from ${ws[0][0]}: ${k} ${base.get(k)} -> ${v}`);
  base ??= verdict;
}
