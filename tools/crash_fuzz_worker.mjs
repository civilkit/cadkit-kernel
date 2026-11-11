// One fuzz case per worker: a trap/hang only kills this worker.
import { parentPort, workerData } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { wasm, features, budgetMs } = workerData;
const mod = new WebAssembly.Module(readFileSync(wasm));
// The kernel's clock is this thread's CPU time, not the wall clock: under load (other workers, another process) the wall
// time of a case varies by a large factor and made the count of slow cases machine-dependent. The time budgets
// (`--budget-ms`) and the parent's hang limit are therefore CPU milliseconds.
const cpu = () => { const u = process.threadCpuUsage(); return (u.user + u.system) / 1000; };
const clock = typeof process.threadCpuUsage === 'function' ? cpu : () => performance.now();
const k = new Kernel(mod, { now: clock, unixMs: () => Date.now() });
const t0 = clock();
const ask = (id, msg) => {
  const r = k.request(JSON.stringify({ id, msg }));
  if (r.trap) return { trap: r.trap };
  try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; }
};
const part = { format: 'ck-part', version: 1, features };
const last = features[features.length - 1]?.type ?? 'none';
try {
  const r = ask(1, { type: 'regen', tag: 'main', part, opts: { solveSketches: true, ...(budgetMs ? { featureTimeMs: budgetMs, regenTimeMs: budgetMs * 2 } : {}) } });
  if (r.trap) parentPort.postMessage({ cls: 'trap', trap: r.trap, last, cpuMs: clock() - t0 });
  else {
    const m = ask(2, { type: 'massProps', tag: 'main', bodies: true });
    const v = ask(3, { type: 'validate', tag: 'main' });
    parentPort.postMessage({ cls: 'ok', errors: (r.result?.errors ?? []).map(([, e]) => e.code), volume: m.result?.volume, area: m.result?.area, issues: (v.result?.issues ?? []).map((x) => x.code), last, cpuMs: clock() - t0 });
  }
} catch (e) { parentPort.postMessage({ cls: 'trap', trap: String(e), last }); }
