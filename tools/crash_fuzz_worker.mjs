// One fuzz case per worker: a trap/hang only kills this worker.
import { parentPort, workerData } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { Kernel } from './kernel.mjs';
const { wasm, features } = workerData;
const mod = new WebAssembly.Module(readFileSync(wasm));
const k = new Kernel(mod);
const ask = (id, msg) => {
  const r = k.request(JSON.stringify({ id, msg }));
  if (r.trap) return { trap: r.trap };
  try { return JSON.parse(r.json); } catch { return { trap: 'unparsable reply' }; }
};
const part = { format: 'ck-part', version: 1, features };
const last = features[features.length - 1]?.type ?? 'none';
try {
  const r = ask(1, { type: 'regen', tag: 'main', part, opts: { solveSketches: true } });
  if (r.trap) parentPort.postMessage({ cls: 'trap', trap: r.trap, last });
  else {
    const m = ask(2, { type: 'massProps', tag: 'main', bodies: true });
    const v = ask(3, { type: 'validate', tag: 'main' });
    parentPort.postMessage({ cls: 'ok', errors: (r.result?.errors ?? []).map(([, e]) => e.code), volume: m.result?.volume, area: m.result?.area, issues: (v.result?.issues ?? []).map((x) => x.code), last });
  }
} catch (e) { parentPort.postMessage({ cls: 'trap', trap: String(e), last }); }
