// Scripted agent benchmark over the first 10 bench parts (offline: the stub
// replays the recorded translations; swap in a real Model for live runs).
// usage: node packages/bench.mjs [--first N] [--wasm path]
// (build first: npm --prefix packages run build)
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadTasks, replayModel, runAgentBench } from './dist/session/src/bench.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const opt = (f) => { const i = args.indexOf(f); return i < 0 ? null : args[i + 1]; };
if (opt('--wasm')) process.env.CK_WASM = opt('--wasm');

const tasks = loadTasks({ first: opt('--first') ? +opt('--first') : 10 });
const docs = new Map(tasks.map((t) => {
  const f = path.join(here, '..', 'tools/bench/benchcad', `${t.id}.ckpart.json`);
  try {
    return [t.id, JSON.parse(readFileSync(f, 'utf8'))];
  } catch {
    return [t.id, { features: [] }];
  }
}));
const report = await runAgentBench({ tasks, model: replayModel(docs), log: (l) => console.log(l) });
console.log('by family: ' + Object.entries(report.byFamily).map(([f, s]) => `${f} ${s.pass}/${s.total}`).join(' '));
const n = report.results.filter((r) => r.pass).length;
console.log(`agent-bench ${n}/${report.results.length}`);
process.exit(n === report.results.length ? 0 : 1);
