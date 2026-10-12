import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadTasks, replayModel, runAgentBench } from './bench.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');
const wasm = path.join(ROOT, 'target/wasm32-unknown-unknown/release/ck_wasm.wasm');

function docOf(id: string) {
  return JSON.parse(readFileSync(path.join(ROOT, 'tools/bench/benchcad', `${id}.ckpart.json`), 'utf8')) as { features: Array<Record<string, unknown>> };
}

describe('agent bench', { skip: !existsSync(wasm) && 'no wasm built' }, () => {
  it('loads tasks with ground truth', () => {
    const tasks = loadTasks({ first: 10 });
    assert.equal(tasks.length, 10);
    for (const t of tasks) {
      assert.ok(t.description.includes(t.id));
      assert.ok(t.volume > 0);
    }
  });

  it('replay passes, drop-last fails', async () => {
    const tasks = loadTasks({ first: 2 });
    const docs = new Map(tasks.map((t) => [t.id, docOf(t.id)]));
    const pass = await runAgentBench({ wasmPath: wasm, tasks, model: replayModel(docs) });
    assert.ok(pass.results.every((r) => r.pass), JSON.stringify(pass.results.map((r) => [r.id, r.detail])));
    const broken = new Map(tasks.map((t) => [t.id, { features: docOf(t.id).features.slice(0, -1) }]));
    const fail = await runAgentBench({ wasmPath: wasm, tasks, model: replayModel(broken) });
    assert.ok(fail.results.every((r) => !r.pass));
    assert.ok(Object.keys(fail.byFamily).length > 0);
  });
});
