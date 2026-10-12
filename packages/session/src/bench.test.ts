import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadTasks, replayModel, runAgentBench, runEditBench } from './bench.js';
import { defaultWasmPath as wasmPath } from '../../kernel-client/src/worker.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');

function docOf(id: string) {
  return JSON.parse(readFileSync(path.join(ROOT, 'tools/bench/benchcad', `${id}.ckpart.json`), 'utf8')) as { features: Array<Record<string, unknown>> };
}

describe('agent bench', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
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
    const pass = await runAgentBench({ tasks, model: replayModel(docs) });
    assert.ok(pass.results.every((r) => r.pass), JSON.stringify(pass.results.map((r) => [r.id, r.detail])));
    const broken = new Map(tasks.map((t) => [t.id, { features: docOf(t.id).features.slice(0, -1) }]));
    const fail = await runAgentBench({ tasks, model: replayModel(broken) });
    assert.ok(fail.results.every((r) => !r.pass));
    assert.ok(Object.keys(fail.byFamily).length > 0);
  });

  it('edit tasks pass with exact volumes', async () => {
    const { results } = await runEditBench({});
    assert.ok(results.length >= 4);
    for (const r of results) assert.equal(r.pass, true, `${r.id}: ${r.detail}`);
  });
});
