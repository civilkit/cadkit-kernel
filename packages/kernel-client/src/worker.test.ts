import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { createWorkerKernel, defaultWasmPath } from './worker.js';
import { Part } from '../../builder/src/index.js';

function plate() {
  const p = new Part('Plate', 'aluminium');
  const base = p.sketch('Outline', 'front', (s) => s.rect(0, 0, 100, 60));
  p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
  return p.toJSON();
}

describe('worker kernel', { skip: !existsSync(defaultWasmPath()) && 'no wasm built' }, () => {
  it('boots and regens the plate with no errors', async () => {
    const k = await createWorkerKernel();
    try {
      assert.ok(k.commit.length > 0);
      const r = await k.request({ type: 'regen', tag: 't', part: plate() });
      assert.equal(r.ok, true);
      const result = r.result as { errors: unknown[]; body: { faces: unknown[] } };
      assert.deepEqual(result.errors, []);
      assert.ok(result.body.faces.length > 0);
    } finally {
      await k.close();
    }
  });

  it('reports the missing-sketch error shape', async () => {
    const k = await createWorkerKernel();
    try {
      const bad = { ...plate(), features: [{ type: 'extrude', op: 'add', sketch: 'F99', end: 'blind', depth: 1, id: 'F1' }] };
      const r = await k.request({ type: 'regen', tag: 't', part: bad });
      const result = r.result as { errors: Array<[string, { code: string }]> };
      assert.equal(result.errors[0][0], 'F1');
      assert.equal(result.errors[0][1].code, 'missingSketch');
    } finally {
      await k.close();
    }
  });
});

describe('worker kernel, missing wasm', () => {
  it('names the path and the fix instead of a vague worker error', async () => {
    await assert.rejects(createWorkerKernel('/nonexistent/ck_wasm.wasm'), /kernel wasm not found at \/nonexistent\/ck_wasm\.wasm.*CK_WASM/s);
  });
});

describe('worker kernel, worker dies after startup', { skip: !existsSync(defaultWasmPath()) && 'no wasm built' }, () => {
  // the worker's thread is reached through the Worker the kernel owns: terminate it from outside
  // by making the host exit mid-call (a request type the host cannot answer is not needed: exit via env hook below)
  it('answers a waiting call and every later call with a trap instead of hanging', async () => {
    const { Worker } = await import('node:worker_threads');
    const orig = Worker.prototype.postMessage;
    let killed = false;
    Worker.prototype.postMessage = function (this: InstanceType<typeof Worker>, ...a: Parameters<typeof orig>) {
      const m = a[0] as { kind?: string };
      if (m?.kind === 'call' && !killed) { killed = true; void this.terminate(); return; }
      return orig.apply(this, a);
    } as typeof orig;
    const k = await createWorkerKernel();
    try {
      const first = await Promise.race([
        k.request({ type: 'regen', tag: 't', part: plate() }),
        new Promise<never>((_, rej) => setTimeout(() => rej(new Error('hung')), 10000)),
      ]);
      assert.equal(first.ok, false);
      assert.match(first.trap ?? '', /worker exited/);
      const later = await k.request({ type: 'regen', tag: 't', part: plate() });
      assert.equal(later.ok, false);
      assert.match(later.trap ?? '', /worker exited/);
    } finally {
      Worker.prototype.postMessage = orig;
      await k.close();
    }
  });
});

describe('PendingCalls', () => {
  it('fails everything waiting once, and answers later calls at once', async () => {
    const { PendingCalls } = await import('./pending.js');
    const p = new PendingCalls();
    const a = p.open();
    p.fail('gone');
    p.fail('second reason ignored');
    assert.deepEqual(await a.reply, { ok: false, trap: 'gone' });
    assert.deepEqual(await p.open().reply, { ok: false, trap: 'gone' });
    assert.equal(p.size, 0);
  });
});

describe('hostScriptPath', () => {
  it('finds the compiled host', async () => {
    const { hostScriptPath } = await import('./worker.js');
    assert.ok(existsSync(hostScriptPath()));
  });
});
