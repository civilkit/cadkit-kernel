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
