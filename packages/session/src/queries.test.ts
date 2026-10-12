import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { check, describe as describeNames, heal, importSat, importStep, massProps, resolveDescriptions, validate } from './queries.js';
import { defaultWasmPath as wasmPath } from '../../kernel-client/src/worker.js';
import type { KernelReply } from '../../kernel-client/src/protocol.js';
import type { KernelTransport } from '../../driver/src/verify.js';

function fake(result: unknown, seen: Record<string, unknown>[] = []): KernelTransport {
  return {
    async request(msg: Record<string, unknown>): Promise<KernelReply> {
      seen.push(msg);
      return { ok: true, result };
    },
  };
}

function failing(kind: 'trap' | 'error' | 'null'): KernelTransport {
  return {
    async request(): Promise<KernelReply> {
      if (kind === 'trap') return { ok: false, trap: 'boom' };
      if (kind === 'error') return { ok: false, error: 'nope' };
      return { ok: true, result: null };
    },
  };
}

describe('queries', () => {
  it('unwraps results and tags requests', async () => {
    const seen: Record<string, unknown>[] = [];
    const k = fake({ passed: 1, failed: 0, results: [] }, seen);
    const c = await check(k, [{ kind: 'valid' }], { tag: 's1' });
    assert.equal(c.passed, 1);
    assert.equal(seen[0].type, 'check');
    assert.equal(seen[0].tag, 's1');
  });

  it('check accepts the shorthand object', async () => {
    const seen: Record<string, unknown>[] = [];
    await check(fake({ passed: 0, failed: 0, results: [] }, seen), { volume: 100 });
    assert.deepEqual(seen[0].expect, { volume: 100 });
  });

  it('resolve/describe pass descriptions and names through', async () => {
    const seen: Record<string, unknown>[] = [];
    const k = fake({ results: [{ status: 'unique' }] }, seen);
    const r = await resolveDescriptions(k, [{ kind: 'face', name: 'F2.C0' }]);
    assert.equal(r[0].status, 'unique');
    const k2 = fake({ items: [{ found: true, text: 'cap' }] }, seen);
    const d = await describeNames(k2, ['F2.C0']);
    assert.equal(d[0].text, 'cap');
  });

  it('validate/heal/massProps/imports shape their replies', async () => {
    assert.deepEqual(await validate(fake({ ok: true, issues: [] })), { ok: true, issues: [] });
    assert.deepEqual((await heal(fake({ fixed: [], remaining: [], volume: { before: 1, after: 1 } }))).fixed, []);
    assert.equal((await massProps(fake({ volume: 5 }), {})).volume, 5);
    const im = await importStep(fake({ errors: [], body: null }), 'DATA');
    assert.deepEqual(im.errors, []);
    const is = await importSat(fake({ errors: [] }), 'DATA');
    assert.deepEqual(is.errors, []);
  });

  it('throws on trap, envelope error and null result', async () => {
    await assert.rejects(check(failing('trap'), []), /trap: boom/);
    await assert.rejects(check(failing('error'), []), /nope/);
    await assert.rejects(check(failing('null'), []), /empty result/);
  });
});

describe('queries live', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('runs the whole tool table on the plate', async () => {
    const { Session } = await import('./session.js');
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate');
    const base = p.sketch('Outline', 'front', (sk) => sk.rect(0, 0, 10, 5));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 2 });
    const s = await Session.open(p.toJSON());
    try {
      assert.equal((await s.validate()).ok, true);
      const c = await s.check({ volume: 100, faces: 6 });
      assert.equal(c.failed, 0);
      const d = await s.describe(['F2.C0']);
      assert.equal(d[0].found, true);
      const r = await s.resolve([{ kind: 'face', name: 'F2.C0' }]);
      assert.equal(r[0].status, 'unique');
      assert.equal((await s.heal()).remaining.length, 0);
      assert.ok(Math.abs(((await s.massProps()).volume ?? 0) - 100) < 1e-6);
      // STEP round-trip through a fresh tag
      const kernel = (s as unknown as { kernel: KernelTransport }).kernel;
      const text = await s.step();
      assert.ok(text.length > 100);
      const back = await importStep(kernel, text, { tag: 'imp2' });
      assert.deepEqual(back.errors, []);
      const m2 = await massProps(kernel, { tag: 'imp2' });
      assert.ok(Math.abs((m2.volume ?? 0) - 100) < 1e-6);
    } finally {
      await s.close();
    }
  });
});
