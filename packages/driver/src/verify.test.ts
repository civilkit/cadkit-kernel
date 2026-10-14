import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { verify, hintFor, type KernelTransport } from './verify.js';
import { defaultWasmPath as wasmPath } from '../../kernel-client/src/worker.js';
import type { KernelReply } from '../../kernel-client/src/protocol.js';
import { emptyPart } from '../../part-model/src/index.js';

function fake(replies: Record<string, unknown>): KernelTransport {
  return {
    async request(msg: Record<string, unknown>): Promise<KernelReply> {
      const r = replies[msg.type as string];
      if (r instanceof Error) throw r;
      return r as KernelReply;
    },
  };
}

const part = emptyPart('T');

describe('verify', () => {
  it('returns done with no hints on the clean path', async () => {
    const k = fake({
      regen: { ok: true, result: { errors: [], body: {} } },
      validate: { ok: true, result: { ok: true, issues: [] } },
      check: { ok: true, result: { ok: true, passed: 1, failed: 0, results: [{ index: 0, kind: 'valid', pass: true }] } },
    });
    const r = await verify(k, part);
    assert.equal(r.ok, true);
    assert.deepEqual(r.hints, []);
    assert.equal(r.validate?.ok, true);
    assert.equal(r.check?.failed, 0);
  });

  it('points regen errors at the feature with fix text', async () => {
    const k = fake({
      regen: { ok: true, result: { errors: [['F2', { code: 'missingSketch', params: {} }]], body: {} } },
      validate: { ok: true, result: null },
      check: { ok: true, result: null },
    });
    const r = await verify(k, part);
    assert.equal(r.ok, false);
    assert.equal(r.hints.length, 1);
    assert.equal(r.hints[0].feature, 'F2');
    assert.equal(r.hints[0].code, 'missingSketch');
    assert.match(r.hints[0].hint, /F2\.sketch/);
  });

  it('maps validate issues to owning features', async () => {
    const k = fake({
      regen: { ok: true, result: { errors: [], body: {} } },
      validate: { ok: true, result: { ok: false, issues: [{ code: 'openShell', entity: 'F2.C0', detail: 'gap' }] } },
      check: { ok: true, result: { ok: true, passed: 1, failed: 0, results: [] } },
    });
    const r = await verify(k, part);
    assert.equal(r.ok, false);
    assert.equal(r.hints[0].feature, 'F2');
    assert.equal(r.hints[0].code, 'openShell');
  });

  it('turns failed expectations into actionable hints', async () => {
    const k = fake({
      regen: { ok: true, result: { errors: [], body: {} } },
      validate: { ok: true, result: { ok: true, issues: [] } },
      check: { ok: true, result: { ok: false, passed: 0, failed: 1, results: [{ index: 0, kind: 'volume', pass: false, measured: 5, expected: { min: 10 }, detail: 'volume 5 vs [10, inf]' }] } },
    });
    const r = await verify(k, part, { expect: [{ kind: 'volume', min: 10 }] });
    assert.equal(r.ok, false);
    assert.equal(r.hints[0].code, 'expect:volume');
    assert.match(r.hints[0].hint, /adjust the last feature/);
  });

  it('survives traps and null bodies', async () => {
    const k = fake({ regen: { ok: false, trap: 'unreachable' } });
    const r = await verify(k, part);
    assert.equal(r.ok, false);
    assert.equal(r.hints[0].code, 'trap');
  });

  it('never reports done on an empty part', async () => {
    const k = fake({
      regen: { ok: true, result: { errors: [], body: null } },
      validate: { ok: true, result: null },
      check: { ok: true, result: null },
    });
    const r = await verify(k, part);
    assert.equal(r.ok, false);
    assert.equal(r.hints[0].code, 'noBody');
  });
});

describe('verify live', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('verifies the plate through the worker', async () => {
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate', 'aluminium');
    const base = p.sketch('Outline', 'front', (s) => s.rect(0, 0, 100, 60));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
    const k = await createWorkerKernel();
    try {
      const r = await verify(k, p.toJSON(), { expect: [{ kind: 'volume', min: 1 }, { kind: 'valid' }] });
      assert.equal(r.ok, true);
      assert.deepEqual(r.hints, []);
    } finally {
      await k.close();
    }
  });

  it('verifies a hole feature with point-entity centres', async () => {
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Disc', 'aluminium');
    const s = p.sketch('Disc', 'front', (sk) => { sk.circle(0, 0, 20); });
    p.extrude({ sketch: s, op: 'add', end: 'blind', depth: 10 });
    const c = p.sketch('Centres', 'front', (sk) => { sk.point(0, 0); });
    p.hole({ sketch: c, diameter: 8, flip: true });
    const k = await createWorkerKernel();
    try {
      const r = await verify(k, p.toJSON(), { expect: [{ kind: 'volume', equals: Math.PI * 384 * 10, relTol: 1e-3 }] });
      assert.equal(r.ok, true);
    } finally {
      await k.close();
    }
  });
});

describe('verify document expectations', () => {
  function seenFake(): { k: KernelTransport; seen: Array<Record<string, unknown>> } {
    const seen: Array<Record<string, unknown>> = [];
    const k: KernelTransport = {
      async request(msg: Record<string, unknown>): Promise<KernelReply> {
        seen.push(msg);
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: {} } };
        if (msg.type === 'validate') return { ok: true, result: { ok: true, issues: [] } };
        return { ok: true, result: { ok: true, passed: 1, failed: 0, results: [] } };
      },
    };
    return { k, seen };
  }

  it('checks the document expect array by default', async () => {
    const { k, seen } = seenFake();
    const doc = { ...emptyPart('T'), expect: [{ kind: 'volume', min: 10 }] };
    const r = await verify(k, doc);
    assert.deepEqual((seen.find((m) => m.type === 'check') as Record<string, unknown>).expect, [{ kind: 'volume', min: 10 }]);
    assert.equal(r.ok, true);
  });

  it('explicit expect wins over the document', async () => {
    const { k, seen } = seenFake();
    const doc = { ...emptyPart('T'), expect: [{ kind: 'volume', min: 10 }] };
    await verify(k, doc, { expect: [{ kind: 'valid' }] });
    assert.deepEqual((seen.find((m) => m.type === 'check') as Record<string, unknown>).expect, [{ kind: 'valid' }]);
  });
});

describe('booleanFailed hint', () => {
  it('names the two nearly coincident walls when the kernel found them', () => {
    const h = hintFor('F5', 'booleanFailed', { hostFace: 'F2.S.e1~2', toolFace: 'F4.S.e1~2', gap: 0.0008 });
    assert.ok(h.hint.includes('F4.S.e1~2') && h.hint.includes('F2.S.e1~2') && h.hint.includes('0.0008'), h.hint);
  });
  it('keeps the general advice without them', () => {
    assert.ok(hintFor('F5', 'booleanFailed', {}).hint.includes('overlap in a volume'));
  });
});
