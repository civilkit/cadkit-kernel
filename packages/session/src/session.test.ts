import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Session, checkOp, referencers } from './session.js';
import { defaultWasmPath as wasmPath } from '../../kernel-client/src/worker.js';
import type { KernelReply } from '../../kernel-client/src/protocol.js';
import { emptyPart } from '../../part-model/src/index.js';
import type { KernelTransport } from '../../driver/src/verify.js';

const cleanBody = {
  faces: [{ id: 'F2.C0' }, { id: 'F2.S.e1' }],
  edges: [{ key: 'k1' }],
  box: { min: [0, 0, 0], max: [100, 60, 10] },
};

function fake(opts?: { errors?: Array<[string, { code: string; params: Record<string, unknown> }]> }): KernelTransport {
  return {
    async request(msg: Record<string, unknown>): Promise<KernelReply> {
      if (msg.type === 'regen') return { ok: true, result: { errors: opts?.errors ?? [], body: cleanBody } };
      if (msg.type === 'massProps') return { ok: true, result: { volume: 100, bodies: [{ volume: 100 }] } };
      if (msg.type === 'refDiff') return { ok: true, result: { warnings: [] } };
      if (msg.type === 'validate') return { ok: true, result: { ok: true, issues: [] } };
      if (msg.type === 'check') return { ok: true, result: { ok: true, passed: 1, failed: 0, results: [] } };
      throw new Error('unexpected ' + (msg.type as string));
    },
  };
}

function plateDoc() {
  return {
    ...emptyPart('P'),
    features: [
      { type: 'sketch', name: 'S', id: 'F1', plane: { kind: 'std', id: 'front' }, points: [], entities: [], constraints: [] },
      { type: 'extrude', op: 'add', sketch: 'F1', end: 'blind', depth: 10, id: 'F2' },
    ],
    seq: 3,
  } as ReturnType<typeof emptyPart>;
}

describe('session ops (pure)', () => {
  it('add assigns the next F-id; unknown types rejected', async () => {
    const s = new Session(plateDoc(), fake(), 't', 'tag');
    const r = await s.edit([{ op: 'add', feature: { type: 'fillet', radius: 1, edges: [] } }], 'round it');
    assert.equal(r.ok, true);
    assert.equal(r.record?.id, 'edit-1');
    assert.equal(r.record?.author, 't');
    assert.equal(r.record?.reason, 'round it');
    assert.equal(s.document.features.at(-1)?.id, 'F3');
    const bad = await s.edit([{ op: 'add', feature: { type: 'nope' } }]);
    assert.equal(bad.ok, false);
    assert.match(bad.schemaError ?? '', /unknown feature type/);
    assert.equal(s.document.features.length, 3);
  });

  it('set / move / replace work; id and type are protected', async () => {
    const s = new Session(plateDoc(), fake(), 't', 'tag');
    assert.equal((await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 5 }])).ok, true);
    assert.equal((s.document.features[1] as { depth: number }).depth, 5);
    assert.match((await s.edit([{ op: 'set', id: 'F9', path: 'depth', value: 5 }])).schemaError ?? '', /unknown feature/);
    assert.match((await s.edit([{ op: 'set', id: 'F2', path: 'id', value: 'F9' }])).schemaError ?? '', /cannot set/);
    assert.equal((await s.edit([{ op: 'move', id: 'F2', to: 0 }])).ok, true);
    assert.equal(s.document.features[0].id, 'F2');
    assert.match((await s.edit([{ op: 'move', id: 'F2', to: 9 }])).schemaError ?? '', /out of range/);
    assert.equal((await s.edit([{ op: 'replace', id: 'F2', feature: { type: 'fillet', radius: 2, edges: [] } }])).ok, true);
    assert.equal(s.document.features[0].id, 'F2');
    assert.equal(s.document.features[0].type, 'fillet');
    assert.equal(s.document.features[1].id, 'F1');
  });

  it('remove refuses referenced features without cascade', async () => {
    const s = new Session(plateDoc(), fake(), 't', 'tag');
    assert.deepEqual(referencers(s.document, 'F1'), ['F2']);
    const r = await s.edit([{ op: 'remove', id: 'F1' }]);
    assert.equal(r.ok, false);
    assert.match(r.schemaError ?? '', /still referenced by F2/);
    const c = await s.edit([{ op: 'remove', id: 'F1', cascade: true }], 'drop base');
    assert.equal(c.ok, true);
    assert.deepEqual(s.document.features.map((f) => f.id), []);
    assert.equal(s.history().length, 1);
  });

  it('failed batches leave the document unchanged (atomic)', async () => {
    const s = new Session(plateDoc(), fake({ errors: [['F3', { code: 'filletChamfer', params: {} }]] }), 't', 'tag');
    const before = JSON.stringify(s.document);
    const r = await s.edit([{ op: 'add', feature: { type: 'fillet', radius: 99, edges: [] } }]);
    assert.equal(r.ok, false);
    assert.equal(r.record, null);
    assert.equal(r.hints?.[0].feature, 'F3');
    assert.equal(JSON.stringify(s.document), before);
    assert.equal(s.history().length, 0);
  });

  it('undo reverts the last N batches', async () => {
    const s = new Session(plateDoc(), fake(), 't', 'tag');
    await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 5 }], 'thin');
    await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 2 }], 'thinner');
    assert.equal((s.document.features[1] as { depth: number }).depth, 2);
    const u = await s.undo(2);
    assert.deepEqual(u.records.map((r) => r.reason), ['thin', 'thinner']);
    assert.equal(u.resynced, true);
    assert.equal((s.document.features[1] as { depth: number }).depth, 10);
    assert.equal(s.history().length, 0);
  });

  it('checkOp covers the guards', () => {
    const doc = plateDoc();
    assert.match(checkOp(doc, { op: 'add', feature: { type: 'x' } }) ?? '', /unknown feature type/);
    assert.match(checkOp(doc, { op: 'add', feature: { type: 'fillet', id: 'F2' } }) ?? '', /already exists/);
    assert.match(checkOp(doc, { op: 'remove', id: 'F9' }) ?? '', /unknown feature/);
    assert.match(checkOp(doc, { op: 'replace', id: 'F2', feature: { type: 'x' } }) ?? '', /unknown feature type/);
  });
});

describe('session live', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('edits depth, diffs volume, undoes', async () => {
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate', 'aluminium');
    const base = p.sketch('Outline', 'front', (sk) => sk.rect(0, 0, 100, 60));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10, name: 'Plate' });
    const s = await Session.open(p.toJSON(), { author: 'agent' });
    try {
      const r = await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 5 }], 'halve it');
      assert.equal(r.ok, true);
      const close = (a: number | null, b: number) => assert.ok(a !== null && Math.abs(a - b) < 1e-6, `${a} ~= ${b}`);
      close(r.diff?.volume.before ?? null, 60000);
      close(r.diff?.volume.after ?? null, 30000);
      assert.deepEqual(r.refWarnings, []);
      const u = await s.undo();
      assert.equal(u.records.length, 1);
      close((await s.facts()).volume, 60000);
      const v = await s.verify();
      assert.equal(v.ok, true);
    } finally {
      await s.close();
    }
  });
});

describe('session persistence', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('saves, reopens with history, undoes and replays', async () => {
    const { mkdtempSync, readFileSync } = await import('node:fs');
    const { tmpdir } = await import('node:os');
    const dir = mkdtempSync(path.join(tmpdir(), 'ck-sess-'));
    const s = await Session.open(undefined, { author: 'agent' });
    try {
      assert.equal((await s.edit([{ op: 'add', feature: { type: 'sketch', name: 'S', plane: { kind: 'std', id: 'front' }, points: [{ id: 'O', x: 0, y: 0, fixed: true }, { id: 'p1', x: 0, y: 0 }, { id: 'p2', x: 10, y: 0 }, { id: 'p3', x: 10, y: 5 }, { id: 'p4', x: 0, y: 5 }], entities: [{ id: 'e5', type: 'line', p1: 'p1', p2: 'p2' }, { id: 'e6', type: 'line', p1: 'p2', p2: 'p3' }, { id: 'e7', type: 'line', p1: 'p3', p2: 'p4' }, { id: 'e8', type: 'line', p1: 'p4', p2: 'p1' }], constraints: [] } }], 'sketch it')).ok, true);
      assert.equal((await s.edit([{ op: 'add', feature: { type: 'extrude', op: 'add', sketch: 'F1', end: 'blind', depth: 2 } }], 'block it')).ok, true);
      const saved = await s.save(dir, 't1');
      assert.ok(saved.part.endsWith('t1.ckpart'));
      assert.ok(JSON.parse(readFileSync(saved.log, 'utf8')).entries.length === 2);

      const r = await Session.openSaved(dir, 't1');
      try {
        assert.deepEqual(r.document, s.document);
        assert.deepEqual(r.history().map((h) => [h.id, h.author, h.reason]), [['edit-1', 'agent', 'sketch it'], ['edit-2', 'agent', 'block it']]);
        const u = await r.undo();
        assert.equal(u.records.length, 1);
        assert.equal(r.document.features.length, 1);
      } finally {
        await r.close();
      }

      const fresh = await Session.open(undefined, { author: 'replay' });
      try {
        const { replaySteps } = await import('./session.js');
        const log = JSON.parse(readFileSync(saved.log, 'utf8'));
        const rep = await replaySteps(fresh, log);
        assert.equal(rep.failed, null);
        assert.equal(rep.applied, 2);
        assert.deepEqual(fresh.document, s.document);
      } finally {
        await fresh.close();
      }
    } finally {
      await s.close();
    }
  });
});

describe('session intent', () => {
  const docWithBody = () => plateDoc();

  function fake(checkResults: Array<Record<string, unknown>> = []): KernelTransport {
    return {
      async request(msg: Record<string, unknown>): Promise<KernelReply> {
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: cleanBody } };
        if (msg.type === 'massProps') return { ok: true, result: { volume: 100, bodies: [{ volume: 100 }] } };
        if (msg.type === 'refDiff') return { ok: true, result: { warnings: [] } };
        if (msg.type === 'validate') return { ok: true, result: { ok: true, issues: [] } };
        if (msg.type === 'check') {
          const failed = checkResults.filter((r) => !r.pass);
          return { ok: true, result: { ok: failed.length === 0, passed: checkResults.length - failed.length, failed: failed.length, results: checkResults } };
        }
        throw new Error('unexpected ' + (msg.type as string));
      },
    };
  }

  it('setExpect records history and survives undo', async () => {
    const s = new Session(docWithBody(), fake(), 't', 'tag');
    const rec = s.setExpect([{ kind: 'volume', min: 10 }], 'pin volume');
    assert.equal(rec.id, 'edit-1');
    assert.deepEqual(s.document.expect, [{ kind: 'volume', min: 10 }]);
    assert.equal(s.history().length, 1);
    await s.undo();
    assert.equal(s.document.expect, undefined);
  });

  it('flags edits that break document expectations without reverting', async () => {
    const s = new Session(docWithBody(), fake([{ index: 0, kind: 'volume', pass: false, measured: 5, expected: { min: 10 }, detail: 'low' }]), 't', 'tag');
    s.setExpect([{ kind: 'volume', min: 10 }]);
    const r = await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 5 }]);
    assert.equal(r.ok, true); // flagged, not reverted
    assert.equal(r.intent?.failed, 1);
    assert.equal(r.hints?.[0].code, 'expect:volume');
    assert.match(r.hints?.[0].hint ?? '', /edit-2/);
    assert.equal((s.document.features[1] as { depth: number }).depth, 5);
  });

  it('clean intent stays silent', async () => {
    const s = new Session(docWithBody(), fake([{ index: 0, kind: 'volume', pass: true }]), 't', 'tag');
    s.setExpect([{ kind: 'volume', min: 10 }]);
    const r = await s.edit([{ op: 'set', id: 'F2', path: 'depth', value: 5 }]);
    assert.equal(r.ok, true);
    assert.equal(r.intent?.failed, 0);
    assert.equal(r.hints, undefined);
  });
});
