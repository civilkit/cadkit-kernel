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
