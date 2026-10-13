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
    assert.match((await s.edit([{ op: 'move', id: 'F2', to: 0 }])).schemaError ?? '', /referencing later feature F1/);
    assert.match((await s.edit([{ op: 'move', id: 'F2', to: 9 }])).schemaError ?? '', /out of range/);
    assert.equal((await s.edit([{ op: 'replace', id: 'F2', feature: { type: 'fillet', radius: 2, edges: [] } }])).ok, true);
    assert.equal(s.document.features[1].id, 'F2');
    assert.equal(s.document.features[1].type, 'fillet');
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

  it('preview forwards opts.upTo to the regen request, and omits opts when absent', async () => {
    const seen: Array<Record<string, unknown>> = [];
    const kernel: KernelTransport = {
      async request(msg: Record<string, unknown>) {
        seen.push(msg);
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: cleanBody } };
        throw new Error('unexpected ' + String(msg.type));
      },
    };
    const s = new Session(plateDoc(), kernel, 't');
    await s.preview([], { upTo: 1 });
    const regen = seen.find((m) => m.type === 'regen');
    assert.deepEqual(regen?.opts, { upTo: 1 });

    seen.length = 0;
    await s.preview([]);
    const bare = seen.find((m) => m.type === 'regen');
    assert.equal(bare !== undefined && 'opts' in bare, false);
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

describe('session bodyForView', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('an empty document returns null', async () => {
    const s = await Session.open(emptyPart('Empty'));
    try {
      assert.equal(await s.bodyForView(), null);
    } finally {
      await s.close();
    }
  });

  it('a plate returns a mesh with edges and its sketch frame', async () => {
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate');
    const base = p.sketch('S', 'front', (sk) => sk.rect(0, 0, 10, 10));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 5 });
    const s = await Session.open(p.toJSON());
    try {
      const v = await s.bodyForView();
      assert.ok(v, 'expected a body');
      assert.ok(v.mesh.positions.length > 0);
      assert.ok(v.edges.length > 0);
      assert.ok(v.box.max.length === 3);
      assert.equal(v.sketches.length, 1);
      assert.equal(v.sketches[0].id, 'F1');
      assert.equal(v.sketches[0].sketch.entities.length, 4);
      assert.deepEqual(v.sketches[0].frame.o, [0, 0, 0]);
      assert.deepEqual(v.curves, []);
      assert.deepEqual(v.planes, []);
      assert.deepEqual(v.markers, []);
    } finally {
      await s.close();
    }
  });

  it('a helixed part returns the helix curve and datum planes', async () => {
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Spring');
    const prof = p.sketch('C', 'front', (sk) => { sk.circle(0, 0, 4); });
    p.helix({ sketch: prof, pitch: 2, revolutions: 2, height: 4 });
    p.offsetPlane('top', 10, 'P');
    p.extrude({ sketch: prof, op: 'add', end: 'blind', depth: 5 });
    const s = await Session.open(p.toJSON());
    try {
      const v = await s.bodyForView();
      assert.ok(v, 'expected a body');
      assert.ok(v.curves.length >= 1, 'expected the helix curve');
      assert.ok(v.curves[0].pts.length > 2);
      assert.equal(v.curves[0].pts[0].length, 3);
      assert.equal(v.planes.length, 1, 'expected the datum plane');
      assert.equal(v.planes[0].id, 'F3');
      assert.equal(v.planes[0].frame.x.length, 3);
      assert.equal(v.sketches.length, 1, 'only the circle sketch, not the helix');
    } finally {
      await s.close();
    }
  });
});

describe('session preview', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  async function plate() {
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate');
    const base = p.sketch('S', 'front', (sk) => sk.rect(0, 0, 10, 10));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 5 });
    return Session.open(p.toJSON());
  }

  it('leaves the document and history() untouched', async () => {
    const s = await plate();
    try {
      const docBefore = JSON.stringify(s.document);
      const histBefore = JSON.stringify(s.history());

      const r = await s.preview([{ op: 'set', id: 'F2', path: 'depth', value: 12 }]);

      assert.equal(r.ok, true);
      assert.ok(r.body, 'expected a preview body');
      assert.ok(r.body.mesh.positions.length > 0);
      assert.equal(JSON.stringify(s.document), docBefore);
      assert.equal(JSON.stringify(s.history()), histBefore);
      // the session tag still answers for the un-previewed document
      assert.ok((await s.facts()).faces.length > 0);
    } finally {
      await s.close();
    }
  });

  it('a failing op reports the error and the hint', async () => {
    const s = await plate();
    try {
      const r = await s.preview([
        {
          op: 'add',
          feature: {
            type: 'sketch', name: 'Far', id: 'F9', plane: { kind: 'std', id: 'front' },
            points: [
              { id: 'O', x: 100, y: 0, fixed: true },
              { id: 'p1', x: 100, y: 0 },
              { id: 'p2', x: 110, y: 0 },
              { id: 'p3', x: 110, y: 10 },
              { id: 'p4', x: 100, y: 10 },
            ],
            entities: [
              { id: 'e5', type: 'line', p1: 'p1', p2: 'p2' },
              { id: 'e6', type: 'line', p1: 'p2', p2: 'p3' },
              { id: 'e7', type: 'line', p1: 'p3', p2: 'p4' },
              { id: 'e8', type: 'line', p1: 'p4', p2: 'p1' },
            ],
            constraints: [],
          },
        },
        { op: 'add', feature: { type: 'extrude', op: 'cut', sketch: 'F9', end: 'blind', depth: 5 } },
      ]);

      assert.equal(r.ok, false);
      assert.equal(r.body, null);
      assert.ok(r.errors.length > 0, 'expected regen errors');
      assert.ok(r.hints.some((h) => /cuts nothing/.test(h.hint)), 'expected the cutNoEffect hint');
    } finally {
      await s.close();
    }
  });

  it('a preview followed by the real edit gives the same body', async () => {
    const s = await plate();
    try {
      const ops = [{ op: 'set', id: 'F2', path: 'depth', value: 12 } as const];
      const pr = await s.preview(ops);
      const er = await s.edit([...ops]);
      assert.equal(er.ok, true);
      assert.ok(pr.body);
      const v = await s.bodyForView();
      assert.ok(v);
      assert.deepEqual(Array.from(v.mesh.positions), Array.from(pr.body.mesh.positions));
      assert.deepEqual(v.edges, pr.body.edges);
    } finally {
      await s.close();
    }
  });

  it('two quick previews: the first result is dropped', async () => {
    const s = await plate();
    try {
      const depthBefore = (s.document.features.find((f) => f.id === 'F2') as { depth: number }).depth;
      const p1 = s.preview([{ op: 'set', id: 'F2', path: 'depth', value: 7 }]);
      const p2 = s.preview([{ op: 'set', id: 'F2', path: 'depth', value: 12 }]);
      const [r1, r2] = await Promise.all([p1, p2]);

      assert.equal(r1.stale, true);
      assert.equal(r1.body, null, 'the superseded preview carries no body');
      assert.notEqual(r2.stale, true);
      assert.ok(r2.body, 'the newest preview carries the body');
      assert.equal((s.document.features.find((f) => f.id === 'F2') as { depth: number }).depth, depthBefore);
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
