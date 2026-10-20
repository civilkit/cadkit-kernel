import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Session, checkOp, referencers, refCheckNeeded, hasNameRefs, isCosmeticOp } from './session.js';
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

  it('bodyForView shows a sketch-only part, and still returns null for an empty document', async () => {
    const frame = { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
    const withSketch = { async request(msg: Record<string, unknown>) { if (msg.type === 'regen') return { ok: true, result: { errors: [], body: null, sketches: [['F1', { frame, sketch: { type: 'sketch', id: 'F1' } }]] } }; throw new Error('unexpected'); } } as KernelTransport;
    const empty = { async request(msg: Record<string, unknown>) { if (msg.type === 'regen') return { ok: true, result: { errors: [], body: null } }; throw new Error('unexpected'); } } as KernelTransport;
    const v = await new Session(plateDoc(), withSketch, 't').bodyForView();
    assert.ok(v, 'a sketch-only part has a view');
    assert.equal(v!.mesh.positions.length, 0);
    assert.equal(v!.sketches.length, 1);
    assert.equal(await new Session(plateDoc(), empty, 't').bodyForView(), null);
  });

  it('a preview with no solid yet still returns its sketches', async () => {
    const frame = { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
    const kernel: KernelTransport = {
      async request(msg: Record<string, unknown>) {
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: null, sketches: [['F1', { frame, sketch: { type: 'sketch', id: 'F1' } }]] } };
        throw new Error('unexpected ' + String(msg.type));
      },
    };
    const s = new Session(plateDoc(), kernel, 't');
    const r = await s.preview([], { upTo: 1 });
    assert.equal(r.ok, true);
    assert.equal(r.body, null);
    assert.equal(r.sketches?.length, 1);
    assert.equal(r.sketches?.[0].id, 'F1');
  });

  it('preview forwards opts.sketchStatus and returns the per-sketch status', async () => {
    const seen: Array<Record<string, unknown>> = [];
    const status = { dof: 0, fullyDefined: true, conflicts: [], redundant: [], solved: true, state: 'full', residual: 0 };
    const kernel: KernelTransport = {
      async request(msg: Record<string, unknown>) {
        seen.push(msg);
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: cleanBody, sketches: [['F1', { frame: { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] }, sketch: { type: 'sketch', id: 'F1' }, ...((msg.opts as { sketchStatus?: boolean } | undefined)?.sketchStatus ? { status } : {}) }]] } };
        throw new Error('unexpected ' + String(msg.type));
      },
    };
    const s = new Session(plateDoc(), kernel, 't');
    const r = await s.preview([], { upTo: 1, sketchStatus: true });
    assert.deepEqual(seen.find((m) => m.type === 'regen')?.opts, { upTo: 1, sketchStatus: true });
    assert.deepEqual(r.body?.sketches[0]?.status, status);
    const plain = await s.preview([]);
    assert.equal(plain.body?.sketches[0]?.status, undefined);
  });

  it('preview forwards opts.solveSketches to the kernel', async () => {
    const seen: Array<Record<string, unknown>> = [];
    const kernel: KernelTransport = {
      async request(msg: Record<string, unknown>) {
        seen.push(msg);
        if (msg.type === 'regen') return { ok: true, result: { errors: [], body: cleanBody, sketches: [] } };
        throw new Error('unexpected ' + String(msg.type));
      },
    };
    const s = new Session(plateDoc(), kernel, 't');
    await s.preview([], { upTo: 2, solveSketches: true });
    assert.deepEqual(seen.find((m) => m.type === 'regen')?.opts, { upTo: 2, solveSketches: true });
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

/** A plate, a sketch and a fillet that name faces of it, a second block on its cap and a last fillet: references at several
 *  depths of the history (the Rust test refdiff_cache has the same part). */
function refDoc(): ReturnType<typeof emptyPart> {
  const sk = (id: string, pts: Array<[string, number, number]>, ents: unknown[], plane: unknown) => ({ type: 'sketch', id, plane, points: pts.map(([i, x, y]) => ({ id: i, x, y })), entities: ents, constraints: [] });
  const rect = [{ type: 'line', id: 'e1', p1: 'p1', p2: 'p2' }, { type: 'line', id: 'e2', p1: 'p2', p2: 'p3' }, { type: 'line', id: 'e3', p1: 'p3', p2: 'p4' }, { type: 'line', id: 'e4', p1: 'p4', p2: 'p1' }];
  return {
    ...emptyPart('R'),
    features: [
      sk('F1', [['p1', 0, 0], ['p2', 40, 0], ['p3', 40, 20], ['p4', 0, 20]], rect, { kind: 'std', id: 'top' }),
      { type: 'extrude', id: 'F2', op: 'add', sketch: 'F1', end: 'blind', depth: 10 },
      sk('F3', [['a', 10, 10]], [{ type: 'circle', id: 'e1', c: 'a', r: 3 }], { kind: 'face', id: 'F2.C1' }),
      { type: 'fillet', id: 'F4', radius: 1, edges: [{ faces: ['F2.C1', 'F2.S.e1'], at: [20, 0, 0] }] },
      sk('F5', [['b', 30, 10]], [{ type: 'circle', id: 'e1', c: 'b', r: 4 }], { kind: 'face', id: 'F2.C1' }),
      { type: 'extrude', id: 'F6', op: 'add', sketch: 'F5', end: 'blind', depth: 5 },
      { type: 'fillet', id: 'F7', radius: 0.5, edges: [{ faces: ['F2.C1', 'F2.S.e3'], at: [20, 20, 0] }] },
    ],
    seq: 8,
  } as unknown as ReturnType<typeof emptyPart>;
}

describe('deferred reference check', () => {
  it('knows which features name faces and edges', () => {
    assert.equal(hasNameRefs({ plane: { kind: 'face', id: 'F2.C1' } }), true);
    assert.equal(hasNameRefs({ edges: [{ faces: ['a', 'b'], at: [1, 2, 3] }] }), true);
    assert.equal(hasNameRefs({ sketch: 'F1', depth: 10, id: 'F2' }), false);
  });

  it('is skipped when no later feature names a face or edge, and when only appending', () => {
    const d = refDoc();
    const set = (id: string, path: string, value: unknown) => [{ op: 'set', id, path, value }] as never;
    const same = (ops: never) => refCheckNeeded(d, d, ops);
    assert.equal(same(set('F7', 'radius', 0.6)), false, 'last feature');
    assert.equal(same(set('F6', 'depth', 6)), true, 'F7 names faces after it');
    assert.equal(same(set('F2', 'depth', 12)), true);
    assert.equal(same([{ op: 'remove', id: 'F6' }] as never), true);
    assert.equal(same([{ op: 'move', id: 'F7', to: 2 }] as never), true);
    assert.equal(same([{ op: 'add', feature: { type: 'fillet' } }] as never), false, 'append');
    assert.equal(same([{ op: 'add', feature: { type: 'fillet' }, index: 2 }] as never), true);
  });

  it('is skipped for edits of labels only (folders, names)', () => {
    const d = refDoc();
    const f2 = d.features.find((f) => f.id === 'F2') as unknown as Record<string, unknown>;
    const same = (ops: never) => refCheckNeeded(d, d, ops);
    assert.equal(same([{ op: 'set', id: 'F2', path: 'group', value: 'Folder' }, { op: 'set', id: 'F3', path: 'group', value: 'Folder' }] as never), false);
    assert.equal(same([{ op: 'set', id: 'F2', path: 'name', value: 'Base' }, { op: 'set', id: 'F2', path: 'autoName', value: false }] as never), false);
    const { group: _g, ...rest } = { ...f2, group: 'Folder' };
    assert.equal(same([{ op: 'replace', id: 'F2', feature: rest }] as never), false, 'ungroup is a replace without the key');
    assert.equal(same([{ op: 'replace', id: 'F2', feature: { ...rest, depth: 99 } }] as never), true, 'a replace that changes more');
    assert.equal(same([{ op: 'set', id: 'F2', path: 'group', value: 'F' }, { op: 'set', id: 'F2', path: 'depth', value: 3 }] as never), true, 'mixed');
    assert.equal(same([{ op: 'set', id: 'F2', path: 'suppressed', value: true }] as never), true, 'suppressed is not a label');
    assert.equal(isCosmeticOp(d, { op: 'remove', id: 'F2' } as never), false);
  });

  function spy(): { k: KernelTransport; log: string[]; refDiffs: Array<{ base: unknown; part: unknown }> } {
    const log: string[] = [];
    const refDiffs: Array<{ base: unknown; part: unknown }> = [];
    const inner = fake();
    return { log, refDiffs, k: { request: async (m) => { log.push(m.type as string); if (m.type === 'refDiff') refDiffs.push({ base: m.base, part: m.part }); return inner.request(m); } } };
  }
  const lean = { lean: true };
  const edit6 = [{ op: 'set', id: 'F6', path: 'depth', value: 6 }] as never;

  it('does not run for an edit it cannot say anything about', async () => {
    const t = spy();
    const s = new Session(refDoc(), t.k);
    const r = await s.edit([{ op: 'set', id: 'F7', path: 'radius', value: 0.6 }], undefined, lean);
    assert.equal(r.refWarningsLater, undefined);
    assert.equal(t.refDiffs.length, 0);
  });

  it('waits for the session to be idle, then runs once', async () => {
    const t = spy();
    const s = new Session(refDoc(), t.k);
    s.refCheckIdleMs = 400;
    const r = await s.edit(edit6, undefined, lean);
    assert.ok(r.refWarningsLater);
    await new Promise((x) => setTimeout(x, 150));
    assert.equal(t.refDiffs.length, 0, 'not before the idle time');
    assert.deepEqual(await r.refWarningsLater, []);
    assert.equal(t.refDiffs.length, 1);
  });

  it('is dropped when anything reaches the kernel while it waits', async () => {
    const t = spy();
    const s = new Session(refDoc(), t.k);
    s.refCheckIdleMs = 400;
    const r = await s.edit(edit6, undefined, lean);
    await new Promise((x) => setTimeout(x, 100));
    await s.preview([{ op: 'set', id: 'F2', path: 'depth', value: 11 }]);
    assert.deepEqual(await r.refWarningsLater, []);
    await new Promise((x) => setTimeout(x, 600));
    assert.equal(t.refDiffs.length, 0, 'never started behind new work');
  });

  it('can be asked for on demand, and a non-lean edit still returns it', async () => {
    const t = spy();
    const s = new Session(refDoc(), t.k);
    await s.edit(edit6, undefined, lean);
    assert.deepEqual(await s.refWarnings(), []);
    assert.equal(t.refDiffs.length, 1);
    assert.equal((t.refDiffs[0].base as { features: Array<{ depth?: number }> }).features[5].depth, 5);
    assert.equal((t.refDiffs[0].part as { features: Array<{ depth?: number }> }).features[5].depth, 6);
    const t2 = spy();
    const s2 = new Session(refDoc(), t2.k);
    const r = await s2.edit([{ op: 'set', id: 'F7', path: 'radius', value: 0.6 }]);
    assert.equal(t2.refDiffs.length, 1, 'non-lean: always, in the edit');
    assert.deepEqual(r.refWarnings, []);
    const t3 = spy();
    const r3 = await new Session(refDoc(), t3.k).edit(edit6, undefined, { refCheck: 'none' });
    assert.equal(t3.refDiffs.length, 0);
    assert.deepEqual(r3.refWarnings, []);
  });
});

describe('skipped reference checks have no warnings', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('matches the full path on edits, removals and moves', async () => {
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    const k = await createWorkerKernel();
    try {
      const base = refDoc();
      const warnings = async (b: unknown, p: unknown, noCache: boolean) => {
        const r = await k.request({ type: 'refDiff', base: b, part: p, ...(noCache ? { noCache: true } : {}) });
        assert.equal(r.ok, true);
        return (r.result as { warnings: unknown[] }).warnings;
      };
      const cases: Array<[string, EditOpT[]]> = [
        ['last radius', [{ op: 'set', id: 'F7', path: 'radius', value: 0.6 }]],
        ['F6 depth', [{ op: 'set', id: 'F6', path: 'depth', value: 7 }]],
        ['F4 radius', [{ op: 'set', id: 'F4', path: 'radius', value: 1.4 }]],
        ['F2 depth', [{ op: 'set', id: 'F2', path: 'depth', value: 14 }]],
        ['F5 circle', [{ op: 'set', id: 'F5', path: 'points', value: [{ id: 'b', x: 25, y: 10 }] }]],
        ['plane', [{ op: 'set', id: 'F1', path: 'plane', value: { kind: 'std', id: 'front' } }]],
        ['suppress F6', [{ op: 'set', id: 'F6', path: 'suppressed', value: true }]],
        ['remove F2', [{ op: 'remove', id: 'F2', cascade: false }]],
        ['move F7', [{ op: 'move', id: 'F7', to: 2 }]],
        ['two sets', [{ op: 'set', id: 'F7', path: 'radius', value: 0.7 }, { op: 'set', id: 'F4', path: 'radius', value: 1.2 }]],
      ];
      let skipped = 0, needed = 0;
      for (const [name, ops] of cases) {
        const s = new Session(base, { request: async () => ({ ok: true, result: { errors: [], body: cleanBody } }) } as KernelTransport);
        const before = s.document;
        const applied = await s.edit(ops as never).catch(() => null);
        if (!applied?.ok) continue;
        const part = s.document;
        await k.request({ type: 'regen', tag: 'a', part: before });
        await k.request({ type: 'regen', tag: 'a', part });
        const full = await warnings(before, part, true);
        if (!refCheckNeeded(before, part, ops as never)) {
          skipped++;
          assert.deepEqual(full, [], `${name}: skipped but the full path warns`);
        } else {
          needed++;
          assert.deepEqual(await warnings(before, part, false), full, `${name}: cached path differs`);
        }
      }
      assert.ok(skipped >= 1 && needed >= 5, `${skipped} skipped, ${needed} checked`);
    } finally {
      await k.close();
    }
  });
});
type EditOpT = import('./session.js').EditOp;
