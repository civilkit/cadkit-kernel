import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { angles, check, clearance, closestPoint, describe as describeNames, distance, exportMesh, extrema, heal, importSat, importStep, massProps, maxDistance, probe, resolveDescriptions, section, sections, symmetry, thickness, thicknessAt, validate } from './queries.js';
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

describe('exportMesh', () => {
  it('sends only the options given, and turns the buffers into a mesh', async () => {
    const seen: Record<string, unknown>[] = [];
    const buf = (type: string, data: ArrayLike<number>) => ({ $buf: 0, type, len: data.length, data: type === 'f32' ? Float32Array.from(data) : Int32Array.from(data) });
    const k = fake({ chord: 0.5, angle: 0.6, chordRaised: false, triangles: 1, faces: ['F1.a', 'F1.b'], failed: [], positions: buf('f32', [0, 0, 0, 1, 0, 0, 0, 1, 0]), triFace: buf('i32', [1]) }, seen);
    const r = await exportMesh(k, { quality: 'coarse', chord_mm: 0.5 }, { tag: 's1' });
    assert.deepEqual(seen[0], { type: 'exportMesh', tag: 's1', quality: 'coarse', chord_mm: 0.5 });
    assert.equal(r.chord, 0.5);
    assert.equal(r.triangles, 1);
    assert.deepEqual(Array.from(r.mesh.triFace), [1]);
    assert.deepEqual(r.mesh.faceIds, ['F1.a', 'F1.b']);
    assert.equal(r.mesh.positions.length, 9);
    await exportMesh(k);
    assert.deepEqual(seen[1], { type: 'exportMesh', tag: 'main' });
  });
  it('throws on a refusal and on no body', async () => {
    await assert.rejects(exportMesh(failing('error')), /nope/);
    await assert.rejects(exportMesh(failing('null')), /empty result/);
  });
});

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

  it('section / sections / probe send the documented messages', async () => {
    const seen: Record<string, unknown>[] = [];
    await section(fake({ loops: [], area: 0 }, seen), { origin: [0, 0, 1], normal: [0, 0, 1] }, { tag: 't' });
    await sections(fake({ areas: [] }, seen), { axis: 'z', n: 5 });
    await probe(fake({ points: [] }, seen), { points: [[0, 0, 0]] });
    assert.deepEqual(seen, [
      { type: 'section', tag: 't', plane: { origin: [0, 0, 1], normal: [0, 0, 1] } },
      { type: 'sections', tag: 'main', axis: 'z', n: 5 },
      { type: 'probe', tag: 'main', points: [[0, 0, 0]] },
    ]);
  });

  it('measurement wrappers send the documented messages', async () => {
    const seen: Record<string, unknown>[] = [];
    await distance(fake({ distance: 0 }, seen), 'F2.C0', [1, 2, 3], { tagB: 'o', samples: 9 });
    await maxDistance(fake({ maxDistance: 0 }, seen), { tagB: 'o', both: true, samples: 100 }, { tag: 't' });
    await thickness(fake({ thin: [] }, seen), { method: 'sphere', top: 3 });
    await thicknessAt(fake({}, seen), { point: [0, 0, 0] });
    await closestPoint(fake({ results: [] }, seen), [[0, 0, 0]], { on: 'edge', k: 2 });
    await angles(fake({}, seen), { edges: 'all', draft: { pull: [0, 0, 1], min: 1 } });
    await clearance(fake({ pairs: [] }, seen), { min: 2, include: 'gap', limit: 5 });
    await symmetry(fake({ planes: [] }, seen), { samples: 500 });
    await extrema(fake({}, seen), { dir: [0, 0, 1], of: 'F2.C1', samples: 8 });
    assert.deepEqual(seen, [
      { type: 'distance', tag: 'main', a: 'F2.C0', b: [1, 2, 3], tagB: 'o', opts: { samples: 9 } },
      { type: 'maxDistance', tag: 't', tagB: 'o', opts: { samples: 100, both: true } },
      { type: 'thickness', tag: 'main', opts: { method: 'sphere', top: 3 } },
      { type: 'thicknessAt', tag: 'main', point: [0, 0, 0] },
      { type: 'closestPoint', tag: 'main', points: [[0, 0, 0]], on: 'edge', k: 2 },
      { type: 'angles', tag: 'main', edges: 'all', draft: { pull: [0, 0, 1], min: 1 } },
      { type: 'clearance', tag: 'main', min: 2, opts: { include: 'gap', limit: 5 } },
      { type: 'symmetry', tag: 'main', opts: { samples: 500 } },
      { type: 'extrema', tag: 'main', dir: [0, 0, 1], of: 'F2.C1', opts: { samples: 8 } },
    ]);
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
      // verification requests: 10 x 5 x 2 plate, volume 100
      const st = await s.sections({ axis: 'x', n: 21 });
      assert.equal(st.areas.length, 21);
      assert.ok((st.volumeError ?? 1) < 1e-6, `volumeError ${st.volumeError}`);
      assert.ok(Math.abs(st.areas[10] - 10) < 1e-6);
      const sec = await s.section({ origin: [5, 2.5, 0], normal: [1, 0, 0] });
      assert.ok(Math.abs(sec.area - 10) < 1e-6);
      assert.equal(sec.loops.length, 1);
      assert.equal(sec.loops[0].kind, 'outer');
      assert.ok(sec.loops[0].edges.every((e) => e.type === 'line'));
      const empty = await s.section({ origin: [500, 0, 0], normal: [1, 0, 0] });
      assert.equal(empty.loops.length, 0);
      assert.equal(empty.area, 0);
      // the plate extrudes to one side of the sketch plane: exactly one of the two mirror points is inside
      const pr = await s.probe({ points: [[5, 2.5, 1], [5, 2.5, -1], [50, 2.5, 0]], rays: [{ origin: [5, 2.5, -10], dir: [0, 0, 1] }] });
      const ins = [pr.points?.[0].inside, pr.points?.[1].inside];
      assert.equal(ins.filter((x) => x === true).length, 1, JSON.stringify(ins));
      const dIn = pr.points?.[ins[0] === true ? 0 : 1].distance ?? 0;
      assert.ok(Math.abs(dIn + 1) < 1e-6, `distance ${dIn}`);
      assert.equal(pr.points?.[2].inside, false);
      assert.equal(pr.rays?.[0].hits.length, 2);
      assert.equal(pr.rays?.[0].hits[0].entering, true);
      // measurement requests on the 10 x 5 x 2 plate
      const th = await s.thickness();
      assert.ok(Math.abs((th.minThickness ?? 0) - 2) < 1e-6, `thickness ${th.minThickness}`);
      const tsph = await s.thickness({ method: 'sphere' });
      assert.ok(Math.abs((tsph.minThickness ?? 0) - 2) < 1e-5);
      const an = await s.angles({ edges: 'all' });
      assert.equal(an.edges?.length, 12);
      assert.ok(an.edges?.every((e) => Math.abs(e.angle - 90) < 1e-6));
      const ds = await s.distance([5, 2.5, 20], 'body');
      assert.ok(Math.abs(ds.distance - 18) < 1e-9 || Math.abs(ds.distance - 22) < 1e-9, `distance ${ds.distance}`);
      const cl = await s.clearance({ min: 3 });
      assert.equal(cl.pairs.filter((x) => x.kind === 'wall').length, 1);
      const mx = await s.maxDistance();
      assert.ok(mx.maxDistance < 1e-9);
      const ex = await s.extrema({ dir: [1, 0, 0] });
      assert.ok(Math.abs(ex.extent - 10) < 1e-9 && ex.exact, JSON.stringify(ex));
      assert.equal(ex.max.entity?.kind, 'face');
      const ez = await s.extrema({ dir: [0, 0, 1] });
      assert.ok(Math.abs(ez.extent - 2) < 1e-9);
      const sy = await s.symmetry();
      assert.ok(sy.symmetric.length >= 3);
      const cp = await s.closestPoint([[100, 100, 100]], { on: 'vertex' });
      assert.equal(cp.results.length, 1);
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
