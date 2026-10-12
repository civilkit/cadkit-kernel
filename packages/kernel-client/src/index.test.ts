import { describe, it, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import { evalKernelInPage, type EvalArgs } from './index.js';
import { emptyPart } from '../../part-model/src/index.js';

// The page-side function reads its kernel handle from globalThis, so a plain
// stub exercises the exact same code path the browser runs — no Playwright.
function stubKernel(replies: Record<string, (msg: Record<string, unknown>) => unknown>, seen: string[]) {
  (globalThis as unknown as Record<string, unknown>).window = {
    cadkit: {
      kernel: {
        send(msg: Record<string, unknown>, cb: (v: unknown) => void) {
          seen.push(msg.type as string);
          cb(replies[msg.type as string](msg));
        },
      },
    },
  };
}

afterEach(() => {
  delete (globalThis as unknown as Record<string, unknown>).window;
});

const args: EvalArgs = { part: emptyPart('T'), mass: true, step: true, stl: true, faces: true };

const body = {
  box: { min: [0, 0, 0], max: [10, 5, 2] },
  faces: [{ id: 'F2.C0', kind: 'cap', mesh: { triangles: 99 } }, { id: 'F2.S.e1', kind: 'side' }],
  edges: [{ key: 'e1' }],
  mesh: { positions: new Float32Array(18) }, // 2 triangles
};

describe('kernel-client', () => {
  it('collects regen + mass + step + mesh on the ok path', async () => {
    const seen: string[] = [];
    stubKernel({
      regen: () => ({ errors: [], body }),
      massProps: () => ({ volume: 100, area: 50, com: [5, 2.5, 1] }),
      step: (m) => `STEP(${m.name})`,
    }, seen);
    const r = await evalKernelInPage(args);
    assert.equal(r.ok, true);
    assert.deepEqual(seen, ['regen', 'massProps', 'step']);
    assert.equal(r.triangles, 2);
    assert.equal(r.faces, 2);
    assert.equal(r.edges, 1);
    assert.deepEqual(r.box, { min: [0, 0, 0], max: [10, 5, 2] });
    assert.deepEqual(r.mass, { volume: 100, area: 50, com: [5, 2.5, 1] });
    assert.equal(r.stepText, 'STEP(T)');
    assert.equal(r.positions?.length, 18);
  });

  it('strips mesh out of the face list', async () => {
    const seen: string[] = [];
    stubKernel({ regen: () => ({ errors: [], body }), massProps: () => ({}), step: () => '' }, seen);
    const r = await evalKernelInPage(args);
    assert.equal(r.faceList?.length, 2);
    assert.ok(!('mesh' in (r.faceList?.[0] as Record<string, unknown>)));
  });

  it('reports !ok when the regen has errors', async () => {
    const seen: string[] = [];
    stubKernel({
      regen: () => ({ errors: [['F9', { code: 'missingSketch', params: {} }]], body }),
      massProps: () => ({}),
      step: () => '',
    }, seen);
    const r = await evalKernelInPage(args);
    assert.equal(r.ok, false);
    assert.equal(r.errors.length, 1);
  });

  it('reports !ok on an empty body', async () => {
    const seen: string[] = [];
    stubKernel({ regen: () => ({ errors: [], body: { empty: true } }) }, seen);
    const r = await evalKernelInPage({ ...args, mass: false, step: false, stl: false, faces: false });
    assert.equal(r.ok, false);
    assert.deepEqual(seen, ['regen']);
  });
});
