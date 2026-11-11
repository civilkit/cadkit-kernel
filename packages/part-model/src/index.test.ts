import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { emptyPart, isSketchFeature, type Feature } from './index.js';

describe('part-model', () => {
  it('emptyPart returns the v1 envelope', () => {
    assert.deepEqual(emptyPart('X', 'aluminium'), {
      format: 'ck-part',
      version: 1,
      name: 'X',
      features: [],
      seq: 1,
      rollback: null,
      material: 'aluminium',
      counters: {},
    });
  });

  it('isSketchFeature narrows on type', () => {
    const sk = { type: 'sketch', id: 'F1' } as Feature;
    const ex = { type: 'extrude', id: 'F2' } as Feature;
    assert.equal(isSketchFeature(sk), true);
    assert.equal(isSketchFeature(ex), false);
    if (isSketchFeature(sk)) assert.equal(sk.id, 'F1');
  });
});

describe('feature inventory', () => {
  it('covers every wire type the kernel dispatches on', async () => {
    const { FEATURE_TYPES } = await import('./features.js');
    const dispatch = ['sphere', 'sketch', 'sketch3d', 'extrude', 'revolve', 'sweep', 'loft', 'fillet', 'chamfer', 'draft', 'shell', 'hole', 'helix', 'rib', 'lip', 'lpattern', 'cpattern', 'mirror', 'trimSurface', 'surfacePlanar', 'surfaceRevolve', 'knit', 'fill', 'insertPart', 'mate', 'plane', 'deleteBody', 'combine', 'importStep', 'repairPart'];
    for (const t of dispatch) assert.ok((FEATURE_TYPES as readonly string[]).includes(t), t);
    assert.equal(FEATURE_TYPES.length, 63);
  });
});
