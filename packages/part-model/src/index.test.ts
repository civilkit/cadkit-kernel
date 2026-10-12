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
