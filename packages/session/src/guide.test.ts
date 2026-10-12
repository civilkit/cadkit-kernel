import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { GUIDE_VERSION, getGuide } from './guide.js';
import { listFixes } from '../../driver/src/verify.js';

describe('guide', () => {
  it('is versioned and covers every coded fix', () => {
    assert.match(GUIDE_VERSION, /^ck-guide\/\d+$/);
    const g = getGuide();
    assert.equal(g.version, GUIDE_VERSION);
    for (const f of listFixes()) assert.ok(g.text.includes(f.code), f.code);
  });

  it('carries the naming rule and recipes', () => {
    const { text } = getGuide();
    assert.ok(text.includes('F2.S.e10'));
    assert.ok(text.includes('Prefer referencing caps'));
    assert.ok(text.includes('plate with hole pattern'));
    assert.ok(text.includes('throughBoth'));
    assert.ok(text.includes('ADR 0001'));
  });
});
