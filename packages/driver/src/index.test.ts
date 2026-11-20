import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { loadSpec, stlBinary } from './index.js';
import { emptyPart } from '../../part-model/src/index.js';

describe('driver stlBinary', () => {
  it('writes header, count, +Z normal and vertices for one triangle', () => {
    const pos = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]);
    const buf = stlBinary(pos, 'tri');
    assert.equal(buf.length, 84 + 50);
    assert.equal(buf.readUInt32LE(80), 1);
    assert.equal(buf.subarray(0, 3).toString(), 'tri');
    assert.equal(buf.readFloatLE(84), 0); // nx
    assert.equal(buf.readFloatLE(88), 0); // ny
    assert.equal(buf.readFloatLE(92), 1); // nz
    assert.equal(buf.readFloatLE(96), 0); // v0.x
    assert.equal(buf.readFloatLE(108), 1); // v1.x
  });
});

describe('driver loadSpec', () => {
  it('loads a JSON spec file', async () => {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ck-'));
    const f = path.join(dir, 'part.json');
    const doc = emptyPart('J', 'steel');
    fs.writeFileSync(f, JSON.stringify(doc));
    assert.deepEqual(await loadSpec(f), doc);
  });

  it('loads a builder .mjs via its default export', async () => {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ck-'));
    const f = path.join(dir, 'builder.mjs');
    fs.writeFileSync(f, `export default function build() { return ${JSON.stringify(emptyPart('M'))}; }`);
    const doc = await loadSpec(f);
    assert.equal(doc.name, 'M');
  });
});
