import { describe, it, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { loadSpec, run, stlBinary } from './index.js';
import type { EvalPage } from '../../kernel-client/src/index.js';
import { emptyPart } from '../../part-model/src/index.js';

afterEach(() => {
  delete (globalThis as unknown as Record<string, unknown>).window;
});

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

describe('driver run', () => {
  // Fake page: invokes the function directly, exactly as page.evaluate would
  // after serializing it — this also proves evalKernelInPage is self-contained.
  function fakePage(): EvalPage {
    return { evaluate: async (fn, arg) => (fn as (a: typeof arg) => Promise<unknown>)(arg) as never };
  }

  function stubKernel() {
    (globalThis as unknown as Record<string, unknown>).window = {
      cadkit: {
        kernel: {
          send(msg: Record<string, unknown>, cb: (v: unknown) => void) {
            if (msg.type === 'regen') {
              cb({
                errors: [],
                body: {
                  box: { min: [0, 0, 0], max: [100, 60, 10] },
                  faces: [{ id: 'F2.C0' }],
                  edges: [],
                  mesh: { positions: new Float32Array(9) },
                },
              });
            } else if (msg.type === 'massProps') {
              cb({ volume: 57989.123456789, area: 15803.19, com: [50, 30, 5], axes: [[1, 0, 0]] });
            } else if (msg.type === 'step') {
              cb('STEPDATA');
            }
          },
        },
      },
    };
  }

  it('writes step/stl files, strips raws, drops axes, rounds to 7 sig figs', async () => {
    stubKernel();
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ck-'));
    const stepF = path.join(dir, 'out.step');
    const stlF = path.join(dir, 'out.stl');
    const out = await run(fakePage(), emptyPart('Plate'), { mass: true, step: stepF, stl: stlF });
    assert.equal(out.ok, true);
    assert.equal(fs.readFileSync(stepF, 'utf8'), 'STEPDATA');
    assert.deepEqual(out.step, { file: stepF, bytes: 8 });
    assert.equal(fs.statSync(stlF).size, 84 + 50);
    assert.deepEqual(out.stl, { file: stlF, triangles: 1 });
    assert.ok(!('stepText' in out));
    assert.ok(!('positions' in out));
    assert.ok(!('axes' in (out.mass as Record<string, unknown>)));
    assert.equal((out.mass as Record<string, unknown>).volume, +(57989.123456789).toPrecision(7));
  });

  it('keeps axes with full:true', async () => {
    stubKernel();
    const out = await run(fakePage(), emptyPart('Plate'), { mass: true, full: true });
    assert.deepEqual((out.mass as Record<string, unknown>).axes, [[1, 0, 0]]);
  });
});
