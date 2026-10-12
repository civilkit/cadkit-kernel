import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { meshFromBody } from './mesh.js';
import { renderMesh } from './render.js';
import { defaultWasmPath as wasmPath } from '../../kernel-client/src/worker.js';

const OUT = process.env.CK_RENDER_OUT ?? null;

describe('render live', { skip: !existsSync(wasmPath()) && 'no wasm built' }, () => {
  it('renders the plate with a highlighted cap and legend', async () => {
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    const { Part } = await import('../../builder/src/index.js');
    const p = new Part('Plate');
    const base = p.sketch('Outline', 'front', (sk) => sk.rect(0, 0, 100, 60));
    p.extrude({ sketch: base, op: 'add', end: 'blind', depth: 10 });
    const k = await createWorkerKernel();
    try {
      const r = await k.request({ type: 'regen', tag: 't', part: p.toJSON() });
      const body = (r.result as { body: Parameters<typeof meshFromBody>[0] }).body;
      const mesh = meshFromBody(body);
      assert.ok(mesh.faceIds.includes('F2.C1'));
      const out = renderMesh(mesh, { width: 320, height: 240, view: 'iso', highlights: ['F2.C1', 'nope'] });
      assert.deepEqual(out.png.subarray(0, 4), Buffer.from([137, 80, 78, 71]));
      assert.equal(out.legend.length, 1); // unknown ids ignored
      assert.equal(out.legend[0].face, 'F2.C1');
      // Highlight colour is actually on screen (scan above the legend strip).
      const [hr, hg, hb] = out.legend[0].color;
      let hits = 0;
      const frame = (await import('./raster.js')).renderTriangles(mesh, 'iso', 320, 240, new Map([[mesh.faceIds.indexOf('F2.C1'), out.legend[0].color]]));
      for (let i = 0; i < 320 * 220; i++) {
        if (frame.rgb[i * 3] === hr && frame.rgb[i * 3 + 1] === hg && frame.rgb[i * 3 + 2] === hb) hits++;
      }
      assert.ok(hits > 100, `only ${hits} highlight pixels`);
      if (OUT) writeFileSync(path.join(OUT, 'plate-iso.png'), out.png);
    } finally {
      await k.close();
    }
  });
});
