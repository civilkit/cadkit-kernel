import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { encodePng } from './png.js';
import { renderTriangles, type RGB } from './raster.js';
import type { MeshData } from './mesh.js';

function pngDims(png: Buffer): { w: number; h: number } {
  assert.deepEqual([...png.subarray(0, 8)], [137, 80, 78, 71, 13, 10, 26, 10]);
  return { w: png.readUInt32BE(16), h: png.readUInt32BE(20) };
}

describe('png', () => {
  it('encodes signature and dimensions', () => {
    const rgb = new Uint8Array([255, 0, 0, 0, 255, 0]);
    const png = encodePng(2, 1, rgb);
    assert.deepEqual(pngDims(png), { w: 2, h: 1 });
  });
});

describe('raster', () => {
  // One triangle in z=0 covering the centre of a front view (XY plane).
  const mesh: MeshData = {
    positions: new Float32Array([-2, -2, 0, 2, -2, 0, 0, 2, 0]),
    normals: new Float32Array([0, 0, 1, 0, 0, 1, 0, 0, 1]),
    triFace: new Int32Array([0]),
    faceIds: ['F1.C0'],
  };

  it('paints the triangle over the background', () => {
    const red: RGB = [255, 0, 0];
    const f = renderTriangles(mesh, 'front', 100, 100, new Map([[0, red]]));
    const at = (x: number, y: number) => [f.rgb[(y * 100 + x) * 3], f.rgb[(y * 100 + x) * 3 + 1], f.rgb[(y * 100 + x) * 3 + 2]];
    assert.deepEqual(at(50, 50), [255, 0, 0]); // centre of the triangle
    assert.deepEqual(at(2, 2), [16, 22, 30]); // background corner
  });

  it('shades unhighlighted faces grey, not background', () => {
    const f = renderTriangles(mesh, 'front', 100, 100, new Map());
    const px = [f.rgb[(50 * 100 + 50) * 3], f.rgb[(50 * 100 + 50) * 3 + 1], f.rgb[(50 * 100 + 50) * 3 + 2]];
    assert.ok(px[0] > 60 && px[0] === px[1] && px[1] === px[2]);
  });

  it('fitBox frames the box instead of the mesh (close-up)', () => {
    const red: RGB = [255, 0, 0];
    const at = (f: { rgb: Uint8Array }, x: number, y: number) =>
      [f.rgb[(y * 100 + x) * 3], f.rgb[(y * 100 + x) * 3 + 1], f.rgb[(y * 100 + x) * 3 + 2]];
    // Without fitBox the triangle spans -2..2, so its top-left pixel is background.
    const wide = renderTriangles(mesh, 'front', 100, 100, new Map([[0, red]]));
    assert.deepEqual(at(wide, 3, 3), [16, 22, 30]);
    // Framing the middle 1 x 1 of it zooms in: the same pixel is now the triangle.
    const close = renderTriangles(mesh, 'front', 100, 100, new Map([[0, red]]),
      { min: [-0.5, -0.5, -1], max: [0.5, 0.5, 1] });
    assert.deepEqual(at(close, 3, 3), [255, 0, 0]);
    assert.deepEqual(at(close, 50, 50), [255, 0, 0]);
  });
});
