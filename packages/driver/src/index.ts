// Importable core of the repo's CLI driver: spec loading and STL encoding.

import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import type { PartDoc } from '../../part-model/src/index.js';

/** Load a part spec: JSON file, or a builder .mjs whose default export returns a Part. */
export async function loadSpec(file: string): Promise<PartDoc> {
  if (/\.(mjs|js)$/.test(file)) {
    const m = await import(pathToFileURL(path.resolve(file)).href + '?t=' + Date.now());
    const v = typeof m.default === 'function' ? await m.default() : m.default;
    return JSON.parse(JSON.stringify(v)) as PartDoc;
  }
  return JSON.parse(fs.readFileSync(file, 'utf8')) as PartDoc;
}

/** Binary STL from a Float32Array of xyz triples (3 vertices per triangle). */
export function stlBinary(positions: Float32Array, name?: string): Buffer {
  const n = Math.floor(positions.length / 9);
  const buf = Buffer.alloc(84 + n * 50);
  buf.write((name || 'part').slice(0, 79), 0);
  buf.writeUInt32LE(n, 80);
  for (let t = 0; t < n; t++) {
    const o = 84 + t * 50;
    const p = positions.slice(t * 9, t * 9 + 9);
    const u = [p[3] - p[0], p[4] - p[1], p[5] - p[2]];
    const v = [p[6] - p[0], p[7] - p[1], p[8] - p[2]];
    let nx = u[1] * v[2] - u[2] * v[1];
    let ny = u[2] * v[0] - u[0] * v[2];
    let nz = u[0] * v[1] - u[1] * v[0];
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    [nx, ny, nz, ...p].forEach((x, i) => buf.writeFloatLE(x, o + i * 4));
  }
  return buf;
}
