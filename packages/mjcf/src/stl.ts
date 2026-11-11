// Binary STL: 80-byte header, u32 triangle count, 50 bytes per triangle (normal, 3 vertices, u16 attribute).

/** Triangle soup (9 floats per triangle) -> binary STL. Normals are recomputed from the winding. */
export function writeStl(positions: ArrayLike<number>, header = 'ck-cad mjcf export'): Uint8Array {
  const n = Math.floor(positions.length / 9);
  const buf = new ArrayBuffer(84 + 50 * n);
  const dv = new DataView(buf);
  const h = new TextEncoder().encode(header.slice(0, 80));
  new Uint8Array(buf, 0, h.length).set(h);
  dv.setUint32(80, n, true);
  let off = 84;
  for (let t = 0; t < n; t++) {
    const p = (i: number) => positions[t * 9 + i];
    const ux = p(3) - p(0), uy = p(4) - p(1), uz = p(5) - p(2), vx = p(6) - p(0), vy = p(7) - p(1), vz = p(8) - p(2);
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    for (const v of [nx, ny, nz]) { dv.setFloat32(off, v, true); off += 4; }
    for (let i = 0; i < 9; i++) { dv.setFloat32(off, p(i), true); off += 4; }
    dv.setUint16(off, 0, true); off += 2;
  }
  return new Uint8Array(buf);
}

/** Binary STL -> triangle soup. */
export function readStl(data: Uint8Array): Float32Array {
  const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const n = dv.getUint32(80, true);
  if (84 + 50 * n > data.byteLength) throw new Error('stl: truncated');
  const out = new Float32Array(n * 9);
  for (let t = 0; t < n; t++) for (let i = 0; i < 9; i++) out[t * 9 + i] = dv.getFloat32(84 + 50 * t + 12 + 4 * i, true);
  return out;
}
