// Mesh input: $buf-decoded regen bodies carry positions/normals (f32 xyz per
// vertex, 3 vertices per triangle) and triFace (i32 face index per triangle
// into body.faces). This normalises every accepted shape to TypedArrays.
import type { DecodedBuf } from '../../kernel-client/src/protocol.js';

type Arr = Float32Array | Int32Array | number[] | DecodedBuf;

export interface MeshData {
  positions: Float32Array;
  normals: Float32Array;
  triFace: Int32Array;
  faceIds: string[];
}

function isDecoded(v: unknown): v is DecodedBuf {
  return !!v && typeof v === 'object' && 'data' in (v as Record<string, unknown>);
}

function f32(v: Arr | undefined, len: number): Float32Array {
  if (!v) return new Float32Array(len);
  if (v instanceof Float32Array) return v;
  if (isDecoded(v)) {
    const d = v.data;
    return d instanceof Float32Array ? d : Float32Array.from(d as ArrayLike<number>);
  }
  return Float32Array.from(v as ArrayLike<number>);
}

function i32(v: Arr | undefined, len: number): Int32Array {
  if (!v) return new Int32Array(len);
  if (v instanceof Int32Array) return v;
  if (isDecoded(v)) {
    const d = v.data;
    return d instanceof Int32Array ? d : Int32Array.from(d as ArrayLike<number>);
  }
  return Int32Array.from(v as ArrayLike<number>);
}

export function meshFromBody(body: {
  faces?: Array<{ id: string }>;
  mesh?: { positions?: Arr; normals?: Arr; triFace?: Arr };
}): MeshData {
  const faces = body.faces ?? [];
  const tris = Math.floor(f32(body.mesh?.positions, 0).length / 9);
  return {
    positions: f32(body.mesh?.positions, tris * 9),
    normals: f32(body.mesh?.normals, tris * 9),
    triFace: i32(body.mesh?.triFace, tris),
    faceIds: faces.map((f) => f.id),
  };
}

export function bboxOf(pos: Float32Array): { min: number[]; max: number[] } {
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 3) {
    for (let a = 0; a < 3; a++) {
      if (pos[i + a] < min[a]) min[a] = pos[i + a];
      if (pos[i + a] > max[a]) max[a] = pos[i + a];
    }
  }
  return { min, max };
}
