# Replies and buffers

Every call returns one byte string: a length-prefixed JSON document, followed by an optional binary area that holds bulk arrays such as the display mesh. This chapter specifies the layout, the buffer references in the JSON, and how to decode them.

## The frame

```text
offset 0        4                      4+L        B = (4+L+7) & ~7
        +-------+----------------------+----------+---------------------------+
        | L     | JSON text, L bytes   | padding  | binary area               |
        | u32le | UTF-8                | spaces   | buffer 0, buffer 1, ...   |
        +-------+----------------------+----------+---------------------------+
```

- `L`, the first four bytes, is the byte length of the JSON text, as an unsigned 32-bit integer in little-endian order.
- The JSON text follows, UTF-8 encoded, without a terminator.
- Padding follows, up to the next multiple of 8 counted from the start of the frame. The padding bytes are ASCII spaces (`0x20`), so the padded text is still valid JSON. If `4 + L` is already a multiple of 8, there is no padding.
- The binary area starts at `B = (4 + L + 7) & ~7`. Buffers are packed one after another in the order they were created; nothing follows the last one.
- The value that `ck_request` returns is the length of the whole frame, binary area included. A frame without buffers is just the padded JSON.

A `ping` reply is the smallest example. For the body `{"id":1,"msg":{"type":"ping"}}` and a module built without a revision, the JSON is `{"ok":true,"result":{"pong":true},"ms":0}` (41 bytes), padded with 3 spaces to a frame of 48 bytes. From a real run of a module that reports its revision, the frame has 72 bytes: `40 00 00 00` (L = 64), the 64 bytes of JSON, and 4 spaces.

## The JSON envelope

The JSON is always an object with one of two shapes:

```json
{ "ok": true,  "result": <value>, "ms": 0.58 }
{ "ok": false, "error": "unknown request nope", "ms": 0.01 }
```

| Key | Meaning |
|---|---|
| `ok` | `true` when the request ran and produced a result. |
| `result` | The result; its shape depends on the request type. Present when `ok` is true. It can be any JSON value: an object, an array, a string (`step`), or `null` (for example a query on a tag that has no body). |
| `error` | A message string. Present when `ok` is false. There is no structured error object at this level; see [Errors](errors.md). |
| `ms` | The time the call took in milliseconds, measured with the host's `ck_now`. `0` when the host clock is constant. |
| `buffers`, `trace` | Only with `"debug": true` in the request ([Protocol overview](overview.md)). `buffers` is `{ "count": n, "bytes": b }` for the binary area (ok replies only). |

An error reply has no binary area. An `ok: true` reply says only that the request ran: an operation that fails in a way the request models (a feature that cannot be built, a validator that finds issues, an import that cannot parse its file) reports that inside `result`.

Numbers are standard JSON numbers written in the shortest form that reads back to the same 64-bit float: `100`, `0.5`, `59497.34517542564`, `1.4332395860321886e-17`. A number that is not finite (NaN, infinity) cannot be written and is written as `null`; the continuity report, for example, uses `null` for values it could not compute. Object keys keep a fixed order, which is the order the tables in this book use.

## Buffer references

A bulk array is not in the JSON. In its place the JSON has a reference object with exactly these four keys, in this order:

```json
{ "$buf": 0, "type": "f32", "len": 936, "offset": 0 }
```

| Key | Meaning |
|---|---|
| `$buf` | The index of the buffer in creation order, starting at 0. The index is not the order of appearance in the JSON: in a `regen` result, `body.silhouette` is buffer 4 but comes before `body.mesh`. |
| `type` | The element type: `f32`, `f64`, `u32`, `i32` or `u8`. |
| `len` | The number of **elements** (not bytes). |
| `offset` | The byte offset of the first element, counted from the start of the binary area `B`, not from the start of the frame. |

Element types:

| `type` | Element | Bytes |
|---|---|---|
| `f32` | IEEE 754 single | 4 |
| `f64` | IEEE 754 double | 8 |
| `u32` | unsigned 32-bit integer | 4 |
| `i32` | signed 32-bit integer | 4 |
| `u8` | unsigned byte | 1 |

All multi-byte elements are little-endian (the native order of WebAssembly). Before a buffer is appended, the area is padded with zero bytes to a multiple of the element size, so that every buffer starts at an offset that suits its element type; `offset` is always a multiple of the element size. Because `B` is a multiple of 8, a typed-array view of the frame's own memory is aligned whenever the frame itself starts at an aligned address. The reference decoder below copies each buffer instead, which makes alignment of the source irrelevant.

A zero-length buffer is legal: `{ "$buf": 4, "type": "f32", "len": 0, "offset": 11648 }` is an empty array placed at the end of the area (the silhouette of a body without curved faces).

### Which results carry buffers

| Result | Buffers, in creation order |
|---|---|
| `regen` and the import requests: `body` | `mesh.positions` (f32), `mesh.normals` (f32), `mesh.mids` (f32), `mesh.triFace` (i32), `silhouette` (f32) |
| the same, for each entry of `surfaces` | the same five, following those of `body` |
| `curvatureMap`: `bodies[0].mesh` | `positions` (f32), `normals` (f32), `k` (f32), `triFace` (i32) |

No other request returns buffers. The arrays are:

| Array | Length | Content |
|---|---|---|
| `mesh.positions` | 9 per triangle | The display mesh as unwelded triangles: three vertices of x, y, z. Vertices are not shared between triangles. |
| `mesh.normals` | 9 per triangle | The exact outward surface normal at each vertex (for a plane, its normal). |
| `mesh.mids` | 9 per triangle | For the triangle's edges v0-v1, v1-v2, v2-v0, the correction `n(mid) - (n_a + n_b) / 2` between the true normal at the edge midpoint and the mean of the normals at its ends. A shader adds a multiple of these to the interpolated normal to round the shading of coarse curved faces. All zero on planes. |
| `mesh.triFace` | 1 per triangle | The index of the triangle's face in `body.faces`. |
| `silhouette` | 12 per record | Candidate silhouette edges: `a` (x, y, z), `b` (x, y, z), `nA` (x, y, z), `nB` (x, y, z). A record exists for each mesh edge shared by two triangles of the same face whose dihedral angle is strictly convex; `nA` and `nB` are the facet normals of the two triangles (the earlier and the later triangle). They are the data a renderer needs to draw the silhouette lines of curved faces. Planar faces and concave hole walls have none. |
| `k` (curvature map) | 6 per triangle | The principal curvatures k1 and k2 at each of the three vertices. |

`body.box` is the bounds of the f32 mesh positions together with the edge polylines.

## Decoding

1. Read `L` from the first 4 bytes, little-endian.
2. Decode bytes `4 .. 4+L` as UTF-8 and parse them as JSON.
3. Compute `B = (4 + L + 7) & ~7`.
4. Walk the parsed value. Replace every object that has a `$buf` key by the typed array of the right type over bytes `B + offset .. B + offset + len * size`.

In JavaScript (the same function as in [Call it from Node](../getting-started/node.md)):

```js
const ARRAYS = { f32: Float32Array, f64: Float64Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };

function decodeReply(bytes) {                       // bytes: a Uint8Array holding the whole frame
  const L = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getUint32(0, true);
  const json = JSON.parse(new TextDecoder().decode(bytes.subarray(4, 4 + L)));
  const B = (4 + L + 7) & ~7;
  const walk = (v) => {
    if (Array.isArray(v)) return v.map(walk);
    if (v === null || typeof v !== 'object') return v;
    if ('$buf' in v) {
      const A = ARRAYS[v.type], start = B + v.offset;
      return new A(bytes.slice(start, start + v.len * A.BYTES_PER_ELEMENT).buffer);
    }
    return Object.fromEntries(Object.entries(v).map(([k, x]) => [k, walk(x)]));
  };
  return walk(json);
}
```

`bytes.slice` copies; a faster host that wants views must make sure the offsets it uses are aligned for the element type.

### A real frame

A `regen` of the 100 x 60 x 10 plate with a hole (seven faces, 104 triangles) returns a frame of 17304 bytes. The JSON is `L = 5652` bytes, so `B = 5656` (the 4-byte prefix and the JSON already end on a multiple of 8, so no padding is added) and the binary area is 11648 bytes. The references in `body` are:

```text
"silhouette": { "$buf": 4, "type": "f32", "len": 0,   "offset": 11648 },
"mesh": {
  "positions": { "$buf": 0, "type": "f32", "len": 936, "offset": 0 },
  "normals":   { "$buf": 1, "type": "f32", "len": 936, "offset": 3744 },
  "mids":      { "$buf": 2, "type": "f32", "len": 936, "offset": 7488 },
  "triFace":   { "$buf": 3, "type": "i32", "len": 104, "offset": 11232 }
}
```

936 floats are 104 triangles times 9 numbers; each f32 buffer is 3744 bytes, `triFace` is 416 bytes, and 3744 + 3744 + 3744 + 416 = 11648. The cylinder of the hole is curved but its walls are concave, so the silhouette is empty.

`book/tools/run.mjs` prints each buffer as `<buffer type xN>` unless it is given `--full`.

## See also

- [Protocol overview](overview.md)
- [Regeneration](regen.md)
- [Errors](errors.md)
- [Tessellation](../kernel/tessellation.md)
