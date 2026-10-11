# CadKit kernel — source recovery & rebuild

The original Rust source was lost to an SSD failure. What survived: the built
`kernel.wasm`, the minified frontend, part files, and probe scripts. This
workspace is the reconstructed skeleton, rebuilt from artifacts.

## Repository layout

```
/tmp-purr/
  kernel.wasm          ORIGINAL binary (5.5MB, no debug info) — the oracle
  kernel.wat           wabt WAT dump (104MB) — the disassembly
  kernel.strings       strings extracted from the binary
  rs_paths.txt         177 source file paths recovered from panic metadata
  app.pretty.js        beautified frontend (37.5k lines) — client-side spec
  oracle-flange.json   golden kernel response for coupling-sample.ckpart
  rebuild/             THIS WORKSPACE — 10 crates, layout = rs_paths.txt
  oracle.mjs           harness: run any msg against the ORIGINAL wasm
```

## The kernel interface (wire protocol)

Transport: a single JSON request/response call over wasm linear memory,
plus binary buffers for bulk arrays.

### Exports (the ABI)

| export | signature | meaning |
|---|---|---|
| `ck_alloc(n) -> ptr` | n bytes | allocate space in linear memory for the request |
| `ck_request(ptr, len) -> u32` | raw request | run one message; returns total reply length |
| `ck_reply() -> ptr` | | pointer to the reply |
| `ck_commit() -> ptr`, `ck_commit_len() -> n` | | static build id string: `"class-a-surfacing da3c6f49..."` |
| imports | `ck_panic(ptr,len)`, `ck_now() -> f64`, `ck_unix_ms() -> f64` | host callbacks |

### Request (JSON)

```json
{"id": <n>, "msg": { ... }}
```

`msg` is discriminated by its `type` field. Exactly **17** request types
exist (brute-forced against kernel.wasm — every identifier in kernel.strings +
app.pretty.js was sent as `msg.type`): `ping`, `regen`, `matchEdges`, `drag`,
`convert`, `sketchOffset`, `refCurve`, `measure`, `massProps`, `interference`,
`step`, `inspect`, `continuity`, `curvatureAt`, `curvatureMap`, `comb`,
`drawing`. Feature operations (`cutWithSurface`, `deleteFace`, `editCurve`,
`freeform`, `dome`, ...) are part-document feature types carried inside a
`regen`, not requests.

Example: `{type:"regen", tag:"main", part:<.ckpart object>, opts:{upTo:n}}`

### Response (binary envelope)

```
[u32 LE: json_len][json bytes][pad to 8][buffer 0][buffer 1]...
```

- JSON is `{"ok":true,"result":{...}}` or `{"ok":false,"error":"...","ms":...}`.
- Any JSON object with `{"$buf": i, "type": "f32|f64|u32|i32|u8", "len": n,
  "offset": o}` is a view into the binary area after the JSON: element `i`
  starts at byte `pad(8) + o`, holding `n` elements of `type`.
- The `body.mesh` of a regen result is always buffer-backed (positions,
  normals, mids, triFace) — the 4.3MB flange golden has ~1.5M f32s.

### regen result shape (pinned by oracle-flange.json)

```jsonc
{"errors": [],
 "components": [{"id":"F1","dof":6,"place":{"o":[..],"x":[..],"y":[..],"z":[..]}}],
 "planes": [], "markers": [], "sketches": [], "sketches3": [], "curves": [],
 "body": {"box": {"min":[..],"max":[..]},
          "faces": [{"id":"F4.S.e1@F1.1","kind":"cylinder","planar":false,
                     "meta":{"feature":"F4","entity":"e1@F1.1"},"lump":0}],
          "edges": [{"pts":[..],"faces":[..],"key":"..","mid":[..],
                     "circle":{"c":[..],"n":[..],"r":25}} or "line":true],
          "silhouette": {"$buf":4,...},
          "mesh": {"positions":{"$buf":0,...}, "normals":{"$buf":1,...},
                   "mids":{"$buf":2,...}, "triFace":{"$buf":3,...}}},
 "surfaces": []}
```

Face kinds: plane, cylinder, cone, torus, spline (16 spline faces in the
golden — full NURBS surfaces survive the pipeline).

## How to run the oracle

```bash
cd rebuild
node oracle.mjs ../coupling-sample.ckpart ../out.json
# or a custom msg:
node oracle.mjs ../bracket.ckpart ../out.json '{"type":"regen","tag":"plain","part":<obj>}'
```

## Crate map (file layout pinned by rs_paths.txt)

| crate | role |
|---|---|
| `ck-core` | linalg (Vec3/mat/quat), quad roots, kdtree, tracing, xform |
| `ck-geom` | curves/surfaces: curve, curve2, pcurve, surface, fit, nurbs/* |
| `ck-topo` | B-rep topology: body, make (+revolve), edit, orient, region, check, mass |
| `ck-ops` | operations: boolean/*, blend/* (17 files), loft/*, sweep, offset, shell, draft, helix, pipecut, surfacing/* (incl. rebuild/), intersect/*, bvh |
| `ck-model` | document model: part, features/*, sketch (solver!), sketch3, expr, refs, regen |
| `ck-tess` | tessellation: cdt, curvature, face, mesh |
| `ck-server` | message dispatch: session, model, geometry, query, view, drawing, comb, curvature |
| `ck-json` | zero-dep JSON (parser/writer/DOM) |
| `ck-io` | STEP export (step/write.rs) — full AP203/AP214 writer strings in kernel.strings |
| `ck-wasm` | the ABI boundary: ck_alloc/ck_request/ck_reply/ck_commit |

## Rebuild rules

1. **Never invent wire formats.** Every JSON field name, message type, and
   error string is pinned by artifacts. When in doubt, grep kernel.strings
   or the oracle golden.
2. **Float formatting**: shortest round-trip (`format!("{}", f64)`), integers
   without `.0` — match the golden byte-for-byte.
3. **Bottom-up order**: ck-core -> ck-geom/nurbs -> ck-topo -> ck-ops ->
   ck-model/features -> ck-server -> ck-wasm. Lock each layer with a golden
   diff before moving on.
4. **Decompile as you go**: the WAT (`kernel.wat`) has all 4,213 functions;
   mangled names only, so identify functions by their string references and
   call sites, not names.
5. `no_std`-ish kernel: the original used std with `panic=abort` on
   wasm32-unknown-unknown, dlmalloc, no wasm-bindgen.
