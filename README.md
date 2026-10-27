# CadKit kernel

A B-rep CAD kernel in Rust that compiles to WebAssembly. A part is a JSON feature tree
(`.ckpart`); the kernel rebuilds it into a solid body with faces, edges, a display mesh and
measurements, and exports STEP.

- Sketches with a constraint solver, extrude, revolve, sweep, loft, shell, draft, helix, hole,
  fillet/chamfer (including variable and corner blends), booleans, patterns, mirror, surfacing
  features (knit, fill, trim, thicken, extend), assemblies with mates.
- Reads and writes STEP; imports SketchUp (`.skp`) and SolidWorks (`.sldprt`) geometry.
- Runs in a browser worker, in Node, or as a native service (`ck-native`).

## Layout

| crate | role |
|---|---|
| `ck-core` | linear algebra, root finding, k-d tree, tracing, transforms |
| `ck-geom` | curves and surfaces: analytic types, NURBS, fitting, projection |
| `ck-topo` | B-rep topology: bodies, construction, editing, orientation, regions, checks, mass properties |
| `ck-ops` | booleans, blends, loft, sweep, offset, shell, draft, helix, surfacing, intersection |
| `ck-model` | the document model: part, features, sketch solver, expressions, references, regeneration |
| `ck-tess` | tessellation: constrained Delaunay, curvature, face meshing |
| `ck-check` | validity and tessellation checks |
| `ck-heal` | healing of imported geometry |
| `ck-io` | STEP reader and writer |
| `ck-skp`, `ck-sldprt` | SketchUp and SolidWorks readers |
| `ck-text` | text outlines |
| `ck-recog` | feature recognition |
| `ck-json` | dependency-free JSON parser and writer |
| `ck-server` | message dispatch |
| `ck-wasm` | the WebAssembly ABI |
| `ck-native` | HTTP service around the same dispatch |

`packages/` holds the TypeScript side: the kernel client (browser worker, Node worker, native
service), the part-model builders, sessions, and the render helpers.

## Wire protocol

One JSON request and one reply per call, through the module's linear memory, with binary buffers
for bulk arrays.

| export | meaning |
|---|---|
| `ck_alloc(n) -> ptr` | allocate `n` bytes for the request |
| `ck_request(ptr, len) -> u32` | run one message; returns the reply length |
| `ck_reply() -> ptr` | pointer to the reply |
| `ck_commit() -> ptr`, `ck_commit_len() -> n` | build id string |

Imports (module `env`): `ck_panic(ptr, len)`, `ck_now() -> f64`, `ck_unix_ms() -> f64`.

Request: `{"id": <n>, "msg": { "type": "...", ... }}`. The request types are `ping`, `regen`,
`matchEdges`, `drag`, `convert`, `sketchOffset`, `refCurve`, `measure`, `massProps`,
`interference`, `step`, `inspect`, `continuity`, `curvatureAt`, `curvatureMap`, `comb` and
`drawing`. Feature operations are part-document feature types carried inside a `regen`.
Example: `{"type":"regen","tag":"main","part":<.ckpart object>,"opts":{"upTo":n}}`.

Reply: `[u32 LE: json_len][json bytes][pad to 8][buffer 0][buffer 1]...`. The JSON is
`{"ok":true,"result":{...}}` or `{"ok":false,"error":"..."}`. A JSON object
`{"$buf": i, "type": "f32|f64|u32|i32|u8", "len": n, "offset": o}` is a view into the binary area:
element `i` starts at byte `pad(8) + o` and holds `n` elements of `type`. The `body.mesh` of a
regen result is always buffer-backed.

## Build and test

```
cargo build -p ck-wasm --target wasm32-unknown-unknown --profile fast
cargo test --workspace
node tools/gate.mjs --fast        # the regression gate
```

The gate compares against a reference build of a previous kernel and a set of corpora;
`tools/remote_build.sh` runs the build and the checks on a remote Linux box. Environment switches are named
`CK_*` (for example `CK_WASM` selects the wasm the TypeScript tests load).

## Part files

A part is JSON with `"format": "ck-part"`. Files saved by earlier versions are still read and come
back as `ck-part`.

## Licence

MIT OR Apache-2.0. See `LICENSE-MIT` and `LICENSE-APACHE`. Third-party material and its licences
are listed in `THIRD_PARTY.md`.
