# SketchUp import (`ck-skp`, request `importSkp`)

`crates/ck-skp` reads SketchUp `.skp` files. It is a Rust reimplementation of the reader half of OpenSKP (MIT, (c) Ahsan Mehmood),
which reverse engineered both containers (attribution: `crates/ck-skp/NOTICE`, `THIRD_PARTY.md`). No SketchUp SDK is used.

```
{type:"importSkp", tag:"t", name:"model", data:"<base64 of the .skp file>", opts:{hidden?:bool, maxFaces?:n, up?:"z"|"y"|"yFlip"}}
```

The reply is regen-shaped like `importStep` / `importSldprt` (`body`, `errors`, plus `import`). A refused file answers
`body:null` and `errors:[["import",{code,params:{message,offset?}}]]` and leaves the tag alone. The kit session method is
`session.importSkp(base64, name?, {hidden?, maxFaces?, up?})` (`packages/session`).

## Wiring

| Layer | Where |
|---|---|
| reader | `crates/ck-skp` (no dependency except `ck-geom` / `ck-topo` for the body) |
| request | `ck-server` `importSkp` (`geometry::import_skp`, hook `import_skp`) |
| wasm hook | `ck-wasm` `import_skp_hook`, behind the cargo feature **`skp`** |
| kit | `packages/session` `importSkp`; check tool `tools/skp_check.mjs` |

The wasm feature **`skp` is on by default** (`default = ["skp"]` in `ck-wasm`): the reader adds 323,936 bytes (7,093,626 -> 7,417,562, +4.6 %) to `ck_wasm.wasm`.
Build with `cargo build -p ck-wasm --target wasm32-unknown-unknown --release`; `--no-default-features` drops it, and then `importSkp` answers
the usual "not implemented" error; every other request is unchanged.

## What the body is

A SketchUp model is a surface model. The body has one planar face per SketchUp face (outer loop plus holes, outer
counter-clockwise about the front normal), vertices welded within 1e-4 mm, edges shared between neighbouring faces, instance
transforms flattened (nesting capped at 32), units converted from inches to millimetres, Z up as in SketchUp unless `opts.up` says otherwise (below). One lump with
one shell per connected piece. It is closed only where the model is: `import.openEdges` / `nonManifoldEdges` say how much is
not. Faces are named `skp.F<n>`; `import.faceMaterials[n]` indexes `import.materials` (`color`, `opacity`, `doubleSided`,
`textured`). Texture image bytes are read (`Material.texture.data`, GLB export embeds them) but not sent over the wire.
Hidden faces, instances and tags are skipped unless `opts.hidden`. More than `maxFaces` (default 50,000) faces is refused
with `limit.faces`.

## Orientation (`opts.up`)

SketchUp is Z up and right-handed; the kit and its planes are Y up. `opts.up` rotates the whole model, applied to the finished
scene (after instance flattening and the inch to mm scale, so instance transforms and units are already resolved):

| `up` | (x, y, z) maps to | meaning |
|---|---|---|
| `"z"` (default) | (x, y, z) | as in the file |
| `"y"` | (x, z, -y) | -90 degrees about X: SketchUp Z becomes Y, SketchUp's front (-Y) faces +Z |
| `"yFlip"` | (-x, -z, -y) | `"y"` then 180 degrees about Z: the upside-down fix |

Matrix of `"y"` (row major, column vectors): `[1 0 0; 0 0 1; 0 -1 0]`; of `"yFlip"`: `[-1 0 0; 0 0 -1; 0 -1 0]`. Both have determinant +1, so
winding, face order and names (`skp.F<n>`), areas, volumes, open and non-manifold counts are unchanged, normals and plane frames
rotate with the geometry and `import.bboxMm` is the box of the rotated body. The kernel default stays `"z"` (additive, old callers
see no change); the kit asks for `"y"`. The reply's `import` carries `up` and `upMatrix` (the nine numbers, row major). Any other
value is a request error (`ok:false`). In the library: `BakeOptions { up: Up::Y, .. }`.

## Library use

```rust
let model = ck_skp::parse(&bytes)?;                         // both containers
let scene = ck_skp::bake(&model, &Limits::default(), &BakeOptions::default())?;  // world-space triangles per material, mm
let glb = ck_skp::export::to_glb(&scene, true);             // also to_obj
let body = ck_skp::body::build_body(&scene_with_polygons, &BodyOptions::default())?; // BakeOptions{keep_polygons:true}
```

## Robustness

Input is untrusted. No panic paths (no indexing on file data, checked arithmetic on offsets), structured `SkpError {code,
message, offset}`, limits in `Limits` / `BakeOptions` (file 512 MB, ZIP entry 512 MB with the ratio guard, TLV 40 M nodes and
depth 64, archive 40 M objects and depth 160, face vertices, instances 2 M, triangles 30 M). `tests/robustness.rs` is a seeded
mutation fuzzer (byte flips, truncations, splices, zero fills, 32-bit fields set huge, also on the inflated `model.dat` re-wrapped
in a valid ZIP): about 1.6 s per run, `CK_SKP_FUZZ_ITERS=20000` for a long run. It found one hang that OpenSKP shares
(a layer record cut off at the end of the data loops forever in its Python reader); fixed.

## Parity with OpenSKP

`cargo test -p ck-skp --test openskp_parity` checks the 17 committed fixtures against the numbers OpenSKP's Python package
reports (entity totals, primitives, scene materials, bounding box; triangles within 1 % and area within 0.5 %, they depend on
the ear clipper) and against the values asserted in OpenSKP's own tests. `OPENSKP_FIXTURES=<clone> cargo test -p ck-skp
--release --test openskp_parity -- --nocapture openskp_tree` runs all 80 files of the clone (26 distinct); `crates/ck-skp/tools/
openskp_ref.py` regenerates the reference numbers. Result: 80 of 80 parse; vertices, edges, faces, instances, definitions and
primitives are identical to OpenSKP for all 26 distinct files, bounding boxes agree to 0.05 mm. OpenSKP's "11 of 15" figure
refers to its own private sweep of old-format files that is not in the clone; all 22 distinct classic-container files of the clone
(versions 3, 6, 7, 14, 17, 18, 20) and the 4 distinct modern ones (2021+) parse.

## Not covered (on purpose)

Writer, editor, code generator; scenes (pages) and cameras, dimensions, text, section planes, construction lines and
points (walked for alignment where the classic format needs it, then dropped); attribute dictionaries / dynamic component
properties (skipped, not decoded); styles other than face colours; loose edges and curve sets (no wire output); analytic arcs;
the shapely-based merge of overlapping holes (overlapping holes are bridged as given); instanced (non-baked) scenes; IFC, DXF,
STL, PLY, Fragments exports. The classic format has no meta units: `import.units` is null there (geometry is inches anyway).
