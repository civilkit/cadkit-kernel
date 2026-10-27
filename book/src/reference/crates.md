# Crate map

The kernel is a Cargo workspace of 17 crates under `crates/`. This table is read from each crate's `Cargo.toml` and `lib.rs`. Dependencies are the `[dependencies]` of the manifest (dev-dependencies are noted separately).

## Table

| Crate | Role | Main modules | Depends on |
|---|---|---|---|
| `ck-core` | Math and utilities shared by everything | `linalg` (vectors, matrices, frames), `xform`, `quad` (root finding), `kdtree`, `fxhash`, `memo`, `budget` (work budgets), `trace` | none |
| `ck-geom` | Curves and surfaces, analytic and NURBS; representation only, no topology | `curve`, `curve2` (sketch and parameter space), `pcurve`, `surface`, `nurbs` (basis, B-spline curves and surfaces, rational), `fit`, `analysis` (curvature, derivatives), `convert` | `ck-core` |
| `ck-json` | Zero-dependency JSON value, parser and writer; keeps key order; shortest round-trip floats | one file | none |
| `ck-topo` | The B-rep: arena-based body of lumps, shells, faces, loops, coedges, edges and vertices; builders, mass properties, region classification, edits | `body`, `make` (extrude, revolve), `mass`, `orient`, `region`, `edit`, `check` (structural check), `facekey`, `trig` (ported sine and cosine) | `ck-core`, `ck-geom`, `ck-json` |
| `ck-check` | The independent solid validator behind `validate`: topology, geometry and display-mesh checks with stable issue codes | `shell`, `wire`, `edge`, `solid`, `tessellation` | `ck-geom`, `ck-topo`, `ck-core` |
| `ck-heal` | Healing: vertex merging, sewing, small-feature removal, orientation, tolerant edges; also damages bodies for testing | `vertex`, `sew`, `small`, `wire`, `orient`, `tolerance`, `damage` | `ck-geom`, `ck-topo`, `ck-check` (dev: `ck-model`, `ck-json`, `ck-core`) |
| `ck-tess` | Tessellation of bodies into the display mesh (positions, normals, triangle-to-face map, silhouettes) | `cdt` (constrained Delaunay), `curvature` (tolerances), `face`, `mesh`, `comb`, `curvmap`. Feature `trace` | `ck-core`, `ck-geom`, `ck-topo` (dev: `ck-json`) |
| `ck-ops` | Modelling operations: the geometric work behind the features | `boolean`, `blend` (fillet, chamfer, g2), `loft`, `sweep`, `helix`, `offset`, `shell`, `draft`, `pipecut`, `surfacing` (fill, knit, trim, extend, fair, thicken), `intersect` (curve and surface intersections), `bvh`, `analysis` (the `distance`, `thickness`, `section`, `probe` and related requests), `section`, `measure`, `interference`, `drawing`, `dfm`, `profile` (design rules), `agent` (queries). Feature `sanity-shadow` | `ck-core`, `ck-geom`, `ck-topo`, `ck-json`, `ck-tess`, `ck-check` |
| `ck-text` | Lettering: built-in fonts, TrueType and OpenType outline reader, layout into closed loops | `ttf`, `fonts` (registry), `sha256`; layout in `lib` | none |
| `ck-model` | The document model and feature program: parts, expressions, references, sketches and solver, features, regeneration driver | `expr`, `refs`, `planes`, `sketch` (solver, profiles, offset, spline, text), `sketch3`, `features` (one module per family), `threads`, `regen` | `ck-core`, `ck-geom`, `ck-json`, `ck-topo`, `ck-ops`, `ck-check`, `ck-text` (dev: `ck-io`, `ck-tess`) |
| `ck-io` | STEP export and import, ACIS SAT import | `step` (`write`, `read`, `parse`), `sat` (`parse`, `build`) | `ck-core`, `ck-geom`, `ck-topo` (dev: `ck-json`, `ck-check`, `ck-tess`) |
| `ck-skp` | SketchUp reader, both containers; bakes to meshes or to a planar-face body | `vff` (modern), `legacy`, `legacy_ar` (classic), `zip`, `inflate`, `tlv`, `xml`, `model`, `scene`, `tri`, `body`, `export`, `error` | `ck-geom`, `ck-topo` (dev: `ck-check`, `ck-tess`) |
| `ck-sldprt` | SolidWorks part reader, to a body | `container`, `inflate`, `xt` (Parasolid transmit), `ir`, `build`, `error` | `ck-geom`, `ck-topo` (dev: `ck-check`, `ck-tess`) |
| `ck-recog` | Feature recognition of imported bodies | `holes`, `prism`, `revolve`, `emit`, `rebuild`, `request`, `model` | `ck-core`, `ck-geom`, `ck-json`, `ck-topo`, `ck-model`, `ck-ops` (dev: `ck-io`) |
| `ck-server` | Request envelope, dispatch table, reply framing, tag-keyed session state, hooks for the geometry layers | `session`, `reply`, `query`, `geometry` (mass, interference, import and step requests), `model`, `agent`, `drawing`, `comb`, `curvature`, `trace`, `clock`, `view` | `ck-json` |
| `ck-wasm` | The WebAssembly module: the five-function ABI and the engine that wires every request to the geometry crates | `engine`, `regen_cache`, `view_cache`, `drag`, `profile_q`, `refdiff`, `sketch_q`, `host`. Features `skp`, `sanity-shadow`. Crate types `cdylib` and `rlib` | `ck-json`, `ck-server`, `ck-model`, `ck-core`, `ck-geom`, `ck-topo`, `ck-check`, `ck-heal`, `ck-ops`, `ck-io`, `ck-sldprt`, `ck-tess`, `ck-recog`, `ck-text`; optional `ck-skp` |
| `ck-native` | A local HTTP service around the same dispatch, one worker thread per session; a library and the `ck-native` binary | `store` (token and pairing records), `main` | `ck-server`, `ck-wasm`, `ck-ops`, `ck-core`, `ck-json`, `ck-text` |

## Layers

From the bottom up:

1. **Foundations:** `ck-core`, `ck-json`, `ck-text`.
2. **Geometry and topology:** `ck-geom`, then `ck-topo`.
3. **Analysis and meshing:** `ck-check` (validation), `ck-tess` (display mesh), `ck-heal`.
4. **Operations:** `ck-ops`.
5. **Document model:** `ck-model` (features, regeneration), `ck-recog`.
6. **Interchange:** `ck-io`, `ck-skp`, `ck-sldprt`. They build bodies and depend only on geometry and topology.
7. **Protocol and hosts:** `ck-server` (dispatch; it depends only on `ck-json` and receives geometry behaviour through hooks), `ck-wasm` (installs the hooks and exports the ABI), `ck-native` (HTTP).

`ck-wasm` is the only crate that sees every other one; it is where a request type such as `importStep` or `step` is tied to `ck-io` and `ck-heal`. A native test calls `ck_wasm::engine::install()` and then `ck_server::handle`, which is the path the module exports use. See [architecture](../kernel/architecture.md).

## Build profiles

Defined in the workspace `Cargo.toml`:

| Profile | Settings | Use |
|---|---|---|
| `release` | optimisation 3, link-time optimisation, `panic = "abort"`, one code-generation unit | The shipped WebAssembly module |
| `service` | `release` with `panic = "unwind"` | `ck-native`, which must survive a panic in one session |
| `fast` | `release` without link-time optimisation, 16 code-generation units, incremental, `panic = "unwind"` | Quick native test runs |

## See also

- [Architecture](../kernel/architecture.md)
- [Testing](../quality/testing.md)
- [Build the kernel](../getting-started/build.md)
- [Glossary](glossary.md)
