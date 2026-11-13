# CadKit kernel

A B-rep geometry kernel for parametric part features, written in Rust and compiled to WebAssembly for the
[CadKit](https://civilkit.io/cad/) CAD app. A host sends it a part (a JSON document listing features:
sketches, extrudes, fillets, shells, ribs, holes, patterns, split bodies and lip joints, surfaces, sheet metal,
assemblies and so on) and gets back the solid, its faces and edges, a display mesh, a status for every feature,
and measurements.

- App: <https://civilkit.io/cad/>
- App repository (editor, scripting, MCP server): <https://github.com/civilkit/cadkit-kit>
- Kernel book: `book/` (mdBook sources: part format, protocol, internals, validation, import and export)

## Goals

- **Verifiable.** Results can be checked without trusting the kernel: an independent solid validator
  (`ck-check`), closed-form oracles for volume, area and centre of mass, round trips through STEP, and a
  regression gate that counts how many cases pass (see below).
- **Driven by an LLM or a script.** The interface is plain JSON in, JSON out, with read-only queries to
  describe a part, resolve a name, take a section, probe a point, measure and run design-for-manufacture
  checks (`docs/agent-queries.md`).
- **Structured errors with hints.** A failed feature reports a stable error code with parameters, not free
  text; a broken reference comes back with candidate replacements, and `refDiff` compares two versions of a part.
- **Robust before byte-exact.** A feature that cannot be built contributes nothing and the rest of the
  history still runs, so a failing edit can be understood rather than lost.
- **Small and dependency-free.** The workspace has no third-party Rust dependencies (`Cargo.lock` lists only
  the workspace crates); a build needs no network.

## Crates

All crates are in `crates/`.

| Crate | What it is |
|---|---|
| `ck-core` | Math and utilities: vectors, matrices, transforms, polynomial roots, k-d tree, time budgets, memoisation, tracing |
| `ck-geom` | Curve and surface geometry, analytic and NURBS (representation only, no topology) |
| `ck-topo` | B-rep topology: bodies, shells, faces, edges, construction and editing, orientation, regions, mass properties, face keys for persistent naming |
| `ck-ops` | Modelling operations: booleans, fillets and chamfers (blends), loft, sweep, offset, shell, draft, helix, pipe cut, surfacing, sections, measurement, drawing projection, design-for-manufacture queries |
| `ck-model` | The part document: features, sketches and the constraint solver, 3D sketches, expressions, references, and regeneration |
| `ck-tess` | Tessellation of bodies into display meshes (constrained Delaunay triangulation per face) |
| `ck-check` | Independent solid validator (topology, geometry, tessellation checks); adapted from brepkit |
| `ck-heal` | Shape healing driven by the validator: merge vertices, remove degenerate pieces, sew, fix orientation; every repair is reported |
| `ck-io` | STEP read and write, and ACIS SAT import |
| `ck-sldprt` | SolidWorks part (`.sldprt`) reader that builds a body directly |
| `ck-swedit` | Editable feature history for an imported SolidWorks part, recovered from the file's feature list and face ids |
| `ck-skp` | SketchUp (`.skp`) reader (both container formats), producing meshes and a body of planar faces |
| `ck-recog` | Feature recognition: turns a history-free solid (STEP or SolidWorks import) into a parametric feature tree |
| `ck-text` | Lettering: TrueType outlines of two bundled open-licensed fonts, laid out as closed sketch loops |
| `ck-json` | Zero-dependency JSON parser, writer and DOM shared by every crate |
| `ck-server` | Request envelope, the request dispatch table, session state and reply framing |
| `ck-wasm` | The WebAssembly boundary (`cdylib`): `ck_alloc`, `ck_request`, `ck_reply`, `ck_commit` |
| `ck-native` | Local HTTP service (127.0.0.1, token and pairing) around the same dispatch, for heavy parts outside the browser |

`packages/` holds TypeScript packages used by the tools (kernel client, session, part model, builder, MCP
server, renderer); they are not needed to build the kernel.

## Build and test

Requirements: a Rust toolchain with the `wasm32-unknown-unknown` target
(`rustup target add wasm32-unknown-unknown`), and Node.js 22 for the tools.

```sh
# WebAssembly module (fast iteration profile: no LTO)
cargo build -p ck-wasm --target wasm32-unknown-unknown --profile fast
# -> target/wasm32-unknown-unknown/fast/ck_wasm.wasm

# Shipped build (LTO, panic = abort); tools/build_wasm.sh also runs wasm-opt if it is installed
cargo build -p ck-wasm --target wasm32-unknown-unknown --release

# Native local service
cargo build --profile service -p ck-native

# Rust unit tests (what CI runs; ck-core's ffi_diff test needs an external comparison library and is skipped)
cargo test --workspace --profile fast --lib --bins
```

The app expects the module at `apps/cad/public/kernel/ck_wasm.wasm` in the kit repository (see its README).
`book/src/getting-started/` shows how to call the module from Node.

### Regression gate

`tools/gate.mjs` runs the built wasm over a corpus of parts and prints a scoreboard of counts (higher is
better), compared with a committed baseline. A drop in a hard metric fails the gate. Hard metrics include
valid output (no feature errors, positive volume, closed manifold shell), closed-form oracle checks of volume,
area, centre of mass and bounding box, STEP export-then-import round trips, ACIS SAT fixtures and healing of
deliberately damaged bodies; soft metrics include persistent-name stability, the agent queries and the
BenchCAD and brepjs part sets.

```sh
node tools/gate.mjs --fast --standalone --wasm target/wasm32-unknown-unknown/fast/ck_wasm.wasm
```

`--standalone` runs the metrics that judge this build alone; it is used automatically when no comparison
build is present. `--update` rewrites the baseline in the commit that improves a metric.
`tools/scoreboard.standalone.json` is that baseline, as `[passed, total]` per metric. Many other
`tools/*_check.mjs` and `*_sweep.mjs` scripts cover single areas (booleans, fillets, STEP, tessellation,
mass properties, crash fuzzing). Continuous integration is described in `docs/ci.md`.

## Status

Under active development. The part format, the request types and the feature set are described in the book;
the current baseline numbers are in `tools/scoreboard.standalone.json` and open problems in `docs/issues.md`
and `docs/roadmap.md`. Several features still return the stable error code `notImplemented` for some inputs
(`docs/issues.md`, K11), and the drawing queries have known gaps (K20, K21).

## Documentation

- `docs/kernel-interface.md`: WebAssembly ABI, request and reply envelope, regen result
- `book/`: the full reference
- `docs/agent-queries.md`: the read-only queries meant for agents
- `docs/`: design notes, sweeps and corpora per feature

## Licence

MIT OR Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`). Code adapted from or inspired by other projects, with its
licence, is listed in `THIRD_PARTY.md`.
