# Testing

The kernel is tested at four levels: Rust unit tests inside each crate, Rust integration tests that drive whole requests, Node scripts that run the built WebAssembly module against corpora and sweeps, and a regression gate that compares a scoreboard of metrics with a committed baseline. This chapter describes the first, second and fourth; the corpora are in [corpora and sweeps](corpora.md).

## Rust tests per crate

Every crate is tested on its own. A text search finds about 1,200 `#[test]` functions in the workspace (approximate, counted by `grep`); about 30 more are `#[ignore]`d probes, described below.

| Crate | Where the tests are | What they cover |
|---|---|---|
| `ck-core`, `ck-geom` | `src/` modules | Linear algebra, quadrature, k-d tree, budget clock; curves, surfaces, NURBS evaluation, fitting, conversions |
| `ck-topo` | `src/` modules | Extrusion and revolution builders, orientation, mass properties, region classification, edits, naming |
| `ck-check` | `src/tests.rs` | A good box validates clean; the same box damaged (open shell, flipped face, ...) must report the expected code |
| `ck-heal` | `src/tests.rs`, `tests/integration.rs` | Damage a valid body (`damage.rs` has the kinds), heal it, require `validate` clean and the volume unchanged |
| `ck-tess` | `src/`, `tests/` | Constrained Delaunay triangulation, per-face meshing, poles and domes |
| `ck-ops` | `src/` (about 190 tests) and `tests/` | Booleans (grid fuzz of 600 sequences, spheres, tangent and toleranced operands), blends, lofts, sweeps, shell, surfacing, surface-surface intersection |
| `ck-model` | `src/` and 72 files in `tests/` | Feature behaviour by family: blends, booleans, sweeps, lofts, shell and thicken, patterns, threads, text, mates, projection; regression files for named bugs; the robustness suite (below) |
| `ck-io` | `tests/` | STEP writer entities, STEP round trip, hand-written import fixtures, NIST-derived cases, ACIS SAT fixtures and dialects, multi-solid files |
| `ck-skp`, `ck-sldprt` | `tests/` | Sample files from each container version, bodies against known mass properties, a mutation fuzzer that must never panic |
| `ck-text` | `src/tests.rs`, `src/font_tests.rs` | Glyph area, nesting, orientation, anchors, arcs, metrics; twelve fixture fonts; 900 damaged font files |
| `ck-recog` | `tests/` | Recognition of kernel-built parts and rebuild accuracy |
| `ck-server`, `ck-wasm` | `src/`, `tests/` | The request envelope; full requests through the dispatch table: STEP, SAT, SketchUp imports, healing, regen cache, loft families, section and measure queries, part versions |
| `ck-native` | `tests/service.rs` | The local HTTP service: replies equal direct calls, sessions do not share state, authentication and origin checks, size limits, keep-alive, idle expiry, pairing |

To run one crate:

```sh
cargo test -p ck-io
cargo test -p ck-model --test blend_overhang           # one test file
cargo test -p ck-io --test dump write_ours -- --ignored  # one ignored probe
```

Heavy crates are faster with the `fast` profile, which keeps release optimisation but turns link-time optimisation off and uses 16 code-generation units:

```sh
cargo test --profile fast -p ck-model
```

Ignored tests are developer probes or documented open items. Probes read an environment variable (`STEP`, `CK_PART`, `PART`, `CASE`) naming the input to inspect, and print timings or dumps. Open items carry their reason in the attribute, so `cargo test -- --ignored` lists them:

- a blade-to-hub ring blend whose ball-centre solve diverges at a shallow dihedral;
- three near-tangent rail spacings of a marine-fuse case that leave an open edge;
- the sphere patch of a concave fillet corner, whose seam is sampled finer than its neighbour (30 T-junctions among 462 mesh edges);
- adjacent conic or curvature fillets at a corner of two spline sections (`blendFailed`);
- the torus eye of one imported part, dropped from the display mesh.

### Invariants the tests check

The same few properties recur across the suites. A result is accepted only if:

1. **It validates.** `validate` (the independent checker in `ck-check`) reports no issue, which covers closed manifold shells, Euler characteristic, loop closure and orientation, edge and vertex agreement with their faces, positive volume, and with `mesh: true` a watertight display mesh. See [topology and geometry checks](../validation/checks.md).
2. **It has the right numbers.** Volume, area and centre of mass equal a closed-form value (a prism, a revolved profile, a sphere) or a value computed another way, typically to 1e-6 relative. For free-form shapes, a numerical integral or a section-area integral is used.
3. **It survives a round trip.** STEP export followed by import keeps volume and area (1e-6), the face and edge counts, and a clean `validate`.
4. **It is deterministic.** A warm regeneration that reuses the cache is byte-identical to a cold one; repeated runs give the same bytes. The sine and cosine used by the kernel are ported (`ck_topo::trig`) because the platform math library can differ by one unit in the last place from the WebAssembly build.
5. **It never traps and never hangs.** Every error path must return a structured code. `crates/ck-model/tests/robust_*.rs` run documents on a worker thread with a time limit and fail on a panic, a hang, or an error with an empty code; the cases in `tools/cases/crash/` are replayed there.
6. **Untrusted input is refused, not crashed on.** Importers get truncation, bit-flip and splice fuzzing.

## Native and WebAssembly

The shipped kernel is the crate `ck-wasm` compiled to `wasm32-unknown-unknown`. Its exports are `ck_alloc`, `ck_request`, `ck_reply`, `ck_commit` and `ck_commit_len`; the host provides `ck_now`, `ck_unix_ms` and `ck_panic`. The same engine can run natively: tests call `ck_wasm::engine::install()` and then `ck_server::handle(bytes)`, the same entry point the exports use. So the Rust suites exercise the production request path without a browser.

Differences to keep in mind:

- **Panics.** The release profile sets `panic = "abort"`, so a panic in the module is a WebAssembly trap that cannot be caught. Test targets use `panic = "unwind"`. The `service` profile (release optimisation with unwinding) is for the native HTTP service, which catches a panic and ends only that session.
- **Clocks.** Time budgets (`featureTimeMs`, `regenTimeMs`) are active only where the host supplies a clock. Native tests without a clock are never cut short, which keeps unit tests deterministic; `robust_budget.rs` drives the budget with a counter clock.
- **Floating point.** Release builds use one code-generation unit, which gave the same results as more units and a smaller module.
- **Speed.** Native runs are roughly two to three times faster than the module in the same machine.

The Node scripts in `tools/` always run the built module through a small host (`tools/kernel.mjs`) that instantiates it, supplies the clock imports, and re-instantiates it after a trap. Build the module with `cargo build -p ck-wasm --target wasm32-unknown-unknown --release` (or `tools/build_wasm.sh`, which also runs `wasm-opt` when available), then pass it with `--wasm`; most scripts default to the release path.

The TypeScript packages (`packages/part-model`, `builder`, `kernel-client`, `driver`, `session` and `render`) have their own tests: `npm test` in `packages/` builds all of them and runs `node --test` over every compiled `*.test.js` file under `dist/*/src/`. One test walks every part under `tools/cases` and the BenchCAD parts and checks that each rebuilds through the builder to the same document and regenerates to the same result.

## What runs from a bare checkout

Some checks compare the kernel live against a second kernel build, called here the comparison build: parts of `tools/gate.mjs`, and `tools/edit_robustness_check.mjs`. The comparison build is a separate `kernel.wasm` that must be placed next to the checkout (the scripts look for `../../kernel.wasm` relative to `tools/`; `edit_robustness_check.mjs` also takes `--orig path`). It is not distributed with the repository. The gate runs without it in standalone mode (below); everything else here runs from a bare checkout plus a built module:

| Needs only the repository and a built module | Needs the comparison build |
|---|---|
| `cargo test` for every crate | `tools/gate.mjs` without `--standalone` (it opens the comparison build before running any metric) |
| `text_corpus`, `boolean_corpus`, `blend_corpus` | The gate metrics `wire`, `regen.*`, `mass.*`, `vol.*` and `step` (replies, volumes and STEP text compared with the comparison build's) |
| `lint_protocol`, `section_check`, `thread_check` | The gate metric `edit.robust` and `tools/edit_robustness_check.mjs` |
| `skp_check`, `sldprt_check`, `regen_cache_check` | |
| `tools/gate.mjs --standalone`: the metrics that judge the module alone (`valid.*`, `build.*`, `oracle.*`, `import.*`, `heal.repair`, `naming.stable`, `iface.*`, `bench.*`) | |
| The sweeps and `crash_fuzz`, run with `--wasm <module> --check <baseline>` against their committed baselines | |

Scripts that load a module take `--wasm`; several default to a path outside the checkout, so pass it explicitly (for example `--wasm target/wasm32-unknown-unknown/release/ck_wasm.wasm`).

## The regression gate

`tools/gate.mjs` runs the built module and the comparison build side by side over a fixed set of cases, computes a scoreboard of counts (items that pass out of items tried), compares it with the committed `tools/scoreboard.json`, prints each metric with its change, and exits with status 1 if a hard metric went down. With a comparison build next to the checkout it runs every metric; the numbers below are from that run. Without one (or with `--standalone`) it prints `standalone: no comparison build` and runs only the metrics that judge the module alone, which it compares with its own baseline, `tools/scoreboard.standalone.json` (`--update` writes that file in this mode). In standalone mode `valid.*` counts the parts that build and `build.*` counts how many do; the metrics that compare with the comparison build (`wire`, `regen.*`, `mass.*`, `vol.*`, `step`) and `edit.robust` are not run, so a standalone pass does not replace the full gate for a change to geometry or output formats.

```sh
node tools/gate.mjs [--quiet] [--standalone] [--wasm path/to/ck_wasm.wasm]   # all metrics need the comparison build, except with --standalone
node tools/gate.mjs --update      # rewrite the baseline, in the commit that improves a metric
```

Some metrics compare the two builds' replies (`wire`, `regen.*`, `mass.*`, `vol.*`, `step`) and the quick edit check (`edit.robust`); they are not listed here. The metrics that judge the kernel on its own, against closed-form values, round trips or ground-truth files, are:

| Metric | Kind | Committed count | What it counts |
|---|---|---|---|
| analytic-reference metric | hard | 59 / 59 | Volume, area, centre and bounding box against closed-form values derived from the feature parameters (1e-6) |
| `valid.brep` | hard | 280 / 319 | No feature error, positive volume, every edge bounds two faces, `validate` clean |
| `heal.repair` | hard | 1740 / 1745 | Damaged copies of the valid bodies (seven damage kinds) healed back to `validate` clean with the volume within 1e-6 |
| `import.roundtrip` | hard | 258 / 259 | STEP export then `importStep`: volume and area 1e-6, same face and edge counts, clean `validate` |
| `import.heal.noop` | hard | 258 / 258 | Healing an already valid import changes nothing |
| `import.sat` | hard | 5 / 5 | The five closed-form SAT fixtures import clean with the expected volume |
| `bench.benchcad`, `.step` | soft | 49 / 50, 45 / 50 | The BenchCAD parts against their ground truth, and the ground-truth STEP files through `importStep` |
| `bench.brepjs.step` | soft | 23 / 25 | The brepjs reference STEP files through `importStep` |
| `naming.stable` | soft | 214 / 216 | References to earlier faces still resolve to the same geometry after an upstream dimension edit |
| `iface.*` | soft | 58 / 58 each | Agent-query self-consistency (`describe`, `resolve`, `check`, `refDiff`) on every fourth part |

These too are computed inside `gate.mjs` and run in standalone mode; their scripts (`tools/bench/run.mjs`, `tools/naming_stability.mjs`, `tools/agent_iface.mjs`) can be run alone. Hard metrics fail the gate when they fall. Soft metrics print `WARN(soft)`. A metric not in the baseline prints `(new)`.

## Results of the checks run for this book

These runs used the module built from this repository and the commands shown:

| Command | Result |
|---|---|
| `node tools/text_corpus.mjs --quiet` | 126 / 126 cases ok (53 s) |
| `node tools/boolean_corpus.mjs --quiet` | 145 / 147 ok, 2 `booleanFailed` |
| `node tools/blend_corpus.mjs` | 142 / 150 ok, 0 failed, 6 known, 2 without a reference volume |
| `node tools/lint_protocol.mjs` | 120,635 requests over 57 request types, 0 violations |
| `node tools/edit_robustness_check.mjs --quick` (needs the comparison build) | 12 parts, 24 edits, 22 comparable, 0 failures |
| `node tools/section_check.mjs --quick`, `node tools/thread_check.mjs` | both ok |
| `node tools/regen_cache_check.mjs` | 2,827 checks ok (9.5 min) |
| `node tools/feature_sweep.mjs --samples 15 --seed 7 --wasm <module> --check tools/feature_sweep_baseline.json` | 0 regressions (120 ok) |
| `node tools/skp_check.mjs` | 17 sample files: 14 imported, 3 refused with `body.empty`, 0 traps |

## See also

- [Corpora and sweeps](corpora.md)
- [Robustness under edits](robustness.md)
- [Known limits](limits.md)
- [Topology and geometry checks](../validation/checks.md)
- [Crate map](../reference/crates.md)
