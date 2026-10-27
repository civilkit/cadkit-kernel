# Architecture

This chapter describes how the kernel is divided into crates, how they depend on each other, and the rules the whole code base follows for numerics, errors, panics and threads.

## Layers

The kernel is a Cargo workspace (`crates/*`). Dependencies only point downwards in the table below, and no crate depends on a crate in a higher layer.

| layer | crates | what they hold |
|---|---|---|
| 0 | `ck-core`, `ck-json`, `ck-text` | Math and utilities (`ck-core`), the JSON reader and writer (`ck-json`), outline fonts and lettering (`ck-text`). None has dependencies. |
| 1 | `ck-geom` | Curves, surfaces and NURBS. Representation only: no topology. Depends on `ck-core`. |
| 2 | `ck-topo` | The B-rep data model, primitive construction (extrude, revolve), local edits, orientation, mass properties and point classification. Depends on `ck-core`, `ck-geom`, `ck-json`. |
| 3 | `ck-tess`, `ck-check` | The tessellator and the independent solid validator. Both read a body and never change it. |
| 4 | `ck-ops`, `ck-heal`, `ck-io`, `ck-skp`, `ck-sldprt` | Modelling operations (booleans, blends, loft, sweep, offset, shell, draft, surfacing, measurement), repair, STEP and ACIS SAT, and the SketchUp and SolidWorks readers. |
| 5 | `ck-model`, `ck-recog` | The document model and the regeneration program (`ck-model`), and feature recognition for imported solids (`ck-recog`). |
| 6 | `ck-server`, `ck-wasm`, `ck-native` | Request dispatch and replies (`ck-server`), the WebAssembly entry points (`ck-wasm`), the local HTTP service (`ck-native`). |

Some details that the table hides:

- `ck-ops` depends on `ck-tess` and `ck-check`: the section and probe requests cut the display mesh, and a few operations validate their own result.
- `ck-heal` depends on `ck-check`. A repair pass is driven by the findings of the validator, so it only touches what the validator reported.
- `ck-server` depends only on `ck-json`. It knows the envelope, the request table and the session state, and gets geometry through hooks that `ck-wasm` installs (`ck_wasm::engine::install`). Without the hooks a request that needs a body answers with an error.
- `ck-wasm` is the only crate that sees everything. It contains the glue that needs several crates at once: the regeneration hook, the regeneration cache, the view cache, the `validate` and `heal` requests, and the queries that read the snapshot of the last regeneration.

The crates map one to one to the chapters of this part: [the B-rep model](brep.md) is `ck-topo`, [geometry](geometry.md) is `ck-geom`, [operations](operations.md) is `ck-ops`, [tessellation](tessellation.md) is `ck-tess`, [regeneration](regeneration.md) is `ck-model` and the cache in `ck-wasm`. Validation is `ck-check` and `ck-heal`; see the [validation overview](../validation/overview.md).

### What `ck-core` provides

| module | contents |
|---|---|
| `linalg` | `Vec2`, `Vec3`, `Mat3`, `Mat4`, `Quat`, `Plane` and `Frame` (origin and three axes), the tolerance constants. |
| `xform` | `Xform`, a rigid or affine map `p -> r*p + t`. `a * b` applies `b` first. |
| `quad` | Real roots of quadratics, cubics and quartics, polished by Newton steps against the given coefficients. Used by closed-form intersections. |
| `kdtree` | A 3D tree for nearest, k-nearest, radius and weld queries. Ties are broken by lower index so results are deterministic. |
| `budget` | Wall-clock work budgets (see below). |
| `memo` | Content-keyed memo tables for pure geometric functions (see below). |
| `fxhash` | A small multiply-rotate hasher for integer-keyed maps. |
| `trace` | The debug trace that a request with `"debug": true` returns. |

The shared tolerance constants in `linalg` are `EPS_LINEAR = 1e-6`, `EPS_POINT = 1e-9`, `EPS_PARAM = 1e-12`, `EPS_MACHINE = 1e-15` and `EPS_COARSE = 1e-3`. Most operations use the first as their working tolerance; see [tolerances](../validation/tolerances.md).

## The code path of one request

```text
host -> ck_request -> ck_server::handle   envelope, dispatch on msg.type
                          |
                          +-> hooks (ck-wasm engine)
                                |-> regen: ck-model::regen -> features -> ck-ops / ck-topo
                                |         -> ck-tess (display mesh) -> reply JSON + buffers
                                '-> queries: read the snapshot of the tag
                                          (ck-topo mass, ck-ops measure, ck-check, ck-heal, ...)
```

A request carries a `tag`. The result of a `regen` is kept under that tag as a snapshot (the body, the sketches and the assembly state), and later queries on the same tag read it. The wire format is described in the [protocol part](../protocol/overview.md).

## Numerics and determinism

The kernel computes in IEEE-754 double precision (`f64`) throughout. Results are meant to be reproducible: the same document gives the same bytes on every run of one build. The following rules support that; the one place where platforms can differ is noted below.

- **No float reassociation.** The release profile uses one codegen unit and no fast-math. The compiler may not reorder floating-point sums, so a build's results do not depend on inlining decisions.
- **Same `sin` and `cos` in the angle-critical code.** The system math library differs between platforms in the last bit. `ck_topo::trig` is a port of the standard msun kernels, and revolve, draft and mate rotations call it instead of `f64::sin` and `f64::cos`, so a native build produces the same bits as the WebAssembly build for those. Other code calls the standard functions, which are the same on WebAssembly and may differ in the last bit on a native platform.
- **Guarded normalisation.** `Vec3::normalize` returns the zero vector, not NaN, when the length is not above `1e-300` or not finite. `try_normalize` returns `None` in that case. Callers that need to detect degenerate input use the second form.
- **Stable ordering.** Where a result is a list, it has a defined order: intersection results are sorted by the parameter on the first curve, the BVH pair traversal has a fixed order, and the tree queries break ties by index. Hash maps from `fxhash` are used only where iteration order is never observed.
- **Caches do not change results.** The content-keyed memo tables (`ck_core::memo`) are keyed by the exact bits of their inputs, so a hit returns the value the computation would have produced. The regeneration cache and the face-mesh cache are exact in the same sense; see [regeneration](regeneration.md).
- **Local origin for far parts.** A boolean whose operands are far from the origin compared with their size (centre farther than `1e4` diagonals) is computed in coordinates centred on the operands and moved back, because absolute tolerances such as `1e-6` are below the resolution of `f64` squared distances near `1e6` mm.

Floating-point tolerances are model-space distances in millimetres, not relative numbers, except where the text says "bounding-box relative"; see [tolerances](../validation/tolerances.md).

## Platform constraints

The kernel is compiled to `wasm32-unknown-unknown` and also runs natively (tests, command-line tools, the local service). It is not `no_std`: it uses `std` collections, strings and `thread_local!`. It does restrict itself to what a sandbox can offer:

- No file, network or process access in the kernel crates. Time is the only host service: the host passes `ck_now` and `ck_unix_ms`, and the work budgets and the request timer use those injected functions. The exceptions are debug switches (environment variables such as `CK_OFFSET_DEBUG` and stopwatch code behind them) that print diagnostics on native builds; they have no effect in the WebAssembly module.
- No `unsafe` code in `ck-core`, `ck-geom`, `ck-topo`, `ck-ops`, `ck-tess`, `ck-check`, `ck-heal` or `ck-model`. The only `unsafe` in the kernel proper is the optional logging import of the `trace` feature in `ck-tess`, which is off in normal builds. The WebAssembly entry points in `ck-wasm` use `unsafe` for the pointer interface, and `ck-skp` and `ck-sldprt` forbid it.
- No random numbers and no threads inside the kernel.
- JSON is handled by `ck-json`, which keeps object keys in insertion order and prints floats in the shortest form that round-trips.

The WebAssembly module exports five functions (`ck_alloc`, `ck_request`, `ck_reply`, `ck_commit`, `ck_commit_len`) and imports three (`ck_panic`, `ck_now`, `ck_unix_ms`); see [calling it from Node](../getting-started/node.md).

## Panics and traps

The release profile is built with `panic = "abort"`. A Rust panic in the WebAssembly module is therefore a trap (`unreachable`) that cannot be caught at the request boundary. What happens instead:

1. A panic hook installed on first use calls the host's `ck_panic(ptr, len)` with the panic message.
2. The module traps. The host sees a `WebAssembly.RuntimeError`, treats the instance as lost, and creates a new instance. All tags and caches of the old instance are gone.

Because a trap gives no structured feedback, the code base follows a rule instead of a safety net: no `unwrap()`, `expect()`, unchecked index or map lookup on data that comes from a document or from geometry. Failures are returned as errors with a code and, where possible, a hint. Every loop whose length depends on geometry has a bound or polls the work budget. The crash-fuzz run in the repository (5000 generated documents on a release build) ended with no traps, but its baseline still lists three open hangs (slow near-degenerate cases, which end in a structured result or a budget error when run alone but exceed the harness timeout under load); see [robustness under edits](../quality/robustness.md).

The native service is built with a separate `service` profile that keeps unwinding. Each session runs on its own worker thread; a panic is caught with `catch_unwind`, the session ends with the error `kernel panic`, and the log records only the source location, not document text.

### Work budgets

`ck_core::budget` implements two nested wall-clock limits that long loops poll:

| limit | request option | default |
|---|---|---|
| one feature | `opts.featureTimeMs` | 120000 ms (`0` = unlimited) |
| the whole regeneration | `opts.regenTimeMs` | none |

The limits are active only where the host installed a clock. The WebAssembly build wires the host's `ck_now`, so a host must return real time from it: under the book's own runner `ck_now` returns 0, so the budgets never fire there. A loop that finds its limit passed stops and returns its own `...BudgetExceeded` code; the regeneration driver turns any failure that happened while a limit was running out into `featureBudgetExceeded` or `regenBudgetExceeded`, with the elapsed and permitted milliseconds and the code the feature would have reported in `cause`. See [feature errors](../validation/diagnostics.md).

## Threads

One kernel instance is single-threaded. The kernel keeps its state (the session tags, caches, memo tables, the budget clock) in `thread_local!` variables, 51 of them in the kernel crates, so two instances never share anything and a host gets parallelism by running several instances.

- In the browser or Node, one WebAssembly instance per worker is the unit.
- In the native service one worker thread serves one session. The thread gets a large stack (256 MiB by default, `Config::stack`) because some recursive algorithms are deep on large bodies.
- Two process-wide atomic flags in `ck-ops::intersect::ss` switch intersection seeding variants, and a set of counters in `intersect::patch` exists for profiling. They are written at the start of a regeneration; they are the only shared mutable state in the kernel crates.

Multi-threaded geometry inside one request is not implemented (see [known limits](../quality/limits.md)).

## See also

- [The B-rep model](brep.md)
- [Operations](operations.md)
- [Regeneration and caching](regeneration.md)
- [Validation overview](../validation/overview.md)
- [Crate map](../reference/crates.md)
