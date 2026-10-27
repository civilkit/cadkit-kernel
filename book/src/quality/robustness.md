# Robustness under edits

A kernel driven by a person nudging one number, or by a program editing a part in a loop, must not turn a model that builds into a broken one, must not crash, and must not hang. This chapter describes the rules the code follows and the checks that enforce them: structured errors and work budgets, the crash fuzzer, the single-parameter edit check, and the determinism and concurrency checks.

## The rule: errors are values

A failed request must return a precise error and leave the kernel usable. The release module is built with `panic = "abort"`, so a Rust panic is a WebAssembly trap with no way to catch it at the request boundary. There is no generic safety net; the defence is this rule:

- No `unwrap()`, `expect()`, slice index or map lookup by key on data that comes from a document or from geometry (a topology lookup, a curve fit, an intersection result, a parsed parameter). Use `get`, `first` or `ok_or_else` and return an error.
- The error has a code, a message saying what failed and, where possible, a hint on what to change (`shellUnsupported`, `blendOverflow`, `booleanFailed` with its parameters).
- If a missing key means a logic bug, fix the logic and add a test. If the input is genuinely unsupported, return a structured refusal.
- Every loop whose trip count depends on geometry has a bound or a progress check. A loop that can run long polls the work budget.
- A hang is a result or a structured error. Slow is allowed; unbounded is not.

## Work budgets

Wall-clock limits are active only where the host installs a clock (`ck-wasm` takes it from `ck_now`; native tests without a clock are never cut short). They are options on the `regen` request:

| Option | Default | Meaning |
|---|---|---|
| `opts.featureTimeMs` | 120000 | Time allowed for one feature; `0` means unlimited |
| `opts.regenTimeMs` | none | Time allowed for the whole document; `0` or absent means unlimited |

The feature default is large so that a result does not depend on machine load: a pattern that takes 5 s on a quiet machine took over 30 s on a busy one and failed with a budget error. Choose `regenTimeMs` larger than `featureTimeMs`. The budget is polled in every boolean attempt and its retry ladder, the face-pair imprint loop and the pattern instance loop.

| Code | When | Parameters |
|---|---|---|
| `featureBudgetExceeded` | A feature ran past `featureTimeMs`; its result is not applied | `budgetMs`, `elapsedMs`, `hint`, `cause` (the code the feature would have reported) |
| `regenBudgetExceeded` | The document ran past `regenTimeMs`; this and the following features did not run | the same |
| `booleanBudgetExceeded` | A boolean was stopped by a limit, or a failing boolean that had already run 12 s did not start its three robust retries | `budgetMs`, `elapsedMs`, `hint` |

A feature that fails while a limit is running out reports the limit, whatever its own code was. Slow cases still exist, for example a linear pattern with count 100 of a revolve followed by a fillet, which takes 5 to 30 s on a quiet machine; it ends in a result or `featureBudgetExceeded`, never a trap. The cost is the boolean with a 100-instance tool, a matter of speed rather than a missing bound.

## The crash fuzzer

`tools/crash_fuzz.mjs` generates seeded, schema-valid documents of 2 to 8 features with degenerate parameters and runs each in its own worker, so a trap or timeout kills only that case. It then shrinks every distinct trap or hang to a small document.

```sh
node tools/crash_fuzz.mjs --cases 5000 --seed 7 --budget-ms 400 --timeout 6000 --json out.json
node tools/crash_fuzz.mjs --cases 200 --seed 7 --no-shrink --json out.json
node tools/crash_fuzz.mjs --cases 5000 --seed 7 --check tools/crash_fuzz_baseline.json
```

Classes:

| Class | Meaning | Acceptable |
|---|---|---|
| `ok` | A valid result | yes |
| `structured-error` | A useful error reply | yes |
| `trap` | WebAssembly `unreachable` or panic | no |
| `hang` | Per-case timeout | no; 3 remain open (below) |
| `invalid-result` | An `ok` reply whose `validate` has issues | tracked |
| `wrong-answer` | A positive-volume or area invariant broken | no |
| `surface-only`, `degenerate` | Documents with no solid, or with noise-sized bodies | yes |

The committed baseline of 5000 cases (seed 7, `--budget-ms 400`) is 668 ok, 4290 structured errors, 3 hangs, 24 invalid results, 13 surface-only and 2 degenerate documents, and no traps. The 3 hangs (`case_34`, `case_644`, `case_2668`) are an open defect, not a pass: near-degenerate bodies (a thin revolve with mirror and chamfers, for example) whose display mesh and validation are very slow. They exceed the 6 s per-case timeout when eight workers share the machine and take 2 to 3.5 s alone. `--check` fails only on new traps, hangs and invalid results. The first run of this fuzzer found 238 traps, all one root cause: a thicken of a seamed cylinder sheet whose seam edge had no offset image. It was fixed, along with a dense-matrix curve fit that used gigabytes for long projected curves. The 24 invalid results are `TESSELLATION_OPEN`, `VOLUME_NONPOSITIVE` and `EDGE_OFF_SURFACE` on mirrored or revolved near-degenerate bodies; they are not crashes.

To add a case: find it with the fuzzer, save the minimal document (`minimal[].features` of the output) as `tools/cases/crash/<id>.json` with `{"id","was","key","features"}` and no private data, run it natively with `cargo test --profile fast -p ck-model --test robust_cases`, add an assertion for the intended outcome in a `robust_<area>.rs` test (a valid result or a specific error code, not only "does not panic"), and refresh the baseline with `--update-baseline`. `--check` exits non-zero when a case that was `ok` or a structured error is now a trap, hang or invalid result. The Rust side is `crates/ck-model/tests/robust_*.rs`: the shared runner executes a regen on a worker thread with a time limit, and `robust_cases.rs` replays every file in `tools/cases/crash/`.

The protocol itself is fuzzed by `tools/lint_protocol.mjs`: it sends every request type (taken from the dispatch table) with a minimal valid body, malformed envelopes and field fuzz (wrong types, huge numbers, empty arrays and objects), and requires a well-formed reply for each. In this chapter's run: 120,635 requests over 57 types, 0 violations.

## Edit robustness

`tools/edit_robustness_check.mjs` applies single-parameter edits and applies the same edit to the kernel and to the comparison build, a second kernel build supplied separately and not distributed with the repository (placed at `../../kernel.wasm` relative to `tools/`, or given with `--orig`). It cannot be run from a bare checkout. For each edit:

- **Parts:** `tools/cases/*.ckpart` and the 50 BenchCAD parts.
- **Edits:** for a spread of features per part (default 5), the first numeric leaf (preferring depth, offset, angle, radius, distance, count, height, width, thickness) multiplied by 0.8, 1.25 and 1.5. Edits that make the input nonsense (an arc whose endpoints stop being equidistant from its centre, crossing sketch lines) are skipped.
- **A case counts** when the comparison build regenerates the edited part with no feature error. The kernel fails the case when it then has a feature error, a trap, a hang, or a volume more than `--vol-tol` (default 1e-3 relative) from the comparison build's. A part whose unedited volume already differs between the two builds is judged on errors only.
- Each part runs in a worker process (`--jobs`, `--timeout`). `tools/edit_robustness_known.json` lists understood failures, each with its evidence; anything else fails the run.

`--quick` (12 small parts, 2 features, factor 1.25, about 7 s) is the gate metric `edit.robust`. In a run with the comparison build: 24 edits, 2 skipped because the comparison build itself errors, 22 comparable, 0 failures. These counts reproduce only with the comparison build.

The full run after the fixes, with the comparison build: 741 cases, 116 skipped, 625 comparable, 5 failures, all in the known list. Before the fixes it was 39 failures in 636. The fixed causes, each with a regression test in `crates/ck-model/tests`, show what edits tend to break:

1. Shell cavities that overshoot an opened face by a whole wall: free-form walls continue past their domain with a second-order polynomial, so neighbours drift apart with the square of the overshoot. The cavity now overshoots by a quarter of the wall.
2. A boolean on a revolved sphere whose seam-only loop is the whole parameter domain.
3. Fillet and chamfer picks by face names that shift when `~k` numbering of a shell's inner and outer faces changes after an edit: fall back to the same face families and keep all picks on one connected rim.
4. Zero-length closed edge remnants of a conic cut at its seam.

Open items from the same notes: a circular pattern of lofted blades fails with `booleanFailed` for some angles (two blades 170 degrees apart, four at 100, five and eight equal blades over 360), and one twisted-drill partial revolve cut differs by 0.23 percent with no ground truth.

## Determinism

The kernel is deterministic for a given input and module: the same document gives the same bytes. Checks:

- `tools/regen_cache_check.mjs` verifies that a warm regeneration (a cache hit on a document prefix) is byte-identical to a cold regeneration of the same document in a fresh instance. It covers every case part and BenchCAD part: a repeated identical request (full hit), a changed first, middle and last feature, removing the last feature, remove then re-add (undo), swapping two adjacent features, renames, a tiny cache cap (eviction must not change results), changed options, and `opts.noCache`. In this chapter's run: 2,827 checks ok in 9.5 minutes. `--slow` adds the 140 s cold comparisons of the largest sample.
- The sine and cosine used in geometry are ported (`ck_topo::trig`) so native and WebAssembly builds agree to the last bit where the platform math library would differ by one unit in the last place.
- The fonts used for lettering are compiled in; no system data is read.
- The STEP writer's only host-dependent value is the time stamp in the file header, taken from the host clock.

## Parallelism

The kernel itself is single-threaded in WebAssembly. State is kept in thread-locals, one set per kernel instance:

- In a browser or worker, one module instance is one session.
- The native service (`ck-native`) runs one worker thread per session, each with its own kernel state (regen cache, tags, reply buffer) and a 256 MiB stack, because regen recurses deeply. Its test checks that sessions do not share state and that a reply equals the direct call.
- The fuzzer, sweeps and edit check use worker processes or threads, one kernel instance each.

Multithreaded booleans are an open item (a trial with native threads gave a 1.38 times speed-up), listed in [known limits](limits.md). The font table for lettering is a process-wide registry (a mutex-guarded static), so it is per instance in a WebAssembly module but shared by all sessions of one native service process.

## See also

- [Testing](testing.md)
- [Corpora and sweeps](corpora.md)
- [Known limits](limits.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
- [Regeneration and caching](../kernel/regeneration.md)
