# Robustness: no traps, no hangs

Purpose: an LLM drives the kernel and must get exact feedback about what it can and cannot do. A wasm trap
(panic) gives no feedback at all, and a request that never returns gives none either. So every failure is a
structured error with a precise code, and every long computation either finishes or stops with a structured
budget error.

`panic = "abort"` is set for the release profile, so a Rust panic is a wasm `unreachable` trap and cannot be
caught at the request boundary (`catch_unwind` does nothing under abort, and unwinding on
`wasm32-unknown-unknown` needs an unstable toolchain). There is therefore no generic safety net in
`crates/ck-wasm`: the rule below is the only defence.

## The rule

* No `unwrap()`, `expect()`, slice index or `map[&key]` on data that comes from a user document or from
  geometry (topology lookups, a curve fit, an intersection result, a parsed parameter). Use `get`, `first`,
  `ok_or_else` and return an error.
* The error has a precise code, a message saying WHAT failed and, where possible, a hint on what to change
  (see `shellUnsupported`, `blendOverflow`, `boolean_failed_params`: `FeatureError::with("hint", ...)`, or
  `ThickenError::detail(code, message)` for the ops crates).
* Do not just catch and ignore. If a missing key means a real logic bug, fix the logic and add a test (the
  thicken panic was one: the seam edge of a cylinder sheet had no offset image). If the input is genuinely
  unsupported, return the structured refusal.
* Every loop whose trip count depends on the geometry has a bound or a progress check. A loop that can run
  long polls the work budget (below) and returns an error when it is out of time.
* A hang is a result or a structured error. Slow is not a hang, but unbounded is.

## Work budgets (`ck_core::budget`)

Wall-clock limits, active only where the host installs a clock (`ck-wasm` does, from `ck_now`; native tests
and tools without a clock are never cut short, so unit tests stay deterministic):

| option (regen `opts`) | default | meaning |
|---|---|---|
| `featureTimeMs` | 30000 | one feature; `0` = unlimited |
| `regenTimeMs` | none | the whole document; `0`/absent = unlimited |

Loops poll `ck_core::budget::poll()` (cheap: reads the clock every 16th call) or `expired()` and stop when it
returns true. Polled today: every boolean attempt and the retry ladder, the imprint face-pair loop, the pattern
instance loop. A feature that fails while a limit is running out reports the limit, whatever its own code was.

## Error codes added

| code | when | params |
|---|---|---|
| `featureBudgetExceeded` | a feature ran past `featureTimeMs`; its result is not applied | `budgetMs`, `elapsedMs`, `hint`, `cause` (the code the feature would have reported) |
| `regenBudgetExceeded` | the document ran past `regenTimeMs`; this and the following features were not run | same |
| `booleanBudgetExceeded` | a boolean was stopped by a limit, or a failing boolean that had already run 12 s did not start its three robust retries | `budgetMs`, `elapsedMs`, `hint` |
| `thickenFailed` (new details) | thicken: an edge has no offset image, the offset boundary does not chain, the offset vertices do not weld (sheet extremely large or small against the tolerance), or the volume is not positive | `detail` (message plus hint) |

## Fixed in this work

* thicken of a seamed cylinder sheet panicked (238 of the 5000 fuzz cases): the offset layer never imaged the
  cylinder's seam edge. The seam now moves radially with the surface, and a loop holding a seam keeps the sheet's
  order and directions (head-to-tail chaining cannot orient closed rings). The free-form thicken and the trim
  snapshots no longer index maps by key.
* `ck_geom::fit::interpolate_with` built a dense (n+1)^2 matrix. For the thousands of samples of a long projected
  or propagated curve this is gigabytes: it made the fillet sweep template `rrect_loop` run for minutes and then
  trap, and dominated pattern-then-fillet documents (33 s to 10 s). It now solves from row windows with the same
  operations in the same order (bit-identical, tested). `fit.rs` still has dense systems in
  `interpolate_cubic_tangents` and `approximate_with`; they are not on a failing path yet.
* Time budgets, above.

## Tests

`crates/ck-model/tests/robust_*.rs`; the shared runner is `tests/robust_common/mod.rs` (runs a regen on a worker
thread with a time limit, so a hang fails the test; `ck_core::budget` is driven by a counter clock in
`robust_budget.rs`). `robust_cases.rs` runs every file in `tools/cases/crash/` and fails on a panic, a hang or an
error with an empty code.

## Adding a fuzz case

1. Find it: `node tools/crash_fuzz.mjs --cases 5000 --seed 7 --budget-ms 400 --timeout 6000 --json out.json`
   (omit `--no-shrink` to shrink every distinct trap/hang group; or `--shrink-input out.json`). `--budget-ms N`
   sets `featureTimeMs=N`, `regenTimeMs=2N`; leave it off to test the default behaviour with `--timeout 30000`.
   The worker uses a real clock, which the kernel budgets need.
2. Save the minimal document (`minimal[].features` of the output) as
   `tools/cases/crash/<id>.json`: `{"id":"...","was":"trap|hang","key":"short description","features":[...]}`.
   No private paths, hosts or customer data in it.
3. Run it natively: `cargo test --profile fast -p ck-model --test robust_cases` (all cases) or point
   `ROBUST_CASE_DIR` at a directory to try a case before committing; `ROBUST_TIMING=1` prints features that take
   over 0.3 s, `ROBUST_VALIDATE=1` also validates the result.
4. Add an assertion for the intended outcome in a `robust_<area>.rs` test (a valid result, or a specific error
   code), not only "does not panic".
5. Refresh the baseline: `node tools/crash_fuzz.mjs --cases 5000 --seed 7 --budget-ms 400 --timeout 6000
   --update-baseline tools/crash_fuzz_baseline.json --wasm-label <label> --wasm <wasm>`. `--check` exits non-zero
   when a case that was ok or a structured error is now a trap, hang or invalid result.
