# Crash / hang fuzzer

`tools/crash_fuzz.mjs` generates seeded, schema-valid feature documents (2 to
8 features) with degenerate parameters and runs each case in its own worker.
A trap or timeout kills only that case. It then shrinks each distinct
trap/hang to a small document.

```sh
node tools/crash_fuzz.mjs --cases 5000 --seed 7 --update-baseline tools/crash_fuzz_baseline.json
node tools/crash_fuzz.mjs --cases 200 --seed 7 --no-shrink --json out.json
node tools/crash_fuzz.mjs --shrink-input out.json --update-baseline tools/crash_fuzz_baseline.json
```

`--timeout` is **CPU milliseconds** of the case (the worker thread's own CPU time, which is also the kernel's clock for
`--budget-ms`), not wall time: under load the wall time of one case varied by a large factor, so the number of slow cases
depended on the machine. A reply that used more CPU time than the limit counts as a `hang`; `--wall-factor` (default 6)
caps the wall time at that many times the limit, to kill a case that never returns. Each result carries its `cpuMs`.
`--check` takes an optional baseline path (`--check --json out.json`, as the gate calls it, no longer reads `--json` as one).

Classes: `ok`, `structured-error` (a useful error reply), `trap` (wasm
unreachable / panic), `hang` (per-case timeout), `invalid-result` (`ok` reply
but `validate` issues), `wrong-answer` (positive/area invariant broken).
`structured-error` is acceptable; traps and hangs are the priority.

## Result (kit wasm, seed 7, 5000 cases)

| class | count | share |
|---|---:|---:|
| ok | 662 | 13.2 % |
| structured-error | 4002 | 80.0 % |
| **trap** | **238** | **4.8 %** |
| **hang** | **74** | **1.5 %** |
| invalid-result | 24 | 0.5 % |
| wrong-answer | 0 | 0 % |

The 5000-case run takes about 4 minutes with 8 workers; shrinking all 23
distinct groups takes another ~2 minutes. `/tmp/fuzz5000-final.json` keeps the
features of every failure.

## Trap groups (15)

All traps are panics in `crates/ck-ops/src/surfacing/thicken.rs`; the first
eight are `thicken.rs:550:56`, the rest `thicken.rs:568:43`.

| cases | minimal id | base | minimal feature chain | panic |
|---:|---|---|---|---|
| 57 | `case_104` | surface | sketch, surfaceExtrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:thicken |
| 28 | `case_247` | surface | sketch, surfaceExtrude, thicken, mirror, sketch, extrude | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:draft |
| 28 | `case_456` | surface | sketch, surfaceExtrude, thicken, lpattern, sketch | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:extrude |
| 25 | `case_1` | surface | sketch, surfaceExtrude, thicken, chamfer, draft | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:chamfer |
| 24 | `case_362` | surface | sketch, surfaceExtrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:mirror |
| 21 | `case_498` | surface | sketch, surfaceExtrude, thicken, lpattern, sketch, extrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:shell |
| 20 | `case_6` | surface | sketch, surfaceExtrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:lpattern |
| 20 | `case_28` | surface | sketch, surfaceExtrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:550:56` no entry found for key:fillet |
| 3 | `case_828` | surface | sketch, surfaceExtrude, thicken, shell, lpattern, sketch, extrude, mirror | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:shell |
| 3 | `case_2960` | surface | sketch, surfaceExtrude, thicken, shell, draft, thicken | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:thicken |
| 2 | `case_191` | surface | sketch, surfaceExtrude, thicken | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:extrude |
| 2 | `case_255` | surface | sketch, surfaceExtrude, thicken, draft, chamfer, mirror | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:fillet |
| 2 | `case_835` | surface | sketch, surfaceExtrude, thicken, lpattern | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:mirror |
| 2 | `case_892` | surface | sketch, surfaceExtrude, thicken, shell | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:draft |
| 1 | `case_2273` | surface | sketch, surfaceExtrude, thicken, shell, draft, chamfer | `crates/ck-ops/src/surfacing/thicken.rs:568:43` no entry found for key:lpattern |

## Hang groups (8)

All timeouts are 1.5 s. The common shape is a pattern/mirror on a cylinder,
revolve or loft, or a later feature after one.

| cases | minimal id | base | minimal feature chain | last feature |
|---:|---|---|---|---|
| 18 | `case_646` | cyl | sketch, extrude, lpattern | `timeout`  |
| 13 | `case_351` | loft | sketch, plane, sketch, loft, lpattern, shell, thicken, sketch, extrude | `timeout`  |
| 10 | `case_886` | revolve | sketch, revolve, sketch, extrude, lpattern | `timeout`  |
| 9 | `case_49` | revolve | sketch, revolve, lpattern | `timeout`  |
| 7 | `case_34` | revolve | sketch, revolve, mirror | `timeout`  |
| 7 | `case_96` | cyl | sketch, extrude, lpattern | `timeout`  |
| 6 | `case_424` | loft | sketch, plane, sketch, loft, chamfer, sketch, extrude | `timeout`  |
| 4 | `case_799` | cyl | sketch, extrude, lpattern | `timeout`  |

## Invalid-result (24)
`TESSELLATION_OPEN` on mirrored/revolved bodies, and
`VOLUME_NONPOSITIVE` after a second extrude. These are not crashes; list in
`tools/crash_fuzz_baseline.json` under their case ids.

## Minimal repros

Every minimal document is in `tools/crash_fuzz_baseline.json` (`minimal`),
with its full feature list. The tables above are the 23 distinct groups;
`case_104` (57 cases) is the largest: a circular `surfaceExtrude` followed by
`thicken` panics at `thicken.rs:550`.

## Cone wasm (`c917f6ac`)
5000-case replay: 647 ok, 4037 structured-error, 238 trap, 39 hang, 24
invalid, 15 surface-only/degenerate (pass classes). Traps remain the same
`thicken.rs` root (line shifted to 647). 35 former hangs became structured
errors: `extrude` 9, `fillet` 8, `lpattern` 5, `mirror` 4, `thicken` 3,
`draft` 2, `shell` 2, `chamfer` 2. The 15 former `wrong-answer` cases replay
identically on the previous gated wasm and this one; they are surface-only or
degenerate documents, not a cone regression.

## After the robustness work

Traps 238 to 0, hangs mostly bounded by time budgets and a faster curve fit; see `docs/robustness.md` (results,
error codes, the rule, how to add a case). Minimal cases live in `tools/cases/crash/` and run in
`crates/ck-model/tests/robust_cases.rs`. New options: `--budget-ms N`, `--wasm-label`.
