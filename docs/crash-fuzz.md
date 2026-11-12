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

## Baseline re-recorded (seed 7, 5000 cases, budget 400 ms, timeout 6000 CPU ms)

`tools/crash_fuzz_baseline.json` was re-recorded on the kernel after the regressions below were fixed (label
`fuzz-fix-56e05637`). Counts: ok 684, structured-error 4292, invalid-result 9 (was 24), surface-only 13, degenerate 2,
trap 0, hang 0 (was 3). `--check` reports 0 regressions. The recorded `minimal` list is empty because no trap or hang group is
left (the old minimal documents are in git history and the cases in `tools/cases/crash/` run in `robust_cases.rs`). The slowest
case of the run used 1.6 s of CPU time (`case_4781`), far under the 6 s limit; `case_4259` (the degenerate face of
kernel issue 21) now ends in a structured error after 0.12 s of CPU time.

Still `invalid-result`, all of them in the old baseline too: `TESSELLATION_OPEN` in `case_244`, `case_1714`, `case_3811`,
`case_4935`, and `EDGE_OFF_SURFACE` in `case_1517`, `case_1757`, `case_2140`, `case_3431`, `case_4236`. They are valid
inputs that give a body the validator rejects (an open mesh or an edge off its surface), not crashes; each needs its own
investigation.

## Fixed regressions

The earlier baseline had 24 `invalid-result` cases; a run on a later main had 87 to 95. The harness was not the cause: the
old and the CPU-clock harness give 94 and 95 on the same wasm. Three mechanisms:

1. The vertex snap in `ck-tess` (the harmonic patch fill commit `de97069e`) joined vertices within an absolute 1e-4 (and 1e-3
   in the parameters). On a face whose smallest extent is under about a tenth of a millimetre that joined every corner of
   every triangle (71 cases: thin or tiny plain extrudes, their patterns and mirrors, sweeps, thickened surfaces). The snap
   distances are capped at a thousandth of the face's smallest extent (test `small_plain_extrudes_mesh_closed`).
2. The same snap joined the inner and the outer rim of a ring whose width is under 1e-4 (a surface extruded to a cylinder and
   thickened by 1e-5: `case_3670`, kernel issue 46), so both end caps of the tube had no triangles. The snap is now also
   capped at a quarter of the smallest gap between two loops of the face (`boundary_loop_gap` in `ck-tess/src/face.rs`).
3. A depth of 0 or below is clamped to a 1e-6 mm slab (K22), which is a valid solid. Patterning, mirroring or otherwise
   combining it runs a boolean whose welding tolerance is also 1e-6, so the two ends of each side edge merged, the side
   faces collapsed, and the answer was an open or non-manifold shell, faces with no triangles, or dropped material, all
   as `ok` (`case_642`, `case_834`, `case_1114`, `case_3369`, `case_4392`; issues 46 and 56). The boolean refuses an
   operand that has an edge between two distinct vertices no farther apart than the weld tolerance (length and vertex distance
   <= `tol * (1 + 1e-3)`, the relative 1e-3 being rounding of the transforms that placed the operand) with
   `booleanFailed` plus `thickness` and a hint (`thin_edge` in `ck-ops/src/boolean/apply.rs`). The vertex pool merges points
   within `tol` and the clamp is `MIN_DEPTH == 1e-6 == tol`, so this is exactly the slab and anything the weld would collapse.
   The first version of the rule used `2 * tol` and refused a valid operand: a 1.2 micrometre sliver left by line-tangent
   cylinders (kernel issue 50, `tangent_wires_tube_wall.rs`) is between 1x and 2x `tol`, its ends stay apart in the pool, and
   a stepper-motor part failed on it. The slab itself still builds.
   Tests: `crates/ck-model/tests/fuzz_invalid_results.rs` (the five cases refuse or stay valid; a 1.2 to 3 micrometre slab is
   not refused).

`case_4392` was not caused by the tessellator commits `faa77119` and `be8f8987`: the same result is on their parent.
