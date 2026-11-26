# Boolean timing: a cylinder cut into a box

Measured at commit db8f43c7 (branch `boolperf`). This note reports our own numbers only; it makes no statement about any other CAD
kernel. Harnesses: `tools/bench_boolean_cyl.mjs` (wasm in node, the app's path) and
`crates/ck-model/tests/bench_boolean_cyl.rs` (native, `#[ignore]`d:
`cargo-queue test --profile fast -p ck-model --test bench_boolean_cyl -- --ignored --nocapture`).
Every figure is the median of 20 runs after 3 warm-up runs. Part: box 100 x 60 x 40, cylinder r 10; plates are 300 x 200 x 10 with d10 holes.

## Machines

* Mac: Apple M1, 8 cores, idle apart from the run. Native = `--profile fast` (release speed, no LTO); wasm = `--release` build in node.
* CI box: Xeon E5520 (2.27 GHz, 2009-era), 16 threads, load average 14 to 30 during the run (other jobs were running), so the box figures are
  an upper bound and noisy. Wasm in node only; the native test was not run there (the box runner does not pass `--ignored`).

## (a) The boolean call alone, native, Mac (`ck_ops::boolean::boolean`, tol 1e-6)

| operation | ms |
|---|---|
| cut, cylinder through | 0.65 |
| cut, cylinder blind 20 | 0.44 |
| union, cylinder through (sticks out both ends) | 0.70 |
| union, boss h20 on top | 0.63 |
| intersect, cylinder through | 0.58 |
| intersect, cylinder wholly inside the box | 2.93 |

Building the operands (box extrude 0.016 ms, cylinder extrude 0.006 ms) is negligible.

## (b) The feature path (regen of the whole part, cold), Mac, ms

| case | native | wasm (node) | wasm, CI box (loaded) |
|---|---|---|---|
| box only | 0.04 | 1.2 | 3.3 |
| sketch circle + extrude cut, through | 1.24 | 9.7 | 32.9 |
| sketch circle + extrude cut, blind 20 | 0.87 | 9.1 | 24.1 |
| extrude boss (union) | 1.15 | 7.7 | 16.3 |
| hole feature d20, through | 0.93 | 8.3 | 15.8 |
| hole feature d20, blind 20 | 0.64 | 8.5 | 20.6 |
| plate only | 0.06 | 0.6 | 2.4 |
| plate, 1 hole (circle in one cut) | 0.72 | 7.5 | 11.6 |
| plate, 10 circles in one cut | 8.8 | 32 | 91 |
| plate, 50 circles in one cut | 45 | 209 | 501 |
| plate, 10 holes: hole feature + linear pattern | 6.6 | 39 | 109 |
| plate, 50 holes: hole feature + linear pattern | 1973 | 4689 | 8635 |
| plate, 50 separate hole features | 372 | 1046 | 3100 |

The wasm column is a different thing from the native one: it is the whole `regen` message, which also tessellates the result, builds the
reply JSON and crosses the wasm boundary, so about 7 to 9 ms of the wasm figure for a single cut is not the boolean (the boolean call is
0.4 to 0.7 ms). Intersect has no feature-path row: an extrude with `op` other than `cut` behaves as `add`.

## (c) Tolerance knob

Booleans are exact (surfaces are intersected analytically; the only knob is the geometric tolerance argument, 1e-6 here). The faceting is
the tessellation tolerance, chosen at mesh time (export presets), and does not change the boolean. Tessellating the box-minus-cylinder result,
native Mac: coarse (chord 0.49, 64 triangles) 1.2 ms; normal (chord 0.049, 148 triangles) 2.2 ms; fine (chord 0.012, 276 triangles) 4.6 ms.

## What looks slow

* A single cylinder cut, union or hole is not slow: under 1.3 ms native, under 10 ms through wasm on an idle M1 (about 15 to 33 ms on the loaded CI box).
* Many holes through a linear pattern are. 10 pattern holes cost 6.6 ms native, 50 cost 1973 ms (5x the holes, 300x the time), and 50 holes made by
  50 separate hole features cost 372 ms, so the pattern is 5x slower than doing the same cuts one by one. A single sketch with 50 circles in one cut is 45 ms.
  Run-to-run spread of the 50-hole pattern in wasm is large (2.0 to 6.5 s on the Mac).
* Profile (macOS `sample`, 10 s inside the 50-hole pattern, native): about 94 % of the time is `pattern::accumulate` calling `boolean` one copy
  at a time; inside it `boolean_at_merge` calls `ck_topo::region::classify`, which integrates every face of the growing body
  (`mass::face_integral_with` plus `Surface::derivs`, and `winding_number`). The classification cost grows with the face count of the accumulated
  plate, so the total is roughly quadratic or worse in the copy count. Not changed by this note; it is the lead if the pattern case matters.
* The intersect with a cylinder wholly inside the box (2.9 ms) is 4 to 5x the other operations.

## Not measured

* Native on the CI box (the box runner does not pass `--ignored`); the box wasm figures were taken while the box was loaded.
* Native with the shipped `--release` (LTO) profile: native figures use `--profile fast`.
* Intersect through the feature path, and any comparison with another kernel (we have no access to its timings).
