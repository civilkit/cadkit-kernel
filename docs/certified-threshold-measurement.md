# Certified SSI marching: what the sine threshold costs

Measurement of commit 6b0dd33c (certified step check up to sine 0.6, a span that does not contract is proved in up to 8 boxes), 20 Nov 2026.
Scripts: `tools/bench/cert_timing.mjs` (wall and CPU per part, results), `tools/bench/cert_counts.mjs` (the gate's bench counts per wasm).

## Variants (wasm only, built on the CI box with `--checks none`)

| Name | Source | Commit |
|---|---|---|
| V0 | parent of 6b0dd33c, detached | 82c4312d |
| V1 | current main | 35d24698 |
| VA | main with `CLOSE_SIN = 0.12` (branch `cert-opt-a`) | 43a53f6e |
| VB | main, a step whose span proof fails is retried with the edge-span proof (`prove_span`: continued spline) before halving (branch `cert-opt-b`) | e322892e |

V0 is NOT a clean baseline for 6b0dd33c: 102 commits separate it from main (procedural geometry, hole, pattern, boolean changes). It gives
wrong propeller geometry (10 faces, volume 20113 against 23892) and is 18x slower than V1 on spur_gear and 4x on vented-louvre-case, so its
times on those parts say nothing about the threshold. VA (main with only the threshold put back) is the clean comparison.

## Timings (laptop, node, fresh kernel per run, 3 runs, min / median wall ms; CPU within a few percent)

Load average 4 to 6 during the batches (below 8). The gridfinity bin is the brepjs example `gridfinity-bin` (the benchcad
`gridfinity_bin_000166` is a 11 ms part on every variant).

| Part | V0 | V1 | VA | VB |
|---|---|---|---|---|
| gridfinity-bin (brepjs) | 389 / 398 | 2501 / 2603 | 484 / 552 | 4386 / 4424 |
| propeller_000192 | 5504 / 5519 (wrong part) | 8320 / 8725 | 7056 / 7281 | 9928 / 10113 |
| coil_spring_000026 | 2074 / 2082 | 2334 / 2336 | 2364 / 2451 | 2128 / 2416 |
| torsion_spring_000054 | 207 / 208 | 240 / 243 | 233 / 240 | 254 / 262 |
| rubber-foot (brepjs) | 40 / 41 | 90 / 100 | 90 / 98 | 97 / 102 |
| fluted-knob (brepjs) | 345 / 378 | 367 / 384 | 323 / 323 | 330 / 360 |
| vented-louvre-case (brepjs) | 2343 / 2416 | 568 / 573 | 591 / 657 | not run |
| spur_gear_000098 | 3204 / 3270 | 179 / 191 | 184 / 189 | 182 / 186 |
| bevel_gear_000437 | 349 / 361 | 112 / 132 | 134 / 231 | 189 / 224 |
| heat_sink_000029, worm_screw_000021, washer_000001 | within noise | | | |

A single-run screen of all 50 benchcad and 25 brepjs parts on V1 vs V0, VA, VB found no other part where the threshold matters
(the next largest V1 over VA differences: torsion spring 1.4x, bevel gear 1.4x, heat sink 1.2x, all under 0.15 s).
Only gridfinity-bin (5x) and propeller (1.2 to 1.3x) are tracer-bound.

## Results against V1

| | VA | VB |
|---|---|---|
| Volume, face count, validity, errors, all 50 benchcad + 25 brepjs parts | identical, except propeller volume differs by 1e-6 (same 53 faces) | identical |
| bench.benchcad / benchcad.step / brepjs.step | 49/50, 49/50, 24/25 (same parts) | 49/50, 49/50, 24/25 (same parts) |
| Sample counts of the traced curves | not measured (no tool exposes them) | not measured |

V0 for reference: bench.benchcad 48/50 (propeller fails), others the same.

## Reading

- VA recovers the whole gridfinity-bin cost (2.5 s to 0.48 s, within 25% of V0) and about 15% of propeller. Results are the same on every part tried.
- VB does not help: it is slower than V1 on gridfinity-bin (4.4 s) and propeller (9.9 s), because the failing proofs on analytic surfaces are
  proved twice (the edge proof repeats the failing one) and the failures are not edge failures. The spline-edge case is already handled in
  `check_curves` (`prove_span`); the tracer's halving at the edge is not what costs here.
- The cost is the 0.12 to 0.6 band: 6b0dd33c's own note says sampling is identical across 0.12, 0.3 and 0.6 on 47 shallow cases, so the
  extra proofs there buy no change in the trace on the parts measured, but the proofs and the spans split into boxes cost the time.

## Recommendation

Take A (`CLOSE_SIN = 0.12`) as a candidate; do not take B. Caveats:

- A reverts the stated purpose of 6b0dd33c's last step ("certify up to 0.6 again"): hops between branches at sines between 0.12 and 0.6 would no
  longer be caught by the tracer's step check. Its message reports no hop found in that band on its cases, but those 47 cases are not a
  proof, and `check_curves` (diagnostics) still proves and repairs unproved spans afterwards. A middle value (0.3) was not built or measured.
- The old reason for 0.12 (a blend tipping over an edge on glibc libm, blend_breakout_drafted) was the box failing at benign crossings, which
  6b0dd33c fixed; with A those crossings above 0.12 are not proved, so their steps are not halved either: the sampling is the uncertified march's.
  That is the sampling 6b0dd33c wanted, so A should not reintroduce it, but the glibc blend corpus has not been run.

**No full gate has been run on any variant.** The local runs above are the benchcad and brepjs bench counts on macOS libm only
(not the CI box's glibc libm), no crash fuzz, no cargo tests, no blend corpus. Any change to `ss.rs` needs the full gate before merging.

## Decision (20 Nov 2026, release session)

Keep the 0.6 threshold (main as it is). The absolute cost on a normal machine is small (gridfinity-bin 0.5 s to 2.5 s, propeller about 8 s), robustness
is preferred over speed, and the caveats above (hops between 0.12 and 0.6 not caught; blend_breakout_drafted on glibc never run on VA; no crash fuzz,
cargo tests or glibc corpus run) are the kind of risk not worth taking for a 2 to 5 times gain on two samples.

The branch `cert-opt-a` (43a53f6e, `CLOSE_SIN = 0.12`) is kept. Revisit it only if (1) the full gate including crash fuzz and the glibc
`blend_breakout_drafted` pass on it, and (2) a middle value (0.3) has been measured; or if users report slow regeneration on parts with many
B-spline patches. Option B (`cert-opt-b`) is dropped: it was slower than main on both parts.
