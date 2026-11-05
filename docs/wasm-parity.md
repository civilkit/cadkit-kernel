# Wasm parity of the fillet table

The fillet regression table (`crates/ck-model/tests/blend_table.rs`, `docs/blend-table.md`) runs natively; the product
runs the kernel as wasm. `tools/wasm_parity.sh` runs every table case through the wasm the way the app does and compares
it with the native table, case by case.

## Running

```
tools/wasm_parity.sh                          # whole table: wasm build + native table + wasm run, on the CI box (~10 min)
tools/wasm_parity.sh --case daisy-ring-n6     # the table's filter (=id, ^prefix, substring; comma = or)
tools/wasm_parity.sh --musl-libm              # native table with the wasm's libm: leaves only the non-libm differences
tools/wasm_parity.sh --wasm-only              # no native table, no comparison (traps and failed faces only)
```

Output: one line per difference, trap or failed face, then the counts:

```
DIFF daisy-hub-n8-T-r0.5 native blendFailed (the result is not a closed solid: ...) vs wasm ok
TRAP <id> [fresh] regen: unreachable panic=panicked at crates/ck-tess/src/face.rs:...
FAILED-FACE <id> [cached] F90.B11~1: face has no mesh triangles (tessellation failed: ...)
wasm parity: 162 cases, wasm ok 33, 12 differing (1 fresh vs cached, 1 over the time budget), 0 traps, 0 failed faces, 252 mesh notes
```

The per-case JSON (both runs, native outcome, mesh notes) is in `target/wasm-parity/parity/parity.json`, the remote log
in `target/wasm-parity/last.log`. The exit code is non-zero when anything differs or traps.

## What it does

`tools/wasm_parity.mjs` (node, worker threads, 300 s per case) runs each case twice in the wasm, with a real clock as
the app's worker has (the per-feature time budget is live):

- `fresh`: a new instance, one `regen` of the whole document (base features, the optional first fillet, the fillet);
- `cached`: the base (+ first fillet) regenerated first in the same instance, then the whole document, which resumes
  from the regen cache like an edit in the app.

The `regen` reply carries the display tessellation, so a tessellator panic shows as a trap there. Then `validate` with
`mesh:false` (the native table's validity check), `massProps` (volume) and `validate` with the mesh witness
(`TESSELLATION_OPEN` issues = faces the display tessellator failed on; `meshNotes` are counted only).

A case differs when the native and wasm outcomes differ (`ok` / `invalid` / the error code), when both built and the
volumes differ by more than 1e-6 relative, when both refused with different details, or when `fresh` and `cached`
disagree. The native side is the `BLEND_JSON` of a table run made in the same remote run (`remote_build.sh --checks
wasmparity` compares only with a table written by that run).

## Findings (kernel main 2951c79, 2026-11-03)

166 cases: wasm builds 33, native 31; 15 differ (16 in one run of the 162-case table at 730f5a8 that also had the
time-budget case), 0 traps, 0 failed faces.

| case | native | wasm | cause |
|---|---|---|---|
| daisy-hub-n5-T-r1 | volume 12169.647184 | 12171.143121 | libm |
| daisy-hub-n7-T-r0.5 | volume 14272.000734 | 14271.368088 | libm |
| daisy-hub-n5-T-r2, daisy-hub-n7-T-r2 | missingEdge | blendFailed | libm |
| daisy-hub-n8-T-r0.5 | blendFailed (not a closed solid) | ok (volume short of the reference) | libm |
| daisy-ring-n5-T-r0.8-ref | blendFailed | ok (volume short of the reference) | libm |
| daisy-hub-n8-rim-then-wall-r0.5/0.3/0.2/0.1 | blendFailed (not a closed solid) | blendUnsupported (singular centre) | libm |
| daisy-ring-n6-T-r0.5, daisy-ring-n6-rim-then-wall-r0.5/0.3/0.2/0.1 | blendFailed, no detail | blendFailed (not a closed solid: 8 edges ...) | not libm, open |
| cone-pocket-rim-oneside-r2 | blendFailed (74 s) | featureBudgetExceeded (fresh, 121 s); blendFailed (cached) | time budget |

None of them changes a table verdict: every case that differs is `expected_fail` or `hard` on both sides (the wasm `ok`
cases fail the reference volume).

### libm (10 cases)

The wasm32 build takes `sin`, `cos`, `atan2`, `acos`, `exp`, `pow`, ... from compiler-builtins' port of musl's libm;
the native Linux test binary calls glibc's (and macOS's libm on a dev machine), which differ in the last ulp for some
arguments. `ck_topo::trig` already pins `sin`/`cos` for the revolve frames, but the blend code calls the std functions.
An ulp in a blend's centre or spring computation moves a vertex, and a marginal case flips between built, refused at
the validity gate, or refused earlier.

Evidence: `tools/libm_shim` is an LD_PRELOAD library that replaces the transcendental functions the native test
binary imports from glibc (`nm -D --undefined-only`) by the `libm` crate (the same musl port). With it
(`--musl-libm`) the native table reproduces the wasm for all ten: the same outcomes and bit-identical volumes on every
case that builds on both sides. Perturbing the hash of the kernel's Fx maps (iteration order) changed nothing.
`tools/wasm_parity.sh --musl-libm` then reports one difference only on the 162-case table (daisy-ring-n6-T-r0.5 below; the
cone-pocket run stayed under the budget that time).

### daisy-ring-n6-T-r0.5 and the n6 rim-then-wall cases (not libm)

With the shim, every native attempt still fails inside the blend (`blend fail: check: [NonManifoldEdge ...]` in the
`CK_BLEND_DEBUG` trace), so the refusal has no detail; in the wasm one attempt gets through that check and the validity
gate refuses its result (8 edges not shared by exactly two faces). Some other platform difference reaches the blend's
own check on this case (the wasm is 32-bit: `usize` width, hashbrown's probing group width, unstable-sort tie order,
or a ±0 from `min`/`max` are candidates); not pinned down. The outcome (refused) is the same.

### cone-pocket-rim-oneside-r2 (time budget)

The fillet runs 74 s natively and about 120 s in the wasm on the box; the app's worker installs a clock, so the
120 s per-feature budget (`ck_core::budget::DEFAULT_FEATURE_MS`) ends it with `featureBudgetExceeded`. Native tests
install no clock and never time out. Which side of 120 s a run lands on depends on the machine and its load (the cached
run finished at 93 s). The three cone-pocket-rim-oneside cases all take 100-120 s in the wasm.

## The tessellator trap (face.rs:689)

No case traps or fails a face today: the validity gate refuses the bodies the trap came from. To find them,
`crates/ck-model/tests/blend_tess.rs` (off unless `BLEND_TESS=1`) meshes every table result at the display tolerance
and at every `exportMesh` quality, catching panics; `CK_BLEND_GATE=0` meshes the raw results the gate refuses,
`BLEND_TESS_SCALES` adds runs with the fillet radii scaled, `BLEND_TESS_AREA=1` lists faces whose mesh area is more
than 5 % off the exact area, and `CK_TESS_DBG=1` prints `an open chain was emptied by the junction drop` when the path
of the old trap is taken. Run it with the wasm's libm (`--musl-libm`):

```
tools/remote_build.sh --checks none --no-wasm --no-sync-back --musl-libm --cargo-test model --test blend_tess \
  --env BLEND_TESS=1 --env CK_BLEND_GATE=0 --env CK_TESS_DBG=1 --env BLEND_TESS_SCALES=0.5,0.7,0.8,0.9,0.95,1.05,1.1,1.25,1.5
```

Over 1620 bodies (162 cases x 10 radii) the path is taken only with the gate off: with the wasm's libm on
daisy-ring-n6-T-r0.5 at radius 0.475 and daisy-ring-n6-T-r1 at 0.9 (faces F90.B3~1, F90.B7~1, F90.B11~1), and with
glibc (a finer radius sweep) on daisy-ring-n6-T-r0.5 at 0.505 (faces F90.B3~1, F90.B3~2, F90.B19~2) and 0.6.

Why those faces fail: the blend face's own loop uses two edges twice each (slits across the band just below the top
rim, at v about 1.54-1.56, between u = pi/3 and 1.416), with sliver edges (0.003-0.02 rad) between them. The
tessellator reads an edge used twice by one face as a seam and cuts the loop there; the piece between the two slits is
one short circle-chain edge sampled at its two ends only, both of them merged-circle junctions, so the junction drop
leaves an empty chain. Before 9b26d90 the segment walk read `chain[usize::MAX]` (the trap); now the first attempt
refuses the face ("boundary collapsed") and the escalation (trim ends welded at 1e-6) meshes it with the right area.

`crates/ck-model/tests/blend_tess_empty_chain.rs` is the regression test: the 0.505 body (gate off, set in the test),
meshed with no failed face and the three faces' areas right. With the face.rs of 9b26d90's parent it panics at
`face.rs:689:40: index out of bounds: the len is 0 but the index is 0`, the trap of the report, now reproduced
natively. The root cause is the blend output (a face that touches itself along slit edges), which the gate refuses
(the wasm then reports `blendFailed`, the result is not a closed solid); nothing more to fix in ck-tess.
