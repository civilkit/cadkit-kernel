# Fillet regression table

`crates/ck-model/tests/blend_table.rs` runs the synthetic fillet cases in `crates/ck-model/tests/fixtures/blend_cases.json`
(written by `tools/gen_blend_fixtures.mjs`) and judges each against `fixtures/blend_expect.json`.

## Running

On the CI box (prints only the per-case lines and the summary; the full log is kept in `target/blend-table/last.log`):

```
tools/blend_table.sh --case '=daisy-hub-n5-V-r1'   # one case (quote it: `=` at a word start is special in zsh)
tools/blend_table.sh --case -V-r                   # a family: every id containing "-V-r" (the daisy V sets)
tools/blend_table.sh --case hole-corner,pipe-tee   # comma = or
tools/blend_table.sh --mode fail                   # only the cases expected to fail
tools/blend_table.sh --json --debug                # + target/blend-table/blend/{table.json,debug/<id>.log}
```

Filter terms: `=id` matches that id only, `^p` ids starting with `p`, anything else is a substring of the id.
Locally the same variables work on `cargo test -p ck-model --profile fast --test blend_table -- --nocapture`:
`BLEND_CASE`, `BLEND_MODE=fail|pass`, `BLEND_JSON=<file>`, `BLEND_DEBUG_DIR=<dir>`, `BLEND_TABLE_OUT=<file>`.

A filtered run judges only the cases it ran. Without a filter every case runs and an expectation without a case
is also reported. Once the box has built the branch, one case or one family comes back in about 20 s
(rsync about 8 s, the incremental rebuild about 6 s); the full table takes about 100 s, 80 s of which is the table.

## Reading the output

```
daisy-hub-n5-T-r1 XFAIL volume volume 12169.6472 vs reference 12174.4127 (base 12226.0455) dvol=+9.230%  t=1.21
<id>              <status> <outcome> <detail, at most 80 chars>                            <dvol>       <seconds>
```

- status: `PASS` (expected to pass, passes), `XFAIL` (expected to fail, fails), `REGRESSION` (expected to pass,
  fails), `NEWPASS` (expected to fail, now passes: flip it). `NOEXPECT`: the case has no expectation.
- outcome: `ok`, the refusal's error code (`blendFailed`, `blendUnsupported`, `blendOverflow`, `missingEdge` ...),
  `invalid` (the body fails validation), `novolume`, `sharp` (a case marked `"sharp": true`, or any case with
  `BLEND_SHARP=1`: a selected edge still has points on the result's boundary, i.e. it was not rounded), or `volume`
  (the volume disagrees with the reference).
- dvol: how far the volume removed (or added) is from the reference's, in % of the reference's; positive means the
  volume is below the reference's (more removed, or less added). `-` when there is no reference or no body. A case passes within 0.2 %.
- t: seconds for the case (base body + fillet).

After the lines: a cluster table (failure mode, count, the first five ids), any problems, and the summary
`N cases, P pass, X expected_fail, W hard, K problems (target X)`. The failure mode comes from one function,
`failure_mode` in the test: known details map to fixed labels (`normal 1`, `foot 1`, `branching selection`,
`concave chain end`, `spring overflow`, `centre did not converge`, `singular centre`, `edge used by N`,
`volume mismatch`); otherwise the first three words of the detail, or `<code> (no detail)`. When adding a new label,
add it there so clusters stay stable. `--json` writes the same per case with `mode`, volumes and validity.

## Adding a case, flipping an expectation

1. Add the case to the spec of `tools/gen_blend_fixtures.mjs` and regenerate `blend_cases.json` (see the header of
   that script for the reference volumes).
2. Add an entry to `blend_expect.json`: `"<id>": {"expect":"expected_fail","observed":"..."}`. To rewrite the whole
   file from what the engine does now, run the table with `BLEND_TABLE_OUT=<file>` and review the diff.
3. When a fix makes a case pass, the table reports `NEWPASS` and fails: set its `expect` to `"pass"`.

## Stepped disc cases

The `stepped-disc` and `stepped-disc-sunk` families (64 cases) are a 70 mm disc, 10 tall, whose top is split by a line through the
middle with one half moved +5 mm (`stepped-disc`) or -5 mm (`stepped-disc-sunk`): every single edge, the rim of each half with the
tangent chain on and off, the wall edges (E2 wall bottom, E5 wall top, E6/E7 the vertical wall|cylinder edges) in pairs and
together, fillet and chamfer at 1 to 4 mm. They are NOT written by `tools/gen_blend_fixtures.mjs`: they come from
`STEP_EMIT=<file> cargo test -p ck-model --test stepped_disc_blend emit_corpus -- --nocapture` (the line `CORPUS_JSON` carries the
document), appended by hand to `blend_cases.json` / `blend_expect.json`. The `reference` volume is the engine's own result at the
time of writing (`"source":"baseline"`), so these cases guard against regressions; they are not OCCT comparisons. Per-case times
(`t=`) are 0.01 to 0.5 s, the baseline for speed work. `tests/stepped_disc_blend.rs` holds the strict checks behind them: the exact
(analytic) volume of the rim blends, the wall-edge blends against an integral, no edge or mesh point outside the cylinder, and
`SWEEP=1` runs every subset of the 8 edges.

The `stepped-disc-top-*` and `stepped-disc-small-top-*` cases (32) are the raised half's top selection: rim arc + wall-top edge + one
vertical wall edge (and the mirror), tangent chain on, 0.5 to 3 mm, on the 70 mm disc and on a 42 mm disc with a 5 mm step. All the
chamfers build. The fillets of those three edges meet at a corner of two planes and a cylinder, where the vertex patch (`corner.rs`,
planar faces only) does not exist: the tools leave the corner open at some sizes and close it by chance at others, so the fillet is
refused with "three rounded edges meet at a corner with a curved face or edge" (no size hint) at 1 and 3 mm on the 70 mm disc, and the
sizes that do build are a mitre, not a rolling-ball patch. One of them (42 mm disc, mirror selection, 1 mm) builds a solid that fails
validation (VERTEX_OFF_CURVE 8e-3 mm) and is recorded as an expected failure. A proper fix is a sphere patch for plane, plane and
cylinder faces: days of kernel work.

Known and not asserted: slivers (edges shorter than 0.05 mm) where a blend ends tangent to a face or where an E5 and an E6/E7
chamfer meet on the round wall; one arc of a seam-split rim chamfered without the chain runs on past the seam (about 7 % too much);
a 0.5 mm fillet of one half of the sunk piece's seam-split rim leaves a pcurve 1e-3 off its surface.

## hard

`"expect":"hard"` with a `"reason"` (and optionally `"alsoFails":["occt","solidcore"]`) marks a case that the
reference kernels we compared also fail. It does NOT mean we will not do it: it means no known engine does it yet,
so it is worked after the cases at least one engine solves, and a valid answer may well exist (a vertex blend that
consumes a short arc, for example). It is judged like `expected_fail` (it must still fail; passing reports
`NEWPASS`, then set it to `"pass"`) but counted on its own and left out of the target count. Use it only with
evidence that other kernels fail too, and say which in `alsoFails`.

## Debug dumps

With `--debug` (`BLEND_DEBUG_DIR`), each selected case is run a second time in its own process with
`CK_BLEND_DEBUG=1`, and its output goes to `<dir>/<id>.log`; earlier `.log` files there are removed first.
`CK_BLEND_DEBUG2` / `CK_BLEND_DEBUG3` set for a local run are passed on. The traces come from
`crates/ck-ops/src/blend/diag.rs` and the blend modules; `blend fail:` lines give the reason a step gave up.
Read the last lines of a log first. Debug runs take about twice as long, so filter to a few cases.

## Wasm

The product runs the kernel as wasm, which can decide a marginal case differently (its libm differs in the last
ulp). `tools/wasm_parity.sh` runs the table through the wasm and lists the cases that differ; see `docs/wasm-parity.md`.
