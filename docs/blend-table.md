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
