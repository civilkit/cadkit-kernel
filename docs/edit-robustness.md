# Edit robustness

A small parameter edit must not turn a model the original kernel still builds into a broken one
(this matters most for an LLM agent or a user nudging one number).

`tools/edit_robustness_check.mjs` applies single-parameter edits and compares both kernels:

- parts: `tools/cases/*.ckpart` and the 50 BenchCAD parts (`--hair` adds the 44-feature large sample,
  features 18 and later sampled, minutes);
- edits: for a spread sample of features per part (default 5), the first numeric leaf (preferring
  depth/offset/angle/r/radius/distance/count/height/width/thickness/x/y) times 0.8, 1.25 and 1.5;
- edits that make the INPUT nonsense are skipped: an arc whose endpoints stop being equidistant from
  its centre, crossing line segments in a sketch;
- a case counts when the ORIGINAL regens with no feature error. It FAILS when ours then has a feature
  error, a trap, a hang, or a volume more than `--vol-tol` (1e-3) from the original's. A part whose
  unedited volume already differs from the original's is judged on errors only (baseline gap, e.g.
  the eyebolt);
- each part runs in a worker process (`--jobs`, `--timeout`); both kernels reuse the regen cache
  across the edits of one part.

`tools/edit_robustness_known.json` lists failures that are understood and NOT regressions of ours
(the original is wrong, or the case is open); each entry carries its evidence. Anything else fails
the run (exit 1). `--quick` (12 small parts, 2 features, factor 1.25, about 7 s) is the gate metric
`edit.robust` (HARD).

## Results (2026-10-14)

| run | cases | original errors (skipped) | comparable | ours fail |
|---|---|---|---|---|
| before the fixes | 768 | 132 | 636 | 39 |
| after (full, 54 parts) | 741 | 116 | 625 | 5, all in the known list |

Fixed causes, each with a test in `crates/ck-model/tests/` (`shell_thickness_edit.rs`,
`edit_regressions.rs`):

1. shell: the cavity overshoots an opened face by a quarter of the wall, not the whole wall.
   Free-form walls and fillets continue past their domain with a second-order polynomial, so two
   neighbours drift apart with the square of the overshoot; at 2.5 mm the large sample corner solve
   jumped and the edge fits failed (7 downstream errors F28, F30, F35, F38, F46, F48, F50 all came
   from the F28 shell).
2. boolean: a revolved sphere's seam-only loop is the whole parameter domain; sphere cut by a circle
   now splits into cap and rest (ball knob).
3. fillet/chamfer picks: when `~k` numbers of a shell's inner/outer faces change after an edit, fall
   back to the same face families and keep all picks on one connected rim (enclosure, rect frame,
   pipe flange).
4. blend: zero-length closed edge remnants (conic cut at its seam) are dropped (cable panel).

## Left

- `propeller_000192` cpattern angle edits: the union of the lofted blade copies used to fail
  (`booleanFailed`); fixed 2026-10-15 by the boolean robust mode (crossing cut curves, sliver edge pieces,
  sliver classification; docs/spec-agent-interface.md "Boolean robustness"). Re-run the edit sweep to refresh
  the numbers; angles that bring two blade tips together within ~1e-3 are the ones the robust mode exists for.
- `twisted_drill_000546` partial revolve cut: ours vs original differ by 0.23 %, no ground truth.
- `enclosure` depth edits: the original fillets an inner edge; ours is the consistent choice.
