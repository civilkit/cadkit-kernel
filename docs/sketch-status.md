# Sketch solver status and live solve (`opts.sketchStatus`, `opts.solveSketches`)

The sketch editor needs "fully defined / under-defined / over-constrained" per sketch. The original kernel
reports none of it (probed: `regen` entries are `{frame, profiles, sketch}` only; conflicting or redundant
constraints raise **no** feature error, the sketch just stays as sent; the only `drag` request is a face drag,
and `solveSketch`/`sketchStatus`/`solve` are unknown requests). In the original the *client* computes the figure
(`si()` / `ti()` in the app); `crates/ck-model/src/sketch/solver.rs` already ports that (`solver::status`).
Our status keeps its semantics (`state` over/full/under, `dof = unknowns - rank`) and adds the constraint ids.

## Request

`{"type":"regen","part":...,"opts":{"sketchStatus":true}}`. Off by default, so replies (and the gate's byte
comparison of whole replies) are unchanged. Only a literal `true` enables it.

## Reply

Each `sketches` entry `[featureId, {frame, profiles, sketch, status}]` gains:

```json
"status": {"dof": 0, "fullyDefined": true, "conflicts": [], "redundant": [],
           "solved": true, "state": "full", "residual": 0}
```

| field | meaning |
|---|---|
| `dof` | x,y of every non-fixed point + r of circles/ellipses, minus Jacobian rank (a free point is 2) |
| `fullyDefined` | `state == "full"` and `solved` |
| `conflicts` | constraint ids that cannot hold together with the constraints before them (document order) |
| `redundant` | constraint ids implied by the ones before them |
| `solved` | coordinates satisfy every constraint (max abs residual <= 1e-7), or a solve from them converges |
| `state` | `"over"` if any constraint is dependent, else `"full"` at dof 0, else `"under"` (the original client's `si().status`) |
| `residual` | max abs residual at the coordinates in the reply (-1 if not finite or the system cannot be built) |
| `problems` | coded problems `{code, message, constraint?, entity?}` (see "Problems" below); omitted when empty, before `suppressed` |
| `suppressed` | constraint ids flagged `suppressed:true`; document order, omitted entirely when empty (rides along as the last key) |

Notes:

* Regen only re-solves (and writes back coordinates) when a constraint `expr` changes a value; that is
  unchanged. A draft whose coordinates do not yet satisfy the constraints therefore reports
  `solved: true` with `residual > 0` if the solver converges from them, and `solved: false` when it does not
  (over-constrained or contradictory).
* Rank test: equations in solver order are orthogonalised incrementally (Gram-Schmidt, two passes) against
  the accepted rows of the central-difference Jacobian at the solution; a row is independent if the part
  outside the span exceeds `1e-6` of its norm (norm < `1e-9`: dependent). Blame is deterministic: the later
  constraint in document order is reported. A dependent row with residual > `1e-6` is a conflict, otherwise
  redundant; a constraint listed as conflict is not repeated under redundant.
* If the equation system cannot be built (e.g. a constraint names a missing point) the kernel leaves the
  sketch unsolved without an error; status then has `solved: false`, `dof` = free coordinates.
* A constraint object may carry `"suppressed": true` (agent input, additive): the solver ignores it
  everywhere — build, solve, rank, `dof`, `conflicts`/`redundant`, `sketchFreedoms`, and the coincident
  pairs `sketchAnalyze` uses to silence `nearCoincident`. The document roundtrips the flag verbatim with
  the constraint, and `status.suppressed` lists the flagged ids. Suppressing the constraint a `conflicts`
  entry blames clears the conflict (the remaining system is solved and `full`).
* Code: `crates/ck-model/src/sketch/status.rs`. Types: `SketchSolverStatus` in `packages/part-model`.
  Checks: `cargo test -p ck-model --lib status`, `node tools/sketch_status_check.mjs` (through the wasm),
  `node tools/probe_sketch_status.mjs --orig | --status` (probe the oracle / ours),
  `node tools/bench/sketch_status_timing.mjs` (cost on the BenchCAD parts).

## Problems (solve guard)

`crates/ck-model/src/sketch/guard.rs`. Additive (the oracle has no such check); cases from the solidcraft sketch
fuzzer (PRs #8988, #9049, #8562, #8611, #8577, #8600), pinned in `crates/ck-model/tests/sketch_guard.rs`.

| code | when | effect |
|---|---|---|
| `sketchCurveShrunk` | a converged solve takes a line from >= 1e-3 to < 1e-5 long, or a circle / arc / ellipse radius the same way (`entity` names the first one) | the solve is not taken: the coordinates stay as sent, `solved: false` |
| `sketchSameCurveTwice` | parallel, perpendicular, collinear, equal, tangent or concentric with one curve as both refs | named only |
| `sketchMidpointOwnEnd` | a midpoint whose point is an end of its own line | named only (its solve then shrinks the line) |
| `sketchLineEndsCoincident` | a coincident on the two ends of one line | named only (its solve then shrinks the line) |

A zero-length line still "meets" its parallel, angle or tangent (the residual divides by its length), so such a
solve converges and used to be reported solved while the sketch had collapsed (a rectangle corner made the midpoint
of the opposite side, two opposite corners made coincident, a dragged end put on its line's own midpoint). The
guard runs wherever regen solves (an `expr` change, `solveSketches`) and in the status' own solve. Curves already
that short before the solve are left alone. Picks are listed in constraint order, then the shrunk curve.

## Live solve (`opts.solveSketches`)

`{"type":"regen","part":...,"opts":{"solveSketches":true}}` (literal `true` only). Off by default, so every
existing reply stays byte-identical (checked against a pre-option build by `tools/sketch_solve_check.mjs`).
It is for the GUI (drag / edit), not for agents; the agent guide does not mention it.

For every `sketch` feature regen runs the existing solver (`solver::solve_sketch`, the client `Qe()`/`Bs()`
port) from the supplied coordinates with ALL constraints, including a `fix` constraint with `value: [x, y]`
(the GUI's temporary drag pin; the solver treats it as a hard equation on that point). The SOLVED
coordinates are used both in `sketches[i].sketch.points` and for the profiles that downstream features
(extrude, ...) consume, so a preview body matches the sketch. Circle/ellipse radii are written back too.
Every constraint except a `suppressed` one takes part.

* Sketches whose coordinates already satisfy every constraint (max abs residual <= 1e-7) are not solved
  again (no extra solve); a sketch already re-solved because an `expr` changed a value is not solved twice.
* If the solve does not converge or the constraints conflict, the supplied coordinates are kept, the
  reply carries `status.solved: false` with the `conflicts` / `redundant` lists, and NO feature error is
  raised (the original raises none either). A conflicting pin on a fully constrained sketch is named in
  `conflicts` (the later constraint in document order is blamed).
* `solveSketches` implies `sketchStatus`: every `sketches` entry carries `status`, computed on the
  coordinates in the reply. `sketchStatus: true` alongside gives an identical reply.
* Cost: 50 BenchCAD parts, interleaved median of 7 runs, +0.5% total
  (`node tools/bench/sketch_status_timing.mjs --option solveSketches`).
* Checks: `cargo test -p ck-model --lib solve_sketches`, `node tools/sketch_solve_check.mjs`
  (fixture `tools/fixtures/sketch_solve_off.json` = sha256 of option-off replies from a build without the option).
