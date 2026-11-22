# D3: `sketch/solver.rs` and `sketch3/solver.rs`, one core or two?

Status: proposal, nothing implemented. Line counts are today's: 2 191 and 1 549 (3 740 together; the figures in the brief,
2 055 and 1 548, were older).

## What each file is

- `sketch/solver.rs` holds two things in one file. (1) The 2D builder, `build(&Sketch) -> System` (client `$s()`), with the
  per-constraint code, the 2D curve evaluators (`Other`, `Curve2`) and the spline continuity, `onCurve` and `tension`
  constraints, about 1 400 lines. (2) The numeric core: `V`, `Equation`, `System`, `residuals`, `jacobian`, the
  Gauss-Newton `solve` / `solve_oracle` / `solve_from`, `SolveOpts`, `SolveResult`, `analyze`, `status`, `lu_solve`, plus
  the shared kernels `as_1250`, `ls_kernel`, `cross3`, about 800 lines.
- `sketch3/solver.rs` is the 3D builder `build(&Sketch3) -> System` (client `Jo()`), its types (`V3`, `P3`, `Wt`, `Ds`,
  `B3`, `Ru3`, `Yo3`) and the 3D curve helpers. It has no solver of its own: `solve_sketch3` calls
  `sketch::solver::solve_oracle` on the `System` it builds.

So the common core already exists and is already shared. `sketch3` imports `Equation`, `System`, `V`, `SolveOpts`,
`SolveResult`, `as_1250`, `ls_kernel`, `cross3` from the 2D file, and `Bspline`, `gauss_jordan`, `ns3` from `sketch::spline`
and `sketch::smooth`.

## What is duplicated

Measured as distinct source lines (25 or more characters, trimmed) appearing in both files: 61 of about 510 in the 3D file
(12 percent), and most of those are error strings, `Box::new(move |x: &[f64]| {` and loop headers. Real clones:

- `refn` (7 lines), identical.
- `eval_cvspline_derivs` and `eval_cvspline_derivs3` (about 15 lines): the same body over `[f64; 2]` and `[f64; 3]`.
- Parallel structure, not shared text: `Builder`/`B3`, `Ent`/`Ent3`, `Other`/`Ru3`, `Curve2`/`Yo3`, `tangent`/`tangent_specs`,
  `continuity`. Each is a port of a different client function and differs in dimension, in the numeric rounding rules
  (the 3D file notes plain `sqrt(x²+y²+z²)` against the 2D `hypot`, "probe-pinned"), and in the entity kinds (3D has `ref`, arc frames).

## Is a common core worth it

Two different questions.

1. Move the numeric core out of `sketch/solver.rs` into its own module (`sketch/system.rs`: `V`, `Equation`, `System`,
   `solve*`, `analyze`, `status`, `lu_solve`, kernels). Worth it, cheap and safe: pure move, no behaviour change, the file
   with the 2D builder drops to about 1 400 lines, and `sketch3` imports from a name that says what it is.
   Gate: the golden tests (`tests/sketch_golden.rs`, `sketch_solver_regress.rs` and the in-file 3D tests) must stay bit-identical. Re-export from `sketch::solver` so no other import changes.
2. Unify the two builders over a dimension-generic point type (`Pt<const D: usize>`). Not worth it. Only 12 percent of the
   text coincides, and the code is pinned bit for bit to two different client routines (last-bit oracle values in the
   tests). Generic arithmetic would change operation order or rounding in places we cannot see in review, for a saving of
   perhaps 150 lines. The two `eval_cvspline_derivs*` and `refn` can be unified with a small trait or by moving `refn` to a
   shared helper module in the same step as item 1.

## Recommendation

Do item 1 together with the `refn` move (one commit, golden tests unchanged, quick check unchanged). Leave the builders
separate. Re-open only if a third dimension-specific builder appears or if the oracle pinning is dropped.
