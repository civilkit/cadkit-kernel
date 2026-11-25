# Sketch dimensions: arc length, driven dimensions, model edges

## Arc length

`{"type":"arclength","id":"k1","refs":["<arc id>"],"value":25}`. Residual: `|p1 - c| * sweep - value`, sweep the counter-clockwise
angle from the start `p1` to the end `p2` about the centre `c`, in `(0, 2 pi]` (`sketch::measure::arc_sweep`). An arc whose
ends coincide is the full circle (`2 pi r`, never 0); a circle ref counts as its circumference `2 pi r`; an ellipse or line is
refused (`arclength needs an arc or a circle`). One equation, one DOF. The jump at sweep 0 / 2 pi is not smooth, so a
dimension that must cross it does not; keep the end on the intended side.

## Driven (measured) dimensions

Any dimension with `"driven": true` is ignored by the solver, status, analysis and guard, and the regen echo (`Sketch::to_json`)
writes `Sketch::measure` of the solved sketch into its `value` (mm; `angle` in degrees, 0 to 360, from line 1 to line 2 with
`s1`/`s2`). `sketchAnalyze` and `sketchFreedoms` also return `"driven":[{"id":"k1","value":31.41}]` (only when there are driven dimensions), measured on the sketch as drawn, so the editor needs no regen round trip. A driven dimension never conflicts and never removes a DOF.

## Dimension to a model edge

Existing: the `convert` request (`ck_ops::convert_proj`) projects model edges or faces into the sketch frame as fixed
points and entities (`from:{kind:"edge",faces,at,shape}`): straight edges become lines, circles parallel to the plane
circles or arcs, edge-on circles lines, oblique full circles ellipses, the rest splines. Those entities are ordinary
operands, so a `driven` `distance`/`pldist`/`radius`/`arclength`/`angle` to them already measures the model edge. Their
points are `fixed`; they are NOT refreshed on regen (a snapshot of the body at conversion time).

Not built: a regen-time `projected` entity `{"type":"projected","id":"e9","edge":{"faces":["F2.S.e1","F2.S.e2"],"at":[x,y,z]}}`
that re-projects on every regen. Design: in `regen_sketch`, after the plane frame resolves and before the solve, resolve each
`projected` entity against the body of the features before this sketch (it depends on them, like a face plane), project with
`convert_proj::project_curve` into the frame, and write fixed points plus a construction line/arc/circle under the entity's id.
Edge lookup by `faces` (sorted) and nearest edge to `at`; no match gives error `projectedEdgeLost` (entity id in params), a
degenerate projection (edge-on to a point) `projectedEdgeDegenerate`. Names are stable as long as the two faces keep their ids;
`at` is refreshed to the edge midpoint after each successful match. Blockers: `regen_sketch` has no body access (the body is
JSON in `RegenState`; the exec trait owns the B-rep), so it needs a new `FeatureExec` hook.
