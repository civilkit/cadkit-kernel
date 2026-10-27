# Tolerances

This chapter explains which tolerances the kernel uses, what each one means, which operations use which default, how a tolerance gets from a request to the code, and what the tolerance fields on edges and vertices do. All lengths are in millimetres.

## Two kinds of tolerance

- **Absolute tolerances** are model distances. Two points closer than `1e-6` mm are the same point; an edge may lie `1e-6 * (1 + |p|) * 10` from its face. They apply to geometry, whatever its size.
- **Bounding-box-relative tolerances** scale with the size of the body. They belong to display and verification, not to geometry: the tessellation chord height, the thresholds of the mesh check, and the sag of a section.

Mixing the two is deliberate. Modelling must not depend on how big a part is, so its tolerances are absolute. A picture of a 1 m part and a 10 mm part should look alike, so tessellation is relative.

## Where each tolerance comes from

| quantity | value | where it is used |
|---|---|---|
| Model tolerance `TOL` | `1e-6` | Passed by the feature layer to every operation in `ck-ops`: the welding distance of the boolean, the tolerance of the intersections, knit, thicken, trim, extend, fill, pipe cut, and the tolerance of the validator in a few self-checks. |
| Shared constants in `ck-core` | `EPS_LINEAR` `1e-6`, `EPS_POINT` `1e-9`, `EPS_PARAM` `1e-12`, `EPS_MACHINE` `1e-15`, `EPS_COARSE` `1e-3` | Zero tests and coincidence tests inside geometry code. |
| Boolean welding retries | `10 x` and `100 x TOL` | When a boolean does not close. Free-form operands start at `10 x`. Robust mode repeats `10 x`, `1 x`, `100 x`. |
| Fuzzy value | feature field `tolerance`, mm; `autoFuzzy` tries `1e-7`, `1e-5`, `1e-3` | Snaps nearly coincident planes, cylinders and spheres of a tool onto the host before a boolean. |
| Intersection | `TOL` `1e-6`; marched curves fitted to `tol / 100` | `ck-ops::intersect`. |
| Extrude depth floor | `1e-6` | A depth of zero or less is raised to it. |
| Sweep profile-normal check | `5e-5` on `abs(t . n) - 1` | `sweepProfileNotNormal`. |
| Loft | validator at `1e-6` | Whether the default matching of stations gives a valid body; if not, other matchings are tried. |
| Free-form offset fit | `1e-5` surfaces, `3e-3` edge curves | Shell and offset of free-form faces. |
| Free-form blend fit | about `1.2e-5` | Rolling-ball fillets of free-form edges. |
| `projectCurve` | field `tolerance`, default `0.001`; chain join `1e-3` | The fit of the projected curve, and the distance within which runs are joined. |
| `editCurve` | field `tolerance`, default `0.01` | Fit of an edited curve. |
| Point classification | within 1 percent of the box diagonal of the boundary: nearest-feature test; farther: winding number | `ck_topo::region::classify`. |
| Operands far from the origin | centre farther than `1e4` diagonals | Boolean works in local coordinates. |
| Tessellation | chord `max(1e-3, 4e-4 * diagonal)`, angle `0.3` rad | See [tessellation](../kernel/tessellation.md). |
| Section | sag `2e-5 * diagonal` | `section`: curved segments are bisected until the chord sag is below it. |
| `check` expectations | relative `1e-4` for volume and area; `1e-4` of the diagonal for boxes | `check`, unless `tol`, `relTol` is given. |
| Validator `tol` | `1e-6` (request field `tol`) | `validate`; see below. |
| Heal `tol` | `1e-4` on import, `1e-5` for the `heal` request | Repair distance; detection runs at `min(tol, 1e-6)`. |
| Tolerance cap | `0.05` mm (request `healTolerance`) | Largest tolerance that may sit on an edge or vertex. |

## How a tolerance is chosen and passed

1. **Modelling.** `ck_model::features::TOL` is the single value handed to the operations. A feature does not choose its own, with these exceptions: the fuzzy `tolerance` field of any feature that runs a boolean, the `tolerance` fields of `projectCurve` and `editCurve` (fit tolerances), and operations that need a looser internal value (offset fits, blend fits). A fuzzy `tolerance` must be positive and finite, otherwise it is treated as 0 (exact).
2. **Booleans.** Called with `TOL`, then retried with the ladder in [operations](../kernel/operations.md). The tolerance that finally decided a boolean is reported as the warning `booleanFuzzy`.
3. **Validation.** `validate` takes `tol` from the request, default `1e-6`, and passes it into every test of the validator. The same number is also the one used by the repair pass to detect what to repair.
4. **Tessellation.** Computed from the body by `body_tol`; not set by the request.
5. **Cache.** Results do not depend on a tolerance that is not in the document, so none is part of the regeneration cache key except `autoHeal` and `healTol`.

## The validator's formulas

With `scale = 1 + |p|`, where `p` is the start of the edge's curve:

| test | passes when |
|---|---|
| vertex on its edge's curve end | `distance <= max(10 * tol * scale, edge.tol, vertex.tol)` |
| edge on its face (9 samples per coedge) | `deviation <= max(10 * tol * scale, 1e-6 * scale, edge.tol)` |
| loop closure (end of a coedge to start of the next) | same vertex, or distance `<= max(tol, both vertex tolerances)` |
| edge not degenerate | length `>= tol` |
| edge or vertex tolerance | `<= cap` |

The factor 10 and the scale make the test insensitive to rounding in coordinates far from the origin. The test is therefore looser than the working tolerance of the operations by at least a factor of 10, and more for parts far from the origin, which is why a freshly built body passes with room to spare.

## Tolerance fields on edges and vertices

Every `Edge` and every `Vertex` has a `tol` (default 0, meaning exact).

- **Edge `tol`** is the largest distance the edge's curve may lie from the faces that use it. Imported B-spline faces often sit a few `1e-3` mm away from the analytic faces they meet, and no single curve lies on both.
- **Vertex `tol`** is the largest distance the vertex may lie from the end of each incident edge curve.
- **Only repair sets them.** Modelling operations create exact entities. `split_edge` copies the tolerance to both halves and `Body::append` clones it.
- **The validator honours them:** a deviation up to `max(model tolerance, the entity's own tolerance)` passes; a tolerance above the cap is an error.
- **Results.** The `validate` reply has `toleranceUsed` (the largest value in the body) and, when it is not 0, a `notes` sentence.
- **Not propagated.** Booleans and blends build new, exact edges; the result is exact where it is new. `fillet` and `chamfer` refuse an edge that carries a tolerance with `blendTolerantEdge`. Mass properties, tessellation (edges are shared polylines, so there are no cracks) and STEP export ignore the tolerance.

The `heal` request sets edge and vertex tolerances when asked (`tolerant: true`): per edge, the largest distance of its curve to its faces (the validator's 9 sample points, 8 intervals, then 33 where those fail), multiplied by 1.05, when it is above the strict tolerance and within the cap. A deviation above the cap is reported, not repaired. See [healing](healing.md).

## Choosing a tolerance in practice

- Leave `tol` at the default for bodies the kernel built. A finding at `1e-6` on such a body is a defect, not a rounding problem.
- For imported bodies, validate at the default first, then decide: heal with `tolerant: true` (the tolerance is recorded and stays on the body), or validate with a larger `tol` for a one-off judgement. The first is preferable because later operations see the tolerance.
- For a boolean that fails on nearly coincident faces, use the feature's fuzzy `tolerance` (between `1e-5` and `1e-3`) or `autoFuzzy`, not a larger welding tolerance; raising the welding tolerance for the whole arrangement makes every decision in it fuzzy.
- Do not try to control the display mesh through tolerances; see [tessellation](../kernel/tessellation.md).

## See also

- [The validate request](validate.md)
- [Healing](healing.md)
- [Operations](../kernel/operations.md)
- [Tessellation (kernel)](../kernel/tessellation.md)
