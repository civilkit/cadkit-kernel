# projectCurve

Orthogonal projection of a sketch (or a planar curve feature) onto faces: one B-spline lying on the surface
(groove path, guide curve). Code: `crates/ck-model/src/features/project_curve.rs`; tests:
`crates/ck-model/tests/project_curve.rs`; corpus against the oracle kernel: `tools/project_curve_corpus.mjs` with the cases
in `tools/cases/project_curve_cases.mjs` and `project_curve_cases2.mjs` (~230 cases), run by
`tools/remote_build.sh --checks projcurve`.

## Payload

`sketch` | `curve` (a feature id whose result is a curve in `curves[]`: `editCurve`, `projectCurve`; a `curve` wins when both
are given), `faces[]` (face names of the solid or of a surface body; empty or missing = every face of the solid and of all
surface bodies), `degree` (3, rounded, 2..9), `tolerance` (0.001 mm, > 0), `direction` (`both` default, `forward` =
along the sketch plane normal, `reverse` = against it). A numeric field may be a number, `"=expression"` or a numeric
string; null / missing / text that is no number gives the default.

Result: `curves[] = [id, {pts, axis:false, t0:0, t1:1}]` and a marker `{degree, count, deviation, length, pieces,
corners, closed}` (`count` = control points, `length` = length of the projected data, `pieces` = runs, `corners` = joints
where the tangent turns by more than 1 degree, `closed` = the ends meet).

## Check order (all probed on the oracle)

1. no solid and no surface body, or no face at all: `projectCurveNoFaces` (so an empty payload before any body is NoFaces,
   which is what `EMPTY_FEATURE_ERRORS` lists; with a body it is `projectCurveNoInput`)
2. neither `sketch` nor `curve`: `projectCurveNoInput`
3. `curve` (wins): unresolved, or not a curve feature: `missingReference`; fewer than 3 points, collinear or not within 1e-4
   of one plane: `projectCurvePlane`. Else `sketch`: unresolved or not a sketch: `missingSketch`; no projectable entity
   (construction lines, points, zero length, text): `projectCurveNoInput`
4. degree outside 2..9: `editCurveDegree {min:2, max:9}` (also before unknown faces and before the geometry)
5. tolerance <= 0 or an unknown `direction`: `invalidValue`; a failing `=expression`: `badExpression`
6. a face name that is not a face of the solid or a surface: `missingReference`
7. nothing hit: `projectCurveMisses`; the hits do not form ONE connected chain: `projectCurveSeveral {count, points}` where
   `points` lists both ends of every OPEN chain (closed chains contribute none; the order is not fixed)

## What the projection does

* Every sample (800..1040 per entity) is cast along the normal against every picked face, and ALL crossings inside the
  trimmed face are kept: the near and the far sheet of a cylinder, sphere, torus or cone are both hit. A `forward` /
  `reverse` ray starting behind the face (the sketch plane through the body) sees only one side.
* Consecutive hits on one face are a run (a "piece"). The ends of a run are bisected to the exact face boundary or silhouette;
  runs are re-sampled so the spacing is even (the fit needs data along the whole curve).
* Runs whose ends are closer than 1e-3 (a fixed distance, independent of `tolerance`) form a chain: entities joint to joint,
  neighbouring faces at their shared edge, the two sheets of a curved face at a silhouette. Several runs meeting in one point
  (a sketch corner on a silhouette) are paired by the smoothest continuation. A circle clipped by a boundary where the kept
  arc wraps around the circle's start is one run.
* A ray that only grazes the surface (equator of a sphere seen from the pole) is no hit; a ray lying inside a face is skipped.
* Several chains (two sheets that do not meet, two faces that both receive the curve, a hole in the face between two pieces)
  are `projectCurveSeveral`; a part of the curve outside the trimmed face is simply dropped.
* The chain is fitted as one B-spline through the runs in order (ck-ops `curve_edit`, as `editCurve` with `method:
  tolerance, keepCorners`): knot of multiplicity `degree` at every corner, control points raised until the deviation is within
  `tolerance`, at most 400 (a curve that cannot reach it gets 400 control points, no error, like the oracle). Because
  editCurve measures at 201 places only, the curve is re-checked densely and the fit tightened up to three times.

## Known differences to the oracle

* A source point exactly on the plane of a side face (box side seen edge-on): the oracle projects the ray lying in that face as
  a segment down the face; ours skips such a ray (`box-seam-between-faces`).
* A curve produced by `editCurve` is fitted differently there (internal notes (not published)), so a projection of it differs by about 1e-2.
* Tolerances below the sampling resolution of the data (~3e-5 for a long spline at 800..1040 samples; the data chords sit that far off the true curve) are reported honestly: the marker's
  `deviation` is what the fit achieves against our dense data; the oracle reports its own minimum (1e-6).
* The fit itself (control point counts, exact knots) is ours: counts differ from the oracle for the same tolerance, the curves
  agree within the tolerance (the corpus compares curves by Hausdorff distance, total length, ends, `pieces`, `corners`, `closed`).
* Hole and boundary ends are as exact as the face classifier (the uv polygon of a hole is a few 1e-5 off).

## Consumers

The curve is in `curves[]` like an `editCurve` or `helix` result and is taken as `{kind:"curve", id}` by:

* `editCurve` (`curves[]`) and `projectCurve` itself (`curve`, planar curves only);
* `sweep`: as `guides[]` (via `ref_samples`) and as `path` (a spline through at most 120 of its display points);
* `plane` with `mode: "curve"` (`curves[]`; a planar projected curve gives its plane).

Not consumers (they take sketches, edges or faces only): `loft` guides / centre line, `splitLine`, `trimSurface`,
`cutWithSurface`, `curvepattern`, `fillpattern`. The corpus pins editCurve, the sweep path, the sweep guide and the plane
against the oracle (`consumer-*` cases).
