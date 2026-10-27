# Geometry

`ck-geom` holds the curves and surfaces that bodies are made of. It is representation only: it evaluates, differentiates, converts and fits geometry, and knows nothing about topology. This chapter lists the types, their parameterisations, the NURBS machinery, fitting, projection, and the intersection routines in `ck-ops` that the booleans rely on.

## Curves

`Curve` (3D) has six variants:

| variant | definition | parameter |
|---|---|---|
| `Line` | `o + t d`, `d` a unit vector | arc length |
| `Circle` | `c + r (cos t x + sin t y)`, normal = `frame.z` | angle, period 2 pi |
| `Ellipse` | `c + a cos t x + b sin t y` | angle, period 2 pi |
| `Conic` | focus form `p / (1 + e cos t)` about `frame.o`: circle, parabola, hyperbola branch | polar angle |
| `Sinusoid` | `o + t x + a sin(w t + phase) y` in a frame plane | `t` along `x` |
| `BSpline` | B-spline or NURBS curve | knot parameter |

`Curve2` is the 2D counterpart used for sketches and for pcurves: `Line`, `Circle`, `Ellipse` and `BSpline`.

Every curve offers `eval`, derivatives up to second order, `tangent`, `closest_param` and `closest_point`, `length_between`, `segment` (a trimmed copy), `reversed`, `transform`, `sample` and `kind`. `kind()` returns the string used on the wire: `line`, `circle`, `ellipse`, `conic`, `sinusoid` or `spline`. A circle reports `circle` and its centre, normal and radius in the edge record of a reply.

## Surfaces

`Surface` has ten variants. The analytic parameterisations follow ISO 10303-42, so STEP export can write them one to one.

| variant | `eval(u, v)` | domain | periods |
|---|---|---|---|
| `Plane` | `o + u x + v y` | unbounded | none |
| `Cylinder { r }` | `o + r (cos u x + sin u y) + v z` | `u` in [0, 2 pi), `v` unbounded | `u` |
| `Cone { r, semi_angle }` | `o + (r + v tan a)(cos u x + sin u y) + v z`, `r` = radius at `v = 0` | as the cylinder | `u` |
| `Sphere { r }` | `o + r cos v (cos u x + sin u y) + r sin v z` | `v` in [-pi/2, pi/2] | `u` |
| `Torus { major, minor }` | `o + (R + r cos v)(cos u x + sin u y) + r sin v z` | both [0, 2 pi) | `u` and `v` |
| `Revolution { profile, axis_o, axis }` | the profile curve rotated about an axis | `u` angle, `v` along the profile | `u` |
| `Extrusion { profile, dir }` | the profile curve translated along `dir` (its length is kept) | `u` along the profile, `v` unbounded | the profile's |
| `BSpline` | tensor-product B-spline or NURBS | the knot ranges | none |
| `Screw` | a profile carried by a rotation plus an axial advance (helical sweep) | bounded in both | none |
| `Offset { base, dist }` | `base + dist N`, the base surface moved along its normal | the base's | the base's |

`kind()` returns `plane`, `cylinder`, `cone`, `sphere`, `torus`, `revolution`, `extrusion`, `spline` or `offset`. A `Screw` patch reports `spline`. These are the values of `faces[].kind` in a regeneration reply.

Offset surfaces are exact, not fitted: the second derivatives come from differentiating the unit normal of the base, using the base's third partial derivatives (exact for analytic surfaces and for B-splines). An offset cylinder of radius 10 moved by 2 has principal curvatures 1/12 and 0, as it should.

Every surface offers `eval`, first and second derivatives (`derivs1`, `derivs2`), `normal`, `closest_point` and `closest_param`, `distance`, `domain`, `periods`, `is_pole` and `is_planar`. Variants of the closest point exist for a warm start from a hint (`closest_point_near`) and for a bounded search (`closest_point_within`), which is how a marching intersection and a loop of projections stay cheap.

## B-splines and NURBS

`BSplineCurve<P>` (`P` is `Vec2` or `Vec3`) and `BSplineSurface` store the degree, the knot vector, the control points and optional weights (`None` means non-rational). Knot vectors are full: a clamped curve has `ctrl.len() + degree + 1` knots with end multiplicity `degree + 1`. A surface keeps its control net as `ctrl[i * nv + j]` with `u` as the slow index.

The routines follow *The NURBS Book* (Piegl and Tiller): span search, basis functions and their derivatives, knot insertion, refinement, splitting at a parameter, degree elevation, knot removal within a tolerance, Bezier decomposition, iso-curves of a surface and surface transposition. Rational evaluation works in homogeneous coordinates, including the derivatives. At a knot value the span search walks back over repeated end knots, so clamped ends evaluate to the end point.

Closest-point queries on a curve seed from samples and polish with Newton steps, skipping spans whose speed bound proves they cannot contain a nearer point. Surfaces seed from a parameter grid and polish with a two-variable Newton iteration; a B-spline surface keeps a cached preparation per surface (`closest_prep`) for loops of queries.

### Conversions

`convert` turns analytic geometry into NURBS exactly. A circle or ellipse arc becomes a rational quadratic with quarter-turn spans and weights `1` and `sqrt(2)/2`; lines become degree 1; analytic surfaces become rational patches (`surface_to_nurbs`). A revolution or extrusion of a spline profile becomes a surface by `convert::revolve` and `convert::extrude`. The `refCurve` request returns this form for an edge. For the hole edge of a plate it is a degree 2 curve with nine control points and weights `[1, 0.7071, 1, 0.7071, 1, 0.7071, 1, 0.7071, 1]`, closed.

## Fitting

`fit` builds B-splines through data, following the same book:

| function | what it does |
|---|---|
| `interpolate(points, degree, param)` | Global interpolation with averaged knots. `Param` is `Uniform`, `ChordLength` or `Centripetal`. |
| `interpolate_cubic_tangents` | Cubic interpolation with prescribed end tangents. |
| `approximate(points, degree, n_ctrl, param)` | Least-squares fit with a fixed control point count and fixed end points. |
| `approximate_tol(points, degree, tol, param)` | Raises the control point count until the maximum deviation is within the tolerance. |
| `max_deviation` | Distance of the data to the curve. |
| `interpolate_surface`, `grid_surface` | Tensor-product interpolation through a grid of points. |

The interpolation solves a banded system row by row, so thousands of samples cost memory proportional to the point count, not its square. Two fits (`interpolate_cubic_tangents` and `approximate_with`) still build dense matrices and are not used on long inputs.

Fitting is where tolerances enter geometry: intersection curves, pcurves of non-analytic pairs, offsets of free-form faces and lofts through sections are all fitted splines, with deviations of the order `1e-5` (see [tolerances](../validation/tolerances.md)).

## Curves on surfaces

A coedge's pcurve (see [the B-rep model](brep.md)) is a `Curve2` in the face's uv space. `pcurve::exact_pcurve` gives an exact one, parameterised like the 3D curve, for the analytic pairs:

- a line or a circle or ellipse on a plane;
- an axial line on a cylinder or cone (constant `u`);
- a circle about the axis on a cylinder, cone, sphere or torus (constant `v`).

For every other pair, `pcurve_of` projects samples of the 3D curve onto the surface and fits a cubic through the uv points; `pcurve_of_dense` doubles the sample count (up to 1024) until the pcurve reproduces the 3D curve to a requested relative deviation. `project_points` unwraps periodic parameters so the uv sequence is continuous across a seam, including for B-spline surfaces that close on themselves (`closed_periods`).

`CurveOnSurface` composes a pcurve and a surface into a 3D curve `S(uv(t))` with derivatives, which the validator uses to measure how far an edge lies from its faces.

## Differential analysis

`analysis` provides curvature and torsion of space curves from their derivatives, the Frenet frame, a rotation-minimising frame along a polyline, and the fundamental forms, normal curvature and principal curvatures of a surface. `curvatureAt` returns them for a point of a face. On the fillet of a 2 mm round the principal curvatures are `k1 = 0.5` and `k2 = 0`, the Gaussian curvature 0 and the mean curvature 0.25. The sign convention is positive where the surface bends away from the outward normal.

## Intersection support

Booleans, blends and sections need intersections. They live in `ck-ops::intersect` and share one convention: a curve is given with a parameter range, a periodic curve may use a range outside [0, 2 pi) (an edge range such as [4, 4 + 2 pi]) and results are mapped into that range, and `tol` is a model distance. The default is `1e-6`. Results are sorted by the parameter on the first curve.

| pair | method |
|---|---|
| curve and curve (`cc`) | Closed forms for line and line, line and circle, circle and circle (coplanar or skew). Otherwise polyline seeding followed by a two-variable Newton iteration. Coincident stretches are reported as overlaps (parameter intervals on both curves), not as point lists. |
| curve and surface (`cs`) | Closed forms for a line against a plane, cylinder, cone, sphere (quadratic) and torus (quartic); a circle against a plane. Otherwise seeds from the distance function and a three-variable Newton iteration. A curve that lies in the surface is reported as an interval. |
| surface and surface (`ss`) | Analytic carriers where they exist (below), otherwise marching. |

For two surfaces the analytic cases are: plane and plane (line, or coincident); plane and cylinder (zero to two axial lines, a circle, an ellipse); plane and sphere (circle); plane and cone (circle, ellipse, two generators through the apex); plane and torus (latitude circles, or the two meridian circles); two cylinders with parallel axes (lines) or the same axis; coaxial cylinder, cone and sphere pairs and two spheres (circles, by intersecting the meridians). Parabolas and hyperbolas from a plane and a cone fall through to marching.

Marching seeds the intersection from a grid of patches. Each surface is cut into 10 by 10 parameter cells with padded boxes, only overlapping cell pairs are searched, and a Newton corrector refines the seeds. A predictor-corrector march with adaptive steps then follows each curve, and a cubic B-spline is fitted through the traced points, refined until the deviation is within `tol / 100`. Marched curves meet their neighbours only to about `1e-6` to `1e-5`, which is why the boolean retries at looser welding tolerances; see [operations](operations.md).

Real roots of low-degree polynomials come from `ck_core::quad`, which uses a cancellation-free quadratic formula, trigonometric and Cardano solutions for cubics, Ferrari's method for quartics, and Newton polishing against the given coefficients.

## Projection of curves onto faces

The `projectCurve` feature projects a sketch or a planar curve orthogonally onto faces and returns one B-spline that lies on them. It is used for groove paths and guide curves. It is built from these steps:

1. Every sample of the source (800 to 1040 per entity) is cast along the projection direction against all selected faces, and every crossing inside a trimmed face is kept, so the near and far sheets of a cylinder or sphere are both hit.
2. Consecutive hits on one face form a run. The ends of a run are bisected to the exact face boundary or silhouette, and runs are resampled evenly.
3. Runs whose ends are closer than `1e-3` (a fixed distance, independent of the requested tolerance) join into one chain. If the hits do not form one connected chain, the feature fails with `projectCurveSeveral` and lists the open ends.
4. The chain is fitted as one B-spline, degree 3 by default (2 to 9 allowed), with a knot of multiplicity `degree` at every corner where the tangent turns by more than 1 degree. Control points are added, at most 400, until the deviation is within `tolerance` (default `0.001` mm). The fit is rechecked densely and tightened up to three times. The marker returned with the curve states the achieved `deviation`.

The check order and all error codes are in the feature's reference (`projectCurveNoInput`, `projectCurveNoFaces`, `projectCurveMisses`, `projectCurveSeveral`, `projectCurvePlane`, `projectCurveFailed`; see [surfaces and curves](../format/features-surfaces.md)). A ray that only grazes a surface is not a hit, and tolerances below the sampling resolution of the data (about `3e-5` for a long spline) are reported with the deviation actually reached.

## See also

- [The B-rep model](brep.md)
- [Operations](operations.md)
- [Tolerances](../validation/tolerances.md)
- [Surfaces and curves](../format/features-surfaces.md)
