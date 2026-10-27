# Operations

`ck-ops` is the feature engine: booleans, blends, loft, sweep, offset, shell, draft, helix, pipe cut and the surfacing operations. This chapter describes how each works, what tolerance it uses and how it fails. The fields of the corresponding features are in the [format part](../format/features.md); here the focus is the geometry.

## Common behaviour

- **Inputs and outputs.** An operation takes resolved inputs (bodies, face and edge handles, numbers) and returns a new `Body` or an error with a `code` and an optional `detail`. The feature layer in `ck-model` resolves references, evaluates expressions, calls the operation and turns an error into the entry `["F3", {"code": ..., "params": {...}}]` of the reply's `errors`.
- **A failed feature changes nothing.** The body is left as it was before the feature, and the following features still run on it. A later feature that depended on the failed one reports its own error, for example `missingEdge`. See [feature errors](../validation/diagnostics.md).
- **Working tolerance.** The feature layer passes `TOL = 1e-6` mm to operations (`ck_model::features::TOL`). It is the welding distance of the boolean, the tolerance of intersections, and the model tolerance handed to the validator. Operations that fit splines use their own looser internal tolerances (see below).
- **Exact first.** Whenever the geometry allows, results are built from exact analytic surfaces: a fillet along a line between two planes is a cylinder, a chamfer is a plane, a cut through a block with a cylinder gives an exact cylinder face. Free-form results (lofts, sweeps along splines, offsets of free-form faces) are NURBS.
- **Self-check.** An operation does not report success on a body it can see is broken. A boolean checks the topology of its result (open edges, non-manifold edges, broken loops) and its volume against the operands; a loft, a shell and a move-face validate their result with the full validator.
- **Names.** Every operation names the faces it creates; see [persistent names](naming.md).

## Booleans

`ck_ops::boolean::boolean(a, b, op, tol)` with `op` one of `Union`, `Cut` (`a - b`) and `Intersect`. The pipeline has five stages:

1. **Imprint.** Every face of `a` is intersected with every face of `b` (candidate pairs come from a bounding-volume hierarchy). The intersection curves are trimmed to both faces. A welded vertex pool (tolerance `tol`) is shared by both bodies, the existing edges are split at every pool point on them, and coincident pieces are merged into one edge. Coincident face pairs imprint each other's boundaries instead of an intersection curve.
2. **Fragments.** In each imprinted face a loop walk finds the cells: at each vertex the next half-edge is the first one clockwise about the outward normal, so every cycle has its face on the left. Cycles with positive area are outer loops; negative ones are holes and go to the smallest outer loop that contains them.
3. **Classification.** Each fragment is classified against the other solid as inside, outside, on-with-same-normal or on-with-opposite-normal. A point on a face is detected first by an explicit point-on-face test; other points use the winding number of `ck_topo::region`.
4. **Selection.** The operation decides which fragments to keep, to flip (the faces of the tool in a cut) and to merge (coincident same-facing fragments, such as the caps of two flush blocks).
5. **Assembly.** A new body is built with exact pcurves, lumps are formed, and faces keep their names; the result's face order is fixed (faces of `a` untouched, `a` modified, `b` untouched, `b` modified, then merged faces).

Tool faces keep the tool feature's ids: cutting a hole with feature `F4` leaves a face `F4.S.e1` on the plate. A face split into several pieces gets `~1`, `~2` ... numbered by the lexicographic order of the fragment centroids.

### Tolerance ladder

Marched intersection curves meet their neighbours only to about `1e-6` to `1e-5`, so an arrangement can fail to close at the base tolerance. The boolean therefore escalates:

1. The plain attempt at `tol`. If an operand has B-spline or offset faces and `tol` is the default, the ladder starts at `10 tol` and falls back to `tol` only if the whole raised ladder fails.
2. On `booleanFailed`, retries at `10 tol` and `100 tol`.
3. If every attempt failed, *robust mode*: the same attempts at `10 tol`, `tol` and `100 tol` with three sliver-tolerant steps switched on (splitting two cut curves where they cross, merging edge pieces within `100 tol` that join the same vertices, and classifying a fragment sampled within a sliver's width of the other solid by a vote of several rays). Robust mode only ever turns a failure into a result. It stops early, with `booleanBudgetExceeded`, if the earlier attempts already used 12 seconds.

A result that assembled into a closed manifold is then checked against the operands (the *sanity* check):

| operation | volume bound |
|---|---|
| union | `max(va, vb) <= vr <= va + vb` |
| cut | `va - vb <= vr <= va` |
| intersect | `0 <= vr <= min(va, vb)` |

The overlap that the result implies must also fit in the overlap of the operands' boxes, and a union's box must contain both operand boxes. A result outside the bounds is valid but wrong (a region of an operand was dropped); the ladder tries looser tolerances and a split-only pass, and if nothing plausible comes out the feature fails with `booleanVolumeMismatch` and the body stays as before.

### Fuzzy booleans

Features that run a boolean accept `tolerance` (the *fuzzy value*, mm) and `autoFuzzy`. With a fuzzy value, every planar, cylindrical or spherical face of the tool that lies within the value of an equal-kind face of the host (same normal, axis or centre) is first moved onto that face exactly, so the boolean sees coincident faces and merges them. The result differs from the exact boolean of the unsnapped inputs by at most the fuzzy value, and a snap that fails or changes the tool's volume by more than the value times its area is discarded. With `autoFuzzy`, a boolean that fails, comes out implausible or comes out less valid than its operands is retried at `1e-7`, `1e-5` and `1e-3` (values above the explicit one), and the first plausible, valid result wins. The value used is reported as the warning `booleanFuzzy`.

### Special cases handled

- **Operands far from the origin** are shifted to a local origin and moved back.
- **Contact along an edge.** Two parts that touch only along an edge leave an edge shared by four faces. The edge is split into two coincident edges, one per material wedge, and the parts become separate lumps.
- **A cut that misses** is the error `cutNoEffect`; the body is unchanged. A tool that sits entirely in a cavity of the body is a valid no-op and gives no error.
- **A lone sphere with a pole on the other solid's surface** needs a special path, because the intersection circle passes through the poles and the uv polygon cannot hold a pole as a point.

### Failure

| code | meaning and parameters |
|---|---|
| `booleanFailed` | The result did not close. When the cause is diagnosed the error carries `faces` (the faces around the open edges), `issue` and `at` (a point), or `hostFace`, `toolFace` and `gap` when two nearly coincident free-form walls are the cause. Always with a `hint`. |
| `booleanTangent` | An exact tangency whose result has an odd Euler characteristic. Carries `euler` and a `hint`. |
| `booleanVolumeMismatch` | The result closed but is outside the sanity bounds. Carries `what` (`volume` or `box`), `expected_min`, `expected_max`, `got`. |
| `booleanBudgetExceeded` | The time budget ran out. |
| `cutNoEffect` | A cut or intersection changed nothing. Reported in `errors`; the body is unchanged. |

The usual remedies are in the hints: move the tool by a small amount (0.01 mm), unite bodies that touch first, make near-coincident faces exactly coincident, or give the feature a fuzzy `tolerance` between `1e-5` and `1e-3`.

## Blends: fillets and chamfers

A blend is built as a boolean with a *tool*, not by editing faces in place. For each edge, a tool solid (the corner prism between the two faces, with the rolling-ball profile as one face) is made and then cut from the body for a convex edge or united for a concave edge. The tool's profile face becomes the blend face `B<k>` of the feature. Straight edges between planes give a cylinder (fillet) or a plane (chamfer); circular edges between coaxial planes, cylinders and cones use a revolved profile; free-form edges (B-spline or revolution faces, spline intersection curves) are handled by a rolling-ball construction that fits a spline tool (`blend::freeform`).

| feature options | result |
|---|---|
| `fillet` `radius` | Circular profile, constant radius. A pick may carry its own `radius`; unequal radii at a vertex give a mitre and the warning `cornerNotTangent` with the largest normal jump (`deviation`, degrees). |
| `filletType: "variable"` with `radii: [near, far]` | The radius changes along each picked edge. An end radius of 0 is built with the smallest positive end radius that works (warning `filletRadiusApproximated`). |
| `filletType: "fullRound"` with `side1`, `center`, `side2` | A ball tangent to three planar faces; the center face vanishes. |
| `profile: "conic"` (`rho` in (0, 1)), `"chord"`, `"curvature"` (`shape` in [0.1, 1]) | Other section shapes. |
| `chamfer` `chamferType: "equal"` | Equal distance. |
| `"distances"` (`distance`, `distance2`) and `"angle"` (`distance`, `angle`) | Two distances or a distance and an angle, measured from the first face (named by `face`, or the first of each pick's faces); `flip` swaps the legs. |
| `"vertex"` | Cuts off the corner nearest `at` where three edges meet. |
| `propagate: true` | The selection grows along edges that continue tangentially. |

Corners are the hard part. Three convex edges with one radius meeting at a vertex get the rolling ball's sphere patch `V<k>`. A vertex with two convex edges and one concave edge is built in two stages (the concave edge first, then the two rim edges and the arc as one tangent chain, a torus patch). Four edges that flip between convex and concave at a vertex (a block overhanging a wall) get a B-spline corner patch `P0`. Reflex corners of two convex edges are cut at the bisector plane through the vertex.

Verified results (a 40 x 20 x 10 block, the edge between `F2.C1` and `F2.S.e1`):

| feature | volume | closed form |
|---|---|---|
| fillet, radius 2 | 7965.6637 | `8000 - (1 - pi/4) 4 * 40 = 7965.6637` |
| chamfer, equal 2 | 7920 | `8000 - 2 * 40 = 7920` |
| chamfer, distances 2 and 3 | 7880 | `8000 - 3 * 40 = 7880` |
| chamfer, distance 2, angle 30 | 7953.81 | `8000 - 0.5 * 2 * (2 tan 30) * 40` |

All results validated clean.

### Blend failures

| code | meaning |
|---|---|
| `invalidValue` | Radius or distance not positive. |
| `missingEdge` | An edge pick (`{faces, at}`) matches no edge. A pick takes the edge between the two named faces nearest to `at`. |
| `blendOverflow` | The blend runs off its faces. `detail` names the blend (`F3.B0: spring leaves its face away from its edges`). For curved ends, `max` gives the largest radius that fits, and `curvedEnd` says which end. A radius of 30 on a 10 mm block edge gives this error. |
| `blendTooLarge` | A distance exceeds the edge or face (vertex chamfers, variable radii). |
| `blendSize` | No blend fits between the faces. |
| `blendCorner` | A corner patch cannot be built (for example convex and concave edges that are not perpendicular); the hint says to blend them in separate features. |
| `blendNonManifold`, `blendFailed`, `blendUnsupported` | The tool could not be applied, or the case is outside what the builders cover (with a `detail` where known). |
| `blendTolerantEdge` | An imported edge carries a tolerance; the blend needs an exact edge. |
| `variableRadiusMismatch` | Edges that meet at a vertex have different radii there. |
| `filletRho`, `filletShape` | Profile parameter outside its range. |

## Loft

`loft` makes a solid or, with `surface`, a sheet through two or more closed profiles (open chains for a sheet).

- **Stations.** The profiles are brought to a common number of pieces: the longest piece of a profile with fewer pieces is bisected at its parameter midpoint until the counts agree (a lone circle becomes two half circles). A profile that runs the other way round than its predecessor is reversed, and each profile's start is rotated to the cyclic shift that minimises the summed squared distance between the start points of consecutive stations.
- **Rails.** The edge through matching points of the stations is a straight line for two profiles and a natural cubic spline through them for more, parameterised by cumulative chord length between the station centroids.
- **Faces.** Profile pieces become rational B-splines (arcs as quarter-turn quadratics). Side faces interpolate the homogeneous control points of the stations, so the profiles are exact iso-curves. Two parallel flat squares give planes; a square to a circle gives four spline faces. Names: `S.<entity>` per piece of the first profile (cut pieces `~k`), `C0` at the first profile, `C1` at the last.
- **Guides** are curves that touch every profile. They cut the profile pieces at the contact, and a face that has a guide as a rail becomes a Gordon surface (rails and station curves blended, minus their tensor product), which contains the guide and the stations exactly. `influence` other than `next` is refused.
- **A centre line** makes the sections between the profiles follow a planar spine. A moving frame carries each profile's local coordinates along the spine and the coordinates are interpolated by a natural cubic spline. The end profiles stay pinned exactly, and a straight spine changes nothing.

A 20 x 20 square lofted to a 10 x 10 square 30 mm away has volume 7000, the frustum formula `(A1 + A2 + sqrt(A1 A2)) h / 3`. The result of a loft is validated; if the default matching gives an invalid body, other matchings are tried. Codes: `loftProfiles` (fewer than two), `loftProfileOpen`, `loftProfileNotPlanar`, `loftProfileSelfIntersect`, `loftSectionTooSmall`, `loftGuideInvalid`, `loftGuideMiss`, `loftGuideOrder`, `loftCentreLineReach`, `loftCentreLineTight`, `loftCentreLineKink`, `loftCentreLineApex`, `loftCentreLineGuides`, `loftFailed` and `loftInvalidResult`.

## Sweep

`sweep` moves a planar profile along a path of lines, arcs, helices and splines.

- A path that is one rigid motion is built exactly. All lines in one direction is a translation (built like an extrude), an arc about an axis is a rotation (built like a revolve), and a helix is a screw motion with B-spline side faces.
- A path of several pieces (lines and arcs) is swept piece by piece and the pieces are united with mitre joints. Tangent joints need no mitre; a corner between two lines is mitred when it turns by less than 166 degrees. A 6 mm circle swept along a 20 mm line and a perpendicular 20 mm line gives a volume of 1130.9734, equal to 2e-9 relative to a straight tube of length 40. Helices and splines inside a multi-piece path are refused (`sweepPathInvalid`).
- A single spline path (a `sketch3d` spline: a natural cubic through the points, parameterised by chord length) carries the section by rotation-minimising frames (the double-reflection method), which do not roll with the torsion as Frenet frames would. Rows are placed at least 2 per cubic piece and in proportion to how much the tangent turns, and the side faces interpolate the rows.
- `twist` rotates the profile about the path as it travels. It is supported along a straight path only. On a multi-piece path the feature fails with an error whose `code` is the text `TODO(rebuild): twisted sweep along a multi-piece path`; codes that start with `TODO(rebuild)` mark cases that are not implemented yet and are not stable names. A 6 x 6 square swept 20 mm with a 90 degree twist keeps its area at every station: the volume is 720.
- The end of the path nearest to the profile starts the sweep, and the path direction is otherwise irrelevant.
- **Guides** (one or two) make the section follow guide curves, with modes `scale` (the default: scale and turn), `turn` and `stretch` (two guides).

The path tangent at the start must be perpendicular to the profile plane to within `5e-5` (`sweepProfileNotNormal`). Other codes: `sweepPathInvalid`, `sweepGuidesCount`, `sweepGuideStretchTwo`, `sweepGuideNotTouching`, `sweepGuideNotAlong`, `sweepGuideShort`, `sweepGuidesSamePoint`, `sweepGuideDegenerate`, `sweepGuideOnPath`, `sweepGuidesTwist` and `sweepFailed`.

## Offset and shell

`offset` moves every face of a closed body along its outward normal by its own signed distance. Vertices are re-solved as the point at the right distance from all incident offset surfaces, and edge curves follow their vertices (lines keep their direction, circles stay coaxial). Free-form faces are re-fitted to B-splines (internal fit tolerances: `1e-5` for surfaces, `3e-3` for edge curves). Codes: `offsetTooLarge` (the offset passes a centre of curvature, an edge shrinks to nothing or reverses), `offsetUnsupported`, `offsetDegenerate`, `offsetFitFailed`.

`shell` is `body - cavity`: the cavity is the body offset inward by the wall thickness on every kept face, and outward by a quarter of the wall on each opened face so that the cavity runs out through it. The opened face survives as a rim ring under its own name; every other face is split by the boolean into an outer and an inner wall, named `~1` and `~2` by centroid order. With `outward: true` the wall grows outside instead. For a 40 x 20 x 10 block shelled with thickness 2 through the top face, the volume is 3392 (= 8000 - 36 * 16 * 8) and the body validates clean. Shelling a free-form face is supported for lofted hulls; the margin by which an opened free-form face's offset is continued is capped at 12 wall thicknesses and reduced if the continuation folds.

`shellTooThick` reports the cause in `detail` and gives `maxThickness` and `minRadius`. `maxThickness` is a safe estimate, not the exact limit: for the 40 x 20 x 10 block the true limit is 10, and the replies gave 8.9 to 9.35.

## Draft

`draft` rotates planar faces about a hinge (the intersection with the neutral plane) and turns cylinders into cones. The face normals are exact; edges and vertices are re-derived by intersecting the new surfaces. Draft 5 degrees on the four sides of the block about its bottom face gives volume 7485.27, matching `8000 - 3000 tan(5) + (1000/3) tan^2(5)` with the tangent of the draft. Codes: `draftParallelFace` (a face parallel to the neutral plane), `draftAxisNotParallel`, `draftTangentFaces`, `draftTooLarge` (an edge turns over; `detail` names it), `draftUnsupported`, `draftFailed`. Extrusion with a draft is a separate path in `ck-topo`.

## Helix

`helix` produces a curve, not a body: `p(t) = o + r (x cos t + y sin t) + a (pitch / 2 pi) t` for `t` in `[0, 2 pi * revolutions]`, on the circle of its sketch, with `leftHand` and `reverse` options. `definedBy` is `pitchRevs`, `heightRevs` or `heightPitch`. The curve is sampled with `max(2, ceil(32 * revolutions))` segments. The helix serves as a sweep path and as a thread axis.

## Pipe cut

`pipeCut` removes everything of the body's faces that a ball of radius `r` rolled along the selected edges touches. The result is a *surface body* (a sheet): the pipe surface itself is not part of it, and `fill` can close the gap. The cutters (a cylinder or torus sector per edge, a ball per end vertex) only split faces in the boolean's imprint stage; a fragment is kept when the distance from an interior sample point to the nearest selected edge is at least `r`. Codes: `pipeCutRadius` (with `max`, 0.999 of a circle edge's radius), `pipeCutFailed`, `pipeCutNothingLeft`.

## Surfacing

Surface bodies are open shells kept apart from the solid. The operations are:

| operation | what it does | codes |
|---|---|---|
| `knit` | Sews faces from several bodies into one by merging coincident edges (tolerance `1e-6`). `solid: true` requires a closed result. | `knitNotClosed`, `knitNearMiss` |
| `trimSurface` | Cuts surfaces with a tool plane and keeps one side; `mutual` keeps the tool uncut. All-planar inputs take an exact path (a half-plane clip in uv); anything else goes through surface-surface intersection. | `trimNoIntersection`, `trimNothingLeft` |
| `extend` | Extends a sheet across an open boundary edge by a distance or to a target surface. | `extendNotBoundary`, `extendNoTarget`, `extendPointBehind`, `extendUnsupported`, `extendFolds` |
| `fill` | Patches a boundary chain plus its closing chord. A planar boundary gives the exact plane. For a non-planar boundary the patch is a ruled surface between the chain and the chord: the fairing modes (`meowNURBS`, `plate`, `curvature`, `variation`) and the continuity options (`G0` to `G3`) are accepted, but they do not yet shape a non-planar patch (a known gap). | `fillNotClosed`, `fillInvalid`, `fillFailed` |
| `thicken` | Turns a sheet into a solid wall (`front`, `back` or `both`), mitring corners. Creased free-form sheets are offset face by face with mitred corner points. | `thickenReach` (offsets do not meet at a corner), `thickenTooThick`, `thickenFailed` |
| `rebuild` (`freeform`) | Re-fits a face onto a degree and span net, keeping its edges. | `rebuildPeriodicFace`, `rebuildTooFine`, `rebuildFailed` |

Verified: a 20 + 10 mm open chain extruded 10 mm and thickened with thickness 2 on `both` gives a volume of 600, which is the sheet area 300 times 2, with walls `F3.W0` to `F3.W5`.

## Measurement and analysis

`measure`, `interference`, `section`, `probe`, `distance`, `thickness` and the related queries are operations in `ck-ops` too (`measure.rs`, `interference.rs`, `section.rs`, `analysis.rs`, `dfm.rs`). They are described, with accuracy, in [mass properties and interference](../validation/analysis.md).

## Tolerances at a glance

| where | value |
|---|---|
| Model tolerance passed to operations, boolean welding distance, intersection tolerance | `1e-6` mm |
| Boolean retry ladder | `10x`, `100x` the welding distance; robust mode adds the base again |
| Fuzzy boolean values tried by `autoFuzzy` | `1e-7`, `1e-5`, `1e-3` mm |
| Marched intersection fit | deviation within `tol / 100` |
| Free-form offset fit: surfaces; edge curves | `1e-5`; `3e-3` |
| Sweep profile-normal check | `5e-5` |

See [tolerances](../validation/tolerances.md) for how these relate to the validator's.

## See also

- [Geometry](geometry.md)
- [Regeneration and caching](regeneration.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
- [Edge and face edits](../format/features-edits.md)
- [Solids from sketches](../format/features-solids.md)
- [Surfaces and curves](../format/features-surfaces.md)
