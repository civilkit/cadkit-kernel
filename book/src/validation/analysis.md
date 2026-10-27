# Mass properties and interference

This chapter describes the read-only requests that measure a body: mass properties, `measure`, `interference`, sections and probes, and the distance, thickness and clearance family. For each it says what is computed, how, and how accurate the answer is, with values checked against closed forms. All of them read the body of a tag and never change it.

## Common behaviour

- Every request takes `tag` (default `main`) and answers `null` when the tag holds no solid body, and `{"ok": false, "error": "noState: ..."}` when the tag has no state.
- Lengths are in millimetres, volumes in cubic millimetres, areas in square millimetres.
- The requests from `distance` onwards stop their loops after `opts.timeMs` (default 8000 ms) and answer with what they have. Their replies carry `elapsedMs` and `truncated`; `truncated: true` means the result is partial, for example a lower bound of a maximum.
- Replies name faces by their persistent name, so an answer can be used as a reference in a later feature.

The examples use a 40 x 20 x 10 block (feature `F2`, an extrude of the rectangle `F1`) with a radius 4 hole through its middle, made by a circle sketch `F3` at (20, 10) extruded as a cut `F4` with `end: "throughBoth"`. The hole wall is then `F4.S.e1`. A hole made with the `hole` feature instead names its wall `F4.S.h1` (see [persistent names](../kernel/naming.md)); the numbers below are the same.

## Mass properties

```json
{ "type": "massProps", "tag": "main", "bodies": true, "densities": [0.0078] }
```

| field | meaning |
|---|---|
| `bodies` | `true` adds `bodies[]`, one entry per lump: `volume`, `area`, `com`, and `name` when the lump has one. |
| `densities` | One density per lump (0.0078 g/mm3 for steel). Implies `bodies: true`. A length that is not the number of lumps is an error that names the expected count. |

Reply:

| field | meaning |
|---|---|
| `volume`, `area` | Total volume and surface area. |
| `com` | Centre of mass of the volume (density 1). |
| `principal` | The three principal moments of inertia about the centre of mass, ascending, for density 1. |
| `axes` | The unit eigenvectors of `principal`: `axes[i]` belongs to `principal[i]`. |
| `bodies[]` | Per-lump `volume`, `area`, `com`, and `mass = volume * density` when `densities` was given. |
| `byMass` | With `densities`: `mass`, `com`, `principal` and `axes` of the whole body weighted by the densities. |

For the block with the hole the reply is (rounded):

```json
{ "volume": 7497.3452, "area": 2950.7964, "com": [20, 10, 5],
  "principal": [327133.92, 1127133.92, 1329312.09],
  "axes": [[1, 0, 0], [0, 1, 0], [0, 0, 1]] }
```

With `densities: [0.0078]` the lump also has `mass: 58.4793` (7497.3452 x 0.0078).

### How it is computed

Volume, area, centroid and inertia come from the faces by the divergence theorem, with no mesh. Each face integral over the trimmed parameter domain becomes a boundary integral by Green's theorem along the coedge pcurves, and both integrals use composite 8-point Gauss-Legendre quadrature. The principal moments come from a Jacobi eigen-solver for the symmetric inertia tensor. See [the B-rep model](../kernel/brep.md).

### Accuracy

Analytic faces with exact pcurves reach machine precision. Results through the request against closed forms:

| solid | volume relative error | area relative error |
|---|---|---|
| block 40 x 20 x 10 | 7e-16 (`8000.0000000000055`) | 3e-16 |
| block with radius 4 hole | 1e-15 (7497.34517542564 against 7497.345175425633) | 5e-16 |
| sphere, radius 10 | 4e-16 | 2e-16 |
| cylinder, radius 50, height 100 | 3e-16 | 3e-16 |
| torus, R = 10, r = 3 | 8e-16 | 0 |

The principal moments of the plain block are `V (b^2 + c^2) / 12` for each axis: 333333.33, 1133333.33 and 1333333.33. In the repository's mass-property sweep (box, cylinder, cone, sphere, torus, tube, two-station loft, revolve, line, arc and L sweeps), volume, area, centre and inertia agree with an independent mass-property integrator to about 1e-14 or better. The one exception in that sweep is the three-section loft through circles: the integrator is exact for the surface the kernel builds, but that surface differs from the thru-sections surface of the independent system, and the two disagree by up to 6.7 percent in volume and 10 percent in inertia. That is a difference in the shape, not in the integration; see [known limits](../quality/limits.md).

Free-form faces (B-spline, swept, lofted) are integrated by the same quadrature on their spans. The quadrature error is small compared with the fit error of the surface itself, so for a free-form body the accuracy of the answer is the accuracy of the surface, not of the integral.

### Limits

- **Overlapping lumps count twice.** Two lumps that overlap (a `newBody` over a block) are summed: lumps of 8000 and 2000 mm3 that overlap by 1000 mm3 give 10000. Use `interference` to find overlaps and `combine` to unite them.
- **Repeated principal moments.** For a body with a repeated moment (a cylinder, a sphere, a cube) the axes are an arbitrary orthonormal basis of the eigenspace, with no sign convention. The moments are exact; the directions carry no meaning.
- **Sheets have no volume.** Sheet bodies are kept apart from the solid state, and the request answers for the solid only.

## measure

```json
{ "type": "measure", "refs": [ {"kind": "face", "id": "F2.S.e1"}, {"kind": "face", "id": "F4.S.e1"} ] }
```

A reference is a face `{kind: "face", id}` or an edge `{kind: "edge", faces: [a, b], at}`. The reply has `items` with the facts of each reference and, for exactly two references, the minimum distance and the points:

- a face item: `kind`, `surface`, `area`; for a plane the outward `normal`, for a cylinder `radius`, `center` and `axis`, for a cone `center` and `axis`, for a sphere `radius` and `center`;
- an edge item: `kind`, `length`; for a line `direction`; for a circle `radius`, `center` and `axis`; and `wedge`, the angle between the outward normals of the two faces at the edge midpoint;
- for a pair: `distance` and `points`, and where they apply `angle`, `normalDistance` and `centerDistance`.

The block's side face and the hole wall are 6 mm apart, with `points` `[[20, 0, 0], [20, 6, 0]]`. The area of the hole wall is 251.3274, which is `2 pi * 4 * 10`. The pair distance samples both references, projects every sample onto the other and refines the best three by alternating projection, so two references that touch pick one of their zero-distance points. The `center` of a cylinder is the origin of the surface's frame, not necessarily a point inside the face: its position along the axis depends on how the face was built: `(20, 10, -78.7)` for the circle-cut wall above in one run, `(20, 10, 10)` with axis `(0, 0, -1)` for a wall made by the `hole` feature.

## interference

`interference` reports overlap between the lumps of a body: `{bodies, pairs}`, where each pair is `{a, b, volume, com, box}` (and `aName`, `bName` when the lumps are named) describing the volume common to two lumps, computed as a boolean of the pair.

For the 40 x 20 x 10 block and a second block from `x` 30 to 50 and `y` 5 to 15, added with `newBody`:

```json
{ "bodies": 2,
  "pairs": [ { "a": "F2.C0", "b": "F4.C0", "bName": "tool", "volume": 1000,
               "com": [35, 10, 5], "box": { "min": [30, 5, 0], "max": [40, 15, 10] } } ] }
```

The volume 1000 is the 10 x 10 x 10 overlap. Lumps that only touch give no pair. A pair whose boolean fails reports an `error` instead. A union merges overlapping solids, so a body gets overlapping lumps only from `newBody` and `moveBody`. `validate` reports the overlap as information.

## section and sections

```json
{ "type": "section", "plane": { "origin": [0, 10, 0], "normal": [0, 1, 0] } }
```

The reply gives `loops`, each `{kind: "outer" | "inner", closed, area, edges, pts}`, the plane `frame` (`z` is the normalised normal, `x` is the world Z crossed with the normal, or world X when the normal is within 2.5 degrees of Z), the signed total `area` (outer minus inner), `bbox2d`, and `meshArea`, the area of the same cut computed from the display polygons (320.041 for the section at `y = 10` below, against the exact `area` 320). `area` is the number to use; `meshArea` is a check on the cut. Coordinates are in the plane frame. Loops run counter-clockwise around material. A plane that misses the body gives no loops and area 0.

The edges of a loop are exact where the geometry allows: `line` for planar faces; `arc` with `centre`, `radius`, `startAngle` and a signed `sweep` for runs of one curved face that lie on a circle (a cylinder, cone or torus cut perpendicular to its axis, any sphere cut); otherwise a `polyline` whose vertices are on the exact surface and whose sagitta is below `2e-5` of the body diagonal.

How it works: the display mesh of each face is cut by the plane, the segments are chained into loops, each loop vertex is snapped onto the exact surfaces that meet there by Newton iteration, and curved segments are bisected until the sagitta is small enough. Tangent slivers with an area below `1e-9` of the squared diagonal are dropped. If a face failed to mesh, the loops are incomplete: `openChains` appears and those pieces are omitted.

For the block with the hole, the section at `y = 10` (through the hole's axis) has area 320 (`40 * 10 - 8 * 10`) in two loops of 160. The section at `z = 5` has an outer loop of area 800 and an inner loop that is an exact arc of radius 4 with area 50.2655 (`pi * 4^2`).

`sections` takes `axis` (`x`, `y`, `z` or a vector) and `n` (default 21) or explicit `positions`, and returns `areas` and the number of loops at each position, with `volume` integrated from the areas (midpoint rule for cell centres, trapezoid rule for explicit positions), `exactVolume` from the mass properties and the relative `volumeError`; `exact: false` skips the mass-property computation. For the block with the hole and 5 sections along `z`, every section has area 749.7345, and the integrated volume 7497.34519 differs from the exact 7497.34518 by 2e-9. For smooth profiles the rule's error is about 1e-3 at 41 sections; a profile with steps needs more, while the section areas themselves stay exact.

## probe

```json
{ "type": "probe", "points": [[20, 10, 5], [1, 1, 5]], "rays": [ { "origin": [-5, 10, 5], "dir": [1, 0, 0] } ] }
```

At most 2000 points and 2000 rays per request. For each point: `inside` (`true`, `false` or `"boundary"`), the signed `distance` to the nearest face (negative inside, 0 on the boundary), the `face` and the foot point `nearest`. For each ray: `hits`, sorted by `t` (the direction is normalised, so `t` is a distance), each `{t, point, face, entering}`, with `tangent: true` on a grazing hit.

On the block with the hole:

| query | answer |
|---|---|
| point `(20, 10, 5)` (in the hole) | `inside: false`, `distance: 4`, face `F4.S.e1`, nearest `(24, 10, 5)` |
| point `(1, 1, 5)` | `inside: true`, `distance: -1`, face `F2.S.e1` |
| point `(24, 10, 5)` (on the hole wall) | `inside: "boundary"`, `distance: 0` |
| point `(50, 50, 50)` | `distance: 50.9902`, which is `sqrt(10^2 + 30^2 + 40^2)` |
| ray from `(-5, 10, 5)` along `+x` | hits at `t` = 5 (enters, `F2.S.e4`), 21 (exits, hole wall), 29 (enters, hole wall), 45 (exits, `F2.S.e2`) |

Containment uses a parity test of up to seven fixed-direction rays, falling back to the nearest face when every ray is ambiguous.

## Distance, thickness and the measurement set

These requests live in `ck-ops::analysis`. Entities (`a`, `b`, `ids`) are a face id, an edge key, a vertex `V<n>`, a point `[x, y, z]`, `lump:<n>` or `"body"`, or an array of those.

| request | what it computes | accuracy |
|---|---|---|
| `distance {a, b}` | The minimum distance of two entities, with the two points, what they lie on, and `intersects: true` with the extent when they touch. `tagB` takes `b` from another tag. | Exact point-to-surface projections, a closed form for two segments, and a multi-start alternating projection otherwise. Against an independent routine on exported STEP, 80 face pairs of analytic parts agree to 4e-15 and 60 pairs on B-spline parts to 2.6e-8. A minimum in the interior of two doubly curved faces and far from every grid sample can be missed; raise `opts.samples`. |
| `maxDistance {a, b}` | The one-sided Hausdorff distance: the maximum over `a` of the distance to `b`. Reply: `maxDistance`, `location`, `errorBound`, `upperBound`, `rms`, `coverage`. | The distance function is 1-Lipschitz, so the true maximum lies in `[maxDistance, maxDistance + errorBound]`. |
| `thickness` | Wall thickness: from samples on every face, a ray along the inward normal to its first exit. `thin` lists the thinnest places, clustered. `method: "sphere"` refines the thinnest with an inscribed ball. | A sampled minimum; a feature thinner than the sample spacing can be missed (raise `opts.samples`). The ray method overestimates in wedges and at corners. |
| `thicknessAt {point}` or `{face}` | Thickness at one place, or min, max and mean over a face. | As above. |
| `closestPoint {points, on, ids, k}` | The nearest point of the body (or of faces, edges or vertices) to each point, what it lies on, and the signed distance. At most 2000 points. | Exact projection. |
| `angles` | Per two-faced edge the interior dihedral angle through the material (90 for a block edge, 270 for a re-entrant edge), the type `convex`, `concave` or `smooth`, and min and max along the edge. Also the angle between two faces, and draft angles against a pull direction. | Exact for planar faces; sampled at five points otherwise. |
| `clearance {min}` | Face pairs closer than `min`, classed `gap`, `wall` or `contact`. | Seeds from the display mesh and refines on the surfaces; a face whose mesh failed can hide a pair. Pairs that share a vertex are skipped. |
| `symmetry` | Candidate mirror planes, each with the maximum and rms deviation of the mirrored surface. The same plane can be listed twice when a coordinate axis and a principal axis coincide (reported for some bodies; the block with the hole lists three distinct planes). | Sampled; a plane reports `exact: true` when every sample was measured. |
| `extrema {dir}` | The lowest and highest point of the body along a direction. | Exact (`method: "vertices"`) when every face is planar and every edge straight; otherwise sampled and refined to about 1e-9. |

### angles

`angles` needs at least one of these parameters; a bare request answers `angles: edges ('all' | [keys]), faces [a, b] or draft {pull} is required`.

| parameter | meaning |
|---|---|
| `edges` | `"all"` or a list of edge keys such as `"F2.C1\|F2.S.e1#2"`. |
| `faces` | `[a, b]`: the angle between two faces. |
| `draft` | `{pull, faces?, min?}`: draft angle of each face (or of `faces`) against the pull direction; with `min`, walls under that angle are listed in `violations`. |

| reply field | content |
|---|---|
| `edges[]` | `edge`, `faces`, `angle` (interior dihedral through the material, degrees), `normalAngle`, `type` (`convex`, `concave`, `smooth`), `min`, `max`, `at`. For the edge above: `angle` 90, `convex`, `at` `[20, 0, 10]`. |
| `between` | For two faces with a shared edge: `angle`, `normalAngle`, `type`, `sharedEdge`, `at` (90 for `F2.S.e1` and `F2.S.e2`). Without a shared edge: `normalAngle`, `angle`, `distance`, `pointA`, `pointB`, `parallel` (the side `F2.S.e1` and the hole wall: `normalAngle` 180, `distance` 6, `parallel: true`). |
| `draft[]` | `face`, `draft` (degrees, `asin(n . pull)`), `min`, `max`, `class` (`vertical` below 0.01, `undercut`, `parting` above 89.99, `ok`). With pull `(0, 0, 1)` the block's sides are `vertical`, `F2.C0` is `undercut` at -90 and `F2.C1` `parting` at 90. |
| `elapsedMs`, `truncated` | As for every request of this group. |

### closestPoint

| parameter | meaning |
|---|---|
| `points` | Required: `[[x, y, z], ...]`, at most 2000. Without it the answer is `closestPoint: points [[x, y, z], ...] is required`. |
| `on` | `"body"` (default), `"face"`, `"edge"` or `"vertex"`. |
| `ids` | Restrict `face`, `edge` or `vertex` queries to these entities. |
| `k` | With `k > 1`, return the `k` nearest entities per point (one list per point). |

The reply is `{on, results, elapsedMs, truncated}`. For `on: "body"` each result has `signedDistance` (negative inside), `inside`, and `nearest: {distance, point, entity: {kind, id}, face, uv}`; for the point `(30, 10, 5)` it is `-5` with the nearest point `(30, 10, 0)` on `F2.C0`. For `on: "face"`, `"edge"` or `"vertex"` each result is `{distance, point, entity}` plus `uv` for a face or the curve parameter `t` for an edge (for `(30, 10, 5)` and `on: "edge"`: distance 7.8102 to the edge `F2.C0|F4.S.e1#12` at `(24, 10, 0)`).

Verified on the block with the hole:

- `distance` between the side `F2.S.e1` and the hole wall `F4.S.e1`: 6, with points `(20, 0, z)` and `(20, 6, z)`.
- `thickness`: `minThickness` 6 (the wall between the hole and the side) and `meanThickness` 18.69, from 2485 samples.
- `clearance` with `min: 7`: two pairs at distance 6, both of kind `wall` (the hole wall against `F2.S.e1` and against `F2.S.e3`).
- `maxDistance` from the hole wall to the plane `x = 40`: `maxDistance` 23.9997, `errorBound` 1.04, `upperBound` 25.04. The true maximum is 24; it lies inside the bound, and the lower end is within 3e-4 of it.
- `symmetry`: the three coordinate planes through the centre of mass are symmetric, with a maximum deviation near 3e-15.
- `extrema` along `x`: 0 to 40, `extent` 40.

## check: expectations

`check {expect}` measures the body and compares it with expected values in one call. The kinds are `volume`, `area`, `mass` (with `density`), `bbox`, `com`, the counts `faces`, `edges`, `vertices`, `solids`, `holes` and `innerLoops`, and `valid` (the validator reports no issue). An expectation is `{kind, equals | min | max, tol?, relTol?}`, and the reply is `{ok, passed, failed, results, measured}`. For the block with the hole, `volume equals 7497.35 relTol 1e-6` passes (the window is plus or minus 0.0075), `holes equals 1` passes, and `faces equals 8` fails with `faces 7 vs [8, 8]`. The default tolerance for volume and area is 1e-4 relative.

## See also

- [Validation overview](overview.md)
- [The B-rep model](../kernel/brep.md)
- [Queries (protocol)](../protocol/queries.md)
- [Tessellation (kernel)](../kernel/tessellation.md)
