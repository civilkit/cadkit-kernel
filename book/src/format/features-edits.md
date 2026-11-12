# Edge and face edits

These features change the body that earlier features made: they round or bevel edges (`fillet`, `chamfer`), tilt faces (`draft`), hollow the body (`shell`), cut holes and threads (`hole`, `thread`), add or cut a band along a rim (`lip`), push faces (`moveFace`) and remove whole bodies (`deleteBody`). They refer to faces by name and to edges by the two faces that meet there (see [identifiers and references](references.md)). The examples append a feature to the plate of [the overview](overview.md) (40 x 20 x 10 mm, faces `F2.C0`, `F2.C1`, `F2.S.e1` ... `F2.S.e4`).

A failed edit leaves the body as it was before the feature.

## fillet

Rounds edges with a circular arc.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `radius` | number or expression | `0` | Radius in mm. It must be above 0 (`invalidValue`). |
| `edges` | list of edge picks | required | `{"faces": [a, b], "at": [x, y, z]}`; optional `radius` or `radii` per pick. No edges, or an edge that cannot be found, is `missingEdge`. |
| `profile` | string | `"circular"` | `circular`, `conic` (`rho` between 0 and 1, default 0.5; `filletRho` otherwise), `chord` (the radius is the chord width) or `curvature` (`shape` between 0.1 and 1, default 0.5; `filletShape` otherwise). |
| `propagate` | boolean | `false` | Grow the selection along edges that continue the picked ones tangentially (the outline of a rounded rectangle from one pick). |
| `filletType` | string | omitted | Leave it out for a constant radius. `"variable"` (see below) and `"fullRound"` are the only accepted values; any other string, including `"circular"`, is `unsupported`. |
| `radii` | `[near, far]` | none | `variable`: the radius at the end of each picked edge nearer the pick point, and at the other end. Both at least 0, not both 0. |
| `side1`, `center`, `side2` | face names | none | `fullRound`: the three faces the ball rolls between. |

A pick may carry its own `radius` (that edge is rounded with it; the feature's `radius` is the default) or, for a variable fillet, its own `radii`. Edges that meet at a vertex with different radii end in a mitre and the feature carries a `cornerNotTangent {deviation}` warning with the largest normal jump in degrees. Edges that meet at a vertex of a variable fillet must have the same radius there (`variableRadiusMismatch`).

Faces: `B0`, `B1`, ... one per rounded edge in the order of the picks; `V0`, ... for the spherical patch where three rounded edges meet at a convex corner.

```json
{
  "type": "fillet", "id": "F3", "radius": 2,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7965.6637 mm³. The new face is `F3.B0`. The removed material is 40 x (4 - pi 2²) / 4 = 34.3363 mm³.


A pick may set its own radius:

```json
{
  "type": "fillet", "id": "F3", "radius": 2,
  "edges": [
    {"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10], "radius": 1},
    {"faces": ["F2.C1", "F2.S.e3"], "at": [20, 20, 10]}
  ]
}
```

Run result: no errors; 8 faces, volume 7957.0796 mm³. Faces `F3.B0`, `F3.B1`: the first edge has radius 1 and the second the default 2.


A variable radius, 1 mm at the end of the edge nearer the pick and 3 mm at the other:

```json
{
  "type": "fillet", "id": "F3", "filletType": "variable", "radii": [1, 3],
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7962.8023 mm³.


`propagate` follows tangent edges; here the second pick is the next edge of the outline:

```json
{
  "type": "fillet", "id": "F3", "radius": 2, "propagate": true,
  "edges": [
    {"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]},
    {"faces": ["F2.C1", "F2.S.e2"], "at": [40, 10, 10]}
  ]
}
```

Run result: no errors; 8 faces, volume 7949.2625 mm³. Faces `F3.B0`, `F3.B1`.


`fullRound` rolls a ball between two side faces and a centre face; the centre face vanishes:

```json
{
  "type": "fillet", "id": "F3", "filletType": "fullRound", "side1": "F2.S.e1", "center": "F2.C1",
  "side2": "F2.S.e3"
}
```

Run result: no errors; 4 faces, volume 6283.1853 mm³. The 10 mm high plate becomes a half cylinder of radius 10: pi 10² / 2 x 40 = 6283.1853. The top face is gone and one round face `F3.B0` remains.


A radius the faces cannot take:

```json
{
  "type": "fillet", "id": "F3", "radius": 15,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: errors `F3`: `blendOverflow` `{"detail":"F3.B0: spring leaves its face away from its edges"}`; 6 faces, volume 8000 mm³.


Errors: `invalidValue` (radius not above 0, bad `radii`), `missingEdge`, `blendOverflow {detail}` (the round leaves its faces), `blendTooLarge`, `blendFailed {detail}`, `blendUnsupported`, `unsupported {what}`, `filletRho`, `filletShape`, `variableRadiusMismatch`, `fullRoundUnsupported`, `missingReference` (a face of `fullRound`), `badExpression`. Warnings: `cornerNotTangent`, `filletRadiusApproximated` (a variable end radius of 0 is built with a small positive radius; `params.used` says which).

## chamfer

Bevels edges.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `chamferType` | string | `"equal"` | `equal`, `distances` (alias `distanceDistance`), `angle` (alias `angleDistance`), `vertex`. Any other value is `unsupported {what: "chamfer <type>"}`. |
| `distance` | number or expression | `0` | The leg length. Must be above 0. |
| `distance2` | number or expression | `distance` | `distances`: the leg on the second face. |
| `angle` | number or expression | none | `angle`: degrees between the chamfer and the first face. |
| `face` | face name | first face of each pick | The face on which `distance` is measured. Every pick must border it. |
| `flip` | boolean | `false` | Swaps the two legs. |
| `propagate` | boolean | `false` | As for `fillet`. |
| `edges` | list of edge picks | required | For `vertex`: one pick `{"faces": [f1, f2, f3], "at": [..]}` naming the three faces of the corner. |
| `distance3` | number or expression | `distance` | `vertex`: the third leg (`distance2` is the second). |

A chamfer has constant legs along its edge: there is no variable chamfer (setbacks that differ at the two ends of an edge). A variable `fillet` has `radii`. The order of the `edges` list does not change the result.

For `distances` and `angle`, `distance` is measured on the first face of the pick (the first entry of `faces`) unless `face` names it. Picks of one feature must agree on which face comes first (mixed orders are `unsupported`); naming `face` makes a ring of edges agree.

Faces: `B0`, `B1`, ... per chamfered edge.

```json
{
  "type": "chamfer", "id": "F3", "chamferType": "equal", "distance": 2,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7920 mm³. Removed: 40 x 2 x 2 / 2 = 80 mm³. New face `F3.B0`.


Two distances; `distance` is on the first face of the pick:

```json
{
  "type": "chamfer", "id": "F3", "chamferType": "distances", "distance": 2, "distance2": 3,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7880 mm³. Removed: 40 x 2 x 3 / 2 = 120 mm³; 2 mm on `F2.C1` (the first face of the pick) and 3 mm on `F2.S.e1`.


A distance and an angle:

```json
{
  "type": "chamfer", "id": "F3", "chamferType": "angle", "distance": 2, "angle": 30,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7953.812 mm³.


A vertex chamfer cuts off a corner:

```json
{
  "type": "chamfer", "id": "F3", "chamferType": "vertex", "distance": 2,
  "edges": [{"faces": ["F2.C1", "F2.S.e1", "F2.S.e4"], "at": [0, 0, 10]}]
}
```

Run result: no errors; 7 faces, volume 7998.6667 mm³.


Errors: `invalidValue` (distance not above 0), `missingEdge`, `unsupported`, `chamferVertexEdges` (the corner does not have three edges), `blendTooLarge`, `blendOverflow`, `blendFailed`, `blendUnsupported`.

## draft

Tilts faces about a neutral plane.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `neutral` | plane reference | required | The plane that stays put: a standard plane, a plane feature or a planar face. Its normal is the pull direction (reversed by `flip`). A face neutral pulls into its material. |
| `faces` | list of face names | required | The faces to tilt. A name without `~k` selects every piece. Faces that continue a drafted face tangentially are drafted with it. |
| `angle` | number or expression | `0` | Degrees. |
| `flip` | boolean | `false` | Reverses the pull direction. |

Face names do not change. An `angle` of 0 (also the default when `angle` is missing) is `invalidValue`. Errors: `nothingToCut` (no body), `missingReference` (neutral or faces), `noFaces`, `draftParallelFace`, `draftAxisNotParallel`, `draftUnsupported`, `draftTangentFaces`, `draftTooLarge {detail}` (an edge turns over; for tapers that close to a point `maxHeight`), `draftFailed`, `invalidValue`.

```json
{
  "type": "draft", "id": "F3", "neutral": {"kind": "std", "id": "front"}, "faces": ["F2.S.e1"],
  "angle": 5
}
```

Run result: no errors; 6 faces, volume 7825.0227 mm³. The face `F2.S.e1` keeps its name and leans by 5 degrees; the volume drops by 174.9773 mm³.


With a face as neutral plane, the four sides tilt towards the bottom:

```json
{
  "type": "draft", "id": "F3", "neutral": {"kind": "face", "id": "F2.C0"},
  "faces": ["F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"], "angle": 5
}
```

Run result: no errors; 6 faces, volume 7485.2737 mm³.


An angle that turns an edge over:

```json
{
  "type": "draft", "id": "F3", "neutral": {"kind": "std", "id": "front"}, "faces": ["F2.S.e1"],
  "angle": 80
}
```

Run result: errors `F3`: `draftTooLarge` `{"detail":"edge between F2.S.e2 and F2.C1 turns over"}`; 6 faces, volume 8000 mm³.


## shell

Hollows the body to a wall thickness, removing the listed faces.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `faces` | list of face names | required | The faces to open. An empty list closes the cavity. A name that does not exist is `missingReference`. |
| `thickness` | number or expression | `0` | Wall thickness; must be above 0. |
| `outward` | boolean | `false` | Grow the wall outwards instead of inwards. |

Every wall face is doubled: the outer face and the face of the cavity share a name and are told apart as `~1` and `~2` (see [identifiers and references](references.md)). A body with several bodies in it shells only the body that holds the opened faces.

```json
{"type": "shell", "id": "F3", "faces": ["F2.C1"], "thickness": 2}
```

Run result: no errors; 11 faces, volume 3392 mm³. Volume 40 x 20 x 10 - 36 x 16 x 8 = 3392. Faces `F2.C0~1`, `F2.S.e1~1`, `F2.S.e2~2`, `F2.S.e3~2`, `F2.S.e4~1`, `F2.C1`, `F2.C0~2`, `F2.S.e1~2`, `F2.S.e2~1`, `F2.S.e3~1`, `F2.S.e4~2`.


```json
{"type": "shell", "id": "F3", "faces": ["F2.C1"], "thickness": 2, "outward": true}
```

Run result: no errors; 11 faces, volume 4672 mm³. The walls grow outwards: box [-2,-2,-2] to [42,22,10].


Errors carry numbers when the limit is known: `shellTooThick {detail, limit, minRadius, maxThickness, face}` where `limit` is `"curvature"` (the offset passes a centre of curvature) or `"thin"` (two sheets closer than twice the thickness) and `maxThickness` a thickness that works. Others: `nothingToCut`, `invalidValue`, `missingReference`, `shellUnsupported`, `shellFailed`, `offsetUnsupported` and `offsetTooLarge` (both with `detail`).

```json
{"type": "shell", "id": "F3", "faces": ["F2.C1"], "thickness": 12}
```

Run result: errors `F3`: `shellTooThick` `{"detail":"an edge reverses","limit":"thin","minRadius":19.5,"maxThickness":9.26}`; 6 faces, volume 8000 mm³.


## hole

Cuts holes at the `point` entities of a sketch, into the face the sketch lies on.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | A sketch with `point` entities. The holes go against the sketch normal (into the material of a sketch on a face). A sketch with no points is `holeNoPoints`. |
| `standard` | string | | Read and ignored: the only table is ISO. |
| `size` | string | none | `M2`, `M2.5`, `M3`, `M4`, `M5`, `M6`, `M8`, `M10`, `M12`, `M16`, `M20` or `M24`. |
| `fit` | string | `"normal"` | `normal`, `tap` (simple holes only), `close` or `loose`. |
| `kind` | string | `"simple"` | `simple`, `cbore` (counterbore), `countersink` (alias `csk`) or `csink` (the older spelling: a plain hole unless `cskDiameter` is given). |
| `diameter` | number or expression | none | A plain diameter instead of `size`/`fit` for a simple hole; also for the other kinds when the table does not apply (see below). |
| `end` | string | `"blind"` | `blind` uses `depth`; any other value is through the whole body. |
| `depth` | number or expression | `0` | Depth of a blind hole. A blind hole needs a depth above 0: with `depth` missing or 0 the cut has no volume and the feature fails with `booleanFailed` (`params.issue: "openEdge"`, `params.faces` the face the sketch lies on), a misleading code for a missing depth. |
| `tip` | boolean | `false` | A blind hole ends in a 118 degree drill-point cone. |
| `flip` | boolean | `false` | Cut along the sketch normal instead of against it. |
| `target` | selection or list | none | Drill only the named bodies (`{"body":"A"}`); see `target` in [solids](features-solids.md). Without it the holes cut every body they meet. |
| `cboreDiameter`, `cboreDepth` | number or expression | table | Counterbore size. Each is independent. |
| `cskDiameter`, `cskAngle` | number or expression | table, `90` | Countersink head diameter and included angle in degrees. |

**Many points.** One `hole` feature takes any number of points; the cost grows about in step with the number of holes. Each hole is a separate tool solid; a hole is united with another only when the two overlap or touch, and all the tools are then cut from the body in one step. (Earlier builds folded every hole into one growing union, which made the time grow with the square of the number of points: 10 x 10 holes took 7.6 s, 20 x 20 about 131 s and 30 x 30 about 748 s.) On the build machine a 30 x 30 grid of through holes in one feature (900 holes, 906 faces) now takes about 3.3 s, 20 x 20 about 1.2 s and 10 x 10 under 0.1 s. The faces, their names and the volume are the same as before. Above a few hundred points the default feature time limit (when a clock is present) can still trip on a slow machine; split the points over several `hole` features or raise the limit.

The hole diameter by size and fit, and the head sizes (mm):

| Size | `tap` | `close` | `normal` | `loose` | Counterbore diameter | Counterbore depth | Countersink head |
|---|---|---|---|---|---|---|---|
| M2 | 1.6 | 2.2 | 2.4 | 2.6 | 4.3 | 2.3 | 4.4 |
| M2.5 | 2.05 | 2.7 | 2.9 | 3.1 | 5.0 | 2.8 | 5.5 |
| M3 | 2.5 | 3.2 | 3.4 | 3.6 | 6.5 | 3.4 | 6.5 |
| M4 | 3.3 | 4.3 | 4.5 | 4.8 | 8.0 | 4.4 | 8.6 |
| M5 | 4.2 | 5.3 | 5.5 | 5.8 | 10.0 | 5.4 | 10.4 |
| M6 | 5.0 | 6.4 | 6.6 | 7.0 | 11.0 | 6.4 | 12.4 |
| M8 | 6.8 | 8.4 | 9.0 | 10.0 | 15.0 | 8.6 | 16.4 |
| M10 | 8.5 | 10.5 | 11.0 | 12.0 | 18.0 | 10.6 | 20.4 |
| M12 | 10.2 | 13.0 | 13.5 | 14.5 | 20.0 | 12.6 | 24.4 |
| M16 | 14.0 | 17.0 | 17.5 | 18.5 | 26.0 | 16.6 | 32.4 |
| M20 | 17.5 | 21.0 | 22.0 | 24.0 | 33.0 | 20.6 | 40.4 |
| M24 | 21.0 | 25.0 | 26.0 | 28.0 | 40.0 | 24.8 | 48.4 |

For counterbore and countersink holes the `tap` fit reads as `normal`. A countersink cone starts with the head diameter at the top face and tapers to the hole diameter; its depth is `(head - d) / 2 / tan(angle / 2)`.

A `diameter` also serves a counterbore or countersink when no table `size` applies, but then the step or cone must be given (`cboreDiameter` and `cboreDepth`, or `cskDiameter`). If one is missing the hole is drilled as a simple hole of `diameter` and the reply carries a warning `holeDowngraded {requested, built: "simple", missing, detail, hint}`.

Faces made by one hole (the top disc lies in the existing face and has no name), with `~k` appended per point when the sketch has several:

| Kind | Faces |
|---|---|
| `simple` | `h1` wall, `h2` bottom (the cone with `tip`). |
| `cbore` | `h1` counterbore wall, `h2` step floor, `h3` hole wall, `h4` bottom. |
| `countersink` | `K0` cone, `h1` wall, `h2` bottom. |

A `through` hole has no bottom face. All faces are written `<feature>.S.<tag>`, for example `F4.S.h1`.

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "normal",
    "kind": "simple", "end": "blind", "depth": 6
  }
]
```

Run result: no errors; 8 faces, volume 7794.7283 mm³. Diameter 6.6, depth 6: 205.2717 mm³ removed = pi 3.3² 6. Faces `F4.S.h2`, `F4.S.h1`.


Counterbore:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M5", "fit": "normal",
    "kind": "cbore", "end": "blind", "depth": 8
  }
]
```

Run result: no errors; 10 faces, volume 7514.1134 mm³. Faces `F4.S.h4`, `F4.S.h3`, `F4.S.h2`, `F4.S.h1`.


Countersink, through:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M5", "fit": "normal",
    "kind": "countersink", "end": "through"
  }
]
```

Run result: no errors; 8 faces, volume 7695.159 mm³. Faces `F4.S.K0`, `F4.S.h1`.


Two holes from one sketch:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}, {"id": "p2", "x": 30, "y": 10}],
    "entities": [{"type": "point", "id": "e1", "p": "p1"}, {"type": "point", "id": "e2", "p": "p2"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "tap",
    "kind": "simple", "end": "blind", "depth": 6
  }
]
```

Run result: no errors; 10 faces, volume 7764.3806 mm³. Faces `F4.S.h2~1`, `F4.S.h1~1`, `F4.S.h2~2`, `F4.S.h1~2`.


A plain diameter without the step dimensions downgrades a counterbore:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "kind": "cbore", "diameter": 7.5, "end": "blind",
    "depth": 6
  }
]
```

Run result: no errors; warnings `F4`: `holeDowngraded` `{"requested":"counterbore","built":"simple","missing":["cboreDiameter","cboreDepth"],"detail":"the counterbore has no cboreDiameter or cboreDepth to build it from, so a plain hole of the given diameter was drilled","hint`; 8 faces, volume 7734.9281 mm³.


An unknown size:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M7", "end": "blind",
    "depth": 6
  }
]
```

Run result: errors `F4`: `holeInvalid` `{"detail":"the simple hole has no size for the standard table","hint":"give a standard size (e.g. M6), or a plain diameter"}`; 6 faces, volume 8000 mm³.


Errors: `missingSketch`, `holeNoPoints`, `holeInvalid {detail, hint}` (no size and no diameter, an unknown size, a diameter of 0 or less), `holeCboreInvalid {hint, ...}` (counterbore diameter not larger than the hole, depth 0 or less, or not shallower than the hole), `holeCskDiameter {hint, ...}` (head diameter not larger than the hole, angle outside 0 to 180, cone deeper than the hole), `nothingToCut`, `cutNoEffect` (the hole misses the body or points away from it), `badExpression`.

## thread

Puts a thread on a cylindrical face: the wall of a hole or a shaft. By default it is cosmetic (data for drawings and checks); with `modeled` it cuts a helical groove.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `face` | face name | required | A cylindrical face, for example `F4.S.h1`. A missing face is `missingReference`; a face that is not a cylinder is `threadFaceNotCylinder`. |
| `standard` | string | required | `metric`, `metric-fine`, `UNC`, `UNF`, `BSP`, `BSPT`, `NPT`, `Tr` or `Acme` (aliases such as `ISO`, `G` are accepted). |
| `size` | string | required | A designation from the table of the standard, for example `M6`, `1/4-20`, `G1/4`, `R1/4`. The `threadData` request lists them. |
| `class` | string | `6H`, `6g`, `2B`, `2A`, ... | Tolerance class. Recorded; it does not change the geometry. |
| `length` | number, expression or `"full"` | `"full"` | Thread length in mm, clamped to the face. Zero or less is `invalidValue`. |
| `hand` | string | `"right"` | `right` or `left`. |
| `start` | string | the open end | `top` or `bottom`: the end of the face with the higher or lower coordinate along its axis. |
| `modeled` | boolean | `false` | Cut the real groove. |
| `clearance` | number or expression | `0` | Printed-fit clearance in mm, signed, at most 1 in size (more is `invalidValue`). Positive is looser, negative tighter. It changes the modeled groove only: an internal thread's groove reaches `clearance` further out, an external thread's goes `clearance` deeper (the thread's diameters move by twice that). A groove is never wider than 0.9 of the pitch, so an external thread wants the shaft drawn near the loosened major (nominal major minus twice the clearance); a shaft left at the nominal major is `threadModelFallback`. The `threads` entry carries `clearance` when it is not zero. |

Internal or external is read from the face (concave is internal). A cosmetic thread changes nothing in the body; the reply gains a `threads` list sorted by feature id, one entry per thread with `id`, `face`, `standard`, `size`, `class`, `pitch`, `major`, `minor`, `pitchDiameter`, `angle`, `length`, `hand`, `internal`, `modeled`, `tapered`, `diameter` (of the face), and `start` and `dir` (a point on the axis and the unit direction into the face).

A modeled thread cuts a trapezoidal groove (flank angle 60 for ISO and UN, 55 for BSP, 30 for Tr, 29 for Acme) with flat crests and roots. The groove faces are `T0` to `T5` (`T0` the inner or outer surface, `T1` and `T3` the flanks, `T2` root or bottom, `T4` and `T5` the end caps); those that lie in the void do not remain in the body. A thread that cannot be modeled stays cosmetic and the feature carries `threadModelFallback {reason, hint}`: tapered threads (BSPT, NPT), a groove of more than 60 turns, a length shorter than one turn, a bore or shaft that does not match the thread, or a failed boolean.

An internal thread wants the face diameter within 3 percent of the tap drill, an external one within 3 percent of the major diameter; otherwise the feature carries a warning `threadDiameterMismatch {expected, actual, size, standard, internal, nearest, hint}`. `BSP` is the parallel pipe thread G (ISO 228, G1/16 to G4); `BSPT` is the tapered pipe thread of ISO 7 (sizes `R1/16` to `R4`, and `Rp1/4` or `Rc1/4` find the same size). Both use the 55 degree form; a tapered size has a 1:16 taper on the diameter but is recorded as cosmetic data (`tapered: true`) and the real taper is not cut. Patterns and mirrors carry the threads of their seeds onto the copies (see [patterns and transforms](features-patterns.md)).

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "tap",
    "kind": "simple", "end": "through"
  },
  {"type": "thread", "id": "F5", "face": "F4.S.h1", "standard": "metric", "size": "M6"}
]
```

Run result: no errors; 7 faces, volume 7803.6505 mm³. The `threads` list of the reply: `[{"id":"F5","face":"F4.S.h1","standard":"metric","size":"M6","class":"6H","pitch":1,"major":6,"minor":4.917468,"pitchDiameter":5.350481,"angle":60,"length":10,"hand":"right","internal":true,"modeled":false,"tapered":false,"diameter":5,"start":[10,10,10],"dir":[0,0,-1]}]`.


```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "tap",
    "kind": "simple", "end": "through"
  },
  {"type": "thread", "id": "F5", "face": "F4.S.h1", "standard": "metric", "size": "M6", "modeled": true}
]
```

Run result: no errors; 10 faces, volume 7768.6672 mm³. Groove faces: `F5.T0`, `F5.T1`, `F5.T2`.


An unknown size lists the nearest ones:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "tap",
    "kind": "simple", "end": "through"
  },
  {"type": "thread", "id": "F5", "face": "F4.S.h1", "standard": "metric", "size": "M99"}
]
```

Run result: errors `F5`: `threadSizeUnknown` `{"standard":"metric","size":"M99","suggestions":["M39","M1","M2","M3","M4"],"hint":"\"M99\" is not a metric size; closest: M39, M1, M2, M3, M4 (request threadData for the list)"}`; 7 faces, volume 7803.6505 mm³.


A thread on a hole of the wrong size is built but flagged:

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "p1", "x": 10, "y": 10}], "entities": [{"type": "point", "id": "e1", "p": "p1"}],
    "constraints": []
  },
  {
    "type": "hole", "id": "F4", "sketch": "F3", "standard": "iso", "size": "M6", "fit": "tap",
    "kind": "simple", "end": "through"
  },
  {"type": "thread", "id": "F5", "face": "F4.S.h1", "standard": "metric", "size": "M8"}
]
```

Run result: no errors; warnings `F5`: `threadDiameterMismatch` `{"expected":6.8,"actual":5,"size":"M8","standard":"metric","internal":true,"nearest":"M6","hint":"hole is 5, M8 tap drill is 6.8: drill the hole to 6.8, or choose M6 (tap drill 5)"}`; 7 faces, volume 7803.6505 mm³.


Errors: `threadSizeUnknown {standard, size, suggestions, hint}`, `threadFaceNotCylinder {face, surface, hint}`, `missingReference`, `invalidValue {field}`.

## lip

Adds a raised band (a lip) along rim edges, or cuts a groove into the rim face.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `kind` | string | `"lip"` | `lip` raises the band out of the rim face; `groove` cuts it into the rim face. |
| `faces` | list of face names | required | The rim face. A name without `~k` selects every piece. |
| `edges` | list of edge picks | required | The rim edges to follow: `{"faces": [rim face, neighbour face], "at": [..]}`. |
| `width` | number or expression | `1` | Width of the band, measured inwards from the path. Must be above 0. |
| `height` | number or expression | `1` | Height out of (lip) or depth into (groove) the rim face. Must be above 0. |

The rim face must be planar. Faces: `C1` (the top of a lip) or `C0` (the floor of a groove), and `S.<neighbour face>l` for the inner wall of the band along each edge. The outer side of the band merges with the existing wall.

On the plate shelled with 2 mm walls (the faces of the shell keep the names `F2.S.e1~1` for the outer walls):

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 40, "y": 0},
        {"id": "c", "x": 40, "y": 20},
        {"id": "d", "x": 0, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10},
    {"type": "shell", "id": "F3", "faces": ["F2.C1"], "thickness": 2}
  ]
}
```


Four rim picks, appended to the shelled document above:

```json
{
  "type": "lip", "id": "F4", "kind": "lip", "faces": ["F2.C1"],
  "edges": [
    {"faces": ["F2.C1", "F2.S.e1~1"], "at": [20, 0, 10]},
    {"faces": ["F2.C1", "F2.S.e2~2"], "at": [40, 10, 10]},
    {"faces": ["F2.C1", "F2.S.e3~2"], "at": [20, 20, 10]},
    {"faces": ["F2.C1", "F2.S.e4~1"], "at": [0, 10, 10]}
  ],
  "width": 1, "height": 1.5
}
```

Run result: no errors; 16 faces, volume 3566 mm³. Volume 3566 (3392 + the 1 mm wide, 1.5 mm high band). Faces `F4.C1`, `F4.S.F2.S.e4l`, `F4.S.F2.S.e3l`, `F4.S.F2.S.e2l`, `F4.S.F2.S.e1l`.


```json
{
  "type": "lip", "id": "F4", "kind": "groove", "faces": ["F2.C1"],
  "edges": [
    {"faces": ["F2.C1", "F2.S.e1~1"], "at": [20, 0, 10]},
    {"faces": ["F2.C1", "F2.S.e2~2"], "at": [40, 10, 10]},
    {"faces": ["F2.C1", "F2.S.e3~2"], "at": [20, 20, 10]},
    {"faces": ["F2.C1", "F2.S.e4~1"], "at": [0, 10, 10]}
  ],
  "width": 1, "height": 1.5
}
```

Run result: no errors; 16 faces, volume 3218 mm³. Volume 3218. Faces `F4.C0`, `F4.S.F2.S.e4l`, `F4.S.F2.S.e3l`, `F4.S.F2.S.e2l`, `F4.S.F2.S.e1l`.


Errors: `nothingToCut` (no body), `lipFace` (no rim face), `invalidValue` (width or height), `missingReference` (an edge pick), `booleanFailed`. A rim that is not planar, or several rim faces in different planes, is not built: the code is `TODO(rebuild): lip on a curved / several rim faces needs ck-ops`.

## moveFace

Pushes or pulls faces; their neighbours follow.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `faces` | list of face names | required | Faces of the solid. A name without `~k` selects every piece. An empty list is `noFaces`. |
| `mode` | string | `"offset"` | `offset`: each face moves along its outward normal by `distance` (positive adds material: a pushed plane grows the part, a cylinder gets a larger radius, a hole wall shrinks the hole). `translate`: each planar face moves by the component of `translate` along its outward normal. |
| `distance` | number or expression | `0` | Offset distance. 0 is a no-op. |
| `translate` | `[x, y, z]` | none | Translation for `translate` mode. |
| `flip` | boolean | `false` | Negates `distance`. |

Planes, cylinders, cones, spheres, tori and extruded walls are supported in `offset` mode; `translate` is for planar faces. The unselected neighbours keep their surfaces and are extended or trimmed to meet the moved faces. Faces keep their names, so later references still resolve. Fillets that are tangent to a pushed plane travel with it.

```json
{"type": "moveFace", "id": "F3", "faces": ["F2.C1"], "mode": "offset", "distance": 2.5}
```

Run result: no errors; 6 faces, volume 10000 mm³. The plate is now 12.5 mm high; the face names are unchanged: `F2.C0`, `F2.C1`, `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`.


```json
{"type": "moveFace", "id": "F3", "faces": ["F2.S.e2"], "mode": "translate", "translate": [3, 0, 0]}
```

Run result: no errors; 6 faces, volume 8600 mm³. The right side moves out by 3: box [0,0,0] to [43,20,10].


Errors: `nothingToCut`, `noFaces`, `missingReference {ref}`, `invalidValue {detail}` (bad mode, `translate` missing, non-finite), `moveFaceUnsupported {detail}` (a free-form face, a curved face in `translate` mode, a face of a surface body) and `moveFaceFailed {detail}` (an edge collapses or reverses, the result is not valid, the volume goes the wrong way). Not supported: translating a cylinder sideways (moving a hole), and moving a face past other geometry is not checked against unrelated parts of the body.

```json
{"type": "moveFace", "id": "F3", "faces": ["F2.C1"], "distance": -12}
```

Run result: errors `F3`: `moveFaceFailed` `{"detail":"an edge reverses"}`; 6 faces, volume 8000 mm³.


## deleteBody

Removes whole bodies.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `bodies` | list of selectors | required | `{"body": "Block"}` (every body with that name), `{"face": "F4.C1"}` (the body that holds the face) or `{"surface": "F7"}` (a surface body made by feature F7). An empty list is `noBodies`. |
| `mode` | string | `"delete"` | `delete` removes the selected bodies; `keep` removes all the others. |

Errors: `noBodies`, `missingReference` (a selector matches nothing; a `{body}` selector adds `params.name` and `params.nearest`, up to three existing names).

The plate with a second body `Block` made with `newBody`; the two examples below are appended to this document:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 40, "y": 0},
        {"id": "c", "x": 40, "y": 20},
        {"id": "d", "x": 0, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10},
    {
      "type": "sketch", "id": "F3", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 60, "y": 0},
        {"id": "b", "x": 80, "y": 0},
        {"id": "c", "x": 80, "y": 20},
        {"id": "d", "x": 60, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {
      "type": "extrude", "id": "F4", "op": "newBody", "body": "Block", "sketch": "F3", "end": "blind",
      "depth": 5
    }
  ]
}
```


```json
{"type": "deleteBody", "id": "F5", "mode": "delete", "bodies": [{"body": "Block"}]}
```

Run result: no errors; 6 faces, volume 8000 mm³. Only the plate is left.


```json
{"type": "deleteBody", "id": "F5", "mode": "keep", "bodies": [{"face": "F4.C1"}]}
```

Run result: no errors; 6 faces, volume 2000 mm³. Only the block is left: box [60,0,0] to [80,20,5].


## See also

- [Solids from sketches](features-solids.md)
- [Patterns and transforms](features-patterns.md)
- [Identifiers and references](references.md)
- [Feature errors and diagnostics](../validation/diagnostics.md)
- [Topology and geometry checks](../validation/checks.md)
