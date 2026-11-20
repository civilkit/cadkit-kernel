# Surfaces and curves

Besides solids, a part can hold sheets (surface bodies) and curves. This chapter covers the features that make sheets (`surfacePlanar`, `surfaceExtrude`, `surfaceRevolve`, and on procedural surfaces `sweepSurface`, `blendSurface`), combine and change them (`knit`, `trimSurface`, `fill`, `thicken`, `extend`, `deleteFace`, `freeform`, `pipeCut`), the features that make curves (`projectCurve`, `editCurve`, `offsetCurve`, `compositeCurve`; `helix` is in [solids from sketches](features-solids.md)) and 3D sketches (`sketch3d`). A `loft` with `"surface": true` also makes a sheet.

## Sheets in the reply

A sheet is a body whose faces are open. The regen reply keeps sheets apart from the solid: `body` holds the solid, and `surfaces` is a list of `[feature id, body]` pairs, one per sheet. A feature that consumes sheets removes them from the list. A sheet that turns out to be a closed shell becomes the solid body (`knit` with `solid: true`, `fill` with `merge: true` that closes a shell).

Face references work across the solid and all sheets: a name such as `F2.S.e1` is looked up in whichever body holds it. A feature that takes a sheet accepts the id of the feature that made it, or the name of any of its faces.

## surfacePlanar

A flat sheet bounded by a sketch profile.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | The sketch. `missingSketch` if it is not found. |

The sheet has one face, `P`, with the normal of the sketch plane and the edges of the profile.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F2", "sketch": "S1"}
  ]
}
```

Run result: no errors; no solid body. Surfaces: `F2`: `F2.P`.


## surfaceExtrude

A sheet swept from a profile, without caps.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | Closed regions or open chains. An open chain is closed by a chord for the sweep and the chord's face is dropped. A missing sketch is `noClosedProfile`. |
| `depth` | number or expression | `10` | Length in mm. |
| `end` | string | `"blind"` | `mid` centres the sheet on the sketch plane; any other value is blind. |
| `flip` | boolean | `false` | Sweep against the sketch normal. |

Faces: `S.<entity>`, one per profile entity. Errors: `noClosedProfile`, `badExpression`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"}
      ],
      "constraints": []
    },
    {"type": "surfaceExtrude", "id": "F2", "sketch": "S1", "end": "blind", "depth": 5}
  ]
}
```

Run result: no errors; no solid body. The reply has no `body`; `surfaces` holds `F2`: `F2.S.e1`, `F2.S.e2`. `F2` spans [0,0,0] to [20,10,5].


## surfaceRevolve

A sheet revolved about an axis.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | The profile (regions or open chains). |
| `axis` | string or object | required | A line entity of the sketch by id, or `{"sketch", "id"}` for a line of another sketch, as for `revolve`. |
| `angle` | number or expression | `360` | Degrees. |
| `end`, `flip` | | | As for `revolve`. |

Faces: `S.<entity>`. Errors: `missingSketch`, `missingAxis`, `profileCrossesAxis`, `noClosedProfile`, `invalidValue`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "o", "x": 0, "y": 0},
        {"id": "p", "x": 0, "y": 20},
        {"id": "a", "x": 10, "y": 0},
        {"id": "b", "x": 10, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "ax", "p1": "o", "p2": "p", "construction": true},
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"}
      ],
      "constraints": []
    },
    {"type": "surfaceRevolve", "id": "F2", "sketch": "S1", "axis": "ax", "angle": 360}
  ]
}
```

Run result: no errors; no solid body. A cylindrical sheet: `F2`: `F2.S.e1`.


## knit

Joins sheets along their shared edges.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `surfaces` | list of ids | required | Sheets to join (feature ids or face names). Entries that do not resolve are skipped; if none does the feature is `missingReference`. |
| `solid` | boolean | `false` | Require a closed shell and make a solid of it. |

The inputs are consumed. Without `solid` the result is one sheet registered under the id of the knit; with `solid` it becomes the solid body (united with an existing body). The faces keep their names. Errors: `missingReference`, `knitNotClosed {open, points}` (with `solid`, the shell has `open` open edges; `points` lists their midpoints), `knitNearMiss`.

Three sheets that make a box:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfaceExtrude", "id": "F2", "sketch": "S1", "end": "blind", "depth": 5},
    {"type": "surfacePlanar", "id": "F3", "sketch": "S1"},
    {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 5},
    {
      "type": "sketch", "id": "S2", "plane": {"kind": "feature", "id": "P1"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F5", "sketch": "S2"}
  ]
}
```

Run result: no errors; no solid body. Surfaces: `F2`: `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`; `F3`: `F3.P`; `F5`: `F5.P`.


```json
{"type": "knit", "id": "F6", "surfaces": ["F2", "F3", "F5"], "solid": true}
```

Run result: no errors; 6 faces, volume 1000 mm³. The three sheets are consumed and the closed shell becomes the solid body, keeping the face names: `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`, `F3.P`, `F5.P`.


Leaving out the top leaves four open edges:

```json
{"type": "knit", "id": "F6", "surfaces": ["F2", "F3"], "solid": true}
```

Run result: errors `F6`: `knitNotClosed` `{"open":4,"points":[[10,0,5],[20,5,5],[10,10,5],[0,5,5]]}`; no solid body.


## fill

Patches a hole bounded by a chain of edges.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `edges` | list of edge picks | required | `{"faces": [face], "at": [x, y, z]}`: with one face, the open boundary edge of that face nearest `at`; with two faces, the edge they share. All edges must be in one body and form a closed chain. |
| `merge` | boolean | `false` | Add the patch to the body it borders. A result that closes a shell becomes the solid body. Without it the patch is a sheet of its own. |
| `continuity` | string | `"G0"` | `G0`, `G1`, `G2` or `G3`. On a planar boundary the patch is a plane whatever the value. |
| `fairing` | string | `"meowNURBS"` | `meowNURBS`, `plate`, `curvature` or `variation`. |
| `guideStrength` | number | `0.5` | Strength of the fairing guide. |
| `patch` | string | `"single"` | `single` or `nSided`. `nSided` fills a closed ring of at least 3 edges with N bicubic faces round a centre vertex instead of one planar or ruled face (`continuity` is then `G0` or `G1`; `fairing` and `guideStrength` are not used). |
| `patchTolerance` | number | `1e-4` | `nSided` only: the largest distance of a patch side from the edge it stands for, in mm. |

The patch is one face, `F`. Errors: `missingReference`, `fillNotClosed {detail}` (the chain is open), `fillInvalid` (the edges do not form one chain), `fillFailed {detail}`.

With `patch: "nSided"` each ring edge is split at its parameter midpoint, N inner seam edges run from those midpoints to a new centre vertex, and the faces are `F5.N0` to `F5.N<N-1>`, numbered along the ring in the order of the picks. With `G1` each edge that borders a face leaves tangent to it. Extra errors: `fillContinuity` (`G2` and `G3` are refused), `fillInvalid` for a tolerance that is not positive, a ring of fewer than 3 edges or an unknown `patch`, `fillFailed {detail}` when the patches cannot meet the boundary within `patchTolerance`.

An open box (walls and floor knitted into a sheet `F4`), then the four rim edges filled:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfaceExtrude", "id": "F2", "sketch": "S1", "end": "blind", "depth": 5},
    {"type": "surfacePlanar", "id": "F3", "sketch": "S1"},
    {"type": "knit", "id": "F4", "surfaces": ["F2", "F3"]}
  ]
}
```

Run result: no errors; no solid body. One open shell `F4` (the walls and the floor). `F4`: `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`, `F3.P`.


```json
{
  "type": "fill", "id": "F5",
  "edges": [
    {"faces": ["F2.S.e1"], "at": [10, 0, 5]},
    {"faces": ["F2.S.e2"], "at": [20, 5, 5]},
    {"faces": ["F2.S.e3"], "at": [10, 10, 5]},
    {"faces": ["F2.S.e4"], "at": [0, 5, 5]}
  ],
  "merge": true, "continuity": "G0"
}
```

Run result: no errors; 6 faces, volume 1000 mm³. The patch `F5.F` closes the shell, so the result is a solid of volume 1000: `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`, `F3.P`, `F5.F`.


```json
{
  "type": "fill", "id": "F5",
  "edges": [
    {"faces": ["F2.S.e1"], "at": [10, 0, 5]},
    {"faces": ["F2.S.e2"], "at": [20, 5, 5]},
    {"faces": ["F2.S.e3"], "at": [10, 10, 5]},
    {"faces": ["F2.S.e4"], "at": [0, 5, 5]}
  ],
  "merge": false
}
```

Run result: no errors; no solid body. Without `merge` the patch is a sheet of its own: `F4`: `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`, `F3.P`; `F5`: `F5.F`.


## trimSurface

Cuts sheets with another sheet or a plane, keeping one side.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `surface` | id | | The sheet to trim (feature id or face name). |
| `surfaces` | list of ids | | Several sheets; used instead of `surface` when present. |
| `toolSurface` | id | | A sheet that cuts. |
| `plane` | plane reference | | A plane that cuts, used when there is no `toolSurface`. |
| `flip` | boolean | `false` | Keep the other side. The kept side is the one the tool normal points to. |
| `mutual` | boolean | `false` | Return the tool, uncut, in the same result. |
| `deleteTool` | boolean | `false` | Remove the tool sheet. An absent field keeps the tool; only an explicit `true` removes it. |

The result is one sheet registered under the id of the trim; the kept piece keeps the face names. The trim is built for planar sheets cut by a plane or by another planar sheet. A target that is curved, or a tool that does not cross the target, is not trimmed. Errors: `missingReference`, `trimNoIntersection {detail}`, `trimNothingLeft`, `trimPieceLoop`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "right"},
      "offset": 10
    },
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F3", "sketch": "S1"},
    {"type": "trimSurface", "id": "F4", "surface": "F3", "plane": {"kind": "feature", "id": "P1"}}
  ]
}
```

Run result: no errors; no solid body. The tool plane is x = 10 and the sheet keeps the side its normal points to: `F4` spans [10,0,0] to [20,10,0].


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "right"},
      "offset": 10
    },
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F3", "sketch": "S1"},
    {
      "type": "trimSurface", "id": "F4", "surface": "F3", "plane": {"kind": "feature", "id": "P1"},
      "flip": true
    }
  ]
}
```

Run result: no errors; no solid body. `F4` spans [0,0,0] to [10,10,0].


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "right"},
      "offset": 30
    },
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F3", "sketch": "S1"},
    {"type": "trimSurface", "id": "F4", "surface": "F3", "plane": {"kind": "feature", "id": "P1"}}
  ]
}
```

Run result: errors `F4`: `trimNoIntersection` `{"detail":"the tool does not cut the surface"}`; no solid body.


## thicken

Gives a sheet thickness, making a solid.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `surface` | id | required | The sheet. `missingReference` if it is not found. |
| `thickness` | number or expression | `2` | Thickness in mm. |
| `side` | string | `"front"` | `front` (along the face normal), `back` or `both` (half each way). |
| `thicknessAt` | object | none | A taper instead of `thickness`: `{"along": [x, y, z], "stops": [[s, t], ...]}` with two to six stops, `s` ascending and each `t` above 0. The thickness at a point is interpolated along the direction `along`. |

The sheet is consumed and the solid is united with the body. Faces: `W0`, `W1`, ... for the walls along the sheet's boundary; the sheet's own faces keep their name with `~1` (the side that was thickened) and `~2` (the offset side). Errors: `missingReference`, `thickenFailed`, `thickenTooThick`, `thickenReach`, `offsetTooLarge {limit, ...}`, `invalidValue {detail, hint}` (bad `thicknessAt`). Offsets of curved sheets that would pass a centre of curvature carry the `minRadius` and `maxThickness` fields of `shellTooThick`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F2", "sketch": "S1"},
    {"type": "thicken", "id": "F3", "surface": "F2", "thickness": 2, "side": "front"}
  ]
}
```

Run result: no errors; 6 faces, volume 400 mm³. The sheet is consumed; the slab has the faces `F2.P~1`, `F2.P~2`, `F3.W0`, `F3.W1`, `F3.W2`, `F3.W3` and volume 400.


## extend

Extends a sheet past one or more boundary edges.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `edges` | list of edge picks | required | Boundary edges of a sheet (`{"faces": [face], "at": [..]}`). An edge of a solid is `missingReference`. |
| `distance` | number or expression | `5` | How far to extend. |
| `end` | string | `"distance"` | `distance`, or `surface` with `target: {"id": ...}` to extend up to another sheet. |
| `shape` | string | `"same"` | Only `same` (continue the surface) is built; other values fail with `TODO(rebuild): extend shape other than same needs ck-ops`. |

The faces keep their names. Errors: `missingReference`, `extendNotBoundary`, `extendNoTarget`, `extendPointBehind`, `extendFolds`, `extendUnsupported`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfacePlanar", "id": "F2", "sketch": "S1"},
    {"type": "extend", "id": "F3", "edges": [{"faces": ["F2.P"], "at": [10, 0, 0]}], "distance": 3}
  ]
}
```

Run result: no errors; no solid body. The sheet grows 3 mm at its bottom edge: `F3` spans [0,-3,0] to [20,10,0].


## deleteFace

Removes faces from a solid or sheet. A solid that loses a face becomes a sheet.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `faces` | list of face names | required | Faces to remove. An empty or unresolved list is `missingReference`. |
| `mode` | string | `"delete"` | Any other value fails with `TODO(rebuild): deleteFace fill/heal needs ck-ops`. |

The remaining faces keep their names.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 20, "y": 0},
        {"id": "c", "x": 20, "y": 10},
        {"id": "d", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "surfaceExtrude", "id": "F2", "sketch": "S1", "end": "blind", "depth": 5},
    {"type": "deleteFace", "id": "F3", "faces": ["F2.S.e1"]}
  ]
}
```

Run result: no errors; no solid body. `F3`: `F2.S.e2`, `F2.S.e3`, `F2.S.e4`.


## freeform

Re-fits faces as B-spline surfaces and applies control-point edits.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `faces` | list of face names | required | The faces to rebuild. |
| `degreeU`, `degreeV` | number | `3` | Degrees of the new surface. |
| `spansU`, `spansV` | number | `2` | Spans in each direction (at most 50: `rebuildTooFine`). |
| `edits` | list of objects | none | Control-point edits; they work on exactly one face (`freeformOneFace`). |

The faces keep their names. The reply gains a marker for the feature with the control net of each face. Closed (periodic) faces such as a full cylinder wall are refused with `rebuildPeriodicFace`. The edits are not described further here.

## pipeCut

Rolls a ball along picked edges, or around the rim of picked faces, and cuts away everything it touches. The body becomes a sheet.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `radius` | number or expression | `0` | Ball radius; must be above 0 (`invalidValue`). |
| `edges` | list of edge picks | none | Edges to follow. |
| `faces` | list of face names | none | Faces whose rim edges are followed. |

The picks may sit in the solid or in a sheet; the result replaces that body and is registered under the feature id; the faces that remain keep their names. Errors: `invalidValue`, `missingReference`, `alignPipeCutOneBody` (picks in more than one body), `pipeCutRadius {detail, max}`, `pipeCutNothingLeft`, `pipeCutFailed`.

Appended to the plate of [the overview](overview.md):

```json
{
  "type": "pipeCut", "id": "F3", "radius": 1,
  "edges": [{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}]
}
```

Run result: no errors; no solid body. The solid is replaced by a surface body registered under `F3` (6 faces).


## projectCurve

Projects a sketch, or a planar curve, perpendicular to its plane onto faces and fits one B-spline through the result.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | | The sketch to project (its non-construction curves). |
| `curve` | feature id | | A curve feature (`editCurve`, `projectCurve`) to project instead. It wins when both are given. |
| `faces` | list of face names | all faces | Faces of the solid or of sheets that receive the curve. Empty or missing means every face. |
| `degree` | number | `3` | B-spline degree, 2 to 9. |
| `tolerance` | number | `0.001` | Largest distance of the fit from the projected data (mm), above 0. More control points (up to 400) are used until it is met. |
| `direction` | string | `"both"` | `both`, `forward` (along the sketch normal) or `reverse`. |

Every sample of the curve is cast along the normal against the chosen faces and all crossings inside the trimmed faces are kept. The pieces must form one connected chain; several chains are `projectCurveSeveral {count, points}`. It makes a curve and no faces: the faces it is projected on are not split. The result appears in `curves` as `[id, {pts, axis: false, t0: 0, t1: 1}]` and in `markers` as `{degree, count, deviation, length, pieces, corners, closed}`.

Errors, in the order they are checked: `projectCurveNoFaces` (no solid and no sheet), `projectCurveNoInput` (neither `sketch` nor `curve`, or a sketch with nothing to project), `missingReference` (an unresolved `curve` or face), `projectCurvePlane` (a `curve` that is not planar), `missingSketch`, `editCurveDegree {min, max}`, `invalidValue` (tolerance or `direction`), `badExpression`, `projectCurveMisses`, `projectCurveSeveral`, `projectCurveFailed`.

A circle on a plane 20 mm above the plate (`F3`), projected onto the top face only. Appended to the plate of [the overview](overview.md):

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 20},
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "feature", "id": "P1"},
    "points": [{"id": "c", "x": 20, "y": 10}],
    "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 6}], "constraints": []
  },
  {"type": "projectCurve", "id": "F4", "sketch": "F3", "faces": ["F2.C1"]}
]
```

Run result: no errors; 6 faces, volume 8000 mm³. `curves` holds `F4` (121 points) and `markers` the fit: degree 3, 15 control points, deviation 0.000705, closed true.


Without `faces` the circle also hits the bottom face, which makes two separate chains (same document, `faces` left out):

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 20},
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "feature", "id": "P1"},
    "points": [{"id": "c", "x": 20, "y": 10}],
    "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 6}], "constraints": []
  },
  {"type": "projectCurve", "id": "F4", "sketch": "F3"}
]
```

Run result: errors `F4`: `projectCurveSeveral` `{"count":2,"points":[]}`; 6 faces, volume 8000 mm³.

### Procedural projection

With `"procedural": true` the projection is kept as its definition (a projected curve: the foot of every point of the input on the surface) instead of a fitted B-spline, onto exactly one face. Without the option, or with `false`, everything above holds unchanged.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `procedural` | boolean | `false` | Keep the projection as a procedural curve. |
| `curves`, `curve`, `sketch` | | | The input, read as for [`offsetCurve`](#offsetcurve): curve references in chain order, else a curve feature, else a sketch whose curves form one chain. |
| `faces` | list of one face name | required | The face to project onto. None or several is `invalidValue` (`field: "faces"`). |
| `along` | `"normal"` or `[x, y, z]` | plane normal | `"normal"` projects to the closest point of the surface; a direction projects along it. Without it: the normal of the sketch plane, or of the plane of the curve; a curve in no plane goes to the closest point. |
| `tolerance` | number | `0.001` | Bound of the B-spline the definition converts to (mm). |
| `gapTolerance` | number | `1e-5` | Largest gap between consecutive input pieces (mm). |

`degree` and `direction` are not read. The projection must exist along the whole input and stay inside the trimmed face, else `projectCurveMisses {face, at, detail, hint}`. The result is published like the other procedural curves (see [`offsetCurve`](#offsetcurve)) with `kind: "projected"`, and `face`, `mode` (`normal` or `direction`) and `along` in the marker.

Errors, in order: `projectCurveNoFaces`, `invalidValue` (`gapTolerance`), `projectCurveNoInput`, `missingReference` (input), `missingSketch`, `projectCurveGap`, `projectCurveNotChain`, `invalidValue` (`tolerance`, `faces`), `missingReference` (the face), `invalidValue` (`along`), `projectCurveMisses`, `projectCurveFailed`.

A circle 20 mm above the plate projected straight down onto its top face, kept as a projected curve (appended to the plate of [the overview](overview.md)):

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 20},
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "feature", "id": "P1"},
    "points": [{"id": "c", "x": 20, "y": 10}],
    "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 6}], "constraints": []
  },
  {"type": "projectCurve", "id": "F4", "procedural": true, "sketch": "F3", "faces": ["F2.C1"]}
]
```

Run result: no errors. The marker of `F4`: kind `projected`, mode `direction`, length 37.699 (12π), closed true; every point of the curve is at z = 10.


## offsetCurve

Offsets a curve, or a chain of curves, by a distance in a plane or along a face, and keeps the offset as its definition (an offset curve over the input).

| Field | Type | Default | Meaning |
|---|---|---|---|
| `curves` | list of curve references | | `{"kind": "sketchCurve", "sketch", "entity"}`, `{"kind": "edge", "faces", "at"}` or `{"kind": "curve", "id"}`, in chain order. Each piece is turned to start where the previous one ends. Wins over `curve` and `sketch`. |
| `curve` | feature id | | An earlier curve feature (`helix`, `editCurve`, `projectCurve`, `offsetCurve`, `compositeCurve`). |
| `sketch` | feature id | | A sketch: its non-construction curves, walked into one chain from a free end. |
| `distance` | number | required | Offset distance (mm), not 0. Positive moves along unit(C′ × N): to the right of the direction of travel seen from the side N points to. |
| `normal` | `[x, y, z]` | | N, the normal of the offset plane. |
| `plane` | plane reference | | N from a standard plane, plane feature or planar face. |
| `face` | face name | | Offset in the tangent planes of this face instead (N its outward normal at each point): a curve on a curved face stays on it to first order. |
| `tolerance` | number | `0.001` | Bound of the B-spline the definition converts to (mm). |
| `gapTolerance` | number | `1e-5` | Largest gap between consecutive input pieces (mm). |

Without `normal`, `plane` or `face`, N is the normal of the sketch plane (a `sketch` input) or of the plane the curve lies in. A sketch line drawn from left to right with N = +z therefore moves down (−y) for a positive distance, and a counter-clockwise circle grows.

The offset of lines and circle arcs in their own plane is exact (lines and arcs again); anything else stays an offset curve, evaluated from its definition. The input is read exactly: sketch lines, arcs, circles, ellipses and control-point splines as they are, a through-point spline and a curve feature that is not procedural (`editCurve`, a plain `projectCurve`) as a cubic B-spline through it, a helix from its formula.

Refusals (the ways an offset fails, each with `detail`, `at` and a `hint`):

- `offsetCurveCorner {index, at}`: the input has a tangent discontinuity (a corner between two pieces, at the closing joint of a closed chain, or inside a B-spline), so its offset would have a gap.
- `offsetCurveFold {distance, at, radius}`: the distance passes the radius of curvature (the offset reverses), or the offset of a plane curve runs closer than the distance to another part of the curve (it intersects itself). `radius` is the radius of curvature where it reverses.
- `offsetCurveDirection {at}`: no offset direction: a zero tangent, a tangent along N, or N zero.
- `offsetCurvePlane`: no `normal`, `plane` or `face`, and the curve does not lie in one plane (or is a straight line).

The result appears in `curves` as `[id, {pts, axis: false, t0: 0, t1: 1, procedural: "offset"}]`, with `pts` evaluated on the definition (each line by its ends, every other piece by 128 points). The marker is `{kind, length, closed, pieces, corners, continuity, degree, count, deviation, distance, normal | face}`: `continuity` is the weakest joint (`C0`, `G1`, `C1`, `G2`, `C2`); `degree`, `count` and `deviation` describe the B-spline the definition converts to (deviation 0 when it is exact). A definition that cannot be converted within `tolerance` is `offsetCurveFailed {detail, tolerance}`.

Every feature that takes a curve feature (`{"kind": "curve", "id"}`: the `sweep` and `pipe` path, sweep guides, `curvepattern`, `plane` mode `curve`, `editCurve`, `projectCurve`) takes the result like any other curve; a path made only of lines and arcs is swept along those pieces exactly.

Errors, in order: `invalidValue` (`gapTolerance`), `offsetCurveNoInput`, `missingReference {curve, index}`, `missingSketch`, `offsetCurveGap`, `offsetCurveNotChain`, `invalidValue` (`tolerance`, `distance`), `badExpression`, `offsetCurveCorner`, `missingReference` (`face`, `plane`), `invalidValue` (`normal`), `offsetCurvePlane`, `offsetCurveDirection`, `offsetCurveFold`, `offsetCurveFailed`.

A line and a quarter arc of radius 10, offset 5 toward the centre of the arc, and a tube swept along the offset:

```json
[
  {
    "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
    "points": [{"id": "a", "x": 0, "y": -5}, {"id": "b", "x": 20, "y": -5}, {"id": "c", "x": 20, "y": 5}, {"id": "d", "x": 30, "y": 5}],
    "entities": [{"type": "line", "id": "e1", "p1": "a", "p2": "b"}, {"type": "arc", "id": "e2", "c": "c", "p1": "b", "p2": "d"}],
    "constraints": []
  },
  {"type": "offsetCurve", "id": "O1", "sketch": "S1", "distance": -5},
  {
    "type": "sketch", "id": "S9", "plane": {"kind": "std", "id": "right"},
    "points": [{"id": "c", "x": 0, "y": 0}], "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 1}], "constraints": []
  },
  {"type": "sweep", "id": "W1", "op": "add", "sketch": "S9", "path": {"kind": "curve", "id": "O1"}}
]
```

Run result: no errors. The marker of `O1`: 2 pieces, length 27.854 (20 + 2.5π), deviation 0 (a line and an arc of radius 5); the tube has volume π · 27.854 = 87.51 mm³. A distance of −12 is `offsetCurveFold` with `radius` 10.


## compositeCurve

Joins curves end to end into one curve and keeps it as a composite (the pieces, each with its own parameter, and the continuity of every joint).

| Field | Type | Default | Meaning |
|---|---|---|---|
| `curves`, `curve`, `sketch` | | | The input, as for [`offsetCurve`](#offsetcurve). |
| `gapTolerance` | number | `1e-5` | Largest distance between the end of a piece and the start of the next (mm). |
| `tolerance` | number | `0.001` | Bound of the B-spline the definition converts to (mm). |

The pieces are taken in the order listed, each turned to run on from the previous one. A gap above `gapTolerance` is `compositeCurveGap {index, gap, tolerance, at}` (`index` the piece after the gap); a sketch whose curves do not form one chain is `compositeCurveNotChain {count, left, points}`. Lines, arcs, circles, ellipses and B-splines convert exactly (deviation 0). The result is published as for `offsetCurve`, with `kind: "composite"`; `corners` counts the joints that are not tangent continuous.

Errors, in order: `invalidValue` (`gapTolerance`), `compositeCurveNoInput`, `missingReference {curve, index}` (or the face of an edge reference), `missingSketch`, `compositeCurveGap`, `compositeCurveNotChain`, `invalidValue` (`tolerance`), `compositeCurveFailed`.

Two edges of the top face of the plate, joined (appended to the plate of [the overview](overview.md)):

```json
{
  "type": "compositeCurve", "id": "C1",
  "curves": [
    {"kind": "edge", "faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]},
    {"kind": "edge", "faces": ["F2.C1", "F2.S.e2"], "at": [40, 10, 10]}
  ]
}
```

Run result: no errors. The marker of `C1`: 2 pieces, length 60, 1 corner, continuity `C0`.


## sweepSurface

A sheet whose one face is a profile swept along a path, kept as its definition (a procedural sweep surface: S(u, v) = path(u) + M(u) · L(v), L the profile in the frame at the path start, so the face starts exactly on the profile).

| Field | Type | Default | Meaning |
|---|---|---|---|
| `profile` | curve input | required | A string (an earlier curve feature, else a sketch whose non-construction curves form one chain), one curve reference (`{"kind": "sketchCurve", "sketch", "entity"}`, `{"kind": "edge", "faces", "at"}`, `{"kind": "curve", "id"}`) or a list of them in chain order. Several pieces are joined into one composite. |
| `path` | curve input | required | As `profile`. A procedural curve feature (`offsetCurve`, `compositeCurve`, procedural `projectCurve`) is swept along its definition. |
| `frame` | string | `"tangent"` | `tangent`: the profile turns with the path (x = the path tangent, y = unit(up × x), z = x × y). `translate`: the profile is carried without turning. |
| `up` | `[x, y, z]` | see below | Tangent frame only. Never along the path tangent. |
| `caps` | boolean | `false` | Close a closed planar profile on an open path with plane faces at both ends. |
| `tolerance` | number | `0.001` | Bound of the B-spline the definition converts to (mm). |
| `gapTolerance` | number | `1e-5` | Largest gap between consecutive pieces of an input chain (mm). |

Without `up`, the tangent frame takes the normal of the plane the path lies in (the frame then turns only about it), the world axis furthest from a straight path, and +z for any other path.

The sheet is one face `<id>.W` over the whole parameter rectangle: its edges are the profile at the path start, the profile moved rigidly to the path end, and the two rails (the path translated for a `translate` sweep, the exact iso lines of the definition otherwise). A closed profile or a closed path gives a seam instead of two sides; both closed (a circle round a circle) give a closed sheet. With `caps` the end faces are `<id>.C0` (start) and `<id>.C1` (end) and the sheet is a closed shell with outward normals; caps on a closed path are not needed and are not added. The sheet stays a surface body (`knit` with `solid: true` makes a solid of a closed one).

The marker is `{kind: "sweep", faces, area, frame, closed: [path, profile], caps, pathLength, up, volume, degree: [du, dv], count: [nu, nv], deviation}`: `up` for the tangent frame, `volume` with caps; `degree`, `count` and `deviation` describe the B-spline the definition converts to (what STEP export writes).

Refusals, each with a `hint`:

- `sweepSurfaceNoInput {field}`: no `profile` or no `path` (or a sketch with nothing to use).
- `sweepSurfaceFrame {at, up, detail}`: the up direction is along the path tangent somewhere.
- `sweepSurfaceDegenerate {at, detail}`: the path has no tangent somewhere, or the surface has no area (the profile runs along the sweep direction).
- `sweepSurfaceFold {at, radius, distance}`: turning with the path, the profile reaches past the path's centre of curvature (`radius`), so the surface folds over itself.
- `sweepSurfaceCaps {detail}`: `caps` on an open or non-planar profile.
- `sweepSurfaceFailed {detail, tolerance}`: the definition does not convert to a B-spline within `tolerance`.

Errors, in order: `invalidValue` (`gapTolerance`), `missingReference {curve, index}`, `missingSketch`, `sweepSurfaceGap`, `sweepSurfaceNotChain`, `sweepSurfaceNoInput`, `invalidValue` (`tolerance`, `frame`, `up`), `sweepSurfaceDegenerate` (path), `sweepSurfaceFrame`, `sweepSurfaceFold`, `sweepSurfaceDegenerate` (area), `sweepSurfaceCaps`, `sweepSurfaceFailed`. A self-intersection of parts far apart along the path (a path that comes back near itself) is not detected.

A circle of radius 1.5 square to an arc of radius 6 at its start, swept a quarter turn with caps:

```json
[
  {"type": "plane", "id": "P3", "mode": "frame", "origin": [6, 0, 0], "xDir": [1, 0, 0], "yDir": [0, 0, 1]},
  {"type": "sketch", "id": "S3", "plane": {"kind": "feature", "id": "P3"}, "points": [{"id": "c", "x": 0, "y": 0}], "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 1.5}], "constraints": []},
  {
    "type": "sketch", "id": "S4", "plane": {"kind": "std", "id": "front"},
    "points": [{"id": "c", "x": 0, "y": 0}, {"id": "a", "x": 6, "y": 0}, {"id": "b", "x": 0, "y": 6}],
    "entities": [{"type": "arc", "id": "e1", "c": "c", "p1": "a", "p2": "b"}], "constraints": []
  },
  {"type": "sweepSurface", "id": "W1", "profile": "S3", "path": "S4", "caps": true}
]
```

Run result: no errors. Surfaces: `W1`: `W1.W`, `W1.C0`, `W1.C1`. The marker of `W1`: area 88.826 (2π · 1.5 · 6 · π/2), volume 66.620 (π · 1.5² · 6 · π/2), closed `[false, true]`, up `[0, 0, 1]`.


## blendSurface

A sheet whose one face is the rolling-ball blend between two faces, kept as its definition: at each point of the spine a ball of the radius there touches both faces, and the section is the circular arc of the ball from the contact on the first face to the contact on the second.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `faces` | `[face, face]` | required | The two faces the ball rolls on (of the solid or of sheets). |
| `radius` | number or `[start, end]` | required | Ball radius (mm), or a radius varying linearly along the spine from `start` to `end`. Expressions allowed. |
| `sides` | `[s1, s2]` | toward each other | For each face +1 when the ball lies on the side its outward normal points to, −1 against it. The default turns each face toward the other one: on a convex edge of a solid the ball is inside both (a round), on a concave corner outside both (a fillet). |
| `from`, `to` | `[x, y, z]` | | The span: the spine is the line between them. |
| `at` | `[x, y, z]` | | Without `from` / `to` the spine is the edge the two faces share; `at` picks the nearest when they share several. |
| `tolerance` | number | `0.001` | Bound of the B-spline the definition converts to (mm). |

The ball centre at each spine point lies in the plane through it square to the spine. The sheet is one face `<id>.R` bounded by the two contact curves (exact iso lines of the definition, on the two faces) and the two end arcs (circles). Its outward normal agrees with the first face where the ball touches it (away from the ball centre for a round). A closed spine (a circular edge all the way round) gives a seam.

The marker is `{kind: "blend", faces, area, radius: [start, end], sides, span: "edge" | "points", spineLength, closed, degree, count, deviation}`.

Refusals, each with a `hint`:

- `blendSurfaceNoInput`: no `faces`.
- `blendSurfaceSpan {detail}`: the faces share no edge and there is no `from` / `to`, only one of them, or the same point twice.
- `blendSurfaceSides {face, detail}`: the default sides are undefined (one face lies in the surface of the other).
- `blendSurfaceNoContact {at, u, detail}`: no ball of any radius up to the given one touches both faces inside them along the span (wrong `sides`, faces too far apart, a span off the faces).
- `blendSurfaceRadius {radius, maxRadius, at, detail}`: a smaller ball rolls but this one does not (its contact runs off a face, or it does not fit): `maxRadius` is the largest that fits (the radius law scaled down to it).
- `blendSurfaceFailed {detail, tolerance}`: the definition does not convert to a B-spline within `tolerance`.

Errors, in order: `blendSurfaceNoInput`, `invalidValue` (`faces`: not two different faces), `missingReference`, `invalidValue` (`radius`, `tolerance`, `from`, `to`, `at`), `blendSurfaceSpan`, `blendSurfaceSides`, `invalidValue` (`sides`), `blendSurfaceRadius`, `blendSurfaceNoContact`, `blendSurfaceFailed`.

The round of the top front edge of the plate of [the overview](overview.md) (top `F2.C1` at z = 10, front `F2.S.e1` at y = 0, 40 long), appended to it:

```json
{"type": "blendSurface", "id": "B1", "faces": ["F2.C1", "F2.S.e1"], "radius": 3}
```

Run result: no errors. Surfaces: `B1`: `B1.R`, a quarter cylinder about the line y = 3, z = 7. The marker of `B1`: area 188.496 (π · 3 · 40 / 2), sides `[-1, -1]`, span `edge`. A radius of 12 is `blendSurfaceRadius` with `maxRadius` 10 (the front face is 10 high).


## editCurve

Rebuilds picked curves as one B-spline and lets a client reshape it by its control points.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `curves` | list of curve references | required | `{"kind": "curve", "id": ...}` (an earlier curve feature), `{"kind": "sketchCurve", "sketch": ..., "entity": ...}` or `{"kind": "edge", "faces": [..], "at": [..]}`. |
| `method` | string | | `count` (a fixed number of control points) or `tolerance`. |
| `degree` | number | `5` | B-spline degree. |
| `count` | number | | Control points, for `method: "count"`. |
| `tolerance` | number | `0.01` | For `method: "tolerance"`. |
| `keepEnds` | boolean | `true` | Keep the end points. |
| `keepTangents`, `keepCorners` | boolean | `false` | Keep end tangents / corners. |
| `entries`, `planar`, `plane` | | | Control-point edits and planar constraint. |

It makes a curve and no faces. The result is a curve in `curves` and a marker with the control net, weights, knots and the deviation. Errors: `editCurveNoInput`, `editCurveInputMissing {curve}`, `editCurveDegree`, `editCurveNotChain`, `editCurveNotSmooth`, `editCurveTooFewPoints`, `editCurveTooManyPoints`, `editCurveWeight`, `editCurveNoPlane`.

Appended to the document of the projection above:

```json
{
  "type": "editCurve", "id": "F5", "curves": [{"kind": "curve", "id": "F4"}], "method": "count",
  "count": 8, "degree": 3
}
```

Run result: no errors; 6 faces, volume 8000 mm³. Marker of `F5`: degree 3, 8 control points, deviation 0.033297.


## 3D sketches

A `sketch3d` feature holds points with three coordinates, curves in space and constraints. It is not on a plane, so it has no `plane` field and no profiles; it serves as a path (`sweep`, `normal` planes) and as a place to draw curves.

| Field | Type | Meaning |
|---|---|---|
| `points` | array | `{"id", "x", "y", "z", "fixed"}`. Missing coordinates read as 0. |
| `entities` | array | `line` (`p1`, `p2`), `arc` (`c`, `p1`, `p2`), `spline` (`pts`, `closed`), `cvspline` (`pts`, `degree`, `knots`, `closed`), `point` (`p`) and `ref` (a curve of another sketch with its own B-spline definition). Circles and ellipses are not part of 3D sketches and are dropped. |
| `constraints` | array | Same shape as the 2D constraints, with `fix` taking `[x, y, z]`. |

3D constraint types: `coincident`, `alongX`, `alongY`, `alongZ`, `parallel`, `perpendicular`, `equal`, `collinear`, `pointOnLine`, `midpoint`, `corner`, `fix`, `onPlane`, `tangent`, `g1`, `g2`, `g3`, `onCurve`, `tension`, `distance`, `radius` and `angle`. `alongX` makes a line (or two points) parallel to the X axis, and likewise for Y and Z; the others follow the 2D constraints of the same name. As in 2D, a constraint with an `expr` is re-solved when its value changes and the solved sketch is returned; otherwise the points are used as written.

The reply lists each 3D sketch in `sketches3` as `[id, {sketch}]`, normalised like a 2D sketch (missing arrays added, coordinates converted).

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch3d", "id": "F1",
      "points": [
        {"id": "a", "x": 0, "y": 0, "z": 0, "fixed": true},
        {"id": "b", "x": 10, "y": 1, "z": 1}
      ],
      "entities": [{"type": "line", "id": "e1", "p1": "a", "p2": "b"}],
      "constraints": [
        {"id": "k1", "type": "distance", "refs": ["a", "b"], "value": 10, "expr": "=12"},
        {"id": "k2", "type": "alongX", "refs": ["e1"]}
      ]
    }
  ]
}
```

Run result: no errors; no solid body. `sketches3` holds the solved sketch: `b` moved from (10, 1, 1) to (12, 0, 0).


As a sweep path, a sketch may hold exactly one open `spline` (plus points and lines that give its end tangents through `tangent` constraints). The profile is carried along by a rotation-minimising frame. The profile must be normal to the spline's start direction, which a tangent line can fix:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch3d", "id": "F1",
      "points": [
        {"id": "a", "x": 0, "y": 0, "z": 0},
        {"id": "b", "x": 0, "y": 10, "z": 3},
        {"id": "c", "x": 0, "y": 20, "z": 8},
        {"id": "d", "x": 0, "y": 30, "z": 20},
        {"id": "t", "x": 0, "y": -5, "z": 0}
      ],
      "entities": [
        {"type": "spline", "id": "e1", "pts": ["a", "b", "c", "d"]},
        {"type": "line", "id": "e2", "p1": "a", "p2": "t"}
      ],
      "constraints": [{"id": "k1", "type": "tangent", "refs": ["e1", "e2"]}]
    },
    {
      "type": "sketch", "id": "F2", "plane": {"kind": "std", "id": "top"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 2}], "constraints": []
    },
    {"type": "sweep", "id": "F3", "op": "add", "sketch": "F2", "path": {"kind": "sketch", "id": "F1"}}
  ]
}
```

Run result: no errors; 4 faces, volume 470.6909 mm³. The tangent line fixes the start direction of the spline (along -y from `t` to `a`), which is normal to the top plane where the profile lies. Faces `F3.S.e1~2`, `F3.S.e1~1`, `F3.C0`, `F3.C1`.


## Not built

The feature types `surfaceOffset`, `cutWithSurface`, `dome` and `splitLine` are accepted as names (they are not `unknownFeature`) but are not implemented: they fail with the error code `TODO(rebuild): <type> needs ck-ops`.

## See also

- [Solids from sketches](features-solids.md)
- [Edge and face edits](features-edits.md)
- [Sketches](sketches.md)
- [Planes](planes.md)
- [Persistent names](../kernel/naming.md)
