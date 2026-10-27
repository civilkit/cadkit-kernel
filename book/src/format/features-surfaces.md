# Surfaces and curves

Besides solids, a part can hold sheets (surface bodies) and curves. This chapter covers the features that make sheets (`surfacePlanar`, `surfaceExtrude`, `surfaceRevolve`), combine and change them (`knit`, `trimSurface`, `fill`, `thicken`, `extend`, `deleteFace`, `freeform`, `pipeCut`), the features that make curves (`projectCurve`, `editCurve`; `helix` is in [solids from sketches](features-solids.md)) and 3D sketches (`sketch3d`). A `loft` with `"surface": true` also makes a sheet.

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
| `axis` | string | required | The id of a line entity of the sketch, as for `revolve`. |
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

The patch is one face, `F`. Errors: `missingReference`, `fillNotClosed {detail}` (the chain is open), `fillInvalid` (the edges do not form one chain), `fillFailed {detail}`.

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
