# Solids from sketches

These features build a new solid from sketch profiles or from nothing, and combine it with the body made so far: `extrude`, `revolve`, `sphere`, `sweep`, `loft`, `rib` and `helix` (a curve that `sweep` can follow). The `sketch` feature itself is in [sketches](sketches.md) and `hole`, which cuts holes at sketch points, is in [edge and face edits](features-edits.md).

## Operations and the body

The features that build a tool solid take an `op` that says how the tool meets the body:

| `op` | Effect |
|---|---|
| `add` (default) | The tool is united with the body. The first `add` on an empty body becomes the body. |
| `cut` | The tool is subtracted. A cut with no body is `nothingToCut`. A cut that removes nothing is `cutNoEffect` and leaves the body unchanged. |
| `newBody` (read aliases `new`, `separate`) | The tool becomes a separate body in the same part, not united with what it touches. |

Any other value reads as `add`. `newBody` is accepted by `extrude`, `revolve`, `sweep`, `loft` and `sphere`; any other feature that is given `newBody` fails with `invalidValue`. A `body` name on one of these features, with `op: "newBody"`, names the new body (`bodyNameConflict` if the name is taken; `invalidValue` if `body` is given without `newBody` or is not a non-empty string). See [patterns and transforms](features-patterns.md) for selecting bodies by name.

Every feature of this chapter that combines with the body also reads the fuzzy-boolean fields `tolerance` and `autoFuzzy`; they are described in [features](features.md#fields-every-feature-reads). A boolean that cannot be completed is `booleanFailed`, and a result whose volume is impossible for its operands is `booleanVolumeMismatch`; in both cases the body is left as it was.

## extrude

Sweeps the profile of a sketch along its normal.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | The sketch. A missing or unknown sketch is `missingSketch`. |
| `op` | string | `"add"` | See above. |
| `end` | string | `"blind"` | The end condition (table below). |
| `depth` | number or expression | `0` | Length in mm. Zero or negative is clamped to `1e-6`. |
| `flip` | boolean | `false` | Extrude against the sketch normal. |
| `draft` | number or expression | none | Taper angle in degrees; the profile shrinks towards the end. `draftOut` reverses it. |
| `draftOut` | boolean | `false` | Taper outwards. |
| `twist` | number or expression | `0` | Degrees of rotation over the whole depth, counter-clockwise about the sketch normal. Not combinable with `draft` or `upToNext` (`draftUnsupported`). |
| `twistCentre` | `[x, y]` | sketch origin | The axis of the twist, in sketch coordinates. |
| `regions` | list of entity ids | all regions | Selects which closed regions are swept (see below). |
| `thin` | object | none | Sweeps a wall around the profile curves instead of the filled regions (see below). |
| `endRef` | see below | none | The target of `upToFace`, `offsetFromFace` and `upToPoint`. |
| `offset` | number or expression | `0` | `offsetFromFace`: distance from the face along its outward normal. |
| `strictEnd` | boolean | `false` | Report an unreachable end condition as an error instead of falling back. |
| `body` | string | none | Name of the new body (with `op: "newBody"`). |

End conditions:

| `end` | Extent |
|---|---|
| `blind` | `depth` along the normal (or against it with `flip`). |
| `mid` | `depth` in total, centred on the sketch plane. |
| `through` | Through the whole body: from the sketch plane for a length computed from the body's extent. On an empty body 1000 mm. |
| `throughBoth` | Through the whole body on both sides. On an empty body 1000 mm each way. |
| `upToNext` | To the next face of the body: the tool is a through tool trimmed by the body. With no body: `nothingToCut`. No face to stop at: `upToNextNone`; an add that never meets the body: `upToNextOpen`. |
| `upToFace` | To a planar face named by `endRef`. |
| `offsetFromFace` | To the plane of a face moved by `offset` along the face's outward normal (negative moves into the material). |
| `upToPoint` | To the plane through a point, parallel to the sketch plane. |
| any other | `unknownEnd` with `params.end` holding the value. |

The direction of a `cut` is reversed first: a cut goes against the sketch normal unless `flip` is set, so a sketch on the top face cuts into the material. A cut that runs the wrong way removes nothing and reports `cutNoEffect`.

The face names are `C0` for the cap whose outward normal opposes the sketch normal and `C1` for the other, and `S.<entity>` for the wall made from each profile entity. A profile with several regions numbers them `~1`, `~2`. After a cut, the faces left in the body are the tool's: the floor of a blind pocket cut from a top face is `C0`.

Appended to the plate of [the overview](overview.md) (a sketch on its top face, then the cut):

```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "c", "x": 20, "y": 10}],
    "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 4}], "constraints": []
  },
  {"type": "extrude", "id": "F4", "op": "cut", "sketch": "F3", "end": "blind", "depth": 4}
]
```

Run result: no errors; 8 faces, volume 7798.9381 mm³. The pocket faces are `F4.C0`, `F4.S.e1`.


`flip`, `mid` and `through`, applied to the plate rectangle (the sketch lies in z = 0 with normal +z):

| Feature fields | z range of the plate |
|---|---|
| `"end": "blind", "depth": 10` | 0 to 10 |
| `"end": "blind", "depth": 10, "flip": true` | -10 to 0 |
| `"end": "mid", "depth": 10` | -5 to 5 |
| `"end": "through"` | 0 to 1000 |
| `"end": "throughBoth"` | -1000 to 1000 |
| `"end": "blind", "depth": 0` | 0 to 0.000001 |

### Taper and twist

`draft` tapers the walls inwards by the given angle; `draftOut: true` tapers outwards. A taper that closes the profile before the depth is reached is `draftTooLarge`. For a profile made of one circle (a cone) the error carries `maxHeight`, the depth at which the taper closes to a point, and a successful drafted extrude of such a profile adds `limits: [["F2", {"maxHeight": h}]]` to the regen reply so an editor can limit its depth field. A depth equal to `maxHeight` builds a cone with one apex and no top cap. A drafted extrude with `end` `mid` or `throughBoth` is not built: it fails with the code `TODO(rebuild): mid-plane drafted extrude (boolean unite) needs ck-ops`.

Added after the rectangle sketch `F1` of the plate (without its extrude):

```json
{"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10, "draft": 5}
```

Run result: no errors; 6 faces, volume 7485.2737 mm³. The 40 x 20 face at the sketch plane shrinks towards the top; box [0,0,0] to [40,20,10].


`twist` turns the profile as it extrudes. The side faces are screw surfaces (exact cylinders for circles about the axis, fitted B-splines otherwise); the volume stays area times depth.

Likewise:

```json
{"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10, "twist": 45}
```

Run result: no errors; 6 faces, volume 8000 mm³. The twisted prism keeps the volume of the straight one (40 x 20 x 10 = 8000).


### End references

`endRef` names the target of `upToFace` and `offsetFromFace` (a face) and `upToPoint` (a point).

| End | Forms of `endRef` |
|---|---|
| `upToFace`, `offsetFromFace` | `{"kind": "face", "id": "F4.C0", "at": [x, y, z]}` or the bare face id `"F4.C0"`. `at` is a point on the face: it lets the kernel find the face again if its name has changed (warning `endRefRelinked`). |
| `upToPoint` | `[x, y, z]`, `{"kind": "point", "at": [x, y, z]}` or `{"kind": "vertex", "faces": ["F2.C1", "F2.S.e4", "F2.S.e1"], "at": [x, y, z]}` (the vertex shared by the faces, nearest to `at`). |

The target face must be planar. A plane slanted against the sketch is supported: the tool is trimmed by a boolean and the new end face takes the name of the cap it replaces. Non-planar targets are `upToFaceUnsupported`. A missing or unresolved `endRef` is `missingReference` with `params.field: "endRef"`.

When the end cannot be reached (`upToFaceBehind`: the face is not ahead of the whole profile; `upToFaceOutside`: the face does not cover the profile; `upToFaceUnsupported`; `upToPointBehind`), the feature does not fail. It runs as a blind extrude of `depth` and the reply carries a warning `endConditionFallback {end, reason, fallback: "blind", depth}`. With `"strictEnd": true` the reason becomes the error instead. A missing `endRef` is never downgraded.

Two discs of radius 20, one on the front plane (z 0 to 10) and one above it on a plane at z = 30 (z 30 to 35). `F5` is a small circle on the top face of the lower disc; the rod will start there and end at the underside of the upper disc:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 20}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10},
    {
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"},
      "offset": 30
    },
    {
      "type": "sketch", "id": "F3", "plane": {"kind": "feature", "id": "P1"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 20}], "constraints": []
    },
    {"type": "extrude", "id": "F4", "op": "add", "sketch": "F3", "end": "blind", "depth": 5},
    {
      "type": "sketch", "id": "F5", "plane": {"kind": "face", "id": "F2.C1"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 4}], "constraints": []
    }
  ]
}
```


Appended to that document:

```json
{
  "type": "extrude", "id": "F6", "op": "add", "sketch": "F5", "end": "upToFace",
  "endRef": {"kind": "face", "id": "F4.C0"}
}
```

Run result: no errors; 7 faces, volume 19854.8656 mm³. The rod runs from z = 10 to the underside of the upper disc at z = 30 and joins the two discs: pi 20² 15 + pi 4² 20 = 19854.8656.


With a target that is behind the sketch (the bottom face of the lower disc), the extrude falls back to its `depth`:

```json
{
  "type": "extrude", "id": "F6", "op": "add", "sketch": "F5", "end": "upToFace", "endRef": "F2.C0",
  "depth": 3
}
```

Run result: no errors; warnings `F6`: `endConditionFallback` `{"end":"upToFace","reason":"upToFaceBehind","fallback":"blind","depth":3}`; 8 faces, volume 19000.3524 mm³. The rod is a blind extrude of 3 mm.


### Selecting regions

By default all closed regions of the sketch are swept. `regions` is a list of sketch entity ids; a region is selected when any listed id is a segment of its outer boundary. A selected region keeps its holes; an id that bounds a hole (a circle inside a rectangle) selects that hole as a disc. Names are numbered as if the selection were the whole sketch. Errors: `extrudeRegionUnknown {ids}` (ids that are not entities of the sketch), `extrudeRegionNone {regions}` (an empty list, or ids that bound no region), `invalidValue {field: "regions"}` (not a list of strings). `revolve` takes the same field.

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
        {"id": "d", "x": 0, "y": 20},
        {"id": "h", "x": 60, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"},
        {"type": "circle", "id": "e5", "c": "h", "r": 6}
      ],
      "constraints": []
    },
    {
      "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 4,
      "regions": ["e5"]
    }
  ]
}
```

Run result: no errors; 3 faces, volume 452.3893 mm³. Only the disc is built: `F2.C0`, `F2.C1`, `F2.S.e5`.


### Thin walls

`thin` sweeps a wall around the curves of the profile. An open chain gives a wall in one step; a closed loop gives a hollow wall.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `thickness` | number or expression | `1` | Wall thickness; must be above 0. |
| `side` | string | `"inside"` | `inside`, `outside`, `both` or `mid` (centred). Closed loops are oriented counter-clockwise; for an open chain `inside` is the left of the direction that turns counter-clockwise. |
| `thickness2` | number or expression | `thickness` | Thickness on the second side, for `both`. Zero or more. |
| `ends` | string | `"flat"` | `flat` or `round` (half-disc caps on an open chain). |
| `cornerStyle` | string | `"mitre"` | `mitre` (also spelt `miter`) or `round`. |
| `enabled` | boolean | `true` | `false` keeps the settings but switches the wall off. A `null` or `false` `thin` is off too. |

Offsets are exact for lines, arcs and circles and fitted (within 1e-5) for ellipses and splines. Errors: `thinTooThick {limit, minRadius, maxThickness, thickness, face}` when the offset passes a centre of curvature or a short segment is eaten; `invalidValue {field}` with `thin.thickness`, `thin.side`, `thin.ends` or `thin.cornerStyle`. `thin` also works on `revolve`. It is not available on `sweep`, `loft` or `hole`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
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
    {
      "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 4,
      "thin": {"thickness": 2, "side": "mid"}
    }
  ]
}
```

Run result: no errors; 8 faces, volume 240 mm³. The 2 mm wall along the L-shaped chain has the faces `F2.C0`, `F2.C1`, `F2.S.e1.W1`, `F2.S.e2.W1`, `F2.S.e2.E1`, `F2.S.e2.W0`, `F2.S.e1.W0`, `F2.S.e1.E0`.


### Separate bodies

A second block next to the plate, as its own body (appended to the plate):

```json
[
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
```

Run result: no errors; 12 faces, volume 10000 mm³. The block is a second body next to the plate: 8000 + 2000 = 10000.


### Errors of extrude

| Code | When |
|---|---|
| `missingSketch` | `sketch` names no regenerated sketch. |
| `badExpression` | A numeric field holds a bad expression. |
| `invalidValue` | A sketch point is not finite (`params.point`, `detail`, `hint`); bad `regions`, `thin` or `body` values. |
| `noClosedProfile` | The sketch has no closed region (an open chain, no curves, construction only). |
| `unknownEnd` | `end` is not one of the names above. |
| `missingReference` | An end condition needs a reference that is missing or does not resolve. |
| `nothingToCut` | A cut, or `upToNext`, with no body. |
| `cutNoEffect` | A cut removed nothing. |
| `upToNextNone`, `upToNextOpen` | `upToNext` found no face to stop at. |
| `upToFaceBehind`, `upToFaceOutside`, `upToFaceUnsupported`, `upToPointBehind` | With `strictEnd`; otherwise warnings (see above). |
| `extrudeRegionUnknown`, `extrudeRegionNone` | Bad `regions`. |
| `thinTooThick` | The wall is too thick for the profile. |
| `draftTooLarge`, `draftUnsupported` | A taper that closes, or `twist` with `draft`. |
| `booleanFailed`, `booleanVolumeMismatch` | The boolean with the body failed. |

## revolve

Sweeps the profile about an axis.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | The profile sketch. |
| `axis` | string or object | required | A line entity of the profile's own sketch, by id (it may be a construction line), or a line of another sketch as `{"sketch": "<sketch id>", "id": "<line id>"}` (entity ids restart in each sketch, so the owner is named). Its direction is from `p1` to `p2`. Any other form, an unknown sketch or an unknown line is `missingAxis`. |
| `op` | string | `"add"` | See above. |
| `angle` | number or expression | `360` | Degrees; positive turns right-handed about the axis direction, negative the other way; 360 or more is a full turn. Zero is `invalidValue`. |
| `end` | string | | Only `mid` is read: the sweep is centred on the sketch plane. |
| `flip` | boolean | `false` | Reverses the direction of the sweep. |
| `regions`, `thin`, `body` | | | As for `extrude`. |

A profile must lie on one side of the axis: a region that crosses it is `profileCrossesAxis`. A revolve cut does not reverse its direction (unlike an extrude cut).

Faces: `S.<entity>` for each profile entity; `C0` (at the start, in the sketch plane) and `C1` (at the end) for a partial turn. A full turn has no caps.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "o", "x": 0, "y": 0},
        {"id": "p", "x": 0, "y": 20},
        {"id": "a", "x": 5, "y": 0},
        {"id": "b", "x": 15, "y": 0},
        {"id": "c", "x": 15, "y": 20},
        {"id": "d", "x": 5, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "ax", "p1": "o", "p2": "p", "construction": true},
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    }
  ]
}
```


The profile above is a rectangle from x = 5 to 15 and y = 0 to 20 and a construction line `ax` on the Y axis. Appended:

```json
{"type": "revolve", "id": "F2", "op": "add", "sketch": "F1", "axis": "ax", "end": "blind", "angle": 360}
```

Run result: no errors; 4 faces, volume 12566.3706 mm³. A tube with outer radius 15 and inner radius 5 and height 20: volume pi (15² - 5²) 20 = 12566.3706. Faces `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`; no caps for a full turn.


```json
{"type": "revolve", "id": "F2", "op": "add", "sketch": "F1", "axis": "ax", "angle": 90}
```

Run result: no errors; 6 faces, volume 3141.5927 mm³. A quarter of the tube, with faces `F2.C0`, `F2.C1`, `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`.


The axis can be drawn in a different sketch. With the construction line `ax` moved into its own sketch `F0` (the same plane, a line from (0, 0) to (0, 20)) and left out of `F1`:

```json
{"type": "revolve", "id": "F2", "op": "add", "sketch": "F1", "axis": {"sketch": "F0", "id": "ax"}, "angle": 360}
```

Run result: no errors; 4 faces, volume 12566.3706 mm³, the same tube as with the axis inside `F1`. The axis line is read at its solved position in `F0` and placed by `F0`'s plane. `surfaceRevolve` takes the same forms.


Moving the profile points `a` and `d` to x = -5 puts the profile on both sides of the axis:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "o", "x": 0, "y": 0},
        {"id": "p", "x": 0, "y": 20},
        {"id": "a", "x": -5, "y": 0},
        {"id": "b", "x": 15, "y": 0},
        {"id": "c", "x": 15, "y": 20},
        {"id": "d", "x": -5, "y": 20}
      ],
      "entities": [
        {"type": "line", "id": "ax", "p1": "o", "p2": "p", "construction": true},
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "revolve", "id": "F2", "op": "add", "sketch": "F1", "axis": "ax", "angle": 360}
  ]
}
```

Run result: errors `F2`: `profileCrossesAxis`; no solid body.


Errors: `missingSketch`, `missingAxis`, `profileCrossesAxis`, `noClosedProfile`, `invalidValue` (angle 0), `nothingToCut`, `badExpression`, and the boolean errors of `extrude`.

## sphere

A ball. It needs no sketch.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `center` | `[x, y, z]` | `[0, 0, 0]` | The centre, each component a number or expression. |
| `radius` | number or expression | `0` | Must be above 0, otherwise `invalidValue` with a `detail`. |
| `op` | string | `"add"` | `add`, `cut` or `newBody`. |
| `body` | string | none | Name of the body, with `newBody`. |

The poles lie along +Y. The single face is `S.e1`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [{"type": "sphere", "id": "F1", "op": "add", "center": [0, 0, 0], "radius": 10}]
}
```

Run result: no errors; 1 face, volume 4188.7902 mm³. One spherical face, `F1.S.e1`; the volume is 4/3 pi 10³ = 4188.7902.


## sweep

Moves the profile along a path.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | The profile. It must lie on a plane perpendicular to the path at its start: `sweepProfileNotNormal` otherwise. |
| `path` | reference | required | `{"kind": "sketch", "id": "F1"}` for an open chain of lines and arcs in a sketch (or a `sketch3d` that holds one spline), or `{"kind": "curve", "id": "F5"}` for a helix or another curve feature. A missing or unusable path is `missingPath`. |
| `op` | string | `"add"` | See above. A sweep cut is not reversed. |
| `twist` | number or expression | `0` | Degrees of rotation of the profile along the path. |
| `guides` | list of curve references | none | Guide curves (`{"kind": ...}` with `sketchCurve`, `edge` or `curve`) that the profile follows. |
| `guideMode` | string | `"scale"` | `scale`, `turn` or `stretch`. |
| `surface` | boolean | `false` | A sheet sweep. Not built: the error code is `TODO(rebuild): surface sweep (open profile -> sheet body) needs ck-ops`. |
| `body` | string | none | Name of the new body. |

The path is walked from its free end. Consecutive path pieces must be joined end to end; corners between two lines are mitred. A corner between a line and an arc, or two arcs, that is not tangent is not built (`TODO(rebuild): mitre joint other than line-line (turn < 166 deg)`); make the arc tangent.

Faces: `C0` and `C1` for the profile ends and `S.<profile entity>` for the surface swept by each profile entity. A side face is split `~1`, `~2`, ... along the pieces of the path.

A circle swept along a path made of a line, an arc and a line (a handle). The document is complete:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "name": "Handle path", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "p1", "x": 0, "y": 0},
        {"id": "p2", "x": 0, "y": 14},
        {"id": "p3", "x": 30, "y": 14},
        {"id": "p4", "x": 30, "y": 0},
        {"id": "p5", "x": 15, "y": 14}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "p1", "p2": "p2"},
        {"type": "arc", "id": "e2", "c": "p5", "p1": "p3", "p2": "p2"},
        {"type": "line", "id": "e3", "p1": "p3", "p2": "p4"}
      ],
      "constraints": []
    },
    {
      "type": "sketch", "id": "F2", "plane": {"kind": "std", "id": "top"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 3.5}], "constraints": []
    },
    {"type": "sweep", "id": "F3", "op": "add", "sketch": "F2", "path": {"kind": "sketch", "id": "F1"}}
  ]
}
```

Run result: no errors; 5 faces, volume 2891.1061 mm³. The caps are `F3.C0` and `F3.C1`; the three path pieces (line, arc, line) give three side faces named after the profile circle `e1`: `F3.S.e1~1`, `F3.S.e1~2`, `F3.S.e1~3`.


A helix as a path (a coil of wire):

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "H1", "plane": {"kind": "std", "id": "top"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "helix", "id": "F1", "sketch": "H1", "pitch": 4, "revolutions": 5},
    {
      "type": "sketch", "id": "F2", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 10, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 1.5}], "constraints": []
    },
    {"type": "sweep", "id": "F3", "op": "add", "sketch": "F2", "path": {"kind": "curve", "id": "F1"}}
  ]
}
```

Run result: no errors; 4 faces, volume 2220.661 mm³.


Errors: `missingSketch`, `missingPath`, `sweepPathInvalid` (the path sketch is not one open chain), `sweepProfileNotNormal`, `noClosedProfile`, `nothingToCut`, `sweepFailed`, `sweepGuideMissing`, `sweepGuidesTwist` (twist with guides), `sweepGuideShort`, `sweepGuideStretchTwo` and the other guide codes, `booleanFailed`.

## loft

Joins two or more sections.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketches` | list of feature ids | required | The sections, in order. Fewer than two is `loftProfiles`. |
| `op` | string | `"add"` | Ignored for `surface: true`. |
| `surface` | boolean | `false` | Build a sheet (placed in `surfaces`, not in `body`). Sections may then be open chains. |
| `guides` | list | none | Guide curves: `{"sketch": "G1", "influence": "next"}` or a bare sketch id. Only sketch guides with influence `next` are supported. |
| `centreLine` | `{"sketch": id}` | none | A planar open chain the sections follow. |
| `startNormal`, `endNormal` | number or `true` | none | Tension of the normal at the ends (`true` is 1). |
| `startTangent`, `endTangent`, `startContinuity`, `endContinuity` | | none | Tangency to the faces next to the loft ends. Not available: any value other than `false` or `null` fails with `loftTangentEdges`. |
| `connectors` | list | none | Positions of the start points to match, as fractions or `null`. |
| `apexTangent` | number | `1` | For a point end: 0 to 2, how wide the taper is at the apex. |
| `body` | string | none | Name of the new body. |

Each section must be one closed region (`loftProfile` for several, `noClosedProfile` for none). Sections whose regions have inner loops make a hollow loft: the outline loft minus the loft of the holes (matched across sections by nearest centroid); the numbers of holes must agree, otherwise `loftProfile`. The first or last section may be a sketch holding exactly one `point` entity: the loft ends in a point and has no cap there.

The face names come from the entities of the first section: `S.<entity>`, `C0` and `C1` (no cap at a point end).

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": -10, "y": -10},
        {"id": "b", "x": 10, "y": -10},
        {"id": "c", "x": 10, "y": 10},
        {"id": "d", "x": -10, "y": 10}
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
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"},
      "offset": 30
    },
    {
      "type": "sketch", "id": "S2", "plane": {"kind": "feature", "id": "P1"},
      "points": [
        {"id": "a", "x": -5, "y": -5},
        {"id": "b", "x": 5, "y": -5},
        {"id": "c", "x": 5, "y": 5},
        {"id": "d", "x": -5, "y": 5}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "loft", "id": "F5", "op": "add", "sketches": ["S1", "S2"]}
  ]
}
```

Run result: no errors; 6 faces, volume 7000 mm³. A frustum: (400 + 100 + sqrt(400 x 100)) x 30 / 3 = 7000. Faces `F5.S.e1`, `F5.S.e2`, `F5.S.e3`, `F5.S.e4`, `F5.C0`, `F5.C1`.


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": -10, "y": -10},
        {"id": "b", "x": 10, "y": -10},
        {"id": "c", "x": 10, "y": 10},
        {"id": "d", "x": -10, "y": 10}
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
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"},
      "offset": 30
    },
    {
      "type": "sketch", "id": "S2", "plane": {"kind": "feature", "id": "P1"},
      "points": [{"id": "p", "x": 0, "y": 0}], "entities": [{"type": "point", "id": "e1", "p": "p"}],
      "constraints": []
    },
    {"type": "loft", "id": "F5", "op": "add", "sketches": ["S1", "S2"]}
  ]
}
```

Run result: no errors; 5 faces, volume 4000 mm³. A pyramid: 400 x 30 / 3 = 4000. It has no cap at the apex: `F5.S.e1`, `F5.S.e2`, `F5.S.e3`, `F5.S.e4`, `F5.C0`.


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": -10, "y": -10},
        {"id": "b", "x": 10, "y": -10},
        {"id": "c", "x": 10, "y": 10},
        {"id": "d", "x": -10, "y": 10}
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
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"},
      "offset": 30
    },
    {
      "type": "sketch", "id": "S2", "plane": {"kind": "feature", "id": "P1"},
      "points": [
        {"id": "a", "x": -5, "y": -5},
        {"id": "b", "x": 5, "y": -5},
        {"id": "c", "x": 5, "y": 5},
        {"id": "d", "x": -5, "y": 5}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "loft", "id": "F5", "surface": true, "sketches": ["S1", "S2"]}
  ]
}
```

Run result: no errors; no solid body. The loft is a sheet: `surfaces` holds `F5` with the faces `F5.S.e1`, `F5.S.e2`, `F5.S.e3`, `F5.S.e4`.


Errors: `missingSketch`, `loftProfiles` (fewer than two sections, or all in one plane), `loftProfile` (with `detail` and `hint`), `loftProfileOpen` (mixed open and closed sections), `loftProfileNotPlanar`, `loftFailed`, `loftInvalidResult`, `loftProfileSelfIntersect`, `loftSectionTooSmall`, `loftGuideInvalid`, `loftCentreLineInvalid` (and `...Kink`, `...Reach`, `...Tight`, `...Apex`, `...Guides`), `loftTangentEdges` (any tangency field set; tangency on loft ends is always refused), `nothingToCut`, `booleanFailed`. Loft errors carry `detail` and, when there is one, a `hint`.

## rib

A thin wall from an open sketch chain to the body.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | A sketch with an open chain and no closed region. |
| `thickness` | number or expression | `2` | Total thickness, centred on the sketch plane. Must be above 0. |
| `flip` | boolean | `false` | Which side of the chain is tried first. A successful result does not depend on it. |

The wall fills the space between the chain and the body, extended past the ends of the chain until it meets the body, and is united with the body. Faces: `C0` and `C1` (the two sides), `S.<entity>r` for the free edge along the chain. Errors: `nothingToCut` (no body), `missingSketch`, `invalidValue` (thickness), `ribNeedsOpenSketch` (the sketch has a closed region or no chain), `ribNoContact` (nothing encloses the rib).

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


Appended to the shelled box above (the plate with its top face removed and 2 mm walls):

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "top"}, "offset": 10},
  {
    "type": "sketch", "id": "F4", "plane": {"kind": "feature", "id": "P1"},
    "points": [{"id": "a", "x": 5, "y": -5}, {"id": "b", "x": 35, "y": -5}],
    "entities": [{"type": "line", "id": "e1", "p1": "a", "p2": "b"}], "constraints": []
  },
  {"type": "rib", "id": "F5", "sketch": "F4", "thickness": 2}
]
```

Run result: no errors; 15 faces, volume 3608 mm³. The rib adds 216 mm³ to the shelled box (3392). Its faces: `F5.C0`, `F5.C1`, `F5.S.e1r`.


## helix

A helix curve, for use as a `sweep` path or as a reference for planes and projections. It adds no body.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `sketch` | feature id | required | A sketch whose first circle gives the radius and the centre. No circle is `helixNeedsCircle`. |
| `definedBy` | string | `"pitchRevs"` | Which two of the three sizes are used: `pitchRevs` (`pitch`, `revolutions`), `heightRevs` (`height`, `revolutions`) or `heightPitch` (`height`, `pitch`). Any other value reads as `pitchRevs`. |
| `pitch` | number or expression | `1` | Rise per turn, mm. |
| `revolutions` | number or expression | `10` | Number of turns. |
| `height` | number or expression | `10` | Total rise, mm. |
| `startAngle` | number or expression | `0` | Degrees from the sketch x axis at which the helix starts. |
| `leftHand` | boolean | `false` | Turn the other way. |
| `reverse` | boolean | `false` | Rise against the sketch normal. |

A helix makes a curve and no faces. The axis is the normal of the sketch plane through the circle's centre. A pitch, number of turns or radius that is not above 0 is `invalidValue`. The reply lists the curve in `curves` as `[id, {pts, axis: false, t0: 0, t1: angle, helix: {o, a, x, y, k, r, angle}}]`: `o` is the centre, `a` the axis, `x` the start direction, `y` the direction the helix turns towards, `k = pitch / (2 pi)`, `r` the radius and `angle` the total turn in radians.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "H1", "plane": {"kind": "std", "id": "top"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {
      "type": "helix", "id": "F1", "sketch": "H1", "definedBy": "pitchRevs", "pitch": 4,
      "revolutions": 5
    }
  ]
}
```

Run result: no errors; no solid body. `curves` holds `F1` with 161 points; `helix` is {"o":[0,0,0],"a":[0,1,0],"k":0.6366,"r":10,"angle":31.4159}: axis +Y (the normal of the top plane), 20 mm tall.


## hole

`hole` cuts holes at the `point` entities of a sketch. It is documented with the other edits that remove material from the body, in [edge and face edits](features-edits.md#hole).

## See also

- [Sketches](sketches.md)
- [Edge and face edits](features-edits.md)
- [Patterns and transforms](features-patterns.md)
- [Surfaces and curves](features-surfaces.md)
- [Features](features.md)
- [Operations](../kernel/operations.md)
