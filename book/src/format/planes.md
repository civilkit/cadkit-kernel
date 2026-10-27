# Planes

Sketches are drawn on planes. This chapter covers the three standard planes, the planes that follow a face or a reference, the `plane` feature with all of its modes, and the `axis` feature that planes and other features use as a line.

## Plane references

A plane is given as a reference object wherever a feature needs one: the `plane` of a sketch, the `base` of a plane feature, the `neutral` of a draft, the `plane` of a mirror.

| Reference | Meaning |
|---|---|
| `{"kind": "std", "id": "front"}` | A standard plane: `front`, `top` or `right`. |
| `{"kind": "feature", "id": "P1"}` | The plane made by an earlier `plane` feature. |
| `{"kind": "face", "id": "F2.C1"}` | A planar face of the body made so far. |

Any other reference, a face that is not planar, and a feature that is not a plane resolve to nothing: a sketch then reports `missingReference`.

The frames of the standard planes are in [the overview](overview.md#coordinate-frames). `front` is the XY plane with normal +Z, `top` is the XZ plane with normal +Y and `right` is the ZY plane with normal +X.

### Frame of a face

The frame of a planar face is computed from its plane alone:

- `z` is the outward normal of the face;
- `o` is the point of the face plane closest to the world origin (the origin projected onto the plane);
- `x` is the world Y axis crossed with `z` and normalised; when `z` is within about 8 degrees of the Y axis (`|z.y| > 0.99`), `x` is the world X axis instead;
- `y` is `z × x`.

For the `right` face `F2.S.e2` of the plate this gives `o = (40, 0, 0)`, `x = (0, 0, -1)`, `y = (0, 1, 0)`, `z = (1, 0, 0)`, the frame of the standard `right` plane moved to the face. A sketch on a face starts at the projected world origin, not at the face centre.

## The `plane` feature

A `plane` feature creates a reference plane. Its frame appears in the `planes` list of the regen reply, keyed by the feature id, and other features refer to it as `{"kind": "feature", "id": "P1"}`.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `mode` | string | `"offset"` | How the plane is made. See the table below; an unknown mode behaves as `offset`. |
| `base` | plane reference | none | The plane the mode starts from (`offset`, `angle`, `perpendicular`, `mid`, `parallel`, `tangent`). |
| `offset` | number or expression | `0` | Distance along the plane normal (`offset`, `curve`). |
| `flip` | boolean | `false` | Turns the plane around: `x` and `z` are negated (for `normal` the `z` and `y` axes). In `angle` mode it negates the angle. |
| `axis` | axis reference | none | The line the plane turns about or contains (`angle`, `perpendicular`, `linePoint`, `tangent`). |
| `angle` | number or expression | `0` | Degrees, for `angle`. |
| `base2` | plane reference | none | The second plane of `mid`. |
| `points` | list of point references | none | Points for `threePoints`, `linePoint`, `parallel`, `normalCurve`, `tangent`. |
| `curves` | list of curve references | none | Curves for `curve` and `normalCurve`. |
| `path` | sketch id | none | The path of `normal`. |
| `atEnd` | boolean | `false` | `normal`, `normalCurve`: use the end of the path instead of its start. |
| `face`, `tangentTo`, `second`, `at` | see `tangent` | | The `tangent` mode. |
| `origin`, `xDir`, `yDir` | `[x, y, z]` | none | The `frame` mode. |
| `visible`, `userHidden` | boolean | `true`, `false` | Display flags, ignored by the kernel. |

The frame of every mode is right-handed with `z` the plane normal. The origin `o` is given per mode below.

### Modes

| `mode` | Plane | Origin and axes |
|---|---|---|
| `offset` | Parallel to `base`, moved along its normal by `offset`. | The base frame with `o = base.o + z * offset`. With `flip` the base `x` and `z` are negated before the offset is applied, so the plane faces the other way and the offset follows. |
| `angle` | `base` turned about `axis` by `angle` degrees, right-handed about the axis direction. | The plane contains the axis line; `o` is the origin of the axis. When the axis lies in the base plane it becomes the new `x`. |
| `perpendicular` | Contains `axis` and is perpendicular to `base`. | `x` is the axis direction, `y` the base normal made perpendicular to it. |
| `mid` | Halfway between `base` and `base2`. | For parallel planes the base frame moved halfway. Otherwise the bisector plane through the line where they meet. |
| `threePoints` | Through three points. | `o` is the first point, `x` points towards the second, `z` is `x` crossed with the direction to the third. |
| `linePoint` | Through a line (`axis`) and a point (`points[0]`). | `o` is the foot of the point on the line, `x` the line direction. |
| `parallel` | Parallel to `base`, through a point (`points[0]`). | The base frame moved along its normal to the point. |
| `normal` | Perpendicular to the sketch `path` at its start (or end with `atEnd`). | `z` is the path direction there. The path is an open chain of lines and arcs in a sketch, or a `sketch3d` holding one spline. |
| `normalCurve` | Perpendicular to the first of `curves` at `points[0]` (the nearest point on the curve) or at the start (end with `atEnd`). | `z` is the curve tangent. |
| `curve` | The plane of the planar `curves`, moved along its normal by `offset`. | `o` is the length-weighted centre of the curves. |
| `tangent` | Tangent to a sphere or cylinder `face`. `tangentTo` selects what fixes it: `"point"` (default, `points[0]` or `at`), `"plane"` (parallel to `base`) or `"line"` (through `axis`); `second` takes the other of the two solutions. | `z` is the normal of the tangent plane. |
| `frame` | Given outright by `origin`, `xDir` and `yDir`. | `x` is kept, `y` is made square to it, `z = x × y`. |

Reference kinds used by the modes:

| For | Reference |
|---|---|
| a point | `{"kind": "sketchPoint", "sketch": "F1", "point": "a"}`, `{"kind": "vertex", "faces": ["F2.C1", "F2.S.e1", "F2.S.e2"], "at": [40, 0, 10]}` (the vertex shared by the faces, nearest to `at`) |
| a curve | `{"kind": "sketchCurve", "sketch": "F1", "entity": "e1"}` (a line, arc or circle), `{"kind": "curve", "id": "F5"}` (an earlier curve feature), `{"kind": "edge", "faces": [..], "at": [..]}` (a body edge) |
| an axis | `{"kind": "axis", "axis": "x"}` (a world axis through the origin), `{"kind": "feature", "id": "A1"}` (an `axis` feature), `{"kind": "edge", "faces": [..], "at": [..]}` (a straight edge), `{"kind": "sketchLine", "sketch": "F1", "entity": "e1"}` |

### Errors

| Code | When |
|---|---|
| `missingReference` | A needed field is missing or does not resolve: no `base` (or `base` not a plane), no `axis` for `angle`/`perpendicular`/`linePoint`, fewer than three points for `threePoints`, a curve or face that cannot be found. |
| `planeLineNormal` | `perpendicular`: the axis is parallel to the normal of `base`. |
| `planePointsInLine` | `threePoints` (collinear points) and `frame` (parallel `xDir` and `yDir`). |
| `planePointOnLine` | `linePoint`: the point is on the line. |
| `planesParallel` | `axis` made of two planes that are parallel. |
| `missingPath` | `normal` without a `path`, or a path that has no segments. |
| `sweepPathInvalid` | `normal`: the sketch is not a single open chain. |
| `curvesNotPlanar` | `curve`: the curves do not lie in one plane (`params.dev` is the deviation). |
| `curvesStraight` | `curve`: the curves are collinear, so the plane is not defined. |
| `planeNotTangent` | `tangent`: the face is not a sphere or cylinder, or the requested tangent plane does not exist. |
| `badExpression` | A numeric field holds a bad expression. |

A failed plane feature adds no frame to the reply, so the sketches that use it report `missingReference` too.

### Examples

Offset planes. The `top` plane offset by 10 lies at y = 10; `flip` on a `right` plane negates its normal before the offset, so the second plane is at x = -10:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "top"}, "offset": 10},
    {
      "type": "plane", "id": "P2", "mode": "offset", "base": {"kind": "std", "id": "right"},
      "offset": 10, "flip": true
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,10,0], x [1,0,0], y [0,0,-1], z [0,1,0]. `P2`: o [-10,0,0], x [0,0,1], y [0,1,0], z [-1,0,0].


A sketch on a plane feature, then extruded. The sketch origin is the plane origin:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "front"},
      "offset": 12
    },
    {
      "type": "sketch", "id": "F2", "plane": {"kind": "feature", "id": "P1"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 10, "y": 0},
        {"id": "c", "x": 10, "y": 10},
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
    {"type": "extrude", "id": "F3", "op": "add", "sketch": "F2", "end": "blind", "depth": 3}
  ]
}
```

Run result: no errors; 6 faces, volume 300 mm³. The sketch frame is o [0,0,12], x [1,0,0], y [0,1,0], z [0,0,1]. The body spans z from 12 to 15.


Planes from a face of the plate (appended to the features of the plate in [the overview](overview.md)). The offset is measured along the outward normal of the face:

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "face", "id": "F2.C1"}, "offset": 5},
  {
    "type": "plane", "id": "P2", "mode": "offset", "base": {"kind": "face", "id": "F2.S.e2"},
    "offset": 0
  }
]
```

Run result: no errors; 6 faces, volume 8000 mm³. `P1`: o [0,0,15], x [1,0,0], y [0,1,0], z [0,0,1]. `P2`: o [40,0,0], x [0,0,-1], y [0,1,0], z [1,0,0].


A plane turned about a world axis, and a plane perpendicular to the front plane through the same axis:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "angle", "base": {"kind": "std", "id": "front"},
      "axis": {"kind": "axis", "axis": "x"}, "angle": 30
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,0,0], x [1,0,0], y [0,0.866025404,0.5], z [0,-0.5,0.866025404].


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "perpendicular", "base": {"kind": "std", "id": "front"},
      "axis": {"kind": "axis", "axis": "x"}
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,0,0], x [1,0,0], y [0,0,1], z [0,-1,0].


The axis of a perpendicular plane cannot be parallel to the base normal:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "plane", "id": "P1", "mode": "perpendicular", "base": {"kind": "std", "id": "front"},
      "axis": {"kind": "axis", "axis": "z"}
    }
  ]
}
```

Run result: errors `P1`: `planeLineNormal`; no solid body.


A plane through three sketch points, and a plane from a line and a point:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 10, "y": 0},
        {"id": "c", "x": 0, "y": 10}
      ],
      "entities": [], "constraints": []
    },
    {
      "type": "plane", "id": "P1", "mode": "threePoints",
      "points": [
        {"kind": "sketchPoint", "sketch": "F1", "point": "a"},
        {"kind": "sketchPoint", "sketch": "F1", "point": "b"},
        {"kind": "sketchPoint", "sketch": "F1", "point": "c"}
      ]
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,0,0], x [1,0,0], y [0,1,0], z [0,0,1].


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "a", "x": 5, "y": 8}], "entities": [], "constraints": []
    },
    {
      "type": "plane", "id": "P1", "mode": "linePoint", "axis": {"kind": "axis", "axis": "x"},
      "points": [{"kind": "sketchPoint", "sketch": "F1", "point": "a"}]
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [5,0,0], x [1,0,0], y [0,1,0], z [0,0,1].


A plane normal to the end of a sketch path:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 0, "y": 30}],
      "entities": [{"type": "line", "id": "e1", "p1": "a", "p2": "b"}], "constraints": []
    },
    {"type": "plane", "id": "P1", "mode": "normal", "path": "F1", "atEnd": true}
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,30,0], x [0,0,1], y [1,0,0], z [0,1,0].


## The `axis` feature

An `axis` feature defines a line that other features can use as a pattern direction, a plane axis or a revolve axis reference through `{"kind": "feature", "id": "A1"}`. Its `refs` list holds either one reference or two planes.

| `refs` | Line |
|---|---|
| one world axis, straight edge or sketch line | that line |
| one `{"kind": "face", "id": ...}` naming a cylindrical face | the axis of the cylinder; `faceNotCylindrical` if the face is another surface |
| two planes (standard, feature or face) | the line where they meet, `planesParallel` if they are parallel |
| anything else | `missingReference` |

An axis produces no entry in the reply; it is used by the features after it.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {"type": "axis", "id": "A1", "refs": [{"kind": "axis", "axis": "y"}]},
    {
      "type": "plane", "id": "P1", "mode": "angle", "base": {"kind": "std", "id": "front"},
      "axis": {"kind": "feature", "id": "A1"}, "angle": 45
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,0,0], x [0,1,0], y [-0.707106781,0,0.707106781], z [0.707106781,0,0.707106781].


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {"type": "plane", "id": "Q1", "mode": "offset", "base": {"kind": "std", "id": "front"}, "offset": 5},
    {
      "type": "axis", "id": "A1",
      "refs": [{"kind": "std", "id": "top"}, {"kind": "feature", "id": "Q1"}]
    },
    {
      "type": "plane", "id": "P1", "mode": "angle", "base": {"kind": "std", "id": "front"},
      "axis": {"kind": "feature", "id": "A1"}, "angle": 90
    }
  ]
}
```

Run result: no errors; no solid body. `P1`: o [0,0,5], x [1,0,0], y [0,0,1], z [0,-1,0]. `Q1`: o [0,0,5], x [1,0,0], y [0,1,0], z [0,0,1].


## See also

- [Overview of the part format](overview.md)
- [Sketches](sketches.md)
- [Identifiers and references](references.md)
- [Patterns and transforms](features-patterns.md)
