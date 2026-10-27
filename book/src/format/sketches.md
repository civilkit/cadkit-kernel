# Sketches

A sketch is a feature that holds 2D geometry on a plane: points, curves built on those points, and constraints between them. Solid features take their profiles from sketches. This chapter covers the fields of a sketch, the curve types, the constraint types, how closed profiles are found and what a regen returns. The solver behind the constraints has its own chapter, [the sketch solver](sketch-solver.md).

## Fields of a sketch

| Field | Type | Default | Meaning |
|---|---|---|---|
| `type` | string | required | `"sketch"`. |
| `id` | string | | Feature id, see [identifiers](references.md). |
| `plane` | plane reference | required | `{"kind": "std" \| "feature" \| "face", "id": ...}`, see [planes](planes.md). A missing or unresolved plane is `missingReference`. |
| `points` | array | `[]` | The points of the sketch. |
| `entities` | array | `[]` | The curves, built on points. |
| `constraints` | array | `[]` | The constraints. |
| `seq` | number | `0` | Counter an editor uses to mint ids inside the sketch. Not used by the kernel. |
| `visible` | boolean | `true` | Display flag. Not used by the kernel. |
| `name` | string | `""` | Display name. |

A point `(u, v)` of a sketch is the world point `o + u x + v y` of the frame of its plane. The reply returns that frame for every sketch, so a client does not have to compute it.

### Points

| Field | Type | Default | Meaning |
|---|---|---|---|
| `id` | string | `""` | Point id. Unique in the sketch; a repeated id resolves to the last point with it. |
| `x`, `y` | number | `0` | Coordinates in the plane, mm. A numeric string is converted; anything else is 0. Expressions are not evaluated here. |
| `fixed` | boolean | `false` | The solver does not move this point. |

Editors create every sketch with a fixed point `O` at the origin so constraints can refer to it. The kernel does not require it.

### Entities

Curves refer to points by id. Every entity has an `id` and a `type`; a `construction` flag (`true`) keeps a curve out of profiles and out of sweeps and lofts: it is helper geometry such as an axis line.

| `type` | Fields | Geometry |
|---|---|---|
| `line` | `p1`, `p2` | The segment between two points. |
| `circle` | `c`, `r` | Centre point `c`, radius `r` (number). `"ext": true` or `"from": true` marks a circle whose radius is driven by something else: the solver does not treat `r` as unknown. |
| `arc` | `c`, `p1`, `p2` | Counter-clockwise from `p1` to `p2` around `c`. The radius is the distance from `c` to `p1`; the solver keeps `p2` on the same circle. Swapping `p1` and `p2` gives the complementary arc. |
| `ellipse` | `c`, `p`, `r` | Centre `c`; `p` is a point on the major axis, so the major semi-axis is the distance `c`-`p`; `r` is the minor semi-axis. |
| `point` | `p` | A marker for a point. Used as hole centres and as a point section of a loft. |
| `spline` | `pts`, `closed` | A cubic curve interpolating the points `pts` in order. `closed` joins the last point to the first. |
| `cvspline` | `pts`, `degree`, `knots`, `closed` | A B-spline whose control points are `pts`. `degree` defaults to 1 when absent and is at least 1. `knots` is optional. |
| `text` | `string`, `height`, `at` or `p`, `font`, ... | Lettering whose outlines become closed profiles. See [text and fonts](../io/text.md). |

An entity with a missing or unknown `type` is dropped when the sketch is read, without an error. Missing string fields (`p1`, `c`, ...) read as `""`, which names no point.

### Constraints

A constraint has an `id`, a `type`, `refs` (ids of points or entities) and, for dimensions, a numeric `value`. The constraints are used by the solver (see [the sketch solver](sketch-solver.md)); the table lists what each type says.

| `type` | `refs` | Meaning |
|---|---|---|
| `coincident` | point, point | The points are equal. Circle, arc and ellipse ids stand for their centres, a point entity for its point. |
| `concentric` | point-like, point-like | Same as `coincident`, used for centres. |
| `pointOnLine` | point, line | The point lies on the infinite line through the segment. |
| `pointOnCircle` | point, circle/arc/ellipse | The point lies at the radius from the centre. |
| `midpoint` | point, line | The point is the middle of the segment. |
| `horizontal` | line, or point, point | The two ends have equal `y`. |
| `vertical` | line, or point, point | The two ends have equal `x`. |
| `parallel`, `perpendicular` | line, line | Directions are parallel / at 90 degrees. |
| `collinear` | line, line | Both lines lie on one line. |
| `equal` | line, line or circle/arc, circle/arc | Equal length, or equal radius. |
| `tangent` | line/arc/circle, arc/circle | Tangent. At a shared end point (`at` names it, or it is found) the curves meet tangentially; otherwise they touch: a line at distance `r`, two circles at distance `r1 + r2` (`"internal": true`: the absolute difference of the radii). |
| `fix` | point | The point is at `value: [x, y]`. `pierce` is the same. |
| `symmetric` | point, point, line | The points are mirror images about the line. |
| `distance` | point, point | Distance between the points equals `value`. |
| `hdist`, `vdist` | point, point | Absolute difference of `x` (`y`) equals `value`. |
| `pldist` | point, line | Distance from the point to the line equals `value`. |
| `radius`, `diameter` | circle/arc/ellipse | The radius (twice the radius) equals `value`. |
| `raddiff` | circle/arc, circle/arc | `sign * (r2 - r1)` equals `value`; `sign` is 1 or -1. |
| `angle` | line, line | The angle from the first to the second direction equals `value` degrees; `s1` and `s2` (1 or -1) flip the direction of each line. |
| `g1`, `g2`, `g3` | cvspline, curve | Continuity of an open control-point spline with another curve at the end `at` (a control point id). |
| `onCurve` | point, curve | The point lies at the parameter fraction `value` (0 to 1) of the curve. |
| `tension` | cvspline | Tension at the end `at` with strength `value`. |

Other fields of a constraint:

| Field | Meaning |
|---|---|
| `value` | The number the dimension is driven to. `[x, y]` for `fix`. A numeric string is converted. |
| `expr` | A variable expression that drives `value`, such as `"=w/2"`. See [variables and expressions](expressions.md). |
| `offset` | Where an editor draws the dimension. Kept, not used. |
| `suppressed` | `true` makes the solver ignore the constraint everywhere, while the document keeps it. |
| `at`, `internal`, `s1`, `s2`, `sign` | Options of `tangent`, `angle`, `raddiff` and the spline constraints, as above. |

A constraint that names a missing point or entity, or has too few `refs` or an unknown `type`, is kept in the document but makes the equation system impossible to build; the sketch is then left as it is, without an error.

## What the kernel does with a sketch

The coordinates in the document are authoritative. A regen does not re-solve the constraints of a sketch just because they exist: the points are used as they are written. The solver runs in two cases:

- a constraint has an `expr` and its value differs from the stored `value`: the kernel evaluates the expression, writes the new `value` and re-solves the sketch from the stored coordinates;
- the request has `opts.solveSketches` (see [the sketch solver](sketch-solver.md)).

In both cases the reply returns the solved sketch. If the solve fails the points stay as sent and no error is raised.

```json
{
  "format": "ck-part", "version": 1, "variables": [{"name": "w", "expr": "50"}],
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "O", "x": 0, "y": 0, "fixed": true},
        {"id": "b", "x": 30, "y": 0},
        {"id": "c", "x": 30, "y": 15},
        {"id": "d", "x": 0, "y": 15}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "O", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "O"}
      ],
      "constraints": [
        {"id": "k1", "type": "horizontal", "refs": ["e1"]},
        {"id": "k2", "type": "vertical", "refs": ["e2"]},
        {"id": "k3", "type": "horizontal", "refs": ["e3"]},
        {"id": "k4", "type": "vertical", "refs": ["e4"]},
        {"id": "k5", "type": "distance", "refs": ["O", "b"], "value": 30, "expr": "=w"},
        {"id": "k6", "type": "distance", "refs": ["b", "c"], "value": 15}
      ]
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 5}
  ]
}
```

Run result: no errors; 6 faces, volume 3750 mm³. The reply echoes the sketch with the solved points: `b` is at (50, 0) instead of (30, 0), and `k5` carries `value: 50`. The plate is 50 mm long.


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "O", "x": 0, "y": 0, "fixed": true},
        {"id": "b", "x": 30, "y": 1},
        {"id": "c", "x": 30, "y": 15},
        {"id": "d", "x": 0, "y": 15}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "O", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "O"}
      ],
      "constraints": [{"id": "k1", "type": "horizontal", "refs": ["e1"]}]
    }
  ]
}
```

Run result: no errors; no solid body. The point `b` comes back unchanged at (30, 1): the constraint is not applied because no `expr` changed.


An editor therefore writes solved coordinates; a program that edits a dimension by changing a variable gets them solved.

Regen errors of a sketch feature: `missingReference` (the plane), `badExpression` (a constraint `expr` that does not evaluate), and the lettering errors `textEmpty`, `textNothingToDraw`, `textFontNotLoaded`, `textFontMismatch`, `textBadHeight`, `textBadArc`, `textBadValue` (see [text and fonts](../io/text.md)).

## Profiles

Solid and surface features take their profiles from the `profiles` the kernel finds in a sketch. The rules:

- All curves that are not construction curves are intersected with each other and split at the crossings. Pieces that dangle (that do not belong to a closed loop) are dropped, so a line that crosses a rectangle divides it in two and its overhanging ends vanish.
- Exact duplicates of a piece are dropped. Collinear overlaps are not split.
- Every bounded face of the resulting planar arrangement is a region. A loop inside a region is a hole of it; a loop inside a hole is an island and a region of its own (concentric circles give a ring and a disc).
- A circle or ellipse is one closed loop, a closed spline one loop. A sketch with no closed loop but at least one curve is `open`; most solid features then report `noClosedProfile`.
- Regions are listed in order of their first curve; each `outer` loop runs counter-clockwise and `holes` clockwise.

The reply gives each sketch as `[id, {frame, profiles, sketch}]`, with `profiles = {regions: [{outer: [...], holes: [[...]]}], open: bool}`. A segment is `{kind: "line" | "arc" | "spline", tag: <entity id>, a: [x, y], b: [x, y]}`; circles and ellipses are arcs from angle 0 to 2 pi.

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
        {"id": "h", "x": 20, "y": 10}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"},
        {"type": "circle", "id": "e5", "c": "h", "r": 4}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 4}
  ]
}
```

Run result: no errors; 7 faces, volume 2998.9381 mm³. `profiles` in the reply: 1 region with 1 hole, `open` is false. The faces are `F2.C0`, `F2.C1`, `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`, `F2.S.e5`.


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
        {"id": "x", "x": 20, "y": -5},
        {"id": "y", "x": 20, "y": 25}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "a"},
        {"type": "line", "id": "e5", "p1": "x", "p2": "y"}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 2}
  ]
}
```

Run result: no errors; 12 faces, volume 1600 mm³. The line crosses the rectangle, so the profile has 2 regions; the part of the line outside the rectangle is dropped. The faces are `F2.C0~1`, `F2.C1~1`, `F2.S.e1~1`, `F2.S.e5~1`, `F2.S.e3~1`, `F2.S.e4`, `F2.C0~2`, `F2.C1~2`, `F2.S.e1~2`, `F2.S.e2`, `F2.S.e3~2`, `F2.S.e5~2`.


An extrude, revolve or loft uses all regions unless `regions` selects some; see [solids from sketches](features-solids.md).

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
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 4}
  ]
}
```

Run result: errors `F2`: `noClosedProfile`; no solid body.


Curves: the arc runs counter-clockwise from `p1` to `p2`.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "c", "x": 0, "y": 0},
        {"id": "a", "x": 10, "y": 0},
        {"id": "b", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "arc", "id": "e1", "c": "c", "p1": "a", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "a"}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 2}
  ]
}
```

Run result: no errors; 4 faces, volume 57.0796 mm³. The cross-section is the segment between the chord and the quarter arc, area 28.5398 mm².


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "c", "x": 0, "y": 0},
        {"id": "a", "x": 10, "y": 0},
        {"id": "b", "x": 0, "y": 10}
      ],
      "entities": [
        {"type": "arc", "id": "e1", "c": "c", "p1": "b", "p2": "a"},
        {"type": "line", "id": "e2", "p1": "a", "p2": "b"}
      ],
      "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 2}
  ]
}
```

Run result: no errors; 4 faces, volume 571.2389 mm³. With the arc end points swapped the arc runs the long way round: area 285.6194 mm².


## The echoed sketch

The `sketch` object in the reply is the input after normalisation. Key order and unknown keys are kept. Missing `points`, `entities` and `constraints` arrays are appended; a point gets `id`, `x` and `y` defaults and its `x` and `y` are converted to numbers; a non-object point becomes `{"id": "", "x": 0, "y": 0}`; entities of an unknown type are dropped; a numeric-string constraint `value` becomes a number. The solved coordinates replace the stored ones.

## 3D sketches

A `sketch3d` feature holds points with `x`, `y`, `z`, lines, arcs, splines and control-point splines in space, and its own constraints. It is used as a sweep path and as a path for `normal` planes. Its fields and reply are described in [surfaces and curves](features-surfaces.md#3d-sketches).

## See also

- [The sketch solver](sketch-solver.md)
- [Planes](planes.md)
- [Variables and expressions](expressions.md)
- [Solids from sketches](features-solids.md)
- [Text and fonts](../io/text.md)
