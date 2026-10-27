# Patterns and transforms

These features copy, reflect, move and scale what earlier features made: `lpattern`, `cpattern`, `mirror`, `moveBody`, `scale`, and `combine`, which applies a boolean between whole bodies. The examples use a 60 x 20 x 10 mm bar with a pocket of radius 3 at x = 10 (feature `F4`) or the plate of [the overview](overview.md).

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 0, "y": 0},
        {"id": "b", "x": 60, "y": 0},
        {"id": "c", "x": 60, "y": 20},
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
      "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
      "points": [{"id": "c", "x": 10, "y": 10}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 3}], "constraints": []
    },
    {"type": "extrude", "id": "F4", "op": "cut", "sketch": "F3", "end": "blind", "depth": 4}
  ]
}
```


## Seeds and copies

`lpattern`, `cpattern` and `mirror` copy **features**, not faces. A seed is the id of an earlier feature that made a tool solid: an `extrude`, `revolve`, `sphere`, `sweep`, `loft` or `hole`, or an earlier pattern. The copies of the tool are combined with the body in the same way as the seed was: copies of a cut are cut, copies of a boss are added. A copy that falls outside the body does nothing and is not an error. A seed that is not a tool-making feature, or does not exist, is `missingReference`; an empty `seeds` list is `noSeeds`.

The faces of a copy carry the id of the pattern and the number of the copy: `F4.C0@F5.1`, `F4.S.e1@F5.2`. The copies are numbered from 1; the seed itself keeps its name. A pattern of a pattern chains the suffixes.

Threads on the seed's faces are copied to the copies too: see [edge and face edits](features-edits.md#thread). All three pattern features take `carryThreads` (default `true`) to switch this off; a `thread` feature id can also be listed in `seeds` to copy that thread alone.

## lpattern

Copies the seeds along a direction.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `seeds` | list of feature ids | required | See above. |
| `dir` | `{"kind": "axis", "axis": "x"}` | required | A world axis (`x`, `y` or `z`). Any other reference is `missingReference`. |
| `spacing` | number or expression | `0` | Distance between copies, mm. |
| `count` | number or expression | `0` | Number of instances, **including the seed**. A value of 1 or less makes no copies and is not an error; a fractional count is rounded; more than 10000 is `invalidValue`. |
| `reverse` | boolean | `false` | Copy towards the negative direction. |

Copy `k` (from 1) is the seed moved by `k * spacing` along `dir`.

```json
{
  "type": "lpattern", "id": "F5", "seeds": ["F4"], "dir": {"kind": "axis", "axis": "x"}, "spacing": 15,
  "count": 3
}
```

Run result: no errors; 12 faces, volume 11660.708 mm³. Three pockets in a row: 10, 25 and 40 mm from the left edge. The pocket faces are `F4.C0`, `F4.S.e1`, `F4.C0@F5.1`, `F4.S.e1@F5.1`, `F4.C0@F5.2`, `F4.S.e1@F5.2`.


```json
{
  "type": "lpattern", "id": "F5", "seeds": ["F4"], "dir": {"kind": "axis", "axis": "x"}, "spacing": 15,
  "count": 3, "reverse": true
}
```

Run result: no errors; 8 faces, volume 11886.9027 mm³. The copies go to negative x, outside the bar: no copy touches the body and there is no error.


```json
{
  "type": "lpattern", "id": "F5", "seeds": ["F9"], "dir": {"kind": "axis", "axis": "x"}, "spacing": 15,
  "count": 3
}
```

Run result: errors `F5`: `missingReference`; 8 faces, volume 11886.9027 mm³.


## cpattern

Copies the seeds around an axis.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `seeds` | list of feature ids | required | See above. For assemblies the list may be empty and `bodies` used, see [parts and assemblies](features-assemblies.md). |
| `axis` | `{"kind": "axis", "axis": "z"}` | required | A world axis through the origin. |
| `count` | number or expression | `0` | Number of instances including the seed. |
| `angle` | number or expression | `360` | Degrees. With `equal` it is the total angle; otherwise the step. |
| `equal` | boolean | `true` | `true`: copies are spread evenly: the step is `angle / count` for a full turn (360) and `angle / (count - 1)` otherwise, so the last copy lies at `angle`. `false`: `angle` is the step between neighbours. |
| `reverse` | boolean | `false` | Turn the other way. |

The base is a disc of radius 30 and depth 10 with one through hole of radius 3 at x = 20 (`F4`):

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 30}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10},
    {
      "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
      "points": [{"id": "c", "x": 20, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 3}], "constraints": []
    },
    {"type": "extrude", "id": "F4", "op": "cut", "sketch": "F3", "end": "blind", "depth": 10}
  ]
}
```


Appended to it:

```json
{
  "type": "cpattern", "id": "F5", "seeds": ["F4"], "axis": {"kind": "axis", "axis": "z"}, "count": 4,
  "angle": 360
}
```

Run result: no errors; 7 faces, volume 27143.3605 mm³. Four holes 90 degrees apart (the seed and three copies): volume pi 30² 10 - 4 pi 3² 10 = 27143.3605.


Appended to the same disc document:

```json
{
  "type": "cpattern", "id": "F5", "seeds": ["F4"], "axis": {"kind": "axis", "axis": "z"}, "count": 3,
  "angle": 30, "equal": false
}
```

Run result: no errors; 6 faces, volume 27426.1039 mm³. With `equal: false` the angle is the step: copies at 30 and 60 degrees.


## mirror

Reflects seeds, or the whole body, about a plane.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `seeds` | list of feature ids | required unless `body` | The features to reflect. Each copy is instance 1 of the mirror (`F4.C0@F5.1`); a mirrored cap keeps its cap number. |
| `plane` | plane reference | required | A standard plane, a plane feature or a planar face. A missing plane is `missingReference`. |
| `body` | boolean | `false` | Reflect the whole body made so far and unite it with the body (seeds are then ignored). The copy's faces get `@<mirror id>.1`. A copy clear of the body is added as a separate body without a boolean. |

```json
[
  {"type": "plane", "id": "P1", "mode": "offset", "base": {"kind": "std", "id": "right"}, "offset": 30},
  {"type": "mirror", "id": "F5", "seeds": ["F4"], "plane": {"kind": "feature", "id": "P1"}}
]
```

Run result: no errors; 10 faces, volume 11773.8053 mm³. The pocket at x = 10 is mirrored about the plane x = 30 to x = 50: volume 11773.8053. The copy's faces are `F4.C0@F5.1`, `F4.S.e1@F5.1`.


```json
{"type": "mirror", "id": "F5", "body": true, "seeds": [], "plane": {"kind": "std", "id": "right"}}
```

Run result: no errors; 10 faces, volume 23773.8053 mm³. The whole body is reflected about x = 0 and united with itself: box [-60,0,0] to [60,20,10]; volume 23773.8053 = 2 x 11886.9027.


## moveBody

Moves, rotates or copies whole bodies.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `bodies` | list of selectors | all bodies | Which bodies. `target` is accepted for `bodies`, and `body` when it is an object or array (or a string while `copy` is not true). |
| `translate` | `[x, y, z]` | none | Translation; components may be expressions. |
| `rotate` | object | none | `{"axis": [x, y, z], "origin": [x, y, z], "angle": degrees}`. `axis` can also be `"x"`, `"y"`, `"z"` or `{"kind": "axis", "axis": "x"}`; `origin` defaults to the world origin. |
| `copy` | boolean | `false` | Keep the source body and add a moved copy; its face names get `@<id>.1`. |
| `body` | string | none | With `copy: true`: the name of the copies (`bodyNameConflict` if taken). |
| `mirror` | plane reference | none | With `copy: true`: the copy is the mirror image about this plane (solid bodies only). |

Body selectors: `{"body": "Block"}` (every body with that name), `{"face": "F2.C1"}` or the bare face name (the body that holds the face), `{"feature": "F4"}` or a bare id without a dot (every body holding a face made by that feature, including copies), `{"surface": "F7"}` (a surface body), `{"lump": 2}` (by index). A selector that finds nothing is `missingReference` (with `name` and `nearest` for `{body}`).

The rotation is applied first, then the translation. Without `copy` the bodies move in place and all names stay; moved bodies are not united with what they touch, so the result may hold overlapping bodies (use `combine` afterwards). No motion at all is a no-op. The older keys `dx`, `dy`, `dz` with `axis` and `angle` still read.

Errors: `nothingToCut` (no body), `missingReference`, `invalidValue {detail}` (a vector without three components, a non-finite number, a zero rotation axis), `bodyNameConflict`.

```json
{
  "type": "moveBody", "id": "F5", "translate": [100, 0, 0],
  "rotate": {"axis": [0, 0, 1], "origin": [0, 0, 0], "angle": 90}
}
```

Run result: no errors; 8 faces, volume 11886.9027 mm³. Rotated 90 degrees about z first, then moved 100 in x: box [80,0,0] to [100,60,10]. The names are unchanged.


```json
{"type": "moveBody", "id": "F5", "copy": true, "body": "Copy", "translate": [0, 30, 0]}
```

Run result: no errors; 16 faces, volume 23773.8053 mm³. The copy sits 30 mm further in y and its faces carry `@F5.1`: `F2.C0@F5.1`, `F2.S.e1@F5.1`, `F2.S.e2@F5.1`, ...


## scale

Scales bodies about a point, uniformly or per axis.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `factor` | number, expression or `[sx, sy, sz]` | required | The factors along the world axes. `factors` is a synonym. Each must be finite and above 0. |
| `at` | `[x, y, z]` | none | The fixed point (`origin` is a synonym). |
| `about` | string | `"centroid"` | Without `at`: `"origin"` or `"centroid"` (the centre of mass of the selection). |
| `bodies` | list of selectors | all bodies | As for `moveBody` (`body` and `target` are read too). |

The bodies change in place and every face keeps its name. The result is exact for planes, for cylinders and cones stretched along their axis, for B-spline faces and for spheres and tori under a uniform factor. What cannot be kept exact is refused with `invalidValue {detail, hint}` and the body is left as it was: round faces stretched across their axis, a sphere or torus under a non-uniform factor, and revolution, extrusion, screw and offset surfaces. A factor of 0 or less is refused with `invalidValue {detail: "a scale factor must be above zero"}`; for a negative factor the `hint` suggests `mirror`, but the same hint is also given for 0, where it does not apply.

On the plate of [the overview](overview.md):

```json
{"type": "scale", "id": "F5", "factor": [1, 2, 0.5], "about": "origin"}
```

Run result: no errors; 6 faces, volume 8000 mm³. box [0,0,0] to [40,40,5]; volume 8000 = 8000 x 1 x 2 x 0.5.


A bar with a round pocket cannot be stretched across the pocket axis:

```json
{"type": "scale", "id": "F5", "factor": [1, 2, 0.5], "about": "origin"}
```

Run result: errors `F5`: `invalidValue` `{"detail":"a round face (cylinder or cone) would be stretched across its axis","hint":"scale it by one factor on all three axes, or only along the axis of the round faces (a cylinder stretched along its own axis stays exact)"}`; 8 faces, volume 11886.9027 mm³.


```json
{"type": "scale", "id": "F5", "factor": -1}
```

Run result: errors `F5`: `invalidValue` `{"detail":"a scale factor must be above zero","hint":"a negative factor would mirror the body: use mirror"}`; 6 faces, volume 8000 mm³.


## combine

A boolean between whole bodies of one part. `bodyBoolean` is accepted as a second name for the feature type.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `op` | string | required | `union` (`add`, `join`), `cut` (`subtract`), `intersect` (`common`) or `split`. Anything else is `invalidValue`. |
| `target` | selector | required | Exactly one body (`a` is a synonym): `{"face": ..}`, `{"lump": n}` or `{"body": name}`. Several bodies with the name are `ambiguousBody`. |
| `tools` | list of selectors | required | One or more bodies (`tool` or `b`). A tool may not be the target. |
| `keepTools` | boolean | `false` | Keep the tool bodies; otherwise they are consumed. |
| `body` | string | the target's name | Name of the result body. |
| `tolerance`, `autoFuzzy` | | | See [features](features.md#fields-every-feature-reads). |

`union` joins the target with each tool in turn, `cut` subtracts them, `intersect` keeps what lies inside all, `split` cuts the target into the part outside all tools and the part inside them (two bodies). Faces keep their names; a face divided by the operation gets `~k`. The bodies that take no part come first, then the result.

Errors: `nothingToCut` (no body), `missingReference`, `ambiguousBody`, `invalidValue {detail}`, `bodyNameConflict`, `booleanFailed`, `booleanVolumeMismatch`, `cutNoEffect` (a cut or split changed nothing), `combineEmpty` (an intersect of bodies that do not overlap).

A disc and a second body `Tool` made with `newBody`, overlapping it:

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
      "type": "sketch", "id": "F3", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "a", "x": 10, "y": 10},
        {"id": "b", "x": 30, "y": 10},
        {"id": "c", "x": 30, "y": 30},
        {"id": "d", "x": 10, "y": 30}
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
      "type": "extrude", "id": "F4", "op": "newBody", "body": "Tool", "sketch": "F3", "end": "blind",
      "depth": 10
    }
  ]
}
```


```json
{"type": "combine", "id": "F5", "op": "cut", "target": {"face": "F2.C1"}, "tools": [{"body": "Tool"}]}
```

Run result: no errors; 5 faces, volume 12251.2239 mm³. The tool is subtracted from the disc and consumed; volume 12251.2239.


```json
{
  "type": "combine", "id": "F5", "op": "intersect", "target": {"face": "F2.C1"},
  "tools": [{"body": "Tool"}]
}
```

Run result: no errors; 5 faces, volume 315.1467 mm³. Only the overlap of disc and tool is left.


## Not built

The feature types `curvepattern`, `fillpattern` and `spattern` are accepted as names (they are not `unknownFeature`) but are not implemented: they fail with the error code `TODO(rebuild): <type> needs ck-ops`, except that `spattern` without a `sketch` that names a regenerated sketch reports `missingSketch` first.

## See also

- [Edge and face edits](features-edits.md)
- [Parts and assemblies](features-assemblies.md)
- [Identifiers and references](references.md)
- [Planes](planes.md)
