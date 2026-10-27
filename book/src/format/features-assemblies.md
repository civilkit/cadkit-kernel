# Parts and assemblies

An assembly is a part whose features insert other parts as components and constrain them with mates. This chapter covers `insertPart`, the placement of a component, the `mate` constraints and their kinds, the degrees-of-freedom report, and how a component pattern copies components.

An assembly document is an ordinary part; the top-level field `"kind": "assembly"` marks it for editors but the kernel does not need it.

## insertPart

Puts a copy of another part in the assembly.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `part` | part document | required | The part, embedded in full: a document with `format`, `features` and the other [top-level fields](overview.md). If it is missing or cannot be read (it fails the [format check](overview.md#what-is-checked-when-a-document-is-read)) the feature is `insertPartMissing`. |
| `place` | `{o, x, y, z}` | none | The placement frame: four vectors of three plain numbers. The component's world origin maps to `o` and its axes to `x`, `y`, `z`. |
| `dx`, `dy`, `dz` | number | `0` | A translation, used when there is no `place`. Plain numbers; expressions are not evaluated here. |
| `axis` | `{"kind": "axis", "axis": "x"}` | none | Rotation axis for the `angle` form: a world axis through the origin. |
| `angle` | number or expression | `0` | Degrees about `axis`. The rotation is applied first, then the translation `dx, dy, dz`. |
| `fixed` | boolean | `false` | A fixed component never moves: its degrees of freedom are 0, and mates move the other component. |
| `source` | string | none | A label for where the part came from. Read and kept, not used by the kernel. |

The embedded part is regenerated on its own, with its own `rollback` ignored, and its body becomes the component. Two inserts of the same part document regenerate it once. Components are appended to the body as separate bodies; they are never united, so components may overlap.

Every face of a component is renamed with the id of the insert feature: `E1.C0` in the part becomes `E1.C0@F1.1` for the insert `F1`. Names the part already had after `@` stay in front (`F8.S.h1@F9.1@F1.1`). These are the names mates and later features use.

The regen reply lists the components in `components`, sorted by id: `{id, dof, place}`, where `dof` is 6 for a free component and 0 for a fixed one (mates lower it) and `place` is the frame the component now has.

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 50, "dy": 0, "dz": 0, "axis": {"kind": "axis", "axis": "y"}, "angle": 90
    }
  ]
}
```

Run result: no errors; 6 faces, volume 3141.5927 mm³. Components in the reply: `F1` dof 0, origin [0,0,0]; `F2` dof 6, origin [50,0,0]. The faces of the body are `E1.C0@F1.1`, `E1.C1@F1.1`, `E1.S.e1@F1.1`, `E1.C0@F2.1`, `E1.C1@F2.1`, `E1.S.e1@F2.1`. The volume is the sum of the two pins (3141.5927); nothing is united.


A placement frame instead of `dx, dy, dz`:

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "place": {"o": [0, 0, 50], "x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}
    }
  ]
}
```

Run result: no errors; 3 faces, volume 1570.7963 mm³. Component: `F1` dof 6, origin [0,0,50].


Errors and edge cases:

| Situation | Result |
|---|---|
| No `part`, or an unreadable one | `insertPartMissing`. |
| The part reports errors when regenerated | `insertPartErrors` with `params.detail` set to `"<feature id>: <code>"` of the first error (by feature id). The part is not inserted. |
| The part has no features, or builds no solid body | No error and no component. |

```json
{"format": "ck-part", "version": 1, "features": [{"type": "insertPart", "id": "F1"}]}
```

Run result: errors `F1`: `insertPartMissing`; no solid body.


```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S9", "end": "blind", "depth": 5}
        ]
      }
    }
  ]
}
```

Run result: errors `F1`: `insertPartErrors` `{"detail":"E1: missingSketch"}`; no solid body.


## mate

A mate constrains two components by one face on each. The mate moves the component of `a` onto the component of `b`; when that component is `fixed`, the other one moves instead. Mates are solved in order and each later mate keeps the earlier ones satisfied where it can. The solution is iterative (a damped Gauss-Newton on the placement of the moving component), so positions are accurate to about 1e-9 rather than exact.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `kind` | string | `"coincident"` | See the table of kinds. |
| `a` | face name | required | A face of the moving component, in its component name (`E1.C0@F2.1`). |
| `b` | face name | required | A face of the other component. |
| `distance` | number or expression | `0` | Offset for `distance`, the stop offset of `hinge`. |
| `flip` | boolean | `false` | Reverses the contact side or the direction, depending on the kind. |
| `a2`, `b2` | face names | none | Second faces (width, hinge stop, slot wall). |
| `c` | face name | none | The mirror face of `symmetric`. |
| `angle` | number or expression | kind dependent | Degrees: the angle of an `angle` mate, or the drive of `gear`, `rackPinion` and `screw`. |
| `ratio`, `pitch`, `radius` | number or expression | `1`, `1`, none | Gear ratio, screw pitch (length per turn), pinion pitch radius. |
| `min`, `max`, `of` | | | `limits`: the range, and `of: "distance"` (default) or `"angle"`. |
| `dir` | `[x, y, z]` | none | Travel direction of a planar rack. |

Kinds. "Leaves" is the freedom a free component keeps when mated once to a fixed one (an observed value for the examples run; a second mate removes more):

| `kind` | Faces | Constraint | Leaves |
|---|---|---|---|
| `coincident` | plane, plane | Normals opposed (or aligned with `flip`), the planes touch. | 3 |
| `distance` | plane, plane | As `coincident`, at `distance`. | 3 |
| `parallel` | plane or axis pairs | Directions parallel. | 4 |
| `concentric` | cylinder, cylinder | Axes on one line. | 2 |
| `angle` | plane, cylinder or cone | The angle between the two directions equals `angle`. | 5 |
| `perpendicular` | plane or axis pairs | Directions at 90 degrees. | 5 |
| `tangent` | cylinder or sphere against plane; cylinder or sphere pairs | Contact. `flip` takes the other side of a plane, or internal contact between round faces. | 4 for a cylinder on a plane |
| `lock` | any face of each | The relative pose at the time of the mate is kept. | 0 |
| `symmetric` | plane/plane, cylinder/cylinder, sphere/sphere, and `c` | `a` is the mirror image of `b` about `c` (the component of `c` is treated as fixed). | 3 for a plane pair |
| `width` | `a`, `a2` parallel planes of a tab; `b`, `b2` of a slot | The tab centred in the slot. | 3 |
| `hinge` | cylinders or cones; optional `a2`, `b2` stop planes | Concentric axes, and the stop faces in contact at `distance`. | 1 turn (2 without the stop) |
| `slot` | `a` a pin (cylinder); `b` (and `b2`) planar slot walls | Pin parallel to the walls and centred between them. | 4 |
| `screw` | cylinders (screw, nut) | Concentric, advance = `pitch` x turns; with `angle` the nut is placed that far along the thread. | 1; 0 with `angle` |
| `gear` | cylinders | The follower turns by `-angle / ratio` about its axis for a drive `angle` of `a`, measured from the pose at the time of the mate. | one coupling |
| `rackPinion` | pinion cylinder, rack cylinder or plane with `dir` | Rack travel = `radius` x pinion angle. | one coupling |
| `limits` | two planes, or two directions with `of: "angle"` | The measured value is clamped into `[min, max]`; inside the range the mate removes no freedom. | 6 inside the range |

Couplings (`gear`, `rackPinion`, `screw`) are measured from the placements the components have when the mate is made (after earlier mates were solved), so a later mate that moves the driver drags the follower. The follower is the component of `b`; if that one is fixed, `gear` and `screw` swap the roles, and `rackPinion` is refused with `mateFixed`. Not supported (`mateGeometry`): `symmetric` with `c` on a moving component, `tangent` with a cylinder in `a` and a sphere in `b`, a rack that is neither a cylinder nor a plane with `dir`. An unknown `kind` is `mateGeometry`.

A cap against a cap. `F2` slides on the cap of `F1` (it keeps three freedoms: two translations along the plane and one rotation about the normal):

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 100
    },
    {"type": "mate", "id": "F3", "kind": "coincident", "a": "E1.C0@F2.1", "b": "E1.C1@F1.1"}
  ]
}
```

Run result: no errors; 6 faces, volume 3141.5927 mm³. The cap of `F2` lies on the cap of `F1`: `F1` dof 0, origin [0,0,0]; `F2` dof 3, origin [100,0,19.999999998] (z = 20). It can still slide in x and y and turn about z, so it keeps 3 of its 6 freedoms. The position is converged to about 1e-9, not exact.


Axis on axis, leaving a slide and a turn along the axis:

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 100
    },
    {"type": "mate", "id": "F3", "kind": "concentric", "a": "E1.S.e1@F2.1", "b": "E1.S.e1@F1.1"}
  ]
}
```

Run result: no errors; 6 faces, volume 3141.5927 mm³. The axis of `F2` is on the axis of `F1`: `F1` dof 0, origin [0,0,0]; `F2` dof 2, origin [0,0,0].


A hinge with a stop face leaves one turn:

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 100
    },
    {
      "type": "mate", "id": "F3", "kind": "hinge", "a": "E1.S.e1@F2.1", "b": "E1.S.e1@F1.1",
      "a2": "E1.C0@F2.1", "b2": "E1.C1@F1.1"
    }
  ]
}
```

Run result: no errors; 6 faces, volume 3141.5927 mm³. `F2` sits on top of `F1` on the same axis and can only turn: the `assembly` object says {"total":1,"translational":0,"rotational":1}.


### Errors

| Code | When |
|---|---|
| `missingReference` | A face name does not exist (or `b` is empty). |
| `mateSameBody` | Both faces belong to one component. |
| `mateFixed` | Both components are fixed (and for `rackPinion` a fixed rack). |
| `mateGeometry {detail}` | The kind cannot use the geometry of these faces, or the kind is unknown. |
| `mateConflict` | The solver cannot satisfy this mate together with the earlier ones. |
| `nothingToCut` | The document has no body yet. |

A rejected mate leaves every component where the earlier mates put it, and the other mates still run.

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true, "dx": 100
    },
    {"type": "mate", "id": "F3", "kind": "coincident", "a": "E1.C0@F2.1", "b": "E1.C1@F1.1"}
  ]
}
```

Run result: errors `F3`: `mateFixed`; 6 faces, volume 3141.5927 mm³.


```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "fixed": true
    },
    {
      "type": "insertPart", "id": "F2",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 100
    },
    {
      "type": "mate", "id": "F3", "kind": "distance", "a": "E1.C0@F2.1", "b": "E1.C1@F1.1",
      "distance": 5
    },
    {
      "type": "mate", "id": "F4", "kind": "distance", "a": "E1.C0@F2.1", "b": "E1.C1@F1.1",
      "distance": 9
    }
  ]
}
```

Run result: errors `F4`: `mateConflict`; 6 faces, volume 3141.5927 mm³. The second mate is rejected and `F2` stays where the first put it (z = 25): `F1` dof 0, origin [0,0,0]; `F2` dof 3, origin [100,0,24.999999998].


## The degrees-of-freedom report

When the document uses a mate kind beyond `coincident`, `concentric`, `distance` and `parallel`, or the `regen` request has `opts.dofReport: true`, the reply carries an `assembly` object next to `components`. This is the report of the hinge example above:

```json
{"assembly": {
  "dof": {"total": 1, "translational": 0, "rotational": 1},
  "components": [{"id": "F1", "fixed": true, "dof": 0, "translational": 0, "rotational": 0},
                 {"id": "F2", "fixed": false, "dof": 1, "translational": 0, "rotational": 1}],
  "overConstrained": false,
  "redundant": [],
  "conflicts": []
}}
```

- `dof.total` is the dimension of the motion space of all floating components left by all mates together. `rotational` is the rank of the rotation part, `translational` the rest. Each component's own numbers are the motions it can still make; fixed components report 0.
- `redundant` lists mates that add no constraint (their rows depend on the earlier mates). They are harmless and make `overConstrained` true.
- `conflicts` lists each rejected mate as `{"mate": id, "with": [earlier mates on the same components]}`, with the matching `mateConflict` error.
- The rank comes from a numeric Jacobian of the mate residuals at the solved pose, so a nearly singular pose can be misreported. With more than 30 floating components the null-space part is skipped and the object carries `"truncated": true`.

The same number appears as `components[].dof` once an added kind is present.

## Component patterns

A `cpattern` with an empty `seeds` list and `bodies` set copies whole components around an axis (see [patterns and transforms](features-patterns.md)). `bodies` is a list of `{"face": "<component face>"}`: every component that owns one of the faces is copied.

| Field | Meaning |
|---|---|
| `bodies` | `[{"face": "E1.C0@F1.1"}]`: a face of each component to copy. |
| `axis`, `count`, `angle`, `equal`, `reverse` | As for a circular pattern. |

The copies are new bodies named with the pattern id after the component suffix (`E1.C0@F1.1@F2.1`); they are not listed in `components`.

```json
{
  "format": "ck-part", "version": 1, "kind": "assembly",
  "features": [
    {
      "type": "insertPart", "id": "F1",
      "part": {
        "format": "ck-part", "version": 1,
        "features": [
          {
            "type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"},
            "points": [{"id": "c", "x": 0, "y": 0}],
            "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 5}], "constraints": []
          },
          {"type": "extrude", "id": "E1", "op": "add", "sketch": "S1", "end": "blind", "depth": 20}
        ]
      },
      "dx": 30
    },
    {
      "type": "cpattern", "id": "F2", "seeds": [], "bodies": [{"face": "E1.C0@F1.1"}],
      "axis": {"kind": "axis", "axis": "z"}, "count": 3, "angle": 360
    }
  ]
}
```

Run result: no errors; 9 faces, volume 4712.389 mm³. Three pins around the z axis. The copies are named `E1.C0@F1.1@F2.1`, `E1.C1@F1.1@F2.1`, ... and are not listed in `components` (1 component).


## See also

- [Overview of the part format](overview.md)
- [Patterns and transforms](features-patterns.md)
- [Identifiers and references](references.md)
- [Mass properties and interference](../validation/analysis.md)
