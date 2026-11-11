# A first part

This chapter builds a 100 x 60 x 10 mm plate, regenerates it, adds a hole, validates the result and reads its mass properties. Every reply shown was produced by the kernel through `book/tools/run.mjs`; long replies are trimmed with `...`. Lengths are millimetres.

To follow along, save each message list in a file (here `first.json`) and run

```sh
node book/tools/run.mjs target/wasm32-unknown-unknown/release/ck_wasm.wasm first.json
```

The runner sends the messages in order to one kernel instance, so a later message can refer to the tag of an earlier `regen`.

## The plate

A part is a JSON document: a format name, a version, and a list of features. Each feature has a `type` and an `id`; later features refer to earlier ones by id. The plate needs two features: a sketch with four lines, and an extrude that gives the sketch a thickness.

```json
{
  "format": "ck-part", "version": 1, "name": "Plate",
  "features": [
    { "type": "sketch", "id": "F1", "plane": { "kind": "std", "id": "front" },
      "points": [ { "id": "p1", "x": 0, "y": 0 },   { "id": "p2", "x": 100, "y": 0 },
                  { "id": "p3", "x": 100, "y": 60 }, { "id": "p4", "x": 0, "y": 60 } ],
      "entities": [ { "id": "e1", "type": "line", "p1": "p1", "p2": "p2" },
                    { "id": "e2", "type": "line", "p1": "p2", "p2": "p3" },
                    { "id": "e3", "type": "line", "p1": "p3", "p2": "p4" },
                    { "id": "e4", "type": "line", "p1": "p4", "p2": "p1" } ],
      "constraints": [] },
    { "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10 }
  ]
}
```

The sketch lies on the standard plane `front`, whose normal is +Z. Its points are given by coordinates in the plane; its entities connect points by id. The extrude adds material (`"op": "add"`) from that sketch for a blind depth of 10 along the sketch normal. The [part format](../format/overview.md) chapters describe every field.

To regenerate it, send a `regen` message. `tag` names the state the kernel keeps for this document; every later query names the same tag.

```json
{ "type": "regen", "tag": "plate", "part": { "format": "ck-part", "...": "the document above" } }
```

The reply, trimmed:

```json
{
  "ok": true,
  "result": {
    "errors": [],
    "components": [], "planes": [], "markers": [],
    "sketches": [ [ "F1", { "frame": { "o": [0,0,0], "x": [1,0,0], "y": [0,1,0], "z": [0,0,1] },
                            "profiles": { "regions": [ { "outer": [ ... 4 lines ... ], "holes": [] } ], "open": false },
                            "sketch": { ... the sketch as the kernel read it ... } } ] ],
    "sketches3": [], "curves": [],
    "body": {
      "box": { "min": [0, 0, 0], "max": [100, 60, 10] },
      "faces": [
        { "id": "F2.C0", "kind": "plane", "planar": true, "meta": { "feature": "F2" }, "normal": [0, 0, -1] },
        { "id": "F2.C1", "kind": "plane", "planar": true, "meta": { "feature": "F2" }, "normal": [0, 0, 1] },
        { "id": "F2.S.e1", "kind": "plane", "planar": true, "meta": { "feature": "F2", "entity": "e1" }, "normal": [0, -1, 0] },
        ...
      ],
      "edges": [ { "pts": [[0,0,0],[0,0,10]], "faces": ["F2.S.e1","F2.S.e4"], "key": "F2.S.e1|F2.S.e4#0", "mid": [0,0,5], "line": true }, ... ],
      "silhouette": "<buffer f32 x0>",
      "mesh": { "positions": "<buffer f32 x108>", "normals": "<buffer f32 x108>", "mids": "<buffer f32 x108>", "triFace": "<buffer i32 x12>" }
    },
    "surfaces": []
  },
  "ms": 0
}
```

(The runner prints each binary buffer as a `<buffer type xN>` summary; a real client gets typed arrays, see [Replies and buffers](../protocol/reply.md).) What the reply says:

- `errors` is empty: both features ran.
- `body` is the solid: its bounding `box` is the 100 x 60 x 10 box, and it has six faces and twelve edges.
- The face ids show where each face came from. `F2.C0` and `F2.C1` are the two caps of feature `F2` (bottom, normal -Z; top, normal +Z). `F2.S.e1` to `F2.S.e4` are the side faces made from the sketch entities `e1` to `e4`. These names are stable: they are what later features and queries use to refer to a face ([Persistent names](../kernel/naming.md)).
- The mesh has 12 triangles (`triFace` has 12 entries; `positions` holds 9 numbers per triangle).

## Adding a hole

A hole is two more features: a sketch that holds the hole centre, placed on the top face of the plate, and a `hole` feature that cuts into the face from that sketch. The sketch plane is given as a face id, so the sketch sits on `F2.C1` and its coordinates are measured in that face's plane.

```text
{ "type": "sketch", "id": "F3", "name": "Hole centre", "plane": { "kind": "face", "id": "F2.C1" },
  "points": [ { "id": "c", "x": 20, "y": 30 } ],
  "entities": [ { "id": "e1", "type": "point", "p": "c" } ],
  "constraints": [] },
{ "type": "hole", "id": "F4", "name": "Hole", "sketch": "F3",
  "kind": "simple", "diameter": 8, "end": "through" }
```

Append both to the `features` list of the plate and regenerate under the same tag. Each regen replaces the state of its tag, and the kernel reuses the work of the earlier features that did not change.

```json
{ "type": "regen", "tag": "plate", "part": { ... the four features ... } }
```

In the reply, `errors` is still `[]`, and the body now has seven faces and fourteen edges: one new cylindrical face, and the two circles where it meets the caps.

```json
"faces": [ ..., 
  { "id": "F2.C1", "kind": "plane", "planar": true, "meta": { "feature": "F2" }, "normal": [0, 0, 1] },
  { "id": "F4.S.h1", "kind": "cylinder", "planar": false, "meta": { "feature": "F4", "entity": "h1" } } ],
"edges": [ ...,
  { "pts": [ ... 23 points ... ], "faces": ["F2.C0", "F4.S.h1"], "key": "F2.C0|F4.S.h1#12", "mid": [22.9, 27.3, 0],
    "circle": { "c": [20, 30, 0], "n": [0, 0, -1], "r": 4 } },
  ... ]
```

The cylinder `F4.S.h1` is named for the feature `F4` that made it. Its edges carry `circle` with the centre, the normal and the radius of 4 mm (the diameter was 8). The `pts` of an edge are a polyline of the curve for drawing.

To see how the regen was split into steps, add `"debug": true` to the message: the reply then also has a `trace` with one entry per feature and per stage (`F2 extrude → 6 faces`, `F4 hole → 7 faces`, `boolean Subtract (6 × 3 faces) → 7 faces`, `tessellate 7 faces → 104 triangles`). See [Protocol overview](../protocol/overview.md).

## Validate

```json
{ "type": "validate", "tag": "plate" }
```

```json
{ "ok": true, "result": { "ok": true, "issues": [], "toleranceUsed": 0 }, "ms": 0 }
```

The outer `ok` says the request ran; `result.ok` says the body passed all checks. An invalid body would list `issues`, each with a `code`, the `entity` it concerns and a `detail`. [The validate request](../validation/validate.md) describes the checks.

## Mass properties

```json
{ "type": "massProps", "tag": "plate" }
```

```json
{
  "ok": true,
  "result": {
    "volume": 59497.34517542564,
    "area": 15350.796447372313,
    "com": [50.25345071604068, 30, 4.999999999999997],
    "principal": [18493800.59049692, 50037589.30162367, 67539767.47253016],
    "axes": [ [1, 1.4e-17, 8.3e-19], [-1.4e-17, 1, 2.9e-22], [-8.3e-19, -2.9e-22, 1] ]
  },
  "ms": 0
}
```

These agree with a hand calculation. The volume is the plate minus the hole, 100 x 60 x 10 - pi x 4^2 x 10 = 59497.345 mm^3. The surface area is the plate's 15 200 mm^2 minus the two removed discs, plus the hole wall: 15200 - 2 x pi x 16 + 2 x pi x 4 x 10 = 15350.80 mm^2. The centre of mass is at mid-thickness (z = 5) and has moved a little in x, away from the hole at x = 20. `principal` holds the principal moments of inertia of the solid, taken at unit density and about the centre of mass (volume plays the role of mass), and `axes` the three matching unit directions.

For a mass, give the density of each separate body (here one) per cubic millimetre. Aluminium at 2.7 g/cm^3 is `0.0027` g/mm^3:

```json
{ "type": "massProps", "tag": "plate", "densities": [0.0027] }
```

This adds `bodies: [ { "volume": 59497.3, "area": 15350.8, "com": [...], "mass": 160.64 } ]` and a `byMass` object with the mass (160.64 g) and the principal moments for that density.

## Change it and regenerate

Edit the document and send it again. Making the plate 12 thick (the `depth` of `F2`) gives a new volume of 100 x 60 x 12 - pi x 4^2 x 12 = 71396.81 mm^3; the kernel answers

```json
{ "volume": 71396.81421051078, "area": 16041.06192982975, "com": [50.25345071604068, 30, 6], ... }
```

and the face ids are unchanged, so a feature that referred to `F2.C1` or `F4.S.h1` still works.

When a feature cannot be built, the regen still succeeds as a request: the error is reported per feature and the rest of the document runs on what exists. Setting the hole `diameter` to `0`:

```text
"errors": [
  [ "F4", { "code": "holeInvalid",
            "params": { "hint": "the hole diameter must be greater than zero (a standard `size`, or a positive `diameter`)" } } ]
]
```

The body is then the plate without a hole: six faces, the state from before `F4`. Each entry of `errors` is `[feature id, { code, params }]`, and `params.hint` says what to change. [Errors](../protocol/errors.md) lists the codes.

To stop early, pass `"opts": { "upTo": 2 }` to regenerate only the first two features; with the four-feature document that gives the plate without the hole (six faces, as above).

## See also

- [Call it from Node](node.md)
- [Features](../format/features.md)
- [Regeneration](../protocol/regen.md)
- [Queries](../protocol/queries.md)
- [Errors](../protocol/errors.md)
