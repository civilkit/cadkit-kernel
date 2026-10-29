# Overview of the part format

A part is one JSON document, normally stored as a `.ckpart` file. It holds an ordered list of features; the kernel runs them from first to last (a regeneration, or "regen") and builds the sketches, planes, curves and bodies they describe. This chapter covers the document as a whole: the top-level fields, feature ids, suppression, rollback, units, coordinate frames and what is checked when a document is read.

## The shortest valid part

A part needs a `format` and a `features` array. Nothing else is required.

```json
{"format": "ck-part", "features": []}
```

```json
{"format": "ck-part", "version": 1, "features": []}
```

Run result: no errors; no solid body. An empty `features` list is a valid part.


## Top-level fields

| Field | Type | Default | Meaning |
|---|---|---|---|
| `format` | string | required | Must be `"ck-part"`. Documents saved by earlier versions are read too (see [versions](versions.md)). |
| `version` | number | `1` | Version of the document format. A value above the kernel's own (1) is still read and the regen reply says so (see [versions](versions.md)). A missing or non-numeric value counts as 1. |
| `name` | string | `""` | Display name. The kernel does not use it. |
| `features` | array | required | The regen program, in order. See below. |
| `variables` | array of `{name, expr}` | none | Global variables usable in expressions. See [variables and expressions](expressions.md). |
| `seq` | number | `0` | Counter used by editors to mint feature ids (`"F" + seq`). The kernel reads it and never uses it. |
| `rollback` | number or `null` | `null` | Regenerate only the first N features. See [rollback](#rollback-and-upto). |
| `material` | string | `""` | Material name. The kernel does not use it for geometry or mass properties. |
| `counters` | object of numbers | `{}` | Per-command-name counters that editors use to number feature names (`{"sketch": 4, "extrude": 2}`). Not used by the kernel. |
| `blobs` | object of strings | none | Data that features refer to by key, such as the STEP text of an [`importStep`](features-assemblies.md#importstep) feature. The key is the SHA-256 of the text, in lower-case hex. |
| `kind` | string | `""` | `"assembly"` marks an assembly document. The regen does not branch on it: an assembly is a part whose features are `insertPart` and `mate` (see [parts and assemblies](features-assemblies.md)). |

Any other top-level field (a thumbnail, editor settings, fields written by a newer version) is not read and is left untouched. The kernel never rejects a document for having extra fields.

A complete small part, with the editor fields filled in:

```json
{
  "format": "ck-part", "version": 1, "name": "Plate", "seq": 3, "rollback": null, "material": "steel",
  "counters": {"sketch": 1, "extrude": 1},
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
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10}
  ]
}
```

Run result: no errors; 6 faces, volume 8000 mm³. The faces are `F2.C0`, `F2.C1`, `F2.S.e1`, `F2.S.e2`, `F2.S.e3`, `F2.S.e4`.


Run this way, a 40 x 20 x 10 mm plate has six faces. The names are explained in [identifiers and references](references.md). The other chapters of this part reuse this plate as a base and show only the features they append to it.

## Features

Every entry of `features` is an object with a common envelope and type-specific fields.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `type` | string | `""` | The feature type, for example `"sketch"` or `"extrude"`. [Features](features.md) lists every type. |
| `id` | string | `""` | The feature id (see below). |
| `num` | number | `0` | Display ordinal among features of the same kind. Not used by the kernel. |
| `name` | string | `""` | Display name. Not used by the kernel. |
| `suppressed` | boolean | `false` | When true the feature is skipped. |

All other fields belong to the feature type. A field the kernel does not know is kept and ignored.

### Feature ids and `seq`

An id is a string. Editors write `F1`, `F2`, ... and keep `seq` one above the highest number used, so the next feature gets `"F" + seq`. Plane features may carry other ids; the kernel accepts any string.

Ids are the names that other features and the regen reply use:

- a feature refers to an earlier feature by id (`"sketch": "F1"`, `"seeds": ["F4"]`, `{"kind": "feature", "id": "P1"}`);
- the faces a feature creates start with its id (`F2.C1`), see [identifiers and references](references.md);
- the regen reply lists errors, warnings, sketches and planes keyed by feature id.

The kernel does not check that ids are unique. Results keyed by id overwrite each other, so two features with one id make a document that is hard to read back. A feature without an `id` is keyed by the empty string.

Features are run in list order and can refer only to features before them. A reference to a later feature behaves like a reference to a missing one (for example `missingSketch`).

### Errors do not stop the regen

A feature that fails reports its error under its id, contributes nothing, and leaves the body as it was. The features after it still run.

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
      "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
      "points": [{"id": "c", "x": 20, "y": 10}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 4}], "constraints": []
    },
    {"type": "extrude", "id": "F4", "op": "cut", "sketch": "F3", "end": "blind", "depth": "=oops"},
    {"type": "extrude", "id": "F5", "op": "cut", "sketch": "F3", "end": "blind", "depth": 3}
  ]
}
```

Run result: errors `F4`: `badExpression`; 8 faces, volume 7849.2036 mm³.


Here `F4` fails and `F5` still cuts the pocket from the body that `F2` left. A feature that depends on the output of a failed feature fails too (a missing sketch, a face that does not exist).

### Suppression

`"suppressed": true` removes a feature from the run: it produces no output and no error. The value is read as a truth value: `true`, a non-zero number or a non-empty string all suppress. Here the extrude is added after the rectangle sketch `F1` of the plate and suppressed:

```json
{
  "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10,
  "suppressed": true
}
```

Run result: no errors; no solid body. The sketch is regenerated (1 sketch in the reply); the suppressed extrude adds nothing.


Features after a suppressed one that need its output fail as if it did not exist. Suppressing a sketch makes the features that use it report `missingSketch`.

### Visibility flags

`visible` (on sketches, planes and helices) and `userHidden` (on planes) are display flags for editors. The kernel regenerates the feature in the same way whatever their value: a sketch with `"visible": false` still feeds its extrude, and a plane with `"visible": false` still appears in the `planes` list of the reply.

### Rollback and `upTo`

Two settings limit the number of features that run:

- `rollback` in the document: a number N runs the first N features (`rollback: 1` runs only the first). It is how an editor shows the model at an earlier step.
- `opts.upTo` in the `regen` request: the same limit for one request, without changing the document. See [regeneration](../protocol/regen.md).

When both are present the smaller one wins. A value that is not a number is ignored (`"rollback": "1"` and `"upTo": "1"` do nothing); a negative `rollback` runs no features. Suppressed features inside the range count towards N. An inserted part ignores its own `rollback` (see [parts and assemblies](features-assemblies.md)). Setting `"rollback": 1` on the plate above gives this document:

```json
{
  "format": "ck-part", "version": 1, "rollback": 1,
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
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10}
  ]
}
```

Run result: no errors; no solid body. Only `F1` runs (1 sketch in the reply).


## Units

Lengths are in millimetres and angles are in degrees, in every field and in every reply. Numbers in expressions may carry a unit suffix that converts to these units (`2 cm`, `1 in`); see [variables and expressions](expressions.md).

## Coordinate frames

The world frame is right-handed. The three standard sketch planes are frames with an origin `o` and axes `x`, `y`, `z` (the plane normal):

| Plane | `x` | `y` | `z` (normal) |
|---|---|---|---|
| `front` | (1, 0, 0) | (0, 1, 0) | (0, 0, 1) |
| `top` | (1, 0, 0) | (0, 0, -1) | (0, 1, 0) |
| `right` | (0, 0, -1) | (0, 1, 0) | (1, 0, 0) |

All three pass through the world origin. A sketch point `(u, v)` on a plane with frame `(o, x, y)` is the world point `o + u x + v y`. Reference planes and planar faces provide further frames; see [planes](planes.md) and [sketches](sketches.md).

## What is checked when a document is read

The kernel is lenient on purpose. Reading a document fails in exactly one way, with the text `invalid part: Format`. It happens when:

- the document is not a JSON object;
- `format` is missing, or is not `"ck-part"` (or one of the earlier names, see [versions](versions.md));
- `features` is missing or is not an array.

Over the wire the reply is `{"ok": false, "error": "invalid part: Format"}` (see [errors](../protocol/errors.md)).

Everything else is accepted and handled while the features run, one feature at a time:

| Situation | Result |
|---|---|
| A feature is not an object, or has no `type` | `unknownFeature` with `type: ""`, keyed by `""`. |
| A feature has a type the kernel does not know | `unknownFeature` with the type in `params.type`. |
| A field is missing or has the wrong JSON type | The default is used (see the field tables). Features then report their own errors, for example `missingSketch`. |
| A numeric field holds a string | `"=expr"` is an expression, a plain numeric string is that number, any other string is 0. See [variables and expressions](expressions.md). |
| A boolean field holds a number, string, array or object | Truth value: `true`, non-zero numbers, non-empty strings, arrays and objects are true. `"false"` is a non-empty string and so true. |
| A sketch entity has an unknown `type` | The entity is dropped without an error. |
| `variables` is not an array, or `seq`, `name`, `material`, `counters` have the wrong type | The field is ignored. |

The per-feature error codes are listed with each feature in the chapters that follow, and all of them are collected in [feature errors and diagnostics](../validation/diagnostics.md).

## See also

- [Identifiers and references](references.md)
- [Variables and expressions](expressions.md)
- [Planes](planes.md)
- [Sketches](sketches.md)
- [Features](features.md)
- [Versions and earlier files](versions.md)
- [Regeneration](../protocol/regen.md)
