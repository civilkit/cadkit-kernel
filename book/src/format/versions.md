# Versions and earlier files

A part records the version of the format it was written for. This chapter covers the `format` string and the `version` field, what the kernel does with a file that is newer or older than it expects, and how unknown fields, values and feature types are treated so that documents survive across versions.

## The format string and the version

Every document the kernel writes or documents carries

```json
{"format": "ck-part", "version": 1, "features": []}
```

- `format` is the string `ck-part`. It is the only value of `format` that new documents use, and a document without a readable `format` is refused with `invalid part: Format` (see [the overview](overview.md#what-is-checked-when-a-document-is-read)).
- `version` is a number. The version this kernel knows and writes is **1**. It is not checked against anything when the document is read.

The TypeScript package `part-model` declares the same two constants (`format: 'ck-part'`, `version: 1`) and `emptyPart()` builds a document with them.

## Files saved by earlier versions

Files saved by earlier versions are still read: the older spellings of the format name are accepted and the document is read as `ck-part`. Writers should write `ck-part`; the kernel never writes an older name. The rest of the document is read by the same rules whatever the format name was, so an older file needs no conversion step.

## A file newer than the kernel

A document whose `version` is higher than the kernel's is still read and regenerated. The reply then says so, in one additive field of `result` next to `errors` and `body`:

```json
{"result": {"errors": [], "partVersionNewer": {"file": 2, "kernel": 1}}}
```

`file` is the document's `version`, `kernel` the one this build knows. The field is absent for every document the kernel knows (version 1, 0, or no version), so replies for those documents are unchanged.

The kernel does not refuse a newer file, because most newer files only add things that an older reader can skip. It cannot tell which, so a client that sees `partVersionNewer` should treat a feature that fails with `unknownFeature` as the likely cause of any problem, and should be careful about saving the document back.

A missing `version`, a `version` that is not a number (`"3"` counts as missing) and a number of 1 or less give no warning.

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 5}
  ]
}
```

Run result: no errors; 3 faces, volume 1570.7963 mm³. The reply has no `partVersionNewer` (absent).


```json
{
  "format": "ck-part", "version": 2,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 5}
  ]
}
```

Run result: no errors; 3 faces, volume 1570.7963 mm³. The reply's `result` carries `"partVersionNewer": {"file":2,"kernel":1}`; the body is the same as for version 1.


## Forward compatibility

A reader has to cope with fields and values that a newer writer may add. The kernel's rules:

| What is new | What the kernel does |
|---|---|
| An unknown top-level field | Kept untouched, never read. |
| An unknown field on a feature | Kept in the feature, ignored. For a sketch the reply echoes it back (key order and unknown keys are preserved). |
| An unknown feature `type` | That feature fails with `unknownFeature {type}`. The other features run. |
| An unknown sketch entity `type` | The entity is dropped. |
| An unknown sketch constraint `type` | The constraint is kept; the solver cannot build its system and leaves the sketch as written. |
| An unknown value in an enumerated field | Depends on the field (below). |

Unknown values of enumerated fields:

| Field | Unknown value gives |
|---|---|
| `op` (`extrude`, `revolve`, `sweep`, `loft`, `sphere`) | `add`. |
| `plane` `mode` | `offset`. |
| `hole` `kind`, `fit` | `simple`, `normal`. |
| `helix` `definedBy` | `pitchRevs`. |
| `thicken` `side` | `front`. |
| `extrude` `end` | Error `unknownEnd`. |
| `chamfer` `chamferType`, `fillet` `filletType` | Error `unsupported {what}`. |
| `mate` `kind` | Error `mateGeometry`. |
| `combine` `op`, `extrude` `thin.side` | Error `invalidValue`. |

A part that mixes known and unknown content therefore regenerates as far as it can: here a document from a newer writer with an unknown feature type, unknown fields and a thumbnail:

```json
{
  "format": "ck-part", "version": 3, "futureField": [1, 2], "thumbnail": "data:image/png;base64,AAAA",
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 5},
    {"type": "hoverTool", "id": "F3", "strength": 3},
    {
      "type": "extrude", "id": "F4", "op": "add", "sketch": "F1", "end": "blind", "depth": 5,
      "futureOption": {"nested": true}
    }
  ]
}
```

Run result: errors `F3`: `unknownFeature` `{"type":"hoverTool"}`; 3 faces, volume 1570.7963 mm³. The unknown type fails alone; the unknown fields are ignored.


## What a writer should do

- Write `"format": "ck-part"` and the version of the format being written.
- Keep every field it does not understand when it reads and writes a document back, at the top level and inside features. The kernel does the same when it echoes a sketch.
- Not rely on the kernel to refuse a newer document: check `partVersionNewer` in the regen reply.

## See also

- [Overview of the part format](overview.md)
- [Features](features.md)
- [Regeneration](../protocol/regen.md)
- [Errors](../protocol/errors.md)
