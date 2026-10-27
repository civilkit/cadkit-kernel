# SketchUp

The `importSkp` request reads a SketchUp `.skp` file into a body made of planar faces. The reader is the crate `crates/ck-skp`; it uses no SketchUp library. It reimplements published reverse-engineering work on the file formats (attribution in `crates/ck-skp/NOTICE` and `THIRD_PARTY.md`).

## Request

```json
{"type":"importSkp","tag":"s","name":"model","data":"<base64 of the .skp file>",
 "opts":{"hidden":false,"maxFaces":50000,"up":"z"}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State that receives the body |
| `name` | string | `"model"` | Echoed as `import.name` |
| `data` | string | required | Base64 of the file. URL-safe letters, whitespace and a `data:...;base64,` prefix are accepted. A missing `data` or a character outside base64 is a request error |
| `opts.hidden` | bool | `false` | Include hidden faces, instances and tags |
| `opts.maxFaces` | number | 50000 | Refuse models with more faces (`limit.faces`). Values below 1 are ignored |
| `opts.up` | `"z"`, `"y"`, `"yFlip"` | `"z"` | Rotate the whole model, below. Any other value is a request error |

The request is available when the kernel is built with the `skp` feature of `ck-wasm` (it adds about 4.6 percent to the module). Without the feature `importSkp` answers the usual "not implemented" error. The module used for the examples in this book has the feature.

The reply has the shape of a `regen` result: `body`, `errors`, and an `import` summary. A refused file answers `body: null` with one error `["import", {code, params:{message, offset?}}]` and leaves the tag's state alone.

## What is read

SketchUp has two file containers, and the reader detects which one it has:

| Container | `import.container` | Versions | Layout |
|---|---|---|---|
| Classic | `"classic"` | 3 to 2020 | An MFC `CArchive` object stream |
| Modern | `"modern"` | 2021 and later | A ZIP archive holding `model.dat` (a tag-length-value tree) and material images |

The sample files in `crates/ck-skp/tests/fixtures` cover these versions. The fixtures for classic 3.0, 6.0, 7.0 and 14.0 (`legacy_v3_synthetic`, `legacy_v6_synthetic`, `legacy_v7_synthetic`, `legacy_v2014_synthetic`) are synthetic files written to the old layout, not exports from SketchUp, so support for those versions is checked against hand-made files; classic 17 and 20 and modern 25 are checked against files from the application. `import.version` is the version string stored in the file.

From the file the reader takes vertices, edges, faces with holes and texture coordinates, component definitions and their instances with transforms, groups, tags (layers) and hidden state, and materials (colour, opacity, double-sidedness, texture bytes). Nested instances are flattened (nesting is capped at 32 levels) and the result is baked into world space.

## What the body is

A SketchUp model is a surface model, not a solid. The body therefore has:

- one planar face per SketchUp face, with its holes as inner loops;
- vertices welded within 1e-4 mm and edges shared between neighbouring faces;
- millimetres: SketchUp stores inches, and the importer multiplies by 25.4 (`import.unitScale`);
- faces named `skp.F0`, `skp.F1`, and so on, in creation order;
- one lump, with one shell per connected piece (`import.lumps` counts pieces).

The body is closed only where the model is. `import.openEdges` counts edges used by one face, `import.nonManifoldEdges` those used by more than two. A closed cube from the sample files has `openEdges: 0`; a model made of loose faces has many.

Materials do not live in the body. `import.materials` lists `{color:[r,g,b], opacity, doubleSided, textured}` and `import.faceMaterials[n]` is the material index of face `skp.F<n>`. Texture image bytes are read by the library but not sent over the protocol.

Not imported: scenes (pages) and cameras, dimensions, text, section planes, construction lines and points, loose edges and curves (no wire output), analytic arcs (faces are polygons), attribute dictionaries and dynamic-component properties, and styles other than face colours. Overlapping holes in one face are bridged as given.

## Reply fields

| Field | Meaning |
|---|---|
| `format`, `name` | `"skp"` and the request name |
| `container`, `version` | Container kind and the version string in the file |
| `units` | The model unit in the file (`"Millimeter"` for a modern file); `null` for classic files, which carry no unit |
| `unitScale` | Always 25.4 (inches to millimetres) |
| `up`, `upMatrix` | The orientation option and its 3 x 3 matrix, row major |
| `faces`, `lumps` | Face and connected-piece counts |
| `openEdges`, `nonManifoldEdges` | Boundary and non-manifold edge counts |
| `skippedFaces`, `hiddenSkipped` | Faces that could not be built, and hidden items left out |
| `definitions`, `instances` | Component definitions and baked instances |
| `materials`, `faceMaterials` | As above |
| `bboxMm` | `[xmin, ymin, zmin, xmax, ymax, zmax]` of the finished body |
| `warnings` | Strings, `code: message` |

## Orientation

SketchUp is Z up. The default keeps the file's axes. `opts.up` rotates the finished model (after units and instances are resolved):

| `up` | Maps (x, y, z) to | Meaning |
|---|---|---|
| `"z"` | (x, y, z) | As stored |
| `"y"` | (x, z, -y) | Rotate -90 degrees about X: SketchUp Z becomes Y |
| `"yFlip"` | (-x, -z, -y) | `"y"` and then 180 degrees about Z |

Both rotations have determinant 1, so winding, face order, names, areas and counts do not change.

## Examples

A single face with holes (`holes.skp`, a classic 17 file), with `opts.up` set to `"y"`:

```json
{"format":"skp","container":"classic","version":"{17.0.1}","units":null,"unitScale":25.4,
 "up":"y","upMatrix":[1,0,0,0,0,1,0,-1,0],"faces":1,"lumps":1,"openEdges":8,
 "materials":[{"color":[255,84,84],"opacity":1,"doubleSided":true,"textured":false}],
 "faceMaterials":[0],"bboxMm":[0,0,-2540,5080,0,0],"warnings":[]}
```

With `up: "z"` the box is `[0,0,0,5080,2540,0]`. The body has one planar face `skp.F0`, area 11 870 944 mm2 and volume 0 (it is a sheet).

A modern file with two closed pieces (`SU_File.skp`): 32 faces, 2 lumps, `openEdges: 0`, `validate` reports no issues, and `massProps` with `bodies: true` gives volumes of 1.0e9 mm3 (a 1 m cube) and 1.6145e9 mm3.

A classic furniture model (`chair_and_table.skp`): 128 faces, 28 lumps, 9 definitions, 15 instances, 6 materials, 128 open edges.

Failures, all from real runs:

| Input | Error |
|---|---|
| Base64 of the text `AAAA` | `container.magic`: `not a SketchUp file (bad header magic)`, offset 0 |
| A file with only edges (`blank_v17.skp`) | `body.empty`: `the model has no visible faces to import ...` |
| `chair_and_table.skp` with `maxFaces: 50` | `limit.faces`: `128 faces exceed the import limit of 50; ...` |

## Error codes

Codes are grouped by stage. `offset` is a byte offset when the reader can give one.

| Prefix | Raised when |
|---|---|
| `container.*` | Bad magic, unknown container, no ZIP directory, no `model.dat` |
| `zip.*`, `inflate.*` | Damaged ZIP structure, CRC or size mismatch, corrupt DEFLATE data |
| `tlv.*`, `legacy.*` | Truncated or inconsistent model data in the modern or classic container |
| `limit.*` | A safety limit was exceeded: file size (512 MB), inflated size and compression ratio, node count (40 million) and depth (64), archive objects (40 million) and depth, instances (2 million), triangles (30 million), faces (`maxFaces`) |
| `body.*` | The scene produced no face (`body.empty`) |

The reader never panics on malformed input. Every read is bounds checked, and the crate carries a seeded mutation fuzzer that flips, truncates and splices the sample files.

## Limits

- The result is a surface body: volume is 0 unless the model is closed, and it does not behave like a solid in booleans.
- Models above 50 000 faces are refused unless `maxFaces` is raised; per-face work makes large imports slow (the largest sample, 39 352 faces, takes about 1.1 s in the checks).
- Curves are polygons: circles and arcs arrive as many short planar facets.
- Three of the 17 sample files (no faces, or everything hidden) are refused with `body.empty`; this is the expected answer for models without faces.

## See also

- [STEP](step.md), [SolidWorks](sldprt.md), [ACIS SAT](sat.md)
- [The B-rep model](../kernel/brep.md)
- [Replies and buffers](../protocol/reply.md)
