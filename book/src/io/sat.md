# ACIS SAT

The `importSat` request reads the text form of an ACIS file (`.sat`) and builds a B-rep with exact analytic and NURBS geometry. The importer is in `crates/ck-io/src/sat`. There is no SAT export.

## Request

```json
{"type":"importSat","tag":"s","name":"box.sat","data":"700 0 1 0\n@12 ck-test-gen ... "}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State that receives the body |
| `name` | string | `"part"` | Echoed as `import.name` |
| `data` | string | required | The SAT file as text. A missing `data` is a request error (`importSat: data (the SAT file text) is required`) |

The reply is a `regen`-shaped result with `body`, `errors` and an `import` summary. On failure `body` is `null` and the tag keeps its previous state. On success `massProps`, `validate`, `heal`, `recognize`, `drawing` and `step` work on the tag.

Real output for the bundled box fixture (a 10 x 20 x 30 box):

```json
{"format":"sat","name":"box.sat","units":1,"sheet":false,"faces":6,
 "version":700,"records":85,"bodies":1,"lumps":1,"shells":1,"skipped":{}}
```

`massProps` gives a volume of 6000 mm3 and an area of 2200 mm2, and `validate` is clean.

| `import` field | Meaning |
|---|---|
| `units` | Millimetres per file unit; the geometry is already scaled |
| `sheet` | `true` when every face is double sided, an open sheet body rather than a solid |
| `faces` | Faces of the body |
| `version` | The ACIS version number from the header (700 is ACIS 7.0) |
| `records` | Number of records read |
| `bodies`, `lumps`, `shells` | Counts from the file; several bodies are merged into one kernel body, one lump each |
| `skipped` | Record types that were read past but not used, with counts (for example attributes and pcurves); sorted by name |

## Supported records

| Group | Supported |
|---|---|
| Surfaces | `plane-surface`; `cone-surface` (a cylinder when the sine is 0, otherwise a cone; circular sections only); `sphere-surface`; `torus-surface` (major radius larger than the absolute minor radius); `spline-surface` with an exact `nubs` or `nurbs` net |
| Curves | `straight-curve`; `ellipse-curve` (a circle when the ratio is 1); `intcurve-curve` with an exact `nubs` or `nurbs` block |
| Topology | body, lump, shell, face, loop, coedge, edge, vertex, and the tolerant variants `tedge`, `tcoedge`, `tvertex` |
| Senses | Face sense, coedge sense and edge sense (resolved geometrically against the vertices), single and double sided faces |
| Transforms | Body transforms with rotation, uniform scale and translation |

Dialects handled (the dialect tests also read real exported files, which are not in the repository and cannot be re-run from it; the bundled fixtures are ACIS 7.0 text): CR, CRLF and LF line endings, `-N` record index prefixes, the extra id integer of ACIS 20 and later (dropped at parse time), `{ ref N }` shared spline definitions, control nets stored u-fastest (a net long in v is transposed), and many bodies per file.

ACIS cylinders and cones come without a seam; the importer inserts one when two boundary loops wrap the axis. Sphere faces cut by circles around the pole axis get a meridian seam to an implicit pole.

## Errors

A failed import returns one error, in the same `["import", {code, params}]` pair shape as the other import requests:

```json
["import", {"code":"sat.parse","params":{"message":"bad SAT header (version)","entity":null,"kind":"","line":1}}]
```

The fields below are in `params`, except `code`.

| Field | Meaning |
|---|---|
| `code` | One of `sat.parse`, `sat.empty`, `sat.topology`, `sat.unsupported`, `sat.geometry`, `sat.orientation` |
| `entity` | The record ordinal as `$N`, or `null` |
| `kind` | The record's type (`cone-surface`, ...), empty if unknown |
| `line` | 1-based line in the file (0 when unknown) |
| `message` | What is wrong |

Examples from real runs: the text `hello` gives `sat.parse` with `bad SAT header (version)`; an empty string gives `sat.parse` with `empty file`; a header with a bad unit line gives `bad SAT header (units)` on line 3. A file with no faces is `sat.empty`. The importer never panics on malformed input.

## Not supported

Each of these gives a structured error naming the record:

- non-exact splines (approximated `intcurve` and `spline-surface` forms) and periodic splines;
- elliptical cones;
- torus faces whose loops wrap an axis;
- reflecting or shearing body transforms (only rotation, uniform scale and translation);
- sphere bands whose loops share no azimuth vertex;
- mixed single and double sided faces in one body.

A further limit: the bundled sphere and torus files (one face, no edges) import with volume 0 and `VOLUME_NONPOSITIVE`.

Attributes, pcurves and any other unread record are skipped and counted in `import.skipped`.

## Test fixtures

`crates/ck-io/tests/fixtures/sat` holds seven small files. Five have closed-form volumes in `expected.json`, checked to 1e-6 relative:

| File | Volume (mm3) |
|---|---|
| `box.sat` | 6000 |
| `box_spline.sat` | 6000 |
| `cylinder_hole.sat` | 5277.876 |
| `plate_fillet.sat` | 3922.743 |
| `frustum.sat` | 593.761 |

The other two, `sphere.sat` and `torus.sat`, hold a single face with no edges. They import (1 face, `faces: 1`), but the body has volume 0 and `validate` reports `VOLUME_NONPOSITIVE`; the test expects exactly that, so a closed sphere or torus written in this form is not a usable solid after import. Larger real files are not in the repository (licence and size); the test that reads them looks in a directory named by the environment and skips when it is absent.

## See also

- [STEP](step.md), [SketchUp](skp.md), [SolidWorks](sldprt.md)
- [Healing](../validation/healing.md)
- [Corpora and sweeps](../quality/corpora.md)
