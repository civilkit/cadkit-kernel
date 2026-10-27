# STEP

The kernel writes a body as an ISO 10303-21 file (AP214, `AUTOMOTIVE_DESIGN`) with the `step` request and reads B-rep solids from STEP files with the `importStep` request. Both live in `crates/ck-io` (`step/write.rs`, `step/read.rs`, `step/parse.rs`).

## Export: the `step` request

```json
{"type":"step","tag":"main","name":"plate"}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State to export. A tag that has never been regenerated or imported answers the error `noState: nothing regenerated yet` |
| `name` | string | `"part"` | Part name written into `FILE_NAME`, `PRODUCT`, the solid and the shape representation |

The reply carries the whole file as one JSON string, with no binary buffers:

```json
{"ok":true,"result":"ISO-10303-21;\nHEADER;\n...END-ISO-10303-21;\n","ms":0}
```

`result` is `null` when the tag holds no body. This happens for an empty part and for a part that only builds surfaces (checked: a sketch with a `surfaceExtrude` answers `null`).

The examples in this chapter use a 40 x 20 x 8 mm plate with a 10 mm through hole, built from this part:

```json
{"format":"ck-part","version":1,"name":"plate","features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],
  "entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},
              {"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],
  "constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":8},
 {"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":8},
 {"type":"sketch","id":"F4","plane":{"kind":"feature","id":"F3"},
  "points":[{"id":"p1","x":20,"y":10}],"entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
 {"type":"hole","id":"F5","sketch":"F4","kind":"simple","diameter":10,"end":"through"}]}
```

With `"name":"plate"` in the `step` request, the export is 189 lines and ends with `#178=MANIFOLD_SOLID_BREP`, `#179=ADVANCED_BREP_SHAPE_REPRESENTATION` and `#180=SHAPE_DEFINITION_REPRESENTATION`. If the hole is instead a circle in the first sketch (a point `o` at (20, 10) and a circle of radius 5 centred on `o`, no `hole` feature), the file is 187 lines and the same three entities are `#176`, `#177` and `#178`. The header and the first entities:

```text
ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('CadKit model'),'2;1');
FILE_NAME('plate.step','2026-01-01T00:00:00',(''),(''),'CadKit','CadKit','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1=APPLICATION_CONTEXT('core data for automotive mechanical design processes');
...
#10=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#11=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
...
#178=MANIFOLD_SOLID_BREP('plate',#177);
#179=ADVANCED_BREP_SHAPE_REPRESENTATION('plate',(#178,#18),#14);
#180=SHAPE_DEFINITION_REPRESENTATION(#9,#179);
ENDSEC;
END-ISO-10303-21;
```

The time stamp in `FILE_NAME` is the host clock (`ck_unix_ms`) in UTC. A host that returns a fixed clock gets a reproducible file.

### What is written

- **One product.** Entities `#1` to `#14` are the application context, the product definition and the geometric context. Lengths are millimetres, angles radians, with a distance uncertainty of `1.E-06`. Entities `#15` to `#18` are the world placement.
- **Faces in shell order.** For each face the writer emits the surface, then per boundary the edges (vertices, curve, `EDGE_CURVE`, `ORIENTED_EDGE`), the `EDGE_LOOP`, the bound, and finally the `ADVANCED_FACE`. A `CLOSED_SHELL` follows each shell and then every solid.
- **Surfaces:** `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE`, `SURFACE_OF_LINEAR_EXTRUSION`, `SURFACE_OF_REVOLUTION`, `OFFSET_SURFACE`, and B-spline surfaces (`B_SPLINE_SURFACE_WITH_KNOTS`, or the rational complex entity when the weights differ from 1).
- **Curves:** `LINE`, `CIRCLE`, `ELLIPSE`, B-spline curves (rational as a complex entity). Conic curves are written as `ELLIPSE`, `PARABOLA` or `HYPERBOLA` (not exercised in the checks for this chapter); sinusoids as B-splines.
- **Seams are not written.** A periodic face (cylinder, cone, torus, sphere) is bounded the way STEP usually bounds it: by bare circles. A full sphere or torus with no boundary is split into two faces on one shared surface.
- **Planar faces get a `FACE_OUTER_BOUND`** (the loop of largest area); every other bound is a `FACE_BOUND`.
- **Voids.** A lump with cavities becomes `BREP_WITH_VOIDS`; each void is a `CLOSED_SHELL` wrapped in `ORIENTED_CLOSED_SHELL`.
- **Several lumps** give several `MANIFOLD_SOLID_BREP` entities in one `ADVANCED_BREP_SHAPE_REPRESENTATION`. A two-block part exported as 2 solids (checked).
- **Numbers.** Reals print in the shortest form that reads back exactly, always with a decimal point (`0.`, `25.`, `1.E-7`); scientific notation is used below `1e-6` and from `1e15`. Strings escape `'` and `\`, drop control characters and write non-ASCII characters as `\X2\hhhh\X0\`.

The writer has a code path for open shells (`SHELL_BASED_SURFACE_MODEL` in a `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`). It is marked as unverified in the source and was not exercised here.

### Not written

Colours and styles, layers, per-face or per-body names, assemblies and placements, product structure beyond a single part, and product and manufacturing information. A part with several named bodies exports them as anonymous solids that share the part name.

## Import: the `importStep` request

```json
{"type":"importStep","tag":"imp","name":"plate","data":"ISO-10303-21;\n..."}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State that receives the body |
| `name` | string | `"part"` | Accepted, not used in the reply |
| `data` | string | required | The STEP file text. A missing or non-string value is a request error (`importStep: data must be a STEP file as a string`) |
| `heal` | `"off"`, `false` or object | on | Healing options, below |
| `healTolerance` | number | `0.05` | Cap on the tolerance healing may give an edge or vertex, in mm |

The reply has the shape of a `regen` result: `body` (faces, edges, mesh buffers), `errors`, and an `import` summary. Importing the plate above back gives:

```json
{"errors":[], ...,
 "import":{"solids":1,"faces":7,"unitScale":1,"warnings":[],
           "heal":{"fixed":0,"remaining":0,"toleranceUsed":0}}}
```

| `import` field | Meaning |
|---|---|
| `solids` | Number of `MANIFOLD_SOLID_BREP` and `BREP_WITH_VOIDS` entities read (one lump each) |
| `faces` | Faces of the resulting body |
| `unitScale` | Millimetres per file length unit that was applied |
| `warnings` | Non-fatal problems, each `["import", {code, params:{message, entity?}}]` |
| `heal` | Present unless healing is off: `fixed` repairs made, `remaining` validator findings left, `toleranceUsed` largest edge or vertex tolerance in the body |

The tag's state becomes the imported body, so `massProps`, `validate`, `heal`, `recognize`, `drawing` and `step` work on it. Re-importing the exported plate gives a volume of 5771.681 mm3, the same as the source (checked), and `validate` reports no issues. A failed import leaves the tag's previous state untouched (checked: the volume of the earlier import was still answered after a failed one).

### Face names and multi-solid files

Imported faces are named `I<k>.F<n>`: `k` is the 1-based index of the solid in entity-id order and `n` counts faces over the whole file. The plate imports as `I1.F1` to `I1.F7`; the cylinder face of the hole is `I1.F7`. Names are stable for a given file, but a STEP file carries no history, so they say nothing about how the part was made. For a rebuilt feature tree see [feature recognition](recognition.md).

All solids of a file are read into one body with one lump each. Two disjoint blocks exported by the kernel import as `solids: 2`, 13 faces and the same combined volume and mass properties (checked).

### Units

The reader takes the length and plane-angle units named by the `GLOBAL_UNIT_ASSIGNED_CONTEXT` of the geometric representation context. If there is none it uses the first `LENGTH_UNIT` and `PLANE_ANGLE_UNIT` in the file. SI units with any prefix, and `CONVERSION_BASED_UNIT` (inch, degree and so on), are converted to millimetres and radians. A file whose context declares inches, with the plate's numbers, imports as a 1016 x 508 x 203.2 mm body and reports `unitScale: 25.4` (checked).

### Entities read

| Group | Entities |
|---|---|
| Solids and shells | `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS` (with `ORIENTED_CLOSED_SHELL`), `CLOSED_SHELL` |
| Faces and loops | `ADVANCED_FACE`, `FACE_SURFACE`, `FACE_BOUND`, `FACE_OUTER_BOUND`, `EDGE_LOOP`, `ORIENTED_EDGE` (also nested), `EDGE_CURVE`, `VERTEX_POINT` |
| Surfaces | `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE` (negative radii accepted), `SURFACE_OF_REVOLUTION`, `SURFACE_OF_LINEAR_EXTRUSION`, `OFFSET_SURFACE`, `RECTANGULAR_TRIMMED_SURFACE` (basis surface), B-spline surfaces (simple, complex, rational, Bezier and quasi-uniform forms) |
| Curves | `LINE`, `CIRCLE`, `ELLIPSE`, `PARABOLA`, `HYPERBOLA`, B-spline curves (same forms), `TRIMMED_CURVE` (the bounds come from the edge vertices), `SURFACE_CURVE`, `SEAM_CURVE`, `INTERSECTION_CURVE`, `BOUNDED_SURFACE_CURVE` (the 3D curve is used) |

The parser is a full Part 21 value-tree parser: nested lists, typed parameters, complex (multi-part) entities and comments.

### Topology completion

STEP describes periodic faces differently from the kernel. The importer therefore completes the topology:

- a cylinder bounded by two bare circles gets a seam edge, and a circle is split at the seam position when the two circles' vertices are not aligned;
- cone and sphere faces with a single boundary get their pole structure;
- a full sphere or torus that the file splits into two faces is merged back into one;
- degenerate edges (poles, apexes) are dropped;
- closed B-spline profiles that wind over several laps are unrolled.

A boundary that is a `VERTEX_LOOP` is ignored with a `skipped` warning (`vertex loop ignored`), as is a loop with no non-degenerate edge.

### Tolerances and healing

Exporters often write intersection edges as fitted splines that sit a few thousandths of a millimetre off the faces they bound. After reading, the importer moves an edge that is off both of its faces by more than rounding but less than a file-sized amount onto the faces' intersection, and reports a `skipped` warning for an edge it cannot move.

Healing then runs by default with a model tolerance of 1e-4 mm. It merges near vertices, closes gaps, sews free edges, removes degenerate edges and sliver faces, makes loop and shell orientation consistent, and gives remaining small deviations an edge or vertex tolerance up to the cap (0.05 mm unless `healTolerance` or `heal.cap` says otherwise). A valid body is not touched. Options:

```json
{"type":"importStep","tag":"imp","data":"...","heal":{"tol":1e-3,"sew":true,"fixSmall":true,"fixOrientation":true,"tolerant":true,"cap":0.05}}
```

`"heal":"off"` (or `false`) skips the pass and omits `import.heal` from the reply. See [healing](../validation/healing.md) and [tolerances](../validation/tolerances.md).

### Errors

A failed import answers `body: null` and one entry in `errors`:

| Code | Meaning | Example (run) |
|---|---|---|
| `parse` | The text is not a Part 21 file | `not a STEP file (missing ISO-10303-21 header)`; `expected '(' after keyword at line 77` |
| `noSolid` | The file parses but holds no solid | `no MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS in the file` |
| `unsupported` | An entity the reader does not handle, with its id | replacing the first `PLANE` of the plate by `SWEPT_DISC_SURFACE` gives `{"code":"unsupported","params":{"message":"SWEPT_DISC_SURFACE is not supported","entity":22}}` (entity 21 for the circle-in-sketch variant) |
| `badEntity` | A supported entity with missing or invalid arguments | |
| `topology` | The faces cannot be assembled into a closed shell | |

The reader does not panic on malformed or truncated input; this is covered by a truncation test.

## Limits

- No assemblies: placements and `SHAPE_REPRESENTATION_RELATIONSHIP` are not read. Every solid in the file is imported at its own coordinates.
- Shell-based surface models and open shells are not imported as solids; a file with no solid answers `noSolid`.
- `POLY_LOOP` bounds, composite curves and faces whose boundary winds twice around a periodic surface are not supported.
- Colours, names, layers and product information are ignored.
- Open items: torus-fillet shell orientation and sphere pole boundaries in some files. In four samples of 200 STEP files each from a public research set (seeds 1234, 4242, 5678, 9001), 185 to 188 files per sample import and validate; the rest import but fail `validate`, mostly with `TESSELLATION_OPEN` and `EDGE_OFF_SURFACE` (see [corpora and sweeps](../quality/corpora.md)).
- Known exporter limit: an arc split at a seam vertex but used by a planar face on only one half is written as a closed circle, and the import of that file is then wrong. The cases are `flangeHalf@9` to `flangeHalf@11` and `flangeNut@12` in `tools/brep_cases.mjs`.

## See also

- [SketchUp](skp.md), [SolidWorks](sldprt.md) and [ACIS SAT](sat.md) imports
- [Feature recognition](recognition.md)
- [Healing](../validation/healing.md)
- [Corpora and sweeps](../quality/corpora.md)
