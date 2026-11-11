# SolidWorks

The `importSldprt` request reads a SolidWorks part file (`.SLDPRT`) and builds a body directly from the geometry stored in it. There is no STEP file in between. The reader is the crate `crates/ck-sldprt`, which uses no SolidWorks or Parasolid library; it decodes the file's container and the Parasolid transmit data inside it.

## Request

```json
{"type":"importSldprt","tag":"s","name":"part","data":"<base64 of the .SLDPRT file>"}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State that receives the body |
| `name` | string | `"part"` | Echoed as `import.name` |
| `data` | string | required | Base64 of the file (URL-safe letters, whitespace and a `data:...;base64,` prefix are accepted) |

The reply has the shape of a `regen` result with `body`, `errors` and an `import` summary. A failed import answers `body: null` with `errors: [["import", {code, params:{message}}]]` and leaves the tag's state unchanged. On success the tag holds the body, so `massProps`, `validate`, `heal`, `recognize`, `drawing` and `step` work on it.

## Pipeline

| Stage | Module | What it does |
|---|---|---|
| Container | `container.rs`, `inflate.rs` | Opens the file and extracts named streams, with an own bounded DEFLATE decoder and CRC-32 |
| Transmit | `xt.rs` | Reads the Parasolid neutral binary transmit (schema `SCH_13006` plus the embedded schema edits) |
| Intermediate form | `ir.rs` | Bodies, regions, shells, faces, loops, fins, edges, vertices, and their surfaces and curves, in metres; face colours and feature tags, body density |
| Kernel body | `build.rs` | Converts to millimetres and builds the body: solid regions only, edge ranges from the vertices, fin senses to coedge directions, seams on periodic faces, loop ordering, shell orientation |
| Metadata | `meta.rs` | Feature list, configurations, material, properties, units, versions and preview from the plain streams |

### Containers

| Kind | `import.container` | Files | Detection |
|---|---|---|---|
| OLE2 compound file | `"ole2"` | SolidWorks 2014 and earlier | First 8 bytes `D0 CF 11 E0 A1 B1 1A E1`; streams hold zlib frames |
| Block container | `"modern"` | SolidWorks 2015 and later | A block marker `14 00 06 00 08 00`; every block carries a name, sizes, a CRC-32 and raw DEFLATE data. A block is accepted only if its data inflate to the declared size and the CRC matches |

Any other file is `container.format`. The geometry sits in Parasolid transmits inside the streams, wrapped in GUIDs. The reader takes the live partition (`Contents/Config-0-Partition`). If it holds no solid, as in parts whose geometry is an imported body, the solid bodies of the free-standing part transmits are used instead (and a note says so).

## Reply fields

Real output for a 10 mm cube saved by SolidWorks 2011 (OLE2 container), with the face list shortened to two of its six entries:

```json
{"format":"sldprt","name":"part","solids":1,"faces":6,
 "unitScale":1000,"container":"ole2","schema":"SCH_2201236_20000_13006",
 "stream":"Contents/Config-0-Partition","nodes":165,
 "approximations":[],"notes":[],"swMassProps":null,
 "features":[{"id":2,"name":"Front Plane","type":"Plane","dimensions":[]},
             {"id":3,"name":"Top Plane","type":"Plane","dimensions":[]},
             {"id":4,"name":"Right Plane","type":"Plane","dimensions":[]},
             {"id":5,"name":"Origin","type":"Origin","dimensions":[]},
             {"id":18,"name":"Sketch1","type":"Sketch","dimensions":[]},
             {"id":24,"name":"Boss-Extrude1","type":"Boss-Extrude",
              "dimensions":[{"name":"D1","text":"10","value":10}]}],
 "configurations":[{"name":"Default","material":null}],
 "material":null,"customProperties":[],"configurationProperties":[],"units":null,
 "versions":{"solidworks":4700,"solidworksRelease":2011,"parasolid":"22.01","parasolidModeller":2201236},
 "thumbnail":{"mime":"image/png","width":640,"height":480,"base64":"iVBORw0KGgo..."},
 "omittedForPrivacy":0,
 "faceAttributes":[{"face":"sldprt.F1","xtFace":1,"colour":[0.792,0.820,0.933],
                    "featureId":24,"feature":"Boss-Extrude1","kind":40,"local":1},
                   {"face":"sldprt.F85","xtFace":85,"colour":[0.792,0.820,0.933],
                    "featureId":24,"feature":"Boss-Extrude1","kind":74,"local":4}],
 "bodyAttributes":[{"xtBody":2,"density":1000,"lastFeatureId":24}]}
```

| Field | Meaning |
|---|---|
| `solids`, `faces` | Solid bodies and faces built |
| `unitScale` | Millimetres per file unit; Parasolid stores metres, so 1000 |
| `container` | `"ole2"` or `"modern"` |
| `schema` | Parasolid schema name of the transmit that was read |
| `stream` | The stream the body came from |
| `nodes` | Parasolid node count |
| `approximations` | Carriers that were fitted, each `{what, maxErrorMm}`, below |
| `notes` | Free-text notes (for example that the live partition was empty) |
| `swMassProps` | Mass properties SolidWorks stored in the file (property `SW-MassProp-Config-0`, either container, when present): `volumeMm3`, `areaMm2`, `massKg`, `comMm`. An independent check of the body; `null` in the bundled samples |
| `features`, `configurations`, `material`, `customProperties`, `configurationProperties`, `units`, `versions`, `thumbnail`, `omittedForPrivacy` | The file's metadata, below |
| `faceAttributes`, `bodyAttributes` | Colour and feature of each face, attributes of each body, below |

Every metadata key is always present: a stream the file does not have gives `null` or `[]`, never an error.

## Metadata

Besides the body, a part file holds its feature tree, properties and a preview. They are read from plain streams, with no decoding of SolidWorks' own object archive:

| Field | Stream | Content |
|---|---|---|
| `features` | `swXmlContents/KeyWords` (XML; zlib-wrapped in OLE2) | The feature tree in id order: `{id, name, type, dimensions:[{name, text, value}]}`. `id` is SolidWorks' feature id, the one the faces carry. `type` is the feature type (`Boss-Extrude`, `Extrusion`, `Fillet`, `HoleWizard`, `Revolve`, `Sketch`, `Plane`, ...). `text` is the dimension as SolidWorks displays it, in the document's display units (`10`, `R1`, `360°`, `Ø20`); `value` is its number. The fixed folders of every tree (annotations, lights, bodies, equations, notes, material, favourites, history, user folders) are left out |
| `configurations` | `KeyWords` | `{name, material}` per configuration; `material` is `null` when none is assigned |
| `material` | `swXmlContents/MATERIALTREE` (UTF-16 XML), else `KeyWords` | `{name, densityKgM3}`: the material's name without its library prefix, and its mass density in kg/m^3 (`null` when only the name is known) |
| `customProperties`, `configurationProperties` | `docProps/custom.xml` and `docProps/Config-0-Properties.xml` (modern); the user-defined sections of `DocumentSummaryInformation` and `Config-0-Properties` (OLE2 property sets) | `{name, value}` of the user-defined file properties and of the first configuration's. SolidWorks' own `SW-*` file properties are not listed |
| `units` | `docProps/ISolidWorksInformation.xml` (modern) or the `ISolidWorksInformation` property set | `{system, length, angle, lengthDecimals}`: `MMGS`, `CGS`, `MKS`, `IPS` or `custom`; `mm`, `cm`, `m`, `in`, `ft`, ...; `deg` or `rad`. `null` when the file does not record them (the SolidWorks 2011 samples) |
| `versions` | `_MO_VERSION_<n>` streams; the transmit's schema name | `solidworks`: the file's version number (4700, 9000, 10000, 11000, 15000 in the samples); `solidworksRelease`: its year (2007 + n / 1000: 2011, 2016, 2017, 2018, 2022); `parasolid`: the modeller version of the transmit read (`22.01` to `33.01`) and `parasolidModeller` its number |
| `thumbnail` | `PreviewPNG` | `{mime, width, height, base64}`, the 640 by 480 preview SolidWorks saved, with any text chunks removed; `null` when the file has none |

### Face colour and feature

Every face of the Parasolid body carries attributes SolidWorks wrote: its colour and the id of the feature that built it. The kernel body has no per-face colour, so both are reported in `faceAttributes`, one entry per face built, keyed by the face's id in `body.faces`:

| Field | Meaning |
|---|---|
| `face` | The face id (`sldprt.F<n>`) |
| `xtFace` | The Parasolid face node id `n` |
| `colour` | RGB in 0..1 (attribute `SDL/TYSA_COLOUR`), `null` if the face has none |
| `featureId`, `feature` | The feature that built the face (attribute `FACE_ID_2001`) and its name from `features` |
| `kind`, `local` | Two more fields of `FACE_ID_2001`. Their meaning is inferred from sample files, not documented: `kind` 40 is an end cap, 74 a side face of an extrusion or revolution, 72 a fillet face; `local` numbers the faces a feature made |

In the 41 sample parts every face carries both attributes, and every feature id is in the feature list. The 10 mm cube with a 5 mm hole tags its six box faces with `Boss-Extrude1` and the hole's cylinder with `Cut-Extrude1`.

`bodyAttributes` lists each solid body read: `{xtBody, density, lastFeatureId}`. `lastFeatureId` is the last feature that changed the body. `density` is the value stored on the body (attribute `SDL/TYSA_DENSITY`); its unit is not established (the 2011 samples store 1000 and the later ones 1.0, whatever the material), so use `material.densityKgM3` for mass.

### Privacy

The same files record Windows user names (last saved by, author) and absolute paths (the folder the part was saved in, the save history). The reader does not read those fields or streams (`SW-Folder Name`, `SW-Last Saved By`, `SW-Author`, the `swXmlContents/Features` path list, `_MO_VERSION_<n>/Biography` and `History`, `ModelHeader`). Every string it does return is checked: a value that looks like an absolute path (a drive letter, a UNC prefix, a home directory) or that contains a user name the file records is left out, and `omittedForPrivacy` counts what was left out. The preview PNG is passed on without its text chunks. Tests check this on the fixtures (the user names and folders they record are read from the raw streams and must not appear in the reply) and on synthetic files with a user name and paths in their custom properties.

### A file without a body

When the body cannot be imported (`errors` is set, `body` is `null`), the reply still carries `import` with `format`, `name`, `swMassProps` and the metadata keys above, as long as the container opens. The feature list and thumbnail of a part the kernel cannot build are still available.

## Examples

The four sample parts in `crates/ck-sldprt/tests/fixtures`, imported and checked in this chapter:

| File | Container | Faces | Volume (mm3) | Area (mm2) | `validate` |
|---|---|---|---|---|---|
| 10 mm cube, SolidWorks 2011 | ole2 | 6 | 1000.000 | 600.000 | clean |
| 10 mm cube with a 5 mm hole, SolidWorks 2022 | modern | 7 | 803.650 | 717.810 | clean |
| 10 mm cube, SolidWorks 2022 | modern | 6 | 1000.000 | 600.000 | clean |
| Cone, radius 5, height 10, SolidWorks 2022 | modern | 2 | 261.799 | 254.160 | clean |

The cone's volume is the closed form pi r^2 h / 3 = 261.799. A file that is not a SolidWorks file answers `container.format`: `neither an OLE2 compound file nor a SolidWorks block container`.

## Geometry mapping

| Parasolid | Kernel | Notes |
|---|---|---|
| Plane, cylinder, cone, sphere, torus | The same analytic surfaces | Exact |
| B-surface | B-spline surface | Exact; rational weights are divided out |
| Swept, spun, offset surfaces | Extrusion, revolution, offset surfaces | Exact |
| Rolling-ball blend surface | B-spline surface | Approximated (spine sections, circular cross-sections, cubic interpolation); the deviation is reported |
| Line, circle, ellipse, B-curve | The same curves | Exact |
| Trimmed curve | Basis curve restricted to the range | |
| Intersection curve, surface-parameter curve | B-spline curve | Approximated; the deviation is reported |
| Void region | Skipped | Only solid regions are read; a second shell in a solid region is a cavity |

Parasolid has no seams. The builder adds them: a seam line on cylinders and cones, a meridian on tori and sphere bands, an apex or pole edge on caps, and the standard loops on full spheres and tori. Planar faces order their loops by signed area, curved ones by extent, and each shell is oriented by its signed volume.

Each approximated carrier is listed in `import.approximations` with its maximum deviation. An approximation that deviates by more than 0.05 mm makes the import fail with `geometry.invalid`.

## Errors

Codes are stable strings:

| Prefix | Raised when |
|---|---|
| `container.*` | `format` (neither container), `cfb` (damaged OLE2 structure), `empty`, `no_parasolid` |
| `inflate.*` | Corrupt DEFLATE or zlib data |
| `limit.*` | A safety limit: file 256 MiB, 256 MiB per stream, 768 MiB inflated in total, compression ratio above 2000, 4096 streams, 4 million Parasolid nodes, 50 000 faces |
| `xt.*` | The transmit is empty, has a bad header, schema, stream or topology |
| `topology.*`, `geometry.*` | The faces cannot be assembled, or a surface is outside what can be built |
| `unsupported.*` | `body` (the file holds only sheet or wire bodies), `geometry` (for example a self-intersecting torus) |

The reader does not panic on malformed input. A test feeds truncated, bit-flipped, zeroed and garbage-padded copies of every fixture and requires an error or a body.

## Results on a larger sample set

`tools/sldprt_check.mjs` was run over 41 public sample parts (cubes, cones, spheres, tori, lofts, blends, and the NIST test parts). Only the four fixtures above are in the repository, so the figures below (39 of 41 imported, and the rest) cannot be re-run from the repository. 39 of 41 imported. 33 passed the kernel validator. Of the 37 parts that had a reference volume (computed from a STEP export of the same part), 32 matched to the tolerance (1e-4, or 1e-3 for spline-heavy parts). Parts that failed:

- a part with a self-intersecting (lemon-shaped) torus: `unsupported.geometry`;
- a part with a B-surface whose vertices fall outside the NURBS domain: `geometry.invalid` (a vertex 2.5 mm off its spline surface);
- parts that imported but failed the strict edge-to-surface check (`EDGE_OFF_SURFACE`), all on approximated blend or intersection carriers whose deviations are at most 1e-3 mm, although volume and area match the stored SolidWorks values to 1e-3 or better.

## Limits

- Only the active configuration's body is read. `importSldprt` returns the feature tree as a list (names, types, ids and dimension values, see Metadata); equations and annotations are not read, and other configurations are listed by name only. The editable history comes from the separate [`sldprtHistory`](#feature-history-sldprthistory) request, which rebuilds sketches, extrudes and cuts, fillets, chamfers, holes, shells, patterns, mirrors, revolves and simple lofts where it can. A feature it cannot read back, or a rebuild that does not verify against the file's solid, ends the history there and the rest of the part is the file's exact solid; a part it cannot rebuild at all stays the plain imported solid.
- Only the `(partition)` transmit is applied. The `(deltas)` transmit that follows it is read but not applied: it holds the partition's undo history, ending at the mark the partition already sits at, so the partition is the finished part. On several sample parts SolidWorks' stored volume matches the state one change back (the last cut feature is missing from it), up to 0.4 percent away from the imported solid; the import follows the feature tree, not the stored mass.
- Sheet and wire bodies are not imported. A file with only such bodies answers `unsupported.body`.
- Sheet-metal parts import as the folded solid; the flat pattern and bend table are not read.
- Not supported: self-intersecting tori, B-surfaces with periodic or extended domains whose vertices fall outside the NURBS domain, variable-radius and cliff-edge blends, foreign geometry and mesh bodies.
- The oldest sample is SolidWorks 2011 and the newest SolidWorks 2022. Newer files are untested; the container and schema layers do not depend on the version.
- Speed: the reader takes 5 to 700 ms per part. The `importSldprt` reply also tessellates the body, which dominates on parts with hundreds of faces.

## Feature history: `sldprtHistory`

```json
{"type":"sldprtHistory","name":"part","data":"<base64 of the .SLDPRT file>"}
```

Rebuilds the feature history of the part (sketches, extrudes, fillets, chamfers, holes, shells ...; the crate `crates/ck-swedit`, see `docs/sldprt-editable.md`) as a part document. Read only: no tag, no state. The reply is `{ok, complete, part, outcome}` when the history is editable: `part` is a `ck-part` document that regenerates to the body `importSldprt` reads, `complete` is true when every feature is ours (false: the document ends in an `importBody` feature that stands for the rest of the file and holds the file in `blobs`), `outcome` lists every step and its check. A file that does not rebuild is not an error: the reply is `{ok:false, reason, detail, outcome}` with `reason` `import` (the file cannot be read), `noHistory` (nothing could be read back) or `notEditable`, and the caller keeps `importSldprt`. A missing `data` is an error reply.

## Keeping the built body: `built` and `builtExport`

An `importBody` feature builds its body from the file held in the part's `blobs`; on a spline-rich part that takes minutes. The result can be kept with the document so that the next open does not build it:

```json
{"type":"builtExport","feature":"F9"}
```

answers `{ok, entries:[{feature, key, source, data, rejected?}]}`: the results of `importBody` features built from their file (`source: "built"`) or read from the document (`"saved"`) since the process started, newest last (at most four; `feature` narrows to one). Store `data` (base64 text; it may be compressed for storage) in the part's top-level `built` table under `key`:

```json
{"format":"ck-part","version":1,"blobs":{"<blob key>":"<base64 file>"},"built":{"<key>":"<base64 result>"},"features":[...]}
```

The entry is a cache and never an input. The feature computes the key itself (from the file's blob key, its own id, the kernel version and the format version), and reads the entry only when the key, a checksum and the structure of the stored body all hold; with no entry, a damaged one or one from another kernel version it builds from the file as before and the part comes out the same. The restored body is identical to the built one (same faces and names, same volume, same validity). `rejected` says why an entry under the right key was not used. See `docs/regen-cache.md`, "Saved bodies".

## See also

- [STEP](step.md), [SketchUp](skp.md), [ACIS SAT](sat.md)
- [Feature recognition](recognition.md)
- [Healing](../validation/healing.md)
- [Tolerances](../validation/tolerances.md)
