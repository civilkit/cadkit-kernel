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
| Intermediate form | `ir.rs` | Bodies, regions, shells, faces, loops, fins, edges, vertices, and their surfaces and curves, in metres |
| Kernel body | `build.rs` | Converts to millimetres and builds the body: solid regions only, edge ranges from the vertices, fin senses to coedge directions, seams on periodic faces, loop ordering, shell orientation |

### Containers

| Kind | `import.container` | Files | Detection |
|---|---|---|---|
| OLE2 compound file | `"ole2"` | SolidWorks 2014 and earlier | First 8 bytes `D0 CF 11 E0 A1 B1 1A E1`; streams hold zlib frames |
| Block container | `"modern"` | SolidWorks 2015 and later | A block marker `14 00 06 00 08 00`; every block carries a name, sizes, a CRC-32 and raw DEFLATE data. A block is accepted only if its data inflate to the declared size and the CRC matches |

Any other file is `container.format`. The geometry sits in Parasolid transmits inside the streams, wrapped in GUIDs. The reader takes the live partition (`Contents/Config-0-Partition`). If it holds no solid, as in parts whose geometry is an imported body, the solid bodies of the free-standing part transmits are used instead (and a note says so).

## Reply fields

Real output for a 10 mm cube saved by SolidWorks 2011 (OLE2 container):

```json
{"format":"sldprt","name":"sw2011_cube_10mm_ole2.sldprt","solids":1,"faces":6,
 "unitScale":1000,"container":"ole2","schema":"SCH_2201236_20000_13006",
 "stream":"Contents/Config-0-Partition","nodes":165,
 "approximations":[],"notes":[],"swMassProps":null}
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
| `swMassProps` | Mass properties SolidWorks stored in the file (modern container, when present): `volumeMm3`, `areaMm2`, `massKg`, `comMm`. An independent check of the body; `null` in the bundled samples |

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

- Only the active configuration's body is read. The feature tree, sketches, dimensions, equations, configurations and annotations are not.
- Only the `(partition)` transmit is applied. The `(deltas)` transmit that follows it in the stream is not decoded, so a part whose last edits live only in deltas imports in its earlier state (one sample part is 0.45 percent short of its stored volume).
- Sheet and wire bodies are not imported. A file with only such bodies answers `unsupported.body`.
- Sheet-metal parts import as the folded solid; the flat pattern and bend table are not read.
- Not supported: self-intersecting tori, B-surfaces with periodic or extended domains whose vertices fall outside the NURBS domain, variable-radius and cliff-edge blends, foreign geometry and mesh bodies.
- The oldest sample is SolidWorks 2011 and the newest SolidWorks 2022. Newer files are untested; the container and schema layers do not depend on the version.
- Speed: the reader takes 5 to 700 ms per part. The `importSldprt` reply also tessellates the body, which dominates on parts with hundreds of faces.

## See also

- [STEP](step.md), [SketchUp](skp.md), [ACIS SAT](sat.md)
- [Feature recognition](recognition.md)
- [Healing](../validation/healing.md)
- [Tolerances](../validation/tolerances.md)
