# SolidWorks part import (`ck-sldprt`, request `importSldprt`)

`crates/ck-sldprt` reads a SolidWorks part file (`.SLDPRT`) and builds an `ck-topo` body directly; there is no STEP hop. The
wasm request `importSldprt` returns the same kind of result as `importStep` and leaves the body in the tag, so
`massProps`, `validate`, `inspect`, `step` and the rest work on it.

```
{type:"importSldprt", tag:"t", name:"part", data:"<base64 of the .SLDPRT file>"}
 -> result: {body, errors:[], import:{format:"sldprt", name, solids, faces, unitScale:1000,
             container:"modern"|"ole2", schema, stream, nodes, approximations:[{what, maxErrorMm}], notes:[]}, ...}
 -> failure: body:null, errors:[["import", {code, params:{message}}]] (the tag keeps its previous state)
```

Check script over a directory of parts: `node tools/sldprt_check.mjs --dir DIR [--refs DIR] [--filter S] [--strict]` prints,
per file, faces, volume, area, bbox, validity, milliseconds and the volume compared with a reference (see its header).

## Pipeline

| Phase | Module | What it does |
|---|---|---|
| A container | `container.rs`, `inflate.rs` | OLE2 / CFB (SolidWorks 2014 and older) or the modern block container (2015+) to named streams; own bounded DEFLATE / zlib decoder, CRC-32; GUID-wrapped Parasolid transmits inside the streams |
| B XT + IR | `xt.rs`, `ir.rs` | Parasolid XT neutral-binary reader (base schema SCH_13006 plus the embedded edit scripts), then the intermediate representation: bodies, regions, shells, faces, loops, fins, edges, vertices, surfaces and curves in metres |
| C kernel body | `build.rs` | IR to `ck-topo::Body` in millimetres: solid regions only (the void region is skipped), carriers, edge parameter ranges from the vertices, fin senses to coedge reversals, seams for periodic faces, loop ordering, pcurves, shell orientation |
| D request | `ck-server` `importSldprt`, `ck-wasm` hook | base64 in, regen-shaped result out |

### Version detection (`container::detect`)

1. first 8 bytes `D0 CF 11 E0 A1 B1 1A E1`: OLE2 compound file, SolidWorks 2014 and older (zlib frames inside the streams);
2. else the block marker `14 00 06 00 08 00` occurs: modern container (2015+; bytes 4..8 are the big-endian version word
   `00 00 00 04`). Every block is `marker, type u32, crc32 u32, compressed u32, inflated u32, name length u32, nibble-swapped name,
   raw DEFLATE`. A marker hit is accepted only if the sizes are plausible, the DEFLATE data inflates to exactly the declared
   size and the CRC-32 of the result matches, so the fixed-size cache cells between blocks (which reuse the marker) and chance
   matches are dropped (`skipped_hits`);
3. otherwise `container.format`.

### Limits and robustness (`error::Limits`, all configurable)

File size 256 MiB, 256 MiB per inflated stream, 768 MiB inflated in total, 4096 streams, 4 000 000 XT nodes, 50 000 faces. The
decoder writes into a capped buffer, so a decompression bomb is an error (`limit.*`), not an allocation. CFB chains are checked
for cycles and bounds. The crate is `#![forbid(unsafe_code)]`; every read is bounds checked and errors carry stable codes
(`container.*`, `inflate.*`, `limit.*`, `xt.*`, `topology.*`, `geometry.*`, `unsupported.*`). `tests/import.rs::fuzz_no_panic`
feeds 130 truncated, bit-flipped, zeroed and garbage-padded copies of every fixture and requires an error or a body, never a panic.

### Which body is "the part"

The live partition (`Config-0-Partition`, `(partition)` transmit) wins; its body is the part. The `(deltas)` transmit that
follows it in the same stream is not applied (its node types are not decoded): for the sample parts the partition alone is the
final body, but a part whose edits live only in deltas would import in its pre-edit state (see gaps). The ghost partition is used
only as a last resort. If the partition holds no solid (parts whose geometry is an imported body), the solid bodies of the
free-standing part transmits are taken instead, as the open readers do.

### XT to kernel mapping

| XT | Kernel | Notes |
|---|---|---|
| PLANE, CYLINDER, CONE, SPHERE, TORUS | `Surface::{Plane,Cylinder,Cone,Sphere,Torus}` | exact; cone semi-angle = atan2(sin, cos) |
| B_SURFACE (NURBS) | `Surface::BSpline` | exact; rational weights divided out of the homogeneous vertices |
| SWEPT_SURF, SPUN_SURF, OFFSET_SURF | `Extrusion`, `Revolution`, `Offset` | exact; offset sign follows the base surface sense |
| BLENDED_EDGE (rolling ball) | `BSpline` | approximated: spine sections, circular cross-sections, cubic interpolation; deviation reported |
| LINE, CIRCLE, ELLIPSE, B_CURVE | `Curve::{Line,Circle,Ellipse,BSpline}` | exact |
| TRIMMED_CURVE | basis curve restricted to the range | |
| INTERSECTION (chart points) | `BSpline` | chart points projected onto both supports and refined; deviation reported |
| SP_CURVE | `BSpline` | surface evaluated along the parameter curve; deviation reported |
| tolerant edges (curve only on the fins) | fin curve | |
| face.sense == surface.sense | `Face.sense` | |
| REGION type `V` (void) | skipped | only `S` regions' shells are read; a second shell in a solid region is a cavity, oriented inward |
| fin.vertex is the END vertex; the `+` fin gives the edge direction | | verified on the cube fixtures |

Fix-up passes in `build.rs`: periodic faces have no seam in Parasolid, so two wrapping loops are joined by a seam line (cylinder,
cone) or meridian circle (torus, sphere band), a cone cap or sphere cap gets a seam to the apex / pole, full spheres and tori get
the standard two / four coedge loop; when the two loops share no vertex azimuth the building runs twice, the first pass recording the
edge split points and the second pass splitting those edges for every face that uses them; planar faces order loops by signed
area (outer first), others by extent; each shell's orientation is fixed by its signed volume (outer positive, cavities negative).

## Results

Run of `node tools/sldprt_check.mjs` over the 41 public sample parts (46 files in the spike set minus the private ones): 39 import, 33 are valid in the kernel, 32 of 37 volume comparisons match. Expected volume = the sldprt2step STEP when it imports valid, else SolidWorks' own STEP (`sw`), tolerance 1e-4 (1e-3 for spline-heavy parts). Notes: the SolidWorks STEP of the cone imports with half the volume in the kernel (the true value is pi r^2 h / 3 = 261.80, which this reader gives); on nist_ftc_06 and nist_ftc_10 this reader is closer to the volume stored in the file by SolidWorks (3.3247e6 and about 1.883e5 mm3) than sldprt2step; the `SW area diff` column compares with the area stored in the file (NIST parts only; the older pendulum parts carry stale mass properties). Timings include tessellation of the reply, on a loaded machine.

| part | faces | volume mm3 | area mm2 | valid | ms | expected (source) | rel diff | SW area diff |
|---|---|---|---|---|---|---|---|---|
| drone_157 | 4 | 18.11442 | 68.65951 | ok | 87 | 18.11442 (s2s) | 0.0e+0 | - |
| drone_qfn | 6 | 36 | 96 | ok | 7 | 36 (s2s) | 0.0e+0 | - |
| dump_SW2011_C00_cube_10mm | 6 | 1000 | 600 | ok | 7 | 1000 (sw) | 0.0e+0 | - |
| dump_SW2011_C03_cube_fillet_1mm | 7 | 997.854 | 595.2788 | ok | 8 | 997.854 (s2s) | 0.0e+0 | - |
| dump_SW2011_C04_cube_hole_5mm | 7 | 803.6505 | 717.8097 | ok | 12 | 803.6505 (s2s) | 0.0e+0 | - |
| dump_SW2011_C10_cube_shell_1mm | 11 | 424 | 888 | ok | 6 | 424 (s2s) | 0.0e+0 | - |
| dump_SW2011_C13_cone | 2 | 261.7994 | 254.1602 | ok | 31 | 130.8997 (sw) | 1.0e+0 DIFF | - |
| dump_SW2011_C14_sphere | 1 | 523.5988 | 314.1593 | ok | 124 | 523.5988 (sw) | 0.0e+0 | - |
| dump_SW2011_C15_torus | 1 | 394.7842 | 394.7842 | ok | 134 | 394.7842 (sw) | 0.0e+0 | - |
| dump_SW2011_C16_loft_spline | 6 | 893.8272 | 530.7091 | INVALID EDGE_OFF_SURFACE | 101 | 893.8272 (s2s*) | 3.8e-16 | - |
| dump_SW2011_C20_cylinders_crossed | 8 | 2474.926 | 1170.796 | ok | 114 | 2474.926 (sw) | 5.6e-9 | - |
| dump_SW2011_C21_cylinder_fillet | 4 | 778.9574 | 457.3102 | ok | 44 | 778.9574 (sw) | 5.8e-16 | - |
| dump_SW2022_C00_cube_10mm | 6 | 1000 | 600 | ok | 10 | 1000 (s2s) | 0.0e+0 | - |
| dump_SW2022_C03_cube_fillet_1mm | 7 | 997.854 | 595.2788 | ok | 13 | 997.854 (s2s) | 0.0e+0 | - |
| dump_SW2022_C04_cube_hole_5mm | 7 | 803.6505 | 717.8097 | ok | 17 | 803.6505 (s2s) | 0.0e+0 | - |
| dump_SW2022_C10_cube_shell_1mm | 11 | 424 | 888 | ok | 12 | 424 (s2s) | 0.0e+0 | - |
| dump_SW2022_C13_cone | 2 | 261.7994 | 254.1602 | ok | 39 | 130.8997 (sw) | 1.0e+0 DIFF | - |
| dump_SW2022_C14_sphere | 1 | 523.5988 | 314.1593 | ok | 178 | 523.5988 (sw) | 0.0e+0 | - |
| dump_SW2022_C15_torus | 1 | 394.7842 | 394.7842 | ok | 173 | 394.7842 (sw) | 0.0e+0 | - |
| dump_SW2022_C16_loft_spline | 6 | 739.8188 | 474.7807 | INVALID EDGE_OFF_SURFACE | 72 | 739.8188 (s2s*) | 1.5e-16 | - |
| dump_SW2022_C20_cylinders_crossed | 8 | 2474.926 | 1170.796 | ok | 123 | 2474.926 (s2s) | 2.6e-9 | - |
| dump_SW2022_C21_cylinder_fillet | 4 | 778.9574 | 457.3102 | ok | 36 | 778.9574 (s2s) | 0.0e+0 | - |
| nist_ctc_01 | 139 | 1.4643e+7 | 8.0797e+5 | ok | 386 | 1.4627e+7 (sw*) | 1.1e-3 DIFF | 1.1e-3 |
| nist_ctc_02 | - | - | - | FAIL | 468 | unsupported.geometry: self-intersecting (apple/lemon) torus, major 0.0007339792212194855, minor 0.006 | | |
| nist_ctc_03 | 139 | 3.3139e+5 | 3.5496e+5 | ok | 195 | 3.3139e+5 (s2s) | 1.4e-15 | 5.3e-4 |
| nist_ctc_04 | 406 | 1.7519e+7 | 1.0231e+6 | ok | 1210 | 1.7519e+7 (sw) | 1.1e-5 | 5.8e-4 |
| nist_ctc_05 | 156 | 1.2695e+7 | 8.5799e+5 | INVALID EDGE_OFF_SURFACE | 1434 | - | - | 3.8e-3 |
| nist_ftc_06 | 144 | 3.3235e+6 | 2.8342e+5 | ok | 338 | 3.2871e+6 (s2s) | 1.1e-2 DIFF | 2.7e-3 |
| nist_ftc_07 | - | - | - | FAIL | 438 | geometry.invalid: face #5276: vertex (149.4477, -0.3860, 107.9419) is 2.543e0 mm off its spline surface | | |
| nist_ftc_08 | 249 | 5.0363e+5 | 1.9171e+5 | ok | 1436 | 5.0363e+5 (s2s) | 3.3e-11 | 3.1e-7 |
| nist_ftc_09 | 125 | 1.3645e+5 | 96614.23 | ok | 470 | 1.3645e+5 (s2s) | 0.0e+0 | 3.9e-3 |
| nist_ftc_10 | 206 | 1.8832e+5 | 48495.7 | INVALID EDGE_OFF_SURFACE | 1437 | 1.8786e+5 (s2s) | 2.4e-3 DIFF | 7.5e-4 |
| nist_ftc_11 | 6 | 5129.386 | 5474.478 | ok | 64 | 5129.386 (s2s) | 1.4e-15 | - |
| pend_small_motor_385 | 88 | 23560.76 | 5285.207 | ok | 72 | 23560.76 (s2s) | 0.0e+0 | - |
| pend_bearing_holder | 26 | 15287.81 | 5476.146 | ok | 54 | 15287.81 (s2s) | 1.2e-16 | - |
| pend_bolt | 14 | 8356.315 | 5528.882 | INVALID VERTEX_OFF_CURVE,EDGE_OFF_SURFACE | 3741 | 8356.348 (s2s*) | 3.9e-6 | - |
| pend_encoder | 28 | 41653.8 | 10075.57 | ok | 198 | 41653.8 (s2s) | 0.0e+0 | - |
| pend_spoked_wheel_12T | 73 | 71323.69 | 31229.46 | ok | 392 | - | - | - |
| pend_mount_table_sharft_holder | 23 | 7848.474 | 3580.293 | INVALID EDGE_OFF_SURFACE | 4016 | 7848.447 (s2s*) | 3.4e-6 | - |
| pend_pole | 3 | 3392.92 | 2318.495 | ok | 15 | 3392.92 (s2s) | 0.0e+0 | - |
| pend_table_mount | 56 | 52608.15 | 19282.96 | ok | 80 | 52608.15 (s2s) | 2.8e-16 | - |

Not imported: nist_ctc_02 (lemon torus), nist_ftc_07 (B-surface domain). Invalid in the strict validator: the two loft parts, nist_ctc_05, nist_ftc_10, pend_bolt, pend_mount_table_sharft_holder (all EDGE_OFF_SURFACE on approximated carriers, deviations at most 1e-3 mm). Private test parts (not in the repo): three import and match sldprt2step; one file made only of surface bodies answers unsupported.body; one with a variable blend is refused (blend deviation above 0.05 mm).

## Gaps (what this reader does not do)

| Area | State |
|---|---|
| Feature tree, sketches, dimensions, PMI, configurations | not read; only the body of the first / active partition (`Config-0-Partition`) is imported, other configurations (`Config-N-*`) are separate streams that are ignored. The part's stored mass properties are returned in `import.swMassProps` (from `docProps/ISolidWorksInformation.xml`) as an independent check |
| Sheet metal | the folded body imports as an ordinary solid; flat pattern, bend table and sheet-metal features are not read |
| Surface (sheet) and wire bodies | not imported; a file with only such bodies answers `unsupported.body` (the CANivore-style file made of 11 surface bodies) |
| Parasolid deltas | the `(deltas)` transmit that follows the partition is not decoded (its node types are not in the base schema and the update rule is unknown to the open readers). A part whose last edits live only in the deltas would import in its earlier state; `nist_ftc_09` is 0.45 percent short of SolidWorks' own volume, as is the open `sldprt2step` reader |
| SolidWorks 2023 and later | only SW2022 samples (modern container, display-list version 15000) and older were available; the container and XT layers are version agnostic (schema edit scripts), but newer files are untested |
| Surfaces | not supported: self-intersecting (apple / lemon) tori (`nist_ctc_02`), B-surfaces with periodic or extended domains whose vertices fall outside the NURBS domain (`nist_ftc_07`), cliff-edge and variable-radius blends, foreign (PE) geometry, mesh bodies |
| Approximation | rolling-ball blends, intersection curves and SP-curves are interpolated B-splines; each is reported with its maximum deviation and refused above 0.05 mm. A few parts then fail the kernel validator's tight edge / surface check (`EDGE_OFF_SURFACE`, tolerance about 1e-5 relative) although volume and area match SolidWorks to 1e-3 or better |
| Spheres | a hemisphere bounded by a great circle through the poles is re-expressed with a new axis; patches whose boundary arcs cannot be pole-free for any of 96 candidate axes are left alone |
| Speed | the reader itself takes 5 to 700 ms per part (largest NIST parts 0.3 to 0.7 s); `importSldprt` also tessellates the result for the reply, which dominates on 400-face parts (several seconds, the same cost as `importStep` of the same part) |
