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

See the table at the end of this file (regenerated with `tools/sldprt_check.mjs`), the gaps below and `THIRD_PARTY.md` for sources.
