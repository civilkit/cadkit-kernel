# Kernel gaps against OCCT: prioritised roadmap

Written 2026-10-16 from `docs/roadmap.md`, `docs/accuracy-vs-occt.md` and what the hull, NIST STEP and SolidWorks work found.
Sizes: S = days of agent time, M = about a week, L = several weeks. Status is as of this date.

| # | Gap | What we have today | What OCCT does | Size | Status |
|---|---|---|---|---|---|
| 1 | Shape healing and tolerant geometry | 4 of 11 NIST STEP files import but fail `validate` (B-spline faces about 5e-3 off the exact surfaces they meet); SolidWorks reader blends and intersection curves fail the strict edge check | `ShapeFix`, tolerant edges and vertices, sewing | M | **Started** (agent: heal) |
| 2 | Boolean robustness and scope | `booleanFailed` on coincident or tangent faces, wall-tangent holes; no body-level boolean; no multi-body documents | fuzzy tolerance, n-ary operations, glue and split modes | M | **Started** (agent: boolean corpus, fuzzy, body-level) |
| 3 | Fillet and chamfer remainders | concave three-edge corners done; two-distance and angle chamfer unsupported; variable radius fails at corners; mixed vertices not exact; conic and curvature fillets open | most of these | M | Queued |
| 4 | Offset, shell and thicken of free-form shapes | blunt lofted hulls shell; a body ending in an apex is refused near the tip; thicken on a B-spline sheet unsupported | general thick solid and offset | M | Queued (touches the hull work) |
| 5 | Data exchange breadth | STEP geometry in and out; SolidWorks reader in progress; DXF 2D; cadkit planned for DWG and DGN | STEP with colours, names, assemblies and PMI; IGES; glTF with structure | M | SolidWorks reader finishing; cadkit planned |
| 6 | Sweep and loft options | loft with guides and point ends; sweep; no closed loft, no sweep normal control, multi-section sweeps partial | full pipe-shell family | M | Open |
| 7 | Hidden-line drawings | `drawing` lacks silhouette lines | full hidden-line removal | M | Open |
| 8 | Free-form surface modelling | fill, knit, trim, spline surfaces; no network or constrained plate surfaces; surface-surface intersection on arbitrary NURBS is the weak spot | plate surfaces, Coons, constrained approximation | L | Open |
| 9 | Mesh to B-rep and sewing | none | sewing and mesh conversion | M | Open |
| 10 | Measurement | mass properties, interference, curvature, `probe`, `section` | minimum distance between shapes, minimum thickness, extrema | S | Open |
| 11 | Parallelism | single thread in wasm; native threads gave 1.38x in a trial | multithreaded booleans | L | Open |

## Order and reasoning

1. **Healing (1)** comes first: it decides how far the SolidWorks reader and STEP import get on real parts, and a toleranced
   edge model is a dependency for most of the rest.
2. **Booleans (2)** in parallel: most daily modelling and every import-then-edit flow depends on them.
3. **Fillet and chamfer remainders (3)**, then **offset (4)** which also unblocks the pointed open hull.
4. Items 5 to 10 follow demand: gaps 6 and 7 matter for hull and drawing work; 10 is cheap and agent-useful.

## Gates for every item

Wire 280/280, valid 277/319, vol 319/319 and step 16/21 must not drop; large sample check ok; a corpus or fuzz script per item;
builds and checks run on the CI box (`tools/remote_build.sh`, `docs/remote-build.md`).
