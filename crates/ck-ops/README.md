# ck-ops

Modeling operations: the feature engine. Depends on ck-topo, ck-geom, ck-core.
This is the largest crate (58 modules) and the bulk of the kernel.

## Module map (pinned by rs_paths.txt)

- `boolean/` — apply, assemble, classify, imprint, mod: solid booleans
  (union/cut/intersect) with imprinting and classification
- `blend/` — fillets/chamfers, 17 files: build, cap, chain, corner, g2,
  inplace, interp, march, overlap, rolling, sheet, split, spring, stripe,
  surface, vertex. Error strings: "the chain through X runs into itself",
  "edge between X shrinks to nothing", "both must move by the same distance"
- `loft/` — centre.rs, guides.rs, smooth.rs, stations.rs + loft.rs
- `sweep.rs`, `helix.rs`, `offset.rs`, `shell.rs`, `draft.rs`, `pipecut.rs`
- `surfacing/` — boundary, extend (+reach/shape), fair (+meow.rs!),
  fill, knit, sheet, thicken, trim, rebuild/ (deviation, edits, net,
  replace, mod)
- `intersect/` — cc (curve-curve), cs (curve-surface), ss (surface-surface)
- `bvh.rs` — bounding volume hierarchy for face/edge queries
- `contain.rs`, `continuity.rs`, `curve_edit.rs`, `guide.rs`, `simplify.rs`

## Interface

Each op takes resolved inputs (bodies, faces, edges, params) and returns a
mutated body graph + errors. Feature-level validation and error strings live
in ck-model/src/features; the geometric work lives here.

## Rebuild notes

- Error strings in kernel.strings pin the exact validation messages per op —
  grep them when implementing an op's failure paths.
- The golden part exercises: extrude, revolve, fillet, hole, helix, mate,
  pattern (cpattern), insertPart.

## Reconstruction status (first pass)

Implemented + tested (`cargo test -p ck-ops`, 69 tests, oracle-pinned
where noted):

| module | API | status |
|---|---|---|
| `bvh` | `Aabb`, `Bvh::{build, query_box, query_ray, overlap_pairs, self_pairs}`, `face_box/edge_box`, `face_bvh/edge_bvh/triangle_bvh`, `ray_triangle` | brute-force-checked |
| `intersect::cc` | `intersect(c1,r1,c2,r2,tol) -> CcResult{hits, overlaps}` (closed forms line/line, line/circle, circle/circle; general seed+Newton) | unit |
| `intersect::cs` | `intersect(c,r,s,tol) -> CsResult{hits, on}`, `line_params` (plane/cyl/cone/sphere quadratics, torus quartic), `newton3` | unit |
| `intersect::ss` | `intersect(s1,s2,bounds,tol) -> SsResult::{Disjoint, Coincident, Curves}`; analytic plane×{plane,cyl,sphere,cone,torus}, cyl∥cyl, sphere/sphere, coaxial cyl/cone/sphere; `march` (grid seeds, Newton corrector, adaptive step, cubic fit refined until ≤ tol/100) | unit |
| `boolean` | `boolean(a,b,Op::{Union,Cut,Intersect},tol) -> BoolOut{body,no_effect}`; `imprint`, `assemble::{fragments, select, build, wire_chains, rebuild_in_order}`, `classify::{point_in_face, classify_point}` | **oracle**: face ids/order/normals, edge keys + order + mids, massProps for box−through-cylinder, blind pocket, slot split (`~k`, 2 lumps), box∪box overlap / flush (coplanar merge), hole on box edge, overhanging boss, front through-hole, cylinder×cylinder cross hole (marched SSI, volume within 3e-8) |
| `boolean::apply` | `linear_instances`, `circular_instances`, `circular_step`, `instance`, `apply_instances` (pattern `@F.k`) | oracle transforms + edge numbering |
| `helix` | `helix(frame, c, r, &HelixParams) -> Helix{o,a,x,y,r,k,angle}`, `pts()`, `to_curve()` | oracle fields/pts (definedBy = `pitchRevs`/`heightRevs`/`heightPitch`, NOT pitchHeight/revsHeight) |
| `blend::build` | `fillet(body, &[(faceA, faceB, at)], r, feature, tol)`, `chamfer(.., d, ..)` (equal distance), `blend_edges`/`Spec`, `find_edge`, `in_place_layout(_opts)`, `BlendError{code, detail}`. Straight edges between planes, and CIRCLE edges between coaxial plane / cylinder / cone faces (tool = revolved meridian profile; boolean cut for convex, union for concave). Multi-edge selections run in ROUNDS (chamfer: an edge joins the round unless it shares a vertex with an edge already in it or an earlier deferred one; fillet: one op), junction numbering for chains | oracle faces/order/edge keys/massProps: fillet 1-2 edges, chamfer 1-4 edges and all 60 ordered subsets of a box ring (`tools/cmp_blend_sets.mjs`), hole / pocket / boss / frustum / ring rims (brep cases `holeRim*`, `boss*`, `frustum*`, `ring*`), flange hub fillet + 4 chamfers. Gaps: fillet 3+ edge selections (face order / leftover-edge slots), 3-edge vertex blends (sphere `V0`), unequal chamfers (oracle: `unsupported`), non-coaxial curved edges |
| `blend::vertex` | `chamfer_vertex(body, at, [d1,d2,d3], feature, tol)`, `polyhedron(polys)` | oracle (25 cases, `tools/cmp_vertex_chamfer.mjs`): nearest vertex to `at`, distances in vertex-edge arena order, face order, edge slots, errors invalidValue / blendTooLarge / chamferVertexEdges; planar faces + straight edges only |
| `surfacing::boundary` | `open_boundaries(body, faces) -> BndLoop{steps, closed}` (free-edge chains, face on the left) | unit |
| `surfacing::knit` | `knit(bodies, solid, tol)` — sew faces by coincident edges, one body | unit (2-sheet share, closed box, name survival; solid → `knitNotClosed`) |
| `surfacing::thicken` | `thicken(sheet, feature, t, Side::{Front,Back,Both}, tol) -> Body` — sheet → solid wall | **oracle**: single sheet + square tube (front/back/both) + cylinder tube, exact face order / `~k` layer suffixes / edge keys+endpoints / walls with mitered corners / volume+area (960, 640, 800, 200, 541.92); errors invalidValue / offsetTooLarge detail |
| `surfacing::trim` | `trim(targets, tool, flip, mutual, tol)` — cut surfaces by a tool plane, keep one side | **oracle**: plane×plane both flips + mutual (kept piece edges in exact order, mutual keeps the uncut tool); errors trimNoIntersection ("the tool does not cut the surface" / mutual variant) / trimNothingLeft |
| `surfacing::fill` | `fill(body, chain, continuity, fairing, strength, feature, tol)` — patch across a boundary chain + closing chord | **oracle**: U-sheet fills (2-edge triangle + 3-edge square), chord stored start→end traversed reversed, patch `F6.F` last, plane patch; degenerate chain → fillFailed ("…parameter area 0.000e0"); fillInvalid ("the edges do not form one chain") |
| `surfacing::fair` | `fair(boundary, Fairing::{MeowNurbs,Plate,Curvature,Variation}, strength)`; `meow::fit` | unit (planar → exact plane for all modes/strengths; non-planar → ruled G0) |
| `surfacing::extend` | `extend(body, edges, Shape, End::{Distance, Surface}, …)` (+ `reach::reach_distance`, `shape::Shape`) | **oracle**: planar edge +5 (rebuilt edges in exact order), half-cylinder rim +3 (rim translated along the axis, rails stretched), to-target-plane exactly y=−7; errors extendNotBoundary / extendNoTarget (10× face size) / extendPointBehind |
| `surfacing::rebuild` | `rebuild(body, faces, deg_u, deg_v, spans_u, spans_v, tol)`; `net::{uniform_knots, greville, fit_surface}`, `deviation::deviation`, `replace::replace_face` | **oracle**: box cap rebuilt deg3/spans2 → spline face, edges untouched, massProps 1000/600 (oracle ~2e-10 off — its net is a least-squares fit); errors rebuildPeriodicFace / rebuildTooFine (GUESS) / rebuildFailed |
| `surfacing::sheet` | `sheet(body, faces)` — keep the faces as an open sheet | **oracle**: deleteFace box probe (5 faces left, the deleted cap's edges open, original edge order) |

| `continuity` | `judge_edge(body, e, &Settings) -> EdgeReport`, `reply(body, msg) -> Json` (the `continuity` query, wired in ck-wasm `query_hook`) | **oracle**: G1-G3 fields, normals, curvature/rate, status/fails, max, worst for fillets (planar / cylinder), box / cylinder corners (`ck-model/tests/continuity_golden.rs`); gap = golden-section noise, bit-exact only when the blend edges use the oracle's line parameterisation |
| `draft` | `draft(body, neutral, dir, faces, angle, tol) -> Result<Body, DraftError{code, detail}>` — plane faces rotate about the neutral hinge, cylinders become cones; `interior_point` | **oracle**: face normals bit-exact for 1..44 degrees (two normalisation passes), cylinder -> cone exact, all error codes + details (`invalidValue`, `draftParallelFace`, `draftTooLarge` ...); vertices/edge pts within ~1e-11 (the oracle's offset engine leaves ~1e-10 solver noise, not reconstructed); NOT ported: offset engine f2871 (own `rebuild` stand-in), `~slab`/`~wedge` retry (f2874), extrusion-surface faces (`draftUnsupported`) |
| `contain` | `Containment::{new, point_in_face, classify_point}` | unit; agrees with the winding classifier on grids (face test is a GUESS, see header) |
| `measure` | `reply(body, msg)` (the `measure` query; `resolve_ref`) | **oracle** (`ck-wasm/tests/query_golden.rs`, 3027 queries): face/edge items, wedge, angle, normal/centre distance exact; pair `distance`/`points` reconstructed from the wasm structure (sample, project, refine top 3) — zero-distance contacts pick another point than the oracle's noise-ordered one |
| `curvature_at` | `reply(body, msg)` (`curvatureAt`) | **oracle**: p/n/k1/k2/d1/d2/gaussian/mean; sphere-pole nudge (1e-7 of the v range); needs the `analysis::surface_curvature` d1 sign (pinned) |
| `ref_curve` | `edge_reply`, `curve_reply`, `nurbs_json` (`refCurve`; sketch curves in ck-wasm `sketch_q`) | **oracle**: edge NURBS (P,p,knots,w), start/end/closed |
| `convert_proj` | `project_curve`, `project_ref`, `apply`, `reply` (`convert`) | **oracle**: line/circle/arc/ellipse/edge-on line entities, point reuse (1e-7), id/seq numbering, `from`; oblique arcs/splines: point count right (natural-spline deviation test), fit points direct samples (GUESS) |
| `interference` | `reply(body)` (lump-pair booleans) | **oracle**: `{bodies,pairs:[]}` on every brep part; pair record is a GUESS (parts never contain overlapping lumps) |
| `match_edges` | `match_edges`, `reply` (`matchEdges`: similar / parallel / tangent) | **oracle**: similar = kind + length + dihedral, parallel = line direction / circle axis, results in wire order; tangent order + tolerance partly GUESS |
| `sweep` | `sweep(feature, &Profile, &[PathSeg], twist) -> Body`; `PathSeg::{Line, Arc, Helix}`, `Motion`, `path_motion`, `sweep_motion` | **oracle**: rigid-motion paths only. Line -> ck-topo extrude, arc about an in-plane axis -> revolve (identical numbering), helix / twist / off-plane arc -> screw sweep (spline side faces, cylinders for lines parallel to the axis, caps listed after the sides, closed profile entities split `S.e1~1/~2`); the path end nearest the profile starts the sweep; `sweepProfileNotNormal` (`|\|t·n\|-1\| > 5e-5`). Volumes agree to ~1e-6 (the oracle's own spline fits), faces/edges/keys exact. Multi-piece paths (mitre unions), guides, general frame-march: not reconstructed |

Everything else (loft/*, pipecut,
curve_edit, guide, simplify, other blend/* files,
`surfacing::rebuild::edits`) is a stub whose header lists its
kernel.strings codes.

### How to wire the surface ops (Gate B, ck-model/src/features)

- `knit {surfaces, solid}`: resolve each surface body (`missingReference`)
  → `knit::knit(&bodies, solid, 1e-6)`; map `KnitError{code, detail}`.
- `thicken {surface, thickness, side}`: resolve the surface body →
  `thicken::thicken(&sheet, &feat_id, thickness, side, 1e-6)`; the result
  solid registers under the feature id. Layer names keep the surface
  feature's face names with `~k` copy suffixes; walls are `<feat>.W<k>`.
- `trimSurface {surfaces | surface, toolSurface | plane, flip, mutual,
  deleteTool}`: resolve bodies → `trim::trim(&targets, &tool, flip,
  mutual, 1e-6)`; register under the feature id; drop the tool body when
  `deleteTool && !mutual` (mutual keeps it inside the result).
- `fill {edges, merge, continuity, fairing, guideStrength}`: resolve the
  edge chain (nearest edge to `at` among the named faces; two-face refs =
  the shared edge) → `fill::fill(&body, &chain, continuity, fairing,
  strength, &feat_id, 1e-6)`; `merge:false` = the patch as its own
  surface body.
- `extend {edges, shape, end, distance, point, target}`: resolve edges →
  `extend::extend(&body, &edges, shape, &end, &feat_id, 1e-6)`; `end`
  Surface targets resolve to a body (see reach.rs); vertex ends
  (`point`) are not reconstructed yet (extendPointBehind covers the
  behind case).
- `freeform` (the rebuildFace preset, `edits: []`): resolve faces →
  `rebuild::rebuild(&body, &faces, deg_u, deg_v, spans_u, spans_v, tol)`;
  the per-edge continuity table (edges[]) applies after the refit
  (rebuildContinuityDegree/Rows/Inner, `edits.rs` — Gate B).
- `deleteFace {faces, mode:"delete"}`: `sheet::sheet(&body, &faces)`;
  the solid slot empties (the result is a surface body).

### Probe commands (oracle, run from the worktree with node)

- thicken: `node tools/qall.mjs <req>.json` with parts
  `surfacePlanar square + thicken t=2 side=front/back/both`,
  `surfaceExtrude square tube + thicken` (16 faces/32 edges, volume 960),
  `circle tube r=5 + thicken t=1.5` (volume π(6.5²−5²)·10 = 541.92);
  errors: t=0 → invalidValue, r=5 back t=6 → offsetTooLarge.
- trim: two perpendicular square sheets (x=0 plane y,z∈[−5,5]; z=0 plane
  x,y∈[−6,6]), trimSurface flip=false/true, mutual.
- fill: U-sheet (bottom + 3 walls), fill 2 and 3 top rims (merge, G0);
  the same edge twice → fillFailed "…parameter area 0.000e0".
- extend: square sheet + extend the y=0 edge 5; half-cylinder + extend
  the z=0 rim 3; extend end:"surface" to a plane y=−7 (target ref
  `{kind:"surface", id:"F5"}`).
- rebuild: box + freeform `{faces:[F2.C1], degreeU/V:3, spansU/V:2,
  edits:[]}` → the cap becomes a spline, massProps 1000/600.
- sheet: box + deleteFace F2.C1 → a 5-face surface body.

### Findings for other crates
- Wire edge `#n` after an IN-PLACE op (fillet) is the edge's arena index
  (gaps for dead edges: `#0..#4,#6,#8..#11,#14,#16..`); after booleans /
  patterns the body is rebuilt and numbered in face-walk order (C0, C1,
  S.e1 … → box rail `S.e1|S.e4` is #9 in a pattern result).
- The original kernel's periodic faces have NO seam edges: closed SSI
  circles start at t = 0.62·2π of the cylinder frame (box C1 ∩ through-cut
  cylinder starts at (16.355,·,-6.577)), and arcs are not split where our
  seam lands. We keep ck-topo seams internally; the merge of wire edges
  through seam-only vertices now lives in ck-topo `Body::wire_chains`
  (used by `edges_json` and the STEP writer; `assemble::wire_chains`
  delegates) — fixes the `boss` probe. After an in-place op use
  `Body::seamless_index` for the oracle's arena `#n` (no seam slots).
- `~k` of split faces (ck-topo `number_split_faces`, pinned on closed
  spline extrudes): rank by the centre of the face's boundary
  control-net box, (x, y, z) world-lexicographic. `assemble` still ranks
  boolean fragments by vertex centroid (GUESS) — worth re-checking
  against that rule. Remaining
  known mismatch: start point / `mid` of closed SSI circles that cross a
  seam (through cuts) — ours start at the seam.
- extrude `op:"cut"` goes AGAINST the sketch normal unless `flip`
  (ck-model): unflipped blind cut on the box's C0 sketch → `cutNoEffect`.
- `cutNoEffect` returns the original body unchanged (original edge order).
- cpattern: `equal` → step = angle/count when angle = 360 else
  angle/(count-1); not equal → step = angle.
