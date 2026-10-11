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

Implemented + tested (`cargo test -p ck-ops`, 32 tests, oracle-pinned
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
| `blend::build` | `fillet(body, &[(faceA, faceB, at)], r, feature, tol)`, `find_edge`, `fillet_line_edge`, `BlendError{code, detail}` | oracle faces/order/massProps for 1 edge, 2 edges, vertical edge; errors invalidValue / missingEdge / blendOverflow detail |

Everything else (sweep, loft/*, shell, offset, draft, pipecut, surfacing/*,
contain, continuity, curve_edit, guide, simplify, other blend/* files) is a
stub whose header lists its kernel.strings codes.

### Findings for other crates
- Wire edge `#n` after an IN-PLACE op (fillet) is the edge's arena index
  (gaps for dead edges: `#0..#4,#6,#8..#11,#14,#16..`); after booleans /
  patterns the body is rebuilt and numbered in face-walk order (C0, C1,
  S.e1 … → box rail `S.e1|S.e4` is #9 in a pattern result).
- The original kernel's periodic faces have NO seam edges: closed SSI
  circles start at t = 0.62·2π of the cylinder frame (box C1 ∩ through-cut
  cylinder starts at (16.355,·,-6.577)), and arcs are not split where our
  seam lands. We keep ck-topo seams; `assemble::wire_chains` merges wire
  edges through seam-only vertices (fixes the `boss` probe). Remaining
  known mismatch: start point / `mid` of closed SSI circles that cross a
  seam (through cuts) — ours start at the seam.
- extrude `op:"cut"` goes AGAINST the sketch normal unless `flip`
  (ck-model): unflipped blind cut on the box's C0 sketch → `cutNoEffect`.
- `cutNoEffect` returns the original body unchanged (original edge order).
- cpattern: `equal` → step = angle/count when angle = 360 else
  angle/(count-1); not equal → step = angle.
