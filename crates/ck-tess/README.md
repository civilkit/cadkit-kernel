# ck-tess

Tessellation: B-rep faces to the wire display mesh. Depends on ck-core, ck-geom, ck-topo (ck-json dev-only).

## API

```rust
use ck_tess::mesh::{tessellate, tessellate_with, body_mesh, body_tol, edge_polyline, Tess, Mesh};
let t: Tess = tessellate(&body);                    // regen tolerance from the body bbox
let edges = body.edges_json(&|b, e| t.edge_polyline(b, e)); // body.edges[].pts
let m: &Mesh = &t.mesh;   // positions/normals/mids: Vec<f32>, tri_face: Vec<i32>, silhouette: Vec<f32>, failed: Vec<usize>
let (lo, hi) = m.bbox().unwrap();                    // wire body.box (f32-rounded bounds)
```
- `curvature::Tol { chord, angle }`, `Tol::for_bbox`, `curve_params`, `arc_segments`, `periodic_cut`, …
- `face::mesh_face(body, face, &params, cut, &tol) -> FaceMesh` (uv, pts, nrm, CCW-outward tris).
- `cdt::triangulate(pts, segs)` (exact predicates, parity classification, `insert_interior`).

## Pinned from the original (instrumented kernel.wasm, see below)

| what | rule |
|---|---|
| tolerance (f207) | `chord = max(1e-3, 4e-4·|bbox diag|)`, `angle = 0.3 rad` |
| circle sampling (f3764/f3765) | `step = min(r<=tol ? π/2 : 2acos(1−tol/r), 0.3)`, `n = clamp(ceil(sweep/step),1,1e5)`, `t_i = t0+((t1−t0)·i)/n` |
| ellipse / spline | ellipse formula decoded; spline = knot spans × max(2,deg+1) + adaptive (GUESS) |
| edge refinement | per coedge uv-chord bisection (depth 12, `uvtol = tol/scale`, scale GUESS), then per face normal-angle bisection (depth 8, 0.3) |
| periodic faces (ck-topo region.rs f3444) | cut at the middle of the largest gap of the sorted, wrapped boundary-sample u's (wrap gap first, strictly-larger replaces). Adds one param to each crossing edge = the "one bisected segment" of every full circle in oracle `pts` |
| mesh layout | unwelded: positions/normals/mids 9 f32/tri, triFace 1 i32/tri |
| mids | per tri edge (01,12,20): `n(uv mid) − (n_a+n_b)/2` (shader line 3648 of app.pretty.js) |
| silhouette | 12 f32/record `[a,b,nA,nB]`: same-face welded edges, strictly convex; concave/flat excluded |
| faces | CDT of boundary samples only for planes/ruled-aligned faces (cap N−2, cylinder side 2N) |

Exact oracle matches (tests/oracle_mesh.rs, tests/oracle_mesh.json): box, holed block, cylinder,
D-extrude, revolve 360/90, revolve-on-axis — per-face triangle counts, edge `pts` counts (box/cyl
bit-level polylines), silhouette counts, box/normals/mids; box triangles identical, cylinder side
identical but one quad diagonal. Sphere / torus / cone side: watertight, within 2× of oracle
counts (refinement acceptance rule is a GUESS; oracle cone uses a v-bisection grid, 1167 vs our ~1480).

## Open / GUESS
- Triangle ORDER and vertex rotation inside triangles differ from the oracle (its CDT f3325/f3324 not decompiled).
- Interior refinement criterion (centroid + edge-midpoint chord height, normal angle).
- uv scale for the coedge chord refinement (f3573); near-vertex param filter (vertex tolerances) skipped.
- Cross-face smooth edges in the silhouette; silhouette record order.
- Cut placement matches bit-exactly only where ck-topo pcurves equal the original's (u = t lines do).
- CDT core can `panic!` on corrupt topology (wasm panic=abort).

## Instrumenting the oracle
`tools/wat_instrument.py` adds an `env.ck_log(i32,f64,f64,f64,f64)` import and log calls into
kernel.wat, `wat2wasm --no-check` rebuilds in ~1 s, `tools/runlog.mjs <wasm> <part.json>` prints logs.
`tools/meshdump.mjs part.json out.json` decodes the `$buf` arrays of a regen reply.
