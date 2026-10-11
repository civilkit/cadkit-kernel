# ck-topo

B-rep topology: arena-based solid model. Depends on ck-core, ck-geom, ck-json.

## Modules

| module | contents |
|---|---|
| `body.rs` | `Body { vertices, edges, coedges, loops, faces, shells, lumps }` (Vec arenas + typed ids, `alive` flags), `FaceName`/`Role` id minting + parsing, `edge_key`, wire export `faces_json` / `edges_json`, `append`, `push_instance`, rigid `transform` |
| `make.rs` | `Profile` (frame + regions of oriented `ProfileSeg` loops), generic sweep builder, `extrude(feature, &Profile, ExtrudeOpts)` (blind / flipped / mid / custom dir / draft) |
| `make/revolve.rs` | `revolve(feature, &Profile, axis_o, axis, a0, a1)` (partial or 360°, on-axis vertices/segments) |
| `orient.rs` | `flip_face`, `reverse_loop`, `loop_uv_area`, `loops_consistent`, `fix_planar_loops`, `orient_shell` / `orient_body` |
| `check.rs` | `check(&Body, tol) -> Vec<Issue>` (open / non-manifold / unused edges, bad pairing, broken loops, vertex-off-curve, pcurve-off, degenerate, bad range), `euler`, `is_closed_manifold` |
| `mass.rs` | `face_integral` (exact Green's-theorem face integrator), `Moments`, `mass_props`, `mass_props_json` (the `massProps` reply), `signed_volume`, `eigen_sym` |
| `region.rs` | `winding_number`, `classify -> Inside/Outside/On`, `components`, `rebuild_lumps` (void shells attach to their host lump) |
| `edit.rs` | `split_edge`, `split_face` (new edge between two loop vertices), `delete_face`, `merge_faces` / `merge_coplanar`, `reverse_line_edge`, `gc` |

## Conventions

- Face: `Surface` + `sense` (outward = natural normal Su×Sv when true).
- Every loop is CCW about the OUTWARD normal; first loop = outer.
- Coedge: `rev` = traverses its edge backwards; `pcurve` is parameterised
  by the EDGE parameter. All built faces carry exact pcurves; mass and
  orientation fall back to projection when a pcurve is missing.
- Seam edge = both coedges on one face (periodic faces). Seams are never
  exported. Pole vertices (revolve on axis) have no rail edge.
- Arenas never shrink; edits flag `alive=false`. Export order = creation
  order, which is what pins the wire `#N`.

## Pinned naming grammar (oracle probes + oracle-flange.json)

- face id `<feature>.<role>[~<region>]<suffix>`
  - `S.<entity>`: side face swept from sketch entity (extrude AND revolve),
    `h<n>` entities come from hole features
  - `C0` / `C1`: caps; C0 = LOW end (extrude: min height along the sketch
    normal — also when flipped; revolve: start angle), C1 = high end.
    360° revolves have no caps.
  - `B<k>`: blend/chamfer faces (fillet F10.B0 torus, chamfer F11.B0..B3 cones)
  - `~k`: 1-based region index on caps, only when the profile has >1
    region (`F2.C0~1`, `F2.C0~2`; inside an instance `F2.C0~1@F1.1`);
    one lump per region.
  - suffix: `@<pattern feature>.<k>` for pattern copies (k from 1, the seed
    has none) then `@<insertPart feature>.<n>` for assembly instances:
    `F8.S.h1@F9.1@F1.1`.
- `meta = {feature, entity}`, `entity` only for `S.` faces and includes the
  suffix (`h1@F9.1@F1.1`).
- face JSON key order: `id, kind, planar, meta[, normal][, lump]`;
  `normal` (outward) only for planar faces, `lump` only when the body has
  more than one lump.
- edge key `<a>|<b>#<n>`: face ids sorted bytewise (`F11.B1@F1.1|F6.S.e6@F1.1`,
  `F2.S.e10|F2.S.e9`), `n` = index in the exported edge list. `faces`
  array = the same sorted pair. Edge JSON key order: `pts, faces, key,
  mid, circle{c,n,r} | line:true` (neither for other curves); `mid` = curve
  at mid-parameter.
- sweep creation order (per region: faces C0, C1, sides of outer loop then
  each hole; edges per loop: rails at loop vertices (vertex i = travel start
  of segment i), then per segment its C0 edge and C1 edge). Rails run
  C0 → C1; cap edges keep the sketch ENTITY direction; circles start at
  `c + r·x̂` with `n` = sketch normal (extrude) / axis (revolve rails).
- drafted extrudes: plane/plane edge directions follow intersection
  cross products (`orient_draft_edges`).

## massProps (pinned)

Request `{type:"massProps", tag, bodies:true, densities:[per lump]}`.
Reply `{volume, area, com, principal(asc), axes(rows)[, bodies[{volume,
area,com}]][, byMass{mass, com, principal, axes}]}`. Exact integration
matches the oracle to 1e-12..1e-13 relative on box / cylinder / holed
block / draft / revolves / sphere / torus / cone / two-lump bodies.

## Tests

`cargo test -p ck-topo`: unit tests per module + `tests/oracle.rs`, which
diffs faces/edges (endpoints of `pts`) of 15 bodies against original-kernel
replies stored in `tests/oracle_bodies.json`.

## Open / GUESS

- Edge `pts` discretisation belongs to the mesher: counts fit chord-height
  ≈ 3.74e-4 × bbox diagonal ±1 segment, with extra clamps for small arcs
  (min ~22 segments per full circle in the flange). `Body::sample_edge` is
  only a stand-in; ck-tess should pass its own `pts` to `edges_json`.
- open-edge key format; error
  strings other than `draftUnsupported` / `draftTooLarge` / `draftFailed`.
- Wire `kind` of extrusion/revolution/offset surfaces reported as `spline`.
- Draft: only one-sided (blind/flipped), line/arc profiles; non-tangent
  line/arc corners use a straight rail (approximation of the true conic).
- Eigenvector sign convention (largest component positive) for `axes`.
- Profile loop ordering/rotation is decided by ck-model's profile builder
  (the oracle sketch reply `profiles.regions[].outer/holes` gives the
  order); `ProfileLoop::reversed` is a guess.
