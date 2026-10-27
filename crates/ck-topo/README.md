# ck-topo

B-rep topology: arena-based solid model. Depends on ck-core, ck-geom, ck-json.

## Modules

| module | contents |
|---|---|
| `body.rs` | `Body { vertices, edges, coedges, loops, faces, shells, lumps }` (Vec arenas + typed ids, `alive` flags), `FaceName`/`Role` id minting + parsing, `edge_key`, wire export `faces_json` / `edges_json` (one entry per `wire_chains` chain, split rails hidden), `seamless_index`, `number_split_faces` / `face_ctrl_box` / `is_split_rail`, `append`, `push_instance`, rigid `transform` |
| `trig.rs` | `sin` / `cos` bit-identical to the kernel's (msun port, = what the wasm build uses; native libm differs by an ulp) |
| `make.rs` | `Profile` (frame + regions of oriented `ProfileSeg` loops), generic sweep builder, `extrude(feature, &Profile, ExtrudeOpts)` (blind / flipped / mid / custom dir / draft) |
| `make/revolve.rs` | `revolve(feature, &Profile, axis_o, axis, a0, a1)` (partial or 360°, on-axis vertices/segments) |
| `orient.rs` | `flip_face`, `reverse_loop`, `loop_uv_area`, `loops_consistent`, `fix_planar_loops`, `orient_shell` / `orient_body` |
| `check.rs` | `check(&Body, tol) -> Vec<Issue>` (open / non-manifold / unused edges, bad pairing, broken loops, vertex-off-curve, pcurve-off, degenerate, bad range), `euler`, `is_closed_manifold` |
| `mass.rs` | `face_integral` (exact Green's-theorem face integrator), `Moments`, `mass_props`, `mass_props_json` (the `massProps` reply), `signed_volume`, `eigen_sym` |
| `region.rs` | `winding_number`, `classify -> Inside/Outside/On` (nearest-feature pseudo-normal test within 1% of the bbox diagonal of the boundary, winding number farther out), `components`, `rebuild_lumps` (void shells attach to their host lump) |
| `edit.rs` | `split_edge`, `split_face` (new edge between two loop vertices), `delete_face`, `merge_faces` / `merge_coplanar`, `reverse_line_edge`, `gc` |

## Conventions

- Face: `Surface` + `sense` (outward = natural normal Su×Sv when true).
- Every loop is CCW about the OUTWARD normal; first loop = outer.
- Coedge: `rev` = traverses its edge backwards; `pcurve` is parameterised
  by the EDGE parameter. All built faces carry exact pcurves; mass and
  orientation fall back to projection when a pcurve is missing.
- Seam edge = both coedges on one face (periodic faces). Pole vertices
  (revolve on axis) have no rail edge.
- **The oracle has no seam edges at all** (STEP: cylinders bounded by bare
  circles, full spheres / tori by nothing; a fillet on a cylinder's C1
  circle numbers survivors `#0, #2, #3` — dead C1 = #1, no seam slot;
  ck-ops probe `boss`: one arc through our seam point). ck-topo keeps
  seams INTERNAL (uv-closed loops for mass / mesher / booleans) and makes
  every exporter seam-agnostic: seams take no wire number, seam-split arcs
  are merged (`wire_chains`; `edges_json` and the STEP writer emit one
  edge per chain), `seamless_index` gives oracle arena numbers for
  in-place ops. Dropping seams from the topology would need a seamless
  periodic-face story in mass.rs, ck-tess and the booleans.
- Arenas never shrink; edits flag `alive=false`. Export order = creation
  order, which is what pins the wire `#N`.

## Frames and numerics (pinned against oracle STEP bits, see ck-io)

- The oracle stores PLANE and CIRCLE frames as (o, x, y); their axis is
  x × y computed at write time (f417/f418), the wire plane normal is the
  frame's own z. Side planes of extrudes: x = entity end − start in 3D.
- Revolve: the C1 cap, top curves and vertices are the sketch geometry
  rotated by the Rodrigues matrix (ck-core `Mat3::rotation` formula)
  built with `trig::{sin,cos}`, applied column-wise; rotated frames keep
  x, y re-normalised and z = R·z. An unrotated start keeps the sketch
  geometry bit-for-bit.
- Rail circles: x = radial, y = axis × x (STEP axis = x × y); cylinders /
  cones / spheres / tori keep z = axis.
- `trig.rs` is required for native builds to reproduce wasm bits
  (macOS libm cos(120°) = −0.49999999999999983, kernel −0.4999999999999998);
  checked against the kernel's own f4182/f4184 on 4032 arguments
  (the recorded reference data (not published)).

## Split faces

One entity split into several side faces (ck-model cuts closed splines
at half length) gets `S.<tag>~k`: k = rank of the face's boundary
control-net box centre, (x, y, z) world-lexicographic with a 1e-9 tie
tolerance (`number_split_faces`; 240 random closed spline / cv-spline
extrudes on front/top/right all equal to the oracle; the curve's own box
or mid point get up to 9/60 wrong). Control nets are clamped at both
ends. The edges between the pieces (`is_split_rail`) keep their wire
number but are not listed in `body.edges` (oracle closed-spline extrude:
keys start at `#2`), and are still written to STEP. Wire `kind`
= `Surface::kind()` (kernel.strings kind table: plane cylinder cone
sphere torus spline offset extrusion revolution; probes: extrusion,
revolution).

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
With `densities` every `bodies[]` entry also carries `mass` (= volume ×
density); without, the reply is the oracle-pinned shape (no `mass`). At the
request level (`ck-wasm` `query_hook`): `densities` alone implies
`bodies:true`, and a length that is not one entry per lump is an error
naming the expected count.
Reply `{volume, area, com, principal(asc), axes(rows)[, bodies[{volume,
area,com[, mass]}]][, byMass{mass, com, principal, axes}]}`. Exact integration
matches the oracle to 1e-12..1e-13 relative on box / cylinder / holed
block / draft / revolves / sphere / torus / cone / two-lump bodies.

Eigen solver = the oracle's f3939 (cyclic Jacobi, pairs (0,1),(0,2),(1,2),
up to 64 sweeps until off-diagonal sum < 1e-300, total-order sort, columns
scaled by 1/|v|, NO sign convention — rev90 / rev90flip / cone axes come out
of the rotations).

## Tests

`cargo test -p ck-topo`: unit tests per module + `tests/oracle.rs`, which
diffs faces/edges (endpoints of `pts`) of 15 bodies against original-kernel
replies stored in the recorded reference data (not published).
the recorded reference data (not published): trig vs the kernel's own sin/cos.

## Open / GUESS

- Edge `pts` discretisation belongs to the mesher: counts fit chord-height
  ≈ 3.74e-4 × bbox diagonal ±1 segment, with extra clamps for small arcs
  (min ~22 segments per full circle in the flange). `Body::sample_edge` is
  only a stand-in; ck-tess should pass its own `pts` to `edges_json`.
- open-edge key format; error
  strings other than `draftUnsupported` / `draftTooLarge` / `draftFailed`.
- `classify`: nearest-feature side test assumes `closest_point` finds the global surface minimum (spline / offset faces may need seeding).
- Draft vs oracle STEP: the oracle re-derives drafted side faces with a draft op — vertices ~2e-10 off the exact offset corners, side-plane frames x = −(edge direction), y pointing from the top edge down (logged via f418), C0 edge directions from plane cross products. Not reproduced (draft.step stays a known diff).
- Draft: only one-sided (blind/flipped), line/arc profiles; non-tangent
  line/arc corners use a straight rail (approximation of the true conic).
- massProps repeated moments (cylinder, sphere, torus) and exactly symmetric bodies (rev90flip): the decoded Jacobi turns tensor rounding noise into the axes (oracle cyl: a 45° pair because its I_xx, I_zz are bit-equal), so they match only once the integrator is bit-exact. Decoded but unused: f3561 builds the tensor as e.g. a00 = (Syy − cy(V·cy)) + (Szz − cz(V·cz)), com = first·(1/V); its per-face moments (f3366) are not decoded.
- Profile loop ordering/rotation is decided by ck-model's profile builder
  (the oracle sketch reply `profiles.regions[].outer/holes` gives the
  order); `ProfileLoop::reversed` is a guess.
