# The B-rep model

This chapter describes the boundary-representation data model in `ck-topo`: the entities, their conventions, how bodies are built and edited, and how volume and mass properties are computed from them.

## Entities

A `Body` is a set of arenas, one `Vec` per entity kind. Entities refer to each other by typed 32-bit indices (`VertexId`, `EdgeId`, `CoedgeId`, `LoopId`, `FaceId`, `ShellId`, `LumpId`).

```text
Body
 '- Lump          one solid (or one connected sheet group); has an optional body name
     '- Shell     a closed boundary: the outer skin, or an internal void
         '- Face  a trimmed piece of a surface, with a persistent name
             '- Loop     a closed boundary of the face; the first one is the outer loop
                 '- Coedge   one use of an edge by a face, with a direction and a pcurve
                     '- Edge     a curve segment shared by (normally) two coedges
                         '- Vertex   a point
```

| entity | fields that matter |
|---|---|
| `Vertex` | `p` (the position) and `tol`. |
| `Edge` | `curve` (a 3D curve of `ck-geom`), the trim range `t0 < t1` on that curve, `v0` and `v1` (the vertices at `t0` and `t1`), the list of coedges, and `tol`. |
| `Coedge` | The `edge`, `rev` (it traverses the edge backwards), the loop it belongs to and an optional `pcurve`. |
| `Loop` | The face and the ordered coedges. |
| `Face` | `surface`, `sense`, the loops, the shell, the persistent `name`, and `absorbed` (see below). |
| `Shell` | The faces and the lump. |
| `Lump` | The shells and an optional body `name`. |

`tol` on vertices and edges is the tolerance in millimetres (0 means exact). Modelling operations create only exact entities; the repair pass sets tolerances on imported geometry. See [tolerances](../validation/tolerances.md).

### Conventions

These hold for every body the kernel builds, and the validator and the repair pass rely on them.

- **Face sense.** A face carries a surface and a boolean `sense`. The outward normal is the natural surface normal `Su x Sv` when `sense` is true and its negation otherwise.
- **Loop orientation.** Every loop runs counter-clockwise about the face's outward normal, so the material is on the left when walking along a coedge. The first loop of a face is the outer loop; the others are holes.
- **Coedge direction.** A manifold edge has two coedges, and they run in opposite directions: one has `rev = true`, the other `false`. Two coedges of one edge with the same `rev` are the defect `SHELL_ORIENTATION`.
- **Pcurves.** A coedge may carry a 2D curve in the surface's `(u, v)` space. It is parameterised by the *edge* parameter, so `pcurve.eval(t)` is the `(u, v)` of `edge.curve.eval(t)` on this face. Every face the kernel builds carries exact pcurves. Where a pcurve is missing, mass properties and the validator project the 3D curve onto the surface instead.
- **Seams and poles.** A periodic face (a full cylinder, cone, sphere or torus) is closed in `u` or `v` by a *seam edge*: an edge whose two coedges both belong to the same face. Where a revolve passes through its axis, the vertex is a *pole* with no rail edge. Seams stay inside the model because mass integration, meshing and booleans need uv-closed loops, but they are not exported: the `edges` list of a reply never contains a seam, and an arc that a seam splits is reported as one edge.
- **Arenas never shrink.** Deleting an entity flags it `alive = false`. Indices stay valid, and the order of the live entities is the order they were created. This is what fixes the numbering of edges in the reply (`#N` in an edge key).

### Shells, lumps and voids

A lump is one solid. A lump has one outer shell and any number of void shells (the cavity of a hollow part). A boolean that splits a part in two gives two lumps. Lumps of one body can overlap (see `newBody` in the [features chapter](../format/features.md)); such lumps are legal, and `validate` only reports the overlap as information.

`rebuild_lumps` groups faces into edge-connected components and assigns them to lumps. A component whose signed volume is positive opens a lump; a negative component (inside-out, hence a void) attaches to the lump that contains it. A shell whose faces are not edge-connected is split into one shell per component. A sheet (a surface body made by `surfaceExtrude`, `fill` and similar features) is a body of open shells. The regeneration program keeps sheets apart from the solid body, and the reply lists them under `surfaces`.

### Face names

Every face has a `FaceName`. The name is what documents refer to, and it is built from how the face was made, not from its position in a list:

```text
<feature>.<role>[~<region>][@<instance>...]
    role     S.<sketch entity>    side face swept from a sketch entity
             C<k>                 end cap (C0 = start, C1 = end)
             B<k>                 blend face (fillet, chamfer)
             anything else        written as is (V0 vertex blend, W0 thicken wall, F patch ...)
    ~k       k-th piece when a face was split in several (1-based)
    @F5.2    copy 2 made by pattern F5; chained for nested instances
```

`FaceName::parse` is the inverse of `FaceName::id`. A face that merges into another during a union keeps its name in `Face::absorbed` of the surviving face, and `Body::find_face` looks there when an exact match fails. [Persistent names](naming.md) explains how this keeps later features working.

An edge in a reply is named by the two faces it separates, sorted bytewise, plus its index in the exported edge list: `F2.C1|F2.S.e1#2`. The index changes when the topology changes, so documents refer to an edge by `{faces, at}` (two face names and a point) instead.

## Building bodies

All construction ends in a `Body` with consistent loops and pcurves.

- **Extrude and revolve** (`ck_topo::make`). A `Profile` is a frame plus regions; a region is an outer loop and hole loops of `ProfileSeg` (line, arc or circle pieces with their sketch entity tag). `extrude` takes `ExtrudeOpts` (the start and end offsets along the sketch normal, a direction override, and a draft angle) and `revolve` takes an axis and two angles. Each region becomes its own lump. The creation order is fixed: per region the caps `C0`, `C1`, then the side faces in loop order; per loop first the rails, then for each segment its `C0` edge and its `C1` edge.
- **Operations in `ck-ops`** produce a new body from existing ones. Booleans assemble their result from fragments of the operands' faces; blends and offsets rebuild a body from changed surfaces. See [operations](operations.md).
- **Appending.** `Body::append` copies another body's arenas into this one and returns the new lumps. Documents use it for `newBody` and for pattern instances, which are then renamed with `push_instance`.
- **Transforms.** `Body::transform` applies a rigid transform to every surface, curve and vertex.

## Editing a body

`ck_topo::edit` offers local edits that keep the data structure consistent:

| function | effect |
|---|---|
| `split_edge(e, t)` | Splits an edge at a parameter. Every coedge of the edge gets a partner in its loop. Returns the new vertex and edge. |
| `split_face(f, va, vb, ...)` | Splits a face with a new edge between two vertices of one of its loops. The other part becomes a new face. |
| `delete_face(f)` | Removes a face and leaves its edges open (free edges). |
| `merge_faces`, `merge_coplanar` | Merge adjacent planar faces with the same plane and normal into the face with the lower id. |
| `gc` | Flags edges without live coedges, and vertices without live edges, as dead. |

`ck_topo::orient` fixes orientation: `flip_face`, `reverse_loop`, `fix_planar_loops`, `orient_shell` and `orient_body` (make faces agree with their neighbours and the solid outward), and `loop_uv_area`, which gives the signed area of a loop in uv space.

## Queries on a body

- **Classification.** `region::classify(body, point, tol)` returns `Inside`, `Outside` or `On`. Near the boundary (closer than 1 percent of the box diagonal) the side is read from the nearest feature: a face interior by its outward normal, an edge by the sum of its faces' normals, a vertex by the angle-weighted sum. Farther away the generalised winding number, evaluated with the exact face integrator, decides. Neither method needs a ray-surface intersection, and both are insensitive to seams and poles.
- **Counts.** `check::euler` gives `V - E + F - (L - F)`. For a closed orientable shell this is `2(1 - genus)`: 2 for a block, 0 for a block with a through hole.
- **Boxes.** `Body::bbox` samples every edge (65 points) and is memoised by content.

The validator is a separate crate; see [topology and geometry checks](../validation/checks.md). `ck_topo::check` is the smaller, internal predecessor used inside operations (open and non-manifold edges, broken loops, pcurve deviation).

## Mass properties

`ck_topo::mass` computes volume, area, centre of mass and inertia from the faces, with no mesh involved. The divergence theorem turns the volume into a surface integral of `p . n / 3` over the faces. Each face integral over the trimmed parameter domain is turned into a boundary integral by Green's theorem and evaluated along the coedge pcurves. Both integrals use composite 8-point Gauss-Legendre quadrature, with adaptive panels where the integrand is rough. For analytic faces with exact pcurves this reaches machine precision.

| solid | volume relative error | area relative error |
|---|---|---|
| sphere, radius 10 | 4e-16 | 2e-16 |
| cylinder, radius 50, height 100 | 3e-16 | 3e-16 |
| torus, R = 10, r = 3 | 8e-16 | 0 |

(Measured through the `massProps` request on solids built with the kernel; the reference values are the closed forms.)

Moments are accumulated per lump about a reference point and then combined. The inertia tensor about the centroid goes through a cyclic Jacobi eigen-solver; the principal moments come out ascending and `axes[i]` is the unit eigenvector of `principal[i]`. The eigenvectors have no sign convention, and for repeated moments (a cylinder, a sphere) the axes are an arbitrary basis of the eigenspace. The `massProps` reply shape and its limits are described in [mass properties and interference](../validation/analysis.md).

## See also

- [Geometry](geometry.md)
- [Persistent names](naming.md)
- [Topology and geometry checks](../validation/checks.md)
- [Architecture](architecture.md)
