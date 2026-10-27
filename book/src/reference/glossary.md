# Glossary

Terms used across this book, in alphabetical order. Where a term has a field or code in the kernel, it is given in code font.

**ABI** The binary interface of the WebAssembly module: five exports (`ck_alloc`, `ck_request`, `ck_reply`, `ck_commit`, `ck_commit_len`) and three imports (`ck_now`, `ck_unix_ms`, `ck_panic`). One request in, one reply out.

**Arena** How `ck-topo` stores a body: one vector per entity kind, with typed ids and an `alive` flag. Entities are never removed from the vector; an edit marks them dead. Export order is creation order.

**Assembly** A part that places other parts (`insertPart` instances) and relates them with mates. See [parts and assemblies](../format/features-assemblies.md).

**B-spline** A piecewise polynomial curve or surface defined by control points, a degree and a knot vector. With weights it is rational (NURBS). The kernel uses them for free-form geometry and as the exact form of text outlines.

**Blend** A fillet or chamfer: a surface that replaces a sharp edge. Blend faces are named `B<k>` (and `V<k>` for a corner patch).

**Body** The B-rep result of regeneration: the topology (lumps, shells, faces, loops, coedges, edges, vertices) and the geometry attached to it. A part builds one body, which may hold several lumps.

**Boolean** Union (`add`), cut or intersect of two bodies. The implementation imprints the operands onto each other, classifies the pieces as inside, outside or on the other body, and reassembles the kept pieces.

**Budget** A wall-clock limit on work (`featureTimeMs`, `regenTimeMs`). Active only where the host provides a clock. See [robustness](../quality/robustness.md).

**Cache (regen cache)** Stored intermediate states of a regeneration, keyed by a hash chain over the document prefix, so that an edit near the end of the feature list re-runs only the features after it. See [regeneration and caching](../kernel/regeneration.md).

**CDT** Constrained Delaunay triangulation. The tessellator triangulates each face in its parameter space with the trimming loops as constraints, using exact predicates.

**Chamfer** A blend that replaces an edge by a flat or conical bevel.

**Classification** Deciding whether a point or face piece is inside, outside or on a solid. Uses a nearest-feature pseudo-normal test close to the boundary and a winding number farther out.

**Coedge** One use of an edge by a loop. An edge shared by two faces has two coedges. `rev` says whether the coedge runs against the edge's direction. A seam edge has both coedges on the same face.

**Constraint** A relation in a sketch (coincident, horizontal, distance, radius, tangent, ...) that the sketch solver satisfies by moving points. See [the sketch solver](../format/sketch-solver.md).

**Degrees of freedom (DOF)** The number of independent motions left in a sketch or an assembly after constraints and mates.

**Edge** A bounded piece of a curve, between two vertices, with a parameter range `[t0, t1]` on its curve and an optional tolerance. It is used by the coedges of two faces in a closed manifold body.

**Expression** A string such as `"d/2"` or `"=2*w"` in a field that takes a number. Evaluated against the part's variables. See [variables and expressions](../format/expressions.md).

**Face** A bounded region of a surface. It has a surface, a `sense` (whether its outward normal agrees with the surface's natural normal) and one or more loops. The first loop is the outer boundary. Faces have persistent names such as `F2.S.e1`.

**Feature** One entry of a part's `features` list: a sketch, an extrude, a fillet, a hole, a pattern, a plane, and so on. Features run in order; each has an `id` such as `F4`.

**Fuzzy tolerance** The `tolerance` and `autoFuzzy` options of a boolean-producing feature, which snap nearly coincident faces onto each other before intersecting.

**Healing** Repairing a body so it validates: merging near vertices, sewing gaps, removing degenerate edges and sliver faces, fixing orientation, and giving small deviations an edge tolerance. See [healing](../validation/healing.md).

**Imprint** The step of a boolean that cuts each operand's faces along its intersection curves with the other.

**Knot vector** The non-decreasing parameter values that define a B-spline's basis. Multiplicities control continuity.

**Loop** A closed chain of coedges bounding part of a face. On a face the outer loop runs counter-clockwise about the outward normal and holes run clockwise.

**Lump** A connected solid piece of a body, with one outer shell and zero or more void shells. A body with two separate blocks has two lumps.

**Mass properties** Volume, area, centre of mass and the inertia tensor (reported as principal moments and axes), computed by exact integration over faces (`massProps`).

**Mate** A relation between two parts of an assembly (coincident, angle, hinge, gear, ...) that fixes some of their relative degrees of freedom.

**Mesh** The display triangulation of a body: positions, normals and a triangle-to-face index, sent as binary buffers in the reply. Distinct from the exact B-rep.

**NURBS** Non-uniform rational B-spline: a B-spline with weights, able to represent conics exactly.

**Part** A document (`.ckpart`) with a format, a version and a list of features. Regenerating it yields a body.

**Pcurve** A curve in the (u, v) parameter space of a surface, giving an edge's path on a face. Faces on periodic surfaces rely on pcurves for mass properties and meshing.

**Persistent name** A face or edge identifier that survives edits because it is derived from the feature that made it, for example `F2.C0` (cap), `F2.S.e1` (side of sketch entity `e1`), `F3.B0` (blend). Later features refer to geometry through these names. See [persistent names](../kernel/naming.md).

**Pole** The point where a sphere, cone or revolve closes on its axis. A face at a pole has no rail edge there.

**Profile** The closed loops of a sketch used by a feature, grouped into regions (an outer loop with its holes).

**Regen** Regeneration: running a part's features in order to build the body. The `regen` request. Also the name of the stage in a reply.

**Region** A face of a sketch's planar arrangement: an outer boundary with its holes. Extrudes act on regions.

**Rollback** `part.rollback = n` regenerates only the first `n` features, as the rollback bar of a feature tree does. `opts.upTo` does the same for one request; the smaller wins.

**Seam** The edge where a periodic surface (cylinder, cone, sphere, torus) joins itself. The kernel keeps seams internally and omits them from exported edge lists and STEP files.

**Sense** A boolean orientation flag: of a face against its surface, of a coedge against its edge, of a loop about the outward normal.

**Sheet (surface body)** A body of faces that is not a closed solid, produced by surface features or by importing a surface model.

**Shell** A connected set of faces. A closed shell bounds a volume. A lump has an outer shell and void shells for cavities.

**Sketch** A set of points, entities (lines, arcs, circles, splines, text) and constraints on a plane. The source of profiles.

**SSI** Surface-surface intersection: computing the curves where two surfaces meet, by marching, with analytic cases handled exactly. The weak spot is general NURBS.

**Surface** The geometry of a face. Ten kinds: plane, cylinder, cone, sphere, torus, extrusion, revolution, offset, B-spline and screw (the screw surface of twisted extrudes and helical sweeps reports as `spline` on the wire). See [geometry](../kernel/geometry.md).

**Tag** The name of a state held by the kernel instance. `regen`, imports and queries take a `tag` (default `"main"`); a query on a tag that has no state answers `noState`.

**Tessellation** Turning exact faces into triangles, to a chord-height and angle tolerance. The default chord tolerance is `max(1e-3, 4e-4 x bounding-box diagonal)` mm.

**Tolerance** A distance below which two things are treated as equal. The model tolerance is used by validation and healing; an edge or vertex can carry its own tolerance (set by healing) for geometry that is only approximately consistent, such as imported files. A tolerance above the cap (0.05 mm by default) is an error.

**Transmit** The Parasolid data stream stored inside a SolidWorks file.

**Valid (validation)** A body is valid when `validate` reports no issue: closed manifold shells, consistent orientation, edges on their faces, loops closed, positive volume. See [validation](../validation/overview.md).

**Variable** A named value in a part that expressions can use.

**Vertex** A point where edges meet, with an optional tolerance.

**Void** A cavity inside a lump: an inner shell whose faces point into the cavity. Written as `BREP_WITH_VOIDS` in STEP.

**Welding** Merging points that are closer than a tolerance into one vertex, so that neighbouring faces share edges. Used when importing meshes and surface models (1e-4 mm for SketchUp) and by healing.

**Wire (protocol)** The JSON request and reply format and its binary buffers, as opposed to the B-rep. "Wire" ids are the face ids and edge keys that appear in replies.

## See also

- [The B-rep model](../kernel/brep.md)
- [Persistent names](../kernel/naming.md)
- [Crate map](crates.md)
- [Overview of the part format](../format/overview.md)
