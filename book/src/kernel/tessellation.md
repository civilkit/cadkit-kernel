# Tessellation

`ck-tess` turns the faces of a body into the triangle mesh that a viewer draws, and produces the edge polylines and silhouette data that go with it. This chapter describes how faces are meshed, what the mesh contains, what controls its density, and where it can fail. The checks that use the mesh as a witness are in [tessellation checks](../validation/tessellation.md).

## What the mesh is for

The mesh is a display mesh. It approximates the exact B-rep to a chord height; it is not used for volume, area or any measurement that has an exact answer (those come from `massProps` and the other [analysis requests](../validation/analysis.md)). For a sphere of radius 10 the mesh volume is 0.097 percent below the exact one, for a cylinder of radius 50 and height 100 it is 0.18 percent below, and for a torus with R = 10 and r = 3 it is 0.34 percent below. The sign depends on the surface: polygons inscribed in convex surfaces fall short and those in concave surfaces overshoot, so a body with both can go either way (a block with a radius 4 through hole meshes 0.04 percent above its exact volume when the block is cut with a circle extrude: 7500.31 against 7497.35).

A `regen` reply carries the mesh of the solid body and one mesh for each surface body. Tessellation also runs inside `validate` (to check that the mesh is watertight), `curvatureMap`, `comb`, `section`, `sections` and a few analysis queries.

## Tolerance

Two numbers control the density. Both are fixed by the body; no request option changes them.

| quantity | value |
|---|---|
| chord height | `max(1e-3, 4e-4 * diagonal)` mm, where `diagonal` is the length of the body's bounding-box diagonal; rounded down to four significant figures |
| angle | 0.3 rad between consecutive samples on a curve and between the normals at the two ends of a refined edge |

An empty bounding box, or a NaN diagonal, gives the minimum, `1e-3`. The rounding down makes the tolerance stay the same when an unrelated edit moves the box in the last digits, so unchanged faces keep their mesh bit for bit and the face cache can find them.

Because the chord height scales with the body, the number of segments on a circle depends on the circle's radius relative to the body, with the angle limit as a ceiling. A circle of radius `r` is cut into steps of `min(2 acos(1 - chord / r), 0.3)` radians, which is at least 21 segments for a full circle. A cylinder of radius 50 and height 100 meshes to 61 segments around (the extra one comes from the periodic cut), and a radius 1, height 2 cylinder meshes to the same 61 because chord and radius scale together.

## The pipeline

1. **Edge sampling.** Every edge is sampled once, and its samples are used by both faces that share it. This is what makes the mesh watertight without any welding step: the boundary vertices of neighbouring faces are bit-identical. A line has its two ends. A circle uses the step above. An ellipse uses a step derived from its two semi-axes. A spline gets each knot span cut into `max(2, degree + 1)` pieces, refined adaptively, and then thinned to what the tolerance needs.
2. **Refinement along edges.** For each coedge the parameter list is refined in the face's uv space: bisect while the pcurve midpoint is farther than a scaled tolerance from the uv chord (at most 12 levels). Then, for each coedge, bisect while the angle between the face normals at the two ends exceeds 0.3 rad (at most 8 levels).
3. **Periodic cut.** A periodic face (a cylinder wall, a sphere, a torus) is cut open along a line in uv at the middle of the largest gap between the boundary samples. The cut adds one sample to every edge that crosses it. The face is meshed in the strip between the cut and one period later. Faces with no trimming loops (a full sphere or torus) get the whole domain as a rectangle with pole lines.
4. **Triangulation.** The boundary samples, mapped through the pcurves into a metric-scaled uv frame, are triangulated by a constrained Delaunay triangulation (CDT). The triangles that lie inside the boundary, found by crossing parity, are kept. A planar cap with N boundary points becomes N - 2 triangles, and a cylinder wall with N segments becomes 2N triangles, because no interior points are needed.
5. **Interior refinement.** For curved faces, triangles are refined by inserting the uv centroid of every triangle whose surface point at the centroid, or at the middle of one of its edges, is farther than the chord height from the flat triangle. Passes repeat until nothing is inserted, at most 64 passes and a budget of points per face. Large B-spline faces are first seeded with a uv lattice sized to the number of knot spans so that long thin faces converge quickly, and a surface-aware choice of diagonals avoids fans on long strips.
6. **Output.** Triangles are written counter-clockwise about the face's outward normal (flipped for `sense = false`). Points within `1e-4` of an already emitted point, and within `1e-3` in uv, share a vertex, so that two points an `f32` cannot tell apart do not produce a zero-area triangle.

### The constrained Delaunay triangulation

`cdt.rs` is a standard implementation: a super-triangle encloses the points, points are inserted by splitting a triangle or an edge and restoring the Delaunay property with Lawson flips, and constrained segments are inserted by flipping the edges they cross (Sloan's method) and are never flipped afterwards. All orientation and in-circle decisions use adaptive exact predicates, so the result is a consistent triangulation even for nearly degenerate input. A segment that cannot be inserted is counted and reported as a note on the face.

## What the mesh contains

The reply buffers for a body (see [replies and buffers](../protocol/reply.md)) are:

| buffer | type | size | content |
|---|---|---|---|
| `positions` | f32 | 9 per triangle | Three vertices of each triangle. The mesh is *unwelded*: vertices are not shared between triangles. |
| `normals` | f32 | 9 per triangle | The exact outward surface normal at each vertex (the face normal for planes). |
| `mids` | f32 | 9 per triangle | For each triangle edge (v0v1, v1v2, v2v0), the correction `n(mid) - (n_a + n_b) / 2`, where `n(mid)` is the unit surface normal at the uv midpoint of the edge. A shader adds it to the interpolated normal to shade a curved surface smoothly with coarse triangles. Zero on planes. |
| `triFace` | i32 | 1 per triangle | Index of the face in `body.faces`. |
| `silhouette` | f32 | 12 per candidate edge | `[a, b, nA, nB]` for each mesh edge shared by two triangles of the same face whose dihedral angle is strictly convex. With the two facet normals a viewer can decide, for its current view direction, whether the edge is a silhouette. Concave and flat edges are excluded, so a cylindrical hole has none. |

The body's `box` is the bounds of the `f32` positions. Edge `pts` in the `edges` list are the same samples the faces used.

A box gives 12 triangles, a 40 x 20 x 10 plate with a radius 4 hole gives 152 (the two caps 38 each, the hole wall 68), a block with one radius 2 fillet gives 36, and a sphere of radius 10 gives 17776.

## Welding

The reply mesh has no shared vertices, so a consumer that needs connectivity welds it. Because edge samples are shared, welding by quantised position works: the weld in the validator quantises positions to `max(1e-5 * largest absolute coordinate, 1e-4)` and finds no open edge on a closed body. Welding with a coarser grid than the chord height can merge vertices that belong to different parts of a thin feature, so keep the quantum well below the smallest feature. See [tessellation checks](../validation/tessellation.md) for the exact rule.

## Cache

`tessellate_memo` caches the mesh of each face under a key made from the exact bits of everything that influences it: the surface and sense, every live coedge's edge curve, pcurve, parameter range and final edge samples, the seam flag, the end vertices, the periodic cut and the tolerance. A hit is bit-identical to meshing again. Adding a hole to a plate re-meshes only the faces whose geometry or samples changed. The cache holds up to 128 MB by default (`opts.cacheCap` scales it); see [regeneration and caching](regeneration.md).

## Related meshes

- `curvatureMap` returns a triangle soup with per-vertex principal curvatures `k1` and `k2` for a false-colour overlay. It reuses the display mesh and subdivides curved faces 4 x 4 in uv, with exact curvature at every vertex.
- `comb` returns sample points and curvature vectors along edges, and along iso-parameter lines of faces.

## Known failure modes

A face that cannot be triangulated is left out of the mesh and listed in the mesh's `failed` indices, with a reason. The reasons are `loop winds a period` (a boundary loop goes around a periodic direction without closing), `face has fewer than 3 boundary points`, and `face produced no triangles`. `validate` reports such a face as `TESSELLATION_OPEN` and adds the reason.

A face can also mesh but be suspect. The mesh then carries a note for the face: the boundary graph has an unclosed loop (a vertex of odd degree), or some trim segments could not be restored by the triangulation. `validate` returns these notes as `meshNotes`.

Behaviours to expect:

- **Near-degenerate bodies are expensive.** A disc of height 5e-4 and radius 3 meshes to tens of thousands of triangles, and revolved and mirrored near-degenerate bodies are the main source of validator findings in the crash-fuzz runs.
- **The mesh is coarse for curved faces at the default tolerance.** In the repository's tessellation sweep, volume from the mesh was more than 0.1 percent away from the exact volume for every cylinder and torus and about half of the cones (up to 0.8 percent for a torus), while boxes, spheres and two-station lofts stayed within it. Treat the mesh as a picture.
- **Trim curves that touch.** A crescent-shaped cap, or a face whose trim curves cross by a few `1e-6`, is the typical case where the triangulation has trouble. Several such cases (a bore wall cut by a branch hole, a torus band spanning a full period, a sphere zone whose circle junction lies on the cut line) are fixed and kept as regression tests; the rules above handle them, but new combinations can still fail.
- **Tolerance is not user-controlled.** There is no per-request chord height; a client that needs exact numbers should use the exact queries, not the mesh.
- **The display mesh is not the measurement.** Section areas, mass, distances and clearances are computed on the exact surfaces; the `section` request cuts the mesh and then snaps the cut points onto the exact surfaces, and `clearance` seeds from the mesh and refines on the surfaces.

## See also

- [Tessellation checks](../validation/tessellation.md)
- [The B-rep model](brep.md)
- [Replies and buffers](../protocol/reply.md)
- [Regeneration and caching](regeneration.md)
