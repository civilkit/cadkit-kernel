# Tessellation checks

The `TESSELLATION_OPEN` check compares the display mesh of a body with the B-rep it came from. This chapter explains what it measures, the exact thresholds, and why it can report a hole on a body whose B-rep checks pass (and, less often, stay silent on a mesh with a hole).

## Why there is a mesh check

The B-rep checks of [topology and geometry checks](checks.md) look at the data structure. A boolean can still assemble a body that passes them while one face lost its trim loops and has no triangles; on screen the face is a see-through patch. The mesh is the cheapest independent witness of the claim "this is a closed shell". The check lives in `ck_check::tessellation` and works on plain arrays (the mesh positions and the face index of every triangle), so `ck-check` does not depend on the tessellator. The `validate` request tessellates the body and passes the arrays in; `mesh: false` skips it. `regen` does not report it (the only other caller of the mesh test is a regression test in `ck-ops`).

## What an open mesh edge is

The mesh is unwelded: three vertices per triangle. The check welds it first, then looks at the edges of the triangles.

1. **Weld.** Positions are put on a grid of cell size `q` and two vertices are the same if their distance is at most `q`, where

   ```text
   q = max(1e-5 * hi, 1e-4)        hi = the largest absolute coordinate in the mesh
   ```

2. **Count uses.** For each pair of welded vertices (a mesh edge), count the triangles that use it. Edges whose two ends welded into one vertex are ignored. An edge used by exactly one triangle is *open*. In a watertight closed mesh every edge is used by two triangles. An open edge belongs to the face of its triangle.

Counted this way, a block with one face removed has 4 open edges, and a block has none.

## What the check forgives

Two faces that share an edge both use the edge's samples, so ordinarily no seam appears. A crack still can come from two sources: a trim curve that ends at a cusp, and an edge that two faces sampled differently. A tolerance `crack` separates that hairline from a hole:

```text
crack = max(2e-3 * hi, 0.01)
```

An open edge is dropped from the count when either of these holds:

- **Both ends near another face's open edge.** Both end points lie within `crack` of an open edge that belongs to a different face. Two faces that sample their shared edge with different points (a T-junction) leave pairs of such edges.
- **Covered.** Nine samples along the edge all lie within `crack` of a triangle of another face. The neighbour covers the edge to within the chord deviation.

The remaining open edges are reported as one issue:

```json
{ "code": "TESSELLATION_OPEN",
  "entity": "F2.C1|F2.S.e1",
  "detail": "12 mesh edges are used by a single triangle (the display mesh has holes)" }
```

The entity lists up to six faces with open edges, in face order, joined by `|`.

## The other two reports

The same code is used for two more findings, reported per face.

- **No triangles.** A face of the body that has no triangle in the mesh. The detail says whether the tessellator failed and why: `face has no mesh triangles (tessellation failed: face produced no triangles)`. The reasons are `loop winds a period`, `face has fewer than 3 boundary points` and `face produced no triangles`; if the face is simply absent from the mesh the text ends in `(no triangles)`.
- **Area mismatch.** For a plane, cylinder, cone, sphere or torus face, the mesh area is compared with the exact area (an exact integral). The face is reported when the mesh area is below 90 percent or above 110 percent of the exact area and the difference also exceeds `5e-4 * hi * hi`. The second condition keeps slivers and specks, whose inscribed polygon cannot be seen to differ, out of the report. The detail reads `the mesh covers 62% of the face area (a of b)`. This catches a face meshed only partly, for example a cylinder wall triangulated around its cut-outs but not along its length, which shows no open edge of its own. Free-form faces are not tested, because their exact area costs more than meshing the whole body.

## Numbers for a small part

For a 40 x 20 x 10 mm block, `hi` is 40, so the weld distance is `q = 4e-4`, the crack tolerance is `0.08`, and the area floor is `5e-4 * 40 * 40 = 0.8` mm squared. A part that sits far from the origin has a large `hi`, so all three thresholds grow with its distance from the origin: a 10 mm part at x = 1000 has `q = 0.01` and `crack = 2`.

## Why it can fire on a valid B-rep

The check fires on the mesh, so any cause that damages the mesh and not the B-rep shows up. In order of how often they occur:

1. **The tessellator gave up on a face.** Trim loops that touch or cross by a few `1e-6`, a loop that winds around a periodic direction, a face whose boundary graph has an odd-degree vertex, trim segments the triangulation could not restore. `validate` returns the tessellator's notes for faces that meshed but are suspect, as `meshNotes`.
2. **A face meshed only partly.** The area check.
3. **Edges sampled differently by their two faces.** A hairline wider than `crack`. Edges are sampled once and shared by their faces, so this is rare.
4. **A thin or tiny feature.** Features smaller than the weld distance `q` collapse; a near-degenerate body (a disc 5e-4 mm high) meshes to tens of thousands of slivers.

The crash-fuzz runs found `TESSELLATION_OPEN` together with `VOLUME_NONPOSITIVE` and `EDGE_OFF_SURFACE` on mirrored and revolved near-degenerate bodies.

To separate a mesh problem from a B-rep problem, run `validate` with `mesh: false`. If that is clean and the default is not, the B-rep is probably fine and the tessellator could not mesh a face. Then:

- read `meshNotes` and the detail of the issue to find the face;
- check the face with `describe` and the surrounding features for a trim that nearly coincides with another edge;
- change the feature that produced the face by a small amount (a tenth of a millimetre) and compare;
- if it is a real case, keep the document: tessellation failures of valid bodies are bugs and the document is the regression case.

## Limits of the check

- **A hole bounded by several faces can be forgiven.** The first forgiveness rule looks only at the end points of an open edge. When a face is missing from the mesh and the faces around it each contribute one open edge, every open edge ends at the corner where its neighbour's open edge ends, so all of them are dropped. A block exported without its top face has 4 open mesh edges by the strict count and the validator reports only `SHELL_OPEN` and `EULER`, no `TESSELLATION_OPEN`. A face with no triangles is still caught by the first report above, and a body with free edges is caught by the B-rep checks, so the rule only matters when both are clean.
- **Free-form faces are not area-checked.** A B-spline face with a part of its mesh missing is found only through its open edges.
- **Two overlapping lumps are not a mesh hole.** The check does not look at intersections.
- **The mesh is a display mesh.** Passing says the mesh has no visible holes, not that it is accurate; see [tessellation](../kernel/tessellation.md) for the chord tolerance and its volume error.

## See also

- [The validate request](validate.md)
- [Topology and geometry checks](checks.md)
- [Tessellation (kernel)](../kernel/tessellation.md)
- [Feature errors and diagnostics](diagnostics.md)
