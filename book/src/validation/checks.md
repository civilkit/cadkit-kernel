# Topology and geometry checks

This chapter lists every issue code that the validator in `ck-check` can report: what it tests, the thresholds, what usually causes it and what to do. For how the checks are called and what the reply looks like, see [the validate request](validate.md).

## How an issue is reported

An issue has three fields.

| field | content |
|---|---|
| `code` | A stable machine identifier, upper case with underscores. |
| `entity` | What the issue is about. Faces use their persistent name (`F2.C1`). Edges are written `edge#N[faceA\|faceB]` with the names of the faces that use the edge. Shells, lumps and components are `shell#N`, `lump#N` and `component#N`. |
| `detail` | A sentence with the numbers. |

The `N` in `edge#N` is the position of the edge in the body's internal edge list. It need not equal the `#N` at the end of an edge key in a reply, because seam edges are internal and are not numbered there; use the face names to find the edge. Faces of imported bodies are named `I1.F1`, `I1.F2` and so on.

The validator runs four groups of checks in this order: shell and edge use, loops, edges and vertices, solids. A body is valid when no check reports anything. `tol` is the model tolerance of the request (default `1e-6` mm) and `cap` the largest edge or vertex tolerance accepted (default 0.05 mm).

## Shell and edge use

These tests establish that the faces form closed, consistently oriented skins.

| code | entity | meaning | typical cause | fix |
|---|---|---|---|---|
| `EDGE_UNUSED` | edge | The edge is used by no face. | A face was deleted and the edge was left behind by a repair or an import. | Healing flags edges without any face use as dead when it sews; otherwise remove the edge's source. |
| `SHELL_OPEN` | edge | The edge is used by only one face (a free or boundary edge). Detail: `free (boundary) edge: used by 1 face`. | A missing face, a gap between two faces, edges of two faces that nearly coincide but were never joined (an import), a failed boolean. The solid is not closed. | Heal with sewing (`heal`); for imports raise the sewing tolerance; add the missing face; in a modelling sequence, find the feature before the first issue and change the operands so the boolean can close. |
| `SHELL_ORIENTATION` | edge | The two faces that use the edge run along it in the same direction. | A face with a flipped sense, a loop with the wrong direction. | Heal (`fixOrientation`) makes face senses consistent; a flipped face is `FACE_FLIPPED` in the report. |
| `SHELL_NONMANIFOLD` | edge | The edge is used by three or more faces. Detail: `edge used by N faces`. | Two solids touching along an edge that were not split into two edges; a face glued onto an existing edge. | Separate the touching parts into two lumps, or avoid exact contact along a line. Booleans split such contact edges themselves. |
| `SHELL_EMPTY` | shell | A shell with no live faces. | A repair or edit removed every face of the shell. | Heal drops empty shells (`SHELL_REMOVED`). |
| `SHELL_DISCONNECTED` | face | The faces of one shell are not all connected through shared edges. The entity is the first unreachable face; detail `k of n faces reachable from the first face`. | Two solids or a solid and a void put in one shell by an import or an edit. The builders split such components into their own shells. | Heal does not repair this code on its own: `SHELL_SPLIT` runs only after a sewing step that joined edges. The STEP importer separates components into lumps itself. |
| `EULER` | component | For an edge-connected component of the body, `V - E + F - (L - F)` is odd or greater than 2. The detail prints V, E, F and L. | A component with free edges (the count of an open shell is odd, for example 1 for a box without one face), or an inconsistent loop structure. | Fix the open edges first; the `EULER` issue then disappears. Genus is allowed: 2 for a block, 0 for one through-hole, -2 for two. |

The Euler count is made per edge-connected component of the whole body, across shells, because a surface patch (from `fill` or `knit`) can sit in its own shell next to the shell it caps.

## Loops

These checks look at the closed boundaries of faces.

| code | entity | meaning | typical cause | fix |
|---|---|---|---|---|
| `FACE_NO_LOOP` | face | A planar face with no boundary loop. (A closed analytic face, a full sphere or torus, may have none and is accepted.) | A repair or edit removed the boundary. | Remove the face or rebuild it from its edges. |
| `LOOP_EMPTY` | face | A loop with no edges. | As above. | As above. |
| `LOOP_OPEN` | face | The end vertex of a coedge is not the start vertex of the next one, and their positions are farther apart than `max(tol, vertex tolerances)`. Detail: `coedge i end != coedge j start`. | A gap in an imported boundary, edges stored in the wrong order, a vertex that was moved. | Heal merges near-miss vertices (within the healing tolerance) and re-chains the loop (`VERTEX_MERGED`, `LOOP_CHAINED`). |
| `LOOP_SELF_INTERSECT` | face | Two non-adjacent edges of one loop of a planar face cross. | A bow-tie profile, a bad trim. | Redraw the profile or the trim. Not repaired. |
| `LOOP_CROSS` | face | An outer loop and a hole of a planar face cross transversally. | A hole that pokes out of its face. | Move the hole inside or let the boolean cut through. Not repaired. |

The crossing test works on the uv polylines of the loops. For faces with curved edges each coedge is subdivided (at least 2 and at most 12 levels) until the chord sag is below `1e-5` of the face size, and the crossing tolerance follows that sag, so thin annuli and near-concentric loops are not flagged. Touching at a shared vertex or running back over a bridge is not a crossing. Only planar faces are tested; loops of curved faces are not checked for self-intersection.

## Edges and vertices

These checks compare the curves with the vertices and with the surfaces. The distance scale is `scale = 1 + |position of the edge's start|`.

| code | entity | meaning and threshold | typical cause | fix |
|---|---|---|---|---|
| `EDGE_RANGE` | edge | `t0 >= t1`: an empty or reversed parameter range on the curve. | A corrupt edit. | Rebuild the edge. |
| `VERTEX_OFF_CURVE` | edge | The vertex is farther from the end of the edge's curve than `max(10 * tol * scale, edge tolerance, vertex tolerance)`. Detail: `start vertex 1.000e-3 off curve`. | Imported geometry whose vertex positions disagree with the fitted curves; a vertex moved without its edges. | Heal with `tolerant: true` records the deviation as a tolerance on the vertex or edge (up to `cap`); or move the vertex onto the curve. |
| `EDGE_DEGENERATE` | edge | The edge is shorter than `tol` (length of the curve over its range). | A zero-length edge from a sketch with coincident points, a sliver left by a boolean. | Heal removes degenerate edges whose ends were merged (`EDGE_REMOVED`). In modelling, avoid features smaller than the tolerance. |
| `EDGE_OFF_SURFACE` | face | The edge curve does not lie on the face: at 9 points per coedge the distance between the surface at the pcurve's uv and the edge curve exceeds `max(10 * tol * scale, 1e-6 * scale, edge tolerance)`. The detail names the edge and whether the test used the `pcurve` or a `projection` (when the coedge has no pcurve). | Exporters that fit B-spline faces a few `1e-3` away from the analytic surfaces they meet; a face moved without its edges. | Heal with `tolerant: true` (cap 0.05 mm); for bodies built here this signals a defect to report. |
| `EDGE_TOLERANCE_EXCEEDED` | edge | The edge's own tolerance is above `cap` (default 0.05 mm). | A toleranced edge from a repair with a larger cap than the validation cap. | Raise `healTolerance` on the request, or repair the geometry instead of tolerating it. |
| `VERTEX_TOLERANCE_EXCEEDED` | edge | A vertex tolerance of the edge's ends is above `cap`. | As above. | As above. |

## Solids

| code | entity | meaning | typical cause | fix |
|---|---|---|---|---|
| `VOLUME_NONPOSITIVE` | lump | The lump's signed volume is zero, negative or not a number: the solid is inside out or degenerate. Detail: `signed volume ... (inside-out or degenerate)`. | All faces flipped; a degenerate solid made of slivers. | Heal makes the lump outward (`SOLID_FLIPPED`). A degenerate solid must be rebuilt. |
| `SOLID_MULTIPLE_OUTERS` | lump | One lump holds more than one outer shell: closed components with positive volume beyond the first. Negative closed components are its internal voids and are fine. | Two solids left in one lump. | No request repairs this code and `heal` does not touch it; the STEP importer separates components into lumps itself, so it does not arise there. |

A body can hold several lumps (a disjoint union, a cut that splits a part, a multi-solid import). Each lump is judged on its own and the entity names the lump, so breaking one solid does not hide the state of the others. Lumps that overlap each other are legal; the request reports that as information, not as an issue.

## Display mesh

| code | entity | meaning |
|---|---|---|
| `TESSELLATION_OPEN` | face, or several faces joined by `\|` | The display mesh has a hole although the B-rep is closed. Added by the `validate` request, not by the B-rep checks. See [tessellation checks](tessellation.md). |

## What the validator does not check

- Intersection of faces that are not neighbours. Two walls that pass through each other, a body that crosses itself, or two lumps of one body that overlap are not reported as issues.
- Whether a hole loop lies inside its outer loop on a curved face, or whether loops of curved faces cross.
- Surface-level defects: a surface that folds over itself, a NURBS patch with a degenerate corner, a normal that flips inside the face.
- Continuity between faces (tangency, curvature); see the `continuity` request.
- That `pcurve`s exist. A coedge without a pcurve is projected, and only its deviation is judged.

## Reproduced and unit-tested codes

The following reproduce through the request interface: `SHELL_OPEN` and `EULER` (a block exported to STEP with one face removed, then imported without healing), `SHELL_ORIENTATION` (one face of that block flipped), `VERTEX_OFF_CURVE` (the coordinates of a `VERTEX_POINT` of the exported block moved by 1e-3), `EDGE_OFF_SURFACE` (the origin of an edge's `LINE` moved by 1e-3), and `VERTEX_TOLERANCE_EXCEEDED` and `EDGE_TOLERANCE_EXCEEDED` (the tolerances that healing recorded for those two cases, 1.05e-3, checked against a cap of 1e-4); see [the validate request](validate.md). The other codes are covered by the unit tests of `ck-check` (a hole crossing its outer loop, a bow-tie polygon, two outer shells in one lump, a flipped solid, a shell with unreachable faces). They did not arise in the attempts made for this book through the request interface, because the STEP importer and the modelling operations repair or reject those defects before the validator runs (for example, a STEP file with all faces reversed, or with two solids in one shell, imports as a valid body).

## See also

- [The validate request](validate.md)
- [Healing](healing.md)
- [Tessellation checks](tessellation.md)
- [Tolerances](tolerances.md)
- [The B-rep model](../kernel/brep.md)
