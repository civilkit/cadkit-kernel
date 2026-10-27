# Shell and thicken of free-form lofts

Two gaps from internal notes (not published) (multi-station polygon lofts): shelling a closed loft with a
free-form face opened, and thickening a surface loft whose faces meet in creases. This note records
what the original kernel does (as far as it was read or observed), which parts of its decompiled
output were used, and what we do. Nothing here is transliterated from the decompilation; the
algorithms were re-derived and checked against the original's results.

## What the original does

### Thicken (read from the decompiled output and black-box dumps)

- The thicken body is one large function (decompiled `f3107`, in `all.c`). It holds the strings
  `thickenReach` ("the offset faces do not meet at a corner (the offset reaches past a face)"),
  `thickenTooThick` ("an edge vanishes", "runs to a point at ... its offset there is no point"),
  `thickenPointedPole`, `thickenFailed` ("a wall's loops do not tell its side", "the thickened body is
  not closed") and the layer names `front offset copy` / `back offset copy`.
- It walks the sheet's vertices (records of 0xa0 bytes) and solves a corner point for each through
  `f2551` / `f2552` / `f2553` (the corner solvers), failing with `thickenReach` when the faces at the
  vertex have no common offset point. So creases are offset by mitering, there is no per-face
  offset followed by a boolean.
- Debug dumps of the result (`regen` with `debug: true`) show the structure: one layer is the sheet
  itself, the other layer's faces are exact offset surfaces (`kind: offset`) of the sheet's faces;
  every rim edge gets a spline wall (`F9.W<n>`); walls meet along short straight connector edges
  that join each sheet corner to its mitered corner.
- Numerically, the layer's corner points agree with "offset surfaces of all faces at the vertex
  intersected with the plane normal to the crease" to about 0.01 mm. A layer rim edge between two
  such corners of a straight sheet rim is the straight edge between them (not the rim translated by
  the offset), which matters at the 0.2 % level for volumes.
- Failures that are structural in the original: the outside (or both-way) thicken of the 7-station
  open "tinnie" U sheet is refused with `thickenReach` even for 0.2 mm, and so is the outside offset
  of the closed-triangle loft. In the original the offset surfaces do not reach the corner's miter
  (the bottom face is only 40 mm wide at the stern); with the faces continued as far as the corner
  needs, the offset is perfectly regular, see below.

### Shell (black-box)

The shell function was not traced in the decompilation (its error codes are looked up through a
table by index, so the strings do not lead to the function). Its behaviour on a free-form opened face
was established from results: the original returns a valid solid for the closed hull with the top
free-form face removed, and our construction (below) reproduces its volume to 1e-7 relative, so the
two agree on the semantics: the cavity is the body offset inward on every kept face and outward
(overshooting) on the opened face, cut from the body.

## What we do

### Shell (`crates/ck-ops/src/shell.rs`, `crates/ck-ops/src/offset.rs`)

The existing construction (cavity = body offset by `-t` on kept faces and by a small overshoot on the
opened face, `body - cavity`) was kept; three things stopped it working on creased lofts:

1. The opened face had to be planar. A B-spline opened face is now overshot exactly like a plane
   (its fitted offset by the overshoot distance).
2. The margin by which a free-form face's offset surface is continued past the face (slack for the
   edge curves) was 3 % of the face's extent, which on a 3.5 m hull is 100 mm of second-order
   continuation that folds over where the last loft span turns quickly. The margin is now capped
   to 12 wall thicknesses (in mm, per end of the face), and when the fit still fails because the
   continuation folds the margin is divided by four and the fit retried (`fit_face_with_margin`).
   A direction of degree 1 (the straight section lines of a polygon loft) continues exactly, so
   there the margin may be as wide as the narrowest end of the face needs.
3. The edge between a loft's planar end cap and a creased free-form side was offset as a straight
   line. The offset of the side is not ruled in the cap plane, so the true meeting curve of the
   two offset faces bends away from the line by up to 0.3 mm. The line is now used only when it
   stays on both offset surfaces; otherwise the edge is sampled like any other free-form edge.

Also the fit gets 24 rounds instead of 14 (a long planar B-spline face needs more refinement).

### Thicken (`crates/ck-ops/src/surfacing/thicken/freeform.rs`, `thicken_creased`)

Used when a shared edge of the sheet is a crease (adjacent normals differ by more than about 1 degree);
smooth sheets keep the earlier path unchanged.

- Layer surfaces: fitted offsets of every face (as before).
- Layer vertices: the offset surfaces of all faces at the vertex plus, for a vertex with fewer than
  three conditions, the plane normal to the crease edge (or, off creases, to the rim edge). This is
  the miter point. If it does not exist or is farther than 50 thicknesses from the sheet vertex:
  `thickenReach`.
- Layer edges: shared edges through the same sampled meeting-curve code as the shell offset
  (`offset_spline_edge`); rim edges are the rim moved along the face normal by the thickness plus
  the corner corrections (miter corner minus the plain normal offset of the vertex) blended
  linearly along the edge, carried back onto the offset surface. Between two plain corners this
  is the plain normal offset; between two miter corners of a straight rim it is the straight
  edge joining them, which is what the original produces.
- Walls: the ruled surface between the sheet's rim edge and the layer's rim edge, sampled at
  matching fractions so that the connector edges are the end rulings; the coedges carry exact
  pcurves (the wall's parameters are the sampling fractions), which keeps edge-to-face projection
  well conditioned on these thin, sheared strips.

Why this and not the alternatives: building the inner cavity of the shell as a loft of in-plane inset
sections would measure the wall thickness in the station planes, not along the surface normal (wrong
where the hull flares quickly); offsetting each face and booleaning leaves the sharp-crease miters to the
boolean. The mitered layer directly reproduces the original's structure and volumes.

## Results

Original-kernel volumes are recorded in `tools/cases/original-gaps/orig_volumes.json`. Tests
(`crates/ck-model/tests/hull_shell_loft.rs`): 10 of 10 shell cases within 0.003 %, 5 of 5 thicken
cases within 0.005 %, the open-section sheet within 0.003 %, plus wall-versus-area sanity tests, a
rounded-section hull, a tapered loft with an analytic wall (exact), and the outside / both-way
thickens of the open-section sheet (valid solids here; the original refuses them with
`thickenReach`).
