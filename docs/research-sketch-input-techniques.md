# Research: sketch input techniques (FreeCAD, Onshape, Fusion)

Ideas only, paraphrased. Status: CONFIRMED = stated by a cited page; UNCONFIRMED = not found in a fetched source (several FreeCAD wiki pages returned an anti-bot "Access Denied", so FreeCAD details rest on the blog, search excerpts and a mirror).

## 1. FreeCAD 1.0+ Sketcher

On-view parameters (OVP), step by step (https://blog.freecad.org/2024/10/20/tutorial-on-view-parameters/):
- Setting: Preferences > Sketcher > Tool Parameters, modes None / Dimensions only (default) / Positions and dimensions. CONFIRMED.
- Rectangle: X-length box is active first; type, Enter, then the Y box activates; Enter again finishes. CONFIRMED.
- Circle: one box (diameter). Line: length then angle (typed, or mouse; Ctrl snaps the angle in 5 degree steps). Slot: length, angle, end radius. CONFIRMED.
- "Positions and dimensions": start X/Y boxes appear before the first click. CONFIRMED.
- Author says to move between boxes with Enter or Tab instead of clicking. CONFIRMED (same page).
- Tab semantics: typing then Tab applies a constraint, updates the shape, moves focus to the next widget (https://www.ondsel.com/blog/floating-input-widgets, the original patch write-up). CONFIRMED for the patch; Enter/lock details not covered there.
- Confirmed values add related constraints; OVP-made constraints take precedence over auto-constraints; the cursor coordinate readout is hidden while OVP shows (via Sketcher Preferences wiki excerpt, https://wiki.freecad.org/Sketcher_Preferences, seen only as a search snippet).
- Lock behaviour: commenters on the tutorial disagree whether moving the mouse after typing changes the value; author did not answer. UNCONFIRMED. (A search snippet claimed a typed value stays locked until its box is clicked again; treat as UNCONFIRMED.)
- Driving versus non-driving display, continue mode, auto-constraint markers: no fetchable source. UNCONFIRMED. Only note: 1.0 revamped geometry colours/dashes (https://discuss.pixls.us/t/freecad-1-0-new-features-and-the-larger-picture/46690, snippet).

Polyline mode switching (https://wiki.freecad.org/Sketcher_Polyline, via search snippet, mirror at gitea.suzanne.soy FreeCAD-Doc localwiki Sketcher_Polyline.html):
- First segment is always a straight line. M cycles the mode for the next segment: perpendicular line, tangent line, tangent arc, left perpendicular arc, right perpendicular arc, back to plain (coincident-only) line. Ctrl snaps arc sweep in 45 degree steps relative to the previous segment. Commit "add perpendicular mode and use M-key" (https://gitea.suzanne.soy/suzanne.soy/FreeCAD/commit/6b0f50d98cdce5159aafda2b69abf2396b174419). Order on current builds UNCONFIRMED.
- Known edge case: perpendicular/tangent modes failing when closing onto the first point (https://tracker.freecad.org/view.php?id=4177, closed "no change required"). A 90-degree tangent arc mode was requested (https://tracker.freecad.org/view.php?id=3472).

## 2. Onshape

Line tool (https://cad.onshape.com/help/Content/Sketch/line.htm):
- L activates. Click per endpoint; each new click continues the chain. Or press-drag-release for one segment.
- Type a number right after placing the second point and press Enter: dimension applied without the dimension tool; "=#" assigns a variable. CONFIRMED. A white length box appears after the second point; rectangles take horizontal then auto-jump to vertical (https://cad.onshape.com/help/Content/Sketch/dimension.htm via search snippet).
- Tangent arc: Shift+A, or hover near the line's end point until the cursor icon changes to the arc icon; then click the arc end. After one arc the tool returns to Line; Shift+A again to chain another. CONFIRMED (line.htm).
- Tangent Arc tool (https://cad.onshape.com/help/Content/Sketch/arc_tangent.htm): starts at the end of an existing line; click start, click end, or press-hold-drag-release. After creation type a radius and Enter. CONFIRMED.
- Forum: hover triggers it; moving away and back returns to Line; "A" switches line to arc and "L" back, but you must re-snap to the endpoint (https://forum.onshape.com/discussion/27544/possible-to-turn-of-the-tangent-arc-automatic-switching-when-using-line, search snippet). Drag-back-through-start toggle is a SolidWorks behaviour; no Onshape equivalent found.
- Inference: suppress per operation by holding Shift (forum report, https://forum.onshape.com/discussion/comment/5270; Drawing docs mention Shift too). No global toggle (https://forum.onshape.com/discussion/20746/how-do-i-turn-off-the-tangent-arcs-created-automatically-function).

## 3. Fusion 360

- Line (https://help.autodesk.com/cloudhelp/ENU/Fusion-Sketch/files/SKT-CREATE-LINES.htm): click start; click next points, or enter length and angle before clicking. Arc: hover an endpoint, click-drag; arc starts tangent to the previous segment; moving to the opposite side of that segment flips direction; radius can be set before more segments. Enter finishes; click start point closes. CONFIRMED. Tab is not named on this page: UNCONFIRMED.
- Tangent Arc tool (https://help.autodesk.com/cloudhelp/ENU/Fusion-Sketch/files/SKT-CREATE-ARCS.htm): click on existing geometry (gets a tangent constraint), then click or type a radius. Snapping to a point adds matching constraints automatically; Sketch Palette switches types. CONFIRMED.

## Keyboard map (as documented)

| Key | FreeCAD | Onshape | Fusion |
|---|---|---|---|
| Tab / Enter | next OVP box, applies constraint | Enter commits typed dimension | Enter finishes tool |
| M | cycle polyline segment mode | - | - |
| Shift+A / A / L | - | tangent arc / arc / line | - |
| Ctrl | angle 5 deg, arc 45 deg snap | - | - |
| Shift | - | suppress inference (reported) | - |

## Typed value to constraint

FreeCAD: confirming a box (Enter/Tab) adds a driving dimension plus related constraints, which override auto-constraints. Onshape: number + Enter after placement adds a normal dimension (driving). Fusion: typed length/angle become dimensions. Pattern: an unconfirmed field is live preview only; a confirmed field is a constraint.

## Tangent-arc continuation and edge cases

Trigger differs: Onshape = hover end point or Shift+A; Fusion = press-drag from end point, side of previous tangent line decides arc direction; FreeCAD = mode key M. Edge cases: accidental switch when the cursor pauses near the end point (Onshape forum); closing onto the first point in tangent/perpendicular mode (FreeCAD #4177); arc direction ambiguity (Fusion flips by side); arc requires a previous segment (all three).

## Likes and complaints

Liked: type-while-drawing with automatic jump to the next field; one tool for chained line and arc. Complained: unwanted auto-tangent arcs, no off switch (Onshape forum links above); mouse movement possibly overwriting typed values and unclear lock state (FreeCAD tutorial comments); cursor readout vanishing under OVP.

## Interaction rules to adopt

State is `{tool, draft, fields, mode}`; events in, draft operations out.

1. `key Tab | Enter` in a field with typed text: `commitField(k)` (emit dimension constraint on the draft entity, mark field locked), focus next unlocked field; after the last field emit `finalizeSegment` and restart chain from its end point.
2. `pointer move` with a locked field: `previewGeometry` solves with locked fields pinned; unlocked fields follow the pointer; a locked value is never overwritten by pointer moves.
3. `key Escape` in a focused field: `unlockField(k)` and clear text; Escape with no field focus: `discardDraft` and end chain (keeps committed segments).
4. `key M` (or `A`/`L`) with chain length >= 1: `setSegmentMode(next)` where mode is in line, tangentArc; if chain is empty, ignore. No hover-based switching.
5. `pointer down+drag` starting on the chain end point: `beginSegment(tangentArc)` with centre derived from the previous tangent; `drag crosses the previous segment's line`: `flipArcDirection`; release commits.
6. `click` in tangentArc mode: `commitSegment` with a `tangent` constraint to the previous segment at the shared end point; mode stays until the user changes it (no auto-revert), or resets to line if a setting says so.
7. `click on start point` of the chain: `commitSegment` plus `coincident` constraint, `closeProfile`, end chain; tangent/perpendicular closing falls back to plain line if the arc is unsolvable.
8. `modifier Shift down` at commit: `suppressInference` (no auto coincident/horizontal/tangent for that click); `modifier Ctrl`: `snapAngle` (5 deg for lines, 45 deg for arc sweep). Show inferred constraints as pending glyphs before the click.
