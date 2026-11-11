# Editable SolidWorks import (`ck-swedit`, spike)

`crates/ck-swedit` turns an imported SolidWorks part (`ck-sldprt`) into one of our part documents: a feature history
(planes, sketches, extrudes, fillets, chamfers, holes, shells) that regenerates to the same body. It is a spike: the
numbers below say what rebuilds and why the rest does not.

## Idea

The file gives three things we already read, and nothing of the sketches needs decoding:

1. the exact Parasolid body (`import_sldprt`);
2. the feature tree in order, with types and dimension values (`swXmlContents/KeyWords`, `SwMeta::features`);
3. on every face, the feature that made it (`FACE_ID_2001`, `FaceTag::sw_ids`; a face two features share, such as a
   cut wall flush with a side, carries one tag per feature; the tag's kind code 40 marks an extrude's end cap).

For each feature in tree order the faces it left on the final body are read back into the feature's own parameters:

| feature | read back from its faces |
|---|---|
| boss / cut extrude | direction: the one that makes every face a cap (plane normal to it) or a side (plane or cylinder along it), decided by the cap tags, then by the extent that equals the listed `D1`; extent: between the caps, else the faces' extent; profile: the outline of a cap whose rim borders only the feature's own sides (inner loops of other features are left out), else the cycles of the side faces through their edges along the direction, else the side *traces* (every side's infinite line / circle in the sketch plane, with its two ends; ends are paired across sides at the intersection of their traces nearest to both, cheapest first, which recovers corners a later fillet, chamfer or slot removed, and continuation across a side a later cut split; two ends left over are joined by a line, the side a flush face of another feature absorbed); a cut without caps reaching the body's extent is `through`; the sketch sits at the bottom of a boss and at the mouth of a cut |
| drafted boss / cut (`draft.rs`) | tried when the plain read-back fails on tilted sides: direction: a cap normal or cone axis about which every side (tilted plane, cone) makes the same `n . d` (`sin` of the draft); sign: towards the tool's travel is `draft` (inward), away `draftOut`; extent between the caps, with one cap and sides that end short the listed `D1` from the cap; profile at the sketch level (the neutral plane): a clean cap there, else the cycle of the sides through their shared edges (each side's trace at the level: plane -> line, cone -> circle, meeting at the corner nearest the shared edge), else the traces with ends carried down the sides' rulings and paired as for the plain extrude (corners a later fillet removed); a through cut is blind a little past the body. A separate SolidWorks `Draft` feature on the base leaves its faces tagged with the extrude, so it reads back as the extrude's draft and the `Draft` step has no faces. Tests: `tests/draft.rs` (drafted base boss with round corners, boss on a box, outward boss, pocket, round-cornered pocket, filleted drafted corners, far end from `D1`) |
| fillet | every strip face (cylinder / torus; spheres are corner patches) with the two neighbours it is tangent to; the pick point is the sharp edge the supports would meet at (`c - r (nA + nB) / (1 + nA.nB)`, `c` on the rolling ball's centre curve); one feature per radius |
| chamfer | every strip plane / cone with the two neighbours of its longest edges; pick point on the supports' intersection line / circle; `distance` and `distance2` measured from each rail to the other support (`chamferType: distances` when unequal) |
| hole wizard | the `ck-recog` hole groups made of the feature's faces (diameter, through / depth, drill tip, counterbore, countersink); identical holes share one sketch; a hole the grouper refuses (its mouth breaks an edge) is rebuilt as a round cut |
| shell | removed faces: planar faces outside the feature with an inner loop bordering only the shell's faces; thickness: the listed `D1`, else the offset between an inner plane and its parallel |

Sketch planes are standard planes with an offset when the direction is a principal axis, else a `frame` plane.
Dimensions are scaled by the document's length unit (`SwUnits::length`). Support faces of fillets and chamfers are
named by the extrude that made them (`F3.S.e2`, `F3.C1`); `ck-ops` resolves a pick by the faces' features and the
pick point even when the names moved, so a strip between two earlier blends resolves too.

The document is regenerated (`ck_recog::rebuild`) and compared with the imported body (`verify`: symmetric
difference by our booleans, area, bounding box). The first feature that cannot be read back (or a regen error or a
failed verification) stops the rebuild there: the steps before it that pass their local checks stay editable and an
`importBody` feature (`replace`) puts the file's own body in place of theirs, so the final body is exactly the plain
import (`prefix.rs`, design and numbers in `sldprt-editable-wp4.md`); with no such step the file keeps its plain
solid. Every step is reported (`Outcome::steps`, `Outcome::checks`).

## Tests

`crates/ck-swedit/tests/prefix.rs`: the kept prefix (unsupported third step, nothing to keep, wrong depth and regen
error localised to their step, a cut that adds material, a committed fixture whose composed document regenerates to
the import). `crates/ck-swedit/tests/synthetic.rs`: round trips on parts made in-test (box; boss, pocket and through slot;
fillet, chamfer and holes; shell; an oblique extrusion; an unsupported feature that falls back).
`tests/corpus.rs` runs over a local directory of part files (`SW_CORPUS`, default `corpus-local/` at the workspace
root, not in the repository) and writes a Markdown report (`SWEDIT_OUT`).

## Results on the local corpus (38 part files; test fixtures without a body are not counted)

Current state: 23 of 38 files rebuild completely (21 exact, symmetric difference and area below 1e-6; 2 close), 15 are
partial (the rebuilt features followed by the file's own solid for the rest), and none fall back to the plain solid
alone. No file rebuilds to a wrong body: the verification never had to reject one. On the earlier 41-file corpus the
same run gave 23 complete, 15 partial and 3 plain fallbacks; the 3 files that were the fallbacks held foreign bodies and
were removed from the corpus. (The first measurement, 16 of 45 complete, predates the revolve, pattern, loft and blend
work.)

Why the 15 partial files stop: blend regeneration (`missingEdge` on 4 files, `blendOverflow` on 1, an unnamed fillet
face on 1, a rebuilt body 28% off on 1); extrude profile or direction not recovered (5: a spline face, no side faces
in 2, no shared direction, no profile); modelled threads (2); an off-axis linear pattern (1); a counterbore cap (1).

The two tables below are from that earlier 45-file run and are kept as a record of the feature-level gaps at that time;
the revolve, pattern and loft rows no longer describe the current state.

Per feature step over all files (a file stops at its first unsupported feature, so later features are not counted):

| type | rebuilt | unsupported |
|---|---|---|
| boss / cut extrude | 63 | 8 |
| fillet | 7 | 1 |
| chamfer | 2 | 2 |
| hole wizard | 3 | 0 |
| shell | 3 | 0 |
| revolve | 0 | 9 |
| imported body | 0 | 3 |
| linear pattern | 0 | 2 |
| loft | 0 | 2 (now 2 close, see Lofts) |
| thread | 0 | 1 |

Fallback causes, ranked: revolve not implemented (9 files, 6 of them whole-body cone / sphere / torus test parts);
extrude profile not recovered (5: sides consumed by later features in ways the trace pairing does not close, an arc
side whose samples leave no free gap, a feature whose faces sit at three levels); imported bodies (3: there is no
history to rebuild); drafted extrudes (2: the base's sides are tilted planes); linear patterns (2); lofts (2);
chamfers (2: supports chosen wrong on a chamfer between parallel planes, a support face of an earlier blend);
a fillet strip tangent to only one neighbour (1); a thread (1, cosmetic, should be skipped); one part regenerates
with `missingEdge` on its chamfers (the pick points land off the rebuilt edges).

The realistic parts (the NIST test cases and the pendulum parts) rebuild their first 1 to 8 features before stopping;
the base extrude of every one of them is now recovered except the two drafted ones.

## Patterns and mirrors (`pattern.rs`)

`LPattern`, `CirPattern` and `Mirror` features are read back from the faces tagged with the pattern's own id (the
seed keeps its tag; the feature list names no seed and lists spacings rounded for display). A candidate transform comes
from pairs (face of an earlier feature, face of the pattern) of the same surface type and size: a translation along a
world axis, a rotation about a world axis through the origin, a reflection in a plane. It is accepted when every
pattern face is the image (as a surface: plane, cylinder, cone, sphere, torus) of a face of an earlier feature under
one of the instance transforms and at least one such face moved. Counts come from the dimensions (`D1`, `D2`), the
spacing from the geometry. The seeds are the earlier extrude / hole / pattern features, chosen greedily by the
pattern faces their moved faces explain. Emitted as `lpattern` (one or two directions) / `cpattern` / `mirror`; the
copies' faces are named `<seed face>@<pattern>.<k>` so later blends can pick them. Limits: our `lpattern` direction and
`cpattern` axis are world axes (a circular pattern about any other axis falls back); the mirror plane may be any plane;
fillets or other features inside the pattern's faces make it fall back.

## Lofts (`loft.rs`)

`Loft` features (boss; a name starting `Cut` or a cut loft type is a cut) are read back from their faces: the lateral
faces are the feature's faces except its planar end caps (`FACE_ID_2001` kind 77). The edges of the lateral faces that
border any other face form closed planar loops; loops in one plane are one section (largest = outline, the others
inner loops, a hollow loft). Two section planes give a two-section loft; one plane whose rails meet at a single vertex
gives a loft to a point. SolidWorks splits the section curves where the lateral faces meet, so both sections carry the
same piece count and the kernel's start matching pairs them as SolidWorks did. Pieces become lines, arcs / circles
(also B-spline edges that run on a line or circle) or `cvspline`s (non-rational B-splines with their control points and
knots). Straight rails are a plain (ruled) loft; curved rails that are single cubic spans leaving a section along its
normal are `startNormal` / `endNormal` with the tension `3 |C1 - C0| / L` (`L` the chord between the section
centroids, the kernel's loft parameter). The section on earlier material (no cap) is the first sketch, else the lower
one along the loft's main axis; the order matters for our loft's surfaces. Not read back (the file falls back):
more than two sections, guide curves, tangency to adjacent faces, rational spline sections, thin lofts. Faces:
`<loft>.S.<entity of the first section>`, caps `C0` / `C1`. Tests: `tests/loft.rs` (square to circle, normal end
constraints, boss on a block, cut into a block, to a point, hollow, three sections falling back).

On the local corpus both saves of the loft test part rebuild `close` (relative error 1e-5 and 4e-5): SolidWorks
stores the circle-side of every lateral face as a cubic approximation of the arc, ours is the exact rational arc. Their
verification goes through the boolean symmetric difference (spline faces are not matched by `fastverify`) and takes
over a minute per file.

## Profiles and blend supports later features consumed (`profile.rs`, `blend.rs`, `shell.rs`)

On the large realistic parts most extrudes lose part of their outline to later features. The read-back now:

* closes side-trace ends nothing meets with chords the material justifies: a boss's chord through material the
  features rebuilt so far made (any closing there gives the same union; the ends may first run on along their sides
  into that material), or nowhere through the final body (a later cut removed that stretch); a cut's chord nowhere
  through the earlier material (the cut breaks out of the body; its ends run on outwards first). The two free ends of
  one open chain are tried before ends of two chains, cheapest first; then a side interrupted in the middle continues
  along its own trace. The earlier material is the document rebuilt so far (`ck_recog::rebuild`), queried with
  `ck_ops::contain`. Ends no safe chord reaches stay unrecoverable (the walls and the cap there are both gone);
* rejects outlines that are no simple polygon (crossing, sides running back over a neighbour, overlapping
  collinear sides, loops without area), drops sides of zero length, and pairs two sides at most once per corner;
* merges faces covering the same stretch of one trace (a side split along the direction) into one side, and finds
  the ends of an arc side despite sample points a hair outside them (a blend trimming the side across the direction);
* retries with the faces the feature made itself when its tagged faces include walls it only shares;
* takes the union of the cap outlines of one level (one cap per region: a cut of two holes);
* with one cap between the ends of the faces, or a boss whose sides end short of the listed depth, measures the far
  end as the listed `D1` from the cap;
* counts faces a split line cut out of a side (tagged with the split line only) as sides, names unclaimed
  split-line faces lying on a side after it; `Split Line` features themselves are skipped (no geometry).

Faces keep the name of the earlier feature they are flush with (sides and caps). Blends: a neighbour a later feature
made is no support (unless that leaves fewer than two); a planar chamfer strip's supports are the two planes whose
intersection runs in the strip with rails parallel to it (never the caps across its ends); a torus between a support
and a later face is a corner patch; the pick point lies on both support surfaces; the supports are named as the
document rebuilt so far names the faces of its edge at the pick point (merged and split faces included); blend faces
are named `B<k>` by edge order. Shell: the rim of a removed face may also border faces of later features.

Tests: `tests/recovery.rs` (two tabs whose inner ends sit in the base, two notches cut out of an edge, chamfer on a
short vertical edge between caps, chamfer whose rail a later chamfer cut back, concave fillet at a boss foot).

## Sheet metal parts (`sheet.rs`)

A SolidWorks sheet metal part is read as editable sheet metal as far as the tree can be read back; the rest stays the
imported remainder (`sldprt-editable-wp4.md`), so the body is always the file's own.

| SolidWorks feature | ours | how |
|---|---|---|
| `Sheet-Metal` with a `Thickness` dimension (the rules; no faces) | `sheetRules` | `Thickness`, `D1` = bend radius, `D2` = k-factor (kept only in 0..1; else the kernel default) |
| `Sheet-MetalN` (no dimensions), `Flat-Pattern`, `Cut list`, bend-line / bounding-box sketches | skipped | no geometry |
| `Base-Flange` | `baseFlange` on the rules | the faces are read like a boss extrude (`profile::extrude`) with the cap outline as the later features left it (corner rounds and cuts into the rim baked in, holes later features cut left out); the extrude depth must equal the rule's thickness |
| `Fillet` whose faces are round corners of the plate or of the leg of an edge flange (cylinders about the normal of a base flange or edge flange cap that meet that cap) | no feature | already in the outline of the base flange or of the leg (read from the final cap); a corner round that carries a lower id than the flange it rounds is read after that flange (`leg_round_order`: ids are not the history's order there) |
| `Cut-Extrude` / hole wizard slot square to the plate, into its rim | no feature | already in the base flange outline |
| `Cut-Extrude` square to the plate, inside it, through the sheet | `sheetCut` | the cut's sketch (an extrude cut converted) |
| round hole wizard hole through the plate | `sheetCut` | so that flanges can follow it (the kernel refuses a flange after a plain hole) |
| other hole wizard slots and holes (kinds `HoleWizard`, `... Slot`, `... Hole`) | the hole / cut chain of plain parts | |
| `Edge-Flange` (+ the `EdgeBend` feature after it, no step of its own) | `edgeFlange` with `profile`, `from` / `to`, `angle`, `radius`, `position: outside` | see below |
| `Miter-Flange`, `Hem`, `Jog`, `Sketched-Bend`, ... | unsupported | the step stops the read; the steps before it are the editable prefix, the file's body the remainder |

**Edge flanges** (`sheet::edge_flange`). The `EdgeBend` feature holds the bend: two cylinders about one axis whose radii differ by the
thickness (the smaller is the `radius`). The angle the leg turns through is the angle between the radius vectors of the inner
cylinder's two tangent lines; the flange turns towards the axis (which tells the side of the parent: the pick point is put on that
side of the parent's mid-plane); the bend's extent along the axis is the span on the parent edge (`from` / `to`, on the parent's
mid-plane at the bend's start line). The leg is the outline of the flange's inner cap face (the face tangent to the inner
cylinder) in `(v, h)` coordinates, `h` from the outer virtual sharp (`h = leg + (r + t) tan(a / 2)`), lines and arcs as they are
(`profile`); its inner loops are `holes` (circles, or polygons of lines; a hole with other curves is not read back exactly and
stops the read there). The flange hangs on the parent exactly where the bend starts, so the position is `outside` (nothing of the
parent moves) and the parent is found by the pick: the base's outline edge at the bend, or the edge of an earlier flange's leg
(its outline was read from the final cap, which ends where the next bend starts). SolidWorks's own `height` / offset
dimensions are not used: only the faces. What does not read back stops the history there (the editable prefix and the imported
remainder, as for every other feature): a flange whose bend or leg face has been split or cut through, a leg with a hole that has
other curves than a circle, a bend with a non-uniform radius.

Because panels, bends and legs are united in our solid, coplanar faces that SolidWorks keeps apart per feature (a bend's end
face beside the leg's side face) are one face here. The verification (`fastverify::close_by_shell`, only for parts with a base
flange) takes volume and area within 1e-6 and every surface of the file carrying the same total area in the rebuild as `close`
(not `exact`).

The three corpus parts (`..._BRUSHLESS_CHUTE_V3/V4/V5`, 133 to 193 tree entries) read back as follows (test:
`crates/ck-wasm/tests/sldprt_sheet.rs`, skipped when the files are absent; `CK_SLDPRT_SHEET_DIR` names their directory):

| part | Edge-Flange steps with faces / in the tree | result |
|---|---|---|
| V5 | 5 of 17 (the other twelve are suppressed in the file's configuration: no faces, skipped) | the whole history: rules, base flange, 5 edge flanges (a round hole in one leg, arcs in two legs, one on the leg of another), the corner rounds, the sheet cuts and slots; no remainder; volume error 4e-13 of the file's |
| V4 | 7 of 17 | the whole history (7 edge flanges), no remainder; volume error 3e-13 |
| V3 | 8 of 17 | the history regenerates to 3.3e-6 of the volume (the oblique bend ends are free-form faces in the file and planes here, so the faces do not pair): `close` is false and the read keeps the editable prefix (62 steps) and the file's body as the remainder |

Not read back: a hem, miter flange, jog or sketched bend (no corpus part has one); an edge flange on a curved edge; a bend that
SolidWorks cuts through later with a feature that is not square to the plate (the cut, not the flange, stops the read).

## Not done (next steps, in order of value)

1. revolves (whole-body cone / sphere / torus parts and revolve features: the `ck-recog` revolve chain restricted to
   the feature's faces about the axis its faces share);
2. extrudes whose sides were drafted later (the faces are cones / tilted planes tagged with the extrude);
3. patterns and mirrors (faces tagged with the pattern carry the seed's shape; emit `lpattern` / `cpattern` /
   `mirror` over the seed feature);
4. fillets approximated as B-spline blends in the import (rolling-ball faces), variable fillets, threads, lofts,
   sweeps; cosmetic threads are no geometry and should be skipped rather than stop the rebuild;
5. per-feature fallback: done (WP4, `sldprt-editable-wp4.md`) as prefix + replace-body remainder; a remainder that
   follows edits of the prefix (the delta booleans, mode B) is the open part.
