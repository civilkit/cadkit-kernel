# Fillet regression table

`crates/ck-model/tests/blend_table.rs` runs the synthetic fillet cases in `crates/ck-model/tests/fixtures/blend_cases.json`
(written by `tools/gen_blend_fixtures.mjs`) and judges each against `fixtures/blend_expect.json`.

## Running

On the CI box (prints only the per-case lines and the summary; the full log is kept in `target/blend-table/last.log`):

```
tools/blend_table.sh --case '=daisy-hub-n5-V-r1'   # one case (quote it: `=` at a word start is special in zsh)
tools/blend_table.sh --case -V-r                   # a family: every id containing "-V-r" (the daisy V sets)
tools/blend_table.sh --case hole-corner,pipe-tee   # comma = or
tools/blend_table.sh --mode fail                   # only the cases expected to fail
tools/blend_table.sh --json --debug                # + target/blend-table/blend/{table.json,debug/<id>.log}
```

Filter terms: `=id` matches that id only, `^p` ids starting with `p`, anything else is a substring of the id.
Locally the same variables work on `cargo test -p ck-model --profile fast --test blend_table -- --nocapture`:
`BLEND_CASE`, `BLEND_MODE=fail|pass`, `BLEND_JSON=<file>`, `BLEND_DEBUG_DIR=<dir>`, `BLEND_TABLE_OUT=<file>`.

A filtered run judges only the cases it ran. Without a filter every case runs and an expectation without a case
is also reported. Once the box has built the branch, one case or one family comes back in about 20 s
(rsync about 8 s, the incremental rebuild about 6 s); the full table takes about 100 s, 80 s of which is the table.

## Reading the output

```
daisy-hub-n5-T-r1 XFAIL volume volume 12169.6472 vs reference 12174.4127 (base 12226.0455) dvol=+9.230%  t=1.21
<id>              <status> <outcome> <detail, at most 80 chars>                            <dvol>       <seconds>
```

- status: `PASS` (expected to pass, passes), `XFAIL` (expected to fail, fails), `REGRESSION` (expected to pass,
  fails), `NEWPASS` (expected to fail, now passes: flip it). `NOEXPECT`: the case has no expectation.
- outcome: `ok`, the refusal's error code (`blendFailed`, `blendUnsupported`, `blendOverflow`, `missingEdge` ...),
  `invalid` (the body fails validation), `novolume`, `sharp` (a case marked `"sharp": true`, or any case with
  `BLEND_SHARP=1`: a selected edge still has points on the result's boundary, i.e. it was not rounded), or `volume`
  (the volume disagrees with the reference).
- dvol: how far the volume removed (or added) is from the reference's, in % of the reference's; positive means the
  volume is below the reference's (more removed, or less added). `-` when there is no reference or no body. A case passes within 0.2 %.
- t: seconds for the case (base body + fillet).

After the lines: a cluster table (failure mode, count, the first five ids), any problems, and the summary
`N cases, P pass, X expected_fail, W hard, K problems (target X)`. The failure mode comes from one function,
`failure_mode` in the test: known details map to fixed labels (`normal 1`, `foot 1`, `branching selection`,
`concave chain end`, `spring overflow`, `centre did not converge`, `singular centre`, `edge used by N`,
`volume mismatch`); otherwise the first three words of the detail, or `<code> (no detail)`. When adding a new label,
add it there so clusters stay stable. `--json` writes the same per case with `mode`, volumes and validity.

## Adding a case, flipping an expectation

1. Add the case to the spec of `tools/gen_blend_fixtures.mjs` and regenerate `blend_cases.json` (see the header of
   that script for the reference volumes).
2. Add an entry to `blend_expect.json`: `"<id>": {"expect":"expected_fail","observed":"..."}`. To rewrite the whole
   file from what the engine does now, run the table with `BLEND_TABLE_OUT=<file>` and review the diff.
3. When a fix makes a case pass, the table reports `NEWPASS` and fails: set its `expect` to `"pass"`.

## Overhang cases

The `overhang-*` families are a 43 x 33.5 x 10 block with a second block of the same size sitting on its top and sticking out past two of its
walls (the saved part behind the "all connected" report). Where the second block's wall and underside meet the first block's top and wall
there are two 4-valent corners, each four selected straight edges as two collinear pairs that flip convex/concave (`overhang.rs`). Rows:
the four edges of each corner (`-v1-4edges`, `-v2-4edges`, the "two lines" a user picks), every proper subset of them (`-v1-sub<picks>`),
both corners (`-both-8edges`), every edge of the part (`-all`, `-all-chain`), and the tangent chain on a single edge or one edge of each line
(`-line-chain`, `-two-lines-chain`), at 0.5, 1 and 2 mm for the first four kinds and 1 mm for the rest (the fillet ones; chamfers below). As the stepped-disc families these are
written by the kernel's test, not by `gen_blend_fixtures.mjs`:
`OH_EMIT=<file> cargo test -p ck-model --test overhang_fillet emit_corpus`, then `node tools/merge_stepped_corpus.mjs <file> overhang`
(replaces every `overhang*` entry; the references are baseline volumes). The strict checks are `tests/overhang_fillet.rs`
(`SWEEP=1` prints every subset with its volume change, patch / step-face counts and folded triangles; `ONE=10,11,12,13 R=1 DUMP=<dir>` one
selection with its patch statistics and an OBJ). What it took: (1) the corner patch's section check demanded the spring line of a hexagon
side to be one segment of the zone section, but where the striped body splits that line at the corner (the convex and the concave stripe of
a pair meet on their shared wall) it is two or more collinear segments; the patch now takes the same split. Before, the corner kept its four
stripes ending in planar steps without a word (the fallback after a failed splice is silent, `CK_BLEND_DEBUG=1` shows the reason). (2) The
staged build of mixed vertices (`mixed.rs`) never offered the corner all four edges at once, so with a concave-concave-convex vertex anywhere in
the selection ("all connected") both corners ended in steps and a ragged fan of triangles; it now splices the patch into its result.
(3) The patch's G1 band is never narrower than 7 grid lines (5 left it 0.15 rad off tangent at every radius).

Chamfers (`overhang-chamfer-*`, ids `-d<distance>`): the same selections with an equal-distance chamfer, plus two distances on one face and the
four edges with two distances or an angle (refused: the first distance cannot be measured on one face for all four, and with unequal legs the
strips of the two pairs stop at different distances on the shared faces, so the hexagon the patch needs does not close: see docs/dead-ends.md). A chamfer of distance d
meets each face where a fillet of radius d has its feet, so the corner has the same zone and hexagon (`Corner::chamfer`: the hexagon's
arc sides are the straight strip ends, the patch is the harmonic fill through them, no tangency asked); the corner's edges are built first,
alone, and the rest of the selection on that body (`chamfer_corners_first`: in one pass the chamfers of a mixed vertex reshape the body about
the corner and the patch union is non-manifold). Before, four chamfered edges ended in two planar triangles (+d^3 volume, the V-shaped peaks,
flaps and open pockets of the report), and the "all connected" chamfer was refused ("edge shared by four faces") at the vertex under the
overhang where two concave edges meet a convex vertical one: that vertex now closes in the first round order that does (concave edges
first, then convex first), ending in planar steps. The zone section also ignores faces of other corners that
lie in the same plane.

The mixed vertex under the overhang (`overhang-v3-*`, `overhang-chamfer-v3-*`; the vertex at (21.5, 10, 16.7) of the saved part: two concave edges
where the walls of block A meet the underside of block B, and the vertical convex corner edge of A between them): the fillet builds it in stages
(`mixed.rs`: convex edge first, then the concave chain on a torus) and is the same part in every pick order. The equal-distance chamfer did not: edge by edge
in the pick order it left planar steps or a face of the tool (`F6.S.l*`) and a different volume per order (13.11, 13.61, 14.11 at d = 1; the edge-only
estimate is 14.11). `chamfer_mixed` (`mixed.rs`, called first by `build::chamfer`) now builds it as the fillet does: the convex strip first (it ends on the
underside of B in a straight edge, not tangent to anything), then the concave edges together with that end edge as one selection, the strips mitred at
their joints (the 135 degree turns of the chain), then the overhang corner patches of the selection. Result at d = 1: four strips, no step, 14.097
(the estimate less the joint terms), every pick order. The "all connected" chamfer takes the same path (its pinned volumes moved by 0.7 d^3: the old result
still carried the step at this vertex). A convex and a concave edge of the vertex with the third left sharp used to depend on the pick order too (a tool
face in one order, a capped end in the other); the convex pick of such a pair now goes first (`has_mixed_pair` in `rounds.rs`), so both orders give the
capped end. The same pair with the 4-valent corner V1 or V2 also fully selected used to leave a face of the tool (`F6.S.l*`) in the part (34 selections of the
probe `MIXPROBE=1 MIXKIND=chamfer cargo test -p ck-model --test overhang_fillet mixed_probe`, valid by every check; all 34 already did on main, which had 161 selections with a tool face in the probe, 315 depending on the pick order): a chamfer whose result carries
one, or lacks an overhang corner's patch, is now refused (`blendCorner`, `guard_corners` in `mixed.rs`) with the vertex, the reason and a hint.

## Stepped disc cases

The `stepped-disc`, `stepped-disc-sunk`, `stepped-disc-small` and `stepped-disc-user` families are a 70 mm disc (42 mm for `-small`),
10 tall, whose top is split by a line through the middle with one half moved +5 mm (`stepped-disc`) or -5 mm (`-sunk`; `-user` is
the saved part whose split plane is 0.0023 mm off the centre): every single edge, the rim of each half with the tangent chain on and
off, the wall edges (E2 wall bottom, E5 wall top, E6/E7 the vertical wall|cylinder edges) in pairs and together, fillet and chamfer
at 1 to 4 mm, and the raised half's corner (rim arc + wall top + one vertical edge) described below. They are NOT written by
`tools/gen_blend_fixtures.mjs` (which keeps the ones already in the file when it regenerates): they come from
`STEP_EMIT=<file> cargo test -p ck-model --test stepped_disc_blend emit_corpus -- --nocapture` (the file carries the document), merged
into `blend_cases.json` / `blend_expect.json` by `node tools/merge_stepped_corpus.mjs <file>` (it replaces every `stepped-disc*` entry).
The `reference` volume is the engine's own result at the time of writing (`"source":"baseline"`), so these cases guard against
regressions; they are not OCCT comparisons. Per-case times (`t=`) are 0.01 to 0.5 s, the baseline for speed work.
`tests/stepped_disc_blend.rs` holds the strict checks behind them: the exact (analytic) volume of the rim blends, the wall-edge blends
against an integral, no edge or mesh point outside the cylinder, and `SWEEP=1` runs every subset of the 8 edges.

The raised half's corner (`stepped-disc*-top-*`, `-top-order-*`, `-top-*-nochain`, `-user-top-*`): rim arc + wall-top edge + one vertical
wall edge, the mirror selection, tangent chain on and off, in every pick order, on the 70 and the 42 mm disc and on the saved part whose
split plane is 0.0023 mm off the centre. Two planes and a cylinder meet there. Fillets build at every radius from 0.5 to 4 mm: the three
rounds end on the planes through the rolling ball's centre (`corner::cyl_corner_geom`) and a sphere patch (`F<n>.V0`,
`corner::cyl_corner_tool`) joins them, tangent to the wall, the top and the cylinder. The volume is the sum of the three single-edge
rounds less what the corners count twice (0.26 d^3 on the planar limit), and the same in every pick order and with the chain on or off.
Strict checks: `tests/stepped_disc_blend.rs`, `fillets_of_the_three_edges_at_the_curved_corner_are_rolling_ball_corners` (both discs, all
sides, 372 selections), `the_snapped_split_part_gets_the_corner_round_in_every_pick_order` (96), and the refusal test; `CMATRIX=1` runs
the whole matrix (768 selections), `CSTRESS=1` other discs, steps and radii.

What it took, for the next corner of this kind: (1) the vertical round of a plane against a convex cylinder is analytic now (its tool
reaches out past the cylinder instead of following the chord of the section, which left the lens between chord and cylinder standing and
sent the round to the free-form strip, which the patch cannot join); (2) with a curved corner in the selection the tools are built in one
round, straight tools first, rim arcs last, whatever the pick order (with the rim tool first, the part whose split is 0.0023 mm off the
centre failed at its far wall corner, where the wall-top tool runs on past the end of the rim round; the cause is not traced further); (3) the tangent-chain attempt (tool ends flush at the seam of the rim) runs first also with
the chain off, the other half of the seam-split rim counted in, or the seam leaves slivers; (4) the validity gate also refuses a result of
a curved corner with an edge off its surface or a face crossing itself (`validity::GEOMETRIC`), so that the next attempt gets its try.

A radius the corner cannot hold is refused up front, in every pick order, with the reason: a vertical edge (the step) that does not run
on 0.25 mm past the ball's contact line (`-too-big` cases: 4.8 and 5 mm on a 5 mm step; the radius must stay 0.25 mm under the step), or a
cylinder too small for the ball (radius above half the cylinder's). All the chamfers build. The corner's own result has no defect; at some
radii the other end of the wall top (the top and rim rounds meeting with the vertical edge unselected) leaves a 1e-4 to 1e-3 mm edge, as
it does without the corner patch (top + rim alone at 3.9 mm on the 42 mm disc), and the two rounds cross there at a right angle.

Known and not asserted: slivers (edges shorter than 0.05 mm) where a blend ends tangent to a face or where an E5 and an E6/E7
chamfer meet on the round wall; one arc of a seam-split rim chamfered without the chain runs on past the seam (about 7 % too much);
a 0.5 mm fillet of one half of the sunk piece's seam-split rim leaves a pcurve 1e-3 off its surface; a single half of a seam-split rim
filleted without the chain, alone (no corner), can come back with a few tiny torus pieces at the seam (an edge of 0.02 mm at 2 mm on the
42 mm disc; the other half alone is refused above 0.71 mm). The same fillet in a selection of more edges takes the flush ends above.

## Wheel (hub, two sector plates, rim)

`wheel-*` (written by `tools/gen_blend_fixtures.mjs`, base built in the script): a hub (radius 7.5, 6 deep), two opposite quarter-sector
plates (radius 7.5 to 50, 5 deep) and a rim ring (radius 50 to 55, 30 deep). R10 on the vertical concave edges (plate side with hub,
plate side with rim): on the plain wheel and, after an R5 where the plate tops meet the rim, on the hub-side four, it builds, and the
volume added is the rolling-ball section of each edge over the plate's depth (`tests/wheel_fillets.rs` checks it to 0.2 %). The rim-side
edges after the R5 (`wheel-r5-then-vertical-r10`, `-rim-side`, `-r3`, `wheel-r2-then-vertical-r10`) are refused, at any radius, with
`blendUnsupported` "concave chain end is not on a planar face" and a hint saying that the round ends on the torus of an earlier round at
a given point: the vertical edge runs up to the R5's torus (the plane face above the plate is only the R5's end wedge), the ball would
have to leave the plane over the convex plate-top edge, and no corner patch joins a concave round to another round (the patch of
`corner.rs` is for three rounds meeting at a vertex, planes plus a cylinder). The other order (R10 first, R5 after) builds (`tests/wheel_fillets.rs`): the eight R10 rounds are now exact cylinders (the rim-side
wall lines, a plane meeting the inside of a ring, are built by the analytic line builder like the hub-side ones: `is_rim_wall_line`, the
tool reaches a little into the material behind the wall so that no tool face lies on it); the R5's chain of arcs then runs on 0.25 rad
past each end of the plate to where the R10 rounds touch the rim wall, where the plate top ends along the top edge of an R10 round, and
its tool is trimmed by that round's cylinder (the overflow trim). That cylinder is tangent to the rim wall the tool's own face lies on, so
the trim is retried with the tool reaching into the material behind its two faces (`INTO_MATERIAL`), and in that retry the far end of the
chain is trimmed as well as the first. The added volume is the R10's plus the R5's fill over the plate top (within 1e-3 of the integral).
(Before, the R10 rounds were free-form and the R5 answered `blendFailed` "no blend face in result": the trim of the contact leaving the top
was taken against the rim wall instead of the round and cut the whole tool away.) A rim-side wall line whose end is on a round of an
earlier feature stays on the free-form path (the R5-then-R10 refusal above).

Corner patches: a convex/concave edge-pair corner (four edges at a vertex) whose patch cannot be built or spliced is refused with
`blendCorner`, naming the vertex, the reason and a hint, instead of returning the body with planar steps at the corner.

## hard

`"expect":"hard"` with a `"reason"` (and optionally `"alsoFails":["occt","solidcore"]`) marks a case that the
reference kernels we compared also fail. It does NOT mean we will not do it: it means no known engine does it yet,
so it is worked after the cases at least one engine solves, and a valid answer may well exist (a vertex blend that
consumes a short arc, for example). It is judged like `expected_fail` (it must still fail; passing reports
`NEWPASS`, then set it to `"pass"`) but counted on its own and left out of the target count. Use it only with
evidence that other kernels fail too, and say which in `alsoFails`.

## Debug dumps

With `--debug` (`BLEND_DEBUG_DIR`), each selected case is run a second time in its own process with
`CK_BLEND_DEBUG=1`, and its output goes to `<dir>/<id>.log`; earlier `.log` files there are removed first.
`CK_BLEND_DEBUG2` / `CK_BLEND_DEBUG3` set for a local run are passed on. The traces come from
`crates/ck-ops/src/blend/diag.rs` and the blend modules; `blend fail:` lines give the reason a step gave up.
Read the last lines of a log first. Debug runs take about twice as long, so filter to a few cases.

## Wasm

The product runs the kernel as wasm, which can decide a marginal case differently (its libm differs in the last
ulp). `tools/wasm_parity.sh` runs the table through the wasm and lists the cases that differ; see `docs/wasm-parity.md`.

## Rounds past their face (branch convex-cliff-overflow)

A convex round or chamfer larger than the face or wall it runs onto is built (on the supports continued, cut from the body)
when one of its two contacts stays on its face; the feature warns `blendRanOffFace` with the `faces` it ran off. Rows that
flipped from expected_fail to pass, with analytic references (the round's section clipped to the 2 mm plate, integrated):
`plate-edge-r3` (2325.004152626), `plate-edge-r5` (2223.647609009), `plate-ring-r3` (2147.845375949; the four rounds meet
in mitres, removed = integral of min(2, depth(s)) over the boundary strips of perimeter 140 - 8 s). All three agree with the
built volume to 0.000 %. No other row of the 447 moved. Still refused: both contacts off their faces.
