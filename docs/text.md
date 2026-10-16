# Lettering: the sketch `text` entity

Engraved or embossed text on parts: dial numerals (`0 10 20 ... 80`), `RPM x100`, `KNOTS`, `OIL PSI`,
`TEMP °F`, nameplates, part numbers. A `text` entity in a sketch turns into closed loops that every
profile consumer (`extrude` add / cut, `thin`, `inspect`, region selection) uses like any other loop.
Crate `ck-text` (outlines, layout), `ck-model/src/sketch/text.rs` (the entity and the profile regions).

## Entity (wire)

```json
{ "type": "text", "id": "t1", "string": "RPM x100", "font": "gauge", "height": 4,
  "at": [0, 0], "p": "p7", "anchor": ["center", "middle"], "angle": 0, "spacing": 1,
  "mirror": false, "polyline": false,
  "arc": { "centre": [0, 0], "c": "p1", "radius": 28, "angleCentre": 90, "side": "outside" } }
```

| Field | Meaning |
|---|---|
| `string` | the text; at most 500 characters (more: `textTruncated`); `\n` is not a line break (one entity per line) |
| `font` | `gauge` (condensed DIN-like sans, default) or `engrave` (classic serif); aliases `sans`, `serif`, `default` |
| `height` | **cap height** (height of `H`) in sketch units (mm). Default 5 |
| `at` `[x, y]` | anchor position. `p` (a sketch point id) overrides `at`, so the position can be constrained, dragged or driven by a parametric point; the text itself takes no constraints |
| `anchor` | `[horizontal, vertical]` (or a string `"center middle"`): `left` `center` `right` x `baseline` `middle` `top`. Default `["left", "baseline"]`. Horizontal refers to the **ink box** of the string (left edge of the first letter, right edge of the last), vertical: baseline = 0, middle = height / 2, top = height |
| `angle` | degrees, counter-clockwise about the anchor |
| `spacing` | letter pitch factor (advance widths times this), default 1, allowed 0 to 20 exclusive |
| `mirror` | mirror about the vertical line through the anchor (text on the back of a clear part) |
| `polyline` | lines within a chord tolerance (0.005 x height) instead of B-spline edges |
| `arc` | lay the letters along a circle (below) |
| `construction` | true: ignored (not laid out, no profile) |

Arc layout: `centre` (or `c`, a sketch point id), `radius` is the radius of the **anchor line** (baseline for the
default vertical anchor; with `middle` / `top` the baseline moves in or out so that the middle / top sits on the
radius), `angleCentre` the direction in degrees (CCW from +x) of the middle of the text as seen from the centre
(default 90), `side`: `outside` (default) = letter tops point away from the centre and the text reads clockwise
(a title above a dial: `angleCentre` 90), `inside` = letter tops point at the centre and the text reads
counter-clockwise (a title below the dial: `angleCentre` 270). Every letter keeps its shape and is rotated as a
whole (tangent to the circle). For upright dial numerals place single `text` entities at the numeral radius with
`angle` 0 (see `tools/text_corpus.mjs`, "dial numerals").

The text is flat on the sketch plane like any sketch: use a sketch on a plane (standard, offset plane, a plane on a
planar face), then `extrude` with `op: "cut"` (engrave, `depth` = engraving depth) or `op: "add"` (emboss).
Sketch handedness: text reads correctly when the sketch plane is viewed from its normal side
(front / top / right and feature planes all verified, see Renders below); `mirror` flips it.

### Text is its own sketch

A text entity is fixed geometry. Its loops are **not** intersected with the other curves of the same sketch: the regions
come after the arrangement's regions, so letters drawn inside a rectangle of the same sketch do not become holes of
it. Keep lettering in a sketch of its own (the normal pattern: one sketch for the plate, one for the text on the plate
face). Several text entities in one sketch are fine (a dial); overlapping letters from different entities or an
overlapping pitch are not merged (below).

## Requests

`textMetrics {string, font?, height, spacing?}` (stateless):

```json
{ "advance": 22.8, "bbox": [0.44, -0.07, 22.49, 5.07], "width": 22.04, "capHeight": 5, "chars": 8,
  "advances": [3.23, 3.22, 3.78, 1.42, 2.84, 1.9, 3.19, 3.19], "warnings": [] }
```

`advance` is the pen advance of the whole string, `bbox` the ink box relative to the pen start on the baseline
(use `width` to size a plate or centre, `advances` for per-letter placement). A string of blanks has an advance and a
zero box. Errors are strings `textEmpty: ...`.

`textOutline {<the entity fields>}` (stateless): `{regions: [{outer: [[x,y]..], holes: [[[x,y]..]..]}], bbox, area, warnings}`.
Polylines (8 points per curved span) of the loops exactly as the sketch will lay them out, for the viewport preview.
`area` is the filled area (outer minus holes).

`inspect {ref: {kind: "sketch", id}}` lists `texts: [{id, string, font, height, ok, bbox, area, glyphs, warnings?}]`
for the sketch's text entities, and the usual `regions` / `areas` include the letters.

## Errors and warnings

Regen of the sketch fails (feature error, `detail` has the message) with: `textEmpty` (empty string), `textNothingToDraw`
(only blanks / unknown letters), `textBadFont`, `textBadHeight` (not a number in (0, 1e5)), `textBadArc` (radius <= 0),
`textBadValue` (a position, angle or spacing is not finite, spacing outside (0, 20)).

Warnings (the sketch still regenerates; `warnings` of the regen result, params `entity` and `detail`):

| Code | When |
|---|---|
| `textTooSmall` | cap height below 0.8 mm: strokes thinner than about 0.1 mm do not engrave or print |
| `textMissingGlyph` | characters the font has no glyph for are skipped (the characters are listed) |
| `textTruncated` | more than 500 characters |
| `textOverlap` | neighbouring letters still overlap after the adjustment below (radius too small): the booleans cannot merge them |
| `textSpacingAdjusted` | neighbouring letters would have touched or overlapped (closer than max(1.2 % of the height, 0.03)): on a straight line the later letters were pushed right in steps of 1 % of the height (serif fonts: the serifs of `vv`, `VV`, `rr` touch at small sizes; `spacing` below 1 on purpose), on an arc the pitch was widened in 4 % steps (tight radius: letters lean into each other), at most 60 % |

## Fonts, licences, determinism

Two subset fonts are compiled in (`crates/ck-text/fonts/`, `include_bytes!`), nothing is read from the machine:
the same entity gives the same geometry everywhere.

| Name | Face | Licence | Size |
|---|---|---|---|
| `gauge` | Barlow Condensed Medium (Basic Latin, Latin-1, bullet, minus, dashes, quotes) | SIL OFL 1.1, `fonts/OFL-BarlowCondensed.txt` | 19.8 KB |
| `engrave` | PT Serif Regular (same set) | SIL OFL 1.1, `fonts/OFL-PTSerif.txt` | 18.9 KB |

Both were subset with fontTools (`pyftsubset`, unhinted, layout tables and names dropped, unicodes
`U+0020-007E,U+00A0-00FF,U+2022,U+2212,U+2013,U+2014,U+2018,U+2019,U+201C,U+201D`) and renamed `gauge.ttf` /
`engrave.ttf` (the PT Serif reserved font name is not reused). The TrueType reader is ours (`ck-text/src/ttf.rs`:
head, maxp, hhea, hmtx, loca, glyf with composites, cmap formats 4 and 12), no external crate. The wasm grows by about
108 KB. Characters outside the subset (CJK, Cyrillic, Greek) give `textMissingGlyph`. A script wanting a different
face has to wait for font upload (not built).

## How the outlines become loops

* A TrueType contour is quadratic; consecutive off-curve points make one **quadratic B-spline** (degree 2, clamped
  knots `0,0,0,1,2,..,m,m,m`), exact, not flattened. Runs are cut every 4 spans, so a letter is a few edges (`O` =
  2 loops of 4 to 8 edges), the extruded walls are B-spline surfaces, not hundreds of facets. Straight parts are lines,
  collinear lines are merged.
* A curved span that deviates from its chord by less than 0.3 % of the cap height becomes a line: the tessellator
  samples such a near-straight B-spline edge differently in the wall and in the face it bounds (open display mesh,
  found with the corpus sweep).
* Barlow rounds every stroke corner by 1.4 % of the cap height. These slivers (single-span curves under 3 % of the cap
  height between two lines) are made sharp: a face 0.1 mm wide cannot be engraved and only hurts the booleans.
* Loops are grouped per letter by nesting depth (even = outer boundary, odd = hole of the loop one level up): `O 0 8 6
  9 A B D P Q R & @ %` give faces with holes, `i j : ;` and the degree sign give separate regions. Outer loops are
  counter-clockwise, holes clockwise, as the sketch profiles expect. Glyph contours of both fonts never cross each
  other (unit test over every glyph).
* The region's segments carry the entity id as `tag`; the side faces of an extrusion are named `<feature>.S.<id>#n`
  like those of any sketch entity.

Debug fields: `runSpans` (1 to 16, default 4) is the longest run of curve spans in one B-spline edge; `edges: true`
on `textOutline` adds `outerEdges` / `holeEdges` (the exact line / B-spline control points of every loop).

## Limits

* Planar only (as every sketch). No text on a curved face: project or wrap is not built.
* No kerning, no ligatures, no bold / italic, no right-to-left or complex scripts; single line per entity.
* Text is not trimmed against, merged with or made holes of other curves of its sketch (see "Text is its own sketch").
* Letters of one entity are kept at least max(1.2 % of the height, 0.03) apart (see `textSpacingAdjusted`); letters of
  different entities that overlap are not merged and not checked.
* Rare display-mesh crack: one case in about 1500 random strings (`gauge`, 3 mm, 2 mm deep, rotated) reports
  `TESSELLATION_OPEN` from `validate {mesh:true}` between two neighbouring walls (the B-rep is valid, the volume is
  right); engraving depth under the cap height never showed it.
* Tiny features: below 0.8 mm cap height `textTooSmall`; engraving depth is the caller's choice (0.1 to 0.5 mm typical).
* 500 characters per entity.

## Checks

* `cargo test -p ck-text` (area / nesting / orientation / anchors / arc / metrics / contour crossing, polyline vs
  B-spline area within 1 %).
* `node tools/text_corpus.mjs [--only substr] [--write dir]`: 54 cases built through the wasm: engraving and
  embossing of gauge texts, every glyph with holes, accents, rotation, mirror, three standard planes plus feature
  planes, dial numerals `0` to `80`, arc text outside and inside, text over a through hole, text past the plate edge,
  error and warning cases. Each checks: no error, no `booleanFailed`, `validate {mesh:true}` clean (watertight
  mesh), volume change = glyph area (shoelace over the `textOutline` loops) x depth within 1 % (the measured
  deviation is under 0.1 %; bounds only for the cases where the text leaves the plate or crosses a hole), time.
* `node tools/text_corpus.mjs --fuzz N --seed S`: random strings (printable ASCII and a few Latin-1), both fonts, 3 to
  20 mm, 0.1 to 2 mm deep, rotated, all anchors, embossed or engraved; 2000+ cases over ten seeds, one display-mesh
  crack (above), no `booleanFailed`, volume within 1 % of glyph area x depth (cases where the text leaves the plate
  only bounds). `--text STRING [--font f --height h --depth d --at x,y --poly --runspans n]` runs one case.
* `tools/lint_protocol.mjs` fuzzes `textMetrics`, `textOutline` and mutates an engraved plate (every text field).

## Renders

Rendered with the kit headless renderer from the corpus parts (`--write`): legible, reading left to right when the
plane is viewed from its normal side on `front`, `top`, `right` and feature planes (offset plane in all corpus cases);
`mirror` reads reversed; the dial shows `0 10 ... 80` upright on a 270 degree sweep; arc text outside reads clockwise
over the dial, arc text inside reads left to right below the centre with the tops pointing at the centre.

## What the script / GUI should expose

See the report that came with this change: script API `text(string, x, y, {font, height, anchor, angle, spacing,
mirror, arc})` on a sketch (builder helper `Sketch.text` exists), `textMetrics` / `textOutline` in the session, a Text
tool in the sketch editor (click for the anchor point, a string field, height, font, anchor grid; the preview from
`textOutline`).
