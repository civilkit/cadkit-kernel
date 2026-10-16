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
| `font` | `gauge` (condensed DIN-like sans, default) or `engrave` (classic serif); aliases `sans`, `serif`, `default`; or the name of a font loaded with `loadFont` (below) |
| `fontHash` | optional: a prefix (any length) of the SHA-256 of the font file the document was made with; a different file loaded under the name is `textFontMismatch` |
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
(only blanks / unknown letters), `textFontNotLoaded` (the font is not `gauge` / `engrave` and nothing is loaded under that name; the message names the font and says how to load it), `textFontMismatch` (`fontHash` does not match the loaded file), `textBadHeight` (not a number in (0, 1e5)), `textBadArc` (radius <= 0),
`textBadValue` (a position, angle or spacing is not finite, spacing outside (0, 20)).

Warnings (the sketch still regenerates; `warnings` of the regen result, params `entity` and `detail`):

| Code | When |
|---|---|
| `textTooSmall` | cap height below 0.8 mm: strokes thinner than about 0.1 mm do not engrave or print |
| `textMissingGlyph` | characters the font has no glyph for are skipped (the characters are listed) |
| `textTruncated` | more than 500 characters |
| `textOverlappingContours` | a loaded font's letter has outlines that cross themselves (overlapping strokes, variable and distressed fonts): not merged, the solid can fail validation (`LOOP_SELF_INTERSECT`); the characters are listed |
| `textOverlap` | neighbouring letters still overlap after the adjustment below (radius too small): the booleans cannot merge them |
| `textSpacingAdjusted` | neighbouring letters would have touched or overlapped (closer than max(1.2 % of the height, 0.03)): on a straight line the later letters were pushed right in steps of 1 % of the height (serif fonts: the serifs of `vv`, `VV`, `rr` touch at small sizes; `spacing` below 1 on purpose), on an arc the pitch was widened in 4 % steps (tight radius: letters lean into each other), at most 60 % |

## Loading fonts: `loadFont`, `unloadFont`, `listFonts`, `subsetFont`

The wasm carries only the two built-in fonts. Any other TTF / OTF is registered for the session (the kernel
instance) from bytes; the kit hosts the font files and fetches them on demand, or a document carries its own
(subsetted) font. The registry is process-wide because regen has no other handle on a request: one kernel instance
= one font table.

`loadFont {name, data, replace?}` (`data` = base64 of the file, a `data:...;base64,` prefix is accepted):

```json
{ "name": "plate-sans", "hash": "9c1f...64 hex", "builtin": false, "family": "Barlow Regular", "bytes": 104068,
  "glyphs": 694, "chars": 525, "unitsPerEm": 1000, "capHeight": 700, "outlines": "glyf",
  "supported": ["Basic Latin", "Latin-1 Supplement", "Latin Extended-A"],
  "coverage": [{ "block": "Basic Latin", "mapped": 95, "of": 95 }],
  "warnings": [{ "code": "fontKerningIgnored", "detail": "..." }],
  "alreadyLoaded": false, "replaced": false }
```

`supported` lists the Unicode blocks (Basic Latin, Latin-1 Supplement, Latin Extended-A / B, Greek, Cyrillic, General
Punctuation, Currency Symbols, Letterlike Symbols, Arrows, Mathematical Operators, Hiragana, Katakana, CJK Unified
Ideographs, Hangul Syllables) in which the font maps at least one character; `coverage` has the counts. `chars` is the
number of mapped characters. Characters a font lacks are `textMissingGlyph` as before.

* Fonts are keyed by **name and content hash**. The hash is the SHA-256 of the file. The same name with the same file
  again is a no-op (`alreadyLoaded: true`). The same name with a different file is the error `fontNameConflict`
  unless `replace: true`, which swaps it (`replaced: true`). The same file under two names is two fonts.
* The built-in names (`gauge`, `engrave`, `sans`, `serif`, `default`) cannot be loaded over or unloaded
  (`fontReserved`). A name is 1 to 64 characters without control characters or leading / trailing blanks (`fontBadName`).
  At most 64 fonts, 24 MB per file (`fontTooMany`, `fontTooLarge`).
* A file that is not a usable TTF / OTF is `fontBadData` with the reason (not a font, WOFF / WOFF2 (give the TTF or
  OTF), missing tables, CFF2, truncated, no Unicode cmap of format 4 or 12). Nothing panics on damaged files
  (fuzzed in `font_tests`).
* The **regen cache key** of a feature includes `name=<sha256 | missing>[^fontHash]` of every font its text entities
  name, so replacing, unloading or loading a font invalidates exactly the features that use it (and what follows).
* A text entity whose font is not loaded makes the sketch fail with `textFontNotLoaded`:
  `font "plate-sans" is not loaded; built-in fonts are gauge and engrave, any other font must be loaded first with
  {type:"loadFont", name:"plate-sans", data:<base64 TTF/OTF>}`. There is no substitution. The same error answers
  `textMetrics` / `textOutline` / `subsetFont` for such a name.

Load warnings (`warnings` of the reply; the font still loads):

| Code | Meaning |
|---|---|
| `fontKerningIgnored` | the file has `kern` or GPOS: letters use plain advance widths |
| `fontLigaturesIgnored` | the file has GSUB: no ligatures or other substitutions |
| `fontVariableDefaultOnly` | variable font (`fvar`): the default instance is read, no axes |
| `fontCollectionFirstFace` | TTC / OTC: the first face is used |
| `fontCffApproximated` | OpenType / CFF: cubic curves are converted to quadratic B-splines within 0.05 % of the em |
| `fontNoCapHeight` | no letter `H`: the cap height is OS/2 `sCapHeight`, else 0.7 em |
| `fontOverlappingContours` | outlines of letters or digits cross themselves or each other (overlapping strokes; typical of variable fonts and distressed typewriter faces); the overlaps are not merged |
| `fontEmbeddingRestricted`, `fontNoSubsetting` | the OS/2 `fsType` licence flags forbid embedding / subsetting |
| `fontNoLatin` | no Latin letters mapped |

`unloadFont {name}` -> `{name, unloaded}` (`false` when nothing was loaded under the name). `listFonts {}` ->
`{fonts: [<loadFont reply without alreadyLoaded / replaced>..]}`, the built-in fonts first, then the loaded ones in
load order.

`subsetFont {name, text}` -> `{name, hash, bytes, glyphs, data, warnings}`: a plain TrueType font (base64 in `data`,
`hash` = its SHA-256) with only the glyphs of `text`, plus `.notdef` and `H` (which carries the cap height). Same
unitsPerEm, same cmap entries for those characters, same advances, composites flattened, hinting and OpenType layout
tables dropped, `name` / `OS/2` copied (so `fsType` stays). A CFF font comes out as TrueType (quadratic
outlines, rounded to font units). The result loads with `loadFont` and gives the same loops as the source for the
characters in `text` (exactly for TrueType sources, within 0.2 % of the area for CFF; tested). Characters the font
lacks are listed in a `textMissingGlyph` warning. The kit embeds this in the document; `fontHash` in the entity then
pins the embedded bytes.

## Fonts, licences, determinism

Two subset fonts are compiled in (`crates/ck-text/fonts/`, `include_bytes!`), nothing is read from the machine:
the same entity gives the same geometry everywhere.

| Name | Face | Licence | Size |
|---|---|---|---|
| `gauge` | Barlow Condensed Medium (Basic Latin, Latin-1, bullet, minus, dashes, quotes) | SIL OFL 1.1, `fonts/OFL-BarlowCondensed.txt` | 19.8 KB |
| `engrave` | PT Serif Regular (same set) | SIL OFL 1.1, `fonts/OFL-PTSerif.txt` | 18.9 KB |

Both were subset with fontTools (`pyftsubset`, unhinted, layout tables and names dropped, unicodes
`U+0020-007E,U+00A0-00FF,U+2022,U+2212,U+2013,U+2014,U+2018,U+2019,U+201C,U+201D`) and renamed `gauge.ttf` /
`engrave.ttf` (the PT Serif reserved font name is not reused). The font reader is ours (`ck-text/src/ttf.rs`:
head, maxp, hhea, hmtx, loca + glyf with composites, `CFF ` Type 2 charstrings, cmap formats 4 and 12, name, OS/2), no external crate. The wasm grows by about
108 KB. Characters outside the subset (CJK, Cyrillic, Greek) give `textMissingGlyph`. A script wanting a different
face loads one with `loadFont` (above). Licences of the fonts the kit may host: THIRD_PARTY.md ("Curated fonts").

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
* No kerning, no ligatures, no bold / italic (pick the bold file), no right-to-left or complex scripts; single line per entity. Variable fonts give their default instance only.
* Connected script faces (Great Vibes, Pacifico, Satisfy): the letters overlap by design; overlapping letters are not merged, so the spacing adjustment pushes them apart (`textSpacingAdjusted`) or reports `textOverlap`, and a 35-character string can take 0.5 to 2 s to lay out. Use them for short names.
* Fonts whose strokes overlap inside one letter (`fontOverlappingContours`): not merged.
* Text is not trimmed against, merged with or made holes of other curves of its sketch (see "Text is its own sketch").
* Letters of one entity are kept at least max(1.2 % of the height, 0.03) apart (see `textSpacingAdjusted`); letters of
  different entities that overlap are not merged and not checked.
* Rare display-mesh crack: one case in about 1500 random strings (`gauge`, 3 mm, 2 mm deep, rotated) reports
  `TESSELLATION_OPEN` from `validate {mesh:true}` between two neighbouring walls (the B-rep is valid, the volume is
  right); engraving depth under the cap height never showed it.
* Tiny features: below 0.8 mm cap height `textTooSmall`; engraving depth is the caller's choice (0.1 to 0.5 mm typical).
* 500 characters per entity.

## Checks

* `cargo test -p ck-text` (`font_tests`: eleven fixture fonts (subsets of the curated list, `crates/ck-text/tests/fixtures`) load and lay out; CFF areas against fontTools; loaded copy of `gauge` identical to the built-in; missing font; name + hash keying; subsets; 900 damaged files; TTC; variable) (area / nesting / orientation / anchors / arc / metrics / contour crossing, polyline vs
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
* `tools/text_corpus.mjs` also runs the loaded-font cases (every fixture font engraved, emboss, on an arc, hash pin, not loaded) and the font scenarios (replace under the same name with the regen cache on, unload, built-in refusal, subset round trip, instance isolation).
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
