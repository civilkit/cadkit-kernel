# Text and fonts

A `text` entity in a sketch turns a string into closed loops that extrudes, thin walls and region selection use like any other sketch loop. Engraved or embossed lettering (dial numerals, nameplates, part numbers) is built this way. The layout code is the crate `crates/ck-text`; the entity and its profile regions are in `crates/ck-model/src/sketch/text.rs`.

## The text entity

```json
{"type":"text","id":"t1","string":"RPM x100","font":"gauge","height":8,
 "at":[30,10],"anchor":["center","middle"],"angle":0,"spacing":1}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `string` | string | `""` | The text, at most 500 characters (more gives the warning `textTruncated`). A newline is not a line break: use one entity per line |
| `font` | string | `"gauge"` | A built-in font (`gauge`, `engrave`; aliases `sans`, `serif`, `default`) or the name of a font loaded with `loadFont` |
| `fontHash` | string | none | A prefix of the SHA-256 of the font file the document was made with. A different file under that name gives `textFontMismatch` |
| `height` | number | 5 | Cap height (the height of `H`) in sketch units. Must be in (0, 1e5) |
| `at` | `[x, y]` | `[0, 0]` | Anchor position |
| `p` | string | none | A sketch point id; overrides `at`, so the position can be constrained or dragged. The text itself takes no constraints |
| `anchor` | `[h, v]` or a string such as `"center middle"` | `["left","baseline"]` | `h` is `left`, `center` or `right` of the ink box; `v` is `baseline`, `middle` (height / 2) or `top` (height) |
| `angle` | number | 0 | Degrees, counter-clockwise about the anchor |
| `spacing` | number | 1 | Letter pitch factor, in (0, 20) |
| `mirror` | bool | false | Mirror about the vertical line through the anchor |
| `polyline` | bool | false | Lines within a chord tolerance (0.005 x height) instead of B-spline edges |
| `kern` | bool | false | Apply the font's pair kerning (`kern` table and GPOS `kern`) |
| `liga` | bool | false | Apply the font's standard ligatures (GSUB `liga`) |
| `merge` | bool | false | Union glyphs whose contours cross themselves (overlapping strokes) before layout |
| `runSpans` | number | 4 | Longest run of curve spans in one B-spline edge, clamped to 1 to 16 |
| `arc` | object | none | Lay the letters along a circle, below |
| `construction` | bool | false | When true the entity is ignored: no layout and no profile |

`arc` fields: `centre` (`[x, y]`) or `c` (a sketch point id), `radius` (of the anchor line), `angleCentre` (degrees counter-clockwise from +x, default 90) and `side`: `outside` (default; letter tops point away from the centre and the text reads clockwise) or `inside` (tops point at the centre). Every letter keeps its shape and is rotated as a whole, tangent to the circle.

### Using it

The text is flat on the sketch plane. Put it in a sketch of its own on a plane (standard, offset, or a plane on a planar face), then use an `extrude` with `op: "cut"` to engrave or `op: "add"` to emboss. Text reads correctly when the plane is viewed from its normal side. Its loops are not intersected with the other curves of the same sketch, so lettering drawn inside a rectangle of the same sketch does not become a hole in it. Keep lettering in its own sketch.

A complete check: a 60 x 20 x 3 mm plate, text `RPM x100` of cap height 8 centred at (30, 10) in a sketch on an offset plane at the top face, cut 0.5 mm deep.

```json
[
 {"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":3},
 {"type":"sketch","id":"F3","plane":{"kind":"feature","id":"P1"},"points":[],
  "entities":[{"id":"t1","type":"text","string":"RPM x100","font":"gauge","height":8,
               "at":[30,10],"anchor":["center","middle"]}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":0.5}
]
```

These are the last three entries of the part's `features`, after the plate's own sketch and extrude (a 60 x 20 rectangle extruded 3 mm on the `front` plane).

Real results: the plate volume falls from 3600 to 3540.149 mm3, which is 59.85 mm3, the glyph area (119.70 mm2, from `inspect`) times the depth 0.5. The body has 134 faces, `errors` is empty, and `validate` with `mesh: true` reports no issues. `inspect` on the sketch lists the entity:

```json
{"id":"t1","string":"RPM x100","font":"gauge","fontHash":"a416bff2...","height":8,"ok":true,
 "bbox":[12.37,5.89,47.63,14.11],"area":119.70,"glyphs":8}
```

## Requests

`textMetrics` and `textOutline` are stateless. The font requests below act on the font table of the kernel instance.

**`textMetrics`** `{string, font?, height, spacing?, kern?, liga?}` measures a string without a sketch. For `RPM x100` at height 5 with `gauge` it returns:

```json
{"advance":22.8,"bbox":[0.443,-0.071,22.486,5.071],"width":22.043,"capHeight":5,"chars":8,
 "advances":[3.236,3.221,3.786,1.429,2.843,1.9,3.193,3.193],"warnings":[]}
```

`advance` is the pen advance of the whole string, `bbox` the ink box relative to the pen start on the baseline, `width` its width and `advances` the per-letter pen advances. An empty string is the error `textEmpty: the text string is empty`.

**`textOutline`** takes the entity fields and returns `{regions, bbox, area, warnings}`. Each region is `{outer:[[x,y],..], holes:[[[x,y],..],..]}`, polylines (8 points per curved span) of the loops exactly as the sketch lays them out; `area` is the filled area. The string `Ab` in `engrave` at height 5 gives 2 regions, each with 1 hole, area 13.15, bbox width 8.48. `edges: true` adds the exact line and B-spline control points of each loop.

## Errors and warnings

Errors make the sketch fail (a feature error with a `detail` message):

| Code | When |
|---|---|
| `textEmpty` | The string is empty |
| `textNothingToDraw` | Only blanks or unknown letters |
| `textFontNotLoaded` | The font is not built in and not loaded. The message names the font and says how to load one. There is no substitution |
| `textFontMismatch` | `fontHash` does not match the loaded file |
| `textBadHeight` | Height not a number in (0, 1e5) |
| `textBadArc` | Arc radius not positive |
| `textBadValue` | A position, angle or spacing is not finite, or spacing is outside (0, 20) |

Warnings leave the sketch valid and appear in the regen result's `warnings`:

| Code | When |
|---|---|
| `textTooSmall` | Cap height below 0.8 mm (checked: `cap height 0.5 is below 0.8: ...`) |
| `textMissingGlyph` | The font has no glyph for some characters; they are skipped and listed |
| `textTruncated` | More than 500 characters |
| `textOverlappingContours` | A loaded font's glyph has contours that cross themselves. Set `merge: true` to union them; otherwise validation can fail with `LOOP_SELF_INTERSECT` |
| `textOverlap` | Neighbouring letters still overlap after the spacing adjustment, so booleans cannot merge them |
| `textSpacingAdjusted` | Letters would have touched or overlapped (closer than max(1.2 percent of the height, 0.03)), so later letters were moved apart: on a line in steps of 1 percent of the height, on an arc by widening the pitch in 4 percent steps, at most 60 percent |

## Fonts

The kernel carries two subset fonts compiled into the binary (about 108 KB of wasm). No font is read from the machine, so the same entity gives the same geometry everywhere.

| Name | Face | Licence | Covers |
|---|---|---|---|
| `gauge` | Barlow Condensed Medium | SIL OFL 1.1 | Basic Latin, Latin-1, bullet, minus, dashes and typographic quotes |
| `engrave` | PT Serif Regular | SIL OFL 1.1 | The same set |

Characters outside that set (CJK, Cyrillic, Greek) give `textMissingGlyph` unless a font that has them is loaded. The font reader is part of the crate: TrueType `glyf` (with composites), OpenType `CFF ` Type 2 charstrings, `cmap` formats 4 and 12, `name`, `OS/2`, `hmtx`, `kern`, GPOS `kern` and GSUB `liga`. No external crate is used.

### Loading a font

`loadFont` `{name, data, replace?}` registers a TTF or OTF for the kernel instance (the font table is process-wide, as regen has no other handle on a request: one WebAssembly instance has one table, and sessions of the native service in one process share it). `data` is base64; a `data:...;base64,` prefix is accepted. The reply for a Latin serif test font:

```json
{"name":"plate-serif","hash":"ce049812...","builtin":false,"bytes":5712,"glyphs":33,"chars":26,
 "unitsPerEm":1000,"capHeight":700,"outlines":"glyf",
 "supported":["Basic Latin","Latin-1 Supplement"],
 "coverage":[{"block":"Basic Latin","mapped":24,"of":95},{"block":"Latin-1 Supplement","mapped":2,"of":96}],
 "warnings":[{"code":"fontKerningIgnored","detail":"..."},{"code":"fontLigaturesIgnored","detail":"..."}],
 "alreadyLoaded":false,"replaced":false}
```

Rules:

- Fonts are keyed by name and by the SHA-256 of the file. Loading the same file again under the same name is a no-op (`alreadyLoaded: true`, checked). The same name with a different file is `fontNameConflict` unless `replace: true`. One file under two names is two fonts.
- The built-in names cannot be loaded over: `fontReserved: "gauge" is a built-in font name; choose another name for the loaded font` (checked).
- A name is 1 to 64 characters without control characters or surrounding blanks (`fontBadName`). At most 64 fonts and 24 MB per file (`fontTooMany`, `fontTooLarge`).
- A file that is not a usable font is `fontBadData`: `not a TrueType / OpenType font (unknown file signature)` (checked). WOFF and WOFF2 are refused (give the TTF or OTF), as are CFF2, truncated files and files without a Unicode `cmap` of format 4 or 12.
- The regen cache key of a feature includes the hash of every font its text names, so replacing, unloading or loading a font invalidates exactly the features that use it.

Load warnings (the font still loads): `fontKerningIgnored`, `fontLigaturesIgnored` (kerning and ligatures apply only when the entity opts in), `fontVariableDefaultOnly` (variable fonts give the default instance), `fontCollectionFirstFace` (TTC and OTC use the first face), `fontCffApproximated` (cubic CFF curves become quadratic B-splines within 0.05 percent of the em), `fontNoCapHeight`, `fontOverlappingContours`, `fontEmbeddingRestricted` and `fontNoSubsetting` (the `OS/2` licence flags), and `fontNoLatin`.

`unloadFont` `{name}` answers `{name, unloaded}` (`false` when nothing was loaded; built-in names are refused). `listFonts` `{}` answers `{fonts:[...]}` with the built-in fonts first, then the loaded ones in load order. `subsetFont` `{name, text}` answers `{name, hash, bytes, glyphs, data, warnings}`: a plain TrueType font (base64 in `data`) holding only the glyphs of `text` plus `.notdef` and `H`. Subsetting `ABC` from the test font gave 4 glyphs and 3168 bytes with a `textMissingGlyph` warning for `C`, which the font lacks. The subset loads with `loadFont` and gives the same outlines; a document can carry a subset and pin it with `fontHash`.

## From outlines to loops

- A TrueType contour is quadratic. Consecutive off-curve points make one quadratic B-spline of degree 2 with clamped knots, exact and not flattened. Runs are cut every `runSpans` spans, so a letter is a few edges and the walls of an extrusion are B-spline surfaces, not hundreds of facets.
- A curved span that deviates from its chord by less than 0.3 percent of the cap height becomes a line. Rounded stroke corners of a few percent of the cap height (single-span curves between two lines) are made sharp, because a face 0.1 mm wide cannot be engraved and only hurts the booleans.
- Loops are grouped per letter by nesting depth: even depth is an outer boundary, odd depth a hole. `O 0 8 6 9 A B D P Q R & @ %` give faces with holes; `i j : ;` give separate regions. Outer loops run counter-clockwise, holes clockwise.
- Side faces of an extrusion are named `<feature>.S.<id>#n` like those of any sketch entity.

## Limits

- Text is planar. It lives on a sketch plane, so there is no wrapping or projection onto a curved face. For a curved part, place a plane tangent to the face and accept that the cut or boss is flat.
- A single line per entity, left to right. No right-to-left or complex scripts, no contextual alternates, no bold or italic synthesis (load the bold file instead).
- Kerning and ligatures are off unless the entity sets `kern` or `liga`. Subsets drop the layout tables, so a subset kerns and ligates as a no-op.
- Connected script faces overlap by design. Overlapping letters are not merged; the spacing adjustment pushes them apart or reports `textOverlap`, and a 35-character string can take 0.5 to 2 s. Use them for short names.
- Glyphs whose boundary only touches itself at a point fall back to the unmerged loops with a warning.
- Letters of different entities that overlap are neither merged nor checked.
- A rare display-mesh crack (about 1 case in 1500 random strings) reports `TESSELLATION_OPEN` from `validate` with `mesh: true` between two neighbouring walls, although the B-rep is valid and the volume is right.
- Below 0.8 mm cap height the strokes are too thin to engrave or print.

## Checks

`cargo test -p ck-text` covers glyph area, nesting, orientation, anchors, arcs, metrics, font loading with twelve fixture fonts, and 900 damaged font files. The corpus runner `node tools/text_corpus.mjs` builds 126 cases through the wasm (engraving and embossing, every glyph with holes, rotation, mirror, several planes, dial numerals, arc text, text past a plate edge, error and warning cases, opt-in kerning, ligatures and merge, loaded fonts). Each case checks that there is no error and no `booleanFailed`, that `validate` with `mesh: true` is clean, and that the volume change equals the glyph area times the depth to within 1 percent. In this chapter's run all 126 passed in 53 s.

## See also

- [Sketches](../format/sketches.md)
- [Solids from sketches](../format/features-solids.md)
- [Regeneration and caching](../kernel/regeneration.md)
- [Testing](../quality/testing.md)
