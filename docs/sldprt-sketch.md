# SolidWorks sketches and feature parameters (`ck_sldprt::sketch`)

`Contents/Config-<n>-ResolvedFeatures` holds, per feature, the object the SolidWorks feature tree keeps in memory. Until now
`ck-sldprt` read only the feature class names from it (`classes.rs`). `ck_sldprt::sketch` reads the sketches (points, lines,
arcs, circles, relations, dimension parameters) and the parameters of the features that follow them (an extrude's depth, a
revolve's angle). It is a reader for what the parts in the local corpus show, not a parser of the object graph: the field layout
of most classes is not known, so records are found by their fixed shape and everything else is skipped.

Source: `crates/ck-sldprt/src/sketch.rs`. Tests: unit tests with small synthetic streams in the module,
`crates/ck-sldprt/tests/sketch.rs` (the four committed fixture parts; the local corpus when `SW_CORPUS` or `corpus-local/` is
there). Research aids: `examples/dump_rf.rs` (writes the stream of each file), `examples/dump_sketch.rs` (prints the decoded
sketches; `V=1` for every point and curve).

Derivation: the layouts were first sketched from the MIT-licensed `sw2sc` converter's notes (read only, nothing run, nothing
copied; see `THIRD_PARTY.md`) and then re-derived and checked on our own sample parts. Where this document and that project differ
the corpus decided: two findings are ours (curve records index points by their position in the record list, not by id; a
point handle in a relation is that position too).

The format notes of the cadmpeg project (`docs/formats/sldprt.md` of github.com/cadmpeg/cadmpeg, CC BY 4.0; sections "Sketch
objects", "Extrusion feature objects", "Principal-plane profile placement" and the plane-frame paragraphs of section 2) were read
as hypotheses, as knowledge only: no text or code is copied. Each one used below (the arc sense, the plane transform stored with
a plane reference, the offset plane's frame, the ellipse record, the end specification word, the operand handles of a dimension)
was re-derived from our own streams and checked against the geometry, the bodies or the `KeyWords` tree before it was used; what
the corpus did not show is marked as unverified. Attribution: `THIRD_PARTY.md`.

## API

```rust
let set = ck_sldprt::sketch::read(&container, &limits)?;       // or sketch::parse(stream_bytes, &limits)
for s in &set.sketches {                                          // in stream order
    s.name; s.feature_id; s.plane; s.plane_refs;                  // PlaneRef: Plane(id) | OnFeature(id) | Ambiguous | Unknown
    s.points;   // SkPoint { id, x, y (metres, sketch plane), located, role, counters, on: [CurveHandle] }
    s.curves;   // Line | Arc { start, end, centre, centre_certain } | Circle { rim, centre, centre_certain } | Ellipse { rim, centre, major, minor } | Unresolved
    s.relations;// Relation { kind: RelationKind, targets: [RelTarget] }
    s.params;   // dimension parameters D1, D2, ... (metres or radians)
    s.dims;     // Dimension { name, kind: Distance|HDist|VDist|Radius|Diameter|Angle, value, targets }: operands decoded, value = measure
    s.frame();  // PlaneFrame { x, y, o }: the three standard planes, or the transform stored with the plane reference
    // (sketch_profile.rs of ck-swedit places a sketch with no frame by matching it to the faces' profile)
    s.exact();  // every curve resolved and every centre certain, no notes
}
set.features;   // FeatureRec { name, id, word, op, params, classes, offset, end, end_type, from_type }: every named feature record
set.feature("Boss-Extrude1").unwrap().params[0].value;           // the extrude depth, metres
set.sketch_before(i);                                             // plain adjacency; the caller must verify
```

Errors (never a panic; `SldprtError::code`): `sketch.no_stream` (no ResolvedFeatures stream), `sketch.malformed` (too short),
`limit.stream` (the stream is over `Limits::max_stream`), `limit.sketch_entities` (over 2 000 000 entity records or 200 000
feature records). Truncated and byte-flipped copies of the fixtures are parsed in the tests and must give an error or a result.

## What was found

All numbers are little-endian; lengths are metres, angles radians. "Modern" is a stream that starts `u32 start number, u16, FF FF`
(SolidWorks 2015 and later, `_MO_VERSION_9000` and up); older streams start `u16, FF FF`.

**Class table and tags.** A new class is `FF FF <schema u16> <len u16> <ASCII name>`; the first object of the class follows. Later
objects of the class start with `0x8000 | class number`. A class takes the next number, its first object the one after, every
later object one. The numbers drift in big files (a tag-looking value inside a field this reader does not know is counted), so
`class_tags` is an estimate. It is used only to order classes: the tags seen in point handle lists (usually two: lines and arcs)
are matched to the curve classes in declaration order, nearest estimated number when fewer tags than classes were seen
(`resolve_tags`). Spline and text handles are told apart from lines and arcs but not read (curves of those classes come out as
`Unresolved`); an ellipse is read (below).

**Feature records.** `<class tag, high bit> FF FE FF <len u8> <UTF-16LE name>`, then four zero bytes, three bytes that are zero
except for an extrusion (a family word `0x0140` boss / `0x01ca` cut and an operation byte: the fields `classes.rs` reads to tell
a cut from a boss), a flag byte (`0x40`, `0x80`, `0xc0`), the feature id (u16) and two zero bytes. The id is the id of the
`KeyWords` tree and of the `FACE_ID` tags (checked: sketch 26 is the first sketch of every SolidWorks 2022 sample part). A
feature record runs to the next one.

**Parameter records** (`D1`, `Hole Dia.`, ...) have the same name shape followed by `FF FF FF FF` after the flag byte; then,
modern: four zero bytes, an empty text (`FF FE FF 00`), two zero bytes; older: two zero bytes; then the value as a double. The
name is the dimension's own (`D1@Sketch1`'s `D1`), not unique in the file. They sit inside the record of the feature that owns
them. Checked: 689 feature dimensions of the corpus compared with the `KeyWords` values (millimetres, degrees, as displayed)
agree (0 differ) after leaving out instances (`Sketch2<3>`) and integer parameters (a pattern count is not a double).

**Entity records** (points and curves). `u32 id`, then the header `FF FF <schema u16> <version u8> FF*8` (schema `0x1f` or `0`,
version 1 or 3 in the corpus), the fixed fields `00 00 80 BF`, six bytes, `04 00 02 00 01 00` (or the `05 00 01 00 01 00`
variant), `00 00 00 00`, `80 BF`, and then `F0 3F`, the top bytes of a double 1.0 (everything between is the same in every
sample bar a few flag bytes; two records of the same header carry zeros there and are not points or curves). After the `F0 3F`:

* SolidWorks 2022 streams put eight zero bytes next, older ones do not; the reader decides per stream by a vote of the point
  records (the SolidWorks 2016 streams of the corpus have the newer header and the older record).
* a point: `1E 00`, x (f64), y (f64), `u16 a`, `u16 n`, then the handles of the curves it lies on. A handle is `<tag, or an
  inline class declaration> <u16 id> FF FF FF FF` and, modern, `00 00 00 00`. The counters `(a, n)` are `(0, k)` for a point on
  `k` curves (the handle list can be shorter: a construction line's end points do not list it), `(6, 0)` for the origin
  reference, `(2, 2)` for the point a circle is stored by (its rim). A reference to something outside the sketch (a projected
  edge, a temporary axis) has a class tag or declaration where a plain record has `00 00` after the fixed fields: the origin
  reference carries coordinates and a name (`Point1@Origin`); a temporary axis reference carries none (`located = false`) but
  still takes its place in the point list, which the relations count.
* a curve: `u16 first, u16 last, u16 flag`, then two padding bytes, a double -1.0 and an i32 *sense*: for the 183 arcs 1 (120) or -1
  (50) (13 read other values: not an arc record of this shape), for circles 1, for lines 0. The sense is the arc direction (see below). The two numbers are positions in the sketch's point records, not ids (a rectangle's
  records are in id order and the two coincide; the circle sketches' and the sphere's do not), `flag` is 0 for construction
  geometry and 1 otherwise. A circle has `first == last`, its rim point. A line and the arc of a half circle over the same two
  points are two records with the same numbers; each takes one of the handles both end points list.
* sketches: the records between one feature record and the next. The sketch's own header record (`first = 31`) and records with
  other flags or numbers outside the point list belong to other kinds of record and are counted in a note, not read.

**Arc centres and circles.** The stream stores an arc by its two end points and a circle by one point on it; the centre is an
ordinary point with no curve handle (counter `(0, k)`). There is no field that names it. `resolve_centres` finds candidates
(handle-less points equidistant from both ends for an arc; the record written just before the rim point, else the origin
reference, for a circle: in every sample the centre is written right before the point that closes the arc) and then applies the
relations (equal radii, tangent lines and arcs): when exactly one assignment satisfies all of them an arc's centre is certain
(`centre_certain`); a circle's centre is certain only when it is the sketch's only possible centre or a dimension of the sketch
equals its radius or diameter (`confirm_radius`, run by `read`). A rounded rectangle is the case that needs the relations: its
fillet arcs are semicircles whose chord midpoint is also equidistant from another arc's ends.

**Relations.** `u32 type`, `02 00 00 00`, two zero bytes (2018 and 2022 streams) or four (2011 and 2016 streams), `FE FF 00 00`, then the handles of what is related: lines and arcs by handle id,
points by position in the point records. Type meanings were found by testing each code against the geometry of all 289
sketches in all four versions (`tests/sketch.rs`): a code is named only when every record that could be tested satisfies one predicate.

| code | meaning | targets | records tested |
|---|---|---|---|
| 4 | horizontal | line | all satisfy |
| 5 | vertical | line | all satisfy |
| 9 | coincident | point, point (also point on a line or arc) | all satisfy (86 point pairs; point on line 21 of 21 resolved) |
| 27 | collinear (not parallel: all 34 resolved are collinear) | line, line | all satisfy |
| 14, 57 | equal (length for lines, radius for arcs); two codes, not told apart | line, line / arc, arc | all satisfy (14: 60 of 60; 57: 12 of 12) |
| 6 | tangent | arc, line / arc, arc | all satisfy (79 arc-line, 6 arc-arc) |
| 12 | midpoint | point, line | 20 of 20 |
| 11 | symmetric about a line | point, point, line | 4 of 4 (the other 138 name a construction line) |
| 25 / 26 | same y / same x | point, point | 40 of 40 / 22 of 22 |
| 17 | probably fix (one line, arc or point; 260 records) | one target | no predicate to test |
| 7, 10, 13, 16, 32, 53, 65, 66 | not identified | | their lines are construction lines or the targets did not decode |

`Other(code)` keeps the rest with the targets. Lines that are construction lines have no curve handle in their end points, so a
relation naming only construction lines cannot be matched to a curve (the "not identified" rows).

**Sketch plane.** The records name every plane or feature the sketch refers to (its plane, the origin, the planes of dimension
anchors, the feature whose face it is on): after the sub-record tag found behind the first `moCompRefPlane_c`, `02 00`, zeros,
16 bytes of `FF` and zeros come the feature id and a save stamp. `Sketch::plane_refs` keeps the ids (stamps and -1 dropped);
`resolve_plane(planes, features, ignore)` (run by `read` with the `KeyWords` plane features and the origin feature) gives
`PlaneRef::Plane(id)` when exactly one reference is a plane feature and none is another feature, `OnFeature(id)` when exactly
one is a non-plane feature, else `Ambiguous` or `Unknown`. In the 2022 streams all 13 sketches resolve to a plane (2 is Front, 3
Top, 4 Right in these parts; an offset plane resolves to its own id). In the 2018 streams 15 of 240 resolve to a plane, 5 to a
feature, 10 are ambiguous and 210 are `Unknown`: most of those streams do not declare `moCompRefPlane_c` at all (the class was
declared in another section), so the marker is not found. 2011 and 2016 streams: `Unknown`.

*Frame.* A plane reference record carries the plane's transform: after the feature id and the save stamp come `00 00`, the plane
object's id, `00 00`, a flag byte and, with flag `1`, twelve f64: the rotation row by row with the plane's x axis, y axis and
normal as its columns, then a translation `t` (a sketch point `s` is at `R (s - t)` in the model, its origin at `-R t`); flag `0`
is the Front plane and stores zeros. `plane_refs` keeps these (`Sketch::plane_frames`) and `resolve_plane` takes the one of the
sketch's own plane (`Sketch::stored_frame`, `Sketch::frame`, `Sketch::to_model`, `PlaneFrame { x, y, o }`). Checked on the
corpus: the Top records of the 4 Top sketches store `1 0 0, 0 0 1, 0 -1 0` (x = +X, y = -Z, normal +Y) and the Right records
`0 0 1, 0 1 0, -1 0 0` (x = -Z, y = +Y, normal +X): the frames the bodies had settled for Front and Right
(`examples/cal_frames.rs`: each sketch's points mapped into model space under the eight in-plane axis assignments and counted
against the body's vertices; Front: x to +X, y to +Y, confirmed weakly because the cube sketches are squares; Right, 6, 6 and 4
matching vertices of three sketches against 0 under every other assignment). Top could not be told from its mirror by a body (its
sketches in the samples are symmetric); the file's own transform says x = +X, y = -Z, which is also what the cadmpeg notes give for
the Top selector, so `verified` is now true for all three. Offset planes: the 4 sketches of the corpus on a plane other than the
three standard ones whose reference carries a transform give a frame with the parent plane's axes and an origin off the origin:
(0, 0.01, 0) m for the loft sample's sketch on `Plane1` (whose own `D1` is 10 mm: the plane lists the distance), (0, 0.06604, 0),
(0, 0.13335, 0) and one with an origin that is not on the normal through the model origin ((0.0615, 0.029, 0) m) for NIST parts
(the plane's own `D1` is the offset from the plane it was made from, not from the origin plane). Sketches on a face have a
transform only where the face reference carries one (none of the 4 `OnFeature` sketches of the corpus does).
The cadmpeg notes also name an offset plane's data object (point, unit normal, flag) after its reference: not used, the stored
transform of the sketch's own reference is enough (unverified: a sketch on an offset plane whose reference has no transform).
Plane normals from cylinder sketches: plane 2 about +Z, 4 about X, 3 about Y.

*No frame in the stream.* 244 of the 289 sketches have no plane reference the reader can find (the 2018 and older streams: the
class is declared in another section). `ck-swedit` then places the sketch by matching it to the profile the faces give (below).

*Dimensions.* The operands of a dimension are written right after its parameter record: `<tag> <u16 slot> FF FF FF FF 00 00 00 00`
(the handle shape of the relation records; first found within 240 bytes of the value, before the next parameter record). A
dimension has two parameter records with its name (`D1`), in either order: one holds a display quantity, the other the driving
value; each is followed by the operands. The relation class written in front of the first one of the stream (`sgPntPntDist`, `sgLLDist`, `sgPntLineDist`,
`sgPntPntHorDist`, `sgPntPntVertDist`, `sgCircleDim`, `sgAnglDim`, `sgValG3Dim`, `sgOffsetDim`, `sgSkOffsetDim`, `sgPoint3DPlaneDist`,
`sgCircularPattCntDim`, `sgLinearPattCntDim`) uses a token afterwards that drifts like every other: the classes are not used. The kind
is told by the operands and settled by the value: a two-point operand list is a distance, a horizontal or a vertical distance
(`DimKind::Distance`, `HDist`, `VDist`), an arc or circle a radius or a diameter, two lines an angle or the distance of parallel
lines, a point and a line a distance; the dimension is kept (`Sketch::dims`, `Dimension`) only when the stored value equals that
measure on the sketch's own geometry (to 1e-7 relative). In `read` the parameter record whose number the `KeyWords` tree lists for
that sketch and name (millimetres, degrees) is the driving one, so a display record that happens to equal a measure is never taken
for it, and a dimension the tree lists under another number is never kept.
Kinds that are not read: three-entity angles (`sgValG3Dim`), offset distances, pattern counts, point to plane distances. Corpus: 534 dimensions are kept over the 289 sketches (below).

*Ellipses.* A curve record with first = last (like a circle's, the point the curve is stored by) whose six u16 after the sense are
not `centre FFFE FFFF ...` but the centre's index followed by the indices of four distinct points is an ellipse (`SkCurve::Ellipse`,
the 104-byte ellipse record of the cadmpeg notes; the notes' u16 `0` before the centre is the centre's index when it is point 0).
It is read only when two of the listed points are perpendicular from the centre and every listed point, the rim point included,
lies on the ellipse with those semi-axes (`major` is the end of the longer one, `minor` the shorter semi-axis in metres). Corpus: one
ellipse in the 38 files (one of the NIST test cases: a sketch whose centre is point 0 and whose axes end at (+-90, 0) and (0, +-120) mm). Splines
(`cptsSplineList_c` after the curve type of 0x40 in the notes): no file of the corpus has a spline handle or such a record, so the
record is not read (unverified, nothing to check it against).

*Extrusion end specification.* The `moEndSpec_c` object (first object of the class declared in the stream, later ones by token)
stores the end type as a u32 sixteen bytes after its token (`FeatureRec::end_type`; `from_type` is the start type word of
`moFromEndSpec_c`, 0 for the sketch plane). The estimate of the token drifts in big streams, so it is taken from the stream: the
token that stands in front of `00 00 01 00` and a small word as often as there are extrusions after the first. The numbers follow
SolidWorks' public end condition enumeration (0 blind, 1 through all, 2 up to next, 3 up to vertex, 4 up to surface, 5 offset from
surface, 6 mid plane, 7 up to body, 9 through all both ways); the corpus checks (`ck-swedit` corpus test, "END SPECS": the word against
the rebuilt step) are in the section on the rebuild. Verified there: 0, 1 and 6; 2, 4 and 5 are consistent with whether a depth is
listed but the rebuild reads them as blind; 3, 7 and 9, a second direction (the extra bytes after types 1 and 2) and a start type
other than 0 (`from_type`) do not occur in the corpus: unverified.

## Corpus results

Local corpus: 38 part files (the NIST test cases, the pendulum parts, the SW2011/SW2022 controlled models), both containers.

* 38 of 38 streams decode (no error); 289 sketches, 1862 lines, 183 arcs, 90 circles, 747 relations whose geometry was tested
  (0 violated), 689 feature parameters compared with `KeyWords` (0 differ).
* 186 of 289 sketches decode exactly (every curve resolved, every centre certain): 10 of 13 in the 2011 files, 20 of 23 in the
  2016 files, 144 of 240 in the 2018 files, 12 of 13 in the 2022 files. The rest have an uncertain circle centre (a circle has no
  geometric test, see above), an arc whose relations leave several centres, or a curve of a class not read (text, offset: 33
  curves; the one ellipse is read now).
* Dimensions (`Sketch::dims`, `read`): 534 driving dimensions decode with their operands and are kept (322 distances, 74
  horizontal, 32 vertical, 44 radii, 36 diameters, 26 angles); 480 of them are listed by name and value in the `KeyWords` tree
  (the other 54 are in sketches or under names the tree does not list: hole wizard sketches, a sketch with no dimension list; none
  is listed there with another value: 0 contradicted). `tests/sketch_dims.rs` recomputes the measure of every one.
* Frames: 15 sketches on Front, 4 on Top, 4 on Right, 4 on another plane that carries a stored transform; the Top and Right
  transforms are the ones used for the standard frames; 244 sketches have no plane reference the reader can find.
* End types of the 101 rebuilt extrusion steps (`ck-swedit` corpus test, "END SPECS"): 0 for 78 (all of them list a depth), 1 for
  11 (none lists a depth), 2 for 2 and 4 for 7 (none lists one), 5 for 2 and 6 for 1 (they list one).
* Against the imported body (`corpus_sketches_against_the_body`): 65 of 71 circles whose centre is certain have the radius of a
  cylinder, cone, sphere or torus of the body; the lines of each part's first sketch have the length of a body edge in the
  share the test prints (a later cut or fillet removes edges). The fixtures: the cube sketch's area times the extrude depth is
  the body's volume to 1e-9, the hole sketch's circle (centre (5, 5), radius 2.5 mm) gives the body's volume to 1e-6, the cone
  triangle turned about its axis (Pappus) gives the cone's volume.
* Versions (the `_MO_VERSION_<n>` of the file): 4700 (SolidWorks 2011, OLE2, 10 files), 9000 (2016, 7 files), 11000 (2018, 10
  files) and 15000 (2022, 11 files) all decode points, curves, relations and parameters. A plane reference decodes in the 2022
  streams and in 5 of the 2018 files' sketches; older streams and the other 2018 sketches give `Unknown` (the record is there
  but the marker class is not declared in this stream, or the nested layout differs).

## Not read, and doubts

* Splines (no file of the corpus has a spline handle or a `cptsSplineList_c`: the cadmpeg notes' record is not read, unverified),
  text (the `sgTextHandle` records of three NIST parts, `moDimText_c`), offset entities and their `sgOffsetDim` / `sgSkOffsetDim`
  dimensions, circular and linear sketch patterns (`sgCircularPattCntDim`, `sgLinearPattCntDim`, the polygon and pattern entries of
  the cadmpeg notes): present in some parts of the corpus, only counted. 33 curve records come out `Unresolved`.
* Dimensions: only the kinds listed above; the relation class of a dimension is not used (see "Dimensions"), so a distance between
  two parallel lines and a point-to-line distance are told by their operands. An angle's sense (which of the four angles between
  two lines) is not read: `ck-swedit` tries the four sign choices of the sketch angle constraint and keeps the one the geometry
  satisfies.
* Arc direction: the i32 after the curve type (1 counter-clockwise, -1 clockwise; the cadmpeg notes say the same, and it agrees
  with the face-derived profile on every sketch the rebuild matched, 104 profiles) is the sense; a record without one is refused.
* Tag classification can fail in a stream whose handle tags drift further than the spacing between curve classes (the big NIST
  files agree because the ordering rule does not depend on the drift); a failure shows as `Unresolved` curves, or as a dimension
  whose operands are `Other` (not kept).
* Relation codes 14 and 57 both mean equal; 17 is unnamed; construction-line relations are not matched.
* Which sketch an extrude uses is by adjacency (`sketch_before`); the reference record's layout is not settled. `ck-swedit` accepts
  the sketch only where it draws the profile the faces give.
* The revolve's axis reference (`moLineRef_w` after the sketch, a point, a direction) is not decoded; the axis is the line of the
  sketch that lies on the axis the faces give (below).
* Only the first configuration's stream (`Config-0-...`, else the lowest) is read.

## Using it for the editable rebuild (`ck-swedit`)

`crates/ck-swedit/src/sketch_profile.rs`. `profile::extrude` reads the profile from the faces as before. It also places the file's
sketch (the nearest sketch before the feature in the stream; every non-construction curve resolved) in the frame of the profile:
with the plane frame of the stream (`Sketch::frame`) where there is one, else by *matching*: of the eight placements of the
sketch's axes along the frame's (four quarter turns, each also mirrored) with the origin where the model origin projects, or else
with a vertex of the sketch on a vertex of the faces' profile, the ones under which the sketch draws the faces' profile (same
length, each piece of one on a piece of the other, `same_profile`) are kept and the one that keeps most of the file's relations and
dimensions true is used. Where the sketch draws that profile with one piece for each piece of the faces', the history carries the
file's sketch (`"source": "sldprt"`): its points and curves in the frame of the profile (arc sense and centres as read), the
circle / arc / line / ellipse entities, its relations (horizontal, vertical, coincident, point on line or circle, collinear,
equal, tangent, midpoint, symmetric, same x or y) and its dimensions (distance, horizontal and vertical distance, radius,
diameter, angle, point-to-line distance) as sketch constraints, with the dimension's name (`D1`) in a `name` field. Every
constraint is solved against the drawn points before it is kept (`holds`: the sketch builds as a solver system and each equation
is satisfied to 1e-6 mm); the ones the geometry does not satisfy are dropped, so the carried sketch is the drawn one and a changed
dimension moves the geometry. The origin reference becomes the fixed point `O`. The faces' sides are named after the entity of the
file's sketch that lies under them (`F<n>.S.<entity>`), as before for the entities drawn from the faces. The faces' profile stays
the fallback and is what the rest of the read-back sees; a sketch that does not draw it, or cannot be placed, changes nothing.
`rebuild_history` reads the history with the file's sketches and, only when that outcome is not exact or close, once more with the
sketches drawn from the faces (`sketch_profile::prefer`), and keeps the better outcome: never worse by construction, at the cost of
a second read of the files that do not rebuild whole (3 of the 42 imported files).

A revolve takes the file's sketch the same way (`revolve.rs`): the profile the faces give is compared with the sketch's loops, and
the axis is the line of the sketch that lies on the axis the faces give (the profile's own edge on the axis, or a construction
line), else a construction line is added there; the boundary guard is the one of every step.

Corpus (`cargo test --release -p ck-swedit --test corpus`, the 63 files of `corpus-local`, 38 of them parts): before and after
are identical: exact 34, close 5, fallback 3 (editable prefix kept for all 3: 36 rebuilt steps kept of 58 read back, as before),
not imported 21 (the downloaded fixtures with no Parasolid). In the histories of the files that have a sketch (200 sketches in all,
the starting body and prefixes included): 67 are the file's own sketches (62 in the 38 part files: the rest are the copies of the
committed fixtures), 36 of those with at least one dimension (all of them in the part files), 73 dimensions and 216 relations written. Of the extrude steps where the faces gave a profile, the file's sketch
drew the same profile in 104 and was not usable in 104: (reasons, `SKETCH_DEBUG=1` prints each) the plane frame is not in the
stream and no placement draws the profile (sketches whose profile is not the sketch before the feature, a sketch that holds more
than the feature's profile, arcs and circles whose centre is not certain, outlines that do not close because a record is not
read), text or offset curves. Revolves: 7 of the 11 revolve steps take the file's sketch (the others have a sketch the reader
cannot settle, the sphere's, or a profile the faces draw differently).

End specifications (the cadmpeg notes' table; `ck-swedit` corpus test "END SPECS" gives the word against the rebuilt step, over
the 101 rebuilt extrusion steps that have one): the end type is shown in each step's note, with where the file's sketch plane
lies on the span the faces give.

| word | meaning | steps | what the corpus shows |
|---|---|---|---|
| 0 | blind | 78 | a depth is listed in all 78; the sketch plane is at an end of the span in 8 of the 9 steps where it is known (and in the middle once: a boss whose sketch lies on the mid plane of a symmetric part) |
| 1 | through all | 11 | no depth listed in all 11; 7 of the 11 read as through cuts (the others end on a face a later feature made) |
| 2 | up to next | 2 | no depth listed; read as blind |
| 4 | up to surface | 7 | no depth listed; read as blind (3) or through (3), one with the sketch plane outside the span |
| 5 | offset from surface | 2 | a depth is listed (the offset); read as blind |
| 6 | mid plane | 1 | a depth is listed and the file's sketch plane is at the middle of the span of the faces: verified; the boss is written blind from its low end, not as a mid plane extrude (the sketch of that step is not the one-to-one match the history takes, so a mid plane boss was not exercised) |
| 3, 7, 9 | up to vertex, up to body, through all both ways | 0 | not in the corpus: unverified |

The end type is only a note: the end of every rebuilt step still comes from the faces and the boolean guard. 0 and 1 are as the
cadmpeg notes say (end type at +16 of `moEndSpec_c`; types 0, 5, 6 and 9 are 24 bytes long); the names of the other numbers are
SolidWorks' public enumeration, consistent with whether a depth is listed.

