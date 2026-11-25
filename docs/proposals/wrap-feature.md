# Proposal: `wrap` (lettering and shapes that follow a curved face) and `projectCurve` `method: "wrap"`

Status: design only, no code. Written against kernel commit 0ab2e9ae. Behaviour reference: the Emboss / Wrap section of a
commercial CAD's user guide (rules below are behaviour only; nothing is portable).

## 1. Problem and what exists

* `text` is a sketch entity on a plane (`docs/text.md`, `ck-text` + `ck-model/src/sketch/text.rs`). Its outlines become closed
  `Region`s (`sketch/profile.rs`: `Seg` = line / circle / ellipse / interpolating spline / B-spline) that `extrude` add / cut uses.
  `docs/text.md` Limits: "No text on a curved face: project or wrap is not built". An extrude of letters onto a mug goes straight,
  so the letters stand off the wall or stop short of it.
* `projectCurve` (`features/project_curve.rs`, `docs/project-curve.md`) casts a sketch along the plane normal onto faces and fits
  one B-spline. It distorts (circumferential lengths shrink with the cosine of the angle to the view) and gives a curve only.
* `knurlBand` (`features/knurl_band.rs`) is the precedent for "write a tool body by hand in ck-topo, one `br.combine` cut";
  `extrude` is the precedent for profile -> tool -> boolean and for face naming.

Goal: lay profiles on the face by length along the surface (letters keep widths, walls stand square to the face, raised or cut
along face normals) for closed profiles (solid result) and open curves (a 3D curve on the face, usable as a `sweep` `path` /
`guides`, `plane` mode `curve`: every `projectCurve` consumer already takes it).

## 2. Wire shape

Two entries, in the existing conventions: payload fields in the feature object, numbers may be `"=expr"` strings, errors are
`{code, params}` with `detail` / `hint`, warnings go to the regen reply's `warnings[]` as `{feature, error:{code, params}}`.

### 2.1 Solid: `wrap`

```json
{"type":"wrap","id":"F9","sketch":"F8","profiles":["t1"],"face":"F1.S.e3","op":"cut","depth":0.4,
 "method":"auto","alongAxis":0,"around":0,"rotation":0,"taper":0,"flip":false,
 "direction":null,"target":null,"tolerance":0.001}
```

| field | meaning |
|---|---|
| `sketch` | sketch feature id (text entities, lines, arcs, splines; any closed region) |
| `profiles` | optional list of profile ids as the sketch `profiles` reply names them (a region id, or a text entity id = all its glyphs); default all closed regions. Open curves in the sketch are ignored (they belong to `projectCurve`) |
| `face` | one face name. Its surface kind picks the algorithm. Plane: `wrapFaceUnsupported` with the hint "use extrude" |
| `op` | `add` (emboss) or `cut` (engrave). `newBody` is `invalidValue` as for the other non-extrude features |
| `depth` | height / depth along the face normals, mm, > 0 |
| `method` | `auto` (default: `wrap` on cylinder and cone, `project` otherwise), `wrap`, `project`. `wrap` on sphere / torus / free-form is `invalidValue {field:"method", detail, hint}` (not developable) |
| `alongAxis` | mm along the face axis (wrap); + = towards the world direction of the axis with the largest positive component |
| `around` | degrees about the axis (cone: true angle about the cone axis) |
| `rotation` | degrees about the centre of the selected profiles, on the face (applied in the development, before the offsets) |
| `taper` | degrees, -45..45, default 0; + widens the profile as it rises, - narrows. 0 = walls along face normals |
| `flip` | reads the sketch from the other side (mirror along the circumference); default reads it from the side the sketch normal points to |
| `direction` | `project` only: `[x,y,z]` or an edge / axis / plane / face reference as other features take; default the sketch plane normal |
| `target` | as `extrude`; default the body owning `face` |
| `tolerance` | mm, > 0, default 0.001: bound for fitted edges (3.4) |

Result: the body, as `extrude`. Faces: walls `<id>.S.<region>.<edge>`, floor (cut) / top (add) cap `<id>.C0`. `markers[] = [id,
{profiles, turns, deviation, method, surface}]` (`turns` = widest profile extent / circumference, `deviation` = worst fit
deviation in mm, `surface` = `cylinder|cone|sphere|torus|freeform`).

Errors (each with `profile` = id of the offending profile where one applies, plus `detail` / `hint`):

| code | when | params |
|---|---|---|
| `wrapNoInput` | no `sketch`, or no closed region (only open curves: hint "use projectCurve method wrap") | `hint` |
| `missingReference` / `missingSketch` | unknown face / sketch (existing codes) | as existing |
| `wrapFaceUnsupported` | planar face, procedural sweep / blend face we cannot offset | `{faceKind, hint}` |
| `wrapPlane` | sketch plane perpendicular to the axis (no axis direction in the plane) | `{detail, hint}` |
| `wrapProfileTooWide` | one profile, or the whole selection, spans a full turn or more | `{profile, extent, circumference, hint}` (shorten the text, reduce its height, or split it over two wraps) |
| `wrapProfileOffFace` | a profile falls off the trimmed face, crosses a hole boundary, or on a cone reaches the apex or the far end | `{profile, at:[x,y,z], hint}` |
| `wrapProfilesOverlap` | two profiles overlap after wrapping (cone near the apex, `rotation`) | `{profiles:[a,b]}` |
| `wrapDepthTooLarge` | `add` in a bore with depth >= radius | `{depth, max, hint}` (a cut through a wall is a warning, not this) |
| `wrapTaperTooLarge` | the tapered top outline self-intersects or loses a counter | `{profile, maxTaper, hint}` |
| `wrapToolInvalid` | the tool failed its closed / manifold / positive-volume check | `{detail}` |
| boolean codes | `booleanFailed` etc. unchanged; on any failure the body is left as it was | |

Warnings: `wrapApproximated {maxDeviation, tolerance}` (a fit hit the 400 control point cap), `wrapCutThrough {profile, wall}`
(engraving deeper than the wall there: pierces, result valid), plus the existing `cutNoEffect`. No silent fallbacks: `auto`
only chooses a method up front.

### 2.2 Curve: `projectCurve` `method: "wrap"`

Same payload as `projectCurve` plus `method` (`project` default = today, unchanged; `wrap`), `alongAxis`, `around`, `rotation`,
`flip` (as 2.1), `entities[]` (optional subset; default every non-construction entity, open or closed). `faces[]` must name exactly one
cylindrical or conical face (`wrapFaceUnsupported` otherwise; phase 2 builds cylinders, a cone is `wrapFaceUnsupported` until phase 3). Only a
`sketch` input is wrapped (a `curve` input is `wrapNoInput`; `procedural: true` is `invalidValue`); `direction` is not read. The result is the same `curves[]` / `markers[]` entry `projectCurve` writes (all
consumers in `docs/project-curve.md` unchanged); the marker adds `turns`. Differences from the solid: a curve may run past one
turn (a helical groove path is the point; in 3D there is no seam problem); limit is the fit's 400 control points, past that
`wrapCurveTooLong {length, maxControlPoints, deviation, tolerance, hint}` (decided phase 2: the length that fits depends on curvature, so the
params report the fit that failed instead of a length bound). A `wrapProfileOffFace` for a curve also carries `entity` (the entity id). Disjoint entities give the existing `projectCurveSeveral`. An end off the trimmed face is
`wrapProfileOffFace` (projectCurve drops the part off the face; a wrap must not, the length would lie). Why not a new type: the
consumers, `curves[]` bookkeeping and the picker already live under `projectCurve`, and the GUI offers Project / Wrap as a toggle.

## 3. Algorithm

### 3.1 Common: the development map

For a developable host (cylinder, cone) build the isometric map `D` from sketch `(x, y)` to `(s, t)` on the face, then to 3D.

1. Anchor and reading frame. Plane parallel to the axis: the anchor is the point of the face nearest the plane. Plane through the
   axis: the side its normal points to. `e_z` = the axis direction projected into the sketch plane (sign rule of `alongAxis`),
   `e_s` = the other in-plane direction; sketch coordinates are re-expressed in `(e_s, e_z)` about the sketch origin. Axis
   perpendicular to the plane: `wrapPlane`. `flip` mirrors `s`.
2. `rotation` about the centre of the selected profiles (2D), then `alongAxis` (z) and `around` (s at the reference radius).
3. Orientation: `(e_s, e_z, n_face)` is right-handed like the sketch's `(x, y, normal)`: the sketch reads the same from the side of
   the face normal as from the sketch plane normal. In a bore the face normal points into the hole, so letters read correctly
   from inside and `add` raises towards the axis.

### 3.2 Cylinder (radius r, axis a)

`D(s, z) = o + z a + r (cos th, sin th)`, `th = th0 + s / r`. Arc length on the face is preserved (widths kept: this is the rule, not
a projection). At radius `r + t` the same `(th, z)` is used: walls are radial, "square to the face". One-turn test: the
`s`-extent of each profile and of the whole selection must be < `2 pi r (1 - 1e-6)`. Seam: build the tool on a fresh cylinder
frame whose seam sits at `th0 + pi`, so no loop crosses it; the booleans meet the host's own seam as today
(`tests/fillet_cylinder_seam.rs`). Bore: sense reversed; a cut of depth d goes to `r + d`, an add to `r - h`.

### 3.3 Cone (half angle alpha, apex P0, axis a)

Isometric development: slant distance `rho` and angle `th` map to polar `(rho, th sin(alpha))`. Sketch in the frame of 3.1 with
the anchor at slant `rho0` is rolled on: `rho = hypot(x, rho0 + y)`, `psi = atan2(x, rho0 + y)`, `th = th0 + psi / sin(alpha)`.
Lengths, angles and areas are the sketch's. A profile reaching `rho <= 0` (apex) or past the sector (`2 pi sin(alpha)`) is
`wrapProfileOffFace` / `wrapProfileTooWide`. A countersink is the same code with reversed sense. Heights go along the cone normal
`n(th)` (constant along a ruling); the offset cone is the same axis with the apex moved by `h / sin(alpha)` (exact `Surface::Cone`).

### 3.4 Curved profile edges: exact vs fitted

Exact cases (no fit, no tolerance):
* cylinder: an edge parallel to the axis -> a ruling, wall a radial plane (`Surface::Plane`); an edge with `z` const -> a circle, wall
  the plane `z = const` (annular sector). A sketch circle or tilted line is NOT a circle / line on the face: fitted.
* cone: an edge along a ruling -> plane wall through the axis; an edge with `rho` const -> a circle.

Everything else (splines, text outlines, arcs, tilted lines):
1. adaptive sampling of the 2D edge until the chord error of the mapped 3D curve is below `tolerance / 4`;
2. cubic B-spline by `ck_geom::fit::approximate_tol` (what `curve_edit` uses), bound `tolerance`, at most 400 control points (then
   `wrapApproximated`); profile corners (tangent break > 1 degree) split the loop into edges so a fit never rounds a corner;
3. base rail (`R_ref`) and top rail (`R_ref + h`, on a cone `+ h n(th)`) share parameterisation and knots, so corresponding points
   lie on one normal; the wall is the degree (1, 3) ruled `Surface::BSpline` between them (exact up to the rail fit);
4. verify: sample the wall and measure the distance to `D` of the exact edge at the right height; above `tolerance`: tighten
   once, then `wrapApproximated`. The reported `deviation` is that distance.

Caps are exact analytic cylinders / cones trimmed by the loops; the loop B-splines lie on them to the fit tolerance, the same
situation as extruded text edges against a host face today (the boolean welding tolerance already falls back 1x / 10x / 100x).

### 3.5 Building the tool and the booleans

Per region: outer loop + hole loops (`Region.holes`, letters with counters). Tool = closed solid: base cap at `R_base`, top cap at
`R_top` (both trimmed by same-topology loops) and the walls. `cut`: `R_base = r - depth`, `R_top = r + eps`; `add`: `R_base = r - eps`,
`R_top = r + depth`; `eps = max(0.05 depth, 5 x boolean tol)` so the tool crosses the host transversally and never has a face
coincident with it (the weak case in `docs/boolean-corpus.md`). In a bore the radial signs swap. All regions are united into ONE tool
body (disjoint lumps), one boolean per feature, as `knurlBand`.

Taper `phi`: the top rail is the 2D offset of the profile by `h tan(phi)` in the development (`sketch/offset.rs`, as `thin`), mapped at
the top radius, correspondence by closest point; `eps` continues the slope. A top offset that loses a loop or self-intersects is
`wrapTaperTooLarge`, never a bad solid.

Checks as `knurlBand`: tool closed, manifold, positive volume (`wrapToolInvalid`); `br.combine(&tool, Add|Cut, true)`; afterwards no
more validity issues than before and the lump count kept, else restore the body and fail.

### 3.6 Project method (sphere, torus, free-form; or cylinder / cone with `method: "project"`)

1. Cast each sampled loop point along `direction` (default the sketch plane normal). The rays and the trimmed-face inside test are
   `project_curve.rs`'s `Caster` / `LoopCache` (promote to `pub(crate)`). Two hits (a ball): the one nearest in front of the sketch,
   else nearest behind. A sample with no hit, outside the trimmed face, or past the silhouette: `wrapProfileOffFace {profile, at}`.
2. Base rail = fitted B-spline of the projected samples (3.4 steps 1-2). Top rail = each base sample moved by `h` along the face
   normal there, same parameterisation; wall = ruled surface (each ruling is a normal).
3. Caps: the host surface offset by `-eps` / `-depth` and `+depth` / `+eps`. Sphere (`r +- t`) and torus (`minor +- t`) are exact
   analytic; free-form uses the offset machinery of `ck-ops/surfacing/thicken/freeform.rs` or `Surface::Offset`. Whether the booleans
   accept `Surface::Offset` faces was not checked here: the largest open question, resolved at the start of phase 4.
4. Taper as 3.5, in the tangent plane at the profile centre.

### 3.7 Open curves (2.2)

Map each entity through `D` (adaptive sampling as 3.4 step 1), join runs as `chain_up` does, feed `curve_edit::edit_curve`
(`by_tolerance`, `keep_corners`) exactly as `projectCurve` does (reuse `PieceIn`, `EditOpts`, the marker writer; only the producer of
runs changes). Exact lines and circles are still written as the fitted B-spline (one code path; every consumer takes B-splines).
A `sweep` path takes a spline through at most 120 display points, so a long multi-turn path is coarse: separate follow-up if the
corpus shows deviation above 0.01.

## 4. What is reused

* ck-text outlines -> `Region`s (unchanged; `wrap` reads the sketch's `Profiles` as `extrude` does; `Seg` / `SegGeom` give exact
  line / arc / spline parameters to sample).
* `features/project_curve.rs`: `Caster`, `LoopCache`, `chain_up`, the fit and marker writer, `curves[]` bookkeeping.
* `ck-ops::curve_edit::edit_curve`, `ck_geom::fit::approximate_tol`, `sketch/offset.rs`.
* ck-topo `Body` builders as in `knurl_band.rs` (`add_face`, `add_loop`, `add_coedge` with pcurves), `Surface::{Cylinder, Cone,
  Sphere, Torus, BSpline, Plane}`, `FaceName`.
* `Brep::combine`, `ck_check::validate`, `bodies::check_feature` / `target`, `known_fields.rs`, the regen feature-type list and
  `EMPTY_FEATURE_ERRORS` (`wrap`: `wrapNoInput`).
* New: the map `D` (cylinder, cone) and its inverse for the off-face test; `features/wrap.rs` (payload, checks, errors);
  `features/wrap_tool.rs` (rails, ruled walls, caps, loop assembly); the `method: "wrap"` runs producer in `project_curve.rs`.

## 5. Tests

In `crates/ck-model/tests/wrap.rs`, plus wasm corpus `tools/wrap_corpus.mjs` (run by `tools/remote_build.sh --checks wrap`).

1. Text on a mug-like cylinder (r 40, wall 3, height 90, bore r 37): `wrap` cut of "CADKIT 1/2", 6 mm cap height, depth 0.4, fonts
   `gauge` and `engrave`; add of the same, depth 0.5. Checks: `ck_check::validate` clean and `validate {mesh:true}` watertight; no
   `booleanFailed`; volume change equals the integral of the glyph area over the radial shell (computed in the test from the
   `textOutline` loops: area x depth x (1 + depth / 2r) for the outward add) within 1.5 %; each glyph's `s` extent on the face equals the
   sketch glyph width within `tolerance`; STEP export -> import: volume within 1e-4, face count equal.
2. Rectangle label (axial and circumferential edges only): all walls planes, `deviation == 0`, exact volume.
3. Open curves: sketch spline, line, arc drawn flat -> `projectCurve method:"wrap"` on a cylinder: curve length equals the sketch
   length within `tolerance`, every sampled point at distance r from the axis within `tolerance`; a 3-turn line equals a `helix`; a
   `sweep` of a circle along it is valid.
4. One-turn limit: text of circumference + 1 mm -> `wrapProfileTooWide` naming the profile; 0.999 turn builds with the seam clear;
   `wrapProfileOffFace` for a profile past the cylinder end or over a through hole; `wrapPlane` for a plane perpendicular to the axis.
5. Bore: engrave and emboss inside a bore; reading direction checked with an asymmetric glyph ("R").
6. Offsets: `alongAxis` / `around` / `rotation` equal moving the sketch by hand (volume and centroid within tolerance); `=expr` accepted.
7. Taper +-10 degrees: volume between the analytic bounds; too large on a counter -> `wrapTaperTooLarge`.
8. Cone (countersink, frustum): lengths equal the sketch, apex refusal. Then sphere, torus, B-spline face with `project`: an off-face
   profile named, top cap offset = depth.
9. Persistence: regen cache key, `target`, `.ckpart` round trip, `lint_protocol` mutation fuzz of every field.
10. Crash fuzz seeds extended with `wrap`; `docs/crash-fuzz.md` baseline stays clean.

## 6. Phase plan

| phase | content | size |
|---|---|---|
| 1 | `wrap` on a cylinder (outside and bore), closed profiles, cut + add, offsets, one-turn / off-face refusals, exact walls for axial / circumferential edges, fitted for the rest, tool check; tests 1, 2, 4, 5, 6, wasm corpus | ~1500 lines, 5-7 days |
| 2 (built) | `projectCurve method:"wrap"` on cylinders (open curves, multi-turn), consumers checked; test 3 | ~400 lines, 2 days |
| 3 | cone development (+ countersink), taper; tests 7, 8 (cone) | ~600 lines, 3 days |
| 4 | `project` method for sphere / torus / free-form (after the offset-cap decision), `wrapProfileOffFace` names | ~800 lines, 4 days |
| 5 | docs (`docs/text.md` Limits line replaced, `spec-agent-interface.md`), corpus into the quick set, crash fuzz | ~2 days |

The GUI can start against phase 1's payload: the fields of 2.1 are final for cylinders; `direction` and `method: "project"` are inert
until phase 4.

## 7. Risks and open questions

* Booleans with B-spline walls trimmed against a curved host: the extruded-text precedent (spline walls, planar host) is solid; this is
  new. Walls are radial (transversal to the host), the benign case; keep `eps` above the boolean tolerance. Mitigation: corpus of text
  on three radii (10 / 40 / 150 mm) x three heights (2 / 6 / 20 mm) with fuzz seeds; the existing robust retry (`bool_retry`) applies.
* Small radius, large letters: radial walls of an `add` are wider at the top than the base; strokes may self-touch at the inner rail
  (counter of "e" on a 3 mm radius): `wrapToolInvalid`, never a bad body. Test radius 3 with 2 mm text.
* Engraving deeper than a thin wall: valid result, `wrapCutThrough`.
* Cone apex refused; sphere poles by the `apply::unpole` rotation if a projected loop contains a pole (phase 4).
* `Surface::Offset` in `ck-ops::boolean` unchecked: if unsupported, free-form caps are fitted B-spline sheets from a grid of offset
  samples (tolerance as 3.4), slower; sphere / torus keep exact analytic offsets either way.
* `sweep` path limit of 120 display points (3.7).
* `tolerance` default 0.001 mm matches `projectCurve`; the internal rail tolerance is a quarter of it so wall plus cap stay inside.
* Side and sign rules of 3.1 (which side of a bore, the `alongAxis` sign on a flat-lying axis) are the least obvious UX; fixed here
  so kit and kernel agree, covered by tests 5 and 6.
