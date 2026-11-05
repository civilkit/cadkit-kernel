# Agent queries: check, resolve, describe, refDiff, section, sections, probe, distance, thickness and the measurement set

Kernel-side pieces of `spec-agent-interface.md` (ADR 0002 items 3-4, ADR 0003
item 5). New request types, extensions to the 21 existing ones (which are
unchanged). Code: `crates/ck-ops/src/agent.rs` (queries),
`crates/ck-wasm/src/refdiff.rs` (reference diff). Tests:
`crates/ck-wasm/tests/agent_iface.rs`, `tools/agent_iface.mjs` (gate metrics
`iface.*`, soft), `tools/lint_protocol.mjs`.

`check`, `resolve`, `describe` take `tag` (default `main`) and answer `null`
on a tag with no body, like the other body queries.

## check `{expect}`

`expect` is an array of `{kind, equals | min/max, tol?, relTol?, id?}` or the
shorthand object `{volume: 1000, faces: 7, edges: {min: 10}}`.

| kind | measured | notes |
|---|---|---|
| `volume`, `area` | massProps | default tolerance 1e-4 relative |
| `mass` | volume x `density` | `density` required |
| `bbox` | min / max / size | give any of `min`,`max`,`size` as `[x,y,z]`; `tol` absolute (default 1e-4 of the diagonal). Exact for line and circle edges; free-form faces use the display mesh |
| `com` | centre of mass | `at:[x,y,z]`, `tol` |
| `faces`, `edges`, `vertices`, `solids` | counts | `edges` = wire edges (seam-split arcs count once) |
| `holes` | concave cylindrical faces bounded by full circles (through and blind holes) | |
| `innerLoops` | inner loops over all faces | |
| `valid` | validator issue count must be 0 | `tol` for the validator |

Reply: `{ok, passed, failed, results:[{index, id?, kind, pass, measured, expected, detail}], measured:{...all}}`.
An unusable expectation (unknown kind, missing value) fails with an explanatory `detail`.

## resolve `{descriptions:[..]}` (or `description`)

Reply per description: `{description, status: unique|ambiguous|none|invalid, ambiguous, count, matches:[..], narrowing:[{filter, remaining}], hint?, error?, resolvedBy?}`.
Each match is the `describe` record of the entity, including a ready-to-store
`name` and `ref` (`{kind:"face",id}` or the `{faces,at}` edge ref used by
fillet/chamfer). More than one match is never collapsed: status is
`ambiguous` with all candidates. Unknown fields or bad values are `invalid`
(never silently ignored).

### Description grammar

An object with `kind` (`"face"` | `"edge"`) and any of the filters below; all
given filters must hold. Direction values: `"x"` `"y"` `"z"` (sign-insensitive),
`"+x"` `"-z"` (signed) or `[x,y,z]` (signed); matched to about 0.08 degrees.
Number values: `5` (relative 1e-6), `{value,tol}` or `{min,max}`.

| field | face | edge | meaning |
|---|---|---|---|
| `surface` | yes | | `plane cylinder cone sphere torus spline revolution extrusion offset` (string or list) |
| `curve` | | yes | `line circle other` |
| `normal` | yes | | outward normal (planes only) |
| `axis` | yes | | axis of cylinder / cone / torus / revolution |
| `dir` | | yes | direction of a line edge |
| `radius` | yes | yes | cylinder / sphere / torus minor; circle edge |
| `area`, `length` | `area` | `length` | |
| `role` | yes | | `cap side blend other` (from the persistent name) |
| `entity` | yes | | sketch entity of a side face (`e1`) |
| `concave`, `hole` | yes | | cylinder with material outside; hole barrel |
| `feature` | yes | yes | feature id (or list) that made the face / that owns an adjacent face |
| `name` | yes | yes | exact name, or prefix with trailing `*` (`"F2.S.*"`) |
| `touches` | yes | yes | adjacent to / bounded by this face name |
| `body` | yes | yes | name (or list) of the body (lump) the face / edge is in; bodies that overlap each have their own coincident faces, this tells them apart |
| `between` | | yes | `[faceA, faceB]` |
| `extreme` + `along` | yes | yes | `"max"`/`"min"` of the face centroid / edge midpoint along `x|y|z` among the matches so far; `along` defaults to the axis of `normal`/`axis`/`dir` |
| `near` (+`within`) | yes | yes | `[x,y,z]` tie-break: keeps the single nearest candidate only when it is strictly nearest (reported as `resolvedBy:"near"`); `within` is a hard radius filter |

Example: `{kind:"face", surface:"plane", normal:"+z", extreme:"max", feature:"F3"}`.
Filters apply in a fixed order (feature, name, touches, radius, shape fields,
extreme, near), so `extreme` picks among the already filtered faces.

## describe `{names:[..]}`

Each name is a face id, an edge key (`A|B#seq`, `body.edges[].key`), `{kind:"face",id}`
or an edge ref `{faces:[a,b],at}`. Reply `{items:[..]}` with `found`, the facts
(`surface`/`curve`, `normal`/`axis`/`dir`, `radius`, `area`/`length`, `centroid`/`mid`,
`feature`, `role`, `adjacent`, `outermost` = body-bbox side a planar face lies on)
and a one-line `text`. A name that no longer exists returns `found:false`,
`error` and `nearest` candidate names (stale reference feedback, ADR 0002).

## refDiff `{base, part}` (regen warning)

Stateless: regenerates the previous document (`base`) and the edited one
(`part`) privately, records the entity every face id / `{faces,at}` edge
reference resolves to at the feature that uses it, and compares each
reference present in both with the rules of `tools/naming_stability.mjs`
(surface kind, direction, neighbour kinds, size ratio 0.25..4, position).
Reply `{ok, checked, warnings:[{code, feature, ref, reason, before, after}], features:{removed, added, reordered}, errors}`;
`code` is `referenceChanged` (re-pointed to a different entity) or
`referenceLost` (no longer resolves). Ordinary parameter edits do not warn.
The `regen` reply is deliberately unchanged (wire-pinned); a front end calls
`refDiff` with the pre-edit document when applying edit, delete or move ops.

## section `{tag?, plane:{origin, normal}}` (verification)

Read-only. The cross-section of the body (or tagged state) at a plane. Reply
`{loops:[{kind:"outer"|"inner", closed:true, area, edges:[...], pts:[[x,y]..]}], frame:{o,x,y,z}, area, bbox2d:[minx,miny,maxx,maxy]}`;
coordinates are in the plane frame (`z` = the normalised normal, `x` = Z cross normal, or world X when the normal is
within 2.5 degrees of Z). Loops run counter-clockwise around material (`outer`, positive area) and clockwise around
holes (`inner`); `area` is the signed sum, i.e. outer minus inner; each loop's `area` is positive. Edge records:
`{type:"line", from, to, face}`, `{type:"arc", centre, radius, start, end, startAngle, sweep (signed), ccw, face}`,
`{type:"polyline", pts, face}`; `face` is the persistent face id. Several lumps give several outer loops. A plane that
misses the body gives `loops:[]`, `area:0` (not an error). No body: `null`. A plane exactly through a planar face
treats on-plane vertices as lying on the positive side, so it returns that face outline for one orientation and nothing
for the other.

How exact: the tessellation is cut by the plane, chained into loops, every loop vertex is Newton-snapped onto the exact
surfaces meeting there, and curved segments are bisected until the sagitta is below 2e-5 of the body diagonal. Planar
faces give exact `line`s; a run of one curved face lying on one circle (cylinder / cone / torus cut perpendicular to the
axis, any sphere cut) becomes an exact `arc` and its area uses the exact circular segment; everything else (oblique cuts
of cylinders and cones, tori through the axis, spline and swept surfaces) is a `polyline` on the exact surface with the
sagitta bound. Loops with area below 1e-9 of the squared diagonal (tangent slivers) are dropped. `openChains` appears
when the mesh left an unclosed chain (a face that failed to tessellate); those pieces are omitted.

## sections `{tag?, axis:"x"|"y"|"z"|[nx,ny,nz], n | positions}` (verification)

Read-only. A stack of sections along an axis through the origin direction: `n` cell centres (default 21) between the
mesh extent of the body along the axis, or explicit `positions` (signed distance along the unit axis; at most 1000).
Reply `{axis, range:[lo,hi], rule, positions, areas, loops:[count per position], volume, exactVolume, volumeError}`;
`exact:false` skips the `massProps` computation (seconds on a large body) and omits `exactVolume` / `volumeError`.
`volume` is the midpoint rule over cell centres (rule `midpoint`) or the trapezoid rule over explicit positions
(`trapezoid`, covering only the span between the first and last); `exactVolume` is `massProps` and `volumeError` the
relative difference, an independent cross-check of the volume and an area profile. The integration error of the rule
itself is about 1e-3 at n=41 for smooth profiles (box 1e-9, cylinder along its axis 1e-9, sphere 3e-4); a profile with
steps (large sample: head / handle junction) needs about 800 sections for 1e-4, while the section areas themselves stay
exact; use more sections for a tighter bound.

## probe `{tag?, points?:[[x,y,z]], rays?:[{origin, dir}]}` (verification)

Read-only; at most 2000 points and 2000 rays per request (an error beyond). `points` reply
`[{inside:true|false|"boundary", distance, face, nearest:[x,y,z]}]`: `distance` is the signed distance to the nearest face
(negative inside, 0 on the boundary), `face` its persistent id, `nearest` the foot point (face interiors and boundary
edges are both searched). `rays` reply `[{hits:[{t, point, face, entering, tangent?}]}]` sorted by `t` (direction is
normalised, `t` is the distance); `entering` is true when the ray goes from outside to inside; a hit on a shared edge
is reported once; a ray lying in a face skips that face; `tangent:true` marks grazing hits. Containment uses the
existing seven-ray parity classifier of `ck-ops::contain`.

## Measurement and analysis (read-only)

`distance`, `maxDistance`, `thickness`, `thicknessAt`, `closestPoint`, `angles`, `clearance`, `symmetry`, `extrema`
(`crates/ck-ops/src/analysis.rs`; tests: its unit tests, `crates/ck-wasm/tests/measure.rs`, `tools/measure_check.mjs`,
`tools/measure_occt_check.py`). All take `tag` (default `main`), answer `null` on a tag with no body, return
`{ok:false,error}` with the field named for bad input, and never change the body. Options may sit in `opts` or at the
top level of the request.

**Entities** (`a`, `b`, `ids`): a face id as in `describe` (a duplicated id selects every face with it), an edge key
as in `describe` (`F2.C1|F2.S.e1#3`) or `e<n>`, a vertex `V<n>`, a point `[x,y,z]`, `lump:<n>` (the faces of one
lump), `"body"`, an array of any of these, or an object `{face|edge|vertex|lump|point|faces|edges}`.
Replies name the face / edge / vertex a point lies on as `{kind, id}` (`point` has no id).

### distance `{a, b, tagB?, opts?:{samples}}`

Minimum distance of two entities. `tagB` takes `b` from the body of another tag (a rebuilt part against the original).

```
-> {"type":"distance","tag":"main","a":"lump:0","b":"lump:1"}
<- {"distance":5,"pointA":[10,0,-5],"pointB":[15,0,-5],"a":{"kind":"face","id":"F2.C2"},"b":{"kind":"face","id":"F3.C0"},
    "intersects":false,"direction":[1,0,0],"aLabel":"lump:0","bLabel":"lump:1"}
```

`intersects:true` with `distance:0` and `intersection:{kind:"point"|"curve"|"area"}` when the entities touch (the
extent is the share of A's samples within 1e-7 of B); `kind:"containment"` (with `boundaryDistance`) when, across two
tags, one solid lies inside the other. Method: exact point-to-face / point-to-edge projections (the surface's own
closest point, trimmed by the face loops), closed form for two line segments, and for everything else a multi-start
alternating projection (sample A and B, best four starts, project back and forth until the gain is under 1e-15
relative, 400 iterations at most). Sets of faces are pruned by box distance. Pairs whose minimum lies in the
interior of both faces (sphere / sphere, cylinder / cylinder) are found from the grid starts: grids are
`opts.samples` x `opts.samples` per face (default 7).

### maxDistance `{a?, b?, tagB?, opts?:{samples, both}}`

One-sided Hausdorff distance: the maximum over the surface of `a` (default whole body) of the minimum distance to `b`
(default whole body, `tagB` for another tag). Samples a grid inside every face plus its edges (`opts.samples`, default
3000 over the body, visited in a stride order so a cut run still covers every face). Reply `{maxDistance, location, nearest, errorBound, upperBound, rms, samples, coverage, face}`: the
distance function is 1-Lipschitz, so the true maximum lies in `[maxDistance, maxDistance + errorBound]` where
`errorBound` is the covering radius of the sample grid. `both:true` adds the other direction:
`{hausdorff, errorBound, aToB, bToA}`.

### thickness `{opts?:{samples, top, radius, below, method:"ray"|"sphere", sphereSamples}}`

Wall thickness of a solid. Each face is sampled on a uv grid (about `samples` in total, default 2500); from each
sample a ray goes along the inward normal and the first exit (`contain.rs` ray hits, `entering:false`) gives the
thickness there. `method:"sphere"` refines the thinnest `sphereSamples` (default 200) with the largest inscribed ball
tangent at the sample (thickness = 2 r, never above the ray value; needs more time). Reply
`{method, samples, noExit, minThickness, location:{thickness, point, farPoint, face, farFace, direction},
meanThickness, clusterRadius, thin:[...]}`; `thin` lists the `top` (default 10) thinnest places, thinnest first,
clustered greedily so that two entries are at least `radius` apart (default max(2 x min, 2 % of the diagonal));
`below` drops entries thicker than it. `noExit` counts samples without an exit (open shells, ray lost).

### thicknessAt `{point}` or `{face, opts?:{samples}}`

`point`: the thickness along the inward normal at the surface point nearest to it
(`{distanceToSurface, result:{...}}`). `face`: min (with its location) / max / mean over the face.

### closestPoint `{points:[[x,y,z]], on?:"body"|"face"|"edge"|"vertex", ids?, k?}`

At most 2000 points per call. `on:"body"` (default): the nearest boundary point with what it is on
(`entity:{kind:"vertex"|"edge"|"face", id}`, `face`, the inside flag and `signedDistance`, negative inside, like
`probe`). `face` / `edge` / `vertex`: the nearest entity of that kind (restricted to `ids` if given) with `uv` (face) or
curve parameter `t` (edge). `k > 1` returns the k nearest entities per point as a list.

### angles `{edges?:"all"|[keys], faces?:[a,b], draft?:{pull, faces?, min?}}`

`edges`: per two-faced edge `{edge, faces, angle, normalAngle, type, min, max, at}`: `angle` is the interior dihedral
angle through the material in degrees (a box edge is 90, a re-entrant edge of an L is 270), `type` is `convex`,
`concave` or `smooth` (normals within 0.5 degrees), `min`/`max` over five points along the edge (curved edges and
faces vary). `faces`: `between` for two faces: with a shared edge its dihedral angle; otherwise the angle between the
normals at their closest points (`angle`, `normalAngle`, `distance`, `parallel`). `draft`: for each face (or `faces`)
the draft angle against the pull direction, `asin(n . pull)` in degrees over a 5 x 5 grid (`draft`, `min`, `max`,
`class`: `vertical` (|d| < 0.01), `undercut` (negative), `parting` (|d| > 89.99), `ok`); with `min` the walls (not
the parting faces) under that angle are listed in `violations`.

### clearance `{min, a?, b?, opts?:{include:"gap"|"wall"|"both", limit}}`

Face pairs closer than `min` (default pairs: every face of the body with every other, skipping pairs that share a
vertex since they touch by construction). `kind` says what lies between them: `gap` (air between two features),
`wall` (material between them, e.g. a hole to the part edge: the same number `thickness` reports) or `contact`.
Reply `{min, count, pairsTested, truncated, pairs:[{a, b, distance, kind, pointA, pointB}]}` sorted by distance, at
most `limit` (default 200) listed. Box-pruned, at most 4000 faces per side.

### symmetry `{planes?:[{origin, normal}], opts?:{samples, tol}}`

Candidate mirror planes: the coordinate axes and the principal axes through the centre of mass (and the coordinate
axes through the box centre), or the given `planes`. For each, the surface is sampled (`samples`, default 600),
mirrored and measured against the body: `maxDeviation` and `rmsDeviation` of the mirrored surface from the original
(the mirror is an involution, so one direction is the whole Hausdorff distance). `symmetric` lists the planes with
`maxDeviation <= tol` (default 1e-4 of the diagonal); `best` is the least deviating; `planes` has all, sorted. The
symmetric-difference volume of the booleans is not computed.

### extrema `{dir, of?, opts?:{samples}}`

Lowest and highest point of the body (or of `of`, any entity) along the direction `dir` (any length, not zero): the
extent of the part along an axis, a stock size, the topmost point. Positions are `dir . p` with `dir` normalised.

```
-> {"type":"extrema","tag":"main","dir":[1,1,1]}
<- {"direction":[0.577,0.577,0.577],
    "min":{"value":0,"point":[0,0,0],"entity":{"kind":"vertex","id":"V3"},"at":[{"kind":"vertex","id":"V3"}]},
    "max":{"value":51.96,"point":[40,30,20],"entity":{"kind":"vertex","id":"V6"},"at":[{"kind":"vertex","id":"V6"}]},
    "extent":51.96,"method":"vertices","exact":true,"samples":412,"elapsedMs":3,"truncated":false}
```

`entity` is what the extreme point lies on and `at` everything that attains it (at most 20): faces whose interior
reaches the extreme (a planar face perpendicular to `dir` is attained over its whole area; `point` is then one point of
it), else edges (a whole edge at the extreme, or a curved edge touching it in its interior), else vertices (a corner,
or the end of an edge). `of` takes the entities of `distance`; for a point `min` and `max` coincide.

How exact: every face is sampled on a `samples` x `samples` grid (default 16, at most 64) and every edge at `4 x samples`
points; for curved faces and edges the two best samples of each side are refined by a pattern search to about 1e-10 of
the cell size, inside the face and along the edge, so a smooth extreme (sphere, cylinder, torus, cone, fillets) comes
out to about 1e-9. `method:"vertices"`, `exact:true` means every face searched is planar and every edge straight: the
extreme is then a vertex or a whole edge / face and the value is exact to rounding. Otherwise `method:"sampled+refined"`,
`exact:false`: an extreme of a free-form face narrower than the grid spacing (a small bump) can be missed, raise
`opts.samples`. `samples` in the reply counts the points evaluated. Budget as below (`truncated:true`: some faces or
edges were not searched, so the extent is a lower bound).

### Budget: `elapsedMs`, `truncated`

Every measurement reply carries `elapsedMs` (wall time inside the request) and `truncated`. The requests stop their
loops after `opts.timeMs` (default 8000 ms) and answer with what they have: `truncated:true` means the result is
partial, a lower bound for a maximum (`maxDistance`, `thickness` samples, `symmetry` deviation) or an incomplete list
(`clearance`). `maxDistance` adds `coverage` (the share of its sample points that were measured). `truncated` is also set
when `clearance` finds more pairs than `limit` (`listedAll:false`) or more candidate pairs than `opts.maxPairs`
(default 4000, closest boxes first). `symmetry` stops testing a plane after a few dozen points once one is farther
than `tol` from the body: that plane has `exact:false` and its `maxDeviation` is a lower bound (`opts.full:true` measures
every sample). `clearance` skips a face pair when its display meshes are farther apart than `min` plus twice the mesh spacing (a face whose mesh failed or is incomplete can therefore hide a pair), and seeds each pair from the closest display-mesh points and refines once (at most 60
projections), so for doubly curved pairs the distance is a local minimum near the mesh minimum.

### Accuracy and timing

`tools/measure_occt_check.py` (python3.12 + OCP) compares `distance` of random face pairs of sample parts with
`BRepExtrema_DistShapeShape` on the exported STEP (faces matched through an interior point): 80 pairs on four
plane / cylinder / cone parts (mounting plate, bracket, motor bracket, pipe elbow) agree to 4e-15 (57 with nonzero
distance); 60 pairs on the large sample shell, swept handle, loft bottle and surface patch (B-spline faces) agree to
2.6e-8.

large sample part (125 faces, mostly B-spline), default options, dev machine, measured through the wasm:

| request | before this change | now (default options) |
|---|---|---|
| `thickness` | 10.2 s (4548 samples) | 3.3 s (1492 samples), min 0.069 mm |
| `maxDistance` (body to itself) | 10.3 s | 7.3 s, `truncated:false`, `coverage:1` (4487 samples) |
| `symmetry` | 131.6 s | 4.6 s, `truncated:false` (2987 samples) |
| `clearance` `min:3` | 115.9 s | 8.0 s, `truncated:true` (364 pairs refined, 259 found; list is partial) |
| `closestPoint` (3 points) | 0.24 s | 0.22 s |
| `angles` (all edges) | 0.52 s | 0.52 s |
| `distance` point to body | 55 ms | 56 ms |

`maxDistance` and `symmetry` cost about 1.5 ms per sample point (exact B-spline projections), `thickness` about 2 ms
per ray; the earlier "7 ms" figures were wrong. `clearance` costs about 25 ms per refined face pair on this part, so
the full set at `min:3` needs a longer `opts.timeMs`.

Sweep (`node tools/measure_check.mjs --sample 30`, 29 parts measured, 1 skipped for regen errors, 117 s, 0 violations): every
reply has `elapsedMs` / `truncated`; `maxDistance` of a body to itself is under 1e-6 on all of them, and `distance`
agrees with `probe` to 1e-6. Replies cut by the 8 s budget: `maxDistance` on the pan-head screw, twisted drill and
flange coupling; `clearance` on the spline hub, knuckle hinge, flange coupling, stepper motor and speedboat dash.
`tools/measure_check.mjs` runs everything on sample parts (`--sample N` for an evenly spaced subset; no trap, finite
output, `elapsedMs` / `truncated` present, sane ranges, `distance` agrees with `probe`, timings).

### Limits

- Distance is exact for the pairs where the projections are exact; the multi-start can miss a minimum that lies in
  the interior of two doubly curved faces and is far from every grid sample (raise `opts.samples`). Near-parallel
  curved pairs converge linearly (400 iterations then stop at the best seen).
- `distance` to a lump inside another lump of the same body is the boundary distance (containment is only tested
  across two tags).
- `thickness` is a sampled minimum: a feature thinner than the sample spacing between two samples can be missed
  (raise `opts.samples`; the thin list shows where the samples are densest). Edges and vertices are not sampled.
  `method:"ray"` measures along the normal, which overestimates in wedges and at corners; `sphere` does not.
- `clearance` skips faces that share a vertex, so two faces of a tight V groove are not reported.
- Only same-session tags can be compared (`tagB`).

## fitQuadrics `{tag?, opts?:{tol, fitTol, maxStepDeg, minStrips, smoothDeg, cones, merge, volumeTol, dryRun}}`

Refits the facet fans of a mesh body (rings and arcs of planar strips with parallel rails, or rails through one apex) as cylinder and
cone faces with circular end edges, and replaces the tag's body when it validates no worse and the volume changes by less than
`volumeTol` (default 5 %; the change is the chord sag of the facets). Reply: `{applied, reason?, tol, fitTol, faces:{before,after},
volume:{before,after,relative}, issues:{before,after}, merged, groups:[{kind, face, strips, full, axis, origin, radius|halfAngle,
maxResidual, sag}], declined:[{face, strips, reason}]}`. Spheres, tori and random meshes are left alone (`smoothProfile` or no rails).
Details: the book chapter "Cylinders and cones from facets".

## Not done

- `regen` does not accept `expect` (intent checks run only through `check`).
- `refDiff` compares the pre/post documents; references stored inside
  sketches or inside nested `insertPart` parts are not walked, and feature
  ids must be stable across the edit (a re-minted id looks like add+remove).
- No successor tracking for split faces (`~1`, `~2`), no tolerance on
  `near` other than strictly-nearest.

## Fonts for lettering: loadFont, unloadFont, listFonts, subsetFont (session state)

These four are the only requests of this file that change kernel state outside a tag: the font table of the kernel
instance (docs/text.md, "Loading fonts"). The text sketch entity uses a loaded font by name.

* `loadFont {name, data:<base64 TTF/OTF>, replace?}` -> `{name, hash, glyphs, chars, supported:[..], coverage:[..],
  outlines:"glyf"|"cff", warnings:[{code, detail}], alreadyLoaded, replaced, ..}`. Errors: `fontBadData`, `fontBadName`,
  `fontReserved`, `fontNameConflict` (same name, other file, no `replace`), `fontTooLarge`, `fontTooMany`.
* `unloadFont {name}` -> `{name, unloaded}`.
* `listFonts {}` -> `{fonts:[..]}` (built-ins first).
* `subsetFont {name, text}` -> `{name, hash, bytes, glyphs, data:<base64 TTF>, warnings}`; `textFontNotLoaded` for an unknown name.
* A document naming a font that is not loaded fails the sketch with `textFontNotLoaded` (names the font, tells how to load it).
