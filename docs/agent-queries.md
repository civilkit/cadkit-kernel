# Agent queries: check, resolve, describe, refDiff, section, sections, probe

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

## Not done

- `regen` does not accept `expect` (intent checks run only through `check`).
- `refDiff` compares the pre/post documents; references stored inside
  sketches or inside nested `insertPart` parts are not walked, and feature
  ids must be stable across the edit (a re-minted id looks like add+remove).
- No successor tracking for split faces (`~1`, `~2`), no tolerance on
  `near` other than strictly-nearest.
