# Agent queries: check, resolve, describe, refDiff

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

## Not done

- `regen` does not accept `expect` (intent checks run only through `check`).
- `refDiff` compares the pre/post documents; references stored inside
  sketches or inside nested `insertPart` parts are not walked, and feature
  ids must be stable across the edit (a re-minted id looks like add+remove).
- No successor tracking for split faces (`~1`, `~2`), no tolerance on
  `near` other than strictly-nearest.
