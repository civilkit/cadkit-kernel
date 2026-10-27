# Persistent names

A part refers to faces and edges of earlier features: "fillet these edges", "sketch on this face". The kernel names faces by how they were made, so a reference keeps meaning the same thing when a dimension upstream changes. This chapter describes the naming scheme, how references resolve, what keeps names stable, and where they break.

## Face names

A face name has the form

```text
<feature>.<role>[~<k>][@<instance>...]
```

| part | meaning |
|---|---|
| `<feature>` | The id of the feature that made the face: `F2`. |
| `S.<entity>` | A side face swept from the sketch entity `<entity>` of the feature's sketch: `F2.S.e1`. Extrude, revolve, sweep and loft all name side faces this way. Faces from a `hole` feature use `h<n>` for the hole's entity: `F4.S.h1`. |
| `C0`, `C1` | The end caps. `C0` is the low end (an extrude: the minimum along the sketch normal, also when flipped; a revolve: the start angle) and `C1` the high end. A full 360 degree revolve has no caps. |
| `B<k>` | The `k`-th blend face (fillet or chamfer) of the feature, counted from 0: `F3.B0`. |
| `V<k>`, `P<k>`, `W<k>`, `F` | A sphere corner patch of a fillet, a free-form corner patch, a thicken wall, a fill patch. |
| `~<k>` | The `k`-th piece (1-based) of a face that was split into several. |
| `@<feature>.<n>` | Copy `n` made by a pattern feature (the seed has no suffix); chained for nested copies and for parts inserted into assemblies: `F8.S.h1@F9.1@F1.1`. |

Examples from runs of the kernel:

| operation | names produced |
|---|---|
| extrude of a rectangle `F1` by feature `F2` | `F2.C0`, `F2.C1`, `F2.S.e1` to `F2.S.e4` |
| cut by a circle extrude `F4` through the block | the block's faces keep their names; the new wall is `F4.S.e1` (a tool face keeps the tool feature's id, with the sense flipped) |
| fillet `F3` on one edge | `F3.B0`; the faces next to it keep their names |
| three fillets meeting at a convex corner | `F3.B0`, `F3.B1`, `F3.B2` and the sphere patch `F3.V0` |
| a cut that splits a block in two | each face that is cut in two becomes `F2.C0~1` and `F2.C0~2`, and so on, and the body has two lumps |
| shell of a block through its top face | the opened top keeps `F2.C1`; every other face is split into its outer and inner wall, named `~1` and `~2` by centroid order, for example `F2.S.e1~1` and `F2.S.e1~2` (which number is the outer wall depends on the face) |
| pattern `F5` with count 3 of a hole made by `F4` | `F4.S.e1` (seed), `F4.S.e1@F5.1`, `F4.S.e1@F5.2` |
| thicken `F3` of a sheet | the layers keep the sheet's names with `~k`, and the walls are `F3.W0`, `F3.W1`, ... |

Several rules fix the suffix numbers and the order of faces so that they do not depend on rounding: a face split into several pieces is numbered by the lexicographic order of the pieces' centroids, a closed spline that is cut at half its length gets its pieces numbered by the rank of their control-net box centres, and a face absorbed into another one by a union keeps its old name as an alias (below).

## Edge names

An edge is named by the two faces it separates, in bytewise order, followed by its index in the exported edge list of the body:

```text
F2.C1|F2.S.e1#2
```

The `#2` is not persistent. It is the position of the edge in the edge list and changes whenever the topology of the body changes. A document therefore never stores an edge key. Features that need an edge, such as fillet, chamfer and pipe cut, store an *edge reference*:

```json
{ "faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10] }
```

The kernel takes the edge between the two named faces that lies nearest to the point `at`. The point is a tie-breaker between edges that are otherwise the same (two faces can meet along more than one edge), not the primary key. A reference whose face names are right survives a change of dimensions even when `at` is no longer exactly on the edge.

## How a reference resolves

1. **Face names.** `Body::find_face` looks for a live face whose name equals the string. If there is none, it looks for a face whose `absorbed` list contains the name. When two coplanar faces merge in a union, the surviving face keeps its own name and records the other's name in `absorbed`, so a later feature that still refers to the old name finds the merged face. A test: after uniting a block with another block flush against it, a fillet on the edge between `F4.C1` and `F4.S.e3` still works, although `F4.C1` has merged into `F2.C1`.
2. **Edge references.** The two names are resolved as above, the edges whose faces are that pair are collected, and the one nearest `at` wins.
3. **No match.** A face name that does not resolve fails the feature with `missingReference`, and an edge reference with `missingEdge`. `describe` gives a more helpful answer to a name that no longer exists: `found: false`, an `error`, and the three `nearest` names ranked by similarity (for `F2.S.e9`, it offers `F2.S.e1` first).

## Stability across edits

Names come from the construction, not from positions, so changing a number does not change a name. In the following run the same part was regenerated with three different block sizes and fillet radii (40 x 20 x 10 with r = 2, 60 x 35 x 15 with r = 4, and 10 x 10 x 30 with r = 1); the face lists were identical: `F2.C0 F2.S.e3 F2.C1 F2.S.e1 F2.S.e4 F2.S.e2 F3.B0`.

The repository measures this as a gate: edits of upstream dimensions across 37 test cases leave 214 of 216 edits with every reference still resolving to the same kind of geometry, and none resolving to a wrong entity. The two losses are edits that change the topology.

### What breaks a name

- **Deleting or reordering an upstream feature** can re-point a name without any error: the name still resolves, but to a different face than the author meant. This is the known weak point of the scheme, and the reason `refDiff` exists (below).
- **Edits that change the topology**: a dimension that makes a face vanish, split in two, or merge with its neighbour. A `~k` piece number is the order of the pieces' centroids, so it can swap when the pieces move past each other; a shelled body's `~1` and `~2` can change meaning after an edit that changes which wall is the outer one.
- **Split faces have no successor tracking.** If `F2.C1` is split into `F2.C1~1` and `F2.C1~2` by a later edit, a reference to `F2.C1` stops resolving; nothing says which piece is meant.
- **Sketch entity ids.** A side-face name embeds the id of the sketch entity (`e1`), so a sketch whose entity ids change renames its faces.

## Finding out when references change

Two requests help an editor keep references valid.

`refDiff {base, part}` regenerates the previous document and the edited document privately, records which entity every face name and `{faces, at}` reference resolves to at the feature that uses it, and compares them. The comparison is by surface kind, direction, the kinds of the neighbouring faces, size (ratio between 0.25 and 4) and position, so an ordinary dimension change that keeps the topology does not warn. The reply is `{ok, checked, warnings, features, errors}`; a warning has `code` `referenceChanged` (the name now points at a different entity) or `referenceLost` (it no longer resolves), the `feature` that uses it, the `ref`, a `reason`, and `before` and `after` descriptions. `features` lists the ids `removed` and `added` and whether the order changed. References stored inside sketches or inside a nested part are not walked, and feature ids must be stable across the edit: an id that is re-minted looks like a removal plus an addition.

`describe {names}` returns the facts of a name (surface kind, size, direction, centroid, the feature and role, the adjacent faces and a one-line `text`), so an author can check that a reference means what they think. `resolve {descriptions}` goes the other way: given a description such as `{kind: "face", surface: "plane", normal: "+z", extreme: "max"}` it returns the matching names, each with a ready-to-store `name` and `ref`. If more than one entity matches, the status is `ambiguous` and every candidate is listed; the kernel never picks silently. For the full grammar see [queries](../protocol/queries.md).

## See also

- [The B-rep model](brep.md)
- [Identifiers and references](../format/references.md)
- [Regeneration and caching](regeneration.md)
- [Queries](../protocol/queries.md)
