# Identifiers and references

Features refer to each other, to sketch entities and to the faces and edges of the body by name. This chapter defines the identifiers, the grammar of face names, how edges are picked, and what keeps a name stable.

## Kinds of identifier

| Identifier | Scope | Example | Where it is used |
|---|---|---|---|
| Feature id | the part | `F4`, `P1` | `sketch`, `seeds`, `path`, `{"kind": "feature", "id": ...}` and as the first part of face names. |
| Sketch point id | one sketch | `a`, `p3`, `O` | `entities`, `constraints`, `{"kind": "sketchPoint", ...}`. |
| Sketch entity id | one sketch | `e1`, `ax` | `constraints`, `axis` of a revolve, `regions`, in face names. |
| Constraint id | one sketch | `k1` | Solver status (`conflicts`, `redundant`). |
| Face name | the body | `F2.C1`, `F2.S.e3~2@F5.1` | Anywhere a face is referenced. |
| Edge key | the body | `F2.C1\|F2.S.e1#0` | In replies and queries. Edges are referenced by their two faces. |
| Body name | the body | `Case` | The `body` field and `{"body": ...}` selectors. |

Ids are free-form strings; editors use `F<n>` for features, `e<n>` for sketch entities and `p<n>` for points. The kernel only requires that a reference finds an entity with the same text.

A sketch entity id appears in the names of the faces made from it, so renaming an entity renames those faces.

## Reference objects

A reference to a geometric object is a string or an object with a `kind`:

| Form | Meaning | Used by |
|---|---|---|
| `"F3"` | A feature id, or a sketch entity id when the field takes one (a revolve `axis`). | `sketch`, `seeds`, `surfaces` |
| `{"kind": "std", "id": "front"}` | A standard plane. | planes, mirrors, drafts |
| `{"kind": "face", "id": "F2.C1"}` | A face of the body. | sketch planes, plane bases, `endRef`, `neutral` |
| `{"kind": "feature", "id": "P1"}` | The plane or axis made by a feature. | planes, patterns |
| `{"kind": "curve", "id": "F11"}` | A curve feature (a helix or a curve made by `projectCurve`). | `sweep` path, curve lists |
| `{"kind": "axis", "axis": "x"}` | A world axis through the origin. | patterns, plane axes, `insertPart` |
| `{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}` | An edge: see below. | `fillet`, `chamfer`, `lip`, `fill`, `extend`, `pipeCut` |

The kernel keeps any other `{kind}` object unchanged; a feature that cannot use it reports `missingReference`. The kinds that only planes and curve lists take (`sketchPoint`, `vertex`, `sketchCurve`, `sketchLine`, `edge`) are in [planes](planes.md).

## Face names

A face name has the grammar

```text
name     := feature '.' role [ '~' k ] { '@' instance }
role     := 'C' digits            a cap
          | 'S.' entity           a side face from a sketch entity
          | 'B' digits            a blend (fillet or chamfer) face
          | other                 a tag written by the feature (h1, T0, P, F, W0, ...)
instance := feature '.' digits
```

- `feature` is the id of the feature that made the face.
- `~k` numbers the pieces when one logical face was split, counted from 1. A sketch with several closed regions numbers them: each region has its own caps (`F2.C0~1`, `F2.C0~2`) and a side face is numbered when its entity is shared by or split between regions. A shell makes outer and inner copies of each wall (`F2.C0~1` is the outer floor and `F2.C0~2` the inner one for the plate below).
- `@F5.1` marks a copy. A pattern, mirror, `moveBody` with `copy` or an inserted part adds the id of the feature and the number of the instance. Copies of copies chain: `F8.S.h1@F9.1@F1.1`.

### Names made by each feature

| Feature | Faces |
|---|---|
| `extrude` | `C0` (the cap whose outward normal is against the sketch normal), `C1` (outward normal along the sketch normal), `S.<entity>` for every profile entity. With `thin`: `S.<entity>.W0`, `.W1` (the two walls), `.E0`, `.E1` (end caps of an open chain), `.W0r`, `.W0m`, `.W0b` (corner pieces). |
| `revolve` | `S.<entity>`; `C0` at the start angle (in the sketch plane) and `C1` at the end for a partial turn; no caps for a full turn. |
| `sphere` | `S.e1`. |
| `sweep` | `S.<profile entity>` (split `~k` along the path pieces), `C0` and `C1`. |
| `loft` | `S.<entity>` named by the entities of the first section, `C0` and `C1`; no cap at a point end. |
| `hole` | `S.h1` (wall), `S.h2` (bottom), `S.h3`, `S.h4` and `S.K0` for the stepped and cone kinds; `~k` per hole when a sketch has several points. |
| `fillet`, `chamfer` | `B0`, `B1`, ... one per blended edge, `V0`... for the sphere patch at a vertex where three edges meet. |
| `rib` | `C0`, `C1`, `S.<entity>r`. |
| `lip` | `C1` (top of a lip) or `C0` (floor of a groove) and `S.<rim neighbour face>l`. |
| `thread` (modeled) | `T0` ... `T5`, the faces of the groove tool. |
| `thicken` | `W0`, `W1`, ... walls; the sheet faces keep their name with `~1` and `~2`. |
| `surfaceExtrude`, `surfaceRevolve` | `S.<entity>`. |
| `surfacePlanar` | `P`. |
| `fill` | `F`. |
| `knit`, `trimSurface`, `deleteFace` | The faces of the input surfaces keep their names. |
| `draft`, `moveFace`, `scale`, `moveBody` without `copy`, `combine` | All names are unchanged. A boolean that splits a face adds `~k`. |
| `lpattern`, `cpattern`, `mirror`, `moveBody` with `copy` | The copies of the seed faces, with `@<feature>.<k>` (k from 1; the seed is not renamed). |
| `insertPart` | Every face of the part, with `@<insert id>.1`. |

The chapters on [solids](features-solids.md), [edits](features-edits.md), [patterns](features-patterns.md), [surfaces](features-surfaces.md) and [assemblies](features-assemblies.md) give the details with each feature.

```json
{"type": "shell", "id": "F3", "faces": ["F2.C1"], "thickness": 2}
```

Run result: no errors; 11 faces, volume 3392 mm³. The faces are `F2.C0~1`, `F2.S.e1~1`, `F2.S.e2~2`, `F2.S.e3~2`, `F2.S.e4~1`, `F2.C1`, `F2.C0~2`, `F2.S.e1~2`, `F2.S.e2~1`, `F2.S.e3~1`, `F2.S.e4~2`.


```json
[
  {
    "type": "sketch", "id": "F3", "plane": {"kind": "face", "id": "F2.C1"},
    "points": [{"id": "c", "x": 8, "y": 10}],
    "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 3}], "constraints": []
  },
  {"type": "extrude", "id": "F4", "op": "cut", "sketch": "F3", "end": "blind", "depth": 4},
  {
    "type": "lpattern", "id": "F5", "seeds": ["F4"], "dir": {"kind": "axis", "axis": "x"}, "spacing": 12,
    "count": 3
  }
]
```

Run result: no errors; 12 faces, volume 7660.708 mm³. The pocket faces are `F4.C0`, `F4.S.e1`, `F4.C0@F5.1`, `F4.S.e1@F5.1`, `F4.C0@F5.2`, `F4.S.e1@F5.2`.


## Edges

Edges have no names of their own. An edge is the meeting of two faces, so a reference to an edge is the pair of face names plus a point on the edge:

```json
{"faces": ["F2.C1", "F2.S.e1"], "at": [20, 0, 10]}
```

The kernel takes the edge shared by the two faces that is nearest to `at`. When only one edge joins the faces `at` makes no difference; it is a tie-breaker for faces that meet in several edges (a cylinder and its cap, split faces), and it keeps a reference unambiguous after upstream edits move the edge.

In replies an edge is identified by a key `A|B#n`: the two face names in byte order, and a counter `n` that tells apart edges between the same pair of faces. `body.edges[].key` carries it.

## Stability of names

A name is derived from how the geometry was made, never from a position or an index into the mesh. Editing the numbers of an earlier feature (a depth, a radius, a sketch dimension) keeps names that still exist, so later features follow. Names change when topology changes:

- adding or removing a closed region in a sketch changes the `~k` numbering of the faces of the features built from it;
- deleting or reordering a feature leaves references to it pointing at faces that no longer exist: the feature that uses them reports `missingReference`, or in the worst case points at a different face with the same name. The `refDiff` request reports such changes (see [queries](../protocol/queries.md));
- a face split by a later feature is named with `~k` pieces; `moveFace`, `draft` and `lip` accept the name without `~k` and take every piece, other features need the exact name.

An edge pick never moves to another feature's faces or to another body. It needs both face names (or their `~k` family) to exist; if either is gone the feature reports `missingEdge`, however close the `at` point is to some other edge. The only fallback is for a pair whose `~k` numbers shifted: the edge between faces of the same two features that passes through `at` (within 1e-6 relative). A suppressed maker therefore makes its dependants report `missingEdge` and nothing else is rounded; unsuppressing it restores the pick. Renaming a sketch entity renames its faces, so picks that used the old id are lost, not re-pointed.

Prefer caps (`C0`, `C1`) and sketch-entity sides (`S.<entity>`) over positions, and let the `resolve` query turn a description into names (see [queries](../protocol/queries.md)).

```json
{"type": "moveFace", "id": "F3", "faces": ["F9.C1"], "distance": 1}
```

Run result: errors `F3`: `missingReference` `{"ref":"F9.C1"}`; 6 faces, volume 8000 mm³.


## Body names

A feature that makes a separate body can name it: `"op": "newBody", "body": "Case"`. The name is stored on the body (a lump) and follows it through later features. `{"body": "Case"}` then selects it in `moveBody`, `combine`, `deleteBody` and `scale`. See [patterns and transforms](features-patterns.md) and [edge and face edits](features-edits.md).

## See also

- [Overview of the part format](overview.md)
- [Sketches](sketches.md)
- [Planes](planes.md)
- [Features](features.md)
- [Persistent names](../kernel/naming.md)
- [Queries](../protocol/queries.md)
