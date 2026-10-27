# The validate request

`validate` runs the independent solid validator on the body of a tag and returns the issues it finds. This chapter gives the parameters, the reply, and examples including a deliberately broken body.

## Request

```json
{ "type": "validate", "tag": "main", "tol": 1e-6 }
```

| field | type | default | meaning |
|---|---|---|---|
| `tag` | string | `"main"` | The state to validate: the body left by the last `regen`, `importStep`, `importSat`, `importSldprt` or `importSkp` on that tag. |
| `tol` | number | `1e-6` | Model tolerance in mm. It scales the vertex, edge and loop tests; see [tolerances](tolerances.md). |
| `healTolerance` | number | `0.05` | The cap on the tolerance recorded on an edge or vertex. A larger tolerance is an error (`EDGE_TOLERANCE_EXCEEDED`, `VERTEX_TOLERANCE_EXCEEDED`). |
| `mesh` | boolean | `true` | Tessellate the body and check that the display mesh is watertight (`TESSELLATION_OPEN`). `false` skips the tessellation, which makes the request much cheaper on a large body. |

The validator does not change the body.

## Reply

```json
{
  "ok": true,
  "issues": [],
  "toleranceUsed": 0
}
```

| field | meaning |
|---|---|
| `ok` | `true` when `issues` is empty. |
| `issues` | An array of `{code, entity, detail}`; see [topology and geometry checks](checks.md) for every code. |
| `toleranceUsed` | The largest edge or vertex tolerance in the body, in mm. 0 when every entity is exact. |
| `notes` | Present when `toleranceUsed` is above 0: one sentence naming the largest tolerance and the cap. |
| `meshNotes` | Present when the tessellator marked a face as suspect: strings `<face>: <note>`. Not issues; see [tessellation checks](tessellation.md). |
| `info` | Present when the body has more than one lump and some of them overlap: one record per pair, `{code: "bodiesOverlap", severity: "info", a, b, volume}` with the names of the lumps when they have one. Overlap is legal (`newBody`, `moveBody`), so it is information, not an issue. |

The request answers `null` as its `result` when the tag exists but holds no body (a document of only sketches, or one whose solid features all failed), and `{"ok": false, "error": "noState: nothing regenerated yet"}` when the tag has no state at all.

A solid body in the state is what is validated. Surface bodies (sheets from `surfaceExtrude`, `fill`, and similar) are kept apart from it, so a document whose features produce only sheets answers `null`. Features that turn sheets into solids (`thicken`, `knit` with `solid`) put the solid into the state.

## What the request runs

In this order: the shell and edge checks, the loop checks, the edge and vertex checks, the solid checks (all from `ck-check`, with the tolerance and the cap above), then, unless `mesh` is `false`, the display mesh is built and its watertightness checked. The mesh check is an independent witness for the claim that the B-rep is a valid closed shell: a boolean can produce a body whose B-rep checks pass while one face lost its trims and meshes to nothing.

## Examples

### A valid body

A 40 x 20 x 10 block with a radius 4 hole through it:

```json
{ "type": "validate", "tag": "t" }
```

```json
{ "ok": true, "issues": [], "toleranceUsed": 0 }
```

Every body the kernel builds from correct input should answer like this. The crash-fuzz runs generate random documents and count the bodies that do not; of 5000 documents, 24 produced an invalid result (`TESSELLATION_OPEN`, `VOLUME_NONPOSITIVE` or `EDGE_OFF_SURFACE`, mostly on near-degenerate mirrored or revolved bodies), and none produced a trap.

### A broken body

Export the block to STEP with the `step` request, delete the last face from the `CLOSED_SHELL` list in the text, and import it without healing:

```json
{ "type": "importStep", "tag": "bad", "heal": "off", "data": "<STEP text with 5 of 6 faces>" }
```

Then:

```json
{ "type": "validate", "tag": "bad" }
```

```json
{
  "ok": false,
  "issues": [
    { "code": "SHELL_OPEN", "entity": "edge#0[I1.F1]", "detail": "free (boundary) edge: used by 1 face" },
    { "code": "SHELL_OPEN", "entity": "edge#7[I1.F2]", "detail": "free (boundary) edge: used by 1 face" },
    { "code": "SHELL_OPEN", "entity": "edge#9[I1.F3]", "detail": "free (boundary) edge: used by 1 face" },
    { "code": "SHELL_OPEN", "entity": "edge#11[I1.F5]", "detail": "free (boundary) edge: used by 1 face" },
    { "code": "EULER", "entity": "component#0", "detail": "V-E+F-(L-F) = 1 (V=8 E=12 F=5 L=5)" }
  ],
  "toleranceUsed": 0
}
```

The four edges that bordered the missing face are free; the Euler count of five faces is odd. The `heal` request cannot repair this: it has nothing to sew the free edges to, and answers with the same five issues in `remaining`.

The same block with one face's orientation flag flipped gives four `SHELL_ORIENTATION` issues, one for each edge of that face (`two face uses run the same direction`).

### The effect of `tol`

Sketch the block on the `front` plane (the rectangle from (0, 0) to (40, 20), extruded 10), export it with `step`, add 1e-3 to the x coordinate of the `CARTESIAN_POINT` of one `VERTEX_POINT` (entity `#22` in this export, the vertex at `(0, 20, 0)`), and import it with `heal: "off"`. The entity numbers belong to this particular export; the effect is that one vertex lies 1e-3 mm from the ends of the curves that meet there. Which issues appear depends on `tol`:

| request | result |
|---|---|
| `tol` 1e-6 (default) | two `VERTEX_OFF_CURVE` issues, `start vertex 1.000e-3 off curve` (on `edge#0[I1.F1\|I1.F6]` and `edge#11[I1.F5\|I1.F6]`) |
| `tol` 1e-2 | `ok: true` |

Moving each of the 8 vertex points by 1e-3 along each of the three axes (24 cases) gave `VERTEX_OFF_CURVE` and nothing else, and every case was repaired by `heal`.

Displacing a curve instead of a vertex gives edge-to-face deviations. Adding 1e-3 to x of the origin point of the `LINE` of edge 0 (entity `#26`) moves that straight edge off its two faces while its vertices stay put. The default tolerance then reports `VERTEX_OFF_CURVE` twice (the vertices are no longer on the displaced line) and `EDGE_OFF_SURFACE` on face `I1.F6` (`edge#0[I1.F1|I1.F6] pcurve deviates 1.000e-3`); `tol` 1e-2 gives `ok: true`.

The comparison is against `10 * tol * (1 + |p|)` with `p` the start of the edge's curve: for an edge that starts about 20 mm from the origin the limit is about `2e-4` at the default tolerance and about `2` at `1e-2`. Raising `tol` hides a defect rather than fixing it: after a validation at a loose tolerance the body is not exact to the tolerance of the operations that follow.

### Tolerances and the cap

After `heal` with tolerant geometry the body from the vertex case carries a vertex tolerance of 1.05e-3 (`VERTEX_TOLERANCE` in the repair report), and the displaced-line case carries an edge tolerance of 1.05e-3 (`EDGE_TOLERANCE`). Both validate clean at the default cap:

```json
{ "ok": true, "issues": [], "toleranceUsed": 0.00105,
  "notes": ["body carries edge/vertex tolerances up to 1.050e-3 mm (cap 5.000e-2)"] }
```

With a smaller cap (`healTolerance: 1e-4`) the vertex case reports one `VERTEX_TOLERANCE_EXCEEDED` per edge that ends in the vertex (three issues):

```json
{ "ok": false,
  "issues": [ { "code": "VERTEX_TOLERANCE_EXCEEDED", "entity": "edge#0[I1.F1|I1.F6]",
                "detail": "vertex tolerance 1.050e-3 exceeds the cap 1.000e-4" },
              { "code": "VERTEX_TOLERANCE_EXCEEDED", "entity": "edge#1[I1.F1|I1.F5]",
                "detail": "vertex tolerance 1.050e-3 exceeds the cap 1.000e-4" },
              { "code": "VERTEX_TOLERANCE_EXCEEDED", "entity": "edge#11[I1.F5|I1.F6]",
                "detail": "vertex tolerance 1.050e-3 exceeds the cap 1.000e-4" } ],
  "toleranceUsed": 0.00105,
  "notes": ["body carries edge/vertex tolerances up to 1.050e-3 mm (cap 1.000e-4)"] }
```

and the displaced-line case reports a single `EDGE_TOLERANCE_EXCEEDED` on `edge#0[I1.F1|I1.F6]` (`edge tolerance 1.050e-3 exceeds the cap 1.000e-4`).

### Overlapping lumps

A block and a second block added with `op: "newBody"` that overlaps it by 10 x 10 x 10 is valid, with the overlap as information:

```json
{ "ok": true, "issues": [], "toleranceUsed": 0,
  "info": [ { "code": "bodiesOverlap", "severity": "info", "a": "F2.C0", "b": "F4.C0",
              "bName": "tool", "volume": 1000.0 } ] }
```

## Using validate in a loop

A client that builds or edits a part usually calls `regen`, reads `errors`, and then `validate` with the default tolerance. If `errors` is empty and `issues` is not, the issues point at the body itself (the `entity` field gives the faces). The [overview](overview.md) describes how to combine the layers. The `check` request with the expectation `{"kind": "valid"}` runs the same validator and counts the issues, so it can be combined with other expectations (volume, face count) in one call.

## See also

- [Validation overview](overview.md)
- [Topology and geometry checks](checks.md)
- [Tessellation checks](tessellation.md)
- [Tolerances](tolerances.md)
- [Healing](healing.md)
