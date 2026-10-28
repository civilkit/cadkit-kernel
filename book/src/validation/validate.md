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
| `opts.validateTimeMs` | number | `60000` | Wall-clock budget of the whole request in ms; `0` is unlimited. When it runs out the remaining checks are not run and the reply says which (see [the time budget](#the-time-budget)). Only enforced where the host provides a clock (the wasm module always does). |
| `opts.quick` | boolean | `false` | Run only the topology and bounding checks: the shell and edge-use checks, loop closure, and edge and vertex deviation. Leaves out the planar loop crossing test, the solid (volume) checks, the display mesh and its area witness, and the overlap scan of multi-lump bodies. See [the quick mode](#the-quick-mode). |

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
| `stats` | Always present: `{totalMs, <check>: ms, ...}`, the wall-clock time of the request and of each check that ran, in ms. The check names are `shell`, `loops`, `edges`, `solids`, `tessellation`, `meshAreaWitness`, `meshOpenEdges` and, for a body with several lumps, `overlap`. All 0 when the host has no clock. |
| `validateBudgetExceeded` | Present when the time budget ran out; see below. |
| `quick`, `checksSkipped` | Present when `opts.quick` was set: `true`, and the names of the checks that were left out. |
| `meshNotes` | Present when the tessellator marked a face as suspect: strings `<face>: <note>`. Not issues; see [tessellation checks](tessellation.md). |
| `info` | Present when the body has more than one lump and some of them overlap: one record per pair, `{code: "bodiesOverlap", severity: "info", a, b, volume}` with the names of the lumps when they have one. Overlap is legal (`newBody`, `moveBody`), so it is information, not an issue. |

The request answers `null` as its `result` when the tag exists but holds no body (a document of only sketches, or one whose solid features all failed), and `{"ok": false, "error": "noState: nothing regenerated yet"}` when the tag has no state at all.

A solid body in the state is what is validated. Surface bodies (sheets from `surfaceExtrude`, `fill`, and similar) are kept apart from it, so a document whose features produce only sheets answers `null`. Features that turn sheets into solids (`thicken`, `knit` with `solid`) put the solid into the state.

## What the request runs

In this order: the shell and edge checks, the loop checks, the edge and vertex checks, the solid checks (all from `ck-check`, with the tolerance and the cap above), then, unless `mesh` is `false`, the display mesh is built and its watertightness checked. The mesh check is an independent witness for the claim that the B-rep is a valid closed shell: a boolean can produce a body whose B-rep checks pass while one face lost its trims and meshes to nothing.

## The time budget

The cost of `validate` grows with the number of faces, edges and loops, and the mesh checks (re-tessellation, and an exact area integral for every analytic face) cost as much as meshing the whole body. A request that would run for minutes is cut off instead: the budget `opts.validateTimeMs` (default 60 s) is checked between the checks and inside their long loops (each face, each edge, each loop segment, each face of the area witness). When it runs out, the checks that have not started are not run, and the reply is the honest partial result: the issues found so far, `ok: false` (a run that was cut short is not a clean bill of health), and

```json
"validateBudgetExceeded": {
  "budgetMs": 60000, "elapsedMs": 60112,
  "checksNotRun": ["solids", "tessellation", "meshAreaWitness", "meshOpenEdges"],
  "checksPartial": ["loops"],
  "detail": "validate used up its time budget (opts.validateTimeMs); ..."
}
```

`checksNotRun` names the checks that never started; `checksPartial` the ones that were running and stopped early (their findings up to that point are in `issues`). Two stretches cannot be interrupted: the tessellation itself (it is one call; the budget is judged when it returns, and the mesh checks after it are then not run) and the volume integrals of the `solids` check. Raise `validateTimeMs`, set it to `0`, or use the quick mode to get a full answer on a very large body.

The check order is `shell`, `loops`, `edges`, `solids`, then the mesh checks, then `overlap`; the cheap topology checks therefore always run first. The `stats` object of every reply shows where the time went.

## The quick mode

`opts.quick: true` answers a sanity question about a big part without the expensive checks. It runs `shell` (every edge used by two faces with opposite directions, connectivity, Euler-Poincare), `loops` (closed loops; the planar crossing test `loopCrossings` is left out) and `edges` (vertex on curve, edge on surface, tolerances, degenerate edges). It leaves out `loopCrossings`, `solids` (positive volume, one outer shell per solid), `tessellation`, `meshAreaWitness`, `meshOpenEdges` and `overlap`; the reply names them in `checksSkipped`. A body that passes the quick mode can still fail the full request (an inside-out body, a face that meshes to nothing).

## Growth with the size of the part

`tools/validate_scale_check.mjs` times `validate` on generated parts. Wall-clock seconds for the whole request (mesh on) on the build machine, no time budget:

| faces | plate with holes | plate with joined pads | disjoint boxes (one lump each) |
|---|---|---|---|
| 500 | 2.1 | 0.10 | 0.08 |
| 1,000 | 5.5 | 0.16 | 0.12 |
| 2,000 | 10.5 | 0.33 | 0.23 |
| 4,000 | 14.0 | 0.75 | 0.47 |

Growth is now close to linear in the number of faces. Two stretches used to grow with a power above 1 and were rebuilt on an index: the crossing test of the loops of a planar face (every pair of loop segments was compared; a plate with 500 holes spent 34 of 36 s there; the growth is quadratic, which puts a plate with 4,000 holes at roughly half an hour) and the overlap scan of a body with many lumps (a single-lump copy of the body was built, with a full clean-up pass per deleted face, for every lump; 333 lumps took 2 minutes and the growth is cubic, which puts 667 lumps at roughly a quarter of an hour). What remains is the display mesh: on a plate with thousands of holes the tessellation and the area witness are about three quarters of the time. Use `stats` to see where a given part spends its time, `opts.quick` for a cheap sanity check, and `opts.validateTimeMs` to bound the request.

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
