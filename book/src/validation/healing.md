# Healing

`ck-heal` repairs the defects that the validator finds in a body: gaps, free edges, misordered or reversed loops, flipped faces, inside-out solids, and geometry that is a little off its faces. It is meant for imported bodies and for bodies that a failed edit left damaged. This chapter lists what it detects and repairs, how to call it, and what it does not do.

## Principles

- **Driven by the validator.** Healing first runs the validator at a strict tolerance (`min(tol, 1e-6)`) and only touches what it reports. A valid body is returned unchanged: healing is a fixed point on valid bodies, and a test in the repository checks that no fix is recorded and that the volume agrees to 1e-12.
- **Repair within `tol`.** Detection is strict; repair may move things by up to the healing tolerance `tol`.
- **Every repair is reported** as `{code, entity, detail}`, and the findings that are left as `remaining`.
- **Geometry stays.** Healing changes topology (vertices merged, edges sewn, loops re-chained, faces flipped) and records tolerances. It does not move or refit curves and surfaces to a compromise position.

## The passes

The passes run in this order, and the sequence repeats up to four times while it keeps changing something.

| pass | triggered by | what it does | report codes |
|---|---|---|---|
| Near-miss vertices | `SHELL_OPEN`, `LOOP_OPEN`, `EDGE_DEGENERATE`, `EDGE_UNUSED` | Merges vertices closer than `tol` that bound an unpaired or tiny edge, or lie on both sides of a loop gap. Only these candidates are considered, so healthy vertices are never touched. | `VERTEX_MERGED` |
| Degenerate edges and faces | `EDGE_DEGENERATE` | Drops edges shorter than `tol` whose ends were merged from every loop, and deletes faces left without a loop (never the last face of a shell). | `EDGE_REMOVED`, `FACE_REMOVED` |
| Loops | `LOOP_OPEN` | Re-chains the coedges of a loop head to tail by a bounded depth-first search (up to 8 solutions), flipping coedges that run against the chain, and splits a loop that closes early into several. | `LOOP_CHAINED`, `LOOP_UNFIXED` |
| Sewing | `SHELL_OPEN`, `EDGE_UNUSED`, `SHELL_EMPTY` | Pairs free edges that join the same two vertices and coincide: 5 sample points of the two curves within `5 * tol`, in the same or the opposite direction. The two become one edge with two coedges. Then shells and lumps of the joined faces are merged, shells that fell apart are split, empty shells dropped. | `EDGE_SEWN`, `SHELL_MERGED`, `LUMP_MERGED`, `SHELL_SPLIT`, `SHELL_REMOVED` |
| Orientation | `SHELL_ORIENTATION`, `VOLUME_NONPOSITIVE` | Reverses loops against their face sense, flips faces that disagree with their neighbours, and turns an inside-out solid outward. | `LOOP_REVERSED`, `COEDGE_FLIPPED`, `FACE_FLIPPED`, `SOLID_FLIPPED` |
| Tolerant geometry | `tolerant: true`, after the structural passes | Gives an edge whose curve lies farther from its faces than the strict tolerance a tolerance equal to the measured deviation times 1.05 (9 sample points first, 8 intervals, then 33 where those fail), provided that is within the cap. Gives vertices farther than that from their edge ends the same treatment. | `EDGE_TOLERANCE`, `VERTEX_TOLERANCE` |

The pass switches are `sew` (near-miss vertices, loop gaps and sewing), `fixSmall` (degenerate edges and slivers), `fixOrientation`, and `tolerant`.

## Calling it

### The `heal` request

```json
{ "type": "heal", "tag": "main", "tol": 1e-5,
  "heal": { "sew": true, "fixSmall": true, "fixOrientation": true, "tolerant": true, "cap": 0.05 },
  "dryRun": false }
```

| field | default | meaning |
|---|---|---|
| `tag` | `"main"` | The state to repair. |
| `tol` | `1e-5` | Healing tolerance, mm. A value in `heal.tol` is used when `tol` is not given. |
| `heal` | all passes on | An object with the switches above and `tol`, `cap`. `"off"` or `false` runs the structural passes only, without the tolerant pass. |
| `healTolerance` | `0.05` | The cap on a tolerance that may be recorded (the same as `heal.cap`). |
| `dryRun` | `false` | Report what would be repaired and leave the body alone. |
| `damage` | none | A self-test: break a copy of the body in a named way, repair that copy, and report whether the repair restored it. The body in the state is not changed. |

Reply:

```json
{ "fixed": [ { "code": "VERTEX_TOLERANCE", "entity": "vertex#0", "detail": "vertex tolerance set to 1.050e-3" } ],
  "remaining": [],
  "volume": { "before": 8000.0, "after": 8000.0 } }
```

`fixed` and `remaining` are described above; `remaining` has the shape of `validate` issues. `volume` is the volume before and after, a quick check that nothing moved. Unless `dryRun` or `damage` is given, the repaired body replaces the state of the tag.

### Healing on import

`importStep` heals by default with `tol` `1e-4`. (The SAT, SolidWorks and SketchUp import requests do not run the repair pass; call `heal` on their tag afterwards if the body needs it.) `heal: "off"` disables it; `heal: {tol, sew, fixSmall, fixOrientation, tolerant, cap}` tunes it, and `healTolerance` sets the cap. The reply has `import.heal` with the number of repairs, the number of findings left, and the largest tolerance used:

```json
{ "solids": 1, "faces": 6, "heal": { "fixed": 1, "remaining": 0, "toleranceUsed": 0.00105 } }
```

### Healing during regeneration

`opts.autoHeal: true` (with `opts.healTol`, default `1e-5`) tries a repair pass after every feature that leaves validator issues. The healed body is kept only if it has fewer issues than before, and the feature gets a `healed` entry with the number of fixes, the codes and the number left. The entry is written to `errors` (not `warnings`); a feature that already has an entry keeps it.

### The Rust API

`ck_heal::heal(&mut body, tol) -> HealReport` and `heal_with(&mut body, &HealOpts)`; `HealOpts { tol, sew, fix_small, fix_orientation, tolerant, cap }` (the constructor `HealOpts::new` leaves `tolerant` off, the request turns it on).

## Examples

The `damage` self-test breaks a block in seven ways and repairs the copy (tolerance `1e-5`). All seven are restored, with the volume unchanged (the `tiny` case moves it by 1.3e-6):

| damage | what it does | repairs reported |
|---|---|---|
| `detach` | Un-sews one face, shifting it by 2e-6. | 4 x `VERTEX_MERGED`, 4 x `EDGE_SEWN` |
| `shells` | Detaches half of a shell into a new lump. | 8 x `VERTEX_MERGED`, 8 x `EDGE_SEWN`, `SHELL_MERGED`, `LUMP_MERGED` |
| `flip` | Flips one face. | `FACE_FLIPPED` |
| `invert` | Flips every face. | `SOLID_FLIPPED` |
| `reverseCoedge` | Reverses one coedge. | `LOOP_CHAINED` |
| `shuffle` | Swaps two coedges of a loop. | `LOOP_CHAINED` |
| `tiny` | Splits an edge 1e-9 of its length from its start. | `VERTEX_MERGED`, `EDGE_REMOVED` |

The 40 x 20 x 10 block sketched on the `front` plane (entity numbers as in [the validate request](validate.md)), exported, imported with `heal: "off"`, with 1e-3 added to x of one `VERTEX_POINT`'s coordinates (the vertex then lies 1e-3 from the curves that end there):

| request | result |
|---|---|
| default (`tolerant` on) | fixed: `VERTEX_TOLERANCE` 1.05e-3; remaining: none |
| `heal: {"tolerant": false}` or `heal: "off"` | nothing fixed; remaining: the two `VERTEX_OFF_CURVE` findings |
| `heal: {"cap": 1e-4}` | nothing fixed; the deviation is above the cap, so the findings remain |

If instead the origin of the `LINE` of one edge is displaced by 1e-3 (the curve moves, the vertices do not), the default repair records `EDGE_TOLERANCE` 1.05e-3, and `tolerant: false` leaves two `VERTEX_OFF_CURVE` and one `EDGE_OFF_SURFACE`. In a scan that moved every `CARTESIAN_POINT` of the export by 1e-3 along each axis, 27 of 81 cases stayed valid (points whose move is invisible, such as an axis origin), 24 gave only `VERTEX_OFF_CURVE`, 6 only `EDGE_OFF_SURFACE`, and 24 both; every failing case was repaired by tolerant healing within the default cap. A displacement of 1e-2 gives a tolerance of 1.05e-2, within the cap of 0.05.

## Limits

- **Missing material.** A face that is absent cannot be invented. A block with one face removed has four free edges and nothing to sew them to; `heal` leaves all five findings (`SHELL_OPEN` x 4 and `EULER`) in `remaining`.
- **Gaps larger than `tol`.** Vertices are merged and edges sewn only within `tol` (and curves must agree within `5 * tol`). Larger gaps are reported.
- **Deviations above the cap.** An edge or vertex that is farther off than `cap` is not given a tolerance. Raise the cap, or repair the source.
- **Not repaired.** `SHELL_NONMANIFOLD`, `EULER` on its own, `LOOP_SELF_INTERSECT`, `LOOP_CROSS`, `FACE_NO_LOOP`, `EDGE_RANGE` and `SOLID_MULTIPLE_OUTERS` have no repair pass, and neither has `SHELL_DISCONNECTED` on its own: a body whose only finding is that code is never touched (the `SHELL_SPLIT` fix runs only after a sewing step that joined edges). (The STEP importer separates multiple outer shells into lumps itself.)
- **Tolerant edges are not free.** Booleans and blends build new exact edges and do not propagate tolerances, and `fillet` and `chamfer` refuse a toleranced edge with `blendTolerantEdge`. To model on an imported part, heal it, then use features that do not need those edges, or rebuild the region.
- **Order of results.** The fixes of a pass follow the order of the entities in the body, so the report is deterministic for a given body.

## See also

- [Topology and geometry checks](checks.md)
- [The validate request](validate.md)
- [Tolerances](tolerances.md)
- [STEP](../io/step.md)
