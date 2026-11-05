# Blend notes

## Part of a petal outline: a step at the flank / tip junction, pick order (2026-11-04)

Report: on a five-petal body, a selection round one petal and its neighbour (rim flanks, tip arc, hub arcs, wall
edges) left a small step in the rim fillet where a straight flank runs into the tip arc, and one selected flank was
reported left sharp. Reproduced on `daisy-ring10-5` (hub radius 10, tip arc 8.83, 10 thick) with every combination
of one petal's rim (flank, tip, flank), its two hub arcs and its two wall edges, for one, two and all five petals, at
r 0.8 and 1, picked forward and reversed, compared with the reference engine (and analytic volumes where they exist),
face census, and a new per-edge check (`"sharp": true` rows; `BLEND_SHARP=1` for any row: points along each selected
edge must leave the result's boundary). Three causes; the rows are `daisy-ring-n5-{petal1-rim,petal1-rimhub(-rev),
flank1,flank1tip}-r*`.

### Cause

- The step (`petal1-rim`: +1.33 %, 29 faces instead of 25; r 1 refused). A convex rim edge that runs into the hub
  cylinder ends on it (`end_cylinder_solid`: the tool is cut by the cylinder's solid). That solid was extruded in the
  hub face's own frame, whose seam lies at the arc's start. For the flank that ends where the next hub arc starts
  (one flank of every petal) the seam ran through the corner, where the tool's end meets the cylinder, and the cut
  lost the hub face (open edges). The flush (tangent chain) attempt failed, and `blend_edges` retried with ordinary
  ends, in which the flank and tip tools run on past the tangent points: a step at both flank / tip junctions. At r 1
  the retry failed too. Fix: the clip's seam on the far side of the axis.
- Pick order at a reflex rim corner (`petal1-rimhub-rev`: +0.51 / +0.64 %, 30 faces instead of 27; picked rim first
  it was right). A hub arc's tool ends flush on the radial plane, which is the flank's plane: applied before the
  flank's tool, that end face merges into the flank face and the flank's rim edge now runs on to the hub fillet's
  contact line. The flank tool stopped at the edge's new end, `r` past the corner, and left a stray step face. Fix
  (`analytic.rs`): a flank tool ends at the selection's flush vertex when that vertex lies inside its (lengthened)
  edge, so every tool is the same solid whatever the order. The all-petal rim + hub set picked petal by petal had
  the same fault at one corner (+0.11 %, 45 faces instead of 42; inside the 0.2 % tolerance).
- Part of a tangent chain (`flank1` +7.19 %, r 1 refused; `flank1tip` +4.96 %): a flank picked without the tip arc it
  runs into (or the tip without the flank) ran its tool on by the convex margin past the tangent point and cut into
  the rim of the unselected edge (an overlapping end). Fix (`ends.rs`, `open_tangent_ends`): such an end vertex is
  a flush end; the fillet stops on the plane normal to the edge there. Both reference engines propagate a pick along
  the tangent chain (one flank alone gives the whole rim, 9.908 mm3), so the reference is analytic: the rim section
  swept from the tangent point to the hub cylinder (and Pappus on the tip arc), which matches the reference engine to
  1e-6 on the whole rim.

Table: 8 new rows (7 failed before, all pass now), 97 pass, 63 expected_fail, 16 hard.

### Not changed, not reproduced

- Rim + wall edge with the hub arc at that corner not selected (`petal1-rimwall`, a mixed hub / wall petal): the
  reference engine removes 0.21 mm3 less per such corner. That is the difference the `VTB` reference notes between the
  reference engine and the third-party kernel at the same corner (bottom of the `VTB` petal, +0.42 for two); the torus
  corner matches the third-party kernel, so no row.
- A selected edge left sharp: no synthetic selection left one (before or after the fixes). The step cause above hits
  exactly one flank of every petal, the one that ends where the next hub arc starts, so it may be the same fault on
  the reported part; that needs the part itself.
- Selecting the vertical tangent seams between a flank face and its tip face refuses (`branching blend selection`).

What did not work:

- `open_tangent_ends` on every selected edge: it also stopped the end arcs of the wall fillets that run on past an
  unselected rim at a `VTB` bottom corner (by design, `torus_corner.rs`), and `daisy-ring-n5-VTB-r0.8-ref` lost 0.15
  mm3 (-0.28 %). Edges on a blend face are left out.
- (considered, not tried) ordering the tools, flank before hub arc at every reflex corner, instead of pinning the
  flank tool's end: the `T` rows already apply some hub arcs before their flank and are right, so the order alone is
  not the fault; it was the tool's end.

## A rim arc into a reflex corner whose neighbour is not rounded (2026-11-04)

`daisy-ring-n5-hubrim-r0.8` / `-r1`: the rim edge of one hub arc only (top / hub cylinder), the flank rim edges and
the wall edges at its ends not selected. The convex arc tool (`analytic.rs`, revolved section) ran on past both
ends by the convex margin `(r + 1) / R` radians, so it cut a groove into the walls of both petals: 53 to 58 % more
removed than the flush result. A straight edge already stopped flush at such an end (`flush_end`: the only other face
there is a plane leaning toward the material); the arc had no such rule. Now `reflex_end_plane` (`ends.rs`) finds that
plane and the arc tool ends on it: flush when the plane is radial (the daisy flanks), else run on and clipped by the
plane. Reference: the rim section swept over the arc's angle (Pappus); both rows match it to 1e-4 mm3.

## The daisy rim corners: T, VT, VTB (2026-11-03)

Where a radial flank leaves the hub, the outline has a reflex corner and the body a concave vertical (wall) edge. The
reference results are in `docs/blend-reference-petal.md`. Two different corners come out of it:

- wall edge sharp (`T`): no corner face. The two rim fillets run on past the corner and meet along their intersection
  curve (the crease), from the corner of the two top contact lines down to the wall edge at `H - r`. In the corner
  square the result is the higher of the two fillet surfaces.
- wall edge filleted too (`VT`, `VTB`): the rim ball rolls round the wall fillet on a torus of major radius `2r`
  and minor radius `r` about the wall fillet's axis; the wall fillet ends at `H - r`.

The task brief asked for the torus patch in `T` as well; the reference has none there, and the volumes decide it.
For `daisy-ring-n5-T-r0.8-ref` the straight sections (Pappus on the arcs) remove 54.150 mm3 and the reference 54.645,
so the 10 corners remove 0.0495 each, 0.0966 r^3. The crease (max of the two fillet surfaces over the corner square,
flat-wall estimate) gives 0.0959 r^3; the true rolling-ball opening (a horn torus of major radius `r` about the sharp
edge) gives 0.0753 r^3, 0.11 mm3 short (0.2 % of the change); the flat mitre (min of the two) 0.333 r^3.

### Cause

- `T`: the flank tool ran on past the corner into the hub and was cut by the hub cylinder (`end_cylinder_solid`),
  while the hub arc's revolved tool ran on past both of its corners by the convex margin (`(r + 1) / R` radians) into
  the petals and cut a groove above the hub fillet's surface there: 5.7 to 10.8 % too much removed, or open edges
  where the overrunning tools met (`blendFailed`, `edge used by 4`). `mitre_pairs` (clip both tools at the bisector
  plane) only covers two straight edges between planes.
- `VT` / `VTB`: the wall edges (a line between a plane and a cylinder) go to the free-form builder, whose chains refuse
  a degree-3 vertex (`branching blend selection`). The planar mixed-vertex builder (`blend/mixed.rs`: wall fillets
  first, then the rim plus their end arcs as one tangent chain) only accepted lines between planes.

### Fix

- `blend/torus_corner.rs` (`VT`, `VTB`): `fillet_mixed` hands a selection that is not all lines between planes to
  `fillet_mixed_curved`: a wall line (concave, plane / convex cylinder parallel to it) with one or both rim edges at
  an end selected is filleted first, as an exact analytic cylinder (the analytic line builder, while `WALL` is set:
  ball centre at `r` from the plane and `R + r` from the hub axis; the tool's leg on the hub side is the chord, which
  lies inside the hub), then the rim edges and the wall fillets' end arcs go through `mixed::run` as one tangent
  chain. A wall end with only one rim edge selected (the bottom of a `VTB` petal) gets the same torus, run on past
  the sharp hub edge and trimmed by the hub wall. Results: the reference census and volume to 4e-4 mm3.
- `blend/rim_reflex.rs` (`T`): at a reflex corner between a straight rim edge (top / radial flank) and a rim arc
  (top / hub cylinder) whose wall edge is not selected, both rim tools stop flush at the corner vertex (the flank tool
  on the plane normal to the flank, the hub tool on the radial plane, which is the flank plane). After the round a
  wedge is cut: the flank tool run on into the hub, intersected with the hub tool run on past the corner (its section
  widened `r` beyond the hub wall), clipped to the flank tool's end plane. Every non-fillet face of the wedge lies on
  the top plane or on one of the two flush end planes, so the cut only adds the crease. The crease is given to both
  booleans in closed form through a pair hook (`d = r (1 - cos a)`, height `H - r + r sin a`, plan point at `d` from
  the flank on the circle of radius `R - d`).
- `boolean/assemble.rs`: while `MERGE_TORI` is set (only for the wedge cut), `coincident` recognises one torus. It
  asks the surface/surface intersection, which never reports coincident tori, so the wedge's piece of a hub fillet
  stayed a separate face (52 faces instead of 42). Done for every boolean it broke four tests that rely on torus
  fragments staying apart (`blend_curved_ends` arc-edge boss: 8 blend faces instead of 9; `tess_open` torus band
  spanning a period; `regen_cache`), so it is scoped to the wedge.

Table: 39 rows flip to pass (48 -> 87): 25 `T` (with the reference case), 12 `VT` and the `VT` / `VTB` reference
cases. Still failing, all `hard` (the reference engine and the second kernel fail them too), all on the `daisy-ring`
bases with hub radius 4: `T` r1 (a wedge cut whose result loses about 95 mm3, refused by the boolean sanity check),
`T` r2 (`r` = 2: the hub fillet is a horn torus, major radius 4 - 2 = 2 = minor), `VT` r1 / r2 (on n5 the two wall
fillets of one gap would need 23 degrees of a 20.4 degree hub arc; the builder fails and the free-form path refuses
the branching selection). The `rim-then-wall`
rows now get their rim right and fail on the next step (`concave chain end is not on a planar face`: the wall fillet
ending on the rim fillet, the in-place corner's case, `docs/blend-inplace-prototype.md`).

What did not work:

- Clipping each tool at the bisector, as `mitre_pairs` does for two planes: for a plane and a hub cylinder the
  bisector is a vertical parabolic cylinder (focus on the hub axis), and a planar clip leaves the flat facets. With the
  exact bisector the second tool's clip face coincides with the step the first one left (a coincident free-form face
  pair the boolean does not resolve); any two sequential cuts that meet along the crease have that problem, which is
  why the wedge is a third cut whose extra faces are all planar and coincide with flush end faces.
- Letting the booleans march the crease: at its top end both fillet surfaces are tangent to the top plane and to each
  other; the march followed the tangency (spline edges a few 1e-5 below the top) and left open edges on the
  smaller hubs. The closed-form crease fixed all of these.
- One corner of `daisy-hub-n5-T-r0.3` did not close (a fragment of the flank tool beyond the hub tool kept by the
  wedge's intersection) with the hub tool swept symmetrically about the corner and also with it swept from its own
  start meridian; trying other run-on lengths (2r / 3r, 1.5r / 2.5r, 2.5r / 3.5r) does. The root cause is not known.
- The free-form (B-spline) wall fillet as stage 1 of `VT`: the arc it leaves on the top is not a circle between a
  plane and a coaxial cylinder, so the rim chain cannot take it as an analytic edge.

## The daisy V sets: contact-edge fit deviation (2026-11-03)

The daisy V rows used to refuse with `blendFailed (normal 1)` (and some `normal 2` / `foot 1`).
Each concave vertical edge filleted fine alone; selecting several failed because each chain's blend is a
boolean that rebuilds the body, and the per-face `HINTS`/`BOXES` caches were keyed by `FaceId`. Between chains
the ids changed meaning, so a stale box or warm start pruned the wrong face or aimed `closest_point_near` at
another edge's surface. Clearing both caches for each chain fixes the refusals.

After that the rows stopped refusing but still failed validation: `EDGE_OFF_SURFACE` on the contact edges.
The boolean fits a pcurve on the blend surface for each new edge; on a contact edge the two faces are tangent
by construction, so `refit_edges_on_faces` leaves the fitted pcurve alone. Dropping those pcurves (once, after
all chains, on every blend face of the feature) then left the validator projecting the 3D curve, whose fit
from the boolean's intersection curve disagrees with the B-spline blend surface by about 2e-4 on the petals.
That is just over the validator's limit (`tol * scale * 10`; ~1.6e-4 for a 1e-6 model tolerance).

Fix: for every edge that carries a dropped blend-face pcurve, measure the edge / face deviation and carry it
into `Edge::tol` (capped at 0.04). `ck_check::validate` accepts a deviation up to `max(model tol, edge tol)`,
and the edge tolerance is the designed record of a fitted curve's fit error. All 32 daisy V rows
(`daisy-hub` and `daisy-ring`, N = 5..8, r = 0.3/0.5/1/2) now build, validate and sit within 0.002-0.006 % of
their reference volumes. No pass regressed.

What did not work:

- Clearing only the pcurve without recording the deviation: the projection then fails `EDGE_OFF_SURFACE`.
- Raising the model tolerance globally: it hides unrelated edges and changes the gate's `valid` numbers.
- Dropping the pcurve per chain: a later chain's boolean re-fits pcurves on an earlier chain's blend face.

The rim-only T sets and the branching VT/VTB sets are separate tasks (A2 and the vertex-blend/branching work):
the reference builds the rim corner as the rim blend continued as a torus about the vertical fillet's axis,
which the tool + boolean path does not reproduce; see `docs/blend-reference-petal.md`.
