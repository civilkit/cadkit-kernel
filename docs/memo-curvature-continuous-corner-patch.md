# Decision memo: curvature-continuous (G2) overhang corner patches

Status: proposal for the product owner. Docs only; nothing here changes behaviour. Date: 2026-11-20.

## The question in one paragraph

On the user's overhang fillet part (`_scratch-volume-hunt/overhang_fillet_fail.ckpart`, corner patches `F6.P0` and `F6.P1`) a zebra-stripe view shows the stripes bending where the round and cove tubes meet the corner patch, and a Gaussian-curvature view shows four small hot spots at the patch corners. That is what a join that is tangent-continuous (G1) but not curvature-continuous (G2) looks like. Should we make the patch G2?

(I did not open the `.ckpart` or render it for this memo; the picture above is the kit session's reading, and the code reading below agrees with it.)

## What the patch is today

The patch is built in `crates/ck-ops/src/blend/overhang.rs`, `patch_surface`, and spliced in by the corner builder (called with `n = 49`). In steps:

1. A 49 x 49 grid of points is laid over the six-sided hexagon boundary: the four tube end arcs (two on the bottom chain, two on the top) and the two straight spring lines on the walls.
2. The interior is the discrete Laplace (harmonic) solution of that boundary (`harmonic_fill`). It was chosen over a Coons blend because Coons propagated the ~90 degree corner where two hexagon sides meet along the whole middle line as a fold, which the tessellator turned into needle triangles (`docs/blend-corpus.md`, "what the branch builds").
3. The first interior rows next to the tubes are projected into the tubes' tangent planes, and the first interior columns next to the walls into the wall planes, with a cosine taper over 7 or more grid lines (more for bigger radii). The projection exists so that the fitted cubic's end derivative lands in the neighbour's tangent plane.
4. A bicubic (degree 3 x 3) B-spline interpolates the grid (`ck_geom::fit::interpolate_surface`).

## Continuity at each of the four boundaries today

| Boundary | Neighbour | Position (G0) | Tangent plane (G1) | Curvature (G2) |
|---|---|---|---|---|
| two wall spring lines (straight) | wall plane, curvature 0 | exact (shared hexagon chain) | approximate: grid columns projected into the wall plane | not controlled. The patch has no reason to be flat across the line, so its cross-boundary normal curvature there is whatever the harmonic fill gives |
| two tube end arcs on the bottom chain | round / cove cylinder, normal curvature 1/r | exact | approximate: grid rows projected into the cylinder's tangent planes | not controlled; the target is 1/r across, the patch has no term that aims at it |
| two tube end arcs on the top chain | same | same | same | same |

(That is 2 wall + 4 tube arcs = the six hexagon sides; "four boundaries" in the question is the two tube pairs plus the two walls.)

G1 is therefore discrete and approximate, not exact. The measured boundary normal error after the recent fix (commit `a4037abd`, which moved the tube projection after the wall projection because it was being undone) is about 0.9 degrees on the walls and 1.6 degrees on the tubes, as reported. A flag for whoever runs the numbers again: the regression test `blend_overhang.rs` (`assert_wraps`) allows 0.02 rad (1.15 degrees) at the midpoint of each seam, so a 1.6 degree figure on the tubes is either measured elsewhere on the seam (near the corners, where it is expected to be worst) or is not covered by that test. `g1Defect` in the blend corpus does not look at patch faces at all, so the patch seams are checked only by that Rust test.

Why G2 is visibly missing: across a tube seam the neighbouring cylinder has normal curvature 1/r (0.5 per mm at r = 2). The patch has no curvature condition, so its curvature jumps across the seam. Zebra stripes are a reflection of the normal field, so a curvature jump shows as a kink in the stripe's slope. The same grid projection that makes G1 approximate also adds a small ripple over the taper band.

The four hot spots sit where a tube end arc meets a wall spring line (the hexagon corners). That is partly unavoidable, see "what other kernels do".

## Is G2 achievable?

Yes along the sides; at the four hexagon corners only in a weakened sense.

- At a hexagon corner the two adjacent boundaries want different curvatures (1/r for the tube side, 0 for the wall side) and different tangent planes meet there with a ~90 degree turn of the boundary curve. Matching both exactly at one point requires compatible twist and second-order cross terms (the classic corner compatibility problem for G2 patches). A single smooth patch can only satisfy it by relaxing at the corner or by shrinking the curvature jump toward the corner. So "G2 everywhere" is not a realistic target; "G2 along the sides, a small bounded curvature dip at the corners" is.
- Along the sides, G2 is a bounded, well-understood job. Three routes:

  **A. Same bicubic, constrain the first three rows to a curvature condition.** Keep the grid and the harmonic fill, but replace the projection (step 3) with a small solve that fixes rows 1 and 2 so the interpolant's first and second cross derivatives equal the neighbour's. Cheapest. Risk: a cubic has little freedom, and the taper band is already where ringing shows up; asking it for curvature as well may need a wider band and may re-introduce the fold/ripple trouble. 4 to 6 days.

  **B. Higher degree (quintic) patch built by a constrained fit.** Represent the patch as a degree 5 x 5 B-spline whose boundary control rows are set from position, tangent and curvature of each neighbour, and whose interior minimises a fairness energy (thin-plate / minimal variation) instead of the harmonic grid. This is the standard way to get G2 and is what plate-fill style tools do. 8 to 12 days: new fitting code in `ck-geom` (a constrained least-squares with a banded solve), new corner-compatibility handling, rewiring of the pcurve code (`boundary_pcurve` assumes boundary isolines, which still holds).

  **C. A different patch type: an n-sided or vertex-blend patch with Hermite cross-boundary data.** `crates/ck-geom/src/nsided.rs` already does a central-split G1 fill with cross-boundary tangents; extending it to curvature is the same math as B with a different topology. Larger and riskier (10 to 15 days) because the patch becomes several faces instead of one, which changes face counts, names and the corpus expectations (the OCCT reference has one patch per corner).

## Cost and risk

Code areas: `crates/ck-ops/src/blend/overhang.rs` (`patch_surface`, `chamfer_patch_surface` if chamfer corners follow), `crates/ck-geom/src/fit.rs` (or a new constrained-fit module for B), the splice and pcurve code in the same file, and `crates/ck-model/tests/blend_overhang.rs`.

Tests to add or change: a G2 residual check on every seam (the corner's analogue of `assert_wraps`, comparing normal curvature across the seam, with a tolerance that excludes a few percent of the arc near each hexagon corner); extend `g1Defect` in `tools/blend_corpus.mjs` to look at patch faces; a mesh-quality test (the existing one checks patch G1 and normal jump; it needs a dihedral and flipped-triangle count, since this is exactly where the Coons fold came from).

Corpus impact: the overhang cases (`overhang_*`, five ok, two without an OCCT reference, one expected refusal) compare volume to OCCT at 1e-7 relative. A better-fitting patch moves the volume by a small amount in either direction; I expect it to stay well inside that band but it must be re-measured on the CI box (glibc libm), not on the Mac. Stacked-block and chamfer rows in `docs/blend-table.md` reuse the same patch and must be rerun.

Mesh quality: the main risk. Higher degree and tighter curvature control make the control net wigglier; the fold seen with Coons shows how sensitive the tessellator is at the pinch. Budget a day or two for tessellation tuning in any route.

Rough effort: A 4 to 6 days, B 8 to 12 days, C 10 to 15 days, each including tests and a CI quick plus full gate. These are estimates from reading the code, not from a prototype.

## What other CAD kernels do (what I know, and what I am guessing)

- OCCT (the reference our corpus compares against): its fillet corner patches are filled surfaces (plate / filling style approximations) that are G1 to the stripes. I am fairly sure OCCT does not guarantee G2 at fillet corners; our own docs note OCCT's reference patch is "a plate fill that confines the corner to the boundary". Fairly confident, not verified against the source here.
- Parasolid / ACIS (SolidWorks, NX, Onshape and others): these offer curvature-continuous ("G2" or "conic / curvature") fillet profiles along an edge, and setback or vertex blends at corners. I know the options exist in SolidWorks (fillet "curvature continuous" and "setback" corners). I do not know how the corner patch itself is constructed or how much of it is G2 at the corners. Guess: G2 across the main seams, relaxed at the corner points.
- Class-A surfacing tools (Alias, ICEM) hand build G2 or G3 corner patches. That is the benchmark for zebra-quality, but it is a different product tier than a parametric solid modeller's automatic fillet.

The honest summary: matching OCCT, our reference, means G1 with approximate corners. Going beyond it is a differentiator, not a parity fix.

## Recommendation

Accept G1 now and document it; schedule G2 as a later, optional improvement (route A as a time-boxed spike first, B only if A fails).

Reasons: (1) the corner is correct and watertight, volumes match the reference at 1e-7, and the visible defect is cosmetic and only shows in zebra or curvature analysis; (2) the patch has just been through two rounds of mesh fixes and any change re-opens that risk; (3) the corner cannot be fully G2 anyway, so the hot spots would shrink rather than disappear; (4) nothing in the corpus asks for it. Concretely, now: add to `docs/blend-corpus.md` a line stating the patch is G1 (approximate, 0.9 / 1.6 degree), not G2, and add the G2 residual as a diagnostic print (no assertion) so there is a number to compare against. Do the A spike (about 4 days, one branch, no merge unless the corpus and mesh tests stay green) when a user's real work needs it: automotive-style surfacing, reflective parts, or a customer who looks at zebra stripes.

## Questions the product owner must answer

1. Who looks at zebra stripes or curvature plots on a fillet corner: is this a defect users report, or a power user's inspection?
2. Is parity with OCCT / SolidWorks-style fillets enough, or is "visibly smoother corners" a selling point?
3. Which parts matter: consumer-product styling (G2 matters) or machined mechanical parts (G1 is the norm)?
4. How much is a day of kernel time worth against the blend backlog (face-face fillets, conic and curvature fillets at corners are still "not started" in `docs/blend-corpus.md`)? Curvature-continuous stripe profiles and a G2 corner are the same feature family; doing one without the other may look odd.
5. Is a mesh-quality regression (needle triangles, shading scribble at the pinch) acceptable in exchange for the smoother surface? Our past answer was no.
6. Should the same treatment apply to the chamfer overhang corner (mitre patch, deliberately not tangent)? Probably not, but it should be decided before work starts.
