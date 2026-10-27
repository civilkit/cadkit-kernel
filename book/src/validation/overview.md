# Validation overview

This chapter explains what "valid" means for a body in this kernel, the layers that check it, and how to use them together. The other chapters of this part cover each layer in detail.

## What valid means

A body is valid when the validator (`ck-check`) reports no issue for it. That is a specific claim, built from these tests:

- **Closed, oriented skin.** Every edge is used by exactly two faces, and the two uses run in opposite directions. No edge is free, none is shared by three faces, none is unused.
- **Connected shells.** The faces of a shell are reachable from each other through shared edges, and the Euler characteristic `V - E + F - (L - F)` of each connected piece is even and at most 2.
- **Positive solids.** Each lump has a positive signed volume (its normals point outward) and exactly one outer shell. Voids are inner shells.
- **Closed loops.** The coedges of every loop join head to tail, and the loops of a planar face do not cross themselves or each other.
- **Consistent geometry.** Each vertex lies on the end of its edge's curve, each edge lies on the faces that use it, and no edge is shorter than the tolerance. "Lies on" means within a tolerance that scales with position; see [tolerances](tolerances.md).
- **A watertight display mesh** (in the `validate` request): every face has triangles, the mesh covers each analytic face's area, and no mesh edge is used by only one triangle.

It does not say that two non-adjacent faces do not intersect, that a free-form surface is well behaved, or that the model has the dimensions you meant. The full list of what is and is not checked is in [topology and geometry checks](checks.md).

## The layers

| layer | when it runs | what it tells you | chapter |
|---|---|---|---|
| Construction checks | Inside the operations, on every regeneration. | A boolean checks the topology of its result and its volume against the operands. A loft, a shell and a move-face validate the body they built. A failure becomes a feature error and the body stays as it was. | [operations](../kernel/operations.md) |
| Feature errors and warnings | In every `regen` reply: `errors` and `warnings`. | Which feature could not be built and why, with codes, parameters and hints. | [diagnostics](diagnostics.md) |
| The `validate` request | On demand, on the body of a tag. | The independent B-rep checks, then the mesh check. `ok` and a list of `{code, entity, detail}`. | [validate](validate.md), [checks](checks.md) |
| Tessellation checks | Part of `validate` (`mesh: false` skips them). | That the display mesh has no holes, which is a witness for the closed shell. | [tessellation checks](tessellation.md) |
| Expectations | On demand: `check {expect}`. | That measured values (volume, counts, bounding box, validity) match what you intended. | [analysis](analysis.md) |
| Healing | On import (`importStep`), on demand (`heal`), or per feature (`opts.autoHeal`). | Repairs what the validator reported, within a tolerance, and reports every repair. | [healing](healing.md) |

The layers are independent. Construction checks catch an operation that cannot produce a sound body. Feature errors say which input was wrong. `validate` re-examines the final body with code that shares nothing with the operations except the data structure, so a defect that an operation missed is still found. Expectations check the model against intent, which no structural check can do.

## How to use them together

A client that builds or edits a part, and an agent in particular, can follow this loop.

1. **Regenerate.** Send `regen` and read `errors`. If it is not empty, fix the first feature in document order and regenerate; later errors are often consequences. See [feature errors](diagnostics.md).
2. **Validate.** Send `validate` for the same tag. An empty `issues` list means the body is a valid closed solid. If there are issues, the `entity` field points to faces, and the face name tells the feature that made it (`F4.S.e1` came from `F4`).
3. **Separate mesh from B-rep.** If the only issue is `TESSELLATION_OPEN`, repeat with `mesh: false`. Clean means the B-rep is sound and one face did not mesh; see [tessellation checks](tessellation.md).
4. **Check intent.** Send `check` with the expectations you care about: `valid`, `volume`, `faces`, `holes`, `bbox`. Measure further with `measure`, `section` and `probe` where the numbers matter.
5. **Repair imports.** For a body that came from a file, validate first. If it fails, `heal` (a dry run first, if you want to see the repairs), then validate again. Imported geometry that is a few thousandths of a millimetre off its faces is accepted by recording a tolerance on the edge, not by moving it.

A sequence on a 40 x 20 x 10 block with a hole:

```text
regen            -> errors: []
validate         -> ok: true, issues: [], toleranceUsed: 0
check valid, volume equals 7497.35 (relTol 1e-6), holes equals 1   -> 3 passed
```

and on a block imported without its top face (`heal: "off"`):

```text
validate         -> ok: false, SHELL_OPEN x4, EULER
heal (dry run)   -> fixed: [], remaining: the same 5 findings   (nothing to sew the free edges to)
```

## Which tolerance applies

Modelling uses absolute tolerances of `1e-6` mm. `validate` uses the same default and scales it with position, so a body built here passes with a wide margin. The display mesh and its checks use tolerances relative to the size of the body. Edges and vertices of imported bodies can carry a tolerance of their own, up to a cap of 0.05 mm. [Tolerances](tolerances.md) has the numbers.

## Guarantees and their limits

- Bodies the kernel builds are meant to be valid in the sense above. In the repository's crash-fuzz run of 5000 random documents there were no traps and 24 invalid results (0.5 percent), on near-degenerate mirrored or revolved bodies.
- Valid does not mean accurate: a mesh is a display approximation, and an imported body may carry tolerances.
- Because a failed feature leaves the body unchanged, a body with errors in `errors` can still validate clean. A clean `validate` is no evidence that every feature ran.

## See also

- [Topology and geometry checks](checks.md)
- [The validate request](validate.md)
- [Feature errors and diagnostics](diagnostics.md)
- [Healing](healing.md)
- [Robustness under edits](../quality/robustness.md)
