# Tolerant geometry and import healing

Exporters (SolidWorks, others) write B-spline faces that sit a few 1e-3 mm away from the exact cylinders and
ellipses they meet, so no single edge position lies on both faces. OCCT absorbs this with a tolerance on the edge.
This kernel does the same, without loosening the exact path.

## Model

- `ck_topo::body::Edge::tol` and `Vertex::tol` (mm, default 0 = exact). Only `ck_heal` sets them; modelling
  operations create exact entities, `split_edge` copies the tolerance to both halves, `Body::append` clones it.
- `ck_check::validate` accepts an edge-to-face deviation, a vertex-to-curve distance and a loop gap up to
  `max(model tol, the entity's own tol)`. A tolerance above the cap is an error: `EDGE_TOLERANCE_EXCEEDED` /
  `VERTEX_TOLERANCE_EXCEEDED`. Cap default `ck_check::DEFAULT_TOL_CAP` = 0.05 mm, request field `healTolerance`.
- `validate` replies carry `toleranceUsed` (largest tolerance in the body) and, when it is not 0, a `notes` entry.
- Not propagated: booleans and blends build new exact edges (the result is exact where it is new). `fillet` and
  `chamfer` refuse a toleranced edge with the structured error `blendTolerantEdge`. Mass properties, tessellation
  (edges are shared polylines, so there are no cracks) and STEP export ignore the tolerance.

## Healing pass (`ck_heal::heal_with`, `HealOpts`)

Detection runs at the strict tolerance (min(`tol`, 1e-6)); repair is within `tol`. Entities already valid are never
touched, so healing a valid body is a no-op (gate metric `import.heal.noop`: no fix recorded, volume identical to
1e-12).

1. sew: near-miss vertex merge, gap closing, free-edge sewing, shell/lump merge (`sew`)
2. small: degenerate edges, sliver faces (`fixSmall`)
3. wires: chain, reorder, reverse coedges (`sew`); orientation: loop senses, flipped faces, inside-out solids
   (`fixOrientation`)
4. tolerant geometry (`tolerant`): per edge, the largest distance of its curve to the faces that use it (the
   validator's 8 samples, then 32 where it fails); above the strict tolerance and within the cap it becomes the
   edge tolerance (x1.05). Vertices farther than that from their edge curve end get the same.

## Requests

- `importStep {heal: "off" | {tol, sew, fixSmall, fixOrientation, tolerant, cap}, healTolerance}`: healing is on by
  default (`tol` 1e-4). The reply's `import.heal` has `fixed`, `remaining`, `toleranceUsed`.
- `heal {tag, tol, heal: {...}, dryRun, damage}`: the standalone pass; tolerant geometry is on unless `damage` is set.

## Not healed

Edge curves are not moved to a compromise position between the two faces (the tolerance covers the deviation);
sewing is limited to `tol`; a deviation above the cap is reported, not repaired.
