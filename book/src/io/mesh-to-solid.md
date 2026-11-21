# Mesh to solid

`meshToSolid` turns a triangle mesh (an STL, an OBJ, a mesh the app imported) into a solid the kernel can model on: fillet,
drill, boolean, export as STEP. It is an import: the body goes into the tag, the reply carries `body`, `errors` and the report in
`import`.

```json
{"type":"meshToSolid","tag":"m","name":"bracket","positionsF32":"<base64>","indicesU32":"<base64>"}
```

## What it does

1. Vertices closer than `weldTol` are welded; triangles with two corners on one vertex and duplicate triangles are dropped.
2. Every edge must be used by exactly two triangles. Otherwise the mesh is refused: `meshNotWatertight` lists the gaps (where they
   are), `meshNonManifold` the edges used more often. Holes are not filled unless the request asks for it (`opts.close`, next
   section).
3. Triangles are turned to agree with their neighbours, each closed piece outward; a piece inside another is a void.
4. Coplanar neighbouring triangles become one planar face (with its holes); straight edges keep only their corners.
5. Smooth regions of facets are fitted with a cylinder, a cone or a sphere. A region becomes one exact face when its boundary
   with every neighbour is an exact circle or straight line on both: the edge between a hole and the face it is drilled into is
   a circle, the edge between a round and a tangent wall a line. Regions that do not fit stay faceted.
6. The result is validated. Anything that does not validate goes back to facets.

## Closing an open mesh

A mesh from a scan or a careless export is often not closed: triangles that were never welded, cracks along a border, a few
missing triangles. `opts.close` runs a stage before the watertight check. It is off by default, because it invents geometry:
a caller that does not ask gets today's refusal. Design:

1. **Gap welding.** Only vertices on open edges take part. Those within the tolerance (`close.tolerance`, default 1e-4 of the
   bounding diagonal) and sharing no triangle become one vertex, at the centroid of the group. A merge that would put three
   triangles on one edge is refused (`meshGapAmbiguous`, reason `weldNonManifold`) rather than guessed.
2. **Cracks and T-junctions.** A boundary vertex within the tolerance of an open edge, away from its ends, splits the triangle
   on that edge, so that the edge on the other side finds its twin.
3. **Holes.** The remaining open edges are traced into loops. A loop with at most `maxHoleEdges` edges (default 500) and a
   filled area of at most `maxHoleArea` (default 0.25) of the surface of the mesh is filled, with triangles that run opposite to
   the neighbours. A loop within `planarTol` (1e-3 of its diameter) of its best-fit plane gets a planar cap by ear clipping (best
   shaped ears first, so the cap becomes one planar face when it is coplanar with its surroundings). Other loops of up to 50
   edges get the minimum-area triangulation (dynamic programming, no chord that is already an edge of the mesh); longer ones
   are ear-clipped in the best-fit plane. No smoothing or fairing is done: a hole in a curved surface is closed flat or
   with the least area, not with the curvature of the surroundings.
4. **Checks.** The fill must not cross its own loop (seen along the best-fit plane), itself, another fill, or the rest of the
   mesh (triangle-triangle tests over a grid); otherwise that hole is refused with the reason and where it is. Several holes
   are all checked and listed.

Orientation of the fill follows the neighbours; the later stages (orientation, merging, refit) then run as for any closed mesh,
so a cylinder whose caps were missing still becomes one analytic cylinder face and two planes. Not attempted: repairing
non-manifold edges (kept as `meshNonManifold`), fairing of the fill, holes that touch at a vertex (`pinchedLoop`).

Options (`opts.close` is `true` for the defaults, or an object): `tolerance` (mm), `fill` (false: weld and stitch only),
`maxHoleEdges`, `maxHoleArea`, `planarTol`. The reply's `import.closing` says what was done:

```json
"closing":{"tol":0.0017,"openEdges":4,"mergedVertices":0,"droppedTriangles":0,"stitched":0,"reoriented":0,
  "filledHoles":1,"filledTriangles":2,"filledArea":100,
  "holes":[{"edges":4,"area":100,"planar":true,"maxPlaneDev":0,"triangles":2,"method":"ear","at":[5,5,10]}]}
```

| Situation | Result |
|---|---|
| vertices on a gap closer than the tolerance, no shared triangle | welded (`mergedVertices`) |
| boundary vertex on the interior of an open edge | edge split (`stitched`) |
| small planar hole | planar cap (`method: ear`) |
| small non-planar hole | minimum-area fill (`minArea`), long loop: `earProjected` |
| hole above `maxHoleEdges` or `maxHoleArea` | `meshHoleTooLarge {reason: edges / area}` |
| loop crosses itself; cap would cross itself, another cap or the mesh | `meshHoleSelfIntersects {reason: loopCrossesItself / fillIntersectsItself / fillIntersectsFill / fillIntersectsMesh / noTriangulation}` |
| weld makes an edge non-manifold; holes sharing a vertex; open chain | `meshGapAmbiguous {reason: weldNonManifold / pinchedLoop / brokenLoop}` |
| non-manifold edges | `meshNonManifold` as before |

Every refusal has `message`, `hint`, `positions` (and `holes`, one entry per hole that failed).

## Reading the report

`faceTypes` counts the faces by kind; `analytic.faceted` the facets of curved surfaces that were not refit; `analytic.fitted` the
refit faces with their radius or angle and their residual; `deviation` how far the result is from the mesh (sampled both ways:
expect about the chord height of the tessellation that made the mesh); `volume.relative` the volume change (0 when nothing was
refit, the chord error of the mesh when something was).

## Previewing a closing: `opts.dryRun`

`opts.dryRun: true` runs the conversion up to the closing stage and the watertight check and stops: no body is built, the tag is
not touched. The reply is `{dryRun: true, errors, import}` with no `body`. `import` carries the counts of the first stages and
`closing`, whose `holes[]` each have `fill: {positions: [x,y,z,...], indices: [a,b,c,...]}`, the triangles `opts.close` would
invent for that hole (corners shared inside one hole are listed once). A mesh the conversion would refuse is refused the same
way (`meshNotWatertight`, `meshHoleTooLarge`, ...), so a dialog can show the gaps or the fills before the user commits.

## As a feature: `meshToSolid`

`meshToSolid` as a request leaves the result in a tag. To keep it in the document, add an `meshToSolid` feature: it holds the mesh
in the part's top-level `blobs` table, as `importStep` and `importBody` hold their files, converts it on every regeneration, and
its result is cached so that reopening the document does not convert again.

```json
{"type":"meshToSolid","id":"F1","name":"Bracket scan","blob":"<sha-256 hex>","opts":{"close":true},"replace":false,"source":{"name":"bracket.stl","format":"stl"}}
```

| Field | Meaning | Default |
|---|---|---|
| `blob` | key into `blobs`; the entry is the mesh as base64 text and the key the SHA-256 of that text (checked on every run) | required |
| `opts` | the options of the request: `close`, `weldTol`, `planeTol`, `angleTolDeg`, `maxTriangles`, `maxFaces`, `analytic`, `deviation`, `tol`, `fitTol`, `maxSeconds`, `decimate` | the request defaults |
| `replace` | remove the bodies so far first; otherwise the solid joins beside them | `false` |
| `preview` | dry run: build nothing, add nothing, report what `close` would add | `false` |
| `body` | name of the solid | none |
| `source` | `{name, format}` (`stl`, `obj`, `glb`, `3mf`, `skp`): where the mesh came from; informational (the panel shows it), never read by regen, not part of the cache key | none |

The feature is not an importer and knows no file format: the app reads STL / OBJ / GLB / 3MF / SketchUp and hands the kernel triangles. Faces are named `<id>.I1F<n>`. The blob is one of two layouts, each optionally deflated (raw DEFLATE), then base64:

1. the app's packed mesh `CKM2` (`packages/io/src/meshPack.ts`): `'CKM2'`, flags (bit 0 lossless), pad, nVerts, nTris, step (f64),
   section lengths, then zigzag varint vertex differences on a grid of `step`, and varint triangle corner distances. The kernel
   inflates with the decoder `ck-sldprt` already carries (no new dependency).
2. raw: `u32 nVerts, u32 nTris, f32 x y z (nVerts times), u32 a b c (nTris times)`, little endian, exact length.

The payload is the one the app's reference meshes store (`fmt: 2`, one deflated block; its material section is ignored), so one stored copy can serve both: put the base64 text under `blobs[<SHA-256 of the text>]` and let the document's mesh record and the feature name the same key. The mesh is read in the frame of the blob; a record's own move / rotate / scale is not applied by the kernel.

**Result.** On every successful run (also from the saved cache) the feature carries the warning `meshToSolidReport {detail, report}`:
`report` is the `import` object of the reply (`faceTypes`, `analytic.fitted` / `faceted`, `deviation`, `volume`, `closing` with
`holes[].at`), so a panel can show it after a reload. It is informational. With `preview: true` the warning is
`meshToSolidPreview {detail, report}` with the dry-run report and the `fill` triangles.

**Errors.** A refusal of the conversion is the feature's error with the same code and params: `meshNotWatertight`, `meshNonManifold`,
`meshHoleTooLarge`, `meshHoleSelfIntersects`, `meshGapAmbiguous`, `meshTooLarge`, `meshDegenerate`, `meshBadInput`; params `detail`
and `message` (the same text), `hint`, and the refusal's data (`positions`, `holes`, counts) for highlighting. `meshToSolidInvalid
{detail, hint}` is a feature that is not well formed or a blob that is missing, does not match its key or is not base64.

**Cache.** The converted body and its report are saved in the document's `built` table like those of `importBody` (the
`builtExport` request hands them out; the key covers the blob, the options, the feature id, the kernel version and the format).
A changed blob or changed options miss and convert again.

## Scans

A scan of a free-form object has noise and no flat panels, so nothing merges: every triangle stays a face, and a 100k-triangle
scan is refused as `meshTooLarge` (or converts slowly with `maxFaces` raised: a toy van of 120000 triangles gives 115000 faces and
takes a minute). Decimate first, with `opts.decimate` on `meshToSolid` or with the `meshDecimate` request:

```json
{"type":"meshToSolid","tag":"van","positionsF32":"...","indicesU32":"...","opts":{"close":true,"decimate":{"tol":0.05,"targetTriangles":15000}}}
```

How to choose `tol`, the largest distance the decimated mesh may have from the scan:

- Above the noise. If the scan's noise is 0.02 mm, a `tol` of 0.02 asks the decimator to keep the noise; 0.05 to 0.1 lets it
  average the noise away, which is what you want. A `tol` below the noise gives little reduction.
- Below what you model on top. The solid is the decimated mesh, so a cavity offset, a shell or a fit clearance is measured from
  it: keep `tol` at a fifth of the smallest offset you will apply, or less (a 0.4 mm shell around a scan: `tol` 0.05).
- `targetTriangles` is a cap on the size, `tol` the cap on the error: the decimation stops at whichever comes first, and the reply
  says which (`stoppedBy`). 10000 to 20000 triangles convert in seconds and keep a scan's shape; set no target to get the
  smallest mesh the tolerance allows.
- `keepAngleDeg` (for example 30) is for parts that have sharp edges as well as noisy surfaces (a scan of a machined part): those
  edges and corners are kept exactly instead of being rounded within `tol`.

The reply's `deviation.max` is measured on the finished mesh (not an estimate of the algorithm) and is at most `tol`; if it
cannot be, the mesh is returned unchanged. Two sheets of the surface that are a hair apart (a tyre and its wheel arch) are not
pulled together. A decimated mesh converts like any mesh: `deviation` of the import compares the solid with the decimated mesh,
`import.decimation.deviation` the decimated mesh with the scan. A `meshToSolid` feature takes `opts.decimate` too: it is part of the cache key, the report of the decimation is in `meshToSolidReport.report.decimation` (and in the preview), and the one-line `detail` says how far the mesh was reduced.

## Time of a conversion

The analytic refit is bounded by `opts.maxSeconds` (alias `refitSeconds`; default 3 s of a native machine, 0 for no limit): the seed loop is counted in
work units (`UNITS_PER_SECOND`, 1.5e7), so the limit holds on a host without a clock, and where the host has a clock it is also a
time limit. When it runs out the seeds not yet tried stay faceted and the report says `analytic.budgetExhausted: true` with
`analytic.fittedFraction`. A scanned, free-form mesh has nothing to fit: without the limit the refit of 50000 triangles took 40 to 50 s (a smooth blob
100 s), now about the limit. A body of 5000 faces or more that is the mesh's own triangles skips what would cost seconds (below).
Measured on the CI box (a loaded 16-core machine, `fast` profile, so a laptop in release is faster), noisy free-form scan,
default budget, whole `meshToSolid` with deviation on:

| Triangles | Total | weld | orient | regions | faces | refit | validate | deviation |
|---|---|---|---|---|---|---|---|---|
| 50000 | 4.8 s | 0.18 s | 0.05 s | 0.10 s | 0.16 s | 3.3 s (budget) | 0.9 s | 0 |
| 200000 | 10.1 s | 0.57 s | 0.25 s | 0.49 s | 0.67 s | 4.8 s (budget) | 3.3 s | 0 |

Before the quick validation and the skipped deviation sampling the same meshes took 13 s and 44 s. `stagesMs` in the report (with a clock) lists the time of each stage. A body of more than 5000 faces that is the mesh's own triangles (nothing refit) gets the quick validation (topology and bounds, no loop-crossing or volume check; the report says `validation: "quick"`) and no deviation sampling (`deviation.note` says it is exact); a refit body of more than 20000 faces is not sampled either.

## Cancelling and time

A long conversion cannot be interrupted from outside: the kernel is a single-threaded wasm module in a worker, and the app's
Cancel ("Rebuilding model...") terminates the worker and starts a new one. Inside the kernel a conversion polls the regen's time
budget (`opts.featureTimeMs`, `regenTimeMs`) between its stages (weld, close, orient, merge, build, refit, validate, deviation)
and stops with `meshBudgetExceeded`, reported by a regen as `featureBudgetExceeded` / `regenBudgetExceeded`. The standalone
request has no clock and runs to the end.

## Limits

Tori, free-form surfaces, rounded box corners (sphere patches between three rounds), cylinders cut obliquely and crossing
cylinders stay faceted. A noisy mesh (a scan) merges and fits only with tolerances above its noise (`planeTol`, `angleTolDeg`,
`fitTol`). Above `maxTriangles` (200000) or `maxFaces` (50000) the mesh is refused rather than converted slowly.
