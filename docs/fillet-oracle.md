# Fillet oracle: a ground truth for rolling-ball rounds and chamfers

`tools/fillet_oracle.mjs` computes what a rolling-ball round (or a 45 degree chamfer) of the flat cap rim of an extruded profile must
look like, from the profile alone. It shares no code with the B-rep blend builders, so it can judge them: wrong shape, wrong volume,
a refusal where a ball does fit.

Files: `tools/fillet_oracle_core.mjs` (rasterise, distance transforms, height field; no dependencies), `tools/fillet_oracle.mjs`
(kernel comparison, `--selftest`, `--viewer-data`), `tools/fillet_oracle_plot.py` (matplotlib figures),
`tools/fillet_oracle_viewer.html` (browser viewer; built from `tools/fillet_oracle_viewer.template.html` by
`tools/fillet_oracle_build_viewer.mjs`, which inlines the core and the optional kernel layer).

## Theory

Profile P (closed, any number of loops), height H, flat cap at z = H, vertical walls. Rounding the cap rim with a ball of radius r.

- D(x,y) = distance from (x,y) to the boundary of P.
- Pr = {D >= r}: P eroded in the plane by r. Ball centres sit on Pr at height H - r. Tight bends (swallowtails) vanish by
  themselves, components can split or disappear, and Pr can be empty (no ball of that size fits).
- d(x,y) = distance from (x,y) to Pr (0 inside Pr).

The top surface above (x,y) is the highest point of any ball with its centre on Pr at height H - r:

    round r:   z = H - r + sqrt(r^2 - d^2)   if d <= r
               z = H - r                     if d > r

Columns with d > r are not touched by any ball, so they are cut down to H - r: that is the opening restricted to the cap direction.
The material below H - r keeps its vertical wall (the ball only rolls on the rim), so the surface is continuous at d = r
(sqrt(0) = 0) and tangent to the plateau z = H - r there. On a straight wall d = r - D, i.e. the textbook quarter circle.

Chamfer of distance c (45 degrees): the structuring element is the cone {z <= H - dist(., Pc)} with Pc = P eroded by c. On a straight
wall at depth w from it, the chamfer plane is z = H - c + w for w < c, and d = c - w, so z = H - d. In general

    chamfer c: z = H - min(d, c)

(d to Pc). At a convex polygon corner this gives a round-offset (the cone's footprint), not a mitre: a kernel chamfer that mitres
corners will differ there by design; for smooth outlines there is no difference. Other angles / unequal distances: scale d (z = H - min(d*tan(a), ...)).
The bottom rim is the mirror: z_bottom = H - z_top, same volume.

Not covered by the formula (stated limits): r > H (the ball then also touches the base, a different problem), thin walls where the
base interferes, variable radius, outlines with holes touching the rim (works for the cap formula, untested), non-prismatic parts (see stage 2).

## Implementation (stage 1)

- Profile: the cap boundary of a fine tessellation (chord 0.004 mm) of the kernel's own base extrusion, so sketch splines are
  taken as the kernel evaluates them. Rasterised on a grid (default cell 0.1 mm, even-odd scanline).
- D: exact distance to the polyline per cell (bucketed ring search), not a grid transform.
- d: Felzenszwalb squared EDT of the seed mask {D >= r} gives the nearest seed cells; each seed is moved to the edge of Pr along
  the gradient of D and d is the shortest distance to those edge points (sub-cell). Where the inward normal ray of the cell
  still reaches depth r (the point it ends at has D >= r) the distance is exactly r - D (lower bound attained). Quantisation checks:
  `node tools/fillet_oracle.mjs --selftest` (disc R = 20 vs the analytic surface): at cell 0.1 the error is below 1e-3 mm
  except in the last ~0.3 mm before the vertical wall (d close to r), where the sqrt makes height arbitrarily sensitive to a
  tiny shift: the largest error there was 0.94 mm for r = 19.9 round and 0.03 mm for chamfers. So the comparison statistics below
  also report the error on cells where the oracle slope is below 2 ("non-steep"), which is the meaningful one.
- Kernel: build base + `fillet` (all cap rim edges) with the wasm, `exportMesh` at chord 0.05, rasterise the up-facing triangles into a
  z-buffer (max z) on the same grid. Volumes from `massProps`; removed volume = base volume - result volume (so the area
  discretisation of the oracle cancels in the comparison; oracle removed = sum of (H - z) cell^2).

Run: `node tools/fillet_oracle.mjs --wasm <ck_wasm.wasm> --out <dir> [--only blob|six] [--chamfer] [--radii 1,2,3]`,
then `python3 -I tools/fillet_oracle_plot.py <dir> <png dir>`.

## Two oracle models

- **opening** (the rolling ball, above): ball centres on Pr. At a convex corner or a tight tip Pr has pulled back, so a flat ledge at
  z = H - r remains where no ball reaches.
- **swept-cut**: removed depth = r - sqrt(r^2 - (r - delta)^2) for a round (r - delta for a chamfer), delta = distance to the nearest
  boundary, nothing removed for delta >= r (`heightSwept`). It equals the opening wherever the nearest boundary point's inward
  normal ray still reaches depth r (smooth rims, away from tips and corners), and gives a mitre at convex corners and no ledge at tips.
  It is also the model the spline-tight-bend volume check used.

The viewer shows both volumes, a "Models" difference layer, and, where a kernel layer exists, which model the kernel matches better.

## Results

Wasm used: the newest gated wasm present, `_release-wasm/ck-full-gate/20261126-052013-db8f43c766` (kernel db8f43c7, the parent of the
spline-fillet merge a6b1a03d; a gated wasm for a6b1a03d itself did not exist yet). That wasm refuses a round or chamfer on a rim that
is one closed smooth spline (`blendUnsupported`: "a round on a smooth closed spline rim is not supported yet"), so the blob (9 points)
and the six-point spline have **no kernel result** here (the oracle side is computed and plotted). The tight-bend branch
(spline-tightbend, which builds those) needs its own wasm; a box job for it was submitted once and was still queued behind other jobs
when this was written, so those cases are not yet compared (rerun: `node tools/fillet_oracle.mjs --wasm <that wasm> --out <dir>`).

What could be compared: the **D shape** (case `d`: a 60 mm wide, 18 mm high elliptic arc as an interpolating spline through 7 interior points, closed by the
chord, extruded 20, round on the whole cap rim: the sharp corners are mitred by the kernel, a spline + line rim, as in
`blend_spline_corner.rs`), grid cell 0.1 mm. Removed volume (mm^3) and height error on cells where the oracle slope is below 2
("non-steep"; max / rms in mm):

| r | opening | swept-cut | kernel | kernel vs opening | kernel vs swept-cut | max / rms vs opening | max / rms vs swept-cut |
|---|---|---|---|---|---|---|---|
| 1 | 29.40 | 28.78 | 28.95 | -1.5 % | +0.58 % | 0.736 / 0.017 | 0.112 / 0.009 |
| 2 | 118.45 | 114.08 | 114.33 | -3.5 % | +0.22 % | 1.560 / 0.057 | 0.074 / 0.011 |
| 3 | 268.91 | 253.59 | 253.93 | -5.6 % | +0.14 % | 2.346 / 0.135 | 0.062 / 0.014 |
| 4 | 484.18 | 445.05 | 445.44 | -8.0 % | +0.09 % | 3.156 / 0.257 | 0.076 / 0.017 |
| 5 | 781.93 | 686.18 | 686.63 | -12.2 % | +0.07 % | 4.171 / 0.472 | 0.105 / 0.019 |
| 6 | 1203.02 | 974.56 | 975.07 | -18.9 % | +0.05 % | 5.252 / 0.856 | 0.084 / 0.018 |
| 7 | 1848.04 | 1307.58 | 1308.16 | -29.2 % | +0.04 % | 6.511 / 1.549 | 0.084 / 0.024 |

Reading: on the smooth arc the kernel, the opening and the swept-cut agree to a few hundredths of a millimetre. At the two sharp corners
the kernel does what the swept-cut model says (mitre, material kept), and the opening is wrong by design there (flat ledge at
H - r, a triangle at each end of the chord, 3 to 6.5 mm too low at r = 5 to 7). So **the corner behaviour of the kernel is
the swept-cut model, not the ball opening**; the swept-cut max error 0.06 to 0.11 mm is the kernel's chord tolerance (0.05) plus oracle
quantisation, and the remaining small bias (kernel volume 0.04 to 0.6 % above the swept-cut volume, falling with r) is the tessellation sagitta
of the mesh volume and the staircase of the oracle at the wall. The raw `maxAbs` in the JSON (10 to 16 mm) is at a handful of cells on the
vertical wall (mesh crack / sub-cell wall position): it is not a surface error, which is why the non-steep figures are the ones quoted.
Chamfer on this D: the kernel refuses (`blendUnsupported`, same wasm), so only the oracle values exist (viewer shows the chamfer oracle without a kernel layer).

Self-test (disc R = 20, exact answer known), cell 0.1: height error < 1e-3 mm away from the wall for round r = 1, and 0 to 0.03 mm for chamfers; for rounds the
largest error is 0.3 to 0.9 mm in the last ~0.3 mm before the wall (sqrt singularity), zero elsewhere.

Where the oracle sees the blob (9-point spline, H 25; no kernel result with this wasm; opening model): eroded area at r = 1 / 3.7 / 7 /
15 / 24.9 is 11579 / 10287 / 8782 / 5563 / 2463 mm^2 of 12070 mm^2; oracle removed volume 105 / 1444 / 5305 / 26614 / 78055 mm^3. At r = 15 the round
continues around the whole wall; at the thin left tip and the right end Pr has retreated (tip radius about 1.3 to 1.8 mm, depending on the spline model), so there the
opening leaves flat plateaus at z = H - r (z = 10 at r = 15), 20 to 25 mm long at the left tip and small at the right end, joined to the rounded region
tangentially. The swept-cut model has no plateaus (it removes depth from the distance to the wall alone); the viewer shows the two side by side (Models layer). Which model
a CAD system follows at a tight tip is not established by this run (the D comparison above decides it for sharp corners only).

## Stage 2 (plan only): general solids, any picked edges

A 3D voxel oracle for an arbitrary solid S and a selection of edges E:

1. Voxelise S (inside/outside) from the exported mesh by winding number or ray parity on a grid of 0.1 to 0.25 mm (a few hundred
   cubed cells at most; crop to the bounding box of the selected edges plus a margin of 2r).
2. Opening by a ball of radius r: S_r = {x : dist(x, outside) >= r} (3D EDT of the complement, exact Felzenszwalb in three passes),
   then the opened solid O = dilation of S_r by the ball = {x : dist(x, S_r) <= r}. For a convex edge, S \ O is exactly the material a
   rolling ball removes. For a concave edge (a fillet that adds material) use the closing: C = erosion of the dilation, and the added
   material is C \ S. Convex and concave edges of one pick are handled by the two operators on their own neighbourhoods.
3. Only the picked edges: take the neighbourhood N = union of balls of radius ~1.5 r (a tube around the polyline of each picked edge,
   closed with the end caps where the edge ends in a vertex) and set result = (S outside N) union (O inside N for convex, C inside N for
   concave). Edges of S that are not picked keep their sharp state because they lie outside N; the transition at the boundary of N is at
   material that the opening does not change (more than r from the edge), so it is seamless away from other edges. Picks that stop in
   the middle of a smooth chain need a tangent-continuation rule; compare only at the cells farther than ~r from the pick's ends.
4. Chamfer: replace the ball with the cone/wedge structuring element: the equal-distance chamfer of distance c on a convex edge is the
   opening-like removal by the half-space-bounded wedge; implemented as the 3D analogue of H - min(d, c): erode by the L2 distance c
   to get S_c, then keep the points with dist(x, S_c) <= c measured by the cone's Minkowski metric, i.e. the sum over the
   edge's two faces of the in-plane distance (a "diamond" EDT: the L1-like distance across the two face normals). Unequal
   distances and angles: anisotropic cone; do these only after the equal chamfer agrees with the kernel on the corpus.
5. Compare: mesh the kernel result (chord 0.05), voxelise the same way, report (a) volume difference, (b) the symmetric difference
   volume |K xor O| and (c) the two-sided Hausdorff distance sampled on the surfaces (distance from kernel mesh vertices to the
   oracle surface via the 3D distance field of O, and from oracle surface points back). Tolerances: a few cells (about 0.3 mm)
   for the Hausdorff measure, 0.5 percent for the volume.

Running it across the corpus (`tools/blend_limits` cases, `fillet_sweep` seeds, `blend_corpus`): the corpus cases are already
(part, picks, radius) triples, so the harness is a loop: regen base, find the picked edges, run the kernel result, run the oracle.
Per case classify as
- `refused-but-fits`: the kernel refuses (any error code) while the oracle says a ball of that size fits along the picked edges
  (the picked edges' tube N still contains a non-empty set of ball positions: the cell set {dist to outside >= r} within r of the edge
  is non-empty, and the opened solid differs from S by a connected sheet along the whole pick). This is the list of refusals worth
  fixing; the oracle also gives the largest r that fits (bisection on r), to print next to the refusal's own limit.
- `refused-correctly`: the kernel refuses and the oracle says no ball fits.
- `built-differs`: kernel built, but |volume difference| or symmetric difference above tolerance. Sort by the symmetric difference.
- `built-ok`.
Cost: one 3D EDT of ~10^7 cells is about a second in node; the corpus (a few hundred cases) runs in minutes, no cargo, and can run
beside `fillet_sweep` using the same wasm. A voxel oracle is not exact at the cell scale, so every flag below about 2 cells is reported
as noise and only larger ones are listed; the corpus gate would key on `refused-but-fits` and `built-differs` counts against a baseline file,
as `fillet_sweep_baseline.json` does.

## Viewer

Open `tools/fillet_oracle_viewer.html` in a browser (no server needed; or `python3 -m http.server`). Radius slider, round / chamfer,
top / bottom rim, presets (blob, six-point spline, D shape, notched rectangle), draggable control points on the 2D canvas (click on the
curve adds a point, shift-click or double-click removes, alt-click toggles a corner), live oracle height map and a lit 3D
surface (own WebGL renderer, drag to rotate). Readouts: removed volume, eroded-region area, tightest convex bend radius, tip radius (the
first r at which a boundary point's inward normal stops being a shortest path, marked orange on the slider), inscribed radius.
A kernel layer with a difference map exists for the presets and radii that were built (blue ticks on the slider); the
file built with a kernel layer is produced by `fillet_oracle.mjs --viewer-data data.json` then
`fillet_oracle_build_viewer.mjs out.html data.json`.

### Optional WebGPU path (viewer)

`tools/fillet_oracle_gpu.js` (inlined into the viewer) reproduces, in f32 WGSL, the CPU reference's mask + exact distance to the outline
(one thread per cell, loop over the segments) and the per-radius erosion (seed mask, two squared EDTs with nearest-seed tracking, edge-point
refinement). Detection is layered: `navigator.gpu`, a non-fallback adapter (a software adapter counts as no GPU), a device with the buffer
sizes needed (else CPU), a startup self-test (65x65 disc against the CPU: max height error < 1e-3 mm, volume < 1e-4), then per job validation /
out-of-memory error scopes and the `device.lost` promise; any failure disables the GPU and the recompute falls back to the CPU with a message.
`?gpu=off` disables it from the start. Compute = Auto uses the GPU from about 512x512 cells (262k) up. "Verify GPU" (and every 10th GPU
recompute) recomputes a random 32x32 tile on the CPU (window of r + 4 cells around it, brute-force distances) and shows the differences.
Measured in Chrome on an Apple GPU, blob, 1024x513 cells: outline grid 36 to 42 ms on the GPU vs 1.1 s on the CPU; per-radius erosion 17 to 75 ms vs 142 to 152 ms (includes
readback); the rest of a recompute (heightfield, volumes, 2D and 3D drawing in JS) is 150 to 250 ms. Full-grid GPU vs CPU: 0 mask mismatches, max height error 9e-5 mm
(cells at least half a cell from the wall; 9e-4 mm in the very last wall cells where the sqrt amplifies shifts).
