# Tessellation accuracy sweep

`tools/tess_sweep.mjs` compares the display mesh in the `regen` reply with the
exact `massProps` values. It sums triangle areas and signed tetrahedron volumes
from `body.mesh.positions` / `triFace`, and groups by template and by the
surface kinds present on the body.

```sh
node tools/tess_sweep.mjs --samples 20 --seed 7 --json out.json
node tools/tess_sweep.mjs --samples 20 --seed 7 --update-baseline tools/tess_sweep_baseline.json
node tools/tess_sweep.mjs --samples 20 --seed 7 --check tools/tess_sweep_baseline.json
```

A mesh whose volume or area is more than 0.1 % (1e-3 relative) from exact is
classified `meshVolume` / `meshArea`.

## Result (kit wasm, seed 7, 20 samples/template, 180 cases)

**96 ok, 84 `meshVolume`.**

| template | n | ok | meshVolume | max rel volume | max rel area | surface kinds |
|---|---|---|---|---|---|---|
| box | 20 | 20 | 0 | 9.3e-8 | 6.1e-8 | plane |
| cyl | 20 | 0 | 20 | 4.2e-3 | 1.4e-3 | plane, cylinder |
| cone | 20 | 11 | 9 | 1.5e-3 | 1.1e-3 | plane, cone |
| sphere | 20 | 20 | 0 | 9.7e-4 | 4.3e-4 | sphere |
| torus | 20 | 0 | 20 | 8.2e-3 | 1.8e-3 | torus |
| tube | 20 | 5 | 15 | 1.4e-3 | 8.8e-4 | plane, cylinder |
| loft2 | 20 | 20 | 0 | 9.2e-4 | 6.1e-4 | spline, plane |
| loft3 | 20 | 12 | 8 | 1.7e-3 | 7.3e-4 | spline, plane |
| revolve | 20 | 8 | 12 | 2.7e-3 | 9.0e-4 | plane, cone |

All failures are volume errors; the mesh area stays under the 0.1 % bar in
every case. The worst surface is the torus (0.8 % volume), then the cylinder
(0.4 %). Boxes, spheres and two-section lofts pass; curved-surface chord
height at the default tolerance is the cause.

See internal notes (not published).

## Exact-cone wasm (`c917f6ac`)
`loft2` now carries a true `cone` face. Ten cases fail: volume 7.9e-4 ..
1.71e-3, area up to 1.12e-3, cone face 1706 .. 4592 triangles, chord
deviation 0.017 .. 0.022 mm. The old B-spline version of the same shapes had
576 triangles/face and 5.55e-4 volume error. The plain cone primitive behaves
the same (9/20, max 1.51e-3), so the exact cone face needs finer
triangulation; the 0.1 % threshold is not the problem.
