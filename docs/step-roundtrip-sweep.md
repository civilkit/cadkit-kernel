# STEP export round trip

`tools/step_roundtrip_sweep.mjs` exports each sweep shape with our `step`
request, reads it back with OCP (`STEPControl_Reader`), and re-imports the same
STEP with our `importStep`. It compares volume and area with the original
shape and checks `BRepCheck` on the OCP body and `validate` on our re-import.

```sh
node tools/step_roundtrip_sweep.mjs --samples 10 --seed 7 --json out.json
node tools/step_roundtrip_sweep.mjs --samples 10 --seed 7 --update-baseline tools/step_roundtrip_baseline.json
node tools/step_roundtrip_sweep.mjs --samples 10 --seed 7 --check tools/step_roundtrip_baseline.json
```

Tolerance is 1e-6 relative for volume and area on both paths. STEP files are
written to `/tmp/step_roundtrip/` for inspection; the baseline is class-only.

## Result (kit wasm, seed 7, 10 samples/template, 120 cases)

**100 ok, 20 `ocp-wrongArea`.**

| template | n | ok | failure |
|---|---|---|---|
| box, cyl, cone, sphere, torus, tube | 60 | 60 | |
| loft2 | 10 | 0 | `ocp-wrongArea` 5e-4 .. 8e-4 relative |
| loft3 | 10 | 0 | `ocp-wrongArea` 5e-4 .. 8e-4 relative |
| revolve, sweep_line, sweep_arc, sweep_L | 40 | 40 | |

Every OCP body was `BRepCheck`-valid. On the failing lofts, OCP volume and
our re-imported volume/area match the original to 1e-12 or better; only the
OCP **area** of the exported loft differs (the OCP read area is low). Our own
`importStep` of the same file reproduces both volume and area exactly.

STEP entities in the failing files: `CARTESIAN_POINT` (1360),
`ORIENTED_EDGE` (240), `EDGE_CURVE` (120), `VERTEX_POINT` (80), `EDGE_LOOP`
(80), `ADVANCED_FACE` (80), `AXIS2_PLACEMENT_3D` (60). These are the
B-spline/loft faces; no primitive template exposes the issue.

See internal notes (not published).
