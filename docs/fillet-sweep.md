# Fillet / chamfer sweep against OCCT

`tools/fillet_sweep.mjs` generates seeded parametric cases, runs them through
the kernel, and compares the volume with an OCP replay
(`tools/fillet_sweep_ref.py`). Edges are selected by geometry (midpoint +
direction), never by index.

```sh
node tools/fillet_sweep.mjs --samples 40 --seed 7 \
  --wasm /path/to/work/kit/apps/cad/public/kernel/ck_wasm.wasm
node tools/fillet_sweep.mjs --samples 40 --seed 7 --update-baseline tools/fillet_sweep_baseline.json
node tools/fillet_sweep.mjs --samples 40 --seed 7 --check tools/fillet_sweep_baseline.json
node tools/fillet_sweep.mjs --samples 40 --seed 7 --only box_parallel4_8
```

The baseline stores case ids and classes only, plus the wasm hash, seed and
sample count, so a new wasm can be compared without mixing kernels.

## Result (kit wasm, seed 7, 40 samples/template)

**600 cases: 537 ok, 32 `blendOverflow`, 28 `wrongVolume`, 3 `blendFailed`,
0 `ref-fails`.** OCP built every reference, so all failures are ours.

| template | n | ok | blendOverflow | wrongVolume | blendFailed |
|---|---|---|---|---|---|
| box_edge1 | 40 | 40 | 0 | 0 | 0 |
| box_parallel4 | 40 | 30 | 10 | 0 | 0 |
| box_corner3 | 40 | 40 | 0 | 0 | 0 |
| box_all12 | 40 | 28 | 12 | 0 | 0 |
| cyl_top | 40 | 35 | 5 | 0 | 0 |
| cyl_bot | 40 | 35 | 5 | 0 | 0 |
| tube_outer | 40 | 40 | 0 | 0 | 0 |
| tube_inner | 40 | 40 | 0 | 0 | 0 |
| boss_foot | 40 | 40 | 0 | 0 | 0 |
| boss_top | 40 | 40 | 0 | 0 | 0 |
| hole_rim | 40 | 40 | 0 | 0 | 0 |
| chamfer_box | 40 | 40 | 0 | 0 | 0 |
| chamfer2_box | 40 | 40 | 0 | 0 | 0 |
| chamferA_box | 40 | 40 | 0 | 0 | 0 |
| var_box | 40 | 9 | 0 | 28 | 3 |

## Failure boundaries

| template | class | failing parameter range | all range |
|---|---|---|---|
| box_parallel4 | blendOverflow | radius 5.96 .. 24.62 | radius 0.98 .. 24.62 |
| box_all12 | blendOverflow | radius 5.96 .. 24.62 | radius 0.98 .. 24.62 |
| cyl_top | blendOverflow | radius 6.29 .. 24.42 | radius 0.55 .. 28.54 |
| cyl_bot | blendOverflow | radius 6.29 .. 24.42 | radius 0.55 .. 28.54 |
| var_box | wrongVolume/blendFailed | r1 0.21 .. 6.96; r2 4.29 .. 18.41 | r1 0.21 .. 6.96; r2 3.18 .. 18.41 |

The box failures start when the radius approaches half the shortest side
(the four parallel edges meet); the cylinder failures start when the rim
radius approaches half the height. OCCT succeeds on all of them, so these are
genuine large-radius gaps, not bad references.

See `docs/mimo-brief-fillets.md` for the ranked kernel tasks and exact repro
ids.

## Second seed (seed 11)

Same 600 cases, different parameters: **554 ok, 30 `wrongVolume`, 16
`blendOverflow`, 0 `blendFailed`, 0 `ref-fails`.**

| template | n | ok | blendOverflow | wrongVolume |
|---|---|---|---|---|
| box_parallel4 | 40 | 37 | 3 | 0 |
| box_all12 | 40 | 36 | 4 | 0 |
| cyl_top / cyl_bot | 80 | 74 | 6 | 0 |
| tube_outer / tube_inner | 80 | 78 | 2 | 0 |
| chamferA_box | 40 | 39 | 1 | 0 |
| var_box | 40 | 10 | 0 | 30 |
| all other templates | 280 | 280 | 0 | 0 |

The large-radius boundary moves with the random sizes (seed 7 had more
parallel/all-edge failures; seed 11 hits tube rims and one distance-angle
chamfer). `var_box` remains the only volume-law group. Baseline:
`tools/fillet_sweep_baseline_seed11.json`; `--check` 0 regressions.
