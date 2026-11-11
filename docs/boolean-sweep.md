# Random boolean sweep against OCCT

`tools/boolean_sweep.mjs` generates seeded primitive pairs (box, cylinder,
cone, sphere, torus) with random placement, a set of deliberately hard
relations, union/cut/intersect, and a three-body chain. Volumes are compared
with an OCP replay (`tools/boolean_sweep_ref.py`, `BRepAlgoAPI_Fuse/Cut/Common`).

```sh
node tools/boolean_sweep.mjs --samples 40 --seed 7 --json out.json
node tools/boolean_sweep.mjs --samples 40 --seed 7 --update-baseline tools/boolean_sweep_baseline.json
node tools/boolean_sweep.mjs --samples 40 --seed 7 --check tools/boolean_sweep_baseline.json
```

The baseline stores case ids and classes plus the wasm hash, seed and sample
count. `okNoEffect` and `okEmpty` are the expected cut/intersect outcomes
when the operands do not overlap; they are not failures.

## Result (kit wasm, seed 7, 40 samples/relation, 320 cases)

**146 ok, 80 `okNoEffect`, 65 `okEmpty`, 22 `wrongVolume`, 5 `invalid`,
2 `booleanFailed`, 0 `ref-fails`.**

Failures: 29/320 overall (9.1%), or 29/174 of the cases whose operands
actually overlap (16.7%).

| relation | n | ok | wrongVolume | invalid | booleanFailed | expected no-effect/empty |
|---|---|---|---|---|---|---|
| overlap | 40 | 10 | 19 | 5 | 2 | 4 |
| coincident | 40 | 14 | 0 | 0 | 0 | 26 |
| tangent | 40 | 14 | 0 | 0 | 0 | 26 |
| contained | 40 | 40 | 0 | 0 | 0 | 0 |
| sliver (1e-3 .. 1e-1 gap) | 40 | 14 | 0 | 0 | 0 | 26 |
| angled cylinders (5 .. 30 deg) | 40 | 40 | 0 | 0 | 0 | 0 |
| cone on plate (R 15 .. 26) | 40 | 14 | 0 | 0 | 0 | 26 |
| chain3 | 40 | 0 | 3 | 0 | 0 | 37 |

The hard relations are all exact or correctly refused; the failures are
concentrated in the random overlapping pairs and the three-body chain.

## Failure groups

| group | cases | class | primitives | notes |
|---|---|---|---|---|
| torus involved | 10 | wrongVolume | torus+torus, box+torus, cone+torus, cyl+torus | rel 1.7e-3 .. 9.5e-1 |
| cone+cone | 5 | wrongVolume | cone+cone | rel 1.2e-1 .. 3.1e-1 |
| cone+cylinder | 4 | wrongVolume | cone+cyl | rel 1.3e-3 .. 1.9e-1 |
| sphere+torus / sphere+cone | 4 | invalid / booleanFailed | sphere+torus, cone+sphere | `EDGE_OFF_SURFACE`, `booleanFailed` |
| sphere+cylinder union | 1 | invalid | cyl+sphere | `TESSELLATION_OPEN` |
| chain3 (box+cone+C) | 3 | wrongVolume | box+cone+torus / +sphere / +cone | rel 1.6e-2 .. 3.1e-2 |

Smallest failing sets:

- `overlap_7` sphere r=7.33593 intersect torus R=11.6964, r=4.25803 at
  `[-10.4253, 10.3503, -3.80285]`: kernel 20.470, OCP 437.606 (95% low).
- `overlap_38` cone `r1=9.50592, r2=7.61848, h=45.2023, draft=4.3004` cut by
  cone `r1=19.8773, r2=2.23714, h=26.5154, draft=2.83819`: rel 3.05e-1.
- `overlap_3` sphere r=17.7674 cut by torus R=20.0724, r=4.89090:
  `EDGE_OFF_SURFACE`.
- `overlap_19` box 45.5663x39.5553x20.4268 union sphere r=23.7117:
  `booleanFailed`.
- `overlap_39` cylinder r=8.11865 h=31.9531 union cylinder r=14.4226
  h=22.6069: `TESSELLATION_OPEN`.

See `docs/mimo-brief-boolean.md` for the ranked tasks.
