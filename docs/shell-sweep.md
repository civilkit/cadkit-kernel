# Shell / thicken sweep against OCCT

`tools/shell_sweep.mjs` generates seeded shell and thicken cases, runs them
through the kit wasm, and compares the volume with an OCP replay
(`tools/shell_sweep_ref.py`). Face removals are geometric (`top`, `side_ymin`);
the reference selects the same faces by position.

```sh
node tools/shell_sweep.mjs --samples 40 --seed 7 --only box_top --json out.json
node tools/shell_sweep.mjs --samples 40 --seed 7 --only box_top \
  --update-baseline tools/shell_sweep_baseline.json
```

Run one template at a time. A single process that walks every template can
stop responding after the first few hundred requests; the per-template runs
above are the reliable form. The baseline stores case ids and classes plus
the wasm hash, seed and sample count.

## Result (kit wasm, seed 7, 40 samples/template, 480 cases)

**324 ok, 97 `wrongVolume`, 42 `refused`, 15 `shellFailed`, 2 `draftTooLarge`,
0 `ref-fails`.** `refused` is the expected large-thickness refusal
(`shellTooThick`); OCP also fails most of those.

| template | n | ok | wrongVolume | refused | other |
|---|---|---|---|---|---|
| box_top | 40 | 34 | 0 | 6 | |
| box_side | 40 | 32 | 0 | 8 | |
| box_two | 40 | 34 | 0 | 6 | |
| box_out | 40 | 0 | 40 | 0 | |
| cyl_top | 40 | 39 | 0 | 1 | |
| tube_top | 40 | 25 | 0 | 0 | 15 `shellFailed` |
| cone_top | 40 | 0 | 38 | 0 | 2 `draftTooLarge` |
| l_closed | 40 | 0 | 19 | 21 | |
| fillet_shell | 40 | 40 | 0 | 0 | |
| thicken_planar | 40 | 40 | 0 | 0 | |
| thicken_cyl | 40 | 40 | 0 | 0 | |
| thicken_cone | 40 | 40 | 0 | 0 | |

## Failure groups

| template | class | failing parameters | notes |
|---|---|---|---|
| box_out | wrongVolume | all 40; rel 3.0e-2 .. 1.3e-1 | outward shell with the top removed keeps an extra roof slab; error scales with `w*d*t` |
| tube_top | shellFailed | 15/40; `ro` 14.8 .. 38.4, `ri` 4.4 .. 29.0, `h` 11.7 .. 50.9 | OCP thick solid succeeds |
| cone_top | wrongVolume | 38/40; rel 1.1e-1 .. 2.0e-1 | drafted-cylinder shell; base cone volume matches OCP |
| l_closed | wrongVolume | 19/40; rel 3.4e-2 .. 1.9e+1 | closed shell of a two-box L; the large errors leave most of the solid |
| l_closed | refused | 21/40 | thickness above the wall/feature limit |

The box/cylinder large-thickness refusals are correct behaviour. The four
groups above are volume-law or wall-construction gaps; see
internal notes (not published) for ranked repros.

## Passed

Single-face and two-face box shells (inward), cylinder top shell, shell after
a vertical-edge fillet, and thicken of planar, cylindrical and conical sheets
all passed 40/40. The cylindrical sheet is a loft between two equal circles
(`surfaceExtrude` of a circle currently returns no response, so it is not
used as the passing template).
