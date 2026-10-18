# Hole / pattern / mirror / draft sweep against OCCT

`tools/feature_sweep.mjs` covers features outside the earlier sweeps:
through and blind holes, counterbores, countersinks, linear and circular
patterns, mirror, and draft. Volumes are compared with OCP cuts (analytic
integral for draft).

```sh
node tools/feature_sweep.mjs --samples 15 --seed 7 --json out.json
node tools/feature_sweep.mjs --samples 15 --seed 7 --update-baseline tools/feature_sweep_baseline.json
node tools/feature_sweep.mjs --samples 15 --seed 7 --check tools/feature_sweep_baseline.json
```

## Result (kit wasm, seed 7, 15 samples/template, 120 cases)

**120 ok, 0 failures.**

| template | n | ok | notes |
|---|---:|---:|---|
| hole_through | 15 | 15 | cylinder cut through a box |
| hole_blind | 15 | 15 | flat-bottom cut |
| counterbore | 15 | 15 | large shallow + small through |
| countersink | 15 | 15 | drafted circular cut |
| lpattern | 15 | 15 | 2 .. 12 holes along X |
| cpattern | 15 | 15 | 3 .. 12 holes about Z |
| mirror | 15 | 15 | one hole mirrored across X |
| draft | 15 | 15 | 0.5 .. 15 deg sides, analytic volume |

The counterbore path passes here; this does not exercise the original kernel's
known counterbore error, because the reference is OCP and the shape is a
straight two-step cut. Rib and lip are not in this sweep; they remain in the
existing `brep_cases` / gate checks and are a separate feature-family task.
