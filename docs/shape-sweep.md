# Sweep / loft / revolve sweep against OCCT

`tools/shape_sweep.mjs` generates seeded lofts, revolutions and sweeps and
compares the volume with an OCP replay (`tools/shape_sweep_ref.py`:
`BRepOffsetAPI_ThruSections`, `BRepPrimAPI_MakeRevol`, `BRepOffsetAPI_MakePipe`).

```sh
node tools/shape_sweep.mjs --samples 30 --seed 7 --json out.json
node tools/shape_sweep.mjs --samples 30 --seed 7 --update-baseline tools/shape_sweep_baseline.json
node tools/shape_sweep.mjs --samples 30 --seed 7 --check tools/shape_sweep_baseline.json
```

## Result (kit wasm, seed 7, 30 samples/template, 240 cases)

**223 ok, 16 `wrongVolume`, 1 `ref-fails`, 0 other failures.**

| template | n | ok | wrongVolume | ref-fails |
|---|---|---|---|---|
| loft_circle (2 or 3 circles) | 30 | 14 | 16 | 0 |
| loft_ellipse | 30 | 29 | 0 | 1 |
| loft_rect (rotated) | 30 | 30 | 0 | 0 |
| loft_point | 30 | 30 | 0 | 0 |
| revolve (90 .. 360 deg) | 30 | 30 | 0 | 0 |
| sweep_line | 30 | 30 | 0 | 0 |
| sweep_arc | 30 | 30 | 0 | 0 |
| sweep_L (mitered) | 30 | 30 | 0 | 0 |

## Failure group

All 16 `loft_circle` failures are **three-section** lofts; every two-section
loft passed. Relative volume error 3.0e-3 .. 1.5e-1. OCP builds the loft as
a smooth ThruSections surface.

| case | sections (r, z) | rel |
|---|---|---|
| `loft_circle_29` | 19.8905@0, 6.62500@15.8237, 9.46748@31.1468 | 1.51e-1 |
| `loft_circle_3` | 17.9220@0, 8.47402@25.4760, 13.7752@49.1874 | 6.16e-2 |
| `loft_circle_15` | 16.7023@0, 9.81449@18.2587, 17.2936@39.1748 | 5.05e-2 |
| `loft_circle_2` | 17.4292@0, 12.2272@20.0724, 19.3579@45.7866 | 3.90e-2 |

The one `loft_ellipse` reference failure is an OCP `ThruSections` limitation:
the two ellipses have their major axes swapped (`a=18.2694, b=6.88788` then
`a=8.82541, b=11.8715`). The kernel loft itself regenerated and validated;
no kernel class is assigned.

## Passed

Two-section circle lofts, rotated rectangle lofts, point-ended lofts, all
tested revolve angles (90 .. 360 deg, axis touching the profile), a circle
along a line, a circle along an arc, and a rectangle along a mitered L path.
For the L path OCP needs `RightCorner` transition; the default pipe
self-intersects at the sharp corner.
