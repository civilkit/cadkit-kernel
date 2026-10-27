# Mass-properties sweep against OCCT

`tools/massprops_sweep.mjs` rebuilds the sweep shapes and compares our
`massProps` reply with OCP `GProp_GProps`:
volume, surface area, centre of mass, and the inertia tensor about the CG.

```sh
node tools/massprops_sweep.mjs --samples 20 --seed 7 --json out.json
node tools/massprops_sweep.mjs --samples 20 --seed 7 --update-baseline tools/massprops_sweep_baseline.json
node tools/massprops_sweep.mjs --samples 20 --seed 7 --check tools/massprops_sweep_baseline.json
```

The kernel reply has `principal` moments and `axes`; the tool reconstructs the
global tensor as `sum(principal[k] * axes[k] outer axes[k])` and compares it
with OCP `MatrixOfInertia`. Tolerance is 1e-6 relative for every component.

## Result (kit wasm, seed 7, 20 samples/template, 240 cases)

**220 ok, 20 `wrongVolume`.** All failures are three-section circle lofts;
every other template is exact to machine precision.

| template | n | ok | wrongVolume | max rel volume | max rel area | max rel COM | max rel inertia |
|---|---|---|---|---|---|---|---|
| box | 20 | 20 | 0 | 7.3e-16 | 4.7e-16 | 9.7e-17 | 7.7e-16 |
| cyl | 20 | 20 | 0 | 9.9e-16 | 1.2e-15 | 3.3e-16 | 8.6e-15 |
| cone | 20 | 20 | 0 | 6.4e-16 | 5.6e-16 | 2.9e-16 | 1.0e-14 |
| sphere | 20 | 20 | 0 | 6.0e-16 | 8.5e-16 | 2.6e-16 | 1.4e-15 |
| torus | 20 | 20 | 0 | 7.7e-16 | 9.7e-16 | 1.3e-15 | 7.3e-15 |
| tube | 20 | 20 | 0 | 2.8e-15 | 1.6e-15 | 7.9e-16 | 9.0e-15 |
| loft2 | 20 | 20 | 0 | 4.0e-16 | 4.7e-16 | 2.8e-16 | 1.4e-14 |
| loft3 | 20 | 0 | 20 | 6.7e-2 | 3.7e-2 | 1.6e-2 | 1.0e-1 |
| revolve | 20 | 20 | 0 | 7.4e-16 | 4.6e-16 | 5.0e-16 | 3.1e-15 |
| sweep_line | 20 | 20 | 0 | 1.4e-15 | 1.5e-15 | 4.9e-16 | 1.4e-14 |
| sweep_arc | 20 | 20 | 0 | 1.1e-15 | 7.9e-16 | 1.0e-15 | 5.7e-15 |
| sweep_L | 20 | 20 | 0 | 5.5e-16 | 4.4e-16 | 7.9e-16 | 1.6e-15 |

The only group is the same three-section loft surface-law difference as
`docs/shape-sweep.md`; the mass-property reply is correct for the shape the
kernel actually builds. Worst case `loft3_5` (max rel volume 6.7e-2, area
3.7e-2, COM 1.6e-2, inertia 1.0e-1). See
internal notes (not published).
