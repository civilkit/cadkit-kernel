# Mass-properties sweep against OCCT

`tools/massprops_sweep.mjs` builds the same primitive / loft / revolve / sweep
shapes as the other sweeps and compares `massProps` with OCP
(`BRepGProp`): volume, surface area, centre of mass, and the inertia tensor
about the CG. The kernel returns principal moments and axes; the tensor is
reconstructed from them.

```sh
node tools/massprops_sweep.mjs --samples 20 --seed 7 --json out.json
node tools/massprops_sweep.mjs --samples 20 --seed 7 --update-baseline tools/massprops_sweep_baseline.json
node tools/massprops_sweep.mjs --samples 20 --seed 7 --check tools/massprops_sweep_baseline.json
```

Tolerance is 1e-6 relative on each of volume, area, inertia (normalised by the
largest tensor entry) and CG (normalised by `cbrt(volume)`).

## Result (kit wasm, seed 7, 20 samples/template, 240 cases)

**220 ok, 20 `wrongVolume`.**

| template | max rel volume | max rel area | max rel COM | max rel inertia |
|---|---|---|---|---|
| box | 7.3e-16 | 4.7e-16 | 9.7e-17 | 7.7e-16 |
| cylinder | 9.9e-16 | 1.2e-15 | 3.3e-16 | 8.6e-15 |
| cone | 6.4e-16 | 5.6e-16 | 2.9e-16 | 1.0e-14 |
| sphere | 6.0e-16 | 8.5e-16 | 2.6e-16 | 1.4e-15 |
| torus | 7.7e-16 | 9.7e-16 | 1.3e-15 | 7.3e-15 |
| tube | 2.8e-15 | 1.6e-15 | 7.9e-16 | 9.0e-15 |
| loft2 | 4.0e-16 | 4.7e-16 | 2.8e-16 | 1.4e-14 |
| **loft3** | **6.7e-2** | **3.7e-2** | **1.6e-2** | **1.0e-1** |
| revolve | 7.4e-16 | 4.6e-16 | 5.0e-16 | 3.1e-15 |
| sweep line | 1.4e-15 | 1.5e-15 | 4.9e-16 | 1.4e-14 |
| sweep arc | 1.1e-15 | 7.9e-16 | 1.0e-15 | 5.7e-15 |
| sweep L | 5.5e-16 | 4.4e-16 | 7.9e-16 | 1.6e-15 |

Everything except the **three-section circle loft** is exact to machine
precision. The loft3 errors track the volume-law gap in
`docs/shape-sweep.md`: the CG and inertia differ by the same surface-law
change, not by an independent mass-property bug.

## Reference notes

OCP shapes are placed to match the kernel's frames: the box is centred in X/Y
with its base at Z=0, the cone's wide radius is at the base, and the torus
axis is Y (the kernel revolves about the sketch axis).
