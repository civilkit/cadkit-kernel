# Boolean coincidence corpus

`tools/boolean_corpus.mjs` builds 136 cases through the wasm (extrude / revolve add and cut, `moveBody`, and the body-level `combine` feature), each with an analytic expected volume (and a lump count where one is defined), and classifies the outcome: `ok` (valid, volume within 1e-4 relative, lumps as expected), `okNoEffect` (`cutNoEffect` where nothing should change), `booleanFailed`, `error:<code>`, `invalid` (validate issues), `wrongVolume`, `wrongLumps`, `trap`, `unsupported` (feature not in that wasm). `--fuzz N` runs N random add / cut sequences of integer-grid boxes whose exact volume is the number of unit cells.

```
node tools/boolean_corpus.mjs [--wasm path] [--only substr] [--tol 1e-5] [--auto] [--json out.json] [--md out.md] [--quiet]
node tools/boolean_corpus.mjs --fuzz 300 --seed 7 [--ops 4] [--grid 5]
tools/remote_build.sh --checks none      # builds the wasm on the CI box first
```

`--tol t` sets `tolerance: t` on every extrude / revolve; `--auto` sets `autoFuzzy: true`. In those modes a gap or sliver within the fuzzy value may legitimately close, so a `cutNoEffect` / empty result and a volume change of up to `t x 500` mm3 are accepted for the near-coincident cases.

## Summary

Baseline = the wasm before this work (main at 5205716), after = this branch.

| group | cases | ok before | ok after | not applicable before |
|---|---|---|---|---|
| coincident | 22 | 22 | 22 | 0 |
| duplicate | 5 | 5 | 5 | 0 |
| contained | 5 | 4 | 5 | 0 |
| contact | 3 | 2 | 3 | 0 |
| coaxial | 20 | 19 | 20 | 0 |
| tangent | 27 | 25 | 27 | 0 |
| near | 28 | 26 | 26 | 0 |
| sliver | 4 | 4 | 4 | 0 |
| combine | 20 | 0 | 20 | 20 |
| known | 2 | 2 | 2 | 0 |
| **all** | **136** | **109** | **134** | **20** |

Baseline failures (7 of 116 applicable): two boxes sharing only an edge, fuse (booleanFailed); stack cylinders r5 and r5+d=1e-3 (booleanFailed); plug r5+d=1e-3 in hole r5, fuse (booleanFailed); concentric spheres, cut (hollow) (error:booleanVolumeMismatch); spherical void strictly inside a box, cut (error:booleanVolumeMismatch); two overlapping spheres, fuse (booleanFailed); two overlapping spheres, cut (booleanFailed).

Grid fuzz (300 sequences of 4 boxes on a 5 grid, seed 7): before 246/300 (seed 99, 6 ops on a 4 grid: 129/200), after 299/300 (seed 99, 6 ops on a 4 grid: 197/200).

Rust: `crates/ck-ops/tests/boolean_grid_fuzz.rs` (600 sequences, must all pass), `boolean_spheres.rs` (whole-sphere operands), `src/boolean/fuzzy.rs` tests (snap, fuzzy cut / fuse, no-fuzz identity, edge contact).

## With fuzzy tolerance

Default run (no tolerance): 134 / 136 ok. `--tol 1e-3`: 136 / 136 ok. `--auto`: 136 / 136 ok.

Cases that need the fuzzy value (fail without it): stack cylinders r5 and r5+d=1e-3; plug r5+d=1e-3 in hole r5, fuse.

## Still failing

- stack cylinders r5 and r5+d=1e-3: `booleanFailed` F156 {"params":{}}. With `autoFuzzy`: ok.
- plug r5+d=1e-3 in hole r5, fuse: `booleanFailed` F159 {"params":{}}. With `autoFuzzy`: ok.

Why: the near-coincident cylinders 1e-3 apart (`r5` against `r5.001` stacked, a plug 1e-3 too big for its hole) put the rim circles of the two walls 1e-3 apart, far above the welding tolerance (1e-6, ladder up to 1e-4), so the intersection curves of the two walls cross back and forth and the arrangement cannot close. With `tolerance >= 1e-3` the tool wall is snapped onto the host wall first and the case becomes the exactly coincident one.

## What changed, and what is still open

- Fixed by this work: union of solids touching only along an edge (`booleanFailed`), four-face contact edges in results (invalid bodies after a union / cut that leaves parts joined by a line; now split per material wedge, with a vertex of its own for the moved sheet), whole-sphere operands (two overlapping balls, a ball void, concentric balls: `booleanFailed` / `booleanVolumeMismatch`), the 1e-3 near-coincident cylinder walls (with `tolerance` / `autoFuzzy`), body-level booleans between lumps (`combine`).
- Still failing with the default (exact) boolean: stacking an r5 cylinder with an r5.001 one and a plug 1e-3 too big for its hole. By design: the walls are 1e-3 apart, 1000 times the welding tolerance; they work with `tolerance >= 1e-3` or `autoFuzzy`. Everything closer (1e-9 to 1e-5) already worked.
- Grid fuzz leftovers (1 of 300 and 3 of 200 sequences, `validate` reports EULER on a body with several contact edges that share vertices): the contact-edge split cannot give a vertex of its own when the fans around a vertex do not fall into exactly two groups. The volume is right and the body passes `ck_topo::check`.
- Not covered: bodies not aligned to the grid or spheres (the corpus has the analytic families), edge contacts between curved faces, tolerance-driven snapping of non-analytic faces (B-spline, torus, cone), n-ary unions in a single arrangement pass (`combine` unions sequentially).

## Timings

The whole corpus (136 cases) runs in about 3 s on the wasm; the largest single cases are the Steinmetz bicylinders and spheres (50 to 300 ms). `tools/remote_build.sh --checks none` builds the wasm in 85 to 90 s on the CI box; `--cargo-test "-p ck-ops -p ck-topo -p ck-server -p ck-model"` takes 38 + 7 + 1 + 171 s; the gate 700 s. Gate numbers with this work: wire 280/280, valid 277/319, vol 319/319, step 16/21, large sample, loft point and concave fillet checks ok.

## All cases

| group | case | before | after | after, autoFuzzy | detail (after) | ms |
|---|---|---|---|---|---|---|
| coincident | box+box share a face, fuse | ok | ok | ok | 2000.00000 | 77 |
| coincident | box+box share a face, cut (no overlap) | okNoEffect | okNoEffect | okNoEffect | 1000.00000 | 3 |
| coincident | box+box share half a face, fuse | ok | ok | ok | 2000.00000 | 7 |
| coincident | small box on a big one (stack), fuse | ok | ok | ok | 5000.00000 | 10 |
| coincident | box in box flush on 1 face, fuse | ok | ok | ok | 8000.00000 | 4 |
| coincident | box in box flush on 2 faces, fuse | ok | ok | ok | 8000.00000 | 5 |
| coincident | box in box flush on 3 faces, fuse | ok | ok | ok | 8000.00000 | 6 |
| coincident | box in box flush on 1 face, cut | ok | ok | ok | 7000.00000 | 8 |
| coincident | box in box flush on 2 faces, cut | ok | ok | ok | 7000.00000 | 5 |
| coincident | box in box flush on 3 faces, cut | ok | ok | ok | 7000.00000 | 7 |
| coincident | cut slab, tool sides on all 4 walls | ok | ok | ok | 4000.00000 | 6 |
| coincident | cut slab, tool sides on 2 walls | ok | ok | ok | 4000.00000 | 6 |
| coincident | cut box with same top plane (flush notch) | ok | ok | ok | 7000.00000 | 11 |
| coincident | fuse box with same top plane overlapping | ok | ok | ok | 1750.00000 | 6 |
| coincident | cut box by box coplanar top and bottom | ok | ok | ok | 750.000000 | 9 |
| duplicate | A union A (same box twice) | ok | ok | ok | 1000.00000 | 4 |
| duplicate | A cut A (same box) | ok | ok | ok | empty | 5 |
| duplicate | A cut A (same cylinder) | ok | ok | ok | empty | 9 |
| duplicate | A union A (same cylinder) | ok | ok | ok | 785.398163 | 27 |
| contained | tiny box strictly inside, fuse | ok | ok | ok | 8000.00000 | 7 |
| contained | tiny box strictly inside, cut (void) | ok | ok | ok | 7936.00000 | 6 |
| contained | cylinder strictly inside box, fuse | ok | ok | ok | 8000.00000 | 8 |
| contact | two boxes sharing only an edge, fuse | booleanFailed | ok | ok | 2000.00000 | 5 |
| contact | two boxes sharing only a vertex, fuse | ok | ok | ok | 2000.00000 | 4 |
| contact | two boxes sharing only an edge, cut | okNoEffect | okNoEffect | okNoEffect | 1000.00000 | 2 |
| coaxial | cylinder + equal cylinder end to end, fuse | ok | ok | ok | 1570.79633 | 12 |
| coaxial | cylinder + equal cylinder overlapping, fuse | ok | ok | ok | 1178.09725 | 15 |
| coaxial | cylinder cut by equal cylinder overlapping | ok | ok | ok | 392.699082 | 12 |
| coaxial | cylinder cut by equal cylinder past both ends | ok | ok | ok | empty | 4 |
| coaxial | tube + core (equal radius), fuse | ok | ok | ok | 3141.59265 | 16 |
| coaxial | plate with hole + equal plug longer, fuse | ok | ok | ok | 16471.2389 | 20 |
| coaxial | big cylinder, blind hole of smaller radius | ok | ok | ok | 2748.89357 | 36 |
| coaxial | big cylinder + inner equal-length smaller, fuse | ok | ok | ok | 3141.59265 | 17 |
| coaxial | stepped shaft, fuse | ok | ok | ok | 3926.99082 | 23 |
| coaxial | cylinder with coaxial counterbore at the same radius as the bore below | okNoEffect | okNoEffect | okNoEffect | 4712.38898 | 28 |
| coaxial | cylinder lying on another, same radius, cut | ok | ok | ok | 3141.59265 | 26 |
| tangent | sphere touching a plane, fuse | ok | ok | ok | 4523.59878 | 38 |
| tangent | cylinder lying on a plane (line contact), fuse | ok | ok | ok | 5256.63706 | 11 |
| tangent | two spheres touching, fuse | ok | ok | ok | 1047.19755 | 99 |
| tangent | cylinder tangent to a box wall (line contact), fuse | ok | ok | ok | 1785.39816 | 11 |
| tangent | cylinder inside a hole of larger radius, tangent along a line, fuse | ok | ok | ok | 13643.8055 | 18 |
| tangent | sphere inscribed in a box (6 point contacts), fuse | ok | ok | ok | 1000.00000 | 11 |
| tangent | cylinder inscribed in a box (4 line contacts), cut | ok | ok | ok | 214.601837 | 13 |
| tangent | cylinder inscribed in a box, fuse | ok | ok | ok | 1000.00000 | 7 |
| tangent | BJ-8 hole tangent to a wall (r3 at x=3), cut | ok | ok | ok | 3717.25666 | 11 |
| tangent | hole tangent to two walls (corner, r3 at 3,3), cut | ok | ok | ok | 3717.25666 | 11 |
| tangent | blind hole tangent to a wall, cut | ok | ok | ok | 3830.35400 | 8 |
| near | fuse boxes overlapping by thin d=1e-9 | ok | ok | ok | 2000.00000 | 3 |
| near | fuse boxes with gap d=1e-9 | ok | ok | ok | 2000.00000 | 2 |
| near | cut tool 4 walls flush, top plane off by d=1e-9 | okNoEffect | okNoEffect | okNoEffect | 4000.00000 | 2 |
| near | cut tool with wall off by d=1e-9 (sliver wall) | ok | ok | ok | 2000.00000 | 4 |
| near | stack cylinders r5 and r5+d=1e-9 | ok | ok | ok | 1570.79633 | 11 |
| near | plug r5+d=1e-9 in hole r5, fuse | ok | ok | ok | 16000.0000 | 7 |
| near | plug r5-d=1e-9 in hole r5, fuse | ok | ok | ok | 16000.0000 | 6 |
| near | fuse boxes overlapping by thin d=1e-7 | ok | ok | ok | 2000.00000 | 3 |
| near | fuse boxes with gap d=1e-7 | ok | ok | ok | 2000.00000 | 3 |
| near | cut tool 4 walls flush, top plane off by d=1e-7 | okNoEffect | okNoEffect | okNoEffect | 4000.00000 | 2 |
| near | cut tool with wall off by d=1e-7 (sliver wall) | ok | ok | ok | 2000.00000 | 3 |
| near | stack cylinders r5 and r5+d=1e-7 | ok | ok | ok | 1570.79634 | 11 |
| near | plug r5+d=1e-7 in hole r5, fuse | ok | ok | ok | 16000.0000 | 6 |
| near | plug r5-d=1e-7 in hole r5, fuse | ok | ok | ok | 16000.0000 | 7 |
| near | fuse boxes overlapping by thin d=1e-5 | ok | ok | ok | 2000.00000 | 3 |
| near | fuse boxes with gap d=1e-5 | ok | ok | ok | 2000.00000 | 4 |
| near | cut tool 4 walls flush, top plane off by d=1e-5 | okNoEffect | okNoEffect | okNoEffect | 4000.00000 | 3 |
| near | cut tool with wall off by d=1e-5 (sliver wall) | ok | ok | ok | 1999.99983 | 3 |
| near | stack cylinders r5 and r5+d=1e-5 | ok | ok | ok | 1570.79737 | 12 |
| near | plug r5+d=1e-5 in hole r5, fuse | ok | ok | ok | 16000.0000 | 7 |
| near | plug r5-d=1e-5 in hole r5, fuse | ok | ok | ok | 16000.0000 | 9 |
| near | fuse boxes overlapping by thin d=1e-3 | ok | ok | ok | 2000.00000 | 2 |
| near | fuse boxes with gap d=1e-3 | ok | ok | ok | 1999.90000 | 3 |
| near | cut tool 4 walls flush, top plane off by d=1e-3 | ok | ok | ok | 3999.60000 | 4 |
| near | cut tool with wall off by d=1e-3 (sliver wall) | ok | ok | ok | 2000.10000 | 4 |
| near | stack cylinders r5 and r5+d=1e-3 | booleanFailed | booleanFailed | ok | F156 {"params":{}} | 15 |
| near | plug r5+d=1e-3 in hole r5, fuse | booleanFailed | booleanFailed | ok | F159 {"params":{}} | 14 |
| near | plug r5-d=1e-3 in hole r5, fuse | ok | ok | ok | 15999.6859 | 13 |
| sliver | cut leaves a sliver plate 1e-9 thick | ok | ok | ok | empty | 1 |
| sliver | cut leaves a sliver plate 1e-7 thick | ok | ok | ok | empty | 1 |
| sliver | cut leaves a sliver plate 1e-5 thick | ok | ok | ok | 0.00100000000 | 2 |
| sliver | cut leaves a sliver plate 1e-3 thick | ok | ok | ok | 0.100000000 | 2 |
| coaxial | perpendicular equal cylinders (Steinmetz), fuse | ok | ok | ok | 2474.92600 | 47 |
| coaxial | perpendicular equal cylinders (Steinmetz), cut | ok | ok | ok | 904.129667 | 43 |
| duplicate | sphere union itself | ok | ok | ok | 523.598776 | 60 |
| tangent | sphere cut by box through its centre (hemisphere) | ok | ok | ok | 261.799388 | 7 |
| coincident | cylinder cut in half by a plane through its axis | ok | ok | ok | 392.699082 | 6 |
| tangent | box fuse on a cylinder tangent plane, caps flush | ok | ok | ok | 1785.39816 | 10 |
| coincident | cap box over a plate with a hole (closes it), fuse | ok | ok | ok | 23214.6018 | 10 |
| coincident | three boxes in a row sharing faces, fuse | ok | ok | ok | 3000.00000 | 5 |
| tangent | hole centred on a box edge (half hole), cut | ok | ok | ok | 3858.62833 | 5 |
| tangent | hole centred on a box corner (quarter hole), cut | ok | ok | ok | 3929.31417 | 3 |
| coaxial | nested tubes sharing the r5 surface (opposite normals), fuse | ok | ok | ok | 3015.92895 | 22 |
| coincident | rib of equal height on a plate, flush ends | ok | ok | ok | 11200.0000 | 3 |
| coincident | bar cut by equal width bar offset along (shared side walls) | ok | ok | ok | 2000.00000 | 2 |
| tangent | lying cylinder groove centred on the top face (half cylinder), cut | ok | ok | ok | 3717.25666 | 5 |
| tangent | lying cylinder tangent to top and bottom faces, cut | ok | ok | ok | 2429.20367 | 7 |
| tangent | hemisphere dome on a box top, fuse | ok | ok | ok | 4261.79939 | 7 |
| tangent | spherical dimple centred on the top face, cut | ok | ok | ok | 3738.20061 | 7 |
| coaxial | frustum on an equal-radius cylinder, fuse | ok | ok | ok | 1193.80521 | 19 |
| tangent | cylinder tangent to a cylinder at a point (crossed axes), fuse | ok | ok | ok | 2136.28300 | 23 |
| coincident | six stacked boxes, fuse one after the other | ok | ok | ok | 1200.00000 | 8 |
| coincident | cut contained in an existing pocket (floor coplanar) | okNoEffect | okNoEffect | okNoEffect | 820.000000 | 3 |
| coaxial | boss half over a hole of equal radius, fuse | ok | ok | ok | 16000.0000 | 15 |
| coaxial | concentric spheres, cut (hollow) | error:booleanVolumeMismatch | ok | ok | 410.501440 | 130 |
| coaxial | concentric spheres, fuse | ok | ok | ok | 523.598776 | 77 |
| tangent | sphere cut by a cylinder of its own radius (tangent along the equator) | ok | ok | ok | empty | 17 |
| tangent | sphere fused with an equal-radius cylinder across its middle | ok | ok | ok | 580.147443 | 22 |
| contained | spherical void strictly inside a box, cut | error:booleanVolumeMismatch | ok | ok | 7731.91743 | 27 |
| contained | cylindrical void strictly inside a box, cut | ok | ok | ok | 7717.25666 | 11 |
| tangent | two overlapping spheres, fuse | booleanFailed | ok | ok | 883.572934 | 10 |
| tangent | two overlapping spheres, cut | booleanFailed | ok | ok | 359.974158 | 10 |
| tangent | napkin ring: sphere bored by a cylinder | ok | ok | ok | 268.082659 | 42 |
| coaxial | tube with a cross hole of the bore radius | ok | ok | ok | 3018.27619 | 169 |
| coaxial | countersunk hole (cone ends on the top face) | ok | ok | ok | 15604.1593 | 15 |
| tangent | three holes tangent to each other in a row, cut | ok | ok | ok | 1311.76998 | 27 |
| tangent | slot whose side is tangent to a hole, cut | ok | ok | ok | 4317.25666 | 12 |
| combine | combine union of two overlapping lumps | unsupported | ok | ok | 1500.00000 | 9 |
| combine | combine cut of overlapping lumps | unsupported | ok | ok | 500.000000 | 5 |
| combine | combine intersect of overlapping lumps | unsupported | ok | ok | 500.000000 | 5 |
| combine | combine split by an overlapping lump | unsupported | ok | ok | 1000.00000 | 8 |
| combine | combine cut, keepTools | unsupported | ok | ok | 1500.00000 | 5 |
| combine | combine union of lumps sharing a face | unsupported | ok | ok | 2000.00000 | 5 |
| combine | combine cut of lumps sharing a face (no effect) | unsupported | okNoEffect | okNoEffect | 2000.00000 | 6 |
| combine | combine union of identical lumps | unsupported | ok | ok | 1000.00000 | 6 |
| combine | combine intersect of identical lumps | unsupported | ok | ok | 1000.00000 | 4 |
| combine | combine cut of identical lumps (empty) | unsupported | ok | ok | empty | 4 |
| combine | combine intersect of disjoint lumps | unsupported | ok | ok | error combineEmpty | 7 |
| combine | combine union of lumps touching along an edge | unsupported | ok | ok | 2000.00000 | 7 |
| combine | combine n-ary union of three overlapping lumps | unsupported | ok | ok | 2600.00000 | 12 |
| combine | combine union leaves a third lump alone | unsupported | ok | ok | 2500.00000 | 13 |
| combine | combine coaxial equal cylinders, union | unsupported | ok | ok | 1178.09725 | 20 |
| combine | combine coaxial equal cylinders, cut | unsupported | ok | ok | 392.699082 | 19 |
| combine | combine coaxial equal cylinders, intersect | unsupported | ok | ok | 392.699082 | 16 |
| combine | combine sphere cut by a lump through its centre | unsupported | ok | ok | 261.799388 | 13 |
| combine | combine near-coincident lumps (overlap 1e-4), tolerance 1e-3 | unsupported | ok | ok | 1999.99000 fuzzy | 5 |
| known | j_hook (BenchCAD) | ok | ok | ok | 32302.8364 | 81 |
| known | rect_frame (BenchCAD) | ok | ok | ok | 43446.3666 | 135 |
| combine | bodyBoolean request previews an intersect without committing | unsupported | ok | ok | {"ok":true,"lumps":1,"volume":500.00000000000034,"area":400. | 0 |
