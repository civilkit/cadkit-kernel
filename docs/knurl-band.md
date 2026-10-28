# `knurlBand`: diamond and pyramid knurls built as faces

A diamond or pyramid knurl cut groove set against groove set with booleans costs face pairs that grow with the square of the groove count
(24 + 24 diamond grooves: about 30 s, 1200 faces; dense ones fail with `booleanBudgetExceeded`). The knurled surface is a known function of the position on the band, so
`knurlBand` writes its facets directly and cuts them from the body in ONE boolean against the host cylinder. The facets never meet each other, so the
work is linear in the number of facets.

```json
{"type":"knurlBand","id":"F5","axis":"z","center":[0,0],"radius":10,"from":0,"to":12,
 "pattern":"pyramid","count":24,"depth":0.65,"angle":90,"ringPitch":2.6}
{"type":"knurlBand","id":"F6","axis":"x","center":[0,0],"radius":10,"from":0,"to":12,"pattern":"diamond","count":24,"helixAngle":30}
```

| field | meaning |
|---|---|
| `axis` | `x`, `y` or `z` (default `z`) |
| `center` | the two world coordinates of the axis line: `[x, y]` for z, `[x, z]` for y, `[y, z]` for x (default `[0,0]`) |
| `radius`, `from`, `to` | the cylinder being knurled and the axial range, world coordinates |
| `pattern` | `diamond` (two crossed sets of V grooves, per-hand `count`, turning `helixAngle` degrees from the axis, default 30) or `pyramid` (axial V grooves crossed by ring V grooves; leaves square pyramids) |
| `count` or `pitch` | grooves round the circumference (per hand for a diamond), 6 to 2000 |
| `depth` | default pitch / 4 |
| `angle` | included angle of the V in degrees, default 90 |
| `ringPitch` | pyramid: ring spacing, default the groove pitch; the band holds a whole number of rings |

Only the V profile is built (`profile: "round"` is `invalidValue`: cut it with the sweep and revolve features). Grooves must leave a land
between them (2 half-width at most 0.9 of the pitch), else `invalidValue {field:"count", detail}` names the largest count that fits; grooves whose
widened lattice leaves no land (very wide grooves on few lands) are `knurlBandInvalid`.

## What is built

Per lattice cell (a groove crossing and the cell towards the next two crossings) the surface is five polygons: four flanks, each running from
one crossing to the next with the groove bottom as an edge, and the land between four crossings. A pyramid cell is therefore 5 faces (the flank
and land polygons are exactly planar); a diamond cell is 5 polygons that are split into triangles (10 faces), because a helical flank is not planar.
Cells are clipped to the band. Together with a closed polygonal envelope above the cylinder and two annular end walls they form a closed manifold
solid (the "tool"; faces named `<id>.K<n>`), which is cut from the body in one boolean. The land is lifted a few hundredths of a millimetre above the cylinder, and the lattice grows by the same ratio, so the chords of the land
facets never dip into the material and every flank still crosses the cylinder exactly where the ideal groove edge is: the land of the result is the
host cylinder face itself.

Accuracy: vertices are exact points of the ideal surface, except that the crossings are raised by part of the sagitta of the groove bottom between
two crossings (the bottoms are chords, deeper in the middle than the circle: 2/3 of it for a diamond, 1/2 for a pyramid, which also has straight
bottoms). When the sagitta would exceed 15 percent of the depth (few grooves on a shallow cut), each flank is cut into strips (up to 8) with extra
nodes on the groove bottom and the land edge, and the land becomes a fan of triangles round its middle. A diamond flank is a warped quad and is split
along the diagonal that comes out nearer the ideal surface. Against a numeric integral of the ideal surface the removed volume is within 1 percent for
12 to 36 grooves on a radius 10 cylinder, 3 percent for the coarsest pyramid tested (12 grooves of depth 0.5); the pyramid flanks and lands are exactly
planar when no strips are needed.

## Checks and errors

The tool is verified closed and manifold with positive volume (`knurlBandInvalid {detail}` otherwise), the boolean's own checks judge the volume
(`booleanVolumeMismatch`), the result must not have more validity issues than the body had and must keep its lumps; on any failure the body is left as it was.
More than 60 000 facets is `knurlBandTooLarge`. `nothingToCut` without a body, `cutNoEffect` when the band is not on the body, and the boolean's
own errors (`booleanFailed`, `booleanBudgetExceeded`).

## Timing

Regeneration of the feature in the wasm kernel (node, radius 10, band 12 mm; the boolean method is the kit's twisted-sweep and ring cuts):

| case | knurlBand | boolean cuts |
|---|---|---|
| diamond 12 + 12, 22 degrees | 0.6 s, 434 faces | 2.2 s, 134 faces |
| diamond 16 + 16, 45 degrees | 1.2 s, 978 faces | 10.6 s, 562 faces |
| diamond 24 + 24, 30 degrees | 1.5 s, 1250 faces | 13.2 s, 722 faces |
| pyramid 24 x 8 | 2.7 s, 2186 faces | 5.0 s, 1034 faces |
| pyramid 12 x 4 | 0.7 s, 566 faces | 1.4 s, 278 faces |

Growth with the groove count (diamond at 30 degrees / pyramid with square cells): 12 grooves 0.44 s / 0.40 s, 24 grooves 1.4 s / 0.65 s, 48 grooves
6.3 s / 2.6 s (4850 / 2306 faces). The face count of such a lattice grows with the square of the groove count (cells are as many as there are
crossings); the time is about 1.3 ms per face of the result, and the dense diamonds that the boolean method refuses (`booleanBudgetExceeded`) build.
The tool itself takes 10 to 350 ms natively. A volume integral of the result is not taken: it costs three times the boolean.
