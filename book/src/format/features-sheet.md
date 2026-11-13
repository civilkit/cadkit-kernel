# Sheet metal

A part can be built as a sheet metal part: a base plate of constant thickness with bent flanges, hems and folds, from which the kernel also derives the flat pattern and a bend table. These features are in the list of known type names (see [Features](features.md)). This chapter is a summary; `docs/sheet-metal.md` in the repository has every rule, the probes the numbers were checked against and the limits of each stage.

All lengths are in millimetres and angles in degrees. A number may be an expression string (`"=Thickness*4"`; inside `sheetRules`, `Thickness` is the rule's thickness) or a string with a unit (`"2 mm"`, `"90 deg"`).

## The features

| Type | What it does |
|---|---|
| `sheetRules` | The stock: `name`, `thickness`, `kFactor`, `bendRadius`, `reliefWidth`, `reliefDepth`, `cornerRelief`, `cornerReliefShape` (`notch`, `round`, `rectangular` or `tear`), `hemGap`, `gap`, `active`. Only `name` is required; the rest default from the thickness (2.5 when none is given). A rule with an existing name replaces it. |
| `baseFlange` | The first plate: `sketch` (one closed region, holes allowed), `rule` (a name; default the active rule), `flip`. The region is extruded by the thickness. One per part, as the first body feature. |
| `contourFlange` | The base from an open chain of sketch lines: `sketch`, `curves` (default every line), `distance`, `rule`, `flip`, `reverse`. |
| `edgeFlange` | A bent flange on edges of the plate: `edges`, `height` (required), `angle` (default 90), `radius`, `heightFrom` (`outer`, `inner`, `tangent`), `position` (`inside`, `outside`, `middle`), `flip`, `then` (chained bends, at most 16), and for a partial width `start`, `length` or `from` and `to`, `relief`, or a shaped leg with `profile` and `holes`. |
| `hem` | A 180 degree bend that lays a leg back against the sheet: `edges`, `length`, `gap`, `flip`. |
| `fold` | A bend across the sheet along a line: `points` or `line` (`{sketch, curve}`), `angle`, `radius`, `position` (`centerline`, `start`, `end`, `mould`), `fixed`, `flip`. |
| `unfold`, `refold` | Make the body the flat sheet, and fold it again from the same bend rows. |
| `sheetConvert` | Reads a bent plate of constant thickness as a sheet: `thickness`, `rule`. The result is refused if its volume differs from the body's by more than 1 percent. |
| `sheetCut` | Cuts the closed regions of a sketch out of the sheet through the flat pattern: `sketch`, `regions`. |
| `sheetCorner` | Rounds or chamfers corners of free walls: `corners`, `mode` (`round` or `chamfer`), `radius`, `distance`. |

An edge in `edges` is picked by a point on it (`[x, y, z]` or `{"at": [x, y, z]}`, the nearest outline edge wins), by `{"edge": n}` (index into the outline) or by role (`"P0.side.e2"`). A point on the face the base ended on bends the flange towards that side. The legs and setbacks follow the bend arithmetic: the bend allowance is `BA = (r + k t) a`, with `a` in radians.

```json
{"type": "sheetRules", "id": "R1", "name": "Steel2", "thickness": 2, "kFactor": 0.44, "bendRadius": 2}
{"type": "baseFlange", "id": "B1", "sketch": "S1", "rule": "Steel2"}
{"type": "edgeFlange", "id": "E1", "edges": [[50, 0, 2], [50, 60, 2]], "height": 20, "angle": 90}
```

For a 100 x 60 plate of thickness 2 with k-factor 0.44 and bend radius 2, with two 20 mm flanges on the 100 mm edges, the flat size is 100 x 93.04778684 mm.

## The result

The regen reply gains `sheetMetal`, one entry per sheet feature, sorted by id. Each has the `rule`, a `flatPattern` and `roles` (stable names for the panels, bends and walls, such as `P0.front` and `B1.back`, usable as references). The flat pattern holds `flatSize` (width, length, thickness), the `outline` loops (the outer loop, then the holes), `bends` (for each: `bend`, `feature`, `direction` `up` or `down`, `angleDeg`, `includedAngleDeg`, `radius`, `allowance`, `length`, `start`, `end`), `bendZones` and `unfolded`.

Two stateless requests turn a flat pattern into files: `sheetDxf` (the flat pattern as DXF, `version` `R2000` or `R12`, with bend lines, optional notes and layer options) and `sheetBendTable` (the bend table as CSV or JSON). Both take the `flatPattern` object of a `sheetMetal` entry.

## Errors and warnings

`nothingToCut` (no sheet yet), `missingSketch`, `noClosedProfile`, `invalidValue` (with `field`, `detail` and `hint`), `flangeHeightTooLow` (with `minHeight`) and `sheetMetalUnsupported` (with `detail` and `hint`: what the feature does not build, and what to reorder). A plain extrude cut of a folded sheet is tried as a `sheetCut` first; when that is refused it runs as an ordinary cut and the feature carries the warning `sheetCutFallback`, because the part is then no longer an unfoldable sheet.

## Mirror

`mirror` with `body: true` of a part that is a live sheet keeps it a sheet: with `"replace": true` the image takes the place of the sheet, otherwise the image is joined to it, which gives one sheet only when the plane is square to the base plate and runs along one straight, bend-free edge. Any other plane is refused with `sheetMetalUnsupported`.

## Not built

Flanges on a curved edge, louvres, forming tools, a jog and a mitre flange are not built. A `sheetCut` of a non-rectangular region across a bend is a staircase of wedges in the solid (the flat pattern is exact). `sheetConvert` takes bent plates with straight cylindrical bends only. Arcs in a contour chain and more than one chain are not built.
