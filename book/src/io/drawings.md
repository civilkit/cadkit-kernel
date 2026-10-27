# Drawings

The `drawing` request projects a body into 2D views: visible and hidden lines, curve outlines, centre lines and, optionally, a hatched section. The reply is geometry only, in view coordinates; layout on a sheet, dimensions and annotations are left to the client. The projection code is in `crates/ck-ops/src/drawing`, the request check in `crates/ck-server/src/drawing.rs`.

## Request

```json
{"type":"drawing","tag":"main","hidden":true,
 "views":[{"name":"front"},
          {"name":"top"},
          {"x":[1,0,0],"y":[0,0,1]},
          {"name":"front","section":{"point":[0,0,4],"hatch":3}}]}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State to project. A tag with no state answers `noState: nothing regenerated yet`, even if `views` is malformed |
| `views` | array | `[]` | The views to produce. A missing or non-array value answers `{"views":[]}` |
| `hidden` | bool | `true` | Include hidden lines (`v: false`). With `false` only visible pieces are returned |

Each view is an object with either a `name` or both `x` and `y`:

| View field | Meaning |
|---|---|
| `name` | One of `front`, `back`, `left`, `right`, `top`, `bottom`, `iso` |
| `x`, `y` | Arrays of three numbers: the view's horizontal and vertical axes in world coordinates. The viewing direction is `x` cross `y`, normalised |
| `section` | Optional `{point:[x,y,z], hatch?}`. The body is cut by the plane through `point` normal to the viewing direction; the half on the viewer's side is kept and the cut face is hatched with 45 degree lines `hatch` units apart (default 3) |

If any view lacks a valid name and lacks both `x` and `y`, the whole request fails with `drawing: a view needs a name (front, back, left, right, top, bottom, iso) or x and y` (checked). The part's world axes are Y up; the named views are:

| Name | View x (right) | View y (up) | Looks along |
|---|---|---|---|
| `front` | +X | +Y | -Z (viewer at +Z) |
| `back` | -X | +Y | +Z |
| `right` | -Z | +Y | -X (viewer at +X) |
| `left` | +Z | +Y | +X |
| `top` | +X | -Z | -Y (viewer at +Y) |
| `bottom` | +X | +Z | +Y |
| `iso` | (0.707, 0, -0.707) | derived | from (1, 1, 1) |

A part sketched on the `front` plane and extruded with a positive depth therefore appears in the `front` view as the sketch, and in the `top` view as a strip below the X axis.

## Reply

```json
{"ok":true,"result":{"views":[ {view}, ... ]},"ms":0}
```

Each view is:

| Field | Meaning |
|---|---|
| `min`, `max` | `[u, v]` bounds of all emitted line points |
| `lines` | The drawn pieces, below |
| `edges` | One entry per model edge seen in the view, plus one per outline |
| `axes` | Centre lines of cylinders, each `[[u0,v0],[u1,v1]]` |

A piece in `lines`:

| Field | Meaning |
|---|---|
| `k` | `"e"` edge, `"t"` tangent edge (the two faces meet with parallel normals, as at a fillet boundary), `"o"` outline (the silhouette of a curved face), `"h"` section hatch |
| `v` | `true` if visible, `false` if hidden |
| `p` | Flat polyline `[u0, v0, u1, v1, ...]` in view units (the model's millimetres) |
| `e` | Index into this view's `edges` array. Hatch pieces have no `e` |

An entry in `edges`:

| Field | Meaning |
|---|---|
| `faces` | Sorted ids of the faces meeting at the edge |
| `at` | The edge midpoint in world coordinates |
| `line` | Two projected points, for straight edges |
| `circle` | `{c:[u,v], r}` for a circle seen face-on |
| `mid`, `ends` | The projected midpoint and end points |
| `outline` | `true` for a silhouette entry |
| `round` | `true` for a full circle seen edge-on that spans the view's full width |

Curved pieces are polylines, so a circle arrives as a sequence of points.

## Examples

The plate of the [STEP chapter](step.md): 40 x 20 x 8 mm, sketched on the `front` plane, with a 10 mm through hole at (20, 10) made by a `hole` feature. The part:

```json
{"format":"ck-part","version":1,"name":"plate","features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],
  "entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},
              {"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],
  "constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":8},
 {"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":8},
 {"type":"sketch","id":"F4","plane":{"kind":"feature","id":"F3"},
  "points":[{"id":"p1","x":20,"y":10}],"entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
 {"type":"hole","id":"F5","sketch":"F4","kind":"simple","diameter":10,"end":"through"}]}
```

Results of a fresh run:

| View | `min` | `max` | Pieces | Edges | `edges` with `circle` |
|---|---|---|---|---|---|
| front | [0, 0] | [40, 20] | 7 | 15 | 3 |
| top | [0, -8] | [40, 0] | 6 | 17 | 0 |
| iso | [-5.66, -19.60] | [28.28, 16.33] | 17 | 17 | 0 |

In the front view the four outer edges are straight pieces, for example `{"k":"e","v":true,"p":[0,20,40,20],"e":9}`. The hole rims are three visible polylines: a closed one of 39 points starting at (25, 10) (the rim point on the +u side of the centre (20, 10)), and two partial arcs of 15 and 25 points that end at (16.36, 13.42). Three entries of `edges` carry `"circle":{"c":[20,10],"r":5}`. In the top view the hole appears as two hidden outline pieces, with `e` indices 15 and 16:

```json
[{"k":"o","v":false,"p":[15,-8,15,0],"e":15},{"k":"o","v":false,"p":[25,-8,25,0],"e":16}]
```

and `axes` is `[[[20,-8],[20,0]]]`. With `"hidden":false` the top view keeps its 4 visible pieces and drops the 2 hidden ones.

A section of the front view through `[0, 0, 4]` (half way up the plate's thickness, which runs along Z) returns 25 pieces: edge pieces plus 18 hatch pieces of the form `{"k":"h","v":true,"p":[38.18,0,40,1.82]}`. The cut face is named `SEC.C1` in the `faces` of 6 edge entries. With `hatch: 6` the hatch count drops to 9. A section plane that misses the body gives that view only `{"error":"cutNoEffect"}` (checked with a point at z = 40).

A sphere made by the part `{"format":"ck-part","version":1,"features":[{"type":"sphere","id":"F1","op":"add","center":[0,0,0],"radius":10}]}` gives, in the front view, a single outline piece `k: "o"` of 127 points, `min [-9.997, -10]`, `max [10, 10]` and no `edges`.

## Empty state

A tag whose state has no body (an empty part) answers `null`, or `{}` when `hidden` is truthy. Clients should check for a body first.

## Limits

- No dimensions, tolerances, notes, title blocks or sheet layout. Thread callouts and cosmetic thread symbols are not drawn.
- Curved faces are drawn as polylines; there is no exact arc or spline output.
- Silhouettes of smooth faces other than cylinders come from the display mesh (marching the zero set of the surface normal against the view direction), so they are accurate to the mesh tolerance rather than exact. The emission order of curved pieces and the `round` flag are conventions, not guaranteed stable.
- Duplicate coincident pieces are not merged for curved edges (front and back rims in the example above).
- Hidden-line removal uses the tessellated body as occluder, so complete hidden-line handling for curved occluders is not exact.
- A section keeps one half and cuts with a single plane; offset or stepped section lines are not supported.

## See also

- [Tessellation](../kernel/tessellation.md)
- [Queries](../protocol/queries.md)
- [Mass properties and interference](../validation/analysis.md)
- [Known limits](../quality/limits.md)
