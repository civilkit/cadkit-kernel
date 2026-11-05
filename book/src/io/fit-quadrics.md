# Cylinders and cones from facets

A mesh import (SketchUp, or any polygon soup written as a B-rep) turns a cylinder into a ring of flat strips and a cone into a ring of flat trapezoids or triangles. Such a body measures and exports, but a hole cannot be drilled to its axis, a circular edge cannot be filleted and a STEP file carries hundreds of planes. The `fitQuadrics` request recognises these fans and replaces each by one analytic face. The code is `crates/ck-ops/src/fit_quadrics.rs`; it is deterministic and works on the topology and the corner positions only.

## Request

```json
{"type":"fitQuadrics","tag":"imp","opts":{"tol":0.2}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `tag` | string | `"main"` | State holding the body (usually an import) |
| `opts.tol` | number | 0.5 % of the bounding-box diagonal (mm) | The most a facet may differ from the fitted surface: the chord sag of the strips. A cylinder of radius R in N sides sags by R(1 - cos(pi/N)) |
| `opts.fitTol` | number | `tol` / 10 | The most a facet corner may lie off the fitted surface (the fit residual) |
| `opts.maxStepDeg` | number | 30 | Largest angle between neighbouring rails. A square or hexagonal prism is a prism, not a cylinder; a 12-sided one is the coarsest that is fitted |
| `opts.minStrips` | number | 3 | Fewest strips of a group |
| `opts.smoothDeg` | number | 20 | Coaxial groups meeting at a smaller crease are a curved profile (see below) and are all left alone |
| `opts.cones` | bool | `true` | Recognise cones |
| `opts.merge` | bool | `true` | First merge coplanar neighbours, so triangulated strips become quads (bodies up to 3000 faces; larger ones are reported with `mergeSkipped` and should be merged by the importer) |
| `opts.volumeTol` | number | 0.05 | The fit is kept only if the volume changes by less than this fraction |
| `opts.dryRun` | bool | `false` | Report, keep nothing |

A tag with no state answers the usual `noState` error. A tag with state but no body answers `null`.

## What is recognised

A group is a ring or an arc of planar strips (one outer loop each) joined along rails, the edges two strips share. The rails of a cylinder are parallel; those of a cone pass through one apex. The rails' cross-section must lie on a circle within `fitTol`, in equal angular steps of at most `maxStepDeg`, and the chord sag must not exceed `tol`. An arc may end at a flat face: a shaft with a flat keeps its flat and the rest becomes a cylinder face.

Every other edge of a strip is an end chord at constant height. It becomes a circle, so the face across it has to hold that circle: a plane normal to the axis, or a strip of another accepted group on the same circle. Consecutive arcs on a circle between the same two faces are joined into one edge, a whole circle where they close. A hole in a block comes out as a concave cylinder and two circle edges.

**Left alone, with a reason in `declined`**: `residual`, `sag`, `mixedSense`, `obliqueEnd` (the end is an oblique cut: its edge would be an ellipse), `endNotPlane`, `endMismatch`, `endNotManifold`, `coneAxis`, and `smoothProfile`. The last is what keeps a faceted sphere, torus or ogive intact: its bands are cones, each of them valid on its own, but they meet at creases of a few degrees, so they are the tessellation of a curved profile and replacing them by cones would change the shape. A random triangle mesh has no rails and is untouched.

## Reply

| Field | Meaning |
|---|---|
| `applied` | `true` when the tag's body was replaced |
| `reason` | Why not: `nothing` (no group found), `invalid` (the result validates worse than the input), `volumeChanged`, `dryRun` |
| `tol`, `fitTol` | The tolerances used |
| `faces` | `before` and `after` face counts |
| `volume` | `before`, `after` and `relative` change. For a regular N-gon prism the change is the sag of the inscription: 1.15 % for 24 sides, 0.29 % for 48, 0.07 % for 96 |
| `issues` | `validate` issue counts before and after |
| `merged` | Coplanar merges done first |
| `groups` | One per accepted group: `kind` (`cylinder` or `cone`), `face` (the id of the new face, the first strip's), `strips`, `full`, `axis`, `origin`, `radius` or `halfAngle`, `maxResidual`, `sag` |
| `declined` | `{ face, strips, reason }` |

The new face keeps the first strip's name; the other strips' names are kept in the face's absorbed list, so references to them still resolve.

## Limits

Caps that cut the axis obliquely, strips with holes in them, strips with extra vertices on their rails, spheres, tori and free-form surfaces are not recognised. A crease below `smoothDeg` also hides a real cone that is nearly tangent to a cylinder. The kit does not call this request yet.
