# ck-tess

Tessellation: faces to display meshes. Depends on ck-topo, ck-geom.

## Modules

- `cdt.rs` — constrained Delaunay triangulation (NOTE: the civilkit
  sectionprops-rs crate has a proven zero-dep CDT + exact predicates that
  can seed this — see /path/to/projects/sectionprops-rs/src/mesher/)
- `face.rs` — per-face triangulation (analytic surfaces get structured
  meshes; NURBS get adaptive)
- `curvature.rs` — curvature-driven sizing
- `mesh.rs` — the output mesh + silhouettes

## Interface

Output is the buffer-backed `body.mesh`:
`positions/normals/mids` f32 arrays + `triFace` i32 (face id per triangle),
plus `silhouette` buffer. Types: f32|f64|u32|i32|u8, offset from the
8-aligned post-JSON base. The golden has 1,544,040 f32 positions
(= 128,670 triangles x 12 floats... check exact math when rebuilding) and
171,560 i32 triFace entries.

## Rebuild notes

- Match triangle counts and vertex ordering conventions against the golden
  buffers (offsets are in the JSON, bytes in the trailing binary).
