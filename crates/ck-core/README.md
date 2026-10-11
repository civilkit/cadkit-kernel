# ck-core

Mathematical foundation for everything above it. No dependencies.

## Modules (pinned by rs_paths.txt)

| module | contents (inferred from strings + wasm) |
|---|---|
| `linalg.rs` | Vec2/Vec3/mat3/mat4, quaternion ops, frame/plane math. The `Ot` orientation table in app.pretty.js (front/top/right with o/x/y/z) mirrors the kernel's std frames. |
| `quad.rs` | quadratic/quartic root finding (used by curve-curve intersection, blend spine solving) |
| `kdtree.rs` | spatial index for point/curve queries |
| `trace.rs` | diagnostic tracing (regen trace output) |
| `xform.rs` | transform application to geometry |

## Interface

Pure math: everything takes/returns owned values (`Vec3`, `[f64; 3]`, ...).
No allocation-heavy types live here. This crate is the FIRST rebuild target:
it is small, mechanically decompilable (f64 arithmetic), and everything
depends on it.

## Rebuild notes

- The original is f64 throughout (wasm doubles). Match NaN/inf semantics.
- Cross-check against the oracle: transformed placements in
  `oracle-flange.json` (components[].place) must reproduce bit-for-bit-ish
  (1e-15 relative) once linalg is right.
