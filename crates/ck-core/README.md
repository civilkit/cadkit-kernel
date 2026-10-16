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

## Build notes

- f64 throughout (wasm doubles). Match NaN/inf semantics.
- Cross-check against the oracle: transformed placements in
  `oracle-flange.json` (components[].place) must reproduce bit-for-bit-ish
  (1e-15 relative) once linalg is right.

## Status (first rebuild pass)

- `linalg`: Vec2/Vec3/Mat3 (column storage)/Mat4/Quat/Plane/Frame, angle
  helpers, tolerance constants (values = most frequent `f64.const`s; names
  GUESSed). `Vec3::normalize` matches the kernel's inlined idiom exactly
  (`len > 1e-300` and finite, else ZERO; multiply by `1/len`).
- Frame memory layout pinned by func[912]: `{o@0, x@0x18, y@0x30, z@0x48}`;
  point-plane distance reads the normal from `z`.
- `xform`: `Xform {r: Mat3, t}`; `Xform::from_frame`/`to_frame` bridge for
  `components[].place`; `a * b` applies `b` first.
- `trace`: the `"trace"` array for `debug:true` requests, byte-exact vs the
  oracle with a stepping clock (see module docs for clock-read counts).
- `quad`: quadratic/cubic/quartic real roots + derivative isolation.
- `kdtree`: nearest / k-nearest / radius / weld.
- Differential tests (`tests/ffi_diff.rs`) run only when
  `$CK_ORACLE_DIR/libkerneloracle.a` exists (default `../c2`); build.rs sets
  `cfg(ck_oracle)`. Tests share one C instance and are serialised by a mutex.
