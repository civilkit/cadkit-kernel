//! View-dependent output: the regen "snapshot for the view" (display meshes,
//! edges, silhouette) that every successful regen appends.
//!
//! Wire shape (oracle-flange.json): `body.mesh = {positions,normals,mids:
//! f32 $buf, triFace: i32 $buf}`, `body.silhouette: f32 $buf` — buffers are
//! created mesh-first (indices 0..3) and silhouette last (4).
//! The geometry lives in ck-tess; this module will host the JSON assembly.

pub use crate::session::SNAPSHOT_LABEL;
