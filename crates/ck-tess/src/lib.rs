//! ck-tess: tessellation of B-rep bodies into the wire display mesh.
//!
//! - [`cdt`]: constrained Delaunay triangulation (exact predicates).
//! - [`curvature`]: chord/angle tolerances, curve parameter sampling, edge refinement.
//! - [`face`]: per-face triangulation in (u,v) honouring trims, seams, poles.
//! - [`mesh`]: body mesh assembly (`positions/normals/mids/triFace/silhouette`).
#[cfg(feature = "trace")]
#[link(wasm_import_module = "env")]
extern "C" {
    pub fn ck_log(id: i32, a: f64, b: f64, c: f64, d: f64);
}
/// `tr!(id, a, b, c, d)`: debug log of the original's operation ids (no-op without the `trace` feature).
#[macro_export]
macro_rules! tr {
    ($id:expr, $a:expr, $b:expr, $c:expr, $d:expr) => {
        #[cfg(feature = "trace")]
        unsafe {
            $crate::ck_log($id, $a as f64, $b as f64, $c as f64, $d as f64)
        }
    };
}
pub mod cdt;
pub mod curvature;
pub mod face;
pub mod mesh;
