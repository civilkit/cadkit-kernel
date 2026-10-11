//! ck-tess: tessellation of B-rep bodies into the wire display mesh.
//!
//! - [`cdt`]: constrained Delaunay triangulation (exact predicates).
//! - [`curvature`]: chord/angle tolerances, curve parameter sampling, edge refinement.
//! - [`face`]: per-face triangulation in (u,v) honouring trims, seams, poles.
//! - [`mesh`]: body mesh assembly (`positions/normals/mids/triFace/silhouette`).
pub mod cdt;
pub mod comb;
pub mod curvature;
pub mod face;
pub mod mesh;
