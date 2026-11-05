//! Fillets and chamfers (ck-ops/src/blend/*, 17 files in rs_paths.txt).
//!
//! Implemented (RECONSTRUCTED from oracle probes, not ported from the
//! 2 MB of rolling-ball C in march.rs / rolling.rs / stripe.rs ...):
//! [`build::fillet`] / [`build::chamfer`] — constant-radius fillets and
//! equal-distance chamfers along straight edges between planar faces and
//! along circle edges between coaxial plane / cylinder / cone faces
//! (convex / concave), blend faces `B<k>`, as a boolean with a tool (extruded
//! or revolved corner profile); multi-edge selections in rounds; and
//! [`vertex::chamfer_vertex`] (3-valent corner cut) and
//! [`corner::sphere_corner_tool`] (the `V<k>` sphere patch of three equal
//! fillets at a convex planar corner). [`freeform`]: rolling-ball fillets along
//! chains of free-form edges, also built as a boolean tool. Every result passes
//! [`validity`] before it is returned. The reference's own `cap`, `overlap`,
//! `sheet`, `stripe` and `surface` stages have no counterpart here (superseded by
//! the tool + boolean construction, or deferred).
//!
//! Feature-level codes (kernel.strings): `filletChamfer`, `filletType`,
//! `chamferType`, `chamferVertexEdges`, `nothingToBlend`, `missingEdge`,
//! `blendFailed`, `blendTooLarge`, `blendTooLargeMax`, `blendUnsupported`,
//! `blendCorner`, `blendOverflow`, `blendTangent`, `blendSmoothEdge`,
//! `blendNonManifold`, `blendWidthTooLarge(Max)`, `blendSize`,
//! `filletShape` ("largest radius that fits", ": radius … does not fit;
//! about …"). Detail strings: "edge between X shrinks to nothing", "the
//! chain through X runs into itself", "both must move by the same
//! distance", "edge of blend X is gone after an earlier blend",
//! "X collapses and cannot be removed", "blends overlap: a spring not in
//! its blend's face", "… spring leaves its face away from its edges"
//! (pinned: `F3.B0: spring leaves its face away from its edges` for r=30 on
//! a 10 mm box edge), "a convex and a concave blended edge meet at …",
//! "… blended edges meet at …".

mod analytic;
mod apply;
pub mod build;
mod chain;
mod concave_vertex;
pub mod corner;
mod diag;
mod ends;
mod find;
pub mod freeform;
mod inplace;
pub mod g2;
pub mod interp;
mod layout;
pub mod mixed;
mod model;
pub mod overhang;
mod rim_reflex;
mod rounds;
mod torus_corner;
pub mod validity;
pub mod vertex;

/// Feature error `{code, params:{detail?}}` (oracle: regen `errors[]`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlendError {
    /// Extra error params (`blendG2TooLargeMax`: `max`, `shape`).
    pub params: Vec<(String, String)>,
    pub code: String,
    pub detail: Option<String>,
}

impl BlendError {
    pub fn code(c: &str) -> BlendError {
        BlendError { code: c.into(), detail: None, params: vec![] }
    }
    pub fn detail(c: &str, d: &str) -> BlendError {
        BlendError { code: c.into(), detail: Some(d.into()), params: vec![] }
    }
}

pub use build::fillet;
