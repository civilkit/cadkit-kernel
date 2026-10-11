//! Fillets and chamfers (ck-ops/src/blend/*, 17 files in rs_paths.txt).
//!
//! Implemented: [`build::fillet`] — constant-radius fillet along straight
//! edges between planar faces (convex / concave), blend faces `B<k>`.
//! Everything else is a documented stub (see each file's header for the
//! kernel.strings messages it owns).
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

pub mod build;
pub mod cap;
pub mod chain;
pub mod corner;
pub mod g2;
pub mod inplace;
pub mod interp;
pub mod march;
pub mod overlap;
pub mod rolling;
pub mod sheet;
pub mod split;
pub mod spring;
pub mod stripe;
pub mod surface;
pub mod vertex;

/// Feature error `{code, params:{detail?}}` (oracle: regen `errors[]`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlendError {
    pub code: String,
    pub detail: Option<String>,
}

impl BlendError {
    pub fn code(c: &str) -> BlendError {
        BlendError { code: c.into(), detail: None }
    }
    pub fn detail(c: &str, d: &str) -> BlendError {
        BlendError { code: c.into(), detail: Some(d.into()) }
    }
}

pub use build::fillet;
