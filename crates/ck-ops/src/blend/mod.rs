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
//! fillets at a convex planar corner). The remaining stubs
//! (`cap`, `g2`, `interp`, `overlap`, `sheet`, `stripe`, `surface`) are
//! classified in docs/ck-ops-stubs.md; the original's `inplace`, `rolling`,
//! `march`, `chain`, `spring` and `split` are covered by `build` /
//! `freeform` and were removed.
//!
//! [`freeform`]: rolling-ball fillets along chains of free-form edges (B-spline /
//! revolution faces, spline intersection edges), also built as a boolean tool.
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
pub mod corner;
pub mod freeform;
pub mod g2;
pub mod interp;
pub mod overlap;
pub mod sheet;
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
