//! STUB (not reconstructed yet): compiles to kernel.wasm (ck-ops/src/guide.rs)
//!
//! Guide-curve helpers shared by sweep and loft (guideMode, guideStrength).
//!
//! What is known (packet headers only, nothing ported):
//! * sweep `guides` / `guideMode` (`turn` | `scale` | `stretch`, JSON keys read
//!   by the sweep exec f1182) make the profile follow guide curves while it
//!   travels: `turn` rotates it, `scale` scales it, `stretch` stretches it
//!   between two guides (`sweepGuideStretchTwo`: stretch needs exactly two).
//! * f2706 (guide.rs:235, 8.5 KB, args end in two f64) validates the guides
//!   against profile and path: sweepGuidesCount, sweepGuideNotTouching,
//!   sweepGuideNotAlong, sweepGuideTangent, sweepGuideOnPath,
//!   sweepGuidesSamePoint, sweepGuideDegenerate, sweepGuideStretchTwo; the
//!   regen layer adds sweepGuideMissing / sweepGuidesTwist / sweepGuideShort
//!   / sweepGuidesPath (f1182). f2709 (guide.rs:367) is its f64 clamp helper
//!   (`min > max, or either was NaN`).
//! * guided sweeps are NOT rigid motions of the profile, so they need the
//!   general frame-march of ck-ops/src/sweep.rs (f2878 / f2877), which is not
//!   reconstructed either: `ck_ops::sweep::sweep` handles rigid paths only.
#![allow(unused)]
