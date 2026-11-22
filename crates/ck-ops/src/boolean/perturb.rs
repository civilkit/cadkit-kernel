//! Bounded, deterministic perturbation retry for degenerate solid booleans.
//!
//! When a boolean fails due to a geometric degeneracy (grazing/tangent curved-on-curved
//! intersection, seam alignment, or sliver) and the caller explicitly opts in, this module
//! retries the boolean by applying a strictly bounded rigid transform to the tool body:
//! - translation magnitude `<= MAX_TRANSLATION_MM` (`1e-4` mm), and/or
//! - rotation angle `<= MAX_ROTATION_RAD` (`2e-6` rad) about the tool's bounding-box center
//!   (further clamped so no point on the tool's bounding sphere moves by more than `MAX_TRANSLATION_MM`).
//!
//! A perturbed result is accepted ONLY when it succeeds without a sanity mismatch, has a
//! non-empty effective result, and passes both `ck_topo::check::check` and `ck_check::validate`
//! with no new issues relative to the input operands. The exact [`Perturbation`] applied is
//! returned so the caller can emit an explicit warning and never hide precision shifts silently.

use super::apply::BoolOut;
use super::fuzzy::boolean_fuzzy;
use super::Op;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_topo::body::Body;

/// Maximum Euclidean translation allowed in any perturbation attempt (mm).
pub const MAX_TRANSLATION_MM: f64 = 1e-4;

/// Maximum rotation angle allowed in any perturbation attempt (radians).
pub const MAX_ROTATION_RAD: f64 = 2e-6;

/// Exact rigid perturbation applied to the tool body during a bounded retry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Perturbation {
    /// Translation vector applied to the tool body (mm).
    pub translation: Vec3,
    /// Euclidean magnitude of `translation` (mm), always `<= MAX_TRANSLATION_MM`.
    pub distance_mm: f64,
    /// Rotation angle about the tool's bounding-box center (rad), always `<= MAX_ROTATION_RAD`.
    pub rotation_rad: f64,
    /// Unit rotation axis (`Vec3::Z` when `rotation_rad == 0.0`).
    pub axis: Vec3,
    /// 1-based attempt index in the deterministic schedule.
    pub attempt: usize,
}

/// Whether a boolean error string represents a geometric degeneracy eligible for perturbation retry.
pub fn is_retryable_error(err: &str) -> bool {
    (err.starts_with("booleanFailed") && !err.starts_with("booleanFailed;thinFeature="))
        || err.starts_with("booleanTangent")
}

/// Deterministic list of `(translation, axis, angle_rad)` candidates, ordered from smallest
/// perturbation to largest. Every candidate obeys `translation.len() <= MAX_TRANSLATION_MM`
/// and `angle_rad.abs() <= MAX_ROTATION_RAD`.
fn schedule(max_rot: f64) -> Vec<(Vec3, Vec3, f64)> {
    // Asymmetric unit direction breaks X/Y/Z symmetries simultaneously.
    let diag = Vec3::new(0.6, 0.5, 0.624_499_799_839_839_8); // len == 1.0
    let rot_axis = Vec3::new(0.5, 0.624_499_799_839_839_8, 0.6); // len == 1.0
    let small_t = 1e-5_f64.min(MAX_TRANSLATION_MM);
    let max_t = MAX_TRANSLATION_MM;
    let mid_t = 5e-5_f64.min(MAX_TRANSLATION_MM);
    let r1 = (1.5e-6_f64).min(max_rot);

    let mut out = Vec::with_capacity(16);
    // 1..=2: ultra-small off-axis translation (1e-5 mm)
    out.push((diag * small_t, Vec3::Z, 0.0));
    out.push((diag * -small_t, Vec3::Z, 0.0));
    // 3..=4: bounded off-axis translation (1e-4 mm)
    out.push((diag * max_t, Vec3::Z, 0.0));
    out.push((diag * -max_t, Vec3::Z, 0.0));
    // 5..=10: cardinal translations (1e-4 mm)
    out.push((Vec3::new(max_t, 0.0, 0.0), Vec3::Z, 0.0));
    out.push((Vec3::new(-max_t, 0.0, 0.0), Vec3::Z, 0.0));
    out.push((Vec3::new(0.0, max_t, 0.0), Vec3::Z, 0.0));
    out.push((Vec3::new(0.0, -max_t, 0.0), Vec3::Z, 0.0));
    out.push((Vec3::new(0.0, 0.0, max_t), Vec3::Z, 0.0));
    out.push((Vec3::new(0.0, 0.0, -max_t), Vec3::Z, 0.0));
    // 11..=14: pure tiny rotations about tool center
    if r1 > 0.0 {
        out.push((Vec3::ZERO, Vec3::Z, r1));
        out.push((Vec3::ZERO, Vec3::X, r1));
        out.push((Vec3::ZERO, Vec3::Y, r1));
        out.push((Vec3::ZERO, rot_axis, r1));
        // 15..=16: combined bounded translation + rotation
        out.push((diag * mid_t, rot_axis, r1));
        out.push((diag * -mid_t, rot_axis, -r1));
    }
    out
}

/// Retry `a op b` with a strictly bounded, deterministic sequence of tiny rigid perturbations
/// applied to `b`, validating every candidate with both `ck_topo::check::check` and
/// `ck_check::validate`. Returns the first valid result together with the exact [`Perturbation`].
pub fn retry_perturbed(a: &Body, b: &Body, op: Op, tol: f64, fuzz: f64) -> Option<(BoolOut, Perturbation)> {
    let (lo, hi) = b.bbox();
    let (center, half_diag) = if lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z {
        ((lo + hi) * 0.5, 0.5 * (hi - lo).len())
    } else {
        (Vec3::ZERO, 1.0)
    };
    // Clamp rotation so even the farthest corner of the tool stays within MAX_TRANSLATION_MM.
    let max_rot = MAX_ROTATION_RAD.min(MAX_TRANSLATION_MM / half_diag.max(1.0));

    let base_topo = ck_topo::check::check(a, tol).len() + ck_topo::check::check(b, tol).len();
    let base_valid = ck_check::validate(a, tol).len() + ck_check::validate(b, tol).len();

    for (idx, (t, axis, angle)) in schedule(max_rot).into_iter().enumerate() {
        let dist = t.len();
        debug_assert!(dist <= MAX_TRANSLATION_MM * (1.0 + 1e-12));
        debug_assert!(angle.abs() <= MAX_ROTATION_RAD * (1.0 + 1e-12));

        let mut tool = b.clone();
        let xf = if angle != 0.0 && dist > 0.0 {
            Xform::translation(t).compose(&Xform::rotation(center, axis, angle))
        } else if angle != 0.0 {
            Xform::rotation(center, axis, angle)
        } else {
            Xform::translation(t)
        };
        tool.transform(&xf);

        let Ok(out) = boolean_fuzzy(a, &tool, op, tol, fuzz) else {
            continue;
        };
        if out.no_effect || out.mismatch.is_some() || out.body.num_faces() == 0 {
            continue;
        }
        if ck_topo::check::check(&out.body, tol).len() > base_topo {
            continue;
        }
        if ck_check::validate(&out.body, tol).len() > base_valid {
            continue;
        }

        let p = Perturbation {
            translation: t,
            distance_mm: dist,
            rotation_rad: angle.abs(),
            axis: if angle < 0.0 { -axis } else { axis },
            attempt: idx + 1,
        };
        return Some((out, p));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schedule_is_strictly_bounded_and_deterministic() {
        let s1 = schedule(MAX_ROTATION_RAD);
        let s2 = schedule(MAX_ROTATION_RAD);
        assert_eq!(s1, s2);
        assert!(!s1.is_empty());
        for (t, axis, angle) in &s1 {
            assert!(t.len() <= MAX_TRANSLATION_MM * (1.0 + 1e-12), "t={t:?} len={}", t.len());
            assert!(angle.abs() <= MAX_ROTATION_RAD * (1.0 + 1e-12), "angle={angle}");
            assert!((axis.len() - 1.0).abs() < 1e-12, "axis not unit: {axis:?}");
            assert!(t.len() > 0.0 || angle.abs() > 0.0);
        }
    }

    #[test]
    fn retryable_error_classification() {
        assert!(is_retryable_error("booleanFailed"));
        assert!(is_retryable_error("booleanFailed;nearCoincident=F1|F2|1e-5"));
        assert!(is_retryable_error("booleanFailed;edgeFaces=F1|F2;issue=openEdge;at=0|0|0"));
        assert!(is_retryable_error("booleanTangent;euler=1;hint=test"));
        assert!(!is_retryable_error("booleanFailed;thinFeature=1e-6"));
        assert!(!is_retryable_error("booleanBudgetExceeded;elapsedMs=5000;budgetMs=5000"));
        assert!(!is_retryable_error("cutNoEffect"));
    }
}

