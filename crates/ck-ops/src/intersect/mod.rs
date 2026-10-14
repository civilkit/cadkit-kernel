//! Intersections: curve-curve (cc), curve-surface (cs), surface-surface (ss).
//!
//! Clean implementations (analytic closed forms via ck-core::quad where the
//! pair allows, Newton refinement otherwise), not decompilations: the
//! original routines have not been located in the WAT. All results are
//! deterministic (sorted by the first curve's parameter).
//!
//! Shared conventions:
//! - a curve is given with a parameter range `(t0, t1)`; lines may use an
//!   infinite range, periodic curves may use a range outside [0, 2π) (edge
//!   ranges such as [4, 4+2π]) — returned parameters are mapped into it.
//! - `tol` is a model-space distance (points closer than `tol` coincide).

pub mod cc;
pub mod cs;
pub mod ss;

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;

/// Default modelling tolerance for the ops (mm).
/// GUESS: matches ck-core EPS_LINEAR (1e-6); the kernel's own value is unknown.
pub const TOL: f64 = 1e-6;

/// Map parameter `t` of curve `c` into `[r.0 - slack, r.1 + slack]`, using
/// the curve period for periodic carriers. `None` when outside. The result
/// is clamped into the closed range.
pub fn fit_param(c: &Curve, t: f64, r: (f64, f64), slack: f64) -> Option<f64> {
    let inside = |x: f64| x >= r.0 - slack && x <= r.1 + slack;
    let clamp = |x: f64| x.max(r.0).min(r.1);
    if let Some(p) = c.period() {
        if r.0.is_finite() {
            let k = ((r.0 - slack - t) / p).ceil();
            let x = t + k * p;
            if inside(x) {
                return Some(clamp(x));
            }
            return None;
        }
    }
    if inside(t) {
        Some(clamp(t))
    } else {
        None
    }
}

/// All parameters in range equivalent to `t` (periodic curves may hit the
/// same point at both ends of a closed range, e.g. t0 and t0 + 2π).
pub fn fit_params_all(c: &Curve, t: f64, r: (f64, f64), slack: f64) -> Vec<f64> {
    let mut out = vec![];
    if let (Some(p), true) = (c.period(), r.0.is_finite()) {
        let mut k = ((r.0 - slack - t) / p).ceil();
        loop {
            let x = t + k * p;
            if x > r.1 + slack {
                break;
            }
            out.push(x.max(r.0).min(r.1));
            k += 1.0;
        }
    } else if let Some(x) = fit_param(c, t, r, slack) {
        out.push(x);
    }
    out
}

/// Parameter slack equivalent to a model distance `tol` at `t`.
pub fn param_slack(c: &Curve, t: f64, tol: f64) -> f64 {
    let sp = c.deriv(if t.is_finite() { t } else { 0.0 }).len();
    if sp > 1e-300 {
        tol / sp
    } else {
        tol
    }
}

/// Clamp an infinite range to a finite one around the closest point of a
/// box (used when sampling unbounded lines).
pub fn finite_range(c: &Curve, r: (f64, f64), lo: Vec3, hi: Vec3) -> (f64, f64) {
    if r.0.is_finite() && r.1.is_finite() {
        return r;
    }
    let mut a = f64::INFINITY;
    let mut b = f64::NEG_INFINITY;
    for i in 0..8 {
        let p = Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let t = c.closest_param(p);
        a = a.min(t);
        b = b.max(t);
    }
    (if r.0.is_finite() { r.0 } else { a }, if r.1.is_finite() { r.1 } else { b })
}

/// Solve a 3×3 linear system (rows `a`) by Cramer's rule; `None` if singular.
pub fn solve3(a: [Vec3; 3], b: [f64; 3]) -> Option<Vec3> {
    let det = a[0].dot(a[1].cross(a[2]));
    if det.abs() < 1e-300 {
        return None;
    }
    // columns of the inverse transpose: x = (b0 (a1×a2) + b1 (a2×a0) + b2 (a0×a1)) / det
    Some((a[1].cross(a[2]) * b[0] + a[2].cross(a[0]) * b[1] + a[0].cross(a[1]) * b[2]) / det)
}

/// Signed distance-ish residual of point `p` to surface `s`: (closest uv,
/// closest point, unit normal there).
pub fn project(s: &Surface, p: Vec3) -> ((f64, f64), Vec3, Vec3) {
    let (u, v, q) = s.closest_point(p);
    let n = s.normal(u, v);
    ((u, v), q, n)
}
