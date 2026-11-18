//! Offset curves and surfaces as B-splines within a tolerance.
//!
//! An offset curve moves a curve `C(t)` by `dist` along the unit direction
//! `C'(t) x N` (3D, `N` the normal of the offset plane, the convention of the
//! solidcore offset curve) or along the left normal `perp(C'(t))` (2D). An
//! offset surface moves `S(u, v)` by `dist` along its unit normal. Neither is
//! a B-spline in general, so the offset is fitted
//! ([`crate::approx`]) to the tolerance and then checked for the ways an
//! offset goes wrong (adapted from the solidcore offset tools, Apache-2.0, see
//! THIRD_PARTY.md):
//!
//! - a tangent (normal) discontinuity of the source: the offset has a gap, so it
//!   is no single curve (surface) - `Err`;
//! - a zero tangent / normal, or a tangent parallel to `N`: the direction is
//!   undefined - `Err`;
//! - an offset distance beyond the local radius of curvature: the offset reverses
//!   (curve: `F'` against `C'`; surface: its normal against the source normal)
//!   and is self-intersecting - `Err`;
//! - 2D curves only: an offset that runs closer than `|dist|` to another part
//!   of the source (a global self-intersection) - `Err`.
//!
//! Every success returns the fit and the largest sampled distance between fit
//! and exact offset at equal parameters (at most the tolerance).

use crate::approx::{approximate_curve_fn, approximate_surface_fn};
use crate::nurbs::basis;
use crate::nurbs::bspline::BSplineCurve;
use crate::nurbs::bsurf::BSplineSurface;
use crate::{Pt, Vec2, Vec3};

fn unit<P: Pt>(d: P) -> Option<P> {
    let l = d.len();
    (l.is_finite() && l > 1e-12).then(|| d / l)
}

/// Offset of `c` by `dist` to the left of its direction of travel (the
/// counter-clockwise normal), as a cubic B-spline within `tol`.
pub fn offset_curve_2d(c: &BSplineCurve<Vec2>, dist: f64, tol: f64) -> Result<(BSplineCurve<Vec2>, f64), String> {
    offset_curve_core(c, dist, tol, &|d: Vec2| unit(d).map(|u| u.perp()), true)
}

/// Offset of `c` by `dist` along the unit direction `C'(t) x normal`, as a cubic
/// B-spline within `tol`. `normal` is the normal of the plane the offset is
/// taken in; it must not be parallel to the tangent anywhere.
pub fn offset_curve_3d(c: &BSplineCurve<Vec3>, normal: Vec3, dist: f64, tol: f64) -> Result<(BSplineCurve<Vec3>, f64), String> {
    let n = unit(normal).ok_or("offset plane normal is zero")?;
    offset_curve_core(
        c,
        dist,
        tol,
        &move |d: Vec3| {
            let t = unit(d)?;
            let b = t.cross(n);
            // tangent (nearly) along the normal: direction undefined
            (b.len() > 1e-6).then(|| b.normalize())
        },
        false,
    )
}

fn offset_curve_core<P: Pt>(
    c: &BSplineCurve<P>,
    dist: f64,
    tol: f64,
    dir: &dyn Fn(P) -> Option<P>,
    global_check: bool,
) -> Result<(BSplineCurve<P>, f64), String> {
    if !dist.is_finite() || !(tol > 0.0) {
        return Err("offset distance must be finite and the tolerance positive".into());
    }
    let (a, b) = c.domain();
    let p = c.degree;
    // tangent discontinuities: a gap (or overlap) opens in the offset
    let h = 1e-7 * (b - a);
    for k in basis::distinct_knots(&c.knots).into_iter().filter(|&k| k > a && k < b) {
        if basis::multiplicity(&c.knots, k, 0.0) >= p.max(1) {
            let (tl, tr) = (unit(c.derivs(k - h, 1)[1]), unit(c.derivs(k + h, 1)[1]));
            if let (Some(tl), Some(tr)) = (tl, tr) {
                if tl.dot(tr) < 1.0 - 1e-12 {
                    return Err(format!("the curve has a tangent discontinuity at parameter {k}: its offset has a gap and is not one curve"));
                }
            }
        }
    }
    // direction defined everywhere (dense sample)
    let spans: Vec<f64> = basis::distinct_knots(&c.knots);
    let mut samples = Vec::new();
    for w in spans.windows(2) {
        for s in 0..16 {
            samples.push(w[0] + (w[1] - w[0]) * s as f64 / 16.0);
        }
    }
    samples.push(b);
    for &t in &samples {
        if dir(c.derivs(t, 1)[1]).is_none() {
            return Err(format!("offset direction undefined at parameter {t}: zero tangent, or tangent parallel to the offset-plane normal"));
        }
    }
    let f = |t: f64| match dir(c.derivs(t, 1)[1]) {
        Some(n) => c.eval(t) + n * dist,
        None => c.eval(t),
    };
    let (fit, dev) = approximate_curve_fn(&f, (a, b), 3, tol, &[])?;
    // local self-intersection: the offset must keep the direction of the curve
    for &t in &samples {
        let (cd, fd) = (c.derivs(t, 1)[1], fit.derivs(t, 1)[1]);
        if !(fd.dot(cd) > 0.0) {
            return Err(format!(
                "offset distance {dist} exceeds the radius of curvature near parameter {t}: the offset reverses (self-intersecting)"
            ));
        }
    }
    if global_check && dist != 0.0 {
        for &t in samples.iter().step_by(2) {
            let q = fit.eval(t);
            let d = c.closest_point(q).1.dist(q);
            if d < dist.abs() * (1.0 - 1e-3) - 2.0 * dev - 2.0 * tol {
                return Err(format!(
                    "the offset runs {d:.6} from another part of the curve, closer than the distance {} (self-intersecting offset)",
                    dist.abs()
                ));
            }
        }
    }
    Ok((fit, dev))
}

/// A normal jump across a knot of multiplicity >= degree makes the offset of
/// the surface discontinuous: refuse it. `Err` names the knot.
pub fn check_creases(s: &BSplineSurface) -> Result<(), String> {
    let ((u0, u1), (v0, v1)) = s.domain();
    let n = 9;
    for (dir_u, knots, deg, (a, b), (c0, c1)) in [(true, &s.knots_u, s.deg_u, (u0, u1), (v0, v1)), (false, &s.knots_v, s.deg_v, (v0, v1), (u0, u1))] {
        let h = 1e-7 * (b - a);
        for k in basis::distinct_knots(knots).into_iter().filter(|&k| k > a && k < b) {
            if basis::multiplicity(knots, k, 0.0) < deg.max(1) {
                continue;
            }
            for i in 0..=n {
                let w = c0 + (c1 - c0) * i as f64 / n as f64;
                let (nl, nr) = if dir_u { (s.normal(k - h, w), s.normal(k + h, w)) } else { (s.normal(w, k - h), s.normal(w, k + h)) };
                if nl.len() > 0.5 && nr.len() > 0.5 && nl.dot(nr) < 1.0 - 1e-10 {
                    return Err(format!(
                        "the surface has a normal discontinuity (crease) at {} = {k}: its offset has a gap and is not one surface",
                        if dir_u { "u" } else { "v" }
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Offset of `s` by `dist` along its unit normal as a bicubic B-spline within `tol`
/// (same parameter domain). Refuses creases, degenerate normals and offsets
/// beyond the radius of curvature (see the module notes).
pub fn offset_surface(s: &BSplineSurface, dist: f64, tol: f64) -> Result<(BSplineSurface, f64), String> {
    check_creases(s)?;
    let (ur, vr) = s.domain();
    approximate_offset_surface(
        &|u, v| s.eval(u, v) + s.normal(u, v) * dist,
        &|u, v| s.normal(u, v),
        ur,
        vr,
        tol,
        &[],
        &[],
    )
}

/// Fit the offset surface given as the evaluator `offset(u, v)` (the offset
/// points) whose source has the unit normal `base_normal(u, v)` (the direction
/// the offset moved along, for the fold-over check) as a bicubic B-spline.
pub fn approximate_offset_surface(
    offset: &dyn Fn(f64, f64) -> Vec3,
    base_normal: &dyn Fn(f64, f64) -> Vec3,
    u_range: (f64, f64),
    v_range: (f64, f64),
    tol: f64,
    break_u: &[f64],
    break_v: &[f64],
) -> Result<(BSplineSurface, f64), String> {
    let n = 12usize;
    let at = |r: (f64, f64), i: usize| r.0 + (r.1 - r.0) * i as f64 / n as f64;
    for i in 0..=n {
        for j in 0..=n {
            let (u, v) = (at(u_range, i), at(v_range, j));
            let bn = base_normal(u, v);
            if !(bn.len() > 0.5) || !bn.x.is_finite() || !bn.y.is_finite() || !bn.z.is_finite() {
                return Err(format!("degenerate surface normal at ({u}, {v}): the offset direction is undefined"));
            }
        }
    }
    let (fit, dev) = approximate_surface_fn(offset, u_range, v_range, (3, 3), tol, break_u, break_v)?;
    // fold-over: the offset's own normal must keep the sense of the source's
    let m = 16usize;
    for i in 0..=m {
        for j in 0..=m {
            let u = u_range.0 + (u_range.1 - u_range.0) * i as f64 / m as f64;
            let v = v_range.0 + (v_range.1 - v_range.0) * j as f64 / m as f64;
            let fnrm = fit.normal(u, v);
            if fnrm.len() > 0.5 && fnrm.dot(base_normal(u, v)) <= 0.0 {
                return Err(format!(
                    "offset distance exceeds the radius of curvature near ({u:.6}, {v:.6}): the offset folds over itself (self-intersecting)"
                ));
            }
        }
    }
    Ok((fit, dev))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::approx::fn_curve_deviation;
    use std::f64::consts::PI;

    /// Quarter-ish arc of radius 5 as a B-spline (sampled to a good fit).
    fn arc2(r: f64, a: f64) -> BSplineCurve<Vec2> {
        let f = |t: f64| Vec2::new(r * t.cos(), r * t.sin());
        approximate_curve_fn(&f, (0.0, a), 3, 1e-9, &[]).unwrap().0
    }

    #[test]
    fn offset_2d_of_arc_is_arc() {
        let c = arc2(5.0, 1.5);
        for d in [0.7, -0.7] {
            let (o, dev) = offset_curve_2d(&c, d, 1e-6).unwrap();
            assert!(dev <= 1e-6);
            // left of a CCW arc is toward the centre: radius 5 - d
            for i in 0..=40 {
                let t = 1.5 * i as f64 / 40.0;
                let q = o.eval(t);
                assert!((q.len() - (5.0 - d)).abs() < 2e-6, "{} at {t}", q.len());
            }
        }
        // beyond the radius of curvature (towards the centre): reverses
        assert!(offset_curve_2d(&c, 5.5, 1e-6).is_err());
        assert!(offset_curve_2d(&c, 0.1, 0.0).is_err());
    }

    #[test]
    fn offset_2d_s_curve_matches_exact() {
        let f = |t: f64| Vec2::new(t, (1.2 * t).sin());
        let c = approximate_curve_fn(&f, (0.0, 4.0), 3, 1e-9, &[]).unwrap().0;
        let d = 0.3;
        let (o, dev) = offset_curve_2d(&c, d, 1e-6).unwrap();
        assert!(dev <= 1e-6);
        let exact = |t: f64| {
            let tg = c.derivs(t, 1)[1];
            c.eval(t) + tg.normalize().perp() * d
        };
        assert!(fn_curve_deviation(&exact, (0.0, 4.0), &o, 200) <= 2e-6);
        // every point of the offset is |d| from the curve
        for i in 0..=50 {
            let q = o.eval(4.0 * i as f64 / 50.0);
            assert!((c.closest_point(q).1.dist(q) - d).abs() < 1e-4);
        }
    }

    #[test]
    fn offset_2d_refusals() {
        // a corner: tangent discontinuity
        let corner = BSplineCurve::<Vec2>::new(
            1,
            vec![0., 0., 1., 2., 2.],
            vec![Vec2::new(0., 0.), Vec2::new(1., 0.), Vec2::new(1., 1.)],
        )
        .unwrap();
        assert!(offset_curve_2d(&corner, 0.1, 1e-6).unwrap_err().contains("discontinuity"));
        // a tight U-turn offset on the concave side loops back on itself
        let u = BSplineCurve::<Vec2>::new(
            3,
            vec![0., 0., 0., 0., 1., 1., 1., 1.],
            vec![Vec2::new(0., 0.), Vec2::new(4., 0.), Vec2::new(4., 1.), Vec2::new(0., 1.)],
        )
        .unwrap();
        let (l, r) = (offset_curve_2d(&u, 0.9, 1e-5), offset_curve_2d(&u, -0.9, 1e-5));
        assert!(l.is_err() != r.is_err(), "exactly the concave side refuses");
        // zero tangent
        let z = BSplineCurve::<Vec2>::new(1, vec![0., 0., 1., 1.], vec![Vec2::new(1., 1.), Vec2::new(1., 1.)]).unwrap();
        assert!(offset_curve_2d(&z, 0.1, 1e-6).is_err());
    }

    #[test]
    fn offset_3d_helix_and_normal() {
        let f = |t: f64| Vec3::new(4.0 * t.cos(), 4.0 * t.sin(), 0.0);
        let c = approximate_curve_fn(&f, (0.0, PI), 3, 1e-9, &[]).unwrap().0;
        // normal Z: offset direction C' x Z points outward (radial) for a CCW circle
        let (o, dev) = offset_curve_3d(&c, Vec3::Z, 0.5, 1e-6).unwrap();
        assert!(dev <= 1e-6);
        for i in 0..=30 {
            let t = PI * i as f64 / 30.0;
            assert!((o.eval(t).len() - 4.5).abs() < 2e-6);
            assert!(o.eval(t).z.abs() < 2e-6);
        }
        // tangent parallel to the normal, and a zero normal
        let line = BSplineCurve::<Vec3>::new(1, vec![0., 0., 1., 1.], vec![Vec3::ZERO, Vec3::Z]).unwrap();
        assert!(offset_curve_3d(&line, Vec3::Z, 1.0, 1e-6).is_err());
        assert!(offset_curve_3d(&c, Vec3::ZERO, 1.0, 1e-6).is_err());
        // inward past the radius
        assert!(offset_curve_3d(&c, Vec3::Z, -4.5, 1e-6).is_err());
    }

    fn dome() -> BSplineSurface {
        // paraboloid-ish patch z = 1 - (x^2 + y^2)/8 on [-2,2]^2, curvature radius ~ 4
        let f = |u: f64, v: f64| Vec3::new(u, v, 1.0 - (u * u + v * v) / 8.0);
        approximate_surface_fn(&f, (-2.0, 2.0), (-2.0, 2.0), (3, 3), 1e-9, &[], &[]).unwrap().0
    }

    #[test]
    fn offset_surface_of_patch() {
        let s = dome();
        for d in [0.4, -0.4] {
            let (o, dev) = offset_surface(&s, d, 1e-5).unwrap();
            assert!(dev <= 1e-5);
            assert_eq!(o.domain(), s.domain());
            for i in 0..=10 {
                for j in 0..=10 {
                    let (u, v) = (-2.0 + 0.4 * i as f64, -2.0 + 0.4 * j as f64);
                    let exact = s.eval(u, v) + s.normal(u, v) * d;
                    assert!(o.eval(u, v).dist(exact) <= 2e-5);
                    // |d| from the base surface
                    let (a, b) = s.closest_param(o.eval(u, v));
                    assert!((s.eval(a, b).dist(o.eval(u, v)) - d.abs()).abs() < 1e-3);
                }
            }
        }
        // radius of curvature at the apex is 4 (1/kappa with z = -r^2/8): offsetting
        // 4.5 against the curvature folds the surface
        assert!(offset_surface(&s, -4.5, 1e-4).unwrap_err().contains("folds") || offset_surface(&s, -4.5, 1e-4).is_err());
        assert!(offset_surface(&s, 4.5, 1e-4).is_ok());
    }

    #[test]
    fn offset_surface_refuses_crease() {
        let s = BSplineSurface::new(
            1,
            1,
            vec![0., 0., 1., 2., 2.],
            vec![0., 0., 1., 1.],
            3,
            2,
            vec![
                Vec3::new(0., 0., 0.),
                Vec3::new(0., 1., 0.),
                Vec3::new(1., 0., 0.),
                Vec3::new(1., 1., 0.),
                Vec3::new(2., 0., 1.),
                Vec3::new(2., 1., 1.),
            ],
            None,
        )
        .unwrap();
        assert!(offset_surface(&s, 0.1, 1e-4).unwrap_err().contains("crease"));
    }
}
