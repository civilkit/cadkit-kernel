//! Exact conversions analytic -> NURBS (Piegl & Tiller ch. 7/8).
//!
//! - conic arcs: rational quadratics, ≤ 90° per segment, inner weight
//!   cos(Δθ/2) (A7.1); ellipses are the affine image of the unit circle.
//! - cylinder/cone/sphere/torus/revolution: surfaces of revolution (A8.1)
//!   with u = angle (rational quadratic) and v = profile parameter;
//!   extrusion: profile × linear.
//! Used by the STEP writer (RATIONAL_B_SPLINE_SURFACE output) and by ops
//! that need a uniform NURBS representation ("rational weights nurbs" keys).
//!
//! NOTE: NURBS parameterisations of conics are not angle-proportional; the
//! domain is mapped so the END parameters match ([t0,t1] / [u0,u1]×[v0,v1]),
//! interior parameters differ from the analytic ones.

use crate::curve::Curve;
use crate::curve2::Curve2;
use crate::nurbs::bspline::BSplineCurve;
use crate::nurbs::bsurf::BSplineSurface;
use crate::surface::Surface;
use crate::{Frame, Pt, Vec2, Vec3};
use std::f64::consts::{FRAC_PI_2, TAU};

/// Unit-circle arc θ∈[t0,t1] (t1 > t0, ≤ 2π span) as local 2D rational
/// quadratic control points + weights + knots on [0,1].
pub fn unit_arc(t0: f64, t1: f64) -> (Vec<Vec2>, Vec<f64>, Vec<f64>) {
    let mut te = t1;
    if te < t0 {
        te += TAU;
    }
    let theta = (te - t0).min(TAU);
    let narcs = ((theta / FRAC_PI_2) - 1e-12).ceil().max(1.0) as usize;
    let dth = theta / narcs as f64;
    let w1 = (dth / 2.0).cos();
    let mut pts = vec![Vec2::new(t0.cos(), t0.sin())];
    let mut ws = vec![1.0];
    let mut ang = t0;
    for _ in 0..narcs {
        let mid = ang + dth / 2.0;
        ang += dth;
        pts.push(Vec2::new(mid.cos(), mid.sin()) / w1);
        ws.push(w1);
        pts.push(Vec2::new(ang.cos(), ang.sin()));
        ws.push(1.0);
    }
    let mut knots = vec![0.0; 3];
    for i in 1..narcs {
        let k = i as f64 / narcs as f64;
        knots.push(k);
        knots.push(k);
    }
    knots.extend([1.0; 3]);
    (pts, ws, knots)
}

/// Rational quadratic arc of p = o + a cos t·x + b sin t·y, t ∈ [t0,t1],
/// with domain [t0,t1].
pub fn conic_arc(frame: &Frame, a: f64, b: f64, t0: f64, t1: f64) -> BSplineCurve {
    let (pts, ws, knots) = unit_arc(t0, t1);
    let ctrl = pts.iter().map(|p| frame.o + frame.x * (a * p.x) + frame.y * (b * p.y)).collect();
    let c = BSplineCurve { degree: 2, knots, ctrl, weights: Some(ws) };
    if t1 > t0 {
        c.reparam(t0, t1)
    } else {
        c
    }
}

/// Degree-1 segment a -> b on [t0,t1].
pub fn segment<P: Pt>(a: P, b: P, t0: f64, t1: f64) -> BSplineCurve<P> {
    BSplineCurve { degree: 1, knots: vec![t0, t0, t1, t1], ctrl: vec![a, b], weights: None }
}

/// Exact NURBS of `c` restricted to [t0,t1] (domain [t0,t1]).
pub fn curve_to_nurbs(c: &Curve, t0: f64, t1: f64) -> BSplineCurve {
    match c {
        Curve::Line { .. } => segment(c.eval(t0), c.eval(t1), t0, t1),
        Curve::Circle { frame, r } => conic_arc(frame, *r, *r, t0, t1),
        Curve::Ellipse { frame, a, b } => conic_arc(frame, *a, *b, t0, t1),
        // Conic/sinusoid: no closed rational form in this parameterisation
        // is wired up — approximate by a cubic through 32 samples.
        Curve::Conic { .. } | Curve::Sinusoid { .. } => {
            let n = 32;
            let pts: Vec<Vec3> = (0..=n).map(|i| c.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect();
            let u: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
            let knots = crate::fit::averaged_knots(&u, 3);
            let b = crate::fit::interpolate_with(&pts, 3, &u, knots)
                .or_else(|_| crate::fit::interpolate(&pts, 3, crate::fit::Param::ChordLength))
                .unwrap_or_else(|_| segment(c.eval(t0), c.eval(t1), t0, t1));
            b.reparam(t0, t1)
        }
        Curve::BSpline(b) => b.segment(t0, t1),
    }
}

/// 2D analogue of [`curve_to_nurbs`].
pub fn curve2_to_nurbs(c: &Curve2, t0: f64, t1: f64) -> BSplineCurve<Vec2> {
    let conic = |cc: Vec2, x: Vec2, a: f64, b: f64, ccw: bool| {
        let y = if ccw { x.perp() } else { -x.perp() };
        let (pts, ws, knots) = unit_arc(t0, t1);
        let ctrl = pts.iter().map(|p| cc + x * (a * p.x) + y * (b * p.y)).collect();
        BSplineCurve { degree: 2, knots, ctrl, weights: Some(ws) }.reparam(t0, t1)
    };
    match c {
        Curve2::Line { .. } => segment(c.eval(t0), c.eval(t1), t0, t1),
        Curve2::Circle { c: cc, x, r, ccw } => conic(*cc, *x, *r, *r, *ccw),
        Curve2::Ellipse { c: cc, x, a, b, ccw } => conic(*cc, *x, *a, *b, *ccw),
        Curve2::BSpline(b) => b.segment(t0, t1),
    }
}

/// Surface of revolution of a NURBS profile about (axis_o, axis) over the
/// angle range [a0,a1] (A8.1). u = angle (domain [a0,a1]), v = profile.
/// Angle 0 for each profile point is its own radial direction (STEP
/// SURFACE_OF_REVOLUTION semantics: σ(0,v) = λ(v)).
pub fn revolve(profile: &BSplineCurve, axis_o: Vec3, axis: Vec3, a0: f64, a1: f64) -> BSplineSurface {
    let z = axis.normalize();
    let (arc, aw, ak) = unit_arc(a0, a1);
    let nu = arc.len();
    let nv = profile.ctrl.len();
    let mut ctrl = Vec::with_capacity(nu * nv);
    let mut ws = Vec::with_capacity(nu * nv);
    // unit_arc points are relative to angle 0; profile point at angle 0 is
    // P = O + X, so a local arc point (c, s) maps to O + X c + (z×X) s.
    for i in 0..nu {
        for j in 0..nv {
            let p = profile.ctrl[j];
            let o = axis_o + z * (p - axis_o).dot(z);
            let x = p - o;
            let y = z.cross(x);
            ctrl.push(o + x * arc[i].x + y * arc[i].y);
            ws.push(aw[i] * profile.weight(j));
        }
    }
    let s = BSplineSurface {
        deg_u: 2,
        deg_v: profile.degree,
        knots_u: ak.iter().map(|k| a0 + (a1 - a0) * k).collect(),
        knots_v: profile.knots.clone(),
        nu,
        nv,
        ctrl,
        weights: Some(ws),
    };
    s
}

/// Linear extrusion of a profile: σ = λ(u) + v·dir, v ∈ [v0,v1].
pub fn extrude(profile: &BSplineCurve, dir: Vec3, v0: f64, v1: f64) -> BSplineSurface {
    let nu = profile.ctrl.len();
    let mut ctrl = Vec::with_capacity(nu * 2);
    let mut ws = Vec::with_capacity(nu * 2);
    for i in 0..nu {
        for v in [v0, v1] {
            ctrl.push(profile.ctrl[i] + dir * v);
            ws.push(profile.weight(i));
        }
    }
    BSplineSurface {
        deg_u: profile.degree,
        deg_v: 1,
        knots_u: profile.knots.clone(),
        knots_v: vec![v0, v0, v1, v1],
        nu,
        nv: 2,
        ctrl,
        weights: profile.weights.as_ref().map(|_| ws),
    }
}

/// Exact NURBS of a surface patch [u0,u1]×[v0,v1]. Corner parameters
/// match; interior parameters of rational directions do not. Offsets are
/// not exactly representable -> Err.
pub fn surface_to_nurbs(s: &Surface, (u0, u1): (f64, f64), (v0, v1): (f64, f64)) -> Result<BSplineSurface, String> {
    let rev = |prof: BSplineCurve, f: &Frame| {
        // rotate so angle u0 is measured from frame.x: profiles are built in
        // the x-z half plane, so revolve [u0,u1] directly.
        revolve(&prof, f.o, f.z, u0, u1)
    };
    match s {
        Surface::Plane { frame } => {
            let e = |u: f64, v: f64| frame.o + frame.x * u + frame.y * v;
            let mut b = BSplineSurface::bilinear(e(u0, v0), e(u1, v0), e(u0, v1), e(u1, v1));
            b.knots_u = vec![u0, u0, u1, u1];
            b.knots_v = vec![v0, v0, v1, v1];
            Ok(b)
        }
        Surface::Cylinder { frame, r } => {
            let a = frame.o + frame.x * *r;
            Ok(rev(segment(a + frame.z * v0, a + frame.z * v1, v0, v1), frame))
        }
        Surface::Cone { frame, r, semi_angle } => {
            let t = semi_angle.tan();
            let p = |v: f64| frame.o + frame.x * (r + v * t) + frame.z * v;
            Ok(rev(segment(p(v0), p(v1), v0, v1), frame))
        }
        Surface::Sphere { frame, r } => {
            let pf = Frame { o: frame.o, x: frame.x, y: frame.z, z: -frame.y };
            Ok(rev(conic_arc(&pf, *r, *r, v0, v1), frame))
        }
        Surface::Torus { frame, major, minor } => {
            let pf = Frame { o: frame.o + frame.x * *major, x: frame.x, y: frame.z, z: -frame.y };
            Ok(rev(conic_arc(&pf, *minor, *minor, v0, v1), frame))
        }
        Surface::Revolution { profile, axis_o, axis } => {
            let prof = curve_to_nurbs(profile, v0, v1);
            Ok(revolve(&prof, *axis_o, *axis, u0, u1))
        }
        Surface::Extrusion { profile, dir } => Ok(extrude(&curve_to_nurbs(profile, u0, u1), *dir, v0, v1)),
        Surface::BSpline(b) => {
            let mut c = (**b).clone();
            let ((a0, a1), (b0, b1)) = c.domain();
            if u0 > a0 || u1 < a1 || v0 > b0 || v1 < b1 {
                // trim by knot insertion
                c = trim_surface(&c, (u0, u1), (v0, v1));
            }
            Ok(c)
        }
        Surface::Screw { .. } => {
            // interpolated tensor surface, 48 rows per turn (the density the screw sweeps use)
            let rows = ((48.0 * (u1 - u0).abs() / std::f64::consts::TAU).ceil() as usize).max(8);
            let us: Vec<f64> = (0..=rows).map(|i| u0 + (u1 - u0) * i as f64 / rows as f64).collect();
            let nv = if matches!(s, Surface::Screw { profile, .. } if matches!(**profile, Curve::Line { .. })) { 1 } else { 24 };
            let vs: Vec<f64> = (0..=nv).map(|j| v0 + (v1 - v0) * j as f64 / nv as f64).collect();
            let grid: Vec<Vec<Vec3>> = us.iter().map(|&u| vs.iter().map(|&v| s.eval(u, v)).collect()).collect();
            crate::fit::grid_surface(&grid, &us, &vs, 5, if nv == 1 { 1 } else { 3 })
        }
        Surface::Offset { .. } => Err("offset surfaces have no exact NURBS form".into()),
    }
}

/// Exact sub-patch of a B-spline surface via knot insertion.
fn trim_surface(s: &BSplineSurface, (u0, u1): (f64, f64), (v0, v1): (f64, f64)) -> BSplineSurface {
    // transpose trick: trim columns as curves
    let trim_u = |s: &BSplineSurface, a: f64, b: f64| -> BSplineSurface {
        let mut cols = Vec::new();
        for j in 0..s.nv {
            let c = BSplineCurve {
                degree: s.deg_u,
                knots: s.knots_u.clone(),
                ctrl: (0..s.nu).map(|i| s.cp(i, j)).collect(),
                weights: s.weights.as_ref().map(|_| (0..s.nu).map(|i| s.weight(i, j)).collect()),
            };
            cols.push(c.segment(a, b));
        }
        let nu = cols[0].ctrl.len();
        let mut ctrl = vec![Vec3::ZERO; nu * s.nv];
        let mut ws = vec![1.0; nu * s.nv];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nu {
                ctrl[i * s.nv + j] = c.ctrl[i];
                ws[i * s.nv + j] = c.weight(i);
            }
        }
        BSplineSurface { knots_u: cols[0].knots.clone(), nu, ctrl, weights: s.weights.as_ref().map(|_| ws), ..s.clone() }
    };
    let a = trim_u(s, u0, u1);
    trim_u(&a.transposed(), v0, v1).transposed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_from;
    use crate::testutil::*;

    #[test]
    fn nurbs_circle_is_exact() {
        let f = frame_from(Vec3::new(1., 2., 3.), Vec3::new(1., 1., 0.), Vec3::Z);
        for (t0, t1) in [(0.0, TAU), (0.3, 1.2), (1.0, 5.0)] {
            let c = conic_arc(&f, 2.5, 2.5, t0, t1);
            assert_eq!(c.domain(), (t0, t1));
            assert!(vclose(c.start(), f.o + (f.x * t0.cos() + f.y * t0.sin()) * 2.5, 1e-14));
            assert!(vclose(c.end(), f.o + (f.x * t1.cos() + f.y * t1.sin()) * 2.5, 1e-14));
            for i in 0..=100 {
                let t = t0 + (t1 - t0) * i as f64 / 100.0;
                let p = c.eval(t);
                assert!(close(p.dist(f.o), 2.5, 1e-14));
                assert!((p - f.o).dot(f.z).abs() < 1e-13);
            }
            assert!(close(c.length(), 2.5 * (t1 - t0), 1e-9));
        }
        let full = conic_arc(&f, 1.0, 1.0, 0.0, TAU);
        assert_eq!(full.ctrl.len(), 9);
        assert!(close(full.weights.as_ref().unwrap()[1], std::f64::consts::FRAC_1_SQRT_2, 1e-15));
    }

    #[test]
    fn ellipse_and_curve2() {
        let f = Frame::identity();
        let e = conic_arc(&f, 3.0, 1.0, 0.0, TAU);
        for i in 0..=50 {
            let p = e.eval(i as f64 / 50.0 * TAU);
            assert!(close((p.x / 3.0).powi(2) + p.y * p.y, 1.0, 1e-13));
        }
        let c2 = Curve2::Circle { c: Vec2::new(1., 0.), x: Vec2::Y, r: 2.0, ccw: false };
        let n = curve2_to_nurbs(&c2, 0.5, 2.0);
        assert!(v2close(n.start(), c2.eval(0.5), 1e-14));
        assert!(v2close(n.end(), c2.eval(2.0), 1e-14));
        assert!(close(n.eval(1.1).dist(Vec2::new(1., 0.)), 2.0, 1e-14));
    }

    #[test]
    fn conic_and_sinusoid_nurbs_approx() {
        // no exact rational form wired up: the interpolated spline tracks
        // the carrier closely
        let c = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, 1.3);
        let n = curve_to_nurbs(&c, -0.5, 0.6);
        for i in 0..=20 {
            let t = -0.5 + 1.1 * i as f64 / 20.0;
            assert!(vclose(n.eval(t), c.eval(t), 1e-6), "conic t={t}");
        }
        let s = Curve::sinusoid(Vec3::new(1., 1., 0.), Vec3::Z, Vec3::X, 0.5, 2.0, 0.3);
        let n = curve_to_nurbs(&s, 0.0, 3.0);
        for i in 0..=20 {
            let t = 3.0 * i as f64 / 20.0;
            assert!(vclose(n.eval(t), s.eval(t), 1e-6), "sinusoid t={t}");
        }
    }

    fn check_on(s: &Surface, n: &BSplineSurface, ur: (f64, f64), vr: (f64, f64)) {
        // corners match parameters, all samples lie on the analytic surface
        for (u, v) in [(ur.0, vr.0), (ur.1, vr.0), (ur.0, vr.1), (ur.1, vr.1)] {
            assert!(vclose(n.eval(u, v), s.eval(u, v), 1e-12), "{} corner", s.kind());
        }
        for i in 0..=8 {
            for j in 0..=8 {
                let u = ur.0 + (ur.1 - ur.0) * i as f64 / 8.0;
                let v = vr.0 + (vr.1 - vr.0) * j as f64 / 8.0;
                let p = n.eval(u, v);
                assert!(s.distance(p) < 1e-9, "{} off-surface {}", s.kind(), s.distance(p));
                // same orientation
                let (cu, cv) = s.closest_param(p);
                if n.derivs2(u, v).du.cross(n.derivs2(u, v).dv).len() > 1e-9 {
                    assert!(n.normal(u, v).dot(s.normal(cu, cv)) > 0.99, "{} normal", s.kind());
                }
            }
        }
    }

    #[test]
    fn analytic_surfaces_exact() {
        let f = frame_from(Vec3::new(0.5, -1., 2.), Vec3::new(0.2, 1., 0.3), Vec3::X);
        let cases = [
            (Surface::Plane { frame: f }, (-1.0, 2.0), (0.0, 3.0)),
            (Surface::Cylinder { frame: f, r: 2.0 }, (0.2, 4.0), (-1.0, 3.0)),
            (Surface::Cone { frame: f, r: 2.0, semi_angle: 0.3 }, (0.0, TAU), (0.0, 2.0)),
            (Surface::Sphere { frame: f, r: 3.0 }, (1.0, 5.5), (-1.2, 1.0)),
            (Surface::Torus { frame: f, major: 5.0, minor: 1.0 }, (0.0, TAU), (0.5, 4.0)),
            (
                Surface::Extrusion {
                    profile: Box::new(Curve::circle(Vec3::ZERO, Vec3::Z, 1.0)),
                    dir: Vec3::new(0.1, 0.2, 1.0),
                },
                (0.0, 3.0),
                (0.0, 2.0),
            ),
            (
                Surface::Revolution {
                    profile: Box::new(Curve::line_through(Vec3::new(2., 0., 0.), Vec3::new(3., 0., 2.))),
                    axis_o: Vec3::ZERO,
                    axis: Vec3::Z,
                },
                (0.0, 3.0),
                (0.0, 5f64.sqrt()),
            ),
        ];
        for (s, ur, vr) in cases.iter() {
            let n = surface_to_nurbs(s, *ur, *vr).unwrap();
            check_on(s, &n, *ur, *vr);
        }
        assert!(surface_to_nurbs(&Surface::Offset { base: Box::new(cases[0].0.clone()), dist: 1.0 }, (0., 1.), (0., 1.))
            .is_err());
    }

    #[test]
    fn bspline_patch_trim() {
        let b = BSplineSurface::new(
            2,
            2,
            vec![0., 0., 0., 0.5, 1., 1., 1.],
            vec![0., 0., 0., 1., 1., 1.],
            4,
            3,
            (0..12).map(|k| Vec3::new((k / 3) as f64, (k % 3) as f64, ((k * 7) % 5) as f64 * 0.2)).collect(),
            None,
        )
        .unwrap();
        let s = Surface::BSpline(Box::new(b.clone()));
        let t = surface_to_nurbs(&s, (0.2, 0.8), (0.1, 0.6)).unwrap();
        assert_eq!(t.domain(), ((0.2, 0.8), (0.1, 0.6)));
        assert!(vclose(t.eval(0.5, 0.3), b.eval(0.5, 0.3), 1e-12));
    }
}
