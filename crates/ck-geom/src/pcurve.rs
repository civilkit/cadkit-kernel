//! Curves on surfaces: a 2D parameter-space curve (`Curve2`) composed with
//! a `Surface`, plus construction of pcurves from 3D curves (exact for the
//! common analytic pairs, fitted otherwise).
//!
//! Evidence: kernel.strings curve-kind `pcurve`, analysis keys
//! `deviationMax ofTolerance pcurves`; rs_paths.txt ck-geom/src/pcurve.rs.

use crate::curve::Curve;
use crate::curve2::Curve2;
use crate::fit::{self, Param};
use crate::surface::Surface;
use crate::{Vec2, Vec3};
use std::f64::consts::TAU;

/// Borrowed curve-on-surface view: C(t) = S(uv(t)).
#[derive(Debug, Clone, Copy)]
pub struct CurveOnSurface<'a> {
    pub uv: &'a Curve2,
    pub surf: &'a Surface,
}

impl<'a> CurveOnSurface<'a> {
    pub fn new(uv: &'a Curve2, surf: &'a Surface) -> Self {
        CurveOnSurface { uv, surf }
    }

    pub fn uv_at(&self, t: f64) -> Vec2 {
        self.uv.eval(t)
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        let p = self.uv.eval(t);
        self.surf.eval(p.x, p.y)
    }

    /// `[C, C', C'']` by the chain rule:
    /// C' = Su u' + Sv v'; C'' = Suu u'² + 2Suv u'v' + Svv v'² + Su u'' + Sv v''.
    pub fn derivs(&self, t: f64) -> [Vec3; 3] {
        let d = self.uv.derivs(t, 2);
        let s = self.surf.derivs2(d[0].x, d[0].y);
        let (u1, v1, u2, v2) = (d[1].x, d[1].y, d[2].x, d[2].y);
        [
            s.p,
            s.du * u1 + s.dv * v1,
            s.duu * (u1 * u1) + s.duv * (2.0 * u1 * v1) + s.dvv * (v1 * v1) + s.du * u2 + s.dv * v2,
        ]
    }

    pub fn tangent(&self, t: f64) -> Vec3 {
        self.derivs(t)[1].normalize()
    }

    /// Surface normal along the curve.
    pub fn normal(&self, t: f64) -> Vec3 {
        let p = self.uv.eval(t);
        self.surf.normal(p.x, p.y)
    }

    pub fn length_between(&self, t0: f64, t1: f64) -> f64 {
        let n = 32;
        let mut s = 0.0;
        for i in 0..n {
            let a = t0 + (t1 - t0) * i as f64 / n as f64;
            let b = t0 + (t1 - t0) * (i + 1) as f64 / n as f64;
            let (m, h) = ((a + b) / 2.0, (b - a) / 2.0);
            for &(x, w) in crate::nurbs::bspline::GL8.iter() {
                s += w * h.abs() * self.derivs(m + h * x)[1].len();
            }
        }
        s
    }

    pub fn sample(&self, t0: f64, t1: f64, n: usize) -> Vec<Vec3> {
        (0..=n).map(|i| self.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect()
    }

    /// Max distance between this pcurve and a 3D curve sampled at matching
    /// parameters mapped linearly [t0,t1] -> [s0,s1].
    pub fn deviation(&self, t: (f64, f64), c: &Curve, s: (f64, f64), n: usize) -> f64 {
        (0..=n)
            .map(|i| {
                let f = i as f64 / n as f64;
                self.eval(t.0 + (t.1 - t.0) * f).dist(c.eval(s.0 + (s.1 - s.0) * f))
            })
            .fold(0.0, f64::max)
    }
}

/// Project points to (u,v), unwrapping periodic parameters so the sequence
/// is continuous across seams.
/// Parameter lengths a B-spline surface closes on itself in (its first and
/// last isoline coincide: a sweep or revolve of a closed profile stored as
/// one patch). `Surface::periods` is None for these; the uv of a path
/// crossing the closing isoline must still be unwrapped.
pub fn closed_periods(surf: &Surface) -> (Option<f64>, Option<f64>) {
    let Surface::BSpline(bs) = surf else { return (None, None) };
    let ((u0, u1), (v0, v1)) = bs.domain();
    if !(u1 > u0 && v1 > v0) || !(u1 - u0).is_finite() || !(v1 - v0).is_finite() {
        return (None, None);
    }
    let scale = 1.0 + bs.eval(u0, v0).len();
    let closed_u = (0..5).all(|k| {
        let v = v0 + (v1 - v0) * (k as f64 + 0.5) / 5.0;
        bs.eval(u0, v).dist(bs.eval(u1, v)) < 1e-7 * scale
    });
    let closed_v = (0..5).all(|k| {
        let u = u0 + (u1 - u0) * (k as f64 + 0.5) / 5.0;
        bs.eval(u, v0).dist(bs.eval(u, v1)) < 1e-7 * scale
    });
    (closed_u.then_some(u1 - u0), closed_v.then_some(v1 - v0))
}

pub fn project_points(surf: &Surface, pts: &[Vec3]) -> Vec<Vec2> {
    let (pu, pv) = surf.periods();
    let (cu, cv) = closed_periods(surf);
    let mut out: Vec<Vec2> = Vec::with_capacity(pts.len());
    for &p in pts {
        let warm = match (surf, out.last()) {
            (Surface::BSpline(_) | Surface::Screw { .. }, Some(prev)) => warm_project(surf, prev.x, prev.y, p),
            _ => None,
        };
        let (mut u, mut v) = warm.unwrap_or_else(|| surf.closest_param(p));
        if let Some(prev) = out.last() {
            if let Some(per) = pu {
                u = unwrap_near(u, prev.x, per);
            }
            if let Some(per) = pv {
                v = unwrap_near(v, prev.y, per);
            }
        }
        if let Some(prev) = out.last() {
            if let Some(per) = cu {
                u = unwrap_near(u, prev.x, per);
            }
            if let Some(per) = cv {
                v = unwrap_near(v, prev.y, per);
            }
        }
        out.push(Vec2::new(u, v));
    }
    // a closed B-spline path that starts ON its closing isoline projects to
    // either side of it; after unwrapping, move the whole path into the
    // domain when it only leaves it by that ambiguity
    if let Surface::BSpline(bs) = surf {
        let ((u0, u1), (v0, v1)) = bs.domain();
        for (per, lo, hi, d) in [(cu, u0, u1, 0usize), (cv, v0, v1, 1)] {
            let Some(per) = per else { continue };
            let c = |q: &Vec2| if d == 0 { q.x } else { q.y };
            let tol = 1e-6 * per;
            let mn = out.iter().map(c).fold(f64::INFINITY, f64::min);
            let mx = out.iter().map(c).fold(f64::NEG_INFINITY, f64::max);
            let sh = if mn < lo - tol && mx + per <= hi + tol {
                per
            } else if mx > hi + tol && mn - per >= lo - tol {
                -per
            } else {
                0.0
            };
            if sh != 0.0 {
                for q in out.iter_mut() {
                    if d == 0 {
                        q.x += sh;
                    } else {
                        q.y += sh;
                    }
                }
            }
        }
    }
    // u is arbitrary at a pole: take it from the neighbouring samples, so the
    // sequence stays continuous (a fitted pcurve must not swing across it)
    if pu.is_some() {
        let pole: Vec<bool> = out.iter().map(|q| surf.is_pole(q.x, q.y)).collect();
        if let Some(first) = pole.iter().position(|&b| !b) {
            for i in (0..first).rev() {
                out[i].x = out[i + 1].x;
            }
            for i in first + 1..out.len() {
                if pole[i] {
                    out[i].x = out[i - 1].x;
                } else if pole[i - 1] {
                    // re-unwrap the sample after a pole against the carried u
                    out[i].x = unwrap_near(out[i].x, out[i - 1].x, pu.unwrap());
                }
            }
        }
    }
    out
}

/// Warm-started Gauss-Newton projection of a point that lies ON the surface
/// near the previous parameter (edge samples are consecutive). Accepted only
/// when it lands on `p` to ~1e-10 and stays in the domain; otherwise the
/// caller does the global closest-point search.
fn warm_project(surf: &Surface, mut u: f64, mut v: f64, p: Vec3) -> Option<(f64, f64)> {
    let ((u0, u1), (v0, v1)) = surf.domain();
    let scale = 1.0 + p.len();
    for _ in 0..12 {
        let d = surf.derivs2(u, v);
        let r = p - d.p;
        if r.len() < 1e-10 * scale {
            return Some((u, v));
        }
        let (a, b, c) = (d.du.dot(d.du), d.du.dot(d.dv), d.dv.dot(d.dv));
        let det = a * c - b * b;
        if !(det.abs() > 1e-24 * (a * c).abs()) {
            return None;
        }
        let (ru, rv) = (r.dot(d.du), r.dot(d.dv));
        u += (ru * c - rv * b) / det;
        v += (a * rv - b * ru) / det;
        if !(u.is_finite() && v.is_finite()) || u < u0 || u > u1 || v < v0 || v > v1 {
            return None;
        }
    }
    None
}

/// Shift `x` by whole periods to be nearest `near`.
pub fn unwrap_near(x: f64, near: f64, period: f64) -> f64 {
    x + ((near - x) / period).round() * period
}

/// Exact pcurve for the analytic pairs (parameter of the result tracks the
/// 3D curve parameter 1:1):
/// - line or circle/ellipse on a plane,
/// - axial line on cylinder/cone (u const), circle about the axis on a
///   cylinder/cone/sphere/torus (v const, u = t + phase).
/// Returns None when not an exact pair.
pub fn exact_pcurve(surf: &Surface, c: &Curve, tol: f64) -> Option<Curve2> {
    match (surf, c) {
        (Surface::Plane { frame }, Curve::Line { o, d }) => {
            if d.dot(frame.z).abs() > tol || (*o - frame.o).dot(frame.z).abs() > tol {
                return None;
            }
            let l = frame.to_local(*o);
            Some(Curve2::Line { o: Vec2::new(l.x, l.y), d: Vec2::new(d.dot(frame.x), d.dot(frame.y)) })
        }
        (Surface::Plane { frame }, Curve::Circle { frame: cf, r }) => {
            plane_conic(frame, cf, *r, *r, tol).map(|(c, x, ccw)| Curve2::Circle { c, x, r: *r, ccw })
        }
        (Surface::Plane { frame }, Curve::Ellipse { frame: cf, a, b }) => {
            plane_conic(frame, cf, *a, *b, tol).map(|(c, x, ccw)| Curve2::Ellipse { c, x, a: *a, b: *b, ccw })
        }
        (Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. }, Curve::Line { o, d }) => {
            // axial generator: direction = Sv / |Sv| at u
            let (u, v) = surf.closest_param(*o);
            if surf.eval(u, v).dist(*o) > tol {
                return None;
            }
            let sv = surf.derivs2(u, v).dv;
            let svl = sv.len();
            let along = d.dot(sv / svl);
            if (along.abs() - 1.0).abs() > tol {
                return None;
            }
            let _ = frame;
            // v(t) = v + t·along/|Sv|
            Some(Curve2::Line { o: Vec2::new(u, v), d: Vec2::new(0.0, along.signum() / svl) })
        }
        (
            Surface::Cylinder { frame, .. }
            | Surface::Cone { frame, .. }
            | Surface::Sphere { frame, .. }
            | Surface::Torus { frame, .. },
            Curve::Circle { frame: cf, r: cr },
        ) => {
            // circle coaxial with the surface frame
            if cf.z.cross(frame.z).len() > tol || (cf.o - frame.o).cross(frame.z).len() > tol {
                // great circle through the poles of a sphere: u const, v = ±t + φ0
                if let Surface::Sphere { r, .. } = surf {
                    return sphere_meridian(frame, *r, cf, *cr, tol);
                }
                return None;
            }
            let p0 = cf.o + cf.x * c.circle_info().unwrap().2;
            let (u0, v0) = surf.closest_param(p0);
            if surf.eval(u0, v0).dist(p0) > tol {
                return None;
            }
            let s = if cf.z.dot(frame.z) > 0.0 { 1.0 } else { -1.0 };
            Some(Curve2::Line { o: Vec2::new(u0, v0), d: Vec2::new(s, 0.0) })
        }
        _ => None,
    }
}

/// Great circle of a sphere whose plane contains the sphere's axis (a
/// meridian): exact line pcurve (u constant, v = φ0 ± t; v may run past the
/// pole, the surface evaluation continues onto the opposite meridian).
fn sphere_meridian(frame: &crate::Frame, r: f64, cf: &crate::Frame, cr: f64, tol: f64) -> Option<Curve2> {
    if (cf.o - frame.o).len() > tol || (cr - r).abs() > tol || cf.z.dot(frame.z).abs() > tol {
        return None;
    }
    let (xc, yc) = (cf.x, cf.z.cross(cf.x));
    let mut h = xc - frame.z * xc.dot(frame.z);
    if h.len() < 1e-6 {
        h = yc - frame.z * yc.dot(frame.z);
    }
    if h.len() < 1e-6 {
        return None;
    }
    let m = h.normalize();
    let phi0 = xc.dot(frame.z).atan2(xc.dot(m));
    let tangent = frame.z * phi0.cos() - m * phi0.sin();
    let s = if yc.dot(tangent) > 0.0 { 1.0 } else { -1.0 };
    let u = m.dot(frame.y).atan2(m.dot(frame.x));
    Some(Curve2::Line { o: Vec2::new(u, phi0), d: Vec2::new(0.0, s) })
}

fn plane_conic(pf: &crate::Frame, cf: &crate::Frame, _a: f64, _b: f64, tol: f64) -> Option<(Vec2, Vec2, bool)> {
    if cf.z.cross(pf.z).len() > tol || (cf.o - pf.o).dot(pf.z).abs() > tol {
        return None;
    }
    let c = pf.to_local(cf.o);
    let x = Vec2::new(cf.x.dot(pf.x), cf.x.dot(pf.y)).normalize();
    Some((Vec2::new(c.x, c.y), x, cf.z.dot(pf.z) > 0.0))
}

/// Pcurve of a 3D curve on a surface over [t0,t1]: exact when possible
/// (parameter preserved), otherwise a cubic interpolation of `n` projected
/// samples, reparameterised to [t0,t1].
pub fn pcurve_of(surf: &Surface, c: &Curve, t0: f64, t1: f64, tol: f64) -> Result<Curve2, String> {
    pcurve_of_n(surf, c, t0, t1, tol, None)
}

/// [`pcurve_of`] whose sampled fit is refined (sample count doubled up to
/// 1024) until the pcurve reproduces the 3D curve between the samples to
/// `dev_tol` relative to the curve's size. A sharply curved intersection
/// spline (two cylinders) is ~1e-3 off between 32 cubic-interpolated samples.
pub fn pcurve_of_dense(surf: &Surface, c: &Curve, t0: f64, t1: f64, tol: f64, dev_tol: f64) -> Result<Curve2, String> {
    let mut best = pcurve_of(surf, c, t0, t1, tol)?;
    let scale = 1.0 + c.eval(t0).len();
    let dev = |pc: &Curve2| -> f64 {
        (0..64)
            .map(|k| {
                let t = t0 + (t1 - t0) * (k as f64 + 0.5) / 64.0;
                let uv = pc.eval(t);
                surf.eval(uv.x, uv.y).dist(c.eval(t))
            })
            .fold(0.0, f64::max)
    };
    let mut cur = dev(&best);
    let mut n = 64;
    while cur > dev_tol * scale && n <= 1024 {
        if let Ok(pc) = pcurve_of_n(surf, c, t0, t1, tol, Some(n)) {
            let d = dev(&pc);
            if d < cur {
                best = pc;
                cur = d;
            }
        }
        n *= 2;
    }
    Ok(best)
}

fn pcurve_of_n(surf: &Surface, c: &Curve, t0: f64, t1: f64, tol: f64, n_min: Option<usize>) -> Result<Curve2, String> {
    if let Some(p) = exact_pcurve(surf, c, tol) {
        return Ok(p);
    }
    // sample count: 32, more for a curve winding many times round a
    // periodic surface (a helix): the unwrap needs < half a turn per step
    let mut n = 32;
    if surf.periods().0.is_some() || surf.periods().1.is_some() {
        let coarse: Vec<Vec3> = (0..=256).map(|i| c.eval(t0 + (t1 - t0) * i as f64 / 256.0)).collect();
        let len: f64 = coarse.windows(2).map(|w| w[0].dist(w[1])).sum();
        let rad = match surf {
            Surface::Cylinder { r, .. } | Surface::Cone { r, .. } | Surface::Sphere { r, .. } => *r,
            Surface::Torus { major, minor, .. } => major + minor,
            _ => 1.0,
        }
        .max(1e-6);
        let turns = len / (std::f64::consts::TAU * rad);
        n = (32 + (16.0 * turns).ceil() as usize).clamp(32, 2048);
    }
    if let Some(m) = n_min {
        n = n.max(m);
    }
    let ts_uniform: Vec<f64> = (0..=n).map(|i| t0 + (t1 - t0) * i as f64 / n as f64).collect();
    let pts: Vec<Vec3> = ts_uniform.iter().map(|&t| c.eval(t)).collect();
    let uv_uniform = project_points(surf, &pts);
    // a straight, uniformly traversed uv path (a helix on a cylinder, a
    // screw rail on a sweep face): exact line pcurve
    if n > 32 || matches!(surf, Surface::BSpline(_) | Surface::Screw { .. }) {
        let (a, mut b) = (uv_uniform[0], uv_uniform[n]);
        // a constant coordinate (a screw rail: v = const) stays exactly constant
        if (b.x - a.x).abs() < 1e-9 {
            b.x = a.x;
        }
        if (b.y - a.y).abs() < 1e-9 {
            b.y = a.y;
        }
        let d = (b - a) / (t1 - t0);
        let dev = uv_uniform.iter().enumerate().map(|(i, q)| q.dist(a + d * (t1 - t0) * (i as f64 / n as f64))).fold(0.0, f64::max);
        if dev < 1e-6 {
            return Ok(Curve2::Line { o: a - d * t0, d });
        }
    }
    // fitted samples: the uniform ones plus (for a B-spline curve) the middle of every knot span -- a
    // fitted intersection curve is dense where it turns sharply (a hairpin at the tip of a lens is ~1 mm of
    // a 100 mm curve) and uniform samples cannot follow it
    let mut ts = ts_uniform;
    let mut uv = uv_uniform;
    // (not on a periodic surface: the extra samples would be unwrapped on their own)
    if let (Curve::BSpline(bs), (None, None)) = (c, surf.periods()) {
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let ks: Vec<f64> = bs.knots.iter().copied().filter(|&k| k > lo && k < hi).collect();
        if ks.len() > 4 && ks.len() < 6000 {
            let mut all = vec![lo];
            all.extend(ks);
            all.push(hi);
            all.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
            let extra: Vec<f64> = all.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
            let extra_uv = project_points(surf, &extra.iter().map(|&t| c.eval(t)).collect::<Vec<_>>());
            let mut both: Vec<(f64, Vec2)> = ts.iter().copied().zip(uv.iter().copied()).chain(extra.into_iter().zip(extra_uv)).collect();
            both.sort_by(|a, b| if t1 >= t0 { a.0.total_cmp(&b.0) } else { b.0.total_cmp(&a.0) });
            both.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12 * (1.0 + t1.abs()));
            ts = both.iter().map(|x| x.0).collect();
            uv = both.iter().map(|x| x.1).collect();
        }
    }
    let u: Vec<f64> = ts.iter().map(|&t| (t - t0) / (t1 - t0)).collect();
    let knots = fit::averaged_knots(&u, 3);
    let b = fit::interpolate_with(&uv, 3, &u, knots).or_else(|_| fit::interpolate(&uv, 3, Param::ChordLength))?;
    Ok(Curve2::BSpline(b.reparam(t0, t1)))
}

/// Full-turn helper for callers building seam pcurves.
pub const PERIOD: f64 = TAU;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;
    use crate::{frame_from, Frame};

    #[test]
    fn chain_rule_fd() {
        let s = Surface::Torus { frame: Frame::identity(), major: 4.0, minor: 1.0 };
        let uv = Curve2::ellipse(Vec2::new(1., 1.), Vec2::new(1., 0.3), 0.8, 0.4);
        let cs = CurveOnSurface::new(&uv, &s);
        let h = 1e-5;
        for &t in &[0.2, 1.5, 4.0] {
            let d = cs.derivs(t);
            let fd1 = (cs.eval(t + h) - cs.eval(t - h)) / (2.0 * h);
            let fd2 = (cs.eval(t + h) - d[0] * 2.0 + cs.eval(t - h)) / (h * h);
            assert!(vclose(d[1], fd1, 1e-8));
            assert!(vclose(d[2], fd2, 1e-4));
        }
    }

    #[test]
    fn exact_pairs() {
        let f = frame_from(Vec3::new(1., 2., 3.), Vec3::new(0., 0., 1.), Vec3::new(1., 1., 0.));
        let pl = Surface::Plane { frame: f };
        let line = Curve::line(Vec3::new(2., 2., 3.), Vec3::new(1., -1., 0.));
        let circ = Curve::circle_x(Vec3::new(0., 1., 3.), -Vec3::Z, Vec3::Y, 2.0);
        let cyl = Surface::Cylinder { frame: f, r: 2.0 };
        let gen = Curve::line(cyl.eval(1.0, 0.0), f.z);
        let ring = Curve::circle_x(f.o + f.z * 1.5, f.z, Vec3::new(0., 1., 0.), 2.0);
        let sph = Surface::Sphere { frame: f, r: 2.0 };
        let lat = Curve::circle_x(f.o + f.z * 1.0, -f.z, Vec3::X, 3f64.sqrt());
        for (s, c) in [(&pl, &line), (&pl, &circ), (&cyl, &gen), (&cyl, &ring), (&sph, &lat)] {
            let p = exact_pcurve(s, c, 1e-9).unwrap_or_else(|| panic!("{} on {}", c.kind(), s.kind()));
            let cs = CurveOnSurface::new(&p, s);
            assert!(cs.deviation((0.3, 2.5), c, (0.3, 2.5), 20) < 1e-9, "{} on {}", c.kind(), s.kind());
        }
    }

    #[test]
    fn fitted_pcurve_across_seam() {
        let cyl = Surface::Cylinder { frame: Frame::identity(), r: 1.0 };
        // helix-ish spline crossing the u seam
        let pts: Vec<Vec3> = (0..10)
            .map(|i| {
                let a = -1.0 + i as f64 * 0.2;
                Vec3::new(a.cos(), a.sin(), i as f64 * 0.1)
            })
            .collect();
        let c = Curve::BSpline(fit::interpolate(&pts, 3, Param::ChordLength).unwrap());
        let p = pcurve_of(&cyl, &c, 0.0, 1.0, 1e-9).unwrap();
        let cs = CurveOnSurface::new(&p, &cyl);
        assert!(cs.deviation((0.0, 1.0), &c, (0.0, 1.0), 50) < 1e-4);
        // continuous in u (no 2π jump)
        let uvs = p.sample(0.0, 1.0, 50);
        assert!(uvs.windows(2).all(|w| (w[1].x - w[0].x).abs() < 0.5));
    }
}
