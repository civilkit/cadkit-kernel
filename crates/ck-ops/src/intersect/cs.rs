//! Curve-surface intersection (ck-ops/src/intersect/cs.rs).
//!
//! Closed forms: line vs plane / cylinder / cone / sphere (quadratic) /
//! torus (quartic, ck-core::quad::quartic); circle vs plane
//! (a cos t + b sin t + c = 0). Everything else: sampled seeds of the
//! curve-to-surface distance refined by 3-variable Newton on
//! C(t) − S(u,v) = 0. Curves lying in the surface are reported as
//! `on` parameter intervals.

use super::{fit_params_all, param_slack, solve3};
use ck_core::quad;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CsHit {
    pub t: f64,
    pub uv: (f64, f64),
    pub p: Vec3,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct CsResult {
    pub hits: Vec<CsHit>,
    /// Curve parameter intervals lying on the surface.
    pub on: Vec<(f64, f64)>,
}

/// Intersect curve `c` over `r` with the (untrimmed) surface `s`.
pub fn intersect(c: &Curve, r: (f64, f64), s: &Surface, tol: f64) -> CsResult {
    let mut out = CsResult::default();
    // curve entirely on the surface?
    if lies_on(c, r, s, tol) {
        out.on.push(r);
        return out;
    }
    let ts: Option<Vec<f64>> = match (c, s) {
        (Curve::Line { o, d }, _) => line_params(*o, *d, s, tol),
        (Curve::Circle { frame, r: rad }, Surface::Plane { frame: pf }) => {
            Some(super::cc::circle_plane_params(frame, *rad, pf.o, pf.z))
        }
        _ => None,
    };
    match ts {
        Some(ts) => {
            for t in ts {
                for tt in fit_params_all(c, t, r, param_slack(c, t, tol)) {
                    let p = c.eval(tt);
                    let (u, v, q) = s.closest_point(p);
                    if q.dist(p) <= tol.max(1e-9) * 10.0 {
                        out.hits.push(CsHit { t: tt, uv: (u, v), p });
                    }
                }
            }
        }
        None => general(c, r, s, tol, &mut out),
    }
    out.hits.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    out.hits.dedup_by(|a, b| (a.t - b.t).abs() < 1e-10);
    out
}

/// Sampled test: is the whole range on the surface?
fn lies_on(c: &Curve, r: (f64, f64), s: &Surface, tol: f64) -> bool {
    let (a, b) = if r.0.is_finite() && r.1.is_finite() { r } else { (-1.0, 1.0) };
    (0..=8).all(|i| {
        let p = c.eval(a + (b - a) * (i as f64 + 0.37) / 9.0);
        s.distance(p) <= tol
    }) && match c {
        // lines: two points suffice for plane/cyl/cone generators only if
        // also direction-consistent; the samples above cover it
        _ => true,
    }
}

/// Line `o + t d` against analytic surfaces (None = not a closed-form pair).
pub fn line_params(o: Vec3, d: Vec3, s: &Surface, tol: f64) -> Option<Vec<f64>> {
    let roots = |r: quad::Roots| r.iter().collect::<Vec<f64>>();
    match s {
        Surface::Plane { frame } => {
            let den = d.dot(frame.z);
            if den.abs() < 1e-300 {
                return Some(vec![]);
            }
            Some(vec![(frame.o - o).dot(frame.z) / den])
        }
        Surface::Cylinder { frame, r } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let a = ld.x * ld.x + ld.y * ld.y;
            let b = 2.0 * (lo.x * ld.x + lo.y * ld.y);
            let c = lo.x * lo.x + lo.y * lo.y - r * r;
            if a < 1e-24 {
                return Some(vec![]);
            }
            Some(tangent_quadratic(a, b, c, tol, r.max(1.0)))
        }
        Surface::Cone { frame, r, semi_angle } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let k = semi_angle.tan();
            let rr = r + k * lo.z;
            let a = ld.x * ld.x + ld.y * ld.y - k * k * ld.z * ld.z;
            let b = 2.0 * (lo.x * ld.x + lo.y * ld.y - k * ld.z * rr);
            let c = lo.x * lo.x + lo.y * lo.y - rr * rr;
            Some(tangent_quadratic(a, b, c, tol, r.max(1.0)))
        }
        Surface::Sphere { frame, r } => {
            let w = o - frame.o;
            Some(tangent_quadratic(d.dot(d), 2.0 * w.dot(d), w.dot(w) - r * r, tol, r.max(1.0)))
        }
        Surface::Torus { frame, major, minor } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let a = ld.dot(ld);
            let b = 2.0 * lo.dot(ld);
            let c = lo.dot(lo) + major * major - minor * minor;
            let r4 = 4.0 * major * major;
            let q4 = a * a;
            let q3 = 2.0 * a * b;
            let q2 = b * b + 2.0 * a * c - r4 * (ld.x * ld.x + ld.y * ld.y);
            let q1 = 2.0 * b * c - 2.0 * r4 * (lo.x * ld.x + lo.y * ld.y);
            let q0 = c * c - r4 * (lo.x * lo.x + lo.y * lo.y);
            Some(roots(quad::quartic(q4, q3, q2, q1, q0)))
        }
        _ => None,
    }
}

/// Quadratic roots; a slightly negative discriminant within `tol` of
/// tangency gives the double root.
fn tangent_quadratic(a: f64, b: f64, c: f64, tol: f64, scale: f64) -> Vec<f64> {
    let r = quad::quadratic(a, b, c);
    if !r.is_empty() {
        return r.iter().collect();
    }
    if a.abs() > 1e-300 {
        let t = -b / (2.0 * a);
        let val = (a * t * t + b * t + c).abs();
        // residual ~ 2 r δ for a miss distance δ
        if val <= 2.0 * scale * tol {
            return vec![t];
        }
    }
    vec![]
}

/// Sampling + Newton for arbitrary pairs.
fn general(c: &Curve, r: (f64, f64), s: &Surface, tol: f64, out: &mut CsResult) {
    let (a, b) = if r.0.is_finite() && r.1.is_finite() { r } else { return };
    let n = match c {
        Curve::Line { .. } => 32,
        Curve::Circle { .. } | Curve::Ellipse { .. } => 96,
        Curve::BSpline(bs) => (bs.ctrl.len() * 12).clamp(32, 600),
    };
    let ts: Vec<f64> = (0..=n).map(|i| a + (b - a) * i as f64 / n as f64).collect();
    // signed distance along the sample (sign from the surface normal)
    let sd: Vec<(f64, (f64, f64))> = ts
        .iter()
        .map(|&t| {
            let p = c.eval(t);
            let (u, v, q) = s.closest_point(p);
            ((p - q).dot(s.normal(u, v)), (u, v))
        })
        .collect();
    let mut seeds = vec![];
    for i in 0..n {
        let (d0, d1) = (sd[i].0, sd[i + 1].0);
        if d0 == 0.0 || d0 * d1 < 0.0 {
            seeds.push((ts[i] + (ts[i + 1] - ts[i]) * d0.abs() / (d0.abs() + d1.abs()).max(1e-300), sd[i].1));
        } else if i > 0 {
            // local minimum of |d| (tangency candidate)
            let dm = sd[i - 1].0.abs();
            if d0.abs() <= dm && d0.abs() <= d1.abs() && d0.abs() < (b - a) / n as f64 {
                seeds.push((ts[i], sd[i].1));
            }
        }
    }
    if sd[n].0 == 0.0 {
        seeds.push((ts[n], sd[n].1));
    }
    for (t0, uv0) in seeds {
        if let Some((t, u, v)) = newton3(c, s, t0, uv0) {
            if t < a - 1e-9 || t > b + 1e-9 {
                continue;
            }
            let p = c.eval(t);
            if s.eval(u, v).dist(p) <= tol {
                out.hits.push(CsHit { t: t.max(a).min(b), uv: (u, v), p });
            }
        }
    }
}

/// Newton on C(t) − S(u,v) = 0.
pub fn newton3(c: &Curve, s: &Surface, mut t: f64, (mut u, mut v): (f64, f64)) -> Option<(f64, f64, f64)> {
    for _ in 0..40 {
        let cd = c.derivs(t, 1);
        let sd = s.derivs2(u, v);
        let f = cd[0] - sd.p;
        if f.len() < 1e-14 {
            break;
        }
        // [C' , -Su, -Sv] δ = -f  → rows of the transpose system
        let cols = [cd[1], -sd.du, -sd.dv];
        let rows = [
            Vec3::new(cols[0].x, cols[1].x, cols[2].x),
            Vec3::new(cols[0].y, cols[1].y, cols[2].y),
            Vec3::new(cols[0].z, cols[1].z, cols[2].z),
        ];
        let dx = solve3(rows, [-f.x, -f.y, -f.z])?;
        t += dx.x;
        u += dx.y;
        v += dx.z;
        if !(t.is_finite() && u.is_finite() && v.is_finite()) {
            return None;
        }
        if dx.len() < 1e-15 {
            break;
        }
    }
    Some((t, u, v))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;
    use std::f64::consts::TAU;

    fn all() -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    #[test]
    fn line_vs_quadrics() {
        let f = Frame::identity();
        let l = Curve::line(Vec3::new(-10., 0.5, 0.3), Vec3::X);
        let cyl = Surface::Cylinder { frame: f, r: 2.0 };
        let r = intersect(&l, all(), &cyl, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!(cyl.distance(h.p) < 1e-12);
        }
        let sph = Surface::Sphere { frame: f, r: 3.0 };
        let r = intersect(&l, (0., 100.), &sph, 1e-9);
        assert_eq!(r.hits.len(), 2);
        let cone = Surface::Cone { frame: f, r: 1.0, semi_angle: 0.3 };
        let r = intersect(&l, all(), &cone, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!(cone.distance(h.p) < 1e-12);
        }
        let tor = Surface::Torus { frame: f, major: 5.0, minor: 1.0 };
        let l2 = Curve::line(Vec3::new(-10., 0., 0.), Vec3::X);
        let r = intersect(&l2, all(), &tor, 1e-9);
        let xs: Vec<f64> = r.hits.iter().map(|h| h.p.x).collect();
        assert_eq!(xs.len(), 4);
        for (x, e) in xs.iter().zip([-6., -4., 4., 6.]) {
            assert!((x - e).abs() < 1e-10, "{xs:?}");
        }
        // tangent line to sphere
        let t = Curve::line(Vec3::new(-10., 3., 0.), Vec3::X);
        assert_eq!(intersect(&t, all(), &sph, 1e-9).hits.len(), 1);
        // generator lying on the cylinder
        let g = Curve::line(Vec3::new(2., 0., 0.), Vec3::Z);
        let r = intersect(&g, (0., 5.), &cyl, 1e-9);
        assert!(r.hits.is_empty() && r.on == vec![(0., 5.)]);
    }

    #[test]
    fn circle_vs_plane_and_general() {
        let c = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
        let p = Surface::plane(Vec3::new(1., 0., 0.), Vec3::X);
        let r = intersect(&c, (0., TAU), &p, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!((h.p.x - 1.0).abs() < 1e-12);
        }
        // circle vs sphere (general path): circle radius 2 at z=0 vs sphere
        // r=2 centred (2,0,0) → x = 1 points
        let s = Surface::Sphere { frame: Frame { o: Vec3::new(2., 0., 0.), ..Frame::identity() }, r: 2.0 };
        let r = intersect(&c, (0., TAU), &s, 1e-9);
        assert_eq!(r.hits.len(), 2, "{r:?}");
        for h in &r.hits {
            assert!((h.p.x - 1.0).abs() < 1e-9);
        }
    }
}
