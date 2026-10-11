//! 3D curves: line, circle, ellipse, (rational) B-spline.
//!
//! Evidence:
//! - kernel.strings curve-kind run: `line circle ellipse spline curve conic
//!   sinusoid pcurve` (the first four are the geometric kinds carried here).
//! - oracle-flange.json body.edges[] carries `"circle":{"c":[..],"n":[..],"r":25}`
//!   or `"line":true`; sampled edge pts satisfy p = c + r(cos t·x + sin t·y)
//!   with y = n × x (e.g. c=[3,0,0] n=[-1,0,0] starts at [3,0,25] and moves
//!   towards +y) — the STEP CIRCLE / AXIS2_PLACEMENT_3D parameterisation.
//! - STEP writer: LINE, CIRCLE, ELLIPSE, B_SPLINE_CURVE_WITH_KNOTS.
//!
//! Analytic curves are untrimmed carriers (lines unbounded, conics
//! 2π-periodic); edges in ck-topo hold the [t0,t1] trim.

use crate::nurbs::bspline::{newton_closest, BSplineCurve, GL8};
use crate::{any_perp, frame_from, Frame, Vec3};
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum Curve {
    /// p(t) = o + t·d, `d` unit from the constructors (t is arc length);
    /// methods tolerate |d| != 1.
    Line { o: Vec3, d: Vec3 },
    /// p(t) = c + r(cos t·x + sin t·y); frame.z = circle normal `n`.
    Circle { frame: Frame, r: f64 },
    /// p(t) = c + a cos t·x + b sin t·y (a = semi_axis_1 along x).
    Ellipse { frame: Frame, a: f64, b: f64 },
    BSpline(BSplineCurve),
}

impl Curve {
    /// Line through `o` with direction `d` (normalised).
    pub fn line(o: Vec3, d: Vec3) -> Curve {
        Curve::Line { o, d: d.normalize() }
    }

    /// Line from `a` to `b`, parameterised so t ∈ [0, |b-a|].
    pub fn line_through(a: Vec3, b: Vec3) -> Curve {
        Curve::line(a, b - a)
    }

    /// Circle from centre, normal, radius (x axis chosen by `any_perp`).
    pub fn circle(c: Vec3, n: Vec3, r: f64) -> Curve {
        let n = n.normalize();
        Curve::Circle { frame: frame_from(c, n, any_perp(n)), r }
    }

    /// Circle with explicit start direction `xref` (t = 0 at c + r·x̂).
    pub fn circle_x(c: Vec3, n: Vec3, xref: Vec3, r: f64) -> Curve {
        Curve::Circle { frame: frame_from(c, n, xref), r }
    }

    /// Circle through three points (None if collinear).
    pub fn circle_3pt(p0: Vec3, p1: Vec3, p2: Vec3) -> Option<Curve> {
        let a = p1 - p0;
        let b = p2 - p0;
        let n = a.cross(b);
        let n2 = n.len2();
        if n2 < 1e-24 * a.len2() * b.len2() {
            return None;
        }
        let c = p0 + (b.cross(n) * a.len2() + n.cross(a) * b.len2()) / (2.0 * n2);
        let r = c.dist(p0);
        Some(Curve::circle_x(c, n, p0 - c, r))
    }

    pub fn ellipse(c: Vec3, n: Vec3, xref: Vec3, a: f64, b: f64) -> Curve {
        Curve::Ellipse { frame: frame_from(c, n, xref), a, b }
    }

    /// Wire kind string (kernel.strings curve-kind table).
    pub fn kind(&self) -> &'static str {
        match self {
            Curve::Line { .. } => "line",
            Curve::Circle { .. } => "circle",
            Curve::Ellipse { .. } => "ellipse",
            Curve::BSpline(_) => "spline",
        }
    }

    /// `(c, n, r)` for the edge JSON `"circle":{"c","n","r"}`.
    pub fn circle_info(&self) -> Option<(Vec3, Vec3, f64)> {
        match self {
            Curve::Circle { frame, r } => Some((frame.o, frame.z, *r)),
            _ => None,
        }
    }

    /// Natural parameter range (lines: unbounded).
    pub fn range(&self) -> (f64, f64) {
        match self {
            Curve::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve::Circle { .. } | Curve::Ellipse { .. } => (0.0, TAU),
            Curve::BSpline(b) => b.domain(),
        }
    }

    /// Period for periodic curves.
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Circle { .. } | Curve::Ellipse { .. } => Some(TAU),
            _ => None,
        }
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        match self {
            Curve::Line { o, d } => *o + *d * t,
            Curve::Circle { frame, r } => frame.o + (frame.x * t.cos() + frame.y * t.sin()) * *r,
            Curve::Ellipse { frame, a, b } => frame.o + frame.x * (a * t.cos()) + frame.y * (b * t.sin()),
            Curve::BSpline(c) => c.eval(t),
        }
    }

    /// `[C, C', ..., C^(n)]` at t.
    pub fn derivs(&self, t: f64, n: usize) -> Vec<Vec3> {
        match self {
            Curve::Line { o, d } => {
                let mut v = vec![Vec3::ZERO; n + 1];
                v[0] = *o + *d * t;
                if n >= 1 {
                    v[1] = *d;
                }
                v
            }
            Curve::Circle { frame, r } => conic_derivs(frame, *r, *r, t, n),
            Curve::Ellipse { frame, a, b } => conic_derivs(frame, *a, *b, t, n),
            Curve::BSpline(c) => c.derivs(t, n),
        }
    }

    pub fn deriv(&self, t: f64) -> Vec3 {
        self.derivs(t, 1)[1]
    }

    pub fn tangent(&self, t: f64) -> Vec3 {
        match self {
            Curve::BSpline(c) => c.tangent(t),
            _ => self.deriv(t).normalize(),
        }
    }

    /// Parameter of the closest point to `q` (periodic results in [0, 2π)).
    pub fn closest_param(&self, q: Vec3) -> f64 {
        match self {
            Curve::Line { o, d } => (q - *o).dot(*d) / d.len2(),
            Curve::Circle { frame, .. } => {
                let l = frame.to_local(q);
                if l.x == 0.0 && l.y == 0.0 {
                    0.0
                } else {
                    wrap_angle(l.y.atan2(l.x))
                }
            }
            Curve::Ellipse { frame, a, b } => {
                let l = frame.to_local(q);
                // seed from the scaled angle, refine with Newton (4 seeds
                // to avoid the wrong local minimum near the major axis)
                let seed = (l.y * a).atan2(l.x * b);
                let mut best = (f64::INFINITY, 0.0);
                for k in 0..4 {
                    let s = seed + k as f64 * TAU / 4.0;
                    let t = newton_closest(|t| self.derivs(t, 2), q, s, f64::NEG_INFINITY, f64::INFINITY);
                    let d = self.eval(t).dist(q);
                    if d < best.0 - 1e-15 {
                        best = (d, t);
                    }
                }
                wrap_angle(best.1)
            }
            Curve::BSpline(c) => c.closest_param(q),
        }
    }

    pub fn closest_point(&self, q: Vec3) -> (f64, Vec3) {
        let t = self.closest_param(q);
        (t, self.eval(t))
    }

    /// Arc length over [t0,t1] (analytic for line/circle).
    pub fn length_between(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line { d, .. } => (t1 - t0).abs() * d.len(),
            Curve::Circle { r, .. } => (t1 - t0).abs() * r,
            Curve::Ellipse { .. } => {
                let n = 16;
                let mut s = 0.0;
                for i in 0..n {
                    let a = t0 + (t1 - t0) * i as f64 / n as f64;
                    let b = t0 + (t1 - t0) * (i + 1) as f64 / n as f64;
                    let (m, h) = ((a + b) / 2.0, (b - a) / 2.0);
                    for &(x, w) in GL8.iter() {
                        s += w * h.abs() * self.deriv(m + h * x).len();
                    }
                }
                s
            }
            Curve::BSpline(c) => c.length_between(t0, t1),
        }
    }

    /// Sub-curve on [t0,t1]: splines are cut, analytic carriers are returned
    /// unchanged (their trim lives on the edge).
    pub fn segment(&self, t0: f64, t1: f64) -> Curve {
        match self {
            Curve::BSpline(c) => Curve::BSpline(c.segment(t0, t1)),
            other => other.clone(),
        }
    }

    /// Same carrier, opposite direction. Parameter map t -> t':
    /// lines t' = -t, conics t' = 2π - t, splines t' = a + b - t.
    pub fn reversed(&self) -> Curve {
        match self {
            Curve::Line { o, d } => Curve::Line { o: *o, d: -*d },
            Curve::Circle { frame, r } => Curve::Circle { frame: flip_frame(frame), r: *r },
            Curve::Ellipse { frame, a, b } => Curve::Ellipse { frame: flip_frame(frame), a: *a, b: *b },
            Curve::BSpline(c) => Curve::BSpline(c.reversed()),
        }
    }

    /// Apply a rigid/affine point map `f` with matching vector map `g`
    /// (g = linear part). Exact for rigid motions; conics assume `g` is a
    /// similarity (radius scaled by |g(x)|).
    pub fn transform(&self, f: impl Fn(Vec3) -> Vec3, g: impl Fn(Vec3) -> Vec3) -> Curve {
        let tf = |fr: &Frame| -> (Frame, f64) {
            let x = g(fr.x);
            let s = x.len();
            let z = g(fr.z).normalize();
            (frame_from(f(fr.o), z, x), s)
        };
        match self {
            Curve::Line { o, d } => Curve::Line { o: f(*o), d: g(*d).normalize() },
            Curve::Circle { frame, r } => {
                let (fr, s) = tf(frame);
                Curve::Circle { frame: fr, r: r * s }
            }
            Curve::Ellipse { frame, a, b } => {
                let (fr, s) = tf(frame);
                Curve::Ellipse { frame: fr, a: a * s, b: b * s }
            }
            Curve::BSpline(c) => Curve::BSpline(c.map(&f)),
        }
    }

    /// `n+1` evenly spaced samples over [t0,t1].
    pub fn sample(&self, t0: f64, t1: f64, n: usize) -> Vec<Vec3> {
        (0..=n).map(|i| self.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect()
    }
}

/// Reverse a conic frame: y -> -y, z -> -z (t -> 2π - t).
fn flip_frame(f: &Frame) -> Frame {
    Frame { o: f.o, x: f.x, y: -f.y, z: -f.z }
}

/// Wrap an angle to [0, 2π).
pub fn wrap_angle(t: f64) -> f64 {
    let w = t.rem_euclid(TAU);
    if w >= TAU {
        0.0
    } else {
        w
    }
}

fn conic_derivs(frame: &Frame, a: f64, b: f64, t: f64, n: usize) -> Vec<Vec3> {
    let (s, c) = t.sin_cos();
    // k-th derivative of (cos t, sin t) cycles with period 4
    let cyc = [(c, s), (-s, c), (-c, -s), (s, -c)];
    (0..=n)
        .map(|k| {
            let (cx, sy) = cyc[k % 4];
            let v = frame.x * (a * cx) + frame.y * (b * sy);
            if k == 0 {
                frame.o + v
            } else {
                v
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;

    #[test]
    fn oracle_flange_circle_samples() {
        // oracle-flange.json edge "F4.C0@F1.1|F4.S.e1@F1.1#0":
        // circle {c:[3,0,0], n:[-1,0,0], r:25}, pts[0]=[3,0,25],
        // pts[1]=[3,3.726056654404361,24.720770655628215] (42 segments).
        let c = Curve::circle_x(Vec3::new(3., 0., 0.), Vec3::new(-1., 0., 0.), Vec3::new(0., 0., 1.), 25.0);
        assert!(vclose(c.eval(0.0), Vec3::new(3., 0., 25.), 1e-15));
        let p1 = c.eval(TAU / 42.0);
        assert!(vclose(p1, Vec3::new(3., 3.726056654404361, 24.720770655628215), 1e-14), "{p1:?}");
        let (cc, n, r) = c.circle_info().unwrap();
        assert_eq!((cc, n, r), (Vec3::new(3., 0., 0.), Vec3::new(-1., 0., 0.), 25.0));
        assert_eq!(c.kind(), "circle");
    }

    #[test]
    fn derivs_fd_all() {
        let curves = [
            Curve::line(Vec3::new(1., 2., 3.), Vec3::new(1., 1., 0.)),
            Curve::circle(Vec3::new(1., 0., 0.), Vec3::new(1., 2., 3.), 2.0),
            Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::new(1., 1., 0.), 3.0, 1.0),
        ];
        let h = 1e-5;
        for c in &curves {
            for &t in &[0.3, 1.7, 4.0] {
                let d = c.derivs(t, 3);
                let fd1 = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
                let fd2 = (c.eval(t + h) - d[0] * 2.0 + c.eval(t - h)) / (h * h);
                let fd3 = (c.derivs(t + h, 2)[2] - c.derivs(t - h, 2)[2]) / (2.0 * h);
                assert!(vclose(d[1], fd1, 1e-8));
                assert!(vclose(d[2], fd2, 1e-4));
                assert!(vclose(d[3], fd3, 1e-7));
            }
        }
    }

    #[test]
    fn closest_points() {
        let c = Curve::circle(Vec3::ZERO, Vec3::Z, 2.0);
        let q = c.eval(2.5) * 1.7 + Vec3::Z;
        assert!(close(c.closest_param(q), 2.5, 1e-12));
        let e = Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0);
        for &t in &[0.2, 1.0, 2.0, 3.5, 5.9] {
            let p = e.derivs(t, 1);
            let nrm = Vec3::Z.cross(p[1]).normalize();
            let q = p[0] - nrm * 0.2; // slightly outside
            assert!(close(e.closest_param(q), t, 1e-9), "t={t} got {}", e.closest_param(q));
        }
        let l = Curve::line_through(Vec3::ZERO, Vec3::new(2., 0., 0.));
        assert!(close(l.closest_param(Vec3::new(1.5, 3., 1.)), 1.5, 1e-15));
    }

    #[test]
    fn three_point_circle_and_length() {
        let c = Curve::circle_3pt(Vec3::new(1., 0., 0.), Vec3::new(0., 1., 0.), Vec3::new(-1., 0., 0.)).unwrap();
        let (cc, n, r) = c.circle_info().unwrap();
        assert!(vclose(cc, Vec3::ZERO, 1e-15) && close(r, 1.0, 1e-15) && vclose(n, Vec3::Z, 1e-15));
        assert!(close(c.length_between(0.0, TAU), TAU, 1e-14));
        // ellipse circumference (Ramanujan II is accurate to ~1e-10 here)
        let e = Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0);
        let (a, b) = (3.0f64, 1.0f64);
        let h = ((a - b) / (a + b)).powi(2);
        let ram = std::f64::consts::PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
        assert!(close(e.length_between(0.0, TAU), ram, 1e-7));
    }

    #[test]
    fn reverse_maps() {
        let c = Curve::circle(Vec3::new(1., 2., 3.), Vec3::new(0., 1., 1.), 2.0);
        let r = c.reversed();
        assert!(vclose(r.eval(TAU - 0.7), c.eval(0.7), 1e-14));
        let l = Curve::line(Vec3::ZERO, Vec3::X);
        assert!(vclose(l.reversed().eval(-2.0), l.eval(2.0), 1e-15));
    }
}
