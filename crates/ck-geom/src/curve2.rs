//! 2D curves for sketch space and surface parameter (pcurve) space:
//! line, circle, ellipse, (rational) B-spline — the same analytic set as
//! `curve.rs` (kernel.strings curve kinds `line circle ellipse spline`).
//!
//! Conics carry a unit x direction and a sense: y = x rotated +90° when
//! `ccw`, −90° otherwise, so p(t) = c + a cos t·x + b sin t·y.

use crate::curve::wrap_angle;
use crate::nurbs::bspline::{newton_closest, BSplineCurve, GL8};
use crate::Vec2;
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum Curve2 {
    /// p(t) = o + t·d. `d` is unit from the constructors; pcurve lines may
    /// carry a non-unit speed (all methods handle |d| != 1).
    Line { o: Vec2, d: Vec2 },
    Circle { c: Vec2, x: Vec2, r: f64, ccw: bool },
    Ellipse { c: Vec2, x: Vec2, a: f64, b: f64, ccw: bool },
    BSpline(BSplineCurve<Vec2>),
}

impl Curve2 {
    pub fn line(o: Vec2, d: Vec2) -> Curve2 {
        Curve2::Line { o, d: d.normalize() }
    }
    pub fn line_through(a: Vec2, b: Vec2) -> Curve2 {
        Curve2::line(a, b - a)
    }
    pub fn circle(c: Vec2, r: f64) -> Curve2 {
        Curve2::Circle { c, x: Vec2::X, r, ccw: true }
    }
    pub fn ellipse(c: Vec2, x: Vec2, a: f64, b: f64) -> Curve2 {
        Curve2::Ellipse { c, x: x.normalize(), a, b, ccw: true }
    }

    /// Circle through three points (None if collinear).
    pub fn circle_3pt(p0: Vec2, p1: Vec2, p2: Vec2) -> Option<Curve2> {
        let d = 2.0 * (p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y));
        if d.abs() < 1e-300 {
            return None;
        }
        let (a2, b2, c2) = (p0.len2(), p1.len2(), p2.len2());
        let ux = (a2 * (p1.y - p2.y) + b2 * (p2.y - p0.y) + c2 * (p0.y - p1.y)) / d;
        let uy = (a2 * (p2.x - p1.x) + b2 * (p0.x - p2.x) + c2 * (p1.x - p0.x)) / d;
        let c = Vec2::new(ux, uy);
        Some(Curve2::circle(c, c.dist(p0)))
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Curve2::Line { .. } => "line",
            Curve2::Circle { .. } => "circle",
            Curve2::Ellipse { .. } => "ellipse",
            Curve2::BSpline(_) => "spline",
        }
    }

    pub fn range(&self) -> (f64, f64) {
        match self {
            Curve2::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve2::Circle { .. } | Curve2::Ellipse { .. } => (0.0, TAU),
            Curve2::BSpline(b) => b.domain(),
        }
    }

    pub fn period(&self) -> Option<f64> {
        match self {
            Curve2::Circle { .. } | Curve2::Ellipse { .. } => Some(TAU),
            _ => None,
        }
    }

    pub fn eval(&self, t: f64) -> Vec2 {
        self.derivs(t, 0)[0]
    }

    /// `[C, C', ..., C^(n)]`
    pub fn derivs(&self, t: f64, n: usize) -> Vec<Vec2> {
        match self {
            Curve2::Line { o, d } => {
                let mut v = vec![Vec2::ZERO; n + 1];
                v[0] = *o + *d * t;
                if n >= 1 {
                    v[1] = *d;
                }
                v
            }
            Curve2::Circle { c, x, r, ccw } => conic2(*c, *x, *r, *r, *ccw, t, n),
            Curve2::Ellipse { c, x, a, b, ccw } => conic2(*c, *x, *a, *b, *ccw, t, n),
            Curve2::BSpline(b) => b.derivs(t, n),
        }
    }

    pub fn deriv(&self, t: f64) -> Vec2 {
        self.derivs(t, 1)[1]
    }

    pub fn tangent(&self, t: f64) -> Vec2 {
        match self {
            Curve2::BSpline(b) => b.tangent(t),
            _ => self.deriv(t).normalize(),
        }
    }

    /// Left normal (tangent rotated +90°).
    pub fn normal(&self, t: f64) -> Vec2 {
        self.tangent(t).perp()
    }

    pub fn closest_param(&self, q: Vec2) -> f64 {
        match self {
            Curve2::Line { o, d } => (q - *o).dot(*d) / d.len2(),
            Curve2::Circle { c, x, ccw, .. } => {
                let l = q - *c;
                let (lx, ly) = (l.dot(*x), l.dot(yaxis(*x, *ccw)));
                if lx == 0.0 && ly == 0.0 {
                    0.0
                } else {
                    wrap_angle(ly.atan2(lx))
                }
            }
            Curve2::Ellipse { c, x, a, b, ccw } => {
                let l = q - *c;
                let (lx, ly) = (l.dot(*x), l.dot(yaxis(*x, *ccw)));
                let seed = (ly * a).atan2(lx * b);
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
            Curve2::BSpline(b) => b.closest_param(q),
        }
    }

    pub fn closest_point(&self, q: Vec2) -> (f64, Vec2) {
        let t = self.closest_param(q);
        (t, self.eval(t))
    }

    pub fn length_between(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve2::Line { d, .. } => (t1 - t0).abs() * d.len(),
            Curve2::Circle { r, .. } => (t1 - t0).abs() * r,
            Curve2::Ellipse { .. } => {
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
            Curve2::BSpline(b) => b.length_between(t0, t1),
        }
    }

    /// Opposite direction: lines t' = -t, conics t' = 2π - t, splines a+b-t.
    pub fn reversed(&self) -> Curve2 {
        match self {
            Curve2::Line { o, d } => Curve2::Line { o: *o, d: -*d },
            Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c, x: *x, r: *r, ccw: !*ccw },
            Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c, x: *x, a: *a, b: *b, ccw: !*ccw },
            Curve2::BSpline(b) => Curve2::BSpline(b.reversed()),
        }
    }

    pub fn segment(&self, t0: f64, t1: f64) -> Curve2 {
        match self {
            Curve2::BSpline(b) => Curve2::BSpline(b.segment(t0, t1)),
            o => o.clone(),
        }
    }

    pub fn sample(&self, t0: f64, t1: f64, n: usize) -> Vec<Vec2> {
        (0..=n).map(|i| self.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect()
    }
}

fn yaxis(x: Vec2, ccw: bool) -> Vec2 {
    if ccw {
        x.perp()
    } else {
        -x.perp()
    }
}

fn conic2(c: Vec2, x: Vec2, a: f64, b: f64, ccw: bool, t: f64, n: usize) -> Vec<Vec2> {
    let y = yaxis(x, ccw);
    let (s, co) = t.sin_cos();
    let cyc = [(co, s), (-s, co), (-co, -s), (s, -co)];
    (0..=n)
        .map(|k| {
            let (cx, sy) = cyc[k % 4];
            let v = x * (a * cx) + y * (b * sy);
            if k == 0 {
                c + v
            } else {
                v
            }
        })
        .collect()
}

/// Intersection parameters (t on `a`, s on `b`) of two 2D lines; None if parallel.
pub fn line_line(a: &Curve2, b: &Curve2) -> Option<(f64, f64)> {
    match (a, b) {
        (Curve2::Line { o: o1, d: d1 }, Curve2::Line { o: o2, d: d2 }) => {
            let den = d1.cross(*d2);
            if den.abs() < 1e-15 {
                return None;
            }
            let w = *o2 - *o1;
            Some((w.cross(*d2) / den, w.cross(*d1) / den))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;

    #[test]
    fn derivs_fd() {
        let cs = [
            Curve2::line(Vec2::new(1., 2.), Vec2::new(3., -1.)),
            Curve2::Circle { c: Vec2::new(1., 1.), x: Vec2::new(0.6, 0.8), r: 2.0, ccw: true },
            Curve2::ellipse(Vec2::ZERO, Vec2::new(1., 1.), 3.0, 1.0),
            Curve2::BSpline(
                BSplineCurve::clamped(3, vec![Vec2::new(0., 0.), Vec2::new(1., 2.), Vec2::new(3., 2.), Vec2::new(4., 0.), Vec2::new(5., 1.)])
                    .unwrap(),
            ),
        ];
        let h = 1e-5;
        for c in &cs {
            for &t in &[0.2, 0.5, 0.8] {
                let d = c.derivs(t, 2);
                let fd = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
                assert!(v2close(d[1], fd, 1e-8), "{}", c.kind());
            }
        }
    }

    #[test]
    fn closest_and_reverse() {
        let c = Curve2::Circle { c: Vec2::new(1., 1.), x: Vec2::new(0.6, 0.8), r: 2.0, ccw: true };
        let q = c.eval(1.3) * 1.0 + c.normal(1.3) * -0.3;
        assert!(close(c.closest_param(q), 1.3, 1e-12));
        let r = c.reversed();
        assert!(v2close(r.eval(TAU - 1.3), c.eval(1.3), 1e-14));
        let e = Curve2::ellipse(Vec2::ZERO, Vec2::X, 3.0, 1.0);
        for &t in &[0.3, 2.0, 4.0] {
            let q = e.eval(t) - e.normal(t) * 0.2;
            assert!(close(e.closest_param(q), t, 1e-9));
        }
    }

    #[test]
    fn three_points_and_intersection() {
        let c = Curve2::circle_3pt(Vec2::new(2., 0.), Vec2::new(0., 2.), Vec2::new(-2., 0.)).unwrap();
        if let Curve2::Circle { c, r, .. } = c {
            assert!(v2close(c, Vec2::ZERO, 1e-15) && close(r, 2.0, 1e-15));
        }
        let a = Curve2::line(Vec2::ZERO, Vec2::X);
        let b = Curve2::line(Vec2::new(3., -1.), Vec2::Y);
        let (t, s) = line_line(&a, &b).unwrap();
        assert!(close(t, 3.0, 1e-15) && close(s, 1.0, 1e-15));
        assert!(close(Curve2::circle(Vec2::ZERO, 2.0).length_between(0.0, TAU), 2.0 * TAU, 1e-15));
    }
}
