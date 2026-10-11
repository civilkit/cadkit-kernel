//! Rational (NURBS) weight handling: homogeneous points and rational
//! derivative recovery (Piegl & Tiller A4.2 / A4.4).
//!
//! Evidence: kernel.strings carries the wire keys `rational` `weights`
//! `nurbs` together; the frontend `Ds` class (app.pretty.js ~L1200) stores
//! cvs pre-multiplied by weight with the weight as last coordinate
//! (`[...P*w, w]`) and recovers derivatives with the binomial rows
//! `[[1],[1,1],[1,2,1],[1,3,3,1]]` — exactly A4.2. Same scheme here.

use super::basis::binomial;
use crate::Pt;
use ck_core::linalg::Vec3;
use std::ops::{Add, Mul, Sub};

/// Homogeneous point: `p` is the weighted point (P*w), `w` the weight.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Hom<P: Pt> {
    pub p: P,
    pub w: f64,
}

impl<P: Pt> Hom<P> {
    /// From a Euclidean point and weight.
    pub fn from_point(pt: P, w: f64) -> Self {
        Hom { p: pt * w, w }
    }
    /// Back to Euclidean (projection).
    pub fn point(self) -> P {
        self.p / self.w
    }
    /// Homogeneous distance (used by knot removal checks).
    pub fn dist(self, o: Self) -> f64 {
        let d = self.p - o.p;
        (d.dot(d) + (self.w - o.w).powi(2)).sqrt()
    }
}

impl<P: Pt> Add for Hom<P> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Hom { p: self.p + o.p, w: self.w + o.w }
    }
}
impl<P: Pt> Sub for Hom<P> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Hom { p: self.p - o.p, w: self.w - o.w }
    }
}
impl<P: Pt> Mul<f64> for Hom<P> {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Hom { p: self.p * s, w: self.w * s }
    }
}

/// Lift control points + optional weights to homogeneous form.
pub fn to_hom<P: Pt>(ctrl: &[P], weights: Option<&[f64]>) -> Vec<Hom<P>> {
    ctrl.iter()
        .enumerate()
        .map(|(i, &c)| Hom::from_point(c, weights.map_or(1.0, |w| w[i])))
        .collect()
}

/// Split homogeneous points back into (points, weights). Weights are
/// returned as `None` when all are 1 (non-rational).
pub fn from_hom<P: Pt>(h: &[Hom<P>]) -> (Vec<P>, Option<Vec<f64>>) {
    let pts = h.iter().map(|x| x.point()).collect();
    let ws: Vec<f64> = h.iter().map(|x| x.w).collect();
    let rational = ws.iter().any(|&w| (w - 1.0).abs() > 1e-14);
    (pts, if rational { Some(ws) } else { None })
}

/// True if any weight differs from 1.
pub fn is_rational(weights: Option<&[f64]>) -> bool {
    weights.map_or(false, |w| w.iter().any(|&x| (x - 1.0).abs() > 1e-14))
}

/// Validate weights: same count as control points, all strictly positive.
/// GUESS: message wording is ours.
pub fn validate_weights(weights: &[f64], n_ctrl: usize) -> Result<(), String> {
    if weights.len() != n_ctrl {
        return Err(format!("weight count {} != control points {}", weights.len(), n_ctrl));
    }
    if weights.iter().any(|&w| !(w > 0.0) || !w.is_finite()) {
        return Err("weights must be positive".into());
    }
    Ok(())
}

/// A4.2: Euclidean curve derivatives C^(k) from derivatives of the weighted
/// numerator A^(k) and the weight function w^(k).
pub fn rat_curve_derivs<P: Pt>(aders: &[P], wders: &[f64]) -> Vec<P> {
    let d = aders.len();
    let mut ck: Vec<P> = Vec::with_capacity(d);
    for k in 0..d {
        let mut v = aders[k];
        for i in 1..=k {
            v = v - ck[k - i] * (binomial(k, i) * wders[i]);
        }
        ck.push(v / wders[0]);
    }
    ck
}

/// A4.4: Euclidean surface derivatives S_{k,l} from weighted numerator
/// derivatives A_{k,l} and weight derivatives w_{k,l}; `d` = max total order.
pub fn rat_surface_derivs(aders: &[Vec<Vec3>], wders: &[Vec<f64>], d: usize) -> Vec<Vec<Vec3>> {
    let mut skl = vec![vec![Vec3::ZERO; d + 1]; d + 1];
    for k in 0..=d {
        for l in 0..=(d - k) {
            let mut v = aders[k][l];
            for j in 1..=l {
                v = v - skl[k][l - j] * (binomial(l, j) * wders[0][j]);
            }
            for i in 1..=k {
                v = v - skl[k - i][l] * (binomial(k, i) * wders[i][0]);
                let mut v2 = Vec3::ZERO;
                for j in 1..=l {
                    v2 = v2 + skl[k - i][l - j] * (binomial(l, j) * wders[i][j]);
                }
                v = v - v2 * binomial(k, i);
            }
            skl[k][l] = v / wders[0][0];
        }
    }
    skl
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec2;

    #[test]
    fn hom_roundtrip() {
        let h = Hom::from_point(Vec2::new(2.0, 3.0), 0.5);
        assert_eq!(h.p, Vec2::new(1.0, 1.5));
        assert_eq!(h.point(), Vec2::new(2.0, 3.0));
        let (pts, w) = from_hom(&to_hom(&[Vec2::X, Vec2::Y], None));
        assert_eq!(pts, vec![Vec2::X, Vec2::Y]);
        assert!(w.is_none());
    }

    #[test]
    fn rat_derivs_of_quotient() {
        // C(t) = A(t)/w(t) with A = (t, t^2), w = 1 + t; check at t=0.5
        let t: f64 = 0.5;
        let a = [Vec2::new(t, t * t), Vec2::new(1.0, 2.0 * t), Vec2::new(0.0, 2.0)];
        let w = [1.0 + t, 1.0, 0.0];
        let c = rat_curve_derivs(&a, &w);
        let f = |t: f64| Vec2::new(t / (1.0 + t), t * t / (1.0 + t));
        let h = 1e-5;
        let d1 = (f(t + h) - f(t - h)) / (2.0 * h);
        let d2 = (f(t + h) - f(t) * 2.0 + f(t - h)) / (h * h);
        assert!((c[0] - f(t)).len() < 1e-15);
        assert!((c[1] - d1).len() < 1e-8);
        assert!((c[2] - d2).len() < 1e-4);
    }

    #[test]
    fn weights_validation() {
        assert!(validate_weights(&[1.0, 2.0], 2).is_ok());
        assert!(validate_weights(&[1.0, 0.0], 2).is_err());
        assert!(validate_weights(&[1.0], 2).is_err());
    }
}
