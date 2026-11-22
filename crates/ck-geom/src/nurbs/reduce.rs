//! Degree elevation, knot removal and degree reduction of B-spline surfaces,
//! knot-removal sweeps and degree reduction of curves, all with a stated
//! tolerance.
//!
//! Algorithms: Piegl & Tiller A5.9 (Bezier degree reduction after Eck, one
//! step, with the error bound), A5.8 (knot removal, already on the curve) and
//! the Bezier-decompose / reduce / re-merge scheme of the solidcore NURBS
//! tools (Apache-2.0; see THIRD_PARTY.md). The surface operations run the curve
//! operation on every column (or row) of the net, so the columns share one
//! knot vector.
//!
//! Tolerance contract: every operation that can lose shape re-measures the
//! result against the input on a dense sample (8 points per knot span per
//! direction, the parameters of the input and the result) and refuses, or
//! reverts, when the largest point distance exceeds the tolerance. The measure
//! is a sampled parametric distance, not a proof.

use super::basis::{self, binomial};
use super::bspline::BSplineCurve;
use super::bsurf::BSplineSurface;
use super::rational::Hom;
use crate::{Pt, Vec3};

fn bernstein(p: usize, i: usize, u: f64) -> f64 {
    binomial(p, i) * u.powi(i as i32) * (1.0 - u).powi((p - i) as i32)
}

/// Reduce the degree-`p` Bezier `pw` (p >= 2) by one; returns the control
/// points and Eck's bound of the (homogeneous) error.
fn reduce_bezier<P: Pt>(pw: &[Hom<P>]) -> (Vec<Hom<P>>, f64) {
    let p = pw.len() - 1;
    let pf = p as f64;
    let r = (p - 1) / 2;
    let odd = p % 2 == 1;
    let mut q = vec![Hom::<P>::default(); p];
    q[0] = pw[0];
    q[p - 1] = pw[p];
    let alf = |i: usize| pf / (pf - i as f64);
    let bet = |i: usize| pf / (i as f64 + 1.0);
    let left_end = if odd { r } else { r + 1 };
    for i in 1..left_end {
        let a = alf(i);
        q[i] = pw[i] * a + q[i - 1] * (1.0 - a);
    }
    for i in (r + 1..=p - 2).rev() {
        let b = bet(i);
        q[i] = pw[i + 1] * b + q[i + 1] * (1.0 - b);
    }
    let err = if odd {
        let (a, b) = (alf(r), bet(r));
        let pl = pw[r] * a + q[r - 1] * (1.0 - a);
        let pr = pw[r + 1] * b + q[r + 1] * (1.0 - b);
        q[r] = pl * 0.5 + pr * 0.5;
        let u = 0.5 * (1.0 - (1.0 / pf).sqrt());
        let d = (bernstein(p, r, u) - bernstein(p, r + 1, u)).abs();
        0.5 * (pf - r as f64) / pf * d * pl.dist(pr)
    } else {
        let u = (r as f64 + 1.0) / pf;
        let pl = q[r] * 0.5 + q[r + 1] * 0.5;
        bernstein(p, r + 1, u) * pw[r + 1].dist(pl)
    };
    (q, err)
}

/// Interior distinct knots with their multiplicities.
fn interior_knots(knots: &[f64], a: f64, b: f64) -> Vec<(f64, usize)> {
    basis::distinct_knots(knots)
        .into_iter()
        .filter(|&k| k > a && k < b)
        .map(|k| (k, basis::multiplicity(knots, k, 0.0)))
        .collect()
}

fn sample_params(ka: &[f64], kb: &[f64], per_span: usize) -> Vec<f64> {
    let mut ks: Vec<f64> = basis::distinct_knots(ka);
    ks.extend(basis::distinct_knots(kb));
    ks.sort_by(|a, b| a.total_cmp(b));
    ks.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    let mut out = Vec::new();
    for w in ks.windows(2) {
        for s in 0..per_span {
            out.push(w[0] + (w[1] - w[0]) * s as f64 / per_span as f64);
        }
    }
    if let Some(&l) = ks.last() {
        out.push(l);
    }
    out
}

/// Largest point distance between `a` and `b` at the same parameters, sampled
/// 8 times per knot span (union of both knot vectors). The curves must share a
/// domain.
pub fn curve_param_deviation<P: Pt>(a: &BSplineCurve<P>, b: &BSplineCurve<P>) -> f64 {
    sample_params(&a.knots, &b.knots, 8)
        .into_iter()
        .map(|u| a.eval(u).dist(b.eval(u)))
        .fold(0.0, f64::max)
}

/// Largest point distance between `a` and `b` at the same (u, v), sampled 8 x
/// 8 per knot cell. The surfaces must share a domain.
pub fn surface_param_deviation(a: &BSplineSurface, b: &BSplineSurface) -> f64 {
    let us = sample_params(&a.knots_u, &b.knots_u, 8);
    let vs = sample_params(&a.knots_v, &b.knots_v, 8);
    let (pa, pb) = (a.eval_grid(&us, &vs), b.eval_grid(&us, &vs));
    pa.iter().zip(&pb).map(|(x, y)| x.dist(*y)).fold(0.0, f64::max)
}

impl<P: Pt> BSplineCurve<P> {
    /// Remove every interior knot occurrence that can go while the curve stays
    /// within `tol` of the original (measured on a dense sample after each
    /// removal; a removal that breaks it is undone). Returns the number removed.
    pub fn remove_knots(&mut self, tol: f64) -> usize {
        let orig = self.clone();
        let (a, b) = orig.domain();
        let mut count = 0;
        for (k, _) in interior_knots(&orig.knots, a, b) {
            loop {
                let before = self.clone();
                if !self.remove_knot(k, tol) {
                    break;
                }
                if curve_param_deviation(&orig, self) > tol {
                    *self = before;
                    break;
                }
                count += 1;
            }
        }
        count
    }

    /// Degree-1-lower curve, C0 at every old knot, no knot removal; also the
    /// largest Bezier reduction error bound.
    fn reduce_c0(&self) -> Result<(Self, f64), String> {
        let p = self.degree;
        if p < 2 {
            return Err("degree reduction needs degree >= 2".into());
        }
        let q = p - 1;
        let segs = self.bezier_segments();
        let (a, b) = self.domain();
        let mut knots = vec![a; q + 1];
        let mut hall: Vec<Hom<P>> = Vec::new();
        let mut err = 0.0f64;
        for (si, s) in segs.iter().enumerate() {
            let (r, e) = reduce_bezier(&s.hom());
            err = err.max(e);
            hall.extend_from_slice(&r[if si == 0 { 0 } else { 1 }..]);
            if si + 1 < segs.len() {
                knots.extend(std::iter::repeat(s.domain().1).take(q));
            }
        }
        knots.extend(std::iter::repeat(b).take(q + 1));
        Ok((Self::from_hom_parts(q, knots, &hall, self.weights.is_some()), err))
    }

    /// Reduce the degree by one if the result stays within `tol` of this curve
    /// (sampled, see the module notes); `Err` says why when it cannot. The old
    /// continuity at the knots is kept where the tolerance allows (a knot of
    /// multiplicity m ends with m - 1), otherwise the knot stays.
    pub fn reduce_degree(&self, tol: f64) -> Result<Self, String> {
        let (mut c, err) = self.reduce_c0()?;
        if !self.is_rational() && err > tol {
            return Err(format!("degree reduction error bound {err:.3e} exceeds tolerance {tol:.3e}"));
        }
        let (a, b) = self.domain();
        for (k, m) in interior_knots(&self.knots, a, b) {
            while basis::multiplicity(&c.knots, k, 0.0) > m - 1 {
                if !c.remove_knot(k, 0.25 * tol) {
                    break;
                }
            }
        }
        let dev = curve_param_deviation(self, &c);
        if dev > tol {
            return Err(format!("degree reduction deviates {dev:.3e}, tolerance {tol:.3e}"));
        }
        Ok(c)
    }
}

impl BSplineSurface {
    /// Rebuild from columns (curves along u, one per v index) that share a
    /// knot vector and length.
    fn from_columns(&self, cols: &[BSplineCurve]) -> Self {
        let nu = cols[0].ctrl.len();
        let nv = self.nv;
        let mut ctrl = vec![Vec3::ZERO; nu * nv];
        let mut ws = vec![1.0; nu * nv];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nu {
                ctrl[i * nv + j] = c.ctrl[i];
                ws[i * nv + j] = c.weight(i);
            }
        }
        BSplineSurface {
            deg_u: cols[0].degree,
            knots_u: cols[0].knots.clone(),
            nu,
            ctrl,
            weights: self.weights.as_ref().map(|_| ws),
            ..self.clone()
        }
    }

    fn columns(&self) -> Vec<BSplineCurve> {
        (0..self.nv).map(|j| self.column(j)).collect()
    }

    /// Raise the u degree by `t` (exact).
    pub fn elevate_u(&self, t: usize) -> Self {
        if t == 0 {
            return self.clone();
        }
        let cols: Vec<BSplineCurve> = self.columns().iter().map(|c| c.elevate(t)).collect();
        self.from_columns(&cols)
    }

    /// Raise the v degree by `t` (exact).
    pub fn elevate_v(&self, t: usize) -> Self {
        self.transposed().elevate_u(t).transposed()
    }

    /// Remove one occurrence of interior knot `u` from the u direction if every
    /// column stays within `tol` (control-point distance, A5.8). `self` is
    /// unchanged on failure.
    pub fn remove_knot_u(&mut self, u: f64, tol: f64) -> bool {
        let mut cols = self.columns();
        for c in cols.iter_mut() {
            if !c.remove_knot(u, tol) {
                return false;
            }
        }
        *self = self.from_columns(&cols);
        true
    }

    /// As [`Self::remove_knot_u`] for v.
    pub fn remove_knot_v(&mut self, v: f64, tol: f64) -> bool {
        let mut t = self.transposed();
        let ok = t.remove_knot_u(v, tol);
        if ok {
            *self = t.transposed();
        }
        ok
    }

    /// Remove all u knots that can go while the surface stays within `tol` of
    /// the original (sampled after each removal, a removal that breaks it is
    /// undone). Returns the number of knot occurrences removed.
    pub fn remove_knots_u(&mut self, tol: f64) -> usize {
        let orig = self.clone();
        let ((a, b), _) = orig.domain();
        let mut count = 0;
        for (k, _) in interior_knots(&orig.knots_u, a, b) {
            loop {
                let before = self.clone();
                if !self.remove_knot_u(k, tol) {
                    break;
                }
                if surface_param_deviation(&orig, self) > tol {
                    *self = before;
                    break;
                }
                count += 1;
            }
        }
        count
    }

    /// As [`Self::remove_knots_u`] for v.
    pub fn remove_knots_v(&mut self, tol: f64) -> usize {
        let mut t = self.transposed();
        let n = t.remove_knots_u(tol);
        if n > 0 {
            *self = t.transposed();
        }
        n
    }

    /// Lower the u degree by one if the surface stays within `tol` (sampled);
    /// `Err` says why when it cannot.
    pub fn reduce_degree_u(&self, tol: f64) -> Result<Self, String> {
        if self.deg_u < 2 {
            return Err("degree reduction needs degree >= 2".into());
        }
        let mut cols = Vec::with_capacity(self.nv);
        let mut err = 0.0f64;
        for c in self.columns() {
            let (r, e) = c.reduce_c0()?;
            err = err.max(e);
            cols.push(r);
        }
        if !self.is_rational() && err > tol {
            return Err(format!("degree reduction error bound {err:.3e} exceeds tolerance {tol:.3e}"));
        }
        let mut s = self.from_columns(&cols);
        let ((a, b), _) = self.domain();
        for (k, m) in interior_knots(&self.knots_u, a, b) {
            while basis::multiplicity(&s.knots_u, k, 0.0) > m - 1 {
                if !s.remove_knot_u(k, 0.25 * tol) {
                    break;
                }
            }
        }
        let dev = surface_param_deviation(self, &s);
        if dev > tol {
            return Err(format!("degree reduction deviates {dev:.3e}, tolerance {tol:.3e}"));
        }
        Ok(s)
    }

    /// As [`Self::reduce_degree_u`] for v.
    pub fn reduce_degree_v(&self, tol: f64) -> Result<Self, String> {
        Ok(self.transposed().reduce_degree_u(tol)?.transposed())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec3;

    fn wavy(du: usize, dv: usize, ku: Vec<f64>, kv: Vec<f64>) -> BSplineSurface {
        let nu = ku.len() - du - 1;
        let nv = kv.len() - dv - 1;
        let ctrl = (0..nu * nv)
            .map(|k| {
                let (i, j) = ((k / nv) as f64, (k % nv) as f64);
                Vec3::new(i, j, (0.7 * i).sin() * 0.8 + (1.3 * j).cos() * 0.5)
            })
            .collect();
        BSplineSurface::new(du, dv, ku, kv, nu, nv, ctrl, None).unwrap()
    }

    fn rational_wavy() -> BSplineSurface {
        let mut s = wavy(2, 3, vec![0., 0., 0., 0.4, 1., 1., 1.], vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.]);
        s.weights = Some((0..s.ctrl.len()).map(|k| 1.0 + 0.1 * (k % 4) as f64).collect());
        s
    }

    #[test]
    fn elevate_surface_preserves_shape() {
        for s in [wavy(2, 2, vec![0., 0., 0., 0.3, 0.7, 1., 1., 1.], vec![0., 0., 0., 0.5, 1., 1., 1.]), rational_wavy()] {
            let eu = s.elevate_u(2);
            assert_eq!((eu.deg_u, eu.deg_v), (s.deg_u + 2, s.deg_v));
            assert!(surface_param_deviation(&s, &eu) < 1e-9);
            let ev = s.elevate_v(1);
            assert_eq!((ev.deg_u, ev.deg_v), (s.deg_u, s.deg_v + 1));
            assert!(surface_param_deviation(&s, &ev) < 1e-9);
            let both = s.elevate_u(1).elevate_v(2);
            assert!(surface_param_deviation(&s, &both) < 1e-9);
            assert_eq!(s.elevate_u(0), s);
        }
    }

    #[test]
    fn knot_removal_surface() {
        let s = wavy(3, 3, vec![0., 0., 0., 0., 1., 1., 1., 1.], vec![0., 0., 0., 0., 1., 1., 1., 1.]);
        // inserted knots are removable again, in both directions
        let mut r = s.insert_knot_u(0.3, 2).insert_knot_u(0.6, 1).insert_knot_v(0.5, 2);
        assert!(r.knots_u.len() > s.knots_u.len());
        let nu = r.remove_knots_u(1e-9);
        let nv = r.remove_knots_v(1e-9);
        assert_eq!((nu, nv), (3, 2));
        assert_eq!(r.knots_u, s.knots_u);
        assert_eq!(r.knots_v, s.knots_v);
        assert!(surface_param_deviation(&s, &r) < 1e-9);
        // a real knot is refused at a tight tolerance
        let mut w = wavy(3, 3, vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.], vec![0., 0., 0., 0., 1., 1., 1., 1.]);
        let before = w.clone();
        assert!(!w.remove_knot_u(0.5, 1e-9));
        assert_eq!(w, before);
        assert_eq!(w.remove_knots_u(1e-9), 0);
        // and removed at a loose one, within the tolerance
        let n = w.remove_knots_u(10.0);
        assert_eq!(n, 1);
        assert!(surface_param_deviation(&before, &w) <= 10.0);
    }

    #[test]
    fn knot_removal_curve_cumulative() {
        let c = BSplineCurve::<Vec3>::new(
            3,
            vec![0., 0., 0., 0., 0.25, 0.5, 0.75, 1., 1., 1., 1.],
            (0..7).map(|i| Vec3::new(i as f64, ((i * 5) % 3) as f64 * 0.01, 0.0)).collect(),
        )
        .unwrap();
        let mut d = c.clone();
        let n = d.remove_knots(0.05);
        assert!(curve_param_deviation(&c, &d) <= 0.05);
        assert!(n >= 1);
    }

    #[test]
    fn reduce_elevated_exact() {
        let s = wavy(2, 3, vec![0., 0., 0., 0.4, 1., 1., 1.], vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.]);
        let e = s.elevate_u(1).elevate_v(1);
        let ru = e.reduce_degree_u(1e-9).unwrap();
        assert_eq!(ru.deg_u, 2);
        assert_eq!(ru.knots_u, s.knots_u);
        let r = ru.reduce_degree_v(1e-9).unwrap();
        assert_eq!((r.deg_u, r.deg_v), (2, 3));
        assert_eq!(r.knots_v, s.knots_v);
        assert!(surface_param_deviation(&s, &r) < 1e-9);
        // rational
        let rs = rational_wavy();
        let back = rs.elevate_u(1).reduce_degree_u(1e-8).unwrap();
        assert!(surface_param_deviation(&rs, &back) < 1e-8);
    }

    #[test]
    fn reduce_refuses_when_it_cannot_hold() {
        let s = wavy(3, 3, vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.], vec![0., 0., 0., 0., 1., 1., 1., 1.]);
        let e = s.reduce_degree_u(1e-6);
        assert!(e.is_err(), "{:?}", e.map(|x| x.deg_u));
        assert!(s.reduce_degree_v(1e-6).is_err());
        // loose tolerance: allowed, and held
        let r = s.reduce_degree_u(5.0).unwrap();
        assert_eq!(r.deg_u, 2);
        assert!(surface_param_deviation(&s, &r) <= 5.0);
        // degree 1 refuses
        let b = BSplineSurface::bilinear(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1., 1., 0.));
        assert!(b.reduce_degree_u(1.0).is_err());
    }

    #[test]
    fn curve_reduce() {
        let c = BSplineCurve::<Vec3>::new(
            3,
            vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.],
            vec![Vec3::ZERO, Vec3::X, Vec3::new(2., 1., 0.), Vec3::new(3., 0., 1.), Vec3::new(4., 1., 1.)],
        )
        .unwrap();
        let e = c.elevate(2);
        let r = e.reduce_degree(1e-9).unwrap().reduce_degree(1e-9).unwrap();
        assert_eq!(r.degree, 3);
        assert!(curve_param_deviation(&c, &r) < 1e-9);
        assert!(c.reduce_degree(1e-6).is_err());
        let q = BSplineCurve::<Vec3>::bezier(vec![Vec3::ZERO, Vec3::X, Vec3::new(2., 0.0, 0.)]);
        assert!(q.reduce_degree(1e-9).is_ok());
        assert!(BSplineCurve::<Vec3>::bezier(vec![Vec3::ZERO, Vec3::X]).reduce_degree(1.0).is_err());
    }

    #[test]
    fn curve_reduce_rational() {
        let c = BSplineCurve::new_rational(
            2,
            vec![0., 0., 0., 1., 1., 1.],
            vec![Vec3::ZERO, Vec3::new(1., 1., 0.), Vec3::new(2., 0., 0.)],
            vec![1.0, 0.7, 1.0],
        )
        .unwrap();
        let back = c.elevate(1).reduce_degree(1e-9).unwrap();
        assert!(curve_param_deviation(&c, &back) < 1e-9);
    }
}
