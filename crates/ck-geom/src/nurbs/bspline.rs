//! B-spline / NURBS curves, generic over 2D (`Vec2`) and 3D (`Vec3`) points.
//!
//! Wire shape (app.pretty.js `Ds.toJSON`): `{cvs, weights, degree, knots}`
//! with a full (clamped, p+1 end multiplicity) knot vector of length
//! `cvs.len() + degree + 1`. Evaluation conventions follow the frontend's
//! `Sp`/`wp`/`xp` port of Piegl & Tiller so spans agree at knot values.
//! STEP writer: B_SPLINE_CURVE_WITH_KNOTS / RATIONAL_B_SPLINE_CURVE.

use super::basis::{self, ders_basis_funs, find_span};
use super::rational::{from_hom, rat_curve_derivs, to_hom, validate_weights, Hom};
use crate::{Pt, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct BSplineCurve<P: Pt = Vec3> {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub ctrl: Vec<P>,
    /// `None` = non-rational (all weights 1).
    pub weights: Option<Vec<f64>>,
}

/// 8-point Gauss–Legendre nodes/weights on [-1,1].
pub(crate) const GL8: [(f64, f64); 8] = [
    (-0.9602898564975363, 0.1012285362903763),
    (-0.7966664774136267, 0.2223810344533745),
    (-0.5255324099163290, 0.3137066458778873),
    (-0.1834346424956498, 0.3626837833783620),
    (0.1834346424956498, 0.3626837833783620),
    (0.5255324099163290, 0.3137066458778873),
    (0.7966664774136267, 0.2223810344533745),
    (0.9602898564975363, 0.1012285362903763),
];

/// How many following samples, `step` of curve speed apart, are surely farther from the query point than the
/// best so far, given that this one is `gap` farther (sample k is at least `gap - k*step` away from beating it).
/// The 1e-9 relative margin covers the rounding of the evaluated distances.
fn skippable(gap: f64, step: f64) -> usize {
    if !(gap > 0.0) || !(step >= 0.0) || !step.is_finite() {
        return 0;
    }
    let k = (gap * (1.0 - 1e-9) / step).floor() - 1.0;
    if k >= 1.0 {
        (k as usize).min(1 << 20)
    } else {
        0
    }
}

impl<P: Pt> BSplineCurve<P> {
    /// Non-rational curve. Validates knot count / ordering.
    pub fn new(degree: usize, knots: Vec<f64>, ctrl: Vec<P>) -> Result<Self, String> {
        basis::validate(&knots, ctrl.len(), degree)?;
        Ok(BSplineCurve { degree, knots, ctrl, weights: None })
    }

    /// Rational curve (weights may all be 1; kept as given).
    pub fn new_rational(degree: usize, knots: Vec<f64>, ctrl: Vec<P>, weights: Vec<f64>) -> Result<Self, String> {
        basis::validate(&knots, ctrl.len(), degree)?;
        validate_weights(&weights, ctrl.len())?;
        Ok(BSplineCurve { degree, knots, ctrl, weights: Some(weights) })
    }

    /// Clamped uniform curve on [0,1] through-the-polygon style (not interpolating).
    pub fn clamped(degree: usize, ctrl: Vec<P>) -> Result<Self, String> {
        let p = degree.min(ctrl.len().saturating_sub(1));
        if ctrl.len() < 2 {
            return Err("need at least 2 control points".into());
        }
        let knots = basis::clamped_uniform(ctrl.len(), p);
        Self::new(p, knots, ctrl)
    }

    /// Bezier curve on [0,1] (degree = n-1).
    pub fn bezier(ctrl: Vec<P>) -> Self {
        let p = ctrl.len() - 1;
        let mut knots = vec![0.0; p + 1];
        knots.extend(std::iter::repeat(1.0).take(p + 1));
        BSplineCurve { degree: p, knots, ctrl, weights: None }
    }

    pub fn is_rational(&self) -> bool {
        super::rational::is_rational(self.weights.as_deref())
    }

    pub fn weight(&self, i: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i])
    }

    /// Parameter domain [knots[p], knots[n+1]].
    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.ctrl.len()])
    }

    fn n(&self) -> usize {
        self.ctrl.len() - 1
    }

    pub(crate) fn hom(&self) -> Vec<Hom<P>> {
        to_hom(&self.ctrl, self.weights.as_deref())
    }

    fn from_hom_parts(degree: usize, knots: Vec<f64>, h: &[Hom<P>], keep_rational: bool) -> Self {
        let (ctrl, w) = from_hom(h);
        let weights = if keep_rational { Some(w.unwrap_or_else(|| vec![1.0; ctrl.len()])) } else { w };
        BSplineCurve { degree, knots, ctrl, weights }
    }

    /// Point at `u` (clamped to the domain).
    pub fn eval(&self, u: f64) -> P {
        let p = self.degree;
        if p >= 12 {
            return self.derivs(u, 0)[0];
        }
        // allocation-free path (A2.2 + dot; rational: homogeneous sum, then the quotient)
        let (a, b) = self.domain();
        let u = u.clamp(a, b);
        let span = find_span(self.n(), p, u, &self.knots);
        let (mut n, mut left, mut right) = ([0.0f64; 12], [0.0f64; 12], [0.0f64; 12]);
        n[0] = 1.0;
        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                let den = right[r + 1] + left[j - r];
                let temp = if den != 0.0 { n[r] / den } else { 0.0 };
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }
        if let Some(w) = &self.weights {
            let (mut acc, mut wd) = (P::default(), 0.0);
            for j in 0..=p {
                let i = span - p + j;
                acc = acc + self.ctrl[i] * (n[j] * w[i]);
                wd += n[j] * w[i];
            }
            return acc / wd;
        }
        let mut acc = P::default();
        for j in 0..=p {
            acc = acc + self.ctrl[span - p + j] * n[j];
        }
        acc
    }

    /// `[C(u), C'(u), C''(u)]` without allocating (orders above `d` are zero); same arithmetic as [`Self::derivs`].
    pub fn derivs_arr(&self, u: f64, d: usize) -> [P; 3] {
        debug_assert!(d <= 2);
        let p = self.degree;
        if self.weights.is_none() && p < 12 {
            let (a, b) = self.domain();
            let u = u.clamp(a, b);
            let span = find_span(self.n(), p, u, &self.knots);
            let nd = basis::ders_basis_funs2(span, u, p, d, &self.knots);
            let mut ad = [P::default(); 3];
            // one pass over the control points for every order (bit-identical
            // to a pass per order: each accumulator sees the same operations)
            let kd = d.min(p);
            for j in 0..=p {
                let c = self.ctrl[span - p + j];
                ad[0] = ad[0] + c * (nd[0][j] * 1.0);
                if kd >= 1 {
                    ad[1] = ad[1] + c * (nd[1][j] * 1.0);
                }
                if kd >= 2 {
                    ad[2] = ad[2] + c * (nd[2][j] * 1.0);
                }
            }
            return ad;
        }
        if p < 12 {
            // rational, allocation-free (the same operations as `derivs`: A4.2 on stack arrays)
            let (a, b) = self.domain();
            let u = u.clamp(a, b);
            let span = find_span(self.n(), p, u, &self.knots);
            let nd = basis::ders_basis_funs2(span, u, p, d, &self.knots);
            let mut ad = [P::default(); 3];
            let mut wd = [0.0f64; 3];
            for k in 0..=d.min(p) {
                for j in 0..=p {
                    let i = span - p + j;
                    let w = self.weight(i);
                    ad[k] = ad[k] + self.ctrl[i] * (nd[k][j] * w);
                    wd[k] += nd[k][j] * w;
                }
            }
            let mut ck = [P::default(); 3];
            for k in 0..=d {
                let mut v = ad[k];
                for i in 1..=k {
                    v = v - ck[k - i] * (basis::binomial(k, i) * wd[i]);
                }
                ck[k] = v / wd[0];
            }
            return ck;
        }
        let v = self.derivs(u, d);
        let mut out = [P::default(); 3];
        for (o, x) in out.iter_mut().zip(v) {
            *o = x;
        }
        out
    }

    /// `[C(u), C'(u), ..., C^(d)(u)]` (A3.2 / A4.2). Orders above the degree
    /// are zero for non-rational curves.
    pub fn derivs(&self, u: f64, d: usize) -> Vec<P> {
        let (a, b) = self.domain();
        let u = u.clamp(a, b);
        let p = self.degree;
        let span = find_span(self.n(), p, u, &self.knots);
        if self.weights.is_none() && p < 12 && d <= 2 {
            let nd = basis::ders_basis_funs2(span, u, p, d, &self.knots);
            let mut ad = vec![P::default(); d + 1];
            for k in 0..=d.min(p) {
                for j in 0..=p {
                    ad[k] = ad[k] + self.ctrl[span - p + j] * (nd[k][j] * 1.0);
                }
            }
            return ad;
        }
        let nd = ders_basis_funs(span, u, p, d, &self.knots);
        let mut ad = vec![P::default(); d + 1];
        let mut wd = vec![0.0; d + 1];
        for k in 0..=d.min(p) {
            for j in 0..=p {
                let i = span - p + j;
                let w = self.weight(i);
                ad[k] = ad[k] + self.ctrl[i] * (nd[k][j] * w);
                wd[k] += nd[k][j] * w;
            }
        }
        if self.weights.is_none() {
            return ad;
        }
        rat_curve_derivs(&ad, &wd)
    }

    /// Unit tangent at `u` (falls back to a higher derivative at cusps).
    pub fn tangent(&self, u: f64) -> P {
        let d = self.derivs(u, 2);
        let t = if d[1].len() > 1e-14 { d[1] } else { d[2] };
        let l = t.len();
        if l > 0.0 {
            t / l
        } else {
            t
        }
    }

    pub fn start(&self) -> P {
        self.eval(self.domain().0)
    }
    pub fn end(&self) -> P {
        self.eval(self.domain().1)
    }

    pub fn is_closed(&self, tol: f64) -> bool {
        self.start().dist(self.end()) <= tol
    }

    /// Insert knot `u` `r` times (A5.1, homogeneous). Multiplicity is
    /// capped at the degree.
    pub fn insert_knot(&self, u: f64, r: usize) -> Self {
        let p = self.degree;
        let s = basis::multiplicity(&self.knots, u, 0.0);
        let r = r.min(p.saturating_sub(s));
        if r == 0 {
            return self.clone();
        }
        let np = self.n();
        let k = find_span_insert(np, p, u, &self.knots);
        let uk = &self.knots;
        let pw = self.hom();
        let mp = np + p + 1;
        let mut uq = Vec::with_capacity(uk.len() + r);
        uq.extend_from_slice(&uk[..=k]);
        uq.extend(std::iter::repeat(u).take(r));
        uq.extend_from_slice(&uk[k + 1..=mp]);
        let mut qw = vec![Hom::default(); np + 1 + r];
        for i in 0..=(k - p) {
            qw[i] = pw[i];
        }
        for i in (k - s)..=np {
            qw[i + r] = pw[i];
        }
        let mut rw: Vec<Hom<P>> = (0..=(p - s)).map(|i| pw[k - p + i]).collect();
        let mut l = 0;
        for j in 1..=r {
            l = k - p + j;
            for i in 0..=(p - j - s) {
                let alpha = (u - uk[l + i]) / (uk[i + k + 1] - uk[l + i]);
                rw[i] = rw[i + 1] * alpha + rw[i] * (1.0 - alpha);
            }
            qw[l] = rw[0];
            qw[k + r - j - s] = rw[p - j - s];
        }
        for i in (l + 1)..(k - s) {
            qw[i] = rw[i - l];
        }
        Self::from_hom_parts(p, uq, &qw, self.weights.is_some())
    }

    /// Insert many knots (repeated values allowed).
    pub fn refine(&self, us: &[f64]) -> Self {
        let mut c = self.clone();
        for &u in us {
            c = c.insert_knot(u, 1);
        }
        c
    }

    /// Split at `u` into [a,u] and [u,b]. Panics if `u` is outside the open domain.
    pub fn split(&self, u: f64) -> (Self, Self) {
        let (a, b) = self.domain();
        assert!(u > a && u < b, "split parameter outside the curve domain");
        let p = self.degree;
        let c = self.insert_knot(u, p);
        // after insertion u has multiplicity p (or more if it already had)
        let first = c.knots.iter().position(|&k| k == u).unwrap();
        let mult = basis::multiplicity(&c.knots, u, 0.0);
        let last = first + mult - 1;
        // left: ctrl 0..=last-p ... index of the shared control point = first - 1
        let shared = first - 1; // ctrl index where curve passes through C(u)
        let lh = c.hom();
        let mut lk: Vec<f64> = c.knots[..first].to_vec();
        lk.extend(std::iter::repeat(u).take(p + 1));
        let left_ctrl = &lh[..=shared];
        let mut rk: Vec<f64> = vec![u; p + 1];
        rk.extend_from_slice(&c.knots[last + 1..]);
        let right_ctrl = &lh[shared..];
        let keep = self.weights.is_some();
        (
            Self::from_hom_parts(p, lk, left_ctrl, keep),
            Self::from_hom_parts(p, rk, right_ctrl, keep),
        )
    }

    /// The same curve over its domain with end knots of multiplicity p+1
    /// (an unclamped / periodic-style knot vector is cut back to its domain).
    pub fn clamp_ends(&self) -> Self {
        self.clamp_start().reversed().clamp_start().reversed()
    }

    fn clamp_start(&self) -> Self {
        let p = self.degree;
        let (a, _) = self.domain();
        if p == 0 || basis::multiplicity(&self.knots, a, 0.0) >= p + 1 {
            return self.clone();
        }
        let d = self.insert_knot(a, p);
        let first = d.knots.iter().position(|&k| k == a).unwrap();
        let last = first + basis::multiplicity(&d.knots, a, 0.0) - 1;
        let h = d.hom();
        let mut k: Vec<f64> = vec![a; p + 1];
        k.extend_from_slice(&d.knots[last + 1..]);
        Self::from_hom_parts(p, k, &h[first - 1..], self.weights.is_some())
    }

    /// Sub-curve on [u0, u1] within the domain.
    pub fn segment(&self, u0: f64, u1: f64) -> Self {
        let (a, b) = self.domain();
        let mut c = self.clone();
        if u0 > a {
            c = c.split(u0).1;
        }
        if u1 < b {
            c = c.split(u1).0;
        }
        c
    }

    /// Same geometry, opposite direction; domain [a,b] maps to [a,b].
    pub fn reversed(&self) -> Self {
        let (a, b) = self.domain();
        let knots = self.knots.iter().rev().map(|&k| a + b - k).collect();
        let ctrl = self.ctrl.iter().rev().cloned().collect();
        let weights = self.weights.as_ref().map(|w| w.iter().rev().cloned().collect());
        BSplineCurve { degree: self.degree, knots, ctrl, weights }
    }

    /// Affine reparameterisation to [a,b].
    pub fn reparam(&self, a: f64, b: f64) -> Self {
        let (a0, b0) = self.domain();
        let s = (b - a) / (b0 - a0);
        let mut c = self.clone();
        for k in c.knots.iter_mut() {
            *k = a + (*k - a0) * s;
        }
        c
    }

    /// Decompose into Bezier segments (each on its own knot interval).
    pub fn bezier_segments(&self) -> Vec<Self> {
        let p = self.degree;
        let (a, b) = self.domain();
        let interior: Vec<f64> = basis::distinct_knots(&self.knots).into_iter().filter(|&k| k > a && k < b).collect();
        let mut c = self.clone();
        for &k in &interior {
            let m = basis::multiplicity(&c.knots, k, 0.0);
            if m < p {
                c = c.insert_knot(k, p - m);
            }
        }
        let h = c.hom();
        let keep = self.weights.is_some();
        let mut out = Vec::new();
        let mut breaks = vec![a];
        breaks.extend(interior);
        breaks.push(b);
        for (si, w) in breaks.windows(2).enumerate() {
            let mut kn = vec![w[0]; p + 1];
            kn.extend(std::iter::repeat(w[1]).take(p + 1));
            let seg = &h[si * p..=si * p + p];
            out.push(Self::from_hom_parts(p, kn, seg, keep));
        }
        out
    }

    /// Degree elevation by `t` (exact): Bezier-decompose, elevate each
    /// segment, re-assemble, then remove the extra interior knots.
    pub fn elevate(&self, t: usize) -> Self {
        if t == 0 {
            return self.clone();
        }
        let p = self.degree;
        let (a, b) = self.domain();
        let interior: Vec<(f64, usize)> = basis::distinct_knots(&self.knots)
            .into_iter()
            .filter(|&k| k > a && k < b)
            .map(|k| (k, basis::multiplicity(&self.knots, k, 0.0)))
            .collect();
        let segs = self.bezier_segments();
        let q = p + t;
        let mut hall: Vec<Hom<P>> = Vec::new();
        let mut knots = vec![a; q + 1];
        for (si, s) in segs.iter().enumerate() {
            let e = elevate_bezier(&s.hom(), t);
            let skip = if si == 0 { 0 } else { 1 };
            hall.extend_from_slice(&e[skip..]);
            if si + 1 < segs.len() {
                knots.extend(std::iter::repeat(interior[si].0).take(q));
            }
        }
        knots.extend(std::iter::repeat(b).take(q + 1));
        let mut c = Self::from_hom_parts(q, knots, &hall, self.weights.is_some());
        // Original continuity C^(p-m) at a knot of multiplicity m => the elevated
        // curve needs multiplicity m + t there.
        for &(k, m) in &interior {
            let target = m + t;
            let cur = basis::multiplicity(&c.knots, k, 0.0);
            for _ in target..cur {
                if !c.remove_knot(k, 1e-9) {
                    break;
                }
            }
        }
        c
    }

    /// Try to remove one occurrence of interior knot `u` (A5.8, num=1).
    /// Succeeds only if the shape changes by at most `tol` (homogeneous
    /// distance). Returns whether removal happened.
    pub fn remove_knot(&mut self, u: f64, tol: f64) -> bool {
        let p = self.degree;
        let s = basis::multiplicity(&self.knots, u, 0.0);
        if s == 0 {
            return false;
        }
        let (a, b) = self.domain();
        if u <= a || u >= b {
            return false;
        }
        let r = self.knots.iter().rposition(|&k| k == u).unwrap();
        let uk = self.knots.clone();
        let mut pw = self.hom();
        let ord = p + 1;
        let first = r - p;
        let last = r - s;
        let off = first as isize - 1;
        let mut temp = vec![Hom::<P>::default(); 2 * p + 1];
        temp[0] = pw[off as usize];
        temp[(last as isize + 1 - off) as usize] = pw[last + 1];
        let (mut i, mut j) = (first as isize, last as isize);
        let (mut ii, mut jj) = (1isize, last as isize - off);
        while j - i > 0 {
            let iu = i as usize;
            let ju = j as usize;
            let alfi = (u - uk[iu]) / (uk[iu + ord] - uk[iu]);
            let alfj = (u - uk[ju]) / (uk[ju + ord] - uk[ju]);
            temp[ii as usize] = (pw[iu] - temp[(ii - 1) as usize] * (1.0 - alfi)) * (1.0 / alfi);
            temp[jj as usize] = (pw[ju] - temp[(jj + 1) as usize] * alfj) * (1.0 / (1.0 - alfj));
            i += 1;
            ii += 1;
            j -= 1;
            jj -= 1;
        }
        let ok = if j - i < 0 {
            temp[(ii - 1) as usize].dist(temp[(jj + 1) as usize]) <= tol
        } else {
            let iu = i as usize;
            let alfi = (u - uk[iu]) / (uk[iu + ord] - uk[iu]);
            let est = temp[(ii + 1) as usize] * alfi + temp[(ii - 1) as usize] * (1.0 - alfi);
            pw[iu].dist(est) <= tol
        };
        if !ok {
            return false;
        }
        let (mut i, mut j) = (first as isize, last as isize);
        while j - i > 0 {
            pw[i as usize] = temp[(i - off) as usize];
            pw[j as usize] = temp[(j - off) as usize];
            i += 1;
            j -= 1;
        }
        let fout = (2 * r - s - p) / 2;
        pw.remove(fout);
        let mut knots = uk;
        knots.remove(r);
        *self = Self::from_hom_parts(p, knots, &pw, self.weights.is_some());
        true
    }

    /// Arc length over [u0,u1] (Gauss–Legendre per knot span, 4 sub-pieces).
    pub fn length_between(&self, u0: f64, u1: f64) -> f64 {
        let (lo, hi) = if u0 <= u1 { (u0, u1) } else { (u1, u0) };
        let mut breaks = vec![lo];
        for k in basis::distinct_knots(&self.knots) {
            if k > lo && k < hi {
                breaks.push(k);
            }
        }
        breaks.push(hi);
        let mut total = 0.0;
        for w in breaks.windows(2) {
            let sub = 4;
            for s in 0..sub {
                let a = w[0] + (w[1] - w[0]) * s as f64 / sub as f64;
                let b = w[0] + (w[1] - w[0]) * (s + 1) as f64 / sub as f64;
                let (m, h) = ((a + b) / 2.0, (b - a) / 2.0);
                for &(x, wt) in GL8.iter() {
                    total += wt * h * self.derivs_arr(m + h * x, 1)[1].len();
                }
            }
        }
        total
    }

    pub fn length(&self) -> f64 {
        let (a, b) = self.domain();
        self.length_between(a, b)
    }

    /// Upper bound of |C'(u)| on knot span `s` (`knots[s] < knots[s+1]`): the derivative is a convex combination
    /// of the hodograph control points `p (P[i+1] - P[i]) / (t[i+p+1] - t[i+1])`, i = s-p..s-1. Infinite for a
    /// rational curve (no skipping then). Padded so that rounding in the bound never makes it too small.
    fn span_speed_bound(&self, s: usize) -> f64 {
        if self.weights.is_some() || self.degree == 0 {
            return f64::INFINITY;
        }
        let p = self.degree;
        let mut m = 0.0f64;
        for i in s + 1 - p..s + 1 {
            let den = self.knots[i + p] - self.knots[i];
            if !(den > 0.0) {
                return f64::INFINITY;
            }
            m = m.max((self.ctrl[i] - self.ctrl[i - 1]).len() * p as f64 / den);
        }
        m * (1.0 + 1e-9)
    }

    /// Parameter of the closest point to `q` (sampling seed + Newton).
    pub fn closest_param(&self, q: P) -> f64 {
        self.closest_param_with(q, true)
    }

    /// [`Self::closest_param`]; `skip = false` evaluates every sample (the reference the skipping must match).
    fn closest_param_with(&self, q: P, skip: bool) -> f64 {
        let skippable = |gap: f64, step: f64| if skip { skippable(gap, step) } else { 0 };
        let (a, b) = self.domain();
        let spans = basis::distinct_count(&self.knots).max(2) - 1;
        let n = (spans * (self.degree + 1) * 4).max(16);
        let mut best = (f64::INFINITY, a);
        if spans > 6 {
            // long multi-span curves (helical rails): visit the knot spans
            // nearest to q first and stop when the control-point ball
            // (convex hull) of the remaining ones is farther than the best
            let p = self.degree;
            let m = (p + 1) * 4;
            let ball = |s: usize| -> f64 {
                let pts = &self.ctrl[s - p..=s];
                let mut c = pts[0];
                for &x in &pts[1..] {
                    c = c + x;
                }
                let c = c / pts.len() as f64;
                let r = pts.iter().map(|&x| x.dist(c)).fold(0.0f64, f64::max);
                (q.dist(c) - r).max(0.0)
            };
            let sample = |s: usize, best: &mut (f64, f64)| {
                let (k0, k1) = (self.knots[s].max(a), self.knots[s + 1].min(b));
                // a sample farther from q than `best` by more than the curve can travel to the next one cannot
                // replace it (strict <): skip it unevaluated; the sampled points that remain are the same
                let step = self.span_speed_bound(s) * (k1 - k0) / m as f64;
                let mut j = 0;
                while j <= m {
                    let u = k0 + (k1 - k0) * j as f64 / m as f64;
                    let d = self.eval(u).dist(q);
                    if d < best.0 {
                        *best = (d, u);
                    }
                    j += 1 + skippable(d - best.0, step);
                }
            };
            // two levels: blocks of BLK spans (one cheap ball per block, from
            // the control polygon) are visited nearest first; inside a block
            // the nearest span seeds `best` and every span whose own ball can
            // still beat it is sampled. Same sampled-set rule as a flat scan.
            const BLK: usize = 8;
            let ns = self.ctrl.len();
            let mut blocks: Vec<(f64, usize, usize)> = (p..ns)
                .step_by(BLK)
                .map(|s0| {
                    let s1 = (s0 + BLK).min(ns) - 1;
                    let pts = &self.ctrl[s0 - p..=s1];
                    let c = (pts[0] + pts[pts.len() - 1]) * 0.5;
                    let r2 = pts.iter().map(|&x| (x - c).dot(x - c)).fold(0.0f64, f64::max);
                    ((q.dist(c) - r2.sqrt()).max(0.0), s0, s1)
                })
                .collect();
            blocks.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
            for (lb, s0, s1) in blocks {
                if lb >= best.0 {
                    break;
                }
                let mut cells: Vec<(f64, usize)> = (s0..=s1).filter(|&s| self.knots[s] < self.knots[s + 1]).map(|s| (ball(s), s)).collect();
                if best.0.is_infinite() {
                    if let Some(&(_, sn)) = cells.iter().min_by(|x, y| x.0.total_cmp(&y.0)) {
                        sample(sn, &mut best);
                    }
                }
                cells.sort_by(|x, y| x.0.total_cmp(&y.0));
                for (l, s) in cells {
                    if l >= best.0 {
                        break;
                    }
                    sample(s, &mut best);
                }
            }
            return newton_closest(|u| self.derivs_arr(u, 2), q, best.1, a, b);
        }
        let speed = (self.degree..self.ctrl.len()).filter(|&s| self.knots[s] < self.knots[s + 1]).map(|s| self.span_speed_bound(s)).fold(0.0f64, f64::max);
        let step = speed * (b - a) / n as f64;
        let mut i = 0;
        while i <= n {
            let u = a + (b - a) * i as f64 / n as f64;
            let d = self.eval(u).dist(q);
            if d < best.0 {
                best = (d, u);
            }
            i += 1 + skippable(d - best.0, step);
        }
        newton_closest(|u| self.derivs_arr(u, 2), q, best.1, a, b)
    }

    /// Conservative proximity test: false only when `q` is farther than
    /// `tol` from every knot span's control-point ball (so from the curve);
    /// true means the curve may be within `tol`. Cost is a pass over the
    /// control points, with no curve evaluation.
    pub fn may_be_within(&self, q: P, tol: f64) -> bool {
        let p = self.degree;
        let ns = self.ctrl.len();
        if ns <= p {
            return true;
        }
        const BLK: usize = 8;
        for s0 in (p..ns).step_by(BLK) {
            let s1 = (s0 + BLK).min(ns) - 1;
            let pts = &self.ctrl[s0 - p..=s1];
            let c = (pts[0] + pts[pts.len() - 1]) * 0.5;
            let r2 = pts.iter().map(|&x| (x - c).dot(x - c)).fold(0.0f64, f64::max);
            if q.dist(c) - r2.sqrt() > tol {
                continue;
            }
            for s in s0..=s1 {
                if self.knots[s] < self.knots[s + 1] {
                    let pts = &self.ctrl[s - p..=s];
                    let mut c = pts[0];
                    for &x in &pts[1..] {
                        c = c + x;
                    }
                    let c = c / pts.len() as f64;
                    let r = pts.iter().map(|&x| x.dist(c)).fold(0.0f64, f64::max);
                    if q.dist(c) - r <= tol {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Closest point and its parameter.
    pub fn closest_point(&self, q: P) -> (f64, P) {
        let u = self.closest_param(q);
        (u, self.eval(u))
    }

    /// Apply a point map to the control points (exact for affine maps).
    pub fn map<Q: Pt>(&self, f: impl Fn(P) -> Q) -> BSplineCurve<Q> {
        BSplineCurve {
            degree: self.degree,
            knots: self.knots.clone(),
            ctrl: self.ctrl.iter().map(|&c| f(c)).collect(),
            weights: self.weights.clone(),
        }
    }

    /// Evenly-spaced parameter samples (inclusive), `n >= 1` intervals.
    pub fn sample(&self, n: usize) -> Vec<P> {
        let (a, b) = self.domain();
        (0..=n).map(|i| self.eval(a + (b - a) * i as f64 / n as f64)).collect()
    }
}

/// Newton iteration for the closest point on a parametric curve given a
/// derivative oracle `[C, C', C'']`; clamps to [a,b].
pub(crate) fn newton_closest<P: Pt, R: AsRef<[P]>>(d: impl Fn(f64) -> R, q: P, u0: f64, a: f64, b: f64) -> f64 {
    let mut u = u0;
    for _ in 0..30 {
        let ds = d(u);
        let ds = ds.as_ref();
        let r = ds[0] - q;
        let f = ds[1].dot(r);
        let fp = ds[2].dot(r) + ds[1].dot(ds[1]);
        if fp.abs() < 1e-300 {
            break;
        }
        let nu = (u - f / fp).clamp(a, b);
        if (nu - u).abs() <= 1e-14 * (1.0 + u.abs()) {
            u = nu;
            break;
        }
        u = nu;
    }
    u
}

/// Span for knot insertion: last k with knots[k] <= u (A5.1 convention:
/// at an existing knot, the span to the right).
fn find_span_insert(n: usize, p: usize, u: f64, knots: &[f64]) -> usize {
    let mut k = p;
    while k < n && knots[k + 1] <= u {
        k += 1;
    }
    k
}

/// Elevate a single Bezier segment (homogeneous control points) by `t`.
pub(crate) fn elevate_bezier<P: Pt>(c: &[Hom<P>], t: usize) -> Vec<Hom<P>> {
    let mut cur = c.to_vec();
    for _ in 0..t {
        let p = cur.len() - 1;
        let q = p + 1;
        let mut nx = Vec::with_capacity(q + 1);
        nx.push(cur[0]);
        for i in 1..q {
            let a = i as f64 / q as f64;
            nx.push(cur[i - 1] * a + cur[i] * (1.0 - a));
        }
        nx.push(cur[p]);
        cur = nx;
    }
    cur
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;
    use crate::Vec2;

    fn sample_curve() -> BSplineCurve {
        BSplineCurve::new(
            3,
            vec![0., 0., 0., 0., 0.25, 0.5, 0.75, 1., 1., 1., 1.],
            vec![
                Vec3::new(0., 0., 0.),
                Vec3::new(1., 2., 0.),
                Vec3::new(2., -1., 1.),
                Vec3::new(3., 3., 2.),
                Vec3::new(4., 0., 1.),
                Vec3::new(5., 2., 0.),
                Vec3::new(6., 0., -1.),
            ],
        )
        .unwrap()
    }

    fn rat_curve() -> BSplineCurve {
        let mut c = sample_curve();
        c.weights = Some(vec![1.0, 0.5, 2.0, 1.0, 0.7, 1.3, 1.0]);
        c
    }

    #[test]
    fn endpoints_interpolated() {
        let c = sample_curve();
        assert!(vclose(c.start(), c.ctrl[0], 1e-15));
        assert!(vclose(c.end(), c.ctrl[6], 1e-15));
    }

    #[test]
    fn derivs_fd() {
        for c in [sample_curve(), rat_curve()] {
            let h = 1e-5;
            for &u in &[0.1, 0.3, 0.55, 0.9] {
                let d = c.derivs(u, 2);
                let fd1 = (c.eval(u + h) - c.eval(u - h)) / (2.0 * h);
                let fd2 = (c.eval(u + h) - d[0] * 2.0 + c.eval(u - h)) / (h * h);
                assert!(vclose(d[1], fd1, 1e-7), "{:?} {:?}", d[1], fd1);
                assert!(vclose(d[2], fd2, 1e-3));
            }
        }
    }

    #[test]
    fn knot_insertion_preserves_shape() {
        for c in [sample_curve(), rat_curve()] {
            for (u, r) in [(0.4, 1), (0.5, 2), (0.1, 3), (0.25, 1)] {
                let k = c.insert_knot(u, r);
                for i in 0..=40 {
                    let t = i as f64 / 40.0;
                    assert!(vclose(c.eval(t), k.eval(t), 1e-12), "u={u} r={r} t={t}");
                }
            }
        }
    }

    #[test]
    fn split_and_segment() {
        for c in [sample_curve(), rat_curve()] {
            let (l, r) = c.split(0.4);
            assert_eq!(l.domain(), (0.0, 0.4));
            assert_eq!(r.domain(), (0.4, 1.0));
            for i in 0..=20 {
                let t = 0.4 * i as f64 / 20.0;
                assert!(vclose(l.eval(t), c.eval(t), 1e-12));
                let t2 = 0.4 + 0.6 * i as f64 / 20.0;
                assert!(vclose(r.eval(t2), c.eval(t2), 1e-12));
            }
            let (l, _) = c.split(0.5); // at an existing knot
            assert!(vclose(l.end(), c.eval(0.5), 1e-12));
            let s = c.segment(0.2, 0.7);
            assert!(vclose(s.eval(0.45), c.eval(0.45), 1e-12));
        }
    }

    #[test]
    fn elevate_and_reverse() {
        for c in [sample_curve(), rat_curve()] {
            let e = c.elevate(1);
            assert_eq!(e.degree, 4);
            // interior knots mult 1 -> 2; ctrl = 7 + segs(4)
            assert_eq!(e.ctrl.len(), 11, "{:?}", e.knots);
            for i in 0..=40 {
                let t = i as f64 / 40.0;
                assert!(vclose(c.eval(t), e.eval(t), 1e-10));
            }
            let r = c.reversed();
            assert!(vclose(r.eval(0.3), c.eval(0.7), 1e-12));
        }
    }

    #[test]
    fn remove_inserted_knot() {
        let c = sample_curve();
        let mut k = c.insert_knot(0.4, 1);
        assert!(k.remove_knot(0.4, 1e-9));
        assert_eq!(k.ctrl.len(), c.ctrl.len());
        for (a, b) in k.ctrl.iter().zip(c.ctrl.iter()) {
            assert!(vclose(*a, *b, 1e-12));
        }
        // a genuine knot can't be removed
        let mut c2 = c.clone();
        assert!(!c2.remove_knot(0.5, 1e-9));
    }

    #[test]
    fn length_and_closest_2d() {
        let line = BSplineCurve::bezier(vec![Vec2::new(0., 0.), Vec2::new(1., 1.), Vec2::new(2., 2.)]);
        assert!(close(line.length(), 8f64.sqrt(), 1e-12));
        let (u, p) = line.closest_point(Vec2::new(0.0, 2.0));
        assert!(close(u, 0.5, 1e-10));
        assert!(v2close(p, Vec2::new(1., 1.), 1e-10));
        let c = sample_curve();
        let q = c.eval(0.63) + Vec3::new(0.0, 0.0, 0.0);
        assert!(close(c.closest_param(q), 0.63, 1e-9));
    }

    /// Skipping samples that cannot replace the best must give the very parameter of the full scan: random curves
    /// (degree 1..4, 1..30 spans, repeated knots, rational) and query points near and far.
    #[test]
    fn closest_param_skipping_matches_full_scan() {
        let mut st = 0x9e3779b97f4a7c15u64;
        let mut rnd = move || {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            (st >> 11) as f64 / (1u64 << 53) as f64
        };
        for case in 0..400 {
            let p = 1 + (rnd() * 4.0) as usize;
            let spans = 1 + (rnd() * 30.0) as usize;
            let n = spans + p;
            let mut knots = vec![0.0; p + 1];
            let mut t = 0.0;
            for _ in 0..spans - 1 {
                t += 0.2 + rnd();
                // sometimes a doubled interior knot
                knots.push(t);
                if p > 1 && rnd() < 0.15 {
                    knots.push(t);
                }
            }
            t += 0.2 + rnd();
            knots.extend(vec![t; p + 1]);
            let nc = knots.len() - p - 1;
            let _ = n;
            let ctrl: Vec<Vec3> = (0..nc).map(|i| Vec3::new(i as f64 * 0.7 + rnd(), rnd() * 3.0, rnd() * 2.0)).collect();
            let c = if case % 5 == 0 {
                BSplineCurve::new_rational(p, knots, ctrl, (0..nc).map(|_| 0.5 + rnd()).collect()).unwrap()
            } else {
                BSplineCurve::new(p, knots, ctrl).unwrap()
            };
            let (a, b) = c.domain();
            for _ in 0..12 {
                let on = c.eval(a + (b - a) * rnd());
                let off = Vec3::new(rnd() * 25.0 - 3.0, rnd() * 8.0 - 2.0, rnd() * 6.0 - 2.0);
                for q in [on, on + Vec3::new(0.01, -0.02, 0.0) * rnd(), off] {
                    assert_eq!(c.closest_param(q).to_bits(), c.closest_param_with(q, false).to_bits(), "case {case} q {q:?}");
                }
            }
        }
    }
}
