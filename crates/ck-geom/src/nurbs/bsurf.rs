//! Tensor-product B-spline / NURBS surfaces.
//!
//! Field naming pinned by kernel.strings key run `deg_u deg_v knots_u
//! knots_v ctrl` (surface wire record); a second run `knotsU knotsV nu nv
//! stride controlNet` names the inspect/analysis view. Control net is
//! row-major with u as the slow index: `ctrl[i * nv + j]`, i < nu, j < nv
//! (GUESS: u-major matches STEP B_SPLINE_SURFACE control_points_list
//! ordering, which the STEP writer emits as nested lists per u).
//! STEP: B_SPLINE_SURFACE_WITH_KNOTS / RATIONAL_B_SPLINE_SURFACE.

use super::basis::{self, ders_basis_funs, find_span};
use super::bspline::BSplineCurve;
use super::rational::{rat_surface_derivs, validate_weights};
use crate::surface::SurfDerivs;
use crate::Vec3;

#[derive(Debug, Clone, PartialEq)]
pub struct BSplineSurface {
    pub deg_u: usize,
    pub deg_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub nu: usize,
    pub nv: usize,
    /// `ctrl[i * nv + j]`
    pub ctrl: Vec<Vec3>,
    pub weights: Option<Vec<f64>>,
}

impl BSplineSurface {
    pub fn new(
        deg_u: usize,
        deg_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        nu: usize,
        nv: usize,
        ctrl: Vec<Vec3>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self, String> {
        if ctrl.len() != nu * nv {
            return Err(format!("control net {} != {} x {}", ctrl.len(), nu, nv));
        }
        basis::validate(&knots_u, nu, deg_u)?;
        basis::validate(&knots_v, nv, deg_v)?;
        if let Some(w) = &weights {
            validate_weights(w, ctrl.len())?;
        }
        Ok(BSplineSurface { deg_u, deg_v, knots_u, knots_v, nu, nv, ctrl, weights })
    }

    /// Bilinear patch through four corners: p00 (u0,v0), p10 (u1,v0), p01, p11.
    pub fn bilinear(p00: Vec3, p10: Vec3, p01: Vec3, p11: Vec3) -> Self {
        BSplineSurface {
            deg_u: 1,
            deg_v: 1,
            knots_u: vec![0., 0., 1., 1.],
            knots_v: vec![0., 0., 1., 1.],
            nu: 2,
            nv: 2,
            ctrl: vec![p00, p01, p10, p11],
            weights: None,
        }
    }

    pub fn is_rational(&self) -> bool {
        super::rational::is_rational(self.weights.as_deref())
    }

    #[inline]
    pub fn cp(&self, i: usize, j: usize) -> Vec3 {
        self.ctrl[i * self.nv + j]
    }

    #[inline]
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i * self.nv + j])
    }

    /// ((u0,u1),(v0,v1))
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (
            (self.knots_u[self.deg_u], self.knots_u[self.nu]),
            (self.knots_v[self.deg_v], self.knots_v[self.nv]),
        )
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        if self.weights.is_none() && self.deg_u < 12 && self.deg_v < 12 {
            let ((u0, u1), (v0, v1)) = self.domain();
            let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
            let (p, q) = (self.deg_u, self.deg_v);
            let su = find_span(self.nu - 1, p, u, &self.knots_u);
            let sv = find_span(self.nv - 1, q, v, &self.knots_v);
            let nu = basis::ders_basis_funs2(su, u, p, 0, &self.knots_u);
            let nv = basis::ders_basis_funs2(sv, v, q, 0, &self.knots_v);
            let mut tp = [Vec3::ZERO; 12];
            for s in 0..=q {
                for r in 0..=p {
                    tp[s] = tp[s] + self.cp(su - p + r, sv - q + s) * (nu[0][r] * 1.0);
                }
            }
            let mut a = Vec3::ZERO;
            for s in 0..=q {
                a = a + tp[s] * nv[0][s];
            }
            return a;
        }
        self.derivs(u, v, 0)[0][0]
    }

    /// `skl[k][l]` = ∂^{k+l} S / ∂u^k ∂v^l for k + l <= d (A3.6 / A4.4).
    pub fn derivs(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vec3>> {
        let ((u0, u1), (v0, v1)) = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let (p, q) = (self.deg_u, self.deg_v);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let nu = ders_basis_funs(su, u, p, d, &self.knots_u);
        let nv = ders_basis_funs(sv, v, q, d, &self.knots_v);
        let mut a = vec![vec![Vec3::ZERO; d + 1]; d + 1];
        let mut w = vec![vec![0.0; d + 1]; d + 1];
        for k in 0..=d.min(p) {
            // temp[s] = Σ_r N^(k)_u[r] * Pw[su-p+r][sv-q+s]
            let mut tp = vec![Vec3::ZERO; q + 1];
            let mut tw = vec![0.0; q + 1];
            for s in 0..=q {
                for r in 0..=p {
                    let (i, j) = (su - p + r, sv - q + s);
                    let wt = self.weight(i, j);
                    tp[s] = tp[s] + self.cp(i, j) * (nu[k][r] * wt);
                    tw[s] += nu[k][r] * wt;
                }
            }
            for l in 0..=(d - k).min(q) {
                for s in 0..=q {
                    a[k][l] = a[k][l] + tp[s] * nv[l][s];
                    w[k][l] += tw[s] * nv[l][s];
                }
            }
        }
        if self.weights.is_none() {
            return a;
        }
        rat_surface_derivs(&a, &w, d)
    }

    /// Point + first + second partials.
    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {
        let (p, q) = (self.deg_u, self.deg_v);
        if self.weights.is_none() && p < 12 && q < 12 {
            // allocation-free non-rational path (same arithmetic as `derivs`)
            let ((u0, u1), (v0, v1)) = self.domain();
            let u = u.clamp(u0, u1);
            let v = v.clamp(v0, v1);
            let su = find_span(self.nu - 1, p, u, &self.knots_u);
            let sv = find_span(self.nv - 1, q, v, &self.knots_v);
            let nu = basis::ders_basis_funs2(su, u, p, 2, &self.knots_u);
            let nv = basis::ders_basis_funs2(sv, v, q, 2, &self.knots_v);
            let mut a = [[Vec3::ZERO; 3]; 3];
            for k in 0..=2usize.min(p) {
                let mut tp = [Vec3::ZERO; 12];
                for s in 0..=q {
                    for r in 0..=p {
                        let (i, j) = (su - p + r, sv - q + s);
                        tp[s] = tp[s] + self.cp(i, j) * (nu[k][r] * 1.0);
                    }
                }
                for l in 0..=(2 - k).min(q) {
                    for s in 0..=q {
                        a[k][l] = a[k][l] + tp[s] * nv[l][s];
                    }
                }
            }
            return SurfDerivs { p: a[0][0], du: a[1][0], dv: a[0][1], duu: a[2][0], duv: a[1][1], dvv: a[0][2] };
        }
        let s = self.derivs(u, v, 2);
        SurfDerivs { p: s[0][0], du: s[1][0], dv: s[0][1], duu: s[2][0], duv: s[1][1], dvv: s[0][2] }
    }

    /// Evaluator for repeated first-order evaluation along one iso-v line
    /// of a non-rational surface (None for rational / very high degree).
    /// The v basis is computed once; `segment` collapses the v direction
    /// for one u knot span so each `eval1` costs only a degree-p u basis.
    pub fn vline(&self, v: f64) -> Option<VLine<'_>> {
        if self.weights.is_some() || self.deg_u >= 12 || self.deg_v >= 12 {
            return None;
        }
        let (_, (v0, v1)) = self.domain();
        let v = v.clamp(v0, v1);
        let sv = find_span(self.nv - 1, self.deg_v, v, &self.knots_v);
        let nv = basis::ders_basis_funs2(sv, v, self.deg_v, 1, &self.knots_v);
        Some(VLine { s: self, sv, nv })
    }

    /// Unit normal Su × Sv (zero vector at degenerate points).
    pub fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.derivs2(u, v).normal()
    }

    fn column(&self, j: usize) -> BSplineCurve {
        BSplineCurve {
            degree: self.deg_u,
            knots: self.knots_u.clone(),
            ctrl: (0..self.nu).map(|i| self.cp(i, j)).collect(),
            weights: self.weights.as_ref().map(|_| (0..self.nu).map(|i| self.weight(i, j)).collect()),
        }
    }

    fn row(&self, i: usize) -> BSplineCurve {
        BSplineCurve {
            degree: self.deg_v,
            knots: self.knots_v.clone(),
            ctrl: (0..self.nv).map(|j| self.cp(i, j)).collect(),
            weights: self.weights.as_ref().map(|_| (0..self.nv).map(|j| self.weight(i, j)).collect()),
        }
    }

    /// Iso-parametric curve at fixed `u` (a curve in v). Exact.
    pub fn iso_u(&self, u: f64) -> BSplineCurve {
        // Evaluate each v-column... i.e. collapse u: for each j, homogeneous
        // point of the u-curve through column j at u.
        let mut ctrl = Vec::with_capacity(self.nv);
        let mut ws = Vec::with_capacity(self.nv);
        for j in 0..self.nv {
            let c = self.column(j);
            let h = hom_eval(&c, u);
            ctrl.push(h.0);
            ws.push(h.1);
        }
        let weights = if self.weights.is_some() { Some(ws) } else { None };
        BSplineCurve { degree: self.deg_v, knots: self.knots_v.clone(), ctrl, weights }
    }

    /// Iso-parametric curve at fixed `v` (a curve in u). Exact.
    pub fn iso_v(&self, v: f64) -> BSplineCurve {
        let mut ctrl = Vec::with_capacity(self.nu);
        let mut ws = Vec::with_capacity(self.nu);
        for i in 0..self.nu {
            let h = hom_eval(&self.row(i), v);
            ctrl.push(h.0);
            ws.push(h.1);
        }
        let weights = if self.weights.is_some() { Some(ws) } else { None };
        BSplineCurve { degree: self.deg_u, knots: self.knots_u.clone(), ctrl, weights }
    }

    /// Insert knot `u` `r` times in the u direction.
    pub fn insert_knot_u(&self, u: f64, r: usize) -> Self {
        let cols: Vec<BSplineCurve> = (0..self.nv).map(|j| self.column(j).insert_knot(u, r)).collect();
        let nu = cols[0].ctrl.len();
        let mut ctrl = vec![Vec3::ZERO; nu * self.nv];
        let mut ws = vec![1.0; nu * self.nv];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nu {
                ctrl[i * self.nv + j] = c.ctrl[i];
                ws[i * self.nv + j] = c.weight(i);
            }
        }
        BSplineSurface {
            knots_u: cols[0].knots.clone(),
            nu,
            ctrl,
            weights: self.weights.as_ref().map(|_| ws),
            ..self.clone()
        }
    }

    /// Insert knot `v` `r` times in the v direction.
    pub fn insert_knot_v(&self, v: f64, r: usize) -> Self {
        self.transposed().insert_knot_u(v, r).transposed()
    }

    /// Swap u and v.
    pub fn transposed(&self) -> Self {
        let mut ctrl = Vec::with_capacity(self.ctrl.len());
        let mut ws = Vec::with_capacity(self.ctrl.len());
        for j in 0..self.nv {
            for i in 0..self.nu {
                ctrl.push(self.cp(i, j));
                ws.push(self.weight(i, j));
            }
        }
        BSplineSurface {
            deg_u: self.deg_v,
            deg_v: self.deg_u,
            knots_u: self.knots_v.clone(),
            knots_v: self.knots_u.clone(),
            nu: self.nv,
            nv: self.nu,
            ctrl,
            weights: self.weights.as_ref().map(|_| ws),
        }
    }

    /// Parameters of the closest point to `q` (grid seed + 2D Newton).
    pub fn closest_param(&self, q: Vec3) -> (f64, f64) {
        let ((u0, u1), (v0, v1)) = self.domain();
        let nu = ((basis::distinct_count(&self.knots_u) - 1) * (self.deg_u + 1) * 3).max(12);
        let nv = ((basis::distinct_count(&self.knots_v) - 1) * (self.deg_v + 1) * 3).max(12);
        let mut best = (f64::INFINITY, u0, v0);
        // The seed grid is the same as a brute-force scan, but samples in
        // cells whose control-net box is farther from `q` than the best
        // sample so far are skipped (convex hull property: exact pruning).
        // Long multi-span surfaces (screw sweeps) made the plain scan
        // O(spans) per call.
        let (pu, pv) = (self.deg_u, self.deg_v);
        let spans = |kn: &[f64], p: usize, n: usize| -> Vec<usize> {
            let mut idx = vec![usize::MAX; kn.len()];
            let mut k = 0;
            for s in p..n {
                if kn[s] < kn[s + 1] {
                    idx[s] = k;
                    k += 1;
                }
            }
            idx
        };
        let (iu_of, iv_of) = (spans(&self.knots_u, pu, self.nu), spans(&self.knots_v, pv, self.nv));
        let (cu, cv) = (iu_of.iter().filter(|&&i| i != usize::MAX).count(), iv_of.iter().filter(|&&i| i != usize::MAX).count());
        let mut lb = vec![f64::INFINITY; cu * cv.max(1)];
        for su in pu..self.nu {
            let iu = iu_of[su];
            if iu == usize::MAX {
                continue;
            }
            for sv in pv..self.nv {
                let iv = iv_of[sv];
                if iv == usize::MAX {
                    continue;
                }
                let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
                for i in su - pu..=su {
                    for j in sv - pv..=sv {
                        let c = self.cp(i, j);
                        lo = lo.min(c);
                        hi = hi.max(c);
                    }
                }
                let d = Vec3::new((lo.x - q.x).max(q.x - hi.x).max(0.0), (lo.y - q.y).max(q.y - hi.y).max(0.0), (lo.z - q.z).max(q.z - hi.z).max(0.0));
                lb[iu * cv + iv] = d.len();
            }
        }
        // Seed grid, visited cell by cell (nearest control-net box first):
        // the grid is uniform in (u, v), so the rows / columns that fall in a
        // knot cell are found arithmetically (then confirmed with find_span)
        // instead of scanning and sorting every row per call.
        let vs: Vec<(f64, usize)> = (0..=nv)
            .map(|b| {
                let v = v0 + (v1 - v0) * b as f64 / nv as f64;
                (v, iv_of[find_span(self.nv - 1, pv, v, &self.knots_v)])
            })
            .collect();
        // grid evaluation without the derivative machinery (non-rational)
        let fast = self.weights.is_none();
        let bv: Vec<(usize, Vec<f64>)> = if fast {
            vs.iter()
                .map(|&(v, _)| {
                    let s = find_span(self.nv - 1, pv, v, &self.knots_v);
                    (s, basis::basis_funs(s, v, pv, &self.knots_v))
                })
                .collect()
        } else {
            vec![]
        };
        let mut cols: Vec<Vec<usize>> = vec![vec![]; cv.max(1)];
        for (k, &(_, iv)) in vs.iter().enumerate() {
            cols[iv].push(k);
        }
        let mut cells: Vec<(f64, usize, usize, usize)> = Vec::with_capacity(cu * cv);
        for su in pu..self.nu {
            let iu = iu_of[su];
            if iu == usize::MAX {
                continue;
            }
            for sv in pv..self.nv {
                let iv = iv_of[sv];
                if iv != usize::MAX {
                    cells.push((lb[iu * cv + iv], su, iu, iv));
                }
            }
        }
        let u_at = |a: usize| u0 + (u1 - u0) * a as f64 / nu as f64;
        let mut visit = |c: &(f64, usize, usize, usize), best: &mut (f64, f64, f64)| {
            let (clb, su, _iu, iv) = *c;
            if clb >= best.0 || cols[iv].is_empty() {
                return;
            }
            let (k0, k1) = (self.knots_u[su], self.knots_u[su + 1]);
            let span_u = (u1 - u0).max(f64::MIN_POSITIVE);
            let alo = ((((k0 - u0) / span_u) * nu as f64).floor() - 1.0).max(0.0) as usize;
            let ahi = ((((k1 - u0) / span_u) * nu as f64).ceil() + 1.0).min(nu as f64).max(0.0) as usize;
            for a in alo..=ahi {
                let u = u_at(a);
                if find_span(self.nu - 1, pu, u, &self.knots_u) != su {
                    continue;
                }
                let bu = if fast { basis::basis_funs(su, u, pu, &self.knots_u) } else { vec![] };
                for &k in &cols[iv] {
                    let v = vs[k].0;
                    let d = if fast {
                        let (sv, ref bvk) = bv[k];
                        let mut p = Vec3::ZERO;
                        for (i, &wu) in bu.iter().enumerate() {
                            for (j, &wv) in bvk.iter().enumerate() {
                                p = p + self.cp(su - pu + i, sv - pv + j) * (wu * wv);
                            }
                        }
                        p.dist(q)
                    } else {
                        self.eval(u, v).dist(q)
                    };
                    if d < best.0 {
                        *best = (d, u, v);
                    }
                }
            }
        };
        // seed from the nearest few cells (a cell can hold no grid point),
        // then every other cell whose box can still beat the best sample
        let mut order: Vec<usize> = (0..cells.len()).collect();
        let cmp = |x: &usize, y: &usize| cells[*x].0.total_cmp(&cells[*y].0).then(x.cmp(y));
        let k = 8.min(order.len());
        if k > 0 && k < order.len() {
            order.select_nth_unstable_by(k - 1, cmp);
        }
        order[..k].sort_by(cmp);
        for &i in &order {
            visit(&cells[i], &mut best);
        }
        crate::surface::newton_surface(|u, v| self.derivs2(u, v), q, (best.1, best.2), ((u0, u1), (v0, v1)))
    }

    /// Apply a point map to the control net (exact for affine maps).
    pub fn map(&self, f: impl Fn(Vec3) -> Vec3) -> Self {
        BSplineSurface { ctrl: self.ctrl.iter().map(|&c| f(c)).collect(), ..self.clone() }
    }
}

/// Homogeneous evaluation: returns (Euclidean point, weight) — used to
/// build exact iso curves of rational surfaces.
/// See [`BSplineSurface::vline`].
pub struct VLine<'a> {
    s: &'a BSplineSurface,
    sv: usize,
    nv: [[f64; 12]; 3],
}

/// A VLine restricted to the u knot span containing a given parameter.
pub struct VSegment<'a> {
    s: &'a BSplineSurface,
    su: usize,
    c: [Vec3; 12],
    d: [Vec3; 12],
}

impl<'a> VLine<'a> {
    /// Segment for the span containing `u`; None when `u` is outside the
    /// domain.
    pub fn segment(&self, u: f64) -> Option<VSegment<'a>> {
        let s = self.s;
        let ((u0, u1), _) = s.domain();
        if !(u >= u0 && u <= u1) {
            return None;
        }
        let (p, q) = (s.deg_u, s.deg_v);
        let su = find_span(s.nu - 1, p, u, &s.knots_u);
        let mut c = [Vec3::ZERO; 12];
        let mut d = [Vec3::ZERO; 12];
        for r in 0..=p {
            let i = su - p + r;
            let (mut a, mut b) = (Vec3::ZERO, Vec3::ZERO);
            for t in 0..=q {
                let cp = s.cp(i, self.sv - q + t);
                a = a + cp * self.nv[0][t];
                b = b + cp * self.nv[1][t];
            }
            c[r] = a;
            d[r] = b;
        }
        Some(VSegment { s, su, c, d })
    }
}

impl VSegment<'_> {
    /// (point, S_u, S_v) at `u` (within the segment's span).
    #[inline]
    pub fn eval1(&self, u: f64) -> (Vec3, Vec3, Vec3) {
        let s = self.s;
        let p = s.deg_u;
        let b = basis::ders_basis_funs2(self.su, u, p, 1, &s.knots_u);
        let (mut pt, mut du, mut dv) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for r in 0..=p {
            pt = pt + self.c[r] * b[0][r];
            du = du + self.c[r] * b[1][r];
            dv = dv + self.d[r] * b[0][r];
        }
        (pt, du, dv)
    }

    /// True when `[a, b]` lies inside this segment's knot span.
    pub fn covers(&self, a: f64, b: f64) -> bool {
        let k = &self.s.knots_u;
        let (lo, hi) = (k[self.su], k[self.su + 1]);
        a.min(b) >= lo - 1e-12 && a.max(b) <= hi + 1e-12
    }
}

fn hom_eval(c: &BSplineCurve, u: f64) -> (Vec3, f64) {
    let p = c.degree;
    let (a, b) = c.domain();
    let u = u.clamp(a, b);
    let span = find_span(c.ctrl.len() - 1, p, u, &c.knots);
    let n = basis::basis_funs(span, u, p, &c.knots);
    let mut pw = Vec3::ZERO;
    let mut w = 0.0;
    for j in 0..=p {
        let i = span - p + j;
        let wi = c.weight(i);
        pw = pw + c.ctrl[i] * (n[j] * wi);
        w += n[j] * wi;
    }
    (pw / w, w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;

    fn surf(rational: bool) -> BSplineSurface {
        let (nu, nv) = (4, 5);
        let mut ctrl = Vec::new();
        let mut ws = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (x, y) = (i as f64, j as f64);
                ctrl.push(Vec3::new(x, y, (x * 0.7).sin() * (y * 0.5).cos() + 0.1 * x * y));
                ws.push(1.0 + 0.3 * ((i + 2 * j) % 3) as f64);
            }
        }
        BSplineSurface::new(
            3,
            2,
            basis::clamped_uniform(nu, 3),
            vec![0., 0., 0., 0.3, 0.6, 1., 1., 1.],
            nu,
            nv,
            ctrl,
            if rational { Some(ws) } else { None },
        )
        .unwrap()
    }

    #[test]
    fn corners_interpolated() {
        let s = surf(true);
        assert!(vclose(s.eval(0., 0.), s.cp(0, 0), 1e-14));
        assert!(vclose(s.eval(1., 1.), s.cp(3, 4), 1e-14));
        assert!(vclose(s.eval(1., 0.), s.cp(3, 0), 1e-14));
    }

    #[test]
    fn partials_fd() {
        for s in [surf(false), surf(true)] {
            let h = 1e-5;
            for &(u, v) in &[(0.2, 0.4), (0.7, 0.1), (0.5, 0.8)] {
                let d = s.derivs2(u, v);
                let fu = (s.eval(u + h, v) - s.eval(u - h, v)) / (2.0 * h);
                let fv = (s.eval(u, v + h) - s.eval(u, v - h)) / (2.0 * h);
                assert!(vclose(d.du, fu, 1e-7));
                assert!(vclose(d.dv, fv, 1e-7));
                let fuu = (s.eval(u + h, v) - d.p * 2.0 + s.eval(u - h, v)) / (h * h);
                let fvv = (s.eval(u, v + h) - d.p * 2.0 + s.eval(u, v - h)) / (h * h);
                let fuv = (s.eval(u + h, v + h) - s.eval(u + h, v - h) - s.eval(u - h, v + h) + s.eval(u - h, v - h))
                    / (4.0 * h * h);
                assert!(vclose(d.duu, fuu, 1e-3));
                assert!(vclose(d.dvv, fvv, 1e-3));
                assert!(vclose(d.duv, fuv, 1e-4));
            }
        }
    }

    #[test]
    fn iso_and_insert_exact() {
        for s in [surf(false), surf(true)] {
            let cu = s.iso_u(0.37);
            let cv = s.iso_v(0.52);
            let ku = s.insert_knot_u(0.4, 2);
            let kv = s.insert_knot_v(0.45, 1);
            for i in 0..=10 {
                let t = i as f64 / 10.0;
                assert!(vclose(cu.eval(t), s.eval(0.37, t), 1e-12));
                assert!(vclose(cv.eval(t), s.eval(t, 0.52), 1e-12));
                assert!(vclose(ku.eval(t, 0.3), s.eval(t, 0.3), 1e-12));
                assert!(vclose(kv.eval(0.6, t), s.eval(0.6, t), 1e-12));
            }
            let t = s.transposed();
            assert!(vclose(t.eval(0.3, 0.8), s.eval(0.8, 0.3), 1e-12));
        }
    }

    #[test]
    fn closest_param_recovers() {
        let s = surf(true);
        let p = s.eval(0.33, 0.71);
        let n = s.normal(0.33, 0.71);
        let (u, v) = s.closest_param(p + n * 0.05);
        assert!(close(u, 0.33, 1e-7) && close(v, 0.71, 1e-7), "{u} {v}");
    }

    #[test]
    fn bilinear_plane_normal() {
        let s = BSplineSurface::bilinear(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1., 1., 0.));
        assert!(vclose(s.normal(0.5, 0.5), Vec3::Z, 1e-15));
        assert!(vclose(s.eval(0.5, 0.25), Vec3::new(0.5, 0.25, 0.), 1e-15));
    }
}
