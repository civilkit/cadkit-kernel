//! Tolerance-bounded B-spline approximation of curves and surfaces given as an
//! evaluator closure or as sampled points, plus deviation measures.
//!
//! Method (after the solidcore fit/approximation tools, Apache-2.0, see
//! THIRD_PARTY.md): least-squares fit with end points interpolated
//! ([`crate::fit::approximate_with`]) on a knot vector that is refined by
//! bisecting every knot span whose error exceeds the tolerance, until the
//! error of the fit is within the tolerance (or the refinement cannot go on, an
//! `Err`). Surfaces are fitted as a tensor product, one direction after the
//! other (the least-squares problem is separable).
//!
//! The error reported with a fit is the largest point distance between the fit
//! and the source at the samples used, for an evaluator also at the midpoints
//! between samples: a parametric distance, so an upper bound of the geometric
//! distance at those points. [`curve_deviation`] / [`surface_deviation`] /
//! [`fn_curve_deviation`] / [`fn_surface_deviation`] measure the geometric
//! (closest point) distance independently.

use crate::nurbs::basis::{basis_funs, find_span};
use crate::nurbs::bspline::BSplineCurve;
use crate::nurbs::bsurf::BSplineSurface;
use crate::{Pt, Vec3};

/// Most control points per direction a curve fit may grow to.
const MAX_CTRL: usize = 600;
/// The same for a surface fit.
const MAX_CTRL_SURF: usize = 300;

/// Least-squares fit with the end points interpolated for one knot vector and
/// parameter list, factorised once (Cholesky of the normal equations) so that
/// many point lists (the rows / columns of a surface) cost one back-substitution
/// each.
struct LsFit {
    p: usize,
    knots: Vec<f64>,
    /// basis matrix, row per data parameter, column per control point
    nmat: Vec<Vec<f64>>,
    /// lower Cholesky factor of the interior normal matrix
    chol: Vec<Vec<f64>>,
}

impl LsFit {
    fn new(p: usize, ts: &[f64], knots: Vec<f64>) -> Result<Self, String> {
        let m = ts.len() - 1;
        let nc = knots.len() - p - 1;
        if nc < 2 || nc > ts.len() {
            return Err("control count does not fit the data".into());
        }
        let n = nc - 1;
        let mut nmat = vec![vec![0.0; nc]; m + 1];
        for (k, &t) in ts.iter().enumerate() {
            let s = find_span(n, p, t, &knots);
            let b = basis_funs(s, t, p, &knots);
            for j in 0..=p {
                nmat[k][s - p + j] = b[j];
            }
        }
        let dim = n - 1;
        let mut a = vec![vec![0.0; dim]; dim];
        for i in 0..dim {
            for j in 0..=i {
                let v: f64 = (1..m).map(|k| nmat[k][i + 1] * nmat[k][j + 1]).sum();
                a[i][j] = v;
            }
        }
        // Cholesky in place (lower)
        for i in 0..dim {
            for j in 0..=i {
                let mut v = a[i][j];
                for k in 0..j {
                    v -= a[i][k] * a[j][k];
                }
                if i == j {
                    if !(v > 1e-14) {
                        return Err("singular fit system".into());
                    }
                    a[i][i] = v.sqrt();
                } else {
                    a[i][j] = v / a[j][j];
                }
            }
        }
        Ok(LsFit { p, knots, nmat, chol: a })
    }

    /// Control points for the data `pts` (one per parameter).
    fn solve<P: Pt>(&self, pts: &[P]) -> Vec<P> {
        let m = pts.len() - 1;
        let nc = self.knots.len() - self.p - 1;
        let n = nc - 1;
        let dim = n - 1;
        let mut rhs = vec![P::default(); dim];
        for k in 1..m {
            let r = pts[k] - pts[0] * self.nmat[k][0] - pts[m] * self.nmat[k][n];
            for i in 0..dim {
                let w = self.nmat[k][i + 1];
                if w != 0.0 {
                    rhs[i] = rhs[i] + r * w;
                }
            }
        }
        for i in 0..dim {
            let mut v = rhs[i];
            for k in 0..i {
                v = v - rhs[k] * self.chol[i][k];
            }
            rhs[i] = v / self.chol[i][i];
        }
        for i in (0..dim).rev() {
            let mut v = rhs[i];
            for k in i + 1..dim {
                v = v - rhs[k] * self.chol[k][i];
            }
            rhs[i] = v / self.chol[i][i];
        }
        let mut ctrl = Vec::with_capacity(nc);
        ctrl.push(pts[0]);
        ctrl.extend(rhs);
        ctrl.push(pts[m]);
        ctrl
    }
}

fn mid_params(ts: &[f64]) -> Vec<f64> {
    ts.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect()
}

fn uniform(t0: f64, t1: f64, n: usize) -> Vec<f64> {
    (0..n).map(|i| if i + 1 == n { t1 } else { t0 + (t1 - t0) * i as f64 / (n - 1) as f64 }).collect()
}

/// `ts` plus the knots and span midpoints of `[t0, interior.., t1]`, sorted and distinct.
fn with_spans(mut ts: Vec<f64>, (t0, t1): (f64, f64), interior: &[f64]) -> Vec<f64> {
    let mut edges = vec![t0];
    edges.extend_from_slice(interior);
    edges.push(t1);
    for w in edges.windows(2) {
        ts.extend([w[0], 0.5 * (w[0] + w[1]), w[1]]);
    }
    ts.sort_by(|a, b| a.total_cmp(b));
    ts.dedup_by(|a, b| (*a - *b).abs() <= 1e-13 * (t1 - t0));
    ts
}

fn check_range(t0: f64, t1: f64) -> Result<(), String> {
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
        return Err(format!("empty or non-finite parameter range [{t0}, {t1}]"));
    }
    Ok(())
}

fn clamped_knots(t0: f64, t1: f64, p: usize, interior: &[f64]) -> Vec<f64> {
    let mut k = vec![t0; p + 1];
    k.extend_from_slice(interior);
    k.extend(std::iter::repeat(t1).take(p + 1));
    k
}

/// Interior breakpoints strictly inside (t0, t1), sorted and distinct.
fn interior_of(breaks: &[f64], t0: f64, t1: f64) -> Vec<f64> {
    let mut v: Vec<f64> = breaks.iter().copied().filter(|&b| b.is_finite() && b > t0 && b < t1).collect();
    v.sort_by(|a, b| a.total_cmp(b));
    v.dedup_by(|a, b| (*a - *b).abs() < 1e-12 * (t1 - t0));
    v
}

/// Where to split the span (a, b): its midpoint, or (for sampled data) the
/// median data parameter inside it, `None` when too few data lie inside.
fn split_point(data: Option<&[f64]>, a: f64, b: f64) -> Option<f64> {
    match data {
        None => Some(0.5 * (a + b)),
        Some(ts) => {
            let lo = ts.partition_point(|&t| t <= a);
            let hi = ts.partition_point(|&t| t < b);
            (hi >= lo + 3).then(|| ts[(lo + hi) / 2])
        }
    }
}

/// Bisect every span of `interior` (within `[t0, t1]`) flagged in `bad`
/// (indexed by span). Returns whether a knot was added.
fn refine(interior: &mut Vec<f64>, t0: f64, t1: f64, bad: &[bool], data: Option<&[f64]>) -> bool {
    let mut edges = vec![t0];
    edges.extend_from_slice(interior);
    edges.push(t1);
    let mut added = false;
    for (s, w) in edges.windows(2).enumerate() {
        if bad[s] {
            if let Some(k) = split_point(data, w[0], w[1]) {
                if k > w[0] && k < w[1] {
                    interior.push(k);
                    added = true;
                }
            }
        }
    }
    interior.sort_by(|a, b| a.total_cmp(b));
    added
}

/// Span index of `t` among the edges `[t0, interior.., t1]`.
fn span_of(interior: &[f64], t: f64) -> usize {
    interior.partition_point(|&k| k <= t)
}

/// Source of the data a fit runs on.
enum Source<'a, P> {
    /// Evaluator: sampled on a uniform grid sized to the control count.
    Eval(&'a dyn Fn(f64) -> P),
    /// Fixed data parameters and points.
    Points(&'a [f64], &'a [P]),
}

fn fit_curve<P: Pt>(src: Source<P>, range: (f64, f64), p: usize, tol: f64, breaks: &[f64]) -> Result<(BSplineCurve<P>, f64), String> {
    let (t0, t1) = range;
    check_range(t0, t1)?;
    if !(tol > 0.0) {
        return Err("tolerance must be positive".into());
    }
    if p == 0 {
        return Err("degree must be at least 1".into());
    }
    // a break is a knot of multiplicity p (C0): the source may have a kink there
    let mut interior: Vec<f64> = interior_of(breaks, t0, t1).into_iter().flat_map(|b| std::iter::repeat(b).take(p)).collect();
    let mut last_err = f64::INFINITY;
    for _round in 0..40 {
        let nctrl = interior.len() + p + 1;
        let (ts, pts): (Vec<f64>, Vec<P>) = match &src {
            Source::Eval(f) => {
                let ts = with_spans(uniform(t0, t1, (3 * nctrl + 8).max(32)), (t0, t1), &interior);
                let pts = ts.iter().map(|&t| f(t)).collect();
                (ts, pts)
            }
            Source::Points(ts, pts) => (ts.to_vec(), pts.to_vec()),
        };
        if pts.iter().any(|q| !q.dot(*q).is_finite()) {
            return Err("non-finite sample".into());
        }
        if nctrl > MAX_CTRL || nctrl > ts.len() {
            return Err(format!("tolerance {tol:.3e} not reached (error {last_err:.3e} with {nctrl} control points)"));
        }
        let knots = clamped_knots(t0, t1, p, &interior);
        let c = BSplineCurve::new(p, knots.clone(), LsFit::new(p, &ts, knots)?.solve(&pts))?;
        let mut worst = vec![0.0f64; interior.len() + 1];
        let mut dev = 0.0f64;
        let mut note = |t: f64, e: f64| {
            let s = span_of(&interior, t);
            worst[s] = worst[s].max(e);
            dev = dev.max(e);
        };
        for (&t, q) in ts.iter().zip(&pts) {
            note(t, c.eval(t).dist(*q));
        }
        if let Source::Eval(f) = &src {
            for t in mid_params(&ts) {
                note(t, c.eval(t).dist(f(t)));
            }
        }
        last_err = dev;
        if dev <= tol {
            return Ok((c, dev));
        }
        let bad: Vec<bool> = worst.iter().map(|&e| e > tol).collect();
        let data = if let Source::Points(ts, _) = &src { Some(*ts) } else { None };
        if !refine(&mut interior, t0, t1, &bad, data) {
            return Err(format!("tolerance {tol:.3e} not reached (error {dev:.3e}, no span left to split)"));
        }
    }
    Err(format!("tolerance {tol:.3e} not reached (error {last_err:.3e})"))
}

/// B-spline of degree `p` on `range` within `tol` of the curve `f` (the largest
/// distance between `f(t)` and the fit at the same `t`, sampled including the
/// midpoints between samples, is returned next to the curve). `breaks` are
/// parameters where `f` is not smooth enough for one span (seeded as knots).
/// `Err` when the tolerance is not reached within 600 control points.
pub fn approximate_curve_fn<P: Pt>(
    f: &dyn Fn(f64) -> P,
    range: (f64, f64),
    p: usize,
    tol: f64,
    breaks: &[f64],
) -> Result<(BSplineCurve<P>, f64), String> {
    fit_curve(Source::Eval(f), range, p, tol, breaks)
}

/// B-spline of degree `p` through the neighbourhood of the points `pts` at the
/// (strictly increasing) parameters `ts`, with every point within `tol` of the
/// fit at its own parameter; that distance is returned next to the curve.
pub fn approximate_curve_points<P: Pt>(ts: &[f64], pts: &[P], p: usize, tol: f64) -> Result<(BSplineCurve<P>, f64), String> {
    if ts.len() != pts.len() || ts.len() < 2 {
        return Err("need matching parameters and points (at least 2)".into());
    }
    if ts.windows(2).any(|w| !(w[1] > w[0])) {
        return Err("parameters must be strictly increasing".into());
    }
    fit_curve(Source::Points(ts, pts), (ts[0], ts[ts.len() - 1]), p, tol, &[])
}

enum SSource<'a> {
    Eval(&'a dyn Fn(f64, f64) -> Vec3),
    Grid(&'a [f64], &'a [f64], &'a [Vec<Vec3>]),
}

fn fit_surface(
    src: SSource,
    ur: (f64, f64),
    vr: (f64, f64),
    (pu, pv): (usize, usize),
    tol: f64,
    bu: &[f64],
    bv: &[f64],
) -> Result<(BSplineSurface, f64), String> {
    check_range(ur.0, ur.1)?;
    check_range(vr.0, vr.1)?;
    if !(tol > 0.0) {
        return Err("tolerance must be positive".into());
    }
    if pu == 0 || pv == 0 {
        return Err("degree must be at least 1".into());
    }
    let mut iu: Vec<f64> = interior_of(bu, ur.0, ur.1).into_iter().flat_map(|b| std::iter::repeat(b).take(pu)).collect();
    let mut iv: Vec<f64> = interior_of(bv, vr.0, vr.1).into_iter().flat_map(|b| std::iter::repeat(b).take(pv)).collect();
    let mut last_err = f64::INFINITY;
    for _round in 0..40 {
        let (ncu, ncv) = (iu.len() + pu + 1, iv.len() + pv + 1);
        let (us, vs, grid): (Vec<f64>, Vec<f64>, Vec<Vec<Vec3>>) = match &src {
            SSource::Eval(f) => {
                let us = with_spans(uniform(ur.0, ur.1, (2 * ncu + 8).max(24)), ur, &iu);
                let vs = with_spans(uniform(vr.0, vr.1, (2 * ncv + 8).max(24)), vr, &iv);
                let grid = us.iter().map(|&u| vs.iter().map(|&v| f(u, v)).collect()).collect();
                (us, vs, grid)
            }
            SSource::Grid(us, vs, g) => (us.to_vec(), vs.to_vec(), g.to_vec()),
        };
        if grid.iter().flatten().any(|q| !q.dot(*q).is_finite()) {
            return Err("non-finite sample".into());
        }
        if ncu > MAX_CTRL_SURF || ncv > MAX_CTRL_SURF || ncu > us.len() || ncv > vs.len() {
            return Err(format!("tolerance {tol:.3e} not reached (error {last_err:.3e} with {ncu} x {ncv} control points)"));
        }
        let (ku, kv) = (clamped_knots(ur.0, ur.1, pu, &iu), clamped_knots(vr.0, vr.1, pv, &iv));
        // stage 1: along u, one fit per v sample
        let fu = LsFit::new(pu, &us, ku.clone())?;
        let fv = LsFit::new(pv, &vs, kv.clone())?;
        let mut mid = vec![vec![Vec3::ZERO; vs.len()]; ncu];
        for j in 0..vs.len() {
            let col: Vec<Vec3> = grid.iter().map(|r| r[j]).collect();
            let c = fu.solve(&col);
            for i in 0..ncu {
                mid[i][j] = c[i];
            }
        }
        // stage 2: along v, one fit per control row
        let mut ctrl = Vec::with_capacity(ncu * ncv);
        for row in &mid {
            ctrl.extend(fv.solve(row));
        }
        let s = BSplineSurface::new(pu, pv, ku, kv, ncu, ncv, ctrl, None)?;
        // error at the samples (and, for an evaluator, at the midpoints)
        let (tu, tv) = match &src {
            SSource::Eval(_) => {
                let mut tu = us.clone();
                tu.extend(mid_params(&us));
                let mut tv = vs.clone();
                tv.extend(mid_params(&vs));
                (tu, tv)
            }
            SSource::Grid(..) => (us.clone(), vs.clone()),
        };
        let pts = s.eval_grid(&tu, &tv);
        let mut wu = vec![0.0f64; iu.len() + 1];
        let mut wv = vec![0.0f64; iv.len() + 1];
        let mut dev = 0.0f64;
        for (a, &u) in tu.iter().enumerate() {
            let su = span_of(&iu, u);
            for (b, &v) in tv.iter().enumerate() {
                let truth = match &src {
                    SSource::Eval(f) => f(u, v),
                    SSource::Grid(..) => grid[a][b],
                };
                let e = pts[a * tv.len() + b].dist(truth);
                let sv = span_of(&iv, v);
                wu[su] = wu[su].max(e);
                wv[sv] = wv[sv].max(e);
                dev = dev.max(e);
            }
        }
        last_err = dev;
        if dev <= tol {
            return Ok((s, dev));
        }
        let badu: Vec<bool> = wu.iter().map(|&e| e > tol).collect();
        let badv: Vec<bool> = wv.iter().map(|&e| e > tol).collect();
        let (du, dv) = match &src {
            SSource::Grid(us, vs, _) => (Some(*us), Some(*vs)),
            _ => (None, None),
        };
        let a = refine(&mut iu, ur.0, ur.1, &badu, du);
        let b = refine(&mut iv, vr.0, vr.1, &badv, dv);
        if !(a || b) {
            return Err(format!("tolerance {tol:.3e} not reached (error {dev:.3e}, no span left to split)"));
        }
    }
    Err(format!("tolerance {tol:.3e} not reached (error {last_err:.3e})"))
}

/// B-spline surface of degrees `deg = (pu, pv)` over the given ranges within
/// `tol` of the evaluator `f` (the error is the largest distance at the same
/// parameters on the sample grid and the midpoints between samples, returned
/// next to the surface). `break_u` / `break_v` seed knots where `f` is not
/// smooth. `Err` when the tolerance is not reached within 300 control points
/// per direction.
pub fn approximate_surface_fn(
    f: &dyn Fn(f64, f64) -> Vec3,
    u_range: (f64, f64),
    v_range: (f64, f64),
    deg: (usize, usize),
    tol: f64,
    break_u: &[f64],
    break_v: &[f64],
) -> Result<(BSplineSurface, f64), String> {
    fit_surface(SSource::Eval(f), u_range, v_range, deg, tol, break_u, break_v)
}

/// B-spline surface fitted to the point grid `grid[i][j]` at the (strictly
/// increasing) parameters `us[i]`, `vs[j]`, every point within `tol` of the
/// surface at its own parameters; that distance is returned with the surface.
pub fn approximate_surface_grid(
    us: &[f64],
    vs: &[f64],
    grid: &[Vec<Vec3>],
    deg: (usize, usize),
    tol: f64,
) -> Result<(BSplineSurface, f64), String> {
    if us.len() < 2 || vs.len() < 2 || grid.len() != us.len() || grid.iter().any(|r| r.len() != vs.len()) {
        return Err("grid does not match the parameter lists (at least 2 x 2)".into());
    }
    if us.windows(2).any(|w| !(w[1] > w[0])) || vs.windows(2).any(|w| !(w[1] > w[0])) {
        return Err("parameters must be strictly increasing".into());
    }
    fit_surface(SSource::Grid(us, vs, grid), (us[0], us[us.len() - 1]), (vs[0], vs[vs.len() - 1]), deg, tol, &[], &[])
}

fn per_span_samples(knots: &[f64], per: usize) -> Vec<f64> {
    let ks = crate::nurbs::basis::distinct_knots(knots);
    let mut out = Vec::new();
    for w in ks.windows(2) {
        for s in 0..per {
            out.push(w[0] + (w[1] - w[0]) * s as f64 / per as f64);
        }
    }
    if let Some(&l) = ks.last() {
        out.push(l);
    }
    out
}

/// One-sided deviation: the largest distance from the samples of `a` (6 per
/// knot span) to the curve `b`.
fn curve_one_sided<P: Pt>(a: &BSplineCurve<P>, b: &BSplineCurve<P>) -> f64 {
    per_span_samples(&a.knots, 6).into_iter().map(|u| b.closest_point(a.eval(u)).1.dist(a.eval(u))).fold(0.0, f64::max)
}

/// Largest closest-point distance between two curves, both ways (a sampled
/// Hausdorff distance, 6 samples per knot span).
pub fn curve_deviation<P: Pt>(a: &BSplineCurve<P>, b: &BSplineCurve<P>) -> f64 {
    curve_one_sided(a, b).max(curve_one_sided(b, a))
}

fn surface_one_sided(a: &BSplineSurface, b: &BSplineSurface) -> f64 {
    let us = per_span_samples(&a.knots_u, 3);
    let vs = per_span_samples(&a.knots_v, 3);
    let pts = a.eval_grid(&us, &vs);
    let step = (pts.len() / 4000).max(1);
    let mut dev = 0.0f64;
    for q in pts.iter().step_by(step) {
        let (u, v) = b.closest_param(*q);
        dev = dev.max(b.eval(u, v).dist(*q));
    }
    dev
}

/// Largest closest-point distance between two surfaces, both ways (sampled, 3
/// per knot cell edge, at most about 4000 points per side).
pub fn surface_deviation(a: &BSplineSurface, b: &BSplineSurface) -> f64 {
    surface_one_sided(a, b).max(surface_one_sided(b, a))
}

/// Largest distance from `n + 1` points `f(t)` of the evaluator to the curve.
pub fn fn_curve_deviation<P: Pt>(f: &dyn Fn(f64) -> P, range: (f64, f64), c: &BSplineCurve<P>, n: usize) -> f64 {
    uniform(range.0, range.1, n.max(2) + 1)
        .into_iter()
        .map(|t| {
            let q = f(t);
            c.closest_point(q).1.dist(q)
        })
        .fold(0.0, f64::max)
}

/// Largest distance from an `(n + 1) x (n + 1)` grid of points of the evaluator to the surface.
pub fn fn_surface_deviation(f: &dyn Fn(f64, f64) -> Vec3, u_range: (f64, f64), v_range: (f64, f64), s: &BSplineSurface, n: usize) -> f64 {
    let n = n.max(2) + 1;
    let mut dev = 0.0f64;
    for u in uniform(u_range.0, u_range.1, n) {
        for v in uniform(v_range.0, v_range.1, n) {
            let q = f(u, v);
            let (a, b) = s.closest_param(q);
            dev = dev.max(s.eval(a, b).dist(q));
        }
    }
    dev
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Vec2, Vec3};

    #[test]
    fn curve_fn_holds_tolerance_3d() {
        let f = |t: f64| Vec3::new(t.cos() * 3.0, t.sin() * 2.0, 0.3 * t);
        for tol in [1e-2, 1e-4, 1e-6] {
            let (c, dev) = approximate_curve_fn(&f, (0.0, 5.0), 3, tol, &[]).unwrap();
            assert!(dev <= tol);
            // independent sampled check at the same parameters
            let worst = (0..=500).map(|i| 5.0 * i as f64 / 500.0).map(|t| c.eval(t).dist(f(t))).fold(0.0, f64::max);
            assert!(worst <= tol * 1.5, "{worst} vs {tol}");
            // geometric distance is no larger
            assert!(fn_curve_deviation(&f, (0.0, 5.0), &c, 200) <= tol * 1.5);
        }
    }

    #[test]
    fn curve_fn_2d_and_breaks() {
        // a kink at t = 1: seeded as a break it fits cleanly
        let f = |t: f64| Vec2::new(t, if t < 1.0 { 0.0 } else { t - 1.0 });
        let (c, dev) = approximate_curve_fn(&f, (0.0, 2.0), 3, 1e-8, &[1.0]).unwrap();
        assert!(dev <= 1e-8);
        assert!(c.eval(1.5).dist(f(1.5)) <= 1e-8);
        // invalid input is an error, not a panic
        assert!(approximate_curve_fn(&f, (1.0, 1.0), 3, 1e-3, &[]).is_err());
        assert!(approximate_curve_fn(&f, (0.0, 1.0), 3, 0.0, &[]).is_err());
        assert!(approximate_curve_fn(&|_t: f64| Vec2::new(f64::NAN, 0.0), (0.0, 1.0), 3, 1e-3, &[]).is_err());
    }

    #[test]
    fn curve_points_and_deviation_measure() {
        let ts: Vec<f64> = (0..60).map(|i| i as f64 * 0.1).collect();
        let pts: Vec<Vec3> = ts.iter().map(|&t| Vec3::new(t, t.sin(), (0.5 * t).cos())).collect();
        let (c, dev) = approximate_curve_points(&ts, &pts, 3, 1e-4).unwrap();
        assert!(dev <= 1e-4);
        for (&t, q) in ts.iter().zip(&pts) {
            assert!(c.eval(t).dist(*q) <= 1e-4 + 1e-12);
        }
        let loose = approximate_curve_points(&ts, &pts, 3, 0.05).unwrap().0;
        assert!(loose.ctrl.len() < c.ctrl.len());
        let d = curve_deviation(&c, &loose);
        assert!(d > 0.0 && d < 0.06, "{d}");
        assert!(curve_deviation(&c, &c) < 1e-9);
        assert!(approximate_curve_points(&ts[..1], &pts[..1], 3, 1e-3).is_err());
        let mut bad = ts.clone();
        bad[5] = bad[4];
        assert!(approximate_curve_points(&bad, &pts, 3, 1e-3).is_err());
    }

    #[test]
    fn surface_fn_and_grid() {
        let f = |u: f64, v: f64| Vec3::new(u, v, 0.4 * (2.0 * u).sin() * (1.5 * v).cos());
        let (s, dev) = approximate_surface_fn(&f, (0.0, 2.0), (0.0, 2.0), (3, 3), 1e-5, &[], &[]).unwrap();
        assert!(dev <= 1e-5);
        for i in 0..=20 {
            for j in 0..=20 {
                let (u, v) = (0.1 * i as f64, 0.1 * j as f64);
                assert!(s.eval(u, v).dist(f(u, v)) <= 1.5e-5);
            }
        }
        assert!(fn_surface_deviation(&f, (0.0, 2.0), (0.0, 2.0), &s, 12) <= 1.5e-5);
        // grid input
        let us: Vec<f64> = (0..30).map(|i| i as f64 / 29.0 * 2.0).collect();
        let vs: Vec<f64> = (0..25).map(|i| i as f64 / 24.0 * 2.0).collect();
        let grid: Vec<Vec<Vec3>> = us.iter().map(|&u| vs.iter().map(|&v| f(u, v)).collect()).collect();
        let (g, gdev) = approximate_surface_grid(&us, &vs, &grid, (3, 3), 1e-3).unwrap();
        assert!(gdev <= 1e-3);
        for (i, &u) in us.iter().enumerate() {
            for (j, &v) in vs.iter().enumerate() {
                assert!(g.eval(u, v).dist(grid[i][j]) <= 1e-3 + 1e-12);
            }
        }
        assert!(surface_deviation(&s, &g) < 2e-3);
        assert!(g.nu < us.len() && g.nv < vs.len());
        assert!(approximate_surface_grid(&us[..1], &vs, &grid[..1], (3, 3), 1e-3).is_err());
    }
}
