//! Curve/surface fitting (Piegl & Tiller ch. 9): global interpolation with
//! chord-length / centripetal parameters and averaged knots, cubic
//! interpolation with end tangents, least-squares approximation with fixed
//! end points, and tensor-product surface interpolation.
//!
//! Evidence: rs_paths.txt crates/ck-geom/src/fit.rs; the edit-curve error
//! keys `editCurveFitFailed`, `editCurveToleranceNotReached` and the
//! rebuild keys `degree spans controlPoints knots tolerance` in
//! kernel.strings name these as fit consumers. Exact kernel parameter
//! choice is unpinned. GUESS: chord length (P&T default).

use crate::nurbs::basis::{self, basis_funs, ders_basis_funs, find_span};
use crate::nurbs::bspline::BSplineCurve;
use crate::nurbs::bsurf::BSplineSurface;
use crate::{Pt, Vec3};

/// Parameterisation scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Param {
    Uniform,
    ChordLength,
    Centripetal,
}

/// Parameters in [0,1] for the data points (eq. 9.4–9.6).
pub fn params<P: Pt>(pts: &[P], kind: Param) -> Vec<f64> {
    let n = pts.len();
    if n < 2 {
        return vec![0.0; n];
    }
    let steps: Vec<f64> = pts
        .windows(2)
        .map(|w| match kind {
            Param::Uniform => 1.0,
            Param::ChordLength => w[0].dist(w[1]),
            Param::Centripetal => w[0].dist(w[1]).sqrt(),
        })
        .collect();
    let total: f64 = steps.iter().sum();
    if total <= 0.0 {
        return (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
    }
    let mut out = vec![0.0];
    let mut acc = 0.0;
    for s in &steps[..n - 2] {
        acc += s;
        out.push(acc / total);
    }
    out.push(1.0);
    out
}

/// Knot vector by averaging (eq. 9.8) for `n+1` params, degree `p`.
pub fn averaged_knots(u: &[f64], p: usize) -> Vec<f64> {
    let n = u.len() - 1;
    let mut k = vec![0.0; p + 1];
    for j in 1..=(n - p) {
        k.push(u[j..j + p].iter().sum::<f64>() / p as f64);
    }
    k.extend(std::iter::repeat(1.0).take(p + 1));
    k
}

/// Solve A·X = B (dense, partial pivoting) with vector-valued RHS.
pub fn solve<P: Pt>(mut a: Vec<Vec<f64>>, mut b: Vec<P>) -> Result<Vec<P>, String> {
    let n = b.len();
    for c in 0..n {
        let piv = (c..n).max_by(|&i, &j| a[i][c].abs().partial_cmp(&a[j][c].abs()).unwrap()).unwrap();
        if a[piv][c].abs() < 1e-300 {
            return Err("singular fit system".into());
        }
        a.swap(c, piv);
        b.swap(c, piv);
        for r in (c + 1)..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                b[r] = b[r] - b[c] * f;
            }
        }
    }
    let mut x = vec![P::default(); n];
    for r in (0..n).rev() {
        let mut s = b[r];
        for k in (r + 1)..n {
            s = s - x[k] * a[r][k];
        }
        x[r] = s / a[r][r];
    }
    Ok(x)
}

/// Global interpolation through `pts` (A9.1) with given params/knots.
pub fn interpolate_with<P: Pt>(pts: &[P], p: usize, u: &[f64], knots: Vec<f64>) -> Result<BSplineCurve<P>, String> {
    let n = pts.len() - 1;
    let mut a = vec![vec![0.0; n + 1]; n + 1];
    for (k, &uk) in u.iter().enumerate() {
        let s = find_span(n, p, uk, &knots);
        let b = basis_funs(s, uk, p, &knots);
        for j in 0..=p {
            a[k][s - p + j] = b[j];
        }
    }
    let ctrl = solve(a, pts.to_vec())?;
    BSplineCurve::new(p, knots, ctrl)
}

/// Interpolating B-spline of degree `p` (clamped to n-1) through `pts`.
pub fn interpolate<P: Pt>(pts: &[P], p: usize, kind: Param) -> Result<BSplineCurve<P>, String> {
    if pts.len() < 2 {
        return Err("need at least 2 points".into());
    }
    let p = p.min(pts.len() - 1);
    let u = params(pts, kind);
    let knots = averaged_knots(&u, p);
    interpolate_with(pts, p, &u, knots)
}

/// Cubic interpolation with prescribed end derivatives `d0`, `dn`
/// (w.r.t. the [0,1] parameter; P&T §9.2.4: interior knots = interior params).
pub fn interpolate_cubic_tangents<P: Pt>(pts: &[P], d0: P, dn: P, kind: Param) -> Result<BSplineCurve<P>, String> {
    if pts.len() < 2 {
        return Err("need at least 2 points".into());
    }
    let n = pts.len() - 1;
    let u = params(pts, kind);
    let mut knots = vec![0.0; 4];
    knots.extend_from_slice(&u[1..n]);
    knots.extend([1.0; 4]);
    let m = n + 3; // unknowns
    let mut a = vec![vec![0.0; m]; m];
    let mut b = Vec::with_capacity(m);
    let last = m - 1;
    for (k, &uk) in u.iter().enumerate() {
        let s = find_span(last, 3, uk, &knots);
        let f = basis_funs(s, uk, 3, &knots);
        let row = if k == 0 { 0 } else if k == n { m - 1 } else { k + 1 };
        for j in 0..=3 {
            a[row][s - 3 + j] = f[j];
        }
        b.push((row, pts[k]));
    }
    for (row, uk, d) in [(1usize, 0.0, d0), (m - 2, 1.0, dn)] {
        let s = find_span(last, 3, uk, &knots);
        let f = ders_basis_funs(s, uk, 3, 1, &knots);
        for j in 0..=3 {
            a[row][s - 3 + j] = f[1][j];
        }
        b.push((row, d));
    }
    let mut rhs = vec![P::default(); m];
    for (r, v) in b {
        rhs[r] = v;
    }
    let ctrl = solve(a, rhs)?;
    BSplineCurve::new(3, knots, ctrl)
}

/// Least-squares approximation with `n_ctrl` control points, degree `p`,
/// end points interpolated (§9.4.1, knots by eq. 9.68–9.69).
pub fn approximate<P: Pt>(pts: &[P], p: usize, n_ctrl: usize, kind: Param) -> Result<BSplineCurve<P>, String> {
    let m = pts.len() - 1;
    if n_ctrl > pts.len() {
        return Err("more control points than data points".into());
    }
    if n_ctrl == pts.len() {
        return interpolate(pts, p, kind);
    }
    let p = p.min(n_ctrl - 1);
    let n = n_ctrl - 1;
    let u = params(pts, kind);
    // knots
    let d = (m + 1) as f64 / (n - p + 1) as f64;
    let mut knots = vec![0.0; p + 1];
    for j in 1..=(n - p) {
        let i = (j as f64 * d).floor() as usize;
        let alpha = j as f64 * d - i as f64;
        knots.push((1.0 - alpha) * u[i - 1] + alpha * u[i]);
    }
    knots.extend(std::iter::repeat(1.0).take(p + 1));
    approximate_with(pts, p, &u, knots)
}

/// Least-squares fit with explicit data params `u` and knot vector
/// (control count = knots.len() - p - 1); end points interpolated.
pub fn approximate_with<P: Pt>(pts: &[P], p: usize, u: &[f64], knots: Vec<f64>) -> Result<BSplineCurve<P>, String> {
    let m = pts.len() - 1;
    let n = knots.len() - p - 2;
    if n < 2 {
        return BSplineCurve::new(p, knots, vec![pts[0], pts[m]]);
    }
    // N[k][i] for interior points k=1..m-1, interior ctrl i=1..n-1
    let mut nmat = vec![vec![0.0; n + 1]; m + 1];
    for k in 0..=m {
        let s = find_span(n, p, u[k], &knots);
        let b = basis_funs(s, u[k], p, &knots);
        for j in 0..=p {
            nmat[k][s - p + j] = b[j];
        }
    }
    let rk: Vec<P> = (0..=m).map(|k| pts[k] - pts[0] * nmat[k][0] - pts[m] * nmat[k][n]).collect();
    let dim = n - 1;
    let mut ntn = vec![vec![0.0; dim]; dim];
    let mut rhs = vec![P::default(); dim];
    for i in 0..dim {
        for j in 0..dim {
            ntn[i][j] = (1..m).map(|k| nmat[k][i + 1] * nmat[k][j + 1]).sum();
        }
        let mut s = P::default();
        for k in 1..m {
            s = s + rk[k] * nmat[k][i + 1];
        }
        rhs[i] = s;
    }
    let inner = solve(ntn, rhs)?;
    let mut ctrl = vec![pts[0]];
    ctrl.extend(inner);
    ctrl.push(pts[m]);
    BSplineCurve::new(p, knots, ctrl)
}

/// Approximate to tolerance: grow the control count until the deviation
/// at the data params (an upper bound of the point-curve distance) is <= tol.
pub fn approximate_tol<P: Pt>(pts: &[P], p: usize, tol: f64, kind: Param) -> Result<BSplineCurve<P>, String> {
    let lo = (p + 1).min(pts.len());
    let u = params(pts, kind);
    for n in lo..=pts.len() {
        let c = approximate(pts, p, n, kind)?;
        // |C(u_k) - Q_k| bounds the true distance from above: cheap and safe
        let dev = u.iter().zip(pts).map(|(&uk, &q)| c.eval(uk).dist(q)).fold(0.0, f64::max);
        if dev <= tol {
            return Ok(c);
        }
    }
    interpolate(pts, p, kind)
}

/// Max distance from each point to the curve.
pub fn max_deviation<P: Pt>(c: &BSplineCurve<P>, pts: &[P]) -> f64 {
    pts.iter().map(|&q| c.closest_point(q).1.dist(q)).fold(0.0, f64::max)
}

/// Tensor-product interpolation of a grid `grid[i][j]` (i along u) (A9.4).
pub fn interpolate_surface(grid: &[Vec<Vec3>], pu: usize, pv: usize) -> Result<BSplineSurface, String> {
    let nu = grid.len();
    let nv = grid.first().map_or(0, |r| r.len());
    if nu < 2 || nv < 2 || grid.iter().any(|r| r.len() != nv) {
        return Err("grid must be rectangular, at least 2x2".into());
    }
    let pu = pu.min(nu - 1);
    let pv = pv.min(nv - 1);
    // averaged chord-length params across rows/columns (eq. 9.18)
    let avg = |lines: Vec<Vec<Vec3>>| -> Vec<f64> {
        let k = lines[0].len();
        let mut acc = vec![0.0; k];
        for l in &lines {
            for (a, b) in acc.iter_mut().zip(params(l, Param::ChordLength)) {
                *a += b;
            }
        }
        acc.iter().map(|a| a / lines.len() as f64).collect()
    };
    let uk = avg((0..nv).map(|j| (0..nu).map(|i| grid[i][j]).collect()).collect());
    let vl = avg(grid.to_vec());
    let ku = averaged_knots(&uk, pu);
    let kv = averaged_knots(&vl, pv);
    // interpolate in u for each column, then in v for each row of results
    let mut r = vec![vec![Vec3::ZERO; nv]; nu];
    for j in 0..nv {
        let col: Vec<Vec3> = (0..nu).map(|i| grid[i][j]).collect();
        let c = interpolate_with(&col, pu, &uk, ku.clone())?;
        for i in 0..nu {
            r[i][j] = c.ctrl[i];
        }
    }
    let mut ctrl = Vec::with_capacity(nu * nv);
    for i in 0..nu {
        let c = interpolate_with(&r[i], pv, &vl, kv.clone())?;
        ctrl.extend(c.ctrl);
    }
    BSplineSurface::new(pu, pv, ku, kv, nu, nv, ctrl, None)
}

/// Knots for parameters `u` rescaled to [0, 1], averaged, and mapped back.
pub fn scaled_knots(u: &[f64], p: usize) -> Vec<f64> {
    let (u0, u1) = (u[0], u[u.len() - 1]);
    let un: Vec<f64> = u.iter().map(|x| (x - u0) / (u1 - u0)).collect();
    averaged_knots(&un, p).into_iter().map(|k| u0 + k * (u1 - u0)).collect()
}

/// Surface through `grid[i][j]` (i along the parameters `u`, j along `v`), degrees `pu`, `pv`.
pub fn grid_surface(grid: &[Vec<Vec3>], u: &[f64], v: &[f64], pu: usize, pv: usize) -> Result<BSplineSurface, String> {
    let (nu, nv) = (grid.len(), grid[0].len());
    let (pu, pv) = (pu.min(nu - 1), pv.min(nv - 1));
    let ku = scaled_knots(u, pu);
    let kv = scaled_knots(v, pv);
    let mut r = vec![vec![Vec3::ZERO; nv]; nu];
    for j in 0..nv {
        let col: Vec<Vec3> = (0..nu).map(|i| grid[i][j]).collect();
        let c = interpolate_with(&col, pu, u, ku.clone())?;
        for i in 0..nu {
            r[i][j] = c.ctrl[i];
        }
    }
    let mut ctrl = Vec::with_capacity(nu * nv);
    for row in r.iter() {
        let c = interpolate_with(row, pv, v, kv.clone())?;
        ctrl.extend(c.ctrl);
    }
    BSplineSurface::new(pu, pv, ku, kv, nu, nv, ctrl, None)
}

/// Uniform clamped knots helper re-export for callers building fits.
pub fn uniform_knots(n_ctrl: usize, p: usize) -> Vec<f64> {
    basis::clamped_uniform(n_ctrl, p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;
    use crate::Vec2;

    fn data() -> Vec<Vec3> {
        (0..9).map(|i| {
            let t = i as f64 * 0.4;
            Vec3::new(t.cos() * 3.0, t.sin() * 2.0, t * 0.5)
        })
        .collect()
    }

    #[test]
    fn pt_example_9_1() {
        // P&T Ex 9.1: Q = (0,0),(3,4),(-1,4),(-4,0),(-4,-3); chord params
        // 0, 5/17, 9/17, 14/17, 1; cubic knots 0,0,0,0,28/51,1,1,1,1.
        let q = [Vec2::new(0., 0.), Vec2::new(3., 4.), Vec2::new(-1., 4.), Vec2::new(-4., 0.), Vec2::new(-4., -3.)];
        let u = params(&q, Param::ChordLength);
        for (a, b) in u.iter().zip([0.0, 5.0 / 17.0, 9.0 / 17.0, 14.0 / 17.0, 1.0]) {
            assert!(close(*a, b, 1e-15));
        }
        let k = averaged_knots(&u, 3);
        assert!(close(k[4], 28.0 / 51.0, 1e-15));
        let c = interpolate(&q, 3, Param::ChordLength).unwrap();
        for (uk, qk) in u.iter().zip(q.iter()) {
            assert!(v2close(c.eval(*uk), *qk, 1e-13));
        }
    }

    #[test]
    fn interpolation_passes_through() {
        let pts = data();
        for kind in [Param::Uniform, Param::ChordLength, Param::Centripetal] {
            for p in [1, 2, 3, 5] {
                let c = interpolate(&pts, p, kind).unwrap();
                let u = params(&pts, kind);
                for (uk, q) in u.iter().zip(pts.iter()) {
                    assert!(vclose(c.eval(*uk), *q, 1e-12), "p={p} {kind:?}");
                }
            }
        }
    }

    #[test]
    fn end_tangents() {
        let pts = data();
        let (d0, dn) = (Vec3::new(0., 5., 1.), Vec3::new(-3., 1., 2.));
        let c = interpolate_cubic_tangents(&pts, d0, dn, Param::ChordLength).unwrap();
        assert!(vclose(c.derivs(0.0, 1)[1], d0, 1e-12));
        assert!(vclose(c.derivs(1.0, 1)[1], dn, 1e-12));
        let u = params(&pts, Param::ChordLength);
        for (uk, q) in u.iter().zip(pts.iter()) {
            assert!(vclose(c.eval(*uk), *q, 1e-12));
        }
    }

    #[test]
    fn least_squares_reproduces_spline() {
        // sample a cubic with 6 ctrl points densely; LSQ with 6 ctrl must be ~exact
        let src = BSplineCurve::clamped(
            3,
            vec![Vec3::ZERO, Vec3::new(1., 2., 0.), Vec3::new(2., -1., 1.), Vec3::new(3., 1., 0.), Vec3::new(4., 0., 2.), Vec3::new(5., 1., 1.)],
        )
        .unwrap();
        let pts = src.sample(60);
        let c = approximate(&pts, 3, 6, Param::Uniform).unwrap();
        assert!(vclose(c.start(), pts[0], 1e-15) && vclose(c.end(), pts[60], 1e-15));
        let dev = max_deviation(&c, &pts);
        assert!(dev < 2e-2, "dev={dev}");
        // with the source's own params + knots the LSQ solution is exact
        let u: Vec<f64> = (0..=60).map(|i| i as f64 / 60.0).collect();
        let e = approximate_with(&pts, 3, &u, src.knots.clone()).unwrap();
        for (a, b) in e.ctrl.iter().zip(src.ctrl.iter()) {
            assert!(vclose(*a, *b, 1e-12));
        }
        let t = approximate_tol(&pts, 3, 1e-4, Param::ChordLength).unwrap();
        assert!(max_deviation(&t, &pts) <= 1e-4);
        assert!(t.ctrl.len() < pts.len());
    }

    #[test]
    fn surface_interpolation() {
        let grid: Vec<Vec<Vec3>> = (0..5)
            .map(|i| (0..4).map(|j| Vec3::new(i as f64, j as f64 * 1.3, ((i * j) as f64 * 0.4).sin())).collect())
            .collect();
        let s = interpolate_surface(&grid, 3, 2).unwrap();
        // corners and every grid point lie on the surface
        assert!(vclose(s.eval(0., 0.), grid[0][0], 1e-12));
        assert!(vclose(s.eval(1., 1.), grid[4][3], 1e-12));
        for row in &grid {
            for &p in row {
                let (u, v) = s.closest_param(p);
                assert!(s.eval(u, v).dist(p) < 1e-8);
            }
        }
    }
}
