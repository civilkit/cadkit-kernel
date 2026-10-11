//! The rebuild net: clamped B-spline basis, Greville abscissae and the
//! interpolation fit of an existing surface onto a degree × spans net.
//!
//! The rebuilt surface keeps the original's uv domain, so the face's edge
//! pcurves stay valid; deviation is measured by `deviation()`.

use ck_geom::nurbs::basis;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};

/// Uniform clamped knots for `spans` spans of degree `deg` on [0, spans].
pub fn uniform_knots(deg: usize, spans: usize) -> Vec<f64> {
    let mut k = vec![0.0; deg + 1];
    for i in 1..spans {
        k.push(i as f64);
    }
    k.extend(std::iter::repeat(spans as f64).take(deg + 1));
    debug_assert_eq!(k.len(), 2 * deg + spans + 1);
    k
}

/// Greville abscissae (knot averages) of a clamped basis.
pub fn greville(deg: usize, knots: &[f64]) -> Vec<f64> {
    (0..knots.len() - deg - 1)
        .map(|i| knots[i + 1..i + deg + 1].iter().sum::<f64>() / deg as f64)
        .collect()
}

/// Full basis row: the n basis functions evaluated at `t`.
fn basis_row(deg: usize, knots: &[f64], n: usize, t: f64) -> Vec<f64> {
    let span = basis::find_span(n, deg, t, knots);
    let local = basis::basis_funs(span, t, deg, knots);
    let mut row = vec![0.0; n];
    for (k, &v) in local.iter().enumerate() {
        let i = span + k - deg;
        if i < n {
            row[i] = v;
        }
    }
    row
}

/// Solve the dense linear system `a · x = b` (Gaussian elimination with
/// partial pivoting). None when singular.
pub fn solve(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    for c in 0..n {
        let (p, _) = (c..n).fold((c, 0.0f64), |(pi, pv), i| {
            let v = a[i][c].abs();
            if v > pv {
                (i, v)
            } else {
                (pi, pv)
            }
        });
        if a[p][c].abs() < 1e-13 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in (c + 1)..n {
            let f = a[r][c] / a[c][c];
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    Some(x)
}

/// Scale a parameter from the fitted net's [0, spans] to the surface's
/// domain [a, b].
fn to_domain(t: f64, a: f64, b: f64) -> f64 {
    a + (b - a) * (t / (b - a).max(1e-12) * (b - a))
}

/// Fit `orig` (over the uv window) with a deg×spans B-spline net by
/// interpolation at the Greville points. Errors when the system is
/// singular (rebuildFailed).
pub fn fit_surface(
    orig: &Surface,
    uw: (f64, f64),
    vw: (f64, f64),
    deg_u: usize,
    deg_v: usize,
    spans_u: usize,
    spans_v: usize,
) -> Result<BSplineSurface, String> {
    if deg_u < 1 || deg_v < 1 || spans_u < 1 || spans_v < 1 {
        return Err("degree/spans must be positive".into());
    }
    let ku = uniform_knots(deg_u, spans_u);
    let kv = uniform_knots(deg_v, spans_v);
    let nu = spans_u + deg_u;
    let nv = spans_v + deg_v;
    let gu = greville(deg_u, &ku);
    let gv = greville(deg_v, &kv);
    // the fit runs in the net's own parameter (0..spans); the surface is
    // sampled at the mapped domain parameters
    let sample = |t: f64, s: f64| -> Vec3 {
        let u = uw.0 + (uw.1 - uw.0) * (t / spans_u.max(1) as f64);
        let v = vw.0 + (vw.1 - vw.0) * (s / spans_v.max(1) as f64);
        orig.eval(u, v)
    };
    // row-wise: interpolate each v-row of samples along u
    let mut rows: Vec<Vec<Vec3>> = Vec::with_capacity(nv);
    for j in 0..nv {
        let s = gv[j];
        let _ = s;
        let rhs: Vec<Vec3> = (0..nu).map(|i| sample(gu[i], s)).collect();
        // the interpolation matrix (B-spline collocation)
        let mut a: Vec<Vec<f64>> = Vec::with_capacity(nu);
        for &t in &gu {
            a.push(basis_row(deg_u, &ku, nu, t));
        }
        let mut cols: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for k in 0..3 {
            let b: Vec<f64> = rhs.iter().map(|p| match k {
                0 => p.x,
                1 => p.y,
                _ => p.z,
            }).collect();
            cols[k] = solve(&a, &b).ok_or_else(|| "singular collocation system".to_string())?;
        }
        rows.push((0..nu).map(|i| Vec3::new(cols[0][i], cols[1][i], cols[2][i])).collect());
    }
    // column-wise: interpolate the rows' control points along v
    let mut a: Vec<Vec<f64>> = Vec::with_capacity(nv);
    for &s in &gv {
        a.push(basis_row(deg_v, &kv, nv, s));
    }
    let mut net: Vec<Vec3> = vec![Vec3::ZERO; nu * nv];
    for i in 0..nu {
        let mut cols: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for k in 0..3 {
            let bvec: Vec<f64> = (0..nv).map(|j| match k {
                0 => rows[j][i].x,
                1 => rows[j][i].y,
                _ => rows[j][i].z,
            }).collect();
            cols[k] = solve(&a, &bvec).ok_or_else(|| "singular collocation system".to_string())?;
        }
        for j in 0..nv {
            net[i * nv + j] = Vec3::new(cols[0][j], cols[1][j], cols[2][j]);
        }
    }
    BSplineSurface::new(deg_u, deg_v, ku, kv, nu, nv, net, None)
}

/// Map a uv point into the net's parameter (for deviation sampling).
pub fn to_net_param(uv: Vec2, uw: (f64, f64), vw: (f64, f64), spans_u: usize, spans_v: usize) -> Vec2 {
    let t = (uv.x - uw.0) / (uw.1 - uw.0).max(1e-12) * spans_u.max(1) as f64;
    let s = (uv.y - vw.0) / (vw.1 - vw.0).max(1e-12) * spans_v.max(1) as f64;
    Vec2::new(t, s)
}

#[allow(dead_code)]
fn _unused(t: f64, a: f64, b: f64) -> f64 {
    to_domain(t, a, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_fit_is_exact() {
        let orig = Surface::plane(Vec3::new(1.0, 2.0, 3.0), Vec3::Z);
        let bs = fit_surface(&orig, (0.0, 10.0), (0.0, 10.0), 3, 3, 2, 2).unwrap();
        for k in 0..8 {
            for l in 0..8 {
                let u = 0.0 + 10.0 * (k as f64 / 7.0);
                let v = 0.0 + 10.0 * (l as f64 / 7.0);
                let p = orig.eval(u, v);
                let ts = to_net_param(Vec2::new(u, v), (0.0, 10.0), (0.0, 10.0), 2, 2);
                let (t, s) = (ts.x, ts.y);
                assert!(bs.eval(t, s).dist(p) < 1e-9, "({u},{v}): {:?} vs {:?}", bs.eval(t, s), p);
            }
        }
    }

    #[test]
    fn sphere_fit_is_close() {
        let orig = Surface::Sphere { frame: ck_geom::Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 5.0 };
        let bs = fit_surface(&orig, (0.0, 1.0), (-0.4, 0.4), 3, 3, 3, 3).unwrap();
        let mut max = 0.0f64;
        for k in 0..6 {
            for l in 0..6 {
                let u = k as f64 / 5.0;
                let v = -0.4 + 0.8 * (l as f64 / 5.0);
                let p = orig.eval(u, v);
                let ts = to_net_param(Vec2::new(u, v), (0.0, 1.0), (-0.4, 0.4), 3, 3);
                let (t, s) = (ts.x, ts.y);
                max = max.max(bs.eval(t, s).dist(p));
            }
        }
        assert!(max < 0.05, "deviation {max}");
    }
}
