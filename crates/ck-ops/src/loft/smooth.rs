//! Smoothing of the loft between its stations (ck-ops/src/loft/smooth.rs).
//!
//! Oracle probes (`loft` of 2 / 3 profiles): a rail through the matching
//! points of the stations is the NATURAL cubic spline (zero second
//! derivative at both ends) through them, parameterised by the cumulative
//! chord length between the station centroids (`stations::params`); two
//! stations give the straight line. Reproduced to ~1e-14 on a three-square
//! loft (`y(t)` of a rail: the dyadic rational coefficients of the natural
//! spline through (0, 30, 50)).
//!
//! The spline is returned as a clamped B-spline (degree 3, simple interior
//! knots at the stations, `K + 2` control points) so that the same net
//! interpolation can be run on homogeneous control points of the profile
//! curves to give the NURBS side faces.

use ck_geom::nurbs::basis::{basis_funs, ders_basis_funs, find_span};
use std::cell::Cell;

thread_local! {
    static LOCAL: Cell<bool> = const { Cell::new(false) };
}

/// Run `f` with [`through`] giving the shape-preserving local interpolation ([`through_local`]) instead of the natural
/// spline. The loft sets this for one whole build, so that every rail and every side face of it is interpolated the
/// same way (a rail is the edge of its side faces).
pub fn with_local<R>(on: bool, f: impl FnOnce() -> R) -> R {
    let prev = LOCAL.with(|c| c.replace(on));
    let r = f();
    LOCAL.with(|c| c.set(prev));
    r
}

/// Shape-preserving slopes (Fritsch-Carlson, non-uniform spacing): a channel never leaves the range of its station values,
/// and has a zero slope at a station that is a local extreme of it.
fn pchip_slopes(ts: &[f64], ys: &[f64]) -> Vec<f64> {
    let k = ts.len();
    let h: Vec<f64> = (0..k - 1).map(|i| ts[i + 1] - ts[i]).collect();
    let d: Vec<f64> = (0..k - 1).map(|i| (ys[i + 1] - ys[i]) / h[i]).collect();
    let mut m = vec![0.0; k];
    for i in 1..k - 1 {
        if d[i - 1] * d[i] > 0.0 {
            let (w1, w2) = (2.0 * h[i] + h[i - 1], h[i] + 2.0 * h[i - 1]);
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
    }
    let end = |h0: f64, h1: f64, d0: f64, d1: f64| {
        let m = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
        if m * d0 <= 0.0 {
            0.0
        } else if d0 * d1 <= 0.0 && m.abs() > 3.0 * d0.abs() {
            3.0 * d0
        } else {
            m
        }
    };
    if k == 2 {
        m[0] = d[0];
        m[1] = d[0];
    } else {
        m[0] = end(h[0], h[1], d[0], d[1]);
        m[k - 1] = end(h[k - 2], h[k - 3], d[k - 2], d[k - 3]);
    }
    m
}

/// The shape-preserving local interpolation through `pts[k]` at `ts[k]` (three or more stations): per channel a C1 piecewise
/// cubic Hermite with [`pchip_slopes`], as a clamped cubic B-spline with double interior knots (`2K` control points). Unlike
/// the natural spline, a station only reaches its two neighbours, and a value never overshoots the stations' range.
pub fn through_local<const N: usize>(ts: &[f64], pts: &[[f64; N]]) -> Result<(usize, Vec<f64>, Vec<[f64; N]>), String> {
    let k = ts.len();
    if k < 3 || pts.len() != k {
        return Err("local loft interpolation needs at least three stations".into());
    }
    let mut knots = vec![ts[0]; 4];
    for t in &ts[1..k - 1] {
        knots.push(*t);
        knots.push(*t);
    }
    knots.extend(std::iter::repeat(ts[k - 1]).take(4));
    let mut ctrl = vec![[0.0; N]; 2 * k];
    for ch in 0..N {
        let ys: Vec<f64> = pts.iter().map(|p| p[ch]).collect();
        let m = pchip_slopes(ts, &ys);
        ctrl[0][ch] = ys[0];
        ctrl[2 * k - 1][ch] = ys[k - 1];
        for i in 0..k - 1 {
            let h = ts[i + 1] - ts[i];
            ctrl[2 * i + 1][ch] = ys[i] + m[i] * h / 3.0;
            ctrl[2 * i + 2][ch] = ys[i + 1] - m[i + 1] * h / 3.0;
        }
    }
    Ok((3, knots, ctrl))
}

/// Dense solve of `A X = B` (partial pivoting), `B` with `N` channels.
fn solve<const N: usize>(mut a: Vec<Vec<f64>>, mut b: Vec<[f64; N]>) -> Result<Vec<[f64; N]>, String> {
    let n = b.len();
    for c in 0..n {
        let mut piv = c;
        for i in c + 1..n {
            if a[i][c].abs() > a[piv][c].abs() {
                piv = i;
            }
        }
        if a[piv][c].abs() < 1e-300 {
            return Err("singular loft interpolation system".into());
        }
        a.swap(c, piv);
        b.swap(c, piv);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                for ch in 0..N {
                    b[r][ch] -= f * b[c][ch];
                }
            }
        }
    }
    let mut x = vec![[0.0; N]; n];
    for r in (0..n).rev() {
        let mut s = b[r];
        for k in r + 1..n {
            for ch in 0..N {
                s[ch] -= a[r][k] * x[k][ch];
            }
        }
        for ch in 0..N {
            x[r][ch] = s[ch] / a[r][r];
        }
    }
    Ok(x)
}

/// The loft interpolation through `pts[k]` at parameter `ts[k]`: returns
/// `(degree, knots, ctrl)`. Two stations -> degree 1; more -> natural cubic.
pub fn through<const N: usize>(ts: &[f64], pts: &[[f64; N]]) -> Result<(usize, Vec<f64>, Vec<[f64; N]>), String> {
    let k = ts.len();
    if k < 2 || pts.len() != k {
        return Err("loft needs at least two stations".into());
    }
    if k == 2 {
        return Ok((1, vec![ts[0], ts[0], ts[1], ts[1]], vec![pts[0], pts[1]]));
    }
    if LOCAL.with(|c| c.get()) {
        return through_local(ts, pts);
    }
    let p = 3;
    let n = k + 2;
    let mut knots = vec![ts[0]; 4];
    knots.extend_from_slice(&ts[1..k - 1]);
    knots.extend(std::iter::repeat(ts[k - 1]).take(4));
    let mut a = vec![vec![0.0; n]; n];
    let mut rhs = vec![[0.0; N]; n];
    for (r, (&t, q)) in ts.iter().zip(pts).enumerate() {
        let s = find_span(n - 1, p, t, &knots);
        let b = basis_funs(s, t, p, &knots);
        for j in 0..=p {
            a[r][s - p + j] = b[j];
        }
        rhs[r] = *q;
    }
    for (r, t) in [(k, ts[0]), (k + 1, ts[k - 1])] {
        let s = find_span(n - 1, p, t, &knots);
        let d = ders_basis_funs(s, t, p, 2, &knots);
        for j in 0..=p {
            a[r][s - p + j] = d[2][j];
        }
    }
    let ctrl = solve(a, rhs)?;
    Ok((p, knots, ctrl))
}

/// [`through`] with optional first-derivative end conditions: where `d0` /
/// `d1` is given the derivative at the first / last station replaces the
/// natural (zero second derivative) condition. Two stations with a
/// derivative give a cubic. Oracle (`startNormal` / `endNormal` probes, a
/// square-to-square loft): the rail leaves the profile along its unit normal
/// times the tension, with the natural condition at the other end; the volumes
/// 8207.142857 / 5957.142857 are the closed form of that cubic.
pub fn through_ends<const N: usize>(
    ts: &[f64],
    pts: &[[f64; N]],
    d0: Option<[f64; N]>,
    d1: Option<[f64; N]>,
) -> Result<(usize, Vec<f64>, Vec<[f64; N]>), String> {
    if d0.is_none() && d1.is_none() {
        return through(ts, pts);
    }
    let k = ts.len();
    if k < 2 || pts.len() != k {
        return Err("loft needs at least two stations".into());
    }
    let p = 3;
    let n = k + 2;
    let mut knots = vec![ts[0]; 4];
    knots.extend_from_slice(&ts[1..k - 1]);
    knots.extend(std::iter::repeat(ts[k - 1]).take(4));
    let mut a = vec![vec![0.0; n]; n];
    let mut rhs = vec![[0.0; N]; n];
    for (r, (&t, q)) in ts.iter().zip(pts).enumerate() {
        let s = find_span(n - 1, p, t, &knots);
        let b = basis_funs(s, t, p, &knots);
        for j in 0..=p {
            a[r][s - p + j] = b[j];
        }
        rhs[r] = *q;
    }
    for (r, t, d) in [(k, ts[0], d0), (k + 1, ts[k - 1], d1)] {
        let s = find_span(n - 1, p, t, &knots);
        let ders = ders_basis_funs(s, t, p, 2, &knots);
        let order = if d.is_some() { 1 } else { 2 };
        for j in 0..=p {
            a[r][s - p + j] = ders[order][j];
        }
        if let Some(d) = d {
            rhs[r] = d;
        }
    }
    let ctrl = solve(a, rhs)?;
    Ok((p, knots, ctrl))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::nurbs::bspline::BSplineCurve;
    use ck_geom::Vec3;

    /// Oracle: natural spline through (0, 30, 50) at t = (0, 30, 50) is
    /// linear; through x = (10, 5, 8) the rail value at y = 7.5 is
    /// 8.08203125 (three-square loft probe).
    /// startNormal: x(0)=10, x'(0)=0 with y linear: x(y) = 10 + a y^2 + b y^3 (natural at 30).
    #[test]
    fn clamped_start_matches_the_closed_form() {
        let ts = [0.0, 30.0];
        let pts: Vec<[f64; 2]> = vec![[10.0, 0.0], [5.0, 30.0]];
        let (p, k, c) = through_ends(&ts, &pts, Some([0.0, 1.0]), None).unwrap();
        let cur = ck_geom::nurbs::bspline::BSplineCurve::new(p, k, c.iter().map(|q| Vec3::new(q[0], q[1], 0.0)).collect()).unwrap();
        let b = 5.0 / 54000.0;
        let a = -90.0 * b;
        for y in [3.0, 12.0, 25.0] {
            let q = cur.eval(y);
            assert!((q.y - y).abs() < 1e-9 && (q.x - (10.0 + a * y * y + b * y * y * y)).abs() < 1e-9, "{q:?}");
        }
    }

    #[test]
    fn natural_three_station_rail() {
        let ts = [0.0, 30.0, 50.0];
        let pts: Vec<[f64; 3]> = vec![[10.0, 0.0, 10.0], [5.0, 30.0, 5.0], [8.0, 50.0, 4.0]];
        let (p, k, c) = through(&ts, &pts).unwrap();
        let cur = BSplineCurve::new(p, k, c.iter().map(|q| Vec3::new(q[0], q[1], q[2])).collect()).unwrap();
        let q = cur.eval(7.5);
        assert!((q.x - 8.08203125).abs() < 1e-12, "{q:?}");
        assert!((q.y - 7.5).abs() < 1e-12);
        let q = cur.eval(25.0);
        assert!((q.x - 5.10763888888889).abs() < 1e-12, "{q:?}");
        assert!((q.z - 5.565972222222222).abs() < 1e-12);
    }

    /// The local rails never leave the range of their stations and are flat at a station that is a local extreme; the natural
    /// spline through the same bottle section widths (38, 47.5, 20, 15 over 0, 114, 152, 241) bulges past 47.5.
    #[test]
    fn local_rails_do_not_overshoot() {
        let ts = [0.0, 114.0, 152.0, 241.0];
        let ys: Vec<[f64; 1]> = vec![[38.0], [47.5], [20.0], [15.0]];
        let nat = through(&ts, &ys).unwrap();
        let loc = through_local(&ts, &ys).unwrap();
        let eval = |(p, k, c): &(usize, Vec<f64>, Vec<[f64; 1]>), t: f64| {
            BSplineCurve::new(*p, k.clone(), c.iter().map(|q| Vec3::new(q[0], 0.0, 0.0)).collect()).unwrap().eval(t).x
        };
        let (mut nmax, mut lmax, mut lmin) = (0.0f64, 0.0f64, f64::INFINITY);
        for i in 0..=480 {
            let t = 241.0 * i as f64 / 480.0;
            nmax = nmax.max(eval(&nat, t));
            lmax = lmax.max(eval(&loc, t));
            lmin = lmin.min(eval(&loc, t));
        }
        assert!(nmax > 50.0, "natural {nmax}");
        assert!(lmax <= 47.5 + 1e-9 && lmin >= 15.0 - 1e-9, "local {lmin}..{lmax}");
        for (t, y) in ts.iter().zip([38.0, 47.5, 20.0, 15.0]) {
            assert!((eval(&loc, *t) - y).abs() < 1e-9);
        }
        assert_eq!(loc.2.len(), 2 * ts.len());
    }
}
