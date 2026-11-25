// SPDX-License-Identifier: MIT
//! Curves known by points at parameters (helices: [`Curve::Interpolated`])
//! evaluated on the piecewise cubic through the points, with each point's
//! tangent from the parabola through it and its neighbours (mitcad#161).
//! The curve the builder makes interpolates the same points; between them
//! the straight segments of [`Curve::eval`] are up to a tenth of a
//! millimetre off a thread's helix, and their length a thousandth short,
//! too far for the import to find a helical edge by its middle and length.
//!
//! [`Curve::Interpolated`]: ck_asm::brep::Curve::Interpolated
//! [`Curve::eval`]: ck_asm::brep::Curve::eval

use ck_asm::brep::{self, P3};

/// The derivative at node `i` of the polynomial through it and its
/// neighbours, two on each side (the first or last five at the ends; as
/// many as there are when fewer).
fn tangent(params: &[f64], points: &[P3], i: usize) -> P3 {
    let n = params.len();
    let width = n.min(5);
    let first = i.saturating_sub(width / 2).min(n - width);
    let nodes = &params[first..first + width];
    let x = params[i];
    let mut out = [0.0; 3];
    for (k, &xk) in nodes.iter().enumerate() {
        // The derivative of the k-th Lagrange basis polynomial at x.
        let mut d = 0.0;
        for (m, &xm) in nodes.iter().enumerate() {
            if m == k {
                continue;
            }
            let mut term = 1.0 / (xk - xm);
            for (l, &xl) in nodes.iter().enumerate() {
                if l != k && l != m {
                    term *= (x - xl) / (xk - xl);
                }
            }
            d += term;
        }
        out = brep::add(out, brep::scale(points[first + k], d));
    }
    out
}

/// The segment `t` lies in (clamped to the curve's).
fn segment(params: &[f64], t: f64) -> usize {
    params
        .partition_point(|&p| p <= t)
        .clamp(1, params.len() - 1)
        - 1
}

/// The point at `t` and the derivative there (`params` increasing, at
/// least three).
pub fn point(params: &[f64], points: &[P3], t: f64) -> (P3, P3) {
    let i = segment(params, t);
    let (t0, t1) = (params[i], params[i + 1]);
    let h = t1 - t0;
    if h <= 0.0 {
        return (points[i], tangent(params, points, i));
    }
    let s = (t - t0) / h;
    let (p0, p1) = (points[i], points[i + 1]);
    let (m0, m1) = (
        brep::scale(tangent(params, points, i), h),
        brep::scale(tangent(params, points, i + 1), h),
    );
    let (s2, s3) = (s * s, s * s * s);
    let p = brep::add(
        brep::add(
            brep::scale(p0, 2.0 * s3 - 3.0 * s2 + 1.0),
            brep::scale(m0, s3 - 2.0 * s2 + s),
        ),
        brep::add(
            brep::scale(p1, -2.0 * s3 + 3.0 * s2),
            brep::scale(m1, s3 - s2),
        ),
    );
    let d = brep::add(
        brep::add(
            brep::scale(p0, 6.0 * s2 - 6.0 * s),
            brep::scale(m0, 3.0 * s2 - 4.0 * s + 1.0),
        ),
        brep::add(
            brep::scale(p1, -6.0 * s2 + 6.0 * s),
            brep::scale(m1, 3.0 * s2 - 2.0 * s),
        ),
    );
    (p, brep::scale(d, 1.0 / h))
}

/// The length from `from` to `to` (five-point Gauss rule per segment).
pub fn length(params: &[f64], points: &[P3], from: f64, to: f64) -> f64 {
    const X: [f64; 5] = [
        0.0,
        -0.538_469_310_105_683_1,
        0.538_469_310_105_683_1,
        -0.906_179_845_938_664,
        0.906_179_845_938_664,
    ];
    const W: [f64; 5] = [
        0.568_888_888_888_888_9,
        0.478_628_670_499_366_5,
        0.478_628_670_499_366_5,
        0.236_926_885_056_189_1,
        0.236_926_885_056_189_1,
    ];
    let (a, b) = (from.min(to), from.max(to));
    let mut cuts = vec![a];
    cuts.extend(params.iter().copied().filter(|&p| p > a && p < b));
    cuts.push(b);
    cuts.windows(2)
        .map(|w| {
            let (mid, half) = (0.5 * (w[0] + w[1]), 0.5 * (w[1] - w[0]));
            X.iter()
                .zip(W)
                .map(|(x, w)| w * brep::norm(point(params, points, mid + half * x).1))
                .sum::<f64>()
                * half
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A helix sampled 32 times a turn: the cubic's points lie within a
    /// micrometre of it between the samples, and its length within 1e-5
    /// of the helix' (the straight segments: 0.1 mm off, 1.6e-3 short).
    #[test]
    fn a_sampled_helix_keeps_its_shape_and_length() {
        let (r, pitch) = (23.8, 5.0);
        let helix = |t: f64| [r * t.cos(), r * t.sin(), pitch * t / std::f64::consts::TAU];
        let turns = 6.0;
        let n = 192;
        let end = turns * std::f64::consts::TAU;
        let params: Vec<f64> = (0..=n).map(|i| end * i as f64 / n as f64).collect();
        let points: Vec<P3> = params.iter().map(|&t| helix(t)).collect();
        for k in 0..50 {
            let t = end * (k as f64 + 0.37) / 50.0;
            let (p, d) = point(&params, &points, t);
            assert!(brep::dist(p, helix(t)) < 1e-3, "{t}: {p:?}");
            let exact = [-r * t.sin(), r * t.cos(), pitch / std::f64::consts::TAU];
            assert!(brep::dist(d, exact) < 1e-2 * brep::norm(exact), "{d:?}");
        }
        let exact = turns * (std::f64::consts::TAU * r).hypot(pitch);
        let got = length(&params, &points, 0.0, end);
        assert!((got - exact).abs() < 1e-5 * exact, "{got} {exact}");
    }
}
