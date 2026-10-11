//! Real roots of low-degree polynomials (ck-core/src/quad.rs).
//!
//! Used by curve-curve / line-quadric intersection and blend spine solving
//! (README). The original routine bodies have not been located in the WAT
//! yet, so this is a clean robust implementation, not a decompilation:
//! - quadratic: cancellation-free "citardauq" form `q = −½(b + sgn(b)√Δ)`
//! - cubic: trigonometric method (3 real roots) / Cardano (1 real root)
//! - quartic: Ferrari via the resolvent cubic
//!
//! Every root is Newton-polished against the ORIGINAL coefficients, and the
//! result is sorted ascending with near-duplicates merged. Degenerate
//! leading coefficients fall through to the lower degree.
//! GUESS: double roots are reported once (multiplicity dropped).

/// Up to four real roots, ascending. No allocation.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Roots {
    pub n: usize,
    pub r: [f64; 4],
}

impl Roots {
    pub const NONE: Roots = Roots { n: 0, r: [0.0; 4] };

    pub fn as_slice(&self) -> &[f64] {
        &self.r[..self.n]
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = f64> + '_ {
        self.as_slice().iter().copied()
    }

    fn push(&mut self, x: f64) {
        if x.is_finite() && self.n < 4 {
            self.r[self.n] = x;
            self.n += 1;
        }
    }

    /// Sort ascending and merge roots closer than `tol` (relative to scale).
    fn finish(mut self) -> Roots {
        let s = &mut self.r[..self.n];
        s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut out = Roots::NONE;
        for &x in s.iter() {
            if out.n > 0 {
                let p = out.r[out.n - 1];
                if (x - p).abs() <= 1e-10 * (1.0f64).max(x.abs()).max(p.abs()) {
                    continue;
                }
            }
            out.push(x);
        }
        out
    }
}

/// Horner evaluation; `c[0]` is the highest-degree coefficient.
pub fn poly_eval(c: &[f64], x: f64) -> f64 {
    c.iter().fold(0.0, |acc, &k| acc * x + k)
}

/// Value and derivative (Horner).
pub fn poly_eval_d(c: &[f64], x: f64) -> (f64, f64) {
    let mut p = 0.0;
    let mut d = 0.0;
    for &k in c {
        d = d * x + p;
        p = p * x + k;
    }
    (p, d)
}

/// A few Newton steps on `c` from `x`; keeps the best iterate (never makes
/// |p(x)| worse).
pub fn newton_polish(c: &[f64], x: f64, iters: usize) -> f64 {
    let mut best = x;
    let mut bv = poly_eval(c, x).abs();
    let mut x = x;
    for _ in 0..iters {
        let (p, d) = poly_eval_d(c, x);
        if p == 0.0 || d == 0.0 || !d.is_finite() {
            break;
        }
        let nx = x - p / d;
        let nv = poly_eval(c, nx).abs();
        if !(nv < bv) {
            break;
        }
        best = nx;
        bv = nv;
        x = nx;
    }
    best
}

/// Coefficient considered zero relative to the others.
fn negligible(a: f64, rest: &[f64]) -> bool {
    let m = rest.iter().fold(0.0f64, |m, &v| m.max(v.abs()));
    a == 0.0 || a.abs() <= 1e-14 * m
}

/// Real roots of `a x + b`.
pub fn linear(a: f64, b: f64) -> Roots {
    let mut r = Roots::NONE;
    if a != 0.0 {
        r.push(-b / a);
    }
    r
}

/// Real roots of `a x² + b x + c`.
pub fn quadratic(a: f64, b: f64, c: f64) -> Roots {
    if negligible(a, &[b, c]) {
        return linear(b, c);
    }
    let mut r = Roots::NONE;
    let disc = b * b - 4.0 * a * c;
    // treat tiny negative discriminant (rounding) as a double root
    let scale = (b * b).max((4.0 * a * c).abs());
    if disc < 0.0 {
        if disc >= -1e-14 * scale {
            r.push(-b / (2.0 * a));
        }
        return r;
    }
    if disc == 0.0 {
        r.push(-b / (2.0 * a));
        return r;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (b + if b >= 0.0 { sq } else { -sq });
    r.push(q / a);
    if q != 0.0 {
        r.push(c / q);
    } else {
        r.push(-q / a);
    }
    let co = [a, b, c];
    for i in 0..r.n {
        r.r[i] = newton_polish(&co, r.r[i], 3);
    }
    r.finish()
}

/// Real roots of `a x³ + b x² + c x + d`.
pub fn cubic(a: f64, b: f64, c: f64, d: f64) -> Roots {
    if negligible(a, &[b, c, d]) {
        return quadratic(b, c, d);
    }
    let mut r = Roots::NONE;
    if d == 0.0 {
        // x (a x² + b x + c)
        r.push(0.0);
        for x in quadratic(a, b, c).iter() {
            r.push(x);
        }
        return r.finish();
    }
    let (b, c, d0) = (b / a, c / a, d / a);
    // depressed: x = t − b/3, t³ + p t + q = 0
    let b3 = b / 3.0;
    let p = c - b * b3;
    let q = 2.0 * b3 * b3 * b3 - b3 * c + d0;
    let h = (q * 0.5) * (q * 0.5) + (p / 3.0) * (p / 3.0) * (p / 3.0);
    let scale = ((q * 0.5) * (q * 0.5)).max(((p / 3.0).powi(3)).abs()).max(f64::MIN_POSITIVE);
    if h.abs() <= 1e-14 * scale {
        // repeated root
        if p.abs() <= 1e-14 * (1.0f64).max(b.abs()).max(c.abs()) {
            r.push(-b3);
        } else {
            let u = (-q * 0.5).cbrt();
            r.push(2.0 * u - b3);
            r.push(-u - b3);
        }
    } else if h > 0.0 {
        let sh = h.sqrt();
        let u = (-q * 0.5 + if q <= 0.0 { sh } else { -sh }).cbrt();
        let v = if u != 0.0 { -p / (3.0 * u) } else { 0.0 };
        r.push(u + v - b3);
    } else {
        // three real roots
        let m = 2.0 * (-p / 3.0).sqrt();
        let arg = (3.0 * q / (p * m)).clamp(-1.0, 1.0);
        let th = arg.acos() / 3.0;
        for k in 0..3 {
            r.push(m * (th - 2.0 * std::f64::consts::PI * k as f64 / 3.0).cos() - b3);
        }
    }
    let co = [1.0, b, c, d0];
    for i in 0..r.n {
        r.r[i] = newton_polish(&co, r.r[i], 4);
    }
    r.finish()
}

/// Real roots of `a x⁴ + b x³ + c x² + d x + e`.
pub fn quartic(a: f64, b: f64, c: f64, d: f64, e: f64) -> Roots {
    if negligible(a, &[b, c, d, e]) {
        return cubic(b, c, d, e);
    }
    let mut r = Roots::NONE;
    if e == 0.0 {
        r.push(0.0);
        for x in cubic(a, b, c, d).iter() {
            r.push(x);
        }
        return r.finish();
    }
    let (b, c, d, e) = (b / a, c / a, d / a, e / a);
    // depressed: x = y − b/4 → y⁴ + p y² + q y + s = 0
    let b4 = b * 0.25;
    let bb = b4 * b4;
    let p = c - 6.0 * bb;
    let q = d - 2.0 * c * b4 + 8.0 * bb * b4;
    let s = e - d * b4 + c * bb - 3.0 * bb * bb;
    let sc = (1.0f64).max(p.abs()).max(s.abs().sqrt());
    if q.abs() <= 1e-14 * sc * sc * sc.sqrt() {
        // biquadratic in y²
        for z in quadratic(1.0, p, s).iter() {
            if z > 0.0 {
                let w = z.sqrt();
                r.push(w - b4);
                r.push(-w - b4);
            } else if z >= -1e-14 * sc {
                r.push(-b4);
            }
        }
    } else {
        // resolvent: m³ + 2p m² + (p² − 4s) m − q² = 0, need m > 0
        let res = cubic(1.0, 2.0 * p, p * p - 4.0 * s, -q * q);
        let m = res.iter().fold(f64::NEG_INFINITY, f64::max);
        if m > 0.0 {
            let sm = m.sqrt();
            let k = q / (2.0 * sm);
            let h = 0.5 * (p + m);
            for y in quadratic(1.0, sm, h - k).iter() {
                r.push(y - b4);
            }
            for y in quadratic(1.0, -sm, h + k).iter() {
                r.push(y - b4);
            }
        }
    }
    let co = [1.0, b, c, d, e];
    for i in 0..r.n {
        r.r[i] = newton_polish(&co, r.r[i], 6);
    }
    let r = r.finish();
    // Ferrari loses roots when the depressed form cancels badly (large
    // |b/a|); cross-check with sign-based isolation and keep the richer set.
    let iso = isolate(&co);
    if iso.n > r.n {
        iso
    } else {
        r
    }
}

/// Real roots by derivative isolation: the roots of `p'` split the line
/// into monotone pieces, each holding at most one root of `p`, which is
/// then found by safeguarded bisection/Newton. Critical points where `p`
/// (nearly) vanishes are reported as (even-multiplicity) roots. Degree ≤ 4,
/// `c[0]` = leading coefficient (non-zero).
pub fn isolate(c: &[f64]) -> Roots {
    let n = c.len() - 1;
    let mut out = Roots::NONE;
    if n == 0 || c[0] == 0.0 {
        return out;
    }
    if n == 1 {
        return linear(c[0], c[1]);
    }
    // derivative coefficients
    let mut dc = [0.0; 4];
    for (i, k) in c[..n].iter().enumerate() {
        dc[i] = k * (n - i) as f64;
    }
    let crit = isolate(&dc[..n]);
    // Cauchy bound on |roots|
    let bound = 1.0 + c[1..].iter().fold(0.0f64, |m, &k| m.max((k / c[0]).abs()));
    let mut pts = [0.0; 6];
    let mut np = 0;
    pts[np] = -bound;
    np += 1;
    for x in crit.iter() {
        pts[np] = x.clamp(-bound, bound);
        np += 1;
    }
    pts[np] = bound;
    np += 1;
    let mag = |x: f64| -> f64 { c.iter().enumerate().map(|(i, k)| k.abs() * x.abs().powi((n - i) as i32)).sum() };
    for w in 0..np - 1 {
        let (mut lo, mut hi) = (pts[w], pts[w + 1]);
        let (mut flo, fhi) = (poly_eval(c, lo), poly_eval(c, hi));
        if flo == 0.0 {
            out.push(lo);
            continue;
        }
        if flo.signum() == fhi.signum() {
            continue;
        }
        let mut x = 0.5 * (lo + hi);
        for _ in 0..200 {
            let (p, d) = poly_eval_d(c, x);
            if p == 0.0 {
                break;
            }
            if p.signum() == flo.signum() {
                lo = x;
                flo = p;
            } else {
                hi = x;
            }
            let nx = if d != 0.0 { x - p / d } else { f64::NAN };
            let nx = if nx > lo && nx < hi { nx } else { 0.5 * (lo + hi) };
            if nx == x || (hi - lo) <= 4.0 * f64::EPSILON * x.abs().max(f64::MIN_POSITIVE) {
                x = nx;
                break;
            }
            x = nx;
        }
        out.push(x);
    }
    if pts[np - 1] != 0.0 && poly_eval(c, pts[np - 1]) == 0.0 {
        out.push(pts[np - 1]);
    }
    // touching roots at critical points
    for x in crit.iter() {
        if poly_eval(c, x).abs() <= 1e-12 * mag(x).max(f64::MIN_POSITIVE) {
            out.push(x);
        }
    }
    out.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check(r: Roots, want: &[f64], tol: f64) {
        assert_eq!(r.n, want.len(), "got {:?} want {:?}", r.as_slice(), want);
        for (a, b) in r.iter().zip(want) {
            assert!((a - b).abs() <= tol * (1.0f64).max(b.abs()), "got {:?} want {:?}", r.as_slice(), want);
        }
    }

    #[test]
    fn quadratics() {
        check(quadratic(1.0, -3.0, 2.0), &[1.0, 2.0], 1e-15);
        check(quadratic(1.0, 2.0, 1.0), &[-1.0], 1e-15);
        check(quadratic(1.0, 0.0, 1.0), &[], 0.0);
        check(quadratic(0.0, 2.0, -4.0), &[2.0], 0.0);
        check(quadratic(0.0, 0.0, 1.0), &[], 0.0);
        // catastrophic-cancellation case: roots 1e-8 and 1e8
        check(quadratic(1.0, -(1e8 + 1e-8), 1.0), &[1e-8, 1e8], 1e-14);
    }

    #[test]
    fn cubics() {
        // (x-1)(x-2)(x-3)
        check(cubic(1.0, -6.0, 11.0, -6.0), &[1.0, 2.0, 3.0], 1e-13);
        // x³ - 1 → 1
        check(cubic(1.0, 0.0, 0.0, -1.0), &[1.0], 1e-15);
        // (x-1)²(x+2) = x³ - 3x + 2
        check(cubic(1.0, 0.0, -3.0, 2.0), &[-2.0, 1.0], 1e-7);
        // triple root (x-2)³
        check(cubic(1.0, -6.0, 12.0, -8.0), &[2.0], 1e-5);
        check(cubic(2.0, -4.0, 0.0, 0.0), &[0.0, 2.0], 1e-15);
        check(cubic(0.0, 1.0, -3.0, 2.0), &[1.0, 2.0], 1e-15);
    }

    #[test]
    fn quartics() {
        // (x-1)(x-2)(x-3)(x-4) = x⁴ -10x³ +35x² -50x +24
        check(quartic(1.0, -10.0, 35.0, -50.0, 24.0), &[1.0, 2.0, 3.0, 4.0], 1e-12);
        // x⁴ - 5x² + 4 (biquadratic) → ±1, ±2
        check(quartic(1.0, 0.0, -5.0, 0.0, 4.0), &[-2.0, -1.0, 1.0, 2.0], 1e-14);
        // x⁴ + 1 → none
        check(quartic(1.0, 0.0, 0.0, 0.0, 1.0), &[], 0.0);
        // (x²+1)(x-3)(x+0.5) = x⁴ -2.5x³ -0.5x² -2.5x -1.5
        check(quartic(1.0, -2.5, -0.5, -2.5, -1.5), &[-0.5, 3.0], 1e-12);
        // scaled leading coefficient
        check(quartic(3.0, -30.0, 105.0, -150.0, 72.0), &[1.0, 2.0, 3.0, 4.0], 1e-12);
        check(quartic(1.0, -1.0, 0.0, 0.0, 0.0), &[0.0, 1.0], 1e-15);
    }

    #[test]
    fn residuals_random() {
        // deterministic LCG; every reported root must satisfy p(x)≈0
        let mut s: u64 = 0x9e3779b97f4a7c15;
        let mut rnd = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64 / (1u64 << 53) as f64) * 20.0 - 10.0
        };
        for _ in 0..150 {
            let c = [rnd(), rnd(), rnd(), rnd(), rnd()];
            let rr = quartic(c[0], c[1], c[2], c[3], c[4]);
            for x in rr.iter() {
                let (p, d) = poly_eval_d(&c, x);
                let mag: f64 = c.iter().enumerate().map(|(i, k)| k.abs() * x.abs().powi(4 - i as i32)).sum();
                assert!(p.abs() <= 1e-9 * mag.max(1.0) || p.abs() <= 1e-6 * d.abs(), "{c:?} x={x} p={p}");
            }
            // count sign changes on a fine grid ⇒ lower bound on simple roots
            let mut changes = 0;
            let mut prev = poly_eval(&c, -1e3);
            let mut x = -1e3;
            while x < 1e3 {
                x += 0.02;
                let v = poly_eval(&c, x);
                if v.signum() != prev.signum() && v != 0.0 {
                    changes += 1;
                }
                prev = v;
            }
            assert!(rr.n >= changes.min(4), "{c:?} roots={:?} changes={changes}", rr.as_slice());
        }
    }
}
