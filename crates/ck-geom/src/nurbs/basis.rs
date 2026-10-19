//! B-spline basis functions (Piegl & Tiller, "The NURBS Book", ch. 2).
//!
//! Evidence: the frontend carries a JS port of the same routines
//! (app.pretty.js `wp` = FindSpan with the end-of-domain walk-back past
//! repeated end knots, `xp` = DersBasisFuns A2.3), used to evaluate the
//! `{cvs, weights, degree, knots}` curves the kernel ships. We mirror those
//! conventions exactly so client and kernel agree on spans at knot values.

/// Find the knot span index `s` such that `knots[s] <= u < knots[s+1]`.
///
/// `n` = number of control points - 1, `p` = degree. At/after the end of the
/// domain (`u >= knots[n+1]`) returns the last non-degenerate span (walks
/// back over repeated knots, as app.pretty.js `wp`). Before the start
/// returns `p`.
pub fn find_span(n: usize, p: usize, u: f64, knots: &[f64]) -> usize {
    if u >= knots[n + 1] {
        let mut o = n;
        while o > p && knots[o] >= knots[n + 1] {
            o -= 1;
        }
        return o;
    }
    if u <= knots[p] {
        return p;
    }
    let (mut lo, mut hi) = (p, n + 1);
    // The answer is the one span with `knots[s] <= u < knots[s + 1]` (p < ... <= n here). Guess it from the
    // position of `u` in the domain (the knots of a fitted surface are nearly uniform), try the guess and its
    // neighbours, and bisect only inside the bracket that the failed tries leave; every step keeps
    // `knots[lo] <= u < knots[hi]`, so the result is the bisection's.
    if hi - lo > 4 {
        let (k0, k1) = (knots[p], knots[n + 1]);
        let t = (u - k0) / (k1 - k0);
        if t >= 0.0 && t < 1.0 {
            let g = (p + (t * (n + 1 - p) as f64) as usize).min(n);
            if knots[g] <= u {
                if u < knots[g + 1] {
                    return g;
                }
                lo = g + 1;
                if lo <= n && u < knots[lo + 1] {
                    return lo;
                }
                if lo + 1 <= n && knots[lo + 1] <= u {
                    lo += 1;
                    if u < knots[lo + 1] {
                        return lo;
                    }
                    lo += 1;
                }
            } else {
                hi = g;
                if hi > p && knots[hi - 1] <= u {
                    return hi - 1;
                }
                if hi - 1 > p {
                    hi -= 1;
                    if hi > p && knots[hi - 1] <= u {
                        return hi - 1;
                    }
                    hi -= 1;
                }
            }
        }
    }
    while hi - lo > 1 {
        let mid = (lo + hi) >> 1;
        if u < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo
}

/// Non-vanishing basis functions N_{span-p..=span, p}(u) (A2.2).
pub fn basis_funs(span: usize, u: f64, p: usize, knots: &[f64]) -> Vec<f64> {
    let mut n = vec![0.0; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let den = right[r + 1] + left[j - r];
            let temp = if den != 0.0 { n[r] / den } else { 0.0 };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Basis functions and their derivatives up to order `nd` (A2.3).
/// Returns `ders[k][j]` = k-th derivative of N_{span-p+j, p}(u).
/// Rows k > p are zero.
pub fn ders_basis_funs(span: usize, u: f64, p: usize, nd: usize, knots: &[f64]) -> Vec<Vec<f64>> {
    let mut ndu = vec![vec![0.0; p + 1]; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r] != 0.0 { ndu[r][j - 1] / ndu[j][r] } else { 0.0 };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = vec![vec![0.0; p + 1]; nd + 1];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let mut a = vec![vec![0.0; p + 1]; 2];
    for r in 0..=p as isize {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for k in 1..=(nd.min(p)) as isize {
            let mut d = 0.0;
            let rk = r - k;
            let pk = p as isize - k;
            if r >= k {
                let den = ndu[(pk + 1) as usize][rk as usize];
                a[s2][0] = if den != 0.0 { a[s1][0] / den } else { 0.0 };
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r - 1 <= pk { k - 1 } else { p as isize - r };
            let mut j = j1;
            while j <= j2 {
                let den = ndu[(pk + 1) as usize][(rk + j) as usize];
                a[s2][j as usize] =
                    if den != 0.0 { (a[s1][j as usize] - a[s1][(j - 1) as usize]) / den } else { 0.0 };
                d += a[s2][j as usize] * ndu[(rk + j) as usize][pk as usize];
                j += 1;
            }
            if r <= pk {
                let den = ndu[(pk + 1) as usize][r as usize];
                a[s2][k as usize] = if den != 0.0 { -a[s1][(k - 1) as usize] / den } else { 0.0 };
                d += a[s2][k as usize] * ndu[r as usize][pk as usize];
            }
            ders[k as usize][r as usize] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut f = p as f64;
    for k in 1..=nd.min(p) {
        for j in 0..=p {
            ders[k][j] *= f;
        }
        f *= (p - k) as f64;
    }
    ders
}

/// Values only of the non-vanishing basis functions (p < 12), allocation-free;
/// bit-identical to `ders_basis_funs2(.., nd = 0)[0]` (same recurrences), minus
/// the derivative tables and the 3x12 return value.
#[inline]
pub fn basis_funs2(span: usize, u: f64, p: usize, knots: &[f64]) -> [f64; 12] {
    if p == 3 {
        basis_funs2_n::<4>(span, u, 3, knots)
    } else if p == 2 {
        basis_funs2_n::<3>(span, u, 2, knots)
    } else if p < 6 {
        basis_funs2_n::<6>(span, u, p, knots)
    } else {
        basis_funs2_n::<12>(span, u, p, knots)
    }
}

#[inline(always)]
fn basis_funs2_n<const N: usize>(span: usize, u: f64, p: usize, knots: &[f64]) -> [f64; 12] {
    // ndu column p is what the derivative-free path reads; keep the same
    // (right + left) divisions as the A2.3 table so the values match bit for bit
    let mut n = [0.0f64; N];
    let (mut left, mut right) = ([0.0f64; N], [0.0f64; N]);
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let den = right[r + 1] + left[j - r];
            let temp = if den != 0.0 { n[r] / den } else { 0.0 };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    let mut out = [0.0f64; 12];
    out[..=p].copy_from_slice(&n[..=p]);
    out
}

/// Allocation-free `ders_basis_funs` for nd <= 3, p < 6 (the very arithmetic of A2.3 as written there, on
/// stack tables): `ders[k][j]` is the k-th derivative of the j-th function.
pub fn ders_basis_funs3(span: usize, u: f64, p: usize, nd: usize, knots: &[f64]) -> [[f64; 6]; 4] {
    debug_assert!(p < 6 && nd <= 3);
    let mut ndu = [[0.0f64; 6]; 6];
    let (mut left, mut right) = ([0.0f64; 6], [0.0f64; 6]);
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r] != 0.0 { ndu[r][j - 1] / ndu[j][r] } else { 0.0 };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = [[0.0f64; 6]; 4];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let mut a = [[0.0f64; 6]; 2];
    for r in 0..=p as isize {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for k in 1..=(nd.min(p)) as isize {
            let mut d = 0.0;
            let rk = r - k;
            let pk = p as isize - k;
            if r >= k {
                let den = ndu[(pk + 1) as usize][rk as usize];
                a[s2][0] = if den != 0.0 { a[s1][0] / den } else { 0.0 };
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r - 1 <= pk { k - 1 } else { p as isize - r };
            let mut j = j1;
            while j <= j2 {
                let den = ndu[(pk + 1) as usize][(rk + j) as usize];
                a[s2][j as usize] = if den != 0.0 { (a[s1][j as usize] - a[s1][(j - 1) as usize]) / den } else { 0.0 };
                d += a[s2][j as usize] * ndu[(rk + j) as usize][pk as usize];
                j += 1;
            }
            if r <= pk {
                let den = ndu[(pk + 1) as usize][r as usize];
                a[s2][k as usize] = if den != 0.0 { -a[s1][(k - 1) as usize] / den } else { 0.0 };
                d += a[s2][k as usize] * ndu[r as usize][pk as usize];
            }
            ders[k as usize][r as usize] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut f = p as f64;
    for k in 1..=nd.min(p) {
        for j in 0..=p {
            ders[k][j] *= f;
        }
        f *= (p - k) as f64;
    }
    ders
}

/// Allocation-free `ders_basis_funs` for nd <= 2, p < 12 (same arithmetic).
pub fn ders_basis_funs2(span: usize, u: f64, p: usize, nd: usize, knots: &[f64]) -> [[f64; 12]; 3] {
    // (small working arrays for the usual degrees: the zero-fill of a 12x12
    // table dominated the cost of a cubic evaluation)
    // (literal degrees: the loops unroll)
    if p == 3 {
        ders_basis_funs2_n::<4>(span, u, 3, nd, knots)
    } else if p == 2 {
        ders_basis_funs2_n::<3>(span, u, 2, nd, knots)
    } else if p < 6 {
        ders_basis_funs2_n::<6>(span, u, p, nd, knots)
    } else {
        ders_basis_funs2_n::<12>(span, u, p, nd, knots)
    }
}

#[inline(always)]
fn ders_basis_funs2_n<const N: usize>(span: usize, u: f64, p: usize, nd: usize, knots: &[f64]) -> [[f64; 12]; 3] {
    // The triangular table of A2.3: `ndu[r][j]` = N_{span-j+r, j}(u),
    // `ndu[j][r]` (j > r) = the knot difference u_{i+j} - u_i of that function.
    let mut ndu = [[0.0f64; N]; N];
    let (mut left, mut right) = ([0.0f64; N], [0.0f64; N]);
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r] != 0.0 { ndu[r][j - 1] / ndu[j][r] } else { 0.0 };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = [[0.0f64; 12]; 3];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    if nd == 0 || p == 0 {
        return ders;
    }
    // The a[][] loops of A2.3 for nd <= 2, written out: the coefficients of
    // the first order are +-1 / (u_{i+p} - u_i), those of the second order
    // their differences over the degree p-1 differences. Every quotient,
    // product and sum is the one A2.3 forms (same operands, same order), so
    // the values are bit for bit the same, at a third fewer divisions and
    // no table.
    let quot = |num: f64, den: f64| if den != 0.0 { num / den } else { 0.0 };
    // a[1][0] of row r + 1 and a[1][1] of row r
    let mut inv = [0.0f64; N];
    let mut neg = [0.0f64; N];
    for r in 0..p {
        inv[r] = quot(1.0, ndu[p][r]);
        // (-1) / x rounds to -(1 / x) exactly: one quotient per span width
        neg[r] = -inv[r];
    }
    for r in 0..=p {
        let mut d = 0.0;
        if r >= 1 {
            d = inv[r - 1] * ndu[r - 1][p - 1];
        }
        if r <= p - 1 {
            d += neg[r] * ndu[r][p - 1];
        }
        ders[1][r] = d;
    }
    let f1 = p as f64;
    if nd >= 2 && p >= 2 {
        let f2 = f1 * (p - 1) as f64;
        for r in 0..=p {
            let mut d = 0.0;
            if r >= 2 {
                let a0 = quot(inv[r - 1], ndu[p - 1][r - 2]);
                d = a0 * ndu[r - 2][p - 2];
            }
            if r >= 1 && r <= p - 1 {
                let a1 = quot(neg[r] - inv[r - 1], ndu[p - 1][r - 1]);
                d += a1 * ndu[r - 1][p - 2];
            }
            if r <= p - 2 {
                let a2 = quot(-neg[r], ndu[p - 1][r]);
                d += a2 * ndu[r][p - 2];
            }
            ders[2][r] = d * f2;
        }
    }
    for j in 0..=p {
        ders[1][j] *= f1;
    }
    ders
}

/// Single basis function N_{i,p}(u) (A2.4) — for tests / sparse use.
pub fn one_basis_fun(p: usize, knots: &[f64], i: usize, u: f64) -> f64 {
    let m = knots.len() - 1;
    if (i == 0 && u == knots[0]) || (i == m - p - 1 && u == knots[m]) {
        return 1.0;
    }
    if u < knots[i] || u >= knots[i + p + 1] {
        return 0.0;
    }
    let mut n = vec![0.0; p + 1];
    for j in 0..=p {
        n[j] = if u >= knots[i + j] && u < knots[i + j + 1] { 1.0 } else { 0.0 };
    }
    for k in 1..=p {
        let mut saved = if n[0] == 0.0 { 0.0 } else { ((u - knots[i]) * n[0]) / (knots[i + k] - knots[i]) };
        for j in 0..(p - k + 1) {
            let ul = knots[i + j + 1];
            let ur = knots[i + j + k + 1];
            if n[j + 1] == 0.0 {
                n[j] = saved;
                saved = 0.0;
            } else {
                let temp = n[j + 1] / (ur - ul);
                n[j] = saved + (ur - u) * temp;
                saved = (u - ul) * temp;
            }
        }
    }
    n[0]
}

/// Multiplicity of knot value `u` in `knots` (exact compare within `tol`).
pub fn multiplicity(knots: &[f64], u: f64, tol: f64) -> usize {
    knots.iter().filter(|&&k| (k - u).abs() <= tol).count()
}

/// Distinct knot values (the `breaks` of app.pretty.js `Mp`: `[...new Set(knots)]`).
pub fn distinct_knots(knots: &[f64]) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    for &k in knots {
        if out.last().map_or(true, |&l| l != k) {
            out.push(k);
        }
    }
    out
}

/// `distinct_knots(knots).len()` without allocating.
pub fn distinct_count(knots: &[f64]) -> usize {
    if knots.is_empty() {
        return 0;
    }
    1 + knots.windows(2).filter(|w| w[0] != w[1]).count()
}

/// Clamped uniform knot vector for `n_ctrl` control points, degree `p`, on [0,1].
pub fn clamped_uniform(n_ctrl: usize, p: usize) -> Vec<f64> {
    assert!(n_ctrl > p, "need more control points than the degree");
    let spans = n_ctrl - p;
    let mut k = vec![0.0; p + 1];
    for i in 1..spans {
        k.push(i as f64 / spans as f64);
    }
    k.extend(std::iter::repeat(1.0).take(p + 1));
    k
}

/// Validate a knot vector: length = n_ctrl + p + 1, non-decreasing,
/// interior multiplicity <= p+1. Returns an error message on failure.
/// GUESS: wording is ours — no knot-validation message found in kernel.strings.
pub fn validate(knots: &[f64], n_ctrl: usize, p: usize) -> Result<(), String> {
    if knots.len() != n_ctrl + p + 1 {
        return Err(format!("knot count {} != control points {} + degree {} + 1", knots.len(), n_ctrl, p));
    }
    if knots.windows(2).any(|w| !(w[1] >= w[0])) {
        return Err("knots must be non-decreasing".into());
    }
    if knots[p] >= knots[n_ctrl] {
        return Err("empty parameter domain".into());
    }
    Ok(())
}

/// Binomial coefficient C(n, k) as f64.
pub fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    let mut r = 1.0;
    for i in 0..k {
        r = r * (n - i) as f64 / (i + 1) as f64;
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bisection `find_span` used to be.
    fn find_span_bisect(n: usize, p: usize, u: f64, knots: &[f64]) -> usize {
        if u >= knots[n + 1] {
            let mut o = n;
            while o > p && knots[o] >= knots[n + 1] {
                o -= 1;
            }
            return o;
        }
        if u <= knots[p] {
            return p;
        }
        let (mut lo, mut hi) = (p, n + 1);
        while hi - lo > 1 {
            let mid = (lo + hi) >> 1;
            if u < knots[mid] {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        lo
    }

    /// The guessed `find_span` answers what the bisection did, on uniform, skewed and repeated knots.
    #[test]
    fn find_span_matches_the_bisection() {
        let mut seed = 99u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for case in 0..400 {
            let p = 1 + case % 5;
            let spans = 3 + (rnd() * 60.0) as usize;
            // interior knots: uniform, skewed or with repeats
            let mut inner: Vec<f64> = (1..spans).map(|i| match case % 3 { 0 => i as f64 / spans as f64, 1 => (i as f64 / spans as f64).powi(3), _ => ((rnd() * spans as f64).floor()) / spans as f64 }).collect();
            inner.sort_by(f64::total_cmp);
            let mut k = vec![0.0; p + 1];
            k.extend(inner);
            k.extend(std::iter::repeat(1.0).take(p + 1));
            let n = k.len() - p - 2;
            let mut us: Vec<f64> = (0..300).map(|_| rnd() * 1.2 - 0.1).collect();
            us.extend(k.iter().copied());
            us.extend(k.iter().map(|x| f64::from_bits(x.to_bits() + 1)));
            us.extend(k.iter().map(|x| f64::from_bits(x.to_bits().wrapping_sub(1))));
            for u in us {
                assert_eq!(find_span(n, p, u, &k), find_span_bisect(n, p, u, &k), "p {p} n {n} u {u} knots {k:?}");
            }
        }
    }


    // P&T Ex2.3: U = {0,0,0,1,2,3,4,4,5,5,5}, p=2, u=5/2
    const U: [f64; 11] = [0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.];

    #[test]
    fn span_basic_and_ends() {
        let n = U.len() - 2 - 1 - 1; // m - p - 1 = 10 - 2 - 1 = 7
        assert_eq!(n, 7);
        assert_eq!(find_span(n, 2, 2.5, &U), 4);
        assert_eq!(find_span(n, 2, 0.0, &U), 2);
        assert_eq!(find_span(n, 2, 5.0, &U), 7);
        assert_eq!(find_span(n, 2, 4.0, &U), 7);
        assert_eq!(find_span(n, 2, 3.999, &U), 5);
    }

    #[test]
    fn basis_ex23() {
        let b = basis_funs(4, 2.5, 2, &U);
        assert!((b[0] - 1.0 / 8.0).abs() < 1e-15);
        assert!((b[1] - 6.0 / 8.0).abs() < 1e-15);
        assert!((b[2] - 1.0 / 8.0).abs() < 1e-15);
    }

    #[test]
    fn partition_of_unity_and_one_basis() {
        for i in 0..=50 {
            let u = 5.0 * i as f64 / 50.0;
            let s = find_span(7, 2, u, &U);
            let b = basis_funs(s, u, 2, &U);
            assert!((b.iter().sum::<f64>() - 1.0).abs() < 1e-14);
            for j in 0..=2 {
                assert!((one_basis_fun(2, &U, s - 2 + j, u) - b[j]).abs() < 1e-13, "u={u}");
            }
        }
    }

    #[test]
    fn ders_basis_funs2_is_bit_identical_to_a23() {
        // repeated interior knots, degrees 1..7, points at and between knots
        let mut x = 0x9e3779b97f4a7c15u64;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64
        };
        for p in 1..8usize {
            for _ in 0..40 {
                let n = p + 1 + (rnd() * 6.0) as usize;
                let mut k = vec![0.0; p + 1];
                let mut v = 0.0;
                while k.len() < n + p + 1 - (p + 1) {
                    if rnd() < 0.7 {
                        v += 0.1 + rnd();
                    }
                    k.push(v);
                }
                let end = v + 0.5;
                k.extend(std::iter::repeat(end).take(p + 1));
                for i in 0..60 {
                    let u = if i % 3 == 0 { k[(rnd() * k.len() as f64) as usize] } else { rnd() * end };
                    let s = find_span(n - 1, p, u, &k);
                    let a = ders_basis_funs(s, u, p, 2, &k);
                    let b = ders_basis_funs2(s, u, p, 2, &k);
                    for d in 0..3 {
                        for j in 0..=p {
                            assert_eq!(a[d][j].to_bits(), b[d][j].to_bits(), "p={p} d={d} j={j} u={u} {:?} {} {}", k, a[d][j], b[d][j]);
                        }
                    }
                    let b1 = ders_basis_funs2(s, u, p, 1, &k);
                    for j in 0..=p {
                        assert_eq!(a[1][j].to_bits(), b1[1][j].to_bits());
                        assert_eq!(b1[2][j], 0.0);
                    }
                }
            }
        }
    }

    #[test]
    fn derivs_match_finite_difference() {
        let p = 3;
        let k = [0., 0., 0., 0., 0.3, 0.5, 0.7, 1., 1., 1., 1.];
        let n = k.len() - p - 2;
        let h = 1e-6;
        for &u in &[0.1, 0.33, 0.6, 0.85] {
            let s = find_span(n, p, u, &k);
            let d = ders_basis_funs(s, u, p, 2, &k);
            let bp = basis_funs(s, u + h, p, &k);
            let bm = basis_funs(s, u - h, p, &k);
            for j in 0..=p {
                assert!((d[0][j] - basis_funs(s, u, p, &k)[j]).abs() < 1e-15);
                let fd = (bp[j] - bm[j]) / (2.0 * h);
                assert!((d[1][j] - fd).abs() < 1e-6, "d1 u={u} j={j}");
                let fd2 = (bp[j] - 2.0 * d[0][j] + bm[j]) / (h * h);
                assert!((d[2][j] - fd2).abs() < 1e-2, "d2 u={u} j={j}");
            }
            // derivative of partition of unity is 0
            assert!(d[1].iter().sum::<f64>().abs() < 1e-12);
        }
    }

    #[test]
    fn knot_helpers() {
        assert_eq!(clamped_uniform(5, 3), vec![0., 0., 0., 0., 0.5, 1., 1., 1., 1.]);
        assert_eq!(multiplicity(&U, 4.0, 0.0), 2);
        assert_eq!(distinct_knots(&U), vec![0., 1., 2., 3., 4., 5.]);
        assert!(validate(&U, 8, 2).is_ok());
        assert!(validate(&U, 7, 2).is_err());
        assert_eq!(binomial(5, 2), 10.0);
    }
}
