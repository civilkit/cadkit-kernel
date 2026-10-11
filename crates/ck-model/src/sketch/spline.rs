//! Interpolating sketch spline (`{"type":"spline","pts":[..],"closed":?}`).
//!
//! Port of the client's `lt` class (app.pretty.js l.1237): a C2 cubic
//! spline through the points, parametrised by chord length, with natural
//! end conditions (open) or periodic conditions (closed). Moments are solved
//! with the client's dense Gauss-Jordan `fd()`.
//!
//! Parameter `s` runs over `[0, length]` (sum of chord lengths).
//!
//! Also: the control-point spline (`cvspline`) evaluation used by the sketch
//! solver — the client's `ze` (definition validation + default knots) and
//! `Se`/`ei`/`go`/`ul`/`ho`/`fl` (derivative evaluation), dimension-generic
//! so the 2D solver and the 3D solver share it.

/// Dense Gauss-Jordan with partial pivoting (client `fd()`); zero pivots
/// yield 0 for that unknown.
fn gauss_jordan(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let t = b.len();
    let mut n: Vec<Vec<f64>> = a.iter().zip(b).map(|(r, &v)| {
        let mut r = r.clone();
        r.push(v);
        r
    }).collect();
    for s in 0..t {
        let mut i = s;
        for a in s + 1..t {
            if n[a][s].abs() > n[i][s].abs() {
                i = a;
            }
        }
        n.swap(s, i);
        let o = n[s][s];
        if o.abs() < 1e-300 {
            continue;
        }
        for a in 0..t {
            if a == s {
                continue;
            }
            let l = n[a][s] / o;
            if l != 0.0 {
                for c in s..=t {
                    n[a][c] -= l * n[s][c];
                }
            }
        }
    }
    (0..t).map(|i| if n[i][i].abs() > 1e-300 { n[i][t] / n[i][i] } else { 0.0 }).collect()
}

#[derive(Debug, Clone)]
pub struct InterpSpline {
    pub p: Vec<[f64; 2]>,
    pub closed: bool,
    /// Chord lengths per span.
    pub h: Vec<f64>,
    pub knots: Vec<f64>,
    /// Second-derivative moments per coordinate.
    pub m: [Vec<f64>; 2],
}

impl InterpSpline {
    /// Build from points. Returns `None` for fewer than 2 distinct points
    /// (client `Xi()` drops consecutive duplicates closer than 1e-9).
    pub fn new(pts: &[[f64; 2]], closed: bool) -> Option<InterpSpline> {
        let mut n: Vec<[f64; 2]> = Vec::new();
        for (i, p) in pts.iter().enumerate() {
            if i == 0 || {
                let q = n[n.len() - 1];
                let dx = p[0] - q[0];
                let dy = p[1] - q[1];
                (dx * dx + dy * dy).sqrt() > 1e-9
            } {
                n.push(*p);
            }
        }
        if n.len() < 2 || (closed && n.len() < 3) {
            return None;
        }
        if closed && n.len() > 2 {
            let (f, l) = (n[0], n[n.len() - 1]);
            if {
                let dx = f[0] - l[0];
                let dy = f[1] - l[1];
                (dx * dx + dy * dy).sqrt() < 1e-12
            } {
                n.pop();
            }
        }
        let closed = closed && n.len() > 2;
        let o = n.len();
        let spans = if closed { o } else { o - 1 };
        let h: Vec<f64> = (0..spans)
            .map(|l| {
                let (a, b) = (n[l], n[(l + 1) % o]);
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                dx.hypot(dy).max(1e-12)
            })
            .collect();
        let mut knots = vec![0.0];
        for l in &h {
            knots.push(knots[knots.len() - 1] + l);
        }
        let mut m = [vec![0.0; o], vec![0.0; o]];
        for l in 0..2 {
            if o == 2 && !closed {
                m[l] = vec![0.0, 0.0];
                continue;
            }
            let c: Vec<f64> = n.iter().map(|u| u[l]).collect();
            let mut a = vec![vec![0.0; o]; o];
            let mut d = vec![0.0; o];
            for u in 0..o {
                if !closed && (u == 0 || u == o - 1) {
                    a[u][u] = 1.0;
                    continue;
                }
                let mm = (u + o - 1) % o;
                let p = (u + 1) % o;
                let (g, k) = (h[mm], h[u]);
                a[u][mm] += g;
                a[u][u] += 2.0 * (g + k);
                a[u][p] += k;
                d[u] = 6.0 * ((c[p] - c[u]) / k - (c[u] - c[mm]) / g);
            }
            m[l] = gauss_jordan(&a, &d);
        }
        Some(InterpSpline { p: n, closed, h, knots, m })
    }

    pub fn length(&self) -> f64 {
        self.knots[self.knots.len() - 1]
    }

    fn locate(&self, e: f64) -> (usize, f64) {
        let t = self.length();
        let e = if self.closed { ((e % t) + t) % t } else { e.max(0.0).min(t) };
        let (mut n, mut s) = (0usize, self.h.len());
        while s - n > 1 {
            let i = (n + s) >> 1;
            if self.knots[i] <= e {
                n = i;
            } else {
                s = i;
            }
        }
        (n, e - self.knots[n])
    }

    /// Point (`deriv == 0`) or derivative (`1`, `2`) at arc-length-ish
    /// parameter `e` (client `at()`).
    pub fn at(&self, e: f64, deriv: u32) -> [f64; 2] {
        let n = self.length();
        if !self.closed && (e < 0.0 || e > n) {
            let b = if e < 0.0 { 0.0 } else { n };
            let w = self.at(b, 0);
            let y = self.at(b, 1);
            return match deriv {
                0 => [w[0] + y[0] * (e - b), w[1] + y[1] * (e - b)],
                1 => y,
                _ => [0.0, 0.0],
            };
        }
        let (s, i) = self.locate(e);
        let o = (s + 1) % self.p.len();
        let a = self.h[s];
        let l = a - i;
        let mut out = [0.0; 2];
        for k in 0..2 {
            let (c, hh) = (self.m[k][s], self.m[k][o]);
            let (d, u) = (self.p[s][k], self.p[o][k]);
            out[k] = match deriv {
                0 => (c * l * l * l + hh * i * i * i) / (6.0 * a) + (d / a - (c * a) / 6.0) * l + (u / a - (hh * a) / 6.0) * i,
                1 => (-c * l * l + hh * i * i) / (2.0 * a) - (d / a - (c * a) / 6.0) + (u / a - (hh * a) / 6.0),
                3 => (hh - c) / a,
                _ => (c * l + hh * i) / a,
            };
        }
        out
    }

    pub fn eval(&self, e: f64) -> [f64; 2] {
        self.at(e, 0)
    }

    /// Closest parameter to `q` (client `lt.param` without the optional
    /// Newton hint): sample `max(16, spans*12)` params, then Newton.
    pub fn param(&self, q: &[f64; 2]) -> f64 {
        let n = self.length();
        let o = 16usize.max(self.h.len() * 12);
        let mut s = 0.0f64;
        let mut best = f64::INFINITY;
        for l in 0..=o {
            let c = n * l as f64 / o as f64;
            let p = self.eval(c);
            let d = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]);
            if d < best {
                best = d;
                s = c;
            }
        }
        self.newton_param(q, s)
    }

    /// Client `lt.newtonParam`: 30 steps, step clamped to `0.25 * max(h)`,
    /// NO clamping of the parameter itself (the eval clamps).
    fn newton_param(&self, q: &[f64; 2], t: f64) -> f64 {
        let n = self.length();
        let mut s = t;
        let m = 0.25 * self.h.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        for _ in 0..30 {
            let o = self.eval(s);
            let a = self.at(s, 1);
            let l = self.at(s, 2);
            let c = [o[0] - q[0], o[1] - q[1]];
            let h = c[0] * a[0] + c[1] * a[1];
            let d = a[0] * a[0] + a[1] * a[1] + c[0] * l[0] + c[1] * l[1];
            if !(d.abs() > 1e-300) {
                break;
            }
            let mut u = h / d;
            if u.abs() > m {
                u = u.signum() * m;
            }
            s -= u;
            if u.abs() < 1e-13 * (1.0 + n) {
                break;
            }
        }
        s
    }
}

// ---------------------------------------------------------------------------
// Control-point splines: the client's `ze` / `Se` (app.pretty.js l.677/l.820).
// Generic over the point dimension (2 for the sketch solver, 3 for sketch3,
// 4 for the homogeneous curves behind `ref` entities).

/// Default knot vector (client `fo`): clamped `[0]*p ++ [0..=n-p] ++ [n-p]*p`,
/// periodic uniform `[0..=n]`.
pub fn default_knots(n: usize, p: usize, closed: bool) -> Vec<f64> {
    if closed {
        return (0..=n).map(|i| i as f64).collect();
    }
    let inner = n - p;
    let mut k = vec![0.0; p];
    for i in 0..=inner {
        k.push(i as f64);
    }
    k.extend(std::iter::repeat(inner as f64).take(p));
    k
}

/// Default degree (client `Jt`).
pub fn default_degree(n: usize, closed: bool) -> f64 {
    if closed {
        1.0f64.max(3.0f64.min(n as f64 - 1.0))
    } else if n <= 8 {
        1.0f64.max(n as f64 - 1.0)
    } else {
        5.0
    }
}

/// Validated spline definition (client `ze` return).
#[derive(Debug, Clone)]
pub struct BsDef {
    pub p: usize,
    pub knots: Vec<f64>,
    pub closed: bool,
}

/// Validate + build (client `ze`). `degree`/`knots` as given (`None` = the
/// client's `null` defaults).
pub fn bs_def(n_pts: usize, degree: Option<f64>, knots: Option<&[f64]>, closed: bool) -> Option<BsDef> {
    let d = degree.map(|d| (d + 0.5).floor()).unwrap_or_else(|| default_degree(n_pts, closed));
    if !(d >= 1.0 && d <= 9.0) || d.fract() != 0.0 {
        return None;
    }
    let p = d as usize;
    if closed {
        if n_pts < 3.max(p + 1) {
            return None;
        }
    } else if n_pts < p + 1 {
        return None;
    }
    let k = match knots {
        Some(t) if !t.is_empty() => t.to_vec(),
        _ => default_knots(n_pts, p, closed),
    };
    if k.iter().any(|l| !l.is_finite()) {
        return None;
    }
    for w in k.windows(2) {
        if w[1] < w[0] {
            return None;
        }
    }
    let repeated = |l: &[f64]| {
        (0..l.len()).any(|h| h + p < l.len() && l[h + p] == l[h])
    };
    if closed {
        if k.len() != n_pts + 1 || !(k[n_pts] > k[0]) || repeated(&k[..n_pts]) {
            return None;
        }
    } else {
        if k.len() != n_pts + p + 1 {
            return None;
        }
        for l in 1..=p {
            if k[l] != k[0] || k[k.len() - 1 - l] != k[k.len() - 1] {
                return None;
            }
        }
        if !(k[n_pts] > k[p]) || repeated(&k[p + 1..n_pts]) {
            return None;
        }
    }
    Some(BsDef { p, knots: k, closed })
}

/// Evaluable spline (client `Se` = `ei(def)`).
#[derive(Debug, Clone)]
pub struct Bspline<const D: usize> {
    pub p: usize,
    /// Remapped knots (client `ei` `U`).
    pub u: Vec<f64>,
    /// Control points (periodic-wrapped for closed; client `ei` `C`).
    pub c: Vec<[f64; D]>,
    pub a: f64,
    pub b: f64,
    pub closed: bool,
    /// Distinct knot values in `[a, b]` (client `ld`).
    pub knots_list: Vec<f64>,
}

impl<const D: usize> Bspline<D> {
    pub fn new(def: &BsDef, pts: &[[f64; D]]) -> Bspline<D> {
        let s = pts.len();
        let p = def.p;
        let (u, c) = if def.closed {
            let h = def.knots[s] - def.knots[0];
            let mut u = Vec::with_capacity(s + 2 * p + 1);
            for d in 0..=(s + 2 * p) {
                let v = d as isize - p as isize;
                let m = v.div_euclid(s as isize);
                let r = v - m * s as isize;
                u.push(def.knots[r as usize] + m as f64 * h - def.knots[0]);
            }
            let mut c = Vec::with_capacity(s + p);
            for d in 0..s + p {
                c.push(pts[d % s]);
            }
            (u, c)
        } else {
            let k0 = def.knots[p];
            (def.knots.iter().map(|h| h - k0).collect(), pts.to_vec())
        };
        let a = u[p];
        let b = u[c.len()];
        let mut knots_list = Vec::new();
        for t in p..=c.len() {
            let n = u[t];
            if knots_list.is_empty() || n > knots_list[knots_list.len() - 1] + 1e-12 * (b - a) {
                knots_list.push(n);
            }
        }
        Bspline { p, u, c, a, b, closed: def.closed, knots_list }
    }

    /// Build from raw curve data (client `an`).
    pub fn from_curve(
        pts: &[[f64; D]],
        degree: Option<f64>,
        knots: Option<&[f64]>,
        closed: bool,
    ) -> Option<Bspline<D>> {
        let def = bs_def(pts.len(), degree, knots, closed)?;
        Some(Bspline::new(&def, pts))
    }

    pub fn length(&self) -> f64 {
        self.b
    }

    /// Clamp into the domain (client `fl`).
    fn fl(&self, e: f64) -> f64 {
        if !self.closed {
            return self.a.max(self.b.min(e));
        }
        let t = self.b - self.a;
        let mut n = self.a + (((e - self.a) % t) + t) % t;
        if n >= self.b {
            n = self.a;
        }
        n
    }

    /// Knot span (client `go` — FindSpan with the walks past repeated knots).
    pub fn go(&self, n: f64) -> usize {
        let t = self.c.len();
        let (e, r) = (self.p, &self.u);
        if n >= r[t] {
            let mut o = t - 1;
            while o > e && r[o] >= r[t] {
                o -= 1;
            }
            return o;
        }
        if n <= r[e] {
            let mut o = e;
            while o + 1 < t && r[o + 1] <= r[e] {
                o += 1;
            }
            return o;
        }
        let (mut s, mut i) = (e, t);
        while i - s > 1 {
            let o = (s + i) >> 1;
            if r[o] <= n {
                s = o;
            } else {
                i = o;
            }
        }
        s
    }

    /// Basis derivatives up to `nd` (client `ul`, Piegl & Tiller A2.3).
    pub fn ul(&self, span: usize, n: f64, nd: usize) -> Vec<Vec<f64>> {
        let p = self.p;
        let r = &self.u;
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        ndu[0][0] = 1.0;
        for d in 1..=p {
            left[d] = n - r[span + 1 - d];
            right[d] = r[span + d] - n;
            let mut saved = 0.0;
            for m in 0..d {
                ndu[d][m] = right[m + 1] + left[d - m];
                let temp = if ndu[d][m] != 0.0 { ndu[m][d - 1] / ndu[d][m] } else { 0.0 };
                ndu[m][d] = saved + right[m + 1] * temp;
                saved = left[d - m] * temp;
            }
            ndu[d][d] = saved;
        }
        let mut l = vec![vec![0.0; p + 1]; nd + 1];
        for d in 0..=p {
            l[0][d] = ndu[d][p];
        }
        let mut c = vec![vec![0.0; p + 1]; 2];
        for d in 0..=p {
            let (mut uu, mut m) = (0usize, 1usize);
            c[0][0] = 1.0;
            for pp in 1..=nd.min(p) {
                let mut g = 0.0;
                let k = d as isize - pp as isize;
                let b = p - pp;
                if d >= pp {
                    let kk = k as usize;
                    c[m][0] = if ndu[b + 1][kk] != 0.0 { c[uu][0] / ndu[b + 1][kk] } else { 0.0 };
                    g = c[m][0] * ndu[kk][b];
                }
                let w: usize = if k >= -1 { 1 } else { (-k) as usize };
                let y: usize = if (d as isize - 1) <= b as isize { pp - 1 } else { p - d };
                for s in w..=y {
                    let ks = (k + s as isize) as usize;
                    c[m][s] =
                        if ndu[b + 1][ks] != 0.0 { (c[uu][s] - c[uu][s - 1]) / ndu[b + 1][ks] } else { 0.0 };
                    g += c[m][s] * ndu[ks][b];
                }
                if d <= b {
                    c[m][pp] = if ndu[b + 1][d] != 0.0 { -c[uu][pp - 1] / ndu[b + 1][d] } else { 0.0 };
                    g += c[m][pp] * ndu[d][b];
                }
                l[pp][d] = g;
                std::mem::swap(&mut uu, &mut m);
            }
        }
        let mut h = p as f64;
        for d in 1..=nd.min(p) {
            for uu in 0..=p {
                l[d][uu] *= h;
            }
            h *= (p - d) as f64;
        }
        l
    }

    /// Derivatives `[C(t), C'(t), ..]` (client `ho`).
    pub fn derivs(&self, e: f64, nd: usize) -> Vec<[f64; D]> {
        let t = self.fl(e);
        let s = self.go(t);
        let ders = self.ul(s, t, nd);
        let mut out = vec![[0.0; D]; nd + 1];
        for a in 0..=nd {
            if a > self.p {
                continue;
            }
            for cc in 0..=self.p {
                let d = ders[a][cc];
                for k in 0..D {
                    out[a][k] += self.c[s - self.p + cc][k] * d;
                }
            }
        }
        out
    }

    pub fn at(&self, e: f64, k: usize) -> [f64; D] {
        self.derivs(e, k)[k]
    }

    /// Closest parameter (client `Se.param` without the Newton hint:
    /// `max(16, spans*12*max(1, ceil(degree/3)))` samples + Newton).
    pub fn param(&self, q: &[f64; D]) -> f64 {
        let n = self.b;
        let spans = self.knots_list.len().max(1) - 1;
        let o = 16usize
            .max(spans * 12 * 1usize.max((self.p as f64 / 3.0).ceil() as usize));
        let mut s = 0.0f64;
        let mut best = f64::INFINITY;
        for l in 0..=o {
            let c = n * l as f64 / o as f64;
            let p = self.at(c, 0);
            let d: f64 = (0..D).map(|k| p[k] - q[k]).map(|d| d * d).sum();
            if d < best {
                best = d;
                s = c;
            }
        }
        self.newton_param(q, s)
    }

    /// Client `Se.newtonParam`: 40 steps, step clamped to `0.25 * max knot
    /// interval`, parameter clamped to `[0, length]` when open.
    fn newton_param(&self, q: &[f64; D], t: f64) -> f64 {
        let n = self.b;
        let step =
            0.25 * self.knots_list.windows(2).map(|w| w[1] - w[0]).fold(f64::NEG_INFINITY, f64::max);
        let mut i = t;
        for _ in 0..40 {
            let d = self.derivs(i, 2);
            let h: Vec<f64> = (0..D).map(|k| d[0][k] - q[k]).collect();
            let dd: f64 = (0..D).map(|k| h[k] * d[1][k]).sum();
            let den: f64 = (0..D).map(|k| d[1][k] * d[1][k] + h[k] * d[2][k]).sum();
            if !(den.abs() > 1e-300) {
                break;
            }
            let mut m = dd / den;
            if m.abs() > step {
                m = m.signum() * step;
            }
            i -= m;
            if !self.closed {
                i = 0.0f64.max(n.min(i));
            }
            if m.abs() < 1e-13 * (1.0 + n) {
                break;
            }
        }
        i
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_points() {
        let pts = [[0.0, 0.0], [10.0, 5.0], [20.0, 0.0], [30.0, 4.0]];
        let s = InterpSpline::new(&pts, false).unwrap();
        for (k, p) in pts.iter().enumerate() {
            let q = s.eval(s.knots[k]);
            assert!((q[0] - p[0]).abs() < 1e-9 && (q[1] - p[1]).abs() < 1e-9);
        }
        // natural ends: zero curvature
        let d2 = s.at(0.0, 2);
        assert!(d2[0].abs() < 1e-12 && d2[1].abs() < 1e-12);
    }

    #[test]
    fn closed_is_periodic() {
        let pts = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let s = InterpSpline::new(&pts, true).unwrap();
        assert_eq!(s.h.len(), 4);
        let a = s.at(0.0, 1);
        let b = s.at(s.length(), 1);
        assert!((a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9);
    }
}
