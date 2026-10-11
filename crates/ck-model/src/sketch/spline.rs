//! Interpolating sketch spline (`{"type":"spline","pts":[..],"closed":?}`).
//!
//! Port of the client's `lt` class (app.pretty.js l.1237): a C2 cubic
//! spline through the points, parametrised by chord length, with natural
//! end conditions (open) or periodic conditions (closed). Moments are solved
//! with the client's dense Gauss-Jordan `fd()`.
//!
//! Parameter `s` runs over `[0, length]` (sum of chord lengths).

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
            if i == 0 || (p[0] - n[n.len() - 1][0]).hypot(p[1] - n[n.len() - 1][1]) > 1e-9 {
                n.push(*p);
            }
        }
        if n.len() < 2 || (closed && n.len() < 3) {
            return None;
        }
        if closed && n.len() > 2 {
            let (f, l) = (n[0], n[n.len() - 1]);
            if (f[0] - l[0]).hypot(f[1] - l[1]) < 1e-12 {
                n.pop();
            }
        }
        let closed = closed && n.len() > 2;
        let o = n.len();
        let spans = if closed { o } else { o - 1 };
        let h: Vec<f64> = (0..spans)
            .map(|l| {
                let (a, b) = (n[l], n[(l + 1) % o]);
                (b[0] - a[0]).hypot(b[1] - a[1]).max(1e-12)
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
