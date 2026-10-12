//! Natural cubic spline through 3D points, chord-length parametrised: the
//! curve of a `sketch3d` `spline` entity (the client's `wt` class; a
//! `sweep` along it was verified against the original: tube volume
//! A * length agrees to 3e-7). Optional unit end tangents replace the
//! natural end conditions (a tangent constraint to a line).

use ck_geom::Vec3;

#[derive(Debug, Clone)]
pub struct SplinePath {
    pts: Vec<Vec3>,
    knots: Vec<f64>,
    h: Vec<f64>,
    /// Second derivatives at the points.
    m: Vec<Vec3>,
}

impl SplinePath {
    /// None for fewer than two distinct points. Consecutive points closer
    /// than 1e-9 are merged.
    pub fn new(points: &[Vec3], start_tan: Option<Vec3>, end_tan: Option<Vec3>) -> Option<SplinePath> {
        let mut pts: Vec<Vec3> = vec![];
        for &p in points {
            if pts.last().map_or(true, |q| q.dist(p) > 1e-9) {
                pts.push(p);
            }
        }
        let n = pts.len();
        if n < 2 {
            return None;
        }
        let h: Vec<f64> = (0..n - 1).map(|i| pts[i].dist(pts[i + 1]).max(1e-12)).collect();
        let mut knots = vec![0.0];
        for &x in &h {
            knots.push(knots.last().unwrap() + x);
        }
        let unit = |t: Option<Vec3>| t.and_then(|t| t.try_normalize());
        let (t0, t1) = (unit(start_tan), unit(end_tan));
        let mut m = vec![Vec3::ZERO; n];
        if !(n == 2 && t0.is_none() && t1.is_none()) {
            for c in 0..3 {
                let get = |v: Vec3| [v.x, v.y, v.z][c];
                // tridiagonal system: lo[i] M[i-1] + di[i] M[i] + up[i] M[i+1] = rhs[i]
                let (mut lo, mut di, mut up, mut rhs) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
                for i in 0..n {
                    if i == 0 {
                        match t0 {
                            Some(t) => {
                                di[0] = 2.0 * h[0];
                                up[0] = h[0];
                                rhs[0] = 6.0 * ((get(pts[1]) - get(pts[0])) / h[0] - get(t));
                            }
                            None => di[0] = 1.0,
                        }
                    } else if i == n - 1 {
                        match t1 {
                            Some(t) => {
                                let y = h[n - 2];
                                lo[i] = y;
                                di[i] = 2.0 * y;
                                rhs[i] = 6.0 * (get(t) - (get(pts[i]) - get(pts[i - 1])) / y);
                            }
                            None => di[i] = 1.0,
                        }
                    } else {
                        let (a, b) = (h[i - 1], h[i]);
                        lo[i] = a;
                        di[i] = 2.0 * (a + b);
                        up[i] = b;
                        rhs[i] = 6.0 * ((get(pts[i + 1]) - get(pts[i])) / b - (get(pts[i]) - get(pts[i - 1])) / a);
                    }
                }
                // Thomas algorithm (diagonally dominant)
                for i in 1..n {
                    let w = lo[i] / di[i - 1];
                    di[i] -= w * up[i - 1];
                    rhs[i] -= w * rhs[i - 1];
                }
                let mut x = vec![0.0; n];
                x[n - 1] = rhs[n - 1] / di[n - 1];
                for i in (0..n - 1).rev() {
                    x[i] = (rhs[i] - up[i] * x[i + 1]) / di[i];
                }
                for i in 0..n {
                    match c {
                        0 => m[i].x = x[i],
                        1 => m[i].y = x[i],
                        _ => m[i].z = x[i],
                    }
                }
            }
        }
        Some(SplinePath { pts, knots, h, m })
    }

    pub fn length(&self) -> f64 {
        *self.knots.last().unwrap()
    }

    /// Number of cubic pieces.
    pub fn spans(&self) -> usize {
        self.h.len()
    }

    fn locate(&self, s: f64) -> (usize, f64) {
        let s = s.clamp(0.0, self.length());
        let n = self.knots.partition_point(|&k| k <= s).saturating_sub(1).min(self.h.len() - 1);
        (n, s - self.knots[n])
    }

    pub fn eval(&self, s: f64) -> Vec3 {
        let (n, o) = self.locate(s);
        let (i, a) = (self.h[n], self.h[n] - o);
        let (c, h) = (self.m[n], self.m[n + 1]);
        (c * (a * a * a) + h * (o * o * o)) * (1.0 / (6.0 * i)) + (self.pts[n] * (1.0 / i) - c * (i / 6.0)) * a + (self.pts[n + 1] * (1.0 / i) - h * (i / 6.0)) * o
    }

    /// First derivative (tangent, speed 1 only approximately).
    pub fn d1(&self, s: f64) -> Vec3 {
        let (n, o) = self.locate(s);
        let (i, a) = (self.h[n], self.h[n] - o);
        let (c, h) = (self.m[n], self.m[n + 1]);
        (h * (o * o) - c * (a * a)) * (1.0 / (2.0 * i)) - (self.pts[n] * (1.0 / i) - c * (i / 6.0)) + (self.pts[n + 1] * (1.0 / i) - h * (i / 6.0))
    }

    /// Second derivative.
    pub fn d2(&self, s: f64) -> Vec3 {
        let (n, o) = self.locate(s);
        let (i, a) = (self.h[n], self.h[n] - o);
        (self.m[n] * a + self.m[n + 1] * o) * (1.0 / i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn helix_pts(n: usize, turns: f64) -> Vec<Vec3> {
        (0..=n)
            .map(|i| {
                let th = std::f64::consts::TAU * turns * i as f64 / n as f64;
                Vec3::new(20.0 * th.cos(), 7.0 * th / std::f64::consts::TAU, -20.0 * th.sin())
            })
            .collect()
    }

    #[test]
    fn interpolates_and_has_consistent_derivatives() {
        let pts = helix_pts(24, 2.0);
        let sp = SplinePath::new(&pts, None, None).unwrap();
        for (i, p) in pts.iter().enumerate() {
            assert!(sp.eval(sp.knots[i]).dist(*p) < 1e-9);
        }
        let s = 0.37 * sp.length();
        let h = 1e-5;
        let fd = (sp.eval(s + h) - sp.eval(s - h)) * (0.5 / h);
        assert!(fd.dist(sp.d1(s)) < 1e-6, "{fd:?} {:?}", sp.d1(s));
        let fd2 = (sp.d1(s + h) - sp.d1(s - h)) * (0.5 / h);
        assert!(fd2.dist(sp.d2(s)) < 1e-5);
        // natural end conditions
        assert!(sp.d2(0.0).len() < 1e-9 && sp.d2(sp.length()).len() < 1e-9);
    }

    #[test]
    fn length_matches_the_original_kernel_probe() {
        // original kernel: a circle r=2 swept along this spline has volume 3161.5405511200897 = A * L
        let sp = SplinePath::new(&helix_pts(24, 2.0), None, None).unwrap();
        let n = 4000;
        let l: f64 = (0..n).map(|i| sp.eval(sp.length() * i as f64 / n as f64).dist(sp.eval(sp.length() * (i + 1) as f64 / n as f64))).sum();
        let v = std::f64::consts::PI * 4.0 * l;
        assert!((v - 3161.5405511200897).abs() / 3161.54 < 1e-6, "{v}");
    }

    #[test]
    fn end_tangents_are_honoured() {
        let pts = helix_pts(12, 1.0);
        let t0 = Vec3::new(0.0, 0.05, -1.0).normalize();
        let sp = SplinePath::new(&pts, Some(t0), None).unwrap();
        assert!(sp.d1(0.0).normalize().dist(t0) < 1e-9);
    }
}
