//! Sample cloud + uniform bucket grid over a long multi-span B-spline
//! surface (screw sweeps: hundreds of knot spans). The control-net-box scan
//! of `BSplineSurface::closest_param_within` is O(spans) per query and the
//! booleans query such faces thousands of times; the cloud answers a query
//! with a few bucket lookups (cached per surface by the caller).

use super::bsurf::BSplineSurface;
use crate::Vec3;
use std::collections::HashMap;

pub struct Cloud {
    pts: Vec<(Vec3, f64, f64)>,
    org: Vec3,
    h: f64,
    buckets: HashMap<(i32, i32, i32), Vec<u32>>,
    lo: Vec3,
    hi: Vec3,
}

impl Cloud {
    /// `iu_of` / `iv_of`: span index -> cell index (usize::MAX = empty span).
    pub fn build(s: &BSplineSurface) -> Cloud {
        // 3 x 3 samples inside every non-empty knot-span cell
        const F: [f64; 3] = [1.0 / 6.0, 0.5, 5.0 / 6.0];
        let spans = |kn: &[f64], p: usize, n: usize| -> Vec<(f64, f64)> { (p..n).filter(|&i| kn[i] < kn[i + 1]).map(|i| (kn[i], kn[i + 1])).collect() };
        let su = spans(&s.knots_u, s.deg_u, s.nu);
        let sv = spans(&s.knots_v, s.deg_v, s.nv);
        let mut pts = Vec::with_capacity(su.len() * sv.len() * 9);
        for &(a, b) in &su {
            for &fu in &F {
                let u = a + (b - a) * fu;
                for &(c, d) in &sv {
                    for &fv in &F {
                        let v = c + (d - c) * fv;
                        pts.push((s.eval(u, v), u, v));
                    }
                }
            }
        }
        let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for p in &pts {
            lo = lo.min(p.0);
            hi = hi.max(p.0);
        }
        // sample spacing along v within a cell (consecutive pushes) -> bucket size
        let mut sum = 0.0;
        let mut n = 0usize;
        for w in pts.chunks(3) {
            sum += w[0].0.dist(w[1].0) + w[1].0.dist(w[2].0);
            n += 2;
        }
        let sp_v = if n > 0 { sum / n as f64 } else { 1.0 };
        // spacing along u: stride over one v-cell's worth of samples
        let stride = sv.len() * 3;
        let mut sum_u = 0.0;
        let mut nu = 0usize;
        for i in (0..pts.len().saturating_sub(stride)).step_by(7) {
            sum_u += pts[i].0.dist(pts[i + stride].0);
            nu += 1;
        }
        let sp_u = if nu > 0 { sum_u / nu as f64 } else { 1.0 };
        let diag = (hi - lo).len().max(1e-9);
        let h = (2.5 * sp_u.max(sp_v)).max(diag * 1e-4).max(1e-9);
        let mut c = Cloud { pts, org: lo, h, buckets: HashMap::new(), lo, hi };
        for (i, p) in c.pts.iter().enumerate() {
            let k = c.key(p.0);
            c.buckets.entry(k).or_default().push(i as u32);
        }
        c
    }

    fn key(&self, p: Vec3) -> (i32, i32, i32) {
        let d = (p - self.org) * (1.0 / self.h);
        (d.x.floor() as i32, d.y.floor() as i32, d.z.floor() as i32)
    }

    /// Nearest sample within `r` of `q`: (distance, u, v).
    fn nearest_within(&self, q: Vec3, r: f64) -> Option<(f64, f64, f64)> {
        let (a, b) = (self.key(q - Vec3::new(r, r, r)), self.key(q + Vec3::new(r, r, r)));
        let nb = (b.0 - a.0 + 1) as i64 * (b.1 - a.1 + 1) as i64 * (b.2 - a.2 + 1) as i64;
        let mut best = (r * r, 0.0, 0.0);
        let mut found = false;
        let mut take = |i: u32| {
            let p = &self.pts[i as usize];
            let d2 = (p.0 - q).len2();
            if d2 <= best.0 {
                best = (d2, p.1, p.2);
                found = true;
            }
        };
        if nb > self.buckets.len() as i64 * 2 + 64 {
            // a sphere larger than the occupied grid: scan the buckets
            for v in self.buckets.values() {
                for &i in v {
                    take(i);
                }
            }
        } else {
            for x in a.0..=b.0 {
                for y in a.1..=b.1 {
                    for z in a.2..=b.2 {
                        if let Some(v) = self.buckets.get(&(x, y, z)) {
                            for &i in v {
                                take(i);
                            }
                        }
                    }
                }
            }
        }
        found.then(|| (best.0.sqrt(), best.1, best.2))
    }

    /// Nearest sample to `q` (within `maxd`): (u, v).
    pub fn nearest(&self, q: Vec3, maxd: f64) -> Option<(f64, f64)> {
        let diag = (self.hi - self.lo).len() + (q - self.lo).len().min((q - self.hi).len()) + 2.0 * self.h;
        let mut r = (4.0 * self.h).min(maxd);
        loop {
            if let Some((d, u, v)) = self.nearest_within(q, r) {
                // nearest overall once the searched ball holds it
                if d <= r {
                    return Some((u, v));
                }
            }
            if r >= maxd || r > diag {
                return None;
            }
            r = (r * 2.0).min(maxd);
        }
    }
}
