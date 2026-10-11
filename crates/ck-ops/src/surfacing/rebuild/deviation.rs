//! Deviation of a rebuilt surface from the original (probe stat:
//! "deviation from the original: largest {max} mm, mean {mean} mm").

use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};

/// Sample both surfaces over the uv window and report (max, mean) distance.
pub fn deviation(orig: &Surface, rebuilt: &BSplineSurface, uw: (f64, f64), vw: (f64, f64), n: usize) -> (f64, f64) {
    let mut max = 0.0f64;
    let mut sum = 0.0f64;
    let mut count = 0usize;
    for k in 0..=n {
        for l in 0..=n {
            let u = uw.0 + (uw.1 - uw.0) * (k as f64 / n as f64);
            let v = vw.0 + (vw.1 - vw.0) * (l as f64 / n as f64);
            let p = orig.eval(u, v);
            // find the closest point on the rebuilt surface (Newton from the
            // mapped parameter)
            let (t0, t1) = rebuilt.domain();
            let _ = t1;
            let guess_t = t0.0 + (t0.1 - t0.0) * (k as f64 / n as f64);
            let guess_s = v0_of(rebuilt) + (v1_of(rebuilt) - v0_of(rebuilt)) * (l as f64 / n as f64);
            let q = closest_on(rebuilt, p, (guess_t, guess_s));
            let d = q.dist(p);
            max = max.max(d);
            sum += d;
            count += 1;
        }
    }
    (max, if count > 0 { sum / count as f64 } else { 0.0 })
}

fn v0_of(s: &BSplineSurface) -> f64 {
    s.domain().1 .0
}
fn v1_of(s: &BSplineSurface) -> f64 {
    s.domain().1 .1
}

/// Closest point on the surface (Newton with clamped steps).
fn closest_on(s: &BSplineSurface, p: Vec3, (mut t, mut st): (f64, f64)) -> Vec3 {
    for _ in 0..12 {
        let d = s.derivs2(t, st);
        let r = d.p - p;
        let g = Vec2::new(r.dot(d.du), r.dot(d.dv));
        let a = d.du.dot(d.du);
        let b = d.du.dot(d.dv);
        let c = d.dv.dot(d.dv);
        let det = a * c - b * b;
        if det.abs() < 1e-18 {
            break;
        }
        let dt = -(g.x * c - g.y * b) / det;
        let ds = -(g.y * a - g.x * b) / det;
        let ((t0, t1), (s0, s1)) = s.domain();
        t = (t + dt).clamp(t0, t1);
        st = (st + ds).clamp(s0, s1);
        if dt.abs() < 1e-12 && ds.abs() < 1e-12 {
            break;
        }
    }
    s.eval(t, st)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_rebuild_deviation_is_zero() {
        let orig = Surface::plane(Vec3::ZERO, Vec3::Z);
        let bs = super::super::net::fit_surface(&orig, (0.0, 10.0), (0.0, 10.0), 3, 3, 2, 2).unwrap();
        let (max, mean) = deviation(&orig, &bs, (0.0, 10.0), (0.0, 10.0), 8);
        assert!(max < 1e-8 && mean < 1e-8, "max {max} mean {mean}");
    }
}
