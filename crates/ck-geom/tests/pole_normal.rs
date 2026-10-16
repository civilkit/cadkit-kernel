//! The unit normal of a B-spline surface at a pole (a collapsed row of control points) is the
//! limit of the normal on the way in, not zero.
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::Vec3;

/// A quadratic fan of three base points meeting at an apex (linear in v).
fn fan(apex_at_max_v: bool) -> BSplineSurface {
    let base = [Vec3::new(2.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 0.3), Vec3::new(0.0, 2.5, 0.0)];
    let apex = Vec3::new(0.2, 0.3, 3.0);
    let mut ctrl = vec![];
    for b in base {
        if apex_at_max_v {
            ctrl.extend([b, apex]);
        } else {
            ctrl.extend([apex, b]);
        }
    }
    BSplineSurface::new(2, 1, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 3, 2, ctrl, None).unwrap()
}

#[test]
fn normal_at_a_pole_is_the_limit_normal() {
    for apex_at_max in [true, false] {
        let s = fan(apex_at_max);
        let (vp, vi) = if apex_at_max { (1.0, 1.0 - 1e-6) } else { (0.0, 1e-6) };
        for k in 0..=8 {
            let u = k as f64 / 8.0;
            let n = s.normal(u, vp);
            assert!((n.len() - 1.0).abs() < 1e-9, "u={u}: |n| = {}", n.len());
            let m = s.normal(u, vi);
            assert!(n.dot(m) > 0.9999, "u={u}: pole normal {n:?} vs limit {m:?}");
            let sn = Surface::BSpline(Box::new(s.clone())).normal(u, vp);
            assert!(sn.dot(n) > 1.0 - 1e-12);
        }
    }
}
