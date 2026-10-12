//! Marching SSI of loops much smaller than the march step (a small
//! cylinder through a big sphere, bounds spanning the long cylinder): each
//! closed loop must come back once, complete and on both surfaces. (The
//! marcher used to declare a loop closed within 0.75 * step of its seed
//! and to shadow every seed near a traced point by the nominal step, so
//! small loops degenerated to 2-point curves and the real loops were lost.)
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_ops::intersect::ss::{intersect, Bounds, SsCurve, SsResult};

#[test]
fn small_cylinder_through_big_sphere_gives_two_full_loops() {
    let sph = Surface::Sphere {
        frame: Frame { o: Vec3::new(0.0, 0.0, -38.86), x: Vec3::X, y: Vec3::new(0.0, -1.0, 0.0), z: Vec3::new(0.0, 0.0, -1.0) },
        r: 42.88,
    };
    let cy = Surface::Cylinder { frame: Frame { o: Vec3::new(29.5, 0.0, 0.0), x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 2.12 };
    let b = Bounds { lo: Vec3::new(-43.0, -43.0, -200.0), hi: Vec3::new(43.0, 43.0, 200.0) };
    let SsResult::Curves(c) = intersect(&sph, &cy, Some(b), 1e-6) else { panic!("no curves") };
    assert_eq!(c.len(), 2, "one loop on each hemisphere");
    for k in &c {
        let SsCurve::Fitted { pts, closed, .. } = k else { panic!("expected a fitted curve") };
        assert!(*closed && pts.len() > 20, "closed={closed} n={}", pts.len());
        let cv = k.curve();
        let (a, z) = cv.range();
        for i in 0..=50 {
            let p = cv.eval(a + (z - a) * i as f64 / 50.0);
            assert!(sph.distance(p).abs() < 1e-6 && cy.distance(p).abs() < 1e-6);
        }
    }
}
