//! A marched intersection that runs into the parameter-domain edge of a B-spline surface (here the
//! u = 0 / 2π seam of a revolved patch, as at the seam between the two halves of a loft) must end
//! exactly on that edge, so the curve computed for the neighbouring patch meets it at the same
//! point instead of up to a march step short of it.
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_ops::intersect::ss::{intersect, Bounds, SsCurve, SsResult};

#[test]
fn marched_curve_ends_on_the_seam_of_a_revolved_patch() {
    // cone as a B-spline revolution: profile (10,0,0) -> (5,0,10), seam in the half-plane y = 0, x > 0
    let profile = BSplineCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Vec3::new(10.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 10.0)]).unwrap();
    let cone = Surface::BSpline(Box::new(ck_geom::convert::revolve(&profile, Vec3::ZERO, Vec3::Z, 0.0, std::f64::consts::TAU)));
    // a cylinder straddling the seam
    let cyl = Surface::Cylinder { frame: Frame { o: Vec3::new(7.5, 0.0, 0.0), x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 1.0 };
    let b = Bounds { lo: Vec3::new(-12.0, -12.0, -1.0), hi: Vec3::new(12.0, 12.0, 11.0) };
    let SsResult::Curves(cs) = intersect(&cone, &cyl, Some(b), 1e-6) else { panic!("no curves") };
    let mut open = 0;
    for c in &cs {
        let SsCurve::Fitted { pts, closed, .. } = c else { continue };
        if *closed {
            continue;
        }
        open += 1;
        for p in [pts[0], pts[pts.len() - 1]] {
            // both ends of an arc that leaves the patch lie on the seam half-plane (y = 0)
            assert!(p.y.abs() < 1e-7, "arc end {p:?} is {} off the seam", p.y.abs());
        }
    }
    assert!(open >= 1, "expected open arcs ending on the seam, got {} curves", cs.len());
}
