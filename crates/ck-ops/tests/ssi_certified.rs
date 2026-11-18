//! Marching SSI near tangent contacts and branch points, with the certified step check
//! (`intersect::certified`): two branches of the intersection that pass close together
//! (nearly equal radii, a small cylinder grazing a big one) used to let the corrector hop from
//! one to the other, giving runaway traces (hundreds of laps, never closing) or curves off
//! the surfaces. Each case below has an exact length from a closed-form parametrisation.
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Frame, Vec3};
use ck_ops::intersect::certified::verify_polyline;
use ck_ops::intersect::ss::{intersect, Bounds, SsCurve, SsResult};
use std::f64::consts::TAU;

fn fr(o: Vec3, z: Vec3, x: Vec3) -> Frame {
    frame_from(o, z, x)
}
fn bx(c: f64) -> Bounds {
    Bounds { lo: Vec3::new(-c, -c, -c), hi: Vec3::new(c, c, c) }
}
fn cyl(o: Vec3, z: Vec3, x: Vec3, r: f64) -> Surface {
    Surface::Cylinder { frame: fr(o, z, x), r }
}

fn fitted(r: &SsResult) -> Vec<(&Vec<Vec3>, bool, f64)> {
    r.curves()
        .iter()
        .map(|c| {
            let SsCurve::Fitted { pts, closed, curve } = c else { panic!("expected a fitted curve: {c:?}") };
            let (t0, t1) = curve.range();
            let mut len = 0.0;
            let mut prev = curve.eval(t0);
            for i in 1..=4000 {
                let p = curve.eval(t0 + (t1 - t0) * i as f64 / 4000.0);
                len += p.dist(prev);
                prev = p;
            }
            (pts, *closed, len)
        })
        .collect()
}

/// Length of a closed curve given by a parametrisation over [0, 2 pi).
fn param_len(f: impl Fn(f64) -> Vec3) -> f64 {
    let n = 200_000;
    (0..n).map(|i| f(TAU * i as f64 / n as f64).dist(f(TAU * (i + 1) as f64 / n as f64))).sum()
}

fn check_on_both(a: &Surface, b: &Surface, r: &SsResult, tol: f64) {
    for c in r.curves() {
        let cv = c.curve();
        let (t0, t1) = cv.range();
        for i in 0..=400 {
            let p = cv.eval(t0 + (t1 - t0) * i as f64 / 400.0);
            assert!(a.distance(p) < tol && b.distance(p) < tol, "off the surfaces: {} {}", a.distance(p), b.distance(p));
        }
    }
}

/// Perpendicular cylinders, radii 2 and r (< 2), axes meeting: two closed loops, one on each
/// side (x > 0, x < 0), `x = +- sqrt(4 - (r cos phi)^2)`, `y = r cos phi`, `z = r sin phi`.
#[test]
fn nearly_equal_radii_give_two_loops() {
    for r in [1.9, 1.99, 1.999, 1.9999] {
        let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
        let b = cyl(Vec3::ZERO, Vec3::X, Vec3::Y, r);
        let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
        let cs = fitted(&res);
        assert_eq!(cs.len(), 2, "r={r}: {} curves", cs.len());
        let truth = param_len(|p| Vec3::new((4.0 - (r * p.cos()).powi(2)).sqrt(), r * p.cos(), r * p.sin()));
        for (pts, closed, len) in cs {
            assert!(closed, "r={r}: loop not closed");
            assert!(pts.len() < 3000, "r={r}: {} points", pts.len());
            assert!((len - truth).abs() < 2e-3 * truth, "r={r}: length {len} vs {truth}");
        }
        check_on_both(&a, &b, &res, 1e-6);
    }
}

/// A small cylinder (r 1, along x at height e on y) against a big one (r 2, along z): for
/// e < 1 two separate loops, `x = +- sqrt(4 - (e + cos phi)^2)`, `y = e + cos phi`, `z = sin phi`;
/// e = 1 touches the big wall in a branch point; for e > 1 the loops have merged into one.
#[test]
fn small_cylinder_grazing_a_big_one() {
    let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    for e in [0.999, 0.9, 0.5] {
        let b = cyl(Vec3::new(0.0, e, 0.0), Vec3::X, Vec3::Z, 1.0);
        let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
        let cs = fitted(&res);
        assert_eq!(cs.len(), 2, "e={e}");
        let truth = param_len(|p| Vec3::new((4.0 - (e + p.cos()).powi(2)).sqrt(), e + p.cos(), p.sin()));
        for (_, closed, len) in cs {
            assert!(closed, "e={e}");
            assert!((len - truth).abs() < 2e-3 * truth, "e={e}: length {len} vs {truth}");
        }
        check_on_both(&a, &b, &res, 1e-6);
    }
    // past the branch: one closed curve, on both surfaces (before: two curves 0.3 and 2.3 off them)
    let b = cyl(Vec3::new(0.0, 1.001, 0.0), Vec3::X, Vec3::Z, 1.0);
    let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
    let cs = fitted(&res);
    assert_eq!(cs.len(), 1);
    assert!(cs[0].1);
    check_on_both(&a, &b, &res, 1e-6);
    // the two loops of e = 0.999 sum to the same length as the merged curve, within the merge
    assert!((cs[0].2 - 2.0 * param_len(|p| Vec3::new((4.0 - (0.999 + p.cos()).powi(2)).sqrt(), 0.999 + p.cos(), p.sin()))).abs() < 0.5);
}

/// Equal radii with the axes 1e-3 apart (not meeting): the analytic bisector-plane path does
/// not apply and the curve passes a near branch point twice. One closed curve on both
/// surfaces, about the length of the two ellipses of the meeting case (before: three curves,
/// two of them 6e-4 off the surfaces).
#[test]
fn equal_radii_axes_apart() {
    let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    let b = cyl(Vec3::new(0.0, 1e-3, 0.0), Vec3::X, Vec3::Z, 2.0);
    let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
    let cs = fitted(&res);
    assert_eq!(cs.len(), 1, "{} curves", cs.len());
    assert!(cs[0].1);
    assert!((cs[0].2 - 30.56).abs() < 0.7, "length {}", cs[0].2);
    check_on_both(&a, &b, &res, 1e-6);
}

/// Villarceau circles: the plane through a torus centre bitangent to the tube meets it in two
/// circles of the major radius that cross in two branch points.
#[test]
fn villarceau_circles() {
    let tor = Surface::Torus { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), major: 3.0, minor: 1.0 };
    let th = (1.0f64 / 3.0).asin();
    let pl = Surface::Plane { frame: fr(Vec3::ZERO, Vec3::new(0.0, -th.sin(), th.cos()), Vec3::X) };
    let res = intersect(&pl, &tor, Some(bx(6.0)), 1e-6);
    let cs = fitted(&res);
    assert_eq!(cs.len(), 2);
    for (_, closed, len) in cs {
        assert!(closed);
        assert!((len - TAU * 3.0).abs() < 2e-3 * TAU * 3.0, "length {len}");
    }
    check_on_both(&pl, &tor, &res, 1e-6);
}

/// Tangent along the whole curve (a ball of the tube radius centred on the torus core): the
/// step check must not shrink the steps there (no neighbouring branch to separate).
#[test]
fn tangent_along_a_circle_stays_cheap() {
    let tor = Surface::Torus { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), major: 3.0, minor: 1.0 };
    let sp = Surface::Sphere { frame: fr(Vec3::new(3.0, 0.0, 0.0), Vec3::Z, Vec3::X), r: 1.0 };
    let res = intersect(&sp, &tor, Some(bx(6.0)), 1e-6);
    let cs = fitted(&res);
    assert_eq!(cs.len(), 1);
    assert!(cs[0].1);
    assert!((cs[0].2 - TAU).abs() < 1e-3, "length {}", cs[0].2);
    assert!(cs[0].0.len() < 600, "{} points", cs[0].0.len());
}

/// A single touching point (tangent plane / sphere, a sphere touching a cylinder from
/// inside): nothing to trace, and it must not invent curves.
#[test]
fn point_contacts_give_no_curve() {
    let sp = Surface::Sphere { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), r: 2.0 };
    let pl = Surface::Plane { frame: fr(Vec3::new(0.0, 0.0, 2.0), Vec3::Z, Vec3::X) };
    assert!(intersect(&pl, &sp, Some(bx(4.0)), 1e-6).curves().is_empty());
    let c0 = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    let sp = Surface::Sphere { frame: fr(Vec3::new(1.0, 0.0, 0.0), Vec3::Z, Vec3::X), r: 1.0 };
    assert!(intersect(&sp, &c0, Some(bx(4.0)), 1e-6).curves().is_empty());
    let (a, b) = (cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0), cyl(Vec3::new(2.0, 0.0, 0.0), Vec3::Y, Vec3::Z, 1.0));
    assert!(intersect(&a, &b, Some(bx(4.0)), 1e-6).curves().is_empty());
}

/// The certified verification of a traced curve: nearly all spans are proved to lie on one
/// arc; the failures cluster at the branch points.
#[test]
fn traced_curves_verify() {
    let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    let b = cyl(Vec3::ZERO, Vec3::X, Vec3::Y, 1.9);
    let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
    for (pts, _, _) in fitted(&res) {
        let (ok, _bad, total) = verify_polyline(&a, &b, pts);
        assert!(ok * 10 >= total * 9, "{ok}/{total}");
    }
}

/// A big cylinder (r 20, along z) and a half-size one (r 10, along x) whose top almost touches
/// the big wall (gap d, a near internal tangent): two loops `x = +- sqrt(400 - y^2)`,
/// `y = 10 - d + 10 cos phi`, `z = 10 sin phi`. Before: a single curve of twice the length that
/// hopped between the loops (gap 1e-3, 9e-4 off the surfaces) or a 34000 point runaway (gap 1e-2).
#[test]
fn near_internal_tangent_cylinders() {
    let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 20.0);
    for d in [1e-2, 1e-3, 1e-4] {
        let b = cyl(Vec3::new(0.0, 10.0 - d, 0.0), Vec3::X, Vec3::Z, 10.0);
        let res = intersect(&a, &b, Some(bx(40.0)), 1e-6);
        let cs = fitted(&res);
        assert_eq!(cs.len(), 2, "d={d}");
        let truth = param_len(|p| {
            let y = 10.0 - d + 10.0 * p.cos();
            Vec3::new((400.0 - y * y).sqrt(), y, 10.0 * p.sin())
        });
        for (pts, closed, len) in cs {
            assert!(closed && pts.len() < 3000, "d={d}");
            assert!((len - truth).abs() < 2e-3 * truth, "d={d}: length {len} vs {truth}");
        }
        check_on_both(&a, &b, &res, 1e-6);
    }
}

/// A vertical cylinder of the tube radius shifted 1e-3 off the torus core circle: it grazes
/// the tube along a near-tangent band; one closed curve on both surfaces (before: two curves up
/// to 2e-3 off them).
#[test]
fn vertical_cylinder_near_tube_tangent() {
    let tor = Surface::Torus { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), major: 3.0, minor: 1.0 };
    let cy = cyl(Vec3::new(0.0, 3.001, 0.0), Vec3::Z, Vec3::X, 1.0);
    let res = intersect(&cy, &tor, Some(bx(6.0)), 1e-6);
    let cs = fitted(&res);
    assert_eq!(cs.len(), 1, "{} curves", cs.len());
    assert!(cs[0].1);
    check_on_both(&cy, &tor, &res, 1e-6);
}
