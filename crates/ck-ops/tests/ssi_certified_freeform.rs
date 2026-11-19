//! The certified check of marched SSI curves on free-form surfaces (`certified::check_curves`):
//! bicubic B-spline patches against planes and cylinders, near-tangent contacts, and the cases of
//! ssi_certified.rs with the big cylinder transcribed to its exact NURBS form.
use ck_geom::convert::surface_to_nurbs;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Frame, Vec3};
use ck_ops::intersect::certified::check_curves;
use ck_ops::intersect::ss::{intersect, Bounds, SsCurve, SsResult};
use std::f64::consts::{PI, TAU};

fn fr(o: Vec3, z: Vec3, x: Vec3) -> Frame {
    frame_from(o, z, x)
}
fn bx(c: f64) -> Bounds {
    Bounds { lo: Vec3::new(-c, -c, -c), hi: Vec3::new(c, c, c) }
}
fn cyl(o: Vec3, z: Vec3, x: Vec3, r: f64) -> Surface {
    Surface::Cylinder { frame: fr(o, z, x), r }
}

/// (closed, length) of each fitted curve.
fn loops(r: &SsResult) -> Vec<(bool, f64)> {
    r.curves()
        .iter()
        .map(|c| {
            let SsCurve::Fitted { closed, curve, .. } = c else { panic!("expected a fitted curve") };
            let (t0, t1) = curve.range();
            let mut len = 0.0;
            let mut prev = curve.eval(t0);
            for i in 1..=4000 {
                let p = curve.eval(t0 + (t1 - t0) * i as f64 / 4000.0);
                len += p.dist(prev);
                prev = p;
            }
            (*closed, len)
        })
        .collect()
}

fn param_len(f: impl Fn(f64) -> Vec3) -> f64 {
    let n = 200_000;
    (0..n).map(|i| f(TAU * i as f64 / n as f64).dist(f(TAU * (i + 1) as f64 / n as f64))).sum()
}

fn on_both(a: &Surface, b: &Surface, r: &SsResult, tol: f64) {
    for c in r.curves() {
        let cv = c.curve();
        let (t0, t1) = cv.range();
        for i in 0..=400 {
            let p = cv.eval(t0 + (t1 - t0) * i as f64 / 400.0);
            assert!(a.distance(p) < tol && b.distance(p) < tol, "off the surfaces: {} {}", a.distance(p), b.distance(p));
        }
    }
}

/// A bicubic patch over [-4, 4]^2, heights `z = amp * bump(x, y)` on a 7 x 7 net (a smooth hill of
/// height about `amp` in the middle).
fn hill(amp: f64) -> Surface {
    let n = 7;
    let k = vec![0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0];
    let mut ctrl = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-4.0 + 8.0 * i as f64 / (n - 1) as f64, -4.0 + 8.0 * j as f64 / (n - 1) as f64);
            let z = amp * (-(x * x + y * y) / 6.0).exp() + 0.05 * x;
            ctrl.push(Vec3::new(x, y, z));
        }
    }
    Surface::BSpline(Box::new(BSplineSurface::new(3, 3, k.clone(), k, n, n, ctrl, None).unwrap()))
}

#[test]
fn bicubic_against_plane_and_cylinder_certify() {
    let h = hill(2.0);
    for (tag, s) in [
        ("plane z=1", Surface::Plane { frame: fr(Vec3::new(0.0, 0.0, 1.0), Vec3::Z, Vec3::X) }),
        ("tilted plane", Surface::Plane { frame: fr(Vec3::new(0.0, 0.0, 0.8), Vec3::new(0.2, 0.1, 1.0).normalize(), Vec3::X) }),
        ("vertical cylinder", cyl(Vec3::new(0.3, -0.2, 0.0), Vec3::Z, Vec3::X, 1.5)),
        ("lying cylinder", cyl(Vec3::new(0.0, 0.0, 0.5), Vec3::X, Vec3::Y, 1.2)),
    ] {
        let res = intersect(&h, &s, Some(bx(6.0)), 1e-6);
        assert!(!res.curves().is_empty(), "{tag}: no curve");
        on_both(&h, &s, &res, 1e-6);
        let chk = check_curves(&h, &s, &res);
        assert!(chk.spans > 50, "{tag}: {} spans", chk.spans);
        // left unresolved only where the curve runs off the patch: a box about a point on the domain's edge
        // reaches past it, where the (clamped) surface stops moving and the Jacobian can be singular
        let fitted: Vec<&Vec<Vec3>> = res.curves().iter().filter_map(|c| if let SsCurve::Fitted { pts, .. } = c { Some(pts) } else { None }).collect();
        for i in chk.issues.iter().filter(|i| !i.repaired) {
            let pts = fitted[i.curve];
            let on_edge = |p: Vec3| {
                let (u, v) = h.closest_param(p);
                u.min(v) < 1e-6 || u.max(v) > 1.0 - 1e-6
            };
            assert!(on_edge(pts[i.span]) || on_edge(pts[(i.span + 1) % pts.len()]), "{tag}: span {} unresolved inside the patch", i.span);
        }
        assert!(chk.unresolved() <= 2 * fitted.len(), "{tag}: {:?}", chk.issues);
        assert!(chk.certified * 100 >= chk.spans * 98, "{tag}: {}/{}", chk.certified, chk.spans);
    }
}

/// Near-tangent: a plane just under the top of the hill cuts a small, tightly curved loop where the
/// normals are nearly parallel. The curve is on both surfaces and every span is proved or repaired.
#[test]
fn bicubic_near_tangent_plane() {
    let h = hill(2.0);
    // the hill's top: the highest point of a dense sample
    let mut top: f64 = f64::NEG_INFINITY;
    for i in 0..=400 {
        for j in 0..=400 {
            top = top.max(h.eval(i as f64 / 400.0, j as f64 / 400.0).z);
        }
    }
    for gap in [1e-2, 1e-3] {
        let pl = Surface::Plane { frame: fr(Vec3::new(0.0, 0.0, top - gap), Vec3::Z, Vec3::X) };
        let res = intersect(&h, &pl, Some(bx(6.0)), 1e-6);
        let ls = loops(&res);
        assert_eq!(ls.len(), 1, "gap {gap}: {} curves", ls.len());
        assert!(ls[0].0, "gap {gap}: not closed");
        on_both(&h, &pl, &res, 1e-6);
        let chk = check_curves(&h, &pl, &res);
        assert_eq!(chk.unresolved(), 0, "gap {gap}: {:?}", chk.issues);
    }
}

/// The big cylinder of ssi_certified.rs as its exact NURBS (seam on -y, which no loop of these cases
/// reaches): the same loops as the analytic cylinder, on both surfaces, and every span proved.
#[test]
fn nurbs_cylinder_gives_the_analytic_loops() {
    let big = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    let nb = Surface::BSpline(Box::new(surface_to_nurbs(&big, (1.5 * PI, 1.5 * PI + TAU), (-4.0, 4.0)).unwrap()));
    let mut cases: Vec<(String, Surface, f64)> = Vec::new();
    for r in [1.9, 1.99, 1.999] {
        let truth = param_len(|p| Vec3::new((4.0 - (r * p.cos()).powi(2)).sqrt(), r * p.cos(), r * p.sin()));
        cases.push((format!("radii r={r}"), cyl(Vec3::ZERO, Vec3::X, Vec3::Y, r), truth));
    }
    for e in [0.999, 0.9, 0.5] {
        let truth = param_len(|p| Vec3::new((4.0 - (e + p.cos()).powi(2)).sqrt(), e + p.cos(), p.sin()));
        cases.push((format!("graze e={e}"), cyl(Vec3::new(0.0, e, 0.0), Vec3::X, Vec3::Z, 1.0), truth));
    }
    for (tag, b, truth) in cases {
        let ra = intersect(&big, &b, Some(bx(4.0)), 1e-6);
        let rn = intersect(&nb, &b, Some(bx(4.0)), 1e-6);
        let (la, ln) = (loops(&ra), loops(&rn));
        assert_eq!(ln.len(), la.len(), "{tag}: {} loops vs {} analytic", ln.len(), la.len());
        assert_eq!(ln.len(), 2, "{tag}");
        for (closed, len) in ln {
            assert!(closed, "{tag}: open");
            assert!((len - truth).abs() < 2e-3 * truth, "{tag}: length {len} vs {truth}");
        }
        on_both(&nb, &b, &rn, 1e-6);
        let chk = check_curves(&nb, &b, &rn);
        assert_eq!(chk.unresolved(), 0, "{tag}: {:?}", chk.issues);
    }
    // the near internal tangent of a half-size cylinder (gap 1e-3) against an r 20 NURBS cylinder
    let a20 = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 20.0);
    let n20 = Surface::BSpline(Box::new(surface_to_nurbs(&a20, (1.5 * PI, 1.5 * PI + TAU), (-40.0, 40.0)).unwrap()));
    let d = 1e-3;
    let b = cyl(Vec3::new(0.0, 10.0 - d, 0.0), Vec3::X, Vec3::Z, 10.0);
    let rn = intersect(&n20, &b, Some(bx(40.0)), 1e-6);
    let truth = param_len(|p| {
        let y = 10.0 - d + 10.0 * p.cos();
        Vec3::new((400.0 - y * y).sqrt(), y, 10.0 * p.sin())
    });
    let ln = loops(&rn);
    assert_eq!(ln.len(), 2, "internal tangent: {} loops", ln.len());
    for (closed, len) in ln {
        assert!(closed && (len - truth).abs() < 2e-3 * truth, "internal tangent: {closed} {len} vs {truth}");
    }
}

/// The check reports and repairs: a coarse polyline of a correct curve (one point in 50 to 150 of a
/// trace of one of two close loops) fails some spans, which inserted arc points repair; a span that
/// jumps between the two loops at their neck is reported and not repaired.
#[test]
fn check_reports_and_repairs() {
    let a = cyl(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
    let b = cyl(Vec3::ZERO, Vec3::X, Vec3::Y, 1.999);
    let res = intersect(&a, &b, Some(bx(4.0)), 1e-6);
    let SsCurve::Fitted { pts, curve, closed } = &res.curves()[0] else { panic!() };
    // thin the trace: spans tens of steps long (a sizeable turn of the loop) are not proved as they are
    // and get points of the arc inserted
    let mut repaired = 0;
    for k in [50, 100, 150] {
        let thin: Vec<Vec3> = pts.iter().step_by(k).cloned().collect();
        let coarse = SsResult::Curves(vec![SsCurve::Fitted { pts: thin.clone(), curve: curve.clone(), closed: *closed }]);
        let chk = check_curves(&a, &b, &coarse);
        let fixed = chk.issues.iter().filter(|i| i.repaired).count();
        assert!(chk.polylines[0].len() >= thin.len() + fixed, "step {k}: repair points missing");
        repaired += fixed;
    }
    assert!(repaired > 0, "nothing to repair");
    // a polyline that crosses from one loop (x > 0) to the other (x < 0) at the neck
    let other = &res.curves()[1];
    let SsCurve::Fitted { pts: p2, .. } = other else { panic!() };
    let near = |ps: &Vec<Vec3>| ps.iter().cloned().max_by(|u, v| u.y.total_cmp(&v.y)).unwrap();
    let (pa, pb) = (near(pts), near(p2));
    assert!(pa.x * pb.x < 0.0);
    let hop = SsResult::Curves(vec![SsCurve::Fitted { pts: vec![pa, pb], curve: curve.clone(), closed: false }]);
    let chk = check_curves(&a, &b, &hop);
    assert_eq!(chk.spans, 1);
    assert_eq!(chk.unresolved(), 1, "a hop between the loops was proved: {chk:?}");
}
