//! Post-boolean sanity check (`ck_ops::boolean::sanity`): a wrong but valid
//! result is reported through `BoolOut::mismatch`; legitimate touching,
//! tangent, disjoint and containing configurations are not.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::sanity::check_result;
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::PI;

fn top_at(y: f64) -> Frame {
    Frame { o: Vec3::new(0.0, y, 0.0), x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn region(outer: ProfileLoop) -> Vec<ProfileRegion> {
    vec![ProfileRegion { outer, holes: vec![] }]
}
fn cyl(feature: &str, frame: Frame, cx: f64, cy: f64, r: f64, s0: f64, s1: f64) -> Body {
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(cx, cy), r)]);
    extrude(feature, &Profile::new(frame, region(lp)), ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
}
fn boxb(feature: &str, x0: f64, y0: f64, x1: f64, y1: f64, s0: f64, s1: f64) -> Body {
    let p = [v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    extrude(feature, &Profile::new(top_at(0.0), region(lp)), ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
}
fn vol(b: &Body) -> f64 {
    mass_props(b).volume
}

/// BenchCAD j_hook: a rod and a 180 degree revolved handle (same radius)
/// whose start-cap rim lies exactly on the rod's cylinder.
fn rod_and_handle(a0: f64, a1: f64) -> (Body, Body) {
    let rod = cyl("E1", top_at(48.72), 31.0, 0.0, 6.0, -53.52, 53.52);
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(31.0, 0.0), 6.0)]);
    let handle = revolve("R1", &Profile::new(top_at(99.24), region(lp)), Vec3::new(0.0, 99.24, 0.0), Vec3::Z, a0, a1).unwrap();
    (rod, handle)
}

/// j_hook root cause: the rod's cylinder band above the handle's start rim was
/// kept on both sides of its seam; the seam was cancelled or kept depending on
/// the order the shared edges were listed, and a cancelled seam left a dangling
/// slit plus a second loop (cylinder lateral area 0, volume 14938).
/// The up handle matches CadQuery (22782.6); for the down handle the overlap
/// (1944.2) is a Monte Carlo of rod and tube, so the union is the sum minus it.
#[test]
fn hook_union_is_right() {
    for (a0, a1, want, tol) in [(0.0, PI, 22782.6, 1e-4), (-PI, 0.0, 12105.94 + 11014.48 - 1944.2, 5e-4)] {
        let (rod, handle) = rod_and_handle(a0, a1);
        let out = boolean(&rod, &handle, Op::Union, 1e-6).expect("union");
        assert!(out.mismatch.is_none(), "{:?}", out.mismatch);
        let issues = ck_topo::check::check(&out.body, 1e-6);
        assert!(issues.is_empty(), "{issues:?}");
        let got = vol(&out.body);
        assert!((got - want).abs() < tol * want, "hook {a0}..{a1}: volume {got} vs {want}");
    }
}

#[test]
fn forged_results_are_flagged() {
    // overlapping boxes: a result missing most of b
    let a = boxb("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
    let b = boxb("B", 8.0, 0.0, 30.0, 10.0, 0.0, 10.0);
    let good = boolean(&a, &b, Op::Union, 1e-6).unwrap();
    assert!(good.mismatch.is_none());
    assert!(check_result(&a, &b, Op::Union, &a).is_some(), "result = a drops b");
    assert!(check_result(&a, &b, Op::Union, &good.body).is_none());
    // disjoint: a union that lost one solid
    let c = boxb("C", 100.0, 0.0, 110.0, 10.0, 0.0, 10.0);
    assert!(check_result(&a, &c, Op::Union, &a).is_some());
    // an interior tool cannot cut nothing
    let big = boxb("D", 2.0, 2.0, 8.0, 8.0, 2.0, 8.0);
    assert!(check_result(&a, &big, Op::Cut, &a).is_none(), "tool fully inside: box overlap equals the tool, cut == a is within the padded bound");
    // intersect that came back as the whole of a although b is smaller
    assert!(check_result(&a, &big, Op::Intersect, &a).is_some());
}

#[test]
fn legitimate_configurations_pass() {
    let a = boxb("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
    // face contact, edge contact, disjoint, containment, partial overlap
    let touching = boxb("B", 10.0, 0.0, 20.0, 10.0, 0.0, 10.0);
    let edge = boxb("B", 10.0, 10.0, 20.0, 20.0, 0.0, 10.0);
    let apart = boxb("B", 40.0, 0.0, 50.0, 10.0, 0.0, 10.0);
    let inner = boxb("B", 2.0, 2.0, 8.0, 8.0, 2.0, 8.0);
    let part = boxb("B", 5.0, 5.0, 15.0, 15.0, 5.0, 15.0);
    for (name, t) in [("touching", &touching), ("edge", &edge), ("apart", &apart), ("inner", &inner), ("part", &part)] {
        for op in [Op::Union, Op::Cut, Op::Intersect] {
            // (edge-only contact is a known booleanFailed; a failure is not a wrong result)
            if let Ok(out) = boolean(&a, t, op, 1e-6) {
                assert!(out.mismatch.is_none(), "{name} {op:?}: {:?}", out.mismatch);
            } else {
                assert_eq!(name, "edge", "{name} {op:?} failed");
            }
        }
    }
    // a cylinder tangent to a box wall, and a through hole
    let tan = cyl("C", top_at(0.0), 5.0, -5.0, 5.0, 0.0, 10.0);
    let hole = cyl("H", top_at(0.0), 5.0, -5.0, 2.0, -1.0, 11.0);
    for t in [&tan, &hole] {
        for op in [Op::Union, Op::Cut, Op::Intersect] {
            if let Ok(out) = boolean(&a, t, op, 1e-6) {
                assert!(out.mismatch.is_none(), "{op:?}: {:?}", out.mismatch);
            }
        }
    }
}

/// A solid of revolution of an ellipse (faces on `Surface::Revolution`, integrated along iso-v
/// lines with the profile evaluated once): Pappus gives 2 pi R * pi a b.
#[test]
fn revolved_ellipse_volume() {
    let seg = ProfileSeg { tag: "e1".into(), curve: ck_geom::curve2::Curve2::ellipse(v(20.0, 0.0), v(1.0, 0.0), 4.0, 2.5), t0: 0.0, t1: 2.0 * PI, rev: false };
    let body = revolve("R", &Profile::new(top_at(0.0), region(ProfileLoop::new(vec![seg]))), Vec3::ZERO, Vec3::Z, 0.0, 2.0 * PI).unwrap();
    let want = 2.0 * PI * 20.0 * PI * 4.0 * 2.5;
    let got = vol(&body);
    assert!((got - want).abs() < 1e-6 * want, "mass_props {got} vs {want}");
    let faces: Vec<_> = body.face_ids().collect();
    let sum: f64 = faces.iter().map(|&f| ck_topo::mass::face_signed_volume(&body, f)).sum();
    assert!((sum - want).abs() < 1e-6 * want, "per-face {sum} vs {want}");
}

/// Trim curves that are fitted splines (an oblique cylinder through a cylinder): the volume check
/// integrates their faces with a quarter of the boundary panels, which must stay within 1e-4 of
/// the full integration.
#[test]
fn coarse_volume_check_matches_full_integration_on_spline_trims() {
    let c = (30.0f64.to_radians().cos(), 30.0f64.to_radians().sin());
    let tilted = Frame { o: Vec3::new(0.0, 5.0, 0.0), x: Vec3::X, y: Vec3::new(0.0, -c.1, -c.0), z: Vec3::new(0.0, c.0, -c.1) };
    let host = cyl("H", top_at(0.0), 0.0, 0.0, 10.0, -20.0, 20.0);
    let tool = cyl("T", tilted, 0.0, 0.0, 4.0, -30.0, 30.0);
    for op in [Op::Cut, Op::Union] {
        let out = boolean(&host, &tool, op, 1e-6).expect("boolean");
        assert!(out.mismatch.is_none(), "{:?}", out.mismatch);
        let sum: f64 = out.body.face_ids().map(|f| ck_topo::mass::face_signed_volume(&out.body, f)).sum();
        let want = vol(&out.body);
        assert!((sum - want).abs() < 1e-4 * want, "{op:?}: per-face {sum} vs mass_props {want}");
    }
}

/// The sanity check sums per-face volume integrals memoised per face: the sum must equal the
/// whole-body volume, and the second pass over a chain of cuts (cache warm) must agree.
#[test]
fn per_face_volume_sum_matches_body_volume() {
    let mut body = boxb("B", 0.0, 0.0, 40.0, 30.0, 0.0, 20.0);
    for i in 0..3 {
        let tool = cyl("C", top_at(0.0), 8.0 + 12.0 * i as f64, 15.0, 4.0, -5.0, 25.0);
        let out = boolean(&body, &tool, Op::Cut, 1e-6).expect("cut");
        assert!(out.mismatch.is_none(), "{:?}", out.mismatch);
        let faces: Vec<_> = out.body.face_ids().collect();
        let sum: f64 = faces.iter().map(|&f| ck_topo::mass::face_signed_volume(&out.body, f)).sum();
        let want = vol(&out.body);
        assert!((sum - want).abs() < 1e-6 * want, "cut {i}: per-face {sum} vs body {want}");
        assert!(check_result(&body, &tool, Op::Cut, &out.body).is_none());
        body = out.body;
    }
}
