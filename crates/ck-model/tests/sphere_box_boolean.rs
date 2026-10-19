//! OCCT `bfuse_simple`/`bcut_simple`/`bcommon_simple` sphere+box family (A1, D3, D4):
//! a revolved unit sphere whose seam/pole meets the corner of a unit box. The sphere
//! face is one periodic face; the box imprint crosses its seam twice, so the face's
//! outside-the-box fragment is a loop that walks the seam edge both ways. A naive uv
//! polygon collapses that spike to zero width and its area comes out as the *complement*
//! negated, so the fragment was nested as a hole and `sample_point` failed
//! (`booleanFailed`). The uv polygon and the result pcurves must place the two seam
//! traversals one period apart.
//!
//! Areas pinned from OCCT (`checkprops`): fuse 14.6394, sphere-box cut 13.3518,
//! box-sphere cut 5.2146.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

/// Unit sphere at the origin: the revolved half disc whose axis is the profile's y direction.
fn ball(feature: &str) -> Body {
    let outer = ProfileLoop::new(vec![ProfileSeg::arc("e1", v(0.0, 0.0), v(0.0, -1.0), v(0.0, 1.0)), ProfileSeg::line("e2", v(0.0, 1.0), v(0.0, -1.0))]);
    let prof = Profile::new(top(), vec![ProfileRegion { outer, holes: vec![] }]);
    revolve(feature, &prof, Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, TAU).unwrap()
}

/// Unit box in the octant `x, y in [lo, hi]` and `z in [-hi, -lo]`: its corner `(0, 0, -1)`
/// is the sphere's south pole when `lo = 0`, its `(0, 0, 1)` the north pole when `hi = 0`.
fn block(feature: &str, lo: f64, hi: f64) -> Body {
    let p = [v(lo, lo), v(hi, lo), v(hi, hi), v(lo, hi)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude(feature, &prof, ExtrudeOpts { s0: lo, s1: hi, dir: None, draft: 0.0 }).unwrap()
}

fn octant() -> f64 {
    // intersection volume: one eighth of the unit sphere
    std::f64::consts::PI * 4.0 / 3.0 / 8.0
}

fn check_case(lo: f64, hi: f64) {
    let (s, b) = (ball("F2"), block("F4", lo, hi));
    let expected = [
        (Op::Union, "fuse", 4.0 / 3.0 * std::f64::consts::PI + 1.0 - octant()),
        (Op::Cut, "sphere-box", 4.0 / 3.0 * std::f64::consts::PI - octant()),
        (Op::Intersect, "common", octant()),
    ];
    for (op, name, vol) in expected {
        let r = boolean(&s, &b, op, 1e-6).unwrap_or_else(|e| panic!("{name} [{lo},{hi}]: {e}"));
        assert!(r.mismatch.is_none(), "{name} [{lo},{hi}]: {:?}", r.mismatch);
        assert!(check(&r.body, 1e-6).is_empty(), "{name} [{lo},{hi}]: {:?}", check(&r.body, 1e-6));
        let got = mass_props(&r.body).volume;
        assert!((got - vol).abs() < 1e-3, "{name} [{lo},{hi}]: volume {got} vs {vol}");
    }
    // the opposite operand order: box minus sphere
    let r = boolean(&b, &s, Op::Cut, 1e-6).expect("box-sphere");
    assert!(r.mismatch.is_none());
    assert!(check(&r.body, 1e-6).is_empty());
    let got = mass_props(&r.body).volume;
    assert!((got - (1.0 - octant())).abs() < 1e-3, "box-sphere [{lo},{hi}]: {got}");
}

#[test]
fn sphere_box_corner_with_south_pole() {
    check_case(0.0, 1.0);
}

#[test]
fn sphere_box_corner_with_north_pole() {
    check_case(-1.0, 0.0);
}
