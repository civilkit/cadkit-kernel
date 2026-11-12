//! A boolean of two identical operands at the same place (kit issue internal-org/cadkit-kit#34: a second
//! identical marine propeller hung the rebuild): union and intersect return one operand, cut returns an empty body,
//! at once.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_ops::sweep::{sweep, PathSeg};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;
use std::time::Instant;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn region(outer: ProfileLoop) -> Vec<ProfileRegion> {
    vec![ProfileRegion { outer, holes: vec![] }]
}
fn boxb(f: &str) -> Body {
    let lp = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(0.0, 0.0), v(40.0, 0.0)),
        ProfileSeg::line("e2", v(40.0, 0.0), v(40.0, 20.0)),
        ProfileSeg::line("e3", v(40.0, 20.0), v(0.0, 20.0)),
        ProfileSeg::line("e4", v(0.0, 20.0), v(0.0, 0.0)),
    ]);
    extrude(f, &Profile::new(top(), region(lp)), ExtrudeOpts { s0: 0.0, s1: 10.0, dir: None, draft: 0.0 }).unwrap()
}
fn dome(f: &str) -> Body {
    let outer = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(0.0, 0.0), v(10.0, 0.0)),
        ProfileSeg::line("e2", v(10.0, 0.0), v(10.0, 15.0)),
        ProfileSeg::arc("e3", v(0.0, 15.0), v(10.0, 15.0), v(0.0, 25.0)),
        ProfileSeg::line("e4", v(0.0, 25.0), v(0.0, 0.0)),
    ]);
    revolve(f, &Profile::new(top(), region(outer)), Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, TAU).unwrap()
}
/// A twisted blade-like sweep (free-form faces).
fn twisted(f: &str) -> Body {
    let lp = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(0.0, 0.0), v(6.0, 0.0)),
        ProfileSeg::line("e2", v(6.0, 0.0), v(6.0, 1.5)),
        ProfileSeg::line("e3", v(6.0, 1.5), v(0.0, 1.5)),
        ProfileSeg::line("e4", v(0.0, 1.5), v(0.0, 0.0)),
    ]);
    let prof = Profile::new(Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, region(lp));
    sweep(f, &prof, &[PathSeg::Line { a: Vec3::ZERO, b: Vec3::new(0.0, 0.0, 30.0) }], 1.2).unwrap()
}

fn vol(b: &Body) -> f64 {
    mass_props(b).volume
}

fn check_identical(a: &Body) {
    let b = a.clone();
    let t = Instant::now();
    let u = boolean(a, &b, Op::Union, 1e-6).expect("union");
    let i = boolean(a, &b, Op::Intersect, 1e-6).expect("intersect");
    let c = boolean(a, &b, Op::Cut, 1e-6);
    let dt = t.elapsed().as_secs_f64();
    eprintln!("identical-operand union+intersect+cut: {dt:.3} s");
    assert!(dt < 5.0, "identical-operand booleans took {dt:.1} s");
    for r in [&u, &i] {
        assert!(check(&r.body, 1e-6).is_empty());
        assert!((vol(&r.body) - vol(a)).abs() < 1e-6 * vol(a).abs().max(1.0));
    }
    // nothing left: an empty body
    assert_eq!(c.expect("cut").body.num_faces(), 0);
}

#[test]
fn identical_box() {
    check_identical(&boxb("F2"));
}
#[test]
fn identical_dome() {
    check_identical(&dome("F2"));
}
#[test]
fn identical_twisted_sweep() {
    check_identical(&twisted("F2"));
}

/// A propeller-like solid: a hub with three twisted free-form blades fused to it.
fn propeller(f: &str) -> Body {
    use ck_core::xform::Xform;
    let hub = {
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(0.0, 0.0), 4.0)]);
        extrude(f, &Profile::new(Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, region(lp)), ExtrudeOpts { s0: -3.0, s1: 3.0, dir: None, draft: 0.0 }).unwrap()
    };
    let mut acc = hub;
    for k in 0..3 {
        let mut blade = twisted_blade("B");
        blade.transform(&Xform::rotation(Vec3::ZERO, Vec3::Z, TAU * k as f64 / 3.0));
        acc = boolean(&acc, &blade, Op::Union, 1e-6).expect("hub + blade").body;
    }
    acc
}
/// A blade lying along +x from the hub surface outwards, twisted along its length.
fn twisted_blade(f: &str) -> Body {
    twisted_blade_by(f, 0.9)
}
fn twisted_blade_by(f: &str, twist: f64) -> Body {
    let lp = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(-1.0, -3.0), v(1.0, -3.0)),
        ProfileSeg::line("e2", v(1.0, -3.0), v(1.0, 3.0)),
        ProfileSeg::line("e3", v(1.0, 3.0), v(-1.0, 3.0)),
        ProfileSeg::line("e4", v(-1.0, 3.0), v(-1.0, -3.0)),
    ]);
    let prof = Profile::new(Frame { o: Vec3::new(2.0, 0.0, 0.0), x: Vec3::Y, y: Vec3::Z, z: Vec3::X }, region(lp));
    sweep(f, &prof, &[PathSeg::Line { a: Vec3::new(2.0, 0.0, 0.0), b: Vec3::new(30.0, 0.0, 0.0) }], twist).unwrap()
}
#[test]
fn identical_propeller() {
    let t = Instant::now();
    let p = propeller("F2");
    eprintln!("build propeller: {:.3} s", t.elapsed().as_secs_f64());
    check_identical(&p);
}

/// A copy moved by far more than the tolerance is not "identical": the general path runs and the volumes add up.
#[test]
fn shifted_copy_is_not_identical() {
    use ck_core::xform::Xform;
    let a = boxb("F2");
    let mut b = a.clone();
    b.transform(&Xform::translation(Vec3::new(10.0, 0.0, 0.0)));
    let u = boolean(&a, &b, Op::Union, 1e-6).unwrap();
    assert!((vol(&u.body) - 50.0 * 20.0 * 10.0).abs() < 1e-3, "{}", vol(&u.body));
}

/// A second blade fitted a second time over the first (kit issue 34): same outline, but the free-form faces lie a few
/// thousandths of a millimetre off the first blade's, which no welding tolerance bridges. The union used to grind through
/// every tolerance of the ladder and the robust mode (minutes) and then fail; it now refuses at once, naming the faces.
#[test]
fn near_coincident_blade_over_a_blade_fails_fast_with_the_faces_named() {
    let p = propeller("F2");
    let again = twisted_blade_by("G", 0.902);
    let t = Instant::now();
    let r = boolean(&p, &again, Op::Union, 1e-6);
    let dt = t.elapsed().as_secs_f64();
    eprintln!("near-coincident blade union: {dt:.3} s -> {:?}", r.as_ref().map(|_| ()));
    let e = r.expect_err("the near-coincident union cannot close");
    assert!(e.starts_with("booleanFailed;nearCoincident="), "{e}");
    assert!(dt < 10.0, "took {dt:.1} s");
}
