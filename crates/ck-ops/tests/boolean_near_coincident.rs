//! Union of two coaxial discs whose radii differ by a few micrometres (a gap of 1/128 mm on a radius of 35 mm),
//! with coplanar caps. Parts libraries build many shapes from copies offset by such an epsilon.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::PI;

fn disc(f: &str, r: f64, h: f64) -> Body {
    let top = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y };
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), r)]);
    extrude(f, &Profile::new(top, vec![ProfileRegion { outer: lp, holes: vec![] }]), ExtrudeOpts { s0: 0.0, s1: h, dir: None, draft: 0.0 }).unwrap()
}

fn run(r1: f64, h1: f64, r2: f64, h2: f64, op: Op, want: f64) {
    let out = boolean(&disc("F1", r1, h1), &disc("F2", r2, h2), op, 1e-6).unwrap_or_else(|e| panic!("{op:?} r {r1}/{r2} h {h1}/{h2}: {e:?}"));
    let issues = check(&out.body, 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
    let v = mass_props(&out.body).volume;
    assert!((v - want).abs() < 1e-6 * want.max(1.0), "{op:?} r {r1}/{r2} h {h1}/{h2}: volume {v} vs {want}");
}

#[test]
fn near_coincident_discs() {
    let big = 35.0;
    let small = 34.9921875;
    let a = |r: f64, h: f64| PI * r * r * h;
    // equal heights, either operand the larger one
    for &(r1, r2) in &[(small, big), (big, small)] {
        run(r1, 3.0, r2, 3.0, Op::Union, a(big, 3.0));
        run(r1, 3.0, r2, 3.0, Op::Intersect, a(small, 3.0));
    }
    // the lower operand is the larger one: a stepped union
    run(small, 3.0, big, 2.0, Op::Union, a(big, 2.0) + a(small, 1.0));
    run(big, 2.0, small, 3.0, Op::Union, a(big, 2.0) + a(small, 1.0));
    // a smaller disc cut from a larger one leaves a thin ring
    run(big, 3.0, small, 3.0, Op::Cut, a(big, 3.0) - a(small, 3.0));
}
