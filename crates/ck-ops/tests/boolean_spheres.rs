//! Whole-sphere operands: a ball united with / cut by another ball, and a ball as a void inside a block.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::{PI, TAU};

fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

/// A whole ball of radius `r` centred at (cx, 0, 0): the revolved half disc whose axis is the profile's y direction.
fn ball(feature: &str, cx: f64, r: f64) -> Body {
    let outer = ProfileLoop::new(vec![ProfileSeg::arc("e1", v(cx, 0.0), v(cx, -r), v(cx, r)), ProfileSeg::line("e2", v(cx, r), v(cx, -r))]);
    let prof = Profile::new(top(), vec![ProfileRegion { outer, holes: vec![] }]);
    revolve(feature, &prof, Vec3::new(cx, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0), 0.0, TAU).unwrap()
}

fn block(feature: &str, lo: f64, hi: f64) -> Body {
    let p = [v(lo, lo), v(hi, lo), v(hi, hi), v(lo, hi)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude(feature, &prof, ExtrudeOpts { s0: lo, s1: hi, dir: None, draft: 0.0 }).unwrap()
}

fn ball_vol(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

#[test]
fn ball_volume() {
    assert!((mass_props(&ball("F2", 0.0, 5.0)).volume - ball_vol(5.0)).abs() < 1e-6);
}

#[test]
fn equal_balls_overlapping_unite() {
    let (a, b) = (ball("F2", 0.0, 5.0), ball("F4", 5.0, 5.0));
    let out = boolean(&a, &b, Op::Union, 1e-6).expect("union of overlapping balls").body;
    let lens = PI * 25.0 * 25.0 / 12.0;
    assert!(check(&out, 1e-6).is_empty());
    assert!((mass_props(&out).volume - (2.0 * ball_vol(5.0) - lens)).abs() < 1e-3);
}

#[test]
fn equal_balls_overlapping_cut() {
    let (a, b) = (ball("F2", 0.0, 5.0), ball("F4", 5.0, 5.0));
    let out = boolean(&a, &b, Op::Cut, 1e-6).expect("cut of overlapping balls").body;
    let lens = PI * 25.0 * 25.0 / 12.0;
    assert!(check(&out, 1e-6).is_empty());
    assert!((mass_props(&out).volume - (ball_vol(5.0) - lens)).abs() < 1e-3);
}

#[test]
fn ball_void_inside_a_block() {
    let (a, b) = (block("F2", -10.0, 10.0), ball("F4", 0.0, 4.0));
    let r = boolean(&a, &b, Op::Cut, 1e-6).expect("ball void");
    assert!(r.mismatch.is_none(), "{:?}", r.mismatch);
    assert!(check(&r.body, 1e-6).is_empty());
    assert!((mass_props(&r.body).volume - (8000.0 - ball_vol(4.0))).abs() < 1e-3);
}

#[test]
fn concentric_balls_cut_hollow() {
    let (a, b) = (ball("F2", 0.0, 5.0), ball("F4", 0.0, 3.0));
    let r = boolean(&a, &b, Op::Cut, 1e-6).expect("hollow ball");
    assert!(r.mismatch.is_none(), "{:?}", r.mismatch);
    assert!((mass_props(&r.body).volume - (ball_vol(5.0) - ball_vol(3.0))).abs() < 1e-3);
}
