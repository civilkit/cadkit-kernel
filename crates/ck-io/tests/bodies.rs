//! Test bodies (same constructions as ck-topo/tests/oracle.rs).
#![allow(dead_code)]
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::Body;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::{FRAC_PI_2, TAU};

fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn rect(tags: [&str; 4], x0: f64, y0: f64, x1: f64, y1: f64) -> ProfileLoop {
    let p = [v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)];
    ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect())
}
fn one(fr: Frame, outer: ProfileLoop, holes: Vec<ProfileLoop>) -> Profile {
    Profile::new(fr, vec![ProfileRegion { outer, holes }])
}
const E: [&str; 4] = ["e1", "e2", "e3", "e4"];
pub const NAMES: [&str; 16] = [
    "box", "box3", "cyl", "dext", "draft", "flip", "mid", "two", "rev360", "rev90", "revax360", "revax90", "sphere",
    "torus", "cone", "void",
];

pub fn build(name: &str) -> Body {
    let fr = Frame::identity();
    let r40 = || rect(E, 0.0, 0.0, 40.0, 20.0);
    let rr = |x0: f64| rect(E, x0, 0.0, 20.0, 5.0);
    match name {
        "box" => extrude("F2", &one(top(), r40(), vec![]), ExtrudeOpts::blind(10.0)),
        "box3" => {
            let hole = ProfileLoop::new(vec![
                ProfileSeg::line("h4", v(5.0, 15.0), v(5.0, 5.0)).reversed(),
                ProfileSeg::line("h3", v(15.0, 15.0), v(5.0, 15.0)).reversed(),
                ProfileSeg::line("h2", v(15.0, 5.0), v(15.0, 15.0)).reversed(),
                ProfileSeg::line("h1", v(5.0, 5.0), v(15.0, 5.0)).reversed(),
            ]);
            let circ = ProfileLoop::new(vec![ProfileSeg::circle("e5", v(30.0, 10.0), 4.0).reversed()]);
            extrude("F2", &one(fr, r40(), vec![circ, hole]), ExtrudeOpts::blind(10.0))
        }
        "cyl" => extrude(
            "F2",
            &one(top(), ProfileLoop::new(vec![ProfileSeg::circle("e7", v(5.0, 3.0), 10.0)]), vec![]),
            ExtrudeOpts::blind(10.0),
        ),
        "dext" => {
            let d = ProfileLoop::new(vec![
                ProfileSeg::line("e1", v(-10.0, 0.0), v(10.0, 0.0)),
                ProfileSeg::arc("e2", v(0.0, 0.0), v(10.0, 0.0), v(-10.0, 0.0)),
            ]);
            extrude("F2", &one(fr, d, vec![]), ExtrudeOpts::blind(5.0))
        }
        "draft" => {
            let mut o = ExtrudeOpts::blind(10.0);
            o.draft = 10f64.to_radians();
            extrude("F2", &one(fr, r40(), vec![]), o)
        }
        "flip" => extrude("F2", &one(fr, r40(), vec![]), ExtrudeOpts::flipped(10.0)),
        "mid" => extrude("F2", &one(fr, r40(), vec![]), ExtrudeOpts::mid(10.0)),
        "two" => extrude(
            "F2",
            &Profile::new(
                fr,
                vec![
                    ProfileRegion { outer: rect(["a0", "a1", "a2", "a3"], 0.0, 0.0, 10.0, 10.0), holes: vec![] },
                    ProfileRegion { outer: rect(["b0", "b1", "b2", "b3"], 20.0, 0.0, 30.0, 10.0), holes: vec![] },
                ],
            ),
            ExtrudeOpts::blind(5.0),
        ),
        "rev360" => revolve("F2", &one(fr, rr(10.0), vec![]), Vec3::ZERO, Vec3::Y, 0.0, TAU),
        "rev90" => revolve("F2", &one(fr, rr(10.0), vec![]), Vec3::ZERO, Vec3::Y, 0.0, FRAC_PI_2),
        "revax360" => revolve("F2", &one(fr, rr(0.0), vec![]), Vec3::ZERO, Vec3::Y, 0.0, TAU),
        "revax90" => revolve("F2", &one(fr, rr(0.0), vec![]), Vec3::ZERO, Vec3::Y, 0.0, FRAC_PI_2),
        "sphere" => {
            let semi = ProfileLoop::new(vec![
                ProfileSeg::arc("e1", v(0.0, 0.0), v(0.0, -10.0), v(0.0, 10.0)),
                ProfileSeg::line("e2", v(0.0, 10.0), v(0.0, -10.0)),
            ]);
            revolve("F2", &one(fr, semi, vec![]), Vec3::ZERO, Vec3::Y, 0.0, TAU)
        }
        "torus" => revolve(
            "F2",
            &one(fr, ProfileLoop::new(vec![ProfileSeg::circle("e1", v(20.0, 0.0), 5.0)]), vec![]),
            Vec3::ZERO,
            Vec3::Y,
            0.0,
            TAU,
        ),
        "cone" => {
            let tri = ProfileLoop::new(vec![
                ProfileSeg::line("e1", v(5.0, 0.0), v(15.0, 0.0)),
                ProfileSeg::line("e2", v(15.0, 0.0), v(5.0, 10.0)),
                ProfileSeg::line("e3", v(5.0, 10.0), v(5.0, 0.0)),
            ]);
            revolve("F2", &one(fr, tri, vec![]), Vec3::ZERO, Vec3::Y, 0.0, 120f64.to_radians())
        }
        "void" => {
            let outer = rect(E, 10.0, 0.0, 30.0, 20.0);
            let hole = ProfileLoop::new(vec![ProfileSeg::circle("e5", v(20.0, 10.0), 5.0).reversed()]);
            // regen splits shells by connectivity (void shell joins its host lump)
            revolve("F2", &one(fr, outer, vec![hole]), Vec3::ZERO, Vec3::Y, 0.0, TAU).map(|mut b| {
                ck_topo::region::rebuild_lumps(&mut b);
                b
            })
        }
        _ => unreachable!(),
    }
    .unwrap()
}

