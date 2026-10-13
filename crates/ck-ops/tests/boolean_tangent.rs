//! A hole tangent to the plate's side wall (BJ-8, brepjs conduit-snap-clip F13 / F15):
//! the cylinder touches the wall plane along a line. The cylinder's fragments were
//! merged across that line AND across its relocated seam (the seam test only knew
//! u = 0), leaving a seamless two-loop band of lateral area 0: the hole removed a
//! third of its volume, silently, and a following countersink was `booleanFailed`.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::{PI, TAU};

fn top_at(y: f64) -> Frame {
    Frame { o: Vec3::new(0.0, y, 0.0), x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn region(outer: ProfileLoop) -> Vec<ProfileRegion> {
    vec![ProfileRegion { outer, holes: vec![] }]
}

/// Plate x in [-15, 15], z in [-6, 6], y in [-0.5, 3.5].
fn plate() -> Body {
    let p = [v(-15.0, -6.0), v(15.0, -6.0), v(15.0, 6.0), v(-15.0, 6.0)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    extrude("F10", &Profile::new(top_at(-0.5), region(lp)), ExtrudeOpts { s0: 0.0, s1: 4.0, dir: None, draft: 0.0 }).unwrap()
}

fn hole(cx: f64, r: f64) -> Body {
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(cx, 0.0), r)]);
    extrude("F13", &Profile::new(top_at(-1.0), region(lp)), ExtrudeOpts { s0: 0.0, s1: 5.5, dir: None, draft: 0.0 }).unwrap()
}

/// Countersink: cone from radius r0 at y = 1.6 to r1 at y = 3.9, axis x = cx.
fn csk(cx: f64, r0: f64, r1: f64) -> Body {
    let q = [v(cx, 1.6), v(cx + r0, 1.6), v(cx + r1, 3.9), v(cx, 3.9)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), q[i], q[(i + 1) % 4])).collect());
    revolve("F15", &Profile::new(Frame::identity(), region(lp)), Vec3::new(cx, 0.0, 0.0), Vec3::Y, 0.0, TAU).unwrap()
}

/// Area of a disc of radius r centred at distance d from a wall, on the near side of it.
fn clipped_disc(r: f64, d: f64) -> f64 {
    if r <= d {
        return PI * r * r;
    }
    PI * r * r - (r * r * (d / r).acos() - d * (r * r - d * d).sqrt())
}

fn assert_valid(b: &Body) {
    let issues = check(b, 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
}

fn run(cx: f64) {
    let (hr, r0, r1) = (2.3, 2.3, 4.2);
    let d = cx + 15.0; // distance from the hole axis to the wall x = -15
    let hole_removed = PI * hr * hr * 4.0;
    let h = boolean(&plate(), &hole(cx, hr), Op::Cut, 1e-6).expect("hole");
    assert_valid(&h.body);
    let v_hole = mass_props(&h.body).volume;
    assert!((v_hole - (30.0 * 12.0 * 4.0 - hole_removed)).abs() < 1e-6, "hole volume {v_hole}");
    // the countersink also removes the cone inside the plate (y 1.6 .. 3.5) beyond the hole
    let n = 2000;
    let (y0, y1) = (1.6, 3.5);
    let mut extra = 0.0;
    for i in 0..n {
        let y = y0 + (y1 - y0) * (i as f64 + 0.5) / n as f64;
        let r = r0 + (r1 - r0) * (y - 1.6) / 2.3;
        extra += (clipped_disc(r, d) - PI * hr * hr) * (y1 - y0) / n as f64;
    }
    let c = boolean(&h.body, &csk(cx, r0, r1), Op::Cut, 1e-6).expect("countersink");
    assert_valid(&c.body);
    let v_csk = mass_props(&c.body).volume;
    assert!((v_csk - (v_hole - extra)).abs() < 2e-4 * v_hole, "countersunk volume {v_csk} vs {}", v_hole - extra);
}

#[test]
fn hole_tangent_to_the_side_wall_then_countersink() {
    run(-12.7);
}

#[test]
fn hole_just_inside_and_just_outside_the_wall() {
    // control: the neighbours of the tangent position (wall thickness 0.001 / breakout 0.001)
    run(-12.699);
}
