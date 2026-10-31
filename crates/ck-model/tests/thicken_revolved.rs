//! Thicken of a revolved sheet (sphere cap, hemisphere, sphere band, cone, torus arc; full and partial turn), from
//! solidcore #1502 / PRs #1719, #1735: every case must build a valid solid of the right volume or refuse. A full
//! turn used to be refused at the seam ("non-manifold or dangling edge"), a partial turn came back invalid with the
//! wrong volume (the cap at 180 degrees: 90.27 instead of 173.31).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::f64::consts::PI;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// Front-plane sketch with one profile entity (`ent`, points a, b and optionally c) and the axis line x = 0,
/// sheet-revolved by `angle`, thickened by `t` on `side`. Returns (error codes, volume, validation issues).
fn run(points: &str, ent: &str, angle: f64, t: f64, side: &str) -> (Vec<String>, Option<f64>, Vec<String>) {
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},
          "points":[{points},{{"id":"o","x":0,"y":-20}},{{"id":"q","x":0,"y":20}}],
          "entities":[{ent},{{"type":"line","id":"ax","p1":"o","p2":"q","construction":true}}],"constraints":[]}},
        {{"type":"surfaceRevolve","id":"S","sketch":"F1","axis":"ax","end":"angle","angle":{angle}}},
        {{"type":"thicken","id":"T","surface":"S","thickness":{t},"side":"{side}"}}]}}"#
    );
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let codes = r.errors.iter().map(|e| format!("{}: {}", e.error.code, e.error.describe())).collect();
    match ex.0.body.as_ref() {
        Some(b) => (codes, Some(ck_topo::mass::mass_props(b).volume), ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect()),
        None => (codes, None, vec![]),
    }
}

/// An arc about (cx, cy) from a to b (counter-clockwise), through the sketch points.
fn arc(cx: f64, cy: f64, a: (f64, f64), b: (f64, f64)) -> (String, String) {
    (
        format!(r#"{{"id":"c","x":{cx},"y":{cy}}},{{"id":"a","x":{},"y":{}}},{{"id":"b","x":{},"y":{}}}"#, a.0, a.1, b.0, b.1),
        r#"{"type":"arc","id":"e1","c":"c","p1":"a","p2":"b"}"#.to_string(),
    )
}

/// Volume of the region between a sphere of radius r and r + t (t < 0: inside), polar angles th0..th1 from +y, revolved by `ang`.
fn sphere_zone(r: f64, t: f64, th0: f64, th1: f64, ang: f64) -> f64 {
    let (r0, r1) = (r.min(r + t), r.max(r + t));
    ang / 3.0 * (r1.powi(3) - r0.powi(3)) * (th0.cos() - th1.cos())
}

fn check(name: &str, got: (Vec<String>, Option<f64>, Vec<String>), want: f64) {
    let (codes, v, issues) = got;
    assert!(codes.is_empty(), "{name}: refused {codes:?}");
    assert!(issues.is_empty(), "{name}: invalid {issues:?}");
    let v = v.unwrap();
    assert!((v - want).abs() < 1e-6 * want, "{name}: volume {v} want {want}");
}

#[test]
fn sphere_cap_full_and_half_turn() {
    // r 10 cap from the pole to polar angle 60 (sketch: arc from (8.66, 5) ccw to the pole (0, 10))
    let (p, e) = arc(0.0, 0.0, (8.660254037844386, 5.0), (0.0, 10.0));
    let th = (0.0, PI / 3.0);
    check("cap 360 front", run(&p, &e, 360.0, 1.0, "front"), sphere_zone(10.0, 1.0, th.0, th.1, 2.0 * PI));
    check("cap 360 back", run(&p, &e, 360.0, 1.0, "back"), sphere_zone(10.0, -1.0, th.0, th.1, 2.0 * PI));
    check("cap 180 front", run(&p, &e, 180.0, 1.0, "front"), sphere_zone(10.0, 1.0, th.0, th.1, PI));
    check("cap 270 both", run(&p, &e, 270.0, 1.0, "both"), (1.5 * PI) / 3.0 * (10.5f64.powi(3) - 9.5f64.powi(3)) * (1.0 - th.1.cos()));
    // the solidcore number for the full cap
    let (_, v, _) = run(&p, &e, 360.0, 1.0, "front");
    assert!((v.unwrap() - 331.0 * PI / 3.0).abs() < 1e-6, "{v:?}");
}

#[test]
fn hemisphere_half_ball_and_band() {
    // hemisphere sheet: equator (10, 0) to the pole
    let (p, e) = arc(0.0, 0.0, (10.0, 0.0), (0.0, 10.0));
    check("hemisphere 360", run(&p, &e, 360.0, 1.0, "front"), sphere_zone(10.0, 1.0, 0.0, PI / 2.0, 2.0 * PI));
    // half ball sheet: pole to pole, half a turn
    let (p, e) = arc(0.0, 0.0, (0.0, -10.0), (0.0, 10.0));
    check("half ball 180", run(&p, &e, 180.0, 1.0, "front"), sphere_zone(10.0, 1.0, 0.0, PI, PI));
    // band between polar angles 30 and 60, no pole
    let s3 = 3f64.sqrt() * 5.0;
    let (p, e) = arc(0.0, 0.0, (s3, 5.0), (5.0, s3));
    for (ang, deg) in [(2.0 * PI, 360.0), (PI, 180.0), (1.5 * PI, 270.0)] {
        check(&format!("band {deg}"), run(&p, &e, deg, 1.0, "front"), sphere_zone(10.0, 1.0, PI / 6.0, PI / 3.0, ang));
    }
}

/// Pappus volume of the quadrilateral / annular region between a profile and its offset, swept by `ang`:
/// `pts(s, k)` is the k-th of n + 1 points of the curve offset by s; the region is closed by the end normals.
fn pappus(n: usize, pts: &dyn Fn(f64, usize) -> (f64, f64), s0: f64, s1: f64, ang: f64) -> f64 {
    let mut poly: Vec<(f64, f64)> = (0..=n).map(|k| pts(s0, k)).collect();
    poly.extend((0..=n).rev().map(|k| pts(s1, k)));
    // first moment about the axis (x = radius): integral of x over the region
    let mut m = 0.0;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        m += (a.0 * a.0 + a.0 * b.0 + b.0 * b.0) * (b.1 - a.1) / 6.0;
    }
    (m * ang).abs()
}

#[test]
fn cone_and_torus_arc_sheets() {
    // cone: line (10, 0) - (5, 10); the wall goes to one side for front and the other for back
    let pts = r#"{"id":"a","x":10,"y":0},{"id":"b","x":5,"y":10}"#;
    let ent = r#"{"type":"line","id":"e1","p1":"a","p2":"b"}"#;
    let len = 125f64.sqrt();
    let n = (10.0 / len, 5.0 / len); // a normal of the line (outward from the axis)
    let line = |s: f64, k: usize| {
        let u = k as f64 / 4.0;
        (10.0 - 5.0 * u + n.0 * s, 10.0 * u + n.1 * s)
    };
    for (deg, ang) in [(360.0, 2.0 * PI), (180.0, PI)] {
        let (out, inn) = (pappus(4, &line, 0.0, 1.0, ang), pappus(4, &line, 0.0, -1.0, ang));
        let (f, b) = (run(pts, ent, deg, 1.0, "front"), run(pts, ent, deg, 1.0, "back"));
        let (vf, vb) = (f.1.unwrap_or(0.0), b.1.unwrap_or(0.0));
        let want_f = if (vf - out).abs() < (vf - inn).abs() { out } else { inn };
        let want_b = if want_f == out { inn } else { out };
        check(&format!("cone {deg} front"), f, want_f);
        check(&format!("cone {deg} back"), b, want_b);
        let _ = vb;
    }
    // torus-like arc through (20, -3), (23, 0), (20, 3): centre (20, 0), r 3; the wall is the half annulus r 3..4
    // (front, outward from the tube) or 2..3 (back), revolved: V = ang (20 pi/2 (b^2 - a^2) + 2/3 (b^3 - a^3))
    let (p, e) = arc(20.0, 0.0, (20.0, -3.0), (20.0, 3.0));
    let half_annulus = |a: f64, b: f64, ang: f64| ang * (20.0 * PI / 2.0 * (b * b - a * a) + 2.0 / 3.0 * (b.powi(3) - a.powi(3)));
    for (deg, ang) in [(360.0, 2.0 * PI), (180.0, PI)] {
        check(&format!("torus arc {deg} front"), run(&p, &e, deg, 1.0, "front"), half_annulus(3.0, 4.0, ang));
        check(&format!("torus arc {deg} back"), run(&p, &e, deg, 1.0, "back"), half_annulus(2.0, 3.0, ang));
    }
}
