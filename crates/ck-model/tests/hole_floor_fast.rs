//! `exits_at_floor` (a blind hole whose floor lies on the far side of the body) used to run `classify` twice per centre, and
//! `classify` runs a winding number over every face once the point is not near a face: seconds per call on a body of spline
//! faces, repeated on every rebuild (a 488-face imported part did not open in two minutes). The test pins the new check to
//! the old one on bodies of planes and splines, then bounds the time of a blind hole on a many-face spline body.
#[path = "robust_common/mod.rs"]
mod common;

use ck_core::linalg::Vec3;
use ck_model::features::hole::exits_at_floor;
use ck_topo::body::Body;
use ck_topo::region::{classify, Containment};

/// The check as it was: full `classify` at depth -+ eps.
fn exits_old(body: &Body, at: Vec3, down: Vec3, depth: f64) -> bool {
    let (lo, hi) = ck_model::features::body_box(body);
    let eps = lo.dist(hi) * 1e-6;
    let class = |h: f64| classify(body, at + down * h, 0.0);
    class(depth - eps) == Containment::Inside && class(depth + eps) == Containment::Outside
}

fn rect(id: &str, x0: f64, x1: f64, w: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[
          {{"id":"p1","x":{x0},"y":-{w}}},{{"id":"p2","x":{x1},"y":-{w}}},{{"id":"p3","x":{x1},"y":{w}}},{{"id":"p4","x":{x0},"y":{w}}}],
         "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}
/// 100 x 40 x 30 block, z = 0..30; with a step (z = 10 floor over x > 0) when `step`.
fn block(step: bool) -> String {
    let mut s = format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}}"#, rect("F1", -50.0, 50.0, 20.0));
    if step {
        s += &format!(r#",{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":10,"flip":true}}"#, rect("F3", 0.0, 60.0, 25.0));
    }
    s
}
/// A closed loop of `n` three-point spline edges around a wavy ring (radius about 20 mm), extruded `h` mm: `n` spline side faces.
fn blob(n: usize, h: f64) -> String {
    let m = 2 * n;
    let pt = |i: usize| {
        let t = std::f64::consts::TAU * (i % m) as f64 / m as f64;
        let r = 20.0 + 3.0 * (5.0 * t).sin();
        (r * t.cos(), r * t.sin())
    };
    let pts: Vec<String> = (0..m).map(|i| format!(r#"{{"id":"p{i}","x":{:.6},"y":{:.6}}}"#, pt(i).0, pt(i).1)).collect();
    let ents: Vec<String> = (0..n).map(|k| format!(r#"{{"type":"spline","id":"e{k}","pts":["p{}","p{}","p{}"]}}"#, 2 * k, 2 * k + 1, (2 * k + 2) % m)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},
        {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#,
        pts.join(","),
        ents.join(",")
    )
}
fn part(feats: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#)
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// New against old on `samples` random centres, with depths far from and within a few eps of the body's faces at `planes`.
fn compare(label: &str, body: &Body, planes: &[f64], samples: usize, span: f64) {
    let (lo, hi) = ck_model::features::body_box(body);
    let eps = lo.dist(hi) * 1e-6;
    let down = Vec3::new(0.0, 0.0, 1.0);
    let mut rng = Rng(0x9E3779B97F4A7C15);
    let (mut yes, mut n) = (0, 0);
    for i in 0..samples {
        let at = Vec3::new(lo.x + rng.next() * (hi.x - lo.x), lo.y + rng.next() * (hi.y - lo.y), 0.0);
        let depth = match i % 3 {
            0 => rng.next() * (hi.z + 5.0),
            _ => {
                let pl = planes[(rng.next() * planes.len() as f64) as usize % planes.len()];
                pl + [-6.0, -3.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 3.0, 6.0][(rng.next() * 11.0) as usize % 11] * eps * span
            }
        };
        let (a, b) = (exits_at_floor(body, at, down, depth), exits_old(body, at, down, depth));
        assert_eq!(a, b, "{label}: centre {at:?} depth {depth}");
        n += 1;
        yes += a as usize;
    }
    println!("{label}: {n} samples, {yes} exits");
}

#[test]
fn same_answer_as_the_full_classify_on_planar_bodies() {
    for (label, step, planes) in [("block", false, vec![0.0, 30.0]), ("stepped", true, vec![0.0, 10.0, 30.0])] {
        let b = common::regen_body(&part(&block(step))).expect("body");
        compare(label, &b, &planes, 300, 1.0);
        // exactly at a face and a hair either side: the cases the check is for
        for &pl in &planes {
            for k in [-2.0f64, -1.0, 0.0, 1.0, 2.0] {
                let (lo, hi) = ck_model::features::body_box(&b);
                let eps = lo.dist(hi) * 1e-6;
                for x in [-30.0, 30.0, -10.0] {
                    let at = Vec3::new(x, 0.0, 0.0);
                    let down = Vec3::new(0.0, 0.0, 1.0);
                    let d = pl + k * eps;
                    assert_eq!(exits_at_floor(&b, at, down, d), exits_old(&b, at, down, d), "{label}: x {x} depth {d}");
                }
            }
        }
    }
    // the step floor from inside the solid half does not exit; the far side does
    let b = common::regen_body(&part(&block(true))).unwrap();
    let down = Vec3::new(0.0, 0.0, 1.0);
    assert!(exits_at_floor(&b, Vec3::new(-30.0, 0.0, 0.0), down, 30.0));
    assert!(!exits_at_floor(&b, Vec3::new(-30.0, 0.0, 0.0), down, 15.0));
    assert!(exits_at_floor(&b, Vec3::new(30.0, 0.0, 0.0), down, 30.0));
}

#[test]
fn same_answer_as_the_full_classify_on_a_spline_body() {
    let b = common::regen_body(&part(&blob(12, 30.0))).expect("body");
    assert!(b.face_ids().count() >= 12);
    compare("blob", &b, &[0.0, 30.0], 45, 1.0);
    let down = Vec3::new(0.0, 0.0, 1.0);
    assert!(exits_at_floor(&b, Vec3::new(0.0, 0.0, 0.0), down, 30.0));
    assert!(!exits_at_floor(&b, Vec3::new(0.0, 0.0, 0.0), down, 12.0));
}

const N: usize = 120;

/// A blind hole in a body of many spline faces regenerates in a bounded time (the 488-face part spent 76 % of its open in this check).
#[test]
fn blind_hole_in_a_many_face_spline_body_is_fast() {
    // 24 centres on a grid inside the ring (radius 17 mm at the most), all blind with a floor in the material
    let xs: Vec<(f64, f64)> = (0..24).map(|i| (-9.0 + 3.6 * (i % 6) as f64, -6.0 + 4.0 * (i / 6) as f64)).collect();
    let pts: Vec<String> = xs.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"c{i}","x":{x},"y":{y}}}"#)).collect();
    let ents: Vec<String> = (0..xs.len()).map(|i| format!(r#"{{"type":"point","id":"q{i}","p":"c{i}"}}"#)).collect();
    let hole = format!(
        r#",{{"type":"sketch","id":"HS","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},
        {{"type":"hole","id":"H","sketch":"HS","kind":"simple","diameter":2,"flip":true,"end":"blind","depth":12}}"#,
        pts.join(","),
        ents.join(",")
    );
    let src = part(&format!("{}{hole}", blob(N, 30.0)));
    let t = std::time::Instant::now();
    let (issues, vol) = common::regen_validate(&src);
    let secs = t.elapsed().as_secs_f64();
    assert!(issues.is_empty(), "{issues:?}");
    let base = common::regen_validate(&part(&blob(N, 30.0))).1.unwrap();
    let removed = base - vol.unwrap();
    let want = 24.0 * std::f64::consts::PI * 1.0 * 1.0 * 12.0;
    assert!((removed / want - 1.0).abs() < 0.02, "removed {removed} vs {want}");
    println!("24 centres x 12 mm in a {N}-face spline body: {secs:.2}s");
    assert!(secs < 10.0, "took {secs:.1}s");
}
