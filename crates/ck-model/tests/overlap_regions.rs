//! Crossing / touching closed loops in ONE sketch: the default (no `regions`) cut / add uses the
//! union of every enclosed region (the arrangement of two crossing circles has three regions: two
//! crescents and a lens), not a body of touching lumps.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

/// (errors, volume, validate issues)
fn run(src: &str) -> (Vec<String>, f64, usize) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    match ex.brep.body.as_ref() {
        Some(b) => (errs, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).len()),
        None => (errs, 0.0, 0),
    }
}

const PLATE_V: f64 = 600.0 * 450.0 * 10.0;

/// 10 mm plate (-300..300 x -150..300) plus a second sketch with `circles` (cx, cy, r) and an
/// extrude `op` of it (`extra` = more feature fields).
fn plate(circles: &[(f64, f64, f64)], op: &str, extra: &str) -> String {
    let pts: Vec<String> = circles.iter().enumerate().map(|(i, c)| format!(r#"{{"id":"c{i}","x":{},"y":{}}}"#, c.0, c.1)).collect();
    let ents: Vec<String> = circles.iter().enumerate().map(|(i, c)| format!(r#"{{"type":"circle","id":"h{i}","c":"c{i}","r":{}}}"#, c.2)).collect();
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
 {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":-300,"y":-150}},{{"id":"p2","x":300,"y":-150}},{{"id":"p3","x":300,"y":300}},{{"id":"p4","x":-300,"y":300}}],
  "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"{op}","sketch":"F3","end":"{}","depth":10{extra}}}]}}"#,
        pts.join(","),
        ents.join(","),
        if op == "cut" { "throughBoth" } else { "blind" }
    )
}

fn disc_union_area(circles: &[(f64, f64, f64)]) -> f64 {
    disc_union_area_n(circles, 200000)
}

fn disc_union_area_n(circles: &[(f64, f64, f64)], n: usize) -> f64 {
    // exact for <= 3 circles via inclusion-exclusion is overkill; use fine scan-line numeric integration
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for c in circles {
        lo = lo.min(c.0 - c.2);
        hi = hi.max(c.0 + c.2);
    }
    let dx = (hi - lo) / n as f64;
    let mut a = 0.0;
    for i in 0..n {
        let x = lo + (i as f64 + 0.5) * dx;
        let mut iv: Vec<(f64, f64)> = circles.iter().filter(|c| (x - c.0).abs() < c.2).map(|c| {
            let h = (c.2 * c.2 - (x - c.0).powi(2)).sqrt();
            (c.1 - h, c.1 + h)
        }).collect();
        iv.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap());
        let mut cur = f64::MIN;
        for (s, e) in iv {
            let s = s.max(cur);
            if e > s {
                a += e - s;
                cur = e;
            }
        }
    }
    a * dx
}

const TWO: [(f64, f64, f64); 2] = [(185.0, 74.55, 11.0), (165.469, 82.7345, 11.0)];
const T: f64 = 10.0;

/// Volume removed from the plate by a throughBoth cut of the circles (one sketch), checked valid.
fn removed(circles: &[(f64, f64, f64)], extra: &str) -> f64 {
    let (e, v, iss) = run(&plate(circles, "cut", extra));
    assert!(e.is_empty(), "{e:?}");
    assert_eq!(iss, 0, "validate");
    PLATE_V - v
}

/// Volume of the circles added alone (blind 10) as one sketch.
fn added(circles: &[(f64, f64, f64)]) -> f64 {
    let s = plate(circles, "add", "");
    // drop the plate (F1, F2): keep the F3 sketch and F4 add
    let i = s.find(r#" {"type":"sketch","id":"F3""#).unwrap();
    let head = &s[..s.find(r#" {"type":"sketch","id":"F1""#).unwrap()];
    let src = format!("{head}{}", &s[i..]);
    let (e, v, iss) = run(&src);
    assert!(e.is_empty(), "{e:?}");
    assert_eq!(iss, 0, "validate");
    v
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-3, "{a} vs {b}");
}

#[test]
fn repro_two_crossing_circles_cut_is_the_union() {
    close(removed(&TWO, ""), 7569.796);
    close(removed(&TWO, ""), disc_union_area(&TWO) * T);
}

#[test]
fn two_crossing_circles_add_is_the_union() {
    close(added(&TWO), disc_union_area(&TWO) * T);
}

#[test]
fn three_crossing_circles() {
    let c = [(0.0, 0.0, 11.0), (14.0, 0.0, 11.0), (7.0, 12.0, 11.0)];
    close(removed(&c, ""), disc_union_area(&c) * T);
    close(added(&c), disc_union_area(&c) * T);
    // a chain of three, the outer two not touching each other
    let c = [(0.0, 0.0, 8.0), (12.0, 0.0, 8.0), (24.0, 0.0, 8.0)];
    close(removed(&c, ""), disc_union_area(&c) * T);
    close(added(&c), disc_union_area(&c) * T);
}

#[test]
fn tangent_circles() {
    let c = [(0.0, 0.0, 10.0), (20.0, 0.0, 10.0)];
    close(removed(&c, ""), 2.0 * PI * 100.0 * T);
    close(added(&c), 2.0 * PI * 100.0 * T);
}

#[test]
fn circle_partly_outside_the_plate_outline() {
    // second circle crosses the plate edge x = 300
    let c = [(290.0, 0.0, 11.0), (300.0, 8.0, 11.0)];
    let inside = |cs: &[(f64, f64, f64)]| {
        // union area clipped to x <= 300 by numeric scan
        let n = 200000;
        let (lo, hi) = (270.0, 300.0);
        let dx = (hi - lo) / n as f64;
        let mut a = 0.0;
        for i in 0..n {
            let x = lo + (i as f64 + 0.5) * dx;
            let mut iv: Vec<(f64, f64)> = cs.iter().filter(|c| (x - c.0).abs() < c.2).map(|c| {
                let h = (c.2 * c.2 - (x - c.0).powi(2)).sqrt();
                (c.1 - h, c.1 + h)
            }).collect();
            iv.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap());
            let mut cur = f64::MIN;
            for (s, e) in iv {
                let s = s.max(cur);
                if e > s {
                    a += e - s;
                    cur = e;
                }
            }
        }
        a * dx
    };
    close(removed(&c, ""), inside(&c) * T);
}

#[test]
fn nested_circles_stay_a_ring() {
    // ring r 5..11 and a crossing neighbour is not involved: a circle inside a circle cuts the ring only
    let c = [(0.0, 0.0, 11.0), (0.0, 0.0, 5.0)];
    close(removed(&c, ""), PI * (121.0 - 25.0) * T);
    close(added(&c), PI * (121.0 - 25.0) * T);
}

#[test]
fn explicit_regions_still_select_individual_regions() {
    // the lens only: both circle ids bound it, crescents are each bounded by one of them plus the lens arc
    let lens = removed(&TWO, r#","regions":["h0","h1"]"#);
    let all = disc_union_area(&TWO) * T;
    assert!(lens > 0.0 && lens <= all + 1e-6, "{lens} {all}");
}

/// The corpus case (tools/cases/overlap_regions): a plate cut by two crossing holes of one sketch.
#[test]
fn corpus_case_removes_the_union() {
    let (e, v, iss) = run(include_str!("../../../tools/cases/overlap_regions/crossing_circles_cut.json"));
    assert!(e.is_empty(), "{e:?}");
    assert_eq!(iss, 0);
    close(PLATE_V - v, 7569.796);
}

/// Seeded sweep of 2-4 random circles of one sketch (some crossing, some nested-free), cut and add:
/// the volume must be the union of the discs, never a silent wrong one. Prints failing configs.
#[test]
fn random_circle_clusters_match_the_union() {
    let mut s: u64 = 0x2545F4914F6CDD1D;
    let mut rnd = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut bad = vec![];
    for k in 0..24 {
        let n = 2 + (k % 3);
        let mut cs: Vec<(f64, f64, f64)> = vec![];
        'gen: loop {
            cs.clear();
            for _ in 0..n {
                cs.push((20.0 * rnd(), 20.0 * rnd(), 4.0 + 6.0 * rnd()));
            }
            // reject nested / near-tangent pairs: those have their own tests
            for i in 0..n {
                for j in 0..i {
                    let d = ((cs[i].0 - cs[j].0).powi(2) + (cs[i].1 - cs[j].1).powi(2)).sqrt();
                    if d < (cs[i].2 - cs[j].2).abs() + 0.5 || (d - (cs[i].2 + cs[j].2)).abs() < 0.3 {
                        continue 'gen;
                    }
                }
            }
            break;
        }
        let want = disc_union_area_n(&cs, 20000) * T;
        let cut = std::panic::catch_unwind(|| removed(&cs, ""));
        let add = std::panic::catch_unwind(|| added(&cs));
        for (what, got) in [("cut", cut), ("add", add)] {
            match got {
                Ok(v) if (v - want).abs() < 2e-3 * want => {}
                Ok(v) => bad.push(format!("{what} {cs:?}: {v} vs {want}")),
                Err(_) => bad.push(format!("{what} {cs:?}: error")),
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// Overlapping rectangles and a circle crossing a rectangle edge, all in one sketch.
#[test]
fn crossing_rectangles_and_circle_cut_the_union() {
    // rect A (0,0)-(30,20), rect B (20,10)-(50,40): overlap 10x10; circle r6 at (50,15) crosses B's right edge
    let src = |op: &str| {
        let plate = if op == "cut" {
            r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"q1","x":-100,"y":-100},{"id":"q2","x":100,"y":-100},{"id":"q3","x":100,"y":100},{"id":"q4","x":-100,"y":100}],
  "entities":[{"type":"line","id":"f1","p1":"q1","p2":"q2"},{"type":"line","id":"f2","p1":"q2","p2":"q3"},{"type":"line","id":"f3","p1":"q3","p2":"q4"},{"type":"line","id":"f4","p1":"q4","p2":"q1"}],"constraints":[]},
 {"type":"extrude","id":"F0","op":"add","sketch":"F1","end":"blind","depth":10},"#
        } else {
            ""
        };
        format!(
            r#"{{"format":"ck-part","version":1,"features":[{plate}
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[
  {{"id":"a1","x":0,"y":0}},{{"id":"a2","x":30,"y":0}},{{"id":"a3","x":30,"y":20}},{{"id":"a4","x":0,"y":20}},
  {{"id":"b1","x":20,"y":10}},{{"id":"b2","x":50,"y":10}},{{"id":"b3","x":50,"y":40}},{{"id":"b4","x":20,"y":40}},{{"id":"c","x":50,"y":15}}],
  "entities":[
  {{"type":"line","id":"a_1","p1":"a1","p2":"a2"}},{{"type":"line","id":"a_2","p1":"a2","p2":"a3"}},{{"type":"line","id":"a_3","p1":"a3","p2":"a4"}},{{"type":"line","id":"a_4","p1":"a4","p2":"a1"}},
  {{"type":"line","id":"b_1","p1":"b1","p2":"b2"}},{{"type":"line","id":"b_2","p1":"b2","p2":"b3"}},{{"type":"line","id":"b_3","p1":"b3","p2":"b4"}},{{"type":"line","id":"b_4","p1":"b4","p2":"b1"}},
  {{"type":"circle","id":"h","c":"c","r":6}}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"{op}","sketch":"F3","end":"{}","depth":10}}]}}"#,
            if op == "cut" { "throughBoth" } else { "blind" }
        )
    };
    // union area: 600 + 900 - 100 + circle part outside B and A: circle centre on B's edge (x=50, y 15 in 10..40):
    // the right half lies outside B, and so does half the segment below B's bottom edge (y = 10, 5 from the centre)
    let seg = 36.0 * (5.0f64 / 6.0).acos() - 5.0 * 11.0f64.sqrt();
    let want = (600.0 + 900.0 - 100.0 + PI * 36.0 / 2.0 + seg / 2.0) * T;
    for op in ["cut", "add"] {
        let (e, v, iss) = run(&src(op));
        assert!(e.is_empty(), "{op} {e:?}");
        assert_eq!(iss, 0, "{op}");
        let got = if op == "cut" { PLATE2 - v } else { v };
        assert!((got - want).abs() < 1e-3 * want, "{op}: {got} vs {want}");
    }
}

const PLATE2: f64 = 200.0 * 200.0 * 10.0;
