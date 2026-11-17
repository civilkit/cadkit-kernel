//! `pressPull` (a flat face continued straight, with draft or scale taper) and the `faceRun` query: analytic volumes,
//! persistent names, plain refusals that leave the body unchanged.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_core::linalg::Vec3;
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

struct Run {
    part: Part,
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
    fn bbox(&self) -> ([f64; 3], [f64; 3]) {
        let (a, b) = self.body.as_ref().unwrap().bbox();
        ([a.x, a.y, a.z], [b.x, b.y, b.z])
    }
    fn valid(&self) -> bool {
        ck_check::validate(self.body.as_ref().unwrap(), 1e-6).is_empty()
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { part, result, body: ex.brep.body.clone(), volume }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}


const BLOCK: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// Cylinder of radius 5, height 30.
const ROD: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#;

/// The block with a 6 mm hole through it at (20, 10).
const HOLED: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":20,"y":10}],"entities":[{"type":"circle","id":"c1","c":"c","r":3}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":0}"#;

/// The block with a 20 x 10 pocket 6 mm deep from its y = 0 face: the pocket ceiling at y = 6 faces -y.
const POCKET: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"q1","x":10,"y":5},{"id":"q2","x":30,"y":5},{"id":"q3","x":30,"y":15},{"id":"q4","x":10,"y":15}],
  "entities":[{"type":"line","id":"g1","p1":"q1","p2":"q2"},{"type":"line","id":"g2","p1":"q2","p2":"q3"},{"type":"line","id":"g3","p1":"q3","p2":"q4"},{"type":"line","id":"g4","p1":"q4","p2":"q1"}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":6,"flip":true}"#;

/// A T-slot bar profile (a 20 x 4 flange on a 6 x 10 stem), 30 mm long.
const TBAR: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":20,"y":0},{"id":"c","x":20,"y":4},{"id":"d","x":13,"y":4},{"id":"e","x":13,"y":14},{"id":"f","x":7,"y":14},{"id":"g","x":7,"y":4},{"id":"h","x":0,"y":4}],
  "entities":[{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"},{"type":"line","id":"l3","p1":"c","p2":"d"},{"type":"line","id":"l4","p1":"d","p2":"e"},{"type":"line","id":"l5","p1":"e","p2":"f"},{"type":"line","id":"l6","p1":"f","p2":"g"},{"type":"line","id":"l7","p1":"g","p2":"h"},{"type":"line","id":"l8","p1":"h","p2":"a"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#;

fn with(base: &str, more: &str) -> Run {
    run(&format!("{base},{more}"))
}

fn pp(faces: &str, distance: f64, taper: &str) -> String {
    let t = if taper.is_empty() { String::new() } else { format!(r#","taper":{taper}"#) };
    format!(r#"{{"type":"pressPull","id":"F9","faces":[{faces}],"continue":"straight","distance":{distance}{t}}}"#)
}
fn scale(k: f64) -> String {
    format!(r#"{{"type":"scale","endScale":{k}}}"#)
}
fn draft(a: f64) -> String {
    format!(r#"{{"type":"draft","angle":{a}}}"#)
}

/// The planar face with outward normal along `axis` (0 x, 1 y, 2 z) of sign `sgn`, at coordinate `at`.
fn plane_at(r: &Run, axis: usize, sgn: f64, at: f64) -> String {
    let b = r.body.as_ref().unwrap();
    for f in b.face_ids() {
        if let ck_geom::surface::Surface::Plane { frame } = &b.face(f).surface {
            let n = b.planar_normal(f);
            let c = [n.x, n.y, n.z][axis];
            let o = [frame.o.x, frame.o.y, frame.o.z][axis];
            if c * sgn > 0.999999 && (o - at).abs() < 1e-9 {
                return b.face_id_str(f);
            }
        }
    }
    panic!("no plane at {at}: {:?}", r.faces());
}

fn q(s: &str) -> String {
    format!("\"{s}\"")
}

fn has_error(r: &Run, code: &str) -> bool {
    r.errors().iter().any(|e| e.contains(code))
}

const TAN5: f64 = 0.087_488_663_525_923_99; // tan(5 degrees)

#[test]
fn names_of_the_fixtures() {
    for (n, s) in [("block", BLOCK), ("rod", ROD), ("holed", HOLED), ("pocket", POCKET), ("tbar", TBAR)] {
        let r = run(s);
        assert!(r.errors().is_empty() && r.valid(), "{n}: {:?}", r.errors());
    }
}

#[test]
fn straight_rod_end_out_and_in() {
    let base = run(ROD);
    let r = with(ROD, &pp(&q("F2.C1"), 10.0, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, PI * 25.0 * 40.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    assert_eq!(r.faces(), base.faces(), "the walls merge, the moved end keeps its name");
    assert!(close(r.bbox().1[1], 40.0, 1e-9));
    let r = with(ROD, &pp(&q("F2.C1"), -10.0, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, PI * 25.0 * 20.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    assert_eq!(r.faces(), base.faces());
}

#[test]
fn rod_scale_is_a_frustum() {
    let (h, r0, k) = (10.0, 5.0, 1.1);
    let r = with(ROD, &pp(&q("F2.C1"), h, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let frustum = PI * h / 3.0 * (r0 * r0 + r0 * r0 * k + r0 * r0 * k * k);
    assert!(close(r.volume, PI * 25.0 * 30.0 + frustum, 1e-6), "{} vs {}", r.volume, PI * 25.0 * 30.0 + frustum);
    assert!(r.valid());
    // the moved end keeps its name; the cone is the one new wall, named from the cylinder it continues
    let mut want = run(ROD).faces();
    want.push("F9.S.F2_S_c1".into());
    assert_eq!(r.faces(), want);
    // the moved end is the larger disc
    let b = r.body.as_ref().unwrap();
    assert!(b.find_face("F2.C1").is_some());
    assert!(close(r.bbox().1[0] - r.bbox().0[0], 11.0, 1e-6), "{:?}", r.bbox());
    assert!(close(r.bbox().1[1], 40.0, 1e-9));
}

#[test]
fn rod_draft_and_scale_in() {
    // outward draft 5 degrees over 10 mm
    let r = with(ROD, &pp(&q("F2.C1"), 10.0, &draft(5.0)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let r1 = 5.0 + 10.0 * TAN5;
    let frustum = PI * 10.0 / 3.0 * (25.0 + 5.0 * r1 + r1 * r1);
    assert!(close(r.volume, PI * 25.0 * 30.0 + frustum, 1e-6), "{} vs {}", r.volume, PI * 25.0 * 30.0 + frustum);
    assert!(r.valid());
    // pulling the end into the rod with an outward lean leaves a larger bore than the rod: through the wall, refused
    let r = with(ROD, &pp(&q("F2.C1"), -10.0, &scale(1.1)));
    assert!(has_error(&r, "moveFaceFailed"), "{:?}", r.errors());
    assert!(close(r.volume, PI * 25.0 * 30.0, 1e-9));
}

#[test]
fn block_draft_matches_the_prism_formula() {
    let top = plane_at(&run(BLOCK), 1, 1.0, 10.0);
    let (d, m) = (5.0, TAN5);
    let r = with(BLOCK, &pp(&q(&top), d, &draft(5.0)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let add = 800.0 * d + 120.0 * m * d * d / 2.0 + 4.0 * m * m * d * d * d / 3.0;
    assert!(close(r.volume, 8000.0 + add, 1e-6), "{} vs {}", r.volume, 8000.0 + add);
    assert!(r.valid());
    assert!(r.faces().contains(&top));
    // new walls are named from the faces they continue
    let new: Vec<String> = r.faces().into_iter().filter(|n| n.starts_with("F9.")).collect();
    assert_eq!(new, ["F9.S.F2_S_e1", "F9.S.F2_S_e2", "F9.S.F2_S_e3", "F9.S.F2_S_e4"], "tilted walls do not merge with the vertical ones");
    // an inward lean shrinks it
    let r = with(BLOCK, &pp(&q(&top), d, &draft(-5.0)));
    let add = 800.0 * d - 120.0 * m * d * d / 2.0 + 4.0 * m * m * d * d * d / 3.0;
    assert!(close(r.volume, 8000.0 + add, 1e-6), "{} vs {}", r.volume, 8000.0 + add);
}

#[test]
fn holed_face_scale_and_draft_take_the_hole_along() {
    let top = plane_at(&run(HOLED), 1, 1.0, 10.0);
    let a0 = 800.0 - 9.0 * PI;
    let v0 = a0 * 10.0;
    let (d, k) = (5.0, 1.1);
    let r = with(HOLED, &pp(&q(&top), d, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, v0 + a0 * d * (1.0 + k + k * k) / 3.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    // draft outward: the outline grows, the hole shrinks
    let m = TAN5;
    let r = with(HOLED, &pp(&q(&top), d, &draft(5.0)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let outer = 800.0 * d + 120.0 * m * d * d / 2.0 + 4.0 * m * m * d * d * d / 3.0;
    let hole = PI * (9.0 * d - 6.0 * m * d * d / 2.0 + m * m * d * d * d / 3.0);
    assert!(close(r.volume, v0 + outer - hole, 1e-6), "{} vs {}", r.volume, v0 + outer - hole);
    assert!(r.valid());
}

#[test]
fn tbar_end_straight_draft_scale() {
    let r0 = run(TBAR);
    let a0 = 20.0 * 4.0 + 6.0 * 10.0;
    assert!(close(r0.volume, a0 * 30.0, 1e-9));
    let end = plane_at(&r0, 1, 1.0, 30.0);
    // straight, out and in
    let r = with(TBAR, &pp(&q(&end), 8.0, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, a0 * 38.0, 1e-9));
    assert_eq!(r.faces(), r0.faces());
    let r = with(TBAR, &pp(&q(&end), -8.0, ""));
    assert!(close(r.volume, a0 * 22.0, 1e-9));
    // scale about the area centre: a section at run length t has area a0 * s(t)^2
    let (d, k) = (6.0, 0.8);
    let r = with(TBAR, &pp(&q(&end), d, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, a0 * 30.0 + a0 * d * (1.0 + k + k * k) / 3.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    // the tool of a draft on a polygon with a re-entrant corner: all corners lean out
    let r = with(TBAR, &pp(&q(&end), 4.0, &draft(3.0)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.volume > a0 * 34.0, "{}", r.volume);
    assert!(r.valid());
}

#[test]
fn cut_by_a_tapered_piece_conserves_volume() {
    let r0 = run(POCKET);
    let v0 = r0.volume;
    assert!(close(v0, 40.0 * 20.0 * 20.0 - 200.0 * 6.0, 1e-9), "{v0}");
    let ceiling = plane_at(&r0, 1, -1.0, 6.0);
    // the pocket ceiling pulled 5 mm up into the material, narrowing to 80 %
    let (d, k) = (5.0, 0.8);
    let r = with(POCKET, &pp(&q(&ceiling), -d, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, v0 - 200.0 * d * (1.0 + k + k * k) / 3.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    assert!(r.faces().contains(&ceiling), "{:?}", r.faces());
    // widening to 120 % undercuts the pocket walls: a cavity under the roof
    let k = 1.2;
    let r = with(POCKET, &pp(&q(&ceiling), -d, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, v0 - 200.0 * d * (1.0 + k + k * k) / 3.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    // outward-leaning draft, -3 degrees inward-leaning: the formula of the prism
    let m = 3.0f64.to_radians().tan();
    for (a, s) in [(3.0, 1.0), (-3.0, -1.0)] {
        let w = |t: f64| s * m * t;
        let area = |t: f64| (20.0 + 2.0 * w(t)) * (10.0 + 2.0 * w(t));
        let vp = d / 6.0 * (area(0.0) + 4.0 * area(d / 2.0) + area(d)); // exact for a quadratic
        let r = with(POCKET, &pp(&q(&ceiling), -d, &draft(a)));
        assert!(r.errors().is_empty(), "{a}: {:?}", r.errors());
        assert!(close(r.volume, v0 - vp, 1e-6), "{a}: {} vs {}", r.volume, v0 - vp);
        assert!(r.valid());
    }
}

#[test]
fn pocket_ceiling_pulled_into_the_pocket_with_a_taper() {
    let r0 = run(POCKET);
    let ceiling = plane_at(&r0, 1, -1.0, 6.0);
    // toward the open side, narrowing: joins material inside the pocket
    let (d, k) = (4.0, 0.9);
    let r = with(POCKET, &pp(&q(&ceiling), d, &scale(k)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, r0.volume + 200.0 * d * (1.0 + k + k * k) / 3.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
    // widening would run into the pocket walls
    let r = with(POCKET, &pp(&q(&ceiling), d, &scale(1.1)));
    assert!(has_error(&r, "moveFaceFailed"), "{:?}", r.errors());
    assert!(close(r.volume, r0.volume, 1e-9));
}

#[test]
fn names_survive_a_parameter_change() {
    let top = plane_at(&run(BLOCK), 1, 1.0, 10.0);
    let a = with(BLOCK, &pp(&q(&top), 5.0, &draft(5.0)));
    let b = with(BLOCK, &pp(&q(&top), 9.0, &draft(8.0)));
    assert_eq!(a.faces(), b.faces());
    assert!(a.errors().is_empty() && b.errors().is_empty());
    // a boss sketched on the moved end still finds it
    let tail = format!(
        r#"{},
 {{"type":"sketch","id":"F10","plane":{{"kind":"face","id":"{top}"}},"points":[{{"id":"q","x":0,"y":0}}],"entities":[{{"type":"circle","id":"c1","c":"q","r":2}}],"constraints":[]}},
 {{"type":"extrude","id":"F11","op":"add","sketch":"F10","end":"blind","depth":4}}"#,
        pp(&q(&top), 5.0, "")
    );
    let r = with(BLOCK, &tail);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.bbox().1[1], 19.0, 1e-9), "{:?}", r.bbox());
}

#[test]
fn several_faces_continue_one_after_the_other() {
    // two faces pulled in one feature, each on its own and in order: the wall is 13 high once the top has come up
    let r0 = run(BLOCK);
    let (top, wall) = (plane_at(&r0, 1, 1.0, 10.0), plane_at(&r0, 0, 1.0, 40.0));
    let r = with(BLOCK, &pp(&format!("{},{}", q(&top), q(&wall)), 3.0, ""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 40.0 * 20.0 * 13.0 + 3.0 * 20.0 * 13.0, 1e-6), "{}", r.volume);
    assert!(r.valid());
}

fn unchanged(r: &Run, base: &Run) {
    assert!(close(r.volume, base.volume, 1e-12), "{} vs {}", r.volume, base.volume);
    assert_eq!(r.faces(), base.faces());
}

#[test]
fn refusals_say_why_and_leave_the_body_alone() {
    let base = run(ROD);
    // not flat
    let r = with(ROD, &pp(&q("F2.S.c1"), 5.0, ""));
    assert!(has_error(&r, "pressPullUnsupported"), "{:?}", r.errors());
    assert!(r.errors()[0].contains("flat"), "{:?}", r.errors());
    unchanged(&r, &base);
    // distance 0
    let r = with(ROD, &pp(&q("F2.C1"), 0.0, ""));
    assert!(has_error(&r, "invalidValue") && r.errors()[0].contains("distance"), "{:?}", r.errors());
    unchanged(&r, &base);
    // endScale <= 0
    for k in [0.0, -1.0] {
        let r = with(ROD, &pp(&q("F2.C1"), 5.0, &scale(k)));
        assert!(has_error(&r, "invalidValue") && r.errors()[0].contains("endScale"), "{k}: {:?}", r.errors());
        unchanged(&r, &base);
    }
    // draft angle out of range
    let r = with(ROD, &pp(&q("F2.C1"), 5.0, &draft(86.0)));
    assert!(has_error(&r, "invalidValue") && r.errors()[0].contains("angle"), "{:?}", r.errors());
    // a draft that closes the face before the end of the run: the cone closes at 5 / tan(45) = 5
    let r = with(ROD, &pp(&q("F2.C1"), 8.0, &draft(-45.0)));
    assert!(has_error(&r, "draftTooLarge"), "{:?}", r.errors());
    let e = &r.result.errors[0].error;
    let mh = e.params.iter().find(|(k, _)| k == "maxHeight").and_then(|(_, v)| v.as_f64()).expect("maxHeight");
    assert!(close(mh, 5.0, 1e-6), "{mh}");
    unchanged(&r, &base);
    // a polygon whose taper collapses it
    let blk = run(BLOCK);
    let top = plane_at(&blk, 1, 1.0, 10.0);
    let r = with(BLOCK, &pp(&q(&top), 20.0, &draft(-60.0)));
    assert!(has_error(&r, "draftTooLarge"), "{:?}", r.errors());
    unchanged(&r, &blk);
    // a negative distance that cuts through the body
    let r = with(BLOCK, &pp(&q(&top), -12.0, ""));
    assert!(has_error(&r, "moveFaceFailed"), "{:?}", r.errors());
    let e = &r.result.errors[0].error;
    let ms = e.params.iter().find(|(k, _)| k == "maxSize").expect("maxSize");
    let v = ms.1.get("value").and_then(|v| v.as_f64()).unwrap();
    assert!(v < 0.0 && v >= -10.0, "{v}");
    unchanged(&r, &blk);
    // along is not built yet
    let r = with(ROD, r#"{"type":"pressPull","id":"F9","faces":["F2.C1"],"continue":"along","distance":5}"#);
    assert!(has_error(&r, "invalidValue") && r.errors()[0].contains("continue"), "{:?}", r.errors());
    // no faces, a missing face, no solid
    let r = with(ROD, r#"{"type":"pressPull","id":"F9","faces":[],"distance":5}"#);
    assert!(has_error(&r, "noFaces"), "{:?}", r.errors());
    let r = with(ROD, r#"{"type":"pressPull","id":"F9","faces":["F2.C9"],"distance":5}"#);
    assert!(has_error(&r, "missingReference"), "{:?}", r.errors());
    let r = run(r#"{"type":"pressPull","id":"F9","faces":["F2.C1"],"distance":5}"#);
    assert!(has_error(&r, "nothingToCut"), "{:?}", r.errors());
}

#[test]
fn face_run_reports_centre_normal_reach_and_the_feature_end() {
    let r = run(ROD);
    let b = r.body.as_ref().unwrap();
    let ends = ck_model::features::press_pull::moving_ends(&r.part);
    let msg = ck_json::parse(r#"{"type":"faceRun","faces":["F2.C0","F2.C1","F2.S.c1","F7.C1"]}"#).unwrap();
    let out = ck_model::features::press_pull::face_run(b, &msg, &ends);
    let items = out.get("items").and_then(|i| i.as_arr()).unwrap();
    let num = |j: &ck_json::Json, k: &str| j.get(k).and_then(|v| v.as_f64()).unwrap();
    let vec = |j: &ck_json::Json, k: &str| -> Vec<f64> { j.get(k).and_then(|v| v.as_arr()).unwrap().iter().map(|x| x.as_f64().unwrap()).collect() };
    let c1 = &items[1];
    assert!(close(num(c1, "reach"), 5.0, 1e-9));
    assert!(close(num(c1, "area"), PI * 25.0, 1e-6));
    let (c, n) = (vec(c1, "centre"), vec(c1, "normal"));
    assert!(c[0].abs() < 1e-9 && close(c[1], 30.0, 1e-9) && c[2].abs() < 1e-9, "{c:?}");
    assert!(close(n[1], 1.0, 1e-9));
    assert_eq!(c1.get("endOf").unwrap().get_str("feature"), Some("F2"));
    assert_eq!(c1.get("endOf").unwrap().get_str("input"), Some("depth"));
    // the sketch end does not move with the depth
    assert!(matches!(items[0].get("endOf"), Some(ck_json::Json::Null)));
    assert_eq!(items[2].get("flat"), Some(&ck_json::Json::Bool(false)));
    assert_eq!(items[3].get("found"), Some(&ck_json::Json::Bool(false)));
    // a block face: reach is the half diagonal; after a press pull the end belongs to no feature input
    let blk = run(BLOCK);
    let top = plane_at(&blk, 1, 1.0, 10.0);
    let msg = ck_json::parse(&format!(r#"{{"faces":["{top}"]}}"#)).unwrap();
    let out = ck_model::features::press_pull::face_run(blk.body.as_ref().unwrap(), &msg, &ck_model::features::press_pull::moving_ends(&blk.part));
    let it = &out.get("items").and_then(|i| i.as_arr()).unwrap()[0];
    assert!(close(num(it, "reach"), (20.0f64 * 20.0 + 10.0 * 10.0).sqrt(), 1e-9));
    let c = vec(it, "centre");
    assert!(close(c[0], 20.0, 1e-9) && close(c[2].abs(), 10.0, 1e-9), "{c:?}");
}
