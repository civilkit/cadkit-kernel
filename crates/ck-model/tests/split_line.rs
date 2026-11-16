//! `splitLine` (Split Face): a plane or a sketch line cuts faces of a solid in two; the body keeps its shape and volume.
//!
//! Fixture: a 40 x 20 x 10 box (front plane: x 0..40, y 0..20, z 0..10).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_model::Part;

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
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn full(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn body(&self) -> &ck_topo::body::Body {
        self.body.as_ref().unwrap()
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.body()).volume
    }
    /// Volume of the lump with this name.
    fn named(&self, name: &str) -> f64 {
        let b = self.body();
        let l = b.lump_ids().find(|&l| b.lump(l).name.as_deref() == Some(name)).unwrap_or_else(|| panic!("no body {name}: {:?}", self.names()));
        ck_topo::mass::mass_props(&ck_ops::interference::extract_lump(b, l)).volume
    }
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body();
        b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

const SKETCH: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},
              {"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]}"#;
const EXTRUDE: &str = r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn case(tail: &str) -> Run {
    run(&format!("{SKETCH},{EXTRUDE}{}", if tail.is_empty() { String::new() } else { format!(",{tail}") }))
}

fn valid(r: &Run) {
    let issues = ck_check::validate(r.body(), 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
}

fn faces(r: &Run) -> usize {
    r.body().face_ids().count()
}

fn names(r: &Run) -> Vec<String> {
    let b = r.body();
    let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    v.sort();
    v
}

#[test]
fn plane_splits_one_face_in_two_and_keeps_the_volume() {
    let whole = case("");
    assert!(whole.errors().is_empty(), "{:?}", whole.errors());
    println!("{:?}", names(&whole));
    let v0 = whole.volume();
    let n0 = faces(&whole);
    let r = case(r#"{"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{"kind":"std","id":"right"},"offset":15}"#);
    assert!(r.errors().is_empty(), "{:?}", r.full());
    valid(&r);
    assert_eq!(faces(&r), n0 + 1);
    assert_eq!(r.lumps(), 1);
    assert!((r.volume() - v0).abs() < 1e-9, "{} {v0}", r.volume());
    println!("{:?}", names(&r));
    let pieces: Vec<_> = names(&r).into_iter().filter(|n| n.starts_with("F2.C1")).collect();
    assert_eq!(pieces.len(), 2, "{pieces:?}");
    assert!(pieces.iter().all(|n| n.contains('~')), "{pieces:?}");
}

#[test]
fn two_faces_split_by_one_plane_make_two_extra_faces() {
    let whole = case("");
    let n0 = faces(&whole);
    let r = case(r#"{"type":"splitLine","id":"F3","faces":["F2.C0","F2.C1"],"plane":{"kind":"std","id":"right"},"offset":15}"#);
    assert!(r.errors().is_empty(), "{:?}", r.full());
    valid(&r);
    assert_eq!(faces(&r), n0 + 2);
    assert!((r.volume() - whole.volume()).abs() < 1e-9);
}

#[test]
fn a_sketch_line_projected_on_the_face_splits_it() {
    let whole = case("");
    let n0 = faces(&whole);
    // a line across the top face at x = 12 (the sketch is on the front plane at z = 0; the projection runs along the face normal)
    let sk = r#"{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},
      "points":[{"id":"a","x":12,"y":-5},{"id":"b","x":12,"y":25}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}"#;
    let r = case(&format!(r#"{sk},{{"type":"splitLine","id":"F4","faces":["F2.C1"],"sketch":"F3"}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    valid(&r);
    assert_eq!(faces(&r), n0 + 1);
    assert!((r.volume() - whole.volume()).abs() < 1e-9);
}

#[test]
fn a_diagonal_line_through_corners_splits_the_face() {
    let whole = case("");
    let sk = r#"{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},
      "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":20}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}"#;
    let r = case(&format!(r#"{sk},{{"type":"splitLine","id":"F4","faces":["F2.C1"],"sketch":"F3"}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    valid(&r);
    assert_eq!(faces(&r), whole.body().face_ids().count() + 1);
    assert!((r.volume() - whole.volume()).abs() < 1e-9);
}

#[test]
fn a_sketch_line_that_ends_inside_the_face_is_refused() {
    let sk = r#"{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},
      "points":[{"id":"a","x":12,"y":5},{"id":"b","x":12,"y":15}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}"#;
    let r = case(&format!(r#"{sk},{{"type":"splitLine","id":"F4","faces":["F2.C1"],"sketch":"F3"}}"#));
    assert_eq!(r.errors(), vec!["F4:invalidValue"], "{:?}", r.full());
}

#[test]
fn refusals_have_stable_codes() {
    let t = |tail: &str| case(tail).errors();
    assert_eq!(t(r#"{"type":"splitLine","id":"F3"}"#), vec!["F3:invalidValue"]);
    assert_eq!(t(r#"{"type":"splitLine","id":"F3","faces":["F2.C1"]}"#), vec!["F3:invalidValue"]);
    assert_eq!(t(r#"{"type":"splitLine","id":"F3","faces":["Nope"],"plane":{"kind":"std","id":"right"},"offset":15}"#), vec!["F3:missingReference"]);
    // the plane misses the face
    assert_eq!(t(r#"{"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{"kind":"std","id":"right"},"offset":99}"#), vec!["F3:cutNoEffect"]);
    // parallel to the face
    assert_eq!(t(r#"{"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{"kind":"std","id":"front"},"offset":3}"#), vec!["F3:cutNoEffect"]);
    assert_eq!(t(r#"{"type":"splitLine","id":"F3","faces":["F2.C1"],"sketch":"Nope"}"#), vec!["F3:missingSketch"]);
    // no body at all
    let r = run(r#"{"type":"splitLine","id":"F1","faces":["F2.C1"],"plane":{"kind":"std","id":"right"}}"#);
    assert_eq!(r.errors(), vec!["F1:nothingToCut"]);
}

#[test]
fn a_curved_face_is_refused_not_cut_wrongly() {
    let sk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],
      "entities":[{"type":"circle","id":"e1","c":"c","r":10}],"constraints":[]}"#;
    let ex = r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
    let split = r#"{"type":"splitLine","id":"F3","faces":["F2.S.e1"],"plane":{"kind":"std","id":"right"},"offset":1}"#;
    let r = run(&format!("{sk},{ex},{split}"));
    assert_eq!(r.errors(), vec!["F3:notImplemented"], "{:?}", r.full());
}

/// A box with a 6 mm round hole through it (centre x 20, y 10): a plane that stays clear of the hole cuts the top face; one through
/// the hole would join the outer boundary to the hole, which is refused.
#[test]
fn a_face_with_a_hole_splits_clear_of_the_hole_and_refuses_through_it() {
    let hole = r#"{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":20,"y":10}],
      "entities":[{"type":"circle","id":"e1","c":"c","r":3}],"constraints":[]},
      {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":10,"flip":true}"#;
    let base = case(hole);
    assert!(base.errors().is_empty(), "{:?}", base.full());
    let (n0, v0) = (faces(&base), base.volume());
    let clear = case(&format!(r#"{hole},{{"type":"splitLine","id":"F5","faces":["F2.C1"],"plane":{{"kind":"std","id":"right"}},"offset":8}}"#));
    assert!(clear.errors().is_empty(), "{:?}", clear.full());
    valid(&clear);
    assert_eq!(faces(&clear), n0 + 1);
    assert!((clear.volume() - v0).abs() < 1e-9);
    let through = case(&format!(r#"{hole},{{"type":"splitLine","id":"F5","faces":["F2.C1"],"plane":{{"kind":"std","id":"right"}},"offset":20}}"#));
    assert_eq!(through.errors(), vec!["F5:invalidValue"], "{:?}", through.full());
}
