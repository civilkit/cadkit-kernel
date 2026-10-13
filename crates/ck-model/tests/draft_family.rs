//! A draft names a face that was split into `<id>~k` pieces by its base id
//! (large sample F49: `F32.S.h1` for the six pilot holes `F32.S.h1~1..~6`).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
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

const PART: &str = r#"{"format":"ck-part","version":1,"features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
              {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},
  "points":[{"id":"q1","x":10,"y":10},{"id":"q2","x":30,"y":10}],
  "entities":[{"type":"point","id":"e1","p":"q1"},{"type":"point","id":"e2","p":"q2"}],"constraints":[]},
 {"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":3,"end":"blind","depth":6,"tip":false,"flip":true},
 {"type":"draft","id":"F5","neutral":{"kind":"std","id":"front"},"faces":["F4.S.h1"],"angle":1,"flip":true}
]}"#;

#[test]
fn draft_names_a_split_face_family() {
    let part = Part::from_json(&ck_json::parse(PART).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "regen errors: {:?}", r.errors);
    let b = ex.brep.body.as_ref().expect("body");
    let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    assert!(names.iter().any(|n| n.starts_with("F4.S.h1~")), "the hole wall is split into pieces: {names:?}");
    let cones = b.face_ids().filter(|&f| b.face(f).surface.kind() == "cone").count();
    assert_eq!(cones, 2, "both holes are drafted: {names:?}");
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6));
}
