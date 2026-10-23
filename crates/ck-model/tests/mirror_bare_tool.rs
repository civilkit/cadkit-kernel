//! Mirror of a seed whose tool has no recipe (a hole): the tool body itself is reflected.

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


fn volume(features: &str) -> (f64, Vec<String>) {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect();
    (ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume), errors)
}

const BOX_AND_HOLE: &str = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[
     {"id":"p1","x":-15,"y":-15},{"id":"p2","x":15,"y":-15},{"id":"p3","x":15,"y":15},{"id":"p4","x":-15,"y":15}],
   "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
     {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
  {"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10},
  {"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10,"name":"z=10"},
  {"type":"sketch","id":"S4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"p1","x":8,"y":0}],
   "entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
  {"type":"hole","id":"H1","sketch":"S4","kind":"simple","diameter":4,"end":"through"}"#;

#[test]
fn mirror_of_a_hole_reflects_the_tool_body() {
    let (one, e) = volume(BOX_AND_HOLE);
    assert!(e.is_empty(), "{e:?}");
    let (two, e) = volume(&format!(r#"{BOX_AND_HOLE},{{"type":"mirror","id":"M5","seeds":["H1"],"plane":{{"kind":"std","id":"right"}}}}"#));
    assert!(e.is_empty(), "{e:?}");
    let hole = 9000.0 - one;
    assert!((hole - std::f64::consts::PI * 4.0 * 10.0).abs() < 1e-6, "one hole removes {hole}");
    assert!((9000.0 - two - 2.0 * hole).abs() < 1e-6, "mirrored hole removes {} instead of {}", 9000.0 - two, 2.0 * hole);
}
