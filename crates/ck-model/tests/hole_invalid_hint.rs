//! A refused hole carries a hint saying what to change (the brief's refusal-hint item).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
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

const PART: &str = r#"{"format":"ck-part","version":1,"name":"bad-hole",
 "features":[
  {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[
     {"id":"p1","x":-15,"y":-15},{"id":"p2","x":15,"y":-15},{"id":"p3","x":15,"y":15},{"id":"p4","x":-15,"y":15}],
   "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
     {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
  {"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10},
  {"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10,"name":"z=10"},
  {"type":"sketch","id":"S4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"p1","x":0,"y":0}],
   "entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
  {"type":"hole","id":"H1","sketch":"S4","kind":"simple","size":"NOT_A_SIZE"}
 ]}"#;

#[test]
fn an_unknown_hole_size_carries_a_hint() {
    let part = Part::parse(PART).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let e = res.errors.iter().find(|e| e.error.code == "holeInvalid").expect("holeInvalid");
    let hint = e.error.params.iter().find(|(n, _)| n == "hint").map(|(_, v)| ck_json::to_string(v)).expect("hint");
    assert!(hint.contains("size") && hint.contains("diameter"), "{hint}");
}
