//! Lengths outside what the kernel can honour (a 1e-9 shell, a 1e300 extrude) are refused as `invalidValue`
//! naming the field and the range, instead of failing deep in a boolean with an unrelated code or returning
//! `ok` with a body that does not tessellate (solidcore review 2026-11-07, probes).
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

fn first_error(depth: &str, tail: &str) -> Option<(String, String)> {
    let doc = format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":20,"y":0}},{{"id":"p3","x":20,"y":20}},{{"id":"p4","x":0,"y":20}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":{depth}}}{tail}]}}"#
    );
    let part = Part::from_json(&ck_json::parse(&doc).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    r.errors.first().map(|e| (e.error.code.clone(), e.feature.clone()))
}

#[test]
fn a_huge_extrude_depth_is_refused() {
    assert_eq!(first_error("1e300", ""), Some(("invalidValue".into(), "E2".into())));
}

#[test]
fn a_tiny_shell_thickness_is_refused() {
    let shell = r#",{"type":"shell","id":"F3","faces":["E2.S.e2"],"thickness":1e-9}"#;
    assert_eq!(first_error("10", shell), Some(("invalidValue".into(), "F3".into())));
}

#[test]
fn ordinary_sizes_are_unchanged() {
    assert_eq!(first_error("10", ""), None);
    let shell = r#",{"type":"shell","id":"F3","faces":["E2.S.e2"],"thickness":1}"#;
    assert_eq!(first_error("10", shell), None);
}
