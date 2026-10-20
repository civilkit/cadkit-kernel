//! `offsetUnsupported {detail:"edge curve type"}` carries a hint naming what
//! to change (issue #391: shell after a fillet on drafted edges).

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

#[test]
fn fillet_on_drafted_edges_says_what_to_change() {
    // issue #391's shape: a drafted tower, R6 on the four oblique vertical
    // edges, then shell 2.5 open at the top.
    let part = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"S3","plane":{"kind":"std","id":"front"},"points":[
          {"id":"p1","x":-45,"y":-30},{"id":"p2","x":45,"y":-30},{"id":"p3","x":45,"y":30},{"id":"p4","x":-45,"y":30}],
         "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},
          {"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
        {"type":"extrude","id":"E4","op":"add","sketch":"S3","end":"blind","depth":35,"draft":2,"draftOut":true},
        {"type":"fillet","id":"FI","radius":6,"edges":[
          {"faces":["E4.S.e2","E4.S.e3"],"at":[45.698415,30.698415,20]},
          {"faces":["E4.S.e1","E4.S.e2"],"at":[45.698415,-30.698415,20]},
          {"faces":["E4.S.e3","E4.S.e4"],"at":[-45.698415,30.698415,20]},
          {"faces":["E4.S.e1","E4.S.e4"],"at":[-45.698415,-30.698415,20]}]},
        {"type":"shell","id":"SH","faces":["E4.C1"],"thickness":2.5}]}"#;
    let part = Part::from_json(&ck_json::parse(part).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    assert_eq!(errs.len(), 1, "{errs:?}");
    let e = &errs[0];
    assert!(e.starts_with("SH:offsetUnsupported"), "{e}");
    assert!(e.contains("edge curve type"), "{e}");
    assert!(e.contains("hint"), "{e}");
    assert!(e.contains("shell before the fillet"), "{e}");
}
