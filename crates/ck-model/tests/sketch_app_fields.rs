//! Fields the app keeps at the top level of a sketch feature (view data and editor records) must not raise
//! `unknownField` on regen: `dimPlacement`, `refDims`, `cornerHoles`, and the ones that were already carried (`seq`,
//! `prop`, `gear`). A field nobody carries still warns, so the check cannot pass by accepting everything.
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

fn warnings_of(extra: &str) -> (Vec<String>, usize) {
    let sketch = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"xz"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":5}}],"constraints":[]{extra}}}"#
    );
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{sketch},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":4}}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    (r.warnings.iter().map(|w| w.error.code.clone()).collect(), ex.brep.body.as_ref().unwrap().num_faces())
}

#[test]
fn the_apps_sketch_fields_raise_no_warning() {
    let extra = r#","dimPlacement":{"d1":{"level":2,"side":-1}},"refDims":[{"id":"r1","type":"distance","refs":["c","c"]}],"cornerHoles":{"rect":[0,0,10,10],"radius":2},"seq":3,"prop":{"a":1},"gear":{"teeth":12}"#;
    let (w, faces) = warnings_of(extra);
    assert!(!w.iter().any(|c| c == "unknownField"), "{w:?}");
    assert_eq!(faces, warnings_of("").1, "the fields change nothing about the geometry");
}

#[test]
fn a_dragged_dimension_label_only_is_fine() {
    let (w, _) = warnings_of(r#","dimPlacement":{"d1":{"level":1,"side":1}}"#);
    assert!(!w.iter().any(|c| c == "unknownField"), "{w:?}");
}

#[test]
fn a_field_nobody_carries_still_warns() {
    let (w, _) = warnings_of(r#","noSuchField":1"#);
    assert!(w.iter().any(|c| c == "unknownField"), "{w:?}");
}
