//! Features the kernel does not build yet fail with the stable code `notImplemented` (the name of the
//! feature or option is in the `what` parameter), not with free text.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        None
    }
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

#[test]
fn unbuilt_feature_types_report_not_implemented() {
    for ty in ["dome", "splitLine", "fillpattern", "surfaceOffset", "cutWithSurface"] {
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{{"type":"{ty}","id":"F1"}}]}}"#);
        let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
        let r = regen::regen(&part, &RegenOpts::default(), &mut Exec(Brep::new()));
        let e = &r.errors.first().unwrap_or_else(|| panic!("{ty}: no error")).error;
        assert_eq!(e.code, "notImplemented", "{ty}");
        assert!(e.params.iter().any(|(k, v)| k == "what" && *v == ck_json::Json::Str(ty.to_string())), "{ty}: {:?}", e.params);
    }
}
