//! The fillet size search is on by default: a refused round-over names the largest radius that builds in `maxSize`
//! without the client turning anything on (this binary never calls `set_blend_search`).
use ck_json::Json;
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

/// Regen `features`; the errors.
fn run(features: &str) -> Vec<regen::RegenError> {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    regen::regen(&part, &RegenOpts::default(), &mut ex).errors
}

const PLATE: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[
  {"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":30},{"id":"p4","x":0,"y":30}],
 "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
  {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":2}"#;

fn with_plate(f: &str) -> String {
    format!("{PLATE},{f}")
}

/// The one error's `maxSize` as (param, value, unit), and its hint.
fn max_size(errors: &[regen::RegenError]) -> (String, f64, String, String) {
    assert_eq!(errors.len(), 1, "{:?}", errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let e = &errors[0].error;
    let get = |k: &str| e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
    let m = get("maxSize").unwrap_or_else(|| panic!("no maxSize: {}", e.describe()));
    let param = m.get("param").and_then(|v| v.as_str()).expect("param").to_string();
    let value = m.get("value").and_then(|v| v.as_f64()).expect("value");
    let unit = m.get("unit").and_then(|v| v.as_str()).expect("unit").to_string();
    let hint = match get("hint") {
        Some(Json::Str(h)) => h,
        _ => panic!("no hint: {}", e.describe()),
    };
    assert!(hint.contains(&format!("{value} {unit}")), "the hint names the size: {hint}");
    (param, value, unit, hint)
}

#[test]
fn a_refused_round_over_names_a_radius_without_being_asked() {
    assert!(ck_model::features::max_size::blend_search_enabled());
    let f = |r: f64| {
        run(&with_plate(&format!(
            r#"{{"type":"fillet","id":"F90","radius":{r},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,2,0]}}]}}"#
        )))
    };
    let (param, v, unit, hint) = max_size(&f(3.0));
    assert_eq!((param.as_str(), unit.as_str()), ("radius", "mm"));
    assert!(v > 1.0 && v < 3.0, "{v}");
    assert!(hint.contains(&format!("{v}")), "{hint}");
    assert!(f(v).is_empty(), "builds at {v}");
}
