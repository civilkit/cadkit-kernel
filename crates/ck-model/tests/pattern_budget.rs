//! The work budget reaches the stage of a boolean that classifies fragments: a pattern of 100 overlapping copies is
//! stopped by `featureTimeMs` instead of running on for seconds past it (crash fuzz seed 11, case 181).
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
fn a_large_pattern_stops_at_its_feature_budget_inside_the_boolean() {
    let fs = r#"[{"type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"}, "points": [{"id": "c", "x": 0, "y": 0}], "entities": [{"type": "circle", "id": "e1", "c": "c", "r": -23.144436772121118}], "constraints": []}, {"type": "extrude", "id": "F2", "op": "add", "sketch": "S1", "end": "blind", "depth": -28.551495409570634}, {"type": "lpattern", "id": "F9", "seeds": ["F2"], "dir": {"kind": "axis", "axis": "z"}, "spacing": 28.02503846255131, "count": 100}]"#;
    let src = format!(r#"{{"format":"ck-part","version":1,"features":{fs}}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    static T0: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    T0.get_or_init(std::time::Instant::now);
    ck_core::budget::set_clock(|| T0.get().unwrap().elapsed().as_secs_f64() * 1e3);
    let t = std::time::Instant::now();
    let r = regen::regen(&part, &{ let mut o = RegenOpts::default(); o.feature_time_ms = Some(20.0); o }, &mut ex);
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let codes: Vec<String> = r.errors.iter().map(|e| e.error.code.clone()).collect();
    assert_eq!(codes, vec!["featureBudgetExceeded".to_string()]);
    // unbudgeted, the one boolean of this pattern grinds for ~0.5 s (2.5 s once the row is 100 long) in the stage
    // that classifies the fragments, past the imprint's poll: the budget has to end it there
    assert!(ms < 250.0, "{ms} ms");
}
