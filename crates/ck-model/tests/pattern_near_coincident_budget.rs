//! The work budget reaches the trimming of the imprint: 12 copies of a revolved body a hair apart are stopped
//! by `featureTimeMs` instead of grinding for seconds past it (crash fuzz seed 11, case 525).
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
fn near_coincident_copies_stop_at_the_budget_inside_the_robust_retry() {
    let fs = r#"[{"type": "sketch", "id": "S1", "plane": {"kind": "std", "id": "front"}, "points": [{"id": "p0", "x": 15.19005023187492, "y": 0}, {"id": "p1", "x": 36800185.55671228, "y": 22.885029329848475}, {"id": "p2", "x": 0, "y": 22.885029329848475}, {"id": "p3", "x": 0, "y": 0}, {"id": "a1", "x": 0, "y": 0}, {"id": "a2", "x": 0, "y": 22.885029329848475}], "entities": [{"type": "line", "id": "e1", "p1": "p0", "p2": "p1"}, {"type": "line", "id": "e2", "p1": "p1", "p2": "p2"}, {"type": "line", "id": "e3", "p1": "p2", "p2": "p3"}, {"type": "line", "id": "e4", "p1": "p3", "p2": "p0"}, {"type": "line", "id": "ax", "p1": "a1", "p2": "a2", "construction": true}], "constraints": []}, {"type": "revolve", "id": "F2", "op": "add", "sketch": "S1", "axis": "ax", "angle": 67.51865050359629}, {"type": "lpattern", "id": "F9", "seeds": ["F2"], "dir": {"kind": "axis", "axis": "x"}, "spacing": 1.1795727105332206e-05, "count": 12}]"#;
    let src = format!(r#"{{"format":"ck-part","version":1,"features":{fs}}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    static T0: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    T0.get_or_init(std::time::Instant::now);
    ck_core::budget::set_clock(|| T0.get().unwrap().elapsed().as_secs_f64() * 1e3);
    let t = std::time::Instant::now();
    let r = regen::regen(&part, &{ let mut o = RegenOpts::default(); o.feature_time_ms = Some(300.0); o }, &mut ex);
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let codes: Vec<String> = r.errors.iter().map(|e| e.error.code.clone()).collect();
    assert_eq!(codes, vec!["booleanBudgetExceeded".to_string()]);
    // unbudgeted, the robust retry of the one boolean that applies the pattern spends ~4 s (more in wasm) in trimming the
    // curves of a pair of cone faces (hundreds of ms for each); the budget has to end it there
    assert!(ms < 1500.0, "{ms} ms");
}
