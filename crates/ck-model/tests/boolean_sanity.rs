//! A boolean whose closed, valid result is wrong must surface as a
//! structured feature error (`booleanVolumeMismatch`), never as a silent
//! success. BenchCAD j_hook: rod united with a revolved handle whose
//! start-cap rim lies exactly on the rod's cylinder.
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

#[test]
fn j_hook_union_is_correct_or_reported() {
    let path = format!("{}/../../tools/bench/benchcad/j_hook_001383.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    // up to and including the revolve R1 (rod + handle)
    let n = part.features.iter().position(|f| f.id == "R1").unwrap() + 1;
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts { up_to: Some(n as u32), ..RegenOpts::default() }, &mut ex);
    let body = ex.0.body.as_ref().unwrap();
    let v = ck_topo::mass::mass_props(body).volume;
    let codes: Vec<_> = res.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
    if codes.is_empty() {
        // CadQuery: 22782.6
        assert!((v - 22782.6).abs() < 2e-3 * 22782.6, "silent wrong union: volume {v}");
    } else {
        assert_eq!(codes, vec![("R1".to_string(), "booleanVolumeMismatch".to_string())]);
        let e = &res.errors[0].error;
        let p = |k: &str| e.params.iter().find(|(n, _)| n == k).and_then(|(_, v)| v.as_f64()).unwrap_or_else(|| panic!("param {k}"));
        assert!(p("got") < p("expected_min") || p("got") > p("expected_max"));
        // the rejected union is not committed: the body is still the rod (r 6, length 107.04)
        let rod = std::f64::consts::PI * 36.0 * 107.04;
        assert!((v - rod).abs() < 1e-3 * rod, "body after rejection {v} vs rod {rod}");
    }
}
