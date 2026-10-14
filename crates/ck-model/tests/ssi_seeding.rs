//! Regression: a swept tube whose side meets its guide torus tangentially (kit sample
//! `swept-handle`, `tools/cases/sweptHandle.ckpart`) regenerates in well under a second with the
//! projection-grid seeding; the first patch-box seeding produced 222 tangent seeds on that pair,
//! each a failing 5000-step trace (79 s). Tangent seeds are rejected now and a failing seed shadows
//! its neighbours.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

struct Exec(Brep);
impl FeatureExec for Exec {
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
fn swept_handle_tangent_guide_is_fast() {
    let path = format!("{}/../../tools/cases/sweptHandle.ckpart", env!("CARGO_MANIFEST_DIR"));
    let part = Part::from_json(&ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let secs = t.elapsed().as_secs_f64();
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let st = ck_ops::intersect::patch::stats();
    assert!(st[3] <= ck_ops::intersect::patch::MAX_SEEDS_PUB, "seeds {}", st[3]);
    assert!(secs < 20.0, "regen took {secs:.1} s");
}
