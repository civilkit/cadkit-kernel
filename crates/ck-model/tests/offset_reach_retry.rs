//! The offset reach of a shell: faces away from the opened ones are not continued by the full reach (see `fit_face_with_margin`).

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

/// The 5 mm round of the bottle chain borders no opened face of the 2 mm shell, so its blend patches are offset with the short
/// reach from the start: no free-form fit fails and is retried (each failed full-reach fit cost 2 to 3 s). Counts retries, not time.
#[test]
fn the_shell_of_the_bottle_chain_retries_no_offset_fit() {
    let doc = include_str!("fixtures/bottle_chain.ckpart.json");
    let part = Part::parse(doc).unwrap();
    let mut ex = Exec(Brep::new());
    let before = ck_ops::offset::FIT_RETRIES.load(std::sync::atomic::Ordering::Relaxed);
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let retries = ck_ops::offset::FIT_RETRIES.load(std::sync::atomic::Ordering::Relaxed) - before;
    assert_eq!(retries, 0, "{retries} offset fits failed and were retried");
}
