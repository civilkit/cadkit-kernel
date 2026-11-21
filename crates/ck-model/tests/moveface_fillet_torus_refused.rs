//! Moving or offsetting a boss wall whose foot is blended into the base plate by a fillet torus is refused: the torus would keep its
//! major radius while the wall's radius changes, a body 0.2% off in volume. A part without that fillet still moves.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

fn run() -> Vec<String> {
    std::thread::Builder::new()
        .stack_size(16 << 20)
        .spawn(|| {
            let part = Part::parse(include_str!("fixtures/boss_fillet_offset_wall.json")).unwrap();
            let mut ex = Exec(Brep::new());
            let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
            r.errors.iter().map(|e| format!("{}:{}:{}", e.feature, e.error.code, format!("{:?}", e.error.params))).collect()
        })
        .unwrap()
        .join()
        .unwrap()
}

#[test]
fn boss_wall_next_to_a_base_fillet_torus_is_refused() {
    let errs = run();
    eprintln!("{errs:?}");
    assert_eq!(errs.len(), 1, "{errs:?}");
    assert!(errs[0].starts_with("mf:moveFaceUnsupported:"), "{errs:?}");
    assert!(errs[0].contains("edge curve type"), "{errs:?}");
}
