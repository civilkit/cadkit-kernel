//! SolidCraft #451: cutting a block with a hollow spherical shell. The block sits inside
//! the shell's cavity, so the cut removes nothing: a valid no-op, not the suspicious
//! `cutNoEffect` (a tool that reaches the body yet removes nothing).
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
fn cut_with_a_tool_inside_the_cavity_is_a_valid_no_op() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/cut_misses_inside_cavity.json")).unwrap();
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let b = ex.0.body.as_ref().expect("body");
    let mp = ck_topo::mass::mass_props(b);
    assert!((mp.volume - 512.0).abs() < 1e-6, "volume {} (the 8 mm block is unchanged)", mp.volume);
    assert!(ck_check::validate(b, 1e-6).is_empty());
}
