//! A dovetail corner with thin (3 mm) pins at slope 1:6 (the wood-joinery plugin's recorded features: per tail a
//! blank, two rotated side tools cut from it, the blank cut from the pin board and joined to the tail board) must
//! build, or be refused with a reason; it used to fail with a bare `booleanFailed`.
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

fn run(src: &str) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (ex.0.body, res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

fn check(src: &str) {
    let (body, errors) = run(src);
    assert!(errors.is_empty(), "{errors:?}");
    let b = body.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
    // the pin board and the tail board, each one solid lump
    assert_eq!(ck_topo::region::components(&b).len(), 2);
}

#[test]
fn dovetail_count6_pin3_slope6() {
    check(include_str!("fixtures/dovetail_count6_pin3_slope6.json"));
}

#[test]
fn dovetail_count7_pin3_slope6() {
    check(include_str!("fixtures/dovetail_count7_pin3_slope6.json"));
}

