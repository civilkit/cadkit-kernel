//! Crash-fuzz cases (seed 7) whose loft faces made the spline normal-turn split in ck-tess ten times slower (7 to 760 ms
//! native, past the fuzzer's 6 s CPU limit on the CI box under load, and a node segfault at 8 workers): the rule now
//! applies only to small faces and is capped per face. Measured with the rule unbounded: 24046 and 28170 triangles; with the cap: 46 and 3782.
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

fn tris(src: &str) -> usize {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().expect("a body");
    ck_tess::mesh::tessellate(b).mesh.num_tris()
}

#[test]
fn fuzz_loft_cases_keep_a_modest_mesh() {
    let a = tris(include_str!("fixtures/fuzz_loft_956.json"));
    let b = tris(include_str!("fixtures/fuzz_loft_4823.json"));
    println!("TRIS {a} {b}");
    assert!(a < LIMIT_956 && b < LIMIT_4823, "{a} {b}");
}
const LIMIT_956: usize = 500;
const LIMIT_4823: usize = 8000;
