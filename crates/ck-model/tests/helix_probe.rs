//! Diagnostic (ignored): `PART=path UPTO=n cargo test -p ck-model --profile fast
//! --test helix_probe -- --ignored --nocapture` regenerates a part (up to feature n),
//! printing the elapsed time and volume.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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
#[ignore]
fn helix_probe() {
    let path = std::env::var("PART").unwrap();
    let upto: Option<u32> = std::env::var("UPTO").ok().and_then(|s| s.parse().ok());
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts { up_to: upto, ..RegenOpts::default() }, &mut ex);
    eprintln!("regen {:.2}s errors {}", t.elapsed().as_secs_f64(), r.to_json().to_string().chars().take(300).collect::<String>());
    let b = ex.0.body.as_ref().unwrap();
    let t = Instant::now();
    let m = ck_topo::mass::mass_props(b);
    eprintln!("mass_props {:.2}s {:?} lumps {} faces {}", t.elapsed().as_secs_f64(), m, b.num_lumps(), b.face_ids().count());
}
