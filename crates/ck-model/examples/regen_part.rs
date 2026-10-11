//! Native regen of a .ckpart with per-feature timings (profiling aid).
//! usage: cargo run --release -p ck-model --example regen_part -- <part.ckpart> [upTo]
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use std::time::Instant;

struct Timed(Brep);
impl FeatureExec for Timed {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let t = Instant::now();
        let r = self.0.exec(s, f);
        let spans: Vec<String> = std::mem::take(&mut self.0.spans).into_iter().map(|s| s.label).collect();
        eprintln!("{} {}: {:?} {:.3}s {:?} faces={:?}", f.id, f.type_name, r.as_ref().err().map(|e| e.describe()), t.elapsed().as_secs_f64(), spans, self.0.face_count());
        r
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let part = ck_model::Part::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let mut opts = RegenOpts::default();
    opts.up_to = a.get(2).map(|s| s.parse().unwrap());
    let mut ex = Timed(Brep::new());
    regen::regen(&part, &opts, &mut ex);
}
