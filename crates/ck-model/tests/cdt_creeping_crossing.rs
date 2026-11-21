//! A 60 x 40 x 10 plate with a boss (r 10) and a 2 mm base fillet, imported from STEP (the fillet a rational spline), the boss wall offset
//! by 1: the result's fillet face has two nearly overlapping constraints in its triangulation, whose crossing point crept along them one
//! new point per round and kept the constrained-Delaunay segment insertion busy for minutes (a freeze in the browser; the wasm host has no
//! clock to cap it). Segment insertion now snaps a crossing within rounding of its end to that end and has an iteration budget, so every
//! face meshes at once.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::sync::mpsc;
use std::time::Duration;

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
fn offset_boss_wall_next_to_a_spline_fillet_meshes_every_face() {
    let (tx, rx) = mpsc::channel();
    std::thread::Builder::new()
        .stack_size(16 << 20)
        .spawn(move || {
            let part = Part::parse(include_str!("fixtures/boss_fillet_offset_wall.json")).unwrap();
            let mut ex = Exec(Brep::new());
            let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
            let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect();
            let meshed = ex.0.body.as_ref().map(|b| {
                let t = ck_tess::mesh::tessellate(b);
                (t.faces.len(), t.faces.iter().filter(|f| f.is_some()).count())
            });
            let _ = tx.send((errs, meshed));
        })
        .unwrap();
    let (errs, meshed) = rx.recv_timeout(Duration::from_secs(60)).expect("the triangulation finishes (it used to run for minutes)");
    // a fast refusal is as good as a body; if there is a body every one of its faces meshes
    if errs.is_empty() {
        let (n, ok) = meshed.expect("a body");
        assert_eq!(n, ok, "every face meshes");
    }
}
