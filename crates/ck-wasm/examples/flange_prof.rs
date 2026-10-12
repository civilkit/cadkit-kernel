//! Native profiling aid: regen a .ckpart, then tessellate with per-phase timings.
//! usage: cargo run --release -p ck-wasm --example flange_prof -- <part.ckpart>
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use std::time::Instant;

struct Ex(Brep);
impl regen::FeatureExec for Ex {
    fn face_count(&self) -> Option<usize> { self.0.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.0.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let t = Instant::now();
        let r = self.0.exec(s, f);
        eprintln!("{} {} {:.3}s", f.id, f.type_name, t.elapsed().as_secs_f64());
        r
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let part = ck_model::Part::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let t = Instant::now();
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    eprintln!("regen {:.3}s", t.elapsed().as_secs_f64());
    let body = ex.0.body.take().unwrap();
    if std::env::var("PERFACE").is_ok() { per_face(&body); }
    let t = Instant::now();
    let tess = ck_tess::mesh::tessellate(&body);
    eprintln!("tessellate {:.3}s tris {}", t.elapsed().as_secs_f64(), tess.mesh.tri_face.len());
}

#[allow(dead_code)]
fn per_face(body: &ck_topo::body::Body) {
    let tol = ck_tess::mesh::body_tol(body);
    let edges = ck_tess::mesh::sample_edges(body, &tol);
    let mut rows = vec![];
    for f in body.face_ids() {
        let t = Instant::now();
        let r = ck_tess::face::mesh_face(body, f, &edges.params, &edges.virt, &edges.ssi, edges.cuts[f.idx()], &tol);
        rows.push((t.elapsed().as_secs_f64(), body.face_id_str(f), r.map(|m| m.tris.len()).unwrap_or(0)));
    }
    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    for r in rows.iter().take(15) { eprintln!("  face {} {:.3}s tris {}", r.1, r.0, r.2); }
}
