//! Native profiling aid: regen a .ckpart, then tessellate with per-phase timings.
//! usage: cargo run --release -p ck-wasm --example flange_prof -- <part.ckpart>
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use std::time::Instant;

/// Thread CPU seconds (immune to wall-clock inflation from machine load).
fn cpu() -> f64 {
    #[cfg(unix)]
    {
        extern "C" {
            fn clock_gettime(clk: i32, ts: *mut [i64; 2]) -> i32;
        }
        #[cfg(target_os = "macos")]
        const CLK: i32 = 16;
        #[cfg(not(target_os = "macos"))]
        const CLK: i32 = 3;
        let mut ts = [0i64; 2];
        unsafe { clock_gettime(CLK, &mut ts) };
        return ts[0] as f64 + ts[1] as f64 * 1e-9;
    }
    #[allow(unreachable_code)]
    0.0
}

struct Ex(Brep);
impl regen::FeatureExec for Ex {
    fn face_count(&self) -> Option<usize> { self.0.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.0.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        if std::env::var("PROF_FEAT").map(|v| v == f.id).unwrap_or(false) {
            // handshake for an external sampler: touch the marker, give it a moment to attach
            let _ = std::fs::write(std::env::var("PROF_MARK").unwrap_or("/tmp/prof_ready".into()), "1");
            std::thread::sleep(std::time::Duration::from_millis(1500));
        }
        let t = cpu();
        let r = self.0.exec(s, f);
        let dt = cpu() - t;
        if std::env::var("FEATSTAT").is_ok() {
            // per-feature result (excluded from the timing above): faces + volume
            let (nf, vol) = match &self.0.body {
                Some(b) => (b.face_ids().count(), ck_topo::mass::mass_props(b).volume),
                None => (0, 0.0),
            };
            eprintln!("{} {} {:.3}s faces {} vol {:.4} {}", f.id, f.type_name, dt, nf, vol, if r.is_err() { "ERR" } else { "" });
        } else {
            eprintln!("{} {} {:.3}s", f.id, f.type_name, dt);
        }
        r
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let part = ck_model::Part::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let t = cpu();
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    eprintln!("regen {:.3}s (cpu)", cpu() - t);
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
