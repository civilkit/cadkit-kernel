//! Per-face tessellation sanity: regen a .ckpart natively, tessellate every body and print, per face,
//! triangle count, mesh area and signed volume contribution (divergence theorem), and the failed faces.
//! usage: cargo run -q --profile fast -p ck-model --example tess_probe -- <part.ckpart>
//! Set CK_DUMP3=1 for the boundary-walk trace of ck-tess::face.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};

struct Ex(Brep);
impl FeatureExec for Ex {
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

fn main() {
    for path in std::env::args().skip(1) {
        probe(&path);
    }
}

fn probe(path: &str) {
    eprintln!("== {path}");
    let a = vec![String::new(), path.to_string()];
    let part = ck_model::Part::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let mut ex = Ex(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    eprintln!("errors: {}", r.errors.len());
    let mut all: Vec<(String, &ck_topo::body::Body)> = ex.0.surfs.iter().map(|(i, b)| (i.clone(), b)).collect();
    if let Some(b) = &ex.0.body {
        all.push(("main".into(), b));
    }
    for (id, body) in all {
        let t = ck_tess::mesh::tessellate(body);
        let m = &t.mesh;
        let n = m.tri_face.len();
        let nf = body.face_ids().count();
        let mut per = vec![(0usize, 0.0f64, 0.0f64); nf];
        let mut bb = vec![([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]); nf];
        for i in 0..n {
            let p = |k: usize| [m.positions[i * 9 + k * 3] as f64, m.positions[i * 9 + k * 3 + 1] as f64, m.positions[i * 9 + k * 3 + 2] as f64];
            let (a, b, c) = (p(0), p(1), p(2));
            let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
            let cr = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            let f = m.tri_face[i] as usize;
            for q in [a, b, c] {
                for k in 0..3 {
                    bb[f].0[k] = bb[f].0[k].min(q[k]);
                    bb[f].1[k] = bb[f].1[k].max(q[k]);
                }
            }
            per[f].0 += 1;
            per[f].1 += 0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
            per[f].2 += (a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2]) / 6.0;
        }
        eprintln!("body {id}: failed faces {:?}", m.failed);
        for (i, fid) in body.face_ids().enumerate() {
            eprintln!("  face {i} {}: tris {} area {:.3} vol {:.3} box {:.2?} {:.2?}", format!("{:?}", body.face(fid).surface).chars().take(24).collect::<String>(), per[i].0, per[i].1, per[i].2, bb[i].0, bb[i].1);
        }
    }
}
