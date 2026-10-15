//! Native aid: regen a .ckpart and print per-edge sampling stats and per-face tessellation time.
//! usage: cargo run --profile fast -p ck-wasm --example tess_edges -- <part.ckpart>
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use std::time::Instant;

struct Ex(Brep);
impl regen::FeatureExec for Ex {
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
    let a: Vec<String> = std::env::args().collect();
    let part = ck_model::Part::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    let body = ex.0.body.take().unwrap();
    let tol = ck_tess::mesh::body_tol(&body);
    eprintln!("tol chord {} angle {}", tol.chord, tol.angle);
    let t = Instant::now();
    let edges = ck_tess::mesh::sample_edges(&body, &tol);
    eprintln!("sample_edges {:.3}s", t.elapsed().as_secs_f64());
    for e in body.edge_ids() {
        let ed = body.edge(e);
        let kind = match &ed.curve {
            ck_geom::curve::Curve::BSpline(b) => format!("bspline deg {} knots {}", b.degree, b.knots.len()),
            c => format!("{:?}", c).chars().take(12).collect::<String>(),
        };
        let pcs: Vec<String> = body
            .edge_coedges(e)
            .iter()
            .map(|&c| match &body.coedge(c).pcurve {
                Some(p) => format!("{:?}", p).chars().take(10).collect::<String>(),
                None => "none".into(),
            })
            .collect();
        eprintln!("edge {} t[{:.3},{:.3}] {} -> {} samples pcurves {:?}", e.idx(), ed.t0, ed.t1, kind, edges.params[e.idx()].len(), pcs);
    }
    let mut tot = 0;
    for f in body.face_ids() {
        let t = Instant::now();
        let r = ck_tess::face::mesh_face(&body, f, &edges.params, &edges.virt, &edges.ssi, edges.cuts[f.idx()], &tol);
        let n = r.map(|m| m.tris.len()).unwrap_or(0);
        tot += n;
        eprintln!("face {} {:.3}s tris {}", body.face_id_str(f), t.elapsed().as_secs_f64(), n);
    }
    eprintln!("total tris {tot}");
    let _ = ck_tess::mesh::tessellate_memo(&body);
}
