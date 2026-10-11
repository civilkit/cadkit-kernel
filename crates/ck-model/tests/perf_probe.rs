//! Diagnostic (ignored): `CASE=flangeBolt UPTO=13 cargo test -p ck-model --profile fast
//! --test perf_probe -- --ignored --nocapture` regenerates a tools/cases part, then times
//! face_integral per face (volume moments) and lists the slowest faces.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::time::Instant;

#[test]
#[ignore]
fn perf_probe() {
    let case = std::env::var("CASE").unwrap_or("flangeBolt".into());
    let upto: u32 = std::env::var("UPTO").ok().and_then(|s| s.parse().ok()).unwrap_or(13);
    let path = if case.contains('/') { case.clone() } else { format!("{}/../../tools/cases/{}.ckpart", env!("CARGO_MANIFEST_DIR"), case) };
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
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
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let _ = regen::regen(&part, &RegenOpts { up_to: Some(upto), ..RegenOpts::default() }, &mut ex);
    eprintln!("regen {:.2}s", t.elapsed().as_secs_f64());
    let b = ex.0.body.as_ref().unwrap();
    let t = Instant::now();
    let m = ck_topo::mass::mass_props(b);
    eprintln!("mass_props {:.2}s vol {:?}", t.elapsed().as_secs_f64(), m);
    let mut rows = vec![];
    for f in b.face_ids() {
        let t = Instant::now();
        let v = ck_topo::mass::face_integral::<1>(b, f, 1, &|p, n| [p.dot(n) / 3.0])[0];
        let dt = t.elapsed().as_secs_f64();
        let desc = match &b.face(f).surface {
            ck_geom::surface::Surface::BSpline(s) => format!("bspline {}x{} deg {},{} knots {},{} rat {}", s.nu, s.nv, s.deg_u, s.deg_v, s.knots_u.len(), s.knots_v.len(), s.is_rational()),
            s => s.kind().to_string(),
        };
        rows.push((dt, v, desc, b.face(f).loops.iter().map(|&l| b.lp(l).coedges.len()).collect::<Vec<_>>()));
    }
    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    for r in rows.iter().take(12) {
        eprintln!("{:.3}s v {:.6} {} loops {:?}", r.0, r.1, r.2, r.3);
    }
}
