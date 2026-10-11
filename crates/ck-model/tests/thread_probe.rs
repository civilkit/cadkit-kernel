//! Diagnostic (ignored): `CASE=flangeBolt UPTO=13 cargo test -p ck-model --profile fast
//! --test thread_probe -- --ignored --nocapture` regenerates a tools/cases part with
//! timestamps per trace label.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::time::Instant;

struct T(Instant, usize);
impl Tracer for T {
    fn begin(&mut self, label: &str) -> usize {
        eprintln!("[{:8.2}s] {}", self.0.elapsed().as_secs_f64(), label);
        self.1 += 1;
        self.1
    }
    fn relabel(&mut self, _id: usize, label: &str) {
        eprintln!("[{:8.2}s]   = {}", self.0.elapsed().as_secs_f64(), label);
    }
    fn end(&mut self, _id: usize) {}
}
struct Exec(Brep, T);
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.1)
    }
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
fn thread_probe() {
    let case = std::env::var("CASE").unwrap_or("flangeBolt".into());
    let upto: u32 = std::env::var("UPTO").ok().and_then(|s| s.parse().ok()).unwrap_or(13);
    let path = if case.contains('/') { case.clone() } else { format!("{}/../../tools/cases/{}.ckpart", env!("CARGO_MANIFEST_DIR"), case) };
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new(), T(Instant::now(), 0));
    let r = regen::regen(&part, &RegenOpts { up_to: Some(upto), ..RegenOpts::default() }, &mut ex);
    if let Some(b) = &ex.0.body {
        let mut tot = ck_geom::Vec3::ZERO;
        for f in b.face_ids() {
            let a = ck_topo::mass::face_integral::<3>(b, f, 1, &|_, n| [n.x, n.y, n.z]);
            let ar = ck_topo::mass::face_integral::<1>(b, f, 1, &|_, n| [n.len()])[0];
            let v = ck_geom::Vec3::new(a[0], a[1], a[2]);
            tot = tot + v;
            eprintln!("face {} {} area {:.4} vecarea ({:.4},{:.4},{:.4}) loops {:?}", b.face_id_str(f), b.face(f).surface.kind(), ar, v.x, v.y, v.z, b.face(f).loops.iter().map(|&l| b.lp(l).coedges.len()).collect::<Vec<_>>());
        }
        if let Ok(name) = std::env::var("DUMPFACE") {
            let f = b.find_face(&name).unwrap();
            for res in [1usize, 2, 4] {
                eprintln!("area res {} = {}", res, ck_topo::mass::face_integral::<1>(b, f, res, &|_, n| [n.len()])[0]);
            }
            for &l in &b.face(f).loops {
                eprintln!("loop:");
                for &c in &b.lp(l).coedges {
                    let (ta, tb) = b.coedge_params(c);
                    let ed = b.edge(b.coedge(c).edge);
                    let kind = match &ed.curve {
                        ck_geom::curve::Curve::Line { .. } => "line",
                        ck_geom::curve::Curve::Circle { .. } => "circle",
                        ck_geom::curve::Curve::BSpline(_) => "bspline",
                        _ => "other",
                    };
                    match &b.coedge(c).pcurve {
                        Some(pc) => eprintln!("  {} rev {} t[{:.3},{:.3}] uv {:?} -> {:?}", kind, b.coedge(c).rev, ta, tb, pc.eval(ta), pc.eval(tb)),
                        None => eprintln!("  {} no pcurve", kind),
                    }
                }
            }
        }
        eprintln!("sum vecarea {:?} volume {}", tot, ck_topo::mass::mass_props(b).volume);
    }
    let j = r.to_json().to_string();
    eprintln!("errors: {}", j.split("\"errors\":").nth(1).map(|s| &s[..s.len().min(300)]).unwrap_or(""));
}
