//! Regression: chamfers on curved and tangent edges (BenchCAD tapered_boss:
//! plane / lofted-cone rim split in two spline arcs; u_channel: plane /
//! fillet-cylinder tangent edges next to plane / plane end chamfers).
//! Volumes are the CadQuery ground truth (tools/bench/benchcad/manifest.json).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

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

fn volume(part: &str) -> (usize, f64, usize) {
    let path = format!("{}/../../tools/bench/benchcad/{part}.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    let issues = ck_topo::check::check(b, 1e-6).len();
    (res.errors.len(), ck_topo::mass::mass_props(b).volume, issues)
}

#[test]
fn tapered_boss_rim_chamfer() {
    let (errs, vol, issues) = volume("tapered_boss_000256");
    assert_eq!((errs, issues), (0, 0));
    assert!((vol - 141314.5693).abs() < 1e-3 * 141314.5693 * 0.01, "{vol}");
}

#[test]
fn u_channel_tangent_chamfer() {
    let (errs, vol, issues) = volume("u_channel_000058");
    assert_eq!((errs, issues), (0, 0));
    assert!((vol - 11725934.423).abs() < 1e-5 * 11725934.423, "{vol}");
}
