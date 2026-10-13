//! Blends that end on mitred chamfer faces / clip rims: the tool boolean is retried at looser
//! welding tolerances, like `boolean::boolean` (BenchCAD cable_routing_panel F4 and snap_clip F5).
//! Volumes are closed-form: plate 210.27 x 92.16 x 3.79 minus the rim chamfers (0.95, mitred) is
//! 73172.65; the four vertical r 0.83 fillets then remove (1 - pi/4) r^2 per unit of straight
//! edge (2.84 long) plus a little where they run into the chamfers.
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

fn run(part: &str, up_to: Option<u32>) -> (Vec<String>, f64, usize) {
    let path = format!("{}/../../tools/bench/benchcad/{part}.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts { up_to, ..RegenOpts::default() }, &mut ex);
    let errs = res.errors.iter().map(|e| format!("{} {}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

#[test]
fn corner_fillets_after_a_rim_chamfer() {
    let (e3, v3, _) = run("cable_routing_panel_004961", Some(3));
    assert!(e3.is_empty(), "{e3:?}");
    assert!((v3 - 73172.65).abs() < 0.05, "chamfered plate {v3}");
    // F4: one of the four symmetric corners used to be blendFailed
    let (e4, v4, issues) = run("cable_routing_panel_004961", Some(4));
    assert!(e4.is_empty(), "{e4:?}");
    assert_eq!(issues, 0);
    let removed = v3 - v4;
    assert!((removed - 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 0.83 * 0.83 * 2.84).abs() < 0.1, "fillets removed {removed}");
}
