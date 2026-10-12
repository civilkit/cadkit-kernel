//! Long helical sweeps: the BenchCAD torsion spring (a 12-turn helix sweep united with two tangent
//! legs) used to take minutes (every closest-point query scanned all 577 knot spans, the SSI of the
//! legs' caps marched every turn, the containment test integrated the whole sweep). The budgets
//! below are an order of magnitude over the current time (profile `fast`) and far under the old one.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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

fn regen_file(name: &str, upto: Option<u32>) -> (ck_topo::body::Body, String, f64) {
    let text = std::fs::read_to_string(format!("{}/../../tools/bench/benchcad/{name}.ckpart.json", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts { up_to: upto, ..RegenOpts::default() }, &mut ex);
    let dt = t.elapsed().as_secs_f64();
    (ex.0.body.clone().expect("a body"), r.to_json().to_string(), dt)
}

#[test]
fn torsion_spring_regens_fast_with_two_lumps() {
    let (b, errs, dt) = regen_file("torsion_spring_000054", None);
    assert!(errs.contains(r#""errors":[]"#), "{}", &errs[..errs.len().min(300)]);
    assert_eq!(b.num_lumps(), 2, "coil and the far leg are disjoint solids");
    let m = ck_topo::mass::mass_props(&b);
    // closed form: coil A 2 pi R N with N = height / pitch (not 12), plus the two legs (cylinders), minus
    // the sliver where the second leg's cap meets the coil's start cap
    let a = std::f64::consts::PI * 3.52 * 3.52;
    let coil = a * std::f64::consts::TAU * 56.25 * (206.84 / 17.24);
    let want = coil + a * 220.5 + a * 162.87;
    assert!((m.volume - want).abs() < 5.0, "volume {} vs closed form {want}", m.volume);
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    assert!(dt < 120.0, "regen took {dt:.1} s");
}

#[test]
fn helix_sweep_alone_is_quick_and_exact() {
    let (b, errs, dt) = regen_file("torsion_spring_000054", Some(4));
    assert!(errs.contains(r#""errors":[]"#));
    let m = ck_topo::mass::mass_props(&b);
    let a = std::f64::consts::PI * 3.52 * 3.52;
    let want = a * std::f64::consts::TAU * 56.25 * (206.84 / 17.24);
    assert!((m.volume - want).abs() / want < 2e-5, "volume {} vs {want}", m.volume);
    assert!(dt < 60.0, "sweep took {dt:.1} s");
}
