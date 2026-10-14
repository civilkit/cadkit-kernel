//! Single-parameter edits of a part that the original kernel still builds
//! (docs/edit-robustness.md, tools/edit_robustness_check.mjs). Each case
//! edits one numeric leaf of one feature of a stored part, regenerates, and
//! asserts: no feature error, a valid body, and the volume the original
//! kernel gets for the same edit (relative 1e-3).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

/// `tools/<file>` with feature `at`'s numeric leaf `leaf` (dotted path) set to `base * factor`
/// (`base` is asserted so a changed fixture does not silently test something else).
fn edited(file: &str, at: usize, leaf: &str, base: f64, factor: f64) -> Part {
    let path = format!("{}/../../tools/{file}", env!("CARGO_MANIFEST_DIR"));
    let mut v = ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut o = &mut v.get_mut("features").unwrap().as_arr_mut().unwrap()[at];
    let keys: Vec<&str> = leaf.split('.').collect();
    for k in &keys[..keys.len() - 1] {
        o = match k.parse::<usize>() {
            Ok(i) => &mut o.as_arr_mut().unwrap()[i],
            Err(_) => o.get_mut(k).unwrap(),
        };
    }
    let last = keys[keys.len() - 1];
    let slot = match last.parse::<usize>() {
        Ok(i) => &mut o.as_arr_mut().unwrap()[i],
        Err(_) => o.get_mut(last).unwrap(),
    };
    assert_eq!(slot.as_f64(), Some(base), "{file} feature {at} {leaf}");
    *slot = ck_json::Json::Num(base * factor);
    Part::from_json(&v).unwrap()
}

fn check(part: &Part, want_volume: f64) {
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let b = ex.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
    let vol = ck_topo::mass::mass_props(b).volume;
    assert!((vol - want_volume).abs() <= 1e-3 * want_volume, "volume {vol} vs {want_volume}");
}

/// A sphere (revolved arc) cut through by a coaxial-with-nothing cylinder whose end cap ends inside it:
/// the sphere face splits into a cap and the rest along one circle; the rest is "seam loop + hole" and
/// the seam loop must be the whole domain, not a spike (else the two fragments merge and no sample
/// point is found).
#[test]
fn ball_knob_stem_end_inside_the_ball() {
    // F5 (feature 4) depth 57.26 -> 71.575: the stem now ends inside the ball
    check(&edited("bench/benchcad/ball_knob_007575.ckpart.json", 4, "depth", 57.26, 1.25), 15849.86110205764);
    check(&edited("bench/benchcad/ball_knob_007575.ckpart.json", 4, "depth", 57.26, 1.5), 16180.92791897596);
}
