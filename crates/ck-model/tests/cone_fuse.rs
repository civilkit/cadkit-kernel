//! Task 7 regression: a gently-tapered / wide-base cone standing on a plate must fuse.
//!
//! Before the fix these three all hit `booleanFailed` and left the plate perimeter open: a
//! far-point classification returned `In` for a plate cap because one spline ray crossing (the
//! inner cone wall's second hit) was missed, so the cap was dropped. `Solid::classify` now takes a
//! majority-of-rays vote with a winding tie-break in robust retries instead of trusting one ray.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize { 0 }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec { brep: Brep, labels: Noop }
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> { Some(&mut self.labels) }
    fn face_count(&self) -> Option<usize> { self.brep.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.brep.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> { self.brep.exec(s, f) }
}

/// Regen a fixture and return its error descriptions plus the body volume (if a body exists).
fn build(src: &str) -> (Vec<String>, Option<f64>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let volume = ex.brep.body.as_ref().map(|b| ck_topo::mass::mass_props(b).volume);
    (errors, volume)
}

#[test]
fn gently_tapered_solid_loft_r22_to_r17_fuses() {
    let (errors, volume) = build(include_str!("../../../tools/cases/cone_fuse/loft_r22_17.json"));
    assert!(errors.is_empty(), "regen errors: {errors:?}");
    // plate ~8982 + solid frustum r22->r17 h10 (~12010)
    assert!(volume.unwrap_or(0.0) > 20000.0, "volume too small: {volume:?}");
}

#[test]
fn gently_tapered_solid_loft_r25_to_r20_fuses() {
    let (errors, volume) = build(include_str!("../../../tools/cases/cone_fuse/loft_r25_20.json"));
    assert!(errors.is_empty(), "regen errors: {errors:?}");
    // plate ~8982 + solid frustum r25->r20 h10 (~15970)
    assert!(volume.unwrap_or(0.0) > 24000.0, "volume too small: {volume:?}");
}

#[test]
fn wide_base_thickened_cone_r21_5_to_r13_5_fuses() {
    let (errors, volume) = build(include_str!("../../../tools/cases/cone_fuse/thicken_a21.json"));
    assert!(errors.is_empty(), "regen errors: {errors:?}");
    // plate ~8982 + thickened cone wall r21.5->r13.5 h27 t1 (~3180)
    assert!(volume.unwrap_or(0.0) > 11000.0, "volume too small: {volume:?}");
}
