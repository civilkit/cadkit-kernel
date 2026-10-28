//! Revolve about an axis line that belongs to another sketch: `axis: {"sketch", "id"}` resolves
//! the line in the owning sketch (entity ids restart in each sketch); the bare id still means a
//! line of the profile's own sketch.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

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

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

/// A rectangle x 5..15, y 0..20 (the tube profile), with an optional own-sketch axis line `ax`.
fn profile(with_axis: bool) -> String {
    let axis_pt = if with_axis { r#",{"id":"o","x":0,"y":0},{"id":"p","x":0,"y":20}"# } else { "" };
    let axis_ent = if with_axis { r#"{"type":"line","id":"ax","p1":"o","p2":"p","construction":true},"# } else { "" };
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},
        "points":[{{"id":"a","x":5,"y":0}},{{"id":"b","x":15,"y":0}},{{"id":"c","x":15,"y":20}},{{"id":"d","x":5,"y":20}}{axis_pt}],
        "entities":[{axis_ent}{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},
        {{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    )
}

/// A second sketch holding only the line `ax` on the Y axis.
const AXIS_SKETCH: &str = r#"{"type":"sketch","id":"F0","plane":{"kind":"std","id":"front"},
    "points":[{"id":"o","x":0,"y":0},{"id":"p","x":0,"y":20}],
    "entities":[{"type":"line","id":"ax","p1":"o","p2":"p","construction":true}],"constraints":[]}"#;

fn tube() -> f64 {
    PI * (15.0 * 15.0 - 5.0 * 5.0) * 20.0
}

fn rev(axis: &str) -> String {
    format!(r#"{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":{axis},"angle":360}}"#)
}

#[test]
fn axis_from_another_sketch() {
    let o = run(&format!("{AXIS_SKETCH},{},{}", profile(false), rev(r#"{"sketch":"F0","id":"ax"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!((vol(&o) - tube()).abs() < 1e-6 * tube(), "{} vs {}", vol(&o), tube());
}

#[test]
fn bare_id_still_means_the_own_sketch() {
    let o = run(&format!("{},{}", profile(true), rev(r#""ax""#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!((vol(&o) - tube()).abs() < 1e-6 * tube());
    // a bare id does not reach into another sketch
    let o = run(&format!("{AXIS_SKETCH},{},{}", profile(false), rev(r#""ax""#)));
    assert!(o.errors.iter().any(|e| e.error.code == "missingAxis"), "{:?}", o.errors);
}

#[test]
fn the_named_sketch_decides_when_ids_collide() {
    // both sketches have a line `ax`; the object form names F0
    let o = run(&format!("{AXIS_SKETCH},{},{}", profile(true), rev(r#"{"sketch":"F0","id":"ax"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!((vol(&o) - tube()).abs() < 1e-6 * tube());
}

#[test]
fn unresolvable_references_are_missing_axis() {
    for axis in [r#"{"sketch":"nope","id":"ax"}"#, r#"{"sketch":"F0","id":"nope"}"#, r#"{"sketch":"F0"}"#, r#"{"id":"ax"}"#, "7"] {
        let o = run(&format!("{AXIS_SKETCH},{},{}", profile(false), rev(axis)));
        assert!(o.errors.iter().any(|e| e.error.code == "missingAxis"), "{axis}: {:?}", o.errors);
    }
}

#[test]
fn surface_revolve_takes_the_same_form() {
    let sr = r#"{"type":"surfaceRevolve","id":"F2","sketch":"F1","axis":{"sketch":"F0","id":"ax"},"angle":360}"#;
    let o = run(&format!("{AXIS_SKETCH},{},{sr}", profile(false)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}
