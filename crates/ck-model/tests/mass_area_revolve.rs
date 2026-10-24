//! `massProps` area of a revolved body whose arc is centred on the far side of the axis (a spindle-shaped torus
//! face). Its loops come out clockwise in the face's parameter domain while `sense` is true: the volume terms
//! cancel the pair of signs and were right, the area was added as a NEGATIVE number (4185.01 for a vase of
//! 13156.61). Found with an independent reference set (analytic values, Pappus / Green).
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

/// Profile: bottom disc r20 at z = 0, cone r20 -> r25, a spindle-torus arc (centre (-16, -33), R = 41.1096),
/// cone r15 -> r18, top disc r18; revolved 360 about the vertical axis.
const VASE: &str = r#"{"format":"ck-part","features":[{"type":"sketch","name":"Profile","plane":{"kind":"std","id":"top"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p1","x":20,"y":0},{"id":"p2","x":25,"y":-30},{"id":"p3","x":-16,"y":-33},{"id":"p4","x":15,"y":-60},{"id":"p5","x":18,"y":-80},{"id":"p6","x":0,"y":-80},{"id":"p7","x":0,"y":-10}],"entities":[{"type":"line","p1":"O","p2":"p1","id":"e1"},{"type":"line","p1":"p1","p2":"p2","id":"e2"},{"type":"arc","c":"p3","p1":"p4","p2":"p2","id":"e3"},{"type":"line","p1":"p4","p2":"p5","id":"e4"},{"type":"line","p1":"p5","p2":"p6","id":"e5"},{"type":"line","p1":"p6","p2":"O","id":"e6"},{"type":"line","p1":"O","p2":"p7","id":"e7","construction":true}],"constraints":[],"seq":1,"visible":false,"id":"F1"},{"type":"revolve","op":"add","sketch":"F1","axis":"e7","end":"blind","angle":360,"name":"Revolve1","id":"F2"}]}"#;
/// The same profile mirrored to the other side of the axis (the arc runs the other way round).
const VASE_MIRRORED: &str = r#"{"format":"ck-part","features":[{"type":"sketch","name":"Profile","plane":{"kind":"std","id":"top"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p1","x":-20,"y":0},{"id":"p2","x":-25,"y":-30},{"id":"p3","x":16,"y":-33},{"id":"p4","x":-15,"y":-60},{"id":"p5","x":-18,"y":-80},{"id":"p6","x":0,"y":-80},{"id":"p7","x":0,"y":-10}],"entities":[{"type":"line","p1":"O","p2":"p1","id":"e1"},{"type":"line","p1":"p1","p2":"p2","id":"e2"},{"type":"arc","c":"p3","p1":"p2","p2":"p4","id":"e3"},{"type":"line","p1":"p4","p2":"p5","id":"e4"},{"type":"line","p1":"p5","p2":"p6","id":"e5"},{"type":"line","p1":"p6","p2":"O","id":"e6"},{"type":"line","p1":"O","p2":"p7","id":"e7","construction":true}],"constraints":[],"seq":1,"visible":false,"id":"F1"},{"type":"revolve","op":"add","sketch":"F1","axis":"e7","end":"blind","angle":360,"name":"Revolve1","id":"F2"}]}"#;

// Pappus / Green on the (r, z) profile: V = 2 pi int x dA, A = sum over profile edges of 2 pi int x ds
const VOLUME: f64 = 112499.47865571639;
const AREA: f64 = 13156.614546780698;

fn props(src: &str) -> (f64, f64, Vec<f64>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let b = ex.0.body.as_ref().expect("body");
    assert!(ck_check::validate(b, 1e-6).is_empty());
    let refp = ck_topo::mass::ref_point_of(b);
    let per_face = b.face_ids().map(|f| ck_topo::mass::faces_moments(b, &[f], refp).area).collect();
    let mp = ck_topo::mass::mass_props(b);
    (mp.volume, mp.area, per_face)
}

#[test]
fn vase_area_matches_pappus() {
    for (name, src) in [("vase", VASE), ("mirrored", VASE_MIRRORED)] {
        let (v, a, per_face) = props(src);
        assert!((v - VOLUME).abs() / VOLUME < 1e-9, "{name}: volume {v}");
        assert!((a - AREA).abs() / AREA < 1e-9, "{name}: area {a} vs {AREA}");
        // no face reports a negative area
        assert!(per_face.iter().all(|x| *x > 0.0), "{name}: {per_face:?}");
    }
}
