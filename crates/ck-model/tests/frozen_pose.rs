//! Broken-reference survival (frozen pose): when an upstream face, plane, or axis disappears,
//! a sketch, plane, or axis carrying `frozenFrame` / `frozenAxis` stays alive at its last known
//! pose with a `detachedReference` warning so downstream features keep building.
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

fn rect_sketch(id: &str, plane: &str, extra: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane}{extra},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":20,"y":0}},{{"id":"p3","x":20,"y":20}},{{"id":"p4","x":0,"y":20}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

fn run(features: &str) -> (regen::RegenResult, f64) {
    let json = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&json).unwrap()).unwrap();
    let mut exec = Exec(Brep::default());
    let res = regen::regen(&part, &RegenOpts::default(), &mut exec);
    let vol = exec.0.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    (res, vol)
}

const TOP_AT_10: &str = r#","frozenFrame":{"o":[0,10,0],"x":[1,0,0],"y":[0,0,-1],"z":[0,1,0]}"#;

#[test]
fn sketch_on_deleted_face_survives_at_frozen_frame_and_builds_extrude() {
    // F3 references F2.C1 which does not exist (as if F2 was deleted or suppressed),
    // but F3 carries frozenFrame so F3 and F4 (20x20x5 = 2000 mm^3) still build.
    let f3 = rect_sketch("F3", r#"{"kind":"face","id":"F2.C1"}"#, TOP_AT_10);
    let f4 = r#"{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}"#;
    let (res, vol) = run(&format!("{f3},{f4}"));
    assert!(res.errors.is_empty(), "expected no errors, got {:?}", res.errors);
    assert_eq!(res.warnings.len(), 1);
    assert_eq!(res.warnings[0].feature, "F3");
    assert_eq!(res.warnings[0].error.code, "detachedReference");
    assert!((vol - 2000.0).abs() < 1e-3, "expected 2000 mm^3, got {vol}");
    let sk = res.sketches.iter().find(|s| s.id == "F3").expect("F3 sketch present");
    assert!((sk.frame.o.y - 10.0).abs() < 1e-9);
}

#[test]
fn sketch_without_frozen_frame_still_errors_missing_reference() {
    let f3 = rect_sketch("F3", r#"{"kind":"face","id":"F2.C1"}"#, "");
    let f4 = r#"{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}"#;
    let (res, vol) = run(&format!("{f3},{f4}"));
    assert_eq!(res.errors.len(), 2);
    assert_eq!(res.errors[0].error.code, "missingReference");
    assert_eq!(res.errors[1].error.code, "missingSketch");
    assert!(res.warnings.is_empty());
    assert_eq!(vol, 0.0);
}

#[test]
fn live_reference_takes_precedence_over_frozen_frame_with_no_warning() {
    // F1/F2 builds a 20x20x10 box whose top cap F2.C1 is at y = 10.
    // F3 has a stale frozenFrame at y = 99, but F2.C1 IS live, so F3 uses y = 10 and emits no warning.
    let f1 = rect_sketch("F1", r#"{"kind":"std","id":"top"}"#, "");
    let f2 = r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
    let stale = r#","frozenFrame":{"o":[0,99,0],"x":[1,0,0],"y":[0,0,-1],"z":[0,1,0]}"#;
    let f3 = rect_sketch("F3", r#"{"kind":"face","id":"F2.C1"}"#, stale);
    let (res, _) = run(&format!("{f1},{f2},{f3}"));
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    assert!(res.warnings.is_empty(), "warnings: {:?}", res.warnings);
    let sk = res.sketches.iter().find(|s| s.id == "F3").expect("F3 sketch present");
    assert!((sk.frame.o.y - 10.0).abs() < 1e-9, "live frame must win over frozenFrame: {:?}", sk.frame.o);
}

#[test]
fn plane_feature_survives_missing_base_at_frozen_frame() {
    // P1 offset plane references missing face F2.C1, holds frozenFrame at y=15,
    // and F3 (sketch on P1) + F4 (extrude) build cleanly without error.
    let p1 = r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"face","id":"F2.C1"},"offset":5,"frozenFrame":{"o":[0,15,0],"x":[1,0,0],"y":[0,0,-1],"z":[0,1,0]}}"#;
    let f3 = rect_sketch("F3", r#"{"kind":"feature","id":"P1"}"#, "");
    let f4 = r#"{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}"#;
    let (res, vol) = run(&format!("{p1},{f3},{f4}"));
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    assert_eq!(res.warnings.len(), 1);
    assert_eq!(res.warnings[0].feature, "P1");
    assert_eq!(res.warnings[0].error.code, "detachedReference");
    assert!((vol - 2000.0).abs() < 1e-3, "expected 2000 mm^3, got {vol}");
    let pl = res.planes.iter().find(|p| p.0 == "P1").expect("P1 plane present");
    assert!((pl.1.o.y - 15.0).abs() < 1e-9);
}

#[test]
fn axis_feature_survives_missing_ref_at_frozen_axis() {
    // A1 references missing plane P0, holds frozenAxis along Z, and P2 (angle plane about A1) builds.
    let a1 = r#"{"type":"axis","id":"A1","refs":[{"kind":"feature","id":"P0"},{"kind":"std","id":"front"}],"frozenAxis":{"o":[0,0,0],"d":[0,0,1]}}"#;
    let p2 = r#"{"type":"plane","id":"P2","mode":"angle","base":{"kind":"std","id":"top"},"axis":{"kind":"feature","id":"A1"},"angle":30}"#;
    let (res, _) = run(&format!("{a1},{p2}"));
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    assert_eq!(res.warnings.len(), 1);
    assert_eq!(res.warnings[0].feature, "A1");
    assert_eq!(res.warnings[0].error.code, "detachedReference");
    assert!(res.planes.iter().any(|p| p.0 == "P2"));
}
