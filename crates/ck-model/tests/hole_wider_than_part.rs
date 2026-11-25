//! A countersink / counterbore wider than the part, or a cut that removes the whole body, is refused by name and
//! leaves the body as it was (issue: a countersink wider than the part deleted the body with no error).
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

/// A 30 x 30 x 10 plate, then `tail` features.
fn part(tail: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"name":"plate","features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":-15,"y":-15}},{{"id":"p2","x":15,"y":-15}},{{"id":"p3","x":15,"y":15}},{{"id":"p4","x":-15,"y":15}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"F3","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":10,"name":"z=10"}},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"F3"}},"points":[{{"id":"p1","x":0,"y":0}}],
   "entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}}
  {tail}
 ]}}"#
    )
}

fn run(tail: &str) -> (regen::RegenResult, f64) {
    let p = Part::parse(&part(tail)).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&p, &RegenOpts::default(), &mut ex);
    let v = ex.0.body.as_ref().map(|b| { eprintln!("faces {} lumps {}", b.num_faces(), b.lump_ids().count()); ck_topo::mass::mass_props(b).volume }).unwrap_or(-1.0);
    (res, v)
}

const PLATE_V: f64 = 30.0 * 30.0 * 10.0;

fn code_of(res: &regen::RegenResult, feature: &str) -> Option<String> {
    res.errors.iter().find(|e| e.feature == feature).map(|e| e.error.code.clone())
}
fn param(res: &regen::RegenResult, name: &str) -> String {
    res.errors[0].error.params.iter().find(|(n, _)| n == name).map(|(_, v)| ck_json::to_string(v)).unwrap_or_default()
}

fn hole(extra: &str) -> String {
    format!(r#",{{"type":"hole","id":"H1","sketch":"S4","diameter":4,{extra}}}"#)
}

#[test]
fn a_countersink_wider_than_the_part_refuses_and_keeps_the_body() {
    for extra in [
        r#""kind":"csink","cskDiameter":100,"end":"through""#,
        r#""kind":"csink","cskDiameter":100,"end":"blind","depth":60"#,
    ] {
        let (res, v) = run(&hole(extra));
        assert_eq!(code_of(&res, "H1").as_deref(), Some("holeCskDiameter"), "{extra}: {:?}", res.errors);
        assert!(param(&res, "detail").contains("part"), "{}", param(&res, "detail"));
        assert!(param(&res, "hint").contains("cskDiameter"));
        assert!((v - PLATE_V).abs() < 1e-6, "{extra}: body changed, volume {v}");
    }
}

/// Wider than the cone depth the plate allows but narrower than the part: the cone eats through the plate's own
/// thickness and is refused by the existing depth check, not by the new guard. A cone that fits builds exactly.
#[test]
fn a_countersink_that_fits_builds_with_the_closed_form_volume() {
    let (res, v) = run(&hole(r#""kind":"csink","cskDiameter":8,"cskAngle":90,"end":"through""#));
    assert!(res.errors.is_empty(), "{:?}", res.errors);
    let pi = std::f64::consts::PI;
    let want = PLATE_V - 40.0 * pi - 32.0 * pi / 3.0;
    assert!((v - want).abs() < 1e-6 * PLATE_V, "volume {v} want {want}");
}

#[test]
fn a_counterbore_wider_than_the_part_refuses_and_keeps_the_body() {
    let (res, v) = run(&hole(r#""kind":"cbore","cboreDiameter":100,"cboreDepth":3,"end":"through""#));
    assert_eq!(code_of(&res, "H1").as_deref(), Some("holeCboreInvalid"), "{:?}", res.errors);
    assert!(param(&res, "detail").contains("wider than the part"));
    assert!((v - PLATE_V).abs() < 1e-6, "body changed, volume {v}");
}

#[test]
fn a_counterbore_that_fits_still_builds() {
    let (res, v) = run(&hole(r#""kind":"cbore","cboreDiameter":8,"cboreDepth":3,"end":"through""#));
    assert!(res.errors.is_empty(), "{:?}", res.errors);
    let pi = std::f64::consts::PI;
    let want = PLATE_V - 40.0 * pi - 36.0 * pi;
    assert!((v - want).abs() < 1e-6 * PLATE_V, "volume {v} want {want}");
}

#[test]
fn a_cut_through_the_whole_body_refuses_and_keeps_the_body() {
    let tail = r#",{"type":"sketch","id":"S5","plane":{"kind":"feature","id":"F3"},"points":[
     {"id":"p1","x":-40,"y":-40},{"id":"p2","x":40,"y":-40},{"id":"p3","x":40,"y":40},{"id":"p4","x":-40,"y":40}],
   "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
     {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
  {"type":"extrude","id":"C6","op":"cut","sketch":"S5","end":"through"}"#;
    let (res, v) = run(tail);
    assert_eq!(code_of(&res, "C6").as_deref(), Some("cutRemovedBody"), "{:?}", res.errors);
    assert!(param(&res, "hint").contains("smaller"));
    assert!((v - PLATE_V).abs() < 1e-6, "body changed, volume {v}");
}
