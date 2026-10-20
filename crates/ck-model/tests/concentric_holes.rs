//! SolidCraft #1355: a D8.5 blind hole with a 118-degree drill tip, then a D4 through
//! hole on the same axis. The second cut leaves the drill tip as an annular cone band
//! whose whole-domain loop (apex, base circle, apex) has a seam piece sampled only at its
//! pole; the seam twins must still land one period apart, or the band's uv polygon is a
//! zero-area spike and the cut fails (`booleanFailed`).
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

const PART: &str = r#"{"format":"ck-part","version":1,"name":"concentric-holes",
 "features":[
  {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[
     {"id":"p1","x":-15,"y":-15},{"id":"p2","x":15,"y":-15},{"id":"p3","x":15,"y":15},{"id":"p4","x":-15,"y":15}],
   "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
     {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
  {"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10},
  {"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10,"name":"z=10"},
  {"type":"sketch","id":"S4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"p1","x":0,"y":0}],
   "entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
  {"type":"hole","id":"H1","sketch":"S4","kind":"simple","diameter":8.5,"end":"blind","depth":5,"tip":true},
  {"type":"plane","id":"F5","mode":"offset","base":{"kind":"std","id":"front"},"offset":10,"name":"z=10"},
  {"type":"sketch","id":"S6","plane":{"kind":"feature","id":"F5"},"points":[{"id":"p1","x":0,"y":0}],
   "entities":[{"type":"point","id":"e1","p":"p1"}],"constraints":[]},
  {"type":"hole","id":"H2","sketch":"S6","kind":"simple","diameter":4,"end":"through"}
 ]}"#;

#[test]
fn drill_tip_hole_with_a_coaxial_through_hole() {
    let part = Part::parse(PART).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let b = ex.0.body.as_ref().expect("body");
    let mp = ck_topo::mass::mass_props(b);
    // plate 9000 - (D8.5 x 5 + 118 deg tip + the D4 hole below the tip) = 8627.1633
    assert!((mp.volume - 8627.1633).abs() < 5e-3, "volume {}", mp.volume);
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6));
}
