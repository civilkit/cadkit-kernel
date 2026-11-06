//! A pin extruded across the common face of two bodies that only touch (a shaft and a disc cap to cap): the disc's
//! holes decide whether the fuse works.
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

/// Shaft R19.1 x 32, disc R19.1 x 3 on its top (new body) with `holes` holes, pin r3 from z=32 up 10.
fn part(holes: &[(f64, f64)], pin_r: f64, unite_first: bool) -> String {
    let comb = if unite_first { r#",{"type":"combine","id":"C1","op":"union","target":{"lump":0},"tools":[{"lump":1}]}"# } else { "" };
    let mut pts = String::from(r#"{"id":"p0","x":0,"y":0}"#);
    let mut ents = String::from(r#"{"type":"circle","id":"e0","c":"p0","r":19.1}"#);
    for (i, (x, y)) in holes.iter().enumerate() {
        pts += &format!(r#",{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1);
        ents += &format!(r#",{{"type":"circle","id":"e{}","c":"p{}","r":1.25}}"#, i + 1, i + 1);
    }
    format!(
        r#"{{"format":"ck-part","version":1,"name":"t","features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":19.1}}],"constraints":[]}},
  {{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":32}},
  {{"type":"plane","id":"P2","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":32}},
  {{"type":"sketch","id":"S2","plane":{{"kind":"feature","id":"P2"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"newBody","sketch":"S2","end":"blind","depth":3}}{comb},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":32}},
  {{"type":"sketch","id":"S3","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"p1","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":{pin_r}}}],"constraints":[]}},
  {{"type":"extrude","id":"E3","op":"add","sketch":"S3","end":"blind","depth":10}}
 ]}}"#
    )
}

#[test]
fn pin_across_touching_bodies() {
    let h6 = [(-11.258327165, 6.500005342), (11.258333333, 6.499994658), (-0.000006168, -13.0), (-7.5, 12.990381057), (-7.5, -12.990381057), (15.0, 0.0)];
    let area = std::f64::consts::PI;
    for (name, holes) in [("none", &h6[..0]), ("one", &h6[5..]), ("six", &h6[..])] {
        // shaft + disc minus the holes + the pin above the disc (the pin's part inside the two is theirs already)
        let want = area * 19.1 * 19.1 * 35.0 - holes.len() as f64 * area * 1.25 * 1.25 * 3.0 + area * 9.0 * 7.0;
        for unite_first in [false, true] {
            let part = Part::parse(&part(holes, 3.0, unite_first)).unwrap();
            let mut ex = Exec(Brep::new());
            let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
            assert!(res.errors.is_empty(), "{name} unite_first={unite_first}: {:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
            let b = ex.0.body.as_ref().expect("body");
            let v = ck_topo::mass::mass_props(b).volume;
            assert!((v - want).abs() < 1e-3 * want, "{name} unite_first={unite_first}: volume {v}, want {want}");
            assert_eq!(ck_topo::region::components(b).len(), 1, "{name}: shaft, disc and pin are one body");
            assert!(ck_check::validate(b, 1e-6).is_empty(), "{name}: {:?}", ck_check::validate(b, 1e-6));
        }
    }
}

/// A cut and an add with `target` act on their own body: the bodies that touch it are not sewn to it.
#[test]
fn targeted_features_keep_touching_bodies_apart() {
    let part = Part::parse(
        r#"{"format":"ck-part","version":1,"name":"t","features":[
  {"type":"sketch","id":"S1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":19.1}],"constraints":[]},
  {"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":32},
  {"type":"plane","id":"P2","mode":"offset","base":{"kind":"std","id":"top"},"offset":32},
  {"type":"sketch","id":"S2","plane":{"kind":"feature","id":"P2"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":19.1}],"constraints":[]},
  {"type":"extrude","id":"E2","op":"newBody","sketch":"S2","end":"blind","depth":3},
  {"type":"plane","id":"P3","mode":"offset","base":{"kind":"std","id":"top"},"offset":35},
  {"type":"sketch","id":"S3","plane":{"kind":"feature","id":"P3"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":10}],"constraints":[]},
  {"type":"extrude","id":"E3","op":"newBody","sketch":"S3","end":"blind","depth":5},
  {"type":"plane","id":"P4","mode":"offset","base":{"kind":"std","id":"top"},"offset":40},
  {"type":"sketch","id":"S4","plane":{"kind":"feature","id":"P4"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":7.5}],"constraints":[]},
  {"type":"extrude","id":"E4","op":"cut","sketch":"S4","end":"blind","depth":1.6,"target":"E3.S.e1"},
  {"type":"plane","id":"P5","mode":"offset","base":{"kind":"std","id":"top"},"offset":35},
  {"type":"sketch","id":"S5","plane":{"kind":"feature","id":"P5"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":3}],"constraints":[]},
  {"type":"extrude","id":"E5","op":"add","sketch":"S5","end":"blind","depth":7,"target":"E3.S.e1"}
 ]}"#,
    )
    .unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let b = ex.0.body.as_ref().expect("body");
    assert_eq!(ck_topo::region::components(b).len(), 3, "shaft, disc and the boss with its pin stay three bodies");
    let pi = std::f64::consts::PI;
    // shaft + disc + boss minus the pocket (the pin fills the pocket floor to y=42: pin 35..42 inside r3, pocket 38.4..40 around it)
    let want = pi * 19.1 * 19.1 * 35.0 + pi * 100.0 * 5.0 - pi * (7.5 * 7.5 - 9.0) * 1.6 + pi * 9.0 * 2.0;
    let v = ck_topo::mass::mass_props(b).volume;
    assert!((v - want).abs() < 1e-3 * want, "volume {v}, want {want}");
}
