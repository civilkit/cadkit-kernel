//! A round boss fused onto a body whose D-shaped pin already shares its cylinder (the pin's arc is an arc of the
//! boss circle, the pin's flat cuts off a sliver of the boss): the two cylinders coincide over the arc, the sliver
//! stays as new material.
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

fn run(part: &str) -> (Vec<String>, f64) {
    let part = Part::parse(part).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| e.error.describe()).collect();
    let v = ex.0.body.as_ref().map(|b| ck_topo::mass::mass_props(b).volume).unwrap_or(0.0);
    (errs, v)
}

fn part(d_from: f64, d_depth: f64, c_from: f64, c_depth: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"name":"t","features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":19}}],"constraints":[]}},
  {{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"P2","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{d_from}}},
  {{"type":"sketch","id":"S2","plane":{{"kind":"feature","id":"P2"}},"points":[{{"id":"p1","x":2.5,"y":-1.658312395}},{{"id":"p2","x":2.5,"y":1.658312395}},{{"id":"p3","x":0,"y":0}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"arc","id":"e2","c":"p3","p1":"p2","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S2","end":"blind","depth":{d_depth}}},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{c_from}}},
  {{"type":"sketch","id":"S3","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"p1","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":3}}],"constraints":[]}},
  {{"type":"extrude","id":"E3","op":"add","sketch":"S3","end":"blind","depth":{c_depth}}}
 ]}}"#
    )
}

#[test]
fn round_boss_over_a_d_pin_of_its_own_circle() {
    // pin 8..18 (x<2.5 of the circle), boss 5..15: the boss runs through the base top (10) and along the pin
    for (df, dd, cf, cd) in [(8.0, 10.0, 5.0, 10.0), (10.0, 8.0, 0.0, 14.0), (10.0, 8.0, 10.0, 4.0), (10.0, 8.0, 10.0, 8.0), (10.0, 8.0, 10.0, 12.0)] {
        let (errs, v) = run(&part(df, dd, cf, cd));
        assert!(errs.is_empty(), "pin {df}+{dd}, boss {cf}+{cd}: {errs:?}");
        // base 11341.1 + pin above the base + the boss sliver; sanity: more than the base
        assert!(v > 11340.0, "{v}");
    }
}
