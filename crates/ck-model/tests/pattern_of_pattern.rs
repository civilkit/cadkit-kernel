//! A pattern whose `seeds` is another pattern includes everything that pattern produced, the original seed
//! included (issue 32: the original column of holes was silently dropped from every copy row).
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

/// 100 x 100 x 10 plate, one through hole of diameter 4 at (10, 10) on the top face, then `tail`.
fn plate(tail: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":100,"y":0}},{{"id":"p3","x":100,"y":100}},{{"id":"p4","x":0,"y":100}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":10,"name":"top"}},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"p1","x":10,"y":10}}],
   "entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4","kind":"simple","diameter":4,"end":"through"}}{tail}
 ]}}"#
    )
}

fn run(src: &str) -> (Vec<String>, f64, usize) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.as_ref().expect("body");
    (errs, ck_topo::mass::mass_props(b).volume, b.num_faces())
}

fn lp(id: &str, seeds: &str, axis: &str) -> String {
    format!(r#",{{"type":"lpattern","id":"{id}","seeds":[{seeds}],"dir":{{"kind":"axis","axis":"{axis}"}},"spacing":15,"count":5}}"#)
}

fn expected_volume(holes: usize) -> f64 {
    100.0 * 100.0 * 10.0 - holes as f64 * std::f64::consts::PI * 4.0 * 10.0
}

#[test]
fn pattern_of_a_pattern_keeps_the_original_seed_column() {
    // 5 holes in a row, then that row copied 4 times along y: 25 holes, not 21.
    let (errs, vol, faces) = run(&plate(&(lp("L6", r#""H5""#, "x") + &lp("L7", r#""L6""#, "y"))));
    assert!(errs.is_empty(), "{errs:?}");
    assert!((vol - expected_volume(25)).abs() < 1e-3, "volume {vol}, want {}", expected_volume(25));
    assert_eq!(faces, 6 + 25, "faces");
}

#[test]
fn pattern_of_a_pattern_equals_the_explicit_seed_list() {
    let nested = run(&plate(&(lp("L6", r#""H5""#, "x") + &lp("L7", r#""L6""#, "y"))));
    let listed = run(&plate(&(lp("L6", r#""H5""#, "x") + &lp("L7", r#""H5","L6""#, "y"))));
    assert!(listed.0.is_empty(), "{:?}", listed.0);
    assert_eq!(nested.2, listed.2);
    assert!((nested.1 - listed.1).abs() < 1e-6, "{} vs {}", nested.1, listed.1);
}
