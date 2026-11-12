//! A fillet on the circular rim of a cylinder: the edge is one closed circle whose only vertex lies exactly on the
//! cylinder's seam, so the blend starts and ends there. The removed volume is closed form (Pappus on the corner
//! region between the square r x r and the quarter circle that rounds it).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::f64::consts::PI;

struct Exec(Brep);
impl FeatureExec for Exec {
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

fn build(radius: f64, fillet: f64) -> (Vec<String>, f64, bool) {
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],
   "entities":[{{"type":"circle","id":"e1","c":"c","r":{radius}}}],"constraints":[]}},
  {{"type":"extrude","id":"F2","op":"add","sketch":"S1","end":"blind","depth":20}},
  {{"type":"fillet","id":"F3","radius":{fillet},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[{radius},0,20]}}]}}]}}"#
    );
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.as_ref().expect("body");
    (errors, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).is_empty())
}

/// Volume of a cylinder of radius `rad` and height `h` whose top rim is rounded with radius `r`.
fn rounded_volume(rad: f64, h: f64, r: f64) -> f64 {
    let (sq, qd) = (r * r, PI * r * r / 4.0);
    let (xs, xq) = (rad - r / 2.0, rad - r + 4.0 * r / (3.0 * PI));
    let removed = 2.0 * PI * (sq * xs - qd * xq);
    PI * rad * rad * h - removed
}

#[test]
fn rim_fillet_of_a_cylinder_closes_on_the_seam() {
    for (rad, r) in [(10.0, 2.0), (10.0, 0.5), (6.0, 5.0)] {
        let (errors, volume, valid) = build(rad, r);
        assert!(errors.is_empty(), "R{rad} r{r}: {errors:?}");
        assert!(valid, "R{rad} r{r}: invalid body");
        let want = rounded_volume(rad, 20.0, r);
        assert!((volume - want).abs() < 1e-3, "R{rad} r{r}: volume {volume}, want {want}");
    }
}
