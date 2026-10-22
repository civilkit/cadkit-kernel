//! A variable fillet whose radius closes to 0 at one end is not refused: the nearest end radius the
//! builder accepts is used and a `filletRadiusApproximated` warning says which one.
//! Reference: 40 x 30 x 20 box, the 40 mm top front edge rounded 4 -> 0, V = abc - (1 - pi/4) L a^2 / 3.
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

fn part(radii: &str) -> String {
    format!(
        r#"{{"format":"ck-part","features":[
 {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"p1","x":40,"y":0}},{{"id":"p2","x":40,"y":30}},{{"id":"p3","x":0,"y":30}}],
  "entities":[{{"type":"line","p1":"O","p2":"p1","id":"e1"}},{{"type":"line","p1":"p1","p2":"p2","id":"e2"}},{{"type":"line","p1":"p2","p2":"p3","id":"e3"}},{{"type":"line","p1":"p3","p2":"O","id":"e4"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}},
 {{"type":"fillet","id":"F3","edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[4,0,20]}}],"filletType":"variable","radii":{radii}}}]}}"#
    )
}

fn run(radii: &str) -> (Vec<String>, Vec<String>, f64) {
    let p = Part::parse(&part(radii)).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&p, &RegenOpts::default(), &mut ex);
    let errs = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let warns = r.warnings.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let v = ex.0.body.as_ref().map(|b| ck_topo::mass::mass_props(b).volume).unwrap_or(0.0);
    (errs, warns, v)
}

#[test]
fn a_round_closing_to_zero_is_approximated_with_a_warning() {
    let (errs, warns, v) = run("[4,0]");
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(warns.len(), 1, "{warns:?}");
    let w = &warns[0];
    assert!(w.starts_with("F3:filletRadiusApproximated"), "{w}");
    for key in ["requested", "used", "detail", "hint"] {
        assert!(w.contains(key), "{key} in {w}");
    }
    // exact value 23954.2183; the approximation is within 0.1 percent
    let exact = 24000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 40.0 * 16.0 / 3.0;
    assert!((v - exact).abs() / exact < 1e-3, "volume {v} vs {exact}");
    // either end may be the zero one
    let (errs, warns, v2) = run("[0,4]");
    assert!(errs.is_empty() && warns.len() == 1, "{errs:?} {warns:?}");
    assert!((v2 - exact).abs() / exact < 1e-3, "volume {v2} vs {exact}");
}

#[test]
fn positive_radii_are_unchanged_and_silent() {
    let (errs, warns, v) = run("[4,1]");
    assert!(errs.is_empty() && warns.is_empty(), "{errs:?} {warns:?}");
    assert!(v < 24000.0 && v > 23900.0, "{v}");
}

#[test]
fn bad_radii_are_refused_with_a_detail_and_a_hint() {
    for r in ["[0,0]", "[-1,2]", "[4]", "[4,5,6]"] {
        let (errs, warns, _) = run(r);
        assert!(warns.is_empty(), "{r}: {warns:?}");
        assert_eq!(errs.len(), 1, "{r}: {errs:?}");
        assert!(errs[0].starts_with("F3:invalidValue"), "{r}: {}", errs[0]);
    }
    for r in ["[0,0]", "[-1,2]"] {
        let (errs, _, _) = run(r);
        assert!(errs[0].contains("detail") && errs[0].contains("hint"), "{r}: {}", errs[0]);
    }
}
