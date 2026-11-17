//! A chamfer leg across a disc longer than the disc's radius (solidcraft kernel case `2893-chamfer-two-distances-past-the-top-refused`):
//! the top rim of a cylinder r 10 x 20 chamfered 12 across the top and 3 down the wall runs past the axis, with no face beyond it to
//! meet. It used to build a body whose chamfer face left its surface (EDGE_OFF_SURFACE, no error); now it is refused with the
//! sentence of the wider-than-its-face refusals and the disc's radius as the largest distance. The cone to the centre (10 across)
//! still builds.
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

const CYL: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":10}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#;

/// (errors as `code params`, volume, validity issues) of the cylinder with its top rim chamfered `across` over the top and 3 down the wall.
fn run(across: f64) -> (Vec<String>, f64, usize) {
    let f = format!(
        r#"{{"type":"chamfer","id":"F3","chamferType":"distanceDistance","distance":{across},"distance2":3,"face":"F2.C1","edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[10,0,20]}}]}}"#
    );
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{CYL},{f}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| format!("{} {}", e.error.code, ck_json::to_string(&ck_json::Json::Obj(e.error.params.clone())))).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

#[test]
fn a_chamfer_leg_past_the_axis_of_a_disc_is_refused() {
    let (errs, v, issues) = run(12.0);
    assert_eq!(errs.len(), 1, "built with volume {v}, {issues} issues");
    assert!(errs[0].starts_with("blendOverflow"), "{}", errs[0]);
    assert!(errs[0].contains("wider than the face beside its edge") && errs[0].contains("cannot meet the face beyond it"), "{}", errs[0]);
    assert!(errs[0].contains("largest distance on it is 10"), "{}", errs[0]);
    // the whole top (a cone to the centre) is the most it takes: built, valid, the cone's volume
    let (errs, v, issues) = run(10.0);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    // cylinder minus the ring section: pi 100 20 less the triangle (10 x 3 / 2) swept round the axis at its centroid radius 10 - 10/3
    let want = std::f64::consts::PI * 100.0 * 20.0 - 2.0 * std::f64::consts::PI * (10.0 - 10.0 / 3.0) * 15.0;
    assert!((v - want).abs() < 1e-6 * want, "{v} vs {want}");
}
