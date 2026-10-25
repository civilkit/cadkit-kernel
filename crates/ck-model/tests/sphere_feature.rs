//! The `sphere` feature (centre, radius): the same ball as a revolved half disc, with no sketch.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

const BLOCK: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":-20,"y":-20},{"id":"b","x":20,"y":-20},{"id":"c","x":20,"y":20},{"id":"d","x":-20,"y":20}],
  "entities":[{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"},{"type":"line","id":"l3","p1":"c","p2":"d"},{"type":"line","id":"l4","p1":"d","p2":"a"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":40}"#;

struct Res {
    errors: Vec<String>,
    vol: f64,
    faces: Vec<(String, &'static str)>,
    valid: Vec<String>,
}

fn run(feats: &str) -> Res {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let Some(b) = ex.brep.body.as_ref() else { return Res { errors, vol: 0.0, faces: vec![], valid: vec![] } };
    let faces = b.face_ids().map(|f| (b.face_id_str(f), b.face(f).surface.kind())).collect();
    let valid = ck_check::validate(b, 1e-3).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    Res { errors, vol: ck_topo::mass::mass_props(b).volume, faces, valid }
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

fn ball(id: &str, c: [f64; 3], r: &str, op: &str) -> String {
    format!(r#"{{"type":"sphere","id":"{id}","op":"{op}","center":[{},{},{}],"radius":"{r}"}}"#, c[0], c[1], c[2])
}

#[test]
fn a_lone_sphere_is_one_valid_spherical_face() {
    let r = run(&ball("F1", [3.0, 4.0, 5.0], "5", "add"));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.valid.is_empty(), "{:?}", r.valid);
    assert!(near(r.vol, 4.0 / 3.0 * PI * 125.0, 1e-6), "{}", r.vol);
    assert_eq!(r.faces, [("F1.S.e1".to_string(), "sphere")]);
}

#[test]
fn joined_and_cut_against_a_block_it_is_the_revolved_ball() {
    let v0 = 40.0 * 40.0 * 40.0;
    let dv = 2.0 / 3.0 * PI * 216.0;
    // centre on the top face: the hemisphere is added, then cut
    let add = run(&format!("{BLOCK},{}", ball("F3", [0.0, 40.0, 0.0], "6", "add")));
    assert!(add.errors.is_empty() && add.valid.is_empty(), "{:?} {:?}", add.errors, add.valid);
    assert!(near(add.vol, v0 + dv, 1e-3), "{}", add.vol);
    let cut = run(&format!("{BLOCK},{}", ball("F3", [0.0, 40.0, 0.0], "6", "cut")));
    assert!(cut.errors.is_empty() && cut.valid.is_empty(), "{:?} {:?}", cut.errors, cut.valid);
    assert!(near(cut.vol, v0 - dv, 1e-3), "{}", cut.vol);
}

#[test]
fn radius_is_an_expression_and_must_be_positive() {
    let src = format!(
        r#"{{"format":"ck-part","version":1,"variables":[{{"name":"r","expr":"2*3"}}],"features":[{{"type":"sphere","id":"F1","op":"add","center":[0,0,0],"radius":"=r"}}]}}"#
    );
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    assert!(near(ck_topo::mass::mass_props(ex.brep.body.as_ref().unwrap()).volume, 4.0 / 3.0 * PI * 216.0, 1e-6));
    for bad in ["0", "-1"] {
        let r = run(&ball("F1", [0.0; 3], bad, "add"));
        assert!(r.errors.iter().any(|e| e.contains("invalidValue")), "{bad}: {:?}", r.errors);
    }
    let r = run(&ball("F1", [0.0; 3], "3", "cut"));
    assert!(r.errors.iter().any(|e| e.contains("nothingToCut")), "{:?}", r.errors);
}

#[test]
fn as_a_new_body_it_stays_separate_and_carries_its_name() {
    let r = run(&format!("{BLOCK},{}", r#"{"type":"sphere","id":"F3","op":"newBody","body":"Ball","center":[60,20,0],"radius":"5"}"#));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, 64000.0 + 4.0 / 3.0 * PI * 125.0, 1e-6), "{}", r.vol);
    assert!(r.faces.iter().any(|(n, k)| n == "F3.S.e1" && *k == "sphere"), "{:?}", r.faces);
}
