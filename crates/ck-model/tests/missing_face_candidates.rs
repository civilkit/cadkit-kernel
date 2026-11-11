//! A reference to a face that no longer exists fails with `missingReference` and says what the body has instead, so the
//! reference can be repaired in one step (params `ref`, `candidates`, `hint`).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

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


const BOX: &str = r#"{"type":"sketch","id":"F2","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":30},{"id":"d","x":0,"y":30}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
    {"type":"extrude","id":"F3","op":"add","sketch":"F2","end":"blind","depth":20}"#;

fn shell_of(face: &str) -> Res {
    run(&format!(r#"{BOX},{{"type":"shell","id":"F4","faces":["{face}"],"thickness":2}}"#))
}

#[test]
fn the_box_has_the_faces_the_tests_assume() {
    let r = run(BOX);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let names: Vec<_> = r.faces.iter().map(|f| f.0.clone()).collect();
    assert!(names.iter().any(|n| n == "F3.C1") && names.iter().any(|n| n == "F3.S.e1"), "{names:?}");
}

#[test]
fn a_face_that_exists_still_resolves() {
    let r = shell_of("F3.C1");
    assert!(r.errors.is_empty(), "{:?}", r.errors);
}

#[test]
fn a_gone_face_of_a_live_feature_lists_that_feature_s_faces() {
    // F3 is there, but it has no side from sketch entity e9
    let r = shell_of("F3.S.e9");
    let e = r.errors.first().expect("an error");
    eprintln!("MISSING {e}");
    assert!(e.contains("missingReference"), "{e}");
    assert!(e.contains("\"ref\""), "{e}");
    for want in ["F3.S.e1", "F3.S.e4", "F3.C0", "F3.C1"] {
        assert!(e.contains(want), "{want} missing from {e}");
    }
    assert!(e.contains("the faces of F3 now are"), "{e}");
}

#[test]
fn the_closest_name_comes_first() {
    let r = shell_of("F3.S.e9");
    let e = r.errors.first().expect("an error");
    // names closest to F3.S.e9 are the other sides, before the caps
    let side = e.find("F3.S.e").expect("a side");
    let cap = e.find("F3.C").expect("a cap");
    assert!(side < cap, "{e}");
}

#[test]
fn a_gone_feature_gets_the_closest_names_of_the_whole_model() {
    let r = shell_of("F7.C1");
    let e = r.errors.first().expect("an error");
    assert!(e.contains("missingReference"), "{e}");
    assert!(e.contains("no face of F7 exists at this step"), "{e}");
    assert!(e.contains("F3."), "{e}");
}

fn fillet_of(a: &str, b: &str, at: [f64; 3]) -> Res {
    run(&format!(r#"{BOX},{{"type":"fillet","id":"F4","radius":2,"edges":[{{"faces":["{a}","{b}"],"at":[{},{},{}]}}]}}"#, at[0], at[1], at[2]))
}

#[test]
fn an_edge_between_live_faces_still_fillets() {
    let r = fillet_of("F3.C1", "F3.S.e1", [20.0, 0.0, 20.0]);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
}

#[test]
fn an_edge_pick_with_a_gone_face_says_which_face_and_what_is_there() {
    let r = fillet_of("F3.C1", "F3.S.e9", [20.0, 0.0, 20.0]);
    let e = r.errors.first().expect("an error");
    eprintln!("MISSING EDGE {e}");
    assert!(e.contains("missingEdge"), "{e}");
    assert!(e.contains("\"gone\":[\"F3.S.e9\"]"), "{e}");
    for want in ["F3.S.e1", "F3.S.e4", "F3.C1"] {
        assert!(e.contains(want), "{want} missing from {e}");
    }
}

#[test]
fn an_edge_pick_between_faces_that_no_longer_meet_lists_the_neighbours() {
    // the two long sides of the box face each other: both exist, no edge joins them
    let r = fillet_of("F3.S.e1", "F3.S.e3", [20.0, 0.0, 10.0]);
    let e = r.errors.first().expect("an error");
    eprintln!("MISSING EDGE {e}");
    assert!(e.contains("missingEdge"), "{e}");
    assert!(e.contains("both exist but no edge joins them"), "{e}");
    assert!(e.contains("neighbours"), "{e}");
    assert!(e.contains("F3.C0") && e.contains("F3.C1"), "{e}");
}
