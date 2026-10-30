//! Two coincident solids in one part (a body and an exact copy of it, e.g. a converted part that carries its original solid next
//! to the rebuilt one): an add / cut sketched on a face of one of them used to fail as a boolean with both. It now acts on the lump
//! that holds the face and leaves the other as it was.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

fn cube(id: &str, ext: &str, name: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":10,"y":0}},{{"id":"p3","x":10,"y":10}},{{"id":"p4","x":0,"y":10}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},{{"type":"extrude","id":"{id}","op":"{ext}","sketch":"S{id}","end":"blind","depth":10{body}}}"#,
        body = if ext == "newBody" { format!(r#","body":"{name}""#) } else { String::new() }
    )
}

/// two coincident cubes (the second as a new body), then a 5 mm circle on the top face of the first and `extra`
fn run(extra: &str) -> (Vec<f64>, String) {
    let text = format!(
        r#"{{"format":"ck-part","version":1,"name":"two","features":[{},{},{{"type":"sketch","id":"SC","plane":{{"kind":"face","id":"E1.C1"}},"points":[{{"id":"c","x":5,"y":5}}],"entities":[{{"type":"circle","id":"k","c":"c","r":2.5}}],"constraints":[]}},{}]}}"#,
        cube("E1", "add", "History"), cube("E2", "newBody", "Original"), extra
    );
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.to_json().to_string();
    let b = ex.0.body.clone().expect("a body");
    let vols = ck_topo::mass::lump_moments(&b).iter().map(|m| m.volume).collect();
    assert!(ck_check::validate(&b, 1e-6).is_empty(), "{errs}");
    (vols, errs)
}

const HOLE: f64 = std::f64::consts::PI * 2.5 * 2.5;

#[test]
fn cut_on_a_face_of_one_of_two_coincident_cubes() {
    let (v, errs) = run(r#"{"type":"extrude","id":"X","op":"cut","sketch":"SC","end":"blind","depth":10}"#);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    assert_eq!(v.len(), 2, "{v:?}");
    let mut v = v;
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!((v[0] - (1000.0 - HOLE * 10.0)).abs() < 1e-3 && (v[1] - 1000.0).abs() < 1e-6, "{v:?}");
}

#[test]
fn add_on_a_face_of_one_of_two_coincident_cubes() {
    let (v, errs) = run(r#"{"type":"extrude","id":"X","op":"add","sketch":"SC","end":"blind","depth":5}"#);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    let mut v = v;
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(v.len(), 2, "{v:?}");
    assert!((v[0] - 1000.0).abs() < 1e-6 && (v[1] - (1000.0 + HOLE * 5.0)).abs() < 1e-3, "{v:?}");
}

#[test]
fn new_body_on_a_face_stays_separate() {
    let (v, errs) = run(r#"{"type":"extrude","id":"X","op":"newBody","sketch":"SC","end":"blind","depth":10,"body":"Boss"}"#);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    assert_eq!(v.len(), 3, "{v:?}");
}

#[test]
fn target_names_the_body_a_cut_acts_on() {
    // the sketch is on a face of the first cube; `target` sends the cut to the other, named one
    let (v, errs) = run(r#"{"type":"extrude","id":"X","op":"cut","sketch":"SC","end":"blind","depth":10,"target":{"body":"Original"}}"#);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    assert_eq!(v.len(), 2, "{v:?}");
    // lumps: the untouched one first, then the result
    assert!((v[0] - 1000.0).abs() < 1e-6 && (v[1] - (1000.0 - HOLE * 10.0)).abs() < 1e-3, "{v:?}");
}

#[test]
fn a_target_that_is_no_body_is_refused() {
    let (_, errs) = {
        let text = format!(
            r#"{{"format":"ck-part","version":1,"name":"two","features":[{},{},{{"type":"sketch","id":"SC","plane":{{"kind":"face","id":"E1.C1"}},"points":[{{"id":"c","x":5,"y":5}}],"entities":[{{"type":"circle","id":"k","c":"c","r":2.5}}],"constraints":[]}},{{"type":"extrude","id":"X","op":"cut","sketch":"SC","end":"blind","depth":10,"target":{{"body":"Nope"}}}}]}}"#,
            cube("E1", "add", "History"), cube("E2", "newBody", "Original")
        );
        let part = Part::parse(&text).unwrap();
        let mut ex = Exec(Brep::new());
        let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
        (0, r.to_json().to_string())
    };
    assert!(errs.contains(r#""code":"missingReference""#), "{errs}");
}

// ---- bodies that are not copies of each other ----

fn circle(id: &str, face: &str, x: f64, y: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"face","id":"{face}"}},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"k","c":"c","r":2.5}}],"constraints":[]}}"#)
}

fn regen_features(features: &[String]) -> (Vec<f64>, String) {
    let text = format!(r#"{{"format":"ck-part","version":1,"name":"p","features":[{}]}}"#, features.join(","));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.to_json().to_string();
    let vols = ex.0.body.as_ref().map(|b| ck_topo::mass::lump_moments(b).iter().map(|m| m.volume).collect()).unwrap_or_default();
    (vols, errs)
}

const CUT: &str = r#"{"type":"extrude","id":"X","op":"cut","sketch":"SC","end":"blind","depth":10}"#;

/// a cube with a through hole, and over it a solid copy of the cube as its own body (the original solid and a history body that lacks the hole)
fn holed_and_solid() -> Vec<String> {
    vec![
        cube("E1", "add", "A"),
        circle("SH", "E1.C1", 5.0, 5.0),
        r#"{"type":"extrude","id":"EH","op":"cut","sketch":"SH","end":"blind","depth":10}"#.to_string(),
        cube("E2", "newBody", "Solid"),
    ]
}

#[test]
fn a_cut_where_one_body_already_has_the_hole_cuts_the_other() {
    let mut f = holed_and_solid();
    f.push(circle("SC", "E2.C1", 5.0, 5.0));
    f.push(CUT.to_string());
    let (mut v, errs) = regen_features(&f);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(v.len(), 2, "{v:?}");
    assert!((v[0] - (1000.0 - HOLE * 10.0)).abs() < 1e-3 && (v[1] - (1000.0 - HOLE * 10.0)).abs() < 1e-3, "{v:?}");
}

#[test]
fn a_cut_from_the_face_of_the_holed_body_leaves_the_solid_one_cut_too() {
    // Automatic: every body the shape reaches. The holed body is skipped (the tool lies in its hole), the solid one is cut.
    let mut f = holed_and_solid();
    f.push(circle("SC", "E1.C1", 5.0, 5.0));
    f.push(CUT.to_string());
    let (v, errs) = regen_features(&f);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    assert_eq!(v.len(), 2, "{v:?}");
    assert!(v.iter().all(|x| (x - (1000.0 - HOLE * 10.0)).abs() < 1e-3), "{v:?}");
}

#[test]
fn a_cut_that_touches_no_body_is_an_error_not_a_pass() {
    // the only body with material there is cut already, and the other body is away from it
    let far = cube("E2", "newBody", "Far").replace(r#""x":0,"y":0"#, r#""x":30,"y":0"#).replace(r#""x":10,"y":0"#, r#""x":40,"y":0"#).replace(r#""x":10,"y":10"#, r#""x":40,"y":10"#).replace(r#""x":0,"y":10"#, r#""x":30,"y":10"#);
    let f = vec![
        cube("E1", "add", "A"),
        circle("SH", "E1.C1", 5.0, 5.0),
        r#"{"type":"extrude","id":"EH","op":"cut","sketch":"SH","end":"blind","depth":10}"#.to_string(),
        far,
        circle("SC", "E1.C1", 5.0, 5.0),
        CUT.to_string(),
    ];
    let (_, errs) = regen_features(&f);
    assert!(errs.contains(r#""code":"cutNoEffect""#), "{errs}");
}

#[test]
fn a_cut_of_two_bodies_one_of_which_misses_the_tool_is_not_an_error() {
    let far = cube("E2", "newBody", "Far").replace(r#""x":0,"y":0"#, r#""x":30,"y":0"#).replace(r#""x":10,"y":0"#, r#""x":40,"y":0"#).replace(r#""x":10,"y":10"#, r#""x":40,"y":10"#).replace(r#""x":0,"y":10"#, r#""x":30,"y":10"#);
    let f = vec![cube("E1", "add", "A"), far, circle("SC", "E1.C1", 5.0, 5.0), CUT.to_string()];
    let (mut v, errs) = regen_features(&f);
    assert!(errs.contains(r#""errors":[]"#), "{errs}");
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!((v[0] - (1000.0 - HOLE * 10.0)).abs() < 1e-3 && (v[1] - 1000.0).abs() < 1e-6, "{v:?}");
}

#[test]
fn a_body_keeps_its_name_when_a_cut_is_aimed_at_it() {
    let mut f = holed_and_solid();
    f.push(circle("SC", "E2.C1", 5.0, 5.0));
    f.push(CUT.to_string());
    let text = format!(r#"{{"format":"ck-part","version":1,"name":"p","features":[{}]}}"#, f.join(","));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.to_json().to_string().contains(r#""errors":[]"#));
    let b = ex.0.body.as_ref().unwrap();
    let mut names: Vec<_> = b.lump_ids().filter_map(|l| b.lump(l).name.clone()).collect();
    names.sort();
    assert_eq!(names, vec!["Solid".to_string()]);
}
