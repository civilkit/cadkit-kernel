//! `combine` `cut` whose tools remove nothing from the target (a tool that misses it, is tangent, touches along an edge
//! or a point, sits a hair away, or lies inside a cavity): not an error. The target comes back unchanged and the warning
//! `cutNoEffect {tools, kept}` names the tools (solidcore corpus: ladder 02-booleans 004-011, booleans 451 and 3702).
//! `split` that changes nothing, and the cut of the other features, still answer the error `cutNoEffect`.
#![allow(dead_code)]

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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


struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn warnings(&self) -> Vec<String> {
        self.result.warnings.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn volume(&self) -> f64 {
        self.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume)
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<Option<String>> = b.lump_ids().map(|l| b.lump(l).name.clone()).collect();
        v.sort();
        v
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
    fn warning_params(&self, code: &str) -> ck_json::Json {
        let w = self.result.warnings.iter().find(|w| w.error.code == code).unwrap_or_else(|| panic!("no {code}: {:?}", self.warnings()));
        ck_json::Json::Obj(w.error.params.clone())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

/// A box as a body of its own: x in [x0,x1], sketch-y in [a,b] (the world -z), height h standing on the plane y = y0
/// (the top plane offset by y0; `Pn` is that plane).
fn bx(id: u32, name: &str, x0: f64, x1: f64, a: f64, b: f64, y0: f64, h: f64) -> String {
    format!(
        r#"{{"type":"plane","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{y0},"id":"P{id}","visible":false}},
         {{"type":"sketch","id":"S{id}","plane":{{"kind":"feature","id":"P{id}"}},"points":[{{"id":"p1","x":{x0},"y":{a}}},{{"id":"p2","x":{x1},"y":{a}}},{{"id":"p3","x":{x1},"y":{b}}},{{"id":"p4","x":{x0},"y":{b}}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
         {{"type":"extrude","id":"F{id}","op":"newBody","sketch":"S{id}","end":"blind","depth":{h},"body":"{name}"}}"#
    )
}

fn cut(extra: &str) -> String {
    format!(r#"{{"type":"combine","id":"C","op":"cut","target":{{"body":"T"}},"tools":[{{"body":"K"}}]{extra}}}"#)
}

/// Target T = [0,10]^3, tool K = a 10 mm cube at x in [x0, x0+10], sketch-y [a, a+10], y in [y0, y0+10].
fn t_and_k(x0: f64, a: f64, y0: f64, extra: &str) -> Run {
    run(&format!("{},{},{}", bx(1, "T", 0.0, 10.0, 0.0, 10.0, 0.0, 10.0), bx(2, "K", x0, x0 + 10.0, a, a + 10.0, y0, 10.0), cut(extra)))
}

fn assert_unchanged(r: &Run, why: &str) {
    assert!(r.errors().is_empty(), "{why}: {:?}", r.errors());
    assert_eq!(r.warnings(), vec!["C:cutNoEffect"], "{why}");
    assert!((r.volume() - 1000.0).abs() < 1e-6, "{why}: volume {}", r.volume());
    assert_eq!(r.lumps(), 1, "{why}");
    assert_eq!(r.names(), vec![Some("T".to_string())], "{why}");
    assert!(r.faces().iter().all(|f| f.starts_with("F1.")), "{why}: {:?}", r.faces());
    assert_eq!(r.faces().len(), 6, "{why}");
    assert!(ck_check::validate(r.body.as_ref().unwrap(), 1e-6).is_empty(), "{why}");
}

#[test]
fn a_tool_that_does_not_overlap_the_target_is_a_warning() {
    assert_unchanged(&t_and_k(30.0, 0.0, 0.0, ""), "far away");
}

#[test]
fn a_tool_touching_along_a_face_is_a_warning() {
    assert_unchanged(&t_and_k(10.0, 0.0, 0.0, ""), "face contact");
}

#[test]
fn a_tool_touching_along_an_edge_is_a_warning() {
    // K at x [10,20], y [10,20], same z as T: shares only the line x = 10, y = 10
    assert_unchanged(&t_and_k(10.0, 0.0, 10.0, ""), "edge contact");
}

#[test]
fn a_tool_touching_at_a_vertex_is_a_warning() {
    assert_unchanged(&t_and_k(10.0, 10.0, 10.0, ""), "vertex contact");
}

#[test]
fn a_tool_a_micron_away_is_a_warning() {
    assert_unchanged(&t_and_k(10.0 + 1e-6, 0.0, 0.0, ""), "gap 1e-6");
}

#[test]
fn a_tool_wholly_inside_a_cavity_is_a_warning() {
    // H = [0,30]^3 minus a 10 mm cube [10,20]^3 (a closed void), then a 4 mm cube in the void cuts nothing
    let f = format!(
        "{},{},{},{{\"type\":\"combine\",\"id\":\"V\",\"op\":\"cut\",\"target\":{{\"body\":\"H\"}},\"tools\":[{{\"body\":\"M\"}}]}},{},{{\"type\":\"combine\",\"id\":\"C\",\"op\":\"cut\",\"target\":{{\"body\":\"H\"}},\"tools\":[{{\"body\":\"K\"}}]}}",
        bx(1, "H", 0.0, 30.0, 0.0, 30.0, 0.0, 30.0),
        bx(2, "M", 10.0, 20.0, 10.0, 20.0, 10.0, 10.0),
        "",
        bx(3, "K", 13.0, 17.0, 13.0, 17.0, 13.0, 4.0)
    )
    .replace(",,", ",");
    let r = run(&f);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.warnings(), vec!["C:cutNoEffect"]);
    assert!((r.volume() - (27000.0 - 1000.0)).abs() < 1e-6, "volume {}", r.volume());
    assert_eq!(r.names(), vec![Some("H".to_string())]);
}

#[test]
fn the_warning_names_the_tools_and_the_tools_are_consumed() {
    let r = t_and_k(30.0, 0.0, 0.0, "");
    let p = r.warning_params("cutNoEffect");
    assert_eq!(ck_json::to_string(&p), r#"{"tools":["K"],"kept":false}"#);
    // consumed: no lump K left, as after a cut that removes something
    assert_eq!(r.lumps(), 1);
}

#[test]
fn keep_tools_keeps_the_tool_lump_and_says_so() {
    let r = t_and_k(30.0, 0.0, 0.0, r#","keepTools":true"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(ck_json::to_string(&r.warning_params("cutNoEffect")), r#"{"tools":["K"],"kept":true}"#);
    assert_eq!(r.lumps(), 2);
    assert_eq!(r.names(), vec![Some("K".to_string()), Some("T".to_string())]);
    assert!((r.volume() - 2000.0).abs() < 1e-6);
}

#[test]
fn a_body_name_on_a_no_op_cut_renames_the_unchanged_target() {
    let r = t_and_k(30.0, 0.0, 0.0, r#","body":"Result""#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.names(), vec![Some("Result".to_string())]);
}

#[test]
fn several_tools_none_of_which_cuts_are_all_named() {
    let f = format!(
        "{},{},{},{{\"type\":\"combine\",\"id\":\"C\",\"op\":\"cut\",\"target\":{{\"body\":\"T\"}},\"tools\":[{{\"body\":\"K\"}},{{\"body\":\"L\"}}]}}",
        bx(1, "T", 0.0, 10.0, 0.0, 10.0, 0.0, 10.0),
        bx(2, "K", 30.0, 40.0, 0.0, 10.0, 0.0, 10.0),
        bx(3, "L", 50.0, 60.0, 0.0, 10.0, 0.0, 10.0)
    );
    let r = run(&f);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(ck_json::to_string(&r.warning_params("cutNoEffect")), r#"{"tools":["K","L"],"kept":false}"#);
    assert_eq!(r.lumps(), 1);
}

#[test]
fn a_cut_that_removes_something_has_no_warning() {
    let r = t_and_k(5.0, 0.0, 0.0, "");
    assert!(r.errors().is_empty() && r.warnings().is_empty(), "{:?} {:?}", r.errors(), r.warnings());
    assert!((r.volume() - 500.0).abs() < 1e-6, "{}", r.volume());
}

#[test]
fn one_tool_cuts_and_one_misses_gives_no_warning() {
    let f = format!(
        "{},{},{},{{\"type\":\"combine\",\"id\":\"C\",\"op\":\"cut\",\"target\":{{\"body\":\"T\"}},\"tools\":[{{\"body\":\"K\"}},{{\"body\":\"L\"}}]}}",
        bx(1, "T", 0.0, 10.0, 0.0, 10.0, 0.0, 10.0),
        bx(2, "K", 5.0, 15.0, 0.0, 10.0, 0.0, 10.0),
        bx(3, "L", 50.0, 60.0, 0.0, 10.0, 0.0, 10.0)
    );
    let r = run(&f);
    assert!(r.errors().is_empty() && r.warnings().is_empty(), "{:?} {:?}", r.errors(), r.warnings());
    assert!((r.volume() - 500.0).abs() < 1e-6, "{}", r.volume());
}

#[test]
fn a_split_that_changes_nothing_is_still_refused() {
    // a split promises two lumps; a tool that misses the target cannot give them
    let r = run(&format!(
        "{},{},{}",
        bx(1, "T", 0.0, 10.0, 0.0, 10.0, 0.0, 10.0),
        bx(2, "K", 30.0, 40.0, 0.0, 10.0, 0.0, 10.0),
        r#"{"type":"combine","id":"C","op":"split","target":{"body":"T"},"tools":[{"body":"K"}]}"#
    ));
    assert_eq!(r.errors(), vec!["C:cutNoEffect"]);
    assert!(r.warnings().is_empty());
    assert!((r.volume() - 2000.0).abs() < 1e-6);
}

#[test]
fn an_extrude_cut_that_misses_is_still_refused() {
    let r = run(&format!(
        "{},{},{}",
        bx(1, "T", 0.0, 10.0, 0.0, 10.0, 0.0, 10.0),
        r#"{"type":"sketch","id":"SC","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":30,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":10},{"id":"p4","x":30,"y":10}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#,
        r#"{"type":"extrude","id":"X","op":"cut","sketch":"SC","end":"blind","depth":5}"#
    ));
    assert_eq!(r.errors(), vec!["X:cutNoEffect"]);
}
