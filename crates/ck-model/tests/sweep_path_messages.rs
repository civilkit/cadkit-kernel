//! Sweep paths that are not one spline (features/sweep.rs, ck-ops sweep.rs): a chain of lines and arcs drawn in a
//! `sketch3d` builds when its joints are tangent or line-line; every path the sweep cannot take is refused with a
//! plain message and params (`message`, `supported`, `path` = piece types, `joint`, `at`, `hint`), never an empty
//! `sweepPathInvalid`; pieces of a `compositeCurve` built with a larger `gapTolerance` sweep.
use ck_json::Json;
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

struct Out {
    r: regen::RegenResult,
    brep: Brep,
}

fn run(features: &[&str]) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { r, brep: ex.brep }
}

impl Out {
    fn err(&self) -> &regen::FeatureError {
        assert_eq!(self.r.errors.len(), 1, "one error expected: {:?}", self.r.errors);
        &self.r.errors[0].error
    }
    fn param(&self, k: &str) -> Json {
        self.err().params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or(Json::Null)
    }
    fn text(&self, k: &str) -> String {
        match self.param(k) {
            Json::Str(s) if !s.is_empty() => s,
            o => panic!("param {k} missing or empty: {o:?} in {:?}", self.err()),
        }
    }
    /// Built: no error, valid body of about `want` volume.
    fn built(&self, want: f64, tol: f64) {
        assert!(self.r.errors.is_empty(), "{:?}", self.r.errors);
        let b = self.brep.body.as_ref().expect("a body");
        assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6));
        let v = ck_topo::mass::mass_props(b).volume;
        assert!((v - want).abs() < tol * want, "{v} vs {want}");
    }
    /// A refusal that explains itself: code, plain message, what is supported, the hint, and the path piece types.
    fn explained(&self, code: &str, kinds: &[&str]) {
        assert_eq!(self.err().code, code, "{:?}", self.err());
        for k in ["message", "supported", "hint"] {
            assert!(self.text(k).len() > 20, "{k}");
        }
        let path: Vec<String> = self.param("path").as_arr().map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect()).unwrap_or_default();
        assert_eq!(path, kinds, "{:?}", self.err());
    }
}

const RIGHT: &str = r#"{"kind":"std","id":"right"}"#;
const FRONT: &str = r#"{"kind":"std","id":"front"}"#;

/// A circle of radius 1 at the origin on the right plane (normal +x), and a sweep of it along `path`.
fn tube(path: &str) -> [String; 2] {
    [
        format!(r#"{{"type":"sketch","id":"S9","plane":{RIGHT},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":1}}],"constraints":[]}}"#),
        format!(r#"{{"type":"sweep","id":"W1","op":"add","sketch":"S9","path":{{"kind":"sketch","id":"{path}"}}}}"#),
    ]
}
fn sk3(pts: &[(&str, [f64; 3])], ents: &str) -> String {
    let p: Vec<String> = pts.iter().map(|(id, [x, y, z])| format!(r#"{{"id":"{id}","x":{x},"y":{y},"z":{z}}}"#)).collect();
    format!(r#"{{"type":"sketch3d","id":"P","points":[{}],"entities":[{ents}],"constraints":[]}}"#, p.join(","))
}
fn on3(pts: &[(&str, [f64; 3])], ents: &str) -> Out {
    let (s, w) = (sk3(pts, ents), tube("P"));
    run(&[&s, &w[0], &w[1]])
}
const L1: &str = r#"{"type":"line","id":"e1","p1":"a","p2":"b"}"#;
const L2: &str = r#"{"type":"line","id":"e2","p1":"b","p2":"c"}"#;
const O: [f64; 3] = [0.0, 0.0, 0.0];

// ---- 3D sketch paths that now build -------------------------------------------------------------------------

#[test]
fn a_3d_polyline_of_lines_builds_with_a_mitre() {
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("c", [20.0, 15.0, 0.0])], &format!("{L1},{L2}"));
    o.built(PI * 35.0, 2e-3);
}

#[test]
fn a_3d_line_then_tangent_arc_builds() {
    // arc about (20,10,0) from (20,0,0) to (30,10,0): leaves the line's end along +x
    let arc = r#"{"type":"arc","id":"e2","c":"m","p1":"b","p2":"c"}"#;
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("m", [20.0, 10.0, 0.0]), ("c", [30.0, 10.0, 0.0])], &format!("{L1},{arc}"));
    o.built(PI * (20.0 + 5.0 * PI), 2e-3);
}

#[test]
fn a_3d_arc_alone_builds() {
    let arc = r#"{"type":"arc","id":"e1","c":"m","p1":"a","p2":"c"}"#;
    // a quarter turn about (0,10,0) in the z = 0 plane, starting along +x at the origin
    let o = on3(&[("a", O), ("m", [0.0, 10.0, 0.0]), ("c", [10.0, 10.0, 0.0])], arc);
    o.built(PI * 5.0 * PI, 2e-3);
}

// ---- refusals with a message ------------------------------------------------------------------------------

#[test]
fn a_3d_line_meeting_an_arc_at_a_corner_says_where_and_what_to_change() {
    // the arc about (25,10,0) leaves (20,0,0) at an angle to the line
    let arc = r#"{"type":"arc","id":"e2","c":"m","p1":"b","p2":"c"}"#;
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("m", [25.0, 10.0, 0.0]), ("c", [35.0, 10.0, 0.0])], &format!("{L1},{arc}"));
    o.explained("notImplemented", &["line", "arc"]);
    assert_eq!(o.param("joint").as_f64(), Some(0.0));
    assert_eq!(o.param("at").as_arr().map(|a| a.len()), Some(3));
    assert!(o.text("message").contains("line") && o.text("message").contains("arc") && o.text("message").contains("tangent"));
    assert!(o.text("hint").contains("tangent"));
}

#[test]
fn a_2d_sketch_path_with_a_line_arc_corner_gives_the_same_message() {
    let p = r#"{"type":"sketch","id":"P","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":20,"y":0},{"id":"m","x":25,"y":10},{"id":"c","x":35,"y":10}],"entities":[
        {"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"m","p1":"b","p2":"c"}],"constraints":[]}"#;
    let w = tube("P");
    let o = run(&[p, &w[0], &w[1]]);
    o.explained("notImplemented", &["line", "arc"]);
    assert_eq!(o.param("joint").as_f64(), Some(0.0));
}

#[test]
fn two_splines_say_a_single_spline_is_supported() {
    let sp = r#"{"type":"spline","id":"e1","pts":["a","b","c"]},{"type":"spline","id":"e2","pts":["c","d","e"]}"#;
    let pts = [("a", O), ("b", [10.0, 5.0, 0.0]), ("c", [20.0, 0.0, 0.0]), ("d", [30.0, 5.0, 0.0]), ("e", [40.0, 0.0, 0.0])];
    let o = on3(&pts, sp);
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("2 splines"), "{}", o.text("message"));
    assert!(o.text("hint").contains("one spline"));
    assert!(o.text("supported").contains("single spline"));
}

#[test]
fn a_spline_mixed_with_a_line_names_both() {
    let ents = r#"{"type":"spline","id":"e1","pts":["a","b","c"]},{"type":"line","id":"e2","p1":"c","p2":"d"}"#;
    let pts = [("a", O), ("b", [10.0, 5.0, 0.0]), ("c", [20.0, 0.0, 0.0]), ("d", [30.0, 0.0, 0.0])];
    let o = on3(&pts, ents);
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("spline e1") && o.text("message").contains("line (e2)"), "{}", o.text("message"));
    assert!(o.text("hint").len() > 20);
}

#[test]
fn a_gap_between_3d_pieces_names_the_break() {
    // 0.01 mm between the line ends: the chain breaks after piece 0
    let l2 = r#"{"type":"line","id":"e2","p1":"c","p2":"d"}"#;
    let pts = [("a", O), ("b", [20.0, 0.0, 0.0]), ("c", [20.01, 0.0, 0.0]), ("d", [20.0, 15.0, 0.0])];
    let o = on3(&pts, &format!("{L1},{l2}"));
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert_eq!(o.param("joint").as_f64(), Some(0.0));
    assert_eq!(o.param("at").as_arr().map(|a| a.len()), Some(3));
    assert!(o.text("message").contains("e1") && o.text("message").contains("e2"), "{}", o.text("message"));
    assert!(o.text("hint").contains("compositeCurve"));
}

#[test]
fn a_closed_loop_and_a_branch_are_named() {
    let l3 = r#"{"type":"line","id":"e3","p1":"c","p2":"a"}"#;
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("c", [20.0, 15.0, 0.0])], &format!("{L1},{L2},{l3}"));
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("closed loop"), "{}", o.text("message"));
    let l3 = r#"{"type":"line","id":"e3","p1":"b","p2":"d"}"#;
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("c", [20.0, 15.0, 0.0]), ("d", [20.0, -15.0, 0.0])], &format!("{L1},{L2},{l3}"));
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("branches"), "{}", o.text("message"));
    assert_eq!(o.param("joint").as_f64(), Some(0.0));
}

#[test]
fn a_corner_that_doubles_back_is_named() {
    let o = on3(&[("a", O), ("b", [20.0, 0.0, 0.0]), ("c", [0.0, 1.0, 0.0])], &format!("{L1},{L2}"));
    o.explained("sweepPathInvalid", &["line", "line"]);
    assert_eq!(o.param("joint").as_f64(), Some(0.0));
    assert!(o.text("message").contains("166"), "{}", o.text("message"));
}

// ---- composite joins ------------------------------------------------------------------------------------

/// Line (0,0)-(20,0), then the arc about (20.00002, 10) leaving (20.00002, 0): the pieces are 2e-5 apart.
fn gapped(gap_tol: &str) -> Vec<String> {
    let p = r#"{"type":"sketch","id":"P","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":20,"y":0},{"id":"m","x":20.00002,"y":10},{"id":"d","x":20.00002,"y":0},{"id":"c","x":30.00002,"y":10}],"entities":[
        {"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"m","p1":"d","p2":"c"}],"constraints":[]}"#;
    let comp = format!(r#"{{"type":"compositeCurve","id":"C1","curves":[{{"kind":"sketchCurve","sketch":"P","entity":"e1"}},{{"kind":"sketchCurve","sketch":"P","entity":"e2"}}]{gap_tol}}}"#);
    let w = tube("P");
    let sw = r#"{"type":"sweep","id":"W1","op":"add","sketch":"S9","path":{"kind":"curve","id":"C1"}}"#;
    vec![p.to_string(), comp, w[0].clone(), sw.to_string()]
}

#[test]
fn a_composite_built_with_a_gap_tolerance_sweeps() {
    let f = gapped(r#","gapTolerance":0.0001"#);
    let o = run(&f.iter().map(|s| s.as_str()).collect::<Vec<_>>());
    o.built(PI * (20.0 + 5.0 * PI), 2e-3);
}

#[test]
fn the_same_gap_without_a_composite_is_refused_with_the_gap_named() {
    let f = gapped("");
    let w = tube("P");
    let o = run(&[&f[0], &w[0], &w[1]]);
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("e1") && o.text("message").contains("e2"), "{}", o.text("message"));
    assert!(o.text("hint").contains("compositeCurve") && o.text("hint").contains("gapTolerance"));
}

#[test]
fn a_sketch_with_a_spline_and_a_circle_names_the_entity() {
    let p = format!(r#"{{"type":"sketch","id":"P","plane":{FRONT},"points":[{{"id":"a","x":0,"y":0}},{{"id":"r","x":5,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"a","r":3}}],"constraints":[]}}"#);
    let w = tube("P");
    let o = run(&[&p, &w[0], &w[1]]);
    assert_eq!(o.err().code, "sweepPathInvalid");
    assert!(o.text("message").contains("circle") && o.text("message").contains("e1"), "{}", o.text("message"));
}
