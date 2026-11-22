//! `perpendicular` with a circle or an arc: a line and a curve (the line is a normal: it passes through the
//! centre), two curves (orthogonal circles, d^2 = r1^2 + r2^2). One equation each, refs in either order;
//! line + line is unchanged; a point ref is refused with the supported combinations named.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::sketch::analyze::freedoms;
use ck_model::sketch::solver::{build, solve_sketch, status, SolveOpts};
use ck_model::sketch::status::sketch_status;
use ck_model::{Part, Sketch, SketchEntity};
use std::f64::consts::PI;

fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
    let src = format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
    Sketch::from_json(&ck_json::parse(&src).unwrap())
}
fn xy(s: &Sketch, id: &str) -> [f64; 2] {
    s.point_xy(id).unwrap()
}
fn radius_of(s: &Sketch, id: &str) -> f64 {
    match s.entity(id) {
        Some(SketchEntity::Circle { r, .. }) => *r,
        other => panic!("{id}: {other:?}"),
    }
}
/// All three DOF readers agree; returns the count.
fn dof(s: &Sketch) -> usize {
    let (a, b, c) = (status(s).unwrap().dof, sketch_status(s).dof, freedoms(s).dof);
    assert!(a == b && b == c, "dof readers disagree: {a} {b} {c}");
    a
}
fn solved(mut s: Sketch) -> Sketch {
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    s
}
const REFS: [&str; 2] = ["A", "B"];
fn perp(refs: [&str; 2]) -> String {
    format!(r#"{{"type":"perpendicular","id":"k1","refs":["{}","{}"]}}"#, refs[0], refs[1])
}

// ---- line + circle -------------------------------------------------------------------------------------
const LC_PTS: &str = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":8,"y":1,"fixed":true},{"id":"b","x":3,"y":6}"#;
const LC_ENTS: &str = r#"{"type":"circle","id":"A","c":"o","r":5},{"type":"line","id":"B","p1":"a","p2":"b"}"#;
const CL_ENTS: &str = r#"{"type":"line","id":"A","p1":"a","p2":"b"},{"type":"circle","id":"B","c":"o","r":5}"#;

fn cross_to_centre(s: &Sketch, c: &str, a: &str, b: &str) -> f64 {
    let (c, a, b) = (xy(s, c), xy(s, a), xy(s, b));
    ((c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])) / ((b[0] - a[0]).hypot(b[1] - a[1]))
}

#[test]
fn line_and_circle_the_line_passes_through_the_centre_in_either_order() {
    let base = dof(&sk(LC_PTS, LC_ENTS, ""));
    let mut results = vec![];
    for (ents, refs) in [(LC_ENTS, ["A", "B"]), (LC_ENTS, ["B", "A"]), (CL_ENTS, ["A", "B"]), (CL_ENTS, ["B", "A"])] {
        let s = sk(LC_PTS, ents, &perp(refs));
        assert_eq!(dof(&s), base - 1, "{refs:?}");
        let s = solved(s);
        assert!(cross_to_centre(&s, "o", "a", "b").abs() < 1e-9, "{refs:?}: {:?}", xy(&s, "b"));
        results.push(xy(&s, "b"));
    }
    for r in &results[1..] {
        assert!((r[0] - results[0][0]).abs() < 1e-9 && (r[1] - results[0][1]).abs() < 1e-9, "{results:?}");
    }
}

#[test]
fn line_and_arc_the_line_passes_through_the_arc_centre() {
    // the arc is fixed; its chord is not the reference, its centre is
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"s","x":5,"y":0,"fixed":true},{"id":"t","x":0,"y":5,"fixed":true},{"id":"a","x":8,"y":1,"fixed":true},{"id":"b","x":3,"y":6}"#;
    let ents = r#"{"type":"arc","id":"A","c":"o","p1":"s","p2":"t"},{"type":"line","id":"B","p1":"a","p2":"b"}"#;
    let base = dof(&sk(pts, ents, ""));
    let mut out = vec![];
    for refs in [["A", "B"], ["B", "A"]] {
        let s = sk(pts, ents, &perp(refs));
        assert_eq!(dof(&s), base - 1);
        let s = solved(s);
        assert!(cross_to_centre(&s, "o", "a", "b").abs() < 1e-9);
        out.push(xy(&s, "b"));
    }
    assert!((out[0][0] - out[1][0]).abs() < 1e-9 && (out[0][1] - out[1][1]).abs() < 1e-9);
}

#[test]
fn line_ending_on_the_curve_is_perpendicular_to_it_there() {
    // line from the circle's rim point (3,4) outwards: the normal at the rim
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":3,"y":4,"fixed":true},{"id":"b","x":7,"y":5}"#;
    let s = solved(sk(pts, LC_ENTS, &format!(r#"{},{{"type":"distance","id":"k2","refs":["a","b"],"value":5,"offset":null}}"#, perp(REFS))));
    let (a, b) = (xy(&s, "a"), xy(&s, "b"));
    let (d, n) = ([b[0] - a[0], b[1] - a[1]], [a[0], a[1]]);
    assert!((d[0] * n[1] - d[1] * n[0]).abs() < 1e-9, "{b:?}");
    // only the circle's own radius is left free
    assert_eq!(dof(&sk(pts, LC_ENTS, &format!(r#"{},{{"type":"distance","id":"k2","refs":["a","b"],"value":5,"offset":null}}"#, perp(REFS)))), 1);
}

// ---- circle + circle / arc ------------------------------------------------------------------------------
const CC_PTS: &str = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"q","x":8,"y":0,"fixed":true}"#;
const CC_ENTS: &str = r#"{"type":"circle","id":"A","c":"o","r":5},{"type":"circle","id":"B","c":"q","r":3}"#;
const CC_FIX: &str = r#"{"type":"radius","id":"k0","refs":["A"],"value":5}"#;

#[test]
fn two_circles_are_orthogonal_in_either_order() {
    // B's radius is the one freedom: r2^2 = 8^2 - 5^2
    let base = dof(&sk(CC_PTS, CC_ENTS, CC_FIX));
    for refs in [["A", "B"], ["B", "A"]] {
        let s = sk(CC_PTS, CC_ENTS, &format!("{CC_FIX},{}", perp(refs)));
        assert_eq!(dof(&s), base - 1, "{refs:?}");
        let s = solved(s);
        let (r1, r2) = (radius_of(&s, "A"), radius_of(&s, "B"));
        assert!((64.0 - r1 * r1 - r2 * r2).abs() < 1e-9, "{r1} {r2}");
        assert!((r2 - 39f64.sqrt()).abs() < 1e-9);
    }
}

#[test]
fn two_circles_with_a_free_centre() {
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"q","x":8,"y":1}"#;
    let cons = format!(r#"{CC_FIX},{{"type":"radius","id":"k3","refs":["B"],"value":3}},{}"#, perp(REFS));
    let base = dof(&sk(pts, CC_ENTS, &format!(r#"{CC_FIX},{{"type":"radius","id":"k3","refs":["B"],"value":3}}"#)));
    let s = sk(pts, CC_ENTS, &cons);
    assert_eq!(dof(&s), base - 1, "centre may move along a circle of radius sqrt(34)");
    let s = solved(s);
    let (o, q) = (xy(&s, "o"), xy(&s, "q"));
    assert!(((q[0] - o[0]).powi(2) + (q[1] - o[1]).powi(2) - 34.0).abs() < 1e-9, "{q:?}");
}

#[test]
fn arcs_as_well_as_circles() {
    // arc A: centre (0,0), (5,0)->(0,5); arc B: ends fixed at (10,0),(10,6), centre on y = 3 (one freedom)
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"s","x":5,"y":0,"fixed":true},{"id":"t","x":0,"y":5,"fixed":true},
        {"id":"c","x":6,"y":3},{"id":"u","x":10,"y":0,"fixed":true},{"id":"v","x":10,"y":6,"fixed":true}"#;
    let ents = r#"{"type":"arc","id":"A","c":"o","p1":"s","p2":"t"},{"type":"arc","id":"B","c":"c","p1":"u","p2":"v"}"#;
    assert_eq!(dof(&sk(pts, ents, "")), 1);
    let mut out = vec![];
    for refs in [["A", "B"], ["B", "A"]] {
        let s = sk(pts, ents, &perp(refs));
        assert_eq!(dof(&s), 0, "{refs:?}");
        let s = solved(s);
        let c = xy(&s, "c");
        let (d2, r1, r2) = (c[0] * c[0] + c[1] * c[1], 25.0, (10.0 - c[0]).powi(2) + (6.0 - c[1]).powi(2));
        assert!((d2 - r1 - r2).abs() < 1e-9, "{c:?}");
        assert!((c[0] - 6.25).abs() < 1e-9 && (c[1] - 3.0).abs() < 1e-9, "{c:?}");
        out.push(c);
    }
    assert!((out[0][0] - out[1][0]).abs() < 1e-9);
    // a circle and an arc, both orders
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"c","x":6,"y":3},{"id":"u","x":10,"y":0,"fixed":true},{"id":"v","x":10,"y":6,"fixed":true}"#;
    let ents = r#"{"type":"circle","id":"A","c":"o","r":5},{"type":"arc","id":"B","c":"c","p1":"u","p2":"v"}"#;
    for refs in [["A", "B"], ["B", "A"]] {
        let s = solved(sk(pts, ents, &format!(r#"{CC_FIX},{}"#, perp(refs))));
        assert!((xy(&s, "c")[0] - 6.25).abs() < 1e-9, "{refs:?}");
    }
}

// ---- conflicts, redundancy, line + line, refusal ------------------------------------------------------------
#[test]
fn conflicting_and_redundant_are_detected() {
    // both line ends fixed: either through the centre (redundant) or not (conflict)
    let miss = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":8,"y":1,"fixed":true},{"id":"b","x":3,"y":6,"fixed":true}"#;
    let st = sketch_status(&sk(miss, LC_ENTS, &perp(REFS)));
    assert_eq!(st.conflicts, vec!["k1"], "{st:?}");
    let hit = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":8,"y":0,"fixed":true},{"id":"b","x":3,"y":0,"fixed":true}"#;
    let st = sketch_status(&sk(hit, LC_ENTS, &perp(REFS)));
    assert!(st.conflicts.is_empty() && st.redundant == vec!["k1"], "{st:?}");
    // the same constraint twice: the second one is redundant
    let twice = format!(r#"{},{{"type":"perpendicular","id":"k2","refs":["B","A"]}}"#, perp(REFS));
    let st = sketch_status(&sk(LC_PTS, LC_ENTS, &twice));
    assert!(st.conflicts.is_empty() && st.state == "over" && !st.redundant.is_empty(), "{st:?}");
    // orthogonal circles at the wrong radius, all fixed by dimensions: conflict
    let st = sketch_status(&sk(CC_PTS, CC_ENTS, &format!(r#"{CC_FIX},{{"type":"radius","id":"k3","refs":["B"],"value":3}},{}"#, perp(REFS))));
    assert_eq!(st.conflicts, vec!["k1"], "{st:?}");
}

#[test]
fn line_and_line_is_unchanged() {
    let pts = r#"{"id":"a","x":0,"y":0,"fixed":true},{"id":"b","x":10,"y":1},{"id":"c","x":9,"y":8}"#;
    let ents = r#"{"type":"line","id":"A","p1":"a","p2":"b"},{"type":"line","id":"B","p1":"b","p2":"c"}"#;
    let base = dof(&sk(pts, ents, ""));
    let s = sk(pts, ents, &perp(REFS));
    assert_eq!(build(&s).unwrap().eqs.len(), 1);
    assert_eq!(dof(&s), base - 1);
    let s = solved(s);
    let (a, b, c) = (xy(&s, "a"), xy(&s, "b"), xy(&s, "c"));
    assert!(((b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1])).abs() < 1e-9);
}

#[test]
fn unsupported_refs_say_what_is_supported() {
    let want = "perpendicular needs two lines, a line and a circle or arc, or two circles or arcs";
    let pts = r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":8,"y":1}"#;
    let ents = r#"{"type":"circle","id":"A","c":"o","r":5}"#;
    for refs in [["a", "A"], ["A", "a"]] {
        let e = build(&sk(pts, ents, &perp(refs))).err().expect("refused");
        assert_eq!(e, want);
    }
    let ents = r#"{"type":"circle","id":"A","c":"o","r":5},{"type":"point","id":"P","p":"a"}"#;
    assert_eq!(build(&sk(pts, ents, &perp(["A", "P"]))).err().unwrap(), want);
}

// ---- end to end ---------------------------------------------------------------------------------------------
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

#[test]
fn regen_solves_the_constraint_and_extrudes() {
    // construction circle A (centre fixed, r 5); profile circle B (r 3) whose centre starts off the axis and is
    // pulled to d = sqrt(34) by the orthogonality, then held on the x axis
    let src = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},
         "points":[{"id":"o","x":0,"y":0,"fixed":true},{"id":"c","x":5,"y":2}],
         "entities":[{"type":"circle","id":"A","c":"o","r":5,"construction":true},{"type":"circle","id":"B","c":"c","r":3}],
         "constraints":[{"type":"radius","id":"k1","refs":["A"],"value":5},{"type":"radius","id":"k2","refs":["B"],"value":3},
                        {"type":"perpendicular","id":"k3","refs":["B","A"]},{"type":"horizontal","id":"k4","refs":["o","c"]}]},
        {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":4}]}"#;
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap())), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let j = r.to_json();
    let e = &j.get("sketches").unwrap().as_arr().unwrap()[0].as_arr().unwrap()[1];
    let c = e.get("sketch").unwrap().get("points").unwrap().as_arr().unwrap().iter()
        .find(|p| p.get("id").unwrap().as_str() == Some("c")).unwrap().clone();
    assert!((c.get("x").unwrap().as_f64().unwrap() - 34f64.sqrt()).abs() < 1e-7, "{c:?}");
    assert_eq!(e.get("status").unwrap().get("dof").unwrap().as_f64(), Some(0.0));
    let v = ck_topo::mass::mass_props(ex.brep.body.as_ref().unwrap()).volume;
    assert!((v - PI * 9.0 * 4.0).abs() / v < 1e-6, "{v}");
}
