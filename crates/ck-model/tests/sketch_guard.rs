//! Solve guard (docs/sketch-status.md "Problems"): a solve that shrinks a curve to a point or throws points
//! far from the sketch is not taken, and picks that can only do that are named. Cases from the
//! solidcraft sketch fuzzer (PRs #8988, #9049, #8562, #8611, #8577, #8600, #8931, #8967, #9728), replayed
//! on our solver through the regen reply (`opts.solveSketches`, the GUI's path; the reply's `status` is
//! what an agent reads with `opts.sketchStatus`).
use ck_json::Json;
use ck_model::regen::{regen, NoGeometry, RegenOpts};
use ck_model::Part;

struct Reply {
    pts: Vec<(String, f64, f64)>,
    status: Json,
}

impl Reply {
    fn xy(&self, id: &str) -> (f64, f64) {
        let p = self.pts.iter().find(|p| p.0 == id).unwrap();
        (p.1, p.2)
    }
    fn solved(&self) -> bool {
        self.status.get("solved") == Some(&Json::Bool(true))
    }
    /// `(code, constraint, entity)` of every listed problem.
    fn problems(&self) -> Vec<(String, String, String)> {
        let s = |p: &Json, k: &str| p.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string();
        match self.status.get("problems").and_then(|v| v.as_arr()) {
            Some(a) => a.iter().map(|p| {
                assert!(!s(p, "message").is_empty(), "a problem without a message: {p:?}");
                (s(p, "code"), s(p, "constraint"), s(p, "entity"))
            }).collect(),
            None => vec![],
        }
    }
    fn has(&self, code: &str) -> bool {
        self.problems().iter().any(|p| p.0 == code)
    }
}

fn solve(points: &str, ents: &str, cons: &str, opts: &str) -> Reply {
    let src = format!(r#"{{"format":"ck-part","features":[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}]}}"#);
    let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::from_json(Some(&ck_json::parse(opts).unwrap())), &mut NoGeometry).to_json();
    assert_eq!(r.get("errors").unwrap().as_arr().unwrap().len(), 0, "{}", ck_json::to_string(&r));
    let e = &r.get("sketches").unwrap().as_arr().unwrap()[0].as_arr().unwrap()[1];
    let pts = e.get("sketch").unwrap().get("points").unwrap().as_arr().unwrap().iter()
        .map(|p| (p.get("id").unwrap().as_str().unwrap().to_string(), p.get("x").unwrap().as_f64().unwrap(), p.get("y").unwrap().as_f64().unwrap()))
        .collect();
    Reply { pts, status: e.get("status").cloned().unwrap_or(Json::Null) }
}
const SOLVE: &str = r#"{"solveSketches":true}"#;
const STATUS: &str = r#"{"sketchStatus":true}"#;

const RECT_PTS: &str = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":0,"y":5}"#;
const RECT: &str = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#;
const HV: &str = r#"{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},{"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]}"#;

fn unchanged(r: &Reply, pts: &[(&str, f64, f64)]) {
    for &(id, x, y) in pts {
        assert_eq!(r.xy(id), (x, y), "{id} moved: {:?}", r.pts);
    }
}
const RECT_AS_DRAWN: [(&str, f64, f64); 4] = [("p1", 0.0, 0.0), ("p2", 10.0, 0.0), ("p3", 10.0, 5.0), ("p4", 0.0, 5.0)];

/// solidcraft #8988: a corner made the midpoint of the opposite side collapses the whole rectangle.
#[test]
fn a_corner_as_the_midpoint_of_the_opposite_side_is_refused() {
    for opts in [SOLVE, STATUS] {
        let r = solve(RECT_PTS, RECT, &format!(r#"{HV},{{"type":"midpoint","id":"k5","refs":["p2","e4"]}}"#), opts);
        assert!(!r.solved(), "{opts}: {:?}", r.status);
        assert!(r.has("sketchCurveShrunk"), "{opts}: {:?}", r.status);
        unchanged(&r, &RECT_AS_DRAWN);
    }
}

/// solidcraft #8988: two opposite corners made coincident.
#[test]
fn opposite_corners_made_coincident_are_refused() {
    let r = solve(RECT_PTS, RECT, &format!(r#"{HV},{{"type":"coincident","id":"k5","refs":["p1","p3"]}}"#), SOLVE);
    assert!(!r.solved() && r.has("sketchCurveShrunk"), "{:?}", r.status);
    unchanged(&r, &RECT_AS_DRAWN);
}

/// solidcraft #9049: the squeeze happens over two constraints: the second coincidence takes two sides of
/// the first rectangle from tens of mm to a hair. Two rectangles, a perpendicular across them, two corner pairs.
#[test]
fn a_side_squeezed_over_two_steps_is_refused() {
    let pts = r#"{"id":"p1","x":1,"y":19},{"id":"p2","x":27,"y":19},{"id":"p3","x":27,"y":29},{"id":"p4","x":1,"y":29},
        {"id":"p5","x":-21,"y":-20},{"id":"p6","x":23,"y":-20},{"id":"p7","x":23,"y":-8},{"id":"p8","x":-21,"y":-8}"#;
    let ents = format!(r#"{RECT},{{"type":"line","id":"e5","p1":"p5","p2":"p6"}},{{"type":"line","id":"e6","p1":"p6","p2":"p7"}},{{"type":"line","id":"e7","p1":"p7","p2":"p8"}},{{"type":"line","id":"e8","p1":"p8","p2":"p5"}}"#);
    let hv2 = r#"{"type":"horizontal","id":"c5","refs":["e5"]},{"type":"vertical","id":"c6","refs":["e6"]},{"type":"horizontal","id":"c7","refs":["e7"]},{"type":"vertical","id":"c8","refs":["e8"]}"#;
    let first = format!(r#"{HV},{hv2},{{"type":"coincident","id":"k1","refs":["p5","p1"]}}"#);
    let r1 = solve(pts, &ents, &first, SOLVE);
    assert!(r1.solved(), "{:?}", r1.status);
    // the next pair of corners, from the solved state of the first step
    let pts2: String = r1.pts.iter().map(|(id, x, y)| format!(r#"{{"id":"{id}","x":{x},"y":{y}}}"#)).collect::<Vec<_>>().join(",");
    let r2 = solve(&pts2, &ents, &format!(r#"{first},{{"type":"coincident","id":"k2","refs":["p7","p3"]}}"#), SOLVE);
    let side = |r: &Reply, a: &str, b: &str| {
        let (p, q) = (r.xy(a), r.xy(b));
        (q.0 - p.0).hypot(q.1 - p.1)
    };
    for (a, b) in [("p1", "p2"), ("p2", "p3"), ("p5", "p6"), ("p6", "p7")] {
        assert!(side(&r2, a, b) > 1e-3 || side(&r1, a, b) < 1e-3, "{a}-{b} squeezed to {:e}: {:?}", side(&r2, a, b), r2.status);
    }
}

/// solidcraft #8562: a line's own end as its midpoint.
#[test]
fn a_lines_own_end_as_its_midpoint_is_named_and_refused() {
    let pts = r#"{"id":"a","x":3,"y":-11},{"id":"b","x":25,"y":-27}"#;
    let r = solve(pts, r#"{"type":"line","id":"l1","p1":"a","p2":"b"}"#, r#"{"type":"midpoint","id":"k1","refs":["b","l1"]}"#, SOLVE);
    assert!(r.problems().contains(&("sketchMidpointOwnEnd".into(), "k1".into(), "l1".into())), "{:?}", r.status);
    assert!(!r.solved() && r.has("sketchCurveShrunk"), "{:?}", r.status);
    unchanged(&r, &[("a", 3.0, -11.0), ("b", 25.0, -27.0)]);
}

/// solidcraft #8611: a line's two ends made coincident.
#[test]
fn a_lines_two_ends_made_coincident_are_named_and_refused() {
    let pts = r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":4}"#;
    let r = solve(pts, r#"{"type":"line","id":"l1","p1":"a","p2":"b"}"#, r#"{"type":"coincident","id":"k1","refs":["b","a"]}"#, SOLVE);
    assert!(r.problems().contains(&("sketchLineEndsCoincident".into(), "k1".into(), "l1".into())), "{:?}", r.status);
    assert!(!r.solved(), "{:?}", r.status);
    unchanged(&r, &[("a", 0.0, 0.0), ("b", 10.0, 4.0)]);
}

/// solidcraft #8577: one curve picked twice for a two-curve constraint.
#[test]
fn one_curve_picked_twice_is_named() {
    let pts = r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":4},{"id":"c","x":20,"y":0}"#;
    let ents = r#"{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"circle","id":"o1","c":"c","r":3}"#;
    for (kind, id) in [("parallel", "l1"), ("perpendicular", "l1"), ("collinear", "l1"), ("equal", "l1"), ("equal", "o1"), ("tangent", "o1"), ("concentric", "o1")] {
        let r = solve(pts, ents, &format!(r#"{{"type":"{kind}","id":"k1","refs":["{id}","{id}"]}}"#), STATUS);
        assert!(r.problems().contains(&("sketchSameCurveTwice".into(), "k1".into(), id.into())), "{kind}: {:?}", r.status);
    }
    // two different curves: nothing to name
    let r = solve(r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":4},{"id":"c","x":0,"y":5},{"id":"d","x":10,"y":9}"#,
        r#"{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"c","p2":"d"}"#,
        r#"{"type":"parallel","id":"k1","refs":["l1","l2"]}"#, STATUS);
    assert!(r.problems().is_empty() && r.solved(), "{:?}", r.status);
    assert!(r.status.get("problems").is_none(), "the key only appears when there is a problem");
}

/// A line held at its midpoint on the fixed origin, at 30 degrees to a horizontal line from the origin.
/// `b` is where the dragged end `q` is put, with the GUI's temporary pin there.
fn midpoint_line_drag(bx: f64, by: f64) -> Reply {
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let pts = format!(
        r#"{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"h","x":15,"y":0}},{{"id":"p","x":{},"y":{}}},{{"id":"q","x":{bx},"y":{by}}}"#,
        -10.0 * c,
        -10.0 * s
    );
    let ents = r#"{"type":"line","id":"l1","p1":"p","p2":"q"},{"type":"line","id":"l2","p1":"O","p2":"h"}"#;
    let cons = format!(
        r#"{{"type":"midpoint","id":"k1","refs":["O","l1"]}},{{"type":"horizontal","id":"k2","refs":["l2"]}},{{"type":"angle","id":"k3","refs":["l2","l1"],"value":30}},{{"type":"fix","id":"draft-fix-q","refs":["q"],"value":[{bx},{by}]}}"#
    );
    solve(&pts, ents, &cons, SOLVE)
}

/// solidcraft #8600 / #9728: an end dragged onto its line's own midpoint would press the line to no length.
#[test]
fn a_drag_that_presses_a_line_to_no_length_is_not_taken() {
    let r = midpoint_line_drag(0.0, 0.0);
    assert!(!r.solved(), "{:?} {:?}", r.status, r.pts);
    assert!(r.problems().iter().any(|p| p.0 == "sketchCurveShrunk" && p.2 == "l1"), "{:?}", r.status);
    // a drag along the line, past the midpoint, flips it over and is fine
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let r = midpoint_line_drag(-4.0 * c, -4.0 * s);
    assert!(r.solved() && r.problems().is_empty(), "{:?}", r.status);
    let p = r.xy("p");
    assert!((p.0 - 4.0 * c).abs() < 1e-6 && (p.1 - 4.0 * s).abs() < 1e-6, "{p:?}");
}

/// solidcraft #8967: an end the cursor cannot reach stays where it is, and nothing else slides.
#[test]
fn an_unreachable_drag_leaves_the_other_line_alone() {
    let r = midpoint_line_drag(14.0, 3.0);
    assert!(!r.solved(), "{:?}", r.status);
    assert_eq!(r.xy("h"), (15.0, 0.0));
}

/// solidcraft #8931 / #8562: a sketch whose line is already squeezed to a hair (so no curve shrinks in this
/// step) and an angle dimension on it: the solve must not throw points far away.
#[test]
fn an_angle_on_a_squeezed_line_never_throws_points_far_away() {
    let pts = r#"{"id":"a","x":16,"y":-22},{"id":"b","x":27,"y":6},{"id":"c","x":25,"y":-27},{"id":"d","x":25.0000001,"y":-27.00000005}"#;
    let ents = r#"{"type":"line","id":"l10","p1":"a","p2":"b"},{"type":"line","id":"l11","p1":"c","p2":"d"}"#;
    let cons = r#"{"type":"midpoint","id":"k1","refs":["d","l11"]},{"type":"angle","id":"k2","refs":["l10","l11"],"value":179}"#;
    let r = solve(pts, ents, cons, SOLVE);
    let far = r.pts.iter().map(|p| p.1.abs().max(p.2.abs())).fold(0.0f64, f64::max);
    assert!(far.is_finite() && far < 1e4, "a point went {far:e} away: {:?} {:?}", r.pts, r.status);
}
