//! Sketch-solver regression cases: a wrong branch or a wrong freedom count is a silent error
//! (solved, 0 DOF, no warning), so each class is pinned here.
//!  * parallel / perpendicular held through a far move keeps its sense (no half-turn flip)
//!  * a collapsed (~1e-6) construction line with an angle dimension stays finite and solvable
//!  * a fully constrained sketch reports 0 DOF everywhere (status, freedoms, solved coordinates)
//!  * an offset circle (concentric + radius difference) is fully constrained with its source
use ck_model::sketch::analyze::freedoms;
use ck_model::sketch::solver::{solve_sketch, status, SolveOpts};
use ck_model::sketch::status::sketch_status;
use ck_model::Sketch;

fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
    let src = format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
    Sketch::from_json(&ck_json::parse(&src).unwrap())
}

fn xy(s: &Sketch, id: &str) -> [f64; 2] {
    s.point_xy(id).unwrap()
}

/// Reference line A (0,0)-(10,0) fixed; line B from the fixed point (0,5) towards `ang` degrees at
/// a wrong length (3), with `rel` and a length dimension of 10 on B.
fn two_lines(rel: &str, ang: f64) -> Sketch {
    let (c, s) = (ang.to_radians().cos(), ang.to_radians().sin());
    let pts = format!(
        r#"{{"id":"a1","x":0,"y":0,"fixed":true}},{{"id":"a2","x":10,"y":0,"fixed":true}},{{"id":"b1","x":0,"y":5,"fixed":true}},{{"id":"b2","x":{},"y":{}}}"#,
        3.0 * c,
        5.0 + 3.0 * s
    );
    let ents = r#"{"type":"line","id":"A","p1":"a1","p2":"a2"},{"type":"line","id":"B","p1":"b1","p2":"b2"}"#;
    let cons = format!(r#"{{"type":"{rel}","id":"k1","refs":["A","B"]}},{{"type":"distance","id":"k2","refs":["b1","b2"],"value":10,"offset":null}}"#);
    sk(&pts, ents, &cons)
}

fn dir_b(s: &Sketch) -> [f64; 2] {
    let (p, q) = (xy(s, "b1"), xy(s, "b2"));
    [q[0] - p[0], q[1] - p[1]]
}

#[test]
fn parallel_keeps_its_sense_through_a_far_move() {
    // start B at every 10 degrees except near perpendicular (where either sense is equally close)
    for deg in (0..360).step_by(10).filter(|d| ((*d as f64).to_radians().cos()).abs() > 0.3) {
        let mut s = two_lines("parallel", deg as f64);
        let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(r.ok, "start {deg}: {r:?}");
        let d = dir_b(&s);
        assert!(d[1].abs() < 1e-6, "start {deg}: not parallel {d:?}");
        assert!((d[0].abs() - 10.0).abs() < 1e-6, "start {deg}: length {d:?}");
        let start_dx = (deg as f64).to_radians().cos();
        assert!(d[0] * start_dx > 0.0, "start {deg}: flipped to the half-turned solution {d:?}");
    }
}

#[test]
fn perpendicular_keeps_its_sense_through_a_far_move() {
    for deg in (0..360).step_by(10).filter(|d| ((*d as f64).to_radians().sin()).abs() > 0.3) {
        let mut s = two_lines("perpendicular", deg as f64);
        let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(r.ok, "start {deg}: {r:?}");
        let d = dir_b(&s);
        assert!(d[0].abs() < 1e-6, "start {deg}: not perpendicular {d:?}");
        assert!((d[1].abs() - 10.0).abs() < 1e-6, "start {deg}: length {d:?}");
        let start_dy = (deg as f64).to_radians().sin();
        assert!(d[1] * start_dy > 0.0, "start {deg}: flipped to the half-turned solution {d:?}");
    }
}

#[test]
fn collapsed_construction_line_with_an_angle_dimension() {
    // C is a construction line of length ~1e-6 (its direction row is ill-conditioned);
    // angle(A, C) = 30 degrees, C's first point fixed
    let pts = r#"{"id":"a1","x":0,"y":0,"fixed":true},{"id":"a2","x":10,"y":0,"fixed":true},
        {"id":"c1","x":2,"y":3,"fixed":true},{"id":"c2","x":2.000001,"y":3.0000005}"#;
    let ents = r#"{"type":"line","id":"A","p1":"a1","p2":"a2"},{"type":"line","id":"C","p1":"c1","p2":"c2","construction":true}"#;
    let cons = r#"{"type":"angle","id":"k1","refs":["A","C"],"value":30}"#;
    let mut s = sk(pts, ents, cons);
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(r.ok && r.err < 1e-8, "{r:?}");
    let (c1, c2) = (xy(&s, "c1"), xy(&s, "c2"));
    assert!(c2.iter().all(|v| v.is_finite()));
    let len = ((c2[0] - c1[0]).powi(2) + (c2[1] - c1[1]).powi(2)).sqrt();
    assert!(len > 0.0 && len < 1e-3, "the length is not constrained and must not blow up: {len:e}");
    let ang = (c2[1] - c1[1]).atan2(c2[0] - c1[0]).to_degrees();
    assert!((ang - 30.0).abs() < 1e-3, "angle {ang}");
    // the free point has 2 unknowns and 1 equation: one freedom, and the analysis agrees
    let st = status(&sk(pts, ents, cons)).unwrap();
    assert_eq!(st.dof, 1, "{st:?}");
    let f = freedoms(&sk(pts, ents, cons));
    assert_eq!(f.dof, 1);
    assert!(f.points.iter().all(|p| p.free <= 2));
}

#[test]
fn collapsed_construction_line_with_a_length_pins_it_fully() {
    let pts = r#"{"id":"a1","x":0,"y":0,"fixed":true},{"id":"a2","x":10,"y":0,"fixed":true},
        {"id":"c1","x":2,"y":3,"fixed":true},{"id":"c2","x":2.000001,"y":3.0000005}"#;
    let ents = r#"{"type":"line","id":"A","p1":"a1","p2":"a2"},{"type":"line","id":"C","p1":"c1","p2":"c2","construction":true}"#;
    let cons = r#"{"type":"angle","id":"k1","refs":["A","C"],"value":30},{"type":"distance","id":"k2","refs":["c1","c2"],"value":4,"offset":null}"#;
    let mut s = sk(pts, ents, cons);
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    let (c1, c2) = (xy(&s, "c1"), xy(&s, "c2"));
    assert!((c2[0] - c1[0] - 4.0 * 30f64.to_radians().cos()).abs() < 1e-6, "{c2:?}");
    assert!((c2[1] - c1[1] - 4.0 * 30f64.to_radians().sin()).abs() < 1e-6, "{c2:?}");
    let st = sketch_status(&sk(pts, ents, cons));
    assert_eq!((st.dof, st.state, st.fully_defined), (0, "full", true), "{st:?}");
}

#[test]
fn fully_constrained_sketch_has_no_freedoms() {
    // triangle: p1 fixed, e1 horizontal and 10 long, e2 6 long at 40 degrees to e1, closing edge free of constraints
    let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":9,"y":1},{"id":"p3","x":4,"y":5}"#;
    let ents = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p1","p2":"p3"},{"type":"line","id":"e3","p1":"p2","p2":"p3"}"#;
    let cons = r#"{"type":"fix","id":"k0","refs":["p1"],"value":[0,0]},{"type":"horizontal","id":"k1","refs":["e1"]},
        {"type":"distance","id":"k2","refs":["p1","p2"],"value":10,"offset":null},
        {"type":"distance","id":"k3","refs":["p1","p3"],"value":6,"offset":null},
        {"type":"angle","id":"k4","refs":["e1","e2"],"value":40}"#;
    let s = sk(pts, ents, cons);
    let st = sketch_status(&s);
    assert_eq!((st.dof, st.state, st.fully_defined), (0, "full", true), "{st:?}");
    assert!(st.conflicts.is_empty() && st.redundant.is_empty());
    assert_eq!(status(&s).unwrap().dof, 0);
    let f = freedoms(&s);
    assert_eq!(f.dof, 0);
    assert!(f.points.iter().all(|p| p.free == 0), "{:?}", f.points);
    let mut s = s;
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    let p3 = xy(&s, "p3");
    assert!((p3[0] - 6.0 * 40f64.to_radians().cos()).abs() < 1e-6 && (p3[1] - 6.0 * 40f64.to_radians().sin()).abs() < 1e-6, "{p3:?}");
}

const CIRCLES: &str = r#"{"id":"c","x":3,"y":4},{"id":"d","x":3.5,"y":4.2}"#;
const CIRCLE_ENTS: &str = r#"{"type":"circle","id":"src","c":"c","r":5},{"type":"circle","id":"off","c":"d","r":7.5}"#;

#[test]
fn offset_circle_is_fully_constrained_with_its_source() {
    // source: centre fixed, radius 5; offset: concentric with it, 2 larger
    let cons = r#"{"type":"fix","id":"k0","refs":["c"],"value":[3,4]},{"type":"radius","id":"k1","refs":["src"],"value":5},
        {"type":"coincident","id":"k2","refs":["c","d"]},{"type":"raddiff","id":"k3","refs":["src","off"],"value":2}"#;
    let s = sk(CIRCLES, CIRCLE_ENTS, cons);
    let st = sketch_status(&s);
    assert_eq!((st.dof, st.state, st.fully_defined), (0, "full", true), "{st:?}");
    let f = freedoms(&s);
    assert_eq!(f.dof, 0);
    assert!(f.radii.iter().all(|r| !r.free), "{:?}", f.radii);
    let mut s = s;
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    let d = xy(&s, "d");
    assert!((d[0] - 3.0).abs() < 1e-6 && (d[1] - 4.0).abs() < 1e-6, "{d:?}");
    let r = s.entities.iter().find_map(|e| match e {
        ck_model::SketchEntity::Circle { id, r, .. } if id == "off" => Some(*r),
        _ => None,
    });
    assert!((r.unwrap() - 7.0).abs() < 1e-9, "{r:?}");
}

#[test]
fn offset_circle_follows_its_source_when_the_source_is_free() {
    // dropping the source radius leaves exactly one freedom (the shared radius), not two
    let cons = r#"{"type":"fix","id":"k0","refs":["c"],"value":[3,4]},
        {"type":"coincident","id":"k2","refs":["c","d"]},{"type":"raddiff","id":"k3","refs":["src","off"],"value":2}"#;
    let s = sk(CIRCLES, CIRCLE_ENTS, cons);
    assert_eq!(sketch_status(&s).dof, 1);
    assert_eq!(freedoms(&s).dof, 1);
}
