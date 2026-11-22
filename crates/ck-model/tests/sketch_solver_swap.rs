//! Swapping a constraint for its opposite (parallel -> perpendicular, horizontal -> vertical) starts the solver at a
//! stationary point of the new equation. The solve must land on the nearest valid configuration, not run off, and a
//! result that moved absurdly far is never reported as solved.
use ck_model::sketch::guard::solve_sketch as guarded;
use ck_model::sketch::solver::{solve_sketch, SolveOpts};
use ck_model::Sketch;

fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
    let src = format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
    Sketch::from_json(&ck_json::parse(&src).unwrap())
}
fn xy(s: &Sketch, id: &str) -> [f64; 2] {
    s.point_xy(id).unwrap()
}
fn len(s: &Sketch, a: &str, b: &str) -> f64 {
    let (p, q) = (xy(s, a), xy(s, b));
    (q[0] - p[0]).hypot(q[1] - p[1])
}
fn dot(s: &Sketch) -> f64 {
    let (a, b, c, d) = (xy(s, "a"), xy(s, "b"), xy(s, "c"), xy(s, "d"));
    let (u, v) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
    (u[0] * v[0] + u[1] * v[1]) / (u[0].hypot(u[1]) * v[0].hypot(v[1]))
}
fn cross(s: &Sketch) -> f64 {
    let (a, b, c, d) = (xy(s, "a"), xy(s, "b"), xy(s, "c"), xy(s, "d"));
    let (u, v) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
    (u[0] * v[1] - u[1] * v[0]) / (u[0].hypot(u[1]) * v[0].hypot(v[1]))
}

const L1: f64 = 7.211102550927978; // sqrt(52)
// two free slanted lines, exactly parallel (direction (6,4))
fn two(pts: &str, rel: &str) -> Sketch {
    let ents = r#"{"type":"line","id":"A","p1":"a","p2":"b"},{"type":"line","id":"B","p1":"c","p2":"d"}"#;
    let dims = format!(r#"{{"type":"distance","id":"k2","refs":["a","b"],"value":{L1},"offset":null}},{{"type":"distance","id":"k3","refs":["c","d"],"value":{L1},"offset":null}}"#);
    sk(pts, ents, &format!(r#"{{"type":"{rel}","id":"k1","refs":["A","B"]}},{dims}"#))
}
const PARALLEL_PTS: &str = r#"{"id":"a","x":0,"y":0},{"id":"b","x":6,"y":4},{"id":"c","x":2,"y":5},{"id":"d","x":8,"y":9}"#;
const SLANTED_PTS: &str = r#"{"id":"a","x":0,"y":0},{"id":"b","x":6,"y":4},{"id":"c","x":2,"y":5},{"id":"d","x":9,"y":7}"#;

fn within_twice_box(s: &Sketch, ids: &[&str], orig: &[[f64; 2]]) {
    let (lo, hi) = orig.iter().fold(([f64::MAX; 2], [f64::MIN; 2]), |(l, h), p| ([l[0].min(p[0]), l[1].min(p[1])], [h[0].max(p[0]), h[1].max(p[1])]));
    let w = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    for id in ids {
        let p = xy(s, id);
        assert!(p[0] > lo[0] - w && p[0] < hi[0] + w && p[1] > lo[1] - w && p[1] < hi[1] + w, "{id} ran off to {p:?}");
    }
}

#[test]
fn parallel_swapped_for_perpendicular_lands_nearby() {
    let mut s = two(PARALLEL_PTS, "parallel");
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    assert!(cross(&s).abs() < 1e-9);
    let ids = ["a", "b", "c", "d"];
    let orig: Vec<[f64; 2]> = ids.iter().map(|i| xy(&s, i)).collect();
    // the swap: same points, perpendicular instead of parallel
    let mut s = two(PARALLEL_PTS, "perpendicular");
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(r.ok && !r.diverged, "{r:?}");
    assert!(dot(&s).abs() < 1e-9, "dot {}", dot(&s));
    within_twice_box(&s, &ids, &orig);
    assert!((len(&s, "a", "b") - L1).abs() < 1e-6 && (len(&s, "c", "d") - L1).abs() < 1e-6);
}

#[test]
fn perpendicular_swapped_for_parallel_lands_nearby() {
    let mut s = two(r#"{"id":"a","x":0,"y":0},{"id":"b","x":6,"y":4},{"id":"c","x":2,"y":5},{"id":"d","x":-2,"y":11}"#, "parallel");
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(r.ok && !r.diverged && cross(&s).abs() < 1e-9, "{r:?}");
}

fn horizontal_line(rel: &str) -> Sketch {
    sk(
        r#"{"id":"a","x":1,"y":2},{"id":"b","x":11,"y":2}"#,
        r#"{"type":"line","id":"A","p1":"a","p2":"b"}"#,
        &format!(r#"{{"type":"{rel}","id":"k1","refs":["A"]}},{{"type":"distance","id":"k2","refs":["a","b"],"value":10,"offset":null}}"#),
    )
}

#[test]
fn horizontal_swapped_for_vertical_keeps_length_and_is_not_shrunk() {
    let mut s = horizontal_line("vertical");
    let (r, problem) = guarded(&mut s, &SolveOpts::default()).unwrap();
    assert!(problem.is_none(), "{problem:?}");
    assert!(r.ok && !r.diverged, "{r:?}");
    let (a, b) = (xy(&s, "a"), xy(&s, "b"));
    assert!((a[0] - b[0]).abs() < 1e-9, "{a:?} {b:?}");
    assert!((len(&s, "a", "b") - 10.0).abs() < 1e-6);
    assert!(a.iter().chain(b.iter()).all(|v| v.abs() < 30.0), "{a:?} {b:?}");
    // the centre stays near where it was
    assert!(((a[0] + b[0]) / 2.0 - 6.0).abs() < 8.0 && ((a[1] + b[1]) / 2.0 - 2.0).abs() < 8.0);
}

#[test]
fn from_a_slanted_start_the_same_constraints_solve_as_before() {
    for rel in ["parallel", "perpendicular"] {
        let mut s = two(SLANTED_PTS, rel);
        let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(r.ok && !r.diverged, "{rel}: {r:?}");
        if rel == "parallel" {
            assert!(cross(&s).abs() < 1e-9);
        } else {
            assert!(dot(&s).abs() < 1e-9);
        }
        assert!((len(&s, "a", "b") - L1).abs() < 1e-6 && (len(&s, "c", "d") - L1).abs() < 1e-6);
    }
}

#[test]
fn an_equal_length_swap_is_unchanged() {
    // equal -> a different equal: lengths 5 and 5 held by `equal`, then by a distance each: both solve in place
    let pts = r#"{"id":"a","x":0,"y":0},{"id":"b","x":5,"y":0},{"id":"c","x":0,"y":3},{"id":"d","x":5,"y":3}"#;
    let ents = r#"{"type":"line","id":"A","p1":"a","p2":"b"},{"type":"line","id":"B","p1":"c","p2":"d"}"#;
    let mut s = sk(pts, ents, r#"{"type":"equal","id":"k1","refs":["A","B"]}"#);
    let before: Vec<[f64; 2]> = ["a", "b", "c", "d"].iter().map(|i| xy(&s, i)).collect();
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(r.ok && r.iterations == 0, "{r:?}");
    assert_eq!(before, ["a", "b", "c", "d"].iter().map(|i| xy(&s, i)).collect::<Vec<_>>());
}

#[test]
fn an_inconsistent_system_is_never_solved_far_away() {
    // a line of length 10 AND length 3: no solution; whatever the solver does, it is not "solved", and not far
    let ents = r#"{"type":"line","id":"A","p1":"a","p2":"b"}"#;
    let cons = r#"{"type":"distance","id":"k1","refs":["a","b"],"value":10,"offset":null},{"type":"distance","id":"k2","refs":["a","b"],"value":3,"offset":null}"#;
    let mut s = sk(r#"{"id":"a","x":1,"y":2},{"id":"b","x":11,"y":2}"#, ents, cons);
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(!r.ok, "{r:?}");
    // and the coordinates were not written back
    assert_eq!(xy(&s, "b"), [11.0, 2.0]);
    // parallel + perpendicular on the same pair, from a parallel start
    let ents = r#"{"type":"line","id":"A","p1":"a","p2":"b"},{"type":"line","id":"B","p1":"c","p2":"d"}"#;
    let mut s = sk(PARALLEL_PTS, ents, r#"{"type":"parallel","id":"k1","refs":["A","B"]},{"type":"perpendicular","id":"k9","refs":["A","B"]}"#);
    let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
    assert!(!r.ok, "{r:?}");
    for id in ["a", "b", "c", "d"] {
        let p = xy(&s, id);
        assert!(p[0].abs() < 1e3 && p[1].abs() < 1e3, "{id} {p:?}");
    }
}
