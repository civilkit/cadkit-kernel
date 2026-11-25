//! `arclength` dimension (radius x counter-clockwise sweep) and `driven` (measured) dimensions.
use ck_model::sketch::analyze::{analyze, freedoms};
use ck_model::sketch::solver::{build, solve_sketch, status, SolveOpts};
use ck_model::sketch::status::sketch_status;
use ck_model::Sketch;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
    let src = format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
    Sketch::from_json(&ck_json::parse(&src).unwrap())
}
fn xy(s: &Sketch, id: &str) -> [f64; 2] {
    s.point_xy(id).unwrap()
}
fn solved(mut s: Sketch) -> Sketch {
    assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
    s
}
/// Centre fixed at the origin, start fixed at (20, 0), end free at `end`.
fn arc(end: [f64; 2], cons: &str) -> Sketch {
    sk(
        &format!(r#"{{"id":"o","x":0,"y":0,"fixed":true}},{{"id":"a","x":20,"y":0,"fixed":true}},{{"id":"b","x":{},"y":{}}}"#, end[0], end[1]),
        r#"{"type":"arc","id":"A","c":"o","p1":"a","p2":"b"}"#,
        cons,
    )
}
const AL: &str = r#"{"type":"arclength","id":"k1","refs":["A"],"value":25}"#;
fn length(s: &Sketch) -> f64 {
    let (c, a, b) = (xy(s, "o"), xy(s, "a"), xy(s, "b"));
    let r = (a[0] - c[0]).hypot(a[1] - c[1]);
    let mut t = ((b[1] - c[1]).atan2(b[0] - c[0]) - (a[1] - c[1]).atan2(a[0] - c[0])).rem_euclid(TAU);
    if t < 1e-9 {
        t = TAU;
    }
    r * t
}

#[test]
fn arc_length_reaches_the_value_from_a_ninety_degree_arc_and_from_poor_starts() {
    for end in [[0.0, 20.0], [19.0, 6.0], [-20.0, 1.0], [10.0, -17.0], [0.0, -20.0]] {
        let s = solved(arc(end, AL));
        assert!((length(&s) - 25.0).abs() < 1e-6, "{end:?}: {}", length(&s));
        // the arc keeps its radius (the arc's own link equation)
        let b = xy(&s, "b");
        assert!((b[0].hypot(b[1]) - 20.0).abs() < 1e-6, "{end:?}");
    }
}

#[test]
fn arc_length_is_one_dof_in_every_reader() {
    let free = arc([0.0, 20.0], "");
    let one = arc([0.0, 20.0], AL);
    let d = |s: &Sketch| (status(s).unwrap().dof, sketch_status(s).dof, freedoms(s).dof);
    let (a, b, c) = d(&free);
    assert!(a == b && b == c);
    let (a1, b1, c1) = d(&one);
    assert!(a1 == b1 && b1 == c1);
    assert_eq!(a1 + 1, a);
    assert!(sketch_status(&solved(one)).solved);
}

#[test]
fn residual_and_jacobian_match_the_closed_form() {
    let s = arc([12.0, 15.0], AL);
    let sys = build(&s).unwrap();
    let eq = sys.eqs.iter().find(|e| e.cid == "k1").expect("arclength equation");
    let r0 = eq.eval(&sys.x);
    assert!((r0 - (length(&s) - 25.0)).abs() < 1e-9);
    // d(r * theta)/d(end): with the radius link fixed by the start, the end only turns the sweep
    let (bx, by) = (sys.key_idx["b.x"], sys.key_idx["b.y"]);
    let (b, rb2) = ([12.0f64, 15.0], 12.0f64 * 12.0 + 15.0 * 15.0);
    let h = 1e-6;
    for (i, ana) in [(bx, 20.0 * -b[1] / rb2), (by, 20.0 * b[0] / rb2)] {
        let (mut p, mut m) = (sys.x.clone(), sys.x.clone());
        p[i] += h;
        m[i] -= h;
        let fd = (eq.eval(&p) - eq.eval(&m)) / (2.0 * h);
        assert!((fd - ana).abs() < 1e-6, "{i}: fd {fd} vs {ana}");
    }
}

#[test]
fn full_circle_ends_count_as_two_pi_and_a_circle_as_its_circumference() {
    // coincident ends: 2 pi r, never 0
    let s = arc([20.0, 0.0], "");
    let sys = build(&sk(
        r#"{"id":"o","x":0,"y":0,"fixed":true},{"id":"a","x":20,"y":0,"fixed":true},{"id":"b","x":20,"y":0}"#,
        r#"{"type":"arc","id":"A","c":"o","p1":"a","p2":"b"}"#,
        r#"{"type":"arclength","id":"k1","refs":["A"],"value":0}"#,
    ))
    .unwrap();
    assert!((sys.eqs.iter().find(|e| e.cid == "k1").unwrap().eval(&sys.x) - TAU * 20.0).abs() < 1e-9);
    let _ = s;
    // a circle dimensioned to its circumference solves the radius
    let c = solved(sk(
        r#"{"id":"o","x":0,"y":0,"fixed":true}"#,
        r#"{"type":"circle","id":"C","c":"o","r":3}"#,
        &format!(r#"{{"type":"arclength","id":"k1","refs":["C"],"value":{}}}"#, TAU * 7.5),
    ));
    match c.entity("C") {
        Some(ck_model::SketchEntity::Circle { r, .. }) => assert!((r - 7.5).abs() < 1e-6, "{r}"),
        o => panic!("{o:?}"),
    }
    // a line is refused
    let l = sk(
        r#"{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":0}"#,
        r#"{"type":"line","id":"L","p1":"a","p2":"b"}"#,
        r#"{"type":"arclength","id":"k1","refs":["L"],"value":2}"#,
    );
    assert!(build(&l).is_err());
}

#[test]
fn driven_arc_length_reports_the_measured_value_and_does_not_constrain() {
    let cons = r#"{"type":"arclength","id":"k1","refs":["A"],"value":99,"driven":true}"#;
    let s = arc([0.0, 20.0], cons);
    assert_eq!(sketch_status(&s).dof, sketch_status(&arc([0.0, 20.0], "")).dof);
    let v = ck_json::to_string(&s.to_json());
    let back = Sketch::from_json(&ck_json::parse(&v).unwrap());
    let m = back.constraints[0].value.unwrap();
    assert!((m - 20.0 * FRAC_PI_2).abs() < 1e-9, "{m}");
    // a driving dimension on the same sketch moves the end and the driven one follows
    let both = format!(r#"{cons},{{"type":"distance","id":"k2","refs":["a","b"],"value":20}}"#);
    let s = solved(arc([0.0, 20.0], &both));
    let want = 20.0 * 2.0 * (0.5f64 * 20.0 / 20.0).asin(); // chord = r: 60 degrees
    let j = s.to_json();
    let m = Sketch::from_json(&j).constraints[0].value.unwrap();
    assert!((m - want).abs() < 1e-6 && (m - 20.0 * PI / 3.0).abs() < 1e-6, "{m}");
}

#[test]
fn round_trip_through_json_keeps_type_value_and_driven() {
    let s = arc([0.0, 20.0], AL);
    let j = ck_json::to_string(&s.to_json());
    assert!(j.contains(r#""type":"arclength""#) && j.contains(r#""value":25"#), "{j}");
    let back = Sketch::from_json(&ck_json::parse(&j).unwrap());
    let k = &back.constraints[0];
    assert!(k.kind == "arclength" && k.is_dimension() && !k.driven && k.value == Some(25.0));
    let d = arc([0.0, 20.0], r#"{"type":"arclength","id":"k1","refs":["A"],"driven":true}"#);
    let j = ck_json::to_string(&d.to_json());
    assert!(j.contains(r#""driven":true"#) && j.contains(r#""value":"#), "{j}");
}

#[test]
fn driven_dimensions_to_a_converted_model_edge_measure_it() {
    // what `convert` writes: fixed points, a fixed line carrying `from`; a driven pldist to it measures
    let s = sk(
        r#"{"id":"p","x":3,"y":7},{"id":"m1","x":-5,"y":2,"fixed":true},{"id":"m2","x":9,"y":2,"fixed":true}"#,
        r#"{"type":"line","id":"E","p1":"m1","p2":"m2","construction":true,"from":{"kind":"edge","faces":["F2.S.e1","F2.S.e2"],"at":[2,2,0],"shape":"line"}}"#,
        r#"{"type":"pldist","id":"k1","refs":["p","E"],"driven":true},{"type":"distance","id":"k2","refs":["p","m2"],"driven":true}"#,
    );
    let j = s.to_json();
    let back = Sketch::from_json(&j);
    assert!((back.constraints[0].value.unwrap() - 5.0).abs() < 1e-9);
    assert!((back.constraints[1].value.unwrap() - (36.0f64 + 25.0).sqrt()).abs() < 1e-9);
}

#[test]
fn analyze_and_freedoms_carry_the_measured_value_of_driven_dimensions() {
    let s = arc([0.0, 20.0], r#"{"type":"arclength","id":"k1","refs":["A"],"driven":true},{"type":"radius","id":"k2","refs":["A"],"driven":true}"#);
    let want = 20.0 * FRAC_PI_2;
    for j in [analyze(&s, 1e-6, 1e-6).to_json(), freedoms(&s).to_json()] {
        let d = j.get("driven").and_then(|v| v.as_arr()).expect("driven array");
        assert_eq!(d.len(), 2);
        assert_eq!(d[0].get("id").and_then(|v| v.as_str()), Some("k1"));
        assert!((d[0].get("value").and_then(|v| v.as_f64()).unwrap() - want).abs() < 1e-9);
        assert!((d[1].get("value").and_then(|v| v.as_f64()).unwrap() - 20.0).abs() < 1e-9);
    }
    // no driven dimension: the key is absent
    assert!(freedoms(&arc([0.0, 20.0], AL)).to_json().get("driven").is_none());
}
