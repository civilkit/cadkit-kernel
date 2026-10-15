//! Thin-wall extrude / revolve (`"thin": {thickness, side, ends, cornerStyle, thickness2}`): analytic
//! wall volumes of arcs, lines, rectangles, circles, a corner chain and a free-form spline arc,
//! the cut and revolve variants, face names and the structured too-thick / bad-input errors.
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
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-9)
}

fn param(e: &regen::FeatureError, k: &str) -> Option<Json> {
    e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
}

fn check_valid(o: &Out, what: &str) {
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn names(o: &Out) -> Vec<String> {
    let b = o.brep.body.as_ref().unwrap();
    b.face_ids().map(|f| b.face_id_str(f)).collect()
}

fn sketch(points: &[(&str, f64, f64)], entities: &str) -> String {
    let p: Vec<String> = points.iter().map(|(i, x, y)| format!(r#"{{"id":"{i}","x":{x},"y":{y}}}"#)).collect();
    format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{entities}],"constraints":[]}}"#, p.join(","))
}

fn part(sk: &str, rest: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{rest}]}}"#)
}

fn extrude(thin: &str, h: f64) -> String {
    format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h},"thin":{thin}}}"#)
}

/// Upper half circle of radius 10 about the origin, counter-clockwise from (10,0) to (-10,0).
fn half_circle() -> String {
    sketch(&[("c", 0.0, 0.0), ("p1", 10.0, 0.0), ("p2", -10.0, 0.0)], r#"{"type":"arc","id":"e1","c":"c","p1":"p1","p2":"p2"}"#)
}

const H: f64 = 5.0;

#[test]
fn half_circle_ring_every_side() {
    let ring = |ro: f64, ri: f64| PI / 2.0 * (ro * ro - ri * ri) * H;
    let cases: [(&str, f64); 5] = [
        (r#"{"thickness":2,"side":"inside"}"#, ring(10.0, 8.0)),
        (r#"{"thickness":2,"side":"outside"}"#, ring(12.0, 10.0)),
        (r#"{"thickness":2,"side":"mid"}"#, ring(11.0, 9.0)),
        (r#"{"thickness":2,"side":"both"}"#, ring(12.0, 8.0)),
        (r#"{"thickness":2,"thickness2":1,"side":"both"}"#, ring(11.0, 8.0)),
    ];
    for (thin, expect) in cases {
        let o = run(&part(&half_circle(), &extrude(thin, H)));
        assert!(o.errors.is_empty(), "{thin}: {:?}", o.errors);
        assert!(rel(vol(&o), expect) < 1e-6, "{thin}: {} vs {expect}", vol(&o));
        check_valid(&o, thin);
    }
    // round ends add two half discs of radius t/2
    let o = run(&part(&half_circle(), &extrude(r#"{"thickness":2,"side":"mid","ends":"round"}"#, H)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), ring(11.0, 9.0) + PI * 1.0 * H) < 1e-6, "{}", vol(&o));
    check_valid(&o, "round ends");
}

#[test]
fn face_names_are_persistent() {
    let o = run(&part(&half_circle(), &extrude(r#"{"thickness":2,"side":"inside"}"#, H)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let n = names(&o);
    for want in ["F2.S.e1.W0", "F2.S.e1.W1", "F2.S.e1.E0", "F2.S.e1.E1", "F2.C0", "F2.C1"] {
        assert!(n.contains(&want.to_string()), "{want} missing in {n:?}");
    }
    assert_eq!(n.len(), 6, "{n:?}");
    // the offset side of an inside wall is the smaller radius
    let b = o.brep.body.as_ref().unwrap();
    let w0 = b.face_ids().find(|&f| b.face_id_str(f) == "F2.S.e1.W0").unwrap();
    match &b.face(w0).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => assert!((r - 8.0).abs() < 1e-9, "{r}"),
        s => panic!("W0 is {}", s.kind()),
    }
    // closed loop: outer W1 / inner W0 walls
    let c = sketch(&[("c", 0.0, 0.0)], r#"{"type":"circle","id":"e1","c":"c","r":10}"#);
    let o = run(&part(&c, &extrude(r#"{"thickness":2}"#, H)));
    let n = names(&o);
    assert!(n.contains(&"F2.S.e1.W0".to_string()) && n.contains(&"F2.S.e1.W1".to_string()), "{n:?}");
}

#[test]
fn straight_line_wall() {
    let s = sketch(&[("a", 0.0, 0.0), ("b", 20.0, 0.0)], r#"{"type":"line","id":"e1","p1":"a","p2":"b"}"#);
    for side in ["inside", "outside"] {
        let o = run(&part(&s, &extrude(&format!(r#"{{"thickness":2,"side":"{side}"}}"#), H)));
        assert!(o.errors.is_empty(), "{:?}", o.errors);
        assert!(rel(vol(&o), 2.0 * 20.0 * H) < 1e-9, "{}", vol(&o));
        check_valid(&o, side);
    }
}

#[test]
fn rectangle_hollow_wall() {
    let s = sketch(
        &[("a", 0.0, 0.0), ("b", 20.0, 0.0), ("c", 20.0, 10.0), ("d", 0.0, 10.0)],
        r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}"#,
    );
    let cases = [
        (r#"{"thickness":1,"side":"inside"}"#, 200.0 - 18.0 * 8.0),
        (r#"{"thickness":1,"side":"outside"}"#, 22.0 * 12.0 - 200.0),
        (r#"{"thickness":2,"side":"mid"}"#, 22.0 * 12.0 - 18.0 * 8.0),
        (r#"{"thickness":1,"side":"outside","cornerStyle":"round"}"#, 60.0 + PI),
    ];
    for (thin, area) in cases {
        let o = run(&part(&s, &extrude(thin, H)));
        assert!(o.errors.is_empty(), "{thin}: {:?}", o.errors);
        assert!(rel(vol(&o), area * H) < 1e-6, "{thin}: {} vs {}", vol(&o), area * H);
        check_valid(&o, thin);
    }
}

#[test]
fn circle_hollow_wall() {
    let c = sketch(&[("c", 0.0, 0.0)], r#"{"type":"circle","id":"e1","c":"c","r":10}"#);
    for (side, ro, ri) in [("inside", 10.0, 8.0), ("outside", 12.0, 10.0), ("mid", 11.0, 9.0)] {
        let o = run(&part(&c, &extrude(&format!(r#"{{"thickness":2,"side":"{side}"}}"#), H)));
        assert!(o.errors.is_empty(), "{:?}", o.errors);
        assert!(rel(vol(&o), PI * (ro * ro - ri * ri) * H) < 1e-6, "{side}: {}", vol(&o));
        check_valid(&o, side);
    }
}

#[test]
fn corner_chain_and_tangent_chain() {
    // L: (0,0)-(20,0)-(20,10), mid 2: mitre area 60, round outer corner 60 - 1 + pi/4
    let l = sketch(
        &[("a", 0.0, 0.0), ("b", 20.0, 0.0), ("c", 20.0, 10.0)],
        r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"}"#,
    );
    for (thin, area) in [
        (r#"{"thickness":2,"side":"mid"}"#, 60.0),
        (r#"{"thickness":2,"side":"mid","cornerStyle":"round"}"#, 59.0 + PI / 4.0),
        // inside: only the left strip, the inner corner is trimmed: 2 * (20 + 10) - 2 * 2
        (r#"{"thickness":2,"side":"inside"}"#, 2.0 * 30.0 - 4.0),
    ] {
        let o = run(&part(&l, &extrude(thin, H)));
        assert!(o.errors.is_empty(), "{thin}: {:?}", o.errors);
        assert!(rel(vol(&o), area * H) < 1e-6, "{thin}: {} vs {}", vol(&o), area * H);
        check_valid(&o, thin);
    }
    // a line and a tangent quarter circle: area = t * length exactly (mid)
    let t = sketch(
        &[("a", 0.0, 0.0), ("b", 10.0, 0.0), ("c", 10.0, 10.0), ("d", 20.0, 10.0)],
        r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"c","p1":"b","p2":"d"}"#,
    );
    let o = run(&part(&t, &extrude(r#"{"thickness":2,"side":"mid"}"#, H)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), 2.0 * (10.0 + 10.0 * PI / 2.0) * H) < 1e-6, "{}", vol(&o));
    check_valid(&o, "tangent chain");
}

const C_ARC: [(f64, f64); 6] = [(20.0, 0.0), (14.0, 14.0), (0.0, 20.0), (-14.0, 14.0), (-20.0, 0.0), (-14.0, -14.0)];

fn spline_arc(kind: &str) -> String {
    let p: Vec<(String, f64, f64)> = C_ARC.iter().enumerate().map(|(i, (x, y))| (format!("p{i}"), *x, *y)).collect();
    let pts: Vec<(&str, f64, f64)> = p.iter().map(|(i, x, y)| (i.as_str(), *x, *y)).collect();
    let ids: Vec<String> = (0..C_ARC.len()).map(|i| format!("\"p{i}\"")).collect();
    sketch(&pts, &format!(r#"{{"type":"{kind}","id":"e1","pts":[{}],"closed":false{}}}"#, ids.join(","), if kind == "cvspline" { r#","degree":3"# } else { "" }))
}

fn cap_polygon(o: &Out, face: &str) -> Vec<(f64, f64)> {
    let b = o.brep.body.as_ref().unwrap();
    let cap = b.face_ids().find(|&f| b.face_id_str(f) == face).unwrap();
    let mut pts = vec![];
    for &l in &b.face(cap).loops {
        for &c in &b.lp(l).coedges {
            let (ta, tb) = b.coedge_params(c);
            let ed = b.edge(b.coedge(c).edge);
            for k in 0..2000 {
                let p = ed.curve.eval(ta + (tb - ta) * k as f64 / 2000.0);
                pts.push((p.x, p.y));
            }
        }
    }
    pts
}

fn area(p: &[(f64, f64)]) -> f64 {
    let n = p.len();
    (0.5 * (0..n).map(|i| p[i].0 * p[(i + 1) % n].1 - p[(i + 1) % n].0 * p[i].1).sum::<f64>()).abs()
}

#[test]
fn spline_arc_wall_matches_the_offset_polygon() {
    for kind in ["spline", "cvspline"] {
        let sk = spline_arc(kind);
        let t = 3.0;
        let run_side = |side: &str| run(&part(&sk, &extrude(&format!(r#"{{"thickness":{t},"side":"{side}"}}"#), H)));
        let (i, o, m) = (run_side("inside"), run_side("outside"), run_side("mid"));
        for r in [&i, &o, &m] {
            assert!(r.errors.is_empty(), "{kind}: {:?}", r.errors);
            check_valid(r, kind);
        }
        // band areas: in = tL - t^2/2 Th, out = tL + t^2/2 Th, mid = tL: the mid band is their mean
        let (vi, vo, vm) = (vol(&i), vol(&o), vol(&m));
        assert!(rel(vm, 0.5 * (vi + vo)) < 5e-3, "{kind}: mid {vm} vs mean {}", 0.5 * (vi + vo));
        // the cap outline is the polygon of the wall: its area (shoelace on the dense outline) is the volume / H
        let poly = cap_polygon(&m, "F2.C0");
        assert!(rel(area(&poly), vm / H) < 5e-3, "{kind}: polygon {} vs {}", area(&poly), vm / H);
        // distance of every W0 sample to the W1 outline is the thickness (inside: W0 is the offset)
        let b = i.brep.body.as_ref().unwrap();
        let sample = |name: &str| -> Vec<(f64, f64)> {
            let f = b.face_ids().find(|&f| b.face_id_str(f) == name).unwrap_or_else(|| panic!("{name} missing: {:?}", names(&i)));
            let mut v = vec![];
            for &l in &b.face(f).loops {
                for &c in &b.lp(l).coedges {
                    let (ta, tb) = b.coedge_params(c);
                    let ed = b.edge(b.coedge(c).edge);
                    for k in 0..400 {
                        let p = ed.curve.eval(ta + (tb - ta) * k as f64 / 400.0);
                        v.push((p.x, p.y));
                    }
                }
            }
            v
        };
        let (w0, w1) = (sample("F2.S.e1.W0"), sample("F2.S.e1.W1"));
        let mut worst: f64 = 0.0;
        for p in &w0 {
            let d = w1.iter().map(|q| (p.0 - q.0).hypot(p.1 - q.1)).fold(f64::INFINITY, f64::min);
            worst = worst.max((d - t).abs());
        }
        assert!(worst < 0.05 * t, "{kind}: offset distance deviates by {worst}");
    }
}

#[test]
fn too_thick_gives_hints_not_a_panic() {
    // radius 10 half circle, inside 15: past the centre of curvature
    let o = run(&part(&half_circle(), &extrude(r#"{"thickness":15,"side":"inside"}"#, H)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    let e = &o.errors[0].error;
    assert_eq!(e.code, "thinTooThick", "{}", e.describe());
    let min_r = param(e, "minRadius").and_then(|j| j.as_f64()).expect("minRadius");
    let max_t = param(e, "maxThickness").and_then(|j| j.as_f64()).expect("maxThickness");
    assert!((min_r - 10.0).abs() < 0.01, "{min_r}");
    assert!(max_t > 0.0 && max_t < 10.0, "{max_t}");
    let ok = run(&part(&half_circle(), &extrude(&format!(r#"{{"thickness":{max_t},"side":"inside"}}"#), H)));
    assert!(ok.errors.is_empty(), "{max_t}: {:?}", ok.errors);
    // the same arc pushed outward has no limit
    let out = run(&part(&half_circle(), &extrude(r#"{"thickness":15,"side":"outside"}"#, H)));
    assert!(out.errors.is_empty(), "{:?}", out.errors);
    // spline arc: the tightest bend of the C is about radius 20
    let sk = spline_arc("spline");
    let r = run(&part(&sk, &extrude(r#"{"thickness":60,"side":"inside"}"#, H)));
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    let e = &r.errors[0].error;
    assert_eq!(e.code, "thinTooThick", "{}", e.describe());
    let max_t = param(e, "maxThickness").and_then(|j| j.as_f64()).expect("maxThickness");
    assert!(param(e, "minRadius").is_some() && max_t > 0.0 && max_t < 60.0, "{}", e.describe());
    let ok = run(&part(&sk, &extrude(&format!(r#"{{"thickness":{max_t},"side":"inside"}}"#), H)));
    assert!(ok.errors.is_empty(), "suggested {max_t}: {:?}", ok.errors);
    // a short leg eaten by an inner corner: a U with 1 mm legs
    let v = sketch(
        &[("a", 0.0, 0.0), ("b", 20.0, 0.0), ("c", 20.0, 1.0), ("d", 0.0, 1.0)],
        r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"}"#,
    );
    let r = run(&part(&v, &extrude(r#"{"thickness":3,"side":"inside"}"#, H)));
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    let e = &r.errors[0].error;
    assert_eq!(e.code, "thinTooThick", "{}", e.describe());
    assert!(param(e, "maxThickness").and_then(|j| j.as_f64()).unwrap() < 1.0);
}

#[test]
fn bad_input_is_reported() {
    for (thin, field) in [
        (r#"{"thickness":0}"#, "thin.thickness"),
        (r#"{"thickness":-2}"#, "thin.thickness"),
        (r#"{"thickness":2,"side":"sideways"}"#, "thin.side"),
        (r#"{"thickness":2,"ends":"pointy"}"#, "thin.ends"),
        (r#"{"thickness":2,"cornerStyle":"bevel"}"#, "thin.cornerStyle"),
    ] {
        let o = run(&part(&half_circle(), &extrude(thin, H)));
        assert_eq!(o.errors.len(), 1, "{thin}: {:?}", o.errors);
        let e = &o.errors[0].error;
        assert_eq!(e.code, "invalidValue", "{thin}: {}", e.describe());
        assert_eq!(param(e, "field").and_then(|j| j.as_str().map(str::to_string)).as_deref(), Some(field));
    }
    // an empty sketch has nothing to thicken
    let o = run(&part(&sketch(&[], ""), &extrude(r#"{"thickness":2}"#, H)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].error.code, "noClosedProfile");
    // thin off: the open arc is the usual dead end
    let o = run(&part(&half_circle(), r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5,"thin":{"enabled":false,"thickness":2}}"#));
    assert_eq!(o.errors[0].error.code, "noClosedProfile");
}

#[test]
fn thin_cut_and_expressions() {
    // 40 x 40 x 10 block, a half-ring slot cut through
    let blk = r#"{"type":"sketch","id":"F0","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":-20,"y":-20},{"id":"b","x":20,"y":-20},{"id":"c","x":20,"y":20},{"id":"d","x":-20,"y":20}],
      "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
      {"type":"extrude","id":"F00","op":"add","sketch":"F0","end":"blind","depth":10}"#;
    let src = format!(
        r#"{{"format":"ck-part","version":1,"variables":[{{"name":"wall","expr":"1+1"}}],"features":[{blk},{},{{"type":"extrude","id":"F2","op":"cut","sketch":"F1","end":"mid","depth":40,"thin":{{"thickness":"=wall","side":"mid"}}}}]}}"#,
        half_circle()
    );
    let o = run(&src);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let expect = 40.0 * 40.0 * 10.0 - PI / 2.0 * (11.0 * 11.0 - 9.0 * 9.0) * 10.0;
    assert!(rel(vol(&o), expect) < 1e-6, "{} vs {expect}", vol(&o));
    check_valid(&o, "thin cut");
}

#[test]
fn thin_revolve_tube_and_partial() {
    let sk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":10,"y":0},{"id":"b","x":10,"y":20},{"id":"o","x":0,"y":0},{"id":"q","x":0,"y":20}],
      "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"ax","p1":"o","p2":"q","construction":true}],"constraints":[]}"#;
    let rev = |angle: f64, thin: &str| format!(r#"{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":{angle},"thin":{thin}}}"#);
    let o = run(&part(sk, &rev(360.0, r#"{"thickness":2,"side":"mid"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), PI * (121.0 - 81.0) * 20.0) < 1e-6, "{}", vol(&o));
    check_valid(&o, "tube");
    let o = run(&part(sk, &rev(90.0, r#"{"thickness":2,"side":"outside"}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), PI / 4.0 * (144.0 - 100.0) * 20.0) < 1e-6, "{}", vol(&o));
    // a wall that would cross the axis
    let o = run(&part(sk, &rev(360.0, r#"{"thickness":30,"side":"mid"}"#)));
    assert_eq!(o.errors.len(), 1);
    assert_eq!(o.errors[0].error.code, "profileCrossesAxis");
}

#[test]
fn region_selection_picks_chains() {
    // an open arc and a closed circle in one sketch; `regions` chooses
    let sk = sketch(
        &[("c", 0.0, 0.0), ("p1", 10.0, 0.0), ("p2", -10.0, 0.0), ("m", 40.0, 0.0)],
        r#"{"type":"arc","id":"e1","c":"c","p1":"p1","p2":"p2"},{"type":"circle","id":"e2","c":"m","r":5}"#,
    );
    let one = |regions: &str| {
        run(&part(
            &sk,
            &format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5,"regions":{regions},"thin":{{"thickness":1,"side":"inside"}}}}"#),
        ))
    };
    let o = one(r#"["e1"]"#);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(rel(vol(&o), PI / 2.0 * (100.0 - 81.0) * 5.0) < 1e-6);
    let o = one(r#"["e2"]"#);
    assert!(rel(vol(&o), PI * (25.0 - 16.0) * 5.0) < 1e-6);
    let o = one(r#"["e1","e2"]"#);
    assert!(rel(vol(&o), PI / 2.0 * (100.0 - 81.0) * 5.0 + PI * 9.0 * 5.0) < 1e-6);
    let o = one(r#"["nope"]"#);
    assert_eq!(o.errors[0].error.code, "extrudeRegionUnknown");
    // no `regions`: both
    let o = run(&part(&sk, &extrude(r#"{"thickness":1,"side":"inside"}"#, 5.0)));
    assert!(rel(vol(&o), PI / 2.0 * 19.0 * 5.0 + PI * 9.0 * 5.0) < 1e-6, "{}", vol(&o));
}
