//! `projectCurve`: orthogonal projection of a sketch / planar curve onto faces. The cases mirror the corpus of
//! tools/project_curve_corpus.mjs (run against the oracle kernel there): result kinds, error codes and their order,
//! the near / far sheet of curved faces joined at a silhouette, trimmed faces, chain joining, and that the output
//! curve lies on the surface.
use ck_core::linalg::Vec3;
use ck_json::Json;
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

struct Out {
    r: regen::RegenResult,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    Out { r: regen::regen(&part, &RegenOpts::default(), &mut ex) }
}

impl Out {
    fn code(&self) -> String {
        assert_eq!(self.r.errors.len(), 1, "one error expected: {:?}", self.r.errors);
        self.r.errors[0].error.code.clone()
    }
    fn err_param(&self, k: &str) -> Json {
        self.r.errors[0].error.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or(Json::Null)
    }
    fn ok(&self) -> &Self {
        assert!(self.r.errors.is_empty(), "no error expected: {:?}", self.r.errors);
        self
    }
    fn pts(&self, id: &str) -> Vec<Vec3> {
        let c = self.r.curves.iter().find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).expect("curve");
        c.as_arr().unwrap()[1].get("pts").unwrap().as_arr().unwrap().iter().map(|p| {
            let a = p.as_arr().unwrap();
            Vec3::new(a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap())
        }).collect()
    }
    fn marker(&self, id: &str) -> Json {
        self.r.markers.iter().find(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).expect("marker").as_arr().unwrap()[1].clone()
    }
    fn num(&self, id: &str, k: &str) -> f64 {
        self.marker(id).get(k).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("marker field {k}"))
    }
    fn flag(&self, id: &str, k: &str) -> bool {
        self.marker(id).get(k).and_then(|v| v.as_bool()).unwrap()
    }
}

// ---- documents -----------------------------------------------------------------------------------------

fn point(id: &str, x: f64, y: f64) -> String {
    format!(r#"{{"id":"{id}","x":{x},"y":{y}}}"#)
}
fn line(id: &str, a: &str, b: &str) -> String {
    format!(r#"{{"type":"line","id":"{id}","p1":"{a}","p2":"{b}"}}"#)
}
fn sketch(id: &str, plane: &str, points: &[String], entities: &[String]) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), entities.join(","))
}
const FRONT: &str = r#"{"kind":"std","id":"front"}"#;
const TOP: &str = r#"{"kind":"std","id":"top"}"#;

/// 40 x 20 x 10 box on the front plane (top face F2.C1 at z = 10, bottom F2.C0 at z = 0).
fn box_features() -> String {
    let p = [point("p1", 0.0, 0.0), point("p2", 40.0, 0.0), point("p3", 40.0, 20.0), point("p4", 0.0, 20.0)];
    let e = [line("e1", "p1", "p2"), line("e2", "p2", "p3"), line("e3", "p3", "p4"), line("e4", "p4", "p1")];
    format!(
        r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#,
        sketch("F1", FRONT, &p, &e)
    )
}
/// Cylinder r = 10 about the axis (x = 5, z = -3), y from 0 to 10; face F2.S.e7.
fn cyl_features() -> String {
    format!(
        r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#,
        sketch("F1", TOP, &[point("c", 5.0, 3.0)], &[r#"{"type":"circle","id":"e7","c":"c","r":10}"#.to_string()])
    )
}
/// Sphere r = 10 about the origin; face F2.S.e1.
fn sphere_features() -> String {
    let p = [point("c", 0.0, 0.0), point("s", 0.0, -10.0), point("n", 0.0, 10.0), point("a1", 0.0, 0.0), point("a2", 0.0, 10.0)];
    let e = [
        r#"{"type":"arc","id":"e1","c":"c","p1":"s","p2":"n"}"#.to_string(),
        line("e2", "n", "s"),
        r#"{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}"#.to_string(),
    ];
    format!(r#"{},{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}}"#, sketch("F1", FRONT, &p, &e))
}
/// Cone frustum: r = 15 at y = 0 to r = 5 at y = 10 about the y axis; lateral face F2.S.e2.
fn cone_features() -> String {
    let p = [point("p1", 0.0, 0.0), point("p2", 15.0, 0.0), point("p3", 5.0, 10.0), point("p4", 0.0, 10.0), point("a1", 0.0, 0.0), point("a2", 0.0, 10.0)];
    let e = [
        line("e1", "p1", "p2"),
        line("e2", "p2", "p3"),
        line("e3", "p3", "p4"),
        line("e4", "p4", "p1"),
        r#"{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}"#.to_string(),
    ];
    format!(r#"{},{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}}"#, sketch("F1", FRONT, &p, &e))
}
/// Torus: tube r = 5 on the circle r = 20 about the y axis; face F2.S.e1.
fn torus_features() -> String {
    let p = [point("c", 20.0, 0.0), point("a1", 0.0, 0.0), point("a2", 0.0, 10.0)];
    let e = [r#"{"type":"circle","id":"e1","c":"c","r":5}"#.to_string(), r#"{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}"#.to_string()];
    format!(r#"{},{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}}"#, sketch("F1", FRONT, &p, &e))
}

/// The source sketch F10 on an offset copy (plane F9) of `base` at distance `z`.
fn source(base: &str, z: f64, points: &[String], entities: &[String]) -> String {
    format!(
        r#"{{"type":"plane","id":"F9","mode":"offset","base":{{"kind":"std","id":"{base}"}},"offset":{z}}},{}"#,
        sketch("F10", r#"{"kind":"feature","id":"F9"}"#, points, entities)
    )
}
/// One line from (x1, y1) to (x2, y2).
fn src_line(z: f64, a: (f64, f64), b: (f64, f64)) -> String {
    source("front", z, &[point("s1", a.0, a.1), point("s2", b.0, b.1)], &[line("e1", "s1", "s2")])
}
/// Closed polyline through the points.
fn src_poly(z: f64, pts: &[(f64, f64)], closed: bool) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| point(&format!("s{i}"), q.0, q.1)).collect();
    let n = pts.len();
    let segs = if closed { n } else { n - 1 };
    let e: Vec<String> = (0..segs).map(|i| line(&format!("e{i}"), &format!("s{i}"), &format!("s{}", (i + 1) % n))).collect();
    source("front", z, &p, &e)
}
fn src_circle(z: f64, c: (f64, f64), r: f64) -> String {
    source("front", z, &[point("s1", c.0, c.1)], &[format!(r#"{{"type":"circle","id":"e1","c":"s1","r":{r}}}"#)])
}

fn doc(features: &[&str]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}
fn pc(extra: &str) -> String {
    format!(r#"{{"type":"projectCurve","id":"F20","sketch":"F10"{extra}}}"#)
}

fn len(p: &[Vec3]) -> f64 {
    p.windows(2).map(|w| w[0].dist(w[1])).sum()
}

// ---- planar faces -----------------------------------------------------------------------------------------

#[test]
fn line_on_the_top_face() {
    let o = run(&doc(&[&box_features(), &src_line(20.0, (17.0, 8.0), (23.0, 12.0)), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    o.ok();
    assert!((o.num("F20", "length") - 52f64.sqrt()).abs() < 1e-9);
    assert_eq!(o.num("F20", "count"), 4.0);
    assert_eq!(o.num("F20", "pieces"), 1.0);
    assert_eq!(o.num("F20", "degree"), 3.0);
    assert!(!o.flag("F20", "closed"));
    let p = o.pts("F20");
    assert!(p.iter().all(|q| (q.z - 10.0).abs() < 1e-9), "on the top face z = 10");
    assert!(p[0].dist(Vec3::new(17.0, 8.0, 10.0)) < 1e-9 && p.last().unwrap().dist(Vec3::new(23.0, 12.0, 10.0)) < 1e-9);
}

#[test]
fn direction_forward_reverse_both() {
    let feats = |d: &str, z: f64| doc(&[&box_features(), &src_line(z, (17.0, 8.0), (23.0, 12.0)), &pc(&format!(r#","faces":["F2.C1"],"direction":"{d}""#))]);
    // sketch plane in front of the face (z = 20): only the reverse side reaches it
    assert_eq!(run(&feats("forward", 20.0)).code(), "projectCurveMisses");
    run(&feats("reverse", 20.0)).ok();
    run(&feats("both", 20.0)).ok();
    // behind the face (z = -5) it is the other way round
    run(&feats("forward", -5.0)).ok();
    assert_eq!(run(&feats("reverse", -5.0)).code(), "projectCurveMisses");
    // no direction = both
    run(&doc(&[&box_features(), &src_line(-5.0, (17.0, 8.0), (23.0, 12.0)), &pc(r#","faces":["F2.C1"]"#)])).ok();
    // the sketch plane through the face itself
    run(&doc(&[&box_features(), &src_line(10.0, (17.0, 8.0), (23.0, 12.0)), &pc(r#","faces":["F2.C1"]"#)])).ok();
}

#[test]
fn closed_polyline_has_corners_and_closes() {
    let o = run(&doc(&[&box_features(), &src_poly(20.0, &[(17.0, 8.0), (23.0, 8.0), (23.0, 12.0), (17.0, 12.0)], true), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    o.ok();
    assert!((o.num("F20", "length") - 20.0).abs() < 1e-9);
    assert_eq!(o.num("F20", "pieces"), 4.0);
    assert_eq!(o.num("F20", "corners"), 3.0);
    assert_eq!(o.num("F20", "count"), 13.0, "four cubic pieces share their ends");
    assert!(o.flag("F20", "closed"));
    assert!(o.num("F20", "deviation") < 1e-6);
    // the display polyline reaches the corners (a knot of full multiplicity sits on each)
    let p = o.pts("F20");
    for c in [(17.0, 8.0), (23.0, 8.0), (23.0, 12.0), (17.0, 12.0)] {
        assert!(p.iter().any(|q| q.dist(Vec3::new(c.0, c.1, 10.0)) < 1e-6), "corner {c:?} sampled");
    }
    assert!((len(&p) - 20.0).abs() < 1e-6);
}

#[test]
fn gaps_between_entities_join_below_a_millimetre_thousandth() {
    let two = |g: f64| {
        source("front", 20.0, &[point("a", 17.0, 10.0), point("b", 20.0, 10.0), point("c", 20.0 + g, 10.0), point("d", 23.0 + g, 10.0)], &[line("e1", "a", "b"), line("e2", "c", "d")])
    };
    let go = |g: f64| run(&doc(&[&box_features(), &two(g), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    for g in [0.0, 1e-5, 9e-4] {
        let o = go(g);
        o.ok();
        assert_eq!(o.num("F20", "pieces"), 2.0, "gap {g}");
    }
    for g in [1.1e-3, 0.1, 5.0] {
        let o = go(g);
        assert_eq!(o.code(), "projectCurveSeveral", "gap {g}");
        assert_eq!(o.err_param("count").as_f64(), Some(2.0));
        assert_eq!(o.err_param("points").as_arr().map(|a| a.len()), Some(4), "both ends of both chains");
    }
}

#[test]
fn clipped_by_the_face_boundary_and_by_a_hole() {
    // line from x = 30 to 50 over the top face that ends at x = 40: the part on the face remains
    let o = run(&doc(&[&box_features(), &src_line(20.0, (30.0, 10.0), (50.0, 10.0)), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    o.ok();
    assert!((o.num("F20", "length") - 10.0).abs() < 1e-4);
    let p = o.pts("F20");
    assert!(p.iter().all(|q| q.x <= 40.0 + 1e-4));
    assert!(p.iter().any(|q| (q.x - 40.0).abs() < 1e-4), "ends on the boundary");

    // circle across the boundary (start point on the face, the arc wraps around the parameter seam): one run
    let o = run(&doc(&[&box_features(), &src_circle(20.0, (-1.5, 10.0), 3.0), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    o.ok();
    assert_eq!(o.num("F20", "pieces"), 1.0);
    assert!(!o.flag("F20", "closed"));
    // the part of the circle with x >= 0 is 120 degrees of it
    assert!((o.num("F20", "length") - 2.0 * std::f64::consts::PI).abs() < 1e-3, "{}", o.num("F20", "length"));

    // a hole through the face
    let hole = format!(
        r#"{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth"}}"#,
        sketch("F3", FRONT, &[point("hc", 20.0, 10.0)], &[r#"{"type":"circle","id":"e1","c":"hc","r":3}"#.to_string()])
    );
    let across = run(&doc(&[&box_features(), &hole, &src_line(20.0, (12.0, 10.0), (28.0, 10.0)), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    assert_eq!(across.code(), "projectCurveSeveral");
    assert_eq!(across.err_param("count").as_f64(), Some(2.0));
    let into = run(&doc(&[&box_features(), &hole, &src_line(20.0, (12.0, 10.0), (20.0, 10.0)), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    into.ok();
    assert!((into.num("F20", "length") - 5.0).abs() < 1e-3);
    let inside = run(&doc(&[&box_features(), &hole, &src_circle(20.0, (20.0, 10.0), 1.0), &pc(r#","faces":["F2.C1"],"direction":"reverse""#)]));
    assert_eq!(inside.code(), "projectCurveMisses");
}

#[test]
fn several_faces_several_chains() {
    let feats = |faces: &str| doc(&[&box_features(), &src_line(20.0, (17.0, 8.0), (23.0, 12.0)), &pc(&format!(r#","faces":{faces},"direction":"reverse""#))]);
    // top and bottom both receive the whole line
    for f in [r#"["F2.C0","F2.C1"]"#, "[]"] {
        let o = run(&feats(f));
        assert_eq!(o.code(), "projectCurveSeveral", "{f}");
        assert_eq!(o.err_param("count").as_f64(), Some(2.0));
    }
    // faces missing = every face; a side face seen edge-on takes nothing
    assert_eq!(run(&doc(&[&box_features(), &src_line(20.0, (17.0, 8.0), (23.0, 12.0)), &pc(r#","direction":"reverse""#)])).code(), "projectCurveSeveral");
    assert_eq!(run(&feats(r#"["F2.S.e1"]"#)).code(), "projectCurveMisses");
    // two separate source curves
    let sep = source("front", 20.0, &[point("a", 17.0, 10.0), point("b", 23.0, 10.0), point("c", 17.0, 14.0), point("d", 23.0, 14.0)], &[line("e1", "a", "b"), line("e2", "c", "d")]);
    assert_eq!(run(&doc(&[&box_features(), &sep, &pc(r#","faces":["F2.C1"],"direction":"reverse""#)])).code(), "projectCurveSeveral");
}

// ---- curved faces ------------------------------------------------------------------------------------------

/// Distance of a point from the axis (x = 5, z = -3) parallel to y.
fn cyl_r(q: &Vec3) -> f64 {
    ((q.x - 5.0).powi(2) + (q.z + 3.0).powi(2)).sqrt()
}

#[test]
fn cylinder_near_and_far_sheet() {
    let line_src = src_line(20.0, (2.0, 3.0), (8.0, 7.0));
    // both sheets of the cylinder are hit and do not meet: two chains
    let o = run(&doc(&[&cyl_features(), &line_src, &pc(r#","faces":["F2.S.e7"],"direction":"reverse""#)]));
    assert_eq!(o.code(), "projectCurveSeveral");
    assert_eq!(o.err_param("count").as_f64(), Some(2.0));
    // sketch plane through the axis, one direction: the near sheet only
    let o = run(&doc(&[&cyl_features(), &src_line(-3.0, (2.0, 3.0), (8.0, 7.0)), &pc(r#","faces":["F2.S.e7"],"direction":"forward""#)]));
    o.ok();
    let p = o.pts("F20");
    assert!(p.iter().all(|q| (cyl_r(q) - 10.0).abs() < 2e-3 && q.z > -3.0), "on the near half of the cylinder");
    let exact = (0..400).map(|i| 2.0 + 6.0 * i as f64 / 400.0).map(|x| (x, 3.0 + (x - 2.0) * 4.0 / 6.0)).collect::<Vec<_>>();
    let l: f64 = exact.windows(2).map(|w| {
        let z = |x: f64| -3.0 + (100.0 - (x - 5.0).powi(2)).sqrt();
        Vec3::new(w[0].0, w[0].1, z(w[0].0)).dist(Vec3::new(w[1].0, w[1].1, z(w[1].0)))
    }).sum();
    assert!((o.num("F20", "length") - l).abs() < 5e-3 * l, "{} vs {l}", o.num("F20", "length"));
    // a closed curve on one sheet stays closed
    let o = run(&doc(&[&cyl_features(), &src_circle(-3.0, (5.0, 5.0), 3.0), &pc(r#","faces":["F2.S.e7"],"direction":"forward""#)]));
    o.ok();
    assert!(o.flag("F20", "closed"));
    assert!(o.pts("F20").iter().all(|q| (cyl_r(q) - 10.0).abs() < 2e-3));
}

#[test]
fn sphere_circle_and_tangent_rays() {
    // circle r = 9 about the centre axis: a closed curve on the +z cap
    let o = run(&doc(&[&sphere_features(), &src_circle(0.0, (0.0, 0.0), 9.0), &pc(r#","faces":["F2.S.e1"],"direction":"forward""#)]));
    o.ok();
    assert!(o.flag("F20", "closed"));
    assert!((o.num("F20", "length") - 2.0 * std::f64::consts::PI * 9.0).abs() < 1e-2);
    let p = o.pts("F20");
    assert!(p.iter().all(|q| (q.len() - 10.0).abs() < 4e-3 && q.z > 0.0));
    // both sheets: two closed loops
    assert_eq!(run(&doc(&[&sphere_features(), &src_circle(0.0, (0.0, 0.0), 9.0), &pc(r#","faces":["F2.S.e1"],"direction":"both""#)])).code(), "projectCurveSeveral");
    // r = 10 touches the sphere along the equator: a grazing ray is no hit; r = 12 misses
    assert_eq!(run(&doc(&[&sphere_features(), &src_circle(0.0, (0.0, 0.0), 10.0), &pc(r#","faces":["F2.S.e1"],"direction":"forward""#)])).code(), "projectCurveMisses");
    assert_eq!(run(&doc(&[&sphere_features(), &src_circle(0.0, (0.0, 0.0), 12.0), &pc(r#","faces":["F2.S.e1"],"direction":"forward""#)])).code(), "projectCurveMisses");
}

#[test]
fn cone_sheets_join_at_the_silhouette() {
    // the line ends on the silhouette (8, 7): the far run and the near run are one curve
    let o = run(&doc(&[&cone_features(), &src_line(20.0, (2.0, 3.0), (8.0, 7.0)), &pc(r#","faces":["F2.S.e2"],"direction":"reverse""#)]));
    o.ok();
    assert_eq!(o.num("F20", "pieces"), 2.0);
    assert_eq!(o.num("F20", "corners"), 0.0, "smooth through the silhouette");
    assert!((o.num("F20", "length") - 28.8165).abs() < 5e-3, "{}", o.num("F20", "length"));
    assert!(o.num("F20", "deviation") <= 1e-3);
    let p = o.pts("F20");
    assert!(p.iter().all(|q| ((q.x * q.x + q.z * q.z).sqrt() - (15.0 - q.y)).abs() < 3e-3), "on the cone");
    let ends = (p[0], *p.last().unwrap());
    assert!(ends.0.z * ends.1.z < 0.0 && (ends.0.x - 2.0).abs() < 1e-6 && (ends.1.x - 2.0).abs() < 1e-6, "from the far to the near side");
    // a line that does not reach the silhouette: two chains
    let o = run(&doc(&[&cone_features(), &src_line(20.0, (2.0, 3.0), (6.0, 5.0)), &pc(r#","faces":["F2.S.e2"],"direction":"reverse""#)]));
    assert_eq!(o.code(), "projectCurveSeveral");
    // the closed rectangle whose corner sits on the silhouette: one closed chain over eight runs
    let o = run(&doc(&[&cone_features(), &src_poly(20.0, &[(2.0, 3.0), (8.0, 3.0), (8.0, 7.0), (2.0, 7.0)], true), &pc(r#","faces":["F2.S.e2"],"direction":"reverse""#)]));
    o.ok();
    assert_eq!(o.num("F20", "pieces"), 8.0);
    assert!(o.flag("F20", "closed"));
}

#[test]
fn torus_over_the_hole_three_chains() {
    let o = run(&doc(&[&torus_features(), &src_line(20.0, (-8.0, 0.0), (8.0, 0.0)), &pc(r#","faces":["F2.S.e1"],"direction":"reverse""#)]));
    assert_eq!(o.code(), "projectCurveSeveral");
    assert_eq!(o.err_param("count").as_f64(), Some(3.0));
    // sketch plane through the middle, one direction: a single chain on the tube
    let o = run(&doc(&[&torus_features(), &src_line(0.0, (17.0, -2.0), (23.0, 2.0)), &pc(r#","faces":["F2.S.e1"],"direction":"forward""#)]));
    o.ok();
    assert!(o.pts("F20").iter().all(|q| ((((q.x * q.x + q.z * q.z).sqrt() - 20.0).powi(2) + q.y * q.y).sqrt() - 5.0).abs() < 3e-3));
}

// ---- fit options -------------------------------------------------------------------------------------------

fn sphere_spline(opts: &str) -> Out {
    let p = [point("a", -3.0, -2.0), point("b", -1.0, 2.0), point("c", 1.0, -2.0), point("d", 3.0, 2.0)];
    let src = source("front", 0.0, &p, &[r#"{"type":"spline","id":"e1","pts":["a","b","c","d"]}"#.to_string()]);
    let dir = if opts.contains("direction") { "" } else { r#","direction":"forward""# };
    run(&doc(&[&sphere_features(), &src, &pc(&format!(r#","faces":["F2.S.e1"]{dir}{opts}"#))]))
}

#[test]
fn degree_and_tolerance() {
    let base = sphere_spline("");
    base.ok();
    assert_eq!(base.num("F20", "degree"), 3.0);
    assert!(base.num("F20", "deviation") <= 1e-3);
    for d in [2, 5, 9] {
        let o = sphere_spline(&format!(r#","degree":{d}"#));
        o.ok();
        assert_eq!(o.num("F20", "degree"), d as f64);
    }
    assert_eq!(sphere_spline(r#","degree":4.4"#).num("F20", "degree"), 4.0, "rounded");
    assert_eq!(sphere_spline(r#","degree":"abc""#).num("F20", "degree"), 3.0, "text that is no number: the default");
    assert_eq!(sphere_spline(r#","degree":"=2+2""#).num("F20", "degree"), 4.0);
    assert_eq!(sphere_spline(r#","degree":null,"tolerance":null"#).num("F20", "degree"), 3.0);
    // a looser tolerance needs fewer control points, a tighter one more
    let loose = sphere_spline(r#","tolerance":0.05"#);
    let tight = sphere_spline(r#","tolerance":0.0003"#);
    assert!(loose.num("F20", "count") < base.num("F20", "count") && base.num("F20", "count") < tight.num("F20", "count"));
    assert!(tight.num("F20", "deviation") <= 3e-4 + 1e-9, "{}", tight.num("F20", "deviation"));
    // never more than 400 control points, however tight
    let o = sphere_spline(r#","tolerance":1e-9"#);
    o.ok();
    assert!(o.num("F20", "count") <= 400.0);
    for (opt, code) in [(r#","degree":1"#, "editCurveDegree"), (r#","degree":10"#, "editCurveDegree"), (r#","degree":0"#, "editCurveDegree"), (r#","tolerance":0"#, "invalidValue"), (r#","tolerance":-1"#, "invalidValue"), (r#","direction":"sideways""#, "invalidValue"), (r#","degree":"=0/0""#, "badExpression")] {
        assert_eq!(sphere_spline(opt).code(), code, "{opt}");
    }
    let o = sphere_spline(r#","degree":1"#);
    assert_eq!(o.err_param("min").as_f64(), Some(2.0));
    assert_eq!(o.err_param("max").as_f64(), Some(9.0));
}

// ---- errors and their order --------------------------------------------------------------------------------

#[test]
fn error_codes_and_order() {
    let bx = box_features();
    let f = r#","faces":["F2.C1"],"direction":"reverse""#;
    // nothing to project
    assert_eq!(run(&doc(&[&bx, r#"{"type":"projectCurve","id":"F20"}"#])).code(), "projectCurveNoInput");
    assert_eq!(run(&doc(&[&bx, r#"{"type":"projectCurve","id":"F20","faces":["F2.C1"]}"#])).code(), "projectCurveNoInput");
    // no body at all comes first, whatever else is wrong
    assert_eq!(run(&doc(&[r#"{"type":"projectCurve","id":"F20"}"#])).code(), "projectCurveNoFaces");
    assert_eq!(run(&doc(&[r#"{"type":"projectCurve","id":"F20","sketch":"F99"}"#])).code(), "projectCurveNoFaces");
    assert_eq!(run(&doc(&[&src_line(5.0, (0.0, 0.0), (5.0, 5.0)), &pc(f)])).code(), "projectCurveNoFaces");
    let _ = &bx;
}

#[test]
fn error_codes_references() {
    let bx = box_features();
    let ok_src = src_line(20.0, (17.0, 8.0), (23.0, 12.0));
    let f = r#","faces":["F2.C1"],"direction":"reverse""#;
    let with = |sk: &str, extra: &str| doc(&[&bx, &ok_src, &format!(r#"{{"type":"projectCurve","id":"F20","sketch":"{sk}"{extra}}}"#)]);
    assert_eq!(run(&with("F99", f)).code(), "missingSketch");
    assert_eq!(run(&with("F2", f)).code(), "missingSketch", "an extrude is no sketch");
    assert_eq!(run(&with("F9", f)).code(), "missingSketch", "a plane is no sketch");
    let curve = |id: &str| doc(&[&bx, &ok_src, &format!(r#"{{"type":"projectCurve","id":"F20","curve":"{id}"{f}}}"#)]);
    assert_eq!(run(&curve("F99")).code(), "missingReference");
    assert_eq!(run(&curve("F10")).code(), "missingReference", "a sketch is no curve feature");
    assert_eq!(run(&curve("F2")).code(), "missingReference");
    // curve wins when both are given
    assert_eq!(run(&doc(&[&bx, &ok_src, &format!(r#"{{"type":"projectCurve","id":"F20","sketch":"F10","curve":"F99"{f}}}"#)])).code(), "missingReference");
    // unknown face
    assert_eq!(run(&with("F10", r#","faces":["F77.C1"]"#)).code(), "missingReference");
    // degree is checked before the face names and before the geometry
    assert_eq!(run(&with("F10", r#","faces":["F77.C1"],"degree":1"#)).code(), "editCurveDegree");
    let miss = doc(&[&bx, &src_line(20.0, (100.0, 10.0), (106.0, 14.0)), &pc(f)]);
    assert_eq!(run(&miss).code(), "projectCurveMisses");
    // sketches without anything projectable
    let empty = doc(&[&bx, &sketch("F10", FRONT, &[], &[]), &pc(f)]);
    assert_eq!(run(&empty).code(), "projectCurveNoInput");
    let cons = doc(&[&bx, &sketch("F10", FRONT, &[point("a", 0.0, 0.0), point("b", 5.0, 5.0)], &[r#"{"type":"line","id":"e1","p1":"a","p2":"b","construction":true}"#.to_string()]), &pc(f)]);
    assert_eq!(run(&cons).code(), "projectCurveNoInput");
    let pt = doc(&[&bx, &sketch("F10", FRONT, &[point("a", 20.0, 10.0)], &[r#"{"type":"point","id":"e1","p":"a"}"#.to_string()]), &pc(f)]);
    assert_eq!(run(&pt).code(), "projectCurveNoInput");
    // construction geometry beside real geometry is ignored
    let mixed = doc(&[&bx, &sketch("F10", FRONT, &[point("a", 17.0, 10.0), point("b", 23.0, 10.0), point("c", 20.0, 6.0), point("d", 20.0, 14.0)], &[line("e1", "a", "b"), r#"{"type":"line","id":"e2","p1":"c","p2":"d","construction":true}"#.to_string()]), &pc(r#","faces":["F2.C1"]"#)]);
    let o = run(&mixed);
    o.ok();
    assert_eq!(o.num("F20", "pieces"), 1.0);
}

// ---- curve inputs ------------------------------------------------------------------------------------------

#[test]
fn curve_inputs() {
    let bx = box_features();
    let wavy = source(
        "front",
        20.0,
        &[point("a", 14.0, 8.0), point("b", 18.0, 12.0), point("c", 22.0, 8.0), point("d", 26.0, 12.0)],
        &[r#"{"type":"spline","id":"e1","pts":["a","b","c","d"]}"#.to_string()],
    );
    let first = pc(r#","faces":["F2.C1"],"direction":"reverse""#);
    let second = |dir: &str| format!(r#"{{"type":"projectCurve","id":"F21","curve":"F20","faces":["F2.C0"],"direction":"{dir}"}}"#);
    // the curve lies in z = 10; its plane normal is +z (largest component positive, whatever the traversal): the
    // bottom face at z = 0 is reached against it
    let o = run(&doc(&[&bx, &wavy, &first, &second("reverse")]));
    o.ok();
    assert!(o.pts("F21").iter().all(|q| q.z.abs() < 1e-9));
    assert_eq!(run(&doc(&[&bx, &wavy, &first, &second("forward")])).code(), "projectCurveMisses");
    run(&doc(&[&bx, &wavy, &first, &second("both")])).ok();
    // a straight curve has no plane
    let straight = src_line(20.0, (17.0, 8.0), (23.0, 12.0));
    assert_eq!(run(&doc(&[&bx, &straight, &first, &second("both")])).code(), "projectCurvePlane");
    // a helix is not planar
    let helix = format!(
        r#"{},{{"type":"helix","id":"F11","sketch":"F10","definedBy":"pitchRevs","pitch":5,"revolutions":3}}"#,
        sketch("F10", FRONT, &[point("c", 0.0, 0.0)], &[r#"{"type":"circle","id":"e1","c":"c","r":4}"#.to_string()])
    );
    let o = run(&doc(&[&helix, &box_features().replace("F1", "F31").replace("F2", "F32"), r#"{"type":"projectCurve","id":"F40","curve":"F11","faces":["F32.C1"]}"#]));
    assert_eq!(o.code(), "projectCurvePlane");
}

#[test]
fn projected_curve_feeds_later_features() {
    let bx = box_features();
    let wavy = source(
        "front",
        20.0,
        &[point("a", 14.0, 8.0), point("b", 18.0, 12.0), point("c", 22.0, 8.0), point("d", 26.0, 12.0)],
        &[r#"{"type":"spline","id":"e1","pts":["a","b","c","d"]}"#.to_string()],
    );
    let first = pc(r#","faces":["F2.C1"],"direction":"reverse""#);
    let cref = r#"{"kind":"curve","id":"F20"}"#;
    // editCurve rebuilds it
    let ec = format!(r#"{{"type":"editCurve","id":"F21","curves":[{cref}],"method":"count","degree":3,"count":8}}"#);
    let o = run(&doc(&[&bx, &wavy, &first, &ec]));
    o.ok();
    assert_eq!(o.num("F21", "count"), 8.0);
    // a plane through the curve (mode curve): the plane z = 10
    let pl = format!(r#"{{"type":"plane","id":"F32","mode":"curve","curves":[{cref}]}}"#);
    let o = run(&doc(&[&bx, &wavy, &first, &pl]));
    o.ok();
    let (_, fr) = o.r.planes.iter().find(|p| p.0 == "F32").expect("plane F32");
    assert!(fr.z.x.abs() < 1e-9 && fr.z.y.abs() < 1e-9 && fr.z.z.abs() > 1.0 - 1e-9 && (fr.o.z - 10.0).abs() < 1e-6, "{fr:?}");
    // sweep along it: found as a path (the profile must stand normal to the path start: the oracle says the same)
    let prof = sketch("F30", r#"{"kind":"std","id":"right"}"#, &[point("c", 0.0, 0.0)], &[r#"{"type":"circle","id":"e1","c":"c","r":1}"#.to_string()]);
    let sw = format!(r#"{{"type":"sweep","id":"F31","op":"add","sketch":"F30","path":{cref}}}"#);
    assert_eq!(run(&doc(&[&bx, &wavy, &first, &prof, &sw])).code(), "sweepProfileNotNormal");
}

// ---- surfaces, determinism ---------------------------------------------------------------------------------

#[test]
fn surface_body_faces() {
    // a planar surface body has no solid: faces = all finds it, a named face works too
    let p = [point("p1", 0.0, 0.0), point("p2", 40.0, 0.0), point("p3", 40.0, 20.0), point("p4", 0.0, 20.0)];
    let e = [line("e1", "p1", "p2"), line("e2", "p2", "p3"), line("e3", "p3", "p4"), line("e4", "p4", "p1")];
    let surf = format!(r#"{},{{"type":"surfacePlanar","id":"F2","sketch":"F1"}}"#, sketch("F1", TOP, &p, &e));
    let src = source("top", 5.0, &[point("s1", 5.0, 5.0), point("s2", 15.0, 9.0)], &[line("e1", "s1", "s2")]);
    for faces in [r#","faces":[]"#, r#","faces":["F2.P"]"#] {
        let o = run(&doc(&[&surf, &src, &pc(&format!(r#"{faces},"direction":"reverse""#))]));
        o.ok();
        assert!((o.num("F20", "length") - 116f64.sqrt()).abs() < 1e-6);
    }
}

#[test]
fn registered_and_empty_payload_table() {
    assert!(regen::KNOWN_TYPES.contains(&"projectCurve"));
    // the table is "no references, no body yet": the checked-in entry must be what an empty payload gives then
    let want = regen::EMPTY_FEATURE_ERRORS.iter().find(|(t, _)| *t == "projectCurve").expect("listed").1;
    assert_eq!(run(&doc(&[r#"{"type":"projectCurve","id":"F20"}"#])).code(), want);
    // with a body an empty payload is NoInput (oracle): the table cannot tell the two apart
    assert_eq!(run(&doc(&[&box_features(), r#"{"type":"projectCurve","id":"F20"}"#])).code(), "projectCurveNoInput");
}

#[test]
fn document_round_trip() {
    // save / load: the part serialised and parsed again keeps every payload field and regenerates to the same curve
    let d = doc(&[&sphere_features(), &src_circle(0.0, (0.0, 0.0), 9.0), &pc(r#","faces":["F2.S.e1"],"direction":"forward","degree":4,"tolerance":0.002"#)]);
    let part = Part::from_json(&ck_json::parse(&d).unwrap()).unwrap();
    let text = ck_json::to_string(&part.to_json());
    let again = Part::from_json(&ck_json::parse(&text).unwrap()).unwrap();
    let f = again.features.iter().find(|f| f.type_name == "projectCurve").expect("feature kept");
    assert_eq!(f.raw.get_str("sketch"), Some("F10"));
    assert_eq!(f.raw.get_str("direction"), Some("forward"));
    assert_eq!(f.raw.get("degree").and_then(|v| v.as_f64()), Some(4.0));
    assert_eq!(f.raw.get("tolerance").and_then(|v| v.as_f64()), Some(0.002));
    assert_eq!(f.raw.get("faces").and_then(|v| v.as_arr()).map(|a| a.len()), Some(1));
    let (a, b, c) = (run(&d), run(&text), run(&text));
    a.ok();
    assert_eq!(a.pts("F20"), b.pts("F20"));
    assert_eq!(b.pts("F20"), c.pts("F20"));
    assert_eq!(a.marker("F20").to_string(), b.marker("F20").to_string());
    assert_eq!(a.num("F20", "degree"), 4.0);
}

#[test]
fn regen_is_deterministic() {
    let d = doc(&[&cone_features(), &src_line(20.0, (2.0, 3.0), (8.0, 7.0)), &pc(r#","faces":["F2.S.e2"],"direction":"reverse""#)]);
    let (a, b) = (run(&d), run(&d));
    assert_eq!(a.pts("F20"), b.pts("F20"));
    assert_eq!(a.marker("F20").to_string(), b.marker("F20").to_string());
}
