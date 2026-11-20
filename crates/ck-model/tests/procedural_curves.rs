//! The procedural curve features (features/proc_curve.rs): `compositeCurve`, `offsetCurve` and `projectCurve`
//! with `procedural: true`. Success, validation, the refusals with their codes, regen determinism, the document
//! round trip, and the downstream consumers of a curve feature (sweep path, editCurve, plane through a curve, the
//! features themselves chained).
use ck_core::linalg::Vec3;
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

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { r, brep: ex.brep }
}

impl Out {
    fn code(&self) -> String {
        assert_eq!(self.r.errors.len(), 1, "one error expected: {:?}", self.r.errors);
        self.r.errors[0].error.code.clone()
    }
    fn err_param(&self, k: &str) -> Json {
        self.r.errors[0].error.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or(Json::Null)
    }
    fn has_hint(&self) -> bool {
        matches!(self.err_param("hint"), Json::Str(s) if !s.is_empty())
    }
    fn ok(&self) -> &Self {
        assert!(self.r.errors.is_empty(), "no error expected: {:?}", self.r.errors);
        self
    }
    fn curve(&self, id: &str) -> Json {
        self.r.curves.iter().find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).expect("curve").as_arr().unwrap()[1].clone()
    }
    fn pts(&self, id: &str) -> Vec<Vec3> {
        self.curve(id)
            .get("pts")
            .unwrap()
            .as_arr()
            .unwrap()
            .iter()
            .map(|p| {
                let a = p.as_arr().unwrap();
                Vec3::new(a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap())
            })
            .collect()
    }
    fn marker(&self, id: &str) -> Json {
        self.r.markers.iter().find(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).expect("marker").as_arr().unwrap()[1].clone()
    }
    fn num(&self, id: &str, k: &str) -> f64 {
        self.marker(id).get(k).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("marker field {k}"))
    }
    fn s(&self, id: &str, k: &str) -> String {
        self.marker(id).get_str(k).unwrap_or_else(|| panic!("marker field {k}")).to_string()
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.brep.body.as_ref().expect("a body")).volume
    }
}

fn point(id: &str, x: f64, y: f64) -> String {
    format!(r#"{{"id":"{id}","x":{x},"y":{y}}}"#)
}
fn line(id: &str, a: &str, b: &str) -> String {
    format!(r#"{{"type":"line","id":"{id}","p1":"{a}","p2":"{b}"}}"#)
}
fn arc(id: &str, c: &str, a: &str, b: &str) -> String {
    format!(r#"{{"type":"arc","id":"{id}","c":"{c}","p1":"{a}","p2":"{b}"}}"#)
}
fn sketch(id: &str, plane: &str, points: &[String], entities: &[String]) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), entities.join(","))
}
const FRONT: &str = r#"{"kind":"std","id":"front"}"#;
fn doc(features: &[&str]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}

/// 40 x 20 x 10 box on the front plane (top face F2.C1 at z = 10).
fn box_features() -> String {
    let p = [point("p1", 0.0, 0.0), point("p2", 40.0, 0.0), point("p3", 40.0, 20.0), point("p4", 0.0, 20.0)];
    let e = [line("e1", "p1", "p2"), line("e2", "p2", "p3"), line("e3", "p3", "p4"), line("e4", "p4", "p1")];
    format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#, sketch("F1", FRONT, &p, &e))
}
/// Sphere r = 10 about the origin; face F2.S.e1.
fn sphere_features() -> String {
    let p = [point("c", 0.0, 0.0), point("s", 0.0, -10.0), point("n", 0.0, 10.0), point("a1", 0.0, 0.0), point("a2", 0.0, 10.0)];
    let e = [arc("e1", "c", "s", "n"), line("e2", "n", "s"), r#"{"type":"line","id":"ax","p1":"a1","p2":"a2","construction":true}"#.to_string()];
    format!(r#"{},{{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}}"#, sketch("F1", FRONT, &p, &e))
}
/// A sketch on the front plane offset by `z` (plane P9).
fn on_front(id: &str, z: f64, points: &[String], entities: &[String]) -> String {
    format!(r#"{{"type":"plane","id":"P{id}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{z}}},{}"#, sketch(id, &format!(r#"{{"kind":"feature","id":"P{id}"}}"#), points, entities))
}

/// S1 on the front plane: a line (0,-5) -> (20,-5) and a quarter arc about (20,5), radius 10, to (30,5).
fn line_arc() -> String {
    sketch(
        "S1",
        FRONT,
        &[point("a", 0.0, -5.0), point("b", 20.0, -5.0), point("c", 20.0, 5.0), point("d", 30.0, 5.0)],
        &[line("e1", "a", "b"), arc("e2", "c", "b", "d")],
    )
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

// ---- compositeCurve -----------------------------------------------------------------------------------------

#[test]
fn composite_of_a_sketch_chain() {
    let o = run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1","sketch":"S1"}"#]));
    o.ok();
    assert_eq!(o.s("C1", "kind"), "composite");
    assert_eq!(o.num("C1", "pieces"), 2.0);
    assert_eq!(o.num("C1", "corners"), 0.0);
    assert_ne!(o.s("C1", "continuity"), "C0");
    assert!(close(o.num("C1", "length"), 20.0 + 5.0 * PI, 1e-6), "{}", o.num("C1", "length"));
    // lines and arcs convert exactly
    assert_eq!(o.num("C1", "deviation"), 0.0);
    assert_eq!(o.marker("C1").get("closed").and_then(|v| v.as_bool()), Some(false));
    assert_eq!(o.curve("C1").get_str("procedural"), Some("composite"));
    let p = o.pts("C1");
    // walked from the free end at a or d; every point is on the line or the arc
    let ends = [p[0], *p.last().unwrap()];
    assert!(ends.iter().any(|q| q.dist(Vec3::new(0.0, -5.0, 0.0)) < 1e-9) && ends.iter().any(|q| q.dist(Vec3::new(30.0, 5.0, 0.0)) < 1e-9), "{ends:?}");
    for q in &p {
        let on_line = close(q.y, -5.0, 1e-9) && q.x <= 20.0 + 1e-9;
        let on_arc = close(q.dist(Vec3::new(20.0, 5.0, 0.0)), 10.0, 1e-9);
        assert!(on_line || on_arc, "{q:?}");
    }
}

#[test]
fn composite_of_references_turns_the_pieces() {
    // listed arc first, then the line: the arc is turned to end where the line starts
    let refs = r#"[{"kind":"sketchCurve","sketch":"S1","entity":"e2"},{"kind":"sketchCurve","sketch":"S1","entity":"e1"}]"#;
    let o = run(&doc(&[&line_arc(), &format!(r#"{{"type":"compositeCurve","id":"C1","curves":{refs}}}"#)]));
    o.ok();
    let p = o.pts("C1");
    assert!(p[0].dist(Vec3::new(30.0, 5.0, 0.0)) < 1e-9 && p.last().unwrap().dist(Vec3::new(0.0, -5.0, 0.0)) < 1e-9, "{:?} {:?}", p[0], p.last());
    assert!(close(o.num("C1", "length"), 20.0 + 5.0 * PI, 1e-6));
}

#[test]
fn composite_of_body_edges() {
    // two edges of the top face meeting at (40, 0, 10)
    let refs = r#"[{"kind":"edge","faces":["F2.C1","F2.S.e1"],"at":[20,0,10]},{"kind":"edge","faces":["F2.C1","F2.S.e2"],"at":[40,10,10]}]"#;
    let o = run(&doc(&[&box_features(), &format!(r#"{{"type":"compositeCurve","id":"C1","curves":{refs}}}"#)]));
    o.ok();
    assert!(close(o.num("C1", "length"), 60.0, 1e-9));
    assert_eq!(o.num("C1", "corners"), 1.0);
    assert_eq!(o.s("C1", "continuity"), "C0");
}

#[test]
fn composite_refusals() {
    // a gap of 0.01 between two lines: refused with where and how far, accepted once the tolerance covers it
    let gap = sketch("S1", FRONT, &[point("a", 0.0, 0.0), point("b", 10.0, 0.0), point("c", 10.01, 0.0), point("d", 20.0, 5.0)], &[line("e1", "a", "b"), line("e2", "c", "d")]);
    let refs = r#"[{"kind":"sketchCurve","sketch":"S1","entity":"e1"},{"kind":"sketchCurve","sketch":"S1","entity":"e2"}]"#;
    let o = run(&doc(&[&gap, &format!(r#"{{"type":"compositeCurve","id":"C1","curves":{refs}}}"#)]));
    assert_eq!(o.code(), "compositeCurveGap");
    assert_eq!(o.err_param("index").as_f64(), Some(1.0));
    assert!(close(o.err_param("gap").as_f64().unwrap(), 0.01, 1e-9));
    assert!(o.has_hint());
    run(&doc(&[&gap, &format!(r#"{{"type":"compositeCurve","id":"C1","curves":{refs},"gapTolerance":0.02}}"#)])).ok();
    // the same sketch as a whole is not one chain
    let o = run(&doc(&[&gap, r#"{"type":"compositeCurve","id":"C1","sketch":"S1"}"#]));
    assert_eq!(o.code(), "compositeCurveNotChain");
    assert!(o.has_hint());
    // nothing given; an unresolved reference (with its index); an unknown sketch; a bad gap tolerance
    assert_eq!(run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1"}"#])).code(), "compositeCurveNoInput");
    let o = run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1","curves":[{"kind":"sketchCurve","sketch":"S1","entity":"e1"},{"kind":"sketchCurve","sketch":"S1","entity":"zz"}]}"#]));
    assert_eq!(o.code(), "missingReference");
    assert_eq!(o.err_param("index").as_f64(), Some(1.0));
    assert_eq!(o.err_param("curve").as_str(), Some("zz"));
    assert_eq!(run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1","curve":"NOPE"}"#])).code(), "missingReference");
    assert_eq!(run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1","sketch":"NOPE"}"#])).code(), "missingSketch");
    assert_eq!(run(&doc(&[&line_arc(), r#"{"type":"compositeCurve","id":"C1","sketch":"S1","gapTolerance":-1}"#])).code(), "invalidValue");
    // an unknown face in an edge reference is the structured missingReference of the face
    let o = run(&doc(&[&box_features(), r#"{"type":"compositeCurve","id":"C1","curves":[{"kind":"edge","faces":["F2.C9"],"at":[0,0,10]}]}"#]));
    assert_eq!(o.code(), "missingReference");
}

#[test]
fn registered_with_their_empty_payload_errors() {
    for t in ["offsetCurve", "compositeCurve"] {
        assert!(regen::KNOWN_TYPES.contains(&t));
        let want = regen::EMPTY_FEATURE_ERRORS.iter().find(|(x, _)| *x == t).expect("listed").1;
        assert_eq!(run(&doc(&[&format!(r#"{{"type":"{t}","id":"F20"}}"#)])).code(), want, "{t}");
    }
}

#[test]
fn misspelt_field_warns() {
    let o = run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":-5,"gapTolerence":0.001}"#]));
    o.ok();
    let w = o.r.warnings.iter().find(|w| w.feature == "O1").expect("warning");
    assert_eq!(w.error.code, "unknownField");
}

// ---- offsetCurve --------------------------------------------------------------------------------------------

#[test]
fn offset_of_line_and_arc_is_exact() {
    // C' x N for the line along +x and N = +z is -y: a distance of -5 moves toward the arc centre
    let o = run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":-5}"#]));
    o.ok();
    assert_eq!(o.s("O1", "kind"), "offset");
    assert_eq!(o.num("O1", "deviation"), 0.0);
    assert!(close(o.num("O1", "length"), 20.0 + 2.5 * PI, 1e-6), "{}", o.num("O1", "length"));
    for q in o.pts("O1") {
        let on_line = close(q.y, 0.0, 1e-9) && q.x <= 20.0 + 1e-9;
        let on_arc = close(q.dist(Vec3::new(20.0, 5.0, 0.0)), 5.0, 1e-9);
        assert!(on_line || on_arc, "{q:?}");
    }
    // outward (+5): radius 15
    let o = run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":5}"#]));
    o.ok();
    assert!(close(o.num("O1", "length"), 20.0 + 7.5 * PI, 1e-6));
    // `normal` reversed flips the side
    let o = run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":5,"normal":[0,0,-1]}"#]));
    o.ok();
    assert!(close(o.num("O1", "length"), 20.0 + 2.5 * PI, 1e-6));
}

#[test]
fn offset_of_a_spline_is_within_its_bound() {
    let sk = sketch(
        "S1",
        FRONT,
        &[point("a", 0.0, 0.0), point("b", 10.0, 6.0), point("c", 20.0, 2.0), point("d", 30.0, 8.0)],
        &[r#"{"type":"spline","id":"e1","pts":["a","b","c","d"]}"#.to_string()],
    );
    let o = run(&doc(&[&sk, r#"{"type":"compositeCurve","id":"C1","sketch":"S1"}"#, r#"{"type":"offsetCurve","id":"O1","curve":"C1","distance":1,"tolerance":0.0005}"#]));
    o.ok();
    let dev = o.num("O1", "deviation");
    assert!(dev > 0.0 && dev <= 0.0005, "{dev}");
    // every display point is 1 from the source curve, in its plane
    let base = o.pts("C1");
    for q in o.pts("O1").iter().step_by(8) {
        let d = base.windows(2).map(|w| {
            let ab = w[1] - w[0];
            let t = ((*q - w[0]).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
            (w[0] + ab * t).dist(*q)
        }).fold(f64::INFINITY, f64::min);
        assert!(close(d, 1.0, 2e-3) && q.z.abs() < 1e-9, "{q:?} {d}");
    }
}

#[test]
fn offset_refusals() {
    // past the radius of the arc (10): the offset reverses
    let o = run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":-12}"#]));
    assert_eq!(o.code(), "offsetCurveFold");
    assert!(close(o.err_param("radius").as_f64().unwrap(), 10.0, 1e-6), "{:?}", o.err_param("radius"));
    assert!(o.has_hint());
    // a corner has no single offset
    let l = sketch("S1", FRONT, &[point("a", 0.0, 0.0), point("b", 10.0, 0.0), point("c", 10.0, 10.0)], &[line("e1", "a", "b"), line("e2", "b", "c")]);
    let o = run(&doc(&[&l, r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":1}"#]));
    assert_eq!(o.code(), "offsetCurveCorner");
    assert_eq!(o.err_param("index").as_f64(), Some(1.0));
    // a closed square: corners everywhere, the first found at a joint
    let sq = sketch("S1", FRONT, &[point("a", 0.0, 0.0), point("b", 10.0, 0.0), point("c", 10.0, 10.0), point("d", 0.0, 10.0)], &[line("e1", "a", "b"), line("e2", "b", "c"), line("e3", "c", "d"), line("e4", "d", "a")]);
    assert_eq!(run(&doc(&[&sq, r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":1}"#])).code(), "offsetCurveCorner");
    let o = run(&doc(&[&sq, r#"{"type":"compositeCurve","id":"C1","sketch":"S1"}"#]));
    o.ok();
    assert_eq!(o.marker("C1").get("closed").and_then(|v| v.as_bool()), Some(true));
    assert!(close(o.num("C1", "length"), 40.0, 1e-9));
    // one straight line by reference: no plane to offset in, until one is given
    let one = r#""curves":[{"kind":"sketchCurve","sketch":"S1","entity":"e1"}]"#;
    let o = run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1}}"#)]));
    assert_eq!(o.code(), "offsetCurvePlane");
    assert!(o.has_hint());
    let o = run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1,"normal":[0,0,1]}}"#)]));
    o.ok();
    assert!(o.pts("O1").iter().all(|q| close(q.y, -1.0, 1e-9)));
    let o = run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1,"plane":{{"kind":"std","id":"front"}}}}"#)]));
    o.ok();
    // the normal along the line: no offset direction; a zero normal
    assert_eq!(run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1,"normal":[1,0,0]}}"#)])).code(), "offsetCurveDirection");
    assert_eq!(run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1,"normal":[0,0,0]}}"#)])).code(), "offsetCurveDirection");
    // distance missing, zero, or not a number; an unknown plane
    for d in [r#""#, r#","distance":0"#, r#","distance":"=nope*2""#] {
        let o = run(&doc(&[&line_arc(), &format!(r#"{{"type":"offsetCurve","id":"O1","sketch":"S1"{d}}}"#)]));
        assert!(matches!(o.code().as_str(), "invalidValue" | "badExpression"), "{d}: {}", o.code());
    }
    assert_eq!(run(&doc(&[&l, &format!(r#"{{"type":"offsetCurve","id":"O1",{one},"distance":1,"plane":{{"kind":"feature","id":"NOPE"}}}}"#)])).code(), "missingReference");
    assert_eq!(run(&doc(&[&line_arc(), r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":1,"tolerance":0}"#])).code(), "invalidValue");
}

#[test]
fn offset_on_a_support_face() {
    // a line on the top face (z = 10) from (5,5) to (35,5): C' x n = x X z = -y, so +2 goes to y = 3
    let src = on_front("S2", 10.0, &[point("a", 5.0, 5.0), point("b", 35.0, 5.0)], &[line("e1", "a", "b")]);
    let o = run(&doc(&[&box_features(), &src, r#"{"type":"offsetCurve","id":"O1","sketch":"S2","distance":2,"face":"F2.C1"}"#]));
    o.ok();
    assert_eq!(o.marker("O1").get_str("face"), Some("F2.C1"));
    for q in o.pts("O1") {
        assert!(close(q.y, 3.0, 1e-6) && close(q.z, 10.0, 1e-6), "{q:?}");
    }
    // an unknown face
    let o = run(&doc(&[&box_features(), &src, r#"{"type":"offsetCurve","id":"O1","sketch":"S2","distance":2,"face":"F2.C9"}"#]));
    assert_eq!(o.code(), "missingReference");
}

// ---- projectCurve, procedural -------------------------------------------------------------------------------

fn circle_above(z: f64, c: (f64, f64), r: f64) -> String {
    on_front("S3", z, &[point("c", c.0, c.1)], &[format!(r#"{{"type":"circle","id":"e1","c":"c","r":{r}}}"#)])
}

#[test]
fn projection_onto_a_plane_face() {
    for along in [r#""#, r#","along":"normal""#, r#","along":[0,0,-1]"#] {
        let o = run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), &format!(r#"{{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C1"]{along}}}"#)]));
        o.ok();
        assert_eq!(o.s("F4", "kind"), "projected");
        assert!(close(o.num("F4", "length"), 12.0 * PI, 1e-5), "{along}: {}", o.num("F4", "length"));
        assert_eq!(o.marker("F4").get("closed").and_then(|v| v.as_bool()), Some(true));
        for q in o.pts("F4") {
            assert!(close(q.z, 10.0, 1e-9) && close(q.dist(Vec3::new(20.0, 10.0, 10.0)), 6.0, 1e-9), "{q:?}");
        }
    }
}

#[test]
fn projection_onto_a_sphere_to_the_closest_point() {
    // the circle r = 6 at z = 20 onto the sphere r = 10: radius 10 * 6 / sqrt(436), on the sphere
    let o = run(&doc(&[&sphere_features(), &circle_above(20.0, (0.0, 0.0), 6.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.S.e1"],"along":"normal"}"#]));
    o.ok();
    assert_eq!(o.s("F4", "mode"), "normal");
    let r = 60.0 / 436f64.sqrt();
    assert!(close(o.num("F4", "length"), 2.0 * PI * r, 1e-4), "{}", o.num("F4", "length"));
    for q in o.pts("F4") {
        assert!(close(q.len(), 10.0, 1e-7) && close(q.x.hypot(q.y), r, 1e-6), "{q:?}");
    }
    assert!(o.num("F4", "deviation") <= 0.001);
}

#[test]
fn projection_refusals() {
    // no body
    assert_eq!(run(&doc(&[&circle_above(20.0, (20.0, 10.0), 6.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C1"]}"#])).code(), "projectCurveNoFaces");
    // the circle sticks out of the 40 x 20 top face
    let o = run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 15.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C1"]}"#]));
    assert_eq!(o.code(), "projectCurveMisses");
    assert_eq!(o.err_param("face").as_str(), Some("F2.C1"));
    assert!(o.has_hint());
    // a direction parallel to the face: the line never meets the plane
    let o = run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C1"],"along":[1,0,0]}"#]));
    assert_eq!(o.code(), "projectCurveMisses");
    // not exactly one face; a bad `along`; nothing to project; an unknown face
    for faces in [r#""#, r#","faces":[]"#, r#","faces":["F2.C1","F2.C0"]"#] {
        let o = run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), &format!(r#"{{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3"{faces}}}"#)]));
        assert_eq!(o.code(), "invalidValue", "{faces}");
        assert_eq!(o.err_param("field").as_str(), Some("faces"));
    }
    assert_eq!(run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C1"],"along":[0,0,0]}"#])).code(), "invalidValue");
    assert_eq!(run(&doc(&[&box_features(), r#"{"type":"projectCurve","id":"F4","procedural":true,"faces":["F2.C1"]}"#])).code(), "projectCurveNoInput");
    assert_eq!(run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), r#"{"type":"projectCurve","id":"F4","procedural":true,"sketch":"S3","faces":["F2.C9"]}"#])).code(), "missingReference");
}

#[test]
fn procedural_false_is_the_plain_projection() {
    // the option off (or absent) keeps the fitted multi-face projection and its marker
    for opt in [r#""#, r#","procedural":false"#] {
        let o = run(&doc(&[&box_features(), &circle_above(20.0, (20.0, 10.0), 6.0), &format!(r#"{{"type":"projectCurve","id":"F4","sketch":"S3","faces":["F2.C1"]{opt}}}"#)]));
        o.ok();
        assert!(o.marker("F4").get("kind").is_none());
        assert!(o.curve("F4").get("procedural").is_none());
        assert_eq!(o.num("F4", "degree"), 3.0);
    }
}

// ---- downstream -------------------------------------------------------------------------------------------

#[test]
fn sweep_along_an_offset_curve() {
    // the offset (line y = 0 from x = 0 to 20, then an arc of radius 5) starts at the origin along +x: a circle of
    // radius 1 on the right plane sweeps a tube of volume pi * 1^2 * length (the pieces are swept exactly)
    let prof = sketch("S9", r#"{"kind":"std","id":"right"}"#, &[point("c", 0.0, 0.0)], &[r#"{"type":"circle","id":"e1","c":"c","r":1}"#.to_string()]);
    let off = r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":-5}"#;
    let sw = r#"{"type":"sweep","id":"W1","op":"add","sketch":"S9","path":{"kind":"curve","id":"O1"}}"#;
    let o = run(&doc(&[&line_arc(), off, &prof, sw]));
    o.ok();
    let want = PI * (20.0 + 2.5 * PI);
    assert!(close(o.volume(), want, 1e-3 * want), "{} vs {want}", o.volume());
    // the same along a composite of the offset (curve -> curve) and as a pipe path
    let comp = r#"{"type":"compositeCurve","id":"C2","curve":"O1"}"#;
    let o = run(&doc(&[&line_arc(), off, comp, &prof, &sw.replace(r#""id":"O1""#, r#""id":"C2""#)]));
    o.ok();
    assert!(close(o.volume(), want, 1e-3 * want), "{}", o.volume());
}

#[test]
fn curve_consumers_take_procedural_curves() {
    let off = r#"{"type":"offsetCurve","id":"O1","sketch":"S1","distance":-5}"#;
    let cref = r#"{"kind":"curve","id":"O1"}"#;
    // editCurve rebuilds it as a B-spline
    let ec = format!(r#"{{"type":"editCurve","id":"E1","curves":[{cref}],"method":"count","degree":3,"count":10}}"#);
    let o = run(&doc(&[&line_arc(), off, &ec]));
    o.ok();
    assert_eq!(o.num("E1", "count"), 10.0);
    // a plane through it: z = 0
    let pl = format!(r#"{{"type":"plane","id":"P5","mode":"curve","curves":[{cref}]}}"#);
    let o = run(&doc(&[&line_arc(), off, &pl]));
    o.ok();
    let (_, fr) = o.r.planes.iter().find(|p| p.0 == "P5").expect("plane P5");
    assert!(fr.z.z.abs() > 1.0 - 1e-9 && fr.o.z.abs() < 1e-6, "{fr:?}");
    // the plain projectCurve projects it (a planar curve feature): a circle r = 4 offset to r = 6, onto the top face
    let circ = sketch("S5", FRONT, &[point("c", 20.0, 10.0)], &[r#"{"type":"circle","id":"e1","c":"c","r":4}"#.to_string()]);
    let o = run(&doc(&[&box_features(), &circ, r#"{"type":"offsetCurve","id":"O2","sketch":"S5","distance":2}"#, r#"{"type":"projectCurve","id":"J1","curve":"O2","faces":["F2.C1"]}"#]));
    o.ok();
    assert!(close(o.num("O2", "length"), 12.0 * PI, 1e-9));
    assert!(close(o.num("J1", "length"), 12.0 * PI, 1e-2), "{}", o.num("J1", "length"));
    assert!(o.marker("J1").get("closed").and_then(|v| v.as_bool()).unwrap());
}

#[test]
fn chained_procedural_features() {
    // projecting an offset onto a plane face, and the offset of a composite equal to the offset of the sketch
    let src = on_front("S2", 25.0, &[point("a", 5.0, 5.0), point("b", 30.0, 5.0), point("c", 30.0, 10.0), point("d", 35.0, 10.0)], &[line("e1", "a", "b"), arc("e2", "c", "b", "d")]);
    let off = r#"{"type":"offsetCurve","id":"O1","sketch":"S2","distance":-2}"#;
    let pj = r#"{"type":"projectCurve","id":"J1","procedural":true,"curve":"O1","faces":["F2.C1"],"along":[0,0,-1]}"#;
    let o = run(&doc(&[&box_features(), &src, off, pj]));
    o.ok();
    assert!(close(o.num("J1", "length"), o.num("O1", "length"), 1e-6), "{} {}", o.num("J1", "length"), o.num("O1", "length"));
    assert!(o.pts("J1").iter().all(|q| close(q.z, 10.0, 1e-9)));
    let a = run(&doc(&[&box_features(), &src, off]));
    let b = run(&doc(&[&box_features(), &src, r#"{"type":"compositeCurve","id":"C1","sketch":"S2"}"#, &off.replace(r#""sketch":"S2""#, r#""curve":"C1","normal":[0,0,1]"#)]));
    a.ok();
    b.ok();
    let (pa, pb) = (a.pts("O1"), b.pts("O1"));
    assert_eq!(pa.len(), pb.len());
    assert!(pa.iter().zip(&pb).all(|(x, y)| x.dist(*y) < 1e-9));
}

// ---- determinism, round trip ------------------------------------------------------------------------------

fn full_doc() -> String {
    let src = on_front("S2", 25.0, &[point("a", 5.0, 5.0), point("b", 30.0, 5.0), point("c", 30.0, 10.0), point("d", 35.0, 10.0)], &[line("e1", "a", "b"), arc("e2", "c", "b", "d")]);
    doc(&[
        &box_features(),
        &src,
        r#"{"type":"compositeCurve","id":"C1","sketch":"S2","gapTolerance":0.0001}"#,
        r#"{"type":"offsetCurve","id":"O1","curve":"C1","distance":"=1+1","normal":[0,0,1],"tolerance":0.0005}"#,
        r#"{"type":"projectCurve","id":"J1","procedural":true,"curve":"O1","faces":["F2.C1"],"along":[0,0,-1]}"#,
    ])
}

#[test]
fn regen_is_deterministic() {
    let (a, b) = (run(&full_doc()), run(&full_doc()));
    a.ok();
    assert_eq!(ck_json::to_string(&Json::Arr(a.r.curves.clone())), ck_json::to_string(&Json::Arr(b.r.curves.clone())));
    assert_eq!(ck_json::to_string(&Json::Arr(a.r.markers.clone())), ck_json::to_string(&Json::Arr(b.r.markers.clone())));
}

#[test]
fn document_round_trip() {
    let d = full_doc();
    let part = Part::from_json(&ck_json::parse(&d).unwrap()).unwrap();
    let text = ck_json::to_string(&part.to_json());
    assert_eq!(text, ck_json::to_string(&ck_json::parse(&d).unwrap()));
    let again = Part::from_json(&ck_json::parse(&text).unwrap()).unwrap();
    let f = again.features.iter().find(|f| f.type_name == "offsetCurve").expect("feature kept");
    assert_eq!(f.raw.get_str("distance"), Some("=1+1"));
    let (a, b) = (run(&d), run(&text));
    b.ok();
    assert_eq!(ck_json::to_string(&Json::Arr(a.r.curves)), ck_json::to_string(&Json::Arr(b.r.curves)));
}

#[test]
fn snapshot_keeps_the_definitions() {
    // the regen cache snapshots the Brep: a definition must survive save / load so a later feature builds on it
    let o = run(&full_doc());
    let snap = o.brep.save(None);
    let mut b = Brep::new();
    b.load(&snap);
    assert_eq!(b.proc_curves, o.brep.proc_curves);
    assert_eq!(b.proc_curves.len(), 3);
}
