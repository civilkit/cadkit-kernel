//! Equal-distance chamfers along the outline of an end face: an arc run into straight sides (a stadium end) and a
//! chain that turns by a slight angle at a convex corner. The chain must end in clean mitres (no sliver faces) and
//! remove exactly the analytic volume, whether it is one feature (with or without `propagate`) or several.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}


const BODY: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"right"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":0,"y":10},{"id":"p3","x":0,"y":-10},{"id":"p4","x":25,"y":-10},{"id":"p5","x":25,"y":10}],
"entities":[{"type":"arc","id":"e2","c":"p1","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p5"},{"type":"line","id":"e5","p1":"p5","p2":"p2"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":16}"#;
const ARC: &str = r#"{"faces":["F2.C1","F2.S.e2"],"at":[16,0,10]}"#;
const L3: &str = r#"{"faces":["F2.C1","F2.S.e3"],"at":[16,-10,-12.5]}"#;
const L5: &str = r#"{"faces":["F2.C1","F2.S.e5"],"at":[16,10,-12.5]}"#;

/// (errors, face count, volume, check issues) of the body followed by `features`.
fn run(body: &str, features: &str) -> (Vec<String>, usize, f64, usize) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{body},{features}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, b.num_faces(), ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

fn chamfer(id: &str, d: f64, edges: &[&str], extra: &str) -> String {
    format!(r#"{{"type":"chamfer","id":"{id}","chamferType":"equal","distance":{d},"edges":[{}]{extra}}}"#, edges.join(","))
}

/// Volume of the stadium body (half disc r 10 and a 10 x 25 rectangle, 16 deep) and of the chamfer on `edges` of
/// its end face: the triangle d*d/2 swept along the lines and (about the axis, at radius 10 - d/3) round the arc.
fn expected(d: f64, arc: bool, lines: usize) -> f64 {
    let body = (0.5 * std::f64::consts::PI * 100.0 + 20.0 * 25.0) * 16.0;
    let tri = d * d / 2.0;
    body - if arc { tri * std::f64::consts::PI * (10.0 - d / 3.0) } else { 0.0 } - tri * 25.0 * lines as f64
}

fn check(name: &str, got: (Vec<String>, usize, f64, usize), faces: usize, vol: f64) {
    assert!(got.0.is_empty() && got.3 == 0, "{name}: errors {:?}, check issues {}", got.0, got.3);
    assert_eq!(got.1, faces, "{name}: face count");
    assert!((got.2 - vol).abs() < 2e-3, "{name}: volume {} vs {vol}", got.2);
}

#[test]
fn arc_and_lines_one_feature_end_flush_at_the_tangent_vertices() {
    // 6 faces of the body + 3 blend faces
    let want = expected(2.0, true, 2);
    check("loop", run(BODY, &chamfer("F3", 2.0, &[ARC, L3, L5], "")), 9, want);
    check("loop propagate", run(BODY, &chamfer("F3", 2.0, &[ARC], r#","propagate":true"#)), 9, want);
}

#[test]
fn arc_alone_stops_flush_where_the_outline_runs_on() {
    check("arc", run(BODY, &chamfer("F3", 2.0, &[ARC], "")), 9, expected(2.0, true, 0));
    check("arc + one line", run(BODY, &chamfer("F3", 2.0, &[ARC, L5], "")), 9, expected(2.0, true, 1));
}

#[test]
fn arc_then_lines_in_two_features() {
    let two = format!("{},{}", chamfer("F3", 2.0, &[ARC], ""), chamfer("F4", 2.0, &[L3, L5], ""));
    check("arc then lines", run(BODY, &two), 9, expected(2.0, true, 2));
}

/// An outline that turns by a slight angle at a convex corner (the end face of a part whose corners were chamfered
/// before): the two chamfer bands meet in one mitre edge, with no sliver face between them.
#[test]
fn slight_convex_turn_meets_in_a_clean_mitre() {
    let pts = [(0.0, 10.0), (10.0, 10.0), (25.0, 5.0), (25.0, -5.0), (10.0, -10.0), (0.0, -10.0)];
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, p.0, p.1)).collect();
    let es: Vec<String> = (0..pts.len()).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % pts.len() + 1)).collect();
    let body = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"right"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":16}}"#,
        ps.join(","),
        es.join(",")
    );
    let area = |v: &[(f64, f64)]| 0.5 * (0..v.len()).map(|i| v[i].0 * v[(i + 1) % v.len()].1 - v[(i + 1) % v.len()].0 * v[i].1).sum::<f64>().abs();
    let (a, b) = (r#"{"faces":["F2.C0","F2.S.e1"],"at":[0,10,-5]}"#, r#"{"faces":["F2.C0","F2.S.e2"],"at":[0,7.5,-17.5]}"#);
    let d: f64 = 0.8;
    let removed = (10.0 + (15.0f64 * 15.0 + 25.0).sqrt()) * d * d / 2.0;
    let want = area(&pts) * 16.0 - removed;
    // 6 side faces + 2 end faces + 2 blend faces
    check("mitre", run(&body, &chamfer("F3", d, &[a, b], "")), 10, want);
}
