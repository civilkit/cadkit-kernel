//! Revolve round trips on bodies made in-test: a part document is regenerated, its faces are tagged with the feature
//! that made them (as a SolidWorks file tags them), the history is read back from the tagged body and the rebuilt
//! body must match the original. Also: cosmetic features are skipped, not fatal.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Status, Tagged};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

fn sw_features(fs: &[Json]) -> Vec<SwFeature> {
    let mut out = vec![];
    for f in fs {
        let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
        let ty = f.get_str("type").unwrap();
        let cut = f.get_str("op") == Some("cut");
        let (kind, name, dims): (&str, String, Vec<(&str, f64)>) = match ty {
            "sketch" => ("Sketch", format!("Sketch{id}"), vec![]),
            "plane" => ("Plane", format!("Plane{id}"), vec![]),
            "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}"), f.get_f64("depth").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}"), f.get_f64("depth").map(|d| vec![("D1", d)]).unwrap_or_default()),
            "revolve" if cut => ("RevCut", format!("Cut-Revolve{id}"), vec![]),
            "revolve" => ("Revolve", format!("Revolve{id}"), vec![]),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims: dims.into_iter().map(|(n, v)| SwDim { name: n.into(), text: v.to_string(), value: Some(v) }).collect() });
    }
    out
}

struct Res {
    out: ck_swedit::Outcome,
    types: Vec<String>,
}

/// `extra`: features appended to the SolidWorks list that put nothing on the body (id, kind, name), with the
/// faces (by index into the body's faces) they annotate.
fn run_with(src: &str, extra: &[(i64, &str, &str, Vec<usize>)]) -> Res {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        if let Ok(t) = body.face(fid).name.feature[1..].parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let mut sw = sw_features(&fs);
    let ids: Vec<_> = body.face_ids().collect();
    for (id, kind, name, on) in extra {
        sw.push(SwFeature { id: *id, name: (*name).into(), kind: (*kind).into(), dims: vec![] });
        for &i in on {
            tags[ids[i].idx()].push((*id, 0));
        }
    }
    let t = Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None };
    let out = rebuild_history(&t);
    let types = out.features.iter().map(|f| f.get_str("type").unwrap().to_string()).collect();
    Res { out, types }
}

fn run(src: &str) -> Res {
    run_with(src, &[])
}

fn assert_exact(r: &Res) {
    let o = &r.out;
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {steps:#?}\nfeatures: {}", o.fallback, ck_json::to_string(&Json::Arr(o.features.clone())));
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {} bbox {}\nsteps: {steps:#?}\nfeatures: {}", v.rel, v.area_rel, v.bbox_dev, ck_json::to_string(&Json::Arr(o.features.clone())));
}

/// A turned part: cylinder r 20 (y 0..30), spherical dome of radius 20 about (0,30), profile on the front plane,
/// axis along y.
const DOME: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":20,"y":0},{"id":"p3","x":20,"y":30},{"id":"p4","x":0,"y":50},{"id":"c","x":0,"y":30},{"id":"a1","x":0,"y":-5},{"id":"a2","x":0,"y":55}],
"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"arc","c":"c","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}"#;

#[test]
fn revolve_boss_whole_body() {
    let r = run(&format!("[{DOME}]"));
    assert_exact(&r);
    assert_eq!(r.types, ["sketch", "revolve"]);
    assert_eq!(r.out.steps.len(), 1);
    assert!(r.out.steps[0].note.contains("boss"), "{}", r.out.steps[0].note);
}

#[test]
fn revolve_cone_and_ring() {
    // a cone frustum with a torus-profile ring (a rounded rim) off the axis
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":16,"y":0},{"id":"p3","x":8,"y":20},{"id":"p4","x":0,"y":20},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":20}],
    "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]"#;
    assert_exact(&run(src));
    // a full torus: a circle off the axis
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":25,"y":0},{"id":"a1","x":0,"y":-10},{"id":"a2","x":0,"y":10}],
    "entities":[{"id":"e1","type":"circle","c":"c","r":6},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]"#;
    assert_exact(&run(src));
}

#[test]
fn revolve_cut_groove_and_bore() {
    // a groove cut into the side of the dome part, then a blind bore from the flat bottom
    let src = format!(
        r#"[{DOME},
        {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":17,"y":8}},{{"id":"q2","x":23,"y":8}},{{"id":"q3","x":23,"y":14}},{{"id":"q4","x":17,"y":14}},{{"id":"a1","x":0,"y":0}},{{"id":"a2","x":0,"y":30}}],
        "entities":[{{"id":"e1","type":"line","p1":"q1","p2":"q2"}},{{"id":"e2","type":"line","p1":"q2","p2":"q3"}},{{"id":"e3","type":"line","p1":"q3","p2":"q4"}},{{"id":"e4","type":"line","p1":"q4","p2":"q1"}},{{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}}],"constraints":[]}},
        {{"type":"revolve","id":"F4","op":"cut","sketch":"F3","axis":"ax","angle":360}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":0,"y":-3}},{{"id":"q2","x":6,"y":-3}},{{"id":"q3","x":6,"y":12}},{{"id":"q4","x":0,"y":12}},{{"id":"a1","x":0,"y":-3}},{{"id":"a2","x":0,"y":12}}],
        "entities":[{{"id":"e1","type":"line","p1":"q1","p2":"q2"}},{{"id":"e2","type":"line","p1":"q2","p2":"q3"}},{{"id":"e3","type":"line","p1":"q3","p2":"q4"}},{{"id":"e4","type":"line","p1":"q4","p2":"q1"}},{{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}}],"constraints":[]}},
        {{"type":"revolve","id":"F6","op":"cut","sketch":"F5","axis":"ax","angle":360}}]"#
    );
    let r = run(&src);
    assert_exact(&r);
    assert_eq!(r.types.iter().filter(|t| *t == "revolve").count(), 3, "{:?}", r.types);
    assert!(r.out.steps.iter().filter(|s| s.note.starts_with("cut")).count() == 2);
}

#[test]
fn revolve_partial_angle() {
    // a 120 degree wedge of a ring (profile off the axis), and a 270 degree turned part touching the axis
    let ring = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":10,"y":0},{"id":"p2","x":25,"y":0},{"id":"p3","x":25,"y":12},{"id":"p4","x":10,"y":12},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":12}],
    "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":120}]"#;
    let r = run(ring);
    assert_exact(&r);
    let sw = &r.out.features;
    let rev = sw.iter().find(|f| f.get_str("type") == Some("revolve")).unwrap();
    assert!((rev.get_f64("angle").unwrap() - 120.0).abs() < 1e-6, "{:?}", rev.get_f64("angle"));
    let turned = DOME.replace(r#""angle":360"#, r#""angle":270"#);
    let r = run(&format!("[{turned}]"));
    assert_exact(&r);
    let rev = r.out.features.iter().find(|f| f.get_str("type") == Some("revolve")).unwrap();
    assert!((rev.get_f64("angle").unwrap() - 270.0).abs() < 1e-6);
}

#[test]
fn revolve_boss_on_an_extruded_base() {
    // a turned boss standing on a box: the boss' profile closes against the box
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-30},{"id":"p2","x":30,"y":-30},{"id":"p3","x":30,"y":30},{"id":"p4","x":-30,"y":30}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
    {"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":10},{"id":"p2","x":12,"y":10},{"id":"p3","x":12,"y":25},{"id":"p4","x":6,"y":30},{"id":"p5","x":0,"y":30},{"id":"a1","x":0,"y":10},{"id":"a2","x":0,"y":30}],
    "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p5"},{"id":"e5","type":"line","p1":"p5","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F4","op":"add","sketch":"F3","axis":"ax","angle":360}]"#;
    let r = run(src);
    assert_exact(&r);
    assert_eq!(r.out.steps.len(), 2);
}

#[test]
fn cosmetic_thread_is_skipped() {
    // a cosmetic thread annotates faces earlier features made and adds none
    let r = run_with(&format!("[{DOME}]"), &[(9, "Thread", "Thread1", vec![0])]);
    assert_exact(&r);
    let th = r.out.steps.iter().find(|s| s.name == "Thread1").expect("thread step");
    assert_eq!(th.status, Status::Skipped, "{}", th.note);
    assert!(th.note.contains("cosmetic"), "{}", th.note);
    assert_eq!(r.types, ["sketch", "revolve"]);
}

#[test]
fn thread_with_geometry_falls_back() {
    // a thread that made faces of its own is not cosmetic
    let fs = feats(&format!("[{DOME}]"));
    let (body, _) = rebuild::rebuild(&fs).unwrap();
    let body = body.unwrap();
    let tags: Vec<Vec<(i64, i64)>> = body.faces.iter().map(|_| vec![(9, 0)]).collect();
    let sw = vec![SwFeature { id: 9, name: "Thread1".into(), kind: "Thread".into(), dims: vec![] }];
    let out = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None });
    assert!(out.fallback.is_some());
    assert_eq!(out.steps[0].status, Status::Unsupported);
}

#[test]
fn revolve_whole_sphere() {
    // a half disc about its diameter: one spherical face, no edges
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0},{"id":"p1","x":0,"y":-8},{"id":"p2","x":0,"y":8}],
    "entities":[{"id":"e1","type":"arc","c":"c","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p1"},{"id":"ax","type":"line","p1":"p1","p2":"p2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]"#;
    let r = run(src);
    assert_exact(&r);
}

#[test]
fn revolve_spline_profile() {
    // a vase: the side is a cubic spline (a surface of revolution with a spline profile), flat bottom and top
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":15,"y":0},{"id":"s1","x":22,"y":10},{"id":"s2","x":10,"y":20},{"id":"p3","x":8,"y":30},{"id":"p4","x":0,"y":30},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":30}],
    "entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"cvspline","pts":["p2","s1","s2","p3"],"degree":3,"knots":[0,0,0,0,1,1,1,1]},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
    {"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]"#;
    let r = run(src);
    assert_exact(&r);
    let sk = r.out.features.iter().find(|f| f.get_str("type") == Some("sketch")).unwrap();
    assert!(ck_json::to_string(sk).contains("cvspline"), "{}", ck_json::to_string(sk));
}

/// A disc r 20 (y 0..10) about the y axis.
const DISC: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":20,"y":0},{"id":"p3","x":20,"y":10},{"id":"p4","x":0,"y":10},{"id":"a1","x":0,"y":0},{"id":"a2","x":0,"y":10}],
"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},
{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}"#;

#[test]
fn revolve_corner_taken_by_a_later_cut() {
    // a bore through the disc, then a later revolved cut that steps the top rim: the disc's profile has four open
    // ends (the bore and the step); the rim corner is read back where the cylinder and the top meet
    let src = format!(
        r#"[{DISC},
        {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":0,"y":-1}},{{"id":"q2","x":5,"y":-1}},{{"id":"q3","x":5,"y":11}},{{"id":"q4","x":0,"y":11}},{{"id":"a1","x":0,"y":-1}},{{"id":"a2","x":0,"y":11}}],
        "entities":[{{"id":"e1","type":"line","p1":"q1","p2":"q2"}},{{"id":"e2","type":"line","p1":"q2","p2":"q3"}},{{"id":"e3","type":"line","p1":"q3","p2":"q4"}},{{"id":"e4","type":"line","p1":"q4","p2":"q1"}},{{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}}],"constraints":[]}},
        {{"type":"revolve","id":"F4","op":"cut","sketch":"F3","axis":"ax","angle":360}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":15,"y":8}},{{"id":"q2","x":21,"y":8}},{{"id":"q3","x":21,"y":11}},{{"id":"q4","x":15,"y":11}},{{"id":"a1","x":0,"y":0}},{{"id":"a2","x":0,"y":10}}],
        "entities":[{{"id":"e1","type":"line","p1":"q1","p2":"q2"}},{{"id":"e2","type":"line","p1":"q2","p2":"q3"}},{{"id":"e3","type":"line","p1":"q3","p2":"q4"}},{{"id":"e4","type":"line","p1":"q4","p2":"q1"}},{{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}}],"constraints":[]}},
        {{"type":"revolve","id":"F6","op":"cut","sketch":"F5","axis":"ax","angle":360}}]"#
    );
    let r = run(&src);
    let rev = r.out.steps.iter().find(|s| s.name == "Revolve2").expect("revolve step");
    assert_eq!(rev.status, Status::Rebuilt, "{}", rev.note);
    // the disc's own sketch is the full rectangle, its rim corner included (the notch cut's read-back is not
    // checked here: a cut that takes a corner is closed by a diagonal)
    let sk = ck_json::to_string(&r.out.features[0]);
    assert!(sk.contains(r#""x":20,"y":10"#), "{sk}");
}

#[test]
fn revolve_faces_split_by_a_later_cut() {
    // a channel cut across the top of the disc from rim to rim splits the top face in two: both halves are one piece
    // of the profile
    let src = format!(
        r#"[{DISC},
        {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":-30,"y":7}},{{"id":"q2","x":30,"y":7}},{{"id":"q3","x":30,"y":11}},{{"id":"q4","x":-30,"y":11}}],
        "entities":[{{"id":"e1","type":"line","p1":"q1","p2":"q2"}},{{"id":"e2","type":"line","p1":"q2","p2":"q3"}},{{"id":"e3","type":"line","p1":"q3","p2":"q4"}},{{"id":"e4","type":"line","p1":"q4","p2":"q1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"mid","depth":4}}]"#
    );
    let r = run(&src);
    let rev = r.out.steps.iter().find(|s| s.name == "Revolve2").expect("revolve step");
    assert_eq!(rev.status, Status::Rebuilt, "{}", rev.note);
    assert!(rev.note.contains("pieces on"), "{}", rev.note);
}
