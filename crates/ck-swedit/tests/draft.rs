//! Drafted extrudes: round trips on bodies made in-test (as `synthetic.rs`). The part is regenerated, its faces
//! are tagged with the feature that made them, the history is read back and the rebuilt body must match.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Outcome, Status, Tagged};

fn run(src: &str) -> Outcome {
    let fs = match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    };
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        if let Ok(t) = body.face(fid).name.feature[1..].parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let sw: Vec<SwFeature> = fs
        .iter()
        .map(|f| {
            let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
            let cut = f.get_str("op") == Some("cut");
            let (kind, name) = match f.get_str("type").unwrap() {
                "sketch" => ("Sketch", format!("Sketch{id}")),
                "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}")),
                "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}")),
                "fillet" => ("Fillet", format!("Fillet{id}")),
                t => panic!("{t}"),
            };
            let dims = f.get_f64("depth").or(f.get_f64("radius")).map(|d| vec![SwDim { name: "D1".into(), text: d.to_string(), value: Some(d) }]).unwrap_or_default();
            SwFeature { id, name, kind: kind.into(), dims }
        })
        .collect();
    rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None })
}

fn assert_exact(o: &Outcome) {
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    let feats = ck_json::to_string(&Json::Arr(o.features.clone()));
    assert!(o.fallback.is_none(), "fallback: {:?}\nsteps: {steps:#?}\nfeatures: {feats}", o.fallback);
    let v = o.verify.as_ref().unwrap();
    assert!(v.exact, "not exact: rel {} area {} bbox {}\nsteps: {steps:#?}\nfeatures: {feats}", v.rel, v.area_rel, v.bbox_dev);
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{steps:#?}");
}

/// The drafted extrudes of the rebuilt document: (draft degrees, outward).
fn drafts(o: &Outcome) -> Vec<(f64, bool)> {
    o.features
        .iter()
        .filter(|f| f.get_str("type") == Some("extrude"))
        .filter_map(|f| f.get_f64("draft").map(|d| (d, f.get("draftOut").and_then(|b| b.as_bool()).unwrap_or(false))))
        .collect()
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// A rounded rectangle (half sizes `w`, `h`, corner radius `r`) about (`cx`, `cy`) on `plane`.
fn rounded(id: &str, plane: &str, cx: f64, cy: f64, w: f64, h: f64, r: f64) -> String {
    let p = |k: &str, x: f64, y: f64| format!(r#"{{"id":"{k}","x":{},"y":{}}}"#, cx + x, cy + y);
    let pts = [
        p("a1", w - r, -h),
        p("a2", w, -h + r),
        p("b1", w, h - r),
        p("b2", w - r, h),
        p("c1", -w + r, h),
        p("c2", -w, h - r),
        p("d1", -w, -h + r),
        p("d2", -w + r, -h),
        p("ca", w - r, -h + r),
        p("cb", w - r, h - r),
        p("cc", -w + r, h - r),
        p("cd", -w + r, -h + r),
    ];
    let ents = [
        r#"{"id":"e1","type":"line","p1":"d2","p2":"a1"}"#,
        r#"{"id":"e2","type":"arc","c":"ca","p1":"a1","p2":"a2"}"#,
        r#"{"id":"e3","type":"line","p1":"a2","p2":"b1"}"#,
        r#"{"id":"e4","type":"arc","c":"cb","p1":"b1","p2":"b2"}"#,
        r#"{"id":"e5","type":"line","p1":"b2","p2":"c1"}"#,
        r#"{"id":"e6","type":"arc","c":"cc","p1":"c1","p2":"c2"}"#,
        r#"{"id":"e7","type":"line","p1":"c2","p2":"d1"}"#,
        r#"{"id":"e8","type":"arc","c":"cd","p1":"d1","p2":"d2"}"#,
    ];
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

const TOP: &str = r#"{"kind":"std","id":"top"}"#;
const ON_BOX: &str = r#"{"kind":"face","id":"F2.C1"}"#;

#[test]
fn drafted_base_boss_with_round_corners() {
    // a base feature: both caps, tilted planes and cones; the profile is the bottom cap's outline
    let src = format!(r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"draft":5}}]"#, rounded("F1", TOP, 0.0, 0.0, 20.0, 10.0, 4.0));
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert_eq!(d.len(), 1, "{d:?}");
    assert!((d[0].0 - 5.0).abs() < 1e-6 && !d[0].1, "{d:?}");
}

#[test]
fn drafted_boss_on_a_box() {
    // the boss's bottom merges into the box's top: the profile comes from the side traces at the sketch level
    let src = format!(r#"[{BOX},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":8,"draft":7}}]"#, rounded("F3", ON_BOX, 3.0, -2.0, 12.0, 8.0, 3.0));
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert_eq!(d.len(), 1, "{d:?}");
    assert!((d[0].0 - 7.0).abs() < 1e-6 && !d[0].1, "{d:?}");
}

#[test]
fn outward_drafted_square_boss() {
    let src = format!(r#"[{BOX},{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":6,"draft":10,"draftOut":true}}]"#, rect("F3", ON_BOX, -8.0, -6.0, 10.0, 7.0));
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert!(d.len() == 1 && (d[0].0 - 10.0).abs() < 1e-6 && d[0].1, "{d:?}");
}

#[test]
fn drafted_pocket() {
    // a blind pocket narrowing with depth: tilted walls and a floor
    let src = format!(r#"[{BOX},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":5,"draft":4}}]"#, rect("F3", ON_BOX, -12.0, -9.0, 15.0, 8.0));
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert!(d.len() == 1 && (d[0].0 - 4.0).abs() < 1e-6 && !d[0].1, "{d:?}");
}

#[test]
fn drafted_round_cornered_pocket() {
    let src = format!(r#"[{BOX},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":6,"draft":3}}]"#, rounded("F3", ON_BOX, -4.0, 2.0, 14.0, 9.0, 5.0));
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert!(d.len() == 1 && (d[0].0 - 3.0).abs() < 1e-6 && !d[0].1, "{d:?}");
}

#[test]
fn drafted_base_with_filleted_corners() {
    // a later fillet removes the drafted corners: the corners are re-met from the side traces at the sketch level
    let t = 5f64.to_radians().tan() * 5.0;
    let (x, y) = (20.0 - t, 10.0 - t);
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"draft":5}},
        {{"type":"fillet","id":"F3","radius":2,"edges":[{{"faces":["F2.S.e1","F2.S.e2"],"at":[{x},{},5]}},{{"faces":["F2.S.e2","F2.S.e3"],"at":[{x},{y},5]}},{{"faces":["F2.S.e3","F2.S.e4"],"at":[{},{y},5]}},{{"faces":["F2.S.e4","F2.S.e1"],"at":[{},{},5]}}]}}]"#,
        rect("F1", r#"{"kind":"std","id":"front"}"#, -20.0, -10.0, 20.0, 10.0),
        -y,
        -x,
        -x,
        -y
    );
    let o = run(&src);
    assert_exact(&o);
    let d = drafts(&o);
    assert!(d.len() == 1 && (d[0].0 - 5.0).abs() < 1e-6 && !d[0].1, "{d:?}");
    assert!(o.steps[0].note.contains("corners re-met"), "{}", o.steps[0].note);
}

#[test]
fn drafted_base_whose_bottom_a_later_cut_removed() {
    // a later cut takes off the bottom slab: the boss keeps its top cap only and its sides end short, so the far
    // end comes from the listed depth. The cut is not read back (no sides), so the boss is the editable prefix; the rebuilt
    // drafted boss alone must equal the original boss.
    let boss = format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"draft":6}}"#, rounded("F1", r#"{"kind":"std","id":"front"}"#, 0.0, 0.0, 20.0, 10.0, 4.0));
    let src = format!(r#"[{boss},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth"}}]"#, rect("F3", r#"{"kind":"std","id":"right"}"#, -2.0, -15.0, 1.0, 15.0));
    let o = run(&src);
    assert_eq!(o.steps[0].status, Status::Rebuilt, "{:?}", o.steps);
    assert!(o.steps[0].note.contains("listed depth"), "{}", o.steps[0].note);
    let fs: Vec<Json> = o.features.iter().take_while(|f| (f.get_str("type") != Some("extrude") || f.get_f64("draft").is_some()) && f.get_str("type") != Some("importBody")).cloned().collect();
    let (rebuilt, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let orig = match ck_json::parse(&format!("[{boss}]")).unwrap() {
        Json::Arr(a) => rebuild::rebuild(&a).unwrap().0.unwrap(),
        _ => unreachable!(),
    };
    let v = rebuild::verify(&orig, &rebuilt.unwrap());
    assert!(v.exact, "rel {} area {}: {}", v.rel, v.area_rel, ck_json::to_string(&Json::Arr(fs)));
}
