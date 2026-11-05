//! Modelled threads read back (`thread.rs`): a ridge on a shaft and a groove in a bore, each swept on a screw motion
//! about the axis by the kernel's `explicit` thread, then read back from the faces alone (the thread's faces carry
//! its feature id, as in a SolidWorks file) and rebuilt: the rebuilt body must match the original, and the pitch, hand
//! and add / cut come back.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Outcome, Status, Tagged};

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

fn run(src: &str) -> Outcome {
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
    let sw: Vec<SwFeature> = fs
        .iter()
        .map(|f| {
            let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
            let dim = |v: f64| vec![SwDim { name: "D1".into(), text: v.to_string(), value: Some(v) }];
            match f.get_str("type").unwrap() {
                "sketch" => SwFeature { id, name: format!("Sketch{id}"), kind: "Sketch".into(), dims: vec![] },
                "extrude" => SwFeature { id, name: format!("Boss-Extrude{id}"), kind: "Boss-Extrude".into(), dims: dim(f.get_f64("depth").unwrap()) },
                "thread" => SwFeature { id, name: "Thread1".into(), kind: "Thread".into(), dims: vec![] },
                t => panic!("{t}"),
            }
        })
        .collect();
    rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 })
}

fn circle(id: &str, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":{r}}}],"constraints":[]}}"#)
}

/// The thread feature the read-back emitted.
fn thread(o: &Outcome) -> &Json {
    o.features.iter().find(|f| f.get_str("type") == Some("thread")).unwrap_or_else(|| panic!("no thread feature in {}", ck_json::to_string(&Json::Arr(o.features.clone()))))
}

fn check(o: &Outcome, op: &str, turns: f64, rise: f64) {
    let notes: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback {:?}\n{notes:#?}", o.fallback);
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{notes:#?}");
    let v = o.verify.as_ref().unwrap();
    assert!(v.close, "rel {} area {}\n{notes:#?}", v.rel, v.area_rel);
    let ex = thread(o).get("explicit").unwrap();
    assert_eq!(ex.get_str("op"), Some(op));
    let (a, r) = (ex.get_f64("angle").unwrap(), ex.get_f64("rise").unwrap());
    assert!((a.abs() / std::f64::consts::TAU - turns).abs() < 1e-6, "turns {}", a.abs() / std::f64::consts::TAU);
    assert!((r.abs() - rise).abs() < 1e-6, "rise {r}");
    // the hand: right-handed when the turn about the axis and the advance along it have one sign
    let right = a * r > 0.0;
    let want_right = !notes.iter().any(|n| n.contains("left-handed"));
    assert_eq!(right, want_right, "{notes:#?}");
}

/// A shaft of radius 4 and length 20 (axis y), a V ridge of 5 turns and pitch 1.5 from y = 3 up.
fn ridge(angle: f64) -> String {
    format!(
        r#"[{sk},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}},
        {{"type":"thread","id":"F3","explicit":{{"o":[0,0,0],"axis":[0,1,0],"x":[1,0,0],"profile":[[3.95,2.95],[4.8,3.5],[3.95,4.05]],"angle":{angle},"rise":7.5,"op":"add"}}}}]"#,
        sk = circle("F1", 4.0)
    )
}

#[test]
fn right_handed_ridge_on_a_shaft() {
    let o = run(&ridge(5.0 * std::f64::consts::TAU));
    check(&o, "add", 5.0, 7.5);
    assert!(o.steps.iter().any(|s| s.note.contains("right-handed") && s.note.contains("pitch 1.5000")), "{:?}", o.steps.iter().map(|s| &s.note).collect::<Vec<_>>());
}

#[test]
fn left_handed_ridge_on_a_shaft() {
    let o = run(&ridge(-5.0 * std::f64::consts::TAU));
    check(&o, "add", 5.0, 7.5);
    assert!(o.steps.iter().any(|s| s.note.contains("left-handed")));
}

#[test]
fn groove_in_a_bore() {
    // a bore of radius 4 through a block of radius 9 (the sketch has both circles); the groove reaches out to 4.8
    let sk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":9},{"type":"circle","id":"e2","c":"c","r":4}],"constraints":[]}"#;
    let src = format!(
        r#"[{sk},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}},
        {{"type":"thread","id":"F3","explicit":{{"o":[0,0,0],"axis":[0,1,0],"x":[1,0,0],"profile":[[3.95,2.95],[4.8,3.5],[3.95,4.05]],"angle":{a},"rise":7.5,"op":"cut"}}}}]"#,
        a = 5.0 * std::f64::consts::TAU
    );
    let o = run(&src);
    check(&o, "cut", 5.0, 7.5);
}
