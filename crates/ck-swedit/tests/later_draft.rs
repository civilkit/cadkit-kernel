//! A draft made after the fact: round trip on a body made in-test. A boss, then a draft tilting one of its walls about
//! the face it stands on; the history is read back from the faces alone and must rebuild the same body.

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
                "draft" => ("Draft", format!("Draft{id}")),
                t => panic!("{t}"),
            };
            let dims = f.get_f64("depth").or(f.get_f64("radius")).or(f.get_f64("angle")).map(|d| vec![SwDim { name: "D1".into(), text: d.to_string(), value: Some(d) }]).unwrap_or_default();
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

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;


#[test]
fn one_wall_tilted_by_a_later_draft() {
    let boss = r#"{"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"p1","x":-10,"y":-8},{"id":"p2","x":10,"y":-8},{"id":"p3","x":0,"y":8}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":12},
{"type":"draft","id":"F5","neutral":{"kind":"face","id":"F2.C1"},"faces":["F4.S.e1"],"angle":7}"#;
    let o = run(&format!("[{BOX},{boss}]"));
    let steps: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback: {:?}\n{steps:#?}", o.fallback);
    assert!(o.verify.as_ref().unwrap().exact, "{steps:#?}");
    assert!(steps.iter().any(|s| s.contains("a later draft")), "{steps:#?}");
    assert!(steps.iter().any(|s| s.starts_with("Draft") && s.contains("tilted")), "{steps:#?}");
}
