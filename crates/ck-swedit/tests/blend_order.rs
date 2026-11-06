//! A chamfer SolidWorks ran after another blend that the kernel only gets right when it runs before it (the
//! plane of a corner chamfer runs on down the slanted faces of the earlier bottom chamfer, where the kernel would
//! cap it at the end of its edge): the history is read again with the chamfer first, and kept because the whole
//! body then verifies. The part: a bar (flat sides, round top, bore) with 2 x 15 chamfers on its bottom edges and 2 mm
//! chamfers on its end edges, the corner ones vertical. The solid is built here with the 2 mm chamfers first;
//! the file's tags and order have them last.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::{rebuild_history, Status, Tagged};

const FEATS: &str = r#"[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"right"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":0,"y":-10},{"id":"p3","x":0,"y":10},{"id":"p4","x":25,"y":10},{"id":"p5","x":25,"y":-10}],"entities":[{"type":"circle","id":"e1","c":"p1","r":4.5},{"type":"arc","id":"e2","c":"p1","p1":"p3","p2":"p2"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p5"},{"type":"line","id":"e5","p1":"p5","p2":"p2"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":16},
 {"type":"chamfer","id":"F5","chamferType":"equal","distance":2,"edges":[{"faces":["F2.S.e2","F2.C0"],"at":[0,0,10]},{"faces":["F2.S.e2","F2.C1"],"at":[16,0,10]},{"faces":["F2.C0","F2.S.e3"],"at":[0,10,-12.5]},{"faces":["F2.S.e5","F2.C1"],"at":[16,-10,-5]},{"faces":["F2.S.e3","F2.C1"],"at":[16,10,-5]},{"faces":["F2.C0","F2.S.e5"],"at":[0,-10,-12.5]}]},
 {"type":"chamfer","id":"F3","chamferType":"distances","distance":15,"distance2":2,"edges":[{"faces":["F2.S.e3","F2.S.e4"],"at":[8,10,-25]}]},
 {"type":"chamfer","id":"F4","chamferType":"distances","distance":15,"distance2":2,"edges":[{"faces":["F2.C1","F2.S.e4"],"at":[16,0,-25]}]},
 {"type":"chamfer","id":"F6","chamferType":"distances","distance":15,"distance2":2,"edges":[{"faces":["F2.S.e5","F2.S.e4"],"at":[7,-10,-25]}]}
]"#;

#[test]
fn corner_chamfer_before_the_bottom_chamfers() {
    let Json::Arr(fs) = ck_json::parse(FEATS).unwrap() else { panic!() };
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        if let Ok(t) = body.face(fid).name.feature[1..].parse::<i64>() {
            // (the two bottom chamfers, F3 and F4, are one SolidWorks feature)
            tags[fid.idx()] = vec![(if t == 4 || t == 6 { 3 } else { t }, 0)];
        }
    }
    let f = |id: i64, name: &str, kind: &str| SwFeature { id, name: name.into(), kind: kind.into(), dims: vec![] };
    let mut ex = f(2, "Boss-Extrude1", "Boss-Extrude");
    ex.dims = vec![SwDim { name: "D1".into(), text: "16".into(), value: Some(16.0) }];
    // the file's order: the bottom chamfers, then the corner chamfers
    let mut c1 = f(3, "Chamfer1", "Chamfer");
    c1.dims = vec![SwDim { name: "D1".into(), text: "15".into(), value: Some(15.0) }, SwDim { name: "D3".into(), text: "2".into(), value: Some(2.0) }];
    let sw = vec![f(1, "Sketch1", "Sketch"), ex, c1, f(5, "Chamfer2", "Chamfer")];
    let o = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: None });
    let notes: Vec<String> = o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect();
    assert!(o.fallback.is_none(), "fallback {:?}\n{notes:#?}", o.fallback);
    assert!(o.steps.iter().all(|s| s.status == Status::Rebuilt), "{notes:#?}");
    assert!(o.close(), "{:?}\n{notes:#?}", o.verify.as_ref().map(|v| v.rel));
    assert!(o.steps.iter().any(|s| s.name == "Chamfer2" && s.note.contains("before")), "{notes:#?}");
}
