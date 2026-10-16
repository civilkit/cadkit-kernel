//! Stacked bodies: bosses, pockets and islands are recovered from the wall cycles' height intervals.
use ck_json::Json;
use ck_recog::{rebuild, recognize};

const RECT: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

const SQ: &str = r#"[{"id":"p1","x":-8,"y":-8},{"id":"p2","x":8,"y":-8},{"id":"p3","x":8,"y":8},{"id":"p4","x":-8,"y":8}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}]"#;

fn run(src: &str) -> (bool, bool, f64, String, usize) {
    let feats = match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!(),
    };
    let (body, errs) = rebuild::rebuild(&feats).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let body = body.unwrap();
    let rec = recognize(&body);
    let dbg = ck_json::to_string(&ck_recog::report(&body, &rec));
    let (nb, errs) = rebuild::rebuild(&rec.features).unwrap();
    assert!(errs.is_empty(), "rebuild errors {errs:?} {dbg} {}", ck_json::to_string(&Json::Arr(rec.features.clone())));
    let v = rebuild::verify(&body, &nb.unwrap());
    (v.exact, v.close, v.rel, dbg, rec.unexplained.len())
}

#[test]
fn plate_with_boss() {
    let (exact, _, rel, dbg, un) = run(&format!(
        r#"[{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":{SQ},"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":5}}]"#
    ));
    assert!(exact && un == 0, "rel {rel} {dbg}");
}

#[test]
fn plate_with_pocket() {
    let (exact, _, rel, dbg, un) = run(&format!(
        r#"[{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":{SQ},"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":3}}]"#
    ));
    assert!(exact && un == 0, "rel {rel} {dbg}");
}
