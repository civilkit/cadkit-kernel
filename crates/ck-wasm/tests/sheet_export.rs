//! `sheetDxf` and `sheetBendTable` over the wire: regen a sheet part, hand the `flatPattern` of its `sheetMetal` entry back.

use ck_json::Json;

fn call(msg: &Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg.clone())]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn part() -> Json {
    ck_json::parse(
        r#"{"format":"ck-part","version":1,"features":[
        {"type":"sheetRules","id":"R1","name":"R","thickness":2,"kFactor":0.44,"bendRadius":2},
        {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":100,"y":0},{"id":"p3","x":100,"y":60},{"id":"p4","x":0,"y":60}],
         "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
        {"type":"baseFlange","id":"B1","sketch":"S1"}]}"#,
    )
    .unwrap()
}

#[test]
fn flat_pattern_exports_over_the_wire() {
    ck_wasm::engine::install();
    let msg = Json::obj().with("type", "regen").with("tag", "sx").with("part", part());
    let r = call(&msg);
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let res = r.get("result").unwrap();
    let entry = res.get_arr("sheetMetal").unwrap().iter().find(|e| e.get("flatPattern").is_some()).expect("the base flange entry").clone();
    let fp = entry.get("flatPattern").unwrap().clone();

    let r = call(&Json::obj().with("type", "sheetDxf").with("flatPattern", fp.clone()).with("units", "in").with("version", "R12"));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let res = r.get("result").unwrap();
    assert!(res.get_str("dxf").unwrap().contains("AC1009"));
    assert_eq!(res.get("entities").unwrap().get_f64("CUT_OUTER"), Some(4.0));
    assert_eq!(res.get_f64("bends"), Some(0.0));

    let r = call(&Json::obj().with("type", "sheetBendTable").with("flatPattern", fp).with("format", "json"));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let text = r.get("result").unwrap().get_str("text").unwrap().to_string();
    let j = ck_json::parse(&text).unwrap();
    assert_eq!(j.get_f64("thickness"), Some(2.0));
    assert!(j.get_arr("bends").unwrap().is_empty());

    // a missing flat pattern is an error that says what to pass
    let r = call(&Json::obj().with("type", "sheetDxf"));
    assert_eq!(r.get("ok"), Some(&Json::Bool(false)));
    assert!(r.get_str("error").unwrap().contains("flatPattern"), "{}", ck_json::to_string(&r));
}
