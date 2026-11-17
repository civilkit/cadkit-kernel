//! `faceRun` through the full server path: area centre, outward normal, reach and the feature end of flat faces.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    let j = ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap();
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

#[test]
fn face_run_of_a_rod_end() {
    let part = r#"{"format":"ck-part","version":1,"features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}]}"#;
    let r = call(&format!(r#"{{"type":"regen","tag":"fr","part":{part}}}"#));
    assert_eq!(r.get_arr("errors").map_or(0, |e| e.len()), 0);
    let r = call(r#"{"type":"faceRun","tag":"fr","faces":["F2.C1","F2.C0","F2.S.c1"]}"#);
    let items = r.get_arr("items").unwrap();
    assert_eq!(items.len(), 3);
    assert!((items[0].get_f64("reach").unwrap() - 5.0).abs() < 1e-9);
    assert_eq!(items[0].get("normal").and_then(|p| p.as_vec3()), Some([0.0, 1.0, 0.0]));
    assert_eq!(items[0].get("endOf").unwrap().get_str("kind"), Some("extrude"));
    assert_eq!(items[0].get("endOf").unwrap().get_str("input"), Some("depth"));
    assert_eq!(items[1].get("endOf"), Some(&Json::Null));
    assert_eq!(items[2].get("flat"), Some(&Json::Bool(false)));
}
