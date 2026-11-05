//! `sldprtHistory`: the feature history of a SolidWorks part file as a part document, through the request protocol.
//! Fixtures: the parts in crates/ck-sldprt/tests/fixtures. The document must regenerate to the body `importSldprt` reads.

use ck_json::Json;

fn request(msg: Vec<(String, Json)>) -> Json {
    ck_wasm::engine::install();
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), Json::Obj(msg))]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!("{}/../ck-sldprt/tests/fixtures/{name}.sldprt", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

fn history(bytes: &[u8]) -> Json {
    let r = request(vec![("type".into(), "sldprtHistory".into()), ("data".into(), ck_text::base64_encode(bytes).into())]);
    r.get("result").unwrap_or_else(|| panic!("no result: {}", ck_json::to_string(&r))).clone()
}

fn volume(r: &Json) -> f64 {
    r.get("result").and_then(|x| x.get("volume")).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("no volume: {}", ck_json::to_string(r)))
}

#[test]
fn a_rebuildable_part_answers_its_history_and_regenerates_to_the_plain_import() {
    let file = fixture("sw2022_cube_hole_5mm");
    let h = history(&file);
    assert_eq!(h.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", ck_json::to_string(&h));
    assert_eq!(h.get("complete").and_then(|v| v.as_bool()), Some(true));
    let part = h.get("part").unwrap().clone();
    let feats = part.get("features").and_then(|f| f.as_arr()).unwrap();
    assert!(feats.len() > 1, "{} features", feats.len());
    // the document regenerates (tag "h") to the volume of the plain import (tag "p")
    let rg = request(vec![("type".into(), "regen".into()), ("tag".into(), "h".into()), ("part".into(), part)]);
    let errs = rg.get("result").and_then(|r| r.get("errors")).and_then(|e| e.as_arr()).unwrap();
    assert!(errs.is_empty(), "{}", ck_json::to_string(&rg));
    let imp = request(vec![("type".into(), "importSldprt".into()), ("tag".into(), "p".into()), ("data".into(), ck_text::base64_encode(&file).into())]);
    assert!(imp.get("result").is_some());
    let vh = volume(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "h".into())]));
    let vp = volume(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "p".into())]));
    assert!((vh - vp).abs() <= 1e-6 * vp.abs(), "{vh} vs {vp}");
}

#[test]
fn a_file_that_cannot_be_read_answers_ok_false_with_a_reason() {
    let h = history(&[1, 2, 3, 4]);
    assert_eq!(h.get("ok").and_then(|v| v.as_bool()), Some(false));
    assert_eq!(h.get("reason").and_then(|v| v.as_str()), Some("import"));
}

#[test]
fn data_is_required() {
    let r = request(vec![("type".into(), "sldprtHistory".into())]);
    assert!(r.get("result").is_none() || r.get("error").is_some(), "{}", ck_json::to_string(&r));
}
