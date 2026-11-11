//! `builtExport` and the document's `built` table (docs/regen-cache.md, "Saved bodies"), through the request protocol.
//! Fixture: a part in crates/ck-sldprt/tests/fixtures.

use ck_json::Json;

fn request(msg: Vec<(String, Json)>) -> Json {
    ck_wasm::engine::install();
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), Json::Obj(msg))]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn doc(file: &[u8], built: &str) -> Json {
    let text = ck_text::base64_encode(file);
    let key = ck_model::features::import_step::blob_key(&text);
    let d = format!(
        r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":"{text}"}},"built":{{{built}}},"features":[{{"type":"importBody","id":"F1","format":"sldprt","blob":"{key}","replace":true}}]}}"#
    );
    ck_json::parse(&d).unwrap()
}

fn regen(part: Json) -> Json {
    // noCache: the regen state cache would otherwise answer the second document without running the feature
    let opts = Json::Obj(vec![("noCache".into(), Json::Bool(true))]);
    let r = request(vec![("type".into(), "regen".into()), ("tag".into(), "t".into()), ("part".into(), part), ("opts".into(), opts)]);
    r.get("result").unwrap_or_else(|| panic!("no result: {}", ck_json::to_string(&r))).clone()
}

fn entries() -> Vec<Json> {
    let r = request(vec![("type".into(), "builtExport".into())]);
    let res = r.get("result").unwrap_or_else(|| panic!("no result: {}", ck_json::to_string(&r)));
    assert_eq!(res.get("ok").and_then(|v| v.as_bool()), Some(true));
    res.get("entries").and_then(|e| e.as_arr()).unwrap().to_vec()
}

#[test]
fn export_then_reopen_gives_the_same_body_from_the_saved_entry() {
    let file = std::fs::read(format!("{}/../ck-sldprt/tests/fixtures/sw2022_cube_hole_5mm.sldprt", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let a = regen(doc(&file, ""));
    assert!(a.get("errors").and_then(|e| e.as_arr()).unwrap().is_empty(), "{}", ck_json::to_string(&a));
    let e = entries();
    let e = e.last().unwrap();
    assert_eq!(e.get_str("source"), Some("built"));
    assert_eq!(e.get_str("feature"), Some("F1"));
    let (key, data) = (e.get_str("key").unwrap().to_string(), e.get_str("data").unwrap().to_string());
    assert_eq!(key.len(), 64);

    let b = regen(doc(&file, &format!(r#""{key}":"{data}""#)));
    let e = entries();
    assert_eq!(e.last().unwrap().get_str("source"), Some("saved"));
    assert_eq!(ck_json::to_string(a.get("body").unwrap()), ck_json::to_string(b.get("body").unwrap()), "the view of the saved body differs");
    assert_eq!(ck_json::to_string(a.get("errors").unwrap()), ck_json::to_string(b.get("errors").unwrap()));

    // a damaged entry: rebuilt, same view, and the entry says why it was not used
    let bad = format!("{}A", &data[..data.len() - 8]);
    let c = regen(doc(&file, &format!(r#""{key}":"{bad}""#)));
    let e = entries();
    let last = e.last().unwrap();
    assert_eq!(last.get_str("source"), Some("built"));
    assert!(last.get_str("rejected").is_some());
    assert_eq!(ck_json::to_string(a.get("body").unwrap()), ck_json::to_string(c.get("body").unwrap()));

    // `feature` narrows the answer
    let r = request(vec![("type".into(), "builtExport".into()), ("feature".into(), "nope".into())]);
    assert!(r.get("result").and_then(|x| x.get("entries")).and_then(|e| e.as_arr()).unwrap().is_empty());
}
