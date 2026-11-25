//! `step` reads its own file back: a body that round-trips gives the bare STEP text and no `warnings`; `verify:false` skips it.
use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

#[test]
fn a_body_that_round_trips_exports_without_a_warning() {
    let path = format!("{}/../../tools/cases/bracket.ckpart", env!("CARGO_MANIFEST_DIR"));
    let part = std::fs::read_to_string(path).unwrap();
    let r = call(&format!(r#"{{"type":"regen","tag":"main","part":{part}}}"#));
    assert!(!matches!(r.get("result").and_then(|r| r.get("body")), None | Some(Json::Null)));
    for q in [r#"{"type":"step","tag":"main"}"#, r#"{"type":"step","tag":"main","verify":false}"#] {
        let s = call(q);
        assert_eq!(s.get("ok"), Some(&Json::Bool(true)));
        assert!(s.get("result").and_then(Json::as_str).unwrap().starts_with("ISO-10303-21"));
        assert!(s.get("warnings").is_none(), "{:?}", s.get("warnings"));
    }
}
