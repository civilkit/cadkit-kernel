use ck_json::Json;

fn call(body: &str) -> Json {
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

#[test]
#[ignore]
fn dbg() {
    ck_wasm::engine::install();
    let req = std::fs::read_to_string(std::env::var("REQ").unwrap()).unwrap();
    let r = call(&req);
    println!("regen ok: {:?}", r.get("ok"));
    let s = call(r#"{"id":2,"msg":{"type":"step","tag":"main","name":"x"}}"#);
    let _ = s;
}
