//! A chamfer that leaves a trim far off its cone (the body is invalid, `EDGE_OFF_SURFACE`) used to be meshed with ~23,000
//! triangles on that one face (refinement never came within the tolerance and spent its whole budget), and `validate`
//! then spent about a second on the exact area of the same face. The mesh stays small and the reply stays complete.
use ck_json::Json;

fn call(msg: &str) -> (Json, usize) {
    ck_wasm::engine::install();
    let r = ck_server::handle(format!(r#"{{"id":1,"msg":{msg}}}"#).as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    (ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap(), r.len())
}

#[test]
fn a_trim_far_off_its_surface_does_not_exhaust_the_refinement_budget() {
    let part = include_str!("fixtures/chamfer_off_surface.json");
    let (r, bytes) = call(&format!(r#"{{"type":"regen","tag":"main","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    let tris = res.get("body").and_then(|b| b.get("mesh")).and_then(|m| m.get("triFace")).and_then(|t| t.get_f64("len")).expect("mesh");
    assert!(tris < 8000.0, "{tris} triangles (was 23490)");
    assert!(bytes < 1_500_000, "{bytes} bytes (was 3.5 MB)");
    // the reply is still complete: the feature that broke the body is reported and validate still answers
    let (v, _) = call(r#"{"type":"validate","tag":"main"}"#);
    assert!(v.get("result").and_then(|r| r.get_arr("issues")).is_some_and(|i| !i.is_empty()), "an invalid body is still reported invalid");
}
