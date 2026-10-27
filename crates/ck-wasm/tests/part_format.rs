//! The part format's old names (`formwerk-part`, `fw-part`) through the full server path:
//! `regen` and a nested `insertPart` both read it as `ck-part`. (There is no
//! `open` request type; the kit's file-open path normalises the name itself.)

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

/// 40x20x10 plate on the top plane, written with `format`.
fn plate(format: &str) -> String {
    format!(
        r#"{{"format":"{format}","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}]}}"#
    )
}

#[test]
fn regen_reads_the_fw_format_name() {
    let old = call(&format!(r#"{{"type":"regen","tag":"t","part":{}}}"#, plate("fw-part")));
    let new = call(&format!(r#"{{"type":"regen","tag":"t","part":{}}}"#, plate("ck-part")));
    assert_eq!(old.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&old));
    assert_eq!(old.get("result"), new.get("result"), "{}", ck_json::to_string(&old));
}

#[test]
fn regen_reads_the_old_format_name() {
    let old = call(&format!(r#"{{"type":"regen","tag":"t","part":{}}}"#, plate("formwerk-part")));
    let new = call(&format!(r#"{{"type":"regen","tag":"t","part":{}}}"#, plate("ck-part")));
    assert_eq!(old.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&old));
    assert_eq!(old.get("result"), new.get("result"), "{}", ck_json::to_string(&old));
    let res = old.get("result").unwrap();
    assert_eq!(res.get_arr("errors").unwrap().len(), 0);
    assert!(res.get("body").unwrap().is_obj());
}

/// A nested `insertPart` part with the old name is read too (the nested part
/// goes through `Part::from_json`, not `check_part`).
#[test]
fn insert_part_reads_the_old_format_name() {
    let part = format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"insertPart","id":"F1","name":"c","part":{},"fixed":true,"dx":50}}]}}"#,
        plate("formwerk-part")
    );
    let r = call(&format!(r#"{{"type":"regen","tag":"t","part":{part}}}"#));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let res = r.get("result").unwrap();
    assert_eq!(res.get_arr("errors").unwrap().len(), 0, "{}", ck_json::to_string(&r));
    assert!(res.get("body").unwrap().is_obj());
}
