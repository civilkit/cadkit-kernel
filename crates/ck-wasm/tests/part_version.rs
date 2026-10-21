//! A document with a `version` newer than the kernel's still regenerates; the
//! reply then says so (`partVersionNewer`, docs/spec-agent-interface.md).
//! Replies of documents this kernel knows stay byte identical.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

/// 40x20x10 plate on the top plane, written with `version`.
fn part(version: &str) -> String {
    let version = if version.is_empty() { String::new() } else { format!(",{version}") };
    format!(
        r#"{{"format":"ck-part"{version},"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}]}}"#
    )
}

fn regen(version: &str) -> Json {
    let r = call(&format!(r#"{{"type":"regen","tag":"t","part":{}}}"#, part(version)));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    r.get("result").unwrap().clone()
}

fn without_part_version_newer(mut j: Json) -> Json {
    if let Json::Obj(m) = &mut j {
        m.retain(|(k, _)| k != "partVersionNewer");
    }
    j
}

#[test]
fn a_newer_document_says_so_and_regens_the_same() {
    let known = regen(r#""version":1"#);
    assert!(known.get("partVersionNewer").is_none(), "{}", ck_json::to_string(&known));

    let newer = regen(r#""version":2"#);
    let warn = newer.get("partVersionNewer").expect("partVersionNewer");
    assert_eq!(warn.get_f64("file"), Some(2.0));
    assert_eq!(warn.get_f64("kernel"), Some(1.0));
    assert_eq!(without_part_version_newer(newer), known);
}

#[test]
fn a_missing_version_counts_as_the_kernels() {
    let missing = regen("");
    assert!(missing.get("partVersionNewer").is_none(), "{}", ck_json::to_string(&missing));
    assert_eq!(missing, regen(r#""version":1"#));
}
