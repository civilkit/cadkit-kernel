//! `newBody` through the wire: body names in `massProps {bodies:true}`, overlapping lumps reported by `validate`
//! as `info` (`bodiesOverlap`) and never as an issue (docs/adr/0004-multiple-bodies-in-one-part.md).

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn doc(x1: f64) -> String {
    let sk = |id: &str, x0: f64, x1: f64| {
        format!(
            r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"a","x":{x0},"y":0}},{{"id":"b","x":{x1},"y":0}},{{"id":"c","x":{x1},"y":10}},{{"id":"d","x":{x0},"y":10}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
        )
    };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"S1","end":"blind","depth":10}},{},{{"type":"extrude","id":"F4","op":"newBody","body":"Plate","sketch":"S3","end":"blind","depth":10}}]}}"#,
        sk("S1", 0.0, 10.0),
        sk("S3", x1, x1 + 10.0)
    )
}

#[test]
fn names_in_mass_props_and_overlap_as_info() {
    let r = call(&format!(r#"{{"type":"regen","tag":"nb","part":{}}}"#, doc(5.0)));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let m = call(r#"{"type":"massProps","tag":"nb","bodies":true}"#);
    let bodies = m.get("result").unwrap().get_arr("bodies").unwrap().to_vec();
    assert_eq!(bodies.len(), 2);
    assert_eq!(bodies[0].get_str("name"), None);
    assert_eq!(bodies[1].get_str("name"), Some("Plate"));
    let v = call(r#"{"type":"validate","tag":"nb"}"#);
    let res = v.get("result").unwrap();
    assert_eq!(res.get("ok"), Some(&Json::Bool(true)), "overlap is not an issue: {}", ck_json::to_string(res));
    assert!(res.get_arr("issues").unwrap().is_empty());
    let info = res.get_arr("info").expect("bodiesOverlap info");
    assert_eq!(info.len(), 1);
    assert_eq!(info[0].get_str("code"), Some("bodiesOverlap"));
    assert_eq!(info[0].get_str("severity"), Some("info"));
    assert_eq!(info[0].get_str("bName"), Some("Plate"));
    assert!((info[0].get_f64("volume").unwrap() - 500.0).abs() < 1e-6);
    assert!(info[0].get_str("a").is_some() && info[0].get_str("b").is_some());
}

#[test]
fn disjoint_lumps_have_no_info() {
    let r = call(&format!(r#"{{"type":"regen","tag":"nb2","part":{}}}"#, doc(30.0)));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let v = call(r#"{"type":"validate","tag":"nb2"}"#);
    let res = v.get("result").unwrap();
    assert_eq!(res.get("ok"), Some(&Json::Bool(true)));
    assert!(res.get("info").is_none());
}
