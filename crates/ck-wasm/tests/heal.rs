//! `heal` request + `regen` `opts.autoHeal` through the full server path.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

const PART: &str = r#"{"format":"ck-part","version":1,"features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}]}"#;

fn regen(opts: &str) -> Json {
    call(&format!(r#"{{"type":"regen","tag":"h","part":{PART},"opts":{opts}}}"#))
}

#[test]
fn heal_request_on_valid_body_is_a_noop() {
    regen("{}");
    let r = call(r#"{"type":"heal","tag":"h"}"#);
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)));
    let res = r.get("result").unwrap();
    assert_eq!(res.get_arr("fixed").unwrap().len(), 0);
    assert_eq!(res.get_arr("remaining").unwrap().len(), 0);
}

#[test]
fn heal_self_test_repairs_every_damage() {
    regen("{}");
    for k in ck_heal::damage::KINDS {
        let r = call(&format!(r#"{{"type":"heal","tag":"h","damage":"{k}"}}"#));
        let d = r.get("result").unwrap().get("damage").unwrap();
        if d.get_bool("applied") == Some(true) {
            assert_eq!(d.get_bool("clean"), Some(true), "{k}");
        }
    }
}

#[test]
fn auto_heal_is_off_by_default_and_silent_on_valid_features() {
    for opts in ["{}", r#"{"autoHeal":true}"#] {
        let r = regen(opts);
        assert_eq!(r.get("result").unwrap().get_arr("errors").unwrap().len(), 0, "{opts}");
    }
}
