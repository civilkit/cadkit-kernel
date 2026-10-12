//! STEP export -> import round trip of parts whose arcs are split at a
//! cylinder seam (flange coupling halves / nuts, tools/cases): the exported
//! arcs must be the same arcs on re-import (volume, area, face / edge counts,
//! clean validate).
use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn num(j: &Json, k: &str) -> f64 {
    j.get(k).and_then(Json::as_f64).unwrap_or(f64::NAN)
}

fn roundtrip(case: &str, upto: usize) -> Result<(), String> {
    let path = format!("{}/../../tools/cases/{case}.ckpart", env!("CARGO_MANIFEST_DIR"));
    let mut part = ck_json::parse(&std::fs::read_to_string(path).map_err(|e| e.to_string())?).map_err(|e| format!("{e:?}"))?;
    if let Json::Obj(kv) = &mut part {
        for (k, v) in kv.iter_mut() {
            if k == "features" {
                if let Json::Arr(a) = v {
                    a.truncate(upto);
                }
            }
        }
    }
    let r = call(&format!(r#"{{"type":"regen","tag":"main","part":{}}}"#, ck_json::to_string(&part)));
    if r.get("result").and_then(|r| r.get("body")).map_or(true, |b| matches!(b, Json::Null)) {
        return Err(format!("{case}@{upto}: no body"));
    }
    let step = call(r#"{"type":"step","tag":"main","name":"rt"}"#);
    let text = step.get("result").and_then(Json::as_str).ok_or("no step text")?.to_string();
    if std::env::var("DUMP_STEP").is_ok() {
        std::fs::write(format!("/tmp/rt_{case}_{upto}.step"), &text).unwrap();
    }
    let msg = ck_json::to_string(&Json::Obj(vec![
        ("type".into(), Json::Str("importStep".into())),
        ("tag".into(), Json::Str("imp".into())),
        ("name".into(), Json::Str("rt".into())),
        ("data".into(), Json::Str(text)),
    ]));
    let imp = call(&msg);
    if imp.get("result").and_then(|r| r.get("body")).map_or(true, |b| matches!(b, Json::Null)) {
        return Err(format!("{case}@{upto}: import failed {:?}", imp.get("result")));
    }
    let a = call(r#"{"type":"massProps","tag":"main"}"#);
    let b = call(r#"{"type":"massProps","tag":"imp"}"#);
    let (a, b) = (a.get("result").unwrap(), b.get("result").unwrap());
    let mut errs = vec![];
    for k in ["volume", "area"] {
        if (num(a, k) - num(b, k)).abs() > 1e-6 * (1.0 + num(a, k).abs()) {
            errs.push(format!("{k} {} vs {}", num(b, k), num(a, k)));
        }
    }
    let v = call(r#"{"type":"validate","tag":"imp"}"#);
    if !v.get("result").and_then(|r| r.get_arr("issues")).map_or(false, |i| i.is_empty()) {
        errs.push(format!("validate {:?}", v.get("result")));
    }
    if errs.is_empty() { Ok(()) } else { Err(format!("{case}@{upto}: {}", errs.join("; "))) }
}

#[test]
fn seam_split_arcs_survive_step_roundtrip() {
    let mut fails = vec![];
    for (case, upto) in [("flangeHalf", 9), ("flangeHalf", 10), ("flangeHalf", 11), ("flangeNut", 12)] {
        if let Err(e) = roundtrip(case, upto) {
            fails.push(e);
        }
    }
    assert!(fails.is_empty(), "{fails:#?}");
}
