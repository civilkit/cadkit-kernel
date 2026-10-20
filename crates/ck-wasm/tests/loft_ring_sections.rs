//! Loft between sections that carry inner loops (#1184): the solid is the hollow tube (the loft of
//! the outlines minus the loft of the matching inner loops), not the filled cylinder of the outer
//! circles; sections whose region counts disagree are refused with a detail and a hint.
use ck_json::Json;
use std::f64::consts::PI;

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

/// The #1184 part: two coaxial ring sections (outer r20, inner r12) 20 apart.
fn ring_part(tag: &str) -> (Vec<Json>, f64, usize) {
    let part = format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},
         "points":[{{"id":"p1","x":0,"y":0}}],
         "entities":[{{"id":"e1","type":"circle","c":"p1","r":20}},{{"id":"e2","type":"circle","c":"p1","r":12}}],"constraints":[]}},
        {{"type":"plane","id":"F2","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":20,"visible":false}},
        {{"type":"sketch","id":"S3","plane":{{"kind":"feature","id":"F2"}},
         "points":[{{"id":"p1","x":0,"y":0}}],
         "entities":[{{"id":"e1","type":"circle","c":"p1","r":20}},{{"id":"e2","type":"circle","c":"p1","r":12}}],"constraints":[]}},
        {{"type":"loft","id":"L","op":"add","sketches":["S1","S3"]}}]}}"#
    );
    run(&part, tag)
}

/// Regen `part`, then mass properties + validate; returns (errors, volume, validate issues).
fn run(part: &str, tag: &str) -> (Vec<Json>, f64, usize) {
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    let errors: Vec<Json> = res.get_arr("errors").map(|a| a.to_vec()).unwrap_or_default();
    let m = call(&format!(r#"{{"type":"massProps","tag":"{tag}"}}"#));
    let v = call(&format!(r#"{{"type":"validate","tag":"{tag}"}}"#));
    let issues = v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len());
    (errors, m.get("result").map_or(f64::NAN, |m| num(m, "volume")), issues)
}

#[test]
fn ring_sections_loft_a_hollow_tube() {
    let (errors, vol, issues) = ring_part("ring");
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(issues, 0, "validate issues");
    let want = PI * (20.0f64.powi(2) - 12.0f64.powi(2)) * 20.0;
    let rel = (vol - want).abs() / want;
    assert!(rel < 1e-3, "volume {vol} vs hollow tube {want} (rel {rel})");
}

#[test]
fn ring_sections_with_a_point_apex_loft_a_hollow_cone() {
    let part = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},
         "points":[{"id":"p1","x":0,"y":0}],
         "entities":[{"id":"e1","type":"circle","c":"p1","r":20},{"id":"e2","type":"circle","c":"p1","r":12}],"constraints":[]},
        {"type":"plane","id":"F2","mode":"offset","base":{"kind":"std","id":"front"},"offset":30,"visible":false},
        {"type":"sketch","id":"S3","plane":{"kind":"feature","id":"F2"},
         "points":[{"id":"p","x":0,"y":0}],
         "entities":[{"id":"e1","type":"point","p":"p"}],"constraints":[]},
        {"type":"loft","id":"L","op":"add","sketches":["S1","S3"]}]}"#;
    let (errors, vol, issues) = run(part, "cone");
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(issues, 0, "validate issues");
    let want = PI / 3.0 * 30.0 * (20.0f64.powi(2) - 12.0f64.powi(2));
    let rel = (vol - want).abs() / want;
    assert!(rel < 1e-3, "volume {vol} vs hollow cone {want} (rel {rel})");
}

#[test]
fn sections_with_different_inner_loops_refuse_with_detail_and_hint() {
    let part = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},
         "points":[{"id":"p1","x":0,"y":0}],
         "entities":[{"id":"e1","type":"circle","c":"p1","r":20},{"id":"e2","type":"circle","c":"p1","r":12}],"constraints":[]},
        {"type":"plane","id":"F2","mode":"offset","base":{"kind":"std","id":"front"},"offset":20,"visible":false},
        {"type":"sketch","id":"S3","plane":{"kind":"feature","id":"F2"},
         "points":[{"id":"p1","x":0,"y":0}],
         "entities":[{"id":"e1","type":"circle","c":"p1","r":20}],"constraints":[]},
        {"type":"loft","id":"L","op":"add","sketches":["S1","S3"]}]}"#;
    let (errors, _vol, _issues) = run(part, "mism");
    assert_eq!(errors.len(), 1, "{errors:?}");
    let arr = errors[0].as_arr().expect("error is [id, {...}]");
    let obj = arr.get(1).expect("error object");
    assert_eq!(obj.get_str("code"), Some("loftProfile"), "{obj:?}");
    let params = obj.get("params");
    let detail = params.and_then(|p| p.get_str("detail")).unwrap_or_default();
    let hint = params.and_then(|p| p.get_str("hint")).unwrap_or_default();
    assert!(!detail.is_empty(), "missing detail: {obj:?}");
    assert!(!hint.is_empty(), "missing hint: {obj:?}");
}
