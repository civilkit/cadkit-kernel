//! `meshToSolid` through the full server path: a tessellated drilled block comes in as a triangle soup (JSON numbers and
//! base64 Float32), is stored in the tag as a solid with one cylinder face, and answers validate and massProps; an open
//! mesh is refused with positions and leaves the tag alone.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn result(r: &Json) -> &Json {
    r.get("result").unwrap_or(r)
}

/// A 60 x 40 x 10 block with a 6 mm through hole, tessellated: 9 f32 values per triangle.
fn drilled_block_soup() -> Vec<f32> {
    let src = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
    {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
    {"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"k","x":5,"y":3}],"entities":[{"id":"pt1","type":"point","p":"k"}],"constraints":[]},
    {"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}]"#;
    let feats = match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => unreachable!(),
    };
    let (b, errs) = ck_recog::rebuild::rebuild(&feats).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let b = b.unwrap();
    let (lo, hi) = b.bbox();
    ck_tess::mesh::body_mesh(&b, &ck_tess::curvature::Tol::for_bbox(lo, hi)).positions
}

fn b64(bytes: &[u8]) -> String {
    ck_text::base64_encode(bytes)
}

#[test]
fn soup_in_solid_out() {
    let soup = drilled_block_soup();
    let nums: Vec<String> = soup.iter().map(|v| format!("{v}")).collect();
    let r = call(&format!(r#"{{"type":"meshToSolid","tag":"m","name":"block","positions":[{}]}}"#, nums.join(",")));
    let res = result(&r);
    assert_eq!(res.get_arr("errors").map(|a| a.len()), Some(0), "{}", ck_json::to_string(res).chars().take(600).collect::<String>());
    let imp = res.get("import").expect("import report");
    assert_eq!(imp.get_str("format"), Some("mesh"));
    assert_eq!(ck_json::to_string(imp.get("faceTypes").unwrap()), r#"{"cylinder":1,"plane":6}"#);
    assert_eq!(imp.get_f64("issues"), Some(0.0));
    assert!(imp.path(&["analytic", "applied"]).and_then(|v| v.as_bool()).unwrap());
    let v = imp.path(&["volume", "body"]).and_then(|v| v.as_f64()).unwrap();
    let v_true = 60.0 * 40.0 * 10.0 - std::f64::consts::PI * 9.0 * 10.0;
    assert!((v - v_true).abs() < 1e-3 * v_true, "{v} vs {v_true}");
    // the tag holds the solid
    let val = call(r#"{"type":"validate","tag":"m"}"#);
    let vs = ck_json::to_string(result(&val));
    assert!(vs.contains(r#""ok":true"#) || vs.contains(r#""valid":true"#), "{}", vs.chars().take(400).collect::<String>());
    let mp = call(r#"{"type":"massProps","tag":"m"}"#);
    assert!(ck_json::to_string(result(&mp)).contains("volume"));
}

#[test]
fn base64_float32_with_indices_and_options() {
    let soup = drilled_block_soup();
    let bytes: Vec<u8> = soup.iter().flat_map(|v| v.to_le_bytes()).collect();
    let n = (soup.len() / 3) as u32;
    let ix: Vec<u8> = (0..n).flat_map(|i| i.to_le_bytes()).collect();
    let r = call(&format!(r#"{{"type":"meshToSolid","tag":"m2","positionsF32":"{}","indicesU32":"{}","opts":{{"analytic":false,"deviation":false}}}}"#, b64(&bytes), b64(&ix)));
    let imp = result(&r).get("import").expect("import report");
    assert!(imp.get("analytic").is_none());
    assert_eq!(imp.path(&["faceTypes", "plane"]).and_then(|v| v.as_f64()), imp.get_f64("faces"));
}

#[test]
fn open_mesh_is_refused() {
    let mut soup = drilled_block_soup();
    soup.truncate(soup.len() - 9);
    let nums: Vec<String> = soup.iter().map(|v| format!("{v}")).collect();
    let r = call(&format!(r#"{{"type":"meshToSolid","tag":"m3","positions":[{}]}}"#, nums.join(",")));
    let res = result(&r);
    let e = &res.get_arr("errors").unwrap()[0];
    let e = e.as_arr().unwrap();
    assert_eq!(e[0].as_str(), Some("import"));
    assert_eq!(e[1].get_str("code"), Some("meshNotWatertight"));
    assert_eq!(e[1].path(&["params", "openEdges"]).and_then(|v| v.as_f64()), Some(3.0));
    assert!(e[1].path(&["params", "hint"]).is_some());
    assert!(res.get("body").map_or(true, |b| b.is_null()));
}

#[test]
fn open_mesh_is_closed_on_request() {
    // the drilled block missing its last triangle: refused by default, closed (one 3-edge hole) with opts.close
    let mut soup = drilled_block_soup();
    soup.truncate(soup.len() - 9);
    let nums: Vec<String> = soup.iter().map(|v| format!("{v}")).collect();
    let r = call(&format!(r#"{{"type":"meshToSolid","tag":"m4","positions":[{}],"opts":{{"close":{{"maxHoleEdges":50}}}}}}"#, nums.join(",")));
    let res = result(&r);
    assert_eq!(res.get_arr("errors").map(|a| a.len()), Some(0), "{}", ck_json::to_string(res).chars().take(600).collect::<String>());
    let imp = res.get("import").expect("import report");
    assert_eq!(imp.path(&["closing", "filledHoles"]).and_then(|v| v.as_f64()), Some(1.0));
    assert_eq!(imp.path(&["closing", "filledTriangles"]).and_then(|v| v.as_f64()), Some(1.0));
    assert_eq!(imp.get_f64("issues"), Some(0.0));
    let v = imp.path(&["volume", "body"]).and_then(|v| v.as_f64()).unwrap();
    let v_true = 60.0 * 40.0 * 10.0 - std::f64::consts::PI * 9.0 * 10.0;
    assert!((v - v_true).abs() < 2e-3 * v_true, "{v} vs {v_true}");
}
