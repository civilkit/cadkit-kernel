//! `distance` / `maxDistance` / `thickness` / `closestPoint` / `angles` / `clearance` / `symmetry` through the full
//! server path on sketch + extrude parts (hand-computed values).

use ck_json::Json;

fn raw(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn call(msg: &str) -> Json {
    let j = raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

fn call_err(msg: &str) -> String {
    let j = raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(false)));
    j.get_str("error").unwrap().to_string()
}

fn poly(pts: &[(f64, f64)]) -> (String, String) {
    let n = pts.len();
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let e: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    (p.join(","), e.join(","))
}

fn part(pts: &str, ents: &str, depth: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}]}}"#
    )
}

fn load(tag: &str, p: &str) {
    call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{p}}}"#));
}

fn rect_part(w: f64, h: f64, d: f64) -> String {
    let (p, e) = poly(&[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]);
    part(&p, &e, d)
}

fn num(j: &Json, k: &str) -> f64 {
    j.get_f64(k).unwrap_or_else(|| panic!("no {k}: {}", ck_json::to_string(j)))
}

#[test]
fn distances_between_tags_and_entities() {
    load("m1", &rect_part(40.0, 30.0, 20.0));
    load("m2", &rect_part(40.0, 30.0, 25.0));
    // the tall box sticks out 5 above the short one; both directions
    let r = call(r#"{"type":"maxDistance","tag":"m2","tagB":"m1","opts":{"samples":4000,"both":true}}"#);
    assert!((num(&r, "hausdorff") - 5.0).abs() < 1e-6, "{}", ck_json::to_string(&r));
    assert!((num(r.get("aToB").unwrap(), "maxDistance") - 5.0).abs() < 1e-6);
    let r = call(r#"{"type":"distance","tag":"m1","a":[0,100,0],"b":"body"}"#);
    assert!(num(&r, "distance") > 70.0);
    let r = call(r#"{"type":"distance","tag":"m1","a":"body","b":"body","tagB":"m2"}"#);
    assert_eq!(num(&r, "distance"), 0.0);
    assert_eq!(r.get("intersects"), Some(&Json::Bool(true)));
    assert!(call_err(r#"{"type":"distance","tag":"m1","a":"nope","b":"body"}"#).contains("unknown entity"));
    assert!(call_err(r#"{"type":"distance","tag":"m1","a":"body"}"#).contains("required"));
    assert!(call_err(r#"{"type":"distance","tag":"m1","a":"body","b":"body","tagB":"zzz"}"#).contains("tagB"));
}

#[test]
fn l_bracket_thickness_and_angles() {
    let (p, e) = poly(&[(0.0, 0.0), (40.0, 0.0), (40.0, 3.0), (3.0, 3.0), (3.0, 30.0), (0.0, 30.0)]);
    load("l", &part(&p, &e, 10.0));
    let r = call(r#"{"type":"thickness","tag":"l"}"#);
    assert!((num(&r, "minThickness") - 3.0).abs() < 1e-6, "{}", ck_json::to_string(&r));
    assert!(r.get_arr("thin").unwrap().len() >= 1);
    let r = call(r#"{"type":"angles","tag":"l","edges":"all"}"#);
    let es = r.get_arr("edges").unwrap();
    let concave = es.iter().filter(|e| e.get_str("type") == Some("concave")).count();
    // the inside corner of the L: one concave edge of 270 degrees through the material
    assert_eq!(concave, 1, "{}", ck_json::to_string(&r));
    for e in es {
        let a = num(e, "angle");
        assert!((a - 90.0).abs() < 1e-6 || (a - 270.0).abs() < 1e-6, "{}", ck_json::to_string(e));
    }
}

#[test]
fn hole_to_edge_clearance() {
    // 40 x 30 plate 8 thick, hole radius 4 centred 10 from the left edge: wall 6 to the edge
    let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":30},{"id":"p4","x":0,"y":30},{"id":"c","x":10,"y":15}"#;
    let ents = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"},{"type":"circle","id":"e5","c":"c","r":4}"#;
    load("h", &part(pts, ents, 8.0));
    let r = call(r#"{"type":"clearance","tag":"h","min":7}"#);
    let pairs = r.get_arr("pairs").unwrap();
    assert!(!pairs.is_empty(), "{}", ck_json::to_string(&r));
    assert!((num(&pairs[0], "distance") - 6.0).abs() < 1e-6, "{}", ck_json::to_string(&r));
    assert_eq!(pairs[0].get_str("kind"), Some("wall"));
    let r = call(r#"{"type":"clearance","tag":"h","min":5}"#);
    assert_eq!(num(&r, "count"), 0.0);
    let r = call(r#"{"type":"thickness","tag":"h","opts":{"method":"sphere"}}"#);
    assert!(num(&r, "minThickness") <= 8.0 + 1e-6);
}

#[test]
fn symmetric_box_and_closest() {
    load("s", &rect_part(40.0, 30.0, 20.0));
    let r = call(r#"{"type":"symmetry","tag":"s"}"#);
    assert!(r.get_arr("symmetric").unwrap().len() >= 3, "{}", ck_json::to_string(&r));
    let r = call(r#"{"type":"closestPoint","tag":"s","points":[[-5,10,-5],[100,100,100]],"on":"vertex"}"#);
    assert_eq!(r.get_arr("results").unwrap().len(), 2);
    let r = call(r#"{"type":"closestPoint","tag":"s","points":[[-5,10,-5]]}"#);
    let first = &r.get_arr("results").unwrap()[0];
    assert!(first.get("signedDistance").is_some());
    assert!(call_err(r#"{"type":"closestPoint","tag":"s"}"#).contains("points"));
    assert!(call_err(r#"{"type":"clearance","tag":"s"}"#).contains("min"));
    assert!(call_err(r#"{"type":"angles","tag":"s"}"#).contains("edges"));
}
