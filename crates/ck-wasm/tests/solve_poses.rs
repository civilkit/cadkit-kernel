//! `solvePoses` through the full server path: a hinged cylinder driven through angles, exact placements.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    let j = ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap();
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

const CYL: &str = r#"{"format":"ck-part","version":1,"features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}]}"#;

fn doc() -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
 {{"type":"insertPart","id":"F1","name":"F1","part":{CYL},"fixed":true}},
 {{"type":"insertPart","id":"F2","name":"F2","part":{CYL},"dx":100,"dy":7,"dz":3}},
 {{"type":"mate","id":"F3","kind":"hinge","a":"F2.S.e1@F2.1","b":"F2.S.e1@F1.1","a2":"F2.C0@F2.1","b2":"F2.C1@F1.1"}}]}}"#
    )
}

fn place_x_axis(pose: &Json, id: &str) -> [f64; 3] {
    let c = pose.get_arr("places").unwrap().iter().find(|c| c.get_str("id") == Some(id)).unwrap();
    // frame {o, x, y, z}: the image of the x axis
    c.get("place").unwrap().get("x").and_then(|v| v.as_vec3()).unwrap()
}

#[test]
fn solve_poses_drives_a_hinge() {
    let r = call(&format!(r#"{{"type":"regen","tag":"sp","part":{}}}"#, doc()));
    assert_eq!(r.get_arr("errors").map_or(0, |e| e.len()), 0, "{}", ck_json::to_string(&r));
    let r = call(r#"{"type":"solvePoses","tag":"sp","poses":[{"F3":0},{"F3":90},{"F3":180}]}"#);
    let drivers = r.get_arr("drivers").unwrap();
    assert_eq!(drivers.len(), 1);
    assert_eq!(drivers[0].get_str("mate"), Some("F3"));
    assert_eq!(drivers[0].get_str("unit"), Some("deg"));
    let ps = r.get_arr("poses").unwrap();
    assert_eq!(ps.len(), 3);
    assert!(ps.iter().all(|p| p.get("ok") == Some(&Json::Bool(true))), "{}", ck_json::to_string(&r));
    let ex = |i: usize| place_x_axis(&ps[i], "F2");
    let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6);
    assert!(near(ex(0), [1.0, 0.0, 0.0]), "{:?}", ex(0));
    assert!(near(ex(1), [0.0, 0.0, -1.0]) || near(ex(1), [0.0, 0.0, 1.0]), "{:?}", ex(1));
    assert!(near(ex(2), [-1.0, 0.0, 0.0]), "{:?}", ex(2));
    // errors are plain
    let e = call(r#"{"type":"solvePoses","tag":"sp","poses":[{"Nope":1}]}"#);
    assert_eq!(e.get_str("error"), Some("unknownDriver"));
    assert_eq!(e.get_str("mate"), Some("Nope"));
}

const RAIL: &str = r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":60,"y":0},{"id":"p3","x":60,"y":30},{"id":"p4","x":0,"y":30}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
{"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"q1","x":25,"y":5},{"id":"q2","x":35,"y":5},{"id":"q3","x":35,"y":25},{"id":"q4","x":25,"y":25}],
"entities":[{"type":"line","id":"e1","p1":"q1","p2":"q2"},{"type":"line","id":"e2","p1":"q2","p2":"q3"},{"type":"line","id":"e3","p1":"q3","p2":"q4"},{"type":"line","id":"e4","p1":"q4","p2":"q1"}],"constraints":[]},
{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":10}]}"#;

#[test]
fn sweep_driver_finds_the_slot_end_and_solve_poses_reports_the_contact() {
    let doc = format!(
        r#"{{"format":"ck-part","version":1,"features":[
 {{"type":"insertPart","id":"F1","name":"Rail","part":{RAIL},"fixed":true}},
 {{"type":"insertPart","id":"F2","name":"Pin","part":{CYL_PIN},"dx":30,"dy":0,"dz":-15}},
 {{"type":"mate","id":"F3","kind":"slot","a":"F2.S.e1@F2.1","b":"F4.S.e4@F1.1","b2":"F4.S.e2@F1.1"}}]}}"#,
        CYL_PIN = r#"{"format":"ck-part","version":1,"features":[
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":2}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}]}"#
    );
    let r = call(&format!(r#"{{"type":"regen","tag":"sw","part":{doc}}}"#));
    assert_eq!(r.get_arr("errors").map_or(0, |e| e.len()), 0, "{}", ck_json::to_string(&r));
    let s = call(r#"{"type":"sweepDriver","tag":"sw","driver":"F3","from":0,"to":30,"tol":0.005}"#);
    let stop = s.get("stop").unwrap();
    assert_eq!(stop.get_str("reason"), Some("contact"), "{}", ck_json::to_string(&s));
    assert!((s.get_f64("reached").unwrap().abs() - 8.0).abs() < 0.02, "{}", ck_json::to_string(&s));
    let pair = [stop.get_str("a").unwrap(), stop.get_str("b").unwrap()];
    assert!(pair.contains(&"F1") && pair.contains(&"F2"));
    // a pose past the end reports the contact; one inside does not
    let p = call(r#"{"type":"solvePoses","tag":"sw","poses":[{"F3":5},{"F3":12}]}"#);
    let ps = p.get_arr("poses").unwrap();
    assert!(ps[1].get("contact").is_some(), "{}", ck_json::to_string(&p));
    assert_eq!(ps[1].get("contact").unwrap().get_str("kind"), Some("contact"));
    // contact off: no field
    let p = call(r#"{"type":"solvePoses","tag":"sw","contact":false,"poses":[{"F3":12}]}"#);
    assert!(p.get_arr("poses").unwrap()[0].get("contact").is_none());
    let e = call(r#"{"type":"sweepDriver","tag":"sw","driver":"Nope","to":1}"#);
    assert_eq!(e.get_str("error"), Some("unknownDriver"));
}
