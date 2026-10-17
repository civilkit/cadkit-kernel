//! `massProps` density wire: per-body `mass` when `densities` is given,
//! `densities` alone implies `bodies:true`, and a length mismatch is an
//! error naming the expected count. No-densities replies stay oracle-pinned.

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

/// Two disjoint 10×10×10 boxes (two lumps, each volume 1000).
const TWO: &str = r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":0},{"id":"c","x":10,"y":10},{"id":"d","x":0,"y":10}],
 "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
{"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":100,"y":0},{"id":"b","x":110,"y":0},{"id":"c","x":110,"y":10},{"id":"d","x":100,"y":10}],
 "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":10}]}"#;

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * (1.0 + b.abs()), "{a} vs {b}");
}

#[test]
fn densities_imply_bodies_and_add_per_body_mass() {
    let r = call(&format!(r#"{{"type":"regen","tag":"m","part":{TWO}}}"#));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    // densities alone (no bodies:true): implies bodies, byMass present
    let m = call(r#"{"type":"massProps","tag":"m","densities":[1.0,2.0]}"#);
    assert_eq!(m.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&m));
    let res = m.get("result").unwrap();
    let bodies = res.get_arr("bodies").expect("densities implies bodies:true");
    assert_eq!(bodies.len(), 2);
    close(bodies[0].get_f64("mass").unwrap(), 1000.0);
    close(bodies[1].get_f64("mass").unwrap(), 2000.0);
    let bm = res.get("byMass").expect("byMass with densities");
    close(bm.get_f64("mass").unwrap(), 3000.0);
    // mass-weighted CG x: (1000·5 + 2000·105) / 3000 = 71.666…
    close(bm.get("com").unwrap().as_vec3().unwrap()[0], 215.0 / 3.0);
    // no densities: oracle-pinned shape, no mass / byMass
    let n = call(r#"{"type":"massProps","tag":"m"}"#);
    let nres = n.get("result").unwrap();
    assert!(nres.get("byMass").is_none());
    let nb = nres.get_arr("bodies");
    assert!(nb.is_none() || nb.unwrap().iter().all(|e| e.get("mass").is_none()));
    // length mismatch: structured error naming the expected count
    let e = call(r#"{"type":"massProps","tag":"m","densities":[1.0]}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&e));
    let err = e.get_str("error").unwrap();
    assert!(err.contains("1 entries") && err.contains("2 lump"), "{err}");
}
