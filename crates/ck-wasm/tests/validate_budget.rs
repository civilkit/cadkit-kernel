//! `validate` time budget (`opts.validateTimeMs`), per-check `stats` and the quick mode (`opts.quick`), through the full server path.
use ck_json::Json;
use std::cell::Cell;

thread_local! { static NOW: Cell<f64> = const { Cell::new(0.0) }; }
/// Advances 5 ms per reading: a budget of a few ms runs out after a handful of clock reads.
fn ticking() -> f64 {
    NOW.with(|t| {
        let v = t.get() + 5.0;
        t.set(v);
        v
    })
}

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

/// A 40 x 20 plate with a few holes: a handful of faces.
fn load(tag: &str) {
    let mut pts = String::from(r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}"#);
    let mut ents = String::from(r#"{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}"#);
    for i in 0..4 {
        pts += &format!(r#",{{"id":"c{i}","x":{},"y":10}}"#, 8 + 8 * i);
        ents += &format!(r#",{{"id":"k{i}","type":"circle","c":"c{i}","r":2}}"#);
    }
    let part = format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":5}}]}}"#
    );
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part}}}"#));
    assert_eq!(r.get_arr("errors").map_or(0, |e| e.len()), 0, "{}", ck_json::to_string(&r));
}

fn names(j: &Json, key: &str) -> Vec<String> {
    j.get_arr(key).unwrap().iter().map(|x| x.as_str().unwrap().to_string()).collect()
}

#[test]
fn normal_part_is_unchanged_and_reports_stats() {
    load("vb1");
    let v = call(r#"{"type":"validate","tag":"vb1"}"#);
    assert_eq!(v.get("ok"), Some(&Json::Bool(true)));
    assert!(v.get_arr("issues").unwrap().is_empty());
    assert!(v.get("validateBudgetExceeded").is_none() && v.get("quick").is_none());
    let st = v.get("stats").unwrap();
    for k in ["totalMs", "shell", "loops", "edges", "solids", "tessellation", "meshAreaWitness", "meshOpenEdges"] {
        assert!(st.get_f64(k).is_some(), "stats.{k}: {}", ck_json::to_string(st));
    }
    // a generous explicit budget and 0 (unlimited) give the same answer
    for o in [r#"{"validateTimeMs":600000}"#, r#"{"validateTimeMs":0}"#] {
        let w = call(&format!(r#"{{"type":"validate","tag":"vb1","opts":{o}}}"#));
        assert_eq!(w.get("ok"), Some(&Json::Bool(true)));
        assert!(w.get("validateBudgetExceeded").is_none());
    }
}

#[test]
fn tiny_budget_returns_a_partial_result_and_the_notice() {
    load("vb2");
    ck_server::clock::set_now(ticking);
    let v = call(r#"{"type":"validate","tag":"vb2","opts":{"validateTimeMs":12}}"#);
    ck_server::clock::set_now(|| 0.0);
    assert_eq!(v.get("ok"), Some(&Json::Bool(false)), "a cut-short run is not ok: {}", ck_json::to_string(&v));
    let n = v.get("validateBudgetExceeded").unwrap_or_else(|| panic!("{}", ck_json::to_string(&v)));
    assert_eq!(n.get_f64("budgetMs"), Some(12.0));
    let not_run = names(n, "checksNotRun");
    assert!(not_run.contains(&"solids".to_string()) || not_run.contains(&"meshOpenEdges".to_string()), "{not_run:?}");
    assert!(v.get("issues").is_some() && v.get("stats").is_some());
}

#[test]
fn quick_runs_only_topology_and_bounds() {
    load("vb3");
    let v = call(r#"{"type":"validate","tag":"vb3","opts":{"quick":true}}"#);
    assert_eq!(v.get("ok"), Some(&Json::Bool(true)));
    assert_eq!(v.get("quick"), Some(&Json::Bool(true)));
    let skipped = names(&v, "checksSkipped");
    for k in ["loopCrossings", "solids", "tessellation", "meshAreaWitness", "meshOpenEdges"] {
        assert!(skipped.contains(&k.to_string()), "{skipped:?}");
    }
    let st = v.get("stats").unwrap();
    assert!(st.get("tessellation").is_none() && st.get_f64("shell").is_some());
    assert!(v.get("validateBudgetExceeded").is_none());
}
