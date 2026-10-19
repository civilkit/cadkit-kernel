//! `refDiff` and `matchEdges` continue from the cached state of the prefix the two documents share
//! (docs/regen-cache.md). The warnings must be exactly those of the reference path (`noCache: true`, both documents built
//! from the start). A kernel instance is a thread here (the server and the cache are thread-local).

use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let r = ck_server::handle(format!(r#"{{"id":1,"msg":{msg}}}"#).as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    let j = ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap();
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

/// A 40x20 plate (F1, F2), a sketch on its cap (F3), a fillet of an edge by face names (F4), a second block on the cap
/// (F5, F6) and a second fillet (F7): references into earlier features at several depths of the history.
fn doc(depth: f64, fillet: f64, plane: &str, extra: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"{plane}"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}},
{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"a","x":10,"y":10}}],"entities":[{{"type":"circle","id":"e1","c":"a","r":3}}],"constraints":[]}},
{{"type":"fillet","id":"F4","radius":{fillet},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,0,0]}}]}},
{{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"b","x":30,"y":10}}],"entities":[{{"type":"circle","id":"e1","c":"b","r":4}}],"constraints":[]}},
{{"type":"extrude","id":"F6","op":"add","sketch":"F5","end":"blind","depth":5}},
{{"type":"fillet","id":"F7","radius":0.5,"edges":[{{"faces":["F2.C1","F2.S.e3"],"at":[20,20,0]}}]}}{extra}]}}"#
    )
}

fn feature_json(d: &str, id: &str) -> Json {
    ck_json::parse(d).unwrap().get("features").unwrap().as_arr().unwrap().iter().find(|f| f.get_str("id") == Some(id)).unwrap().clone()
}

fn edit_features(d: &str, f: impl FnOnce(&mut Vec<Json>)) -> String {
    let mut j = ck_json::parse(d).unwrap();
    f(j.get_mut("features").unwrap().as_arr_mut().unwrap());
    ck_json::to_string(&j)
}

fn without(j: &Json, key: &str) -> String {
    let mut j = j.clone();
    if let Json::Obj(kv) = &mut j {
        kv.retain(|(k, _)| k != key);
    }
    ck_json::to_string(&j)
}

/// What the session does: base regenerated, candidate regenerated, then the reference check (both cached).
fn ref_diff(base: &str, part: &str) -> (Json, Json) {
    call(&format!(r#"{{"type":"regen","tag":"main","part":{base}}}"#));
    call(&format!(r#"{{"type":"regen","tag":"main","part":{part}}}"#));
    let fast = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{part},"cacheStats":true}}"#));
    let slow = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{part},"noCache":true}}"#));
    (fast, slow)
}

fn check(name: &str, base: &str, part: &str, expect_resume: bool) -> usize {
    let (fast, slow) = ref_diff(base, part);
    // warnings, features, errors and ok are byte-identical; `checked` only counts references from the first changed feature on
    for key in ["warnings", "features", "errors", "ok"] {
        assert_eq!(fast.get(key).map(ck_json::to_string), slow.get(key).map(ck_json::to_string), "{name}: {key}");
    }
    assert!(fast.get_f64("checked").unwrap() <= slow.get_f64("checked").unwrap(), "{name}: checked");
    let at = fast.get("cacheStats").unwrap().get_f64("resumedAt").unwrap() as usize;
    assert_eq!(at > 0, expect_resume, "{name}: resumedAt {at}");
    fast.get_arr("warnings").unwrap().len()
}

#[test]
fn cached_ref_diff_matches_the_reference_path() {
    std::thread::spawn(|| {
        let base = doc(10.0, 1.0, "top", "");
        // the last feature edited: everything before it is shared
        let last = edit_features(&base, |fs| {
            let n = fs.len();
            if let Json::Obj(kv) = &mut fs[n - 1] {
                kv.iter_mut().find(|(k, _)| k == "radius").unwrap().1 = Json::Num(0.6);
            }
        });
        check("edit last", &base, &last, true);
        // a middle feature (the fillet F4)
        check("edit middle", &base, &doc(10.0, 1.5, "top", ""), true);
        // the extrude depth (F2): only the sketch before it is shared
        check("edit depth", &base, &doc(14.0, 1.0, "top", ""), true);
        // the first sketch moved to another plane re-points the cap (nothing is shared: the first feature changed)
        let w = check("move plane", &base, &doc(10.0, 1.0, "front", ""), false);
        assert!(w > 0, "re-pointed references warn");
        // append a feature
        let extra = format!(r#",{}"#, ck_json::to_string(&feature_json(&doc(10.0, 1.0, "top", ""), "F7")).replace("F7", "F8"));
        check("append", &base, &doc(10.0, 1.0, "top", &extra), true);
        // remove the extrude (everything downstream loses its references), a middle fillet and the last feature
        let w = check("remove extrude", &base, &edit_features(&base, |fs| { fs.remove(1); }), true);
        assert!(w > 0, "lost references warn");
        check("remove middle", &base, &edit_features(&base, |fs| { fs.remove(3); }), true);
        check("remove last", &base, &edit_features(&base, |fs| { fs.pop(); }), true);
        // move: the last fillet to right after the extrude, and the cap sketch to the end
        check("move up", &base, &edit_features(&base, |fs| { let f = fs.remove(6); fs.insert(2, f); }), true);
        check("move down", &base, &edit_features(&base, |fs| { let f = fs.remove(2); fs.push(f); }), true);
        // the first feature edited: nothing is shared (the document key still matches, no prefix)
        let first = base.replace(r#""x":40,"y":0"#, r#""x":41,"y":0"#);
        check("edit first", &base, &first, false);
        // identical documents: nothing to warn about
        assert_eq!(check("same", &base, &base, true), 0);
    })
    .join()
    .unwrap();
}

#[test]
fn ref_diff_is_correct_when_the_cache_is_cold() {
    std::thread::spawn(|| {
        let (base, part) = (doc(10.0, 1.0, "top", ""), doc(10.0, 1.0, "front", ""));
        // nothing regenerated yet: both documents are built from the start and the answer is the reference one
        let fast = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{part},"cacheStats":true}}"#));
        assert_eq!(fast.get("cacheStats").unwrap().get_f64("resumedAt"), Some(0.0));
        let slow = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{part},"noCache":true}}"#));
        assert_eq!(without(&fast, "cacheStats"), ck_json::to_string(&slow));
    })
    .join()
    .unwrap();
}

#[test]
fn match_edges_from_the_cache_equals_a_cold_run() {
    std::thread::spawn(|| {
        let d = doc(10.0, 1.0, "top", "");
        let seed = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,0,0]}"#;
        let ask = |extra: &str| ck_json::to_string(&call(&format!(r#"{{"type":"matchEdges","part":{d},"upTo":null,"mode":"similar","seeds":[{seed}],"exclude":[]{extra}}}"#)));
        let cold = ask("");
        call(&format!(r#"{{"type":"regen","tag":"main","part":{d}}}"#));
        assert_eq!(ask(""), cold);
        assert_eq!(ask(r#","noCache":true"#), cold);
        // a prefix of the document (upTo) is answered from its own cached state
        let up = |extra: &str| ck_json::to_string(&call(&format!(r#"{{"type":"matchEdges","part":{d},"upTo":4,"mode":"parallel","seeds":[{seed}],"exclude":[]{extra}}}"#)));
        assert_eq!(up(""), up(r#","noCache":true"#));
    })
    .join()
    .unwrap();
}
