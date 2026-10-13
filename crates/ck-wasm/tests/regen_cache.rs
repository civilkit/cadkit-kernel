//! Incremental regen (docs/regen-cache.md): a warm reply is byte-identical to a
//! cold reply of the same document in a fresh kernel instance. A kernel
//! instance is a thread here (the server and the cache are thread-local).

use ck_json::Json;

fn raw(msg: &str) -> Vec<u8> {
    ck_wasm::engine::install();
    ck_server::handle(format!(r#"{{"id":1,"msg":{msg}}}"#).as_bytes())
}

fn parse(r: &[u8]) -> Json {
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn sketch(id: &str, w: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":20}},{{"id":"p4","x":0,"y":20}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

/// sketch F1, extrude F2, sketch F3, extrude F4 (cut), with tweakable numbers.
fn part(w: f64, d: f64, w2: f64, d2: f64, extra: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,{extra}"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{d}}},{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":{d2}}}]}}"#,
        sketch("F1", w),
        sketch("F3", w2)
    )
}

fn regen(part: &str, opts: &str, tag: &str) -> Vec<u8> {
    raw(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part},"opts":{opts}}}"#))
}

fn cold(part: &str, opts: &str) -> Vec<u8> {
    let (p, o) = (part.to_string(), opts.to_string());
    std::thread::spawn(move || regen(&p, &o, "main")).join().unwrap()
}

fn stats(r: &[u8]) -> Json {
    parse(r).get("result").unwrap().get("cacheStats").unwrap().clone()
}

#[test]
fn full_hit_and_edits_match_cold() {
    std::thread::spawn(|| {
        let a = part(40.0, 10.0, 10.0, 5.0, "");
        let c0 = regen(&a, "{}", "main");
        assert_eq!(c0, regen(&a, "{}", "main"), "full hit differs from cold");
        assert_eq!(c0, cold(&a, "{}"));
        let st = stats(&regen(&a, r#"{"cacheStats":true}"#, "main"));
        assert_eq!(st.get_bool("fullHit"), Some(true));
        assert_eq!(st.get_f64("reusedFeatures"), Some(4.0));
        // edits at the start, middle and end vs cold, and the undo
        for (w, d, w2, d2) in [(41.0, 10.0, 10.0, 5.0), (40.0, 11.0, 10.0, 5.0), (40.0, 10.0, 10.0, 6.0), (40.0, 10.0, 11.0, 5.0)] {
            let e = part(w, d, w2, d2, "");
            assert_eq!(regen(&e, "{}", "main"), cold(&e, "{}"), "edit {w} {d} {w2} {d2}");
            assert_eq!(regen(&a, "{}", "main"), c0, "undo");
        }
    })
    .join()
    .unwrap();
}

#[test]
fn preview_reuses_the_prefix_of_main() {
    std::thread::spawn(|| {
        let a = part(40.0, 10.0, 10.0, 5.0, "");
        regen(&a, "{}", "main");
        let e = part(40.0, 10.0, 10.0, 6.0, "");
        let r = regen(&e, r#"{"cacheStats":true}"#, "preview");
        let st = stats(&r);
        assert_eq!(st.get_f64("reusedFeatures"), Some(3.0));
        assert_eq!(st.get_f64("ranFeatures"), Some(1.0));
        // identical to a cold preview but for the stats key
        let c = cold(&e, r#"{"cacheStats":true}"#);
        let strip = |r: &[u8]| {
            let mut j = parse(r);
            if let Some(Json::Obj(kv)) = j.get_mut("result") {
                kv.retain(|(k, _)| k != "cacheStats");
            }
            ck_json::to_string(&j)
        };
        assert_eq!(strip(&r), strip(&c));
        // state of the tag is the edited body: both tags answer queries
        assert!(parse(&raw(r#"{"type":"massProps","tag":"preview"}"#)).get_bool("ok") == Some(true));
    })
    .join()
    .unwrap();
}

#[test]
fn no_cache_and_document_level_keys() {
    std::thread::spawn(|| {
        let a = part(40.0, 10.0, 10.0, 5.0, "");
        let c0 = regen(&a, "{}", "main");
        let st = stats(&regen(&a, r#"{"noCache":true,"cacheStats":true}"#, "main"));
        assert_eq!(st.get_bool("enabled"), Some(false));
        assert_eq!(st.get_f64("reusedFeatures"), Some(0.0));
        // name / seq / counters do not change geometry: still a full hit, same bytes
        let renamed = part(40.0, 10.0, 10.0, 5.0, r#""name":"other","seq":9,"counters":{"sketch":3},"rollback":null,"#);
        assert_eq!(regen(&renamed, "{}", "main"), c0);
        assert_eq!(stats(&regen(&renamed, r#"{"cacheStats":true}"#, "main")).get_bool("fullHit"), Some(true));
        // variables are document level: a change must not reuse anything
        let withvar = part(40.0, 10.0, 10.0, 5.0, r#""variables":[{"name":"w","expr":"3"}],"#);
        assert_eq!(stats(&regen(&withvar, r#"{"cacheStats":true}"#, "main")).get_f64("reusedFeatures"), Some(0.0));
        // result-changing options are part of the key
        assert_eq!(regen(&a, r#"{"sketchStatus":true}"#, "main"), cold(&a, r#"{"sketchStatus":true}"#));
        assert_ne!(regen(&a, r#"{"sketchStatus":true}"#, "main"), c0);
    })
    .join()
    .unwrap();
}

#[test]
fn tiny_cap_evicts_and_stays_identical() {
    std::thread::spawn(|| {
        let docs = [part(40.0, 10.0, 10.0, 5.0, ""), part(41.0, 10.0, 10.0, 5.0, ""), part(40.0, 12.0, 10.0, 5.0, ""), part(40.0, 10.0, 10.0, 5.0, "")];
        for d in &docs {
            let r = regen(d, r#"{"cacheCap":6000,"cacheStats":true}"#, "main");
            let st = stats(&r);
            assert!(st.get_f64("bytes").unwrap() <= 6000.0);
            let c = cold(d, "{}");
            let plain = regen(d, r#"{"cacheCap":6000}"#, "main");
            assert_eq!(plain, c);
        }
        let st = stats(&regen(&docs[0], r#"{"cacheCap":6000,"cacheStats":true}"#, "main"));
        assert!(st.get_f64("evicted").unwrap() > 0.0, "{st:?}");
    })
    .join()
    .unwrap();
}
