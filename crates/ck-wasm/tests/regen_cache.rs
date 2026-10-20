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
        assert_eq!(st.get_bool("viewHit"), Some(true), "an identical document replays the cached snapshot");
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

/// The dome cap with a through hole: editing the hole re-meshes only the changed faces, reverting replays the cached
/// snapshot, and every reply equals a cold one byte for byte.
#[test]
fn view_snapshot_and_face_cache_are_transparent() {
    std::thread::spawn(|| {
        let base = include_str!("fixtures/dome_cap.ckpart").trim().to_string();
        let with_hole = |d: f64| {
            let cut = base.rfind("]}").or_else(|| base.rfind(']')).unwrap();
            format!(
                r#"{},{{"type":"sketch","id":"F20","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"h1","x":20,"y":0}}],"entities":[{{"type":"point","id":"e1","p":"h1"}}],"constraints":[]}},{{"type":"hole","id":"F21","sketch":"F20","kind":"simple","diameter":{d},"end":"through"}}{}"#,
                &base[..cut],
                &base[cut..]
            )
        };
        let (h12, h15) = (with_hole(12.0), with_hole(15.0));
        let o = r#"{"cacheStats":true}"#;
        let r12 = regen(&h12, o, "main");
        assert_eq!(stats(&r12).get_bool("viewHit"), Some(false));
        let r15 = regen(&h15, o, "main");
        let st = stats(&r15);
        assert_eq!(st.get_bool("viewHit"), Some(false));
        assert!(st.get_f64("tessFaceHits").unwrap() >= 5.0, "unchanged faces keep their meshes: {}", ck_json::to_string(&st));
        // undo: the snapshot of the 12 mm document is replayed whole
        let back = regen(&h12, o, "main");
        assert_eq!(stats(&back).get_bool("viewHit"), Some(true));
        assert_eq!(stats(&back).get_f64("tessFaceLookups"), Some(0.0));
        let strip = |r: &[u8]| {
            // drop cacheStats from the reply JSON, keep the buffers
            let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
            let mut j = ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap();
            if let Json::Obj(kv) = &mut j {
                if let Some((_, Json::Obj(res))) = kv.iter_mut().find(|(k, _)| k == "result") {
                    res.retain(|(k, _)| k != "cacheStats");
                }
                kv.retain(|(k, _)| k != "ms" && k != "buffers_ms");
            }
            (ck_json::to_string(&j), r[(4 + jl + 7) & !7..].to_vec())
        };
        let c12 = cold(&h12, o);
        let c15 = cold(&h15, o);
        assert!(strip(&back).1 == strip(&c12).1, "replayed buffers differ from cold");
        assert!(strip(&r15).1 == strip(&c15).1, "partly cached buffers differ from cold");
        assert!(strip(&r12).1 == strip(&c12).1);
        assert_eq!(strip(&r12).0, strip(&c12).0.replace("\"viewHit\":false", "\"viewHit\":false"));
    })
    .join()
    .unwrap();
}

// ---------------------------------------------------------------- cosmetic fields (docs/regen-cache.md)

const COSMETIC: [&str; 5] = ["group", "name", "autoName", "visible", "userHidden"];

fn doc(extra_features: bool) -> Json {
    let mut j = ck_json::parse(include_str!("fixtures/dome_cap.ckpart").trim()).unwrap();
    if extra_features {
        if let Some(Json::Arr(fs)) = j.get_mut("features") {
            fs.push(ck_json::parse(r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"top"},"offset":30}"#).unwrap());
            fs.push(ck_json::parse(r#"{"type":"sketch3d","id":"S3","points":[],"entities":[],"constraints":[]}"#).unwrap());
        }
    }
    j
}

fn feats(j: &mut Json) -> &mut Vec<Json> {
    match j.get_mut("features") {
        Some(Json::Arr(a)) => a,
        _ => panic!("no features"),
    }
}

fn set_key(f: &mut Json, k: &str, v: Option<Json>) {
    if let Json::Obj(m) = f {
        m.retain(|(kk, _)| kk != k);
        if let Some(v) = v {
            m.push((k.to_string(), v));
        }
    }
}

/// The reply without the stats and timing, with the buffer area, for byte comparison.
fn norm(r: &[u8]) -> (String, Vec<u8>) {
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    let mut j = ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap();
    if let Json::Obj(kv) = &mut j {
        if let Some((_, Json::Obj(res))) = kv.iter_mut().find(|(k, _)| k == "result") {
            res.retain(|(k, _)| k != "cacheStats");
        }
        kv.retain(|(k, _)| k != "ms" && k != "buffers_ms");
    }
    (ck_json::to_string(&j), r[(4 + jl + 7) & !7..].to_vec())
}

const STATS: &str = r#"{"cacheStats":true}"#;

#[test]
fn cosmetic_edits_keep_every_state_and_reply_like_cold() {
    std::thread::spawn(|| {
        let base = doc(true);
        let n = base.get("features").and_then(|f| f.as_arr()).unwrap().len();
        let base_s = ck_json::to_string(&base);
        let warm = regen(&base_s, STATS, "main");
        assert_eq!(stats(&warm).get_f64("ranFeatures"), Some(n as f64));
        // group the first features (rows 0..3 into one folder), rename a sketch, hide the plane: nothing may run
        let mut edited = base.clone();
        for (i, f) in feats(&mut edited).iter_mut().enumerate() {
            if i < 4 {
                set_key(f, "group", Some(Json::Str("Folder".into())));
            }
        }
        set_key(&mut feats(&mut edited)[0], "name", Some(Json::Str("Renamed sketch".into())));
        set_key(&mut feats(&mut edited)[0], "autoName", Some(Json::Bool(false)));
        set_key(&mut feats(&mut edited)[7], "visible", Some(Json::Bool(false)));
        set_key(&mut feats(&mut edited)[7], "userHidden", Some(Json::Bool(true)));
        set_key(&mut feats(&mut edited)[8], "group", Some(Json::Str("Folder".into())));
        let es = ck_json::to_string(&edited);
        let r = regen(&es, STATS, "main");
        let st = stats(&r);
        assert_eq!(st.get_bool("fullHit"), Some(true), "{}", ck_json::to_string(&st));
        assert_eq!(st.get_f64("ranFeatures"), Some(0.0));
        assert_eq!(st.get_f64("reusedFeatures"), Some(n as f64));
        assert_eq!(st.get_bool("viewHit"), Some(true));
        // ... and the reply is the one of a cold run of the edited document (the sketch echoes carry the new keys)
        let c = cold(&es, STATS);
        assert_eq!(norm(&r), norm(&c), "hit differs from cold");
        assert!(ck_json::to_string(&parse(&r)).contains("Renamed sketch"), "the echo of the sketch shows the new name");
        // the stored state was not corrupted: the original, the edited one, ungrouped, regrouped differently
        let mut ungrouped = edited.clone();
        for f in feats(&mut ungrouped).iter_mut() {
            set_key(f, "group", None);
        }
        let mut regrouped = ungrouped.clone();
        for f in feats(&mut regrouped).iter_mut().skip(2).take(3) {
            set_key(f, "group", Some(Json::Str("B".into())));
        }
        for d in [&base, &ungrouped, &regrouped, &edited, &base] {
            let ds = ck_json::to_string(d);
            let r = regen(&ds, STATS, "main");
            assert_eq!(stats(&r).get_f64("ranFeatures"), Some(0.0));
            assert_eq!(norm(&r), norm(&cold(&ds, STATS)));
        }
        // a cosmetic edit after a real edit at the end reruns only that feature
        let mut tail = base.clone();
        set_key(&mut feats(&mut tail)[6], "radius", Some(Json::Num(1.5)));
        regen(&ck_json::to_string(&tail), "{}", "main");
        for f in feats(&mut tail).iter_mut().take(3) {
            set_key(f, "group", Some(Json::Str("G".into())));
        }
        let ts = ck_json::to_string(&tail);
        let r = regen(&ts, STATS, "main");
        assert_eq!(stats(&r).get_f64("ranFeatures"), Some(0.0));
        assert_eq!(norm(&r), norm(&cold(&ts, STATS)));
    })
    .join()
    .unwrap();
}

/// A cosmetic key can only reach a reply through the echo of a sketch / sketch3d: with every key set on every feature,
/// the cold reply equals the plain one once those keys are removed from the echoes.
#[test]
fn cosmetic_keys_reach_the_reply_only_through_sketch_echoes() {
    std::thread::spawn(|| {
        let base = doc(true);
        let mut all = base.clone();
        for (i, f) in feats(&mut all).iter_mut().enumerate() {
            set_key(f, "group", Some(Json::Str(format!("G{}", i / 3))));
            set_key(f, "name", Some(Json::Str(format!("Name {i}"))));
            set_key(f, "autoName", Some(Json::Bool(i % 2 == 0)));
            set_key(f, "visible", Some(Json::Bool(i % 2 == 1)));
            set_key(f, "userHidden", Some(Json::Bool(i % 3 == 0)));
        }
        let strip_echo = |r: &[u8]| {
            let mut j = parse(r);
            if let Some(Json::Obj(res)) = j.get_mut("result") {
                for (k, v) in res.iter_mut() {
                    if k == "sketches" || k == "sketches3" {
                        if let Json::Arr(items) = v {
                            for it in items {
                                if let Json::Arr(pair) = it {
                                    if let Some(Json::Obj(o)) = pair.get_mut(1) {
                                        if let Some((_, Json::Obj(sk))) = o.iter_mut().find(|(k, _)| k == "sketch") {
                                            sk.retain(|(k, _)| !COSMETIC.contains(&k.as_str()));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            ck_json::to_string(&j)
        };
        let a = cold(&ck_json::to_string(&base), "{}");
        let b = cold(&ck_json::to_string(&all), "{}");
        assert_ne!(a, b, "the echo shows the keys");
        assert_eq!(strip_echo(&a), strip_echo(&b));
        let ja = a[(4 + u32::from_le_bytes([a[0], a[1], a[2], a[3]]) as usize + 7) & !7..].to_vec();
        let jb = b[(4 + u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize + 7) & !7..].to_vec();
        assert_eq!(ja, jb, "buffers (meshes) do not depend on them");
    })
    .join()
    .unwrap();
}

#[test]
fn other_keys_still_rerun() {
    std::thread::spawn(|| {
        let base = doc(false);
        let regen_stats = |j: &Json| stats(&regen(&ck_json::to_string(j), STATS, "main"));
        regen_stats(&base);
        let mut sup = base.clone();
        set_key(&mut feats(&mut sup)[1], "suppressed", Some(Json::Bool(true)));
        let st = regen_stats(&sup);
        assert_eq!(st.get_f64("reusedFeatures"), Some(1.0), "suppressed is not cosmetic");
        let mut note = base.clone();
        set_key(&mut feats(&mut note)[2], "note", Some(Json::Str("x".into())));
        assert_eq!(regen_stats(&note).get_f64("reusedFeatures"), Some(2.0), "unknown keys stay in the key");
        let mut dep = base.clone();
        set_key(&mut feats(&mut dep)[3], "depth", Some(Json::Num(3.0)));
        assert_eq!(regen_stats(&dep).get_f64("reusedFeatures"), Some(3.0));
        // a `name` nested in a feature (a profile, an inserted part) is not a feature label
        let mut nested = base.clone();
        set_key(&mut feats(&mut nested)[4], "meta", Some(ck_json::parse(r#"{"name":"a"}"#).unwrap()));
        regen_stats(&nested);
        set_key(&mut feats(&mut nested)[4], "meta", Some(ck_json::parse(r#"{"name":"b"}"#).unwrap()));
        assert_eq!(regen_stats(&nested).get_f64("reusedFeatures"), Some(4.0));
    })
    .join()
    .unwrap();
}
