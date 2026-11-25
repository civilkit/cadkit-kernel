//! `check` measures without gathering per-face / per-edge facts: the reply must equal, byte for byte, the one of the
//! original path (`legacyMeasure:true`), and `measured:false` with only `valid` expectations drops the `measured`
//! block and nothing else.

fn call_raw(msg: &str) -> String {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    String::from_utf8(r[4..4 + jl].to_vec()).unwrap()
}

const ALL: &str = r#"[{"kind":"volume","min":1},{"kind":"area","min":1},{"kind":"com","at":[0,0,0]},{"kind":"bbox","min":[0,0,0]},
{"kind":"faces","min":1},{"kind":"edges","min":1},{"kind":"vertices","min":1},{"kind":"solids","equals":1},{"kind":"holes","min":0},{"kind":"innerLoops","min":0},{"kind":"valid"}]"#;

fn plate() -> &'static str {
    r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20},{"id":"c5","x":30,"y":10}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"},{"type":"circle","id":"e5","c":"c5","r":4}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}]}"#
}

fn filleted() -> String {
    let p = plate().trim_end().trim_end_matches("]}").to_string();
    format!(r#"{p},{{"type":"fillet","id":"F4","radius":1,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,0,0]}}]}}]}}"#)
}

fn revolved() -> &'static str {
    r#"{"format":"ck-part","version":1,"features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":0},{"id":"c","x":10,"y":6},{"id":"d","x":4,"y":12},{"id":"e","x":0,"y":12},{"id":"o1","x":0,"y":0,"fixed":true},{"id":"o2","x":0,"y":12}],
"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"e"},{"type":"line","id":"e5","p1":"e","p2":"a"},{"type":"line","id":"ax","p1":"o1","p2":"o2","construction":true}],"constraints":[]},
{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360}]}"#
}

fn lofted() -> String {
    let sk = |id: &str, plane: &str, r: f64| {
        let pts: Vec<String> = (0..8).map(|k| {
            let t = k as f64 * std::f64::consts::TAU / 8.0;
            format!(r#"{{"id":"p{k}","x":{},"y":{}}}"#, r * t.cos(), 0.6 * r * t.sin())
        }).collect();
        let ids: Vec<String> = (0..8).map(|k| format!(r#""p{k}""#)).collect();
        format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{{"id":"s","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#, pts.join(","), ids.join(","))
    };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"plane","id":"P1","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":30}},{},{{"type":"loft","id":"L","op":"add","sketches":["S0","S1"]}}]}}"#,
        sk("S0", r#"{"kind":"std","id":"front"}"#, 10.0),
        sk("S1", r#"{"kind":"feature","id":"P1"}"#, 14.0)
    )
}

#[test]
fn check_reply_equals_the_original_path() {
    for (name, part) in [("plate", plate().to_string()), ("filleted", filleted()), ("revolved", revolved().to_string()), ("lofted", lofted())] {
        let tag = format!("cf_{name}");
        let r = call_raw(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part}}}"#));
        assert!(r.starts_with(r#"{"ok":true"#), "{name}: {r}");
        for exp in [ALL, r#"[{"kind":"valid"}]"#, "[]"] {
            let new = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","expect":{exp}}}"#));
            let old = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","legacyMeasure":true,"expect":{exp}}}"#));
            assert!(new.contains(r#""measured":{"volume""#), "{name}: {new}");
            assert_eq!(new, old, "{name} {exp}");
        }
        // opt-in: only `valid` expectations, `measured:false` -> the reply without the block, results unchanged
        let full = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","expect":[{{"kind":"valid"}}]}}"#));
        let lite = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","measured":false,"expect":[{{"kind":"valid"}}]}}"#));
        let cut = full.find(r#","measured":{"volume""#).expect("measured block");
        let end = cut + full[cut..].find(r#""innerLoops":"#).unwrap();
        let end = end + full[end..].find('}').unwrap() + 1;
        assert!(!lite.contains(r#""measured":{"volume""#), "{name}: {lite}");
        assert_eq!(lite, format!("{}{}", &full[..cut], &full[end..]), "{name}");
        // any other expectation ignores the flag
        let keep = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","measured":false,"expect":{ALL}}}"#));
        let norm = call_raw(&format!(r#"{{"type":"check","tag":"{tag}","expect":{ALL}}}"#));
        assert_eq!(keep, norm, "{name}");
    }
}
