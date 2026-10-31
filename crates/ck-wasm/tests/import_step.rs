//! `importStep` through the full server path: regen (golden request bodies
//! of crates/ck-io) -> `step` -> `importStep` on another tag -> the same
//! massProps, a clean `validate`, and the usual body wire shape.
use ck_json::Json;

fn call(body: &str) -> Json {
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn num(j: &Json, k: &str) -> f64 {
    j.get(k).and_then(Json::as_f64).unwrap_or(f64::NAN)
}

#[test]
fn golden_requests_round_trip_through_the_server() {
    ck_wasm::engine::install();
    let dir = format!("{}/../ck-io/tests/golden", env!("CARGO_MANIFEST_DIR"));
    let mut n = 0;
    let mut fails = vec![];
    for ent in std::fs::read_dir(&dir).unwrap() {
        let p = ent.unwrap().path();
        let name = p.file_name().unwrap().to_str().unwrap().to_string();
        let Some(stem) = name.strip_suffix(".req.json") else { continue };
        let req = std::fs::read_to_string(&p).unwrap();
        let r = call(&req);
        if r.get("result").and_then(|r| r.get("body")).map_or(true, |b| matches!(b, Json::Null)) {
            continue; // the reference-only goldens (spline bodies we cannot build yet)
        }
        let step = call(&format!(r#"{{"id":2,"msg":{{"type":"step","tag":"main","name":"{stem}"}}}}"#));
        let text = step.get("result").and_then(Json::as_str).unwrap().to_string();
        let msg = ck_json::to_string(&Json::Obj(vec![
            ("id".into(), Json::Num(3.0)),
            (
                "msg".into(),
                Json::Obj(vec![
                    ("type".into(), Json::Str("importStep".into())),
                    ("tag".into(), Json::Str("imp".into())),
                    ("name".into(), Json::Str(stem.into())),
                    ("data".into(), Json::Str(text)),
                ]),
            ),
        ]));
        let imp = call(&msg);
        assert_eq!(imp.get("ok"), Some(&Json::Bool(true)), "{stem}: {imp:?}");
        let res = imp.get("result").unwrap();
        assert!(res.get_arr("errors").unwrap().is_empty(), "{stem}: {:?}", res.get("errors"));
        assert!(res.get("body").and_then(|b| b.get_arr("faces")).is_some_and(|f| !f.is_empty()), "{stem}");
        let m0 = call(r#"{"id":4,"msg":{"type":"massProps","tag":"main"}}"#);
        let m1 = call(r#"{"id":5,"msg":{"type":"massProps","tag":"imp"}}"#);
        let (a, b) = (m0.get("result").unwrap(), m1.get("result").unwrap());
        for k in ["volume", "area"] {
            if (num(a, k) - num(b, k)).abs() > 1e-6 * (1.0 + num(a, k).abs()) {
                fails.push(format!("{stem}: {k} {} vs {}", num(b, k), num(a, k)));
            }
        }
        let v = call(r#"{"id":6,"msg":{"type":"validate","tag":"imp"}}"#);
        if !v.get("result").and_then(|r| r.get_arr("issues")).unwrap().is_empty() {
            fails.push(format!("{stem}: validate {:?}", v.get("result")));
        }
        n += 1;
    }
    assert!(n >= 15, "only {n} bodies round-tripped");
    assert!(fails.is_empty(), "{}", fails.join("\n"));
}

#[test]
fn import_errors_are_structured_and_keep_the_previous_state() {
    ck_wasm::engine::install();
    let bad = call(r#"{"id":1,"msg":{"type":"importStep","tag":"bad","data":"ISO-10303-21;\nHEADER;ENDSEC;\nDATA;\n#1=PLANE('',#2);\nENDSEC;\nEND-ISO-10303-21;"}}"#);
    assert_eq!(bad.get("ok"), Some(&Json::Bool(true)));
    let errs = bad.get("result").unwrap().get_arr("errors").unwrap();
    assert_eq!(errs.len(), 1);
    let s = ck_json::to_string(&errs[0]);
    assert!(s.contains("noSolid"), "{s}");
    // nothing was stored under the tag
    let m = call(r#"{"id":2,"msg":{"type":"massProps","tag":"bad"}}"#);
    assert_eq!(m.get("ok"), Some(&Json::Bool(false)));
    // data must be a string
    let e = call(r#"{"id":3,"msg":{"type":"importStep"}}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)));
}

fn import_msg(id: u32, tag: &str, text: &str) -> String {
    ck_json::to_string(&Json::Obj(vec![
        ("id".into(), Json::Num(id as f64)),
        (
            "msg".into(),
            Json::Obj(vec![
                ("type".into(), Json::Str("importStep".into())),
                ("tag".into(), Json::Str(tag.into())),
                ("name".into(), Json::Str("t".into())),
                ("data".into(), Json::Str(text.into())),
            ]),
        ),
    ]))
}

/// A part the reader cannot finish is not a half-built solid: it comes back as a surface body of the faces that were
/// read, `import.parts` says what became of it and why, and `step` writes the surface body too.
#[test]
fn a_part_with_a_face_left_out_comes_back_as_a_surface_body_with_its_reasons() {
    ck_wasm::engine::install();
    let req = std::fs::read_to_string(format!("{}/../ck-io/tests/golden/box.req.json", env!("CARGO_MANIFEST_DIR"))).unwrap();
    call(&req);
    let step = call(r#"{"id":2,"msg":{"type":"step","tag":"main","name":"box"}}"#);
    let text = step.get("result").and_then(Json::as_str).unwrap().to_string();
    // the loop of the third face points at an entity that is not there
    let mut at = 0;
    for _ in 0..3 {
        at += text[at..].find("ADVANCED_FACE(").unwrap() + 1;
    }
    let open = at + text[at..].find("(#").unwrap() + 2;
    let end = open + text[open..].find(|c: char| !c.is_ascii_digit()).unwrap();
    let broken = format!("{}999999{}", &text[..open], &text[end..]);
    let imp = call(&import_msg(3, "imp", &broken));
    let res = imp.get("result").unwrap();
    assert!(res.get_arr("errors").unwrap().is_empty(), "{:?}", res.get("errors"));
    assert!(res.get("body").and_then(|b| b.get_arr("faces")).map_or(true, |f| f.is_empty()), "a half-built solid was returned as the body");
    let surfs = res.get_arr("surfaces").unwrap();
    assert_eq!(surfs.len(), 1);
    let parts = res.get("import").and_then(|i| i.get_arr("parts")).unwrap();
    assert_eq!(parts.len(), 1);
    let p = &parts[0];
    assert_eq!(p.get_str("outcome"), Some("surface"));
    assert_eq!((num(p, "faces"), num(p, "facesTotal")), (5.0, 6.0));
    let reasons = p.get_arr("reasons").unwrap();
    assert_eq!(reasons[0].get_str("code"), Some("badEntity"));
    assert!(reasons.iter().any(|r| r.get_str("code") == Some("SHELL_OPEN")));
    // exporting keeps the surface body
    let back = call(r#"{"id":4,"msg":{"type":"step","tag":"imp","name":"again"}}"#);
    let t2 = back.get("result").and_then(Json::as_str).unwrap();
    assert!(t2.contains("SHELL_BASED_SURFACE_MODEL"), "the surface body was dropped from the export");
}
