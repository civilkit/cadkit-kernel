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
