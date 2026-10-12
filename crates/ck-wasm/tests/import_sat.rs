//! `importSat` request: SAT text -> body in the tag's state; validate and
//! massProps then run on it.

use ck_json::Json;

fn call(msg: Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg)]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn obj(kv: Vec<(&str, Json)>) -> Json {
    Json::Obj(kv.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

#[test]
fn import_sat_then_validate_and_mass() {
    ck_wasm::engine::install();
    let sat = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../ck-io/tests/fixtures/sat/cylinder_hole.sat")).unwrap();
    let r = call(obj(vec![("type", "importSat".into()), ("tag", "t".into()), ("name", "cyl".into()), ("data", sat.as_str().into())]));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&r));
    let res = r.get("result").unwrap();
    assert!(res.get_arr("errors").unwrap().is_empty());
    assert_eq!(res.get("body").unwrap().get_arr("faces").unwrap().len(), 4);
    assert_eq!(res.get("import").unwrap().get_str("format"), Some("sat"));
    let v = call(obj(vec![("type", "validate".into()), ("tag", "t".into())]));
    assert_eq!(v.get("result").unwrap().get_arr("issues").unwrap().len(), 0, "{}", ck_json::to_string(&v));
    let m = call(obj(vec![("type", "massProps".into()), ("tag", "t".into())]));
    let vol = m.get("result").unwrap().get_f64("volume").unwrap();
    let want = std::f64::consts::PI * 84.0 * 20.0;
    assert!((vol - want).abs() / want < 1e-6, "{vol} vs {want}");
}

#[test]
fn import_sat_failure_is_structured_and_keeps_state() {
    ck_wasm::engine::install();
    let bad = "700 0 1 0\n@1 x\n1 1e-06 1e-10\nbody $-1 -1 $-1 $1 $-1 $-1 #\nlump $-1 -1 $-1 $-1 $2 $0 #\nshell $-1 -1 $-1 $-1 $-1 $3 $-1 $1 #\nface $-1 -1 $-1 $-1 $-1 $2 $-1 $4 forward single in #\nmesh-surface $-1 -1 $-1 #\nEnd-of-ACIS-data\n";
    let r = call(obj(vec![("type", "importSat".into()), ("tag", "u".into()), ("data", bad.into())]));
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)));
    let errs = r.get("result").unwrap().get_arr("errors").unwrap();
    assert_eq!(errs.len(), 1);
    assert_eq!(errs[0].get_str("code"), Some("sat.unsupported"));
    assert_eq!(errs[0].get_str("entity"), Some("$4"));
    assert!(r.get("result").unwrap().get("body").unwrap().is_null());
    let q = call(obj(vec![("type", "validate".into()), ("tag", "u".into())]));
    assert_eq!(q.get("ok"), Some(&Json::Bool(false)));
    let m = call(obj(vec![("type", "importSat".into()), ("tag", "u".into())]));
    assert_eq!(m.get("ok"), Some(&Json::Bool(false)));
}

/// Real files (outside the repo, `SAT_SAMPLES_DIR`): skipped when absent.
#[test]
fn import_real_files_over_the_wire() {
    ck_wasm::engine::install();
    let dir = std::env::var("SAT_SAMPLES_DIR")
        .unwrap_or_else(|_| "/private/tmp/claude-501/-Volumes-second-disk-tmp-purr-rebuild/296fdc97-740a-4f73-841f-21dbda16b0ed/scratchpad/sat".into());
    for (rel, tag, faces) in [("dice.sat", "dice", 47.0), ("svalinn/ErrorBlocks.SAT", "blocks", 53.0)] {
        let Ok(bytes) = std::fs::read(format!("{dir}/{rel}")) else {
            eprintln!("skip {rel}: absent");
            continue;
        };
        let text = String::from_utf8_lossy(&bytes).to_string();
        let r = call(obj(vec![("type", "importSat".into()), ("tag", tag.into()), ("data", text.as_str().into())]));
        assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{rel}");
        let res = r.get("result").unwrap();
        assert!(res.get_arr("errors").unwrap().is_empty(), "{rel}: {}", ck_json::to_string(res.get_arr("errors").unwrap().first().unwrap()));
        assert_eq!(res.get("import").unwrap().get_f64("faces"), Some(faces), "{rel}");
        let v = call(obj(vec![("type", "validate".into()), ("tag", tag.into())]));
        assert_eq!(v.get("result").unwrap().get_arr("issues").unwrap().len(), 0, "{rel}");
        let m = call(obj(vec![("type", "massProps".into()), ("tag", tag.into())]));
        assert!(m.get("result").unwrap().get_f64("volume").unwrap() > 0.0, "{rel}");
    }
}
