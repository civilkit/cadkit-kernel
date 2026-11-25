//! `importF3d` request: a bare `.smb` blob, a refused file, and (when the corpus is present) the three real Fusion files.
//! `cargo test -p ck-wasm --test import_f3d`; the corpus is `$CK_F3D_CORPUS` (docs/f3d.md).

// the decoder is linked only with the `f3d` feature; the gate names every test file with --test, which a
// `required-features` target refuses, so the file is empty without it instead
#![cfg(feature = "f3d")]

use ck_json::Json;

fn b64(d: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in d.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            if i <= c.len() {
                s.push(T[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

fn call(msg: Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg)]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn import(data: &str, opts: Option<Json>) -> Json {
    ck_wasm::engine::install();
    let mut kv: Vec<(String, Json)> = vec![("type".into(), "importF3d".into()), ("tag".into(), "t".into()), ("data".into(), data.into())];
    if let Some(o) = opts {
        kv.push(("opts".into(), o));
    }
    call(Json::Obj(kv))
}

#[test]
fn a_bare_smb_blob_comes_in_as_a_solid_with_a_report() {
    let r = import(&b64(&ck_asm::testdata::cube_blob()), None);
    let res = r.get("result").unwrap();
    assert!(res.get("errors").and_then(|e| e.as_arr()).map_or(true, |e| e.is_empty()), "{res:?}");
    let imp = res.get("import").unwrap();
    assert_eq!(imp.get_str("format"), Some("f3d"));
    assert_eq!(imp.get_str("container"), Some("blob"));
    assert_eq!(imp.get_f64("solids"), Some(1.0));
    assert_eq!(imp.get_f64("faces"), Some(6.0));
    assert_eq!(imp.get_f64("unitScale"), Some(10.0));
    let report = imp.get("report").unwrap().as_arr().unwrap();
    assert!(report[0].as_str().unwrap().starts_with("Read 1 body blob(s) holding 1 ASM body: 1 valid solid(s)"), "{:?}", report[0]);
    // a solid: no warning
    assert!(res.get("warnings").and_then(|w| w.as_arr()).map_or(true, |w| w.is_empty()));
    assert_eq!(res.get("body").unwrap().get("faces").unwrap().as_arr().unwrap().len(), 6);
}

#[test]
fn a_body_that_is_not_a_solid_is_a_surface_body_with_the_warning() {
    let blob = ck_asm::testdata::cylinder_blob_with(ck_asm::testdata::CylinderVariant { unknown_side: true, ..Default::default() });
    let r = import(&b64(&blob), None);
    let res = r.get("result").unwrap();
    let imp = res.get("import").unwrap();
    assert_eq!((imp.get_f64("solids"), imp.get_f64("surfaces")), (Some(0.0), Some(1.0)));
    assert_eq!(imp.get("skippedFaces").unwrap().as_arr().unwrap().len(), 1);
    let warnings = res.get("warnings").unwrap().as_arr().unwrap();
    let w = warnings[0].as_arr().unwrap();
    assert_eq!(w[1].get_str("code"), Some("importStepParts"));
    let surfaces = res.get("surfaces").unwrap().as_arr().unwrap();
    assert_eq!(surfaces.len(), 1);
    assert_eq!(surfaces[0].as_arr().unwrap()[0].as_str(), Some("import.P1"));
    let reasons = imp.get("parts").unwrap().as_arr().unwrap()[0].get("reasons").unwrap().as_arr().unwrap();
    assert!(reasons.iter().any(|r| r.get_str("code") == Some("unknownGeometry")));
}

#[test]
fn a_file_that_cannot_be_read_is_refused_in_words_and_the_tag_is_left_alone() {
    for (bytes, code) in [(&b"this is not an ASM file"[..], "f3d.asm"), (&b"PK\x03\x04 truncated"[..], "f3d.container")] {
        let r = import(&b64(bytes), None);
        let res = r.get("result").unwrap();
        assert!(matches!(res.get("body"), Some(Json::Null)), "{res:?}");
        let e = res.get("errors").unwrap().as_arr().unwrap()[0].as_arr().unwrap();
        assert_eq!(e[0].as_str(), Some("import"));
        assert_eq!(e[1].get_str("code"), Some(code));
        assert!(!e[1].get("params").unwrap().get_str("message").unwrap().is_empty());
    }
}

fn corpus(rel: &str) -> Option<Vec<u8>> {
    let mut dirs: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(d) = std::env::var("CK_F3D_CORPUS") {
        dirs.push(d.into());
    }
    dirs.push(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../ck-asm/tests/corpus-local"));
    for d in dirs {
        if let Ok(b) = std::fs::read(d.join(rel)) {
            return Some(b);
        }
    }
    eprintln!("skipped: {rel} not found (set CK_F3D_CORPUS)");
    None
}

#[test]
fn the_real_files_come_in_with_their_solids() {
    // allBodies: a history body with a twin in a plain blob is read once: (solids, bodies, faces)
    for (f, solids, bodies, faces) in [("1WireRGBWController.f3d", 63.0, 145.0, 1915.0), ("f3d/F30a.f3z", 1.0, 3.0, 10.0), ("f3d/F30b.f3z", 1.0, 3.0, 10.0)] {
        let Some(bytes) = corpus(f) else { continue };
        let r = import(&b64(&bytes), Some(Json::Obj(vec![("allBodies".into(), Json::Bool(true))])));
        let imp = r.get("result").unwrap().get("import").unwrap();
        assert_eq!(imp.get_str("container"), Some("zip"), "{f}");
        assert_eq!((imp.get_f64("solids"), imp.get_f64("bodies"), imp.get_f64("facesTotal")), (Some(solids), Some(bodies), Some(faces)), "{f}: {imp:?}");
        assert_eq!(imp.get_f64("faces"), imp.get_f64("facesTotal"), "{f}: every face was built");
        assert!(imp.get("skippedFaces").unwrap().as_arr().unwrap().is_empty(), "{f}");
    }
}

#[test]
fn by_default_only_the_designs_own_bodies_come_in() {
    // the top-level bodies of the history blob: (solids, bodies, faces)
    for (f, solids, bodies, faces) in [("1WireRGBWController.f3d", 4.0, 4.0, 464.0), ("f3d/F30a.f3z", 1.0, 1.0, 8.0), ("f3d/F30b.f3z", 1.0, 1.0, 8.0)] {
        let Some(bytes) = corpus(f) else { continue };
        let r = import(&b64(&bytes), None);
        let imp = r.get("result").unwrap().get("import").unwrap();
        assert_eq!((imp.get_f64("solids"), imp.get_f64("bodies"), imp.get_f64("facesTotal")), (Some(solids), Some(bodies), Some(faces)), "{f}: {imp:?}");
        let report: Vec<&str> = imp.get("report").unwrap().as_arr().unwrap().iter().filter_map(|l| l.as_str()).collect();
        assert!(report.iter().any(|l| l.contains("Not read as parts of the design")), "{f}: the report says what was not read: {report:?}");
    }
}
