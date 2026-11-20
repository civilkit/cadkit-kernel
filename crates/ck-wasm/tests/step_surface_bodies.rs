//! STEP export of a document's surface bodies (surfaceExtrude, blendSurface, ...): they are written as named
//! SHELL_BASED_SURFACE_MODELs beside the solids, re-import with the same faces and surface types, and a document with
//! nothing to write is refused with `stepNothingToExport` (it used to answer null: "empty result for step").
use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn regen(tag: &str, part: &str) {
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part}}}"#));
    assert_eq!(r.get_bool("ok"), Some(true), "{r:?}");
    let errs = r.get("result").and_then(|r| r.get_arr("errors")).map_or(0, |e| e.len());
    assert_eq!(errs, 0, "regen errors: {:?}", r.get("result").and_then(|r| r.get("errors")));
}

fn step(tag: &str) -> Result<String, String> {
    let r = call(&format!(r#"{{"type":"step","tag":"{tag}","name":"doc"}}"#));
    match r.get("result").and_then(Json::as_str) {
        Some(t) => Ok(t.to_string()),
        None => Err(r.get_str("error").unwrap_or("no result and no error").to_string()),
    }
}

fn count(t: &str, kind: &str) -> usize {
    t.matches(&format!("={kind}(")).count()
}

/// Re-import `text` on `tag`; faces (solid body + surfaces) and the multiset of surface kinds of its faces.
fn reimport(tag: &str, text: &str) -> (usize, Vec<String>) {
    let msg = ck_json::to_string(&Json::Obj(vec![
        ("type".into(), Json::Str("importStep".into())),
        ("tag".into(), Json::Str(tag.into())),
        ("name".into(), Json::Str("rt".into())),
        ("data".into(), Json::Str(text.to_string())),
    ]));
    let r = call(&msg);
    let res = r.get("result").unwrap_or(&Json::Null);
    let mut kinds = vec![];
    let mut faces = 0;
    let mut bodies: Vec<&Json> = vec![];
    if let Some(b) = res.get("body").filter(|b| !matches!(b, Json::Null)) {
        bodies.push(b);
    }
    for s in res.get_arr("surfaces").unwrap_or(&[]) {
        if let Some(b) = s.as_arr().and_then(|a| a.get(1)) {
            bodies.push(b);
        }
    }
    for b in bodies {
        for f in b.get_arr("faces").unwrap_or(&[]) {
            faces += 1;
            kinds.push(f.get("surface").and_then(|s| s.get_str("type")).or_else(|| f.get_str("type")).unwrap_or("?").to_string());
        }
    }
    kinds.sort();
    (faces, kinds)
}

const RECT: &str = r#"{"type":"sketch","id":"S","plane":{"kind":"std","id":"xz"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":20,"y":0},{"id":"c","x":20,"y":10},{"id":"d","x":0,"y":10}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},{"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],"constraints":[]}"#;
const OPEN: &str = r#"{"type":"sketch","id":"T","plane":{"kind":"std","id":"xz"},"points":[{"id":"a","x":50,"y":0},{"id":"b","x":70,"y":0},{"id":"c","x":70,"y":15}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"}],"constraints":[]}"#;

fn doc(features: &[&str]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}

#[test]
fn surface_only_part_exports_and_reimports() {
    let path = format!("{}/tests/fixtures/blended_surfaces.ckpart", env!("CARGO_MANIFEST_DIR"));
    let part = std::fs::read_to_string(path).unwrap();
    let r = call(&format!(r#"{{"type":"regen","tag":"surf","part":{part}}}"#));
    let res = r.get("result").unwrap();
    assert!(res.get_arr("errors").unwrap().is_empty(), "{:?}", res.get("errors"));
    let shown = res.get_arr("surfaces").unwrap();
    assert!(!shown.is_empty(), "the fixture makes surface bodies");
    let shown_faces: usize = shown.iter().map(|s| s.as_arr().unwrap()[1].get_arr("faces").map_or(0, |f| f.len())).sum();
    let text = step("surf").expect("a surface-only document exports");
    assert!(count(&text, "SHELL_BASED_SURFACE_MODEL") >= 1 && count(&text, "MANIFOLD_SOLID_BREP") == 0, "{}", &text[..text.len().min(400)]);
    assert_eq!(count(&text, "ADVANCED_FACE"), shown_faces, "every face of the surface bodies is written");
    let (faces, kinds) = reimport("surf_back", &text);
    assert_eq!(faces, shown_faces);
    let mut want: Vec<String> = shown.iter().flat_map(|s| s.as_arr().unwrap()[1].get_arr("faces").unwrap().iter().map(|f| f.get("surface").and_then(|s| s.get_str("type")).or_else(|| f.get_str("type")).unwrap_or("?").to_string()).collect::<Vec<_>>()).collect();
    want.sort();
    assert_eq!(kinds, want, "same surface types");
}

#[test]
fn solid_and_surface_export_together_with_names() {
    let part = doc(&[RECT, r#"{"type":"extrude","id":"F2","op":"add","sketch":"S","end":"blind","depth":5}"#, OPEN, r#"{"type":"surfaceExtrude","id":"F4","sketch":"T","end":"blind","depth":10}"#]);
    regen("mix", &part);
    let text = step("mix").expect("exports");
    assert_eq!(count(&text, "MANIFOLD_SOLID_BREP"), 1, "{text}");
    assert_eq!(count(&text, "SHELL_BASED_SURFACE_MODEL"), 1);
    assert_eq!(count(&text, "OPEN_SHELL"), 1);
    assert!(text.contains("SHELL_BASED_SURFACE_MODEL('F4'"), "the surface model carries its body's name");
    let (faces, _) = reimport("mix_back", &text);
    assert_eq!(faces, 6 + 2, "the box's six faces and the surface's two");
}

#[test]
fn suppressed_surface_is_not_written() {
    let part = doc(&[RECT, r#"{"type":"extrude","id":"F2","op":"add","sketch":"S","end":"blind","depth":5}"#, OPEN, r#"{"type":"surfaceExtrude","id":"F4","suppressed":true,"sketch":"T","end":"blind","depth":10}"#]);
    regen("supp", &part);
    let text = step("supp").expect("the solid still exports");
    assert_eq!(count(&text, "SHELL_BASED_SURFACE_MODEL"), 0);
    assert_eq!(count(&text, "MANIFOLD_SOLID_BREP"), 1);
}

#[test]
fn empty_document_is_refused_with_a_code_and_a_hint() {
    regen("none", &doc(&[]));
    let e = step("none").expect_err("nothing to export");
    assert!(e.starts_with("stepNothingToExport:"), "{e}");
    assert!(e.contains("Hint:"), "{e}");
    // sketches alone are no body either
    regen("sketch_only", &doc(&[RECT]));
    assert!(step("sketch_only").unwrap_err().starts_with("stepNothingToExport:"));
}
