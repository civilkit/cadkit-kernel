//! `sldasmTree`: the component tree of a SolidWorks assembly through the request protocol, and its `insertPart` part
//! document. A synthetic assembly container (no third-party file): a top assembly holding a part twice (one suppressed)
//! and a sub-assembly holding the part once.

use ck_json::Json;

fn request(msg: Vec<(String, Json)>) -> Json {
    ck_wasm::engine::install();
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), Json::Obj(msg))]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn container_with(name: &str, data: &[u8]) -> Vec<u8> {
    let mut out = vec![0, 0, 0, 0, 0, 0, 0, 4];
    out.extend_from_slice(&[0x14, 0x00, 0x06, 0x00, 0x08, 0x00]);
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&ck_sldprt::inflate::crc32(data).to_le_bytes());
    out.extend_from_slice(&((5 + data.len()) as u32).to_le_bytes());
    out.extend_from_slice(&(data.len() as u32).to_le_bytes());
    out.extend_from_slice(&(name.len() as u32).to_le_bytes());
    out.extend(name.bytes().map(|b| (b << 4) | (b >> 4)));
    out.push(1);
    out.extend_from_slice(&(data.len() as u16).to_le_bytes());
    out.extend_from_slice(&(!(data.len() as u16)).to_le_bytes());
    out.extend_from_slice(data);
    out.extend_from_slice(&[0; 16]);
    out
}

fn tree() -> String {
    // 90 degrees about z for row vectors, origin (0.1, 0, 0) m
    let rz = "0 1 0 0 -1 0 0 0 0 0 1 0 0.1 0 0 1";
    let id = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
    let r = |n: u32, mref: &str, supp: &str, tf: &str, who: &str| format!(r#"<swReference id="{who}{n}" swComponentName="{who}" swReferenceNumber="{n}" swConfigurationName="Default" swSuppressed="{supp}" swHidden="NO" swModelRef="{mref}" swTransform="{tf}"/>"#);
    format!(
        r#"<?xml version="1.0"?><swSolidWorks><swHeader><swFile id="3" swDocType="ASSEMBLY" swPath="C:\x\TOP.SLDASM"/><swFile id="6" swDocType="PART" swPath="C:\x\Plate.SLDPRT"/><swFile id="9" swDocType="ASSEMBLY" swPath="C:\x\SUB.SLDASM"/></swHeader><swModelList>
<swModel id="5" swName="Plate" swConfigurationName="Default" swFileRef="6"/>
<swModel id="8" swName="SUB" swConfigurationName="Default" swFileRef="9">{sub}</swModel>
<swModel id="2" swName="TOP" swConfigurationName="Default" swFileRef="3">{a}{b}{c}</swModel></swModelList>
<swConfigurationList><swConfiguration id="1" swName="Default" swMostRecentConfiguration="YES" swModelRef="2"/></swConfigurationList></swSolidWorks>"#,
        sub = r(1, "5", "NO", id, "Plate"),
        a = r(1, "5", "NO", id, "Plate"),
        b = r(2, "5", "YES", id, "Plate"),
        c = r(1, "8", "NO", rz, "SUB"),
    )
}

fn num(j: &Json, i: usize) -> f64 {
    j.as_arr().unwrap()[i].as_f64().unwrap()
}

#[test]
fn tree_and_part_document() {
    let file = container_with("swXmlContents/COMPINSTANCETREE", tree().as_bytes());
    let r = request(vec![("type".into(), "sldasmTree".into()), ("data".into(), ck_text::base64_encode(&file).into()), ("asPart".into(), Json::Bool(true))]);
    let r = r.get("result").unwrap_or_else(|| panic!("{}", ck_json::to_string(&r))).clone();
    assert_eq!(r.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("name").and_then(|v| v.as_str()), Some("TOP"));
    assert_eq!(r.get("count").and_then(|v| v.as_f64()), Some(4.0));
    let comps = r.get("components").and_then(|c| c.as_arr()).unwrap();
    assert_eq!(comps.len(), 3);
    assert_eq!(comps[1].get("suppressed").and_then(|v| v.as_bool()), Some(true));
    assert_eq!(comps[2].get("children").and_then(|c| c.as_arr()).unwrap().len(), 1);
    assert_eq!(comps[0].get("part").and_then(|v| v.as_str()), Some("Plate.SLDPRT"));
    // the part document: the suppressed component is skipped, the sub-assembly's part is placed by the composed transform
    let part = r.get("part").unwrap();
    let feats = part.get("features").and_then(|f| f.as_arr()).unwrap();
    assert_eq!(feats.len(), 2);
    assert_eq!(feats[0].get("type").and_then(|v| v.as_str()), Some("insertPart"));
    assert_eq!(feats[0].get("unresolved").and_then(|v| v.as_bool()), Some(true));
    let place = feats[1].get("place").unwrap();
    assert!((num(place.get("o").unwrap(), 0) - 100.0).abs() < 1e-9);
    // the part's x axis lands on +y in the assembly
    assert!((num(place.get("x").unwrap(), 1) - 1.0).abs() < 1e-9 && num(place.get("x").unwrap(), 0).abs() < 1e-9);
    assert_eq!(r.get("unresolved").and_then(|u| u.as_arr()).unwrap().len(), 1);
    // the document is valid for the kernel: it regenerates, the unresolved parts adding nothing and no error
    let rg = request(vec![("type".into(), "regen".into()), ("tag".into(), "a".into()), ("part".into(), part.clone())]);
    let errs = rg.get("result").and_then(|r| r.get("errors")).and_then(|e| e.as_arr()).unwrap();
    assert!(errs.is_empty(), "{}", ck_json::to_string(&rg));
}

#[test]
fn unreadable_files_answer_ok_false_with_a_code() {
    for bytes in [vec![1u8, 2, 3, 4], container_with("Header2", b"x")] {
        let r = request(vec![("type".into(), "sldasmTree".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
        let r = r.get("result").unwrap();
        assert_eq!(r.get("ok").and_then(|v| v.as_bool()), Some(false));
        assert!(r.get("code").and_then(|v| v.as_str()).unwrap().contains('.'));
    }
}

/// Every `.SLDASM` under `SLDASM_CORPUS` (skipped when unset): the tree answers, and its `asPart` document regenerates without errors.
#[test]
fn corpus_assemblies_answer_and_regenerate() {
    let Ok(dir) = std::env::var("SLDASM_CORPUS") else { return };
    let mut n = 0;
    for e in std::fs::read_dir(dir).unwrap().flatten() {
        let p = e.path();
        if !p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldasm")) {
            continue;
        }
        let bytes = std::fs::read(&p).unwrap();
        let t0 = std::time::Instant::now();
        let r = request(vec![("type".into(), "sldasmTree".into()), ("data".into(), ck_text::base64_encode(&bytes).into()), ("asPart".into(), Json::Bool(true))]);
        let ms = t0.elapsed().as_millis();
        let r = r.get("result").unwrap().clone();
        assert_eq!(r.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", p.display());
        let feats = r.get("part").and_then(|p| p.get("features")).and_then(|f| f.as_arr()).unwrap().len();
        let t1 = std::time::Instant::now();
        let rg = request(vec![("type".into(), "regen".into()), ("tag".into(), "a".into()), ("part".into(), r.get("part").unwrap().clone())]);
        let errs = rg.get("result").and_then(|r| r.get("errors")).and_then(|e| e.as_arr()).unwrap();
        assert!(errs.is_empty(), "{}: {}", p.display(), ck_json::to_string(&rg));
        eprintln!("sldasm {}: tree {ms} ms, {feats} insertPart features, regen {} ms", n, t1.elapsed().as_millis());
        n += 1;
    }
    eprintln!("{n} assemblies");
}
