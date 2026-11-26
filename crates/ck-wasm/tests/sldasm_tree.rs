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

fn container_multi(streams: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = vec![0, 0, 0, 0, 0, 0, 0, 4];
    for (name, data) in streams {
        let one = container_with(name, data);
        out.extend_from_slice(&one[8..]);
    }
    out
}

fn mstr(s: &str) -> Vec<u8> {
    let mut v = vec![0xff, 0xfe, 0xff, s.chars().count() as u8];
    for u in s.encode_utf16() {
        v.extend_from_slice(&u.to_le_bytes());
    }
    v
}

/// One mate record of the `Config-N-MatesList` object stream (class declared, name, 19 byte header, operand strings).
fn mate_rec(out: &mut Vec<u8>, class: &str, name: &str, id: u32, suppressed: bool, ops: &[String]) {
    out.extend_from_slice(&[0xff, 0xff, 1, 0]);
    out.extend_from_slice(&(class.len() as u16).to_le_bytes());
    out.extend_from_slice(class.as_bytes());
    out.extend_from_slice(&[0x20, 0x80]);
    out.extend(mstr(name));
    out.extend_from_slice(&[0, 1, 0, 0, 0, 0, 0]);
    out.extend_from_slice(&((id << 8) | 0xc0).to_le_bytes());
    out.extend_from_slice(&[0; 5]);
    out.extend_from_slice(&[0xff, 0xfe, 0xff]);
    out.extend_from_slice(&[0; 13]);
    out.push(suppressed as u8);
    out.extend_from_slice(&[0; 8]);
    for o in ops {
        out.extend_from_slice(&[0, 0, 0, 0]);
        out.extend(mstr(o));
    }
    out.extend_from_slice(&[0; 6]);
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
<swModel id="2" swName="TOP" swConfigurationName="Default" swConfigurationId="0" swFileRef="3">{a}{b}{c}</swModel></swModelList>
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

/// Mates in the reply: type, operands (component paths through the sub-assembly), the suppressed flag and the geometric check.
#[test]
fn mates_are_listed_and_checked() {
    let two = |a: &str, pa: &str, b: &str, pb: &str| vec![format!("{a}@TOP"), format!("{pa}@{a}@TOP"), b.to_string(), format!("{pb}@{b}")];
    let mut m = vec![0x58, 1, 0, 0, 3, 0];
    // Plate-1 (identity) with SUB-1 (90 degrees about z, 100 mm along x); the sub-assembly's part is `SUB-1/Plate-1` = `Plate-1@TOP/SUB-1`
    mate_rec(&mut m, "moMateCoincident", "Coincident1", 0x20, false, &two("Plate-1", "Front Plane", "Plate-1@TOP/SUB-1", "Front Plane"));
    mate_rec(&mut m, "moMateParallel", "Parallel1", 0x21, false, &two("Plate-1", "Front Plane", "Plate-1@TOP/SUB-1", "Front Plane"));
    mate_rec(&mut m, "moMatePerpendicular", "Perpendicular1", 0x22, false, &two("Plate-1", "Right Plane", "Plate-1@TOP/SUB-1", "Right Plane"));
    mate_rec(&mut m, "moMateParallel", "Parallel2", 0x23, false, &two("Plate-1", "Right Plane", "Plate-1@TOP/SUB-1", "Right Plane"));
    mate_rec(&mut m, "moMateConcentric", "Concentric1", 0x24, true, &["moFromSktEntSurfIdRep_c,100,2, \n".to_string(), "Plate-1@TOP".to_string()]);
    let file = container_multi(&[("swXmlContents/COMPINSTANCETREE", tree().as_bytes()), ("Contents/Config-0-MatesList", &m)]);
    let r = request(vec![("type".into(), "sldasmTree".into()), ("data".into(), ck_text::base64_encode(&file).into()), ("asPart".into(), Json::Bool(true))]);
    let r = r.get("result").unwrap_or_else(|| panic!("{}", ck_json::to_string(&r))).clone();
    assert_eq!(r.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("matesStream").and_then(|v| v.as_str()), Some("Config-0-MatesList"));
    let ms = r.get("mates").and_then(|m| m.as_arr()).unwrap();
    assert_eq!(ms.len(), 5);
    let st = |i: usize| ms[i].get("check").and_then(|c| c.get("status")).and_then(|v| v.as_str()).unwrap().to_string();
    assert_eq!(ms[0].get("type").and_then(|v| v.as_str()), Some("coincident"));
    assert_eq!(ms[0].get("b").and_then(|b| b.get("component")).and_then(|v| v.as_str()), Some("SUB-1/Plate-1"));
    assert_eq!((st(0), st(1), st(2)), ("holds".to_string(), "holds".to_string(), "holds".to_string()));
    // Right planes of Plate-1 (normal x) and of the rotated part (normal y) are not parallel
    assert_eq!(st(3), "violated");
    assert_eq!(st(4), "unchecked");
    assert_eq!(ms[4].get("suppressed").and_then(|v| v.as_bool()), Some(true));
    let sum = r.get("mateSummary").unwrap();
    assert_eq!(sum.get("holds").and_then(|v| v.as_f64()), Some(3.0));
    assert_eq!(sum.get("violated").and_then(|v| v.as_f64()), Some(1.0));
    // no kernel mate is written: the faces cannot be named
    assert_eq!(r.get("mateMapping").and_then(|m| m.get("mapped")).and_then(|v| v.as_f64()), Some(0.0));
    // `mates:false` leaves the whole block out
    let r2 = request(vec![("type".into(), "sldasmTree".into()), ("data".into(), ck_text::base64_encode(&file).into()), ("mates".into(), Json::Bool(false))]);
    assert!(r2.get("result").unwrap().get("mates").is_none());
}
