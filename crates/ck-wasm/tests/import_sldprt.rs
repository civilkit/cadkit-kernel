//! `importSldprt` metadata in the reply: feature list, face attributes keyed by the body's face ids, versions, thumbnail;
//! no recorded user name or folder in the reply; the metadata of a file whose body cannot be read. Fixtures: the parts in
//! crates/ck-sldprt/tests/fixtures (see SOURCES.txt there).
//!
//! `CK_SLDPRT_CORPUS=<dir>` (optional) also imports every `.sldprt` under that directory, checks the same privacy rule and the
//! face-to-feature link, and with `CK_SLDPRT_OUT=<file>` writes one JSON line per file (local use; nothing is committed).

use ck_json::Json;
use ck_sldprt::container;

fn b64(d: &[u8]) -> String {
    ck_text::base64_encode(d)
}

fn import(bytes: &[u8]) -> Json {
    ck_wasm::engine::install();
    let msg = Json::Obj(vec![("type".into(), "importSldprt".into()), ("tag".into(), "t".into()), ("name".into(), "part".into()), ("data".into(), b64(bytes).into())]);
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg)]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap().get("result").unwrap().clone()
}

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!("{}/../ck-sldprt/tests/fixtures/{name}.sldprt", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

const FIXTURES: [&str; 4] = ["sw2011_cube_10mm_ole2", "sw2022_cone", "sw2022_cube_10mm", "sw2022_cube_hole_5mm"];

/// The user names and folder a part file records, read from its raw streams (modern: `SW-Last Saved By`, `SW-Author`,
/// `SW-Folder Name`, core `lastModifiedBy`; OLE2: summary author and last author).
fn recorded_private_strings(data: &[u8]) -> Vec<String> {
    let Ok(c) = container::open(data, &ck_sldprt::Limits::default()) else { return vec![] };
    let mut out = vec![];
    let after = |t: &str, key: &str| t.find(key).map(|p| &t[p + key.len()..]).map(|v| v[..v.find('<').unwrap_or(0)].to_string());
    if let Some(s) = c.stream("docProps/ISolidWorksInformation.xml") {
        let t = String::from_utf8_lossy(&s.data);
        for key in ["SW-Last Saved By", "SW-Author", "SW-Folder Name"] {
            if let Some(p) = t.find(&format!("name=\"{key}\"")) {
                out.extend(after(&t[p..], "<vt:lpstr>"));
            }
        }
    }
    if let Some(s) = c.stream("docProps/core.xml") {
        let t = String::from_utf8_lossy(&s.data).into_owned();
        out.extend(after(&t, "lastModifiedBy>"));
        out.extend(after(&t, "creator>"));
    }
    if let Some(s) = c.streams.iter().find(|s| s.name == "SummaryInformation") {
        let d = &s.data;
        let u32at = |p: usize| d.get(p..p + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).unwrap_or(0);
        let base = u32at(44);
        for k in 0..u32at(base + 4).min(64) {
            let (pid, off) = (u32at(base + 8 + 8 * k), base + u32at(base + 12 + 8 * k));
            if (pid == 4 || pid == 8) && u32at(off) == 0x1E {
                let l = u32at(off + 4);
                out.extend(d.get(off + 8..off + 8 + l).map(|b| String::from_utf8_lossy(b).trim_end_matches('\0').to_string()));
            }
        }
    }
    out.retain(|s| !s.trim().is_empty());
    out
}

/// The reply holds none of the private strings and no absolute Windows path.
fn assert_private(label: &str, data: &[u8], reply: &Json) -> usize {
    // the thumbnail's base64 is image data (letters by chance), not text
    let mut reply = reply.clone();
    if let Some(Json::Obj(t)) = reply.get_mut("import").and_then(|i| i.get_mut("thumbnail")) {
        t.retain(|(k, _)| k != "base64");
    }
    let text = ck_json::to_string(&reply).to_lowercase();
    let private = recorded_private_strings(data);
    for p in &private {
        // JSON escapes a backslash as two
        let p = p.to_lowercase().replace('\\', "\\\\");
        assert!(!text.contains(&p), "{label}: a recorded user name or folder is in the reply");
    }
    let b = text.as_bytes();
    let drive = (1..b.len().saturating_sub(3)).any(|i| b[i].is_ascii_alphabetic() && !b[i - 1].is_ascii_alphanumeric() && b[i + 1] == b':' && b[i + 2] == b'\\' && b[i + 3] == b'\\');
    assert!(!drive, "{label}: an absolute path is in the reply");
    private.len()
}

fn arr<'a>(j: &'a Json, k: &str) -> &'a [Json] {
    j.get(k).and_then(Json::as_arr).unwrap_or(&[])
}

/// Every face attribute names a face of the reply's body, and its feature id is in the feature list.
fn assert_faces_linked(label: &str, r: &Json) -> (usize, usize) {
    let imp = r.get("import").unwrap();
    let ids: Vec<&str> = arr(r.get("body").unwrap(), "faces").iter().filter_map(|f| f.get_str("id")).collect();
    let surf: Vec<&str> = arr(r, "surfaces").iter().filter_map(|s| s.as_arr()?.get(1)).flat_map(|b| arr(b, "faces")).filter_map(|f| f.get_str("id")).collect();
    let feats: Vec<f64> = arr(imp, "features").iter().filter_map(|f| f.get_f64("id")).collect();
    let fa = arr(imp, "faceAttributes");
    let mut linked = 0;
    for a in fa {
        let face = a.get_str("face").unwrap();
        assert!(ids.contains(&face) || surf.contains(&face), "{label}: {face} is not a face of the reply");
        if let Some(id) = a.get_f64("featureId") {
            assert!(feats.contains(&id), "{label}: feature {id} of {face} not in the feature list");
            assert!(a.get_str("feature").is_some(), "{label}: {face} has no feature name");
            linked += 1;
        }
    }
    (fa.len(), linked)
}

#[test]
fn reply_carries_features_face_attributes_and_thumbnail() {
    let mut private = 0;
    for name in FIXTURES {
        let data = fixture(name);
        let r = import(&data);
        assert!(arr(&r, "errors").is_empty(), "{name}: {:?}", r.get("errors"));
        let imp = r.get("import").unwrap();
        assert!(!arr(imp, "features").is_empty(), "{name}");
        let (n, linked) = assert_faces_linked(name, &r);
        assert_eq!(n, imp.get_f64("faces").unwrap() as usize, "{name}");
        assert_eq!(linked, n, "{name}: every face is tagged with its feature");
        assert!(arr(imp, "faceAttributes").iter().all(|a| a.get("colour").and_then(Json::as_arr).map_or(false, |c| c.len() == 3)), "{name}");
        let v = imp.get("versions").unwrap();
        assert!(v.get_f64("solidworks").is_some() && v.get_str("parasolid").is_some(), "{name}: {v:?}");
        let t = imp.get("thumbnail").unwrap();
        assert_eq!((t.get_str("mime"), t.get_f64("width"), t.get_f64("height")), (Some("image/png"), Some(640.0), Some(480.0)), "{name}");
        assert!(t.get_str("base64").unwrap().starts_with("iVBORw0KGgo"), "{name}");
        assert_eq!(arr(imp, "bodyAttributes").len(), 1, "{name}");
        assert_eq!(arr(imp, "configurations").len(), 1, "{name}");
        private += assert_private(name, &data, &r);
    }
    assert!(private >= 5, "the fixtures record user names and folders ({private} checked)");
    // the hole's two features
    let r = import(&fixture("sw2022_cube_hole_5mm"));
    let imp = r.get("import").unwrap();
    let f: Vec<(String, String)> = arr(imp, "features").iter().map(|f| (f.get_str("name").unwrap().to_string(), f.get_str("type").unwrap().to_string())).collect();
    assert!(f.contains(&("Boss-Extrude1".into(), "Boss-Extrude".into())) && f.contains(&("Cut-Extrude1".into(), "Extrusion".into())), "{f:?}");
    let cut = arr(imp, "faceAttributes").iter().filter(|a| a.get_str("feature") == Some("Cut-Extrude1")).count();
    assert_eq!(cut, 1);
    let units = imp.get("units").unwrap();
    assert_eq!((units.get_str("system"), units.get_str("length")), (Some("MMGS"), Some("mm")));
}

/// A file whose geometry cannot be read (its partition block is corrupted) still answers its metadata.
#[test]
fn metadata_without_a_body() {
    let mut data = fixture("sw2022_cube_10mm");
    let swapped: Vec<u8> = "Contents/Config-0-Partition".bytes().map(|b| (b << 4) | (b >> 4)).collect();
    // every copy of the block (a file can keep stale ones)
    let hits: Vec<usize> = data.windows(swapped.len()).enumerate().filter(|(_, w)| *w == &swapped[..]).map(|(i, _)| i + swapped.len()).collect();
    assert!(!hits.is_empty());
    for at in hits {
        for b in &mut data[at + 4..at + 12] {
            *b ^= 0x5A;
        }
    }
    let r = import(&data);
    assert!(!arr(&r, "errors").is_empty());
    assert!(matches!(r.get("body"), Some(Json::Null) | None));
    let imp = r.get("import").expect("metadata of a file without a body");
    assert!(arr(imp, "features").iter().any(|f| f.get_str("name") == Some("Boss-Extrude1")));
    assert!(imp.get("thumbnail").and_then(|t| t.get_str("base64")).is_some());
    assert_private("corrupted", &data, &r);
}

/// Local corpus check (skipped unless `CK_SLDPRT_CORPUS` is set).
#[test]
fn corpus_metadata() {
    let Ok(dir) = std::env::var("CK_SLDPRT_CORPUS") else { return };
    let mut files = vec![];
    let mut stack = vec![std::path::PathBuf::from(dir)];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
                files.push(p);
            }
        }
    }
    files.sort();
    let mut lines = vec![];
    let (mut ok, mut meta_only) = (0, 0);
    for p in &files {
        let data = std::fs::read(p).unwrap();
        let r = import(&data);
        let label = p.file_name().unwrap().to_string_lossy().to_string();
        assert_private(&label, &data, &r);
        let Some(imp) = r.get("import") else { panic!("{label}: no import summary") };
        if arr(&r, "errors").is_empty() {
            let (n, linked) = assert_faces_linked(&label, &r);
            assert_eq!(n, linked, "{label}: untagged faces");
            ok += 1;
        } else {
            meta_only += 1;
        }
        let mut summary = imp.clone();
        if let Json::Obj(kv) = &mut summary {
            kv.retain(|(k, _)| !matches!(k.as_str(), "thumbnail" | "parts"));
            let t = imp.get("thumbnail").and_then(|t| t.get_f64("width")).is_some();
            kv.push(("hasThumbnail".into(), Json::Bool(t)));
            let fa = arr(imp, "faceAttributes");
            kv.push(("taggedFaces".into(), Json::Num(fa.iter().filter(|a| a.get_f64("featureId").is_some()).count() as f64)));
            kv.push(("colouredFaces".into(), Json::Num(fa.iter().filter(|a| a.get("colour").and_then(Json::as_arr).is_some()).count() as f64)));
            kv.push(("file".into(), Json::Str(p.parent().and_then(|d| d.file_name()).unwrap_or_default().to_string_lossy().to_string())));
            kv.push(("ok".into(), Json::Bool(arr(&r, "errors").is_empty())));
        }
        lines.push(ck_json::to_string(&summary));
    }
    println!("corpus: {} files, {ok} with a body, {meta_only} metadata only", files.len());
    if let Ok(out) = std::env::var("CK_SLDPRT_OUT") {
        if let Some(d) = std::path::Path::new(&out).parent() {
            std::fs::create_dir_all(d).unwrap();
        }
        std::fs::write(out, lines.join("\n") + "\n").unwrap();
    }
}
