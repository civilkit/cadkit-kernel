//! `sldprtHistory` on SolidWorks sheet metal parts: the `Sheet-Metal` rules and the `Base-Flange` become `sheetRules` and
//! `baseFlange`, every `Edge-Flange` (with its `EdgeBend`) an `edgeFlange` of any width and outline, the cuts through the sheet
//! `sheetCut`s (or part of the outline); what cannot be read back is the imported remainder.
//!
//! The files are not part of the repository; each test is skipped when its file is absent (the directory can be set with
//! `CK_SLDPRT_SHEET_DIR`).

use ck_json::Json;

/// Relative volume tolerance of the app's acceptance check (the kit's `VOLUME_TOLERANCE`).
const VOLUME_TOLERANCE: f64 = 0.005;

fn request(msg: Vec<(String, Json)>) -> Json {
    ck_wasm::engine::install();
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), Json::Obj(msg))]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn file(name: &str) -> Option<Vec<u8>> {
    let dir = std::env::var("CK_SLDPRT_SHEET_DIR").unwrap_or_else(|_| "/path/to/Downloads/2026-01-05_Spinfire_Asm".to_string());
    std::fs::read(format!("{dir}/{name}")).ok()
}

fn num(r: &Json, path: &[&str]) -> Option<f64> {
    let mut j = r;
    for k in path {
        j = j.get(k)?;
    }
    j.as_f64()
}

fn check(name: &str) {
    let Some(bytes) = file(name) else {
        eprintln!("skipped: {name} not found");
        return;
    };
    let h = request(vec![("type".into(), "sldprtHistory".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    let h = h.get("result").unwrap_or_else(|| panic!("no result: {}", ck_json::to_string(&h))).clone();
    assert_eq!(h.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", ck_json::to_string(&h).chars().take(2000).collect::<String>());
    let part = h.get("part").unwrap().clone();
    let feats = part.get("features").and_then(|f| f.as_arr()).unwrap();
    let types: Vec<&str> = feats.iter().filter_map(|f| f.get_str("type")).collect();
    assert!(types.contains(&"sheetRules"), "{types:?}");
    assert!(types.contains(&"baseFlange") || types.contains(&"contourFlange"), "{types:?}");
    assert!(!types.contains(&"importStep"), "{types:?}");
    // the rules as the file stores them
    let rules = feats.iter().find(|f| f.get_str("type") == Some("sheetRules")).unwrap();
    assert_eq!(rules.get("thickness").and_then(|v| v.as_f64()), Some(3.0));
    assert_eq!(rules.get("bendRadius").and_then(|v| v.as_f64()), Some(2.0));
    assert_eq!(rules.get("kFactor").and_then(|v| v.as_f64()), Some(0.5));
    // the document regenerates without errors to the body the plain import reads (within the app's tolerance)
    let rg = request(vec![("type".into(), "regen".into()), ("tag".into(), "h".into()), ("part".into(), part.clone())]);
    let res = rg.get("result").unwrap_or_else(|| panic!("no regen result: {}", ck_json::to_string(&rg)));
    let errs = res.get("errors").and_then(|e| e.as_arr()).unwrap();
    assert!(errs.is_empty(), "{}", ck_json::to_string(res).chars().take(2000).collect::<String>());
    let imp = request(vec![("type".into(), "importSldprt".into()), ("tag".into(), "p".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    assert!(imp.get("result").is_some());
    let vh = num(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "h".into())]), &["result", "volume"]).unwrap();
    let vp = num(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "p".into())]), &["result", "volume"]).unwrap();
    eprintln!("{name}: history volume {vh}, file volume {vp}");
    assert!((vh - vp).abs() <= VOLUME_TOLERANCE * vp.abs(), "{vh} vs {vp}");
    // the flat pattern of the sheet the base flange makes
    let sm = res.get("sheetMetal").and_then(|s| s.as_arr()).unwrap_or_else(|| panic!("no sheetMetal: {}", ck_json::to_string(res).chars().take(2000).collect::<String>()));
    let base = sm.iter().find(|e| e.get_str("type") == Some("baseFlange")).expect("a baseFlange entry");
    let fp = base.get("flatPattern").expect("flat pattern");
    assert_eq!(fp.get("thickness").and_then(|v| v.as_f64()), Some(3.0));
    assert!(fp.get("outline").and_then(|o| o.as_arr()).is_some_and(|o| !o.is_empty()));
}

#[test]
fn chute_v3_is_sheet_metal() {
    check("20260105_FRY002_S3_1068_BRUSHLESS_CHUTE_V3.SLDPRT");
}

#[test]
fn chute_v4_is_sheet_metal() {
    check("20260105_FRY002_S3_1068_BRUSHLESS_CHUTE_V4.SLDPRT");
}

#[test]
fn chute_v5_is_sheet_metal() {
    check("20260105_FRY002_S3_1068_BRUSHLESS_CHUTE_V5.SLDPRT");
}

/// The history of a chute, read back to the end: `flanges` edge flanges (each with its shaped leg), no imported remainder, and a
/// body that agrees with the file's to `1e-6` of the volume (the app accepts 0.5 %).
fn full_history(name: &str, flanges: usize) {
    let Some(bytes) = file(name) else {
        eprintln!("skipped: {name} not found");
        return;
    };
    let h = request(vec![("type".into(), "sldprtHistory".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    let h = h.get("result").unwrap_or_else(|| panic!("no result: {}", ck_json::to_string(&h))).clone();
    let oc = h.get("outcome").cloned().unwrap_or(Json::Null);
    assert_eq!(h.get("ok").and_then(|v| v.as_bool()), Some(true), "{}", ck_json::to_string(&oc.get("fallback").cloned().unwrap_or(Json::Null)));
    assert!(oc.get("fallback").is_none_or(|f| matches!(f, Json::Null)), "history stops: {}", ck_json::to_string(oc.get("fallback").unwrap()));
    let part = h.get("part").unwrap().clone();
    let feats = part.get("features").and_then(|f| f.as_arr()).unwrap();
    let types: Vec<&str> = feats.iter().filter_map(|f| f.get_str("type")).collect();
    assert!(!types.contains(&"importBody") && !types.contains(&"importStep"), "{types:?}");
    let fl: Vec<&Json> = feats.iter().filter(|f| f.get_str("type") == Some("edgeFlange")).collect();
    assert_eq!(fl.len(), flanges, "{types:?}");
    for f in &fl {
        assert!(f.get("profile").is_some_and(|p| p.as_arr().is_some_and(|a| a.len() >= 3)), "every flange has its outline");
        assert!(f.get("edges").and_then(|e| e.as_arr()).is_some_and(|e| e[0].get("from").is_some() && e[0].get("to").is_some()), "and its span");
    }
    // regenerates without errors; the volume is the file's
    let rg = request(vec![("type".into(), "regen".into()), ("tag".into(), "h".into()), ("part".into(), part.clone())]);
    let res = rg.get("result").unwrap_or_else(|| panic!("no regen result: {}", ck_json::to_string(&rg)));
    let errs = res.get("errors").and_then(|e| e.as_arr()).unwrap();
    assert!(errs.is_empty(), "{}", ck_json::to_string(res).chars().take(2000).collect::<String>());
    let imp = request(vec![("type".into(), "importSldprt".into()), ("tag".into(), "p".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    assert!(imp.get("result").is_some());
    let vh = num(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "h".into())]), &["result", "volume"]).unwrap();
    let vp = num(&request(vec![("type".into(), "massProps".into()), ("tag".into(), "p".into())]), &["result", "volume"]).unwrap();
    let err = (vh - vp).abs() / vp.abs();
    let steps = oc.get("steps").and_then(|s| s.as_arr()).unwrap();
    let rebuilt = steps.iter().filter(|s| s.get_str("status") == Some("rebuilt")).count();
    let skipped = steps.iter().filter(|s| s.get_str("status") == Some("skipped")).count();
    eprintln!("{name}: {} edge flange(s) of {} Edge-Flange step(s) ({} with faces), {rebuilt} steps rebuilt, {skipped} skipped; history volume {vh}, file volume {vp}, error {err:.2e}", fl.len(), steps.iter().filter(|s| s.get_str("kind") == Some("Edge-Flange")).count(), steps.iter().filter(|s| s.get_str("kind") == Some("Edge-Flange") && s.get_f64("faces").unwrap_or(0.0) > 0.0).count());
    assert!(err < 1e-6, "{vh} vs {vp}");
    // the flat pattern has one bend row per flange
    let sm = res.get("sheetMetal").and_then(|s| s.as_arr()).unwrap();
    // (entries are sorted by id as text: the last made is the one with the largest number)
    let num_id = |e: &&Json| e.get_str("id").and_then(|i| i.trim_start_matches('F').parse::<usize>().ok()).unwrap_or(0);
    let last = sm.iter().filter(|e| e.get("flatPattern").is_some()).max_by_key(num_id).expect("a flat pattern");
    let bends = last.get("flatPattern").unwrap().get_arr("bends").unwrap();
    assert_eq!(bends.len(), flanges, "one bend row per flange");
}

/// V5: five edge flanges (a 90 degree one with a round hole in its leg, a 60 degree trapezoid, a 90 degree one on the slanted
/// edge with an arc for its tip, a 20.5 degree one on the leg of that, and an 86.9 degree one on the slanted edge), the
/// sheet cuts and the slots; the eleven later Edge-Flange steps made no faces on the file's body (suppressed in this
/// configuration) and are skipped.
#[test]
fn chute_v5_history_to_the_end() {
    full_history("20260105_FRY002_S3_1068_BRUSHLESS_CHUTE_V5.SLDPRT", 5);
}

/// V4: seven edge flanges, the cuts and the slots.
#[test]
fn chute_v4_history_to_the_end() {
    full_history("20260105_FRY002_S3_1068_BRUSHLESS_CHUTE_V4.SLDPRT", 7);
}

/// Prints the steps (and the history's own report) of a file: `CK_DUMP=name cargo test -p ck-wasm --test sldprt_sheet dump -- --nocapture`.
#[test]
fn dump() {
    let Ok(name) = std::env::var("CK_DUMP") else { return };
    let Some(bytes) = file(&name) else { return };
    let h = request(vec![("type".into(), "sldprtHistory".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    let h = h.get("result").unwrap().clone();
    let oc = h.get("outcome").cloned().unwrap_or(Json::Null);
    if let Some(steps) = oc.get("steps").and_then(|s| s.as_arr()) {
        for s in steps {
            eprintln!("STEP {}", ck_json::to_string(s));
        }
    }
    if let Some(fs) = oc.get("features").and_then(|f| f.as_arr()) {
        for f in fs {
            let brief = ck_json::to_string(f);
            eprintln!("FEATURE {}", brief.chars().take(if std::env::var("CK_DUMP_FULL").is_ok() { 4000 } else { 160 }).collect::<String>());
        }
    }
    if let Some(cs) = oc.get("checks").and_then(|f| f.as_arr()) {
        for (i, c) in cs.iter().enumerate() {
            eprintln!("CHECK {i} {}", ck_json::to_string(c).chars().take(500).collect::<String>());
        }
    }
    for k in ["fallback", "prefix", "exact", "close", "errors", "notes", "verify"] {
        if let Some(v) = oc.get(k) {
            eprintln!("{k}: {}", ck_json::to_string(v).chars().take(600).collect::<String>());
        }
    }
}

/// Prints every feature with its faces (surface, normal / axis, extent): `CK_GEO=name cargo test ... geo -- --nocapture`.
#[test]
fn geo() {
    let Ok(name) = std::env::var("CK_GEO") else { return };
    let Some(bytes) = file(&name) else { return };
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let t = ck_swedit::Tagged::from_import(&imp);
    let m = ck_recog::model::Model::new(t.body);
    eprintln!("unit_mm {}", t.unit_mm);
    for f in t.features {
        let dims: Vec<String> = f.dims.iter().map(|d| format!("{}={:?}", d.name, d.value)).collect();
        let faces: Vec<_> = m.faces.iter().copied().filter(|g| t.tags[g.idx()].iter().any(|x| x.0 == f.id)).collect();
        eprintln!("FEAT {} {} [{}] faces={} dims: {}", f.id, f.name, f.kind, faces.len(), dims.join(" "));
        if std::env::var("CK_GEO_FACES").is_err() || !(f.kind.contains("Flange") || f.kind.contains("Cut") || f.kind.contains("Bend") || f.kind.contains("Fillet") || f.kind.contains("Hole")) {
            continue;
        }
        for g in faces {
            let (lo, hi) = t.body.bbox_of_faces(&[g]);
            let k = match m.kind(g) {
                ck_recog::model::Kind::Plane { n, d } => format!("plane n=({:.3},{:.3},{:.3}) d={:.3}", n.x, n.y, n.z, d),
                ck_recog::model::Kind::Cyl { o, a, r, convex } => format!("cyl a=({:.3},{:.3},{:.3}) o=({:.2},{:.2},{:.2}) r={:.3} convex={}", a.x, a.y, a.z, o.x, o.y, o.z, r, convex),
                other => other.name().to_string(),
            };
            eprintln!("   FACE {} {} area={:.2} box=({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2}) tags={:?}", g.idx(), k, m.area(g), lo.x, lo.y, lo.z, hi.x, hi.y, hi.z, t.tags[g.idx()]);
        }
    }
}

struct Exec(ck_model::features::Brep);
impl ck_model::regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &ck_model::regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut ck_model::regen::RegenState, f: &ck_model::Feature) -> Result<(), ck_model::regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// Faces of the rebuilt history that pair with no face of the file's body, and the file's faces left over:
/// `CK_DIFF=name cargo test -p ck-wasm --test sldprt_sheet diff -- --nocapture`.
#[test]
fn diff() {
    let Ok(name) = std::env::var("CK_DIFF") else { return };
    let Some(bytes) = file(&name) else { return };
    let h = request(vec![("type".into(), "sldprtHistory".into()), ("data".into(), ck_text::base64_encode(&bytes).into())]);
    let h = h.get("result").unwrap().clone();
    let feats = h.get("outcome").and_then(|o| o.get("features")).cloned().unwrap();
    let part = ck_model::Part::from_json(&Json::Obj(vec![("format".into(), "ck-part".into()), ("version".into(), Json::Num(1.0)), ("features".into(), feats)])).unwrap();
    let mut ex = Exec(ck_model::features::Brep::new());
    let r = ck_model::regen::regen(&part, &ck_model::regen::RegenOpts::default(), &mut ex);
    eprintln!("regen errors: {:?}", r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect::<Vec<_>>());
    let nb = ex.0.body.as_ref().expect("a body");
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let ob = &imp.body;
    let (mo, mn) = (ck_recog::model::Model::new(ob), ck_recog::model::Model::new(nb));
    eprintln!("faces: file {} rebuilt {}", mo.faces.len(), mn.faces.len());
    let mut free: Vec<ck_topo::body::FaceId> = mn.faces.clone();
    let mut left_o = vec![];
    for &f in &mo.faces {
        let a = mo.area(f);
        match free.iter().position(|&g| (mn.area(g) - a).abs() <= 1e-6 * a.abs().max(1.0) && ck_swedit::fastverify::same_surface(mo.kind(f), mn.kind(g), 1e-6)) {
            Some(i) => {
                free.swap_remove(i);
            }
            None => left_o.push(f),
        }
    }
    let show = |m: &ck_recog::model::Model, b: &ck_topo::body::Body, g: ck_topo::body::FaceId| {
        let (lo, hi) = b.bbox_of_faces(&[g]);
        let k = match m.kind(g) {
            ck_recog::model::Kind::Plane { n, d } => format!("plane n=({:.3},{:.3},{:.3}) d={:.3}", n.x, n.y, n.z, d),
            ck_recog::model::Kind::Cyl { o, a, r, .. } => format!("cyl a=({:.3},{:.3},{:.3}) o=({:.2},{:.2},{:.2}) r={:.3}", a.x, a.y, a.z, o.x, o.y, o.z, r),
            other => other.name().to_string(),
        };
        format!("{k} area={:.4} box=({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})", m.area(g), lo.x, lo.y, lo.z, hi.x, hi.y, hi.z)
    };
    for &f in &left_o {
        eprintln!("FILE-ONLY {}", show(&mo, ob, f));
    }
    for &g in &free {
        eprintln!("REBUILT-ONLY {}", show(&mn, nb, g));
    }
}
