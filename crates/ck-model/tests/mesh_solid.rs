//! `meshToSolid`: a triangle mesh held by the part (`blobs` table, base64, either layout of `ck_mesh::blob`), converted to a
//! solid on regeneration (`meshToSolid`), cached in the `built` table, reported as the warning `meshToSolidReport`,
//! refused with the refusal codes of the conversion (with `positions` and `holes`), previewed with `preview: true`.
use ck_json::Json;
use ck_model::features::built_cache::{self, Source};
use ck_model::features::import_step::blob_key;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

struct Out {
    body: Option<Body>,
    errors: Vec<(String, String, Json)>,
    warnings: Vec<(String, String, Json)>,
}

impl Out {
    fn error(&self, code: &str) -> &Json {
        &self.errors.iter().find(|e| e.1 == code).unwrap_or_else(|| panic!("no error {code}: {:?}", self.errors.iter().map(|e| &e.1).collect::<Vec<_>>())).2
    }
    fn warning(&self, code: &str) -> &Json {
        &self.warnings.iter().find(|e| e.1 == code).unwrap_or_else(|| panic!("no warning {code}: {:?}", self.warnings.iter().map(|e| &e.1).collect::<Vec<_>>())).2
    }
}

fn run_part(part: &Part) -> Out {
    built_cache::forget_all();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(part, &RegenOpts::default(), &mut ex);
    let conv = |v: &[regen::RegenError]| v.iter().map(|e| (e.feature.clone(), e.error.code.clone(), Json::Obj(e.error.params.clone()))).collect();
    Out { body: ex.0.body.clone(), errors: conv(&r.errors), warnings: conv(&r.warnings) }
}

fn run_doc(doc: &str) -> Out {
    run_part(&Part::parse(doc).unwrap())
}

type Mesh = (Vec<f32>, Vec<u32>);

/// Cube `[0,s]^3`, outward.
fn cube(s: f32) -> Mesh {
    let mut p = vec![];
    for k in 0..8u32 {
        p.extend([s * (k & 1) as f32, s * ((k >> 1) & 1) as f32, s * ((k >> 2) & 1) as f32]);
    }
    let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
    let mut ix = vec![];
    for q in quads {
        ix.extend([q[0], q[1], q[2], q[0], q[2], q[3]]);
    }
    (p, ix)
}

/// The cube without its top (the two triangles of quad 1).
fn open_cube() -> Mesh {
    let (p, ix) = cube(10.0);
    (p, ix.chunks(3).enumerate().filter(|(i, _)| !(2..4).contains(i)).flat_map(|(_, t)| t.iter().copied()).collect())
}

fn text_of(m: &Mesh) -> String {
    ck_text::base64_encode(&ck_mesh::blob::encode_raw(&m.0, &m.1))
}

/// A part whose `blobs` table holds `text`, and `built` entries if given, with `features` (`{key}` replaced by the blob's key).
fn doc_text(text: &str, features: &str, built: &[(String, String)]) -> String {
    let key = blob_key(text);
    let b = built.iter().map(|(k, v)| format!(r#""{k}":"{v}""#)).collect::<Vec<_>>().join(",");
    format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":"{text}"}},"built":{{{b}}},"features":[{}]}}"#, features.replace("{key}", &key))
}

fn doc(m: &Mesh, features: &str) -> String {
    doc_text(&text_of(m), features, &[])
}

const F: &str = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}"}"#;
const F_CLOSE: &str = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"close":true}}"#;

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

#[test]
fn a_closed_cube_builds_and_reports() {
    let o = run_doc(&doc(&cube(10.0), F));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.clone().expect("body");
    assert_eq!(b.num_faces(), 6);
    assert!((vol(&b) - 1000.0).abs() < 1e-6, "{}", vol(&b));
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    let mut names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    names.sort();
    assert_eq!(names, ["F1.I1F1", "F1.I1F2", "F1.I1F3", "F1.I1F4", "F1.I1F5", "F1.I1F6"]);
    let w = o.warning("meshToSolidReport");
    assert!(w.get_str("detail").unwrap().contains("12 triangles became 6 faces"), "{w:?}");
    let r = w.get("report").expect("report");
    assert_eq!(r.get_str("format"), Some("mesh"));
    assert_eq!(r.get_str("name"), Some("Scan"));
    assert_eq!(r.get_f64("faces"), Some(6.0));
    assert_eq!(r.path(&["faceTypes", "plane"]).and_then(|v| v.as_f64()), Some(6.0));
    assert!(r.get("deviation").is_some() && r.path(&["volume", "relative"]).is_some());
    assert!(r.get("closing").is_none());
}

#[test]
fn an_open_mesh_is_refused_with_the_gap_positions() {
    let o = run_doc(&doc(&open_cube(), F));
    assert!(o.body.is_none());
    let e = o.error("meshNotWatertight");
    assert!(e.get_str("hint").unwrap().contains("opts.close"));
    assert_eq!(e.get_str("detail"), e.get_str("message"));
    let pos = e.get_arr("positions").expect("positions");
    assert_eq!(pos.len(), 4, "the four edges of the hole");
    // every listed edge midpoint is on the open rim z = 10
    assert!(pos.iter().all(|p| p.at(2).and_then(|z| z.as_f64()) == Some(10.0)), "{pos:?}");
    assert_eq!(e.get_f64("openEdges"), Some(4.0));
    assert!(o.warnings.iter().all(|w| w.1 != "meshToSolidReport"));
}

#[test]
fn closing_builds_it_and_the_report_says_what_was_added() {
    let o = run_doc(&doc(&open_cube(), F_CLOSE));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.clone().unwrap();
    assert!((vol(&b) - 1000.0).abs() < 1e-6);
    let w = o.warning("meshToSolidReport");
    assert!(w.get_str("detail").unwrap().contains("closing filled 1 hole"), "{w:?}");
    let c = w.path(&["report", "closing"]).expect("closing");
    assert_eq!(c.get_f64("filledHoles"), Some(1.0));
    let at = c.path(&["holes"]).and_then(|h| h.at(0)).and_then(|h| h.get("at")).and_then(|a| a.as_vec3()).unwrap();
    assert!((at[0] - 5.0).abs() < 1e-9 && (at[1] - 5.0).abs() < 1e-9 && (at[2] - 10.0).abs() < 1e-9, "{at:?}");
    // the report does not carry the triangles (that is the preview)
    assert!(c.path(&["holes"]).and_then(|h| h.at(0)).and_then(|h| h.get("fill")).is_none());
}

#[test]
fn a_hole_too_large_is_refused_with_holes_and_positions() {
    let f = r#"{"type":"meshToSolid","id":"F1","blob":"{key}","opts":{"close":{"maxHoleEdges":3}}}"#;
    let o = run_doc(&doc(&open_cube(), f));
    let e = o.error("meshHoleTooLarge");
    assert!(e.get_arr("positions").is_some_and(|p| !p.is_empty()));
    let holes = e.get_arr("holes").expect("holes");
    assert_eq!(holes.len(), 1);
    assert_eq!(holes[0].get_str("reason"), Some("edges"));
    assert!(holes[0].get("at").is_some());
}

/// Build once; the saved entry the process remembers.
fn built(m: &Mesh, features: &str) -> (Body, String, String) {
    let o = run_doc(&doc(m, features));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let k = built_cache::kept();
    assert_eq!(k.len(), 1);
    assert_eq!(k[0].source, Source::Built);
    (o.body.unwrap(), k[0].key.clone(), ck_text::base64_encode(&k[0].bytes))
}

#[test]
fn a_saved_result_is_read_not_built_and_keeps_the_report() {
    let m = open_cube();
    let (b1, key, data) = built(&m, F_CLOSE);
    let built_doc = doc_text(&text_of(&m), F_CLOSE, &[(key.clone(), data)]);
    let o = run_doc(&built_doc);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let k = built_cache::kept();
    assert_eq!(k.len(), 1);
    assert_eq!(k[0].source, Source::Saved, "second regen: {:?}", k[0].rejected);
    assert_eq!(k[0].key, key);
    assert_eq!(o.body.as_ref().unwrap(), &b1);
    // the report came with it
    assert_eq!(o.warning("meshToSolidReport").path(&["report", "closing", "filledHoles"]).and_then(|v| v.as_f64()), Some(1.0));
    // the saved result never needs the mesh: a doc whose blob would not decode but that has the entry cannot exist (the key
    // names the blob), so check that the lookup is by key: another feature id does not find it
    let other = doc_text(&text_of(&m), &F_CLOSE.replace("F1", "F2"), &[(key, ck_text::base64_encode(&built_cache::kept()[0].bytes))]);
    run_doc(&other);
    assert_eq!(built_cache::kept()[0].source, Source::Built);
}

#[test]
fn another_blob_or_other_options_rebuild() {
    let (_, key, data) = built(&cube(10.0), F);
    // another mesh under the same feature: the old entry is not its key
    let bigger = doc_text(&text_of(&cube(20.0)), F, &[(key.clone(), data.clone())]);
    let o = run_doc(&bigger);
    assert!(o.errors.is_empty());
    assert_eq!(built_cache::kept()[0].source, Source::Built);
    assert!((vol(o.body.as_ref().unwrap()) - 8000.0).abs() < 1e-6);
    assert_ne!(built_cache::kept()[0].key, key);
    // the same mesh with other options
    let f2 = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"analytic":false}}"#;
    let same = doc_text(&text_of(&cube(10.0)), f2, &[(key.clone(), data)]);
    run_doc(&same);
    assert_eq!(built_cache::kept()[0].source, Source::Built);
    assert_ne!(built_cache::kept()[0].key, key);
}

#[test]
fn save_and_load_through_json() {
    let m = open_cube();
    let part = Part::parse(&doc(&m, F_CLOSE)).unwrap();
    let a = run_part(&part);
    let text = ck_json::to_string(&part.to_json());
    let again = Part::parse(&text).unwrap();
    let b = run_part(&again);
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_eq!(a.body, b.body);
    assert!(text.contains(r#""type":"meshToSolid""#) && text.contains(r#""blobs""#) && text.contains(r#""close":true"#), "{text}");
    assert_eq!(ck_json::to_string(a.warning("meshToSolidReport")), ck_json::to_string(b.warning("meshToSolidReport")));
}

#[test]
fn both_blob_layouts_and_the_apps_packed_mesh_build() {
    let hex = |s: &str| -> Vec<u8> { (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect() };
    for (name, bytes) in [("packed", hex("434b4d3200000000080000000c0000002d431cebe2361a3f2e0000002400000000000000000000c09a0c0000bf9a0cc09a0c00c09a0c0000bf9a0cbf9a0cc09a0cc09a0c0000bf9a0cc09a0c00c09a0c0000000100080206000001080204100e061006080c04020c020a10080410040c0e0a020e0206")), ("packed, deflated", hex("6589310a803010042747084738426ab1b4d5c2c2d622a5f8c17c46df62eb2744441bb79919b62ceb08a080017d69cf636ae601e8f8b65783addacdd79ffc3d38540238159f53c841cd8b49cceab3b7142549b800")), ("raw", ck_mesh::blob::encode_raw(&cube(10.0).0, &cube(10.0).1))] {
        let o = run_doc(&doc_text(&ck_text::base64_encode(&bytes), F, &[]));
        assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
        assert!((vol(o.body.as_ref().unwrap()) - 1000.0).abs() < 1e-6, "{name}");
    }
}

#[test]
fn preview_lists_the_fill_and_adds_nothing() {
    let f = r#"{"type":"meshToSolid","id":"F1","blob":"{key}","preview":true,"opts":{"close":true}}"#;
    let o = run_doc(&doc(&open_cube(), f));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(o.body.is_none(), "a preview builds nothing");
    assert!(o.warnings.iter().all(|w| w.1 != "meshToSolidReport"));
    let r = o.warning("meshToSolidPreview").get("report").unwrap();
    assert_eq!(r.get_bool("dryRun"), Some(true));
    let fill = r.path(&["closing", "holes"]).and_then(|h| h.at(0)).and_then(|h| h.get("fill")).expect("fill");
    assert_eq!(fill.get_arr("indices").unwrap().len(), 6);
    assert_eq!(fill.get_arr("positions").unwrap().len(), 12);
    // an open mesh without close: the refusal, as for the build
    let f = r#"{"type":"meshToSolid","id":"F1","blob":"{key}","preview":true}"#;
    assert!(run_doc(&doc(&open_cube(), f)).errors.iter().any(|e| e.1 == "meshNotWatertight"));
}

const BOX: &str = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":20,"y":-20},{"id":"p2","x":80,"y":-20},{"id":"p3","x":80,"y":20},{"id":"p4","x":20,"y":20}],"entities":[{"id":"l1","type":"line","p1":"p1","p2":"p2"},{"id":"l2","type":"line","p1":"p2","p2":"p3"},{"id":"l3","type":"line","p1":"p3","p2":"p4"},{"id":"l4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}"#;

#[test]
fn replace_and_adding_beside_the_bodies_so_far() {
    let m = cube(10.0);
    let n = |b: &Body| b.lump_ids().count();
    let beside = run_doc(&doc(&m, &format!(r#"{BOX},{{"type":"meshToSolid","id":"F1","blob":"{{key}}","body":"Scan"}}"#)));
    assert!(beside.errors.is_empty(), "{:?}", beside.errors);
    let b = beside.body.unwrap();
    assert_eq!(n(&b), 2);
    assert!((vol(&b) - (60.0 * 40.0 * 10.0 + 1000.0)).abs() < 1e-6);
    assert!(b.lump_ids().any(|l| b.lump(l).name.as_deref() == Some("Scan")));
    let rep = run_doc(&doc(&m, &format!(r#"{BOX},{{"type":"meshToSolid","id":"F1","blob":"{{key}}","replace":true}}"#)));
    let b = rep.body.unwrap();
    assert_eq!(n(&b), 1);
    assert!((vol(&b) - 1000.0).abs() < 1e-6);
}

#[test]
fn a_feature_that_is_not_well_formed_is_meshToSolidInvalid() {
    let file = text_of(&cube(10.0));
    let code = |features: &str| -> String {
        let o = run_doc(&doc_text(&file, features, &[]));
        assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
        assert_eq!(o.errors[0].1, "meshToSolidInvalid", "{:?}", o.errors);
        ck_json::to_string(&o.errors[0].2)
    };
    assert!(code(r#"{"type":"meshToSolid","id":"F1"}"#).contains("no data"));
    assert!(code(r#"{"type":"meshToSolid","id":"F1","blob":"{key}","source":"scan.stl"}"#).contains("source"));
    assert!(code(r#"{"type":"meshToSolid","id":"F1","blob":"00"}"#).contains("no blob"));
    assert!(code(r#"{"type":"meshToSolid","id":"F1","blob":"{key}","replace":1}"#).contains("replace"));
    assert!(code(r#"{"type":"meshToSolid","id":"F1","blob":"{key}","opts":3}"#).contains("opts"));
    // a blob whose key is not the hash of its text, and one that is not a mesh
    let d = format!(r#"{{"format":"ck-part","version":1,"blobs":{{"aa":"{file}"}},"features":[{{"type":"meshToSolid","id":"F1","blob":"aa"}}]}}"#);
    assert!(run_doc(&d).errors[0].2.get_str("detail").unwrap().contains("does not match"));
    let junk = ck_text::base64_encode(&[7u8; 64]);
    let o = run_doc(&doc_text(&junk, F, &[]));
    assert_eq!(o.errors[0].1, "meshBadInput", "{:?}", o.errors);
}

#[test]
fn source_is_informational_only() {
    let m = open_cube();
    let with = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"close":true},"source":{"name":"bracket.stl","format":"stl"}}"#;
    let (_, key, _) = built(&m, F_CLOSE);
    let o = run_doc(&doc(&m, with));
    assert!(o.errors.is_empty() && o.warnings.iter().all(|w| w.1 != "unknownField"), "{:?} {:?}", o.errors, o.warnings);
    // not part of the cache key
    assert_eq!(built_cache::kept()[0].key, key);
}

#[test]
fn listed_as_a_feature_type() {
    assert!(regen::KNOWN_TYPES.contains(&"meshToSolid"));
    let want = regen::EMPTY_FEATURE_ERRORS.iter().find(|(t, _)| *t == "meshToSolid").expect("listed").1;
    let o = run_doc(r#"{"format":"ck-part","version":1,"features":[{"type":"meshToSolid","id":"F1"}]}"#);
    assert_eq!(o.errors[0].1, want);
    // a misspelt field is the usual unknownField warning
    let o = run_doc(&doc(&cube(10.0), r#"{"type":"meshToSolid","id":"F1","blob":"{key}","replce":true}"#));
    assert!(o.warnings.iter().any(|w| w.1 == "unknownField"));
}

/// A closed, bumpy, noisy UV blob (a scan-like free-form mesh): `nlat` x `nlon` quads.
fn freeform(nlat: usize, nlon: usize) -> Mesh {
    let mut s = 987654321u64;
    let mut rnd = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0
    };
    let r = |th: f64, ph: f64| 25.0 * (1.0 + 0.08 * (3.0 * th).sin() * (2.0 * ph).cos() + 0.05 * (7.0 * th + ph).sin());
    let pi = std::f64::consts::PI;
    let mut p = vec![0.0f32, 0.0, r(0.0, 0.0) as f32];
    for i in 1..nlat {
        let th = pi * i as f64 / nlat as f64;
        for j in 0..nlon {
            let ph = 2.0 * pi * j as f64 / nlon as f64;
            let rr = r(th, ph) + 0.05 * rnd();
            p.extend([(rr * th.sin() * ph.cos()) as f32, (rr * th.sin() * ph.sin()) as f32, (rr * th.cos()) as f32]);
        }
    }
    p.extend([0.0, 0.0, -r(pi, 0.0) as f32]);
    let ring = |i: usize, j: usize| (1 + (i - 1) * nlon + j % nlon) as u32;
    let south = (p.len() / 3 - 1) as u32;
    let mut ix = vec![];
    for j in 0..nlon {
        ix.extend([0, ring(1, j), ring(1, j + 1)]);
        ix.extend([south, ring(nlat - 1, j + 1), ring(nlat - 1, j)]);
    }
    for i in 1..nlat - 1 {
        for j in 0..nlon {
            let (a, b, c, d) = (ring(i, j), ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j));
            ix.extend([a, d, c, a, c, b]);
        }
    }
    (p, ix)
}

#[test]
fn a_budgeted_free_form_scan_finishes_quickly_and_stays_valid() {
    let m = freeform(40, 80);
    let t = std::time::Instant::now();
    let f = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"maxSeconds":0.002}}"#;
    let o = run_doc(&doc(&m, f));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.clone().expect("body");
    // the converter's own tolerance for a noisy mesh (1e-5 of the diagonal, about 6e-4 here), not the exact-model one
    let iss = ck_check::validate(&b, 1e-3);
    assert!(iss.is_empty(), "{} issues, first {:?}; faces {}", iss.len(), iss.first(), b.num_faces());
    let a = o.warnings.iter().find(|w| w.1 == "meshToSolidReport").unwrap().2.get("report").unwrap().get("analytic").expect("analytic").clone();
    assert_eq!(a.get_bool("budgetExhausted"), Some(true), "{a:?}");
    assert!(a.get_f64("fittedFraction").unwrap() < 0.5);
    let detail = o.warning("meshToSolidReport").get_str("detail").unwrap().to_string();
    assert!(detail.contains("the fit stopped at its work limit (opts.maxSeconds)") && detail.contains("% of the surface is fitted, the rest stays faceted"), "{detail}");
    assert!(t.elapsed().as_secs_f64() < 60.0, "{:?}", t.elapsed());
}

/// `opts.decimate` through the feature: accepted, part of the cache key, reported in `meshToSolidReport` and in the preview.
#[test]
fn decimate_option_reduces_reports_and_keys_the_cache() {
    let m = freeform(40, 80);
    let plain = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"analytic":false,"deviation":false}}"#;
    let dec = r#"{"type":"meshToSolid","id":"F1","name":"Scan","blob":"{key}","opts":{"analytic":false,"deviation":false,"decimate":{"tol":0.2,"targetTriangles":1500}}}"#;
    let (b0, key0, data0) = built(&m, plain);
    let o = run_doc(&doc(&m, dec));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().expect("body");
    assert!(b.num_faces() <= 1500 && b.num_faces() < b0.num_faces() / 3, "{} faces against {}", b.num_faces(), b0.num_faces());
    let w = o.warning("meshToSolidReport");
    let d = w.path(&["report", "decimation"]).expect("decimation in the report");
    assert!(d.get_f64("triangles").unwrap() <= 1500.0);
    assert!(d.path(&["deviation", "max"]).and_then(|v| v.as_f64()).unwrap() <= 0.2);
    let detail = w.get_str("detail").unwrap();
    assert!(detail.contains("decimated from"), "{detail}");
    // another option set is another cache entry: the saved result of the plain feature is not taken for it
    let key1 = built_cache::kept()[0].key.clone();
    assert_ne!(key0, key1);
    let again = doc_text(&text_of(&m), dec, &[(key0, data0)]);
    run_doc(&again);
    assert_eq!(built_cache::kept()[0].source, Source::Built);
    // the dry run decimates too and says so
    let pv = r#"{"type":"meshToSolid","id":"F1","blob":"{key}","preview":true,"opts":{"decimate":{"tol":0.2,"targetTriangles":1500}}}"#;
    let o = run_doc(&doc(&m, pv));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(o.body.is_none());
    assert!(o.warning("meshToSolidPreview").path(&["report", "decimation", "triangles"]).and_then(|v| v.as_f64()).unwrap() <= 1500.0);
    // a bad tol is the conversion's refusal
    let bad = r#"{"type":"meshToSolid","id":"F1","blob":"{key}","opts":{"decimate":{}}}"#;
    assert!(run_doc(&doc(&m, bad)).errors.iter().any(|e| e.1 == "meshBadInput"));
}
