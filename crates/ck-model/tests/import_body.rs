//! `importBody`: the body of a SolidWorks part file held by the part (blob table, base64), read on regeneration, with
//! `replace` putting it in place of the bodies so far. The files are the small permissively licensed fixtures of
//! `ck-sldprt` (tests/fixtures, see SOURCES.txt there).
use ck_json::Json;
use ck_model::features::import_body::sort_sldprt;
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

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ck-sldprt/tests/fixtures").join(name)).unwrap()
}

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
}

fn run_doc(doc: &str) -> Out {
    let part = Part::parse(doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { body: ex.0.body.clone(), errors: r.errors.iter().map(|e| format!("{}:{} {}", e.feature, e.error.code, ck_json::to_string(&Json::Obj(e.error.params.clone())))).collect() }
}

/// A part whose `blobs` table holds `file` as base64, with `features` (`{key}` in them is replaced by the blob's key).
fn doc(file: &[u8], features: &str) -> String {
    let text = ck_text::base64_encode(file);
    let key = blob_key(&text);
    format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":"{text}"}},"features":[{}]}}"#, features.replace("{key}", &key))
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// The body the `importSldprt` request gives for the file.
fn plain(file: &[u8]) -> Body {
    let imp = ck_sldprt::import_sldprt(file).unwrap();
    sort_sldprt(imp.body, &imp.parts, "import", None).solid
}

#[test]
fn reads_the_file_as_the_plain_import_does() {
    for name in ["sw2022_cube_hole_5mm.sldprt", "sw2022_cone.sldprt", "sw2011_cube_10mm_ole2.sldprt"] {
        let file = fixture(name);
        let o = run_doc(&doc(&file, r#"{"type":"importBody","id":"F1","format":"sldprt","blob":"{key}"}"#));
        assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
        let b = o.body.expect("body");
        let p = plain(&file);
        assert_eq!(b.num_faces(), p.num_faces(), "{name}");
        assert!((vol(&b) - vol(&p)).abs() <= 1e-12 * vol(&p).abs().max(1.0), "{name}: {} vs {}", vol(&b), vol(&p));
        assert!(ck_check::validate(&b, 1e-6).is_empty(), "{name}");
        // faces `<id>.I<solid>F<n>`, numbered from 1 in the file's order
        let mut names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        names.sort();
        assert!(names.iter().all(|n| n.starts_with("F1.I1F")), "{name}: {names:?}");
        assert!(names.contains(&"F1.I1F1".to_string()), "{name}: {names:?}");
        names.dedup();
        assert_eq!(names.len(), b.num_faces(), "{name}: names repeat");
    }
}

#[test]
fn replace_puts_the_file_in_place_of_the_bodies_so_far() {
    let file = fixture("sw2022_cube_hole_5mm.sldprt");
    let p = plain(&file);
    let o = run_doc(&doc(&file, &format!(r#"{BOX},{{"type":"importBody","id":"F3","format":"sldprt","blob":"{{key}}","replace":true,"from":{{"id":3,"name":"Hole1","kind":"HoleWizard"}},"reason":"test","steps":[]}}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    assert_eq!(b.lump_ids().count(), 1);
    assert_eq!(b.num_faces(), p.num_faces());
    assert!((vol(&b) - vol(&p)).abs() <= 1e-12 * vol(&p));
    assert!(b.face_ids().all(|f| b.face(f).name.feature == "F3"), "a face of the box survived the replace");
    // without `replace` the file's solid joins beside the box
    let o = run_doc(&doc(&file, &format!(r#"{BOX},{{"type":"importBody","id":"F3","format":"sldprt","blob":"{{key}}"}}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    assert_eq!(b.lump_ids().count(), 2);
    assert!((vol(&b) - vol(&p) - 60.0 * 40.0 * 10.0).abs() < 1e-6 * vol(&b));
}

#[test]
fn features_after_the_remainder_reference_its_faces() {
    // a box cut through the imported cube's top face: the cut names a face of the import
    let file = fixture("sw2022_cube_10mm.sldprt");
    let p = plain(&file);
    let hi = p.bbox().1;
    let top = format!("F1.I1F{}", {
        let imp = ck_sldprt::import_sldprt(&file).unwrap();
        let faces = &imp.parts[0].faces;
        // the face whose vertices all lie at the top of the box
        1 + faces.iter().position(|&f| imp.body.face_edges(f).iter().all(|&e| (imp.body.vertex(imp.body.edge(e).v0).p.z - hi.z).abs() < 1e-9)).unwrap()
    });
    let sk = format!(
        r#"{{"type":"sketch","id":"F2","plane":{{"kind":"face","id":"{top}"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":2}}],"constraints":[]}},{{"type":"extrude","id":"F3","op":"cut","sketch":"F2","end":"blind","depth":3}}"#
    );
    let o = run_doc(&doc(&file, &format!(r#"{{"type":"importBody","id":"F1","format":"sldprt","blob":"{{key}}","replace":true}},{sk}"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    // (the circle sits at the face frame's origin, a corner or the centre of the face: some of it cuts)
    assert!(vol(&b) < vol(&p) - 1.0 && vol(&b) > vol(&p) - std::f64::consts::PI * 4.0 * 3.0 - 1e-6, "{} of {}", vol(&b), vol(&p));
    assert!(b.face_ids().any(|f| b.face(f).name.feature == "F3"));
}

#[test]
fn bad_data_is_an_error_never_a_panic() {
    let file = fixture("sw2022_cube_10mm.sldprt");
    let code = |d: &str| {
        let o = run_doc(d);
        assert!(o.body.is_none(), "{d}");
        assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
        assert!(o.errors[0].starts_with("F1:importBodyInvalid "), "{:?}", o.errors);
        o.errors[0].clone()
    };
    let one = |f: &str| format!(r#"{{"format":"ck-part","version":1,"features":[{f}]}}"#);
    assert!(code(&one(r#"{"type":"importBody","id":"F1"}"#)).contains("format"));
    assert!(code(&one(r#"{"type":"importBody","id":"F1","format":"step","blob":"00"}"#)).contains("not supported"));
    assert!(code(&one(r#"{"type":"importBody","id":"F1","format":"sldprt"}"#)).contains("no data"));
    assert!(code(&one(r#"{"type":"importBody","id":"F1","format":"sldprt","blob":"00"}"#)).contains("no blob"));
    assert!(code(&one(r#"{"type":"importBody","id":"F1","format":"sldprt","replace":1,"blob":"00"}"#)).contains("replace"));
    // the key does not match the text
    let d = doc(&file, r#"{"type":"importBody","id":"F1","format":"sldprt","blob":"{key}"}"#);
    let text = ck_text::base64_encode(&file);
    let mut other = file.clone();
    other[100] ^= 1;
    assert!(code(&d.replacen(&text, &ck_text::base64_encode(&other), 1)).contains("does not match"));
    // not base64, not a part file
    let bad = |t: &str| format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{}":"{t}"}},"features":[{{"type":"importBody","id":"F1","format":"sldprt","blob":"{}"}}]}}"#, blob_key(t), blob_key(t));
    code(&bad("@@@"));
    code(&bad(&ck_text::base64_encode(b"hello, this is not a part file")));
    // a truncated file: an error or a body (the reader may still find the solid), never a panic
    let o = run_doc(&bad(&ck_text::base64_encode(&file[..file.len() / 3])));
    assert!(o.body.is_some() || o.errors.iter().any(|e| e.contains("importBodyInvalid")), "{:?}", o.errors);
}

#[test]
fn registered_with_its_empty_payload_error() {
    assert!(regen::KNOWN_TYPES.contains(&"importBody"));
    let want = regen::EMPTY_FEATURE_ERRORS.iter().find(|(t, _)| *t == "importBody").expect("listed").1;
    let o = run_doc(r#"{"format":"ck-part","version":1,"features":[{"type":"importBody","id":"F1"}]}"#);
    assert!(o.errors[0].starts_with(&format!("F1:{want} ")), "{:?}", o.errors);
}
