//! Saved results of `importBody` (`features::built_cache`): a document that carries a good saved result gets the same
//! body without building it; a missing, damaged or foreign one means a rebuild from the file, with the same result.
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

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ck-sldprt/tests/fixtures").join(name)).unwrap()
}

const FEATURE: &str = r#"{"type":"importBody","id":"F1","format":"sldprt","blob":"KEY","replace":true}"#;

/// The part document: the file in `blobs`, one `importBody` (id `id`), and `built` entries `(key, base64)` if given.
fn doc(file: &[u8], id: &str, built: &[(String, String)]) -> String {
    let text = ck_text::base64_encode(file);
    let key = blob_key(&text);
    let f = FEATURE.replace("KEY", &key).replace("F1", id);
    let b = built.iter().map(|(k, v)| format!(r#""{k}":"{v}""#)).collect::<Vec<_>>().join(",");
    format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":"{text}"}},"built":{{{b}}},"features":[{f}]}}"#)
}

fn run(d: &str) -> (Body, Vec<String>) {
    built_cache::forget_all();
    let part = Part::parse(d).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (ex.0.body.clone().expect("body"), r.errors.iter().map(|e| e.error.code.to_string()).collect())
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn names(b: &Body) -> Vec<String> {
    b.face_ids().map(|f| b.face_id_str(f)).collect()
}

/// Build once, and the saved entry the process remembers.
fn built(file: &[u8], id: &str) -> (Body, String, String) {
    let (b, errs) = run(&doc(file, id, &[]));
    assert!(errs.is_empty(), "{errs:?}");
    let k = built_cache::kept();
    assert_eq!(k.len(), 1);
    assert_eq!(k[0].source, Source::Built);
    assert_eq!(k[0].feature, id);
    (b, k[0].key.clone(), ck_text::base64_encode(&k[0].bytes))
}

#[test]
fn a_saved_result_gives_the_same_body_without_building() {
    for name in ["sw2022_cube_hole_5mm.sldprt", "sw2022_cone.sldprt", "sw2011_cube_10mm_ole2.sldprt"] {
        let file = fixture(name);
        let (b1, key, data) = built(&file, "F1");
        let (b2, errs) = run(&doc(&file, "F1", &[(key.clone(), data)]));
        assert!(errs.is_empty(), "{name}: {errs:?}");
        let k = built_cache::kept();
        assert_eq!(k.len(), 1, "{name}");
        assert_eq!(k[0].source, Source::Saved, "{name}");
        assert_eq!(k[0].key, key);
        assert_eq!(b1, b2, "{name}: the saved body differs");
        assert_eq!(names(&b1), names(&b2), "{name}");
        assert!((vol(&b1) - vol(&b2)).abs() <= 1e-9 * vol(&b1).abs(), "{name}");
        assert!(ck_check::validate(&b2, 1e-6).is_empty(), "{name}");
        assert_eq!(b1.num_faces(), b2.num_faces());
    }
}

#[test]
fn damaged_or_foreign_entries_rebuild() {
    let file = fixture("sw2022_cube_hole_5mm.sldprt");
    let (b1, key, data) = built(&file, "F1");
    let bytes = ck_text::base64_decode(&data).unwrap();
    let flip = |i: usize| {
        let mut d = bytes.clone();
        let i = i % d.len();
        d[i] ^= 0x10;
        ck_text::base64_encode(&d)
    };
    let cases: Vec<(&str, String)> = vec![
        ("flipped byte (front)", flip(10)),
        ("flipped byte (middle)", flip(bytes.len() / 2)),
        ("flipped byte (checksum)", flip(bytes.len() - 3)),
        ("truncated", ck_text::base64_encode(&bytes[..bytes.len() / 2])),
        ("empty", String::new()),
        ("not base64", "!!!not base64".into()),
        ("other bytes", ck_text::base64_encode(b"CKBSxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")),
    ];
    for (what, bad) in cases {
        let (b2, errs) = run(&doc(&file, "F1", &[(key.clone(), bad)]));
        assert!(errs.is_empty(), "{what}: {errs:?}");
        let k = built_cache::kept();
        assert_eq!(k[0].source, Source::Built, "{what}: a damaged entry was used");
        assert!(k[0].rejected.is_some(), "{what}: no reason given");
        assert_eq!(b1, b2, "{what}");
    }
    // an entry under another key (other feature id, other kernel, other file) is not looked at
    let (_, other_key, other_data) = built(&file, "F7");
    assert_ne!(other_key, key);
    let (b3, errs) = run(&doc(&file, "F1", &[(other_key, other_data)]));
    assert!(errs.is_empty());
    assert_eq!(built_cache::kept()[0].source, Source::Built);
    assert!(built_cache::kept()[0].rejected.is_none());
    assert_eq!(b1, b3);
}

#[test]
fn the_key_depends_on_what_the_result_depends_on() {
    let k = |kind: &str, blob: &str, feat: &str| built_cache::key(kind, blob, feat);
    assert_eq!(k("importBody", "aa", "F1"), k("importBody", "aa", "F1"));
    assert_ne!(k("importBody", "aa", "F1"), k("importBody", "ab", "F1"));
    assert_ne!(k("importBody", "aa", "F1"), k("importBody", "aa", "F2"));
    assert_ne!(k("importBody", "aa", "F1"), k("importStep", "aa", "F1"));
    assert_eq!(k("importBody", "aa", "F1").len(), 64);
}

#[test]
fn encode_decode_keeps_the_parts() {
    let file = fixture("sw2022_cube_hole_5mm.sldprt");
    let imp = ck_sldprt::import_sldprt(&file).unwrap();
    let s = ck_model::features::import_body::sort_sldprt(imp.body, &imp.parts, "F1", Some("F1"));
    let bytes = built_cache::encode(&s);
    let t = built_cache::decode(&bytes).unwrap();
    assert_eq!(s.solid, t.solid);
    assert_eq!(s.surfaces.len(), t.surfaces.len());
    assert_eq!(s.parts.len(), t.parts.len());
    for (a, b) in s.parts.iter().zip(&t.parts) {
        assert_eq!((&a.label, a.outcome, a.faces, a.faces_total, a.reasons.len()), (&b.label, b.outcome, b.faces, b.faces_total, b.reasons.len()));
    }
}

/// A saved result of one file is never used for another file, even when the document keeps the old entry in `built`
/// (an edited source gets a new blob key, hence a new entry key, and is built from its own file).
#[test]
fn a_changed_source_ignores_the_old_entry() {
    let a = fixture("sw2022_cube_hole_5mm.sldprt");
    let b = fixture("sw2022_cone.sldprt");
    let (body_a, key_a, data_a) = built(&a, "F1");
    let (body_b, _, _) = built(&b, "F1");
    let (got, errs) = run(&doc(&b, "F1", &[(key_a, data_a)]));
    assert!(errs.is_empty(), "{errs:?}");
    let k = built_cache::kept();
    assert_eq!(k[0].source, Source::Built, "the entry of another file was used");
    assert!(k[0].rejected.is_none());
    assert_eq!(got, body_b);
    assert_ne!(got, body_a);
}

/// Pins the digest of the saved bytes of one import. When this fails the import builds a different body (or the codec
/// writes different bytes) from the same file at the same crate version: bump `built_cache::BUILT_REV` (or the codec
/// version), then update the digest here. Without the bump, documents saved by the old build would keep serving the old
/// body, because the crate version does not change between builds.
#[test]
fn saved_bytes_are_deterministic_and_pinned() {
    let file = fixture("sw2022_cube_hole_5mm.sldprt");
    let digest = || {
        let imp = ck_sldprt::import_sldprt(&file).unwrap();
        let s = ck_model::features::import_body::sort_sldprt(imp.body, &imp.parts, "F1", Some("F1"));
        ck_text::sha256_hex(&ck_text::sha256(&[&built_cache::encode(&s)]))
    };
    assert_eq!(digest(), digest(), "two encodes of the same import differ");
    assert_eq!(digest(), "4ae3f691be12797982c36dc61d75b7199b5e4e62731287b8a89facc55339d7c1", "BUILT_REV rule: see the comment on this test");
}
