//! A4: the tessellator must refuse a blend result whose face boundary collapses, not panic.
//!
//! The regression table records the daisy N=6 top-rim r0.5 case as an expected failure (volume short of the reference).
//! Meshing that body under wasm hit an index-out-of-bounds panic in `ck_tess::face` when an open boundary chain had
//! every point dropped as a merged-circle junction: `n - 1` wrapped and `chain[0]` was read. The fix skips empty chains
//! so the face returns the ordinary "fewer than 3 boundary points" refusal. This test meshes the case and asserts the
//! mesh reports failures as strings instead of unwinding.
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

const CASES: &str = include_str!("fixtures/blend_cases.json");

fn regen_doc(features: &[Json]) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(features.to_vec())));
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r
        .errors
        .iter()
        .map(|e| match e.error.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str()) {
            Some(d) => format!("{} ({d})", e.error.code),
            None => e.error.code.clone(),
        })
        .collect();
    (ex.0.body.clone(), errs)
}

#[test]
fn daisy_n6_top_rim_half_meshes_as_a_refusal() {
    let cases = ck_json::parse(CASES).unwrap();
    let bases = cases.get("bases").unwrap();
    let case = cases
        .get_arr("cases")
        .unwrap()
        .into_iter()
        .find(|c| c.get_str("id") == Some("daisy-hub-n6-T-r0.5"))
        .expect("daisy-hub-n6-T-r0.5 is in the blend table");
    let base = bases.get(case.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
    let mut feats = base;
    if let Some(pre) = case.get("pre") {
        feats.push(pre.clone());
    }
    feats.push(case.get("fillet").unwrap().clone());
    let (body, errs) = regen_doc(&feats);
    let body = body.expect("the case builds a body");
    assert!(errs.is_empty(), "regen errors: {errs:?}");
    // The panic this guards against was in the tessellator, not in regen: meshing must return, never unwind.
    let mesh = ck_tess::mesh::tessellate(&body).mesh;
    eprintln!(
        "daisy-hub-n6-T-r0.5: {} triangles, {} failed faces",
        mesh.tri_face.len(),
        mesh.failed.len()
    );
    for (face, why) in mesh.failed.iter().zip(mesh.failed_why.iter()) {
        eprintln!("  face {face} refused: {why}");
    }
}
