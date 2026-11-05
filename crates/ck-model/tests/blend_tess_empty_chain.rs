//! The tessellator on a blend result whose face boundary empties an open chain (the wasm trap of the fillet table,
//! `crates/ck-tess/src/face.rs`, fixed in 9b26d90; docs/wasm-parity.md).
//!
//! The fillet table case daisy-ring-n6-T-r0.5 built at radius 0.505 with the blend validity gate off
//! (`CK_BLEND_GATE=0`) gives a body whose blend faces F90.B3~1, F90.B3~2 and F90.B19~2 each use two edges twice in
//! their own loop (slits across the band just below the top rim) with sliver edges between them. The tessellator cuts a loop at an edge the
//! face uses twice (a seam); the piece between the two slits is one short merged-circle edge sampled at its two ends,
//! both junctions, and the junction drop empties it. Before 9b26d90 the segment walk then read `chain[usize::MAX]` (an
//! index-out-of-bounds panic, a trap in the wasm kernel). Now the first attempt refuses the face and the escalation
//! (trim ends welded at 1e-6) meshes it: no face may fail, and every face's mesh must cover its exact area.
//!
//! The body is the one the x86-64 Linux build produces (the CI box); another platform's libm can build a slightly
//! different body that does not take the path (the test then still passes). `CK_TESS_DBG=1` prints
//! `an open chain was emptied by the junction drop` when it is taken.
use ck_core::linalg::Vec3;
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

#[test]
fn blend_face_with_an_emptied_chain_meshes() {
    // the raw blend result, as before the gate existed (the only test in this binary: the variable is process-wide)
    std::env::set_var("CK_BLEND_GATE", "0");
    let cases = ck_json::parse(CASES).unwrap();
    let case = cases.get_arr("cases").unwrap().iter().find(|c| c.get_str("id") == Some("daisy-ring-n6-T-r0.5")).expect("the case is in the table");
    let mut feats = cases.get("bases").unwrap().get(case.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
    let mut fillet = case.get("fillet").unwrap().clone();
    fillet.set("radius", 0.505);
    feats.push(fillet);
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(feats)));
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&Part::parse(&doc).unwrap(), &RegenOpts::default(), &mut ex);
    let Some(b) = ex.0.body.filter(|_| r.errors.is_empty()) else {
        eprintln!("the fillet did not build on this platform ({:?}): nothing to mesh", r.errors.iter().map(|e| &e.error.code).collect::<Vec<_>>());
        return;
    };
    let t = ck_tess::mesh::tessellate(&b);
    let m = &t.mesh;
    let ids: Vec<_> = b.face_ids().collect();
    let failed: Vec<String> = m.failed.iter().zip(&m.failed_why).map(|(&i, w)| format!("{}: {w}", b.face_id_str(ids[i]))).collect();
    assert!(failed.is_empty(), "faces not meshed: {failed:?}");
    let mut area = vec![0.0f64; ids.len()];
    let p = |k: usize| Vec3::new(m.positions[k] as f64, m.positions[k + 1] as f64, m.positions[k + 2] as f64);
    for (k, &fi) in m.tri_face.iter().enumerate() {
        let (a, bb, c) = (p(9 * k), p(9 * k + 3), p(9 * k + 6));
        area[fi as usize] += 0.5 * (bb - a).cross(c - a).len();
    }
    for name in ["F90.B3~1", "F90.B3~2", "F90.B19~2"] {
        let Some(i) = ids.iter().position(|&f| b.face_id_str(f) == name) else { continue };
        let exact = ck_topo::mass::face_integral::<1>(&b, ids[i], 1, &|_, n| [n.len()])[0].abs();
        assert!((area[i] / exact).ln().abs() < 0.05, "{name}: mesh area {} vs exact {exact}", area[i]);
    }
}
