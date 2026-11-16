//! Shared harness of the sketch-free solid tests (torus, pipe, coil, boundingSolid, dome).
#![allow(dead_code)]
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

pub struct Exec(pub Brep);
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

pub struct Res {
    pub errors: Vec<String>,
    pub codes: Vec<String>,
    pub vol: f64,
    pub faces: Vec<(String, &'static str)>,
    pub lumps: usize,
    pub valid: Vec<String>,
    pub bbox: Option<([f64; 3], [f64; 3])>,
}

pub fn run(feats: &str) -> Res {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feats}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    let codes = r.errors.iter().map(|e| e.error.code.clone()).collect();
    let Some(b) = ex.0.body.as_ref() else { return Res { errors, codes, vol: 0.0, faces: vec![], lumps: 0, valid: vec![], bbox: None } };
    let faces = b.face_ids().map(|f| (b.face_id_str(f), b.face(f).surface.kind())).collect();
    let valid = ck_check::validate(b, 1e-3).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    let (lo, hi) = ck_model::features::body_box(b);
    Res { errors, codes, vol: ck_topo::mass::mass_props(b).volume, faces, lumps: b.lump_ids().count(), valid, bbox: Some(([lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z])) }
}

pub fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

pub const BLOCK: &str = r#"{"type":"box","id":"F1","op":"add","center":[0,0,0],"size":[40,40,40]}"#;
