//! One-off measurement: build the `importBody` of a part file, write the built body with `ck_topo::codec`, read it back
//! and compare, then regenerate the same document with the saved result in its `built` table.
//! `body_blob_time FILE [OUT]` prints build / encode / decode / saved-regen seconds, sizes and the checks.
use ck_json::Json;
use ck_model::features::import_step::blob_key;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let file = std::fs::read(&a[0]).unwrap();
    let text = ck_text::base64_encode(&file);
    let key = blob_key(&text);
    let doc = format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":"{text}"}},"features":[{{"type":"importBody","id":"F1","format":"sldprt","blob":"{key}","replace":true}}]}}"#);
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let build = t.elapsed().as_secs_f64();
    let b = ex.0.body.clone().expect("body");
    println!("errors: {:?}", r.errors.iter().map(|e| e.error.code.to_string()).collect::<Vec<_>>());
    println!("build {build:.2}s faces {} est_bytes {}", b.num_faces(), b.est_bytes());
    let t = Instant::now();
    let bytes = ck_topo::codec::encode(&b);
    println!("encode {:.3}s raw {} bytes", t.elapsed().as_secs_f64(), bytes.len());
    if let Some(o) = a.get(1) {
        std::fs::write(o, &bytes).unwrap();
    }
    let t = Instant::now();
    let b2 = ck_topo::codec::decode(&bytes).unwrap();
    println!("decode {:.3}s equal {}", t.elapsed().as_secs_f64(), b2 == b);
    let (v1, v2) = (ck_topo::mass::mass_props(&b).volume, ck_topo::mass::mass_props(&b2).volume);
    println!("volume {v1} {v2} rel {:e}", ((v1 - v2) / v1).abs());
    println!("validate issues {}", ck_check::validate(&b2, 1e-6).len());
    let e = ck_model::features::built_cache::kept().pop().expect("kept entry");
    println!("entry source {:?} bytes {} key {}", e.source, e.bytes.len(), e.key);
    println!("surface bodies {} faces {:?}", ex.0.surfs.len(), ex.0.surfs.iter().map(|(_, s)| s.num_faces()).collect::<Vec<_>>());
    if let Some(o) = a.get(2) {
        std::fs::write(o, &*e.bytes).unwrap();
    }
    let doc2 = doc.replacen(r#""version":1,"#, &format!(r#""version":1,"built":{{"{}":"{}"}},"#, e.key, ck_text::base64_encode(&e.bytes)), 1);
    let part2 = Part::parse(&doc2).unwrap();
    let mut ex2 = Exec(Brep::new());
    let t = Instant::now();
    let r2 = regen::regen(&part2, &RegenOpts::default(), &mut ex2);
    println!("saved regen {:.3}s errors {} equal-to-built {}", t.elapsed().as_secs_f64(), r2.errors.len(), ex2.0.body.as_ref() == Some(&b));
    println!("source now {:?}", ck_model::features::built_cache::kept().pop().map(|e| e.source));
    let _ = Json::Null;
}
