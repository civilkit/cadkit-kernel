//! Shared runner for the robustness tests: regenerate a part and report the error codes.
//! A case must never panic and never run past the time limit (a hang fails the test).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let t = std::time::Instant::now();
        let r = self.brep.exec(s, f);
        if std::env::var_os("ROBUST_TIMING").is_some() && t.elapsed().as_secs_f64() > 0.3 {
            eprintln!("  feature {} took {:.2}s ({})", f.id, t.elapsed().as_secs_f64(), if r.is_ok() { "ok" } else { "err" });
        }
        r
    }
}

/// Validation issues (`code: detail`) and volume of the final body, if any.
pub fn regen_validate(src: &str) -> (Vec<String>, Option<f64>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let _ = regen::regen(&part, &opts, &mut ex);
    match ex.brep.body.as_ref() {
        None => (vec![], None),
        Some(b) => (
            {
                // the validate request also tessellates (a watertight display mesh is part of the check)
                let t = std::time::Instant::now();
                let mesh = ck_tess::mesh::tessellate(b).mesh;
                if std::env::var_os("ROBUST_TIMING").is_some() {
                    eprintln!("  tessellate {:.2}s ({} triangles)", t.elapsed().as_secs_f64(), mesh.tri_face.len());
                }
                Vec::<String>::new()
            }
            .into_iter()
            .chain(
            ck_check::validate(b, 1e-6).iter().map(|i| format!("{}: {}", i.code, i.detail)))
            .collect(),
            Some(ck_topo::mass::mass_props(b).volume),
        ),
    }
}

/// Regenerate a part JSON; returns (feature id, error code, message) per failed feature.
pub fn regen_codes(src: &str) -> Vec<(String, String, String)> {
    regen_codes_opts(src, r#"{"solveSketches":true}"#)
}

/// [`regen_codes`] with explicit regen options (JSON).
pub fn regen_codes_opts(src: &str, opts: &str) -> Vec<(String, String, String)> {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(opts).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone(), e.error.params.iter().map(|(k, v)| format!("{k}={}", ck_json::to_string(v))).collect::<Vec<_>>().join(";"))).collect()
}

/// Same, on a worker thread with a time limit. Panics (test failure) on a panic or a timeout.
pub fn regen_codes_bounded(src: String, secs: u64) -> Vec<(String, String, String)> {
    let (tx, rx) = std::sync::mpsc::channel();
    let h = std::thread::Builder::new().stack_size(64 << 20).spawn(move || {
        let _ = tx.send(regen_codes(&src));
    }).unwrap();
    match rx.recv_timeout(std::time::Duration::from_secs(secs)) {
        Ok(v) => {
            let _ = h.join();
            v
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("hang: no result within {secs}s"),
        Err(_) => {
            let _ = h.join();
            panic!("panic inside regen")
        }
    }
}

/// tools/cases/crash/<id>.json -> part JSON text (the file holds {"features":[...]}).
pub fn crash_case(id: &str) -> String {
    // ROBUST_CASE_DIR overrides the directory (to try a fuzz case before it is committed)
    let dir = std::env::var("ROBUST_CASE_DIR").unwrap_or_else(|_| format!("{}/../../tools/cases/crash", env!("CARGO_MANIFEST_DIR")));
    let p = format!("{dir}/{id}.json");
    let j = ck_json::parse(&std::fs::read_to_string(&p).unwrap()).unwrap();
    let feats = ck_json::to_string(j.get("features").unwrap());
    format!(r#"{{"format":"ck-part","version":1,"features":{feats}}}"#)
}

/// Debug dump of the final body's loops (face, surface kind, coedge edge ids with direction).
#[allow(dead_code)]
pub fn dump(src: &str) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let _ = regen::regen(&part, &opts, &mut ex);
    let b = ex.brep.body.as_ref().unwrap();
    for f in b.face_ids() {
        let fc = b.face(f);
        eprintln!("face {} {:?} sense={}", b.face_id_str(f), std::mem::discriminant(&fc.surface), fc.sense);
        for &l in &fc.loops {
            let cs: Vec<String> = b.lp(l).coedges.iter().map(|&c| { let co = b.coedge(c); let e = b.edge(co.edge); format!("e{}{}({}->{})", co.edge.idx(), if co.rev { "-" } else { "+" }, e.v0.idx(), e.v1.idx()) }).collect();
            eprintln!("   loop {}", cs.join(" "));
        }
    }
}
