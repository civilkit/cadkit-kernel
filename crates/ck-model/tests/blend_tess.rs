//! Display tessellation of every fillet regression table result (`fixtures/blend_cases.json`): the tessellator must
//! return for every body a blend produces, never panic (a panic is a trap in the wasm kernel), and the faces it
//! refuses are listed with the reason. Each body is meshed at the display tolerance and at every `exportMesh` quality.
//! See docs/wasm-parity.md.
//!
//! Off unless `BLEND_TESS=1` (the table already regenerates every case; this adds the meshing). Run it with the blend
//! validity gate off (`CK_BLEND_GATE=0`) to mesh the raw results the gate would refuse: those are the bodies a face
//! failure or a trap came from before the gate existed. `BLEND_CASE` filters as in blend_table.rs (substring, `=id`).
//! Prints one `TESS <id> ...` line per panic / failed face and a summary; fails only on a panic.
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

/// (body when the fillet feature reported no error, error codes)
fn regen_doc(features: &[Json]) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(features.to_vec())));
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| e.error.code.clone()).collect();
    (if errs.is_empty() { ex.0.body.clone() } else { None }, errs)
}

fn selected(id: &str, filter: &str) -> bool {
    filter.split(',').map(str::trim).filter(|t| !t.is_empty()).any(|t| match t.strip_prefix('=') {
        Some(x) => id == x,
        None => id.contains(t),
    })
}

/// The fillet feature with its radius and every per-edge radius times `k`.
fn scaled(f: &Json, k: f64) -> Json {
    fn scale(j: &mut Json, k: f64) {
        if let Json::Obj(kv) = j {
            for (key, v) in kv.iter_mut() {
                match (key.as_str(), v) {
                    ("radius", Json::Num(r)) => *r *= k,
                    ("edges", Json::Arr(es)) => es.iter_mut().for_each(|e| scale(e, k)),
                    _ => {}
                }
            }
        }
    }
    let mut f = f.clone();
    scale(&mut f, k);
    f
}

/// Faces whose mesh area is off the exact face area by more than 5 % (`|ln ratio| > 0.05`, the tessellator's own
/// bar for a good face): `<face> mesh <a> exact <b>`.
fn poor_faces(b: &ck_topo::body::Body, m: &ck_tess::mesh::Mesh) -> Vec<String> {
    let ids: Vec<_> = b.face_ids().collect();
    let mut area = vec![0.0f64; ids.len()];
    let p = |k: usize| ck_core::linalg::Vec3::new(m.positions[k] as f64, m.positions[k + 1] as f64, m.positions[k + 2] as f64);
    for (t, &fi) in m.tri_face.iter().enumerate() {
        let (a, bb, c) = (p(9 * t), p(9 * t + 3), p(9 * t + 6));
        if let Some(x) = area.get_mut(fi as usize) {
            *x += 0.5 * (bb - a).cross(c - a).len();
        }
    }
    let mut out = vec![];
    for (i, &f) in ids.iter().enumerate() {
        let exact = ck_topo::mass::face_integral::<1>(b, f, 1, &|_, n| [n.len()])[0].abs();
        if exact > 0.0 && (area[i] / exact).ln().abs() > 0.05 {
            out.push(format!("{} mesh {:.6} exact {:.6}", b.face_id_str(f), area[i], exact));
        }
    }
    out
}

#[test]
fn blend_results_tessellate_without_panicking() {
    if std::env::var("BLEND_TESS").ok().as_deref() != Some("1") {
        return;
    }
    let cases = ck_json::parse(CASES).unwrap();
    let bases = cases.get("bases").unwrap();
    let filter = std::env::var("BLEND_CASE").ok().filter(|v| !v.is_empty());
    let picked: Vec<&Json> =
        cases.get_arr("cases").unwrap().iter().filter(|c| filter.as_deref().map_or(true, |f| selected(c.get_str("id").unwrap(), f))).collect();
    let mut scales = vec![1.0];
    scales.extend(std::env::var("BLEND_TESS_SCALES").unwrap_or_default().split(',').filter_map(|x| x.trim().parse::<f64>().ok()));
    let list: Vec<(&Json, f64)> = picked.iter().flat_map(|&c| scales.iter().map(move |&k| (c, k))).collect();
    // (lines, meshed, panicked, failed faces)
    let rows: Vec<std::sync::Mutex<Option<(Vec<String>, bool, bool, usize)>>> = list.iter().map(|_| Default::default()).collect();
    let next = std::sync::atomic::AtomicUsize::new(0);
    std::thread::scope(|sc| {
        for _ in 0..std::thread::available_parallelism().map_or(4, |n| n.get()).min(16) {
            std::thread::Builder::new()
                .stack_size(64 << 20)
                .spawn_scoped(sc, || loop {
                    let i = next.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    let Some(&(case, k)) = list.get(i) else { break };
                    let id = if k == 1.0 { case.get_str("id").unwrap().to_string() } else { format!("{}@x{k}", case.get_str("id").unwrap()) };
                    let mut feats = bases.get(case.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
                    feats.extend(case.get("pre").cloned());
                    feats.push(scaled(case.get("fillet").unwrap(), k));
                    let (body, _) = regen_doc(&feats);
                    let mut lines = vec![];
                    let Some(body) = body else {
                        *rows[i].lock().unwrap() = Some((lines, false, false, 0));
                        continue;
                    };
                    if let Ok(d) = std::env::var("BLEND_TESS_STEP") {
                        let opts = ck_io::step::write::StepOpts { unix_ms: 0.0 };
                        let _ = std::fs::write(std::path::Path::new(&d).join(format!("{id}.step")), ck_io::step::write::write_step(&body, &id, &opts));
                    }
                    // the display mesh, then the export qualities (exportMesh: a coarser or finer edge sampling)
                    let (mut panicked, mut nfail) = (false, 0);
                    for q in [None, Some("coarse"), Some("normal"), Some("fine")] {
                        let tol = ck_tess::mesh::export_tol(&body, q, None).unwrap().0;
                        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match q {
                            None => ck_tess::mesh::tessellate(&body).mesh,
                            Some(_) => ck_tess::mesh::tessellate_with(&body, &tol).mesh,
                        }));
                        let qn = q.unwrap_or("display");
                        match r {
                            Err(p) => {
                                let m = p.downcast_ref::<String>().cloned().or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_default();
                                lines.push(format!("TESS {id} [{qn}] PANIC {m}"));
                                panicked = true;
                            }
                            Ok(m) => {
                                let ids: Vec<_> = body.face_ids().collect();
                                if q.is_none() && std::env::var("BLEND_TESS_AREA").is_ok() {
                                    lines.extend(poor_faces(&body, &m).into_iter().map(|x| format!("TESS {id} [display] POOR {x}")));
                                }
                                for (f, why) in m.failed.iter().zip(&m.failed_why) {
                                    lines.push(format!("TESS {id} [{qn}] FAILED {}: {why}", ids.get(*f).map_or("?".into(), |&x| body.face_id_str(x))));
                                }
                                nfail += m.failed.len();
                            }
                        }
                    }
                    *rows[i].lock().unwrap() = Some((lines, true, panicked, nfail));
                })
                .unwrap();
        }
    });
    let (mut meshed, mut panics, mut failed) = (0, 0, 0);
    for r in rows {
        let (lines, m, p, n) = r.into_inner().unwrap().unwrap();
        for l in lines {
            println!("{l}");
        }
        meshed += m as usize;
        panics += p as usize;
        failed += n;
    }
    let gate = if std::env::var("CK_BLEND_GATE").ok().as_deref() == Some("0") { "off" } else { "on" };
    println!("blend tess: {} cases, {meshed} bodies meshed, {panics} panics, {failed} failed faces (gate {gate})", list.len());
    assert_eq!(panics, 0, "the tessellator panicked (a wasm trap)");
}
