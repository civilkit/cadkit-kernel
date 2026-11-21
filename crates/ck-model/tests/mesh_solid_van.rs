//! By hand (`CK_DOC_FILE=<part json>`): a document with a `meshToSolid` feature of a noisy ~24000 triangle mesh, a scaled
//! copy of its body, an armour block and a `combine` cut with the scaled copy as the tool. Prints the regen time of each
//! prefix of the feature list (cold, nothing cached) and of the whole document again (warm), and checks the result is valid.
//! Skipped without the variable.
use ck_json::Json;
use ck_model::features::built_cache;
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

fn run(doc: &Json, n: usize, cold: bool) -> (f64, Option<ck_topo::body::Body>, Vec<String>) {
    let mut d = doc.clone();
    if let Json::Obj(o) = &mut d {
        for (k, v) in o.iter_mut() {
            if k == "features" {
                if let Json::Arr(a) = v {
                    a.truncate(n);
                }
            }
        }
    }
    let part = Part::parse(&ck_json::to_string(&d)).unwrap();
    if cold {
        built_cache::forget_all();
    }
    let t = std::time::Instant::now();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let s = t.elapsed().as_secs_f64();
    let errs = r.errors.iter().map(|e| format!("{}:{} {}", e.feature, e.error.code, ck_json::to_string(&Json::Obj(e.error.params.clone())).chars().take(300).collect::<String>())).collect();
    (s, ex.0.body.clone(), errs)
}

#[test]
fn van_document() {
    let Ok(path) = std::env::var("CK_DOC_FILE") else { return };
    let doc = ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let nf = doc.get_arr("features").unwrap().len();
    let mut prev = 0.0;
    for n in 1..=nf {
        let (s, b, errs) = run(&doc, n, true);
        let (faces, lumps, vol) = b.as_ref().map_or((0, 0, 0.0), |b| (b.num_faces(), b.lump_ids().count(), ck_topo::mass::mass_props(b).volume));
        eprintln!("VAN prefix {n}: cold {s:.2} s (+{:.2}); faces {faces} lumps {lumps} volume {vol:.3} errors {errs:?}", s - prev);
        prev = s;
    }
    let (_, _, _) = run(&doc, nf, true);
    let (s, b, errs) = run(&doc, nf, false);
    eprintln!("VAN whole document again, built cache warm: {s:.2} s");
    assert!(errs.is_empty(), "{errs:?}");
    let b = b.unwrap();
    let iss = ck_check::validate(&b, 1e-3);
    eprintln!("VAN final: faces {} lumps {} issues {} {:?}", b.num_faces(), b.lump_ids().count(), iss.len(), iss.first());
    for l in b.lump_ids() {
        let lump = b.lump(l);
        eprintln!("VAN lump {:?} name {:?} shells {}", l.idx(), lump.name, lump.shells.len());
    }
    eprintln!("VAN volume total {:.3}", ck_topo::mass::mass_props(&b).volume);
}

/// By hand (`CK_DOC_FILE`): where the time of the cavity cut goes, the stages around and inside `boolean`.
#[test]
fn van_cut_profile() {
    let Ok(path) = std::env::var("CK_DOC_FILE") else { return };
    let doc = ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let text = match doc.get("blobs") {
        Some(Json::Obj(o)) => o[0].1.as_str().unwrap().to_string(),
        _ => panic!("no blobs"),
    };
    ck_core::budget::set_clock(|| {
        thread_local! { static T0: std::time::Instant = std::time::Instant::now(); }
        T0.with(|t| t.elapsed().as_secs_f64() * 1e3)
    });
    let m = ck_mesh::blob::decode_base64(&text).unwrap();
    let o = ck_mesh::Opts { close: Some(Default::default()), ..Default::default() };
    let t = std::time::Instant::now();
    let van = ck_mesh::mesh_to_solid(&m.positions, Some(&m.indices), &o).unwrap().body;
    eprintln!("PROF convert {:.2} s", t.elapsed().as_secs_f64());
    let c = ck_topo::mass::mass_props(&van).com;
    // the scaled faceted copy: scale the vertices about the centroid and convert again
    let mut pts = m.positions.clone();
    for p in pts.chunks_mut(3) {
        p[0] = c.x + (p[0] - c.x) * 1.0075;
        p[1] = c.y + (p[1] - c.y) * 1.0075;
        p[2] = c.z + (p[2] - c.z) * 1.0075;
    }
    let tool2 = ck_mesh::mesh_to_solid(&pts, Some(&m.indices), &o).unwrap().body;
    let block_pos: Vec<f64> = vec![-35., -15., -40., 35., -15., -40., 35., 100., -40., -35., 100., -40., -35., -15., 40., 35., -15., 40., 35., 100., 40., -35., 100., 40.];
    let quads = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
    let mut ix = vec![];
    for q in quads {
        ix.extend([q[0], q[1], q[2], q[0], q[2], q[3]]);
    }
    let block = ck_mesh::mesh_to_solid(&block_pos, Some(&ix), &ck_mesh::Opts::default()).unwrap().body;
    let lap = |what: &str, t: std::time::Instant| eprintln!("PROF {what}: {:.2} s", t.elapsed().as_secs_f64());
    let t = std::time::Instant::now();
    let _ = ck_topo::check::check(&block, 1e-6).len();
    let _ = ck_topo::check::check(&tool2, 1e-6).len();
    lap("topo::check of both operands", t);
    let t = std::time::Instant::now();
    let r = ck_ops::boolean::apply::boolean(&block, &tool2, ck_ops::boolean::Op::Cut, 1e-6).unwrap();
    lap("boolean (the whole of ck_ops boolean, incl. sanity)", t);
    let t = std::time::Instant::now();
    let n = ck_topo::check::check(&r.body, 1e-6).len();
    lap(&format!("topo::check of the result ({n} issues)"), t);
    let t = std::time::Instant::now();
    let v = ck_topo::mass::mass_props(&r.body).volume;
    lap(&format!("mass_props of the result (volume {v:.1})"), t);
    let t = std::time::Instant::now();
    let _ = ck_check::validate(&r.body, 1e-3);
    lap("ck_check::validate of the result", t);
}
