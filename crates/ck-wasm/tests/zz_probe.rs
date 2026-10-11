//! Throwaway native probe: regen a brep case natively, tessellate, compare per face against the original's mesh
//! (gold dumped by the scratchpad node script). usage: PROBE_CASE=name (or all) cargo test -p ck-wasm --test zz_probe -- --nocapture
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

const GDIR: &str = "/private/tmp/claude-501/-Volumes-second-disk-tmp-purr-rebuild/296fdc97-740a-4f73-841f-21dbda16b0ed/scratchpad/gold/";

struct Nop;
impl Tracer for Nop {
    fn begin(&mut self, _l: &str) -> usize { 0 }
    fn relabel(&mut self, _i: usize, _l: &str) {}
    fn end(&mut self, _i: usize) {}
}
struct Exec { brep: Brep, t: Nop }
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> { Some(&mut self.t) }
    fn face_count(&self) -> Option<usize> { self.brep.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.brep.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = self.brep.exec(s, f);
        self.brep.flush_spans(&mut self.t);
        r
    }
}
fn nums(j: &Json) -> Vec<f64> { j.as_arr().unwrap().iter().map(|x| x.as_f64().unwrap()).collect() }

/// returns per-face status strings
fn probe(name: &str) -> Vec<(i32, String)> {
    let ptxt = std::fs::read_to_string(format!("{GDIR}{name}.part.json")).unwrap();
    let part = Part::from_json(&ck_json::parse(&ptxt).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), t: Nop };
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let body = ex.brep.body.as_ref().expect("body");
    if std::env::var("PROBE_BBOX").is_ok() { let (lo, hi) = body.bbox(); eprintln!("BBOX {:?} {:?} tol {:?}", lo, hi, ck_tess::mesh::body_tol(body)); for e in body.edge_ids() { let ed = body.edge(e); eprintln!("EDGE {} {:?} t {} {}", e.idx(), ed.curve.kind(), ed.t0, ed.t1); } }
    if std::env::var("PROBE_SENSE").is_ok() { for f in body.face_ids() { let fc = body.face(f); eprintln!("SENSE {} f{} sense={} kind={}", name, f.idx(), fc.sense, fc.surface.kind()); } }
    let t = ck_tess::mesh::tessellate(body);
    for &wi in &t.mesh.failed { let fid: Vec<_> = body.face_ids().collect(); let r = ck_tess::face::mesh_face(body, fid[wi], &t.edges.params, &t.edges.virt, t.edges.cuts[fid[wi].idx()], &t.tol.unwrap()); eprintln!("FAILED face {} -> {:?}", wi, r.err()); }
    let gtxt = std::fs::read_to_string(format!("{GDIR}{name}.json")).unwrap();
    let gc = ck_json::parse(&gtxt).unwrap();
    let gpos: Vec<f32> = nums(gc.get("positions").unwrap()).into_iter().map(|x| x as f32).collect();
    let gtf: Vec<i32> = nums(gc.get("triFace").unwrap()).into_iter().map(|x| x as i32).collect();
    let nf = body.num_faces() as i32;
    let tri = |pos: &[f32], i: usize| -> [u32; 9] { let mut a = [0u32; 9]; for k in 0..9 { a[k] = pos[i * 9 + k].to_bits(); } a };
    let mut out = vec![];
    for f in 0..nf.max(gtf.iter().copied().max().unwrap_or(0) + 1) {
        let want: Vec<[u32; 9]> = (0..gtf.len()).filter(|&i| gtf[i] == f).map(|i| tri(&gpos, i)).collect();
        let got: Vec<[u32; 9]> = (0..t.mesh.tri_face.len()).filter(|&i| t.mesh.tri_face[i] == f).map(|i| tri(&t.mesh.positions, i)).collect();
        if std::env::var("PROBE_V").is_ok() && want != got { for i in 0..want.len().min(3) { eprintln!("WANT f{f} {:?}", want[i].map(f32::from_bits)); if i < got.len() { eprintln!("GOT  f{f} {:?}", got[i].map(f32::from_bits)); } } }
        let st = if want == got { "ok".to_string() } else if want.len() != got.len() { format!("count {}/{}", want.len(), got.len()) } else {
            let norm = |v: &[[u32; 9]]| { let mut s: Vec<[u32; 3]> = v.iter().map(|t| { let mut k = [(t[0],t[1],t[2]),(t[3],t[4],t[5]),(t[6],t[7],t[8])]; k.sort(); [k[0].0 ^ k[1].1 ^ k[2].2, 0, 0] }).collect(); s.sort(); s };
            let _ = norm;
            let mut sa: Vec<Vec<(u32, u32, u32)>> = want.iter().map(|t| { let mut k = vec![(t[0],t[1],t[2]),(t[3],t[4],t[5]),(t[6],t[7],t[8])]; k.sort(); k }).collect();
            let mut sb: Vec<Vec<(u32, u32, u32)>> = got.iter().map(|t| { let mut k = vec![(t[0],t[1],t[2]),(t[3],t[4],t[5]),(t[6],t[7],t[8])]; k.sort(); k }).collect();
            let first = (0..want.len()).find(|&i| want[i] != got[i]).unwrap();
            let seq_same = sa.iter().zip(&sb).all(|(a, b)| a == b);
            sa.sort(); sb.sort();
            if sa == sb { if seq_same { "rot".to_string() } else { format!("order@{first}") } } else { format!("set@{first}") }
        };
        out.push((f, st));
    }
    out
}

#[test]
fn probe_cases() {
    let names: Vec<String> = match std::env::var("PROBE_CASE") {
        Ok(s) if s != "all" => s.split(',').map(|x| x.to_string()).collect(),
        _ => {
            let mut v: Vec<String> = std::fs::read_dir(GDIR).unwrap().filter_map(|e| { let n = e.unwrap().file_name().into_string().unwrap(); n.strip_suffix(".part.json").map(|s| s.to_string()) }).collect();
            v.sort();
            v
        }
    };
    let (mut ok, mut tot) = (0, 0);
    for n in names {
        let r = std::panic::catch_unwind(|| probe(&n));
        match r {
            Ok(r) => {
                let good = r.iter().filter(|x| x.1 == "ok").count();
                ok += good;
                tot += r.len();
                let bad: Vec<String> = r.iter().filter(|x| x.1 != "ok").take(8).map(|x| format!("f{}:{}", x.0, x.1)).collect();
                println!("PROBE {n:<20} {good}/{} {}", r.len(), bad.join(" "));
            }
            Err(_) => println!("PROBE {n:<20} PANIC"),
        }
    }
    println!("PROBE TOTAL faces ok {ok}/{tot}");
}
