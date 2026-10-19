//! Benchmark (ignored): per-feature wall time of the fan shroud (tools/cases/perf/fan_shroud.ckpart), plus the
//! display tessellation of the body after each solid feature (what a viewer regen of that prefix also pays).
//! `cargo test --profile fast -p ck-model --test fan_shroud_times -- --ignored --nocapture`
//! (`CK_PART=<file>` times another document; `CK_TESS=0` skips the per-feature tessellation; `CK_FOCUS=F44 CK_REPS=5`
//! repeats one feature from the same input; `CK_DUMP=<file>` writes the final body's check issues, face names and
//! edge face-pairs, to diff the naming of two builds). On macOS each feature also reports the instructions it
//! retired (the machine's load does not change them; replaying a feature with `CK_FOCUS` hits the memo tables
//! of its first run, so the per-feature column of an ordinary run is the one to compare).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

/// Instructions retired by this process so far (macOS `proc_pid_rusage`, RUSAGE_INFO_V4), `None` elsewhere.
#[cfg(target_os = "macos")]
fn instructions() -> Option<u64> {
    extern "C" {
        fn proc_pid_rusage(pid: i32, flavor: i32, buffer: *mut u64) -> i32;
    }
    // rusage_info_v4: a 16-byte uuid, then u64 fields; ri_instructions is the 30th of them
    let mut buf = [0u64; 64];
    let r = unsafe { proc_pid_rusage(std::process::id() as i32, 4, buf.as_mut_ptr()) };
    (r == 0).then(|| buf[2 + 29])
}
#[cfg(not(target_os = "macos"))]
fn instructions() -> Option<u64> {
    None
}

struct Exec {
    brep: Brep,
    tess: bool,
    rows: Vec<(String, String, f64, f64, usize, f64)>,
}
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        // CK_FOCUS=<feature id> CK_REPS=n: run that feature n extra times first from the same input (for a sampling profiler)
        if std::env::var("CK_FOCUS").is_ok_and(|v| v == f.id) {
            let reps = std::env::var("CK_REPS").ok().and_then(|v| v.parse().ok()).unwrap_or(3usize);
            let snap = self.brep.save(None);
            for _ in 0..reps {
                let t = Instant::now();
                let _ = self.brep.exec(s, f);
                eprintln!("  {} rep {:.2}s", f.id, t.elapsed().as_secs_f64());
                self.brep.load(&snap);
            }
        }
        let (t, i0) = (Instant::now(), instructions());
        let r = self.brep.exec(s, f);
        let dt = t.elapsed().as_secs_f64();
        let gi = instructions().zip(i0).map_or(f64::NAN, |(a, b)| (a - b) as f64 * 1e-9);
        let (mut tt, mut nf) = (0.0, 0);
        if self.tess && f.type_name != "sketch" && f.type_name != "plane" {
            if let Some(b) = self.brep.body.as_ref() {
                let t = Instant::now();
                let _ = ck_tess::mesh::tessellate(b);
                tt = t.elapsed().as_secs_f64();
                nf = b.num_faces();
            }
        }
        self.rows.push((f.id.clone(), f.type_name.clone(), dt, tt, nf, gi));
        r
    }
}

#[test]
#[ignore]
fn fan_shroud_feature_times() {
    let path = std::env::var("CK_PART").unwrap_or_else(|_| format!("{}/../../tools/cases/perf/fan_shroud.ckpart", env!("CARGO_MANIFEST_DIR")));
    let part = Part::from_json(&ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), tess: std::env::var("CK_TESS").map_or(true, |v| v != "0"), rows: vec![] };
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let total = t.elapsed().as_secs_f64();
    for e in &r.errors {
        eprintln!("ERROR {e:?}");
    }
    eprintln!("regen total {total:.2}s (feature exec + tessellation)");
    for (id, ty, s, tt, nf, gi) in ex.rows.iter().filter(|r| r.2 + r.3 > 0.05) {
        eprintln!("  {id:<5} {ty:<10} exec {s:6.2}s  {gi:6.2} G instr  tess {tt:6.2}s  faces {nf}");
    }
    eprintln!("exec instructions {:.2} G", ex.rows.iter().map(|r| r.5).sum::<f64>());
    let b = ex.brep.body.as_ref().unwrap();
    let m = ck_topo::mass::mass_props(b);
    let (lo, hi) = b.bbox();
    eprintln!("volume {:.6} faces {} bbox {:?} {:?}", m.volume, b.num_faces(), lo, hi);
    // bit-level fingerprint of the whole final body (a behaviour-preserving speed-up keeps it)
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    format!("{b:?}").hash(&mut h);
    eprintln!("body fingerprint {:016x}", h.finish());
    if let Ok(p) = std::env::var("CK_DUMP") {
        let mut s = String::new();
        s += &format!("issues {:?}\n", ck_topo::check::check(b, 1e-6));
        for f in b.face_ids() {
            s += &format!("face {} vol {:.6}\n", b.face_id_str(f), ck_topo::mass::face_signed_volume(b, f));
        }
        for e in b.edge_ids() {
            let mut n: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
            n.sort();
            let ed = b.edge(e);
            s += &format!("edge {} {} {:?}\n", n.join("|"), ed.curve.kind(), ed.curve.eval(0.5 * (ed.t0 + ed.t1)));
        }
        std::fs::write(p, s).unwrap();
    }
}
