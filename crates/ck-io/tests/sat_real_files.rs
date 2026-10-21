//! Real-world SAT files (not in the repo: licences unknown / size). Set
//! `SAT_SAMPLES_DIR`; each test skips when its file is absent.

use ck_io::sat::import_sat;

fn dir() -> String {
    std::env::var("SAT_SAMPLES_DIR").unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/../../../scratch-sat").into())
}

fn run(rel: &str, min_faces: usize) {
    let p = format!("{}/{rel}", dir());
    let Ok(bytes) = std::fs::read(&p) else {
        eprintln!("skip: {p} absent");
        return;
    };
    let text = String::from_utf8_lossy(&bytes);
    let t0 = std::time::Instant::now();
    let imp = import_sat(&text).unwrap_or_else(|e| panic!("{rel}: {e}"));
    let s = &imp.summary;
    let t1 = t0.elapsed();
    let m = ck_topo::mass::lump_moments(&imp.body);
    let t2 = t0.elapsed();
    let issues = ck_check::validate(&imp.body, 1e-6);
    eprintln!("{rel}: import {t1:?}, +mass {:?}, +validate {:?}", t2 - t1, t0.elapsed() - t2);
    eprintln!("{rel}: bodies={} lumps={} faces={} issues={} vols={:?} skipped={:?}", s.bodies, s.lumps, s.faces, issues.len(), m.iter().map(|x| x.volume).collect::<Vec<_>>(), s.skipped);
    assert!(imp.body.num_faces() >= min_faces, "{rel}: faces {}", imp.body.num_faces());
    assert!(issues.is_empty(), "{rel}: {issues:?}");
    assert!(!m.is_empty() && m.iter().all(|x| x.volume > 0.0), "{rel}: volumes");
}

#[test]
fn dice_sat() {
    run("dice.sat", 40);
}

#[test]
fn svalinn_error_blocks() {
    run("svalinn/ErrorBlocks.SAT", 4 * 6);
}
