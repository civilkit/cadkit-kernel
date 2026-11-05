//! Pattern and hole read-back over the files of the local corpus that used to stop on an off-axis linear pattern or a
//! counterbored hole wizard (formerly: blend regeneration over the files that used to stop on a fillet or chamfer step (not in the
//! repository: `SW_CORPUS` names the corpus directory, default `corpus-local` at the workspace root; without it, or
//! without a file, the test passes and does nothing). The number of read-back steps the editable prefix keeps must
//! not fall below what these two files reached, and the importBody remainder must still land on the file's solid.

use ck_swedit::{rebuild_history, Status, Tagged};
use std::path::{Path, PathBuf};

fn find(dir: &Path, stem: &str) -> Option<PathBuf> {
    let rd = std::fs::read_dir(dir).ok()?;
    let mut es: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            if p.file_name().map_or(false, |n| n == stem) {
                // the folder of the sample holds the one part file
                if let Some(f) = std::fs::read_dir(&p).ok()?.flatten().map(|e| e.path()).find(|f| f.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt"))) {
                    return Some(f);
                }
            }
            if let Some(f) = find(&p, stem) {
                return Some(f);
            }
        }
    }
    None
}

/// (complete, rebuilt steps kept, composed body equals the plain import) of one file.
fn run(path: &Path) -> (bool, usize, bool) {
    let dbg = std::env::var("PH_DEBUG").is_ok();
    let bytes = std::fs::read(path).unwrap();
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let t = Tagged::from_import(&imp);
    let out = rebuild_history(&t);
    if dbg {
        for s in &out.steps {
            eprintln!("{} [{}] {:?} {}", s.name, s.kind, s.status, s.note);
        }
        if let Some(c) = out.checks.iter().find_map(|c| c.failure()) {
            eprintln!("check failed on step {}: {c}", out.checks.len());
        }
        if let Some(r) = &out.remainder {
            eprintln!("remainder from {} ({}): {} ({} steps)", r.from.1, r.from.2, r.reason, r.steps.len());
        }
        eprintln!("exact={} close={} prefix={} verify={:?} fallback={:?}", out.exact(), out.close(), out.prefix, out.verify.as_ref().map(|v| v.rel), out.fallback);
    }
    let rebuilt = |n: usize| out.steps[..n].iter().filter(|s| s.status == Status::Rebuilt).count();
    if out.exact() || out.close() {
        return (true, rebuilt(out.steps.len()), true);
    }
    let kept = rebuilt(out.prefix.min(out.steps.len()));
    let mut same = out.prefix == 0;
    if out.prefix > 0 {
        let plain = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "import", None).solid;
        if let Ok((Some(b), errs)) = ck_swedit::prefix::regen_part(&out.part_json(Some(&bytes))) {
            let (mp, mb) = (ck_topo::mass::mass_props(&plain), ck_topo::mass::mass_props(&b));
            let rel = ((mp.volume - mb.volume).abs() / mp.volume.abs().max(1e-12)).max((mp.area - mb.area).abs() / mp.area.abs().max(1e-12));
            same = errs.is_empty() && plain.num_faces() == b.num_faces() && rel < 1e-12;
        }
    }
    (false, kept, same)
}

#[test]
fn pattern_and_hole_files_keep_their_steps() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    // (sample, must be complete, rebuilt steps kept at least)
    let cases: [(&str, bool, usize); 2] = [
        // an off-axis-looking linear pattern whose copies carry holes of a hole feature that comes later in the tree
        ("nist_ftc_06", false, 14),
        // counterbored holes whose far end / walls later features cut into
        ("nist_ftc_10", false, 17),
    ];
    for (stem, complete, min_kept) in cases {
        let Some(path) = find(&dir, stem) else {
            eprintln!("{stem}: not in the corpus at {}", dir.display());
            continue;
        };
        let (done, kept, same) = run(&path);
        eprintln!("{stem}: complete {done}, kept {kept}");
        assert!(same, "{stem}: the composed body no longer lands on the file's solid");
        assert!(kept >= min_kept, "{stem}: {kept} rebuilt steps kept, at least {min_kept} expected");
        if complete {
            assert!(done, "{stem}: no longer rebuilds to the end");
        }
    }
}

/// nist_ftc_10's small clearance holes break into a bore along a saddle edge that reaches further than the edge's
/// vertices and mid points: the cut has to run to that extreme (a flat end 0.2 mm short left a ridge on every hole
/// and put the rebuilt area 1.5e-3 off the file's). With the holes right, the rebuilt history is within 5e-4 of the
/// file; what is left is the concave vertex patches of the first fillet.
#[test]
fn ftc_10_holes_reach_the_bore() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let Some(path) = find(&dir, "nist_ftc_10") else {
        eprintln!("nist_ftc_10: not in the corpus at {}", dir.display());
        return;
    };
    let imp = ck_sldprt::import_sldprt(&std::fs::read(path).unwrap()).unwrap();
    let out = rebuild_history(&Tagged::from_import(&imp));
    let v = out.verify.as_ref().expect("the whole history regenerates");
    assert!(v.rel < 5e-4, "rebuilt body is {:.2e} off the file's", v.rel);
    assert_eq!(out.steps.iter().filter(|s| s.status == Status::Rebuilt).count(), 19);
}
