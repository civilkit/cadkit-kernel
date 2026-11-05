//! Blend regeneration over the files of the local corpus that used to stop on a fillet or chamfer step (not in the
//! repository: `SW_CORPUS` names the corpus directory, default `corpus-local` at the workspace root; without it, or
//! without a file, the test passes and does nothing). The number of read-back steps the editable prefix keeps must
//! not fall below what the blend work reached, and the importBody remainder must still land on the file's solid.

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
    let bytes = std::fs::read(path).unwrap();
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let t = Tagged::from_import(&imp);
    let out = rebuild_history(&t);
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
fn blend_files_keep_their_steps() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    // (sample, must be complete, rebuilt steps kept at least)
    let cases: [(&str, bool, usize); 7] = [
        // a through cut whose mouth a later chamfer trimmed: the file now rebuilds to the end
        ("nist_ftc_09", true, 33),
        // free-form blend strips where the file has cylinders, and the steps after them
        ("nist_ftc_08", false, 24),
        // a boss whose end a later fillet trimmed runs on to the fillet's other support
        ("pend_table_mount", false, 13),
        ("nist_ctc_01", false, 3),
        ("nist_ctc_03", false, 2),
        ("pend_encoder", false, 2),
        // a spoked web whose opening corners and rim rounds meet at unequal radii: read back as the full annulus,
        // full-circle rounds and the openings cut last (reordered, verified against the whole body)
        ("pend_spoked_wheel_12T", true, 6),
    ];
    for (stem, complete, min_kept) in cases {
        let Some(path) = find(&dir, stem) else {
            eprintln!("{stem}: not in the corpus at {}", dir.display());
            continue;
        };
        let (done, kept, same) = run(&path);
        assert!(same, "{stem}: the composed body no longer lands on the file's solid");
        assert!(kept >= min_kept, "{stem}: {kept} rebuilt steps kept, at least {min_kept} expected");
        if complete {
            assert!(done, "{stem}: no longer rebuilds to the end");
        }
    }
}

/// The table mount's bracket reaches the floor it sits on and its lug boss keeps the height of its sides, so the last
/// chamfer finds its concave edge (it used to stop on a missing edge).
#[test]
fn table_mount_last_chamfer_finds_its_edge() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let Some(path) = find(&dir, "pend_table_mount") else { return };
    let imp = ck_sldprt::import_sldprt(&std::fs::read(path).unwrap()).unwrap();
    let out = rebuild_history(&Tagged::from_import(&imp));
    let why = out.fallback.clone().unwrap_or_default();
    assert!(!why.contains("missingEdge"), "pend_table_mount: {why}");
}
