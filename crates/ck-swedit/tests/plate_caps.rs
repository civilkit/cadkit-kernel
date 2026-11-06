//! A boss whose only faces on the final body are two caps at its sketch plane (a plate: later bevels, cuts and rounds
//! took all its sides, and the file lists no depth): read back from the outer outline of the caps, the rounds on its
//! rim taken off again, running down to the material before it. Runs over the local corpus (`SW_CORPUS`, default
//! `corpus-local/samples` at the workspace root); passes without it.

use ck_swedit::{rebuild_history, Status, Tagged};
use std::path::PathBuf;

#[test]
fn plate_read_from_its_sketch_plane_caps() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local/samples"));
    let Ok(bytes) = std::fs::read(dir.join("nist_ftc_07/part.SLDPRT")) else { return };
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let t = Tagged::from_import(&imp);
    let o = rebuild_history(&t);
    let s = o.steps.iter().find(|s| s.name == "Boss-Extrude2").unwrap();
    assert_eq!(s.status, Status::Rebuilt, "{}", s.note);
    // 0.5 in from the plate's sketch plane down to the top of the first boss; a rectangle with sharp corners
    assert!(s.note.contains("depth 12.7000") && s.note.contains("4 segment(s)"), "{}", s.note);
    // both steps are kept in the editable prefix and pass their checks
    let kept = o.steps[..o.prefix].iter().filter(|s| s.status == Status::Rebuilt).count();
    assert!(kept >= 2, "kept {kept}: {:?}", o.steps.iter().map(|s| format!("{} {:?}", s.name, s.status)).collect::<Vec<_>>());
    assert!(o.checks.iter().all(|c| c.ok()), "{:?}", o.checks);
}
