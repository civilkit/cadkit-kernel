//! Regression: the history rebuild of the corpus loft parts stays fast (it once took 14-160 s: the verification's symmetric-difference booleans on the B-spline lateral faces, see
//! `fastverify.rs`).
//! Reads `SW_CORPUS` (default `corpus-local` at the workspace root); passes and does nothing without the files.

use ck_swedit::{rebuild_history, Tagged};
use std::path::PathBuf;

#[test]
fn loft_history_is_fast() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    for name in ["dump_SW2022_C16_loft_spline", "dump_SW2011_C16_loft_spline"] {
        let Some(path) = ["samples", "downloaded", "."].iter().map(|d| dir.join(d).join(name).join("model.SLDPRT")).find(|p| p.exists()) else {
            eprintln!("no {name} in {}", dir.display());
            continue;
        };
        let bytes = std::fs::read(path).unwrap();
        let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
        let t = Tagged::from_import(&imp);
        let t0 = std::time::Instant::now();
        let out = rebuild_history(&t);
        let ms = t0.elapsed().as_millis();
        eprintln!("{name}: {ms} ms, exact {}, close {}, prefix {}, fallback {:?}", out.exact(), out.close(), out.prefix, out.fallback);
        assert!(ms < 5000, "{name}: history rebuild took {ms} ms");
        // the result is the same as with the boolean verification: rebuilt to the end, close but not exact (the
        // rebuilt loft surface differs from the file's by ~1e-5 of the volume)
        assert!(out.fallback.is_none() && out.close() && !out.exact(), "{name}: {:?}", out.fallback);
        let rel = out.verify.as_ref().unwrap().rel;
        assert!(rel > 1e-6 && rel < 1e-4, "{name}: rel {rel:e}");
    }
}
