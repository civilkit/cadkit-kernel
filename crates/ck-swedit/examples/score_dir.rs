//! Scores a directory of `.sldprt` files (walks sub-folders; no files are part of the repository):
//!
//!     cargo run --release -p ck-swedit --example score_dir -- <dir> [<dir> ...]
//!
//! One line per file: class (`exact` / `close` / `partial` / `plain` / `not-imported`), steps kept / total, validity
//! (first validator code, or `valid`), volume against the file's stored volume (relative error, `n/a` when the file
//! stores none or a placeholder), seconds, and the first stop reason.

use ck_swedit::{classify, rebuild_history, Class, Status, Tagged};
use std::path::{Path, PathBuf};

fn walk(d: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(d) else { return };
    let mut es: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            walk(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
            out.push(p);
        }
    }
}

fn main() {
    let mut files = vec![];
    for a in std::env::args().skip(1) {
        let p = PathBuf::from(a);
        if p.is_dir() {
            walk(&p, &mut files);
        } else {
            files.push(p);
        }
    }
    let mut tally = std::collections::BTreeMap::<&str, usize>::new();
    for p in &files {
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        let t0 = std::time::Instant::now();
        let Ok(bytes) = std::fs::read(p) else { continue };
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                *tally.entry("not-imported").or_default() += 1;
                println!("{name} | not-imported | - | - | - | {:.2} | {}", t0.elapsed().as_secs_f64(), e.code);
                continue;
            }
        };
        let issues = ck_check::validate(&imp.body, 1e-6);
        let valid = match issues.first() {
            None => "valid".to_string(),
            Some(i) => format!("INVALID({}): {}", issues.len(), i.code),
        };
        // a file of sheet bodies: sorted as the importSldprt request sorts it, they must all come out as surface bodies
        let valid = if imp.sheets > 0 {
            let s = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "import", None);
            let area: f64 = s.surfaces.iter().map(|(_, b)| ck_topo::mass::mass_props(b).area).sum();
            format!("SURFACES({} sheets: {} surface bodies, {} solid faces, area {:.0} mm2)", imp.sheets, s.surfaces.len(), s.solid.num_faces(), area)
        } else {
            valid
        };
        let vol = ck_topo::mass::mass_props(&imp.body).volume;
        let dvol = match imp.meta.mass {
            Some(m) if m.volume_m3 > 0.0 && (m.volume_m3 * 1e9 - 1000.0).abs() > 1e-6 => format!("{:+.2}%", (vol / (m.volume_m3 * 1e9) - 1.0) * 100.0),
            _ => "n/a".into(),
        };
        let t = Tagged::from_import(&imp);
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rebuild_history(&t)));
        let secs = t0.elapsed().as_secs_f64();
        let Ok(out) = r else {
            *tally.entry("plain").or_default() += 1;
            println!("{name} | plain | - | {valid} | {dvol} | {secs:.2} | PANIC");
            continue;
        };
        let total = imp.meta.features.iter().filter(|f| !matches!(classify(f), Class::Skip)).count();
        let kept = if out.prefix > 0 { out.steps[..out.prefix].iter().filter(|s| s.status == Status::Rebuilt).count() } else { out.steps.iter().filter(|s| s.status == Status::Rebuilt).count() };
        let class = if out.exact() {
            "exact"
        } else if out.close() {
            "close"
        } else if out.prefix > 0 {
            "partial"
        } else {
            "plain"
        };
        *tally.entry(class).or_default() += 1;
        let stop = if class == "exact" || class == "close" {
            String::new()
        } else if let Some(s) = out.steps.iter().find(|s| s.status == Status::Unsupported) {
            format!("{} [{}]: {}", s.kind, s.name, s.note)
        } else {
            out.fallback.clone().unwrap_or_default()
        };
        let stop: String = stop.chars().take(110).collect();
        println!("{name} | {class} | {kept}/{total} | {valid} | {dvol} | {secs:.2} | {stop}");
    }
    let s: Vec<String> = tally.iter().map(|(k, v)| format!("{k} {v}")).collect();
    println!("TOTAL {} files: {}", files.len(), s.join(", "));
}
