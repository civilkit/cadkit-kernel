//! Editable rebuild over a directory of part files (not in the repository). `SW_CORPUS` names the directory
//! (default: `corpus-local` at the workspace root, when present); every `.sldprt` below it is imported, rebuilt and
//! verified. `SWEDIT_OUT` names a Markdown report to write. Without a corpus the test passes and does nothing.

use ck_swedit::{classify, rebuild_history, Class, Status, Tagged};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    let mut es: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            files(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
            out.push(p);
        }
    }
}

/// A label for the report: the file's parent folder, else its stem.
fn label(p: &Path) -> String {
    let parent = p.parent().and_then(|d| d.file_name()).map(|s| s.to_string_lossy().to_string());
    match parent {
        Some(d) if !["samples", "downloaded", "corpus-local"].contains(&d.as_str()) => d,
        _ => p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default(),
    }
}

#[test]
fn corpus() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let mut fs = vec![];
    files(&dir, &mut fs);
    if fs.is_empty() {
        eprintln!("no corpus at {}", dir.display());
        return;
    }
    let mut rows: Vec<String> = vec![];
    let mut slow: Vec<(String, u128)> = vec![];
    let mut per_kind: BTreeMap<String, [usize; 3]> = BTreeMap::new(); // rebuilt, unsupported, skipped
    let mut causes: BTreeMap<String, usize> = BTreeMap::new();
    let (mut n_exact, mut n_close, mut n_fallback, mut n_noimport) = (0, 0, 0, 0);
    // files with an editable prefix: (label, prefix steps, steps read back, composed body verified)
    let mut prefixes: Vec<(String, usize, usize, String)> = vec![];
    let mut detail = String::new();
    for p in &fs {
        let name = label(p);
        let bytes = std::fs::read(p).unwrap();
        let t0 = std::time::Instant::now();
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                n_noimport += 1;
                rows.push(format!("| {name} | - | - | not imported | {} |", e.code));
                continue;
            }
        };
        // reading the file (container, Parasolid, faces, pcurves, orientation) is bounded on its own: it took tens of
        // seconds on files with many free-form faces and long intersection curves, whatever their size (closest-point
        // grid scans, face integrals of rational faces); about 5 s was the slowest measured file, native release
        let import_ms = t0.elapsed().as_millis();
        let import_limit: u128 = std::env::var("SWEDIT_MAX_IMPORT_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(15_000);
        assert!(import_ms < import_limit, "{name}: {import_ms} ms to import (bound {import_limit} ms)");
        let t = Tagged::from_import(&imp);
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rebuild_history(&t)));
        let ms = t0.elapsed().as_millis();
        // a file that reads back slowly is slow in the app too (wasm): no file may take more than this
        slow.push((name.clone(), ms));
        let out = match r {
            Ok(o) => o,
            Err(_) => {
                n_fallback += 1;
                *causes.entry("panic".into()).or_default() += 1;
                rows.push(format!("| {name} | {} | - | PANIC | |", imp.body.num_faces()));
                continue;
            }
        };
        let nfeat = imp.meta.features.iter().filter(|f| !matches!(classify(f), Class::Skip)).count();
        for s in &out.steps {
            let k = per_kind.entry(s.kind.clone()).or_default();
            match s.status {
                Status::Rebuilt => k[0] += 1,
                Status::Unsupported => k[1] += 1,
                Status::Skipped => k[2] += 1,
            }
        }
        let status = if out.exact() {
            n_exact += 1;
            "exact".to_string()
        } else if out.close() {
            n_close += 1;
            "close".to_string()
        } else {
            n_fallback += 1;
            let why = out.fallback.clone().unwrap_or_default();
            let cause = if let Some(s) = out.steps.iter().find(|s| s.status == Status::Unsupported) {
                format!("{}: {}", s.kind, s.note.split('(').next().unwrap_or("").trim())
            } else if why.starts_with("regen") {
                "regen error".to_string()
            } else if why.starts_with("rebuilt body differs") {
                "volume mismatch".to_string()
            } else {
                why.clone()
            };
            *causes.entry(cause).or_default() += 1;
            if out.prefix > 0 {
                // the part document with the file as its blob must regenerate to the plain import
                let t1 = std::time::Instant::now();
                let p = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "import", None).solid;
                let comp = match ck_swedit::prefix::regen_part(&out.part_json(Some(&bytes))) {
                    Ok((Some(b), errs)) if errs.is_empty() => {
                        // the same reader and sorting: the same faces, volume and area to rounding (no boolean verify)
                        let (mp, mb) = (ck_topo::mass::mass_props(&p), ck_topo::mass::mass_props(&b));
                        let rel = ((mp.volume - mb.volume).abs() / mp.volume.abs().max(1e-12)).max((mp.area - mb.area).abs() / mp.area.abs().max(1e-12));
                        if p.num_faces() == b.num_faces() && rel < 1e-12 { "exact".to_string() } else { format!("DIFFERS {rel:.2e}, faces {} / {}", b.num_faces(), p.num_faces()) }
                    }
                    // the file's regions are no valid solid: the plain import has surface bodies only, and so has this
                    Ok((None, errs)) if errs.is_empty() && p.num_faces() == 0 => "exact (surface bodies, no solid)".to_string(),
                    Ok((_, errs)) => format!("REGEN {}", errs.join(", ")),
                    Err(e) => format!("PART {e}"),
                };
                let comp = format!("{comp}, {} ms", t1.elapsed().as_millis());
                let read = out.steps.iter().filter(|s| s.status == Status::Rebuilt).count();
                let kept = out.steps[..out.prefix].iter().filter(|s| s.status == Status::Rebuilt).count();
                prefixes.push((name.clone(), kept, read, comp.clone()));
                format!("prefix {} + remainder ({comp}): {why}", out.prefix)
            } else {
                format!("fallback: {why}")
            }
        };
        let rebuilt = out.steps.iter().filter(|s| s.status == Status::Rebuilt).count();
        // the bolt's modelled thread (a ridge swept along a helix) is read back and the history is complete
        if name == "pend_bolt" {
            assert!(out.exact() || out.close(), "pend_bolt: history not complete: {:?}", out.fallback);
            assert!(out.steps.iter().any(|s| s.kind == "Thread" && s.status == Status::Rebuilt), "pend_bolt: thread not rebuilt");
        }
        // NIST FTC 10 is a complete editable history (all 25 steps, no imported remainder)
        if name == "nist_ftc_10" {
            assert!(out.exact() || out.close(), "nist_ftc_10: history not complete: {:?}", out.fallback);
            assert!(out.remainder.is_none() && out.steps.iter().filter(|s| s.status == Status::Rebuilt).count() >= 19, "nist_ftc_10: steps missing");
        }
        // open time: the files whose verification used to take a minute (a modelled thread's boolean difference, the
        // whole prefix regenerated at every blend) stay within about 5x of the measured native release time
        let bound_ms: u128 = match name.as_str() {
            "pend_mount_table_sharft_holder" => 12_000,
            "pend_bolt" => 25_000,
            "nist_ftc_08" => 20_000,
            "nist_ftc_10" => 10_000,
            "pend_spoked_wheel_12T" => 18_000,
            // partial files: a reordered reading is tried only where an order could help (see `reorder.rs`)
            "nist_ftc_06" => 6_000,
            "pend_table_mount" => 10_000,
            "nist_ftc_07" | "nist_ctc_02" => 10_000,
            _ => u128::MAX,
        };
        assert!(ms < bound_ms, "{name}: {ms} ms to read back (bound {bound_ms} ms)");
        let v = out.verify.as_ref().map(|v| format!("{:.3e}", v.rel)).unwrap_or("-".into());
        rows.push(format!("| {name} | {} | {rebuilt}/{nfeat} | {status} | {v} | {ms} |", imp.body.num_faces()));
        detail.push_str(&format!("\n### {name}\n\n"));
        for s in &out.steps {
            detail.push_str(&format!("- {} `{}` ({} faces): {:?} {}\n", s.name, s.kind, s.faces, s.status, s.note));
        }
        if !out.errors.is_empty() {
            detail.push_str(&format!("- regen errors: {}\n", out.errors.join(", ")));
        }
        if let Some(c) = out.checks.iter().find_map(|c| c.failure()) {
            detail.push_str(&format!("- check failed on step {}: {c}\n", out.checks.len()));
        }
        if let Some(r) = &out.remainder {
            detail.push_str(&format!("- remainder from {} ({}): {} ({} steps)\n", r.from.1, r.from.2, r.reason, r.steps.len()));
        }
    }
    // lower bounds (files present in the corpus only): the extrudes of the spoked wheel, whose profile faces later
    // cuts touched, are read back and kept
    if let Some((_, kept, read, _)) = prefixes.iter().find(|p| p.0 == "pend_spoked_wheel_12T") {
        assert!(*read >= 5 && *kept >= 4, "pend_spoked_wheel_12T: {read} steps read back, {kept} kept");
    }
    // the encoder's disc and shaft are separate bodies that touch (boss with merge off): read back as new bodies
    // and the complete history rebuilds (the file has several bodies: later boss and cut act on the body they belong to)
    // (the pin through them and the pocket in the boss act on their own body: target)
    assert!(!prefixes.iter().any(|p| p.0 == "pend_encoder"), "pend_encoder: no longer a complete history");
    // the table mount's last chamfer keeps its listed distance (the supports are picked by it)
    if let Some((_, kept, read, _)) = prefixes.iter().find(|p| p.0 == "pend_table_mount") {
        assert!(*read >= 14 && *kept >= 13, "pend_table_mount: {read} steps read back, {kept} kept");
    }
    // the NIST FTC 06 rib (a two-line open profile run on to the body, ending exactly at a block corner), the cut with
    // a wall flaring out to the body's boundary and the boss feet under a later round are read back and kept
    if let Some((_, kept, read, _)) = prefixes.iter().find(|p| p.0 == "nist_ftc_06") {
        assert!(*read >= 21 && *kept >= 20, "nist_ftc_06: {read} steps read back, {kept} kept");
    }
    // NIST FTC 06 is a complete history (the holes that open into a gap end at their exit, the rib-foot rounds end on
    // the rib's slanted face); NIST CTC 04 keeps the fillet of the two in-line bosses, CTC 05 the chamfer of the bore
    // whose mouth it trimmed
    assert!(!prefixes.iter().any(|p| p.0 == "nist_ftc_06"), "nist_ftc_06: no longer a complete history");
    if let Some((_, kept, _, _)) = prefixes.iter().find(|p| p.0 == "nist_ctc_04") {
        assert!(*kept >= 21, "nist_ctc_04: {kept} steps kept");
    }
    if let Some((_, kept, _, _)) = prefixes.iter().find(|p| p.0 == "nist_ctc_05") {
        assert!(*kept >= 15, "nist_ctc_05: {kept} steps kept");
    }
    let mut md = String::new();
    md.push_str(&format!("# Editable rebuild: {} files\n\nexact {n_exact}, close {n_close}, fallback {n_fallback}, not imported {n_noimport}\n\n", fs.len()));
    let kept: usize = prefixes.iter().map(|p| p.1).sum();
    let read: usize = prefixes.iter().map(|p| p.2).sum();
    md.push_str(&format!("editable prefix + remainder: {} of the {n_fallback} fallback files ({kept} rebuilt steps kept of {read} read back)\n\n", prefixes.len()));
    md.push_str("| part | faces | rebuilt/features | result | rel. error | ms |\n|---|---|---|---|---|---|\n");
    for r in &rows {
        md.push_str(r);
        md.push('\n');
    }
    md.push_str("\n## Editable prefixes\n\n| part | rebuilt steps kept | rebuilt steps read back | composed body |\n|---|---|---|---|\n");
    for (n, k, r, c) in &prefixes {
        md.push_str(&format!("| {n} | {k} | {r} | {c} |\n"));
    }
    md.push_str("\n## Per feature type (steps over all files)\n\n| type | rebuilt | unsupported | skipped (no faces) |\n|---|---|---|---|\n");
    for (k, v) in &per_kind {
        md.push_str(&format!("| {k} | {} | {} | {} |\n", v[0], v[1], v[2]));
    }
    let mut cv: Vec<(&String, &usize)> = causes.iter().collect();
    cv.sort_by(|a, b| b.1.cmp(a.1));
    md.push_str("\n## Fallback causes\n\n");
    for (c, k) in cv {
        md.push_str(&format!("- {k} x {c}\n"));
    }
    md.push_str("\n## Per file\n");
    md.push_str(&detail);
    println!("{md}");
    if let Ok(out) = std::env::var("SWEDIT_OUT") {
        if let Some(d) = Path::new(&out).parent() {
            let _ = std::fs::create_dir_all(d);
        }
        std::fs::write(&out, md).unwrap();
    }
    let limit: u128 = std::env::var("SWEDIT_MAX_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(20_000);
    let over: Vec<&(String, u128)> = slow.iter().filter(|s| s.1 > limit).collect();
    assert!(over.is_empty(), "files slower than {limit} ms: {over:?}");
}
