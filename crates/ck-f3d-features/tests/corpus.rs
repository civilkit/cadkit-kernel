//! The Fusion corpus through the timeline mapping and the guard.
//!
//! `$CK_F3D_CORPUS` is a directory of `.f3d` / `.f3z` files (searched recursively; they are third-party and never in the
//! repository). Unset or empty: the test says so and passes. Per design it counts the timeline items, those mapped and
//! those placeholders, and whether the regenerated history has the volume and box of the body the ASM reader builds from
//! the file (the guard). Only aggregates are written, to `tools/f3d_features_baseline.json` with
//! `CK_F3D_UPDATE_BASELINE=1` (no file names); otherwise the numbers must not fall below the baseline.
//! `CK_F3D_NAMES=1` prints each file's line with its name, for reading locally; `CK_F3D_FILTER=<text>` runs the files
//! whose path contains the text.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use ck_f3d_features::{Status, Verdict, guard, timeline_to_features};
use serde_json::{Value, json};

fn find_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            find_files(&p, out);
        } else if matches!(p.extension().and_then(|x| x.to_str()), Some("f3d" | "f3z")) {
            out.push(p);
        }
    }
}

/// A reason without the names and numbers in it, so equal reasons count together.
fn generic(s: &str) -> String {
    let mut out = String::new();
    let mut quoted = false;
    for ch in s.chars() {
        match ch {
            '"' => {
                quoted = !quoted;
                if quoted {
                    out.push_str("\"..\"");
                }
            }
            _ if quoted => {}
            c if c.is_ascii_digit() => {
                if !out.ends_with('#') {
                    out.push('#');
                }
            }
            c => out.push(c),
        }
    }
    out.chars().take(110).collect()
}

#[derive(Default)]
struct Totals {
    files: usize,
    designs: usize,
    no_timeline: usize,
    items: usize,
    mapped: usize,
    placeholder: usize,
    fully_mapped: usize,
    verified: usize,
    verified_fully_mapped: usize,
    comparable: usize,
    errored: usize,
    by_type: BTreeMap<String, [usize; 2]>,
    reasons: BTreeMap<String, usize>,
    unverified_reasons: BTreeMap<String, usize>,
}

#[test]
fn the_whole_corpus_through_the_timeline_mapping() {
    let Ok(dir) = std::env::var("CK_F3D_CORPUS") else {
        eprintln!("skipped: CK_F3D_CORPUS is not set");
        return;
    };
    let names = std::env::var("CK_F3D_NAMES").is_ok();
    let filter = std::env::var("CK_F3D_FILTER").ok();
    let mut files = Vec::new();
    find_files(&PathBuf::from(dir), &mut files);
    files.sort();
    if files.is_empty() {
        eprintln!("skipped: no .f3d / .f3z files in CK_F3D_CORPUS");
        return;
    }
    if let Some(f) = &filter {
        files.retain(|p| p.to_string_lossy().contains(f.as_str()));
    }
    let mut t = Totals::default();
    for (n, path) in files.iter().enumerate() {
        let label = if names { path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default() } else { format!("#{n}") };
        let Ok(bytes) = std::fs::read(path) else { continue };
        t.files += 1;
        let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let designs = match std::panic::catch_unwind(|| ck_f3d_design::decode_bytes(bytes.clone(), &name)) {
            Ok(Ok(d)) => d,
            _ => {
                t.errored += 1;
                eprintln!("{label}: the design could not be decoded");
                continue;
            }
        };
        let references = guard::measure_documents(&bytes);
        for (di, design) in designs.into_iter().enumerate() {
            let reference = match references.get(di) {
                Some(Ok(m)) => Some(*m),
                Some(Err(e)) => {
                    if names {
                        eprintln!("{label}: no reference body: {e}");
                    }
                    None
                }
                None => None,
            };
            t.designs += 1;
            let Some(dump) = design.dump else {
                t.no_timeline += 1;
                continue;
            };
            if dump.timeline_items().is_empty() {
                t.no_timeline += 1;
                eprintln!("{label}: no timeline");
                continue;
            }
            let import = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| timeline_to_features(&dump))) {
                Ok(i) => i,
                Err(_) => {
                    t.errored += 1;
                    eprintln!("{label}: the mapping panicked");
                    continue;
                }
            };
            let (mapped, placeholder) = (import.mapped(), import.placeholders());
            t.items += import.items.len();
            t.mapped += mapped;
            t.placeholder += placeholder;
            for it in &import.items {
                let e = t.by_type.entry(if it.object_type.is_empty() { "(unknown)".into() } else { it.object_type.clone() }).or_default();
                e[usize::from(it.status == Status::Placeholder)] += 1;
                if it.status == Status::Placeholder
                    && let Some(n) = &it.note
                {
                    *t.reasons.entry(generic(n)).or_default() += 1;
                }
            }
            let full = placeholder == 0;
            t.fully_mapped += usize::from(full);
            let check = guard::verify(&import, reference.as_ref());
            if reference.is_some() {
                t.comparable += 1;
            }
            if names && filter.is_some() {
                for it in &import.items {
                    eprintln!("  item {} {} {:?} {:?}", it.index, it.object_type, it.status, it.note);
                }
                eprintln!("  check: reason {:?} errors {:?} ref {:?} hist {:?}", check.reason, check.errors, check.reference, check.history);
            }
            let ok = check.verdict == Verdict::Verified;
            t.verified += usize::from(ok);
            t.verified_fully_mapped += usize::from(ok && full);
            if !ok && let Some(r) = &check.reason {
                *t.unverified_reasons.entry(generic(r)).or_default() += 1;
            }
            eprintln!(
                "{label}#{di}: items {} mapped {mapped} placeholder {placeholder} verified {} volΔ {} boxΔ {}{}",
                import.items.len(),
                ok,
                check.volume_delta.map_or("-".into(), |v| format!("{:+.4}", v)),
                check.box_delta.map_or("-".into(), |v| format!("{v:.4}")),
                if names { format!(" | {}", check.reason.as_deref().unwrap_or("")) } else { String::new() },
            );
        }
    }
    let by_type: serde_json::Map<String, Value> = t
        .by_type
        .iter()
        .map(|(k, v)| (k.clone(), json!({"mapped": v[0], "placeholder": v[1]})))
        .collect();
    let summary = json!({
        "_about": "Aggregate numbers of crates/ck-f3d-features/tests/corpus.rs over the Fusion corpus (no file names): timeline items mapped to part features or left as placeholders, and designs whose regenerated history has the volume and box of the imported body. Rewritten by CK_F3D_UPDATE_BASELINE=1.",
        "files": t.files,
        "designs": t.designs,
        "designsWithoutTimeline": t.no_timeline,
        "decodeFailures": t.errored,
        "items": t.items,
        "mapped": t.mapped,
        "placeholder": t.placeholder,
        "fullyMappedDesigns": t.fully_mapped,
        "comparableDesigns": t.comparable,
        "verifiedDesigns": t.verified,
        "verifiedFullyMappedDesigns": t.verified_fully_mapped,
        "byType": by_type,
        "placeholderReasons": t.reasons,
        "unverifiedReasons": t.unverified_reasons,
    });
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/f3d_features_baseline.json");
    eprintln!("{}", serde_json::to_string_pretty(&summary).unwrap_or_default());
    if filter.is_some() {
        return;
    }
    if std::env::var("CK_F3D_UPDATE_BASELINE").is_ok() {
        let text = serde_json::to_string_pretty(&summary).unwrap_or_default() + "\n";
        std::fs::write(&path, text).expect("writes the baseline");
        return;
    }
    if let Ok(text) = std::fs::read_to_string(&path)
        && let Ok(base) = serde_json::from_str::<Value>(&text)
    {
        let get = |v: &Value, k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        for key in ["mapped", "verifiedDesigns", "verifiedFullyMappedDesigns", "fullyMappedDesigns"] {
            assert!(
                get(&summary, key) >= get(&base, key),
                "{key} fell from {} to {}",
                get(&base, key),
                get(&summary, key)
            );
        }
    }
}
