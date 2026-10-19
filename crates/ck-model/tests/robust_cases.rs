//! Every minimal crash/hang case from the fuzzer (tools/cases/crash/*.json) must finish within the time
//! limit without panicking, and any failure must be a structured error with a code. See docs/robustness.md.
#[path = "robust_common/mod.rs"]
mod common;

#[test]
fn crash_cases_do_not_panic_or_hang() {
    let dir = format!("{}/../../tools/cases/crash", env!("CARGO_MANIFEST_DIR"));
    let mut ids: Vec<String> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok()?.file_name().to_str()?.strip_suffix(".json").map(String::from))
        .collect();
    ids.sort();
    assert!(!ids.is_empty());
    for id in ids {
        let t = std::time::Instant::now();
        let errs = common::regen_codes_bounded(common::crash_case(&id), 20);
        eprintln!("{id}: {:.2}s {:?}", t.elapsed().as_secs_f64(), errs.iter().map(|e| format!("{}:{} {}", e.0, e.1, e.2)).collect::<Vec<_>>());
        for (feat, code, msg) in &errs {
            assert!(!code.is_empty(), "{id}: feature {feat} failed with an empty code ({msg})");
        }
    }
}
