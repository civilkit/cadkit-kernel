//! Work budgets: a feature (or the whole regen) that runs past its time budget stops with a structured
//! `...BudgetExceeded` error instead of grinding on. The clock here is a counter (1 ms per reading), so the
//! test is deterministic and does not depend on the speed of the machine.
#[path = "robust_common/mod.rs"]
mod common;

use std::cell::Cell;

thread_local! { static TICKS: Cell<f64> = const { Cell::new(0.0) }; }
fn counter_clock() -> f64 {
    TICKS.with(|t| {
        t.set(t.get() + 1.0);
        t.get()
    })
}

fn with_budget(id: &str, opts: &str) -> Vec<(String, String, String)> {
    let src = common::crash_case(id);
    let opts = opts.to_string();
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            ck_core::budget::set_clock(counter_clock);
            common::regen_codes_opts(&src, &opts)
        })
        .unwrap()
        .join()
        .unwrap()
}

#[test]
fn feature_over_budget_reports_a_budget_error_with_numbers() {
    // case_646: a cylinder, then a linear pattern of it (many boolean attempts)
    let errs = with_budget("case_646", r#"{"solveSketches":true,"featureTimeMs":40}"#);
    let e = errs.iter().find(|e| e.1.ends_with("BudgetExceeded")).unwrap_or_else(|| panic!("no budget error in {errs:?}"));
    assert!(e.2.contains("budgetMs=40") && e.2.contains("elapsedMs=") && e.2.contains("hint="), "{e:?}");
}

#[test]
fn regen_over_budget_skips_the_remaining_features() {
    let errs = with_budget("case_646", r#"{"solveSketches":true,"featureTimeMs":0,"regenTimeMs":30}"#);
    assert!(errs.iter().any(|e| e.1 == "regenBudgetExceeded"), "{errs:?}");
}

#[test]
fn no_clock_no_budget() {
    // without a host clock the limits are inert (native tests, tools): the same case runs to its own result
    let errs = common::regen_codes_opts(&common::crash_case("case_646"), r#"{"solveSketches":true,"featureTimeMs":1}"#);
    assert!(errs.iter().all(|e| !e.1.ends_with("BudgetExceeded")), "{errs:?}");
}
