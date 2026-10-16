//! Shape healing over ck-topo bodies. Fix catalogue and ordering adapted
//! from brepkit v2.129.15 (MIT OR Apache-2.0), crates/heal/src; problems
//! are detected with ck-check and every repair is reported.
//!
//! Fixes, in pass order: near-miss vertex merging, degenerate edge / small
//! face removal, open / misordered / reversed loop chaining, free-edge
//! sewing (with shell + lump merging), face orientation consistency and
//! inside-out solid flipping. Each pass only touches what the validator
//! flagged, so a valid body is returned unchanged.

pub mod damage;
mod orient;
mod sew;
mod small;
mod tolerance;
mod vertex;
mod wire;

use ck_check::Issue;
use ck_topo::body::Body;

/// One repair performed.
#[derive(Debug, Clone, PartialEq)]
pub struct Fixed {
    pub code: &'static str,
    pub entity: String,
    pub detail: String,
}

#[derive(Debug, Clone, Default)]
pub struct HealReport {
    pub fixed: Vec<Fixed>,
    /// Validator findings left after healing.
    pub remaining: Vec<Issue>,
}

/// What `heal_with` may do.
#[derive(Debug, Clone, Copy)]
pub struct HealOpts {
    /// Model tolerance (mm) used by the structural fixes (merge, sew, small).
    pub tol: f64,
    /// Merge near vertices, close gaps and sew free edges.
    pub sew: bool,
    /// Drop degenerate edges and sliver faces.
    pub fix_small: bool,
    /// Make loop senses and shell orientation consistent.
    pub fix_orientation: bool,
    /// Give edges / vertices that deviate from their faces a tolerance.
    pub tolerant: bool,
    /// Largest edge / vertex tolerance (mm) that may be set (default 0.05).
    pub cap: f64,
}

impl HealOpts {
    pub fn new(tol: f64) -> HealOpts {
        HealOpts { tol, sew: true, fix_small: true, fix_orientation: true, tolerant: false, cap: ck_check::DEFAULT_TOL_CAP }
    }
}

/// Heal `b` in place with model tolerance `tol` (mm). No tolerant-edge pass.
pub fn heal(b: &mut Body, tol: f64) -> HealReport {
    heal_with(b, &HealOpts::new(tol))
}

/// `heal` with every fixer switchable and the tolerant-geometry pass.
pub fn heal_with(b: &mut Body, o: &HealOpts) -> HealReport {
    let mut report = heal_structure(b, o);
    if o.tolerant {
        // tolerances are measured against the strict validation tolerance, not
        // the (looser) structural one, so `validate` agrees afterwards
        let strict = o.tol.min(1e-6);
        let mut fixed = std::mem::take(&mut report.fixed);
        let (set, over) = tolerance::set_tolerances(b, strict, o.cap, &mut fixed);
        if set + over > 0 {
            report.remaining = ck_check::validate_cap(b, strict, o.cap);
        }
        report.fixed = fixed;
    } else if !report.fixed.is_empty() {
        report.remaining = ck_check::validate_cap(b, o.tol, o.cap);
    }
    report
}

fn heal_structure(b: &mut Body, o: &HealOpts) -> HealReport {
    let tol = o.tol;
    let mut fixed: Vec<Fixed> = vec![];
    let mut issues = ck_check::validate_cap(b, tol, o.cap);
    for _ in 0..4 {
        if issues.is_empty() {
            break;
        }
        let has = |c: &str| issues.iter().any(|i| i.code == c);
        let n0 = fixed.len();
        if o.sew && (has("SHELL_OPEN") || has("LOOP_OPEN") || has("EDGE_DEGENERATE") || has("EDGE_UNUSED")) {
            vertex::merge_near_vertices(b, tol, &mut fixed);
        }
        if o.fix_small && has("EDGE_DEGENERATE") {
            small::remove_degenerate(b, tol, &mut fixed);
        }
        if o.sew && has("LOOP_OPEN") {
            wire::fix_loops(b, &mut fixed);
        }
        if o.sew && (has("SHELL_OPEN") || has("EDGE_UNUSED") || has("SHELL_EMPTY")) {
            let joined = sew::sew_free_edges(b, tol, &mut fixed);
            sew::merge_shells(b, &joined, &mut fixed);
        }
        if fixed.len() == n0 {
            break;
        }
        // sewing can introduce orientation conflicts: re-check, then orient
        issues = ck_check::validate_cap(b, tol, o.cap);
        if o.fix_orientation && issues.iter().any(|i| matches!(i.code, "SHELL_ORIENTATION" | "VOLUME_NONPOSITIVE")) {
            orient::fix_loop_senses(b, &mut fixed);
            orient::fix_orientation(b, &mut fixed);
            issues = ck_check::validate_cap(b, tol, o.cap);
        }
    }
    // orientation-only damage never reaches the structural fixes above
    if o.fix_orientation && issues.iter().any(|i| matches!(i.code, "SHELL_ORIENTATION" | "VOLUME_NONPOSITIVE")) {
        orient::fix_loop_senses(b, &mut fixed);
        orient::fix_orientation(b, &mut fixed);
        issues = ck_check::validate_cap(b, tol, o.cap);
    }
    HealReport { fixed, remaining: issues }
}

#[cfg(test)]
mod tests;
