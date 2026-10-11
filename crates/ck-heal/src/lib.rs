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

/// Heal `b` in place with model tolerance `tol` (mm).
pub fn heal(b: &mut Body, tol: f64) -> HealReport {
    let mut fixed: Vec<Fixed> = vec![];
    let mut issues = ck_check::validate(b, tol);
    for _ in 0..4 {
        if issues.is_empty() {
            break;
        }
        let has = |c: &str| issues.iter().any(|i| i.code == c);
        let n0 = fixed.len();
        if has("SHELL_OPEN") || has("LOOP_OPEN") || has("EDGE_DEGENERATE") || has("EDGE_UNUSED") {
            vertex::merge_near_vertices(b, tol, &mut fixed);
        }
        if has("EDGE_DEGENERATE") {
            small::remove_degenerate(b, tol, &mut fixed);
        }
        if has("LOOP_OPEN") {
            wire::fix_loops(b, &mut fixed);
        }
        if has("SHELL_OPEN") || has("EDGE_UNUSED") || has("SHELL_EMPTY") {
            let joined = sew::sew_free_edges(b, tol, &mut fixed);
            sew::merge_shells(b, &joined, &mut fixed);
        }
        if fixed.len() == n0 {
            break;
        }
        // sewing can introduce orientation conflicts: re-check, then orient
        issues = ck_check::validate(b, tol);
        if issues.iter().any(|i| matches!(i.code, "SHELL_ORIENTATION" | "VOLUME_NONPOSITIVE")) {
            orient::fix_loop_senses(b, &mut fixed);
            orient::fix_orientation(b, &mut fixed);
            issues = ck_check::validate(b, tol);
        }
    }
    // orientation-only damage never reaches the structural fixes above
    if issues.iter().any(|i| matches!(i.code, "SHELL_ORIENTATION" | "VOLUME_NONPOSITIVE")) {
        orient::fix_loop_senses(b, &mut fixed);
        orient::fix_orientation(b, &mut fixed);
        issues = ck_check::validate(b, tol);
    }
    HealReport { fixed, remaining: issues }
}

#[cfg(test)]
mod tests;
