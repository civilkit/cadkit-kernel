//! Sketch solver status: degrees of freedom, conflicting and redundant
//! constraints, convergence. Additive: the oracle kernel reports none of this
//! (the original client derives "fully defined" itself, `si()`/`ti()` in the
//! app; `solver::status` is that port). Wire shape, added to a `sketches`
//! entry only when the regen request carries `opts.sketchStatus: true`:
//!
//! ```text
//! "status": {"dof": 0, "fullyDefined": true, "conflicts": [], "redundant": [],
//!            "solved": true, "state": "full", "residual": 1e-12}
//! ```
//!
//! * `dof` = unknowns - rank(J). Unknowns are `x`,`y` of every non-fixed point
//!   plus `r` of circles/ellipses (the solver's unknowns, so a free point is 2).
//! * `state` is the oracle client's `si().status`: `"over"` when any equation
//!   is dependent on earlier ones (rank < equations), else `"full"` at dof 0,
//!   else `"under"`. `fullyDefined` = `state == "full"` and `solved`.
//! * `solved`: the sketch's coordinates satisfy every constraint (max |f| <=
//!   1e-7, the solver's success bound), or a solve from them converges. The
//!   regen reply only carries re-solved coordinates when a constraint `expr`
//!   changed a value (oracle behaviour, unchanged), so a draft whose
//!   coordinates are off still reports `solved: true` when it converges;
//!   `residual` is max |f| at the coordinates in the reply.
//! * Rank: equations are taken in solver order (arc radius links, then the
//!   constraints in document order) and orthogonalised incrementally
//!   (modified Gram-Schmidt with one re-orthogonalisation pass) against the
//!   rows already accepted, on the central-difference Jacobian at the solution.
//!   A row is independent when the part of it outside the accepted span has
//!   norm > `1e-6` x its own norm (rows with norm < `1e-9` are dependent).
//!   This is a column-pivot-free QR of Jᵀ in constraint order, so the verdict
//!   on which constraint is the extra one is deterministic: the LATER
//!   constraint is blamed.
//! * A constraint with a dependent row is `conflicts` when that row's residual
//!   at the solution is > `1e-6` (it cannot hold together with the earlier
//!   ones), else `redundant` (it is implied). Conflicts win over redundant.
//!   Ids are listed in document order, once each.
//! * If the system cannot be built (missing point etc.; the kernel then leaves
//!   the sketch unsolved without an error) `solved` is false and `dof` is the
//!   number of free coordinates.

use super::solver::{build, jacobian, residuals, solve, SolveOpts};
use super::Sketch;
use ck_json::Json;

/// Relative independence threshold (see module docs).
pub const RANK_TOL: f64 = 1e-6;
/// Residual above which a dependent row is a conflict.
pub const CONFLICT_TOL: f64 = 1e-6;
/// Satisfied-constraints bound (the solver's success bound).
pub const SOLVED_TOL: f64 = 1e-7;

#[derive(Debug, Clone, PartialEq)]
pub struct SketchStatus {
    pub dof: usize,
    pub fully_defined: bool,
    pub conflicts: Vec<String>,
    pub redundant: Vec<String>,
    pub solved: bool,
    /// `"under"` | `"full"` | `"over"`.
    pub state: &'static str,
    pub residual: f64,
}

impl SketchStatus {
    pub fn to_json(&self) -> Json {
        let ids = |v: &[String]| Json::Arr(v.iter().map(|s| Json::Str(s.clone())).collect());
        Json::Obj(vec![
            ("dof".into(), Json::Num(self.dof as f64)),
            ("fullyDefined".into(), Json::Bool(self.fully_defined)),
            ("conflicts".into(), ids(&self.conflicts)),
            ("redundant".into(), ids(&self.redundant)),
            ("solved".into(), Json::Bool(self.solved)),
            ("state".into(), Json::Str(self.state.into())),
            ("residual".into(), Json::Num(if self.residual.is_finite() { self.residual } else { -1.0 })),
        ])
    }
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|a| a * a).sum::<f64>().sqrt()
}

/// Incremental row-space basis (orthonormal rows).
struct Basis {
    q: Vec<Vec<f64>>,
}

impl Basis {
    /// Try to add `row`; false if it lies in the span of the accepted rows.
    fn add(&mut self, row: &[f64]) -> bool {
        let n0 = norm(row);
        if !(n0 >= 1e-9) {
            return false;
        }
        let mut v = row.to_vec();
        for _ in 0..2 {
            for q in &self.q {
                let d: f64 = v.iter().zip(q).map(|(a, b)| a * b).sum();
                for (a, b) in v.iter_mut().zip(q) {
                    *a -= d * b;
                }
            }
        }
        let n1 = norm(&v);
        if n1 > RANK_TOL * n0 {
            for a in v.iter_mut() {
                *a /= n1;
            }
            self.q.push(v);
            true
        } else {
            false
        }
    }
}

pub fn sketch_status(sk: &Sketch) -> SketchStatus {
    let sys = match build(sk) {
        Ok(s) => s,
        Err(_) => {
            let free = sk.points.iter().filter(|p| !p.fixed).count() * 2;
            return SketchStatus { dof: free, fully_defined: false, conflicts: vec![], redundant: vec![], solved: false, state: "under", residual: f64::NAN };
        }
    };
    let n = sys.x.len();
    let mut x = sys.x.clone();
    let res0 = residuals(&sys, &x);
    let residual = res0.iter().fold(0.0f64, |m, v| if v.is_nan() || m.is_nan() { f64::NAN } else { m.max(v.abs()) });
    let mut res = res0;
    let mut solved = residual <= SOLVED_TOL;
    if !solved {
        let t = solve(&sys, &SolveOpts::default());
        if t.ok {
            solved = true;
            x = t.x;
            res = residuals(&sys, &x);
        }
    }
    let jac = jacobian(&sys, &mut x);
    let mut basis = Basis { q: Vec::new() };
    let mut dependent = vec![false; sys.eqs.len()];
    let mut row = vec![0.0; n];
    for (i, sparse) in jac.iter().enumerate() {
        row.iter_mut().for_each(|a| *a = 0.0);
        for &(c, v) in sparse {
            row[c] = v;
        }
        dependent[i] = !basis.add(&row);
    }
    let rank = basis.q.len();
    let mut conflicts: Vec<String> = Vec::new();
    let mut redundant: Vec<String> = Vec::new();
    let mut structural_dep = false;
    for (i, eq) in sys.eqs.iter().enumerate() {
        if !dependent[i] {
            continue;
        }
        if eq.cid.starts_with("arc:") {
            structural_dep = true;
            continue;
        }
        let bad = !(res[i].abs() <= CONFLICT_TOL);
        let (to, other) = if bad { (&mut conflicts, &redundant) } else { (&mut redundant, &conflicts) };
        if !to.contains(&eq.cid) && !other.contains(&eq.cid) {
            to.push(eq.cid.clone());
        }
    }
    // a constraint that is a conflict in one row and redundant in another counts once, as a conflict
    redundant.retain(|c| !conflicts.contains(c));
    let order = |v: &mut Vec<String>| v.sort_by_key(|c| sk.constraints.iter().position(|k| &k.id == c).unwrap_or(usize::MAX));
    order(&mut conflicts);
    order(&mut redundant);
    let dof = n - rank;
    let over = structural_dep || !conflicts.is_empty() || !redundant.is_empty();
    let state = if over {
        "over"
    } else if dof == 0 {
        "full"
    } else {
        "under"
    };
    SketchStatus { dof, fully_defined: state == "full" && solved, conflicts, redundant, solved, state, residual }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
        let src = format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
        Sketch::from_json(&ck_json::parse(&src).unwrap())
    }
    const RECT_ENTS: &str = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#;
    const HV: &str = r#"{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},{"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]}"#;
    const RECT_PTS: &str = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":0,"y":5}"#;
    fn dist(id: &str, a: &str, b: &str, v: f64) -> String {
        format!(r#"{{"type":"distance","id":"{id}","refs":["{a}","{b}"],"value":{v},"offset":null}}"#)
    }

    #[test]
    fn free_point_is_two_dof() {
        let s = sketch_status(&sk(r#"{"id":"p1","x":1,"y":2}"#, "", ""));
        assert_eq!((s.dof, s.fully_defined, s.state, s.solved), (2, false, "under", true));
        assert!(s.conflicts.is_empty() && s.redundant.is_empty());
    }

    #[test]
    fn fixed_point_is_zero_dof() {
        let s = sketch_status(&sk(r#"{"id":"p1","x":1,"y":2,"fixed":true}"#, "", ""));
        assert_eq!((s.dof, s.fully_defined), (0, true));
    }

    #[test]
    fn rectangle_without_constraints_is_eight_dof() {
        let s = sketch_status(&sk(RECT_PTS, RECT_ENTS, ""));
        assert_eq!((s.dof, s.state), (8, "under"));
    }

    #[test]
    fn rectangle_with_hv_is_four_dof() {
        let s = sketch_status(&sk(RECT_PTS, RECT_ENTS, HV));
        assert_eq!((s.dof, s.state), (4, "under"));
    }

    #[test]
    fn fully_constrained_rectangle_is_zero_dof() {
        let cons = format!(r#"{HV},{{"type":"fix","id":"c5","refs":["p1"],"value":[0,0]}},{},{}"#, dist("c6", "p1", "p2", 10.0), dist("c7", "p2", "p3", 5.0));
        let s = sketch_status(&sk(RECT_PTS, RECT_ENTS, &cons));
        assert_eq!((s.dof, s.fully_defined, s.state), (0, true, "full"), "{s:?}");
        assert!(s.solved && s.conflicts.is_empty() && s.redundant.is_empty());
    }

    #[test]
    fn over_constrained_triangle_blames_the_last_length() {
        let cons = format!(
            r#"{{"type":"fix","id":"c1","refs":["p1"],"value":[0,0]}},{{"type":"horizontal","id":"c2","refs":["e1"]}},{{"type":"vertical","id":"c3","refs":["e3"]}},{},{},{}"#,
            dist("c4", "p1", "p2", 3.0),
            dist("c5", "p1", "p3", 4.0),
            dist("c6", "p2", "p3", 7.0)
        );
        let s = sketch_status(&sk(
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":3,"y":0},{"id":"p3","x":0,"y":4}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p1"}"#,
            &cons,
        ));
        assert_eq!(s.conflicts, vec!["c6"], "{s:?}");
        assert!(s.redundant.is_empty());
        assert_eq!((s.dof, s.state, s.fully_defined, s.solved), (0, "over", false, false));
    }

    #[test]
    fn duplicate_constraint_is_redundant_not_conflicting() {
        let cons = format!(r#"{{"type":"fix","id":"c1","refs":["p1"],"value":[0,0]}},{{"type":"horizontal","id":"c2","refs":["e1"]}},{{"type":"horizontal","id":"c3","refs":["e1"]}},{}"#, dist("c4", "p1", "p2", 10.0));
        let s = sketch_status(&sk(r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0}"#, r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"}"#, &cons));
        assert_eq!(s.redundant, vec!["c3"], "{s:?}");
        assert!(s.conflicts.is_empty());
        assert_eq!((s.dof, s.state, s.solved, s.fully_defined), (0, "over", true, false));
    }

    #[test]
    fn unsolved_draft_that_converges_is_solved() {
        // rectangle drawn off-axis: H/V constraints do not hold at the drawn coordinates
        let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":1},{"id":"p3","x":11,"y":5},{"id":"p4","x":1,"y":6}"#;
        let s = sketch_status(&sk(pts, RECT_ENTS, HV));
        assert!(s.solved && s.residual > 0.5, "{s:?}");
        assert_eq!(s.dof, 4);
    }

    #[test]
    fn matches_oracle_client_rank_on_cases() {
        // same dof as the ported client analysis (solver::status / ti())
        for (pts, ents, cons) in [
            (RECT_PTS, RECT_ENTS, String::new()),
            (RECT_PTS, RECT_ENTS, HV.to_string()),
            (RECT_PTS, RECT_ENTS, format!("{HV},{}", dist("c5", "p1", "p2", 10.0))),
        ] {
            let k = sk(pts, ents, &cons);
            let a = crate::sketch::solver::status(&k).unwrap();
            let b = sketch_status(&k);
            assert_eq!(a.dof, b.dof);
            assert_eq!(a.status, b.state);
        }
    }

    #[test]
    fn arc_and_circle_unknowns() {
        // circle: centre (2) + radius (1) = 3 dof; a radius constraint removes one
        let s = sketch_status(&sk(r#"{"id":"c","x":0,"y":0}"#, r#"{"type":"circle","id":"e1","c":"c","r":5}"#, ""));
        assert_eq!(s.dof, 3);
        let s = sketch_status(&sk(r#"{"id":"c","x":0,"y":0}"#, r#"{"type":"circle","id":"e1","c":"c","r":5}"#, r#"{"type":"radius","id":"k1","refs":["e1"],"value":5}"#));
        assert_eq!(s.dof, 2, "{s:?}");
    }
}
