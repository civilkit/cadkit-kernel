//! Solve guard: a solve that shrinks a curve to a point is not taken, and the constraint picks that can only
//! do that are named (docs/sketch-status.md "Problems"). Additive: the oracle kernel has no such check.
//!
//! * Shrunk (solidcraft #8988, #9049, #8600): a line longer than [`SHRUNK_BEFORE`] before the solve and
//!   shorter than [`SHRUNK_AFTER`] after it, or a circle / arc / ellipse whose radius goes the same way.
//!   Such a solve satisfies its equations (a zero-length line has no direction left for its parallel,
//!   angle or tangent to hold, so they read as met), yet the sketch the user drew is gone. The regen keeps
//!   the coordinates as sent and the status says `solved: false` with a `sketchCurveShrunk` problem.
//!   Curves already that short before the solve are left alone (no length a dimension asks for is that small).
//! * Picks, named whatever the solve does (they stay in the document; a loaded design still opens):
//!   `sketchSameCurveTwice` (#8577: parallel, perpendicular, collinear, equal, tangent or concentric
//!   with one curve as both refs), `sketchMidpointOwnEnd` (#8562: a line's own end as its midpoint),
//!   `sketchLineEndsCoincident` (#8611: a line's two ends made coincident).

use super::solver::{System, SolveOpts, SolveResult};
use super::{Sketch, SketchEntity};
use ck_json::Json;

/// A curve at least this long (mm) before a solve ...
pub const SHRUNK_BEFORE: f64 = 1e-3;
/// ... and shorter than this after it was shrunk to a point (#9049: 1e-6 missed a squeeze over two solves).
pub const SHRUNK_AFTER: f64 = 1e-5;

/// One problem in the sketch status: a stable `code`, a plain-words `message`, and what it is about.
#[derive(Debug, Clone, PartialEq)]
pub struct Problem {
    pub code: &'static str,
    pub message: String,
    pub constraint: Option<String>,
    pub entity: Option<String>,
}

impl Problem {
    pub fn to_json(&self) -> Json {
        let mut o = vec![("code".into(), Json::Str(self.code.into())), ("message".into(), Json::Str(self.message.clone()))];
        if let Some(c) = &self.constraint {
            o.push(("constraint".into(), Json::Str(c.clone())));
        }
        if let Some(e) = &self.entity {
            o.push(("entity".into(), Json::Str(e.clone())));
        }
        Json::Obj(o)
    }
}

pub const CURVE_SHRUNK: &str = "sketchCurveShrunk";
pub const SAME_CURVE_TWICE: &str = "sketchSameCurveTwice";
pub const MIDPOINT_OWN_END: &str = "sketchMidpointOwnEnd";
pub const LINE_ENDS_COINCIDENT: &str = "sketchLineEndsCoincident";

/// Point position in the unknowns `x` (a fixed point keeps its sketch coordinates).
fn xy(sk: &Sketch, sys: &System, x: &[f64], id: &str) -> Option<[f64; 2]> {
    match sys.key_idx.get(&format!("{id}.x")) {
        Some(&i) => Some([x[i], x[i + 1]]),
        None => sk.point(id).map(|p| [p.x, p.y]),
    }
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

/// The size of each curve at the unknowns `x`: line length, circle / ellipse radius, arc radius.
fn sizes(sk: &Sketch, sys: &System, x: &[f64]) -> Vec<(String, f64)> {
    let pd = |a: &str, b: &str| match (xy(sk, sys, x, a), xy(sk, sys, x, b)) {
        (Some(p), Some(q)) => dist(p, q),
        _ => f64::NAN,
    };
    let rad = |id: &str, r: f64| sys.key_idx.get(&format!("{id}.r")).map_or(r, |&i| x[i]).abs();
    sk.entities
        .iter()
        .filter_map(|e| match e {
            SketchEntity::Line { id, p1, p2, .. } => Some((id.clone(), pd(p1, p2))),
            SketchEntity::Arc { id, c, p1, .. } => Some((id.clone(), pd(c, p1))),
            SketchEntity::Circle { id, r, .. } => Some((id.clone(), rad(id, *r))),
            SketchEntity::Ellipse { id, c, p, r, .. } => Some((id.clone(), rad(id, *r).min(pd(c, p)))),
            _ => None,
        })
        .collect()
}

/// The first curve the step `before -> after` shrank to a point, as a problem.
pub fn shrunk(sk: &Sketch, sys: &System, before: &[f64], after: &[f64]) -> Option<Problem> {
    let (b, a) = (sizes(sk, sys, before), sizes(sk, sys, after));
    b.iter().zip(&a).find(|((_, lb), (_, la))| *lb >= SHRUNK_BEFORE && *la < SHRUNK_AFTER).map(|((id, _), _)| Problem {
        code: CURVE_SHRUNK,
        message: format!("that would shrink {id} to a point: its constraints can only be met without it"),
        constraint: None,
        entity: Some(id.clone()),
    })
}

/// Build and solve, then write the solution back only when it converged and shrank no curve (otherwise the
/// sketch keeps its coordinates). `Err` = the system cannot be built (as `solver::solve_sketch`).
pub fn solve_sketch(sk: &mut Sketch, opts: &SolveOpts) -> Result<(SolveResult, Option<Problem>), String> {
    let sys = super::solver::build(sk)?;
    let mut res = super::solver::solve(&sys, opts);
    let problem = if res.ok { shrunk(sk, &sys, &sys.x, &res.x) } else { None };
    if problem.is_some() {
        res.ok = false;
    }
    if res.ok {
        super::solver::write_back(sk, &sys, &res.x);
    }
    Ok((res, problem))
}

/// Constraint picks that can only shrink a curve or say nothing (see module docs), in document order.
pub fn picks(sk: &Sketch) -> Vec<Problem> {
    let ends = |id: &str| match sk.entity(id) {
        Some(SketchEntity::Line { p1, p2, .. }) => Some((p1.as_str(), p2.as_str())),
        _ => None,
    };
    let mut out = Vec::new();
    for k in sk.constraints.iter().filter(|k| !k.suppressed) {
        let r = |i: usize| k.refs.get(i).map(|s| s.as_str()).unwrap_or("");
        let mut push = |code, message: String, entity: &str| {
            out.push(Problem { code, message, constraint: Some(k.id.clone()), entity: Some(entity.to_string()) })
        };
        match k.kind.as_str() {
            "parallel" | "perpendicular" | "collinear" | "equal" | "tangent" | "concentric" if !r(0).is_empty() && r(0) == r(1) => {
                push(SAME_CURVE_TWICE, format!("{} picks {} twice: a {} needs two different curves", k.id, r(0), k.kind), r(0))
            }
            "midpoint" => {
                if let Some((a, b)) = ends(r(1)) {
                    if r(0) == a || r(0) == b {
                        push(MIDPOINT_OWN_END, format!("{} is an end of {}, so it can't be its midpoint: the line would shrink to a point", r(0), r(1)), r(1));
                    }
                }
            }
            "coincident" => {
                let line = sk.entities.iter().find_map(|e| match e {
                    SketchEntity::Line { id, p1, p2, .. } if (r(0) == p1 && r(1) == p2) || (r(0) == p2 && r(1) == p1) => Some(id.as_str()),
                    _ => None,
                });
                if let Some(l) = line {
                    push(LINE_ENDS_COINCIDENT, format!("{} and {} are the two ends of {}: making them coincident would shrink the line to a point", r(0), r(1), l), l);
                }
            }
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(points: &str, ents: &str, cons: &str) -> Sketch {
        let src = format!(r#"{{"type":"sketch","id":"F1","points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}"#);
        Sketch::from_json(&ck_json::parse(&src).unwrap())
    }

    #[test]
    fn shrunk_needs_a_real_length_before_and_a_hair_after() {
        let s = sk(r#"{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":0}"#, r#"{"type":"line","id":"l1","p1":"a","p2":"b"}"#, "");
        let sys = super::super::solver::build(&s).unwrap();
        let at = |bx: f64| vec![0.0, 0.0, bx, 0.0];
        assert_eq!(shrunk(&s, &sys, &at(1.0), &at(9e-6)).map(|p| p.entity), Some(Some("l1".to_string())));
        assert!(shrunk(&s, &sys, &at(1.0), &at(2e-5)).is_none());
        // already a hair before: left alone
        assert!(shrunk(&s, &sys, &at(5e-4), &at(0.0)).is_none());
    }

    #[test]
    fn a_shrinking_solve_keeps_the_coordinates() {
        let mut s = sk(r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":4}"#, r#"{"type":"line","id":"l1","p1":"a","p2":"b"}"#, r#"{"type":"coincident","id":"k1","refs":["a","b"]}"#);
        let (r, p) = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(!r.ok && p.unwrap().code == CURVE_SHRUNK);
        assert_eq!(s.point_xy("b"), Some([10.0, 4.0]));
        let j = ck_json::to_string(&picks(&s)[0].to_json());
        assert!(j.starts_with(r#"{"code":"sketchLineEndsCoincident","message":"a and b are the two ends of l1"#), "{j}");
    }
}
