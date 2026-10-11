//! Shared helpers of the two sketch solvers (kernel `ck-model/src/sketch/
//! smooth.rs`): the tension residual `ns` (client `Ns`), and the cvspline
//! end finders `yl` / `ji` used by the `tension` and continuity constraints.
//!
//! The kernel pads 2D points to 3D (`z = 0`) before calling into these, so
//! they are dimension-generic; lengths are plain `sqrt(x²+y²(+z²))` — NOT
//! the correctly-rounded `hypot` the 2D distance constraints use.

use super::{Constraint, Sketch, SketchEntity};

/// Tension residual (client `Ns`): `r` = `e + 1` control points from the
/// constrained end, `t` = the target spacing. Returns `e` residuals.
pub fn ns<const D: usize>(r: &[[f64; D]], e: usize, t: f64) -> Vec<f64> {
    let n = [r[1][0] - r[0][0], r[1][1] - r[0][1]];
    let s = (n[0] * n[0] + n[1] * n[1]).sqrt();
    let mut i = vec![s - t];
    if e < 2 {
        return i;
    }
    let o = if 1e-150 < s { [n[0] / s, n[1] / s] } else { n };
    i.push((r[2][0] - r[0][0]) * o[0] + (r[2][1] - r[0][1]) * o[1] - 2.0 * t);
    if e < 3 {
        return i;
    }
    i.push((r[3][0] - r[0][0]) * o[0] + (r[3][1] - r[0][1]) * o[1] - 3.0 * t);
    i
}

/// Tension residual, 3D variant (`f832` / client `Ns` with 3-vectors).
pub fn ns3(r: &[[f64; 3]], e: usize, t: f64) -> Vec<f64> {
    let n = [r[1][0] - r[0][0], r[1][1] - r[0][1], r[1][2] - r[0][2]];
    let s = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let mut i = vec![s - t];
    if e < 2 {
        return i;
    }
    let o = if 1e-150 < s { [n[0] / s, n[1] / s, n[2] / s] } else { n };
    i.push((r[2][0] - r[0][0]) * o[0] + (r[2][1] - r[0][1]) * o[1] + (r[2][2] - r[0][2]) * o[2] - 2.0 * t);
    if e < 3 {
        return i;
    }
    i.push((r[3][0] - r[0][0]) * o[0] + (r[3][1] - r[0][1]) * o[1] + (r[3][2] - r[0][2]) * o[2] - 3.0 * t);
    i
}

fn cvspline_of<'a>(sk: &'a Sketch, id: &str) -> Option<&'a SketchEntity> {
    match sk.entities.iter().rev().find(|e| e.id() == id) {
        Some(e @ SketchEntity::CvSpline { closed: false, .. }) => Some(e),
        _ => None,
    }
}

/// The end of an open cvspline a constraint attaches to (client `yl`):
/// the `at` field when given (must be an end), else the end shared with the
/// other referenced entity (its type/id/point-id strings plus `pts`).
pub fn yl(k: &Constraint, sk: &Sketch) -> Result<String, ()> {
    let spline = cvspline_of(sk, k.refs.first().map(|s| s.as_str()).unwrap_or("")).ok_or(())?;
    let pts = match spline {
        SketchEntity::CvSpline { pts, .. } => pts,
        _ => unreachable!(),
    };
    let ends = [pts.first().cloned().unwrap_or_default(), pts.last().cloned().unwrap_or_default()];
    if let Some(at) = &k.at {
        if at.is_empty() || !ends.contains(at) {
            return Err(());
        }
        return Ok(at.clone());
    }
    let other = k.refs.get(1).ok_or(())?;
    let mut strings: Vec<String> = Vec::new();
    if let Some(e) = sk.entities.iter().rev().find(|e| e.id() == other.as_str()) {
        strings.push(e.type_name().to_string());
        strings.extend(e.point_ids().iter().map(|s| s.to_string()));
    }
    ends.into_iter().find(|e| !e.is_empty() && strings.contains(e)).ok_or(())
}

/// Highest continuity level of the `g1`/`g2`/`g3` constraints on `spline_id`
/// at the end `at` (client `ji`).
pub fn ji(sk: &Sketch, spline_id: &str, at: &str) -> u32 {
    let mut s = 0;
    for k in &sk.constraints {
        if !matches!(k.kind.as_str(), "g1" | "g2" | "g3") {
            continue;
        }
        if k.refs.first().map(|s| s.as_str()) != Some(spline_id) {
            continue;
        }
        let o = match yl(k, sk) {
            Ok(o) => o,
            Err(()) => continue,
        };
        if o == at {
            s = s.max(k.kind.as_bytes().get(1).copied().unwrap_or(b'0').wrapping_sub(b'0') as u32);
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tension_residuals() {
        let r = [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0]];
        assert_eq!(ns(&r, 1, 2.0), vec![0.0]);
        assert_eq!(ns(&r, 2, 2.0), vec![0.0, 0.0]);
        assert_eq!(ns(&r, 3, 2.0), vec![0.0, 0.0, 0.0]);
        assert_eq!(ns(&r, 1, 2.5), vec![-0.5]);
    }
}
