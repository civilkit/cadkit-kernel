//! Measuring a dimension on the solved sketch: the value of a `driven` (reference) dimension, and the shared arc
//! sweep used by the `arclength` residual.
//!
//! `arclength` (one arc, or a circle): radius times swept angle. Arcs run counter-clockwise from `p1` (start) to
//! `p2` (end), the sweep is in `(0, 2 pi]`: an arc whose two ends coincide is the full circle (2 pi), never zero.
//! A circle counts as its circumference.

use super::{Constraint, Sketch, SketchEntity};
use std::f64::consts::TAU;

/// Counter-clockwise sweep in `(0, 2 pi]` from `a` to `b` about `c`; coincident ends give the full circle.
pub fn arc_sweep(c: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (u, v) = ([a[0] - c[0], a[1] - c[1]], [b[0] - c[0], b[1] - c[1]]);
    let mut s = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]);
    if s < 0.0 {
        s += TAU;
    }
    if s < 1e-9 {
        s = TAU;
    }
    s
}

impl Sketch {
    /// The solved value of a dimension (`None` when a ref is missing or of the wrong kind). Angles are degrees in
    /// `[0, 360)` from the first line's direction to the second's (with `s1`/`s2`).
    pub fn measure(&self, k: &Constraint) -> Option<f64> {
        let r = |i: usize| k.refs.get(i).map(|s| s.as_str());
        let pt = |i: usize| self.point_xy(r(i)?);
        let d = |a: [f64; 2], b: [f64; 2]| [b[0] - a[0], b[1] - a[1]];
        let ends = |i: usize| -> Option<[[f64; 2]; 2]> {
            match self.entity(r(i)?)? {
                SketchEntity::Line { p1, p2, .. } => Some([self.point_xy(p1)?, self.point_xy(p2)?]),
                _ => None,
            }
        };
        let radius = |i: usize| -> Option<f64> {
            match self.entity(r(i)?)? {
                SketchEntity::Circle { r, .. } | SketchEntity::Ellipse { r, .. } => Some(*r),
                SketchEntity::Arc { c, p1, .. } => {
                    let v = d(self.point_xy(c)?, self.point_xy(p1)?);
                    Some(v[0].hypot(v[1]))
                }
                _ => None,
            }
        };
        match k.kind.as_str() {
            "distance" => {
                let v = d(pt(0)?, pt(1)?);
                Some(v[0].hypot(v[1]))
            }
            "hdist" => Some((pt(1)?[0] - pt(0)?[0]).abs()),
            "vdist" => Some((pt(1)?[1] - pt(0)?[1]).abs()),
            "pldist" => {
                let (p, [a, b]) = (pt(0)?, ends(1)?);
                let (l, q) = (d(a, b), d(a, p));
                Some(((l[0] * q[1] - l[1] * q[0]) / l[0].hypot(l[1])).abs())
            }
            "radius" => radius(0),
            "diameter" => Some(2.0 * radius(0)?),
            "raddiff" => Some(k.sign * (radius(1)? - radius(0)?)),
            "angle" => {
                let (a, b) = (ends(0)?, ends(1)?);
                let (u, v) = (d(a[0], a[1]), d(b[0], b[1]));
                let (u, v) = ([u[0] * k.s1, u[1] * k.s1], [v[0] * k.s2, v[1] * k.s2]);
                let t = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).to_degrees();
                Some(if t < 0.0 { t + 360.0 } else { t })
            }
            "arclength" => match self.entity(r(0)?)? {
                SketchEntity::Arc { c, p1, p2, .. } => {
                    let (c, a, b) = (self.point_xy(c)?, self.point_xy(p1)?, self.point_xy(p2)?);
                    let v = d(c, a);
                    Some(v[0].hypot(v[1]) * arc_sweep(c, a, b))
                }
                SketchEntity::Circle { r, .. } => Some(TAU * r),
                _ => None,
            },
            _ => None,
        }
    }
}
