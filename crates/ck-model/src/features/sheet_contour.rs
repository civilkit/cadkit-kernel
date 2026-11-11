//! Sheet metal, stage 2: the contour flange (docs/sheet-metal.md). An open chain of sketch lines is the cross-section of
//! the sheet, extruded by a distance. The material lies to one side of the chain (right of the direction of travel, the
//! left with `flip`), the chain itself is the face of the sheet it is drawn on, and each corner of the chain is a bend of the
//! rule's radius whose virtual sharp is the corner: the setback is `(r + t) tan(a/2)` where the chain is the outer face of the
//! bend (the bend turns towards the material) and `r tan(a/2)` where it is the inner face. Everything is exact and planar
//! (the sketch plane), so the flat pattern, the bend table and the folded section follow analytically.

use super::sheet::{BendRow, FlatPattern, Rule, P};
use ck_geom::Vec2;
use ck_topo::make::{ProfileLoop, ProfileSeg};

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
fn mul(a: P, k: f64) -> P {
    [a[0] * k, a[1] * k]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn len(a: P) -> f64 {
    a[0].hypot(a[1])
}
fn v2(p: P) -> Vec2 {
    Vec2::new(p[0], p[1])
}

/// The chain and how it is extruded.
#[derive(Debug, Clone, PartialEq)]
pub struct Contour {
    pub feature: String,
    /// Vertices of the chain in the plane of its sketch (collinear interior vertices removed).
    pub pts: Vec<P>,
    /// Sketch entity tag of each segment.
    pub tags: Vec<String>,
    /// Material to the right of the direction of travel (`false`: left, `flip`).
    pub right: bool,
    /// Extruded against the sketch normal.
    pub reverse: bool,
    pub distance: f64,
}

/// One bend of the chain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CBend {
    /// Vertex (index into `pts`) the bend rounds.
    pub vertex: usize,
    pub angle: f64,
    pub left_turn: bool,
    /// The chain is the outer face of this bend (the bend turns towards the material).
    pub line_outer: bool,
    pub setback: f64,
    pub allowance: f64,
    /// Radius of the arc on the chain's face.
    pub r_line: f64,
    pub center: P,
    /// Tangent points on the chain's face.
    pub ta: P,
    pub tb: P,
}

/// The planned chain: bends and straight legs.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub bends: Vec<CBend>,
    /// Straight length of every segment.
    pub legs: Vec<f64>,
}

impl Contour {
    /// Unit direction of segment `i`.
    pub fn dir(&self, i: usize) -> P {
        let d = sub(self.pts[i + 1], self.pts[i]);
        mul(d, 1.0 / len(d))
    }

    /// Unit normal of segment `i` towards the material.
    pub fn mat(&self, i: usize) -> P {
        let d = self.dir(i);
        if self.right {
            [d[1], -d[0]]
        } else {
            [-d[1], d[0]]
        }
    }

    pub fn segments(&self) -> usize {
        self.pts.len() - 1
    }

    /// Work out the bends. `Err(detail)` when a segment is too short for its bends or two segments fold back on each other.
    pub fn plan(&self, rule: &Rule) -> Result<Plan, String> {
        let (t, r) = (rule.thickness, rule.bend_radius);
        let n = self.segments();
        let mut bends = vec![];
        for v in 1..n {
            let (d0, d1) = (self.dir(v - 1), self.dir(v));
            let cr = cross(d0, d1);
            let angle = cr.atan2(dot(d0, d1)).abs();
            if angle < 1e-9 {
                continue;
            }
            if angle > 179.5f64.to_radians() {
                return Err(format!("segments {} and {} fold back on each other", v, v + 1));
            }
            let left_turn = cr > 0.0;
            // the centre of the bend is on the side the chain turns to; the material is on `right`
            let line_outer = left_turn != self.right;
            let tn = (angle / 2.0).tan();
            let (r_line, setback) = if line_outer { (r + t, (r + t) * tn) } else { (r, r * tn) };
            let corner = self.pts[v];
            let ta = sub(corner, mul(d0, setback));
            let tb = add(corner, mul(d1, setback));
            let turn_n = if left_turn { [-d0[1], d0[0]] } else { [d0[1], -d0[0]] };
            let center = add(ta, mul(turn_n, r_line));
            bends.push(CBend { vertex: v, angle, left_turn, line_outer, setback, allowance: (r + rule.k_factor * t) * angle, r_line, center, ta, tb });
        }
        let mut legs = vec![];
        for i in 0..n {
            let l = len(sub(self.pts[i + 1], self.pts[i]));
            let s0 = bends.iter().find(|b| b.vertex == i).map_or(0.0, |b| b.setback);
            let s1 = bends.iter().find(|b| b.vertex == i + 1).map_or(0.0, |b| b.setback);
            if l - s0 - s1 < -1e-9 {
                return Err(format!("segment {} is shorter ({l:.4}) than its bends ({:.4})", i + 1, s0 + s1));
            }
            legs.push((l - s0 - s1).max(0.0));
        }
        Ok(Plan { bends, legs })
    }

    /// The closed cross-section of the sheet: the chain's face, the face `t` towards the material, the two end caps.
    pub fn section(&self, rule: &Rule, plan: &Plan) -> ProfileLoop {
        let t = rule.thickness;
        let n = self.segments();
        let off = |p: P, i: usize| add(p, mul(self.mat(i), t));
        let travel = |tag: &str, c: P, from: P, to: P, ccw: bool| {
            if ccw {
                ProfileSeg::arc(tag, v2(c), v2(from), v2(to))
            } else {
                ProfileSeg::arc(tag, v2(c), v2(to), v2(from)).reversed()
            }
        };
        // the chain's face, start to end, and the other face as the same pieces with their ends
        let mut line: Vec<ProfileSeg> = vec![];
        let mut other: Vec<ProfileSeg> = vec![];
        let mut cur = self.pts[0];
        let mut cur_o = off(cur, 0);
        for i in 0..n {
            let b_end = plan.bends.iter().find(|b| b.vertex == i + 1);
            let tag = format!("l{i}");
            let (end, end_o) = match b_end {
                Some(b) => (b.ta, off(b.ta, i)),
                None => (self.pts[i + 1], off(self.pts[i + 1], i)),
            };
            if len(sub(end, cur)) > 1e-9 {
                line.push(ProfileSeg::line(&tag, v2(cur), v2(end)));
                other.push(ProfileSeg::line(&format!("{tag}o"), v2(cur_o), v2(end_o)));
            }
            cur = end;
            cur_o = end_o;
            if let Some(b) = b_end {
                let (tb, tb_o) = (b.tb, off(b.tb, i + 1));
                let ccw = b.left_turn;
                line.push(travel(&format!("b{i}"), b.center, cur, tb, ccw));
                // the other face: same centre, the tangent points moved along the normals of their segments
                other.push(travel(&format!("b{i}o"), b.center, cur_o, tb_o, ccw));
                cur = tb;
                cur_o = tb_o;
            }
        }
        let mut segs = line;
        segs.push(ProfileSeg::line("end", v2(cur), v2(cur_o)));
        segs.extend(other.into_iter().rev().map(|s| s.reversed()));
        segs.push(ProfileSeg::line("start", v2(off(self.pts[0], 0)), v2(self.pts[0])));
        ProfileLoop::new(segs)
    }

    /// Flat pattern: the strip, `x` along the chain from its start, `y` along the extrusion.
    pub fn flat_pattern(&self, rule: &Rule, plan: &Plan) -> FlatPattern {
        // `flip` (material to the left) and `reverse` each mirror the strip: the flat shows it from the other side when
        // exactly one of them is set (observed behaviour of the reference)
        let (y0, y1) = if self.right == self.reverse { (-self.distance, 0.0) } else { (0.0, self.distance) };
        let mut x = 0.0;
        let mut bends = vec![];
        let mut zones = vec![];
        for (i, leg) in plan.legs.iter().enumerate() {
            x += leg;
            if let Some(b) = plan.bends.iter().find(|b| b.vertex == i + 1) {
                bends.push(BendRow {
                    index: bends.len() + 1,
                    up: b.left_turn != self.right,
                    angle: b.angle.to_degrees(),
                    included_angle: 180.0 - b.angle.to_degrees(),
                    radius: rule.bend_radius,
                    allowance: b.allowance,
                    length: self.distance,
                    start: [x + b.allowance * 0.5, y0],
                    end: [x + b.allowance * 0.5, y1],
                    feature: self.feature.clone(),
                    edge: i,
                });
                zones.push([x, x + b.allowance, y0, y1]);
                x += b.allowance;
            }
        }
        let outline = vec![vec![[0.0, y0], [x, y0], [x, y1], [0.0, y1]]];
        FlatPattern::new(rule, outline, bends, zones)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contour(pts: &[P], right: bool) -> Contour {
        Contour { feature: "C1".into(), pts: pts.to_vec(), tags: (0..pts.len() - 1).map(|i| format!("e{}", i + 1)).collect(), right, reverse: false, distance: 40.0 }
    }

    #[test]
    fn setbacks_follow_the_material_side() {
        let mut rule = Rule::new("R", 2.0);
        rule.bend_radius = 2.0;
        // material right of travel, a right turn: the chain is the outer face (setback (r+t) tan 45 = 4)
        let c = contour(&[[0.0, 0.0], [0.0, 20.0], [30.0, 20.0]], true);
        let p = c.plan(&rule).unwrap();
        assert!((p.bends[0].setback - 4.0).abs() < 1e-12 && p.bends[0].line_outer);
        assert!((p.legs[0] - 16.0).abs() < 1e-12 && (p.legs[1] - 26.0).abs() < 1e-12);
        let fp = c.flat_pattern(&rule, &p);
        assert!((fp.size[0] - 46.523893421169305).abs() < 1e-9 && fp.bends[0].up);
        // flipped: the chain is the inner face (setback r tan 45 = 2)
        let c = contour(&[[0.0, 0.0], [0.0, 20.0], [30.0, 20.0]], false);
        let p = c.plan(&rule).unwrap();
        assert!((p.bends[0].setback - 2.0).abs() < 1e-12 && !p.bends[0].line_outer);
        // too short
        let c = contour(&[[0.0, 0.0], [0.0, 3.0], [30.0, 3.0]], true);
        assert!(c.plan(&rule).is_err());
    }
}
