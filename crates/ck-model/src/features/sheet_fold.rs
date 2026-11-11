//! Sheet metal, stage 2: folds across the base plate (docs/sheet-metal.md). The pure 2D part: the fold lines and their bend zones,
//! the panels the base falls into, which panel hangs on which through which bend, and the flat pattern rows.
//!
//! A fold is a line in the plane of the base. Its bend zone is a strip of the width of the bend allowance, which sits on the line
//! according to `position`; the flat pattern does not change size (the zone is part of the sheet). The side of the root panel
//! stays, the other side turns about the bend. `nu` points from the root side to the moving side, `u(p) = (p - o) . nu`
//! is the signed distance from the line and the zone is `u` in `[z0, z1]`.
//!
//! Limits (reported as `sheetMetalUnsupported`): the outline is straight and convex, the zone meets the outline at right angles
//! (the strip cuts the same chord at both of its edges), zones of different folds do not overlap, holes do not meet a zone.

use super::sheet::{BendRow, FlatPattern, Sheet, P};

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
fn mul(a: P, k: f64) -> P {
    [a[0] * k, a[1] * k]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FoldPos {
    Centerline,
    Start,
    End,
    Mould,
}

impl FoldPos {
    pub fn parse(s: &str) -> Option<FoldPos> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "centerline" | "centreline" | "center" | "centre" => FoldPos::Centerline,
            "start" => FoldPos::Start,
            "end" => FoldPos::End,
            "mould" | "mold" => FoldPos::Mould,
            _ => return None,
        })
    }
    pub fn as_str(self) -> &'static str {
        match self {
            FoldPos::Centerline => "centerline",
            FoldPos::Start => "start",
            FoldPos::End => "end",
            FoldPos::Mould => "mould",
        }
    }
}

/// A fold across the base.
#[derive(Debug, Clone, PartialEq)]
pub struct Fold {
    pub feature: String,
    /// A point on the fold line and its unit direction.
    pub o: P,
    pub d: P,
    /// Unit normal from the root side to the moving side.
    pub nu: P,
    pub angle: f64,
    pub radius: f64,
    /// The moving side turns towards the cap face of the base.
    pub up: bool,
    pub position: FoldPos,
    /// The zone in `u`.
    pub z: [f64; 2],
    pub allowance: f64,
    /// `(r + t) tan(a/2)`.
    pub setback: f64,
}

impl Fold {
    pub fn u(&self, p: P) -> f64 {
        dot(sub(p, self.o), self.nu)
    }
    pub fn v(&self, p: P) -> f64 {
        dot(sub(p, self.o), self.d)
    }
}

/// The zone limits `[z0, z1]` for a position: the allowance wide, the line on its centre, at the root-side edge (`start`),
/// at the far edge (`end`), or with the root-side edge a setback before the line (`mould`: the line is the outer virtual sharp).
pub fn zone_of(position: FoldPos, allowance: f64, setback: f64) -> [f64; 2] {
    let z0 = match position {
        FoldPos::Centerline => -allowance * 0.5,
        FoldPos::Start => 0.0,
        FoldPos::End => -allowance,
        FoldPos::Mould => -setback,
    };
    [z0, z0 + allowance]
}

type Poly = Vec<(P, String)>;

/// Clip a polygon (vertices with the tag of the edge that leaves them) by `keep(p) >= 0`; new edges along the clip line get `tag`.
fn clip(poly: &Poly, keep: &dyn Fn(P) -> f64, tag: &str) -> Poly {
    let n = poly.len();
    let mut out: Poly = vec![];
    for i in 0..n {
        let (a, ta) = (&poly[i].0, &poly[i].1);
        let b = poly[(i + 1) % n].0;
        let (da, db) = (keep(*a), keep(b));
        let (ia, ib) = (da >= -1e-9, db >= -1e-9);
        let cut = || {
            let s = da / (da - db);
            add(*a, mul(sub(b, *a), s))
        };
        match (ia, ib) {
            (true, true) => out.push((*a, ta.clone())),
            (true, false) => {
                out.push((*a, ta.clone()));
                out.push((cut(), tag.to_string()));
            }
            (false, true) => out.push((cut(), ta.clone())),
            (false, false) => {}
        }
    }
    // drop repeated vertices
    let mut v: Poly = vec![];
    for q in out {
        if v.last().map_or(true, |l| (l.0[0] - q.0[0]).hypot(l.0[1] - q.0[1]) > 1e-9) {
            v.push(q);
        }
    }
    while v.len() > 1 && (v[0].0[0] - v[v.len() - 1].0[0]).hypot(v[0].0[1] - v[v.len() - 1].0[1]) <= 1e-9 {
        v.pop();
    }
    v
}

fn area(p: &Poly) -> f64 {
    let n = p.len();
    (0..n).map(|i| p[i].0[0] * p[(i + 1) % n].0[1] - p[i].0[1] * p[(i + 1) % n].0[0]).sum::<f64>() * 0.5
}

/// The part of the outline on one side of a line (`u >= 0` for `positive`, `u <= 0` otherwise, `u` the distance along `n` from
/// `o`): its corners, area and centroid.
pub fn half(outline: &[P], o: P, n: P, positive: bool) -> (Vec<P>, f64, P) {
    let poly: Poly = outline.iter().map(|p| (*p, String::new())).collect();
    let sg = if positive { 1.0 } else { -1.0 };
    let c = clip(&poly, &|p| sg * dot(sub(p, o), n), "");
    let a = area(&c);
    if c.len() < 3 || a.abs() < 1e-12 {
        return (vec![], 0.0, o);
    }
    let m = c.len();
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..m {
        let (p, q) = (c[i].0, c[(i + 1) % m].0);
        let w = p[0] * q[1] - q[0] * p[1];
        cx += (p[0] + q[0]) * w;
        cy += (p[1] + q[1]) * w;
    }
    (c.iter().map(|x| x.0).collect(), a.abs(), [cx / (6.0 * a), cy / (6.0 * a)])
}

/// A piece of the base between zones.
#[derive(Debug, Clone)]
pub struct Cell {
    /// Corners (CCW) with the tag of the edge leaving each: a sketch entity of the outline, or `z<k>a` / `z<k>b` for the
    /// root-side / moving-side edge of the zone of fold `k` (0-based).
    pub poly: Vec<(P, String)>,
    /// Per fold: beyond its zone (the moving side).
    pub signs: Vec<bool>,
}

/// Panels of the base: cell 0 holds the root point; `parent[i] = (cell, fold)` it hangs on; `order` is a walk from the root.
#[derive(Debug, Clone)]
pub struct Layout {
    pub cells: Vec<Cell>,
    pub parent: Vec<Option<(usize, usize)>>,
}

/// Cut the (convex, CCW) outline by the zones of the folds. `Err(detail)` for the unsupported cases.
pub fn layout(sheet: &Sheet, root: P) -> Result<Layout, String> {
    let outline: Poly = (0..sheet.outline.len()).map(|i| (sheet.outline[i], sheet.tags[i].clone())).collect();
    let mut cells = vec![Cell { poly: outline, signs: vec![] }];
    for (k, f) in sheet.folds.iter().enumerate() {
        let mut next = vec![];
        for c in cells {
            for beyond in [false, true] {
                let keep: Box<dyn Fn(P) -> f64> = if beyond { Box::new(move |p| f.u(p) - f.z[1]) } else { Box::new(move |p| f.z[0] - f.u(p)) };
                let tag = format!("z{k}{}", if beyond { "b" } else { "a" });
                let poly = clip(&c.poly, &*keep, &tag);
                if poly.len() >= 3 && area(&poly).abs() > 1e-9 {
                    let mut signs = c.signs.clone();
                    signs.push(beyond);
                    next.push(Cell { poly, signs });
                }
            }
        }
        cells = next;
    }
    // the root cell first, then breadth first over the folds
    let in_cell = |c: &Cell, p: P| c.poly.iter().enumerate().all(|(i, (a, _))| {
        let b = c.poly[(i + 1) % c.poly.len()].0;
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -1e-9
    });
    let root_i = cells.iter().position(|c| in_cell(c, root)).ok_or("the fixed side of the first fold lies inside a bend zone")?;
    let mut order = vec![root_i];
    let mut parent_of: Vec<Option<(usize, usize)>> = vec![None; cells.len()];
    let mut head = 0;
    while head < order.len() {
        let a = order[head];
        head += 1;
        for (b, cb) in cells.iter().enumerate() {
            if order.contains(&b) {
                continue;
            }
            let diff: Vec<usize> = (0..sheet.folds.len()).filter(|&k| cb.signs[k] != cells[a].signs[k]).collect();
            if diff.len() == 1 && cb.signs[diff[0]] {
                parent_of[b] = Some((a, diff[0]));
                order.push(b);
            }
        }
    }
    if order.len() != cells.len() {
        return Err("the folds cut the base into pieces that do not hang together as a chain of bends".into());
    }
    // renumber: root = 0, in walk order
    let mut idx = vec![0; cells.len()];
    for (new, &old) in order.iter().enumerate() {
        idx[old] = new;
    }
    let ncells: Vec<Cell> = order.iter().map(|&o| cells[o].clone()).collect();
    let parent: Vec<Option<(usize, usize)>> = order.iter().map(|&o| parent_of[o].map(|(p, k)| (idx[p], k))).collect();
    Ok(Layout { cells: ncells, parent })
}

/// The chord of the outline in the zone of fold `k`: `[v0, v1]` along the line, equal at both zone edges.
pub fn chord(sheet: &Sheet, k: usize) -> Result<[f64; 2], String> {
    let f = &sheet.folds[k];
    let outline: Poly = (0..sheet.outline.len()).map(|i| (sheet.outline[i], String::new())).collect();
    let strip = clip(&clip(&outline, &|p| f.u(p) - f.z[0], ""), &|p| f.z[1] - f.u(p), "");
    if strip.len() < 3 || area(&strip).abs() < 1e-9 {
        return Err(format!("the fold line of {} does not cross the base", f.feature));
    }
    let mut ends = [[f64::MAX, f64::MIN]; 2];
    for (p, _) in &strip {
        for (e, z) in f.z.iter().enumerate() {
            if (f.u(*p) - z).abs() < 1e-7 {
                let v = f.v(*p);
                ends[e][0] = ends[e][0].min(v);
                ends[e][1] = ends[e][1].max(v);
            }
        }
    }
    if ends.iter().any(|e| e[0] > e[1]) || (ends[0][0] - ends[1][0]).abs() > 1e-6 || (ends[0][1] - ends[1][1]).abs() > 1e-6 {
        return Err(format!("the fold line of {} meets the outline at an angle: a bend zone needs to end at right angles (fold square to the edges it crosses)", f.feature));
    }
    Ok(ends[0])
}

/// Bounding box `[xmin, xmax, ymin, ymax]` of the zone of fold `k` within the outline.
pub fn zone_box(sheet: &Sheet, k: usize) -> [f64; 4] {
    let f = &sheet.folds[k];
    let outline: Poly = (0..sheet.outline.len()).map(|i| (sheet.outline[i], String::new())).collect();
    let strip = clip(&clip(&outline, &|p| f.u(p) - f.z[0], ""), &|p| f.z[1] - f.u(p), "");
    let mut b = [f64::MAX, f64::MIN, f64::MAX, f64::MIN];
    for (p, _) in &strip {
        b = [b[0].min(p[0]), b[1].max(p[0]), b[2].min(p[1]), b[3].max(p[1])];
    }
    b
}

/// Check a new fold (the last of `sheet.folds`) against the others and the holes. `Err(detail)` when unsupported.
pub fn check(sheet: &Sheet) -> Result<(), String> {
    if !sheet.straight() {
        return Err("the base outline has curved edges".into());
    }
    let n = sheet.outline.len();
    for i in 0..n {
        let (a, b, c) = (sheet.outline[i], sheet.outline[(i + 1) % n], sheet.outline[(i + 2) % n]);
        if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) < -1e-9 {
            return Err("the base outline is not convex".into());
        }
    }
    for k in 0..sheet.folds.len() {
        chord(sheet, k)?;
    }
    let outline: Poly = (0..n).map(|i| (sheet.outline[i], String::new())).collect();
    for a in 0..sheet.folds.len() {
        for b in a + 1..sheet.folds.len() {
            let (fa, fb) = (&sheet.folds[a], &sheet.folds[b]);
            let s = clip(&clip(&clip(&clip(&outline, &|p| fa.u(p) - fa.z[0], ""), &|p| fa.z[1] - fa.u(p), ""), &|p| fb.u(p) - fb.z[0], ""), &|p| fb.z[1] - fb.u(p), "");
            if s.len() >= 3 && area(&s).abs() > 1e-6 {
                return Err(format!("the bend zones of {} and {} overlap", fa.feature, fb.feature));
            }
        }
    }
    for h in &sheet.holes {
        for f in &sheet.folds {
            let (lo, hi) = h.iter().fold((f64::MAX, f64::MIN), |(l, hi), p| (l.min(f.u(*p)), hi.max(f.u(*p))));
            if hi > f.z[0] - 1e-9 && lo < f.z[1] + 1e-9 {
                return Err(format!("a hole of the base meets the bend zone of {}", f.feature));
            }
        }
    }
    Ok(())
}

impl Sheet {
    /// The flat pattern of a folded base: the outline as sketched, a bend row for every fold.
    pub fn fold_flat_pattern(&self) -> FlatPattern {
        let mut loops = vec![super::sheet::clean(self.outline.clone())];
        loops.extend(self.holes.iter().cloned());
        let mut bends = vec![];
        let mut zones = vec![];
        for (k, f) in self.folds.iter().enumerate() {
            let [v0, v1] = chord(self, k).unwrap_or([0.0, 0.0]);
            let um = (f.z[0] + f.z[1]) * 0.5;
            let at = |v: f64| add(add(f.o, mul(f.nu, um)), mul(f.d, v));
            bends.push(BendRow {
                index: k + 1,
                up: f.up,
                angle: f.angle.to_degrees(),
                included_angle: 180.0 - f.angle.to_degrees(),
                radius: f.radius,
                allowance: f.allowance,
                length: v1 - v0,
                start: at(v0),
                end: at(v1),
                feature: f.feature.clone(),
                edge: 0,
            });
            zones.push(zone_box(self, k));
        }
        FlatPattern::new(&self.rule, loops, bends, zones)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zones_follow_the_position() {
        let (ba, sb) = (3.015928947446201, 2.309401076758503);
        assert_eq!(zone_of(FoldPos::Start, ba, sb), [0.0, ba]);
        assert_eq!(zone_of(FoldPos::End, ba, sb), [-ba, 0.0]);
        let c = zone_of(FoldPos::Centerline, ba, sb);
        assert!((c[0] + c[1]).abs() < 1e-12 && (c[1] - c[0] - ba).abs() < 1e-12);
        let m = zone_of(FoldPos::Mould, ba, sb);
        assert_eq!(m[0], -sb);
        assert!((m[1] - (ba - sb)).abs() < 1e-12);
        assert!(FoldPos::parse("Mould").is_some() && FoldPos::parse("middle").is_none());
    }

    #[test]
    fn halves_of_a_rectangle() {
        let r = [[0.0, 0.0], [100.0, 0.0], [100.0, 60.0], [0.0, 60.0]];
        let (_, a, c) = half(&r, [40.0, 0.0], [1.0, 0.0], true);
        assert!((a - 3600.0).abs() < 1e-9 && (c[0] - 70.0).abs() < 1e-9 && (c[1] - 30.0).abs() < 1e-9);
        let (_, a, _) = half(&r, [40.0, 0.0], [1.0, 0.0], false);
        assert!((a - 2400.0).abs() < 1e-9);
    }
}
