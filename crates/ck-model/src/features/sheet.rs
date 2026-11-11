//! Sheet metal, stage 1: the sheet model, the bend arithmetic and the flat pattern (docs/sheet-metal.md).
//!
//! The model is plain data in the plane of the base sketch: the outline of the base flange (a CCW polygon), the sheet rule that
//! was active when it was made, and the edge flanges on its edges. Everything the flat pattern needs follows from it
//! analytically, so the unfolded size, the outline loops, the bend lines and the bend table are exact (no mesh, no solver).
//! `sheet_features.rs` builds the folded solid from the same model.
//!
//! Conventions (edge `i` of the outline runs from `A_i` to `A_i+1`; the material lies to its left):
//! * `d_i` is the unit direction of the edge, `m_i = (-d_i.y, d_i.x)` the unit inward normal.
//! * A flange has a bend of inside radius `r` and angle `a` (0 < a < 180, the angle the leg turns through), sheet thickness `t`.
//!   The outer virtual sharp sits `S = (r + t) tan(a/2)` before the first tangent line of the bend. `b` is how far the straight
//!   part of the base stops short of the edge: `S` for `position: inside` (the flange's outer face stays within the outline),
//!   `0` for `outside` (the outline edge is where the bend starts), `S/2` for `middle`.
//! * The bend allowance is `BA = (r + k t) a`. In the flat pattern the bend zone covers `s` in `[b - BA, b]` (`s` = distance
//!   inward from the outline edge, so the flange runs to negative `s`) and the leg `[b - BA - L, b - BA]`; the bend line is the
//!   middle of the zone.
//! * The height is measured from the outer virtual sharp (`height_from: outer`, default) or the inner one (`inner`), along the
//!   leg: `L = height - S` or `height - r tan(a/2)`. `tangent`: `L = height - (r + t) min(1, tan(a/2))` (measured from the
//!   plane through the tangent line of the bend; observed behaviour, equal to `outer` up to 90 degrees).

use ck_json::{Json, Obj};

pub type P = [f64; 2];

pub const EPS: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReliefShape {
    Notch,
    Round,
    Rectangular,
    Tear,
}

impl ReliefShape {
    pub fn parse(s: &str) -> Option<ReliefShape> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "notch" => ReliefShape::Notch,
            "round" => ReliefShape::Round,
            "rectangular" | "rectangle" | "rect" => ReliefShape::Rectangular,
            "tear" => ReliefShape::Tear,
            _ => return None,
        })
    }
    pub fn as_str(self) -> &'static str {
        match self {
            ReliefShape::Notch => "notch",
            ReliefShape::Round => "round",
            ReliefShape::Rectangular => "rectangular",
            ReliefShape::Tear => "tear",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightFrom {
    Outer,
    Inner,
    Tangent,
}

impl HeightFrom {
    pub fn parse(s: &str) -> Option<HeightFrom> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "outer" | "outside" => HeightFrom::Outer,
            "inner" | "inside" => HeightFrom::Inner,
            "tangent" => HeightFrom::Tangent,
            _ => return None,
        })
    }
    pub fn as_str(self) -> &'static str {
        match self {
            HeightFrom::Outer => "outer",
            HeightFrom::Inner => "inner",
            HeightFrom::Tangent => "tangent",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Position {
    Inside,
    Outside,
    Middle,
}

impl Position {
    pub fn parse(s: &str) -> Option<Position> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "inside" | "material_inside" => Position::Inside,
            "outside" | "material_outside" => Position::Outside,
            "middle" | "mid" => Position::Middle,
            _ => return None,
        })
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Position::Inside => "inside",
            Position::Outside => "outside",
            Position::Middle => "middle",
        }
    }
}

/// A sheet rule: the stock and the bend and relief defaults. Lengths in mm.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    pub name: String,
    pub thickness: f64,
    pub k_factor: f64,
    pub bend_radius: f64,
    pub relief_width: f64,
    pub relief_depth: f64,
    pub corner_relief: f64,
    pub corner_relief_shape: ReliefShape,
    pub hem_gap: f64,
    pub gap: f64,
}

/// Thickness of a rule that names none.
pub const DEFAULT_THICKNESS: f64 = 2.5;
/// K-factor of a rule that names none.
pub const DEFAULT_K: f64 = 0.44;

impl Rule {
    /// The defaults for a stock of thickness `t`: bend radius `t`, relief width `t`, relief depth `t/2`, corner relief `4t`,
    /// gap `t`, hem gap 0.01, k-factor 0.44, notch corner.
    pub fn new(name: &str, t: f64) -> Rule {
        Rule {
            name: name.into(),
            thickness: t,
            k_factor: DEFAULT_K,
            bend_radius: t,
            relief_width: t,
            relief_depth: t * 0.5,
            corner_relief: t * 4.0,
            corner_relief_shape: ReliefShape::Notch,
            hem_gap: 0.01,
            gap: t,
        }
    }

    pub fn to_json(&self) -> Json {
        Obj::new()
            .put("name", self.name.as_str())
            .put("thickness", self.thickness)
            .put("kFactor", self.k_factor)
            .put("bendRadius", self.bend_radius)
            .put("reliefWidth", self.relief_width)
            .put("reliefDepth", self.relief_depth)
            .put("cornerRelief", self.corner_relief)
            .put("cornerReliefShape", self.corner_relief_shape.as_str())
            .put("hemGap", self.hem_gap)
            .put("gap", self.gap)
            .build()
    }
}

/// An edge flange on edge `edge` of the base outline.
#[derive(Debug, Clone, PartialEq)]
pub struct Flange {
    /// The `edgeFlange` feature that made it.
    pub feature: String,
    pub edge: usize,
    pub height: f64,
    /// The angle the leg turns through, radians.
    pub angle: f64,
    /// Inside bend radius.
    pub radius: f64,
    pub height_from: HeightFrom,
    pub position: Position,
    /// The flange bends towards the cap face of the base (its `N` side); `false`: towards the face it started from.
    pub up: bool,
    /// Flanges of one feature share a batch (their neighbouring legs are trimmed by the rule's gap).
    pub batch: usize,
    /// Extent of the leg along the edge, measured from the start of the edge. Fixed when the flange is made: a flange
    /// added later next to it trims the bend zone but not the leg.
    pub leg: [f64; 2],
    /// A hem: a 180 degree bend of inside radius `radius` (the hem gap), `height` is the length to the outer face of the hem.
    pub hem: bool,
    /// Further bends on the far edge of the leg (at most [`MAX_THEN`]), each its own angle and height, the radius, height
    /// reference and position of the flange.
    pub then: Vec<Next>,
}

/// Most bends a flange may chain after its first.
pub const MAX_THEN: usize = 16;

/// A further bend of a flange: `flip` turns it the other way from the flange's own bend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Next {
    pub angle: f64,
    pub height: f64,
    pub flip: bool,
}

/// The arithmetic of one bend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bend {
    /// `(r + t) tan(a/2)`: from the outer virtual sharp to the tangent line.
    pub setback: f64,
    /// Where the straight part of the base stops, from the outline edge.
    pub b: f64,
    pub allowance: f64,
    /// Length of the straight leg.
    pub leg: f64,
}

/// Bend arithmetic for a flange of the given height. `Err(minimum height)` when the height does not reach the end of the bend.
pub fn bend_of(t: f64, k: f64, r: f64, angle: f64, height: f64, from: HeightFrom, pos: Position) -> Result<Bend, f64> {
    let tn = (angle / 2.0).tan();
    let setback = (r + t) * tn;
    let reach = match from {
        HeightFrom::Outer => setback,
        HeightFrom::Inner => r * tn,
        HeightFrom::Tangent => (r + t) * tn.min(1.0),
    };
    let leg = height - reach;
    if leg < -1e-9 {
        return Err(reach);
    }
    let b = match pos {
        Position::Inside => setback,
        Position::Middle => setback * 0.5,
        Position::Outside => 0.0,
    };
    Ok(Bend { setback, b, allowance: (r + k * t) * angle, leg: leg.max(0.0) })
}

/// Offset of the start of a bend from the edge it hangs on, for the flange's position setting.
pub fn offset_b(setback: f64, pos: Position) -> f64 {
    match pos {
        Position::Inside => setback,
        Position::Middle => setback * 0.5,
        Position::Outside => 0.0,
    }
}

fn reach_of(t: f64, r: f64, angle: f64, from: HeightFrom) -> f64 {
    let tn = (angle / 2.0).tan();
    match from {
        HeightFrom::Outer => (r + t) * tn,
        HeightFrom::Inner => r * tn,
        HeightFrom::Tangent => (r + t) * tn.min(1.0),
    }
}

/// The bends of a flange and of the bends chained after it: for bend `j` the setback, the offset `b` of its start from the
/// edge it hangs on (the first: from the outline edge), the allowance and the straight leg after it. The leg of bend `j` is
/// `height_j - reach_j - b_{j+1}`: the next bend starts `b_{j+1}` before the far edge of the leg, the last leg runs to the
/// end (observed behaviour of the reference). A hem has one bend of 180 degrees, `height` the length to its outer face.
/// `Err((j, minimum height))` when a height does not reach the end of its bend.
#[allow(clippy::too_many_arguments)]
pub fn chain_of(t: f64, k: f64, r: f64, angle: f64, height: f64, from: HeightFrom, pos: Position, hem: bool, then: &[Next]) -> Result<Vec<Bend>, (usize, f64)> {
    if hem {
        let reach = r + t;
        if height < reach - 1e-9 {
            return Err((0, reach));
        }
        return Ok(vec![Bend { setback: reach, b: 0.0, allowance: (r + k * t) * std::f64::consts::PI, leg: (height - reach).max(0.0) }]);
    }
    let n = 1 + then.len();
    let ang = |j: usize| if j == 0 { angle } else { then[j - 1].angle };
    let hgt = |j: usize| if j == 0 { height } else { then[j - 1].height };
    let setback = |j: usize| (r + t) * (ang(j) / 2.0).tan();
    let mut out = vec![];
    for j in 0..n {
        let next_b = if j + 1 < n { offset_b(setback(j + 1), pos) } else { 0.0 };
        let reach = reach_of(t, r, ang(j), from);
        let leg = hgt(j) - reach - next_b;
        if leg < -1e-9 {
            return Err((j, reach + next_b));
        }
        out.push(Bend { setback: setback(j), b: offset_b(setback(j), pos), allowance: (r + k * t) * ang(j), leg: leg.max(0.0) });
    }
    Ok(out)
}

/// The base flange and what is attached to it.
#[derive(Debug, Clone, PartialEq)]
pub struct Sheet {
    pub base: String,
    pub rule: Rule,
    /// CCW outline of the base in the plane of its sketch.
    pub outline: Vec<P>,
    /// Sketch entity tag of each outline edge.
    pub tags: Vec<String>,
    /// Outline edge `i` is a straight sketch line (a flange needs that); the others are curve samples.
    pub line: Vec<bool>,
    /// Holes of the base, as polylines (copied to the flat pattern).
    pub holes: Vec<Vec<P>>,
    pub flanges: Vec<Flange>,
    /// Folds across the base (exclusive with flanges in stage 2, `sheet_fold.rs`).
    pub folds: Vec<super::sheet_fold::Fold>,
    /// A point inside the panel that stays put when the base is folded.
    pub fold_root: Option<P>,
}

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
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn len(a: P) -> f64 {
    a[0].hypot(a[1])
}

/// Signed area of a polygon (CCW > 0).
pub fn area(p: &[P]) -> f64 {
    let n = p.len();
    (0..n).map(|i| cross(p[i], p[(i + 1) % n])).sum::<f64>() * 0.5
}

/// Frame of one outline edge.
#[derive(Debug, Clone, Copy)]
pub struct EdgeFrame {
    pub a: P,
    pub b: P,
    pub d: P,
    pub m: P,
    pub len: f64,
}

impl Sheet {
    pub fn thickness(&self) -> f64 {
        self.rule.thickness
    }

    /// Every outline edge is a straight line.
    pub fn straight(&self) -> bool {
        self.line.iter().all(|l| *l)
    }

    pub fn n_edges(&self) -> usize {
        self.outline.len()
    }

    pub fn edge(&self, i: usize) -> EdgeFrame {
        let n = self.outline.len();
        let (a, b) = (self.outline[i], self.outline[(i + 1) % n]);
        let l = len(sub(b, a));
        let d = mul(sub(b, a), 1.0 / l);
        EdgeFrame { a, b, d, m: [-d[1], d[0]], len: l }
    }

    pub fn flange_on(&self, edge: usize) -> Option<&Flange> {
        self.flanges.iter().find(|f| f.edge == edge)
    }

    /// The bend of a flange (heights were checked when it was made).
    pub fn bend(&self, f: &Flange) -> Bend {
        self.chain(f).first().copied().unwrap_or(Bend { setback: 0.0, b: 0.0, allowance: 0.0, leg: 0.0 })
    }

    /// Every bend of the flange: its own, then the chained ones.
    pub fn chain(&self, f: &Flange) -> Vec<Bend> {
        chain_of(self.rule.thickness, self.rule.k_factor, f.radius, f.angle, f.height, f.height_from, f.position, f.hem, &f.then).unwrap_or_default()
    }

    /// Where the straight part of the base stops short of edge `i` (0 for an edge without a flange).
    pub fn inset(&self, i: usize) -> f64 {
        self.flange_on(i).map_or(0.0, |f| self.bend(f).b)
    }

    /// The outline of the straight part of the base: every flanged edge moved inward by its `b`, neighbours cut by the
    /// intersections of the moved lines. `q[i]` is the corner at the start of edge `i`.
    pub fn straight_part(&self) -> Vec<P> {
        let n = self.n_edges();
        let line = |i: usize| {
            let e = self.edge(i);
            (add(e.a, mul(e.m, self.inset(i))), e.d)
        };
        (0..n)
            .map(|k| {
                let (p0, d0) = line((k + n - 1) % n);
                let (p1, d1) = line(k);
                let den = cross(d0, d1);
                if den.abs() < 1e-9 {
                    return p1;
                }
                // p0 + d0 s = p1 + d1 u
                let s = cross(sub(p1, p0), d1) / den;
                add(p0, mul(d0, s))
            })
            .collect()
    }

    /// Extent `[x0, x1]` along edge `i` (from its start) of the straight part of the base, i.e. of the bend zone.
    pub fn zone(&self, i: usize, q: &[P]) -> [f64; 2] {
        let e = self.edge(i);
        let n = self.n_edges();
        [dot(sub(q[i], e.a), e.d), dot(sub(q[(i + 1) % n], e.a), e.d)]
    }

    /// Leg extent of a new flange on edge `i`: the zone, with the ends beside a flange of the same batch
    /// (at right angles, both bends 90 degrees) pulled back so that the two legs stay `gap` apart.
    pub fn leg_extent(&self, i: usize, q: &[P]) -> [f64; 2] {
        let me = self.flange_on(i).expect("flange");
        let n = self.n_edges();
        let mut z = self.zone(i, q);
        let e = self.edge(i);
        let right = |a: f64| (a - std::f64::consts::FRAC_PI_2).abs() < 1e-6;
        let trim = |j: usize| -> Option<f64> {
            let o = self.flange_on(j)?;
            let ej = self.edge(j);
            if o.batch != me.batch || !right(o.angle) || !right(me.angle) || dot(e.d, ej.d).abs() > 1e-6 {
                return None;
            }
            // the neighbouring wall's inner face is `b - r` from its edge, the legs keep `gap` between their inner corners
            Some(self.bend(o).b - o.radius + self.rule.gap / std::f64::consts::SQRT_2)
        };
        if let Some(c) = trim((i + n - 1) % n) {
            z[0] = c.min(z[1]);
        }
        if let Some(c) = trim((i + 1) % n) {
            z[1] = (e.len - c).max(z[0]);
        }
        z
    }

    /// Fix the leg extents of the flanges that have none yet (`leg` all zero) once the batch is in.
    pub fn settle_batch(&mut self, batch: usize) {
        let q = self.straight_part();
        let idx: Vec<usize> = (0..self.flanges.len()).filter(|&k| self.flanges[k].batch == batch).collect();
        for k in idx {
            let e = self.flanges[k].edge;
            self.flanges[k].leg = self.leg_extent(e, &q);
        }
    }

    /// The flat pattern.
    pub fn flat_pattern(&self) -> FlatPattern {
        if !self.folds.is_empty() {
            return self.fold_flat_pattern();
        }
        let n = self.n_edges();
        let q = self.straight_part();
        let mut pts: Vec<P> = vec![];
        for i in 0..n {
            let e = self.edge(i);
            let at = |x: f64, s: f64| add(add(e.a, mul(e.d, x)), mul(e.m, s));
            match self.flange_on(i) {
                None => pts.push(q[i]),
                Some(f) => {
                    let ch = self.chain(f);
                    if ch.is_empty() {
                        pts.push(q[i]);
                        continue;
                    }
                    let bd = ch[0];
                    let [x0, x1] = self.zone(i, &q);
                    let [l0, l1] = f.leg;
                    let s1 = bd.b - bd.allowance;
                    let s2 = chain_end(&ch);
                    pts.extend([at(x0, bd.b), at(x0, s1), at(l0, s1), at(l0, s2), at(l1, s2), at(l1, s1), at(x1, s1), at(x1, bd.b)]);
                }
            }
        }
        let mut loops = vec![clean(pts)];
        loops.extend(self.holes.iter().cloned());
        let mut bends = vec![];
        let mut zones = vec![];
        for f in self.flanges.iter() {
            let ch = self.chain(f);
            if ch.is_empty() {
                continue;
            }
            let e = self.edge(f.edge);
            let [x0, x1] = self.zone(f.edge, &q);
            let at = |x: f64, s: f64| add(add(e.a, mul(e.d, x)), mul(e.m, s));
            // `hi` is the side of the zone nearest the edge: the first bend's `b`, then where the leg before ended
            let mut hi = ch[0].b;
            for (j, bd) in ch.iter().enumerate() {
                let (xa, xb) = if j == 0 { (x0, x1) } else { (f.leg[0], f.leg[1]) };
                let lo = hi - bd.allowance;
                let sl = hi - bd.allowance * 0.5;
                let (c0, c1) = (at(xa, lo), at(xb, hi));
                zones.push([c0[0].min(c1[0]), c0[0].max(c1[0]), c0[1].min(c1[1]), c0[1].max(c1[1])]);
                let (up, angle) = if j == 0 { (f.up, f.angle) } else { (f.up ^ f.then[j - 1].flip, f.then[j - 1].angle) };
                bends.push(BendRow {
                    index: bends.len() + 1,
                    up,
                    angle: angle.to_degrees(),
                    included_angle: 180.0 - angle.to_degrees(),
                    radius: f.radius,
                    allowance: bd.allowance,
                    length: xb - xa,
                    start: at(xa, sl),
                    end: at(xb, sl),
                    feature: f.feature.clone(),
                    edge: f.edge,
                });
                hi = lo - bd.leg;
            }
        }
        FlatPattern::new(&self.rule, loops, bends, zones)
    }
}

/// Where the last leg of a chain ends (`s` inward from the outline edge, negative = beyond it).
pub fn chain_end(ch: &[Bend]) -> f64 {
    let mut s = ch[0].b;
    for bd in ch {
        s -= bd.allowance + bd.leg;
    }
    s
}

/// Drop repeated and collinear points of a closed polygon.
pub fn clean(p: Vec<P>) -> Vec<P> {
    let mut v: Vec<P> = vec![];
    for q in p {
        if v.last().map_or(true, |l| len(sub(q, *l)) > 1e-9) {
            v.push(q);
        }
    }
    while v.len() > 1 && len(sub(v[0], v[v.len() - 1])) <= 1e-9 {
        v.pop();
    }
    loop {
        let n = v.len();
        if n < 4 {
            return v;
        }
        let drop = (0..n).find(|&i| {
            let (a, b, c) = (v[(i + n - 1) % n], v[i], v[(i + 1) % n]);
            let (u, w) = (sub(b, a), sub(c, b));
            cross(u, w).abs() <= 1e-9 * len(u).max(1.0) * len(w).max(1.0)
        });
        match drop {
            Some(i) => {
                v.remove(i);
            }
            None => return v,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BendRow {
    pub index: usize,
    pub up: bool,
    pub angle: f64,
    pub included_angle: f64,
    pub radius: f64,
    pub allowance: f64,
    /// Length of the bend line.
    pub length: f64,
    pub start: P,
    pub end: P,
    pub feature: String,
    pub edge: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FlatPattern {
    pub rule: String,
    pub thickness: f64,
    pub k_factor: f64,
    /// Width, height (in the plane of the base sketch) and thickness.
    pub size: [f64; 3],
    /// The outer loop first, then the holes.
    pub outline: Vec<Vec<P>>,
    pub bends: Vec<BendRow>,
    /// Bounding box `[xmin, xmax, ymin, ymax]` of each bend zone, in bend order.
    pub zones: Vec<[f64; 4]>,
    /// The body is shown unfolded (`unfold` feature).
    pub unfolded: bool,
}

fn pt(p: P) -> Json {
    Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])])
}

impl FlatPattern {
    /// The flat pattern of an outline (outer loop first), its bends and their zones.
    pub fn new(rule: &Rule, outline: Vec<Vec<P>>, bends: Vec<BendRow>, zones: Vec<[f64; 4]>) -> FlatPattern {
        let mut lo = [f64::MAX; 2];
        let mut hi = [f64::MIN; 2];
        for p in &outline[0] {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        FlatPattern {
            rule: rule.name.clone(),
            thickness: rule.thickness,
            k_factor: rule.k_factor,
            size: [hi[0] - lo[0], hi[1] - lo[1], rule.thickness],
            outline,
            bends,
            zones,
            unfolded: false,
        }
    }

    pub fn to_json(&self) -> Json {
        let bends: Vec<Json> = self
            .bends
            .iter()
            .map(|b| {
                Obj::new()
                    .put("bend", b.index as f64)
                    .put("role", format!("B{}", b.index))
                    .put("feature", b.feature.as_str())
                    .put("direction", if b.up { "up" } else { "down" })
                    .put("angleDeg", b.angle)
                    .put("includedAngleDeg", b.included_angle)
                    .put("radius", b.radius)
                    .put("allowance", b.allowance)
                    .put("length", b.length)
                    .put("start", pt(b.start))
                    .put("end", pt(b.end))
                    .build()
            })
            .collect();
        Obj::new()
            .put("rule", self.rule.as_str())
            .put("thickness", self.thickness)
            .put("kFactor", self.k_factor)
            .put("flatSize", Json::Arr(self.size.iter().map(|v| Json::Num(*v)).collect()))
            .put("outline", Json::Arr(self.outline.iter().map(|l| Json::Arr(l.iter().map(|p| pt(*p)).collect())).collect()))
            .put("bends", Json::Arr(bends))
            .put("bendZones", Json::Arr(self.zones.iter().map(|z| Json::Arr(z.iter().map(|v| Json::Num(*v)).collect())).collect()))
            .put("unfolded", self.unfolded)
            .build()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} vs {b}");
    }

    fn rect(w: f64, h: f64, rule: Rule) -> Sheet {
        Sheet {
            base: "F1".into(),
            rule,
            outline: vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]],
            tags: vec!["e1".into(), "e2".into(), "e3".into(), "e4".into()],
            line: vec![true; 4],
            holes: vec![],
            flanges: vec![],
            folds: vec![],
            fold_root: None,
        }
    }

    fn rule(t: f64, k: f64, r: f64) -> Rule {
        let mut ru = Rule::new("R", t);
        ru.k_factor = k;
        ru.bend_radius = r;
        ru
    }

    fn flange(edge: usize, h: f64, deg: f64, r: f64, batch: usize) -> Flange {
        Flange {
            feature: format!("E{batch}"),
            edge,
            height: h,
            angle: deg.to_radians(),
            radius: r,
            height_from: HeightFrom::Outer,
            position: Position::Inside,
            up: true,
            batch,
            leg: [0.0, 0.0],
            hem: false,
            then: vec![],
        }
    }

    #[test]
    fn rule_defaults() {
        let r = Rule::new("A", 3.0);
        assert_eq!((r.bend_radius, r.relief_width, r.relief_depth, r.corner_relief, r.gap, r.hem_gap, r.k_factor), (3.0, 3.0, 1.5, 12.0, 3.0, 0.01, 0.44));
        assert_eq!(r.corner_relief_shape, ReliefShape::Notch);
    }

    #[test]
    fn allowance_and_setback() {
        let b = bend_of(2.0, 0.44, 2.0, std::f64::consts::FRAC_PI_2, 20.0, HeightFrom::Outer, Position::Inside).unwrap();
        close(b.allowance, 4.523893421169302);
        close(b.setback, 4.0);
        close(b.leg, 16.0);
        close(b.b, 4.0);
        let b = bend_of(2.0, 0.44, 2.0, 45f64.to_radians(), 20.0, HeightFrom::Outer, Position::Inside).unwrap();
        close(b.allowance, 2.261946710584651);
        close(b.setback, 4.0 * (22.5f64).to_radians().tan());
        close(b.leg + b.setback, 20.0);
        let b = bend_of(2.0, 0.44, 2.0, 120f64.to_radians(), 20.0, HeightFrom::Inner, Position::Inside).unwrap();
        close(b.leg + 2.0 * 60f64.to_radians().tan(), 20.0);
        let b = bend_of(2.0, 0.44, 2.0, 120f64.to_radians(), 20.0, HeightFrom::Tangent, Position::Inside).unwrap();
        close(b.leg, 16.0);
        // the height must reach the end of the bend
        match bend_of(2.0, 0.44, 2.0, std::f64::consts::FRAC_PI_2, 3.0, HeightFrom::Outer, Position::Inside) {
            Err(min) => close(min, 4.0),
            Ok(_) => panic!("height 3 must be refused"),
        }
        assert!(bend_of(2.0, 0.44, 2.0, std::f64::consts::FRAC_PI_2, 4.0, HeightFrom::Outer, Position::Inside).is_ok());
    }

    #[test]
    fn chained_legs_and_hem() {
        use std::f64::consts::{FRAC_PI_2, PI};
        let next = |angle: f64, height: f64| Next { angle, height, flip: false };
        let ch = chain_of(2.0, 0.44, 2.0, FRAC_PI_2, 20.0, HeightFrom::Outer, Position::Inside, false, &[next(FRAC_PI_2, 10.0)]).unwrap();
        close(ch[0].leg, 12.0);
        close(ch[1].leg, 6.0);
        // position outside: the next bend starts at the far edge, so the first leg keeps its setback
        let ch = chain_of(2.0, 0.44, 2.0, FRAC_PI_2, 20.0, HeightFrom::Outer, Position::Outside, false, &[next(FRAC_PI_2, 10.0)]).unwrap();
        close(ch[0].leg, 16.0);
        close(ch[0].b, 0.0);
        // the second height must reach its bend
        assert_eq!(chain_of(2.0, 0.44, 2.0, FRAC_PI_2, 20.0, HeightFrom::Outer, Position::Inside, false, &[next(FRAC_PI_2, 3.0)]).unwrap_err().0, 1);
        // hem: leg = length - (gap + t), allowance (gap + k t) pi, no setback
        let h = chain_of(2.0, 0.44, 0.01, PI, 10.0, HeightFrom::Outer, Position::Inside, true, &[]).unwrap();
        close(h[0].leg, 7.99);
        close(h[0].allowance, 2.796017461694916);
        close(h[0].b, 0.0);
        assert!(chain_of(2.0, 0.44, 0.01, PI, 2.0, HeightFrom::Outer, Position::Inside, true, &[]).is_err());
    }

    #[test]
    fn two_flanges_flat_size() {
        let mut s = rect(100.0, 60.0, rule(2.0, 0.44, 2.0));
        s.flanges = vec![flange(0, 20.0, 90.0, 2.0, 1), flange(2, 20.0, 90.0, 2.0, 1)];
        s.settle_batch(1);
        let fp = s.flat_pattern();
        close(fp.size[0], 100.0);
        close(fp.size[1], 93.04778684233861);
        assert_eq!(fp.bends.len(), 2);
        close(fp.bends[0].start[1], 1.7380532894153489);
        close(fp.bends[1].start[1], 58.26194671058465);
        assert!(fp.bends.iter().all(|b| b.up));
        assert_eq!(fp.outline[0].len(), 4);
    }

    #[test]
    fn four_flanges_notch() {
        let mut s = rect(100.0, 60.0, rule(2.0, 0.44, 2.0));
        s.flanges = vec![flange(0, 15.0, 90.0, 2.0, 1), flange(2, 15.0, 90.0, 2.0, 1), flange(3, 15.0, 90.0, 2.0, 1), flange(1, 15.0, 90.0, 2.0, 1)];
        s.settle_batch(1);
        let fp = s.flat_pattern();
        close(fp.size[0], 123.04778684233861);
        close(fp.size[1], 83.04778684233861);
        assert_eq!(fp.outline[0].len(), 28);
        // the legs stop `gap` short of each other: t + gap / sqrt 2 from the outer corner
        let near = |p: P, x: f64, y: f64| (p[0] - x).abs() < 1e-6 && (p[1] - y).abs() < 1e-6;
        assert!(fp.outline[0].iter().any(|p| near(*p, 2.0 + 2.0 / 2f64.sqrt(), -11.523893421169302)));
        close(fp.bends[2].length, 52.0);
    }
}
