//! Closed-profile (region) detection for a sketch — the `profiles` object of
//! a regen sketch result: `{"regions":[{"outer":[seg..],"holes":[[seg..]..]}],
//! "open":bool}` with `seg = {"kind":"line"|"arc"|"spline","tag":<entity id>,
//! "a":[x,y],"b":[x,y]}`.
//!
//! Behaviour pinned by oracle probes (see tests):
//! * all non-construction curves are intersected pairwise and split, giving a
//!   planar arrangement; dangling pieces are pruned (rect + crossing line ->
//!   two regions, the overhangs vanish); exact duplicate pieces are dropped
//!   (a second line over an existing edge, either direction); collinear
//!   overlaps are *not* split (a half-length segment lying on an edge is
//!   ignored);
//! * every bounded face of a connected component is a candidate region;
//!   components nested inside faces of other components alternate
//!   region/hole by even-odd depth (3 concentric circles -> ring + disk);
//! * a region's `outer` is the CCW face loop, starting at its lowest
//!   half-edge (pieces numbered in entity order, forward then reverse);
//!   `holes` are the CW outlines of directly nested components (the
//!   reversed CCW outline), regions are ordered by their first half-edge;
//! * circles/ellipses report kind `"arc"` with `a` at angle 0 and `b` at
//!   angle 2π (`[10,-2.4492935982947065e-15]` for r = 10); closed splines
//!   are split at half length; cvsplines report kind `"spline"`;
//! * segment ends are *evaluated* curve points, so they can differ from the
//!   sketch points in the last ulps (oracle: solved rectangle `e3.b.x =
//!   -9.499999902526266` for `p4.x = -9.499999902526271`); the exact
//!   kernel parametrisation is not reproduced yet (tests use 1e-9);
//! * `open` is true iff there is no region but at least one curve.
//!
//! Known gaps: intersection points of arcs/splines are not bit-identical to
//! the kernel's (tests compare with 1e-9 tolerance); cvspline evaluation uses
//! clamped/periodic uniform B-splines; ellipse intersections are sampled.

use super::spline::InterpSpline;
use super::{Sketch, SketchEntity};
use ck_json::Json;
use std::f64::consts::PI;

type P = [f64; 2];

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn dist(a: P, b: P) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Curve geometry of one sketch entity, with its own parameter.
#[derive(Debug, Clone)]
pub enum Geom {
    /// `p + d·t`, t ∈ [0,1].
    Line { p: P, d: P },
    /// `c + r(cos θ, sin θ)`.
    Circle { c: P, r: f64 },
    /// `c + u cos θ + v sin θ`.
    Ellipse { c: P, u: P, v: P },
    /// Interpolating spline, parameter = chord length.
    Spline(InterpSpline),
    /// B-spline (control points, degree, knots).
    BSpline { cv: Vec<P>, degree: usize, knots: Vec<f64> },
}

impl Geom {
    pub fn eval(&self, t: f64) -> P {
        match self {
            Geom::Line { p, d } => [p[0] + d[0] * t, p[1] + d[1] * t],
            Geom::Circle { c, r } => [c[0] + r * t.cos(), c[1] + r * t.sin()],
            Geom::Ellipse { c, u, v } => [c[0] + u[0] * t.cos() + v[0] * t.sin(), c[1] + u[1] * t.cos() + v[1] * t.sin()],
            Geom::Spline(s) => s.eval(t),
            Geom::BSpline { cv, degree, knots } => de_boor(cv, *degree, knots, t),
        }
    }
    /// First derivative.
    pub fn d1(&self, t: f64) -> P {
        match self {
            Geom::Line { d, .. } => *d,
            Geom::Circle { r, .. } => [-r * t.sin(), r * t.cos()],
            Geom::Ellipse { u, v, .. } => [-u[0] * t.sin() + v[0] * t.cos(), -u[1] * t.sin() + v[1] * t.cos()],
            Geom::Spline(s) => s.at(t, 1),
            Geom::BSpline { .. } => {
                let h = 1e-6;
                let (a, b) = (self.eval(t - h), self.eval(t + h));
                [(b[0] - a[0]) / (2.0 * h), (b[1] - a[1]) / (2.0 * h)]
            }
        }
    }
    /// Second derivative.
    pub fn d2(&self, t: f64) -> P {
        match self {
            Geom::Line { .. } => [0.0, 0.0],
            Geom::Circle { r, .. } => [-r * t.cos(), -r * t.sin()],
            Geom::Ellipse { u, v, .. } => [-u[0] * t.cos() - v[0] * t.sin(), -u[1] * t.cos() - v[1] * t.sin()],
            Geom::Spline(s) => s.at(t, 2),
            Geom::BSpline { .. } => {
                let h = 1e-4;
                let (a, m, b) = (self.eval(t - h), self.eval(t), self.eval(t + h));
                [(a[0] - 2.0 * m[0] + b[0]) / (h * h), (a[1] - 2.0 * m[1] + b[1]) / (h * h)]
            }
        }
    }
}

fn de_boor(cv: &[P], p: usize, k: &[f64], t: f64) -> P {
    let n = cv.len();
    let lo = k[p];
    let hi = k[n];
    let t = t.max(lo).min(hi);
    let mut s = p;
    while s + 1 < n && k[s + 1] <= t {
        s += 1;
    }
    let mut d: Vec<P> = (0..=p).map(|j| cv[j + s - p]).collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = j + s - p;
            let den = k[i + p + 1 - r] - k[i];
            let a = if den.abs() < 1e-300 { 0.0 } else { (t - k[i]) / den };
            d[j] = [(1.0 - a) * d[j - 1][0] + a * d[j][0], (1.0 - a) * d[j - 1][1] + a * d[j][1]];
        }
    }
    d[p]
}

/// A trimmed curve (entity) in the arrangement.
#[derive(Debug, Clone)]
struct Curve {
    tag: String,
    kind: &'static str,
    geom: Geom,
    t0: f64,
    t1: f64,
    closed: bool,
}

/// Geometry of an output segment (for consumers such as extrude).
#[derive(Debug, Clone)]
pub struct SegGeom {
    pub geom: Geom,
    /// Parameter at `a` and at `b` (decreasing when traversed in reverse).
    pub ta: f64,
    pub tb: f64,
}

/// One boundary segment of a profile loop.
#[derive(Debug, Clone)]
pub struct Seg {
    pub kind: &'static str,
    pub tag: String,
    pub a: P,
    pub b: P,
    pub curve: SegGeom,
}

impl Seg {
    pub fn to_json(&self) -> Json {
        let pt = |p: P| Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])]);
        Json::Obj(vec![
            ("kind".into(), Json::Str(self.kind.into())),
            ("tag".into(), Json::Str(self.tag.clone())),
            ("a".into(), pt(self.a)),
            ("b".into(), pt(self.b)),
        ])
    }
    /// Polyline samples from `a` to `b` (inclusive), `n` intervals.
    pub fn sample(&self, n: usize) -> Vec<P> {
        let n = n.max(1);
        (0..=n)
            .map(|i| self.curve.geom.eval(self.curve.ta + (self.curve.tb - self.curve.ta) * i as f64 / n as f64))
            .collect()
    }
}

#[derive(Debug, Clone)]
pub struct Region {
    pub outer: Vec<Seg>,
    pub holes: Vec<Vec<Seg>>,
}

#[derive(Debug, Clone, Default)]
pub struct Profiles {
    pub regions: Vec<Region>,
    pub open: bool,
}

impl Profiles {
    pub fn to_json(&self) -> Json {
        let lp = |l: &Vec<Seg>| Json::Arr(l.iter().map(|s| s.to_json()).collect());
        Json::Obj(vec![
            (
                "regions".into(),
                Json::Arr(
                    self.regions
                        .iter()
                        .map(|r| {
                            Json::Obj(vec![
                                ("outer".into(), lp(&r.outer)),
                                ("holes".into(), Json::Arr(r.holes.iter().map(lp).collect())),
                            ])
                        })
                        .collect(),
                ),
            ),
            ("open".into(), Json::Bool(self.open)),
        ])
    }
}

fn norm_sweep(mut o: f64) -> f64 {
    // client tn(): sweep in (0, 2π]
    while o <= 1e-12 {
        o += 2.0 * PI;
    }
    while o > 2.0 * PI + 1e-12 {
        o -= 2.0 * PI;
    }
    o
}

fn default_knots(n: usize, p: usize, closed: bool) -> Vec<f64> {
    if closed {
        (0..=n).map(|i| i as f64).collect()
    } else {
        let inner = n - p; // number of spans
        let mut k = vec![0.0; p + 1];
        for i in 1..inner {
            k.push(i as f64 / inner as f64);
        }
        k.extend(std::iter::repeat(1.0).take(p + 1));
        k
    }
}

fn curves_of(sk: &Sketch) -> (Vec<Curve>, bool) {
    let pt = |id: &str| sk.point(id).map(|p| [p.x, p.y]);
    let mut out = Vec::new();
    let mut any = false;
    for e in &sk.entities {
        if e.construction() {
            continue;
        }
        let tag = e.id().to_string();
        let c = match e {
            SketchEntity::Line { p1, p2, .. } => {
                let (Some(a), Some(b)) = (pt(p1), pt(p2)) else { continue };
                if dist(a, b) < 1e-12 {
                    continue;
                }
                Curve { tag, kind: "line", geom: Geom::Line { p: a, d: sub(b, a) }, t0: 0.0, t1: 1.0, closed: false }
            }
            SketchEntity::Circle { c, r, .. } => {
                let Some(cc) = pt(c) else { continue };
                if r.abs() < 1e-12 {
                    continue;
                }
                Curve { tag, kind: "arc", geom: Geom::Circle { c: cc, r: r.abs() }, t0: 0.0, t1: 2.0 * PI, closed: true }
            }
            SketchEntity::Arc { c, p1, p2, .. } => {
                let (Some(cc), Some(a), Some(b)) = (pt(c), pt(p1), pt(p2)) else { continue };
                let a0 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
                let sw = norm_sweep((b[1] - cc[1]).atan2(b[0] - cc[0]) - a0);
                let r = dist(a, cc);
                if r < 1e-12 {
                    continue;
                }
                let closed = (sw - 2.0 * PI).abs() < 1e-9;
                Curve { tag, kind: "arc", geom: Geom::Circle { c: cc, r }, t0: a0, t1: a0 + sw, closed }
            }
            SketchEntity::Ellipse { c, p, r, .. } => {
                let (Some(cc), Some(pp)) = (pt(c), pt(p)) else { continue };
                let u = sub(pp, cc);
                let lu = u[0].hypot(u[1]);
                if lu < 1e-12 {
                    continue;
                }
                let v = [-u[1] / lu * r, u[0] / lu * r];
                Curve { tag, kind: "arc", geom: Geom::Ellipse { c: cc, u, v }, t0: 0.0, t1: 2.0 * PI, closed: true }
            }
            SketchEntity::Spline { pts, closed, .. } => {
                let ps: Vec<P> = pts.iter().filter_map(|p| pt(p)).collect();
                let Some(s) = InterpSpline::new(&ps, *closed) else { continue };
                let (l, cl) = (s.length(), s.closed);
                Curve { tag, kind: "spline", geom: Geom::Spline(s), t0: 0.0, t1: l, closed: cl }
            }
            SketchEntity::CvSpline { pts, degree, knots, closed, .. } => {
                let mut cv: Vec<P> = pts.iter().filter_map(|p| pt(p)).collect();
                let p = (*degree as usize).max(1);
                if cv.len() < p + 1 {
                    continue;
                }
                // GUESS: closed cvsplines as periodic uniform (wrap p control points).
                let k = if *closed {
                    for i in 0..p {
                        let q = cv[i];
                        cv.push(q);
                    }
                    (0..cv.len() + p + 1).map(|i| i as f64).collect()
                } else {
                    knots.clone().filter(|k| k.len() == cv.len() + p + 1).unwrap_or_else(|| default_knots(cv.len(), p, false))
                };
                let (t0, t1) = (k[p], k[cv.len()]);
                Curve { tag, kind: "spline", geom: Geom::BSpline { cv, degree: p, knots: k }, t0, t1, closed: *closed }
            }
            SketchEntity::Point { .. } => continue,
        };
        any = true;
        out.push(c);
    }
    (out, any)
}

// ---------------------------------------------------------------------------
// intersections

const EPS_T: f64 = 1e-9;

fn add_param(v: &mut Vec<f64>, c: &Curve, t: f64) {
    v.push(t);
    let _ = c;
}

/// Normalise an angle into the arc's parameter range; None when outside.
fn arc_param(c: &Curve, th: f64) -> Option<f64> {
    let span = c.t1 - c.t0;
    let mut d = (th - c.t0) % (2.0 * PI);
    if d < 0.0 {
        d += 2.0 * PI;
    }
    if d > span + 1e-9 {
        if (d - 2.0 * PI).abs() < 1e-9 {
            return Some(c.t0);
        }
        return None;
    }
    Some(c.t0 + d.min(span))
}

fn inter_line_line(a: &Curve, b: &Curve, ta: &mut Vec<f64>, tb: &mut Vec<f64>) {
    let (Geom::Line { p, d }, Geom::Line { p: q, d: e }) = (&a.geom, &b.geom) else { return };
    let den = cross(*d, *e);
    let scale = (d[0].hypot(d[1])) * (e[0].hypot(e[1]));
    if den.abs() <= 1e-12 * scale {
        return; // parallel / collinear: never split (oracle)
    }
    let w = sub(*q, *p);
    let t = cross(w, *e) / den;
    let s = cross(w, *d) / den;
    let tol_t = 1e-9 / d[0].hypot(d[1]).max(1e-300) * 1e3;
    let tol_s = 1e-9 / e[0].hypot(e[1]).max(1e-300) * 1e3;
    if t >= -tol_t && t <= 1.0 + tol_t && s >= -tol_s && s <= 1.0 + tol_s {
        add_param(ta, a, t.max(0.0).min(1.0));
        add_param(tb, b, s.max(0.0).min(1.0));
    }
}

fn inter_line_circle(l: &Curve, c: &Curve, tl: &mut Vec<f64>, tc: &mut Vec<f64>) {
    let (Geom::Line { p, d }, Geom::Circle { c: cc, r }) = (&l.geom, &c.geom) else { return };
    let f = sub(*p, *cc);
    let a = dot(*d, *d);
    let b = 2.0 * dot(f, *d);
    let k = dot(f, f) - r * r;
    let disc = b * b - 4.0 * a * k;
    let len = a.sqrt();
    let tol = 1e-9 * (1.0 + r) * 4.0 * a * 1e3;
    if disc < -tol {
        return;
    }
    let sq = disc.max(0.0).sqrt();
    let mut roots = vec![(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)];
    if sq < 1e-12 {
        roots.truncate(1);
    }
    let tol_t = 1e-6 / len.max(1e-300);
    for t in roots {
        if t < -tol_t || t > 1.0 + tol_t {
            continue;
        }
        let t = t.max(0.0).min(1.0);
        let q = l.geom.eval(t);
        let th = (q[1] - cc[1]).atan2(q[0] - cc[0]);
        if let Some(u) = arc_param(c, th) {
            tl.push(t);
            tc.push(u);
        }
    }
}

fn inter_circle_circle(a: &Curve, b: &Curve, ta: &mut Vec<f64>, tb: &mut Vec<f64>) {
    let (Geom::Circle { c: c1, r: r1 }, Geom::Circle { c: c2, r: r2 }) = (&a.geom, &b.geom) else { return };
    let dv = sub(*c2, *c1);
    let d = dv[0].hypot(dv[1]);
    if d < 1e-12 {
        return; // concentric: no crossing (equal circles = duplicates)
    }
    let tol = 1e-9 * (1.0 + r1 + r2);
    if d > r1 + r2 + tol || d < (r1 - r2).abs() - tol {
        return;
    }
    let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let h2 = r1 * r1 - x * x;
    let h = h2.max(0.0).sqrt();
    let ux = [dv[0] / d, dv[1] / d];
    let base = [c1[0] + ux[0] * x, c1[1] + ux[1] * x];
    let mut pts = vec![[base[0] - ux[1] * h, base[1] + ux[0] * h]];
    if h > 1e-9 {
        pts.push([base[0] + ux[1] * h, base[1] - ux[0] * h]);
    }
    for q in pts {
        let t1 = (q[1] - c1[1]).atan2(q[0] - c1[0]);
        let t2 = (q[1] - c2[1]).atan2(q[0] - c2[0]);
        if let (Some(u), Some(v)) = (arc_param(a, t1), arc_param(b, t2)) {
            ta.push(u);
            tb.push(v);
        }
    }
}

fn samples(c: &Curve) -> Vec<(f64, P)> {
    let n = match &c.geom {
        Geom::Line { .. } => 1,
        Geom::Circle { .. } | Geom::Ellipse { .. } => (((c.t1 - c.t0).abs() / (PI / 64.0)).ceil() as usize).max(4),
        Geom::Spline(s) => s.h.len() * 32,
        Geom::BSpline { cv, .. } => cv.len() * 32,
    };
    (0..=n)
        .map(|i| {
            let t = c.t0 + (c.t1 - c.t0) * i as f64 / n as f64;
            (t, c.geom.eval(t))
        })
        .collect()
}

/// Generic intersection: polyline crossing + Newton refinement on P(s) = Q(t).
fn inter_generic(a: &Curve, b: &Curve, ta: &mut Vec<f64>, tb: &mut Vec<f64>) {
    let sa = samples(a);
    let sb = samples(b);
    for i in 0..sa.len() - 1 {
        for j in 0..sb.len() - 1 {
            let (p0, p1) = (sa[i].1, sa[i + 1].1);
            let (q0, q1) = (sb[j].1, sb[j + 1].1);
            let d = sub(p1, p0);
            let e = sub(q1, q0);
            let den = cross(d, e);
            if den.abs() < 1e-300 {
                continue;
            }
            let w = sub(q0, p0);
            let u = cross(w, e) / den;
            let v = cross(w, d) / den;
            if !(-1e-9..=1.0 + 1e-9).contains(&u) || !(-1e-9..=1.0 + 1e-9).contains(&v) {
                continue;
            }
            let mut s = sa[i].0 + (sa[i + 1].0 - sa[i].0) * u;
            let mut t = sb[j].0 + (sb[j + 1].0 - sb[j].0) * v;
            for _ in 0..20 {
                let f = sub(a.geom.eval(s), b.geom.eval(t));
                let da = a.geom.d1(s);
                let db = b.geom.d1(t);
                // [da, -db] [ds dt]^T = -f
                let det = cross(da, [-db[0], -db[1]]);
                if det.abs() < 1e-300 {
                    break;
                }
                let ds = cross([-f[0], -f[1]], [-db[0], -db[1]]) / det;
                let dt = cross(da, [-f[0], -f[1]]) / det;
                s += ds;
                t += dt;
                if ds.abs() + dt.abs() < 1e-14 {
                    break;
                }
            }
            let clamp = |c: &Curve, x: f64| x.max(c.t0.min(c.t1)).min(c.t0.max(c.t1));
            let (s, t) = (clamp(a, s), clamp(b, t));
            if dist(a.geom.eval(s), b.geom.eval(t)) < 1e-6 {
                ta.push(s);
                tb.push(t);
            }
        }
    }
}

fn intersect(a: &Curve, b: &Curve, ta: &mut Vec<f64>, tb: &mut Vec<f64>) {
    match (&a.geom, &b.geom) {
        (Geom::Line { .. }, Geom::Line { .. }) => inter_line_line(a, b, ta, tb),
        (Geom::Line { .. }, Geom::Circle { .. }) => inter_line_circle(a, b, ta, tb),
        (Geom::Circle { .. }, Geom::Line { .. }) => inter_line_circle(b, a, tb, ta),
        (Geom::Circle { .. }, Geom::Circle { .. }) => inter_circle_circle(a, b, ta, tb),
        _ => inter_generic(a, b, ta, tb),
    }
}

// ---------------------------------------------------------------------------
// arrangement

#[derive(Debug, Clone)]
struct Piece {
    curve: usize,
    t0: f64,
    t1: f64,
    v0: usize,
    v1: usize,
    alive: bool,
}

struct Graph {
    verts: Vec<P>,
    tol: f64,
}

impl Graph {
    fn vertex(&mut self, p: P) -> usize {
        for (i, v) in self.verts.iter().enumerate() {
            if dist(*v, p) <= self.tol {
                return i;
            }
        }
        self.verts.push(p);
        self.verts.len() - 1
    }
}

fn dedup_sorted(v: &mut Vec<f64>, tol: f64) {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut out: Vec<f64> = Vec::new();
    for x in v.iter() {
        if out.last().map_or(true, |l| (x - l).abs() > tol) {
            out.push(*x);
        }
    }
    *v = out;
}

/// Half-edge `h`: piece `h/2`, forward if even.
fn he_start_param(pcs: &[Piece], h: usize) -> f64 {
    let p = &pcs[h / 2];
    if h % 2 == 0 {
        p.t0
    } else {
        p.t1
    }
}
fn he_end_param(pcs: &[Piece], h: usize) -> f64 {
    let p = &pcs[h / 2];
    if h % 2 == 0 {
        p.t1
    } else {
        p.t0
    }
}
fn he_from(pcs: &[Piece], h: usize) -> usize {
    if h % 2 == 0 {
        pcs[h / 2].v0
    } else {
        pcs[h / 2].v1
    }
}
fn he_to(pcs: &[Piece], h: usize) -> usize {
    if h % 2 == 0 {
        pcs[h / 2].v1
    } else {
        pcs[h / 2].v0
    }
}

/// Outgoing direction angle and signed curvature of a half-edge at its start.
fn he_dir(curves: &[Curve], pcs: &[Piece], h: usize) -> (f64, f64) {
    let p = &pcs[h / 2];
    let g = &curves[p.curve].geom;
    let t = he_start_param(pcs, h);
    let mut d = g.d1(t);
    let dd = g.d2(t);
    let sgn = if (p.t1 - p.t0) >= 0.0 { 1.0 } else { -1.0 };
    let fwd = h % 2 == 0;
    let s = if fwd { sgn } else { -sgn };
    d = [d[0] * s, d[1] * s];
    let sp = d[0].hypot(d[1]);
    // curvature of the traversed curve (left positive); second derivative is
    // unchanged by parameter reversal, first derivative flips.
    let k = if sp > 1e-300 { cross(d, dd) / (sp * sp * sp) } else { 0.0 };
    (d[1].atan2(d[0]), k)
}

fn loop_polygon(curves: &[Curve], pcs: &[Piece], cyc: &[usize]) -> Vec<P> {
    let mut poly = Vec::new();
    for &h in cyc {
        let c = &curves[pcs[h / 2].curve];
        let (ta, tb) = (he_start_param(pcs, h), he_end_param(pcs, h));
        let n = match &c.geom {
            Geom::Line { .. } => 1,
            Geom::Circle { .. } | Geom::Ellipse { .. } => (((tb - ta).abs() / (PI / 64.0)).ceil() as usize).max(2),
            _ => 48,
        };
        for i in 0..n {
            poly.push(c.geom.eval(ta + (tb - ta) * i as f64 / n as f64));
        }
    }
    poly
}

fn area(poly: &[P]) -> f64 {
    let mut a = 0.0;
    for i in 0..poly.len() {
        let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
        a += cross(p, q);
    }
    a / 2.0
}

fn inside(poly: &[P], q: P) -> bool {
    let mut c = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + n - 1) % n]);
        if (a[1] > q[1]) != (b[1] > q[1]) {
            let x = (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0];
            if q[0] < x {
                c = !c;
            }
        }
    }
    c
}

fn seg_of(curves: &[Curve], pcs: &[Piece], h: usize) -> Seg {
    let c = &curves[pcs[h / 2].curve];
    let (ta, tb) = (he_start_param(pcs, h), he_end_param(pcs, h));
    Seg {
        kind: c.kind,
        tag: c.tag.clone(),
        a: c.geom.eval(ta),
        b: c.geom.eval(tb),
        curve: SegGeom { geom: c.geom.clone(), ta, tb },
    }
}

/// Compute the closed profiles of a sketch.
pub fn profiles(sk: &Sketch) -> Profiles {
    let (curves, any) = curves_of(sk);
    let n = curves.len();
    // split parameters
    let mut params: Vec<Vec<f64>> = curves.iter().map(|c| vec![c.t0, c.t1]).collect();
    for i in 0..n {
        for j in i + 1..n {
            let (mut ta, mut tb) = (Vec::new(), Vec::new());
            intersect(&curves[i], &curves[j], &mut ta, &mut tb);
            params[i].extend(ta);
            params[j].extend(tb);
        }
    }
    // pieces
    let mut scale = 1.0f64;
    for c in &curves {
        for t in [c.t0, c.t1] {
            let p = c.geom.eval(t);
            scale = scale.max(p[0].abs()).max(p[1].abs());
        }
    }
    let mut g = Graph { verts: Vec::new(), tol: 1e-9 * scale.max(1.0) * 10.0 };
    let mut pcs: Vec<Piece> = Vec::new();
    for (ci, c) in curves.iter().enumerate() {
        let mut ts = params[ci].clone();
        dedup_sorted(&mut ts, EPS_T * (c.t1 - c.t0).abs().max(1.0));
        let mut cuts: Vec<(f64, f64)> = Vec::new();
        if c.closed {
            // interior split params (excluding the seam unless it's a real cut)
            let span = c.t1 - c.t0;
            let mut inner: Vec<f64> = ts
                .iter()
                .copied()
                .filter(|t| (t - c.t0).abs() > EPS_T * span && (t - c.t1).abs() > EPS_T * span)
                .collect();
            let seam_cut = params[ci].iter().filter(|t| (*t - c.t0).abs() <= EPS_T * span || (*t - c.t1).abs() <= EPS_T * span).count() > 2;
            if seam_cut {
                inner.insert(0, c.t0);
            }
            if inner.is_empty() {
                if let Geom::Spline(_) | Geom::BSpline { .. } = c.geom {
                    // oracle: closed splines are split at half length
                    let m = c.t0 + span / 2.0;
                    cuts.push((c.t0, m));
                    cuts.push((m, c.t1));
                } else {
                    cuts.push((c.t0, c.t1));
                }
            } else {
                for k in 0..inner.len() {
                    let a = inner[k];
                    let b = if k + 1 < inner.len() { inner[k + 1] } else { inner[0] + span };
                    cuts.push((a, b));
                }
            }
        } else {
            for k in 0..ts.len() - 1 {
                cuts.push((ts[k], ts[k + 1]));
            }
        }
        for (a, b) in cuts {
            if (b - a).abs() <= EPS_T * (c.t1 - c.t0).abs().max(1e-300) {
                continue;
            }
            let pa = c.geom.eval(a);
            let pb = c.geom.eval(b);
            let v0 = g.vertex(pa);
            let v1 = g.vertex(pb);
            let mid = c.geom.eval((a + b) / 2.0);
            // drop exact duplicates of an earlier piece
            let dup = pcs.iter().any(|q| {
                ((q.v0 == v0 && q.v1 == v1) || (q.v0 == v1 && q.v1 == v0)) && {
                    let qc = &curves[q.curve];
                    dist(qc.geom.eval((q.t0 + q.t1) / 2.0), mid) <= g.tol * 10.0
                }
            });
            if dup {
                continue;
            }
            pcs.push(Piece { curve: ci, t0: a, t1: b, v0, v1, alive: true });
        }
    }
    // prune dangling pieces
    loop {
        let mut deg = vec![0usize; g.verts.len()];
        for p in pcs.iter().filter(|p| p.alive) {
            deg[p.v0] += 1;
            deg[p.v1] += 1;
        }
        let mut changed = false;
        for p in pcs.iter_mut().filter(|p| p.alive) {
            if deg[p.v0] < 2 || deg[p.v1] < 2 {
                p.alive = false;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    let nh = pcs.len() * 2;
    // outgoing half-edges per vertex, sorted CCW by (angle, curvature)
    let mut out: Vec<Vec<(f64, f64, usize)>> = vec![Vec::new(); g.verts.len()];
    for h in 0..nh {
        if !pcs[h / 2].alive {
            continue;
        }
        let (a, k) = he_dir(&curves, &pcs, h);
        out[he_from(&pcs, h)].push((a, k, h));
    }
    for o in out.iter_mut() {
        o.sort_by(|x, y| {
            if (x.0 - y.0).abs() < 1e-9 {
                x.1.partial_cmp(&y.1).unwrap_or(std::cmp::Ordering::Equal)
            } else {
                x.0.partial_cmp(&y.0).unwrap()
            }
        });
    }
    let next = |h: usize| -> usize {
        let v = he_to(&pcs, h);
        let t = h ^ 1;
        let o = &out[v];
        let i = o.iter().position(|e| e.2 == t).unwrap();
        o[(i + o.len() - 1) % o.len()].2
    };
    // trace cycles
    let mut seen = vec![false; nh];
    let mut faces: Vec<Vec<usize>> = Vec::new();
    let mut exteriors: Vec<Vec<usize>> = Vec::new();
    for h0 in 0..nh {
        if seen[h0] || !pcs[h0 / 2].alive {
            continue;
        }
        let mut cyc = Vec::new();
        let mut h = h0;
        while !seen[h] {
            seen[h] = true;
            cyc.push(h);
            h = next(h);
        }
        let poly = loop_polygon(&curves, &pcs, &cyc);
        if area(&poly) > 0.0 {
            faces.push(cyc);
        } else {
            exteriors.push(cyc);
        }
    }
    // components (union-find over vertices)
    let mut uf: Vec<usize> = (0..g.verts.len()).collect();
    fn find(uf: &mut Vec<usize>, x: usize) -> usize {
        let mut x = x;
        while uf[x] != x {
            uf[x] = uf[uf[x]];
            x = uf[x];
        }
        x
    }
    for p in pcs.iter().filter(|p| p.alive) {
        let (a, b) = (find(&mut uf, p.v0), find(&mut uf, p.v1));
        uf[a] = b;
    }
    let comp_of = |uf: &mut Vec<usize>, cyc: &Vec<usize>| find(uf, pcs[cyc[0] / 2].v0);
    let face_comp: Vec<usize> = faces.iter().map(|f| comp_of(&mut uf, f)).collect();
    let face_poly: Vec<Vec<P>> = faces.iter().map(|f| loop_polygon(&curves, &pcs, f)).collect();
    // a probe point on each face boundary (components are disjoint, so any
    // boundary point of F is inside/outside another component's face as a whole)
    let probe = |cyc: &Vec<usize>| -> P {
        let h = cyc[0];
        let c = &curves[pcs[h / 2].curve];
        c.geom.eval((he_start_param(&pcs, h) + he_end_param(&pcs, h)) / 2.0)
    };
    let containers = |pt: P, comp: usize| -> Vec<usize> {
        (0..faces.len()).filter(|&j| face_comp[j] != comp && inside(&face_poly[j], pt)).collect()
    };
    let depth: Vec<usize> = (0..faces.len()).map(|i| containers(probe(&faces[i]), face_comp[i]).len()).collect();
    // holes: exterior cycles whose innermost container is the face
    let mut holes_of: Vec<Vec<(usize, Vec<usize>)>> = vec![Vec::new(); faces.len()];
    for ext in &exteriors {
        let comp = comp_of(&mut uf, ext);
        let cs = containers(probe(ext), comp);
        if let Some(&inner) = cs.iter().max_by_key(|&&j| depth[j]) {
            // CCW outline = reverse of twins; rotate to min index; hole = reverse again
            let mut ccw: Vec<usize> = ext.iter().rev().map(|h| h ^ 1).collect();
            let k = (0..ccw.len()).min_by_key(|&i| ccw[i]).unwrap();
            ccw.rotate_left(k);
            let hole: Vec<usize> = ccw.iter().rev().map(|h| h ^ 1).collect();
            let key = *ccw.iter().min().unwrap();
            holes_of[inner].push((key, hole));
        }
    }
    let mut regions: Vec<(usize, Region)> = Vec::new();
    for (i, f) in faces.iter().enumerate() {
        if depth[i] % 2 == 1 {
            continue;
        }
        let mut cyc = f.clone();
        let k = (0..cyc.len()).min_by_key(|&j| cyc[j]).unwrap();
        cyc.rotate_left(k);
        let mut hs = holes_of[i].clone();
        hs.sort_by_key(|h| h.0);
        regions.push((
            cyc[0],
            Region {
                outer: cyc.iter().map(|&h| seg_of(&curves, &pcs, h)).collect(),
                holes: hs.iter().map(|(_, h)| h.iter().map(|&x| seg_of(&curves, &pcs, x)).collect()).collect(),
            },
        ));
    }
    regions.sort_by_key(|r| r.0);
    let regions: Vec<Region> = regions.into_iter().map(|r| r.1).collect();
    Profiles { open: regions.is_empty() && any, regions }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(src: &str) -> Sketch {
        Sketch::from_json(&ck_json::parse(src).unwrap())
    }

    /// Compare against an oracle `profiles` JSON with tolerance on numbers.
    fn assert_close(got: &Json, want: &Json, path: &str) {
        match (got, want) {
            (Json::Num(a), Json::Num(b)) => assert!((a - b).abs() <= 1e-9 * (1.0 + b.abs()), "{path}: {a} vs {b}"),
            (Json::Arr(a), Json::Arr(b)) => {
                assert_eq!(a.len(), b.len(), "{path}: len\n got {}\nwant {}", ck_json::to_string(got), ck_json::to_string(want));
                for (i, (x, y)) in a.iter().zip(b).enumerate() {
                    assert_close(x, y, &format!("{path}[{i}]"));
                }
            }
            (Json::Obj(a), Json::Obj(b)) => {
                assert_eq!(a.len(), b.len(), "{path}: keys");
                for ((ka, x), (kb, y)) in a.iter().zip(b) {
                    assert_eq!(ka, kb, "{path}");
                    assert_close(x, y, &format!("{path}.{ka}"));
                }
            }
            _ => assert_eq!(got, want, "{path}"),
        }
    }

    fn check(sketch: &str, oracle: &str) {
        let p = profiles(&sk(sketch));
        assert_close(&p.to_json(), &ck_json::parse(oracle).unwrap(), "profiles");
    }

    const RECT: &str = r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}"#;
    const SQ: &str = r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":0},{"id":"c","x":10,"y":10},{"id":"d","x":0,"y":10}"#;

    #[test]
    fn oracle_rect_holes_spline() {
        check(
            r#"{"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20},{"id":"c1","x":10,"y":10},{"id":"c2","x":30,"y":10},{"id":"a1","x":34,"y":10},{"id":"a2","x":26,"y":10},{"id":"q1","x":100,"y":0},{"id":"q2","x":120,"y":0},{"id":"q3","x":110,"y":10},{"id":"s1","x":200,"y":0},{"id":"s2","x":210,"y":5},{"id":"s3","x":220,"y":0}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"},{"type":"circle","id":"e5","c":"c1","r":3},{"type":"arc","id":"e6","c":"c2","p1":"a1","p2":"a2"},{"type":"line","id":"e7","p1":"a2","p2":"a1"},{"type":"line","id":"e8","p1":"q1","p2":"q2"},{"type":"line","id":"e9","p1":"q2","p2":"q3"},{"type":"spline","id":"e10","pts":["s1","s2","s3"]},{"type":"line","id":"e11","p1":"s3","p2":"s1"},{"type":"point","id":"e12","p":"c1"}],"constraints":[]}"#,
            r#"{"regions": [{"outer": [{"kind": "line", "tag": "e1", "a": [0, 0], "b": [40, 0]}, {"kind": "line", "tag": "e2", "a": [40, 0], "b": [40, 20]}, {"kind": "line", "tag": "e3", "a": [40, 20], "b": [0, 20]}, {"kind": "line", "tag": "e4", "a": [0, 20], "b": [0, 0]}], "holes": [[{"kind": "arc", "tag": "e5", "a": [13, 10], "b": [13, 10]}], [{"kind": "line", "tag": "e7", "a": [34, 10], "b": [26, 10]}, {"kind": "arc", "tag": "e6", "a": [26, 10], "b": [34, 10]}]]}, {"outer": [{"kind": "spline", "tag": "e10", "a": [220, 0], "b": [200, 0]}, {"kind": "line", "tag": "e11", "a": [200, 0], "b": [220, 0]}], "holes": []}], "open": false}"#,
        );
    }

    #[test]
    fn oracle_open_flags() {
        check(r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":0}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}]}"#, r#"{"regions":[],"open":true}"#);
        check(r#"{"points":[],"entities":[]}"#, r#"{"regions":[],"open":false}"#);
        check(r#"{"points":[{"id":"a","x":0,"y":0}],"entities":[{"type":"point","id":"e1","p":"a"}]}"#, r#"{"regions":[],"open":false}"#);
        check(r#"{"points":[{"id":"a","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"a","r":5,"construction":true}]}"#, r#"{"regions":[],"open":false}"#);
    }

    #[test]
    fn oracle_concentric_even_odd() {
        check(
            r#"{"points":[{"id":"a","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"a","r":10},{"type":"circle","id":"e2","c":"a","r":5},{"type":"circle","id":"e3","c":"a","r":2}]}"#,
            r#"{"regions": [{"outer": [{"kind": "arc", "tag": "e1", "a": [10, 0], "b": [10, -2.4492935982947065e-15]}], "holes": [[{"kind": "arc", "tag": "e2", "a": [5, -1.2246467991473533e-15], "b": [5, 0]}]]}, {"outer": [{"kind": "arc", "tag": "e3", "a": [2, 0], "b": [2, -4.898587196589413e-16]}], "holes": []}], "open": false}"#,
        );
    }

    #[test]
    fn oracle_two_circles() {
        check(
            r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":6,"y":0}],"entities":[{"type":"circle","id":"e1","c":"a","r":5},{"type":"circle","id":"e2","c":"b","r":5}]}"#,
            r#"{"regions": [{"outer": [{"kind": "arc", "tag": "e1", "a": [3, 4], "b": [2.9999999999999987, -4.000000000000001]}, {"kind": "arc", "tag": "e2", "a": [2.9999999999999996, -3.9999999999999996], "b": [3.000000000000001, 4.000000000000001]}], "holes": []}, {"outer": [{"kind": "arc", "tag": "e1", "a": [2.9999999999999987, -4.000000000000001], "b": [3.000000000000001, 3.9999999999999996]}, {"kind": "arc", "tag": "e2", "a": [3.000000000000001, 4.000000000000001], "b": [2.9999999999999996, -3.9999999999999996]}], "holes": []}, {"outer": [{"kind": "arc", "tag": "e1", "a": [3.000000000000001, 3.9999999999999996], "b": [2.9999999999999987, -4.000000000000001]}, {"kind": "arc", "tag": "e2", "a": [2.9999999999999996, -3.9999999999999996], "b": [3.0000000000000018, 4.000000000000002]}], "holes": []}], "open": false}"#,
        );
    }

    #[test]
    fn oracle_crossing_line_and_diagonal() {
        let pts = format!(r#"{SQ},{{"id":"e","x":5,"y":-5}},{{"id":"f","x":5,"y":15}}"#);
        check(
            &format!(r#"{{"points":[{pts}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"e","p2":"f"}}]}}"#),
            r#"{"regions": [{"outer": [{"kind": "line", "tag": "e1", "a": [0, 0], "b": [5, 0]}, {"kind": "line", "tag": "e5", "a": [5, 0], "b": [5, 10]}, {"kind": "line", "tag": "e3", "a": [5, 10], "b": [0, 10]}, {"kind": "line", "tag": "e4", "a": [0, 10], "b": [0, 0]}], "holes": []}, {"outer": [{"kind": "line", "tag": "e1", "a": [5, 0], "b": [10, 0]}, {"kind": "line", "tag": "e2", "a": [10, 0], "b": [10, 10]}, {"kind": "line", "tag": "e3", "a": [10, 10], "b": [5, 10]}, {"kind": "line", "tag": "e5", "a": [5, 10], "b": [5, 0]}], "holes": []}], "open": false}"#,
        );
        check(
            &format!(r#"{{"points":[{SQ}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"a","p2":"c"}}]}}"#),
            r#"{"regions": [{"outer": [{"kind": "line", "tag": "e1", "a": [0, 0], "b": [10, 0]}, {"kind": "line", "tag": "e2", "a": [10, 0], "b": [10, 10]}, {"kind": "line", "tag": "e5", "a": [10, 10], "b": [0, 0]}], "holes": []}, {"outer": [{"kind": "line", "tag": "e3", "a": [10, 10], "b": [0, 10]}, {"kind": "line", "tag": "e4", "a": [0, 10], "b": [0, 0]}, {"kind": "line", "tag": "e5", "a": [0, 0], "b": [10, 10]}], "holes": []}], "open": false}"#,
        );
    }

    #[test]
    fn oracle_dangling_duplicates_overlaps() {
        let full = r#"{"regions": [{"outer": [{"kind": "line", "tag": "e1", "a": [0, 0], "b": [10, 0]}, {"kind": "line", "tag": "e2", "a": [10, 0], "b": [10, 10]}, {"kind": "line", "tag": "e3", "a": [10, 10], "b": [0, 10]}, {"kind": "line", "tag": "e4", "a": [0, 10], "b": [0, 0]}], "holes": []}], "open": false}"#;
        // dangling spur into the interior
        check(&format!(r#"{{"points":[{SQ},{{"id":"e","x":5,"y":5}}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"a","p2":"e"}}]}}"#), full);
        // duplicate of e1
        check(&format!(r#"{{"points":[{SQ}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"a","p2":"b"}}]}}"#), full);
        // collinear half-overlap: not split
        check(&format!(r#"{{"points":[{SQ},{{"id":"m","x":5,"y":0}}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"m","p2":"b"}}]}}"#), full);
        // detached line next to a closed loop
        check(&format!(r#"{{"points":[{SQ},{{"id":"m","x":20,"y":20}},{{"id":"n","x":30,"y":20}}],"entities":[{RECT},{{"type":"line","id":"e5","p1":"m","p2":"n"}}]}}"#), full);
    }

    #[test]
    fn oracle_ellipse_cvspline_closed_spline() {
        check(
            r#"{"points":[{"id":"c","x":0,"y":0},{"id":"p","x":10,"y":0}],"entities":[{"type":"ellipse","id":"e1","c":"c","p":"p","r":4}]}"#,
            r#"{"regions": [{"outer": [{"kind": "arc", "tag": "e1", "a": [10, 0], "b": [10, -9.797174393178826e-16]}], "holes": []}], "open": false}"#,
        );
        check(
            r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":10},{"id":"c","x":20,"y":0}],"entities":[{"type":"cvspline","id":"e1","pts":["a","b","c"],"degree":2},{"type":"line","id":"e2","p1":"c","p2":"a"}]}"#,
            r#"{"regions": [{"outer": [{"kind": "spline", "tag": "e1", "a": [20, 0], "b": [0, 0]}, {"kind": "line", "tag": "e2", "a": [0, 0], "b": [20, 0]}], "holes": []}], "open": false}"#,
        );
        check(
            r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":10},{"id":"c","x":20,"y":0}],"entities":[{"type":"spline","id":"e1","pts":["a","b","c"],"closed":true}]}"#,
            r#"{"regions": [{"outer": [{"kind": "spline", "tag": "e1", "a": [19.571067811865476, 3.693980625181293], "b": [0, 0]}, {"kind": "spline", "tag": "e1", "a": [0, 0], "b": [19.571067811865476, 3.693980625181293]}], "holes": []}], "open": false}"#,
        );
        check(
            r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":10},{"id":"c","x":20,"y":0},{"id":"d","x":10,"y":-5},{"id":"e","x":10,"y":20}],"entities":[{"type":"spline","id":"e1","pts":["a","b","c"]},{"type":"line","id":"e2","p1":"c","p2":"a"},{"type":"line","id":"e3","p1":"d","p2":"e"}]}"#,
            r#"{"regions": [{"outer": [{"kind": "spline", "tag": "e1", "a": [10, 10], "b": [0, 0]}, {"kind": "line", "tag": "e2", "a": [0, 0], "b": [10, 0]}, {"kind": "line", "tag": "e3", "a": [10, 0], "b": [10, 10]}], "holes": []}, {"outer": [{"kind": "spline", "tag": "e1", "a": [20, 0], "b": [10, 10]}, {"kind": "line", "tag": "e3", "a": [10, 10], "b": [10, 0]}, {"kind": "line", "tag": "e2", "a": [10, 0], "b": [20, 0]}], "holes": []}], "open": false}"#,
        );
    }

    /// Oracle: a solved rectangle; the kernel's segment ends differ from the
    /// points in the last ulps (TODO: bit-exact piece evaluation).
    #[test]
    fn oracle_line_param_rounding() {
        check(
            r#"{"points":[{"id":"p1","x":-9.499999902516906,"y":0.50000009027764},{"id":"p2","x":20.500000097511588,"y":0.50000009027764},{"id":"p3","x":20.500000097513997,"y":5.499999999894265},{"id":"p4","x":-9.499999902526271,"y":5.499999999894264}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}]}"#,
            r#"{"regions": [{"outer": [{"kind": "line", "tag": "e1", "a": [-9.499999902516906, 0.50000009027764], "b": [20.500000097511588, 0.50000009027764]}, {"kind": "line", "tag": "e2", "a": [20.500000097511588, 0.50000009027764], "b": [20.500000097513997, 5.499999999894264]}, {"kind": "line", "tag": "e3", "a": [20.500000097513997, 5.499999999894265], "b": [-9.499999902526266, 5.499999999894264]}, {"kind": "line", "tag": "e4", "a": [-9.499999902526271, 5.499999999894264], "b": [-9.499999902516906, 0.5000000902776405]}], "holes": []}], "open": false}"#,
        );
    }
}
