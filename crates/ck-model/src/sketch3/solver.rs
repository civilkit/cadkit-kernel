//! The 3D sketch constraint solver.
//!
//! Port of the client's 3D system builder `Jo()` (app.pretty.js l.11667)
//! with helpers `wt` (the interpolating spline with tangent end conditions),
//! `Ru` / `Yo` (the other-entity / onCurve curve evaluators), `Ds` (the
//! `ref` entity's rational B-spline), `Zo` (arc frames) and `Ls`/`Ns`/`ji`
//! (shared with the 2D solver via `sketch::solver` / `sketch::smooth`).
//!
//! The kernel runs the same solve when a constraint's `expr` changes its
//! `value` (regen, `sketch3d` features). Curve lengths are plain
//! `sqrt(x²+y²+z²)` — NOT the correctly-rounded `hypot` of the 2D distance
//! constraints (probe-pinned). See `tests/sketch3_golden.rs`.

use std::collections::HashMap;

use super::{Constraint3, Entity3, Point3, Sketch3};
use crate::sketch::spline::{bs_def, gauss_jordan, Bspline};
use crate::sketch::smooth::ns3;
use crate::sketch::solver::{
    as_1250, cross3, ls_kernel, Equation, SolveOpts, SolveResult, System, V,
};

const DN: f64 = 1e-12;

/// A solver quantity: unknown slot `i >= 0`, or the constant `c` (3D).
#[derive(Debug, Clone, Copy)]
pub struct V3 {
    pub i: isize,
    pub c: f64,
}

impl V3 {
    #[inline]
    pub fn val(&self, x: &[f64]) -> f64 {
        if self.i >= 0 {
            x[self.i as usize]
        } else {
            self.c
        }
    }
}

/// A resolved 3D point (three solver quantities).
#[derive(Debug, Clone, Copy)]
pub struct P3 {
    pub x: V3,
    pub y: V3,
    pub z: V3,
}

impl P3 {
    #[inline]
    pub fn at(&self, x: &[f64]) -> [f64; 3] {
        [self.x.val(x), self.y.val(x), self.z.val(x)]
    }
    fn konst(c: [f64; 3]) -> P3 {
        P3 { x: V3 { i: -1, c: c[0] }, y: V3 { i: -1, c: c[1] }, z: V3 { i: -1, c: c[2] } }
    }
    fn vars(&self) -> Vec<V3> {
        vec![self.x, self.y, self.z]
    }
}

// ---------------------------------------------------------------------------
// vector helpers (client `O`/`Pe`/`B`/`j`/`ce`/`ke` — plain sqrt)

pub fn ce(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
pub fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
/// `ke`: plain-sqrt point distance (client `ke`).
pub fn ke(a: [f64; 3], b: [f64; 3]) -> f64 {
    ce(sub3(a, b))
}
/// Least-absolute-component axis (client `Au`/`bo`).
pub fn au3(v: [f64; 3]) -> [f64; 3] {
    let e = [v[0].abs(), v[1].abs(), v[2].abs()];
    if e[0] <= e[1] && e[0] <= e[2] {
        [1.0, 0.0, 0.0]
    } else if e[1] <= e[2] {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    }
}
/// The two perpendicular residuals of `r` against the plane spanned by
/// `e` and `axis` (client `jo`).
fn jo(r: [f64; 3], e: [f64; 3], axis: [f64; 3]) -> [f64; 2] {
    let n = cross3(e, axis);
    let s = cross3(e, n);
    [dot3(r, n) / (ce(n) + DN), dot3(r, s) / (ce(s) + DN)]
}

// ---------------------------------------------------------------------------
// the interpolating spline with tangent end conditions (client `wt`)

/// Optional unit end tangents (client `$n`): from `tangent` constraints on
/// open splines whose other ref is a line.
/// A tangent end condition captured at build time (client `$n`): the unit
/// tangent at the spline's start (`0`) or end (`1`) end points along
/// `a - b` (the line's contact endpoint minus the far endpoint), resolved
/// from the current values at every residual evaluation.
#[derive(Debug, Clone)]
pub struct TangentSpec {
    pub at_start: bool,
    pub a: P3,
    pub b: P3,
}

/// Collect the tangent specs of `spline_id` from the sketch's `tangent`
/// constraints (client `$n`).
fn tangent_specs(sk: &Sketch3, spline_id: &str, first: &str, last: &str) -> Vec<TangentSpec> {
    let mut out = Vec::new();
    for c in &sk.constraints {
        if c.kind != "tangent" || c.refs.len() != 2 || !c.refs.iter().any(|r| r == spline_id) {
            continue;
        }
        let other = if c.refs[0] == spline_id { &c.refs[1] } else { &c.refs[0] };
        let Some(Entity3::Line { p1, p2, .. }) = sk.entity(other) else { continue };
        for (l, r) in [(p1.as_str(), p2.as_str()), (p2.as_str(), p1.as_str())] {
            let (Ok(a), Ok(b)) = (b3_of_point_unchecked(sk, l), b3_of_point_unchecked(sk, r)) else {
                continue;
            };
            let at_start = if l == first {
                true
            } else if l == last {
                false
            } else {
                continue;
            };
            out.push(TangentSpec { at_start, a, b });
        }
    }
    out
}

/// A point constant (no solver dependency) for the tangent setup lookup.
fn b3_of_point_unchecked(sk: &Sketch3, id: &str) -> Result<P3, ()> {
    let p = sk.point(id).ok_or(())?;
    Ok(P3 { x: V3 { i: -1, c: p.x }, y: V3 { i: -1, c: p.y }, z: V3 { i: -1, c: p.z } })
}

/// Resolve the tangent end conditions at the current values (client `$n`):
/// the later specs overwrite the earlier ones.
fn resolve_tangents(specs: &[TangentSpec], x: &[f64]) -> (Option<[f64; 3]>, Option<[f64; 3]>) {
    let mut start = None;
    let mut end = None;
    for t in specs {
        let v = sub3(t.a.at(x), t.b.at(x));
        if t.at_start {
            start = Some(v);
        } else {
            end = Some(v);
        }
    }
    (start, end)
}

/// The interpolating spline with optional unit end tangents (client `wt`).
#[derive(Debug, Clone)]
pub struct Wt {
    pub p: Vec<[f64; 3]>,
    pub closed: bool,
    pub h: Vec<f64>,
    pub knots: Vec<f64>,
    pub m: Vec<Vec<f64>>,
}

impl Wt {
    pub fn new(pts: &[[f64; 3]], closed: bool, tangents: (Option<[f64; 3]>, Option<[f64; 3]>)) -> Option<Wt> {
        let mut s: Vec<[f64; 3]> = Vec::new();
        for (i, p) in pts.iter().enumerate() {
            if i == 0 || ke(*p, s[s.len() - 1]) > 1e-9 {
                s.push(*p);
            }
        }
        if closed && s.len() > 2 && ke(s[0], s[s.len() - 1]) < 1e-12 {
            s.pop();
        }
        let closed = closed && s.len() > 2;
        let n = s.len();
        let spans = if closed { n } else { n - 1 };
        let h: Vec<f64> = (0..spans)
            .map(|i| {
                let (a, b) = (s[i], s[(i + 1) % n]);
                ke(b, a).max(1e-12)
            })
            .collect();
        let mut knots = vec![0.0];
        for l in &h {
            knots.push(knots[knots.len() - 1] + l);
        }
        let unit = |t: Option<[f64; 3]>| -> Option<[f64; 3]> {
            match t {
                Some(t) if ce(t) > 1e-12 && !closed => Some(scale3(t, 1.0 / ce(t))),
                _ => None,
            }
        };
        let (ts, te) = (unit(tangents.0), unit(tangents.1));
        let m = (0..3)
            .map(|k| {
                if n == 2 && !closed && ts.is_none() && te.is_none() {
                    return vec![0.0, 0.0];
                }
                let mut a: Vec<Vec<f64>> = vec![vec![0.0; n]; n];
                let mut u = vec![0.0; n];
                for p in 0..n {
                    if !closed && p == 0 {
                        if let Some(l) = ts {
                            a[0][0] = 2.0 * h[0];
                            a[0][1] = h[0];
                            u[0] = 6.0 * ((s[1][k] - s[0][k]) / h[0] - l[k]);
                            continue;
                        }
                    }
                    if !closed && p == n - 1 {
                        if let Some(c) = te {
                            let y = h[n - 2];
                            a[p][p - 1] = y;
                            a[p][p] = 2.0 * y;
                            u[p] = 6.0 * (c[k] - (s[p][k] - s[p - 1][k]) / y);
                            continue;
                        }
                    }
                    if !closed && (p == 0 || p == n - 1) {
                        a[p][p] = 1.0;
                        continue;
                    }
                    let g = (p + n - 1) % n;
                    let q = (p + 1) % n;
                    let (b, w) = (h[g], h[p]);
                    a[p][g] += b;
                    a[p][p] += 2.0 * (b + w);
                    a[p][q] += w;
                    u[p] = 6.0 * ((s[q][k] - s[p][k]) / w - (s[p][k] - s[g][k]) / b);
                }
                gauss_jordan(&a, &u)
            })
            .collect();
        Some(Wt { p: s, closed, h, knots, m })
    }

    pub fn valid(&self) -> bool {
        self.p.len() >= 2
    }

    pub fn length(&self) -> f64 {
        self.knots[self.knots.len() - 1]
    }

    fn locate(&self, e: f64) -> usize {
        let mut n = 0usize;
        while n + 1 < self.h.len() && self.knots[n + 1] <= e {
            n += 1;
        }
        n
    }

    pub fn eval(&self, e: f64) -> [f64; 3] {
        let t = self.length();
        let e = e.max(0.0).min(t);
        let n = self.locate(e);
        let s = (n + 1) % self.p.len();
        let i = self.h[n];
        let o = e - self.knots[n];
        let a = i - o;
        let mut out = [0.0; 3];
        for k in 0..3 {
            let (c, hh) = (self.m[k][n], self.m[k][s]);
            out[k] = (c * a * a * a + hh * o * o * o) / (6.0 * i)
                + (self.p[n][k] / i - (c * i) / 6.0) * a
                + (self.p[s][k] / i - (hh * i) / 6.0) * o;
        }
        out
    }

    pub fn at(&self, e: f64, k: u32) -> [f64; 3] {
        if k == 0 {
            return self.eval(e);
        }
        let t = self.length();
        let e = e.max(0.0).min(t);
        let n = self.locate(e);
        let s = (n + 1) % self.p.len();
        let i = self.h[n];
        let o = e - self.knots[n];
        let a = i - o;
        let mut out = [0.0; 3];
        for c in 0..3 {
            let (c0, c1) = (self.m[c][n], self.m[c][s]);
            out[c] = match k {
                3 => (c1 - c0) / i,
                2 => (c0 * a + c1 * o) / i,
                _ => {
                    (-c0 * a * a + c1 * o * o) / (2.0 * i)
                        - (self.p[n][c] / i - (c0 * i) / 6.0)
                        + (self.p[s][c] / i - (c1 * i) / 6.0)
                }
            };
        }
        out
    }
}

// ---------------------------------------------------------------------------
// the `ref` entity's rational curve (client `Ds`)

/// A rational B-spline curve built from `{P, w?, p, knots}` data.
#[derive(Debug, Clone)]
pub struct Ds {
    s: Bspline<4>,
}

impl Ds {
    pub fn new(j: &ck_json::Json) -> Option<Ds> {
        let p = j.get("P")?.as_arr()?;
        if p.len() < 2 {
            return None;
        }
        let pts: Vec<[f64; 3]> = p
            .iter()
            .map(|q| {
                let q = q.as_arr()?;
                Some([q.get(0)?.as_f64()?, q.get(1)?.as_f64()?, q.get(2)?.as_f64()?])
            })
            .collect::<Option<Vec<_>>>()?;
        let degree = j.get("p").and_then(|d| d.as_f64());
        let knots = j.get("knots")?.as_arr()?.iter().filter_map(|x| x.as_f64()).collect::<Vec<f64>>();
        // w: used only when present with the right length, else ones
        let w = j
            .get("w")
            .and_then(|w| w.as_arr())
            .filter(|a| a.len() == pts.len())
            .map(|a| a.iter().filter_map(|x| x.as_f64()).collect::<Vec<f64>>())
            .unwrap_or_else(|| vec![1.0; pts.len()]);
        let hom: Vec<[f64; 4]> = pts
            .iter()
            .zip(&w)
            .map(|(q, wi)| [q[0] * wi, q[1] * wi, q[2] * wi, *wi])
            .collect();
        let def = bs_def(hom.len(), degree, Some(&knots), false)?;
        Some(Ds { s: Bspline::new(&def, &hom) })
    }

    /// Dehomogenised derivatives (client `Ds.derivs`).
    pub fn derivs(&self, e: f64, t: usize) -> Vec<[f64; 3]> {
        let n = self.s.derivs(e, t);
        let l: [[&[f64]; 4]; 4] = [[&[1.0], &[1.0], &[1.0], &[1.0]], [&[1.0], &[1.0, 1.0], &[1.0], &[1.0]], [&[1.0], &[1.0], &[1.0, 2.0], &[1.0]], [&[1.0], &[1.0], &[1.0], &[1.0, 3.0, 3.0, 1.0]]];
        let mut a: Vec<[f64; 3]> = Vec::with_capacity(t + 1);
        for c in 0..=t {
            let mut hh = [n[c][0], n[c][1], n[c][2]];
            for d in 1..=c {
                for k in 0..3 {
                    hh[k] += a[c - d][k] * (-l[c][d][0] * n[d][3]);
                }
            }
            a.push(scale3(hh, 1.0 / n[0][3]));
        }
        a
    }

    pub fn at(&self, e: f64, t: usize) -> [f64; 3] {
        self.derivs(e, t)[t]
    }

    pub fn length(&self) -> f64 {
        self.s.length()
    }
}

// ---------------------------------------------------------------------------
// the arc frame (client `Zo`)

pub struct ArcFrame {
    pub c: [f64; 3],
    pub r: f64,
    pub u: [f64; 3],
    pub v: [f64; 3],
    pub angle: f64,
}

pub fn arc_frame(c: [f64; 3], p1: [f64; 3], p2: [f64; 3]) -> Option<ArcFrame> {
    let n = sub3(p1, c);
    let s = sub3(p2, c);
    let i = ce(n);
    let o = ce(s);
    if i < 1e-9 || o < 1e-9 {
        return None;
    }
    let a = cross3(n, s);
    let l = ce(a);
    if l <= 1e-9 * i * o {
        return None;
    }
    let nn = scale3(a, 1.0 / l);
    let uu = scale3(n, 1.0 / i);
    Some(ArcFrame { c, r: i, u: uu, v: cross3(nn, uu), angle: l.atan2(dot3(n, s)) })
}

/// Closest parameter on a curve (client `ar`, 96 samples + 30 Newton steps).
fn ar_closest3(
    at: &dyn Fn(f64, u32) -> [f64; 3],
    q: [f64; 3],
    t0: f64,
    t1: f64,
    closed: bool,
) -> f64 {
    let mut i = t0;
    let mut best = f64::INFINITY;
    for a in 0..=96usize {
        let l = t0 + ((t1 - t0) * a as f64) / 96.0;
        let c = ke(at(l, 0), q);
        if c < best {
            best = c;
            i = l;
        }
    }
    for _ in 0..30 {
        let l = sub3(at(i, 0), q);
        let c = at(i, 1);
        let hh = at(i, 2);
        let d = dot3(c, c) + dot3(l, hh);
        if !(d.abs() > 1e-300) {
            break;
        }
        let u = dot3(l, c) / d;
        i -= u;
        if !closed {
            i = i.max(t0.min(t1)).min(t0.max(t1));
        }
        if u.abs() < 1e-13 * (1.0 + (t1 - t0).abs()) {
            break;
        }
    }
    i
}

// ---------------------------------------------------------------------------
// the builder

struct B3<'a> {
    sk: &'a Sketch3,
    key_idx: HashMap<String, usize>,
    x: Vec<f64>,
    eqs: Vec<Equation>,
}

#[derive(Clone)]
enum Ent3 {
    Line { a: P3, b: P3 },
    Arc { c: P3, p1: P3, p2: P3 },
    Spline { ids: Vec<String>, pts: Vec<P3>, closed: bool },
    CvSpline { pts: Vec<P3>, degree: f64, knots: Option<Vec<f64>>, closed: bool },
    Point { p: P3 },
    Ref { curve: Option<Ds>, closed: bool, pts: Vec<String> },
}

impl<'a> B3<'a> {
    fn var(&self, key: &str, c: f64) -> V3 {
        match self.key_idx.get(key) {
            Some(&i) => V3 { i: i as isize, c },
            None => V3 { i: -1, c },
        }
    }
    /// Point by id (client `o`).
    fn point(&self, id: &str) -> Result<P3, String> {
        let p = self.sk.point(id).ok_or_else(|| format!("missing point {id}"))?;
        Ok(P3 {
            x: self.var(&format!("{id}.x"), p.x),
            y: self.var(&format!("{id}.y"), p.y),
            z: self.var(&format!("{id}.z"), p.z),
        })
    }
    /// Entity by id (client `c`).
    fn entity(&self, id: &str) -> Result<Ent3, String> {
        let e = self.sk.entity(id).ok_or_else(|| format!("missing entity {id}"))?;
        Ok(match e {
            Entity3::Line { p1, p2, .. } => Ent3::Line { a: self.point(p1)?, b: self.point(p2)? },
            Entity3::Arc { c, p1, p2, .. } => {
                Ent3::Arc { c: self.point(c)?, p1: self.point(p1)?, p2: self.point(p2)? }
            }
            Entity3::Spline { pts, closed, .. } => Ent3::Spline {
                ids: pts.clone(),
                pts: pts.iter().map(|p| self.point(p)).collect::<Result<_, _>>()?,
                closed: *closed,
            },
            Entity3::CvSpline { pts, degree, knots, closed, .. } => Ent3::CvSpline {
                pts: pts.iter().map(|p| self.point(p)).collect::<Result<_, _>>()?,
                degree: (*degree).max(1) as f64,
                knots: knots.clone(),
                closed: *closed,
            },
            Entity3::Point { p, .. } => Ent3::Point { p: self.point(p)? },
            Entity3::Ref { curve, closed, pts, .. } => Ent3::Ref {
                curve: curve.as_ref().and_then(Ds::new),
                closed: *closed,
                pts: pts.clone(),
            },
        })
    }
    /// Point-like reference: a sketch point, a point entity, or an arc centre
    /// (client `d`). NOTE: circles/ellipses are not 3D entities.
    fn pointish(&self, id: &str) -> Result<P3, String> {
        if self.sk.point(id).is_some() {
            return self.point(id);
        }
        match self.sk.entity(id) {
            Some(Entity3::Point { p, .. }) => self.point(p),
            Some(Entity3::Arc { c, .. }) => self.point(c),
            _ => Err(format!("not a point {id}")),
        }
    }
    /// Both endpoints of a line (client `h`).
    fn line(&self, id: &str) -> Result<(P3, P3), String> {
        match self.entity(id)? {
            Ent3::Line { a, b } => Ok((a, b)),
            _ => Err(format!("not a line {id}")),
        }
    }
    fn eq(&mut self, cid: &str, vars: &[V3], f: Box<dyn Fn(&[f64]) -> f64>) {
        let mut deps = Vec::new();
        for v in vars {
            if v.i >= 0 && !deps.contains(&(v.i as usize)) {
                deps.push(v.i as usize);
            }
        }
        self.eqs.push(Equation { cid: cid.to_string(), deps, f });
    }
}

/// The std plane normals (client `sc`).
fn std_plane(id: &str) -> Option<[f64; 3]> {
    match id {
        "front" => Some([0.0, 0.0, 1.0]),
        "top" => Some([0.0, 1.0, 0.0]),
        "right" => Some([1.0, 0.0, 0.0]),
        _ => None,
    }
}

/// The `onPlane` plane (client `Du`): a std id or `{o, n}` (n normalised).
fn du(j: Option<&ck_json::Json>) -> Option<([f64; 3], [f64; 3])> {
    let j = j?;
    if let Some(s) = j.as_str() {
        return std_plane(s).map(|n| ([0.0; 3], n));
    }
    let oa = j.get("o")?.as_arr()?;
    let na = j.get("n")?.as_arr()?;
    let o = [oa.get(0)?.as_f64()?, oa.get(1)?.as_f64()?, oa.get(2)?.as_f64()?];
    let n = [na.get(0)?.as_f64()?, na.get(1)?.as_f64()?, na.get(2)?.as_f64()?];
    let l = ce(n);
    let n = if l < 1e-300 { [0.0; 3] } else { scale3(n, 1.0 / l) };
    Some((o, n))
}

fn refn<'b>(refs: &'b [String], i: usize) -> Result<&'b str, String> {
    refs.get(i).map(|s| s.as_str()).ok_or_else(|| "missing ref".to_string())
}

/// Point ids the entity is built on, plus tangent-line endpoints for
/// splines (client `Lu`).
fn lu_deps(b: &B3, id: &str) -> Result<Vec<P3>, String> {
    let e = b.sk.entity(id).ok_or_else(|| format!("missing entity {id}"))?;
    let mut ids: Vec<&str> = e.point_ids();
    if let Entity3::Spline { .. } = e {
        for c in &b.sk.constraints {
            if c.kind == "tangent" && c.refs.iter().any(|r| r == id) {
                for r in &c.refs {
                    if let Some(Entity3::Line { p1, p2, .. }) = b.sk.entity(r) {
                        ids.push(p1);
                        ids.push(p2);
                    }
                }
            }
        }
    }
    ids.iter().map(|p| b.point(p)).collect()
}

// ---------------------------------------------------------------------------
// the other-entity derivative provider (client `Ru`)

#[derive(Clone)]
enum Ru3 {
    Line { a: P3, b: P3 },
    Arc { c: P3, p1: P3, p2: P3 },
    Ref { curve: Option<Ds>, closed: bool, at_start: Option<bool> },
    Spline { pts: Vec<P3>, closed: bool, end: Option<usize>, tangents: Vec<TangentSpec> },
    CvSpline { pts: Vec<P3>, degree: f64, knots: Option<Vec<f64>>, closed: bool, end: Option<usize> },
}

impl Ru3 {
    fn deps(&self) -> Vec<V3> {
        match self {
            Ru3::Line { a, b } => [a.vars(), b.vars()].concat(),
            Ru3::Arc { c, p1, p2 } => [c.vars(), p1.vars(), p2.vars()].concat(),
            Ru3::Ref { .. } => vec![],
            Ru3::Spline { pts, .. } | Ru3::CvSpline { pts, .. } => {
                pts.iter().flat_map(|p| p.vars()).collect()
            }
        }
    }

    fn derivs(&self, x: &[f64], h: [f64; 3]) -> Option<[[f64; 3]; 4]> {
        let zero = [0.0f64; 3];
        match self {
            Ru3::Line { a, b } => {
                let (pa, pb) = (a.at(x), b.at(x));
                Some([h, sub3(pb, pa), zero, zero])
            }
            Ru3::Arc { c, p1, p2 } => {
                let d = arc_frame(c.at(x), p1.at(x), p2.at(x))?;
                let u = sub3(h, d.c);
                let m = dot3(u, d.v).atan2(dot3(u, d.u));
                let (co, si) = (m.cos(), m.sin());
                let k = [
                    d.u[0] * co + d.v[0] * si,
                    d.u[1] * co + d.v[1] * si,
                    d.u[2] * co + d.v[2] * si,
                ];
                let b = [
                    d.u[0] * -si + d.v[0] * co,
                    d.u[1] * -si + d.v[1] * co,
                    d.u[2] * -si + d.v[2] * co,
                ];
                Some([
                    [d.c[0] + k[0] * d.r, d.c[1] + k[1] * d.r, d.c[2] + k[2] * d.r],
                    scale3(b, d.r),
                    scale3(k, -d.r),
                    scale3(b, -d.r),
                ])
            }
            Ru3::Ref { curve, closed, at_start } => {
                let o = curve.as_ref()?;
                let tt = match *at_start {
                    Some(true) => 0.0,
                    Some(false) => o.length(),
                    None => ar_closest3(&|t, k| o.at(t, k as usize), h, 0.0, o.length(), *closed),
                };
                Some([o.at(tt, 0), o.at(tt, 1), o.at(tt, 2), o.at(tt, 3)])
            }
            Ru3::Spline { pts, closed, end, tangents } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Wt::new(&vals, *closed, resolve_tangents(tangents, x))?;
                if !u.valid() {
                    return None;
                }
                let m = contact_64(&|t, k| u.at(t, k as u32), *end, u.length(), h);
                Some([u.at(m, 0), u.at(m, 1), u.at(m, 2), u.at(m, 3)])
            }
            Ru3::CvSpline { pts, degree, knots, closed, end } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Bspline::from_curve(&vals, Some(*degree), knots.as_deref(), *closed)?;
                let m = contact_64(&|t, k| u.at(t, k as usize), *end, u.length(), h);
                Some([u.at(m, 0), u.at(m, 1), u.at(m, 2), u.at(m, 3)])
            }
        }
    }
}

/// The contact parameter of a spline-like other curve (client `Ru`'s `m`):
/// an end when shared, else a 65-sample search + 30 clamped Newton steps.
fn contact_64(at: &dyn Fn(f64, u32) -> [f64; 3], end: Option<usize>, length: f64, h: [f64; 3]) -> f64 {
    match end {
        Some(0) => 0.0,
        Some(1) => length,
        _ => {
            let mut m = 0.0f64;
            let mut best = f64::INFINITY;
            for g in 0..=64usize {
                let k = length * g as f64 / 64.0;
                let b = ke(at(k, 0), h);
                if b < best {
                    best = b;
                    m = k;
                }
            }
            for _ in 0..30 {
                let k = sub3(at(m, 0), h);
                let b = at(m, 1);
                let w = at(m, 2);
                let y = dot3(b, b) + dot3(k, w);
                if !(y.abs() > 1e-300) {
                    break;
                }
                let s = dot3(k, b) / y;
                m = (m - s).max(0.0).min(length);
                if s.abs() < 1e-13 * (1.0 + length) {
                    break;
                }
            }
            m
        }
    }
}

// ---------------------------------------------------------------------------
// the onCurve curve wrapper (client `Yo`)

#[derive(Clone)]
enum Yo3 {
    Line { a: P3, b: P3 },
    Arc { c: P3, p1: P3, p2: P3 },
    Spline { pts: Vec<P3>, closed: bool, tangents: Vec<TangentSpec> },
    CvSpline { pts: Vec<P3>, degree: f64, knots: Option<Vec<f64>>, closed: bool },
    Ref { curve: Option<Ds>, closed: bool },
}

impl Yo3 {
    /// `(t0, t1)`; `None` = the curve does not exist (degenerate geometry).
    fn span(&self, x: &[f64]) -> Option<(f64, f64)> {
        match self {
            Yo3::Line { a, b } => {
                let c = ke(a.at(x), b.at(x));
                if !(c > 1e-9) {
                    return None;
                }
                Some((0.0, c))
            }
            Yo3::Arc { c, p1, p2 } => {
                let d = arc_frame(c.at(x), p1.at(x), p2.at(x))?;
                Some((0.0, d.angle))
            }
            Yo3::Spline { pts, closed, tangents } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Wt::new(&vals, *closed, resolve_tangents(tangents, x))?;
                if !u.valid() {
                    return None;
                }
                Some((0.0, u.length()))
            }
            Yo3::CvSpline { pts, degree, knots, closed } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Bspline::from_curve(&vals, Some(*degree), knots.as_deref(), *closed)?;
                Some((0.0, u.length()))
            }
            Yo3::Ref { curve, .. } => {
                let o = curve.as_ref()?;
                Some((0.0, o.length()))
            }
        }
    }

    fn at(&self, x: &[f64], t: f64) -> Option<[f64; 3]> {
        match self {
            Yo3::Line { a, b } => {
                let (pa, pb) = (a.at(x), b.at(x));
                let c = ke(pa, pb);
                if !(c > 1e-9) {
                    return None;
                }
                let h = scale3(sub3(pb, pa), 1.0 / c);
                Some([pa[0] + h[0] * t, pa[1] + h[1] * t, pa[2] + h[2] * t])
            }
            Yo3::Arc { c, p1, p2 } => {
                let d = arc_frame(c.at(x), p1.at(x), p2.at(x))?;
                let (co, si) = (t.cos(), t.sin());
                Some([
                    d.c[0] + d.u[0] * d.r * co + d.v[0] * d.r * si,
                    d.c[1] + d.u[1] * d.r * co + d.v[1] * d.r * si,
                    d.c[2] + d.u[2] * d.r * co + d.v[2] * d.r * si,
                ])
            }
            Yo3::Spline { pts, closed, tangents } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Wt::new(&vals, *closed, resolve_tangents(tangents, x))?;
                Some(u.eval(t))
            }
            Yo3::CvSpline { pts, degree, knots, closed } => {
                let vals: Vec<[f64; 3]> = pts.iter().map(|p| p.at(x)).collect();
                let u = Bspline::from_curve(&vals, Some(*degree), knots.as_deref(), *closed)?;
                Some(u.at(t, 0))
            }
            Yo3::Ref { curve, .. } => Some(curve.as_ref()?.at(t, 0)),
        }
    }
}

// ---------------------------------------------------------------------------
// the max-continuity-level finder (client `ji` via the shared smooth helper)

fn yl3(k: &Constraint3, sk: &Sketch3) -> Result<String, ()> {
    let Some(Entity3::CvSpline { pts, closed: false, .. }) =
        sk.entity(k.refs.first().map(|s| s.as_str()).unwrap_or(""))
    else {
        return Err(());
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
    if let Some(e) = sk.entity(other) {
        strings.push(e.type_name().to_string());
        strings.extend(e.point_ids().iter().map(|s| s.to_string()));
    }
    ends.into_iter().find(|e| !e.is_empty() && strings.contains(e)).ok_or(())
}

fn ji3(sk: &Sketch3, spline_id: &str, at: &str) -> u32 {
    let mut s = 0;
    for k in &sk.constraints {
        if !matches!(k.kind.as_str(), "g1" | "g2" | "g3") {
            continue;
        }
        if k.refs.first().map(|s| s.as_str()) != Some(spline_id) {
            continue;
        }
        let o = match yl3(k, sk) {
            Ok(o) => o,
            Err(()) => continue,
        };
        if o == at {
            s = s.max(k.kind.as_bytes().get(1).copied().unwrap_or(b'0').wrapping_sub(b'0') as u32);
        }
    }
    s
}

// ---------------------------------------------------------------------------
// Ru / Yo constructors (client `Ru`/`Yo` + `Lu`)

fn ru(b: &B3, id: &str, end: &str, raw: &Entity3) -> Result<Ru3, String> {
    Ok(match raw {
        Entity3::Line { .. } => match b.entity(id)? {
            Ent3::Line { a, b: b2 } => Ru3::Line { a, b: b2 },
            _ => unreachable!(),
        },
        Entity3::Arc { .. } => match b.entity(id)? {
            Ent3::Arc { c, p1, p2 } => Ru3::Arc { c, p1, p2 },
            _ => unreachable!(),
        },
        Entity3::Ref { curve, closed, pts, .. } => Ru3::Ref {
            curve: curve.as_ref().and_then(Ds::new),
            closed: *closed,
            at_start: if *closed || pts.is_empty() {
                None
            } else if pts[0] == end {
                Some(true)
            } else if pts[pts.len() - 1] == end {
                Some(false)
            } else {
                None
            },
        },
        Entity3::Spline { pts, .. } => {
            let (ids, vals): (Vec<String>, Vec<P3>) = {
                let mut a = Vec::new();
                let mut v = Vec::new();
                for p in pts {
                    a.push(p.clone());
                    v.push(b.point(p)?);
                }
                (a, v)
            };
            let end = if *raw_spline_closed(raw) {
                None
            } else if pts[0] == end {
                Some(0usize)
            } else if pts[pts.len() - 1] == end {
                Some(1usize)
            } else {
                None
            };
            let tangents = tangent_specs(b.sk, id, &ids[0], ids.last().unwrap());
            Ru3::Spline { pts: vals, closed: *raw_spline_closed(raw), end, tangents }
        }
        Entity3::CvSpline { pts, degree, knots, closed, .. } => {
            let vals: Vec<P3> = pts.iter().map(|p| b.point(p)).collect::<Result<_, _>>()?;
            let end = if *closed {
                None
            } else if pts[0] == end {
                Some(0usize)
            } else if pts[pts.len() - 1] == end {
                Some(1usize)
            } else {
                None
            };
            Ru3::CvSpline { pts: vals, degree: (*degree).max(1) as f64, knots: knots.clone(), closed: *closed, end }
        }
        other => return Err(format!("end relation to a {}", other.type_name())),
    })
}

fn raw_spline_closed(raw: &Entity3) -> &bool {
    match raw {
        Entity3::Spline { closed, .. } => closed,
        _ => &false,
    }
}

fn yo(b: &B3, id: &str) -> Result<Yo3, String> {
    let raw = b.sk.entity(id).ok_or("onCurve: not a curve")?;
    Ok(match raw {
        Entity3::Line { .. } => match b.entity(id)? {
            Ent3::Line { a, b: b2 } => Yo3::Line { a, b: b2 },
            _ => unreachable!(),
        },
        Entity3::Arc { .. } => match b.entity(id)? {
            Ent3::Arc { c, p1, p2 } => Yo3::Arc { c, p1, p2 },
            _ => unreachable!(),
        },
        Entity3::Spline { pts, closed, .. } => {
            let (ids, vals): (Vec<String>, Vec<P3>) = {
                let mut a = Vec::new();
                let mut v = Vec::new();
                for p in pts {
                    a.push(p.clone());
                    v.push(b.point(p)?);
                }
                (a, v)
            };
            let tangents = tangent_specs(b.sk, id, &ids[0], ids.last().unwrap());
            Yo3::Spline { pts: vals, closed: *closed, tangents }
        }
        Entity3::CvSpline { pts, degree, knots, closed, .. } => Yo3::CvSpline {
            pts: pts.iter().map(|p| b.point(p)).collect::<Result<_, _>>()?,
            degree: (*degree).max(1) as f64,
            knots: knots.clone(),
            closed: *closed,
        },
        Entity3::Ref { curve, closed, .. } => {
            Yo3::Ref { curve: curve.as_ref().and_then(Ds::new), closed: *closed }
        }
        Entity3::Point { .. } => return Err("onCurve: not a curve".into()),
    })
}

/// Cvspline derivative tuple at the start (`t = 0`) or end (`t = NaN`
/// meaning the domain end) built from the current control point values.
fn eval_cvspline_derivs3(
    x: &[f64],
    cps: &[P3],
    degree: f64,
    knots: &Option<Vec<f64>>,
    closed: bool,
    t: f64,
) -> Option<[[f64; 3]; 4]> {
    let d: Vec<[f64; 3]> = cps.iter().map(|p| p.at(x)).collect();
    let u = Bspline::from_curve(&d, Some(degree), knots.as_deref(), closed)?;
    let t = if t.is_nan() { u.length() } else { t };
    let d = u.derivs(t, 3);
    Some([d[0], d[1], d[2], d[3]])
}

// ---------------------------------------------------------------------------
// build

/// Build the 3D equation system (client `Jo()`). Errors mirror the client's
/// throws; the kernel treats any error as "leave the sketch unsolved".
pub fn build(sk: &Sketch3) -> Result<System, String> {
    let mut x = Vec::new();
    let mut keys = Vec::new();
    let mut key_idx = HashMap::new();
    for p in &sk.points {
        if !p.fixed {
            for k in ["x", "y", "z"] {
                let v = match k {
                    "x" => p.x,
                    "y" => p.y,
                    _ => p.z,
                };
                key_idx.insert(format!("{}.{}", p.id, k), x.len());
                keys.push(format!("{}.{}", p.id, k));
                x.push(v);
            }
        }
    }
    let mut b = B3 { sk, key_idx, x, eqs: Vec::new() };

    // arc radius consistency (client `m("arc:" + g.id, ...)`)
    for e in &sk.entities {
        if let Entity3::Arc { id, .. } = e {
            let Ent3::Arc { c, p1, p2 } = b.entity(id)? else { continue };
            let vars = [c.vars(), p1.vars(), p2.vars()].concat();
            if vars.iter().all(|v| v.i < 0) {
                continue;
            }
            b.eq(
                &format!("arc:{id}"),
                &vars,
                Box::new(move |x| ke(p1.at(x), c.at(x)) - ke(p2.at(x), c.at(x))),
            );
        }
    }

    for k in &sk.constraints {
        let cid = k.id.as_str();
        let e = &k.refs;
        let w = k.value.unwrap_or(f64::NAN);
        match k.kind.as_str() {
            "coincident" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                for (cv, tv) in [(c.x, t.x), (c.y, t.y), (c.z, t.z)] {
                    b.eq(cid, &[cv, tv], Box::new(move |x| val(x, cv) - val(x, tv)));
                }
            }
            "alongX" | "alongY" | "alongZ" => {
                let skip = match k.kind.as_str() {
                    "alongX" => 0usize,
                    "alongY" => 1,
                    _ => 2,
                };
                let (c, t) = if e.len() == 1 {
                    b.line(refn(e, 0)?)?
                } else {
                    (b.pointish(refn(e, 0)?)?, b.pointish(refn(e, 1)?)?)
                };
                for (i, (cv, tv)) in [(c.x, t.x), (c.y, t.y), (c.z, t.z)].into_iter().enumerate() {
                    if i == skip {
                        continue;
                    }
                    b.eq(cid, &[cv, tv], Box::new(move |x| val(x, cv) - val(x, tv)));
                }
            }
            "parallel" => {
                let ((a1, a2), (b1, b2)) = (b.line(refn(e, 0)?)?, b.line(refn(e, 1)?)?);
                let axis = au3(sub3(a2.at(&b.x), a1.at(&b.x)));
                let vars = [a1.vars(), a2.vars(), b1.vars(), b2.vars()].concat();
                for p in 0..2usize {
                    let p = p as u32;
                    b.eq(
                        cid,
                        &vars,
                        Box::new(move |x| {
                            let t = sub3(a2.at(x), a1.at(x));
                            let z = sub3(b2.at(x), b1.at(x));
                            // client: jo(z, T, E)[P] / (ce(z) + Dn)
                            let j = jo(z, t, axis)[p as usize];
                            j / (ce(z) + DN)
                        }),
                    );
                }
            }
            "perpendicular" => {
                let ((a1, a2), (b1, b2)) = (b.line(refn(e, 0)?)?, b.line(refn(e, 1)?)?);
                let vars = [a1.vars(), a2.vars(), b1.vars(), b2.vars()].concat();
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |x| {
                        let p = sub3(a2.at(x), a1.at(x));
                        let c = sub3(b2.at(x), b1.at(x));
                        dot3(p, c) / (ce(p) * ce(c) + DN)
                    }),
                );
            }
            "equal" => {
                let (ya, sa) = (b.sk.entity(refn(e, 0)?), b.sk.entity(refn(e, 1)?));
                let (y, s) = match (&ya, &sa) {
                    (Some(y), Some(s)) => (y.type_name(), s.type_name()),
                    _ => return Err("missing entity".into()),
                };
                if (y == "line") != (s == "line") || !(y == "line" || y == "arc") || !(s == "line" || s == "arc") {
                    return Err("equal: mixed entities".into());
                }
                let ends = |t: &str, id: &str| -> Result<(P3, P3), String> {
                    Ok(match t {
                        "line" => {
                            let (a, b) = b.line(id)?;
                            (a, b)
                        }
                        _ => match b.entity(id)? {
                            Ent3::Arc { c, p1, .. } => (c, p1),
                            _ => return Err("equal: mixed entities".into()),
                        },
                    })
                };
                let (x1, x2) = ends(y, refn(e, 0)?)?;
                let (p1, p2) = ends(s, refn(e, 1)?)?;
                let vars = [x1.vars(), x2.vars(), p1.vars(), p2.vars()].concat();
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |x| ke(x1.at(x), x2.at(x)) - ke(p1.at(x), p2.at(x))),
                );
            }
            "collinear" | "pointOnLine" => {
                let (line_id, pts) = if k.kind == "collinear" {
                    (refn(e, 0)?, vec![b.line(refn(e, 1)?)?])
                } else {
                    (refn(e, 1)?, vec![(b.pointish(refn(e, 0)?)?, P3::konst([0.0; 3]))])
                };
                let (y, s) = b.line(line_id)?;
                let axis = au3(sub3(s.at(&b.x), y.at(&b.x)));
                for (m, _) in pts {
                    let vars = [y.vars(), s.vars(), m.vars()].concat();
                    for p in 0..2usize {
                        b.eq(
                            cid,
                            &vars,
                            Box::new(move |x| {
                                let r = sub3(m.at(x), y.at(x));
                                let t = sub3(s.at(x), y.at(x));
                                jo(r, t, axis)[p]
                            }),
                        );
                    }
                }
            }
            "midpoint" => {
                let y = b.pointish(refn(e, 0)?)?;
                let (s, m) = b.line(refn(e, 1)?)?;
                for (yv, sv, mv) in [(y.x, s.x, m.x), (y.y, s.y, m.y), (y.z, s.z, m.z)] {
                    b.eq(
                        cid,
                        &[yv, sv, mv],
                        Box::new(move |x| val(x, yv) - (val(x, sv) + val(x, mv)) / 2.0),
                    );
                }
            }
            "corner" => {
                let y = b.pointish(refn(e, 0)?)?;
                let (s, m) = b.line(refn(e, 1)?)?;
                let (x1, x2) = b.line(refn(e, 2)?)?;
                let axis = au3(sub3(m.at(&b.x), s.at(&b.x)));
                let vars = [s.vars(), m.vars(), y.vars()].concat();
                for p in 0..2usize {
                    b.eq(
                        cid,
                        &vars,
                        Box::new(move |x| {
                            let r = sub3(y.at(x), s.at(x));
                            let t = sub3(m.at(x), s.at(x));
                            jo(r, t, axis)[p]
                        }),
                    );
                }
                let vars2 = [s.vars(), m.vars(), x1.vars(), x2.vars(), y.vars()].concat();
                b.eq(
                    cid,
                    &vars2,
                    Box::new(move |x| {
                        let t = sub3(m.at(x), s.at(x));
                        let z = sub3(x2.at(x), x1.at(x));
                        let r = cross3(z, cross3(t, z));
                        dot3(sub3(y.at(x), x1.at(x)), r) / (ce(r) + DN)
                    }),
                );
            }
            "fix" => {
                let y = b.pointish(refn(e, 0)?)?;
                let [px, py, pz] = k.point.unwrap_or([f64::NAN; 3]);
                for (yv, t) in [(y.x, px), (y.y, py), (y.z, pz)] {
                    b.eq(cid, &[yv], Box::new(move |x| val(x, yv) - t));
                }
            }
            "onPlane" => {
                let Some((o, n)) = du(k.plane.as_ref()) else {
                    return Err("onPlane: no plane".into());
                };
                let ref0 = refn(e, 0)?;
                let pts: Vec<P3> = if matches!(b.sk.entity(ref0), Some(Entity3::Line { .. })) {
                    let (a, b2) = b.line(ref0)?;
                    vec![a, b2]
                } else {
                    vec![b.pointish(ref0)?]
                };
                for m in pts {
                    let vars = m.vars();
                    b.eq(cid, &vars, Box::new(move |x| dot3(sub3(m.at(x), o), n)));
                }
            }
            "tangent" => {
                let (mut ya, mut sa) = (b.sk.entity(refn(e, 0)?), b.sk.entity(refn(e, 1)?));
                if ya.as_ref().map(|y| y.type_name()) != Some("line") {
                    std::mem::swap(&mut ya, &mut sa);
                }
                let Some(y) = &ya else { return Err("missing entity".into()) };
                let Some(s) = &sa else { return Err("missing entity".into()) };
                if y.type_name() == "line" && s.type_name() == "spline" {
                    let Entity3::Spline { pts, closed: false, .. } = s else { unreachable!() };
                    let ends = [pts.first().map(|s| s.as_str()), pts.last().map(|s| s.as_str())];
                    let Entity3::Line { p1, p2, .. } = y else { unreachable!() };
                    if ![p1.as_str(), p2.as_str()].iter().any(|r| ends.contains(&Some(*r))) {
                        return Err("tangent: no contact point".into());
                    }
                    // no equations — validated only (client `Jo`)
                    continue;
                }
                if y.type_name() != "line" || s.type_name() != "arc" {
                    return Err("tangent: a line and an arc".into());
                }
                let Entity3::Line { p1: y1, p2: y2, .. } = y else { unreachable!() };
                let Entity3::Arc { p1: s1, p2: s2, .. } = s else { unreachable!() };
                let ends = [y1.as_str(), y2.as_str()];
                let Some(m) = ends.iter().find(|r| **r == s1.as_str() || **r == s2.as_str()) else {
                    return Err("tangent: no contact point".into());
                };
                let m = b.point(m)?;
                let (e1, e2) = (b.point(y1)?, b.point(y2)?);
                let (c, t1, t2) = (b.pointish(&s.id())?, b.point(s1)?, b.point(s2)?);
                let vars1 = [e1.vars(), e2.vars(), m.vars(), c.vars()].concat();
                b.eq(
                    cid,
                    &vars1,
                    Box::new(move |x| {
                        let dir = sub3(e2.at(x), e1.at(x));
                        let q = sub3(m.at(x), c.at(x));
                        dot3(dir, q) / (ce(dir) * ce(q) + DN)
                    }),
                );
                let vars2 = [e1.vars(), e2.vars(), c.vars(), t1.vars(), t2.vars()].concat();
                b.eq(
                    cid,
                    &vars2,
                    Box::new(move |x| {
                        let c = c.at(x);
                        let q = sub3(e2.at(x), e1.at(x));
                        let v = cross3(sub3(t1.at(x), c), sub3(t2.at(x), c));
                        dot3(q, v) / (ce(q) * ce(v) + DN)
                    }),
                );
            }
            "g1" | "g2" | "g3" => {
                let level = match k.kind.as_bytes().get(1) {
                    Some(b'1') => 1usize,
                    Some(b'2') => 2,
                    Some(b'3') => 3,
                    _ => return Err(format!("{}: bad level", k.kind)),
                };
                let e0 = refn(e, 0)?;
                let e1 = refn(e, 1)?;
                let s_raw = b.sk.entity(e0).ok_or_else(|| format!("missing entity {e0}"))?;
                let m_raw = b.sk.entity(e1).ok_or_else(|| format!("missing entity {e1}"))?;
                let (pts_ids, degree, knots, closed) = match &s_raw {
                    Entity3::CvSpline { pts, degree, knots, closed, .. } => {
                        if *closed {
                            return Err(format!("{}: not an open control point spline", k.kind));
                        }
                        (pts.clone(), (*degree).max(1) as f64, knots.clone(), *closed)
                    }
                    _ => return Err(format!("{}: not an open control point spline", k.kind)),
                };
                if e1 == e0 {
                    return Err(format!("{}: not an open control point spline", k.kind));
                }
                let ends = [pts_ids.first().cloned().unwrap_or_default(), pts_ids.last().cloned().unwrap_or_default()];
                let end = match &k.at {
                    Some(at) => {
                        if at.is_empty() || !ends.contains(at) {
                            return Err(format!("{}: no end", k.kind));
                        }
                        at.clone()
                    }
                    None => {
                        let ids: Vec<String> = {
                            let mut v: Vec<String> = m_raw.point_ids().iter().map(|s| s.to_string()).collect();
                            v.push(m_raw.type_name().to_string());
                            v.push(e1.to_string());
                            v
                        };
                        match ends.iter().find(|e| !e.is_empty() && ids.contains(e)) {
                            Some(e) => e.clone(),
                            None => return Err(format!("{}: no end", k.kind)),
                        }
                    }
                };
                let at_start = end == pts_ids[0];
                let c_all: Vec<P3> = pts_ids.iter().map(|p| b.point(p)).collect::<Result<_, _>>()?;
                let t_vars: Vec<P3> = if at_start {
                    c_all[..level + 1].to_vec()
                } else {
                    c_all[c_all.len() - level - 1..].to_vec()
                };
                let z = b.point(&end)?;
                let other = ru(&b, e1, &end, &m_raw)?;
                let mut vars: Vec<V3> = t_vars.iter().flat_map(|p| p.vars()).collect();
                vars.extend(z.vars());
                vars.extend(other.deps());
                // build-time constants (client `q`/`V`/`K`)
                let x0 = b.x.clone();
                let dq = ke(t_vars[0].at(&x0), t_vars[1].at(&x0));
                let q = if dq.is_nan() { f64::NAN } else { dq.max(1e-9) };
                let v0 = eval_cvspline_derivs3(&x0, &c_all, degree, &knots, closed, if at_start { 0.0 } else { f64::NAN });
                let l0 = v0.as_ref().map(|d| ce(d[1])).unwrap_or(0.0);
                let kconst = (q, if l0 > 1e-9 { l0 } else { q });
                let axis = au3(sub3(t_vars[1].at(&x0), t_vars[0].at(&x0)));
                for i in 0..2 * level {
                    let c_all = c_all.clone();
                    let z = z;
                    let other = other.clone();
                    let kconst = kconst;
                    let (degree, knots, closed) = (degree, knots.clone(), closed);
                    let (i, level) = (i, level);
                    let at_start = at_start;
                    let axis = axis;
                    b.eq(
                        cid,
                        &vars,
                        Box::new(move |x: &[f64]| {
                            let Some(sder) = eval_cvspline_derivs3(
                                x,
                                &c_all,
                                degree,
                                &knots,
                                closed,
                                if at_start { 0.0 } else { f64::NAN },
                            ) else {
                                return 1.0;
                            };
                            let h = z.at(x);
                            let Some(od) = other.derivs(x, h) else { return 1.0 };
                            if std::env::var("CK_G3DBG").is_ok() {
                                eprintln!("sder={sder:?} od={od:?} axis={axis:?} level={level} i={i}");
                            }
                            ls_kernel(level, kconst.0, kconst.1, &sder, &od, true, axis)[i]
                        }),
                    );
                }
            }
            "onCurve" => {
                let y = b.pointish(refn(e, 0)?)?;
                let curve_id = refn(e, 1)?;
                let raw = b.sk.entity(curve_id).ok_or("onCurve: not a curve")?;
                if matches!(raw, Entity3::Point { .. }) {
                    return Err("onCurve: not a curve".into());
                }
                let curve = yo(&b, curve_id)?;
                let mut vars = y.vars();
                vars.extend(lu_deps(&b, curve_id)?.iter().flat_map(|p| p.vars()));
                for q in 0..3usize {
                    let curve = curve.clone();
                    let y = y;
                    let value = w;
                    let q = q;
                    b.eq(
                        cid,
                        &vars,
                        Box::new(move |x: &[f64]| {
                            let Some((t0, t1)) = curve.span(x) else { return 1.0 };
                            let Some(p) = curve.at(x, t0 + value * (t1 - t0)) else { return 1.0 };
                            y.at(x)[q] - p[q]
                        }),
                    );
                }
            }
            "tension" => {
                let e0 = refn(e, 0)?;
                let raw = b.sk.entity(e0).ok_or("tension: not an open control point spline")?;
                let (pts, degree) = match &raw {
                    Entity3::CvSpline { pts, degree, closed: false, .. } => (pts.clone(), (*degree).max(1) as f64),
                    _ => return Err("tension: not an open control point spline".into()),
                };
                let at = k.at.clone().ok_or("tension: at is no end of the spline")?;
                if at.is_empty() {
                    return Err("tension: at is no end of the spline".into());
                }
                let at_start = at == pts[0];
                if !at_start && pts.last().map(|p| p.as_str()) != Some(at.as_str()) {
                    return Err("tension: at is no end of the spline".into());
                }
                let m = 1usize.max(ji3(b.sk, e0, &at) as usize);
                let ordered: Vec<String> = if at_start { pts.clone() } else { pts.iter().rev().cloned().collect() };
                if ordered.len() < m + 1 {
                    return Err("tension: too few control points".into());
                }
                let e_vars: Vec<P3> = ordered[..m + 1].iter().map(|p| b.point(p)).collect::<Result<_, _>>()?;
                let d = b.point(ordered.last().unwrap())?;
                let u = degree;
                let mut vars: Vec<V3> = e_vars.iter().flat_map(|p| p.vars()).collect();
                vars.extend(d.vars());
                for t in 0..m {
                    let e_vars = e_vars.clone();
                    let d = d;
                    let (value, u) = (w, u);
                    let t = t;
                    b.eq(
                        cid,
                        &vars,
                        Box::new(move |x: &[f64]| {
                            let g: Vec<[f64; 3]> = e_vars.iter().map(|p| p.at(x)).collect();
                            let kk = d.at(x);
                            let b_ = value * ke(kk, g[0]) / u;
                            ns3(&g, m, b_)[t]
                        }),
                    );
                }
            }
            "distance" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                let value = w;
                let vars = [c.vars(), t.vars()].concat();
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |x| {
                        let r = c.at(x);
                        let u = t.at(x);
                        ke(u, r) - value
                    }),
                );
            }
            "radius" => {
                let id = refn(e, 0)?;
                if !matches!(b.sk.entity(id), Some(Entity3::Arc { .. })) {
                    return Err("radius: not an arc".into());
                }
                let Ent3::Arc { c, p1, .. } = b.entity(id)? else { unreachable!() };
                let value = w;
                let vars = [c.vars(), p1.vars()].concat();
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |x| ke(p1.at(x), c.at(x)) - value),
                );
            }
            "angle" => {
                let ((y, s), (m, x2)) = (b.line(refn(e, 0)?)?, b.line(refn(e, 1)?)?);
                let e1 = if k.s1 < 0.0 { -1.0 } else { 1.0 };
                let e2 = if k.s2 < 0.0 { -1.0 } else { 1.0 };
                let c = w * std::f64::consts::PI / 180.0;
                let vars = [y.vars(), s.vars(), m.vars(), x2.vars()].concat();
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |x| {
                        let z = scale3(sub3(s.at(x), y.at(x)), e1);
                        let r = scale3(sub3(x2.at(x), m.at(x)), e2);
                        cross3(z, r).let_ce().atan2(dot3(z, r)) - c
                    }),
                );
            }
            other => return Err(format!("unknown constraint {other}")),
        }
    }
    Ok(System { x: b.x, keys, key_idx: b.key_idx, eqs: b.eqs })
}

trait CeExt {
    fn let_ce(self) -> f64;
}
impl CeExt for [f64; 3] {
    fn let_ce(self) -> f64 {
        ce(self)
    }
}

fn val(x: &[f64], v: V3) -> f64 {
    v.val(x)
}

/// Solve a 3D sketch in place: on success writes the solved x/y/z back.
pub fn solve_sketch3(sk: &mut Sketch3, opts: &SolveOpts) -> Result<SolveResult, String> {
    let sys = build(sk)?;
    let res = crate::sketch::solver::solve(&sys, opts);
    if res.ok {
        for p in sk.points.iter_mut() {
            let kx = sys.key_idx.get(&format!("{}.x", p.id)).copied();
            let ky = sys.key_idx.get(&format!("{}.y", p.id)).copied();
            let kz = sys.key_idx.get(&format!("{}.z", p.id)).copied();
            if let (Some(kx), Some(ky), Some(kz)) = (kx, ky, kz) {
                if ky == kx + 1 && kz == kx + 2 {
                    p.x = res.x[kx];
                    p.y = res.x[kx + 1];
                    p.z = res.x[kx + 2];
                }
            }
        }
    }
    Ok(res)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::solver::SolveOpts;

    fn sk3(src: &str) -> Sketch3 {
        Sketch3::from_json(&ck_json::parse(src).unwrap())
    }

    fn solved(src: &str) -> Vec<[f64; 3]> {
        let mut s = sk3(src);
        for c in s.constraints.iter_mut() {
            if let Some(e) = &c.expr {
                c.value = Some(crate::expr::eval(e, None).unwrap());
            }
        }
        assert!(solve_sketch3(&mut s, &SolveOpts::default()).unwrap().ok);
        s.points.iter().map(|p| [p.x, p.y, p.z]).collect()
    }

    /// Oracle goldens (kernel regen with `expr` driving the value).
    #[test]
    fn golden_3d() {
        let along_x = solved(
            r#"{"type":"sketch3d","id":"F1","points":[{"id":"p1","x":0,"y":0,"z":0,"fixed":true},{"id":"p2","x":10,"y":1,"z":2},{"id":"p3","x":20,"y":0,"z":0}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"}],
"constraints":[{"type":"alongX","id":"c1","refs":["e1"]},{"type":"distance","id":"c2","refs":["p1","p3"],"value":20,"expr":"25","offset":null}]}"#,
        );
        assert_eq!(along_x[1], [10.0, -1.4879009135881915e-10, -1.1254286391704227e-10]);
        assert_eq!(along_x[2], [25.000000000281357, 0.0, 0.0]);
        // tension (bit-exact vs the oracle)
        let t = solved(
            r#"{"type":"sketch3d","id":"F1","points":[{"id":"O","x":0,"y":0,"z":0,"fixed":true},{"id":"a","x":1,"y":1,"z":1},{"id":"b","x":3,"y":2,"z":0},{"id":"c","x":6,"y":2,"z":2},{"id":"d","x":8,"y":5,"z":1}],
"entities":[{"type":"cvspline","id":"s1","pts":["a","b","c","d"],"degree":3}],
"constraints":[{"type":"tension","id":"k1","refs":["s1"],"at":"a","value":2},{"type":"distance","id":"k2","refs":["O","d"],"value":9.1,"expr":"9.1","offset":null}]}"#,
        );
        assert_eq!(t[1], [0.48762804794435338, 0.8328343085447143, 1.8794502260471118]);
    }

    #[test]
    fn build_errors_3d() {
        // unknown constraint
        let s = sk3(
            r#"{"type":"sketch3d","id":"F1","points":[{"id":"p","x":0,"y":0,"z":0}],"entities":[],"constraints":[{"type":"bogus","id":"k","refs":["p"]}]}"#,
        );
        assert!(build(&s).is_err());
        // onPlane without a plane
        let s = sk3(
            r#"{"type":"sketch3d","id":"F1","points":[{"id":"p","x":0,"y":0,"z":0}],"entities":[],"constraints":[{"type":"onPlane","id":"k","refs":["p"]}]}"#,
        );
        assert!(build(&s).is_err());
        // g1 without a shared end
        let s = sk3(
            r#"{"type":"sketch3d","id":"F1","points":[{"id":"O","x":0,"y":0,"z":0,"fixed":true},{"id":"a","x":1,"y":1,"z":1},{"id":"b","x":3,"y":2,"z":0},{"id":"c","x":6,"y":2,"z":2},{"id":"d","x":8,"y":5,"z":1},{"id":"e","x":2,"y":7,"z":0},{"id":"f","x":4,"y":9,"z":1}],
"entities":[{"type":"line","id":"e1","p1":"e","p2":"f"},{"type":"cvspline","id":"s1","pts":["a","b","c","d"],"degree":3}],
"constraints":[{"type":"g1","id":"k1","refs":["s1","e1"]}]}"#,
        );
        assert!(build(&s).is_err());
    }

    #[test]
    fn echo_3d() {
        let s = sk3(
            r#"{"type":"sketch3d","id":"F1","name":"s3","zz":5,"points":[{"id":"p1"},{"id":"p2","x":null,"y":true,"z":"3","fixed":1}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"circle","id":"e3","c":"p1","r":"4"},{"type":"ref","id":"e9"}],"constraints":[{"type":"fix","id":"k1","refs":["p1"],"value":[1,"2",3]},{"type":"distance","id":"k2","refs":["p1","p2"],"value":"7"}]}"#,
        );
        assert_eq!(
            ck_json::to_string(&s.to_json()),
            r#"{"type":"sketch3d","id":"F1","name":"s3","zz":5,"points":[{"id":"p1","x":0,"y":0,"z":0},{"id":"p2","x":0,"y":0,"z":3,"fixed":true}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"ref","id":"e9","pts":[]}],"constraints":[{"type":"fix","id":"k1","refs":["p1"],"value":[1,"2",3]},{"type":"distance","id":"k2","refs":["p1","p2"],"value":7}]}"#
        );
    }
}
