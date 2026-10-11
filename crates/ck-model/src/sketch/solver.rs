//! The 2D sketch constraint solver.
//!
//! Algorithm = the client's solver, ported 1:1 (app.pretty.js: `$s()` builds
//! the equation system l.1452, `wl()` the finite-difference Jacobian, `Bs()`
//! the solve l.2077, `dl()` the dense LU l.625, `ti()` the rank/DOF analysis
//! l.2125). The kernel runs the same solve when a constraint's `expr` changes
//! its `value` (oracle: rectangle with `distance p1-p2 expr "25"` moves the
//! points to x = -6.9999999635930275 / 18.000000036305124, reproduced to the
//! last bit by the golden tests below).
//!
//! * Unknowns: `x`,`y` of every non-fixed point, then `r` of every circle
//!   (not `ext`/`from`) and ellipse — in document order.
//! * Equations: one `|ac|-|bc|` per arc (radius consistency), then every
//!   constraint's residual(s) in document order.
//! * Solve: Gauss-Newton on the minimum-norm step
//!   `dx = -W Jᵀ (J W Jᵀ + λI)⁻¹ f`, `λ = 1e-10·max diag + 1e-14`, central
//!   differences `h = 1e-7(1+|x|)`, 12-step halving line search on `Σf²`,
//!   ≤ 60 iterations, stop at `max|f| ≤ 1e-9`, success at `≤ 1e-7`.
//!   Optional per-unknown weights (`1e4` for dragged points) scale `W = 1/w`.
//! * Analysis: Gauss-Jordan rank of J (pivot tolerance `1e-8·max(1,max|J|)`):
//!   `dof = n - rank`, `redundant = rank < m`, and the set of fully
//!   determined unknowns -> status `over` / `full` / `under`.
//!
//! Not yet ported (build fails with an error, i.e. the sketch is left
//! unsolved): spline-continuity `g1`/`g2`/`g3`, `onCurve`, `tension`.

use super::{Sketch, SketchEntity};
use std::collections::HashMap;

/// A solver quantity: unknown slot `i >= 0`, or the constant `c`.
#[derive(Debug, Clone, Copy)]
struct V {
    i: isize,
    c: f64,
}

#[derive(Debug, Clone, Copy)]
struct P2 {
    x: V,
    y: V,
}

#[derive(Debug, Clone, Copy)]
enum Ent {
    Line { a: P2, b: P2 },
    Circle { c: P2, r: V },
    Ellipse { c: P2, r: V },
    Arc { c: P2, a: P2, b: P2 },
    Point,
}

#[inline]
fn val(x: &[f64], v: V) -> f64 {
    if v.i >= 0 {
        x[v.i as usize]
    } else {
        v.c
    }
}
#[inline]
fn pt(x: &[f64], p: P2) -> [f64; 2] {
    [val(x, p.x), val(x, p.y)]
}

pub use cr::cr_hypot;

#[inline]
fn ve(a: f64, b: f64) -> f64 {
    cr_hypot(a, b)
}

/// JS `Math.max(...xs)` semantics: NaN poisons, empty -> -inf.
fn js_max(xs: impl Iterator<Item = f64>) -> f64 {
    let mut m = f64::NEG_INFINITY;
    for v in xs {
        if v.is_nan() {
            return f64::NAN;
        }
        if v > m {
            m = v;
        }
    }
    m
}

type Resid = Box<dyn Fn(&[f64]) -> f64>;

/// One scalar equation `f(x) = 0` from constraint `cid`.
pub struct Equation {
    pub cid: String,
    pub deps: Vec<usize>,
    f: Resid,
}

impl Equation {
    pub fn eval(&self, x: &[f64]) -> f64 {
        (self.f)(x)
    }
}

/// The assembled system (client `$s()` return value).
pub struct System {
    /// Current unknown values.
    pub x: Vec<f64>,
    /// Unknown names: `"p1.x"`, `"p1.y"`, `"e3.r"`.
    pub keys: Vec<String>,
    pub key_idx: HashMap<String, usize>,
    pub eqs: Vec<Equation>,
}

fn deps_of(vs: &[V]) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    for v in vs {
        if v.i >= 0 && !out.contains(&(v.i as usize)) {
            out.push(v.i as usize);
        }
    }
    out
}
fn p2v(p: P2) -> [V; 2] {
    [p.x, p.y]
}
fn ent_vars(e: &Ent) -> Vec<V> {
    match *e {
        Ent::Line { a, b } => vec![a.x, a.y, b.x, b.y],
        Ent::Circle { c, r } | Ent::Ellipse { c, r } => vec![c.x, c.y, r],
        Ent::Arc { c, a, b } => vec![c.x, c.y, a.x, a.y, b.x, b.y],
        Ent::Point => vec![],
    }
}

/// Radius of a circle/ellipse/arc (client `p()`).
fn radius(x: &[f64], e: &Ent) -> f64 {
    match *e {
        Ent::Circle { r, .. } | Ent::Ellipse { r, .. } => val(x, r),
        Ent::Arc { c, a, .. } => {
            let (c, a) = (pt(x, c), pt(x, a));
            ve(a[0] - c[0], a[1] - c[1])
        }
        _ => f64::NAN,
    }
}
fn center(e: &Ent) -> Option<P2> {
    match *e {
        Ent::Circle { c, .. } | Ent::Ellipse { c, .. } | Ent::Arc { c, .. } => Some(c),
        _ => None,
    }
}
fn line_ab(e: &Ent) -> Result<(P2, P2), String> {
    match *e {
        Ent::Line { a, b } | Ent::Arc { a, b, .. } => Ok((a, b)),
        _ => Err("not a line".into()),
    }
}
/// Signed distance of `p` from the line (client `y()`).
fn line_dist(x: &[f64], p: P2, a: P2, b: P2) -> f64 {
    let (pa, pb, t) = (pt(x, a), pt(x, b), pt(x, p));
    let z = pb[0] - pa[0];
    let r = pb[1] - pa[1];
    (z * (t[1] - pa[1]) - r * (t[0] - pa[0])) / (ve(z, r) + 1e-12)
}
/// Direction vector of a line (client `S()`).
fn dir(x: &[f64], a: P2, b: P2) -> [f64; 2] {
    let (e, p) = (pt(x, a), pt(x, b));
    [p[0] - e[0], p[1] - e[1]]
}

struct Builder<'a> {
    sk: &'a Sketch,
    key_idx: HashMap<String, usize>,
    x: Vec<f64>,
    eqs: Vec<Equation>,
}

impl<'a> Builder<'a> {
    fn var(&self, key: &str, c: f64) -> V {
        match self.key_idx.get(key) {
            Some(&i) => V { i: i as isize, c },
            None => V { i: -1, c },
        }
    }
    /// Point by id (client `l()`).
    fn point(&self, id: &str) -> Result<P2, String> {
        // last duplicate wins, like `new Map(points)`
        let p = self.sk.points.iter().rev().find(|p| p.id == id).ok_or_else(|| format!("missing point {id}"))?;
        Ok(P2 { x: self.var(&format!("{id}.x"), p.x), y: self.var(&format!("{id}.y"), p.y) })
    }
    fn raw_entity(&self, id: &str) -> Option<&'a SketchEntity> {
        self.sk.entities.iter().rev().find(|e| e.id() == id)
    }
    /// Entity by id (client `u()`).
    fn entity(&self, id: &str) -> Result<Ent, String> {
        let e = self.raw_entity(id).ok_or_else(|| format!("missing entity {id}"))?;
        Ok(match e {
            SketchEntity::Line { p1, p2, .. } => Ent::Line { a: self.point(p1)?, b: self.point(p2)? },
            SketchEntity::Circle { id, c, r, .. } => {
                Ent::Circle { c: self.point(c)?, r: self.var(&format!("{id}.r"), *r) }
            }
            SketchEntity::Ellipse { id, c, p, r, .. } => {
                self.point(p)?;
                Ent::Ellipse { c: self.point(c)?, r: self.var(&format!("{id}.r"), *r) }
            }
            SketchEntity::Arc { c, p1, p2, .. } => Ent::Arc { c: self.point(c)?, a: self.point(p1)?, b: self.point(p2)? },
            SketchEntity::Point { p, .. } => {
                self.point(p)?;
                Ent::Point
            }
            other => return Err(format!("unknown entity {}", other.type_name())),
        })
    }
    /// Point-like reference: point, point entity, or circle/arc/ellipse centre
    /// (client `k()`).
    fn pointish(&self, id: &str) -> Result<P2, String> {
        if self.sk.points.iter().any(|p| p.id == id) {
            return self.point(id);
        }
        match self.raw_entity(id) {
            Some(SketchEntity::Point { p, .. }) => self.point(p),
            Some(SketchEntity::Circle { c, .. }) | Some(SketchEntity::Arc { c, .. }) | Some(SketchEntity::Ellipse { c, .. }) => {
                self.point(c)
            }
            _ => Err(format!("not a point {id}")),
        }
    }
    fn eq(&mut self, cid: &str, vars: &[V], f: Resid) {
        self.eqs.push(Equation { cid: cid.to_string(), deps: deps_of(vars), f });
    }
}

fn refn<'b>(refs: &'b [String], i: usize) -> Result<&'b str, String> {
    refs.get(i).map(|s| s.as_str()).ok_or_else(|| "missing ref".to_string())
}

/// Build the equation system (client `$s()`). Errors mirror the client's
/// throws ("missing point p9", "unknown constraint bogus", ...); the kernel
/// treats any error as "leave the sketch unsolved" (oracle: no regen error).
pub fn build(sk: &Sketch) -> Result<System, String> {
    let mut x = Vec::new();
    let mut keys = Vec::new();
    let mut key_idx = HashMap::new();
    let mut push = |k: String, v: f64, x: &mut Vec<f64>, keys: &mut Vec<String>| {
        key_idx.insert(k.clone(), x.len());
        keys.push(k);
        x.push(v);
    };
    for p in &sk.points {
        if !p.fixed {
            push(format!("{}.x", p.id), p.x, &mut x, &mut keys);
            push(format!("{}.y", p.id), p.y, &mut x, &mut keys);
        }
    }
    for e in &sk.entities {
        match e {
            SketchEntity::Circle { id, r, driven_r: false, .. } | SketchEntity::Ellipse { id, r, .. } => {
                push(format!("{id}.r"), *r, &mut x, &mut keys)
            }
            _ => {}
        }
    }
    let mut b = Builder { sk, key_idx, x, eqs: Vec::new() };

    for e in &sk.entities {
        if let SketchEntity::Arc { id, .. } = e {
            let ent = b.entity(id)?;
            let vars = ent_vars(&ent);
            if deps_of(&vars).is_empty() {
                continue;
            }
            if let Ent::Arc { c, a, b: bb } = ent {
                b.eq(
                    &format!("arc:{id}"),
                    &vars,
                    Box::new(move |x| {
                        let (p, cc, t) = (pt(x, c), pt(x, a), pt(x, bb));
                        ve(cc[0] - p[0], cc[1] - p[1]) - ve(t[0] - p[0], t[1] - p[1])
                    }),
                );
            }
        }
    }

    for k in &sk.constraints {
        let cid = k.id.as_str();
        let e = &k.refs;
        let value = k.value.unwrap_or(f64::NAN);
        match k.kind.as_str() {
            "coincident" | "concentric" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                b.eq(cid, &[c.x, t.x], Box::new(move |z| val(z, c.x) - val(z, t.x)));
                b.eq(cid, &[c.y, t.y], Box::new(move |z| val(z, c.y) - val(z, t.y)));
            }
            "pointOnLine" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (la, lb) = line_ab(&t)?;
                let mut vars = p2v(c).to_vec();
                vars.extend(ent_vars(&t));
                b.eq(cid, &vars, Box::new(move |z| line_dist(z, c, la, lb)));
            }
            "pointOnCircle" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let tc = center(&t).ok_or("not a circle")?;
                let mut vars = p2v(c).to_vec();
                vars.extend(ent_vars(&t));
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |z| {
                        let (r, cc) = (pt(z, c), pt(z, tc));
                        ve(r[0] - cc[0], r[1] - cc[1]) - radius(z, &t)
                    }),
                );
            }
            "midpoint" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (ta, tb) = line_ab(&t)?;
                b.eq(cid, &[c.x, ta.x, tb.x], Box::new(move |z| val(z, c.x) - (val(z, ta.x) + val(z, tb.x)) / 2.0));
                b.eq(cid, &[c.y, ta.y, tb.y], Box::new(move |z| val(z, c.y) - (val(z, ta.y) + val(z, tb.y)) / 2.0));
            }
            "horizontal" | "vertical" => {
                let (c, t) = if e.len() == 1 {
                    line_ab(&b.entity(&e[0])?)?
                } else {
                    (b.pointish(refn(e, 0)?)?, b.pointish(refn(e, 1)?)?)
                };
                let (cv, tv) = if k.kind == "horizontal" { (c.y, t.y) } else { (c.x, t.x) };
                b.eq(cid, &[cv, tv], Box::new(move |r| val(r, cv) - val(r, tv)));
            }
            "parallel" | "perpendicular" => {
                let c = b.entity(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (ca, cb) = line_ab(&c)?;
                let (ta, tb) = line_ab(&t)?;
                let par = k.kind == "parallel";
                let mut vars = ent_vars(&c);
                vars.extend(ent_vars(&t));
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |r| {
                        let u = dir(r, ca, cb);
                        let q = dir(r, ta, tb);
                        let v = ve(u[0], u[1]) * ve(q[0], q[1]) + 1e-12;
                        (if par { u[0] * q[1] - u[1] * q[0] } else { u[0] * q[0] + u[1] * q[1] }) / v
                    }),
                );
            }
            "collinear" => {
                let c = b.entity(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (ca, cb) = line_ab(&c)?;
                let (ta, tb) = line_ab(&t)?;
                let mut v1 = ent_vars(&c);
                v1.extend(p2v(ta));
                let mut v2 = ent_vars(&c);
                v2.extend(p2v(tb));
                b.eq(cid, &v1, Box::new(move |z| line_dist(z, ta, ca, cb)));
                b.eq(cid, &v2, Box::new(move |z| line_dist(z, tb, ca, cb)));
            }
            "equal" => {
                let c = b.entity(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let size = |r: &[f64], en: &Ent| -> f64 {
                    match *en {
                        Ent::Line { a, b } => {
                            let d = dir(r, a, b);
                            ve(d[0], d[1])
                        }
                        _ => radius(r, en),
                    }
                };
                let mut vars = ent_vars(&c);
                vars.extend(ent_vars(&t));
                b.eq(cid, &vars, Box::new(move |r| size(r, &c) - size(r, &t)));
            }
            "tangent" => tangent(&mut b, k)?,
            "fix" | "pierce" => {
                let c = b.pointish(refn(e, 0)?)?;
                let [px, py] = k.point.unwrap_or([f64::NAN, f64::NAN]);
                b.eq(cid, &[c.x], Box::new(move |t| val(t, c.x) - px));
                b.eq(cid, &[c.y], Box::new(move |t| val(t, c.y) - py));
            }
            "symmetric" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                let zl = b.entity(refn(e, 2)?)?;
                let (za, zb) = line_ab(&zl)?;
                let mut vars = p2v(c).to_vec();
                vars.extend(p2v(t));
                vars.extend(ent_vars(&zl));
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |r| {
                        let (u, q, v, l) = (pt(r, c), pt(r, t), pt(r, za), pt(r, zb));
                        let kx = l[0] - v[0];
                        let ne = l[1] - v[1];
                        let le = [(u[0] + q[0]) / 2.0, (u[1] + q[1]) / 2.0];
                        (kx * (le[1] - v[1]) - ne * (le[0] - v[0])) / (ve(kx, ne) + 1e-12)
                    }),
                );
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |r| {
                        let (u, q) = (pt(r, c), pt(r, t));
                        let v = dir(r, za, zb);
                        ((q[0] - u[0]) * v[0] + (q[1] - u[1]) * v[1]) / (ve(v[0], v[1]) + 1e-12)
                    }),
                );
            }
            "distance" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                b.eq(
                    cid,
                    &[c.x, c.y, t.x, t.y],
                    Box::new(move |z| {
                        let (r, u) = (pt(z, c), pt(z, t));
                        ve(u[0] - r[0], u[1] - r[1]) - value
                    }),
                );
            }
            "hdist" | "vdist" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.pointish(refn(e, 1)?)?;
                let (cv, tv) = if k.kind == "hdist" { (c.x, t.x) } else { (c.y, t.y) };
                b.eq(cid, &[cv, tv], Box::new(move |r| (val(r, tv) - val(r, cv)).abs() - value));
            }
            "pldist" => {
                let c = b.pointish(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (ta, tb) = line_ab(&t)?;
                let mut vars = p2v(c).to_vec();
                vars.extend(ent_vars(&t));
                b.eq(cid, &vars, Box::new(move |z| line_dist(z, c, ta, tb).abs() - value));
            }
            "radius" | "diameter" => {
                let c = b.entity(refn(e, 0)?)?;
                let m = if k.kind == "diameter" { 2.0 } else { 1.0 };
                b.eq(cid, &ent_vars(&c), Box::new(move |z| m * radius(z, &c) - value));
            }
            "raddiff" => {
                let c = b.entity(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let s = k.sign;
                let mut vars = ent_vars(&c);
                vars.extend(ent_vars(&t));
                b.eq(cid, &vars, Box::new(move |r| s * (radius(r, &t) - radius(r, &c)) - value));
            }
            "angle" => {
                let c = b.entity(refn(e, 0)?)?;
                let t = b.entity(refn(e, 1)?)?;
                let (ca, cb) = line_ab(&c)?;
                let (ta, tb) = line_ab(&t)?;
                let (s1, s2) = (k.s1, k.s2);
                let a = value * std::f64::consts::PI / 180.0;
                let (q, v) = (a.cos(), a.sin());
                let mut vars = ent_vars(&c);
                vars.extend(ent_vars(&t));
                b.eq(
                    cid,
                    &vars,
                    Box::new(move |l| {
                        let kk = dir(l, ca, cb).map(|n| n * s1);
                        let ne = dir(l, ta, tb).map(|n| n * s2);
                        let le = ve(kk[0], kk[1]) * ve(ne[0], ne[1]) + 1e-12;
                        let re = kk[0] * ne[1] - kk[1] * ne[0];
                        let ee = kk[0] * ne[0] + kk[1] * ne[1];
                        (re * q - ee * v) / le
                    }),
                );
            }
            // TODO: spline continuity / onCurve / tension (client pd/kd/vo helpers).
            "g1" | "g2" | "g3" | "onCurve" | "tension" => return Err(format!("unsupported constraint {}", k.kind)),
            other => return Err(format!("unknown constraint {other}")),
        }
    }
    let Builder { x, key_idx, eqs, .. } = b;
    Ok(System { x, keys, key_idx, eqs })
}

fn tangent(b: &mut Builder, k: &super::Constraint) -> Result<(), String> {
    let cid = k.id.as_str();
    let e0 = refn(&k.refs, 0)?;
    let e1 = refn(&k.refs, 1)?;
    let c_raw = b.raw_entity(e0).ok_or("missing entity")?;
    let t_raw = b.raw_entity(e1).ok_or("missing entity")?;
    let ends = |v: &SketchEntity| -> Vec<String> {
        match v {
            SketchEntity::Line { p1, p2, .. } | SketchEntity::Arc { p1, p2, .. } => vec![p1.clone(), p2.clone()],
            _ => vec![],
        }
    };
    let (zc, zt) = (ends(c_raw), ends(t_raw));
    let shared = match &k.at {
        Some(at) if zc.contains(at) || zt.contains(at) => Some(at.clone()),
        _ => zc.iter().find(|v| zt.contains(v)).cloned(),
    };
    let (mut l, mut kk) = (b.entity(e0)?, b.entity(e1)?);
    if let Some(r) = shared {
        let v = b.point(&r)?;
        if matches!(kk, Ent::Line { .. }) {
            std::mem::swap(&mut l, &mut kk);
        }
        let mut vars = ent_vars(&l);
        vars.extend(ent_vars(&kk));
        vars.extend(p2v(v));
        if let Ent::Line { a: la, b: lb } = l {
            let kc = center(&kk).ok_or("not a curve")?;
            b.eq(
                cid,
                &vars,
                Box::new(move |ne| {
                    let le = dir(ne, la, lb);
                    let re = pt(ne, v);
                    let ee = pt(ne, kc);
                    let n = [re[0] - ee[0], re[1] - ee[1]];
                    (n[0] * le[0] + n[1] * le[1]) / (ve(le[0], le[1]) * ve(n[0], n[1]) + 1e-12)
                }),
            );
        } else {
            let lc = center(&l).ok_or("not a curve")?;
            let kc = center(&kk).ok_or("not a curve")?;
            b.eq(
                cid,
                &vars,
                Box::new(move |ne| {
                    let le = pt(ne, v);
                    let re = pt(ne, lc);
                    let ee = pt(ne, kc);
                    let n = [le[0] - re[0], le[1] - re[1]];
                    let h = [le[0] - ee[0], le[1] - ee[1]];
                    (n[0] * h[1] - n[1] * h[0]) / (ve(n[0], n[1]) * ve(h[0], h[1]) + 1e-12)
                }),
            );
        }
        return Ok(());
    }
    let (mut u, mut q) = (l, kk);
    if matches!(q, Ent::Line { .. }) {
        std::mem::swap(&mut u, &mut q);
    }
    let mut vars = ent_vars(&u);
    vars.extend(ent_vars(&q));
    if let Ent::Line { a: ua, b: ub } = u {
        let qc = center(&q).ok_or("not a curve")?;
        b.eq(cid, &vars, Box::new(move |v| line_dist(v, qc, ua, ub).abs() - radius(v, &q)));
    } else {
        let internal = k.internal;
        let uc = center(&u).ok_or("not a curve")?;
        let qc = center(&q).ok_or("not a curve")?;
        b.eq(
            cid,
            &vars,
            Box::new(move |l| {
                let (kp, np) = (pt(l, uc), pt(l, qc));
                let le = ve(kp[0] - np[0], kp[1] - np[1]);
                let re = radius(l, &u);
                let ee = radius(l, &q);
                if internal {
                    le - (re - ee).abs()
                } else {
                    le - (re + ee)
                }
            }),
        );
    }
    Ok(())
}

/// Sparse Jacobian rows by central differences (client `wl()`).
fn jacobian(sys: &System, x: &mut [f64]) -> Vec<Vec<(usize, f64)>> {
    let mut rows = Vec::with_capacity(sys.eqs.len());
    for eq in &sys.eqs {
        let mut s = Vec::new();
        for &i in &eq.deps {
            let o = x[i];
            let a = (o.abs() + 1.0) * 1e-7; // WAT f1019: (|o|+1)*1e-7
            x[i] = o + a;
            let l = eq.eval(x);
            x[i] = o - a;
            let c = eq.eval(x);
            x[i] = o;
            let h = (l - c) / (a + a);
            if h != 0.0 {
                s.push((i, h));
            }
        }
        rows.push(s);
    }
    rows
}

fn residuals(sys: &System, x: &[f64]) -> Vec<f64> {
    sys.eqs.iter().map(|e| e.eval(x)).collect()
}

fn sumsq(r: &[f64]) -> f64 {
    r.iter().fold(0.0, |e, t| e + t * t)
}

/// Dense LU with partial pivoting, in place (client `dl()`); `None` when a
/// pivot is below 1e-300.
fn lu_solve(r: &mut [Vec<f64>], e: &mut [f64]) -> Option<Vec<f64>> {
    let t = e.len();
    for s in 0..t {
        let mut i = s;
        let mut o = r[s][s].abs();
        for c in s + 1..t {
            let h = r[c][s].abs();
            if h > o {
                o = h;
                i = c;
            }
        }
        if o < 1e-300 {
            return None;
        }
        if i != s {
            r.swap(s, i);
            e.swap(s, i);
        }
        let l = 1.0 / r[s][s];
        let (top, bot) = r.split_at_mut(s + 1);
        let a = &top[s];
        for (ci, h) in bot.iter_mut().enumerate() {
            let c = s + 1 + ci;
            let d = h[s] * l;
            if d != 0.0 {
                for u in s..t {
                    h[u] -= d * a[u];
                }
                e[c] -= d * e[s];
            }
        }
    }
    let mut n = vec![0.0; t];
    for s in (0..t).rev() {
        let mut i = e[s];
        let o = &r[s];
        for a in s + 1..t {
            i -= o[a] * n[a];
        }
        n[s] = i / o[s];
    }
    Some(n)
}

/// Solver options (client `Bs()` defaults: maxIter 60, tol 1e-9).
#[derive(Debug, Clone)]
pub struct SolveOpts {
    /// Per-unknown weights (dragged unknowns get 1e4); `None` = all 1.
    pub weights: Option<Vec<f64>>,
    pub max_iter: usize,
    pub tol: f64,
}

impl Default for SolveOpts {
    fn default() -> Self {
        SolveOpts { weights: None, max_iter: 60, tol: 1e-9 }
    }
}

#[derive(Debug, Clone)]
pub struct SolveResult {
    pub ok: bool,
    /// Final max |residual|.
    pub err: f64,
    pub iterations: usize,
    pub x: Vec<f64>,
}

/// Damped Gauss-Newton / minimum-norm solve (client `Bs()`).
pub fn solve(sys: &System, opts: &SolveOpts) -> SolveResult {
    let mut s = sys.x.clone();
    let i = s.len();
    let o = sys.eqs.len();
    if o == 0 {
        return SolveResult { ok: true, err: 0.0, iterations: 0, x: s };
    }
    let mut a = vec![1.0; i];
    if let Some(w) = &opts.weights {
        for d in 0..i {
            a[d] = 1.0 / w.get(d).copied().unwrap_or(1.0);
        }
    }
    let mut l = residuals(sys, &s);
    let mut c = js_max(l.iter().map(|v| v.abs()));
    let mut h = 0;
    while h < opts.max_iter && c > opts.tol {
        let d = jacobian(sys, &mut s);
        let mut u: Vec<Vec<(usize, f64)>> = vec![Vec::new(); i];
        for (x, row) in d.iter().enumerate() {
            for &(e, p) in row {
                u[e].push((x, p));
            }
        }
        let mut m = vec![vec![0.0f64; o]; o];
        for x in 0..i {
            let e = &u[x];
            if e.is_empty() {
                continue;
            }
            let p = a[x];
            for &(cc, t) in e {
                for &(z, r) in e {
                    m[cc][z] += p * (t * r); // WAT f1263: m += w*(Jt*Jr)
                }
            }
        }
        let mut p = 0.0f64;
        for x in 0..o {
            // JS Math.max(p, v): NaN poisons
            p = if m[x][x].is_nan() || p.is_nan() { f64::NAN } else { p.max(m[x][x]) };
        }
        let g = 1e-10 * p + 1e-14;
        for x in 0..o {
            m[x][x] += g;
        }
        let mut rhs = l.clone();
        let Some(k) = lu_solve(&mut m, &mut rhs) else { break };
        let mut bstep = vec![0.0; i];
        for (x, row) in d.iter().enumerate() {
            for &(e, p) in row {
                bstep[e] -= a[e] * p * k[x];
            }
        }
        let w = sumsq(&l);
        let mut y = 1.0;
        let mut accepted = false;
        for x in 0..12 {
            let e: Vec<f64> = s.iter().zip(&bstep).map(|(c, t)| c + t * y).collect();
            let pr = residuals(sys, &e);
            if sumsq(&pr) < w || x == 11 {
                s = e;
                l = pr;
                accepted = true;
                break;
            }
            y *= 0.5;
        }
        if !accepted {
            break;
        }
        let mm = js_max(l.iter().map(|v| v.abs()));
        if !mm.is_finite() {
            break;
        }
        c = mm;
        h += 1;
    }
    SolveResult { ok: c <= opts.tol.max(1e-7), err: c, iterations: h, x: s }
}

/// Rank / DOF analysis (client `ti()`).
#[derive(Debug, Clone)]
pub struct Analysis {
    pub rank: usize,
    /// Number of equations.
    pub m: usize,
    /// Number of unknowns.
    pub n: usize,
    pub dof: usize,
    pub redundant: bool,
    /// Keys of fully determined unknowns.
    pub determined: Vec<String>,
}

pub fn analyze(sys: &System) -> Analysis {
    let t = sys.x.len();
    let n = sys.eqs.len();
    let mut xs = sys.x.clone();
    let mut i: Vec<Vec<f64>> = jacobian(sys, &mut xs)
        .into_iter()
        .map(|row| {
            let mut p = vec![0.0; t];
            for (g, k) in row {
                p[g] = k;
            }
            p
        })
        .collect();
    let mut o = 0.0f64;
    for r in &i {
        for &p in r {
            o = o.max(p.abs());
        }
    }
    let a = 1e-8 * o.max(1.0);
    let mut l: Vec<usize> = Vec::new();
    let mut c = 0;
    let mut m = 0;
    while m < t && c < n {
        let mut p: isize = -1;
        let mut g = a;
        for w in c..n {
            if i[w][m].abs() > g {
                g = i[w][m].abs();
                p = w as isize;
            }
        }
        if p < 0 {
            m += 1;
            continue;
        }
        i.swap(c, p as usize);
        let bb = 1.0 / i[c][m];
        for w in 0..t {
            i[c][w] *= bb;
        }
        let k = i[c].clone();
        for w in 0..n {
            if w == c {
                continue;
            }
            let y = i[w][m];
            if y == 0.0 {
                continue;
            }
            for mm in 0..t {
                i[w][mm] -= y * k[mm];
            }
        }
        l.push(m);
        c += 1;
        m += 1;
    }
    let mut h = vec![-1isize; t];
    for (p, &mm) in l.iter().enumerate() {
        h[mm] = p as isize;
    }
    let mut d = vec![true; t];
    for mm in 0..t {
        if h[mm] < 0 {
            d[mm] = false;
            for p in 0..c {
                if i[p][mm].abs() > 1e-7 {
                    d[l[p]] = false;
                }
            }
        }
    }
    let determined = (0..t).filter(|&mm| d[mm]).map(|mm| sys.keys[mm].clone()).collect();
    Analysis { rank: c, m: n, n: t, dof: t - c, redundant: c < n, determined }
}

/// Sketch status summary (client `si()`).
#[derive(Debug, Clone)]
pub struct Status {
    /// `"over"` (redundant), `"full"` (dof 0) or `"under"`.
    pub status: &'static str,
    pub dof: usize,
    pub rank: usize,
    pub equations: usize,
    pub defined_points: Vec<String>,
    pub defined_entities: Vec<String>,
}

pub fn status(sk: &Sketch) -> Result<Status, String> {
    let sys = build(sk)?;
    let t = analyze(&sys);
    let det = |k: &str| t.determined.iter().any(|d| d == k);
    let pdef = |id: &str| -> bool {
        match sk.point(id) {
            Some(p) => p.fixed || (det(&format!("{id}.x")) && det(&format!("{id}.y"))),
            None => false,
        }
    };
    let defined_points: Vec<String> = sk.points.iter().filter(|p| pdef(&p.id)).map(|p| p.id.clone()).collect();
    let mut defined_entities = Vec::new();
    for e in &sk.entities {
        let mut l = e.point_ids().iter().all(|c| defined_points.iter().any(|d| d == c));
        if matches!(e, SketchEntity::Circle { .. } | SketchEntity::Ellipse { .. }) {
            l = l && det(&format!("{}.r", e.id()));
        }
        if l {
            defined_entities.push(e.id().to_string());
        }
    }
    Ok(Status {
        status: if t.redundant {
            "over"
        } else if t.dof == 0 {
            "full"
        } else {
            "under"
        },
        dof: t.dof,
        rank: t.rank,
        equations: t.m,
        defined_points,
        defined_entities,
    })
}

/// Solve a sketch in place (client `Qe()` + `bd()`): on success writes the
/// solved point coordinates and `|r|` back. Build errors are returned as
/// `Err`; a non-converged solve returns `Ok(result)` with `ok == false` and
/// leaves the sketch untouched.
pub fn solve_sketch(sk: &mut Sketch, opts: &SolveOpts) -> Result<SolveResult, String> {
    let sys = build(sk)?;
    let res = solve(&sys, opts);
    if res.ok {
        for p in sk.points.iter_mut() {
            if let Some(&i) = sys.key_idx.get(&format!("{}.x", p.id)) {
                p.x = res.x[i];
                p.y = res.x[i + 1];
            }
        }
        for e in sk.entities.iter_mut() {
            match e {
                SketchEntity::Circle { id, r, .. } | SketchEntity::Ellipse { id, r, .. } => {
                    if let Some(&i) = sys.key_idx.get(&format!("{id}.r")) {
                        *r = res.x[i].abs();
                    }
                }
                _ => {}
            }
        }
    }
    Ok(res)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sk(src: &str) -> Sketch {
        Sketch::from_json(&ck_json::parse(src).unwrap())
    }

    const RECT: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":1},{"id":"p3","x":11,"y":5},{"id":"p4","x":1,"y":6}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],
"constraints":[{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},{"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]},{"type":"distance","id":"c5","refs":["p1","p2"],"value":VAL,"offset":null}]}"#;

    fn pts(s: &Sketch) -> Vec<(f64, f64)> {
        s.points.iter().map(|p| (p.x, p.y)).collect()
    }

    /// Oracle golden: kernel regen of RECT with `expr` driving value to 25.
    #[test]
    fn golden_rect_25() {
        let mut s = sk(&RECT.replace("VAL", "25"));
        let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(r.ok);
        assert_eq!(
            pts(&s),
            vec![
                (-6.9999999635930275, 0.5000000002792861),
                (18.000000036305124, 0.5000000002792861),
                (18.00000003630853, 5.499999999894265),
                (-6.999999963600884, 5.499999999894264)
            ]
        );
    }

    /// Oracle golden: value 30 (variable w = 30).
    #[test]
    fn golden_rect_30() {
        let mut s = sk(&RECT.replace("VAL", "30"));
        assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
        assert_eq!(pts(&s)[0], (-9.499999902516906, 0.50000009027764));
        assert_eq!(pts(&s)[1], (20.500000097511588, 0.50000009027764));
        assert_eq!(pts(&s)[2], (20.500000097513997, 5.499999999894264));
    }

    /// Oracle golden: value 25.4 (expr "1in").
    #[test]
    fn golden_rect_1in() {
        let mut s = sk(&RECT.replace("VAL", "25.4"));
        assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
        assert_eq!(
            pts(&s),
            vec![
                (-7.199999962620648, 0.5000000002848648),
                (18.2000000374065, 0.5000000002848648),
                (18.200000037409836, 5.499999999894265),
                (-7.199999962628626, 5.499999999894264)
            ]
        );
    }

    #[test]
    fn inconsistent_is_not_ok() {
        let mut s = sk(&RECT.replace("VAL", "25").replace(
            r#""offset":null}]"#,
            r#""offset":null},{"type":"hdist","id":"c6","refs":["p1","p2"],"value":10}]"#,
        ));
        let before = pts(&s);
        let r = solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        assert!(!r.ok);
        assert_eq!(pts(&s), before);
    }

    #[test]
    fn build_errors() {
        let s = sk(&RECT.replace("VAL", "25").replace(r#""refs":["p1","p2"]"#, r#""refs":["p1","p9"]"#));
        assert!(build(&s).err().unwrap().contains("p9"));
        let s = sk(&RECT.replace("VAL", "25").replace(r#""type":"distance""#, r#""type":"bogus""#));
        assert!(build(&s).is_err());
    }

    #[test]
    fn status_dof() {
        // free rectangle with H/V only: 8 unknowns, 4 eqs -> under, dof 4
        let s = sk(&RECT.replace(r#",{"type":"distance","id":"c5","refs":["p1","p2"],"value":VAL,"offset":null}"#, ""));
        let st = status(&s).unwrap();
        assert_eq!((st.status, st.dof, st.rank, st.equations), ("under", 4, 4, 4));
        // fix p1 + width + height -> fully defined
        let src = RECT.replace("VAL", "25").replace(r#"{"id":"p1","x":0,"y":0}"#, r#"{"id":"p1","x":0,"y":0,"fixed":true}"#).replace(
            r#""offset":null}]"#,
            r#""offset":null},{"type":"vdist","id":"c6","refs":["p1","p4"],"value":6}]"#,
        );
        let st = status(&sk(&src)).unwrap();
        assert_eq!((st.status, st.dof), ("full", 0));
        assert_eq!(st.defined_points.len(), 4);
        assert_eq!(st.defined_entities.len(), 4);
        // redundant
        let src = src.replace(r#"]}"#, r#",{"type":"horizontal","id":"c7","refs":["p1","p2"]}]}"#);
        assert_eq!(status(&sk(&src)).unwrap().status, "over");
    }

    #[test]
    fn circle_tangent_line() {
        let src = r#"{"type":"sketch","id":"F1","points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"a","x":-10,"y":7},{"id":"b","x":10,"y":7.5}],
"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"circle","id":"e2","c":"O","r":4}],
"constraints":[{"type":"tangent","id":"k1","refs":["e1","e2"]},{"type":"radius","id":"k2","refs":["e2"],"value":5},{"type":"horizontal","id":"k3","refs":["e1"]}]}"#;
        let mut s = sk(src);
        assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
        let a = s.point("a").unwrap();
        let b = s.point("b").unwrap();
        assert!((a.y - b.y).abs() < 1e-7);
        assert!((a.y.abs() - 5.0).abs() < 1e-6, "{}", a.y);
        match s.entity("e2").unwrap() {
            SketchEntity::Circle { r, .. } => assert!((r - 5.0).abs() < 1e-7),
            _ => unreachable!(),
        }
    }

    #[test]
    fn arc_and_angle() {
        let src = r#"{"type":"sketch","id":"F1","points":[{"id":"c","x":0,"y":0,"fixed":true},{"id":"a","x":10,"y":0,"fixed":true},{"id":"b","x":1,"y":8},{"id":"q","x":3,"y":3}],
"entities":[{"type":"arc","id":"e1","c":"c","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"c","p2":"a"},{"type":"line","id":"e3","p1":"c","p2":"b"}],
"constraints":[{"type":"angle","id":"k1","refs":["e2","e3"],"value":60},{"type":"midpoint","id":"k2","refs":["q","e3"]}]}"#;
        let mut s = sk(src);
        assert!(solve_sketch(&mut s, &SolveOpts::default()).unwrap().ok);
        let b = s.point("b").unwrap();
        assert!((b.x.hypot(b.y) - 10.0).abs() < 1e-6);
        assert!((b.y.atan2(b.x).to_degrees() - 60.0).abs() < 1e-5);
        let q = s.point("q").unwrap();
        assert!((q.x - b.x / 2.0).abs() < 1e-7 && (q.y - b.y / 2.0).abs() < 1e-7);
    }

}
#[cfg(test)]
mod scratch_probe {
    #[test]
    fn probe_files() {
        let Ok(list) = std::env::var("PROBE") else { return };
        for f in list.split(',') {
            let src = std::fs::read_to_string(f).unwrap();
            let j = ck_json::parse(&src).unwrap();
            let fj = &j.get("msg").unwrap().get("part").unwrap().get("features").unwrap().as_arr().unwrap()[0];
            let mut s = super::Sketch::from_json(fj);
            for c in s.constraints.iter_mut() {
                if let Some(e) = &c.expr { c.value = crate::expr::eval(e, None).ok(); }
            }
            let r = super::solve_sketch(&mut s, &Default::default()).unwrap();
            eprintln!("PROBE {f} ok={} it={} {:?}", r.ok, r.iterations, s.points.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>());
        }
    }
}

/// Correctly rounded `hypot`.
///
/// kernel.wasm's hypot (WAT f4194 -> f4190) is the CORE-MATH correctly
/// rounded algorithm (fma-corrected sqrt + exact-tie handling), as shipped
/// by recent Rust libm. Host libms are only faithfully rounded, and the
/// solver's finite differences amplify a 1-ulp difference to ~1e-8 in the
/// solved coordinates (oracle probe: two free points + `distance 25` gives
/// p1 = (-7.437964839797623, -0.7437965038973292) only with a CR hypot).
/// Implemented as: float candidate, then exact integer comparison of
/// `(c + c')²` against `4(a² + b²)` to pick the correctly rounded neighbour.
pub mod cr {
    /// Little-endian u64 limbs.
    #[derive(Clone, Debug, PartialEq, Eq)]
    struct Big(Vec<u64>);

    impl Big {
        fn from_u128(v: u128) -> Big {
            Big(vec![v as u64, (v >> 64) as u64])
        }
        fn mul(a: u128, b: u128) -> Big {
            let a = [a as u64, (a >> 64) as u64];
            let b = [b as u64, (b >> 64) as u64];
            let mut r = vec![0u64; 4];
            for i in 0..2 {
                let mut carry: u128 = 0;
                for j in 0..2 {
                    let t = r[i + j] as u128 + (a[i] as u128) * (b[j] as u128) + carry;
                    r[i + j] = t as u64;
                    carry = t >> 64;
                }
                let mut k = i + 2;
                while carry != 0 {
                    let t = r[k] as u128 + carry;
                    r[k] = t as u64;
                    carry = t >> 64;
                    k += 1;
                }
            }
            Big(r)
        }
        fn shl(&self, n: u32) -> Big {
            let (w, b) = ((n / 64) as usize, n % 64);
            let mut r = vec![0u64; self.0.len() + w + 1];
            for (i, &x) in self.0.iter().enumerate() {
                r[i + w] |= x << b;
                if b > 0 {
                    r[i + w + 1] |= x >> (64 - b);
                }
            }
            Big(r)
        }
        fn add(&self, o: &Big) -> Big {
            let n = self.0.len().max(o.0.len()) + 1;
            let mut r = vec![0u64; n];
            let mut carry = 0u128;
            for i in 0..n {
                let t = *self.0.get(i).unwrap_or(&0) as u128 + *o.0.get(i).unwrap_or(&0) as u128 + carry;
                r[i] = t as u64;
                carry = t >> 64;
            }
            Big(r)
        }
        fn cmp(&self, o: &Big) -> std::cmp::Ordering {
            let n = self.0.len().max(o.0.len());
            for i in (0..n).rev() {
                let (a, b) = (*self.0.get(i).unwrap_or(&0), *o.0.get(i).unwrap_or(&0));
                if a != b {
                    return a.cmp(&b);
                }
            }
            std::cmp::Ordering::Equal
        }
    }

    /// Positive finite double -> (mantissa, exponent) with v = m * 2^e.
    fn decomp(v: f64) -> (u128, i32) {
        let bits = v.to_bits();
        let exp = ((bits >> 52) & 0x7ff) as i32;
        let frac = bits & ((1u64 << 52) - 1);
        if exp == 0 {
            (frac as u128, -1074)
        } else {
            ((frac | (1u64 << 52)) as u128, exp - 1075)
        }
    }

    fn next_up(v: f64) -> f64 {
        f64::from_bits(v.to_bits() + 1)
    }
    fn next_down(v: f64) -> f64 {
        f64::from_bits(v.to_bits() - 1)
    }

    /// Sign of `(u+v)^2 - 4 S` where `S = a^2 + b^2` (all positive, finite).
    fn cmp_mid(u: f64, v: f64, a: (u128, i32), b: (u128, i32)) -> std::cmp::Ordering {
        let (mu, eu) = decomp(u);
        let (mv, ev) = decomp(v);
        let e = eu.min(ev);
        let w = (mu << (eu - e)) + (mv << (ev - e));
        // (u+v)^2 = w^2 * 2^(2e);  4S = A^2 2^(2ea+2) + B^2 2^(2eb+2)
        let big_e = (2 * e).min(2 * a.1 + 2).min(2 * b.1 + 2);
        let lhs = Big::mul(w, w).shl((2 * e - big_e) as u32);
        let rhs = Big::mul(a.0, a.0)
            .shl((2 * a.1 + 2 - big_e) as u32)
            .add(&Big::mul(b.0, b.0).shl((2 * b.1 + 2 - big_e) as u32));
        lhs.cmp(&rhs)
    }

    pub fn cr_hypot(x: f64, y: f64) -> f64 {
        let (mut a, mut b) = (x.abs(), y.abs());
        if a.is_infinite() || b.is_infinite() {
            return f64::INFINITY;
        }
        if a.is_nan() || b.is_nan() {
            return f64::NAN;
        }
        if a < b {
            std::mem::swap(&mut a, &mut b);
        }
        if b == 0.0 {
            return a;
        }
        // b/a < 2^-60: the correction b²/2a is far below half an ulp.
        if b < a * f64::from_bits(0x3c30000000000000) {
            return a;
        }
        // exact power-of-two rescale into a safe range
        let (scale, unscale) = if a > 1e300 {
            (2f64.powi(-600), 2f64.powi(600))
        } else if b < 1e-300 {
            (2f64.powi(600), 2f64.powi(-600))
        } else {
            (1.0, 1.0)
        };
        a *= scale;
        b *= scale;
        let (da, db) = (decomp(a), decomp(b));
        let mut c = (a * a + b * b).sqrt();
        use std::cmp::Ordering::*;
        for _ in 0..8 {
            let up = next_up(c);
            match cmp_mid(c, up, da, db) {
                Less => {
                    c = up;
                    continue;
                }
                Equal if c.to_bits() & 1 == 1 => {
                    c = up;
                    continue;
                }
                _ => {}
            }
            let dn = next_down(c);
            match cmp_mid(dn, c, da, db) {
                Greater => {
                    c = dn;
                    continue;
                }
                Equal if c.to_bits() & 1 == 1 => {
                    c = dn;
                    continue;
                }
                _ => {}
            }
            break;
        }
        c * unscale
    }

    #[cfg(test)]
    mod tests {
        use super::cr_hypot;
        #[test]
        fn basics() {
            assert_eq!(cr_hypot(3.0, 4.0), 5.0);
            assert_eq!(cr_hypot(-5.0, 12.0), 13.0);
            assert_eq!(cr_hypot(0.0, 0.0), 0.0);
            assert_eq!(cr_hypot(0.0, -2.5), 2.5);
            assert_eq!(cr_hypot(1e308, 1e308), f64::INFINITY.min(1.4142135623730951e308));
            assert_eq!(cr_hypot(1e-310, 0.0), 1e-310);
            assert_eq!(cr_hypot(1.0, 1e-30), 1.0);
            assert_eq!(cr_hypot(1.0, 1.0), std::f64::consts::SQRT_2);
            assert!(cr_hypot(f64::NAN, 1.0).is_nan());
            assert_eq!(cr_hypot(f64::NAN, f64::INFINITY), f64::INFINITY);
        }
    }
}
