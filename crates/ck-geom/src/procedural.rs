//! Procedural curves: curves kept as their definition (what they are), evaluated
//! from it, and converted to a B-spline only on demand, with the deviation of
//! that B-spline from the definition stated next to it.
//!
//! The design follows solidcore's definitions-and-certificates contract
//! (tangency-labs/solidcore, Apache-2.0, see THIRD_PARTY.md: `curve/composite_curve`,
//! `offset_curve`, `crv_on_surf`, `iso_curve`, `projected_curve`): the definition
//! S answers every evaluation, the B-spline B is an approximation of S built for
//! export, tessellation and the operations that need a uniform form, never an
//! operand. Here the bound of B is measured (the fit's error at its samples and
//! the midpoints between them, plus an independent closest-point check on a
//! denser grid), not proved by interval enclosure as upstream does: it is a
//! sampled bound and is named so.
//!
//! Kinds (variants of [`Curve`]):
//! - [`Curve::Composite`]: ordered pieces joined end to end, one parameter s
//!   (each piece keeps its own speed, so s - s_i = t - t0_i on piece i and the
//!   derivatives are the pieces' own), with the continuity at every joint
//!   measured at construction ([`Continuity`]).
//! - [`Curve::Offset`]: P = C + d·unit(C' × N) with N a fixed plane normal (the
//!   solidcore offset curve convention), or N the unit normal of a surface at the
//!   foot of C (an offset in the surface's tangent plane, the usual case of an
//!   edge of a face).
//! - [`Curve::Projected`]: the foot of C(t) on a surface, along the surface
//!   normal (closest point) or along a fixed direction.
//! - [`Curve::OnSurface`]: C(t) = S(uv(t)) for a 2D curve uv (a pcurve made a
//!   3D curve, see [`crate::pcurve::CurveOnSurface`]).
//! - [`Curve::Iso`]: an iso-parametric line of a surface.
//!
//! Derivatives: exact (closed form or the implicit function theorem on the
//! defining equations) to the order stated on each kind; higher orders are
//! central differences of the highest exact one.

use crate::approx::{approximate_curve_fn, fn_curve_deviation};
use crate::curve::{push_vec3, Curve};
use crate::curve2::Curve2;
use crate::nurbs::bspline::BSplineCurve;
use crate::surface::{SurfDerivs, Surface};
use crate::{Vec2, Vec3};

/// Continuity at a joint of a composite curve, strongest first in `Ord`
/// terms reversed (C2 > G2 > C1 > G1 > C0 by [`Continuity::rank`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Continuity {
    /// Positions meet, tangents differ (a corner).
    C0,
    /// Unit tangents agree.
    G1,
    /// First derivatives agree.
    C1,
    /// Unit tangents and curvature vectors agree.
    G2,
    /// First and second derivatives agree.
    C2,
}

impl Continuity {
    pub fn rank(self) -> u8 {
        match self {
            Continuity::C0 => 0,
            Continuity::G1 => 1,
            Continuity::C1 => 2,
            Continuity::G2 => 3,
            Continuity::C2 => 4,
        }
    }
    /// Tangent continuous (G1 or better).
    pub fn is_smooth(self) -> bool {
        self.rank() >= 1
    }
    pub fn code(self) -> u8 {
        self.rank()
    }
    pub fn from_code(c: u8) -> Option<Continuity> {
        Some(match c {
            0 => Continuity::C0,
            1 => Continuity::G1,
            2 => Continuity::C1,
            3 => Continuity::G2,
            4 => Continuity::C2,
            _ => return None,
        })
    }
}

/// One piece of a composite: `curve` over its own `[t0, t1]`.
#[derive(Debug, Clone, PartialEq)]
pub struct CompositeSeg {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
}

/// Ordered pieces joined end to end. Built by [`CompositeCurve::new`], which
/// checks the joints and measures their continuity; `starts[i]` is the
/// composite parameter where piece i begins (0 for the first; the composite
/// runs over `[0, starts.last + span of the last piece]`).
#[derive(Debug, Clone, PartialEq)]
pub struct CompositeCurve {
    pub segs: Vec<CompositeSeg>,
    /// Continuity at the joint between piece i and i + 1.
    pub joints: Vec<Continuity>,
    pub starts: Vec<f64>,
}

/// Relative tolerance of the joint continuity tests.
const CONT_TOL: f64 = 1e-8;

/// Continuity of the joint between two curves at `a`'s `ta` and `b`'s `tb`
/// (positions assumed to meet).
pub fn joint_continuity(a: &Curve, ta: f64, b: &Curve, tb: f64) -> Continuity {
    let da = a.derivs(ta, 2);
    let db = b.derivs(tb, 2);
    let close = |x: Vec3, y: Vec3| (x - y).len() <= CONT_TOL * (1.0 + x.len().max(y.len()));
    let (ta1, tb1) = (da[1].normalize(), db[1].normalize());
    if !(da[1].len() > 0.0 && db[1].len() > 0.0) || (ta1 - tb1).len() > CONT_TOL.sqrt() * 1e-2 {
        return Continuity::C0;
    }
    let c1 = close(da[1], db[1]);
    if c1 && close(da[2], db[2]) {
        return Continuity::C2;
    }
    let kv = |d: &[Vec3]| {
        let v2 = d[1].len2();
        (d[2] - d[1] * (d[2].dot(d[1]) / v2)) / v2
    };
    if close(kv(&da), kv(&db)) {
        return Continuity::G2;
    }
    if c1 {
        Continuity::C1
    } else {
        Continuity::G1
    }
}

impl CompositeCurve {
    /// Pieces `(curve, t0, t1)` in order, each end within `gap_tol` of the next
    /// start. `Err` names the first bad piece or joint.
    pub fn new(pieces: Vec<(Curve, f64, f64)>, gap_tol: f64) -> Result<CompositeCurve, String> {
        if pieces.is_empty() {
            return Err("a composite curve needs at least one piece".into());
        }
        let mut starts = Vec::with_capacity(pieces.len());
        let mut s = 0.0;
        for (i, (c, t0, t1)) in pieces.iter().enumerate() {
            if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
                return Err(format!("piece {i}: parameter range [{t0}, {t1}] is not a finite increasing interval"));
            }
            if c.nesting() >= MAX_NESTING {
                return Err(format!("piece {i}: nested more than {MAX_NESTING} levels"));
            }
            starts.push(s);
            s += t1 - t0;
        }
        let mut joints = Vec::new();
        for i in 1..pieces.len() {
            let (a, _, a1) = &pieces[i - 1];
            let (b, b0, _) = &pieces[i];
            let gap = a.eval(*a1).dist(b.eval(*b0));
            if !(gap <= gap_tol) {
                return Err(format!("joint {i}: pieces are {gap:.3e} apart (tolerance {gap_tol:.3e})"));
            }
            joints.push(joint_continuity(a, *a1, b, *b0));
        }
        let segs = pieces.into_iter().map(|(curve, t0, t1)| CompositeSeg { curve, t0, t1 }).collect();
        Ok(CompositeCurve { segs, joints, starts })
    }

    /// Composite parameter range `[0, L]`.
    pub fn range(&self) -> (f64, f64) {
        let l = self.segs.last().unwrap();
        (0.0, self.starts[self.starts.len() - 1] + (l.t1 - l.t0))
    }

    /// Piece index and its local parameter at composite parameter `s` (a joint
    /// belongs to the piece that starts there; outside the range the end pieces
    /// extend).
    pub fn locate(&self, s: f64) -> (usize, f64) {
        let i = match self.starts.binary_search_by(|x| x.partial_cmp(&s).unwrap_or(std::cmp::Ordering::Less)) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) => i - 1,
        };
        let g = &self.segs[i];
        (i, g.t0 + (s - self.starts[i]))
    }

    /// Composite parameter of piece `i`'s local `t`.
    pub fn global(&self, i: usize, t: f64) -> f64 {
        self.starts[i] + (t - self.segs[i].t0)
    }

    /// Composite parameters of the joints (interior breaks).
    pub fn breaks(&self) -> Vec<f64> {
        self.starts[1..].to_vec()
    }

    /// End meets start within `tol`.
    pub fn is_closed(&self, tol: f64) -> bool {
        let (a, b) = (self.segs.first().unwrap(), self.segs.last().unwrap());
        a.curve.eval(a.t0).dist(b.curve.eval(b.t1)) <= tol
    }

    /// The weakest joint continuity (C2 for a single piece).
    pub fn min_continuity(&self) -> Continuity {
        self.joints.iter().copied().min_by_key(|c| c.rank()).unwrap_or(Continuity::C2)
    }

    pub fn reversed(&self) -> CompositeCurve {
        let pieces: Vec<(Curve, f64, f64)> = self
            .segs
            .iter()
            .rev()
            .map(|g| {
                let r = g.curve.reversed();
                let (a, b) = (g.curve.reversed_param(g.t1), g.curve.reversed_param(g.t0));
                (r, a.min(b), a.max(b))
            })
            .collect();
        let mut starts = Vec::new();
        let mut s = 0.0;
        for p in &pieces {
            starts.push(s);
            s += p.2 - p.1;
        }
        let joints = self.joints.iter().rev().copied().collect();
        let segs = pieces.into_iter().map(|(curve, t0, t1)| CompositeSeg { curve, t0, t1 }).collect();
        CompositeCurve { segs, joints, starts }
    }

    /// Exact NURBS of the composite over `[s0, s1]`: the pieces' own NURBS
    /// (exact for lines, arcs and splines) joined at the breaks.
    pub fn to_nurbs(&self, s0: f64, s1: f64) -> BSplineCurve {
        let (i0, _) = self.locate(s0);
        let (i1, _) = self.locate(s1);
        let mut i1 = i1;
        // s1 on a joint: the piece ending there, not the empty start of the next
        if i1 > i0 && (s1 - self.starts[i1]).abs() <= 1e-14 * (1.0 + s1.abs()) {
            i1 -= 1;
        }
        let mut parts = Vec::new();
        for i in i0..=i1 {
            let g = &self.segs[i];
            let a = if i == i0 { self.locate(s0).1 } else { g.t0 };
            let b = if i == i1 { g.t0 + (s1 - self.starts[i]) } else { g.t1 };
            if b - a <= 0.0 {
                continue;
            }
            let n = crate::convert::curve_to_nurbs(&g.curve, a, b).clamp_ends();
            parts.push(n.reparam(self.global(i, a), self.global(i, b)));
        }
        join_nurbs(parts)
    }
}

/// Clamped B-splines laid end to end on adjacent domains joined into one
/// (degrees raised to the highest, all rational if one is; a joint gap is
/// closed at the midpoint).
pub fn join_nurbs(parts: Vec<BSplineCurve>) -> BSplineCurve {
    let p = parts.iter().map(|c| c.degree).max().unwrap_or(1);
    let rational = parts.iter().any(|c| c.weights.is_some());
    let mut out: Option<BSplineCurve> = None;
    for c in parts {
        let mut c = c.elevate(p - c.degree).clamp_ends();
        if rational && c.weights.is_none() {
            c.weights = Some(vec![1.0; c.ctrl.len()]);
        }
        match out.as_mut() {
            None => out = Some(c),
            Some(o) => {
                // weights scaled so the shared point has one weight
                if let (Some(wo), Some(wc)) = (o.weights.as_ref(), c.weights.as_mut()) {
                    let k = wo[wo.len() - 1] / wc[0];
                    wc.iter_mut().for_each(|w| *w *= k);
                }
                let last = o.ctrl.len() - 1;
                o.ctrl[last] = (o.ctrl[last] + c.ctrl[0]) * 0.5;
                o.knots.pop();
                o.knots.extend_from_slice(&c.knots[p + 1..]);
                o.ctrl.extend_from_slice(&c.ctrl[1..]);
                if let (Some(wo), Some(wc)) = (o.weights.as_mut(), c.weights.as_ref()) {
                    wo.extend_from_slice(&wc[1..]);
                }
            }
        }
    }
    out.unwrap_or_else(|| crate::convert::segment(Vec3::ZERO, Vec3::ZERO, 0.0, 1.0))
}

/// The plane normal or surface an offset curve is taken against.
#[derive(Debug, Clone, PartialEq)]
pub enum OffsetRef {
    /// P = C + d·unit(C' × N), N fixed.
    Normal(Vec3),
    /// P = C + d·unit(C' × n_S(foot of C)): an offset in the tangent plane of
    /// the surface (on its natural normal's left of travel for d < 0).
    Surface(Box<Surface>),
}

/// Deepest nesting of procedural definitions (bounds the recursive dispatch,
/// as solidcore's `MAX_GEOMETRY_NESTING`).
pub const MAX_NESTING: usize = 16;

/// Central-difference step for derivatives past the exact order.
fn fd_step(t: f64) -> f64 {
    1e-4 * (1.0 + t.abs())
}

/// `[C, ..., C^(n)]` from an exact second-order oracle: orders 3 and 4 by
/// central differences of the second derivative, higher orders zero.
pub(crate) fn derivs_from2(d2: &dyn Fn(f64) -> [Vec3; 3], t: f64, n: usize) -> Vec<Vec3> {
    let c = d2(t);
    let mut v: Vec<Vec3> = c[..=n.min(2)].to_vec();
    if n >= 3 {
        let h = fd_step(t);
        let (a, b) = (d2(t + h)[2], d2(t - h)[2]);
        v.push((a - b) / (2.0 * h));
        if n >= 4 {
            v.push((a - c[2] * 2.0 + b) / (h * h));
        }
        v.resize(n + 1, Vec3::ZERO);
    }
    v
}

/// Foot `(u, v)` of `q` on `s` with its derivative along a curve through q:
/// the foot of the closest point (Newton from `hint`, else the global search)
/// and `(u', v')` from the implicit function theorem on
/// `[(S − q)·Su, (S − q)·Sv] = 0` with q moving at `dq`.
fn foot_jet(s: &Surface, q: Vec3, dq: Vec3, hint: Option<(f64, f64)>) -> ((f64, f64), (f64, f64), SurfDerivs) {
    let (u, v) = match hint {
        Some(h) => {
            let (u, v, _) = s.closest_point_near(q, h);
            (u, v)
        }
        None => s.closest_param(q),
    };
    let d = s.derivs2(u, v);
    let r = d.p - q;
    let j11 = d.du.dot(d.du) + r.dot(d.duu);
    let j12 = d.du.dot(d.dv) + r.dot(d.duv);
    let j22 = d.dv.dot(d.dv) + r.dot(d.dvv);
    let (f, g) = (dq.dot(d.du), dq.dot(d.dv));
    let det = j11 * j22 - j12 * j12;
    let uv1 = if det.abs() > 1e-300 { ((f * j22 - g * j12) / det, (g * j11 - f * j12) / det) } else { (0.0, 0.0) };
    ((u, v), uv1, d)
}

/// Unit normal of `s` and its derivatives along (u', v'): n_u = (W_u − n(n·W_u))/|W|, W = Su × Sv.
/// The unit cross-product normal Su × Sv (the one [`normal_rate`] differentiates, so
/// evaluation and derivatives agree on every frame handedness); `Surface::normal` at a
/// degenerate point.
pub(crate) fn cross_normal(s: &Surface, u: f64, v: f64) -> Vec3 {
    let d = s.derivs1(u, v);
    let w = d.du.cross(d.dv);
    let l = w.len();
    if l > 1e-300 {
        w / l
    } else {
        s.normal(u, v)
    }
}

fn normal_rate(d: &SurfDerivs, uv1: (f64, f64)) -> (Vec3, Vec3) {
    let w0 = d.du.cross(d.dv);
    let w = w0.len();
    if !(w > 1e-300) {
        return (Vec3::ZERO, Vec3::ZERO);
    }
    let n = w0 / w;
    let wu = d.duu.cross(d.dv) + d.du.cross(d.duv);
    let wv = d.duv.cross(d.dv) + d.du.cross(d.dvv);
    let nu = (wu - n * n.dot(wu)) / w;
    let nv = (wv - n * n.dot(wv)) / w;
    (n, nu * uv1.0 + nv * uv1.1)
}

/// Offset with a fixed normal to second order from the base's third:
/// B = C' × N, u = B/|B|, u' = (B' − u b')/b, u'' = (B'' − 2u'b' − u b'')/b,
/// b' = u·B', b'' = u'·B' + u·B''.
fn offset_normal_d2(base: &Curve, dist: f64, nrm: Vec3, t: f64) -> [Vec3; 3] {
    let c = base.derivs(t, 3);
    let (b0, b1, b2) = (c[1].cross(nrm), c[2].cross(nrm), c[3].cross(nrm));
    let b = b0.len();
    if !(b > 1e-300) {
        return [c[0], c[1], c[2]];
    }
    let u = b0 / b;
    let bp = u.dot(b1);
    let u1 = (b1 - u * bp) / b;
    let bpp = u1.dot(b1) + u.dot(b2);
    let u2 = (b2 - u1 * (2.0 * bp) - u * bpp) / b;
    [c[0] + u * dist, c[1] + u1 * dist, c[2] + u2 * dist]
}

/// Offset in a surface's tangent planes to first order (exact), the point and
/// first derivative; `hint` the foot of a nearby point.
fn offset_surface_d1(base: &Curve, dist: f64, s: &Surface, t: f64, hint: Option<(f64, f64)>) -> ([Vec3; 2], (f64, f64)) {
    let c = base.derivs(t, 2);
    let (uv, uv1, d) = foot_jet(s, c[0], c[1], hint);
    let (n, n1) = normal_rate(&d, uv1);
    let b0 = c[1].cross(n);
    let b1 = c[2].cross(n) + c[1].cross(n1);
    let b = b0.len();
    if !(b > 1e-300) {
        return ([c[0], c[1]], uv);
    }
    let u = b0 / b;
    let u1 = (b1 - u * u.dot(b1)) / b;
    ([c[0] + u * dist, c[1] + u1 * dist], uv)
}

/// Normal projection: P = S(foot), P' = Su u' + Sv v'.
fn projected_normal_d1(base: &Curve, s: &Surface, t: f64, hint: Option<(f64, f64)>) -> ([Vec3; 2], (f64, f64)) {
    let c = base.derivs(t, 1);
    let (uv, uv1, d) = foot_jet(s, c[0], c[1], hint);
    ([d.p, d.du * uv1.0 + d.dv * uv1.1], uv)
}

/// Projection along `dir`: S(u, v) = C(t) + λ·D solved by Newton from the
/// closest point; derivatives from [Su Sv −D] x' = C' and
/// [Su Sv −D] x'' = C'' − (Suu u'² + 2Suv u'v' + Svv v'²). Exact to order 2.
fn projected_dir_d2(base: &Curve, s: &Surface, dir: Vec3, t: f64) -> [Vec3; 3] {
    projected_dir_solve(base, s, dir, t).0
}

/// [`projected_dir_d2`] and the residual |S(u, v) − C(t) − λD| of the solve (large
/// where the line through C(t) along D misses the surface).
pub(crate) fn projected_dir_solve(base: &Curve, s: &Surface, dir: Vec3, t: f64) -> ([Vec3; 3], f64) {
    let c = base.derivs(t, 2);
    let (mut u, mut v) = s.closest_param(c[0]);
    let mut lam = (s.eval(u, v) - c[0]).dot(dir) / dir.len2().max(1e-300);
    let ((u0, u1), (v0, v1)) = s.domain();
    for _ in 0..40 {
        let d = s.derivs1(u, v);
        let f = d.p - c[0] - dir * lam;
        let Some(x) = solve3(d.du, d.dv, -dir, -f) else { break };
        let (nu, nv) = ((u + x[0]).clamp(u0, u1), (v + x[1]).clamp(v0, v1));
        let done = (nu - u).abs() <= 1e-15 * (1.0 + u.abs()) && (nv - v).abs() <= 1e-15 * (1.0 + v.abs());
        u = nu;
        v = nv;
        lam += x[2];
        if done {
            break;
        }
    }
    let d = s.derivs2(u, v);
    let res = (d.p - c[0] - dir * lam).len();
    let Some(x1) = solve3(d.du, d.dv, -dir, c[1]) else { return ([d.p, Vec3::ZERO, Vec3::ZERO], res) };
    let (a1, b1) = (x1[0], x1[1]);
    let q = d.duu * (a1 * a1) + d.duv * (2.0 * a1 * b1) + d.dvv * (b1 * b1);
    let x2 = solve3(d.du, d.dv, -dir, c[2] - q).unwrap_or([0.0; 3]);
    ([d.p, d.du * a1 + d.dv * b1, q + d.du * x2[0] + d.dv * x2[1]], res)
}

/// Is the definition of `c` solvable over `[t0, t1]` (sampled at 33 points):
/// finite points, and a directional projection that meets its surface.
pub fn is_defined_on(c: &Curve, t0: f64, t1: f64) -> bool {
    (0..=32).all(|i| {
        let t = t0 + (t1 - t0) * i as f64 / 32.0;
        let p = c.eval(t);
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return false;
        }
        match c {
            Curve::Projected { base, surf, dir: Some(d) } => projected_dir_solve(base, surf, *d, t).1 <= 1e-9 * (1.0 + p.len()),
            Curve::Composite(cc) => {
                let (i, lt) = cc.locate(t);
                !cc.segs[i].curve.is_procedural() || is_defined_on(&cc.segs[i].curve, lt, lt)
            }
            Curve::Offset { base, .. } => !base.is_procedural() || is_defined_on(base, t, t),
            _ => true,
        }
    })
}

/// Solve [a b c] x = r (Cramer), None when singular.
fn solve3(a: Vec3, b: Vec3, c: Vec3, r: Vec3) -> Option<[f64; 3]> {
    let det = a.dot(b.cross(c));
    let scale = a.len() * b.len() * c.len();
    if !(det.abs() > 1e-14 * scale) {
        return None;
    }
    Some([r.dot(b.cross(c)) / det, a.dot(r.cross(c)) / det, a.dot(b.cross(r)) / det])
}

/// [P, P', P''] of a kind exact to first order: P'' by a central difference
/// of the exact P' (the foot of the neighbours is tracked from the centre's).
fn d2_from_d1(d1: &dyn Fn(f64, Option<(f64, f64)>) -> ([Vec3; 2], (f64, f64)), t: f64) -> [Vec3; 3] {
    let (c, uv) = d1(t, None);
    let h = fd_step(t);
    let (a, _) = d1(t + h, Some(uv));
    let (b, _) = d1(t - h, Some(uv));
    [c[0], c[1], (a[1] - b[1]) / (2.0 * h)]
}

/// The exact second-order oracle of a procedural curve (None for the other kinds).
pub(crate) fn proc_d2(c: &Curve, t: f64) -> Option<[Vec3; 3]> {
    Some(match c {
        Curve::Composite(cc) => {
            let (i, lt) = cc.locate(t);
            cc.segs[i].curve.derivs2_arr(lt)
        }
        Curve::Offset { base, dist, dir: OffsetRef::Normal(n) } => offset_normal_d2(base, *dist, *n, t),
        Curve::Offset { base, dist, dir: OffsetRef::Surface(s) } => d2_from_d1(&|t, h| offset_surface_d1(base, *dist, s, t, h), t),
        Curve::Projected { base, surf, dir: None } => d2_from_d1(&|t, h| projected_normal_d1(base, surf, t, h), t),
        Curve::Projected { base, surf, dir: Some(d) } => projected_dir_d2(base, surf, *d, t),
        Curve::OnSurface { surf, uv } => crate::pcurve::CurveOnSurface::new(uv, surf).derivs(t),
        Curve::Iso { surf, fixed_u, at, rev } => {
            let (sg, x) = if *rev { (-1.0, -t) } else { (1.0, t) };
            if *fixed_u {
                let d = surf.derivs2(*at, x);
                [d.p, d.dv * sg, d.dvv]
            } else {
                let d = surf.derivs2(x, *at);
                [d.p, d.du * sg, d.duu]
            }
        }
        _ => return None,
    })
}

/// `[C, ..., C^(n)]` of a procedural curve (None for the other kinds).
pub(crate) fn proc_derivs(c: &Curve, t: f64, n: usize) -> Option<Vec<Vec3>> {
    match c {
        Curve::Composite(cc) => {
            let (i, lt) = cc.locate(t);
            Some(cc.segs[i].curve.derivs(lt, n))
        }
        Curve::Iso { surf, fixed_u, at, rev } if n == 3 => {
            let (sg, x) = if *rev { (-1.0, -t) } else { (1.0, t) };
            let d = proc_d2(c, t)?;
            let th = if *fixed_u { surf.thirds(*at, x).dvvv } else { surf.thirds(x, *at).duuu };
            Some(vec![d[0], d[1], d[2], th * sg])
        }
        Curve::OnSurface { .. } | Curve::Offset { .. } | Curve::Projected { .. } | Curve::Iso { .. } => {
            Some(derivs_from2(&|t| proc_d2(c, t).unwrap(), t, n))
        }
        _ => None,
    }
}

/// Natural range of a procedural curve.
pub(crate) fn proc_range(c: &Curve) -> Option<(f64, f64)> {
    Some(match c {
        Curve::Composite(cc) => cc.range(),
        Curve::Offset { base, .. } | Curve::Projected { base, .. } => base.range(),
        Curve::OnSurface { uv, .. } => uv.range(),
        Curve::Iso { surf, fixed_u, rev, .. } => {
            let (ur, vr) = surf.domain();
            let r = if *fixed_u { vr } else { ur };
            if *rev {
                (-r.1, -r.0)
            } else {
                r
            }
        }
        _ => return None,
    })
}

/// A finite interval to search over: the range, or a window around `seed`.
fn finite_window(r: (f64, f64), seed: f64) -> (f64, f64) {
    let w = 1e3 * (1.0 + seed.abs());
    let a = if r.0.is_finite() { r.0 } else { seed - w };
    let b = if r.1.is_finite() { r.1 } else { seed + w };
    (a, b)
}

/// Closest parameter of a procedural curve: per piece for a composite; else a
/// scan of the range (seeded by the base curve's foot for an offset or a
/// projection) refined by Newton on the exact derivatives.
pub(crate) fn proc_closest(c: &Curve, q: Vec3) -> Option<f64> {
    let (lo, hi) = proc_range(c)?;
    if let Curve::Composite(cc) = c {
        let mut best = (f64::INFINITY, 0.0);
        for (i, g) in cc.segs.iter().enumerate() {
            let seed = g.curve.closest_param(q);
            // a periodic piece's foot may come back a period away from its range
            let seed = match g.curve.period() {
                Some(p) if !(seed >= g.t0 && seed <= g.t1) => {
                    let k = ((g.t0 - seed) / p).ceil();
                    seed + k * p
                }
                _ => seed,
            };
            let mut cands = vec![g.t0, g.t1];
            if seed >= g.t0 && seed <= g.t1 {
                cands.push(seed);
            }
            for t in cands {
                let t = crate::nurbs::bspline::newton_closest(|t| g.curve.derivs(t, 2), q, t, g.t0, g.t1);
                let d = g.curve.eval(t).dist(q);
                if d < best.0 {
                    best = (d, cc.global(i, t));
                }
            }
        }
        return Some(best.1);
    }
    let mut seeds = Vec::new();
    if let Curve::Offset { base, .. } | Curve::Projected { base, .. } = c {
        seeds.push(base.closest_param(q));
    }
    // a curve on a surface: the surface's foot of q, carried into the curve's parameter
    match c {
        Curve::OnSurface { surf, uv } => {
            let (u, v) = surf.closest_param(q);
            seeds.push(uv.closest_param(Vec2::new(u, v)));
        }
        Curve::Iso { surf, fixed_u, rev, .. } => {
            let (u, v) = surf.closest_param(q);
            let x = if *fixed_u { v } else { u };
            seeds.push(if *rev { -x } else { x });
        }
        _ => {}
    }
    let mid = seeds.first().copied().unwrap_or(0.0);
    let (a, b) = finite_window((lo, hi), if lo.is_finite() && hi.is_finite() { 0.5 * (lo + hi) } else { mid });
    let n = 32;
    let mut best = (f64::INFINITY, a);
    for i in 0..=n {
        let t = a + (b - a) * i as f64 / n as f64;
        let d = c.eval(t).dist(q);
        if d < best.0 {
            best = (d, t);
        }
    }
    seeds.push(best.1);
    let mut out = (f64::INFINITY, best.1);
    for s in seeds {
        let s = s.clamp(a, b);
        let t = crate::nurbs::bspline::newton_closest(|t| proc_d2(c, t).unwrap(), q, s, lo, hi);
        let d = c.eval(t).dist(q);
        if d < out.0 {
            out = (d, t);
        }
    }
    let t = out.1;
    Some(match c.period() {
        Some(p) => t.rem_euclid(p),
        None => t,
    })
}

/// Parameters strictly inside `(a, b)` where the speed |C'| may jump: the joints of a
/// composite, carried through the offset or projection of one.
fn speed_breaks(c: &Curve, a: f64, b: f64, out: &mut Vec<f64>) {
    match c {
        Curve::Composite(cc) => out.extend(cc.breaks().into_iter().filter(|&x| x > a && x < b)),
        Curve::Offset { base, .. } | Curve::Projected { base, .. } => speed_breaks(base, a, b, out),
        _ => {}
    }
}

const GK_X: [f64; 8] = [
    0.991455371120812639206854697526329,
    0.949107912342758524526189684047851,
    0.864864423359769072789712788640926,
    0.741531185599394439863864773280788,
    0.586087235467691130294144838258730,
    0.405845151377397166906606412076961,
    0.207784955007898467600689403773245,
    0.0,
];
const GK_WK: [f64; 8] = [
    0.022935322010529224963732008058970,
    0.063092092629978553290700663189204,
    0.104790010322250183839876322541518,
    0.140653259715525918745189590510238,
    0.169004726639267902826583426598550,
    0.190350578064785409913256402421014,
    0.204432940075298892414161999234649,
    0.209482141084727828012999174891714,
];
/// 7-point Gauss weights at `GK_X[1], [3], [5], [7]`.
const GK_WG: [f64; 4] = [
    0.129484966168869693270611432679082,
    0.279705391489276667901467771423780,
    0.381830050505118944950369775488975,
    0.417959183673469387755102040816327,
];

/// Gauss–Kronrod (7, 15) estimate of ∫|C'| over [a, b]: (Kronrod value, |Kronrod − Gauss|).
fn gk15(c: &Curve, a: f64, b: f64) -> (f64, f64) {
    let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
    let f0 = c.deriv(m).len();
    let (mut k, mut g) = (GK_WK[7] * f0, GK_WG[3] * f0);
    for i in 0..7 {
        let dx = h * GK_X[i];
        let f = c.deriv(m - dx).len() + c.deriv(m + dx).len();
        k += GK_WK[i] * f;
        if i % 2 == 1 {
            g += GK_WG[i / 2] * f;
        }
    }
    (k * h.abs(), ((k - g) * h).abs())
}

/// Adaptive bisection until the local error estimate is within its share of `tol`
/// (the error estimate of GK15 is conservative; the true error is far smaller).
fn gk_adaptive(c: &Curve, a: f64, b: f64, tol: f64, depth: u32, total: f64) -> f64 {
    let (k, e) = gk15(c, a, b);
    let share = tol * ((b - a).abs() / total);
    if e <= share || depth >= 30 {
        return k;
    }
    let m = 0.5 * (a + b);
    gk_adaptive(c, a, m, tol, depth + 1, total) + gk_adaptive(c, m, b, tol, depth + 1, total)
}

/// Arc length of a procedural curve over [t0, t1] (either order; the length is
/// not negative). A composite is measured piece by piece; any other kind is split
/// at the joints of the composite it is built on (the speed jumps there) and each
/// smooth span integrated by adaptive Gauss–Kronrod (7, 15) with the error
/// estimate held to 1e-12 of the length, so the result is exact to about that
/// relative error wherever the speed is smooth between the breaks.
pub(crate) fn proc_length(c: &Curve, t0: f64, t1: f64) -> Option<f64> {
    let (a, b) = if t1 >= t0 { (t0, t1) } else { (t1, t0) };
    if let Curve::Composite(cc) = c {
        let mut s = 0.0;
        for (i, g) in cc.segs.iter().enumerate() {
            let (gs, ge) = (cc.starts[i], cc.starts[i] + (g.t1 - g.t0));
            let lo = if i == 0 { a } else { a.max(gs) };
            let hi = if i + 1 == cc.segs.len() { b } else { b.min(ge) };
            if hi > lo {
                s += g.curve.length_between(g.t0 + (lo - gs), g.t0 + (hi - gs));
            }
        }
        return Some(s);
    }
    proc_range(c)?;
    let mut ks = vec![a];
    let mut br = Vec::new();
    speed_breaks(c, a, b, &mut br);
    br.sort_by(|x, y| x.partial_cmp(y).unwrap());
    br.dedup_by(|x, y| (*x - *y).abs() < 1e-14);
    ks.extend(br);
    ks.push(b);
    let mut s = 0.0;
    for w in ks.windows(2) {
        // start from four panels so a feature cannot hide between the sample nodes
        let n = 4;
        let est: f64 = (0..n).map(|i| gk15(c, w[0] + (w[1] - w[0]) * i as f64 / n as f64, w[0] + (w[1] - w[0]) * (i + 1) as f64 / n as f64).0).sum();
        let tol = 1e-12 * est.max(1e-300);
        for i in 0..n {
            let lo = w[0] + (w[1] - w[0]) * i as f64 / n as f64;
            let hi = w[0] + (w[1] - w[0]) * (i + 1) as f64 / n as f64;
            s += gk_adaptive(c, lo, hi, tol, 0, w[1] - w[0]);
        }
    }
    Some(s)
}

/// Parameter of the reversed curve at the point of this curve's `t`
/// (procedural kinds).
pub(crate) fn proc_reversed_param(c: &Curve, t: f64) -> Option<f64> {
    Some(match c {
        Curve::Composite(cc) => cc.range().1 - t,
        Curve::Offset { base, .. } | Curve::Projected { base, .. } => base.reversed_param(t),
        Curve::OnSurface { uv, .. } => curve2_reversed_param(uv, t),
        Curve::Iso { .. } => -t,
        _ => return None,
    })
}

/// Curve2::reversed's parameter map: lines −t, conics 2π − t, splines a + b − t.
pub fn curve2_reversed_param(c: &Curve2, t: f64) -> f64 {
    match c {
        Curve2::Line { .. } => -t,
        Curve2::Circle { .. } | Curve2::Ellipse { .. } => std::f64::consts::TAU - t,
        Curve2::BSpline(b) => {
            let (a, e) = b.domain();
            a + e - t
        }
    }
}

pub(crate) fn proc_reversed(c: &Curve) -> Option<Curve> {
    Some(match c {
        Curve::Composite(cc) => Curve::Composite(Box::new(cc.reversed())),
        // C' flips, so C' × N flips: the reversed offset is the reversed base offset by −d
        Curve::Offset { base, dist, dir } => Curve::Offset { base: Box::new(base.reversed()), dist: -dist, dir: dir.clone() },
        Curve::Projected { base, surf, dir } => Curve::Projected { base: Box::new(base.reversed()), surf: surf.clone(), dir: *dir },
        Curve::OnSurface { surf, uv } => Curve::OnSurface { surf: surf.clone(), uv: uv.reversed() },
        Curve::Iso { surf, fixed_u, at, rev } => Curve::Iso { surf: surf.clone(), fixed_u: *fixed_u, at: *at, rev: !rev },
        _ => return None,
    })
}

/// Map `c` by the affine `f` (linear part `g`, a similarity), keeping its
/// parameterisation: `result.eval(t) = f(c.eval(t))` (a curve whose own
/// transform reverses its sense under a mirror, an arc's frame made right
/// handed again, is reversed back).
pub(crate) fn transform_keep_param(c: &Curve, f: &dyn Fn(Vec3) -> Vec3, g: &dyn Fn(Vec3) -> Vec3, probe: (f64, f64)) -> Curve {
    let m = c.transform_dyn(f, g);
    let (a, b) = finite_window(probe, 0.0);
    let t = a + 0.37 * (b - a);
    let want = f(c.eval(t));
    let scale = 1.0 + want.len();
    if m.eval(t).dist(want) <= 1e-9 * scale {
        return m;
    }
    // an arc's frame made right handed again by the transform runs the other way:
    // the reversal of the mapped curve runs at the original parameter again
    let r = m.reversed();
    if r.eval(t).dist(want) <= 1e-9 * scale {
        return r;
    }
    m
}

pub(crate) fn proc_transform(c: &Curve, f: &dyn Fn(Vec3) -> Vec3, g: &dyn Fn(Vec3) -> Vec3) -> Option<Curve> {
    let s = g(Vec3::X).len();
    let det = g(Vec3::X).dot(g(Vec3::Y).cross(g(Vec3::Z)));
    Some(match c {
        Curve::Composite(cc) => {
            let segs: Vec<CompositeSeg> = cc
                .segs
                .iter()
                .map(|sg| CompositeSeg { curve: transform_keep_param(&sg.curve, f, g, (sg.t0, sg.t1)), t0: sg.t0, t1: sg.t1 })
                .collect();
            Curve::Composite(Box::new(CompositeCurve { segs, joints: cc.joints.clone(), starts: cc.starts.clone() }))
        }
        Curve::Offset { base, dist, dir } => {
            let b = transform_keep_param(base, f, g, base.range());
            let dir = match dir {
                // g(a × b) = det·(g a × g b)/s² for a similarity: N' = sign(det)·g(N)/s
                OffsetRef::Normal(n) => OffsetRef::Normal(g(*n) * (det.signum() / s)),
                OffsetRef::Surface(srf) => OffsetRef::Surface(Box::new(srf.transform_dyn(f, g))),
            };
            // a mirror flips the surface's natural normal with the cross product, so the
            // tangent-plane direction C' × n keeps its sense in both cases
            Curve::Offset { base: Box::new(b), dist: dist * s, dir }
        }
        Curve::Projected { base, surf, dir } => Curve::Projected {
            base: Box::new(transform_keep_param(base, f, g, base.range())),
            surf: Box::new(surf.transform_dyn(f, g)),
            dir: dir.map(|d| g(d)),
        },
        Curve::OnSurface { surf, uv } => Curve::OnSurface { surf: Box::new(surf.transform_dyn(f, g)), uv: uv.clone() },
        Curve::Iso { surf, fixed_u, at, rev } => Curve::Iso { surf: Box::new(surf.transform_dyn(f, g)), fixed_u: *fixed_u, at: *at, rev: *rev },
        _ => return None,
    })
}

/// Exact content key of a 2D curve (bits of every field).
pub fn push_key2(out: &mut Vec<u64>, c: &Curve2) {
    let v2 = |out: &mut Vec<u64>, p: Vec2| out.extend([p.x.to_bits(), p.y.to_bits()]);
    match c {
        Curve2::Line { o, d } => {
            out.push(1);
            v2(out, *o);
            v2(out, *d);
        }
        Curve2::Circle { c, x, r, ccw } => {
            out.push(2);
            v2(out, *c);
            v2(out, *x);
            out.extend([r.to_bits(), *ccw as u64]);
        }
        Curve2::Ellipse { c, x, a, b, ccw } => {
            out.push(3);
            v2(out, *c);
            v2(out, *x);
            out.extend([a.to_bits(), b.to_bits(), *ccw as u64]);
        }
        Curve2::BSpline(b) => {
            out.extend([4, b.degree as u64, b.knots.len() as u64, b.ctrl.len() as u64]);
            out.extend(b.knots.iter().map(|k| k.to_bits()));
            for &p in &b.ctrl {
                v2(out, p);
            }
            match &b.weights {
                None => out.push(0),
                Some(w) => {
                    out.push(w.len() as u64 + 1);
                    out.extend(w.iter().map(|x| x.to_bits()));
                }
            }
        }
    }
}

pub(crate) fn proc_push_key(c: &Curve, out: &mut Vec<u64>) -> bool {
    match c {
        Curve::Composite(cc) => {
            out.extend([21, cc.segs.len() as u64]);
            for g in &cc.segs {
                g.curve.push_key(out);
                out.extend([g.t0.to_bits(), g.t1.to_bits()]);
            }
        }
        Curve::Offset { base, dist, dir } => {
            out.push(22);
            base.push_key(out);
            out.push(dist.to_bits());
            match dir {
                OffsetRef::Normal(n) => {
                    out.push(0);
                    push_vec3(out, *n);
                }
                OffsetRef::Surface(s) => {
                    out.push(1);
                    s.push_key(out);
                }
            }
        }
        Curve::Projected { base, surf, dir } => {
            out.push(23);
            base.push_key(out);
            surf.push_key(out);
            match dir {
                None => out.push(0),
                Some(d) => {
                    out.push(1);
                    push_vec3(out, *d);
                }
            }
        }
        Curve::OnSurface { surf, uv } => {
            out.push(24);
            surf.push_key(out);
            push_key2(out, uv);
        }
        Curve::Iso { surf, fixed_u, at, rev } => {
            out.extend([25, *fixed_u as u64, at.to_bits(), *rev as u64]);
            surf.push_key(out);
        }
        _ => return false,
    }
    true
}

/// Closed-form equivalent of an offset where one exists: the offset of a line
/// (a parallel line) and of a circle in its own plane (a concentric circle).
pub fn offset_exact(base: &Curve, dist: f64, dir: &OffsetRef) -> Option<Curve> {
    let OffsetRef::Normal(n) = dir else { return None };
    match base {
        Curve::Line { o, d } => {
            let b = d.cross(*n);
            (b.len() > 1e-12).then(|| Curve::Line { o: *o + b.normalize() * dist, d: *d })
        }
        Curve::Circle { frame, r } => {
            let nn = n.normalize();
            let c = nn.dot(frame.z);
            if (c.abs() - 1.0).abs() > 1e-12 {
                return None;
            }
            // C' × z is the outward radial direction: radius r + d (r − d for N = −z)
            let r2 = r + dist * c.signum();
            (r2 > 0.0).then(|| Curve::Circle { frame: *frame, r: r2 })
        }
        // a tangent-continuous chain of exact pieces offsets piece by piece, at the same parameters
        Curve::Composite(cc) if cc.joints.iter().all(|j| j.is_smooth()) => {
            let pieces = cc.segs.iter().map(|g| Some((offset_exact(&g.curve, dist, dir)?, g.t0, g.t1))).collect::<Option<Vec<_>>>()?;
            let scale = 1.0 + dist.abs() + pieces.iter().map(|p| p.0.eval(p.1).len()).fold(0.0, f64::max);
            CompositeCurve::new(pieces, 1e-9 * scale).ok().map(|k| Curve::Composite(Box::new(k)))
        }
        _ => None,
    }
}

/// Default tolerance of the on-demand B-spline of a curve piece: 1e-6 of its
/// size (at least 1e-7).
pub fn default_tol(c: &Curve, t0: f64, t1: f64) -> f64 {
    let pts = c.sample(t0, t1, 8);
    let (mut lo, mut hi) = (pts[0], pts[0]);
    for p in &pts {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (1e-6 * lo.dist(hi)).max(1e-7)
}

/// A B-spline of `c` over `[t0, t1]` at the same parameters within `tol`, and
/// its measured bound: the fit's error at its samples and their midpoints,
/// and an independent closest-point check of the definition against the fit
/// on a grid four times as dense, whichever is larger (a sampled bound).
/// Exact (bound 0) for the kinds with a closed form: an analytic curve, a
/// composite of exact pieces, the offset of a line, of a circle in its plane or
/// of a smooth chain of those. Bound 0 means the same point set with the end
/// parameters kept: a rational arc is not angle proportional inside.
/// `Err` when the definition has no solution somewhere on the range or the
/// fit does not reach `tol`.
pub fn certified_nurbs(c: &Curve, t0: f64, t1: f64, tol: f64) -> Result<(BSplineCurve, f64), String> {
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
        return Err(format!("certified_nurbs: [{t0}, {t1}] is not a finite increasing interval"));
    }
    match c {
        Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::BSpline(_) => {
            return Ok((crate::convert::curve_to_nurbs(c, t0, t1), 0.0))
        }
        Curve::Offset { base, dist, dir } => {
            if let Some(e) = offset_exact(base, *dist, dir) {
                return Ok((crate::convert::curve_to_nurbs(&e, t0, t1), 0.0));
            }
        }
        Curve::Composite(cc) if cc.segs.iter().all(|g| is_exact_kind(&g.curve)) => return Ok((cc.to_nurbs(t0, t1), 0.0)),
        _ => {}
    }
    if !is_defined_on(c, t0, t1) {
        return Err("certified_nurbs: the definition has no solution over the range (a projection misses its surface)".into());
    }
    let breaks: Vec<f64> = match c {

        Curve::Composite(cc) => cc.breaks().into_iter().filter(|&b| b > t0 && b < t1).collect(),
        _ => vec![],
    };
    let f = |t: f64| c.eval(t);
    let (b, err) = approximate_curve_fn(&f, (t0, t1), 3, tol, &breaks)?;
    let n = (4 * b.ctrl.len()).clamp(64, 512);
    let check = fn_curve_deviation(&f, (t0, t1), &b, n);
    let bound = err.max(check);
    if bound > tol {
        return Err(format!("certified_nurbs: measured deviation {bound:.3e} exceeds the tolerance {tol:.3e}"));
    }
    Ok((b, bound))
}

fn is_exact_kind(c: &Curve) -> bool {
    match c {
        Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::BSpline(_) => true,
        Curve::Offset { base, dist, dir } => offset_exact(base, *dist, dir).is_some(),
        Curve::Composite(cc) => cc.segs.iter().all(|g| is_exact_kind(&g.curve)),
        _ => false,
    }
}

/// The plain (non-procedural) curve an exporter writes for `c` over `[t0, t1]`:
/// `c` itself when it is not procedural, the closed form of an offset that has
/// one, else its B-spline ([`crate::convert::curve_to_nurbs`]; an unbounded
/// end is cut 10 from the other: an unbounded procedural carrier has no natural
/// extent, and edges are exported over their own finite range). Never panics.
pub fn export_form(c: &Curve, t0: f64, t1: f64) -> Curve {
    if !c.is_procedural() {
        return c.clone();
    }
    if let Curve::Offset { base, dist, dir } = c {
        if let Some(e) = offset_exact(base, *dist, dir).filter(|e| !e.is_procedural()) {
            return e;
        }
    }
    let (a, b) = match (t0.is_finite(), t1.is_finite()) {
        (true, true) => (t0, t1),
        (true, false) => (t0, t0 + 10.0),
        (false, true) => (t1 - 10.0, t1),
        (false, false) => (-10.0, 10.0),
    };
    let (a, b) = if b > a { (a, b) } else { (a, a + 1.0) };
    Curve::BSpline(crate::convert::curve_to_nurbs(c, a, b))
}

/// `curve_to_nurbs` of a procedural curve: the certified B-spline within the
/// default tolerance, or (when the fit cannot reach it) the 32-sample cubic
/// interpolant the conic kinds use.
pub(crate) fn proc_to_nurbs(c: &Curve, t0: f64, t1: f64) -> BSplineCurve {
    if let Curve::Composite(cc) = c {
        // per piece: exact pieces stay exact, procedural ones are fitted
        return cc.to_nurbs(t0, t1);
    }
    if let Curve::Offset { base, dist, dir } = c {
        if let Some(e) = offset_exact(base, *dist, dir) {
            return crate::convert::curve_to_nurbs(&e, t0, t1);
        }
    }
    let tol = default_tol(c, t0, t1);
    match certified_nurbs(c, t0, t1, tol) {
        Ok((b, _)) => b,
        Err(_) => crate::convert::sampled_cubic(c, t0, t1),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;
    use crate::{frame_from, Frame};
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    fn spline() -> Curve {
        let pts = vec![Vec3::new(0., 0., 0.), Vec3::new(1., 0.6, 0.), Vec3::new(2., 0.2, 0.), Vec3::new(3., 1.0, 0.), Vec3::new(4., 0.5, 0.)];
        Curve::BSpline(crate::fit::interpolate(&pts, 3, crate::fit::Param::ChordLength).unwrap())
    }
    fn cyl() -> Surface {
        Surface::Cylinder { frame: frame_from(Vec3::new(0.5, -0.2, 0.1), Vec3::new(0.1, 0.2, 1.0), Vec3::X), r: 3.0 }
    }
    fn sphere() -> Surface {
        Surface::Sphere { frame: Frame::identity(), r: 5.0 }
    }
    /// Line from (−2,0,0) to (0,0,0) then the quarter arc of radius 1 about (0,1,0) to (1,1,0): tangent at the joint.
    fn line_arc() -> Curve {
        let l = Curve::line_through(Vec3::new(-2., 0., 0.), Vec3::ZERO);
        let a = Curve::circle_x(Vec3::new(0., 1., 0.), Vec3::Z, Vec3::new(0., -1., 0.), 1.0);
        Curve::composite(vec![(l, 0.0, 2.0), (a, 0.0, FRAC_PI_2)], 1e-12).unwrap()
    }
    /// One of each procedural kind, with a parameter window inside its range.
    fn all() -> Vec<(Curve, (f64, f64))> {
        let sp = spline();
        let (a, b) = sp.range();
        let pl = Surface::plane(Vec3::new(0., 0., -2.), Vec3::new(0.1, 0.2, 1.0));
        let circ = Curve::circle(Vec3::new(0.3, 0.2, 1.0), Vec3::Z, 2.0);
        let on_cyl = Curve::line_through(Vec3::new(3.5, 0.0, 0.0), Vec3::new(3.6, 1.5, 2.0));
        vec![
            (line_arc(), (0.1, 2.0 + FRAC_PI_2 - 0.1)),
            (Curve::offset(sp.clone(), 0.3, Vec3::Z).unwrap(), (a + 0.1, b - 0.1)),
            (Curve::offset(Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 2.0), -0.4, Vec3::Z).unwrap(), (0.2, 6.0)),
            (Curve::offset_on(Curve::iso(cyl(), true, 0.7), 0.2, cyl()).unwrap(), (-1.0, 2.0)),
            (Curve::projected(sp.clone(), pl.clone(), None).unwrap(), (a + 0.1, b - 0.1)),
            (Curve::projected(circ.clone(), sphere(), None).unwrap(), (0.2, 6.0)),
            (Curve::projected(sp.clone(), pl, Some(Vec3::new(0.3, 0.1, -1.0))).unwrap(), (a + 0.1, b - 0.1)),
            (Curve::projected(on_cyl, cyl(), Some(Vec3::new(1.0, 0.1, 0.0))).unwrap(), (0.1, 2.0)),
            (Curve::on_surface(sphere(), Curve2::line_through(Vec2::new(0.2, -0.5), Vec2::new(2.0, 0.8))), (0.1, 1.9)),
            (Curve::iso(sphere(), false, 0.4), (0.2, 6.0)),
            (Curve::iso(cyl(), true, 1.1).reversed(), (-2.0, 1.0)),
        ]
    }

    #[test]
    fn offset_circle_is_a_circle() {
        let c = Curve::circle(Vec3::new(1., 2., 3.), Vec3::new(0.2, -0.3, 1.0), 2.0);
        let n = Vec3::new(0.2, -0.3, 1.0).normalize();
        for (d, nn, r) in [(0.5, n, 2.5), (0.5, -n, 1.5), (-1.25, n, 0.75)] {
            let o = Curve::offset(c.clone(), d, nn).unwrap();
            let Curve::Circle { frame, .. } = &c else { unreachable!() };
            let k = Curve::Circle { frame: *frame, r };
            for i in 0..=16 {
                let t = TAU * i as f64 / 16.0;
                assert!(vclose(o.eval(t), k.eval(t), 1e-14), "d={d} t={t}");
                let (a, b) = (o.derivs(t, 2), k.derivs(t, 2));
                assert!(vclose(a[1], b[1], 1e-13) && vclose(a[2], b[2], 1e-12), "d={d} t={t}");
            }
            let Curve::Offset { dir, .. } = &o else { unreachable!() };
            assert_eq!(offset_exact(&c, d, dir), Some(k.clone()));
            let (b, bound) = o.to_nurbs_within(0.3, 5.0, 1e-9).unwrap();
            assert_eq!(bound, 0.0);
            assert!(vclose(b.eval(0.3), k.eval(0.3), 1e-13) && vclose(b.eval(5.0), k.eval(5.0), 1e-13));
            assert!(close(o.length_between(0.0, TAU), TAU * r, 1e-12));
        }
        assert_eq!(Curve::offset(c.clone(), 0.5, n).unwrap().kind(), "offset");
    }

    #[test]
    fn offset_of_a_line() {
        let l = Curve::line(Vec3::new(1., 0., 0.), Vec3::new(1., 1., 0.));
        let o = Curve::offset(l.clone(), 2.0, Vec3::Z).unwrap();
        // C' × z for d = (1,1,0)/√2 is (1,−1,0)/√2
        let s = Vec3::new(1., -1., 0.).normalize() * 2.0;
        for &t in &[-3.0, 0.0, 0.7, 10.0] {
            assert!(vclose(o.eval(t), l.eval(t) + s, 1e-14));
            let d = o.derivs(t, 3);
            assert!(vclose(d[1], l.deriv(t), 1e-14) && d[2].len() < 1e-14 && d[3].len() < 1e-8);
        }
        assert_eq!(o.range(), (f64::NEG_INFINITY, f64::INFINITY));
        assert!(close(o.closest_param(Vec3::new(5., 1., 7.)), l.closest_param(Vec3::new(5., 1., 7.) - s), 1e-12));
        let e = offset_exact(&l, 2.0, &OffsetRef::Normal(Vec3::Z)).unwrap();
        for &t in &[-1.0, 2.0] {
            assert!(vclose(e.eval(t), o.eval(t), 1e-14));
        }
        // reversed: the reversed base offset by −d is the same set, at the base's reversed parameter
        let r = o.reversed();
        for &t in &[-1.0, 0.5, 3.0] {
            assert!(vclose(r.eval(o.reversed_param(t)), o.eval(t), 1e-14));
        }
    }

    #[test]
    fn composite_line_arc_reproduces_both_pieces() {
        let c = line_arc();
        let Curve::Composite(cc) = &c else { unreachable!() };
        assert_eq!(cc.joints, vec![Continuity::C1]);
        assert_eq!(c.range(), (0.0, 2.0 + FRAC_PI_2));
        assert_eq!(c.kind(), "composite");
        let l = Curve::line_through(Vec3::new(-2., 0., 0.), Vec3::ZERO);
        let a = Curve::circle_x(Vec3::new(0., 1., 0.), Vec3::Z, Vec3::new(0., -1., 0.), 1.0);
        for i in 0..=20 {
            let s = 2.0 * i as f64 / 20.0;
            if i < 20 {
                assert!(vclose(c.eval(s), l.eval(s), 1e-15));
            }
            let t = FRAC_PI_2 * i as f64 / 20.0;
            assert!(vclose(c.eval(2.0 + t), a.eval(t), 1e-15));
            assert!(vclose(c.derivs(2.0 + t, 3)[3], a.derivs(t, 3)[3], 1e-15));
        }
        // continuous through the joint, tangent continuous too
        let h = 1e-9;
        assert!(c.eval(2.0 - h).dist(c.eval(2.0 + h)) < 3e-9);
        assert!(vclose(c.deriv(2.0 - h), c.deriv(2.0 + h), 1e-8));
        assert!(close(c.length_between(0.0, 2.0 + FRAC_PI_2), 2.0 + FRAC_PI_2, 1e-13));
        assert!(close(c.length_between(1.0, 2.5), 1.5, 1e-13));
        // closest points on either piece
        assert!(close(c.closest_param(Vec3::new(-1.0, -0.5, 0.3)), 1.0, 1e-12));
        let q = a.eval(0.6) + (a.eval(0.6) - Vec3::new(0., 1., 0.)) * 0.4;
        assert!(close(c.closest_param(q), 2.6, 1e-10));
        // exact NURBS, deviation 0: the line and the rational arc joined
        let (b, bound) = c.to_nurbs_within(0.0, 2.0 + FRAC_PI_2, 1e-9).unwrap();
        assert_eq!(bound, 0.0);
        assert_eq!(b.domain(), (0.0, 2.0 + FRAC_PI_2));
        for i in 0..=40 {
            let s = (2.0 + FRAC_PI_2) * i as f64 / 40.0;
            let p = b.eval(s);
            let d = p.dist(c.eval(c.closest_param(p)));
            assert!(d < 1e-12, "s={s} d={d}");
        }
        assert!(vclose(b.eval(2.0), Vec3::ZERO, 1e-14));
        // partial range across the joint
        let b2 = crate::convert::curve_to_nurbs(&c, 1.0, 2.5);
        assert_eq!(b2.domain(), (1.0, 2.5));
        assert!(vclose(b2.eval(1.0), c.eval(1.0), 1e-14) && vclose(b2.eval(2.5), c.eval(2.5), 1e-14));
        // a corner is C0; a gap is refused
        let l2 = Curve::line_through(Vec3::ZERO, Vec3::new(0., 3., 0.));
        let k = Curve::composite(vec![(l.clone(), 0.0, 2.0), (l2.clone(), 0.0, 3.0)], 1e-9).unwrap();
        let Curve::Composite(kc) = &k else { unreachable!() };
        assert_eq!(kc.joints, vec![Continuity::C0]);
        assert!(Curve::composite(vec![(l, 0.0, 1.9), (l2, 0.0, 3.0)], 1e-9).is_err());
    }

    #[test]
    fn offset_of_a_smooth_composite_is_exact() {
        let c = line_arc();
        let o = Curve::offset(c.clone(), -0.25, Vec3::Z).unwrap();
        let Curve::Offset { base, dist, dir } = &o else { unreachable!() };
        let e = offset_exact(base, *dist, dir).unwrap();
        for i in 0..=20 {
            let s = (2.0 + FRAC_PI_2) * i as f64 / 20.0;
            assert!(vclose(e.eval(s), o.eval(s), 1e-14), "s={s}");
        }
        // the arc of radius 1 about (0,1,0) offset inwards by 0.25: radius 0.75
        assert!(close(e.eval(2.0 + 0.5).dist(Vec3::new(0., 1., 0.)), 0.75, 1e-14));
        let (b, bound) = o.to_nurbs_within(0.0, 2.0 + FRAC_PI_2, 1e-9).unwrap();
        assert_eq!(bound, 0.0);
        assert!(vclose(b.eval(2.0), o.eval(2.0), 1e-14));
        assert!(matches!(export_form(&o, 0.0, 3.0), Curve::BSpline(_)));
        // a corner has no single offset
        let k = Curve::composite(vec![(Curve::line_through(Vec3::ZERO, Vec3::X), 0.0, 1.0), (Curve::line_through(Vec3::X, Vec3::new(1., 1., 0.)), 0.0, 1.0)], 1e-12).unwrap();
        assert!(offset_exact(&k, 0.1, &OffsetRef::Normal(Vec3::Z)).is_none());
    }

    #[test]
    fn derivatives_match_finite_differences() {
        for (c, (a, b)) in all() {
            for i in 0..=6 {
                let t = a + (b - a) * (0.05 + 0.9 * i as f64 / 6.0);
                let h = 1e-5 * (1.0 + t.abs());
                let d = c.derivs(t, 2);
                assert!(vclose(d[0], c.eval(t), 1e-12), "{} t={t}", c.kind());
                let fd1 = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
                let fd2 = (c.deriv(t + h) - c.deriv(t - h)) / (2.0 * h);
                let sc = 1.0 + d[1].len() + d[2].len();
                assert!((d[1] - fd1).len() < 1e-6 * sc, "{} t={t}: {:?} vs {:?}", c.kind(), d[1], fd1);
                assert!((d[2] - fd2).len() < 1e-4 * sc, "{} t={t}: {:?} vs {:?}", c.kind(), d[2], fd2);
                let a2 = c.derivs2_arr(t);
                assert!(vclose(a2[1], d[1], 1e-12) && vclose(a2[2], d[2], 1e-9), "{}", c.kind());
            }
        }
    }

    #[test]
    fn certified_nurbs_within_bound() {
        for (c, (a, b)) in all() {
            let tol = 1e-6;
            let (bs, bound) = c.to_nurbs_within(a, b, tol).unwrap_or_else(|e| panic!("{}: {e}", c.kind()));
            assert!(bound <= tol, "{}: {bound}", c.kind());
            // the fit agrees with the definition at the same parameters (a fit) or as a point
            // set (an exact rational form, whose arcs are not angle proportional), also off the
            // fit's samples
            for i in 0..=97 {
                let t = a + (b - a) * i as f64 / 97.0;
                let d = if bound == 0.0 {
                    let p = bs.eval(t);
                    p.dist(c.eval(c.closest_param(p)))
                } else {
                    bs.eval(t).dist(c.eval(t))
                };
                assert!(d <= 2.0 * tol, "{} t={t}: {d}", c.kind());
            }
            // and curve_to_nurbs never fails
            let n = crate::convert::curve_to_nurbs(&c, a, b);
            assert!(vclose(n.eval(a), c.eval(a), 1e-6), "{}", c.kind());
        }
    }

    #[test]
    fn reversed_runs_backwards_on_the_same_points() {
        for (c, (a, b)) in all() {
            let r = c.reversed();
            assert_eq!(r.kind(), c.kind());
            for i in 0..=5 {
                let t = a + (b - a) * i as f64 / 5.0;
                let t2 = c.reversed_param(t);
                assert!(vclose(r.eval(t2), c.eval(t), 1e-9), "{} t={t}", c.kind());
                let (d, e) = (c.deriv(t), r.deriv(t2));
                assert!(vclose(d, -e, 1e-6), "{} t={t}: {d:?} {e:?}", c.kind());
            }
        }
    }

    #[test]
    fn transform_keeps_the_parameterisation() {
        // a rotation with a translation, and a mirror (the arc piece's frame is made right handed again)
        let rot = |p: Vec3| Vec3::new(p.y, -p.x, p.z);
        let mir = |p: Vec3| Vec3::new(-p.x, p.y, p.z);
        let f1 = |p: Vec3| rot(p) + Vec3::new(1., 2., 3.);
        let f2 = |p: Vec3| mir(p) + Vec3::new(0.5, 0., 0.);
        let maps: [(&dyn Fn(Vec3) -> Vec3, &dyn Fn(Vec3) -> Vec3); 2] = [(&f1, &rot), (&f2, &mir)];
        for (f, g) in maps {
            for (c, (a, b)) in all() {
                let m = c.transform_dyn(f, g);
                for i in 0..=4 {
                    let t = a + (b - a) * i as f64 / 4.0;
                    assert!(vclose(m.eval(t), f(c.eval(t)), 1e-9), "{} t={t}: {:?} {:?}", c.kind(), m.eval(t), f(c.eval(t)));
                }
            }
        }
    }

    #[test]
    fn closest_param_recovers_points() {
        for (c, (a, b)) in all() {
            for i in 1..5 {
                let t = a + (b - a) * i as f64 / 5.0;
                let p = c.eval(t);
                let s = c.closest_param(p);
                assert!(c.eval(s).dist(p) < 1e-8, "{} t={t} s={s}", c.kind());
            }
        }
    }

    #[test]
    fn projections_in_closed_form() {
        // a circle above a plane projects onto the plane as the same circle moved down
        let pl = Surface::plane(Vec3::ZERO, Vec3::Z);
        let c = Curve::circle(Vec3::new(1., 2., 5.), Vec3::Z, 1.5);
        let p = Curve::projected(c.clone(), pl.clone(), None).unwrap();
        for i in 0..8 {
            let t = TAU * i as f64 / 8.0;
            assert!(vclose(p.eval(t), c.eval(t) - Vec3::new(0., 0., 5.), 1e-13));
            assert!(vclose(p.deriv(t), c.deriv(t), 1e-12));
        }
        // along a direction: the line q + λD meets the plane z = 0
        let dir = Vec3::new(1.0, 0.0, -1.0);
        let pd = Curve::projected(c.clone(), pl, Some(dir)).unwrap();
        for i in 0..8 {
            let t = TAU * i as f64 / 8.0;
            let q = c.eval(t);
            assert!(vclose(pd.eval(t), q + Vec3::new(q.z, 0., -q.z), 1e-12));
        }
        // a circle of radius 4 at height 3 over a sphere of radius 5: on the sphere already
        let c = Curve::circle(Vec3::new(0., 0., 3.), Vec3::Z, 4.0);
        let p = Curve::projected(c.clone(), sphere(), None).unwrap();
        for i in 0..8 {
            let t = TAU * i as f64 / 8.0;
            assert!(vclose(p.eval(t), c.eval(t), 1e-12), "{:?}", p.eval(t));
        }
        // the circle of radius 8 at height 6 projects to that same latitude circle (radial projection)
        let c2 = Curve::circle(Vec3::new(0., 0., 6.), Vec3::Z, 8.0);
        let p2 = Curve::projected(c2, sphere(), None).unwrap();
        for i in 0..8 {
            let t = TAU * i as f64 / 8.0;
            assert!(vclose(p2.eval(t), c.eval(t), 1e-12), "{:?}", p2.eval(t));
            assert!(vclose(p2.deriv(t), c.deriv(t), 1e-9), "{:?}", p2.deriv(t));
        }
        // iso lines of a cylinder: a ruling and a circle
        let cy = cyl();
        let Surface::Cylinder { frame, r } = &cy else { unreachable!() };
        let ru = Curve::iso(cy.clone(), true, 0.4);
        let ci = Curve::iso(cy.clone(), false, 1.5);
        let circ = Curve::Circle { frame: Frame { o: frame.o + frame.z * 1.5, ..*frame }, r: *r };
        for &t in &[0.0, 1.0, 2.5] {
            assert!(vclose(ru.eval(t), cy.eval(0.4, t), 1e-15));
            assert!(vclose(ci.eval(t), circ.eval(t), 1e-13));
        }
        assert_eq!(ci.period(), Some(TAU));
        assert!(close(ci.length_between(0.0, PI), PI * r, 1e-12));
    }

    #[test]
    fn keys_tell_kinds_apart() {
        let mut keys: Vec<Vec<u64>> = Vec::new();
        for (c, _) in all() {
            let mut k = Vec::new();
            c.push_key(&mut k);
            assert!(!keys.contains(&k), "{}", c.kind());
            keys.push(k);
            assert!(c.is_procedural() && c.nesting() >= 1);
        }
        assert!(Curve::offset(spline(), 1.0, Vec3::ZERO).is_err());
        // nesting is bounded
        let mut c = Curve::line(Vec3::ZERO, Vec3::X);
        let mut refused = false;
        for _ in 0..MAX_NESTING + 2 {
            match Curve::offset(c.clone(), 0.1, Vec3::Z) {
                Ok(o) => c = o,
                Err(_) => {
                    refused = true;
                    break;
                }
            }
        }
        assert!(refused);
    }

    // ---- arc length ----

    /// High-order reference: 4000 panels of 8-point Gauss on a speed function.
    fn ref_len(speed: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
        let n = 4000;
        let mut s = 0.0;
        for i in 0..n {
            let lo = a + (b - a) * i as f64 / n as f64;
            let hi = a + (b - a) * (i + 1) as f64 / n as f64;
            let (m, h) = (0.5 * (lo + hi), 0.5 * (hi - lo));
            for &(x, w) in crate::nurbs::bspline::GL8.iter() {
                s += w * h * speed(m + h * x);
            }
        }
        s
    }
    fn rel(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol * b.abs().max(1e-300)
    }
    /// Line of parameter speed 2 (to the origin) then the quarter arc (speed 1), tangent at the joint.
    fn fast_line_arc() -> Curve {
        let l = Curve::Line { o: Vec3::new(-2., 0., 0.), d: Vec3::new(2., 0., 0.) };
        let a = Curve::circle_x(Vec3::new(0., 1., 0.), Vec3::Z, Vec3::new(0., -1., 0.), 1.0);
        Curve::composite(vec![(l, 0.0, 1.0), (a, 0.0, FRAC_PI_2)], 1e-12).unwrap()
    }

    #[test]
    fn composite_length_with_different_speeds() {
        let c = fast_line_arc();
        let (_, e) = c.range();
        let total = 2.0 + FRAC_PI_2;
        assert!(rel(c.length_between(0.0, e), total, 1e-12));
        assert!(rel(c.length_between(e, 0.0), total, 1e-12));
        // across the joint (global parameter 1): 0.5 of the line (1.0 long) + 0.4 rad of arc
        assert!(rel(c.length_between(0.5, 1.4), 1.0 + 0.4, 1e-12));
        assert!(rel(c.length_between(0.25, 0.75), 1.0, 1e-12));
        assert!(rel(c.reversed().length_between(0.0, e), total, 1e-12));
    }

    #[test]
    fn offset_of_a_composite_splits_at_the_joint() {
        let c = fast_line_arc();
        let (_, e) = c.range();
        // one side runs outside the arc (radius 1 + d), the other inside (1 - d)
        let d = 0.35;
        let l1 = Curve::offset(c.clone(), d, Vec3::Z).unwrap().length_between(0.0, e);
        let l2 = Curve::offset(c.clone(), -d, Vec3::Z).unwrap().length_between(0.0, e);
        let (out, inn) = (2.0 + (1.0 + d) * FRAC_PI_2, 2.0 + (1.0 - d) * FRAC_PI_2);
        let (hi, lo) = if l1 > l2 { (l1, l2) } else { (l2, l1) };
        assert!(rel(hi, out, 1e-9), "{hi} vs {out}");
        assert!(rel(lo, inn, 1e-9), "{lo} vs {inn}");
        // a sub-range straddling the joint: 0.4 of the line parameter (0.8 long) + 0.5 rad of arc
        let o = Curve::offset(c, d, Vec3::Z).unwrap();
        let k = if l1 > l2 { 1.0 + d } else { 1.0 - d };
        assert!(rel(o.length_between(0.6, 1.5), 0.8 + 0.5 * k, 1e-9));
        assert!(rel(o.length_between(1.5, 0.6), 0.8 + 0.5 * k, 1e-9));
    }

    #[test]
    fn offset_of_circle_and_line_are_exact() {
        let c = Curve::circle(Vec3::new(1., 2., 3.), Vec3::new(0.2, -0.3, 1.0), 2.0);
        let n = Vec3::new(0.2, -0.3, 1.0).normalize();
        for (d, nn, r) in [(0.5, n, 2.5), (0.5, -n, 1.5), (-1.25, n, 0.75)] {
            let o = Curve::offset(c.clone(), d, nn).unwrap();
            assert!(rel(o.length_between(0.0, TAU), TAU * r, 1e-11));
            assert!(rel(o.length_between(1.0, 4.0), 3.0 * r, 1e-11));
            assert!(rel(o.length_between(4.0, 1.0), 3.0 * r, 1e-11));
            assert!(rel(o.reversed().length_between(0.5, 2.5), 2.0 * r, 1e-11));
        }
        let l = Curve::Line { o: Vec3::new(1., 0., 0.), d: Vec3::new(3., 4., 0.) };
        let o = Curve::offset(l, 0.7, Vec3::Z).unwrap();
        assert!(rel(o.length_between(-1.0, 2.0), 15.0, 1e-11));
    }

    #[test]
    fn projected_circle_is_an_ellipse_arc() {
        let r = 2.0;
        let (cen, m) = (Vec3::new(0.3, -0.2, 1.0), Vec3::new(0.2, 0.5, 1.0).normalize());
        let circ = Curve::circle(cen, m, r);
        let Curve::Circle { frame, .. } = &circ else { unreachable!() };
        let (e1, e2) = (frame.x, frame.y);
        // along the plane's own normal (z): drop the z component
        let pl = Surface::plane(Vec3::new(0., 0., -2.), Vec3::Z);
        let p = Curve::projected(circ.clone(), pl.clone(), None).unwrap();
        let sp = |t: f64| {
            let d = (e2 * t.cos() - e1 * t.sin()) * r;
            Vec3::new(d.x, d.y, 0.0).len()
        };
        for (a, b) in [(0.0, TAU), (0.4, 2.9), (5.0, 1.0)] {
            assert!(rel(p.length_between(a, b), ref_len(&sp, a.min(b), a.max(b)), 1e-10), "{a} {b}");
        }
        // along an oblique direction onto the same plane: the linear map I - d zT/(d.z)
        let dv = Vec3::new(0.4, -0.3, 1.0);
        let po = Curve::projected(circ, pl, Some(dv)).unwrap();
        let so = |t: f64| {
            let d = (e2 * t.cos() - e1 * t.sin()) * r;
            (d - dv * (d.z / dv.z)).len()
        };
        for (a, b) in [(0.0, TAU), (0.4, 2.9)] {
            assert!(rel(po.length_between(a, b), ref_len(&so, a, b), 1e-10), "{a} {b}");
        }
        assert!(rel(po.reversed().length_between(0.0, TAU), ref_len(&so, 0.0, TAU), 1e-10));
    }

    #[test]
    fn projection_of_a_composite_splits_at_the_joint() {
        // the composite lies in the plane z = 0: projecting onto it keeps the length
        let c = fast_line_arc();
        let (_, e) = c.range();
        let pl = Surface::plane(Vec3::ZERO, Vec3::Z);
        let p = Curve::projected(c.clone(), pl, None).unwrap();
        assert!(rel(p.length_between(0.0, e), 2.0 + FRAC_PI_2, 1e-9));
        assert!(rel(p.length_between(0.5, 1.4), 1.4, 1e-9));
        // onto the plane 0.6 y + 0.8 z = 0 along z (z = -0.75 y): the line (along x) keeps its
        // length, the arc point (x, y) moves at sqrt(dx^2 + dy^2 (1 + 0.75^2))
        let pt = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 0.6, 0.8));
        let q = Curve::projected(c, pt, Some(Vec3::Z)).unwrap();
        let sp = |t: f64| ((-t.sin()).powi(2) + t.cos().powi(2) * (1.0 + 0.5625)).sqrt();
        let want = 2.0 + ref_len(&sp, 0.0, FRAC_PI_2);
        let got = q.length_between(0.0, e);
        assert!(rel(got, want, 1e-9), "{got} vs {want}");
    }

    #[test]
    fn iso_lengths_of_cylinder_sphere_torus() {
        let c = cyl();
        // fixed_u: the generator line (unit speed in v); fixed_v: the circle of radius 3
        assert!(rel(Curve::iso(c.clone(), true, 0.7).length_between(-1.0, 2.5), 3.5, 1e-12));
        assert!(rel(Curve::iso(c.clone(), false, 0.4).length_between(0.0, TAU), TAU * 3.0, 1e-12));
        assert!(rel(Curve::iso(c.clone(), false, 0.4).reversed().length_between(-1.0, 1.0), 6.0, 1e-12));
        let s = sphere();
        // a parallel at latitude v has radius 5 cos v; a meridian has radius 5
        assert!(rel(Curve::iso(s.clone(), false, 0.4).length_between(0.2, 6.0), 5.0 * 0.4_f64.cos() * 5.8, 1e-11));
        assert!(rel(Curve::iso(s, true, 1.3).length_between(-1.0, 0.5), 7.5, 1e-11));
        let t = Surface::Torus { frame: Frame::identity(), major: 4.0, minor: 1.0 };
        assert!(rel(Curve::iso(t.clone(), false, 0.9).length_between(0.0, TAU), TAU * (4.0 + 0.9_f64.cos()), 1e-11));
        assert!(rel(Curve::iso(t.clone(), true, 0.9).length_between(0.0, TAU), TAU, 1e-11));
        assert!(rel(Curve::iso(t, true, 0.9).reversed().length_between(-3.0, -1.0), 2.0, 1e-11));
    }

    #[test]
    fn on_surface_line_in_uv_on_a_cylinder() {
        // unit-speed uv line from (0.2, -0.5) towards (1.8, 1.3) on a radius-3 cylinder
        let uv = Curve2::line_through(Vec2::new(0.2, -0.5), Vec2::new(2.0, 0.8));
        let c = Curve::on_surface(cyl(), uv);
        let n = 1.8_f64.hypot(1.3);
        let sp = ((3.0 * 1.8 / n).powi(2) + (1.3 / n).powi(2)).sqrt(); // line_through is unit speed
        assert!(rel(c.length_between(0.0, 1.0), sp, 1e-11), "{} vs {sp}", c.length_between(0.0, 1.0));
        assert!(rel(c.length_between(0.7, 0.2), 0.5 * sp, 1e-11));
    }

    #[test]
    fn closed_offset_loop_of_a_composite() {
        // a stadium: two lines of speed 3 and two half circles of speed 1, closed
        let (w, r) = (3.0, 1.0);
        let l1 = Curve::Line { o: Vec3::new(0., -r, 0.), d: Vec3::new(w, 0., 0.) };
        let a1 = Curve::circle_x(Vec3::new(w, 0., 0.), Vec3::Z, Vec3::new(0., -1., 0.), r);
        let l2 = Curve::Line { o: Vec3::new(w, r, 0.), d: Vec3::new(-w, 0., 0.) };
        let a2 = Curve::circle_x(Vec3::new(0., 0., 0.), Vec3::Z, Vec3::new(0., 1., 0.), r);
        let c = Curve::composite(vec![(l1, 0.0, 1.0), (a1, 0.0, PI), (l2, 0.0, 1.0), (a2, 0.0, PI)], 1e-12).unwrap();
        let (_, e) = c.range();
        assert!(rel(c.length_between(0.0, e), 2.0 * w + TAU * r, 1e-12));
        let d = 0.25;
        let a = Curve::offset(c.clone(), d, Vec3::Z).unwrap().length_between(0.0, e);
        let b = Curve::offset(c, -d, Vec3::Z).unwrap().length_between(0.0, e);
        let (hi, lo) = (a.max(b), a.min(b));
        assert!(rel(hi, 2.0 * w + TAU * (r + d), 1e-9), "{hi}");
        assert!(rel(lo, 2.0 * w + TAU * (r - d), 1e-9), "{lo}");
    }
}
