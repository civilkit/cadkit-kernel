//! 3D curves: line, circle, ellipse, conic, sinusoid, (rational) B-spline.
//!
//! Evidence:
//! - kernel.strings curve-kind run: `line circle ellipse spline curve conic
//!   sinusoid pcurve` (the carriers carried here; `curve`/`pcurve` are the
//!   generic wrapper and the parameter-space curve).
//! - oracle-flange.json body.edges[] carries `"circle":{"c":[..],"n":[..],"r":25}`
//!   or `"line":true`; sampled edge pts satisfy p = c + r(cos t·x + sin t·y)
//!   with y = n × x (e.g. c=[3,0,0] n=[-1,0,0] starts at [3,0,25] and moves
//!   towards +y) — the STEP CIRCLE / AXIS2_PLACEMENT_3D parameterisation.
//! - STEP writer: LINE, CIRCLE, ELLIPSE, B_SPLINE_CURVE_WITH_KNOTS.
//!
//! Analytic curves are untrimmed carriers (lines unbounded, conics
//! 2π-periodic); edges in ck-topo hold the [t0,t1] trim.
//!
//! `conic` and `sinusoid` (the two tag-run kinds the carriers above don't
//! cover) have no probeable wire record: the oracle emits conic fillet
//! sections only as sampled pts / spline faces (blend corner errors "a
//! conic corner's section", fillet `profile:"conic"` with `rho` default
//! 0.5). Their parameterisations below are therefore GUESSes chosen to slot
//! into the same frame-based style:
//! - conic: focus form r(t) = p/(1 + e·cos t) about `frame.o`, t the polar
//!   angle from `frame.x` — one carrier for circle (e=0) through ellipse,
//!   parabola (e=1) and hyperbola (e>1), the two conic families the
//!   line/circle/ellipse kinds leave out (STEP's CONIC subtypes).
//! - sinusoid: graph of a sine in the frame plane,
//!   p(t) = o + t·x + a·sin(ω t + φ)·y.

use crate::nurbs::bspline::{newton_closest, BSplineCurve, GL8};
use crate::{any_perp, frame_from, Frame, Vec3};
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum Curve {
    /// p(t) = o + t·d, `d` unit from the constructors (t is arc length);
    /// methods tolerate |d| != 1.
    Line { o: Vec3, d: Vec3 },
    /// p(t) = c + r(cos t·x + sin t·y); frame.z = circle normal `n`.
    Circle { frame: Frame, r: f64 },
    /// p(t) = c + a cos t·x + b sin t·y (a = semi_axis_1 along x).
    Ellipse { frame: Frame, a: f64, b: f64 },
    /// General conic in focus form about `frame.o`:
    /// p(t) = ρ(t)·(cos t·x + sin t·y), ρ = p/(1 + e·cos t); t is the polar
    /// angle from `frame.x` (the vertex direction), `p` the semi-latus
    /// rectum, `e` the eccentricity (0 = circle, 1 = parabola, >1 =
    /// hyperbola branch around t = 0). Implicitly |q − o| + e·(q − o)·x = p.
    Conic { frame: Frame, p: f64, e: f64 },
    /// Graph of a sine in the frame plane:
    /// p(t) = o + t·x + a·sin(ω t + φ)·y (unbounded in t).
    Sinusoid { frame: Frame, amp: f64, freq: f64, phase: f64 },
    BSpline(BSplineCurve),
}

/// Pushes the bits of a point (exact content keys, see [`Curve::push_key`]).
pub fn push_vec3(out: &mut Vec<u64>, p: Vec3) {
    out.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
}

/// Pushes the bits of a frame (exact content keys).
pub fn push_frame(out: &mut Vec<u64>, f: &Frame) {
    for p in [f.o, f.x, f.y, f.z] {
        push_vec3(out, p);
    }
}

impl Curve {
    /// Exact content key: the kind and the bits of every field, so two curves push the same words only when
    /// they are bit-for-bit the same curve (for memoising pure functions of curves without hash collisions).
    pub fn push_key(&self, out: &mut Vec<u64>) {
        match self {
            Curve::Line { o, d } => {
                out.push(1);
                push_vec3(out, *o);
                push_vec3(out, *d);
            }
            Curve::Circle { frame, r } => {
                out.push(2);
                push_frame(out, frame);
                out.push(r.to_bits());
            }
            Curve::Ellipse { frame, a, b } => {
                out.push(3);
                push_frame(out, frame);
                out.extend([a.to_bits(), b.to_bits()]);
            }
            Curve::Conic { frame, p, e } => {
                out.push(4);
                push_frame(out, frame);
                out.extend([p.to_bits(), e.to_bits()]);
            }
            Curve::Sinusoid { frame, amp, freq, phase } => {
                out.push(5);
                push_frame(out, frame);
                out.extend([amp.to_bits(), freq.to_bits(), phase.to_bits()]);
            }
            Curve::BSpline(b) => {
                out.extend([6, b.degree as u64, b.knots.len() as u64, b.ctrl.len() as u64]);
                out.extend(b.knots.iter().map(|k| k.to_bits()));
                for &p in &b.ctrl {
                    push_vec3(out, p);
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

    /// Line through `o` with direction `d` (normalised).
    pub fn line(o: Vec3, d: Vec3) -> Curve {
        Curve::Line { o, d: d.normalize() }
    }

    /// Line from `a` to `b`, parameterised so t ∈ [0, |b-a|].
    pub fn line_through(a: Vec3, b: Vec3) -> Curve {
        Curve::line(a, b - a)
    }

    /// Circle from centre, normal, radius (x axis chosen by `any_perp`).
    pub fn circle(c: Vec3, n: Vec3, r: f64) -> Curve {
        let n = n.normalize();
        Curve::Circle { frame: frame_from(c, n, any_perp(n)), r }
    }

    /// Circle with explicit start direction `xref` (t = 0 at c + r·x̂).
    pub fn circle_x(c: Vec3, n: Vec3, xref: Vec3, r: f64) -> Curve {
        Curve::Circle { frame: frame_from(c, n, xref), r }
    }

    /// Circle through three points (None if collinear).
    pub fn circle_3pt(p0: Vec3, p1: Vec3, p2: Vec3) -> Option<Curve> {
        let a = p1 - p0;
        let b = p2 - p0;
        let n = a.cross(b);
        let n2 = n.len2();
        if n2 < 1e-24 * a.len2() * b.len2() {
            return None;
        }
        let c = p0 + (b.cross(n) * a.len2() + n.cross(a) * b.len2()) / (2.0 * n2);
        let r = c.dist(p0);
        Some(Curve::circle_x(c, n, p0 - c, r))
    }

    pub fn ellipse(c: Vec3, n: Vec3, xref: Vec3, a: f64, b: f64) -> Curve {
        Curve::Ellipse { frame: frame_from(c, n, xref), a, b }
    }

    /// General conic (focus form): focus `c`, polar axis `xref`, semi-latus
    /// rectum `p`, eccentricity `e` (0 = circle, 1 = parabola, >1 =
    /// hyperbola).
    pub fn conic(c: Vec3, n: Vec3, xref: Vec3, p: f64, e: f64) -> Curve {
        Curve::Conic { frame: frame_from(c, n, xref), p, e }
    }

    /// Sine graph along `xref` (wavelength 2π/ω, amplitude `amp` in the
    /// plane normal to `n`).
    pub fn sinusoid(c: Vec3, n: Vec3, xref: Vec3, amp: f64, freq: f64, phase: f64) -> Curve {
        Curve::Sinusoid { frame: frame_from(c, n, xref), amp, freq, phase }
    }

    /// Wire kind string (kernel.strings curve-kind table).
    pub fn kind(&self) -> &'static str {
        match self {
            Curve::Line { .. } => "line",
            Curve::Circle { .. } => "circle",
            Curve::Ellipse { .. } => "ellipse",
            Curve::Conic { .. } => "conic",
            Curve::Sinusoid { .. } => "sinusoid",
            Curve::BSpline(_) => "spline",
        }
    }

    /// `(c, n, r)` for the edge JSON `"circle":{"c","n","r"}`.
    pub fn circle_info(&self) -> Option<(Vec3, Vec3, f64)> {
        match self {
            Curve::Circle { frame, r } => Some((frame.o, frame.z, *r)),
            _ => None,
        }
    }

    /// Natural parameter range (lines: unbounded).
    pub fn range(&self) -> (f64, f64) {
        match self {
            Curve::Line { .. } | Curve::Sinusoid { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve::Circle { .. } | Curve::Ellipse { .. } => (0.0, TAU),
            // e ≥ -1: closed (|e| < 1) or the branch/lobe around the vertex
            // direction, bounded by the 1 + e·cos t = 0 asymptotes.
            Curve::Conic { e, .. } if e.abs() < 1.0 => (0.0, TAU),
            Curve::Conic { e, .. } if *e == 1.0 => (-std::f64::consts::PI, std::f64::consts::PI),
            Curve::Conic { e, .. } if *e > 1.0 => {
                let t = (-1.0 / e).acos();
                (-t, t)
            }
            Curve::Conic { e, .. } => {
                // e < -1: the lobe around t = π where 1 + e·cos t > 0
                let t = (-1.0 / e).acos();
                (t, TAU - t)
            }
            Curve::BSpline(b) => b.domain(),
        }
    }

    /// Period for periodic curves.
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Circle { .. } | Curve::Ellipse { .. } => Some(TAU),
            Curve::Conic { e, .. } if e.abs() < 1.0 => Some(TAU),
            _ => None,
        }
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        match self {
            Curve::Line { o, d } => *o + *d * t,
            Curve::Circle { frame, r } => frame.o + (frame.x * t.cos() + frame.y * t.sin()) * *r,
            Curve::Ellipse { frame, a, b } => frame.o + frame.x * (a * t.cos()) + frame.y * (b * t.sin()),
            Curve::Conic { frame, p, e } => {
                let r = p / (1.0 + e * t.cos());
                frame.o + (frame.x * t.cos() + frame.y * t.sin()) * r
            }
            Curve::Sinusoid { frame, amp, freq, phase } => {
                frame.o + frame.x * t + frame.y * (amp * (freq * t + phase).sin())
            }
            Curve::BSpline(c) => c.eval(t),
        }
    }

    /// `[C, C', ..., C^(n)]` at t.
    pub fn derivs(&self, t: f64, n: usize) -> Vec<Vec3> {
        match self {
            Curve::Line { o, d } => {
                let mut v = vec![Vec3::ZERO; n + 1];
                v[0] = *o + *d * t;
                if n >= 1 {
                    v[1] = *d;
                }
                v
            }
            Curve::Circle { frame, r } => conic_derivs(frame, *r, *r, t, n),
            Curve::Ellipse { frame, a, b } => conic_derivs(frame, *a, *b, t, n),
            Curve::Conic { frame, p, e } => polar_derivs(frame, *p, *e, t, n),
            Curve::Sinusoid { frame, amp, freq, phase } => sinusoid_derivs(frame, *amp, *freq, *phase, t, n),
            Curve::BSpline(c) => c.derivs(t, n),
        }
    }

    /// `[C, C', C'']` at t without allocating (same arithmetic as [`Self::derivs`]`(t, 2)`):
    /// the surface evaluators of revolution, screw and extrusion patches call this per point.
    #[inline]
    pub fn derivs2_arr(&self, t: f64) -> [Vec3; 3] {
        match self {
            Curve::Line { o, d } => [*o + *d * t, *d, Vec3::ZERO],
            Curve::BSpline(c) if c.degree < 12 => c.derivs_arr(t, 2),
            Curve::Circle { frame, r } => conic_derivs_arr(frame, *r, *r, t),
            Curve::Ellipse { frame, a, b } => conic_derivs_arr(frame, *a, *b, t),
            _ => {
                let v = self.derivs(t, 2);
                [v[0], v[1], v[2]]
            }
        }
    }

    /// `[C, C']` at t without allocating (same arithmetic as [`Self::derivs`]`(t, 1)`).
    #[inline]
    pub fn derivs1_arr(&self, t: f64) -> [Vec3; 2] {
        match self {
            Curve::Line { o, d } => [*o + *d * t, *d],
            Curve::BSpline(c) if c.degree < 12 => {
                let a = c.derivs_arr(t, 1);
                [a[0], a[1]]
            }
            Curve::Circle { frame, r } => {
                let d = conic_derivs_arr(frame, *r, *r, t);
                [d[0], d[1]]
            }
            Curve::Ellipse { frame, a, b } => {
                let d = conic_derivs_arr(frame, *a, *b, t);
                [d[0], d[1]]
            }
            _ => {
                let v = self.derivs(t, 1);
                [v[0], v[1]]
            }
        }
    }

    pub fn deriv(&self, t: f64) -> Vec3 {
        self.derivs1_arr(t)[1]
    }

    pub fn tangent(&self, t: f64) -> Vec3 {
        match self {
            Curve::BSpline(c) => c.tangent(t),
            _ => self.deriv(t).normalize(),
        }
    }

    /// False only when `q` is certainly farther than `tol` from the curve
    /// (cheap bound; true = unknown). Only B-splines are culled.
    pub fn may_be_within(&self, q: Vec3, tol: f64) -> bool {
        match self {
            Curve::BSpline(b) => b.may_be_within(q, tol),
            _ => true,
        }
    }

    /// Parameter of the closest point to `q` (periodic results in [0, 2π)).
    pub fn closest_param(&self, q: Vec3) -> f64 {
        match self {
            Curve::Line { o, d } => (q - *o).dot(*d) / d.len2(),
            Curve::Circle { frame, .. } => {
                let l = frame.to_local(q);
                if l.x == 0.0 && l.y == 0.0 {
                    0.0
                } else {
                    wrap_angle(l.y.atan2(l.x))
                }
            }
            Curve::Ellipse { frame, a, b } => {
                let l = frame.to_local(q);
                // seed from the scaled angle, refine with Newton (4 seeds
                // to avoid the wrong local minimum near the major axis)
                let seed = (l.y * a).atan2(l.x * b);
                let mut best = (f64::INFINITY, 0.0);
                for k in 0..4 {
                    let s = seed + k as f64 * TAU / 4.0;
                    let t = newton_closest(|t| self.derivs(t, 2), q, s, f64::NEG_INFINITY, f64::INFINITY);
                    let d = self.eval(t).dist(q);
                    if d < best.0 - 1e-15 {
                        best = (d, t);
                    }
                }
                wrap_angle(best.1)
            }
            Curve::Conic { frame, e, .. } => {
                let l = frame.to_local(q);
                if *e == 0.0 {
                    if l.x == 0.0 && l.y == 0.0 {
                        0.0
                    } else {
                        wrap_angle(l.y.atan2(l.x))
                    }
                } else {
                    // polar angle of q as the seed; several offsets to avoid
                    // a wrong local minimum, Newton clamped to the range
                    let seed = l.y.atan2(l.x);
                    let (lo, hi) = self.range();
                    let k = if e.abs() < 1.0 { 4 } else { 2 };
                    let span = if e.abs() < 1.0 { TAU / 4.0 } else { (hi - lo) / 2.0 };
                    let mut best = (f64::INFINITY, 0.0);
                    for i in 0..k {
                        let s = (seed + i as f64 * span).clamp(lo, hi);
                        let t = newton_closest(|t| self.derivs(t, 2), q, s, lo, hi);
                        let d = self.eval(t).dist(q);
                        if d < best.0 - 1e-15 {
                            best = (d, t);
                        }
                    }
                    if self.period().is_some() {
                        wrap_angle(best.1)
                    } else {
                        best.1
                    }
                }
            }
            Curve::Sinusoid { frame, .. } => {
                // the curve is a graph over x: seed at the projection, walk
                // a small neighbourhood of seeds (the wave can bend the
                // closest point away from the projection)
                let l = frame.to_local(q);
                let mut best = (f64::INFINITY, l.x);
                for k in [-1.0, 0.0, 1.0] {
                    let t = newton_closest(|t| self.derivs(t, 2), q, l.x + k, f64::NEG_INFINITY, f64::INFINITY);
                    let d = self.eval(t).dist(q);
                    if d < best.0 - 1e-15 {
                        best = (d, t);
                    }
                }
                best.1
            }
            Curve::BSpline(c) => c.closest_param(q),
        }
    }

    pub fn closest_point(&self, q: Vec3) -> (f64, Vec3) {
        let t = self.closest_param(q);
        (t, self.eval(t))
    }

    /// Arc length over [t0,t1] (analytic for line/circle).
    pub fn length_between(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line { d, .. } => (t1 - t0).abs() * d.len(),
            Curve::Circle { r, .. } => (t1 - t0).abs() * r,
            Curve::Conic { p, e, .. } if *e == 0.0 => (t1 - t0).abs() * p,
            Curve::Ellipse { .. } | Curve::Conic { .. } | Curve::Sinusoid { .. } => {
                let n = 16;
                let mut s = 0.0;
                for i in 0..n {
                    let a = t0 + (t1 - t0) * i as f64 / n as f64;
                    let b = t0 + (t1 - t0) * (i + 1) as f64 / n as f64;
                    let (m, h) = ((a + b) / 2.0, (b - a) / 2.0);
                    for &(x, w) in GL8.iter() {
                        s += w * h.abs() * self.deriv(m + h * x).len();
                    }
                }
                s
            }
            Curve::BSpline(c) => c.length_between(t0, t1),
        }
    }

    /// Sub-curve on [t0,t1]: splines are cut, analytic carriers are returned
    /// unchanged (their trim lives on the edge).
    pub fn segment(&self, t0: f64, t1: f64) -> Curve {
        match self {
            Curve::BSpline(c) => Curve::BSpline(c.segment(t0, t1)),
            other => other.clone(),
        }
    }

    /// Same carrier, opposite direction. Parameter map t -> t':
    /// lines t' = -t, conics t' = 2π - t, splines t' = a + b - t.
    pub fn reversed(&self) -> Curve {
        match self {
            Curve::Line { o, d } => Curve::Line { o: *o, d: -*d },
            Curve::Circle { frame, r } => Curve::Circle { frame: flip_frame(frame), r: *r },
            Curve::Ellipse { frame, a, b } => Curve::Ellipse { frame: flip_frame(frame), a: *a, b: *b },
            Curve::Conic { frame, p, e } => Curve::Conic { frame: flip_frame(frame), p: *p, e: *e },
            // r(t') = c(-t'): flip x and y, keep ω, negate φ
            // (sin(ω t' − φ) = −sin(φ − ω t') with y -> −y)
            Curve::Sinusoid { frame, amp, freq, phase } => Curve::Sinusoid {
                frame: Frame { o: frame.o, x: -frame.x, y: -frame.y, z: frame.z },
                amp: *amp,
                freq: *freq,
                phase: -*phase,
            },
            Curve::BSpline(c) => Curve::BSpline(c.reversed()),
        }
    }

    /// Apply a rigid/affine point map `f` with matching vector map `g`
    /// (g = linear part). Exact for rigid motions; conics assume `g` is a
    /// similarity (radius scaled by |g(x)|).
    pub fn transform(&self, f: impl Fn(Vec3) -> Vec3, g: impl Fn(Vec3) -> Vec3) -> Curve {
        let tf = |fr: &Frame| -> (Frame, f64) {
            let x = g(fr.x);
            let s = x.len();
            let z = g(fr.z).normalize();
            (frame_from(f(fr.o), z, x), s)
        };
        match self {
            Curve::Line { o, d } => Curve::Line { o: f(*o), d: g(*d).normalize() },
            Curve::Circle { frame, r } => {
                let (fr, s) = tf(frame);
                Curve::Circle { frame: fr, r: r * s }
            }
            Curve::Ellipse { frame, a, b } => {
                let (fr, s) = tf(frame);
                Curve::Ellipse { frame: fr, a: a * s, b: b * s }
            }
            Curve::Conic { frame, p, e } => {
                let (fr, s) = tf(frame);
                Curve::Conic { frame: fr, p: p * s, e: *e }
            }
            // lengths scale by s, so the wavelength scales by s: ω' = ω/s
            Curve::Sinusoid { frame, amp, freq, phase } => {
                let (fr, s) = tf(frame);
                Curve::Sinusoid { frame: fr, amp: amp * s, freq: freq / s, phase: *phase }
            }
            Curve::BSpline(c) => Curve::BSpline(c.map(&f)),
        }
    }

    /// `n+1` evenly spaced samples over [t0,t1].
    pub fn sample(&self, t0: f64, t1: f64, n: usize) -> Vec<Vec3> {
        (0..=n).map(|i| self.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect()
    }
}

/// Reverse a conic frame: y -> -y, z -> -z (t -> 2π - t).
fn flip_frame(f: &Frame) -> Frame {
    Frame { o: f.o, x: f.x, y: -f.y, z: -f.z }
}

/// Wrap an angle to [0, 2π).
pub fn wrap_angle(t: f64) -> f64 {
    let w = t.rem_euclid(TAU);
    if w >= TAU {
        0.0
    } else {
        w
    }
}

/// [`conic_derivs`] to second order without allocating (the same arithmetic per entry).
#[inline]
fn conic_derivs_arr(frame: &Frame, a: f64, b: f64, t: f64) -> [Vec3; 3] {
    let (s, c) = t.sin_cos();
    let cyc = [(c, s), (-s, c), (-c, -s), (s, -c)];
    let at = |k: usize| {
        let (cx, sy) = cyc[k];
        frame.x * (a * cx) + frame.y * (b * sy)
    };
    [frame.o + at(0), at(1), at(2)]
}

fn conic_derivs(frame: &Frame, a: f64, b: f64, t: f64, n: usize) -> Vec<Vec3> {
    let (s, c) = t.sin_cos();
    // k-th derivative of (cos t, sin t) cycles with period 4
    let cyc = [(c, s), (-s, c), (-c, -s), (s, -c)];
    (0..=n)
        .map(|k| {
            let (cx, sy) = cyc[k % 4];
            let v = frame.x * (a * cx) + frame.y * (b * sy);
            if k == 0 {
                frame.o + v
            } else {
                v
            }
        })
        .collect()
}

/// Derivatives of the focus-form conic ρ(t) = p/(1 + e·cos t):
/// P = o + ρ·u with u = (cos t·x + sin t·y), u' = (−sin t·x + cos t·y),
/// u'' = −u, u''' = −u', ...; ρ^(k) from the Leibniz recursion on
/// ρ·(1 + e·cos t) = p.
fn polar_derivs(frame: &Frame, p: f64, e: f64, t: f64, n: usize) -> Vec<Vec3> {
    let (s, c) = t.sin_cos();
    let g = 1.0 + e * c;
    // g^(k) = e·(cos t)^(k): cycle [c, −s, −c, s]
    let gcyc = [c, -s, -c, s];
    let mut r = vec![0.0; n + 1];
    r[0] = if g == 0.0 { f64::INFINITY } else { p / g };
    for k in 1..=n {
        let mut acc = 0.0;
        let mut binom = 1.0; // C(k, j)
        for j in 0..k {
            acc += binom * r[j] * e * gcyc[(k - j) % 4];
            binom *= (k - j) as f64 / (j + 1) as f64;
        }
        r[k] = -acc / g;
    }
    let (ux, uy) = (frame.x * c + frame.y * s, frame.x * (-s) + frame.y * c);
    let uder = |k: usize| {
        let sgn = if (k / 2) % 2 == 0 { 1.0 } else { -1.0 };
        if k % 2 == 0 {
            ux * sgn
        } else {
            uy * sgn
        }
    };
    (0..=n)
        .map(|k| {
            // P^(k) = Σ_j C(k,j) ρ^(j) u^(k−j)
            let mut acc = Vec3::ZERO;
            let mut binom = 1.0;
            for j in 0..=k {
                if j > 0 {
                    binom *= (k - j + 1) as f64 / j as f64;
                }
                acc = acc + uder(k - j) * (r[j] * binom);
            }
            if k == 0 {
                frame.o + acc
            } else {
                acc
            }
        })
        .collect()
}

/// Derivatives of the sine graph p = o + t·x + a·sin(ω t + φ)·y: the x term
/// is linear (zero above first order), the sine cycles with period 4 scaled
/// by ω^k.
fn sinusoid_derivs(frame: &Frame, amp: f64, freq: f64, phase: f64, t: f64, n: usize) -> Vec<Vec3> {
    let th = freq * t + phase;
    let (s, c) = th.sin_cos();
    (0..=n)
        .map(|k| match k {
            0 => frame.o + frame.x * t + frame.y * (amp * s),
            // the x term is linear: constant x̂ first, zero above
            1 => frame.x + frame.y * (amp * freq * c),
            _ => {
                let v = match k % 4 {
                    2 => -s,
                    3 => -c,
                    _ => s,
                };
                frame.y * (amp * freq.powi(k as i32) * v)
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;

    /// The allocation-free conic derivatives are the `derivs` entries bit for bit.
    #[test]
    fn conic_derivs_arr_matches_derivs() {
        let cs = [
            Curve::circle_x(Vec3::new(3., 1., -2.), Vec3::new(-1., 0.3, 0.2), Vec3::new(0., 0., 1.), 25.0),
            Curve::Ellipse { frame: frame_from(Vec3::new(1., 2., 3.), Vec3::new(0.2, 0.5, 1.0).normalize(), Vec3::new(1., 0., 0.)), a: 4.0, b: 2.5 },
        ];
        for c in &cs {
            for k in 0..40 {
                let t = -3.0 + 0.37 * k as f64;
                let (v, a1, a2) = (c.derivs(t, 2), c.derivs1_arr(t), c.derivs2_arr(t));
                for i in 0..3 {
                    assert_eq!(v[i], a2[i]);
                }
                assert_eq!([v[0], v[1]], a1);
            }
        }
    }

    #[test]
    fn oracle_flange_circle_samples() {
        // oracle-flange.json edge "F4.C0@F1.1|F4.S.e1@F1.1#0":
        // circle {c:[3,0,0], n:[-1,0,0], r:25}, pts[0]=[3,0,25],
        // pts[1]=[3,3.726056654404361,24.720770655628215] (42 segments).
        let c = Curve::circle_x(Vec3::new(3., 0., 0.), Vec3::new(-1., 0., 0.), Vec3::new(0., 0., 1.), 25.0);
        assert!(vclose(c.eval(0.0), Vec3::new(3., 0., 25.), 1e-15));
        let p1 = c.eval(TAU / 42.0);
        assert!(vclose(p1, Vec3::new(3., 3.726056654404361, 24.720770655628215), 1e-14), "{p1:?}");
        let (cc, n, r) = c.circle_info().unwrap();
        assert_eq!((cc, n, r), (Vec3::new(3., 0., 0.), Vec3::new(-1., 0., 0.), 25.0));
        assert_eq!(c.kind(), "circle");
    }

    #[test]
    fn derivs_fd_all() {
        let curves = [
            Curve::line(Vec3::new(1., 2., 3.), Vec3::new(1., 1., 0.)),
            Curve::circle(Vec3::new(1., 0., 0.), Vec3::new(1., 2., 3.), 2.0),
            Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::new(1., 1., 0.), 3.0, 1.0),
            Curve::conic(Vec3::new(0.5, -1., 2.), Vec3::Z, Vec3::X, 1.0, 0.8),
            Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0, 1.0),
            Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, 1.4),
            Curve::sinusoid(Vec3::new(1., 1., 1.), Vec3::new(0.3, 0.4, 1.), Vec3::new(1., 1., 0.), 0.7, 1.3, 0.4),
        ];
        let h = 1e-5;
        for c in &curves {
            // keep the conics inside their branch (away from the asymptotes)
            let ts: &[f64] = if c.period().is_some() { &[0.3, 1.7, 4.0] } else { &[0.2, 0.7, 1.1] };
            for &t in ts {
                let d = c.derivs(t, 3);
                let fd1 = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
                let fd2 = (c.eval(t + h) - d[0] * 2.0 + c.eval(t - h)) / (h * h);
                let fd3 = (c.derivs(t + h, 2)[2] - c.derivs(t - h, 2)[2]) / (2.0 * h);
                assert!(vclose(d[1], fd1, 1e-8), "{}", c.kind());
                assert!(vclose(d[2], fd2, 1e-4), "{}", c.kind());
                assert!(vclose(d[3], fd3, 1e-7), "{}", c.kind());
            }
        }
    }

    #[test]
    fn conic_is_the_focus_form() {
        // implicit definition: |q − o| + e·(q − o)·x̂ = p
        let x = Vec3::X;
        for e in [0.0, 0.5, 1.0, 2.0] {
            let c = Curve::conic(Vec3::ZERO, Vec3::Z, x, 1.0, e);
            let (lo, hi) = c.range();
            for i in 1..24 {
                let t = lo + (hi - lo) * i as f64 / 25.0;
                let p = c.eval(t);
                let l = p.len();
                assert!(close(l + e * p.dot(x), 1.0, 1e-12), "e={e} t={t}: {l} + {e}·{}", p.dot(x));
            }
            assert_eq!(c.kind(), "conic");
        }
        // e = 0 is exactly the circle of radius p
        let c = Curve::conic(Vec3::new(1., 2., 3.), Vec3::new(1., 1., 0.), Vec3::Z, 2.5, 0.0);
        let k = Curve::circle_x(Vec3::new(1., 2., 3.), Vec3::new(1., 1., 0.), Vec3::Z, 2.5);
        for i in 0..=8 {
            let t = TAU * i as f64 / 8.0;
            assert!(vclose(c.eval(t), k.eval(t), 1e-15));
            assert!(vclose(c.derivs(t, 2)[2], k.derivs(t, 2)[2], 1e-15));
        }
        assert!(close(c.length_between(0.0, TAU), TAU * 2.5, 1e-14));
        // 0 < e < 1 is an ellipse: centre at −a·e·x̂, a = p/(1−e²),
        // b = p/√(1−e²); sampled points satisfy ξ²/a² + η²/b² = 1
        let e = 0.6f64;
        let (a, b) = (1.0 / (1.0 - e * e), 1.0 / (1.0 - e * e).sqrt());
        let c = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, e);
        for i in 0..=48 {
            let t = TAU * i as f64 / 48.0;
            let p = c.eval(t);
            let (xi, eta) = (p.x + a * e, p.y);
            assert!(close(xi * xi / (a * a) + eta * eta / (b * b), 1.0, 1e-12));
        }
        // circumference matches the ellipse with those axes (Ramanujan II)
        let h = ((a - b) / (a + b)).powi(2);
        let ram = std::f64::consts::PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
        assert!(close(c.length_between(0.0, TAU), ram, 1e-7), "{}", c.length_between(0.0, TAU));
        // hyperbola: the branch is bounded by the 1 + e·cos t = 0 asymptotes
        let h = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, 2.0);
        let t0 = (-0.5f64).acos();
        assert!(close(h.range().0, -t0, 1e-15) && close(h.range().1, t0, 1e-15));
        assert!(h.period().is_none());
        assert!(close(h.eval(0.0).x, 1.0 / 3.0, 1e-15)); // vertex at p/(1+e)
    }

    #[test]
    fn conic_closest_and_reverse() {
        // parabola: recover parameters from points pushed off the normal
        let c = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0, 1.0);
        for &t in &[-1.2, -0.4, 0.3, 1.1] {
            let d = c.derivs(t, 1);
            let nrm = Vec3::Z.cross(d[1]).normalize();
            let q = d[0] - nrm * 0.2;
            assert!(close(c.closest_param(q), t, 1e-9), "t={t} got {}", c.closest_param(q));
        }
        // hyperbola branch
        let h = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, 2.0);
        for &t in &[-0.8, 0.0, 0.9] {
            let d = h.derivs(t, 1);
            let nrm = Vec3::Z.cross(d[1]).normalize();
            let q = d[0] - nrm * 0.15;
            assert!(close(h.closest_param(q), t, 1e-9), "t={t} got {}", h.closest_param(q));
        }
        // reversed: r(t') = c(−t') (≡ 2π − t' for the closed cases)
        let e = Curve::conic(Vec3::ZERO, Vec3::Z, Vec3::X, 1.0, 0.6);
        assert!(vclose(e.reversed().eval(TAU - 0.7), e.eval(0.7), 1e-13));
        assert!(vclose(h.reversed().eval(-0.9), h.eval(0.9), 1e-13));
    }

    #[test]
    fn sinusoid_basics() {
        let c = Curve::sinusoid(Vec3::new(1., 1., 1.), Vec3::new(0.3, 0.4, 1.), Vec3::new(1., 1., 0.), 0.7, 1.3, 0.4);
        assert_eq!(c.kind(), "sinusoid");
        assert_eq!(c.range(), (f64::NEG_INFINITY, f64::INFINITY));
        assert!(c.period().is_none());
        // the graph stays in its plane through `o` with normal `n`
        let n = Vec3::new(0.3, 0.4, 1.).normalize();
        for &t in &[0.0, 1.0, -3.0, 7.5] {
            let p = c.eval(t);
            assert!(close((p - Vec3::new(1., 1., 1.)).dot(n), 0.0, 1e-13));
        }
        // straight case: amplitude 0 is the x axis, length is the span
        let s0 = Curve::sinusoid(Vec3::ZERO, Vec3::Z, Vec3::X, 0.0, 2.0, 0.0);
        assert!(close(s0.length_between(-2.0, 3.0), 5.0, 1e-14));
        // closest point: query off the normal recovers the parameter
        for &t in &[0.0, 0.8, 2.0, -1.5, 4.0] {
            let d = c.derivs(t, 1);
            let nrm = Vec3::new(0.3, 0.4, 1.).normalize().cross(d[1]).normalize();
            let q = d[0] - nrm * 0.2;
            assert!(close(c.closest_param(q), t, 1e-9), "t={t} got {}", c.closest_param(q));
        }
        // reversed: r(t') = c(−t')
        for &t in &[0.3, -1.0, 2.2] {
            assert!(vclose(c.reversed().eval(t), c.eval(-t), 1e-13), "t={t}");
        }
        // transform: similarity scale 2 -> c'(τ) = 2·c(τ/2)
        let t2 = c.transform(|p| p * 2.0, |v| v * 2.0);
        if let Curve::Sinusoid { freq, .. } = t2 {
            assert!(close(freq, 1.3 / 2.0, 1e-15));
        } else {
            panic!("not a sinusoid");
        }
        for &tau in &[0.0, 1.0, -2.5] {
            assert!(vclose(t2.eval(tau), c.eval(tau / 2.0) * 2.0, 1e-12), "tau={tau}");
        }
    }

    #[test]
    fn closest_points() {
        let c = Curve::circle(Vec3::ZERO, Vec3::Z, 2.0);
        let q = c.eval(2.5) * 1.7 + Vec3::Z;
        assert!(close(c.closest_param(q), 2.5, 1e-12));
        let e = Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0);
        for &t in &[0.2, 1.0, 2.0, 3.5, 5.9] {
            let p = e.derivs(t, 1);
            let nrm = Vec3::Z.cross(p[1]).normalize();
            let q = p[0] - nrm * 0.2; // slightly outside
            assert!(close(e.closest_param(q), t, 1e-9), "t={t} got {}", e.closest_param(q));
        }
        let l = Curve::line_through(Vec3::ZERO, Vec3::new(2., 0., 0.));
        assert!(close(l.closest_param(Vec3::new(1.5, 3., 1.)), 1.5, 1e-15));
    }

    #[test]
    fn three_point_circle_and_length() {
        let c = Curve::circle_3pt(Vec3::new(1., 0., 0.), Vec3::new(0., 1., 0.), Vec3::new(-1., 0., 0.)).unwrap();
        let (cc, n, r) = c.circle_info().unwrap();
        assert!(vclose(cc, Vec3::ZERO, 1e-15) && close(r, 1.0, 1e-15) && vclose(n, Vec3::Z, 1e-15));
        assert!(close(c.length_between(0.0, TAU), TAU, 1e-14));
        // ellipse circumference (Ramanujan II is accurate to ~1e-10 here)
        let e = Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0);
        let (a, b) = (3.0f64, 1.0f64);
        let h = ((a - b) / (a + b)).powi(2);
        let ram = std::f64::consts::PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
        assert!(close(e.length_between(0.0, TAU), ram, 1e-7));
    }

    #[test]
    fn reverse_maps() {
        let c = Curve::circle(Vec3::new(1., 2., 3.), Vec3::new(0., 1., 1.), 2.0);
        let r = c.reversed();
        assert!(vclose(r.eval(TAU - 0.7), c.eval(0.7), 1e-14));
        let l = Curve::line(Vec3::ZERO, Vec3::X);
        assert!(vclose(l.reversed().eval(-2.0), l.eval(2.0), 1e-15));
    }
}
