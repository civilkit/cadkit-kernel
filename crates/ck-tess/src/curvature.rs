//! Curvature-driven sizing: chord-height / angle tolerances, curve parameter
//! sampling, per-face edge refinement and the periodic-face cut.
//!
//! Everything here is pinned against the original kernel by instrumenting
//! kernel.wasm (ck_log import patched into the WAT, see the ck-tess README):
//!
//! - Tolerances (regen, kernel.wat f207): `chord = max(1e-3, 4e-4 · |bbox
//!   diagonal|)` (`f64.const 0.0004`, `0.001`, NaN → 0.001), `angle = 0.3`
//!   rad (logged as the `param_5` of the normal-angle refinement f3283).
//! - Curve sampling (f3764 → f3765):
//!   - line: `[t0, t1]`
//!   - circle: `step = r <= tol ? π/2 : 2·acos(1 − tol/r)`, `step = min(step,
//!     angle)`, `n = clamp(ceil(|t1−t0| / step), 1, 100000)`
//!   - ellipse: from the two semi-axes, `step = min(angle·b/a, sqrt(8·tol /
//!     (a/b²)) / a)` (a = major, b = minor), same ceil/clamp.
//!   - spline: every knot span cut into `max(2, degree+1)` pieces, each refined
//!     adaptively (f3767; GUESS: chord-height + angle bisection).
//!   - uniform params `t_i = t0 + ((t1 − t0)·i) / n`, the last one exactly `t1`.
//!   Pinned: cylinders r = 1..100 give 35/47/60/65/66/67 uniform segments,
//!   a tiny circle in a big body 21 segments (angle cap 0.3 rad).
//! - Per coedge, in edge-coedge order, the param list is refined in (u,v)
//!   (ck-topo region.rs f3574/f3575): bisect while the pcurve midpoint is more
//!   than `uvtol = max(tol / max(scale, 1e-12), 1e-9)` off the uv chord, depth
//!   ≤ 12. GUESS: `scale` = max(|Su|, |Sv|) at the coedge start.
//! - Then per coedge (ck-tess mesh.rs f3244 → f3283): bisect while the angle
//!   between the face normals at the two ends exceeds `angle`, depth < 8.
//! - Periodic faces are cut open at the middle of the LARGEST gap between the
//!   sorted boundary-sample u values (wrapped into [0, P); the wrap-around gap
//!   is the first candidate and only a strictly larger gap replaces it)
//!   (ck-topo region.rs f3444). The cut adds one param to every edge that
//!   crosses it — that is the single "extra" bisected segment every full
//!   circle shows in the oracle (cyl r=1: 35 uniform + 1 at t = 4.0391905…,
//!   the midpoint of uniform segment 22, chosen by rounding noise in the gaps).
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use std::f64::consts::FRAC_PI_2;

/// Relative chord tolerance: `chord = CHORD_FACTOR · bbox diagonal`.
pub const CHORD_FACTOR: f64 = 4e-4;
/// Lower clamp for the chord tolerance (also used when the box is empty).
pub const MIN_CHORD: f64 = 1e-3;
/// Maximum angle between consecutive samples / normals (radians).
pub const ANGLE: f64 = 0.3;
/// Maximum recursion depth of the uv chord refinement (f3575: `depth > 11` stops).
pub const UV_DEPTH: u32 = 12;
/// Maximum recursion depth of the normal-angle refinement (f3283: `depth >= 8` stops).
pub const NORMAL_DEPTH: u32 = 8;
/// Upper clamp on segments per curve (f3764: 100000).
pub const MAX_SEGMENTS: f64 = 100000.0;

/// Tessellation tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tol {
    /// Chord height (model units).
    pub chord: f64,
    /// Angle (radians) between consecutive samples / adjacent normals.
    pub angle: f64,
}

impl Tol {
    pub fn new(chord: f64, angle: f64) -> Tol {
        Tol { chord, angle }
    }
    /// Regen tolerance for a body with bounds `lo..hi` (f207).
    pub fn for_bbox(lo: Vec3, hi: Vec3) -> Tol {
        let ok = lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z;
        let mut chord = MIN_CHORD;
        if ok {
            let d = (hi - lo).len() * CHORD_FACTOR;
            // NaN -> 0.001, then max(.., 0.001)
            let d = if d.is_nan() { MIN_CHORD } else { d };
            chord = if d > MIN_CHORD { d } else { MIN_CHORD };
        }
        Tol { chord, angle: ANGLE }
    }
}

/// Wasm `f64.min` / `f64.max` style helpers with the NaN handling the
/// decompile shows (`x.is_nan() ? other : x`).
fn nmin(a: f64, b: f64) -> f64 {
    let a2 = if a.is_nan() { b } else { a };
    let b2 = if b.is_nan() { a2 } else { b };
    if b2 <= a2 {
        b2
    } else {
        a2
    }
}

/// Segment count from a float ratio (f3764 tail: ceil, clamp [1, 100000]).
fn seg_count(x: f64) -> usize {
    let c = x.ceil();
    if !c.is_finite() {
        return 1;
    }
    c.clamp(1.0, MAX_SEGMENTS) as usize
}

/// Uniform params (f3765): `t0 + ((t1 - t0) * i) / n`, last = `t1`.
pub fn uniform_params(t0: f64, t1: f64, n: usize) -> Vec<f64> {
    let n = n.max(1);
    let span = t1 - t0;
    let nf = n as f64;
    let mut out: Vec<f64> = (0..n).map(|i| t0 + (span * i as f64) / nf).collect();
    out.push(t1);
    out
}

/// Angular step of a circle of radius `r` (f3764 kind 1).
pub fn circle_step(r: f64, tol: &Tol) -> f64 {
    let chord = if tol.chord.is_nan() || tol.chord <= 1e-9 { 1e-9 } else { tol.chord };
    let s = if r <= chord { FRAC_PI_2 } else { 2.0 * (1.0 - chord / r).acos() };
    nmin(s, tol.angle)
}

/// Number of uniform segments for an arc of radius `r` sweeping `sweep` radians.
pub fn arc_segments(r: f64, sweep: f64, tol: &Tol) -> usize {
    seg_count(sweep.abs() / circle_step(r, tol))
}

/// Initial parameter samples of `curve` on `[t0, t1]` (f3764).
pub fn curve_params(curve: &Curve, t0: f64, t1: f64, tol: &Tol) -> Vec<f64> {
    let chord = if tol.chord.is_nan() || tol.chord <= 1e-9 { 1e-9 } else { tol.chord };
    match curve {
        Curve::Line { .. } => vec![t0, t1],
        Curve::Circle { r, .. } => uniform_params(t0, t1, arc_segments(*r, t1 - t0, tol)),
        Curve::Ellipse { a, b, .. } => {
            let (maj, min) = if a >= b { (*a, *b) } else { (*b, *a) };
            let min = if min.sqrt() <= 1e-12 && min <= 1e-24 { 1e-12 } else { min.max(1e-12) };
            let s_ang = tol.angle * min / maj;
            let s_chd = ((chord * 8.0) / (maj / (min * min))).sqrt() / maj;
            let step = nmin(s_chd, s_ang);
            uniform_params(t0, t1, seg_count((t1 - t0).abs() / step))
        }
        Curve::BSpline(_) => spline_params(curve, t0, t1, tol),
    }
}

/// B-spline sampling: knot spans, `max(2, degree+1)` pieces each, adaptive bisection.
/// GUESS (f3766/f3767 not decompiled): bisection on chord height and tangent angle, depth ≤ 12.
fn spline_params(curve: &Curve, t0: f64, t1: f64, tol: &Tol) -> Vec<f64> {
    let (knots, deg) = match curve {
        Curve::BSpline(b) => (b.knots.clone(), b.degree),
        _ => unreachable!(),
    };
    let mut breaks: Vec<f64> = vec![t0];
    for &k in &knots {
        if k > t0 && k < t1 && k > *breaks.last().unwrap() {
            breaks.push(k);
        }
    }
    breaks.push(t1);
    let pieces = (deg + 1).max(2);
    let mut out = vec![t0];
    for w in breaks.windows(2) {
        let (a, b) = (w[0], w[1]);
        for i in 0..pieces {
            let s0 = a + ((b - a) * i as f64) / pieces as f64;
            let s1 = a + ((b - a) * (i + 1) as f64) / pieces as f64;
            spline_refine(curve, s0, s1, tol, 0, &mut out);
        }
    }
    *out.last_mut().unwrap() = t1;
    out
}

fn spline_refine(c: &Curve, a: f64, b: f64, tol: &Tol, depth: u32, out: &mut Vec<f64>) {
    let (pa, pb) = (c.eval(a), c.eval(b));
    let m = 0.5 * (a + b);
    let pm = c.eval(m);
    let sag = point_line_dist(pm, pa, pb);
    let ang = c.deriv(a).angle(c.deriv(b));
    if depth < UV_DEPTH && (sag > tol.chord || ang > tol.angle) {
        spline_refine(c, a, m, tol, depth + 1, out);
        spline_refine(c, m, b, tol, depth + 1, out);
    } else {
        out.push(b);
    }
}

fn point_line_dist(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let d = b - a;
    let l = d.len();
    if l < 1e-300 {
        return p.dist(a);
    }
    d.cross(p - a).len() / l
}

/// uv chord deviation (f3575): |cross(d_end, d_mid)| / |d_end|, or |d_mid| for a degenerate chord.
fn uv_dev(p0: Vec2, p1: Vec2, pm: Vec2) -> f64 {
    let (ex, ey) = (p1.x - p0.x, p1.y - p0.y);
    let (mx, my) = (pm.x - p0.x, pm.y - p0.y);
    let l = (ex * ex + ey * ey).sqrt();
    if l < 1e-300 {
        (mx * mx + my * my).sqrt()
    } else {
        (ey * mx - ex * my).abs() / l
    }
}

/// One f3575 call: refine `[t0, t1]` in uv, pushing interior bisection points and finally `t1`.
fn refine_uv_seg(pc: &Curve2, mut t0: f64, t1: f64, uvtol: f64, mut depth: u32, out: &mut Vec<f64>) {
    loop {
        let p0 = pc.eval(t0);
        let p1 = pc.eval(t1);
        let m = (t1 + t0) * 0.5;
        let pm = pc.eval(m);
        let dev = uv_dev(p0, p1, pm);
        if depth > UV_DEPTH - 1 || !(dev > uvtol) {
            out.push(t1);
            return;
        }
        depth += 1;
        refine_uv_seg(pc, t0, m, uvtol, depth, out);
        t0 = m;
    }
}

/// Refines a sorted param list against a pcurve in uv (f3574 per coedge).
pub fn refine_uv(pc: &Curve2, params: &[f64], uvtol: f64) -> Vec<f64> {
    if params.len() < 2 {
        return params.to_vec();
    }
    let mut out = vec![params[0]];
    for w in params.windows(2) {
        refine_uv_seg(pc, w[0], w[1], uvtol, 0, &mut out);
    }
    out
}

/// uv tolerance for a coedge on `surf` (f3574: `max(tol / max(scale, 1e-12), 1e-9)`).
/// GUESS: scale = max(|Su|, |Sv|) at `uv`.
pub fn uv_tol(surf: &Surface, uv: Vec2, tol: &Tol) -> f64 {
    let scale = match surf {
        Surface::Plane { .. } => 1.0,
        _ => {
            let (su, sv) = surf.partials(uv.x, uv.y);
            su.len().max(sv.len())
        }
    };
    let s = if scale.is_nan() || scale <= 1e-12 { 1e-12 } else { scale };
    let t = tol.chord / s;
    if t.is_nan() || t <= 1e-9 {
        1e-9
    } else {
        t
    }
}

/// Unit surface normal at (u,v) with a fallback for degenerate points (poles):
/// nudges towards the domain interior.
pub fn surface_normal(surf: &Surface, u: f64, v: f64) -> Vec3 {
    let n = surf.normal(u, v);
    if n.is_finite() && n.len2() > 0.5 {
        return n;
    }
    for &(du, dv) in &[(0.0, -1e-7), (0.0, 1e-7), (1e-7, 0.0), (-1e-7, 0.0), (1e-7, -1e-7), (1e-7, 1e-7)] {
        let n = surf.normal(u + du, v + dv);
        if n.is_finite() && n.len2() > 0.5 {
            return n;
        }
    }
    Vec3::new(0.0, 0.0, 1.0)
}

/// Angle between two unit normals (f3906: atan2(|a×b|, a·b)).
pub fn normal_angle(a: Vec3, b: Vec3) -> f64 {
    a.cross(b).len().atan2(a.dot(b))
}

fn refine_normal_seg(
    uv_at: &dyn Fn(f64) -> Vec2,
    surf: &Surface,
    t0: f64,
    t1: f64,
    n0: Vec3,
    n1: Vec3,
    angle: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    if depth < NORMAL_DEPTH && normal_angle(n0, n1) > angle {
        let m = (t0 + t1) * 0.5;
        let uv = uv_at(m);
        let nm = surface_normal(surf, uv.x, uv.y);
        refine_normal_seg(uv_at, surf, t0, m, n0, nm, angle, depth + 1, out);
        refine_normal_seg(uv_at, surf, m, t1, nm, n1, angle, depth + 1, out);
    } else {
        out.push(t1);
    }
}

/// Normal-angle refinement of a param list on one face (f3244 → f3283).
pub fn refine_normals(uv_at: &dyn Fn(f64) -> Vec2, surf: &Surface, params: &[f64], angle: f64) -> Vec<f64> {
    if params.len() < 2 {
        return params.to_vec();
    }
    let mut out = vec![params[0]];
    let mut prev_t = params[0];
    let uv = uv_at(prev_t);
    let mut prev_n = surface_normal(surf, uv.x, uv.y);
    for &t in &params[1..] {
        let uv = uv_at(t);
        let n = surface_normal(surf, uv.x, uv.y);
        refine_normal_seg(uv_at, surf, prev_t, t, prev_n, n, angle, 0, &mut out);
        prev_t = t;
        prev_n = n;
    }
    out
}

/// `x mod p` into `[0, p)` the way f3444 does it (`fmod`, then `+p` when negative).
pub fn wrap(x: f64, p: f64) -> f64 {
    let r = x % p;
    if r < 0.0 {
        r + p
    } else {
        r
    }
}

/// Periodic cut (f3444): middle of the largest gap between the wrapped,
/// sorted `vals`. The wrap-around gap `(p + first) - last` is the first
/// candidate; only a strictly larger interior gap replaces it. Returns a
/// value in `[0, p)` (None when `vals` is empty).
pub fn periodic_cut(vals: &[f64], p: f64) -> Option<f64> {
    let mut s: Vec<f64> = vals.iter().map(|&x| wrap(x, p)).filter(|x| x.is_finite()).collect();
    if s.is_empty() {
        return None;
    }
    s.sort_by(|a, b| a.total_cmp(b));
    let first = s[0];
    let last = *s.last().unwrap();
    let mut gap = (p + first) - last;
    let mut start = last;
    for w in s.windows(2) {
        let d = w[1] - w[0];
        if d > gap {
            gap = d;
            start = w[0];
        }
    }
    Some(wrap(start + gap * 0.5, p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    fn cyl_tol(r: f64, h: f64) -> Tol {
        // extruded cylinder bbox: 2r x h x 2r
        Tol::for_bbox(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0 * r, h, 2.0 * r))
    }

    /// Uniform segment counts of full circles, pinned by oracle probes
    /// (sketch circle on the top plane, extruded; edge `pts` minus the cut point).
    #[test]
    fn circle_counts_match_oracle() {
        for &(r, h, n) in &[
            (1.0, 10.0, 35),
            (2.0, 10.0, 47),
            (5.0, 10.0, 60),
            (10.0, 10.0, 65),
            (20.0, 10.0, 66),
            (50.0, 10.0, 66),
            (100.0, 10.0, 67),
            (10.0, 1.0, 67),
            (10.0, 100.0, 35),
        ] {
            assert_eq!(arc_segments(r, TAU, &cyl_tol(r, h)), n, "r={r} h={h}");
        }
        // tiny circle in a 190x190x1 box: angle cap -> 21
        let t = Tol::for_bbox(Vec3::new(-1.0, -1.0, 0.0), Vec3::new(200.0, 200.0, 1.0));
        assert_eq!(arc_segments(1.0, TAU, &t), 21);
        assert_eq!(arc_segments(0.05, TAU, &t), 21);
    }

    /// Partial arcs (oracle: dext r=10 180° -> 37, rev90 r=10/20 -> 17/24, cone 120° r=5/15 -> 16/27).
    #[test]
    fn arc_counts_match_oracle() {
        // dext: D-shape r=10 extruded 5 -> bbox 20 x 10 x 5
        let t = Tol::for_bbox(Vec3::new(-10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 5.0));
        assert_eq!(arc_segments(10.0, std::f64::consts::PI, &t), 37);
        // rev90: rect x 10..20, y 0..5 revolved 90° about y -> bbox x0..20 y0..5 z-20..0
        let t = Tol::for_bbox(Vec3::new(0.0, 0.0, -20.0), Vec3::new(20.0, 5.0, 0.0));
        assert_eq!(arc_segments(10.0, FRAC_PI_2, &t), 17);
        assert_eq!(arc_segments(20.0, FRAC_PI_2, &t), 24);
    }

    #[test]
    fn uniform_matches_oracle_bits() {
        // oracle-logged param list of the r=1 cylinder (35 segments)
        let p = uniform_params(0.0, TAU, 35);
        assert_eq!(p[1], 0.17951958020513104);
        assert_eq!(p[22], 3.9494307645128823);
        assert_eq!(p[23], 4.128950344718014);
        assert_eq!(p[35], 6.283185307179586);
    }

    /// The cut of the r=1 cylinder lands at the oracle's extra param.
    #[test]
    fn periodic_cut_matches_oracle() {
        let p = uniform_params(0.0, TAU, 35);
        assert_eq!(periodic_cut(&p, TAU), Some(4.039190554615448));
        // ties -> wrap gap wins -> last segment (oracle cyl r=10: 6.234853112508974)
        let p = uniform_params(0.0, TAU, 65);
        assert_eq!(periodic_cut(&p, TAU), Some(6.234853112508974));
        // r=5 (60 segs, 7 ties, first strict max wins) -> 4.34586983746588
        let p = uniform_params(0.0, TAU, 60);
        assert_eq!(periodic_cut(&p, TAU), Some(4.34586983746588));
    }

    #[test]
    fn tol_rule() {
        let t = Tol::for_bbox(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(t.chord, 1e-3);
        let t = Tol::for_bbox(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
        assert!((t.chord - 4e-3).abs() < 1e-15);
        let t = Tol::for_bbox(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(t.chord, 1e-3);
    }
}
