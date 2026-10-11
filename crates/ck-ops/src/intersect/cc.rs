//! Curve-curve intersection (ck-ops/src/intersect/cc.rs).
//!
//! Closed forms for line/line, line/circle, circle/circle (coplanar and
//! skew) using ck-core::quad; everything else (ellipses, splines, mixed)
//! by polyline seeding + 2-variable Newton on |C1(s) − C2(t)|².
//! Coincident pieces are reported as overlaps (parameter intervals on both
//! curves), not as point lists.

use super::{fit_param, fit_params_all, param_slack, TOL};
use ck_core::quad;
use ck_geom::curve::Curve;
use ck_geom::Vec3;

/// One transversal (or touching) intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CcHit {
    pub t1: f64,
    pub t2: f64,
    pub p: Vec3,
}

/// Coincident piece: `a` on curve 1, `b` on curve 2 (b.0 is the curve-2
/// parameter at a.0; b may run backwards).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CcOverlap {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct CcResult {
    pub hits: Vec<CcHit>,
    pub overlaps: Vec<CcOverlap>,
}

/// Intersect `c1` over `r1` with `c2` over `r2`.
pub fn intersect(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) -> CcResult {
    let raw = match (c1, c2) {
        (Curve::Line { .. }, Curve::Line { .. }) => line_line(c1, r1, c2, r2, tol),
        (Curve::Line { .. }, Curve::Circle { .. }) => line_circle(c1, r1, c2, r2, tol),
        (Curve::Circle { .. }, Curve::Line { .. }) => swap(line_circle(c2, r2, c1, r1, tol)),
        (Curve::Circle { .. }, Curve::Circle { .. }) => circle_circle(c1, r1, c2, r2, tol),
        (Curve::Ellipse { .. }, Curve::Circle { .. }) => ellipse_circle(c1, r1, c2, r2, tol).unwrap_or_else(|| general(c1, r1, c2, r2, tol)),
        (Curve::Circle { .. }, Curve::Ellipse { .. }) => match ellipse_circle(c2, r2, c1, r1, tol) {
            Some(r) => swap(r),
            None => general(c1, r1, c2, r2, tol),
        },
        _ => general(c1, r1, c2, r2, tol),
    };
    finish(raw, tol)
}

fn swap(r: CcResult) -> CcResult {
    CcResult {
        hits: r.hits.into_iter().map(|h| CcHit { t1: h.t2, t2: h.t1, p: h.p }).collect(),
        overlaps: r.overlaps.into_iter().map(|o| CcOverlap { a: o.b, b: o.a }).collect(),
    }
}

/// Sort by t1, drop duplicates and points inside overlaps.
fn finish(mut r: CcResult, tol: f64) -> CcResult {
    r.hits.sort_by(|a, b| a.t1.partial_cmp(&b.t1).unwrap_or(std::cmp::Ordering::Equal));
    let mut out: Vec<CcHit> = vec![];
    for h in r.hits {
        if out.iter().any(|o| o.p.dist(h.p) <= tol && (o.t1 - h.t1).abs() < 1e-6 && (o.t2 - h.t2).abs() < 1e-6) {
            continue;
        }
        out.push(h);
    }
    r.hits = out;
    r.overlaps.sort_by(|a, b| a.a.0.partial_cmp(&b.a.0).unwrap_or(std::cmp::Ordering::Equal));
    r
}

/// Push a hit if both parameters fit their ranges (all periodic aliases).
fn push_hit(out: &mut CcResult, c1: &Curve, r1: (f64, f64), t1: f64, c2: &Curve, r2: (f64, f64), t2: f64, tol: f64) {
    let s1 = param_slack(c1, t1, tol);
    let s2 = param_slack(c2, t2, tol);
    for a in fit_params_all(c1, t1, r1, s1) {
        for b in fit_params_all(c2, t2, r2, s2) {
            let p = (c1.eval(a) + c2.eval(b)) * 0.5;
            if c1.eval(a).dist(c2.eval(b)) <= 2.0 * tol {
                out.hits.push(CcHit { t1: a, t2: b, p });
            }
        }
    }
}

// ---------------------------------------------------------------- lines

fn line_line(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) -> CcResult {
    let (Curve::Line { o: o1, d: d1 }, Curve::Line { o: o2, d: d2 }) = (c1, c2) else { unreachable!() };
    let mut out = CcResult::default();
    let w = *o1 - *o2;
    let (a, b, c) = (d1.dot(*d1), d1.dot(*d2), d2.dot(*d2));
    let (d, e) = (d1.dot(w), d2.dot(w));
    let den = a * c - b * b;
    let n = d1.cross(*d2);
    if n.len2() <= 1e-24 * a * c {
        // parallel: collinear?
        let t = c2.closest_param(*o1);
        if c2.eval(t).dist(*o1) > tol {
            return out;
        }
        overlap_by_projection(&mut out, c1, r1, c2, r2, tol);
        return out;
    }
    let s = (b * e - c * d) / den;
    let t = (a * e - b * d) / den;
    if c1.eval(s).dist(c2.eval(t)) <= tol {
        push_hit(&mut out, c1, r1, s, c2, r2, t, tol);
    }
    out
}

/// Overlap of two coincident carriers given as ranges: project the ends of
/// each range onto the other curve.
fn overlap_by_projection(out: &mut CcResult, c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) {
    // only for lines: map c2's range into c1's parameter (affine)
    let map = |t2: f64| c1.closest_param(c2.eval(t2));
    let (a0, a1) = (map(r2.0), map(r2.1));
    let (lo, hi) = (a0.min(a1), a0.max(a1));
    let s0 = r1.0.max(lo);
    let s1 = r1.1.min(hi);
    if !(s0.is_finite() && s1.is_finite()) {
        return;
    }
    let len = c1.eval(s0).dist(c1.eval(s1));
    if s1 < s0 || len <= tol {
        if len <= tol && s1 >= s0 - param_slack(c1, s0, tol) {
            // touching at one point
            let t2 = c2.closest_param(c1.eval(s0));
            push_hit(out, c1, r1, s0, c2, r2, t2, tol);
        }
        return;
    }
    let b0 = c2.closest_param(c1.eval(s0));
    let b1 = c2.closest_param(c1.eval(s1));
    out.overlaps.push(CcOverlap { a: (s0, s1), b: (b0, b1) });
}

fn line_circle(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) -> CcResult {
    let (Curve::Line { o, d }, Curve::Circle { frame, r }) = (c1, c2) else { unreachable!() };
    let mut out = CcResult::default();
    let n = frame.z;
    let dn = d.dot(n);
    let on = (*o - frame.o).dot(n);
    let mut ts: Vec<f64> = vec![];
    if dn.abs() * d.len().recip() < 1e-12 {
        // line parallel to the circle plane
        if on.abs() > tol {
            return out;
        }
        // |o + t d - c|² = r²
        let w = *o - frame.o;
        let roots = quad::quadratic(d.dot(*d), 2.0 * w.dot(*d), w.dot(w) - r * r);
        ts.extend(roots.iter());
        if roots.is_empty() {
            // near-tangent: closest approach within tol
            let t = -w.dot(*d) / d.dot(*d);
            let p = c1.eval(t);
            if ((p - frame.o).len() - r).abs() <= tol {
                ts.push(t);
            }
        }
    } else {
        let t = -on / dn;
        let p = c1.eval(t);
        if ((p - frame.o).reject(n).len() - r).abs() <= tol {
            ts.push(t);
        }
    }
    for t in ts {
        let p = c1.eval(t);
        let t2 = c2.closest_param(p);
        push_hit(&mut out, c1, r1, t, c2, r2, t2, tol);
    }
    out
}

fn circle_circle(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) -> CcResult {
    let (Curve::Circle { frame: f1, r: ra }, Curve::Circle { frame: f2, r: rb }) = (c1, c2) else { unreachable!() };
    let mut out = CcResult::default();
    let coplanar = f1.z.cross(f2.z).len() < 1e-10 && (f2.o - f1.o).dot(f1.z).abs() <= tol;
    if coplanar {
        let dc = f2.o - f1.o;
        let dist = dc.len();
        if dist <= tol && (ra - rb).abs() <= tol {
            // same circle: overlap of the two ranges on the circle
            circle_overlap(&mut out, c1, r1, c2, r2, tol);
            return out;
        }
        if dist <= tol || dist > ra + rb + tol || dist < (ra - rb).abs() - tol {
            return out;
        }
        // radical line
        let a = (ra * ra - rb * rb + dist * dist) / (2.0 * dist);
        let h2 = ra * ra - a * a;
        let ex = dc / dist;
        let ey = f1.z.cross(ex);
        let base = f1.o + ex * a;
        let pts = if h2 <= (tol * tol).max(0.0) || h2 < 0.0 {
            vec![base]
        } else {
            let h = h2.sqrt();
            vec![base + ey * h, base - ey * h]
        };
        for p in pts {
            push_hit(&mut out, c1, r1, c1.closest_param(p), c2, r2, c2.closest_param(p), tol);
        }
        return out;
    }
    // skew planes: points of circle 1 on plane 2, then check on circle 2
    for t in circle_plane_params(f1, *ra, f2.o, f2.z) {
        let p = c1.eval(t);
        if ((p - f2.o).len() - rb).abs() <= tol {
            push_hit(&mut out, c1, r1, t, c2, r2, c2.closest_param(p), tol);
        }
    }
    out
}

/// Ellipse x circle on a different plane: the ellipse's points on the
/// circle's plane, kept when on the circle (closed form: a tangent contact,
/// e.g. mitre ellipses of two equal fillets touching a face plane, keeps
/// full precision where the Newton fallback loses half the digits).
/// `None` when the planes coincide.
fn ellipse_circle(e: &Curve, re: (f64, f64), c: &Curve, rc: (f64, f64), tol: f64) -> Option<CcResult> {
    let (Curve::Ellipse { frame: fe, a, b }, Curve::Circle { frame: fc, r }) = (e, c) else { return None };
    if fe.z.cross(fc.z).len() < 1e-10 {
        return None;
    }
    let mut out = CcResult::default();
    for t in ellipse_plane_params(fe, *a, *b, fc.o, fc.z) {
        let p = e.eval(t);
        if ((p - fc.o).reject(fc.z).len() - r).abs() <= tol {
            push_hit(&mut out, e, re, t, c, rc, c.closest_param(p), tol);
        }
    }
    Some(out)
}

/// Circle parameters where the circle meets plane (o, n): a cos t + b sin t + c = 0.
pub fn circle_plane_params(f: &ck_geom::Frame, r: f64, o: Vec3, n: Vec3) -> Vec<f64> {
    ellipse_plane_params(f, r, r, o, n)
}

/// Ellipse (semi-axes `ra` along x, `rb` along y) parameters where it meets
/// plane (o, n). A tangency (the two roots closer than ~1e-7 in length: the
/// rounding noise of `acos` near 1 would otherwise split it by ~1e-8) is
/// one root.
pub fn ellipse_plane_params(f: &ck_geom::Frame, ra: f64, rb: f64, o: Vec3, n: Vec3) -> Vec<f64> {
    let a = ra * f.x.dot(n);
    let b = rb * f.y.dot(n);
    let c = (f.o - o).dot(n);
    let m = (a * a + b * b).sqrt();
    if m < 1e-300 {
        return vec![];
    }
    let x = -c / m;
    if x.abs() > 1.0 + 1e-12 {
        return vec![];
    }
    let phi = b.atan2(a);
    let al = x.max(-1.0).min(1.0).acos();
    if al < 1e-9 || (ra != rb && al * ra.max(rb) < 1e-7) {
        vec![ck_geom::curve::wrap_angle(phi)]
    } else {
        let mut v = vec![ck_geom::curve::wrap_angle(phi - al), ck_geom::curve::wrap_angle(phi + al)];
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v
    }
}

/// Two coincident circles (same carrier up to direction): intersect the
/// angular ranges.
fn circle_overlap(out: &mut CcResult, c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) {
    // sample c2's range on c1's parameter
    let n = 64;
    let mut inside = vec![];
    for i in 0..=n {
        let t = r1.0 + (r1.1 - r1.0) * i as f64 / n as f64;
        let p = c1.eval(t);
        let t2 = c2.closest_param(p);
        inside.push(fit_param(c2, t2, r2, param_slack(c2, t2, tol)).is_some());
    }
    // contiguous runs, refined by the end points of c2's range
    let ends2 = [c2.eval(r2.0), c2.eval(r2.1)];
    let mut cuts: Vec<f64> = vec![r1.0, r1.1];
    for p in ends2 {
        for t in fit_params_all(c1, c1.closest_param(p), r1, param_slack(c1, 0.0, tol)) {
            cuts.push(t);
        }
    }
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    let _ = inside;
    for w in cuts.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b - a < 1e-12 {
            continue;
        }
        let m = c1.eval(0.5 * (a + b));
        let tm = c2.closest_param(m);
        if fit_param(c2, tm, r2, 0.0).is_none() {
            continue;
        }
        // c2 params at a, b, continuous through tm
        let ta = near(c2, c2.closest_param(c1.eval(a)), tm);
        let tb = near(c2, c2.closest_param(c1.eval(b)), tm);
        let tm2 = fit_param(c2, tm, r2, 0.0).unwrap();
        let sh = tm2 - tm;
        out.overlaps.push(CcOverlap { a: (a, b), b: (ta + sh, tb + sh) });
    }
    // merge adjacent pieces
    out.overlaps.dedup_by(|b, a| {
        if (b.a.0 - a.a.1).abs() < 1e-12 {
            a.a.1 = b.a.1;
            a.b.1 = b.b.1;
            true
        } else {
            false
        }
    });
}

fn near(c: &Curve, t: f64, m: f64) -> f64 {
    match c.period() {
        Some(p) => {
            // closest alias to m within half a period
            let k = ((m - t) / p).round();
            t + k * p
        }
        None => t,
    }
}

// ---------------------------------------------------------------- general

/// Polyline seeding + Newton. Overlaps are detected as runs of seed points
/// of curve 1 lying on curve 2.
pub fn general(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64), tol: f64) -> CcResult {
    let mut out = CcResult::default();
    if !(r1.0.is_finite() && r1.1.is_finite() && r2.0.is_finite() && r2.1.is_finite()) {
        // bound infinite lines by the other curve's box
        let (lo, hi) = range_box(c2, r2).or_else(|| range_box(c1, r1)).unwrap_or((Vec3::ZERO, Vec3::ZERO));
        let pad = Vec3::new(1.0, 1.0, 1.0) * (hi - lo).len().max(1.0);
        let r1f = super::finite_range(c1, r1, lo - pad, hi + pad);
        let r2f = super::finite_range(c2, r2, lo - pad, hi + pad);
        if !(r1f.0.is_finite() && r1f.1.is_finite() && r2f.0.is_finite() && r2f.1.is_finite()) {
            return out;
        }
        let mut r = general(c1, r1f, c2, r2f, tol);
        r.hits.retain(|h| h.t1 >= r1.0 && h.t1 <= r1.1 && h.t2 >= r2.0 && h.t2 <= r2.1);
        return r;
    }
    let n1 = samples_for(c1);
    let n2 = samples_for(c2);
    let p1 = c1.sample(r1.0, r1.1, n1);
    let p2 = c2.sample(r2.0, r2.1, n2);
    // sample boxes (padded by the longest chord, the reach the segment test
    // below allows) that do not meet cannot intersect: skips the closest-
    // point overlap scan over a spline, the slow part of most misses
    {
        let bx = |p: &[Vec3]| {
            let (mut lo, mut hi, mut m) = (p[0], p[0], 0.0f64);
            for (i, q) in p.iter().enumerate() {
                lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
                hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
                if i > 0 {
                    m = m.max(q.dist(p[i - 1]));
                }
            }
            (lo, hi, m)
        };
        let (lo1, hi1, m1) = bx(&p1);
        let (lo2, hi2, m2) = bx(&p2);
        let pad = m1.max(m2) + tol;
        if lo1.x - pad > hi2.x || lo2.x - pad > hi1.x || lo1.y - pad > hi2.y || lo2.y - pad > hi1.y || lo1.z - pad > hi2.z || lo2.z - pad > hi1.z {
            return out;
        }
    }
    let t1s: Vec<f64> = (0..=n1).map(|i| r1.0 + (r1.1 - r1.0) * i as f64 / n1 as f64).collect();
    let t2s: Vec<f64> = (0..=n2).map(|i| r2.0 + (r2.1 - r2.0) * i as f64 / n2 as f64).collect();
    // overlap check: all samples of c1 (or a run) on c2
    // (a sample farther from c2's polyline than its chord sag cannot lie on
    // c2: skips the closest-point search on long splines such as helices)
    let sag2 = (0..n2)
        .map(|j| c2.eval(0.5 * (t2s[j] + t2s[j + 1])).dist((p2[j] + p2[j + 1]) * 0.5))
        .fold(0.0f64, f64::max);
    let far = 2.0 * sag2 + 4.0 * tol;
    let on2: Vec<bool> = p1
        .iter()
        .map(|&p| {
            if n2 > 8 && (0..n2).all(|j| point_seg_dist(p, p2[j], p2[j + 1]) > far) {
                return false;
            }
            let t = c2.closest_param(p);
            c2.eval(t).dist(p) <= tol && fit_param(c2, t, r2, param_slack(c2, t, tol)).is_some()
        })
        .collect();
    let mut i = 0;
    while i < on2.len() {
        if on2[i] {
            let mut j = i;
            while j + 1 < on2.len() && on2[j + 1] {
                j += 1;
            }
            if j > i {
                let (a, b) = (t1s[i], t1s[j]);
                let ta = c2.closest_param(c1.eval(a));
                let tb = c2.closest_param(c1.eval(b));
                out.overlaps.push(CcOverlap { a: (a, b), b: (ta, tb) });
            }
            i = j + 1;
        } else {
            i += 1;
        }
    }
    // segment pairs closer than their chord lengths (long splines, whose
    // chords run side by side: closer than the chord sags suffices - a
    // crossing of the curves puts the two chords within the sum of the sags)
    let long_pair = n1 >= 200 || n2 >= 200;
    let sag1 = (0..n1).map(|i| c1.eval(0.5 * (t1s[i] + t1s[i + 1])).dist((p1[i] + p1[i + 1]) * 0.5)).fold(0.0f64, f64::max);
    for i in 0..n1 {
        let (a0, a1) = (p1[i], p1[i + 1]);
        for j in 0..n2 {
            let (b0, b1) = (p2[j], p2[j + 1]);
            let (s, t, d) = seg_seg(a0, a1, b0, b1);
            let reach = if long_pair { 1.5 * (sag1 + sag2) + 4.0 * tol } else { 0.5 * (a0.dist(a1) + b0.dist(b1)) + tol };
            if d > reach {
                continue;
            }
            let ta = t1s[i] + (t1s[i + 1] - t1s[i]) * s;
            let tb = t2s[j] + (t2s[j + 1] - t2s[j]) * t;
            // neighbouring segment pairs report the same crossing (long
            // splines running side by side): skip a hit already found
            let (d1, d2) = ((t1s[i + 1] - t1s[i]).abs(), (t2s[j + 1] - t2s[j]).abs());
            if out.hits.iter().any(|h| (h.t1 - ta).abs() <= d1 && (h.t2 - tb).abs() <= d2) {
                continue;
            }
            if let Some((x, y)) = newton2(c1, c2, ta, tb, r1, r2) {
                if c1.eval(x).dist(c2.eval(y)) <= tol {
                    if out.overlaps.iter().any(|o| x >= o.a.0 - 1e-9 && x <= o.a.1 + 1e-9) {
                        continue;
                    }
                    push_hit(&mut out, c1, r1, x, c2, r2, y, tol);
                }
            }
        }
    }
    out
}

fn range_box(c: &Curve, r: (f64, f64)) -> Option<(Vec3, Vec3)> {
    if !(r.0.is_finite() && r.1.is_finite()) {
        return None;
    }
    let pts = c.sample(r.0, r.1, 16);
    let mut lo = pts[0];
    let mut hi = pts[0];
    for p in pts {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    Some((lo, hi))
}

fn samples_for(c: &Curve) -> usize {
    match c {
        Curve::Line { .. } => 1,
        Curve::Circle { .. } | Curve::Ellipse { .. } => 64,
        Curve::BSpline(b) => (b.ctrl.len() * 8).clamp(16, 512),
        Curve::Conic { .. } | Curve::Sinusoid { .. } => 64,
    }
}

/// Closest points of segments [a0,a1], [b0,b1]: (s, t, distance).
fn point_seg_dist(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let d = b - a;
    let l2 = d.dot(d);
    let t = if l2 > 1e-300 { ((p - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.dist(a + d * t)
}

pub fn seg_seg(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3) -> (f64, f64, f64) {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let r = a0 - b0;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (mut s, mut t);
    if a <= 1e-300 && e <= 1e-300 {
        return (0.0, 0.0, a0.dist(b0));
    }
    if a <= 1e-300 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= 1e-300 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let den = a * e - b * b;
            s = if den > 1e-300 { ((b * f - c * e) / den).clamp(0.0, 1.0) } else { 0.0 };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    let p = a0 + d1 * s;
    let q = b0 + d2 * t;
    (s, t, p.dist(q))
}

/// Newton on F(s,t) = C1(s) − C2(t) (least squares, Gauss-Newton), params
/// kept within a slightly widened range.
pub fn newton2(c1: &Curve, c2: &Curve, mut s: f64, mut t: f64, r1: (f64, f64), r2: (f64, f64)) -> Option<(f64, f64)> {
    let w1 = (r1.1 - r1.0).abs() * 0.05 + 1e-9;
    let w2 = (r2.1 - r2.0).abs() * 0.05 + 1e-9;
    for _ in 0..40 {
        let d1 = c1.derivs(s, 1);
        let d2 = c2.derivs(t, 1);
        let f = d1[0] - d2[0];
        if f.len() < 1e-15 {
            break;
        }
        // J = [C1', -C2']; solve (JᵀJ) δ = -Jᵀ f
        let (a, b) = (d1[1], -d2[1]);
        let (aa, ab, bb) = (a.dot(a), a.dot(b), b.dot(b));
        let (fa, fb) = (a.dot(f), b.dot(f));
        let det = aa * bb - ab * ab;
        if det.abs() < 1e-300 {
            return None;
        }
        let ds = (-fa * bb + fb * ab) / det;
        let dt = (-fb * aa + fa * ab) / det;
        s += ds;
        t += dt;
        if !(s.is_finite() && t.is_finite()) {
            return None;
        }
        if c1.period().is_none() && (s < r1.0 - w1 || s > r1.1 + w1) {
            return None;
        }
        if c2.period().is_none() && (t < r2.0 - w2 || t > r2.1 + w2) {
            return None;
        }
        if ds.abs() < 1e-15 && dt.abs() < 1e-15 {
            break;
        }
    }
    Some((s, t))
}

/// Convenience: transversal points only (default tolerance).
pub fn points(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64)) -> Vec<CcHit> {
    intersect(c1, r1, c2, r2, TOL).hits
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::nurbs::bspline::BSplineCurve;
    use std::f64::consts::{PI, TAU};

    #[test]
    fn lines() {
        let a = Curve::line_through(Vec3::ZERO, Vec3::new(10., 0., 0.));
        let b = Curve::line_through(Vec3::new(3., -2., 0.), Vec3::new(3., 5., 0.));
        let r = intersect(&a, (0., 10.), &b, (0., 7.), 1e-9);
        assert_eq!(r.hits.len(), 1);
        assert!((r.hits[0].t1 - 3.0).abs() < 1e-12 && (r.hits[0].t2 - 2.0).abs() < 1e-12);
        // out of range
        assert!(intersect(&a, (4., 10.), &b, (0., 7.), 1e-9).hits.is_empty());
        // skew
        let c = Curve::line_through(Vec3::new(3., -2., 1.), Vec3::new(3., 5., 1.));
        assert!(intersect(&a, (0., 10.), &c, (0., 7.), 1e-9).hits.is_empty());
        // collinear overlap
        let d = Curve::line_through(Vec3::new(12., 0., 0.), Vec3::new(5., 0., 0.));
        let r = intersect(&a, (0., 10.), &d, (0., 7.), 1e-9);
        assert_eq!(r.overlaps.len(), 1);
        let o = r.overlaps[0];
        assert!((o.a.0 - 5.0).abs() < 1e-12 && (o.a.1 - 10.0).abs() < 1e-12);
        assert!((o.b.0 - 7.0).abs() < 1e-12 && (o.b.1 - 2.0).abs() < 1e-12);
    }

    #[test]
    fn line_and_circle() {
        let c = Curve::circle_x(Vec3::new(1., 1., 0.), Vec3::Z, Vec3::X, 2.0);
        let l = Curve::line(Vec3::new(-5., 1., 0.), Vec3::X);
        let r = intersect(&l, (f64::NEG_INFINITY, f64::INFINITY), &c, (0., TAU - 1e-3), 1e-9);
        assert_eq!(r.hits.len(), 2);
        assert!((r.hits[0].t1 - 4.0).abs() < 1e-12 && (r.hits[0].t2 - PI).abs() < 1e-12);
        assert!((r.hits[1].t1 - 8.0).abs() < 1e-12 && r.hits[1].t2.abs() < 1e-12);
        // circle range [0, 2π] closed: t=0 also matches 2π
        let r = intersect(&c, (0., TAU), &l, (f64::NEG_INFINITY, f64::INFINITY), 1e-9);
        assert_eq!(r.hits.len(), 3);
        // tangent
        let t = Curve::line(Vec3::new(-5., 3., 0.), Vec3::X);
        let r = intersect(&t, (-10., 10.), &c, (0., TAU), 1e-9);
        assert_eq!(r.hits.len(), 1);
        assert!((r.hits[0].p - Vec3::new(1., 3., 0.)).len() < 1e-9);
        // piercing the plane
        let p = Curve::line(Vec3::new(3., 1., -4.), Vec3::Z);
        let r = intersect(&p, (-10., 10.), &c, (0., TAU), 1e-9);
        assert_eq!(r.hits.len(), 2); // t2 = 0 and 2π
        assert!((r.hits[0].t1 - 4.0).abs() < 1e-12);
    }

    #[test]
    fn circles() {
        let a = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
        let b = Curve::circle_x(Vec3::new(2., 0., 0.), Vec3::Z, Vec3::X, 2.0);
        let r = intersect(&a, (0., TAU - 1e-3), &b, (0., TAU - 1e-3), 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!((h.p.x - 1.0).abs() < 1e-12 && (h.p.y.abs() - 3f64.sqrt()).abs() < 1e-12);
        }
        // skew: vertical circle through a's points (±2,0,0)
        let v = Curve::circle_x(Vec3::ZERO, Vec3::Y, Vec3::X, 2.0);
        let r = intersect(&a, (0., TAU - 1e-3), &v, (0., TAU - 1e-3), 1e-9);
        assert_eq!(r.hits.len(), 2);
        // coincident arcs
        let r = intersect(&a, (0., PI), &a.reversed(), (PI, TAU), 1e-9);
        assert_eq!(r.overlaps.len(), 1, "{r:?}");
        assert!((r.overlaps[0].a.0 - 0.0).abs() < 1e-9 && (r.overlaps[0].a.1 - PI).abs() < 1e-9);
    }

    #[test]
    fn general_spline_vs_line() {
        let s = BSplineCurve::new(
            3,
            vec![0., 0., 0., 0., 1., 1., 1., 1.],
            vec![Vec3::new(0., 0., 0.), Vec3::new(1., 2., 0.), Vec3::new(2., -2., 0.), Vec3::new(3., 0., 0.)],
        )
        .unwrap();
        let c = Curve::BSpline(s);
        let l = Curve::line(Vec3::new(-1., 0., 0.), Vec3::X);
        let r = intersect(&c, (0., 1.), &l, (-10., 10.), 1e-9);
        // y(t) = 6t(1-t)^2 - 6 t^2 (1-t) = 6t(1-t)(1-2t): roots 0, 0.5, 1
        let ts: Vec<f64> = r.hits.iter().map(|h| h.t1).collect();
        assert_eq!(ts.len(), 3, "{ts:?}");
        assert!((ts[1] - 0.5).abs() < 1e-12);
        // ellipse vs circle (general path)
        let e = Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0);
        let c = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
        let r = intersect(&e, (0., TAU - 1e-6), &c, (0., TAU - 1e-6), 1e-9);
        assert_eq!(r.hits.len(), 4, "{r:?}");
        for h in &r.hits {
            assert!((h.p.len() - 2.0).abs() < 1e-9);
        }
    }
}
