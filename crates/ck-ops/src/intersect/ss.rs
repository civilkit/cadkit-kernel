//! Surface-surface intersection (ck-ops/src/intersect/ss.rs).
//!
//! Analytic cases (exact carriers, untrimmed):
//! - plane/plane → line; coincident planes → `Coincident`
//! - plane/cylinder → 0–2 axial lines, circle (perpendicular), ellipse
//! - plane/sphere → circle
//! - plane/cone → circle, ellipse (closed-form axes from the meridian
//!   generators), 2 generator lines through the apex; parabola/hyperbola
//!   fall through to marching
//! - plane/torus → 1–2 latitude circles (perpendicular) or the 2 meridian
//!   circles (plane through the axis)
//! - cylinder/cylinder with parallel axes → lines; coaxial → `Coincident`
//!   (same radius) or disjoint
//! - coaxial cylinder/cone/sphere pairs and sphere/sphere → circles, by
//!   intersecting the meridian profiles
//! - everything else: marching SSI (grid seeds + predictor/corrector march
//!   + cubic B-spline fit through the traced points), restricted to a box.
//!
//! GUESS: circle reference directions (`x`) follow the first surface's
//! frame x (oracle: box C0 ∩ blind-cut cylinder gives a circle starting
//! at c + r·x_sketch, n = cylinder axis — probe `blindcut`).

use super::solve3;
use ck_geom::curve::Curve;
use ck_geom::fit::{interpolate, Param};
use ck_geom::surface::Surface;
use ck_geom::{any_perp, frame_from, Frame, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub enum SsCurve {
    /// Exact analytic carrier (lines unbounded, conics closed).
    Exact(Curve),
    /// Marched polyline + interpolating cubic B-spline through it.
    Fitted { pts: Vec<Vec3>, curve: Curve, closed: bool },
}

impl SsCurve {
    pub fn curve(&self) -> &Curve {
        match self {
            SsCurve::Exact(c) => c,
            SsCurve::Fitted { curve, .. } => curve,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SsResult {
    Disjoint,
    /// Same surface (up to orientation): the overlap is a region, not curves.
    Coincident,
    Curves(Vec<SsCurve>),
}

impl SsResult {
    pub fn curves(&self) -> &[SsCurve] {
        match self {
            SsResult::Curves(c) => c,
            _ => &[],
        }
    }
}

/// Region of interest for marching (and nothing else): axis-aligned box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub lo: Vec3,
    pub hi: Vec3,
}

/// Intersect two surfaces. `bounds` limits the marching fallback (required
/// for it; without bounds non-analytic pairs return `Curves(vec![])`).
pub fn intersect(s1: &Surface, s2: &Surface, bounds: Option<Bounds>, tol: f64) -> SsResult {
    if let Some(r) = analytic(s1, s2, tol) {
        return r;
    }
    if let Some(r) = analytic(s2, s1, tol) {
        return r;
    }
    match bounds {
        Some(b) => {
            let step = ((b.hi - b.lo).len() / 64.0).max(tol * 100.0);
            let cs = march(s1, s2, b, tol, step);
            if cs.is_empty() {
                SsResult::Disjoint
            } else {
                SsResult::Curves(cs)
            }
        }
        None => SsResult::Curves(vec![]),
    }
}

fn curves(v: Vec<Curve>) -> SsResult {
    if v.is_empty() {
        SsResult::Disjoint
    } else {
        SsResult::Curves(v.into_iter().map(SsCurve::Exact).collect())
    }
}

/// Closed-form pairs with `s1` the "simpler" surface; None = not handled.
fn analytic(s1: &Surface, s2: &Surface, tol: f64) -> Option<SsResult> {
    use Surface::*;
    match (s1, s2) {
        (Plane { frame: a }, Plane { frame: b }) => Some(plane_plane(a, b, tol)),
        (Plane { frame: p }, Cylinder { frame, r }) => Some(plane_cylinder(p, frame, *r, tol)),
        (Plane { frame: p }, Sphere { frame, r }) => Some(plane_sphere(p, frame, *r, tol)),
        (Plane { frame: p }, Cone { frame, r, semi_angle }) => plane_cone(p, frame, *r, *semi_angle, tol),
        (Plane { frame: p }, Torus { frame, major, minor }) => plane_torus(p, frame, *major, *minor, tol),
        (Cylinder { frame: a, r: ra }, Cylinder { frame: b, r: rb }) => cyl_cyl(a, *ra, b, *rb, tol),
        (Sphere { frame: a, r: ra }, Sphere { frame: b, r: rb }) => Some(sphere_sphere(a.o, *ra, b.o, *rb, tol)),
        _ => coaxial(s1, s2, tol),
    }
}

fn plane_plane(a: &Frame, b: &Frame, tol: f64) -> SsResult {
    let pa = a.plane();
    let pb = b.plane();
    if a.z.cross(b.z).len() < 1e-12 {
        return if pa.dist(b.o) <= tol { SsResult::Coincident } else { SsResult::Disjoint };
    }
    match pa.intersect_plane(&pb) {
        Some((p, d)) => curves(vec![Curve::Line { o: p, d }]),
        None => SsResult::Disjoint,
    }
}

fn plane_cylinder(p: &Frame, cf: &Frame, r: f64, tol: f64) -> SsResult {
    let n = p.z;
    let z = cf.z;
    let c = n.dot(z);
    if c.abs() < 1e-12 {
        // plane parallel to the axis
        let dd = (cf.o - p.o).dot(n);
        if dd.abs() > r + tol {
            return SsResult::Disjoint;
        }
        let foot = cf.o - n * dd;
        let w = n.cross(z).normalize();
        let h2 = r * r - dd * dd;
        if h2 <= 2.0 * r * tol {
            return curves(vec![Curve::Line { o: foot, d: z }]);
        }
        let h = h2.sqrt();
        return curves(vec![Curve::Line { o: foot + w * h, d: z }, Curve::Line { o: foot - w * h, d: z }]);
    }
    let t = (p.o - cf.o).dot(n) / c;
    let ctr = cf.o + z * t;
    if (c.abs() - 1.0).abs() < 1e-12 {
        return curves(vec![Curve::Circle { frame: frame_from(ctr, z, cf.x), r }]);
    }
    // ellipse: minor axis ⟂ axis (length r), major in the meridian
    let m = z.cross(n).normalize();
    let big = n.cross(m).normalize();
    let a = r / c.abs();
    let nz = if c > 0.0 { n } else { -n };
    curves(vec![Curve::Ellipse { frame: frame_from(ctr, nz, big), a, b: r }])
}

fn plane_sphere(p: &Frame, sf: &Frame, r: f64, tol: f64) -> SsResult {
    let n = p.z;
    let d = (sf.o - p.o).dot(n);
    let rho2 = r * r - d * d;
    if d.abs() > r + tol || rho2 <= 2.0 * r * tol {
        return SsResult::Disjoint;
    }
    let ctr = sf.o - n * d;
    curves(vec![Curve::Circle { frame: frame_from(ctr, n, sf.x), r: rho2.sqrt() }])
}

fn plane_cone(p: &Frame, cf: &Frame, r: f64, alpha: f64, tol: f64) -> Option<SsResult> {
    let n = p.z;
    let z = cf.z;
    let k = alpha.tan();
    let c = n.dot(z);
    let apex_v = if k.abs() > 1e-300 { -r / k } else { f64::INFINITY };
    let apex = cf.o + z * apex_v;
    if (c.abs() - 1.0).abs() < 1e-12 {
        let v = (p.o - cf.o).dot(n) / c;
        let rho = r + k * v;
        if rho.abs() <= tol {
            return Some(SsResult::Disjoint);
        }
        return Some(curves(vec![Curve::Circle { frame: frame_from(cf.o + z * v, z, cf.x), r: rho.abs() }]));
    }
    let e = (n - z * c).normalize(); // meridian radial direction
    let ne = n.dot(e);
    let sin_a = alpha.sin().abs();
    let through_apex = apex_v.is_finite() && (apex - p.o).dot(n).abs() <= tol;
    if through_apex {
        if c.abs() > sin_a + 1e-12 {
            return Some(SsResult::Disjoint); // single point
        }
        // generators g(u) = radial(u) k + z with n·g = 0
        let w = z.cross(e);
        // radial(u) = cos φ e + sin φ w ; n·radial = cos φ ne
        let x = -c / (k * ne);
        if x.abs() > 1.0 {
            return Some(SsResult::Disjoint);
        }
        let phi = x.acos();
        let mut out = vec![];
        for ph in [phi, -phi] {
            let rad = e * ph.cos() + w * ph.sin();
            out.push(Curve::Line { o: apex, d: (rad * k + z).normalize() });
        }
        if phi.abs() < 1e-9 {
            out.pop();
        }
        return Some(curves(out));
    }
    if c.abs() <= sin_a + 1e-12 {
        return None; // parabola / hyperbola → marching
    }
    // ellipse: the two meridian generators meet the plane at the major-axis ends
    let h = (cf.o - p.o).dot(n);
    let mut ends = vec![];
    for s in [1.0, -1.0] {
        let den = c + s * ne * k;
        if den.abs() < 1e-300 {
            return None;
        }
        let v = -(h + s * ne * r) / den;
        ends.push(cf.o + e * (s * (r + k * v)) + z * v);
    }
    let ctr = (ends[0] + ends[1]) * 0.5;
    let a = ends[0].dist(ends[1]) * 0.5;
    if a <= tol {
        return Some(SsResult::Disjoint);
    }
    let major = (ends[0] - ends[1]).normalize();
    let minor = n.cross(major).normalize();
    // b: point ctr + minor s on the cone: |radial part|² = (r + k v)²
    let l = cf.to_local(ctr);
    let md = cf.vec_to_local(minor);
    let rr = r + k * l.z;
    let qa = md.x * md.x + md.y * md.y - k * k * md.z * md.z;
    let qb = 2.0 * (l.x * md.x + l.y * md.y - k * md.z * rr);
    let qc = l.x * l.x + l.y * l.y - rr * rr;
    let roots = ck_core::quad::quadratic(qa, qb, qc);
    if roots.len() < 2 {
        return None;
    }
    let b = 0.5 * (roots.r[1] - roots.r[0]).abs();
    let nz = if c > 0.0 { n } else { -n };
    Some(curves(vec![Curve::Ellipse { frame: frame_from(ctr, nz, major), a, b }]))
}

fn plane_torus(p: &Frame, tf: &Frame, big: f64, small: f64, tol: f64) -> Option<SsResult> {
    let n = p.z;
    let z = tf.z;
    let c = n.dot(z);
    if (c.abs() - 1.0).abs() < 1e-12 {
        let hgt = (p.o - tf.o).dot(z);
        if hgt.abs() > small + tol {
            return Some(SsResult::Disjoint);
        }
        let w2 = small * small - hgt * hgt;
        let ctr = tf.o + z * hgt;
        if w2 <= 2.0 * small * tol {
            return Some(curves(vec![Curve::Circle { frame: frame_from(ctr, z, tf.x), r: big }]));
        }
        let w = w2.sqrt();
        let mut v = vec![Curve::Circle { frame: frame_from(ctr, z, tf.x), r: big + w }];
        if big - w > tol {
            v.push(Curve::Circle { frame: frame_from(ctr, z, tf.x), r: big - w });
        }
        return Some(curves(v));
    }
    if c.abs() < 1e-12 && (tf.o - p.o).dot(n).abs() <= tol {
        // plane through the axis: two meridian circles
        let w = z.cross(n).normalize();
        let out = vec![
            Curve::Circle { frame: frame_from(tf.o + w * big, n, w), r: small },
            Curve::Circle { frame: frame_from(tf.o - w * big, n, -w), r: small },
        ];
        return Some(curves(out));
    }
    None
}

fn cyl_cyl(a: &Frame, ra: f64, b: &Frame, rb: f64, tol: f64) -> Option<SsResult> {
    if a.z.cross(b.z).len() > 1e-12 {
        return None;
    }
    let z = a.z;
    let dvec = (b.o - a.o).reject(z);
    let dist = dvec.len();
    if dist <= tol {
        return Some(if (ra - rb).abs() <= tol { SsResult::Coincident } else { SsResult::Disjoint });
    }
    if dist > ra + rb + tol || dist < (ra - rb).abs() - tol {
        return Some(SsResult::Disjoint);
    }
    let ex = dvec / dist;
    let ey = z.cross(ex);
    let x = (ra * ra - rb * rb + dist * dist) / (2.0 * dist);
    let h2 = ra * ra - x * x;
    let base = a.o + ex * x;
    if h2 <= 2.0 * ra * tol {
        return Some(curves(vec![Curve::Line { o: base, d: z }]));
    }
    let h = h2.sqrt();
    Some(curves(vec![Curve::Line { o: base + ey * h, d: z }, Curve::Line { o: base - ey * h, d: z }]))
}

fn sphere_sphere(ca: Vec3, ra: f64, cb: Vec3, rb: f64, tol: f64) -> SsResult {
    let d = cb - ca;
    let dist = d.len();
    if dist <= tol {
        return if (ra - rb).abs() <= tol { SsResult::Coincident } else { SsResult::Disjoint };
    }
    if dist > ra + rb + tol || dist < (ra - rb).abs() - tol {
        return SsResult::Disjoint;
    }
    let n = d / dist;
    let x = (ra * ra - rb * rb + dist * dist) / (2.0 * dist);
    let h2 = ra * ra - x * x;
    if h2 <= 2.0 * ra * tol {
        return SsResult::Disjoint;
    }
    curves(vec![Curve::Circle { frame: frame_from(ca + n * x, n, any_perp(n)), r: h2.sqrt() }])
}

/// Meridian profile of a surface of revolution about an axis (o, z):
/// returns (kind, params) with heights measured along z from o.
enum Meridian {
    /// ρ = r
    Cyl(f64),
    /// ρ = |r0 + k h|
    Cone(f64, f64),
    /// ρ² + (h − hc)² = R²
    Sphere(f64, f64),
    /// (ρ − major)² + (h − hc)² = minor²
    Torus(f64, f64, f64),
}

fn meridian(s: &Surface, o: Vec3, z: Vec3, tol: f64) -> Option<Meridian> {
    let on_axis = |p: Vec3| (p - o).cross(z).len() <= tol;
    match s {
        Surface::Cylinder { frame, r } if frame.z.cross(z).len() < 1e-12 && on_axis(frame.o) => Some(Meridian::Cyl(*r)),
        Surface::Cone { frame, r, semi_angle } if frame.z.cross(z).len() < 1e-12 && on_axis(frame.o) => {
            // local v = σ (h − h0)
            let sg = frame.z.dot(z).signum();
            let h0 = (frame.o - o).dot(z);
            let k = semi_angle.tan();
            // r + k σ (h − h0) = (r − k σ h0) + k σ h
            Some(Meridian::Cone(r - k * sg * h0, k * sg))
        }
        Surface::Sphere { frame, r } if on_axis(frame.o) => Some(Meridian::Sphere(*r, (frame.o - o).dot(z))),
        Surface::Torus { frame, major, minor } if frame.z.cross(z).len() < 1e-12 && on_axis(frame.o) => {
            Some(Meridian::Torus(*major, *minor, (frame.o - o).dot(z)))
        }
        _ => None,
    }
}

/// Coaxial cylinder / cone / sphere pairs → circles.
fn coaxial(s1: &Surface, s2: &Surface, tol: f64) -> Option<SsResult> {
    let (o, z, x) = match s1 {
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => (frame.o, frame.z, frame.x),
        _ => return None,
    };
    let m1 = meridian(s1, o, z, tol)?;
    let m2 = meridian(s2, o, z, tol)?;
    use Meridian::*;
    // collect heights h with radius ρ(h)
    let mut hs: Vec<(f64, f64)> = vec![];
    let lin = |a: f64, b: f64| -> Option<f64> {
        if b.abs() < 1e-300 {
            None
        } else {
            Some(-a / b)
        }
    };
    match (&m1, &m2) {
        (Cyl(_), Cyl(_)) => return None, // handled by cyl_cyl
        (Cyl(r), Cone(r0, k)) | (Cone(r0, k), Cyl(r)) => {
            // r0 + k h = r (GUESS: only the ρ > 0 nappe, the one ck-geom
            // cone faces and closest_param use)
            if let Some(h) = lin(r0 - r, *k) {
                hs.push((h, *r));
            }
        }
        (Cyl(r), Sphere(rs, hc)) | (Sphere(rs, hc), Cyl(r)) => {
            let w2 = rs * rs - r * r;
            if w2 > 2.0 * rs * tol {
                let w = w2.sqrt();
                hs.push((hc - w, *r));
                hs.push((hc + w, *r));
            }
        }
        (Cone(a0, ka), Cone(b0, kb)) => {
            if (ka - kb).abs() < 1e-14 && (a0 - b0).abs() <= tol {
                return Some(SsResult::Coincident);
            }
            // a0 + ka h = b0 + kb h, both on their ρ > 0 nappe
            if let Some(h) = lin(a0 - b0, ka - kb) {
                if b0 + kb * h > 0.0 {
                    hs.push((h, a0 + ka * h));
                }
            }
        }
        (Cone(r0, k), Sphere(rs, hc)) | (Sphere(rs, hc), Cone(r0, k)) => {
            // (r0 + k h)² + (h − hc)² = R²
            let qa = k * k + 1.0;
            let qb = 2.0 * (r0 * k - hc);
            let qc = r0 * r0 + hc * hc - rs * rs;
            for h in ck_core::quad::quadratic(qa, qb, qc).iter() {
                hs.push((h, r0 + k * h));
            }
        }
        (Sphere(..), Sphere(..)) => return None,
        (Cyl(r), Torus(big, small, hc)) | (Torus(big, small, hc), Cyl(r)) => {
            // (r − R)² + (h − hc)² = minor²; a tangent contact (the fillet
            // ball touching the cylinder) is one circle
            let w2 = small * small - (r - big) * (r - big);
            if w2 >= -2.0 * small * tol {
                if w2 <= 2.0 * small * tol {
                    hs.push((*hc, *r));
                } else {
                    let w = w2.sqrt();
                    hs.push((hc - w, *r));
                    hs.push((hc + w, *r));
                }
            }
        }
        (Cone(r0, k), Torus(big, small, hc)) | (Torus(big, small, hc), Cone(r0, k)) => {
            // (r0 + k h − R)² + (h − hc)² = minor²; double root = tangent
            let (qa, qb, qc) = (k * k + 1.0, 2.0 * (k * (r0 - big) - hc), (r0 - big) * (r0 - big) + hc * hc - small * small);
            let disc = qb * qb - 4.0 * qa * qc;
            let scale = qb * qb + 4.0 * qa * qc.abs();
            if disc >= -1e-12 * scale {
                if disc <= 1e-12 * scale {
                    let h = -qb / (2.0 * qa);
                    hs.push((h, r0 + k * h));
                } else {
                    for h in ck_core::quad::quadratic(qa, qb, qc).iter() {
                        hs.push((h, r0 + k * h));
                    }
                }
            }
        }
        (Torus(..), _) | (_, Torus(..)) => return None,
    }
    let mut out = vec![];
    hs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    hs.dedup_by(|a, b| (a.0 - b.0).abs() <= tol);
    for (h, rho) in hs {
        if rho > tol {
            out.push(Curve::Circle { frame: frame_from(o + z * h, z, x), r: rho });
        }
    }
    Some(curves(out))
}

// ---------------------------------------------------------------- marching

/// Newton corrector: the point on both surfaces closest to `pred` within
/// the plane through `pred` normal to `dir`.
pub fn correct(s1: &Surface, s2: &Surface, pred: Vec3, dir: Vec3, tol: f64) -> Option<Vec3> {
    let mut p = pred;
    for _ in 0..50 {
        let (u1, v1, q1) = s1.closest_point(p);
        let (u2, v2, q2) = s2.closest_point(p);
        let n1 = s1.normal(u1, v1);
        let n2 = s2.normal(u2, v2);
        let d1 = (p - q1).dot(n1);
        let d2 = (p - q2).dot(n2);
        let d3 = (p - pred).dot(dir);
        if d1.abs() < tol * 1e-4 && d2.abs() < tol * 1e-4 && d3.abs() < tol * 1e-4 {
            return Some(p);
        }
        let dp = solve3([n1, n2, dir], [-d1, -d2, -d3])?;
        p = p + dp;
        if !p.is_finite() {
            return None;
        }
    }
    let ok = s1.distance(p) <= tol && s2.distance(p) <= tol;
    if ok {
        Some(p)
    } else {
        None
    }
}

fn tangent(s1: &Surface, s2: &Surface, p: Vec3) -> Option<Vec3> {
    let (u1, v1) = s1.closest_param(p);
    let (u2, v2) = s2.closest_param(p);
    s1.normal(u1, v1).cross(s2.normal(u2, v2)).try_normalize()
}

fn inside(b: &Bounds, p: Vec3, pad: f64) -> bool {
    p.x >= b.lo.x - pad && p.y >= b.lo.y - pad && p.z >= b.lo.z - pad && p.x <= b.hi.x + pad && p.y <= b.hi.y + pad && p.z <= b.hi.z + pad
}

/// uv box of `s` covering the bounds (angular params: full period).
fn uv_box(s: &Surface, b: &Bounds) -> ((f64, f64), (f64, f64)) {
    let ((du0, du1), (dv0, dv1)) = s.domain();
    let (pu, pv) = s.periods();
    let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..27 {
        let f = |k: usize, a: f64, c: f64| a + (c - a) * 0.5 * k as f64;
        let p = Vec3::new(f(i % 3, b.lo.x, b.hi.x), f((i / 3) % 3, b.lo.y, b.hi.y), f(i / 9, b.lo.z, b.hi.z));
        let (u, v) = s.closest_param(p);
        u0 = u0.min(u);
        u1 = u1.max(u);
        v0 = v0.min(v);
        v1 = v1.max(v);
    }
    let pad_u = (u1 - u0) * 0.1 + 1e-9;
    let pad_v = (v1 - v0) * 0.1 + 1e-9;
    let (u0, u1) = if pu.is_some() { (du0, du1) } else { ((u0 - pad_u).max(du0), (u1 + pad_u).min(du1)) };
    let (v0, v1) = if pv.is_some() { (dv0, dv1) } else { ((v0 - pad_v).max(dv0), (v1 + pad_v).min(dv1)) };
    ((u0, u1), (v0, v1))
}

/// Marching SSI inside `b`. `step` = chord length of the march.
pub fn march(s1: &Surface, s2: &Surface, b: Bounds, tol: f64, step: f64) -> Vec<SsCurve> {
    let diag = (b.hi - b.lo).len().max(step);
    let pad = step * 2.0;
    // seeds from a uv grid on s1 (and on s2, for small features of s2)
    let mut seeds: Vec<Vec3> = vec![];
    for (sa, sb) in [(s1, s2), (s2, s1)] {
        let ((u0, u1), (v0, v1)) = uv_box(sa, &b);
        let n = 24;
        for i in 0..=n {
            for j in 0..=n {
                let p0 = sa.eval(u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
                if !inside(&b, p0, pad) {
                    continue;
                }
                // pull onto both surfaces (alternating projection, then Newton)
                let mut q = p0;
                for _ in 0..8 {
                    q = sa.closest_point(sb.closest_point(q).2).2;
                }
                if q.dist(p0) > diag / n as f64 * 2.0 {
                    continue;
                }
                let Some(t) = tangent(s1, s2, q) else { continue };
                if let Some(p) = correct(s1, s2, q, t, tol) {
                    if inside(&b, p, pad) {
                        seeds.push(p);
                    }
                }
            }
        }
    }
    let mut out: Vec<SsCurve> = vec![];
    let mut traced: Vec<Vec3> = vec![];
    for seed in seeds {
        if traced.iter().any(|q| q.dist(seed) < step * 1.5) {
            continue;
        }
        let Some(t0) = tangent(s1, s2, seed) else { continue };
        let (fwd, closed) = trace(s1, s2, seed, t0, &b, tol, step, pad);
        let pts = if closed {
            let mut p = fwd;
            p.push(seed);
            p
        } else {
            let (bwd, _) = trace(s1, s2, seed, -t0, &b, tol, step, pad);
            let mut p: Vec<Vec3> = bwd.into_iter().rev().collect();
            p.pop(); // seed duplicated
            p.extend(fwd);
            p
        };
        if pts.len() < 2 {
            continue;
        }
        traced.extend(pts.iter().copied());
        let Some((pts, bs)) = fit_refined(s1, s2, pts, tol) else { continue };
        out.push(SsCurve::Fitted { pts, curve: Curve::BSpline(bs), closed });
    }
    out
}

/// Interpolate the traced points with a cubic B-spline, then refine: the
/// spline mid-point of every data span is corrected onto the true
/// intersection and inserted while it deviates by more than tol/100 (so
/// downstream curve-curve trimming at `tol` sees the exact crossings).
fn fit_refined(
    s1: &Surface,
    s2: &Surface,
    mut pts: Vec<Vec3>,
    tol: f64,
) -> Option<(Vec<Vec3>, ck_geom::nurbs::bspline::BSplineCurve)> {
    for _round in 0..8 {
        let deg = 3.min(pts.len() - 1);
        let bs = interpolate(&pts, deg, Param::ChordLength).ok()?;
        let us = ck_geom::fit::params(&pts, Param::ChordLength);
        let mut next = vec![pts[0]];
        let mut refined = false;
        for i in 0..pts.len() - 1 {
            let um = 0.5 * (us[i] + us[i + 1]);
            let pm = bs.eval(um);
            let t = bs.derivs(um, 1)[1].normalize();
            if let Some(q) = correct(s1, s2, pm, t, tol) {
                if q.dist(pm) > tol * 1e-2 && pts.len() < 20000 {
                    next.push(q);
                    refined = true;
                }
            }
            next.push(pts[i + 1]);
        }
        if !refined {
            return Some((pts, bs));
        }
        pts = next;
    }
    let deg = 3.min(pts.len() - 1);
    interpolate(&pts, deg, Param::ChordLength).ok().map(|b| (pts, b))
}

#[allow(clippy::too_many_arguments)]
fn trace(s1: &Surface, s2: &Surface, seed: Vec3, t0: Vec3, b: &Bounds, tol: f64, step: f64, pad: f64) -> (Vec<Vec3>, bool) {
    let mut pts = vec![seed];
    let mut p = seed;
    let mut t = t0;
    let mut h = step;
    for it in 0..5000 {
        let pred = p + t * h;
        let next = correct(s1, s2, pred, t, tol);
        let ok = next.and_then(|q| tangent(s1, s2, q).map(|tq| (q, tq)));
        match ok {
            Some((q, mut tq)) if q.dist(p) < 2.0 * h && q.dist(p) > 0.1 * h => {
                if tq.dot(t) < 0.0 {
                    tq = -tq;
                }
                if tq.dot(t) < 0.966 && h > step / 64.0 {
                    h *= 0.5; // > 15° turn: refine
                    continue;
                }
                // closed loop?
                if it > 2 && q.dist(seed) < 0.75 * h.max(step * 0.5) {
                    return (pts, true);
                }
                pts.push(q);
                p = q;
                t = tq;
                if h < step {
                    h = (h * 1.5).min(step);
                }
                if !inside(b, q, pad) {
                    break;
                }
            }
            _ => {
                if h > step / 64.0 {
                    h *= 0.5;
                    continue;
                }
                break;
            }
        }
    }
    (pts, false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;
    use std::f64::consts::TAU;

    fn bounds(c: f64) -> Bounds {
        Bounds { lo: Vec3::new(-c, -c, -c), hi: Vec3::new(c, c, c) }
    }

    fn on_both(s1: &Surface, s2: &Surface, c: &Curve, r: (f64, f64), tol: f64) -> bool {
        (0..=20).all(|i| {
            let p = c.eval(r.0 + (r.1 - r.0) * i as f64 / 20.0);
            s1.distance(p) <= tol && s2.distance(p) <= tol
        })
    }

    #[test]
    fn plane_cases() {
        let p = Surface::plane(Vec3::new(0., 0., 1.), Vec3::new(0., 0.3, 1.));
        let q = Surface::plane(Vec3::ZERO, Vec3::X);
        let r = intersect(&p, &q, None, 1e-9);
        assert_eq!(r.curves().len(), 1);
        assert!(on_both(&p, &q, r.curves()[0].curve(), (-5., 5.), 1e-12));
        assert_eq!(intersect(&q, &q, None, 1e-9), SsResult::Coincident);
        let cyl = Surface::Cylinder { frame: Frame::identity(), r: 2.0 };
        for s in [&p, &q, &Surface::plane(Vec3::new(0., 0., 3.), Vec3::Z)] {
            let r = intersect(s, &cyl, None, 1e-9);
            assert!(!r.curves().is_empty());
            for c in r.curves() {
                let rng = if matches!(c.curve(), Curve::Line { .. }) { (-5., 5.) } else { (0., TAU) };
                assert!(on_both(s, &cyl, c.curve(), rng, 1e-9), "{c:?}");
            }
        }
        // plane x=0 vs cylinder: two lines at y=±2
        assert_eq!(intersect(&q, &cyl, None, 1e-9).curves().len(), 2);
        let sph = Surface::Sphere { frame: Frame::identity(), r: 2.0 };
        let r = intersect(&p, &sph, None, 1e-9);
        assert!(on_both(&p, &sph, r.curves()[0].curve(), (0., TAU), 1e-9));
        let cone = Surface::Cone { frame: Frame::identity(), r: 2.0, semi_angle: 0.4 };
        let r = intersect(&p, &cone, None, 1e-9);
        assert!(matches!(r.curves()[0].curve(), Curve::Ellipse { .. }), "{r:?}");
        assert!(on_both(&p, &cone, r.curves()[0].curve(), (0., TAU), 1e-9));
        let tor = Surface::Torus { frame: Frame::identity(), major: 5.0, minor: 1.0 };
        let r = intersect(&Surface::plane(Vec3::new(0., 0., 0.5), Vec3::Z), &tor, None, 1e-9);
        assert_eq!(r.curves().len(), 2);
        let r = intersect(&q, &tor, None, 1e-9);
        assert_eq!(r.curves().len(), 2);
        for c in r.curves() {
            assert!(on_both(&q, &tor, c.curve(), (0., TAU), 1e-9));
        }
    }

    #[test]
    fn quadric_pairs() {
        let a = Surface::Cylinder { frame: Frame::identity(), r: 2.0 };
        let b = Surface::Cylinder { frame: Frame { o: Vec3::new(3., 0., 0.), ..Frame::identity() }, r: 2.0 };
        let r = intersect(&a, &b, None, 1e-9);
        assert_eq!(r.curves().len(), 2);
        assert!(r.curves().iter().all(|c| on_both(&a, &b, c.curve(), (-3., 3.), 1e-12)));
        assert_eq!(intersect(&a, &a, None, 1e-9), SsResult::Coincident);
        let s = Surface::Sphere { frame: Frame { o: Vec3::new(0., 0., 1.), ..Frame::identity() }, r: 3.0 };
        let r = intersect(&a, &s, None, 1e-9);
        assert_eq!(r.curves().len(), 2);
        assert!(r.curves().iter().all(|c| on_both(&a, &s, c.curve(), (0., TAU), 1e-9)));
        let k = Surface::Cone { frame: Frame::identity(), r: 1.0, semi_angle: 0.5 };
        let r = intersect(&a, &k, None, 1e-9);
        assert_eq!(r.curves().len(), 1, "{r:?}"); // ρ > 0 nappe only
        assert!(r.curves().iter().all(|c| on_both(&a, &k, c.curve(), (0., TAU), 1e-9)));
        let s2 = Surface::Sphere { frame: Frame { o: Vec3::new(2., 1., 0.), ..Frame::identity() }, r: 2.5 };
        let r = intersect(&s, &s2, None, 1e-9);
        assert!(r.curves().iter().all(|c| on_both(&s, &s2, c.curve(), (0., TAU), 1e-9)));
    }

    #[test]
    fn marching_crossed_cylinders() {
        // two perpendicular cylinders r=2 and r=1: two closed loops
        let a = Surface::Cylinder { frame: Frame::identity(), r: 2.0 };
        let fb = frame_from(Vec3::ZERO, Vec3::X, Vec3::Y);
        let b = Surface::Cylinder { frame: fb, r: 1.0 };
        let r = intersect(&a, &b, Some(bounds(4.0)), 1e-9);
        let cs = r.curves();
        assert_eq!(cs.len(), 2, "{}", cs.len());
        for c in cs {
            let SsCurve::Fitted { pts, curve, closed } = c else { panic!() };
            assert!(*closed);
            for p in pts {
                assert!(a.distance(*p) < 1e-8 && b.distance(*p) < 1e-8);
            }
            // fitted curve stays close to both
            let (t0, t1) = curve.range();
            for i in 0..50 {
                let p = curve.eval(t0 + (t1 - t0) * i as f64 / 50.0);
                assert!(a.distance(p) < 1e-3 && b.distance(p) < 1e-3);
            }
        }
    }
}
