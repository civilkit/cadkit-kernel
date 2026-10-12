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
    // plane vs B-spline: the control net's hull on one side of the plane
    for (a, b) in [(s1, s2), (s2, s1)] {
        if let (Surface::Plane { frame }, Surface::BSpline(bs)) = (a, b) {
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for &c in &bs.ctrl {
                let d = (c - frame.o).dot(frame.z);
                lo = lo.min(d);
                hi = hi.max(d);
            }
            if lo > tol || hi < -tol {
                return SsResult::Disjoint;
            }
        }
    }
    if let Some(r) = screw_rev(s1, s2, tol).or_else(|| screw_rev(s2, s1, tol)) {
        return r;
    }
    if let Some(b) = bounds {
        if let Some(r) = long_spline_implicit(s1, s2, b, tol).or_else(|| long_spline_implicit(s2, s1, b, tol)) {
            return r;
        }
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
    let n = a.z.cross(b.z);
    if n.len() > 1e-12 {
        // equal radii, axes meeting at O (two equal fillets round a box
        // corner): p·a = ±p·b, i.e. the two bisector planes through O cut
        // the cylinder in an ellipse each
        let d = b.o - a.o;
        if (ra - rb).abs() > tol || d.dot(n).abs() > tol * n.len() {
            return None;
        }
        let s = d.cross(b.z).dot(n) / n.dot(n);
        let o = a.o + a.z * s;
        let mut out = vec![];
        for nn in [a.z + b.z, a.z - b.z] {
            if let SsResult::Curves(cs) = plane_cylinder(&frame_from(o, nn, any_perp(nn)), a, ra, tol) {
                out.extend(cs);
            }
        }
        return Some(if out.is_empty() { SsResult::Disjoint } else { SsResult::Curves(out) });
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


// ------------------------------------------------- screw spline x revolution

/// A B-spline surface whose radius and height about the axis of a coaxial
/// revolution surface (cylinder / cone / sphere / torus) are
/// `ρ(u, v) = ρ(v)`, `h(u, v) = h(v) + k (u - u0)` (a helical-sweep side
/// face: u = rotation angle) meets it in curves that follow from the
/// meridian alone: a cylinder gives iso-v lines, the other surfaces one
/// `u = φ(v)` curve per meridian branch. Replaces marching a 7-19 turn
/// spline (minutes). `None` = not such a pair (march instead).
fn screw_rev(sp: &Surface, rev: &Surface, tol: f64) -> Option<SsResult> {
    let Surface::BSpline(bs) = sp else { return None };
    let (o, z) = match rev {
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Sphere { frame, .. } | Surface::Torus { frame, .. } | Surface::Plane { frame } => (frame.o, frame.z),
        _ => return None,
    };
    // a plane normal to the axis is the meridian `h = 0` (the screw check
    // below rejects planes that are not normal to the spline's own axis)
    let m = match rev {
        Surface::Plane { .. } => Meridian::Cone(0.0, f64::INFINITY),
        _ => meridian(rev, o, z, tol)?,
    };
    let ((u0, u1), (v0, v1)) = bs.domain();
    if !(u1 - u0).is_finite() || u1 - u0 < 1e-9 || v1 - v0 < 1e-12 {
        return None;
    }
    let rh = |p: Vec3| {
        let d = p - o;
        let h = d.dot(z);
        ((d - z * h).len(), h)
    };
    let vm = 0.5 * (v0 + v1);
    let k = (rh(bs.eval(u1, vm)).1 - rh(bs.eval(u0, vm)).1) / (u1 - u0);
    if k.abs() < 1e-9 {
        return None;
    }
    let chk = 2e-6;
    for j in 0..=4 {
        let v = v0 + (v1 - v0) * (j as f64 + 0.13) / 4.26;
        let (r0, h0) = rh(bs.eval(u0, v));
        for i in 0..=24 {
            let u = u0 + (u1 - u0) * (i as f64 + 0.37) / 24.74;
            let (r, h) = rh(bs.eval(u, v));
            if (r - r0).abs() > chk || (h - h0 - k * (u - u0)).abs() > chk {
                return None;
            }
        }
    }
    let rho_h = |v: f64| rh(bs.eval(u0, v));
    let nsamp = 96usize;
    let vs: Vec<f64> = (0..=nsamp).map(|i| v0 + (v1 - v0) * i as f64 / nsamp as f64).collect();
    let mut out: Vec<SsCurve> = vec![];
    if let Meridian::Cyl(r) = m {
        // roots of ρ(v) = r
        let g = |v: f64| rho_h(v).0 - r;
        for w in vs.windows(2) {
            let (mut a, mut b) = (w[0], w[1]);
            let (ga, gb) = (g(a), g(b));
            if ga == 0.0 && a != v0 {
                continue; // reported by the previous window
            }
            if ga * gb > 0.0 || (ga == 0.0 && gb == 0.0) {
                continue;
            }
            for _ in 0..60 {
                let c = 0.5 * (a + b);
                if g(c) * ga > 0.0 {
                    a = c;
                } else {
                    b = c;
                }
            }
            let v = 0.5 * (a + b);
            let curve = bs.iso_v(v);
            let n = (bs.nu * 2).max(16);
            let (c0, c1) = (curve.knots[curve.degree], curve.knots[curve.knots.len() - 1 - curve.degree]);
            let pts: Vec<Vec3> = (0..=n).map(|i| curve.eval(c0 + (c1 - c0) * i as f64 / n as f64)).collect();
            out.push(SsCurve::Fitted { pts, curve: Curve::BSpline(curve), closed: false });
        }
        return Some(if out.is_empty() { SsResult::Disjoint } else { SsResult::Curves(out) });
    }
    // heights of branch `b` at radius ρ
    let (nb, hb): (usize, Box<dyn Fn(f64, usize) -> Option<f64>>) = match m {
        Meridian::Cyl(_) => unreachable!(),
        Meridian::Cone(_, kc) if kc.is_infinite() => (1, Box::new(|_, _| Some(0.0))),
        Meridian::Cone(r0, kc) => {
            if kc.abs() < 1e-12 {
                return None;
            }
            (1, Box::new(move |rho, _| Some((rho - r0) / kc)))
        }
        Meridian::Sphere(rs, hc) => (
            2,
            Box::new(move |rho, b| {
                let w2 = rs * rs - rho * rho;
                (w2 >= 0.0).then(|| hc + if b == 0 { -w2.sqrt() } else { w2.sqrt() })
            }),
        ),
        Meridian::Torus(big, small, hc) => (
            2,
            Box::new(move |rho, b| {
                let w2 = small * small - (rho - big) * (rho - big);
                (w2 >= 0.0).then(|| hc + if b == 0 { -w2.sqrt() } else { w2.sqrt() })
            }),
        ),
    };
    for b in 0..nb {
        // φ(v) on this branch, None where undefined or outside [u0, u1]
        let phi = |v: f64| -> Option<f64> {
            let (rho, h) = rho_h(v);
            let p = u0 + (hb(rho, b)? - h) / k;
            (p >= u0 - 1e-12 && p <= u1 + 1e-12).then(|| p.clamp(u0, u1))
        };
        let vals: Vec<Option<f64>> = vs.iter().map(|&v| phi(v)).collect();
        let mut i = 0;
        while i <= nsamp {
            if vals[i].is_none() {
                i += 1;
                continue;
            }
            let mut j = i;
            while j < nsamp && vals[j + 1].is_some() {
                j += 1;
            }
            // run i..=j; extend to the exact boundary on either side
            let mut vlist: Vec<f64> = vs[i..=j].to_vec();
            let edge = |valid: f64, invalid: f64| -> f64 {
                let (mut a, mut c) = (valid, invalid);
                for _ in 0..50 {
                    let mid = 0.5 * (a + c);
                    if phi(mid).is_some() {
                        a = mid;
                    } else {
                        c = mid;
                    }
                }
                a
            };
            if i > 0 {
                vlist.insert(0, edge(vs[i], vs[i - 1]));
            }
            if j < nsamp {
                vlist.push(edge(vs[j], vs[j + 1]));
            }
            let mut pts: Vec<Vec3> = vec![];
            for v in vlist {
                let p = bs.eval(phi(v)?, v);
                if pts.last().map_or(true, |q| q.dist(p) > 1e-9) {
                    pts.push(p);
                }
            }
            if pts.len() >= 2 {
                let c = interpolate(&pts, 3.min(pts.len() - 1), Param::ChordLength).ok()?;
                out.push(SsCurve::Fitted { pts, curve: Curve::BSpline(c), closed: false });
            }
            i = j + 1;
        }
    }
    Some(if out.is_empty() { SsResult::Disjoint } else { SsResult::Curves(out) })
}


// ------------------------------------------- long ruled spline x quadric

/// Implicit function of an analytic surface (zero set = the surface).
fn implicit(s: &Surface) -> Option<Box<dyn Fn(Vec3) -> f64>> {
    Some(match s.clone() {
        Surface::Plane { frame } => Box::new(move |p| (p - frame.o).dot(frame.z)),
        Surface::Cylinder { frame, r } => Box::new(move |p| {
            let d = p - frame.o;
            (d - frame.z * d.dot(frame.z)).len() - r
        }),
        Surface::Cone { frame, r, semi_angle } => {
            let k = semi_angle.tan();
            Box::new(move |p| {
                let d = p - frame.o;
                let h = d.dot(frame.z);
                (d - frame.z * h).len() - (r + k * h)
            })
        }
        Surface::Sphere { frame, r } => Box::new(move |p| (p - frame.o).len() - r),
        Surface::Torus { frame, major, minor } => Box::new(move |p| {
            let d = p - frame.o;
            let h = d.dot(frame.z);
            let rho = (d - frame.z * h).len();
            ((rho - major).powi(2) + h * h).sqrt() - minor
        }),
        _ => return None,
    })
}

/// A long (many u-spans) non-rational B-spline surface that is linear in v
/// (a screw / ruled sweep face) against an analytic surface: the lines
/// `v -> S(u, v)` meet the quadric in 0-2 points, tracked along u inside
/// `b` only. Replaces marching, whose closest-point corrector costs
/// O(spans) per step. `None` = not such a pair.
fn long_spline_implicit(sp: &Surface, other: &Surface, b: Bounds, tol: f64) -> Option<SsResult> {
    let Surface::BSpline(bs) = sp else { return None };
    if bs.weights.is_some() || bs.deg_v != 1 || bs.nv != 2 {
        return None;
    }
    let f = implicit(other)?;
    let spans: Vec<usize> = (bs.deg_u..bs.nu).filter(|&s| bs.knots_u[s] < bs.knots_u[s + 1]).collect();
    if spans.len() < 32 {
        return None;
    }
    let ((_, _), (v0, v1)) = bs.domain();
    let pad = tol.max(1e-9) * 10.0;
    // spans whose control-net box meets the bounds
    let hit: Vec<bool> = spans
        .iter()
        .map(|&s| {
            let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
            for i in s - bs.deg_u..=s {
                for j in 0..2 {
                    let c = bs.cp(i, j);
                    lo = lo.min(c);
                    hi = hi.max(c);
                }
            }
            lo.x <= b.hi.x + pad && hi.x >= b.lo.x - pad && lo.y <= b.hi.y + pad && hi.y >= b.lo.y - pad && lo.z <= b.hi.z + pad && hi.z >= b.lo.z - pad
        })
        .collect();
    let point = |u: f64, t: f64| bs.eval(u, v0 + (v1 - v0) * t);
    // roots t in [0, 1] of f(S(u, t))
    let roots_at = |u: f64| -> Vec<f64> {
        let nt = 8usize;
        let g = |t: f64| f(point(u, t));
        let ts: Vec<f64> = (0..=nt).map(|j| j as f64 / nt as f64).collect();
        let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
        let mut out = vec![];
        for j in 0..nt {
            if gs[j] == 0.0 {
                out.push(ts[j]);
            } else if gs[j] * gs[j + 1] < 0.0 {
                let (mut a, mut c) = (ts[j], ts[j + 1]);
                let ga = gs[j];
                for _ in 0..50 {
                    let m = 0.5 * (a + c);
                    if g(m) * ga > 0.0 {
                        a = m;
                    } else {
                        c = m;
                    }
                }
                out.push(0.5 * (a + c));
            }
        }
        if gs[nt] == 0.0 {
            out.push(1.0);
        }
        out
    };
    let per_span = 4usize;
    let mut out: Vec<SsCurve> = vec![];
    let mut i = 0;
    while i < spans.len() {
        if !hit[i] {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < spans.len() && hit[j + 1] {
            j += 1;
        }
        // run of spans i..=j (extended by one on each side)
        let (ia, ib) = (i.saturating_sub(1), (j + 1).min(spans.len() - 1));
        let mut tracks: Vec<Vec<(f64, f64)>> = vec![];
        let mut active: Vec<usize> = vec![];
        let mut finish = |tr: Vec<(f64, f64)>, out: &mut Vec<SsCurve>| {
            if tr.len() < 2 {
                return;
            }
            let mut pts: Vec<Vec3> = tr.iter().map(|&(u, t)| point(u, t)).collect();
            pts.dedup_by(|a, c| a.dist(*c) < 1e-9);
            if pts.len() < 2 {
                return;
            }
            if pts.len() > 400 {
                let n = pts.len();
                pts = (0..400).map(|k| pts[(k * (n - 1)) / 399]).collect();
            }
            if let Ok(c) = interpolate(&pts, 3.min(pts.len() - 1), Param::ChordLength) {
                out.push(SsCurve::Fitted { pts, curve: Curve::BSpline(c), closed: false });
            }
        };
        let mut prev_u: Option<f64> = None;
        for si in ia..=ib {
            let s = spans[si];
            let (ua, ub) = (bs.knots_u[s], bs.knots_u[s + 1]);
            for k in 0..=per_span {
                if si > ia && k == 0 {
                    continue;
                }
                let u = ua + (ub - ua) * k as f64 / per_span as f64;
                let rs = roots_at(u);
                // match roots to active tracks by nearest t
                let mut used = vec![false; rs.len()];
                let mut still: Vec<usize> = vec![];
                for &ti in &active {
                    let last = tracks[ti].last().unwrap().1;
                    let best = rs.iter().enumerate().filter(|(q, _)| !used[*q]).min_by(|a, c| (a.1 - last).abs().partial_cmp(&(c.1 - last).abs()).unwrap());
                    match best {
                        Some((q, &t)) if (t - last).abs() < 0.2 => {
                            used[q] = true;
                            tracks[ti].push((u, t));
                            still.push(ti);
                        }
                        _ => {
                            // the root left [0, 1] between prev_u and u: end exactly at the boundary
                            if let Some(pu) = prev_u {
                                let edge_t = if last < 0.5 { 0.0 } else { 1.0 };
                                let g = |uu: f64| f(point(uu, edge_t));
                                let (mut a, mut c) = (pu, u);
                                let ga = g(a);
                                if ga * g(c) < 0.0 {
                                    for _ in 0..50 {
                                        let m = 0.5 * (a + c);
                                        if g(m) * ga > 0.0 {
                                            a = m;
                                        } else {
                                            c = m;
                                        }
                                    }
                                    tracks[ti].push((0.5 * (a + c), edge_t));
                                }
                            }
                            let tr = std::mem::take(&mut tracks[ti]);
                            finish(tr, &mut out);
                        }
                    }
                }
                active = still;
                for (q, &t) in rs.iter().enumerate() {
                    if !used[q] {
                        let mut tr = vec![];
                        // started between prev_u and u: begin exactly at the boundary
                        if let Some(pu) = prev_u {
                            if t < 0.05 || t > 0.95 {
                                let edge_t = if t < 0.5 { 0.0 } else { 1.0 };
                                let g = |uu: f64| f(point(uu, edge_t));
                                let (mut a, mut c) = (pu, u);
                                let ga = g(a);
                                if ga * g(c) < 0.0 {
                                    for _ in 0..50 {
                                        let m = 0.5 * (a + c);
                                        if g(m) * ga > 0.0 {
                                            a = m;
                                        } else {
                                            c = m;
                                        }
                                    }
                                    tr.push((0.5 * (a + c), edge_t));
                                }
                            }
                        }
                        tr.push((u, t));
                        tracks.push(tr);
                        active.push(tracks.len() - 1);
                    }
                }
                prev_u = Some(u);
            }
        }
        for ti in active {
            let tr = std::mem::take(&mut tracks[ti]);
            finish(tr, &mut out);
        }
        i = j + 1;
    }
    Some(if out.is_empty() { SsResult::Disjoint } else { SsResult::Curves(out) })
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

// ---- parameter-space marcher (warm-started 4x4 Newton, cf. original f2503)

type Uv4 = [f64; 4];

fn wrap_uv(s: &Surface, u: f64, v: f64) -> (f64, f64) {
    let ((a, b), (c, d)) = s.domain();
    let (pu, pv) = s.periods();
    let f = |x: f64, lo: f64, hi: f64, per: Option<f64>| match per {
        Some(p) if lo.is_finite() => lo + (x - lo).rem_euclid(p),
        _ => x.clamp(lo, hi),
    };
    (f(u, a, b, pu), f(v, c, d, pv))
}

/// Gauss-Newton projection of `t` onto `s` from a warm start.
fn refine_uv(s: &Surface, mut u: f64, mut v: f64, t: Vec3) -> (f64, f64) {
    for _ in 0..3 {
        let d = s.derivs2(u, v);
        let r = t - d.p;
        let (a, b, c) = (d.du.dot(d.du), d.du.dot(d.dv), d.dv.dot(d.dv));
        let det = a * c - b * b;
        if !(det.abs() > 1e-30 * (a * c).abs().max(1e-300)) {
            break;
        }
        let (ru, rv) = (r.dot(d.du), r.dot(d.dv));
        let (du, dv) = ((ru * c - rv * b) / det, (a * rv - b * ru) / det);
        if !du.is_finite() || !dv.is_finite() {
            break;
        }
        (u, v) = wrap_uv(s, u + du, v + dv);
    }
    (u, v)
}

fn solve4(mut a: [[f64; 5]; 4]) -> Option<[f64; 4]> {
    for c in 0..4 {
        let mut piv = c;
        for r in c + 1..4 {
            if a[r][c].abs() > a[piv][c].abs() {
                piv = r;
            }
        }
        if !(a[piv][c].abs() > 1e-14) {
            return None;
        }
        a.swap(c, piv);
        for r in c + 1..4 {
            let f = a[r][c] / a[c][c];
            for k in c..5 {
                a[r][k] -= f * a[c][k];
            }
        }
    }
    let mut x = [0.0; 4];
    for r in (0..4).rev() {
        let mut v = a[r][4];
        for k in r + 1..4 {
            v -= a[r][k] * x[k];
        }
        x[r] = v / a[r][r];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Newton on (u1, v1, u2, v2): S1 = S2 and (S1 - pred)·dir = 0.
fn correct_uv(s1: &Surface, s2: &Surface, pred: Vec3, dir: Vec3, uv: Uv4, tol: f64) -> Option<(Vec3, Uv4)> {
    let (mut u1, mut v1) = refine_uv(s1, uv[0], uv[1], pred);
    let (mut u2, mut v2) = refine_uv(s2, uv[2], uv[3], pred);
    let eps = tol * 1e-4;
    for _ in 0..30 {
        let d1 = s1.derivs2(u1, v1);
        let d2 = s2.derivs2(u2, v2);
        let f = d1.p - d2.p;
        let g = (d1.p - pred).dot(dir);
        if f.x.abs() < eps && f.y.abs() < eps && f.z.abs() < eps && g.abs() < eps {
            return Some(((d1.p + d2.p) * 0.5, [u1, v1, u2, v2]));
        }
        let m = [
            [d1.du.x, d1.dv.x, -d2.du.x, -d2.dv.x, -f.x],
            [d1.du.y, d1.dv.y, -d2.du.y, -d2.dv.y, -f.y],
            [d1.du.z, d1.dv.z, -d2.du.z, -d2.dv.z, -f.z],
            [d1.du.dot(dir), d1.dv.dot(dir), 0.0, 0.0, -g],
        ];
        let x = solve4(m)?;
        (u1, v1) = wrap_uv(s1, u1 + x[0], v1 + x[1]);
        (u2, v2) = wrap_uv(s2, u2 + x[2], v2 + x[3]);
    }
    None
}

fn tangent_uv(s1: &Surface, s2: &Surface, uv: Uv4) -> Option<Vec3> {
    s1.normal(uv[0], uv[1]).cross(s2.normal(uv[2], uv[3])).try_normalize()
}

#[derive(Clone, Copy)]
struct MPt {
    p: Vec3,
    uv: Uv4,
}

/// Marching SSI inside `b`. `step` = chord length of the march.
pub fn march(s1: &Surface, s2: &Surface, b: Bounds, tol: f64, step: f64) -> Vec<SsCurve> {
    let diag = (b.hi - b.lo).len().max(step);
    let pad = step * 2.0;
    // seeds from a uv grid on s1 (and on s2, for small features of s2)
    let mut seeds: Vec<MPt> = vec![];
    for (swap, (sa, sb)) in [(false, (s1, s2)), (true, (s2, s1))] {
        let ((u0, u1), (v0, v1)) = uv_box(sa, &b);
        let n = 24;
        let thr = diag / n as f64 * 2.0;
        for i in 0..=n {
            for j in 0..=n {
                let (ua, va) = (u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
                let p0 = sa.eval(ua, va);
                if !inside(&b, p0, pad) {
                    continue;
                }
                let (ub, vb, q1) = sb.closest_point(p0);
                if q1.dist(p0) > thr {
                    continue;
                }
                let uv = if swap { [ub, vb, ua, va] } else { [ua, va, ub, vb] };
                let Some(t) = tangent_uv(s1, s2, uv) else { continue };
                let pred = (p0 + q1) * 0.5;
                if let Some((p, uv)) = correct_uv(s1, s2, pred, t, uv, tol) {
                    if inside(&b, p, pad) && p.dist(p0) <= thr {
                        seeds.push(MPt { p, uv });
                    }
                }
            }
        }
    }
    let mut out: Vec<SsCurve> = vec![];
    // (point, exclusion radius): a seed this close to an already traced point
    // lies on the same curve; the radius follows the local sample spacing, not
    // the nominal step (loops smaller than the step must not shadow each other)
    let mut traced: Vec<(Vec3, f64)> = vec![];
    for seed in seeds {
        if traced.iter().any(|(q, r)| q.dist(seed.p) < *r) {
            continue;
        }
        let Some(t0) = tangent_uv(s1, s2, seed.uv) else { continue };
        let (fwd, closed) = trace(s1, s2, seed, t0, &b, tol, step, pad);
        let pts = if closed {
            let mut p = fwd;
            p.push(seed);
            p
        } else {
            let (bwd, _) = trace(s1, s2, seed, -t0, &b, tol, step, pad);
            let mut p: Vec<MPt> = bwd.into_iter().rev().collect();
            p.pop(); // seed duplicated
            p.extend(fwd);
            p
        };
        if pts.len() < 2 {
            continue;
        }
        let n = pts.len();
        for i in 0..n {
            let gap = |j: usize| pts[i].p.dist(pts[j].p);
            let sp = if i > 0 { gap(i - 1) } else { 0.0 }.max(if i + 1 < n { gap(i + 1) } else { 0.0 });
            traced.push((pts[i].p, (1.5 * sp).min(step * 1.5).max(tol * 10.0)));
        }
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
    mut mp: Vec<MPt>,
    tol: f64,
) -> Option<(Vec<Vec3>, ck_geom::nurbs::bspline::BSplineCurve)> {
    for _round in 0..8 {
        let pts: Vec<Vec3> = mp.iter().map(|m| m.p).collect();
        let deg = 3.min(pts.len() - 1);
        let bs = interpolate(&pts, deg, Param::ChordLength).ok()?;
        let us = ck_geom::fit::params(&pts, Param::ChordLength);
        let mut next = vec![mp[0]];
        let mut refined = false;
        for i in 0..pts.len() - 1 {
            let um = 0.5 * (us[i] + us[i + 1]);
            let pm = bs.eval(um);
            let t = bs.derivs(um, 1)[1].normalize();
            // warm start: parameters of the nearer data point
            let w = if pm.dist(pts[i]) <= pm.dist(pts[i + 1]) { mp[i].uv } else { mp[i + 1].uv };
            if let Some((q, uv)) = correct_uv(s1, s2, pm, t, w, tol) {
                if q.dist(pm) > tol * 1e-2 && pts.len() < 20000 {
                    next.push(MPt { p: q, uv });
                    refined = true;
                }
            }
            next.push(mp[i + 1]);
        }
        if !refined {
            return Some((pts, bs));
        }
        mp = next;
    }
    let pts: Vec<Vec3> = mp.iter().map(|m| m.p).collect();
    let deg = 3.min(pts.len() - 1);
    interpolate(&pts, deg, Param::ChordLength).ok().map(|b| (pts, b))
}

#[allow(clippy::too_many_arguments)]
fn trace(s1: &Surface, s2: &Surface, seed: MPt, t0: Vec3, b: &Bounds, tol: f64, step: f64, pad: f64) -> (Vec<MPt>, bool) {
    let mut pts = vec![seed];
    let mut cur = seed;
    let mut t = t0;
    let mut h = step;
    for it in 0..5000 {
        let pred = cur.p + t * h;
        let next = correct_uv(s1, s2, pred, t, cur.uv, tol);
        let ok = next.and_then(|(q, uv)| tangent_uv(s1, s2, uv).map(|tq| (q, uv, tq)));
        match ok {
            Some((q, uv, mut tq)) if q.dist(cur.p) < 2.0 * h && q.dist(cur.p) > 0.1 * h => {
                if tq.dot(t) < 0.0 {
                    tq = -tq;
                }
                if tq.dot(t) < 0.966 && h > step / 64.0 {
                    h *= 0.5; // > 15° turn: refine
                    continue;
                }
                // closed loop?
                if it > 2 && pts.len() >= 3 && q.dist(seed.p) < 0.75 * h {
                    return (pts, true);
                }
                cur = MPt { p: q, uv };
                pts.push(cur);
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
