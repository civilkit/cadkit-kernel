//! Analytic recognition for B-spline geometry read from STEP.
//!
//! Exporters write straight edges, circles and flat faces as B-spline entities (a straight edge as a degree-1 or
//! degree-3 spline, a circle as a rational quadratic, a loft wall or a planar patch as a B-spline surface). Kept as
//! splines they send fillet, chamfer, offset, shell and move-face down their free-form paths (or refuse them), so the
//! reader turns them back into the line, circle or plane they are:
//!
//! * a curve whose control points are collinear and run monotonically along the line becomes a [`Curve::Line`];
//! * a curve whose samples all lie on one circle in one plane, running one way for at most one turn, becomes a
//!   [`Curve::Circle`] with the sense of travel as the circle's counter-clockwise direction;
//! * a surface one of whose parameter directions runs round a circle and stays on that circle's cylinder along the other
//!   becomes a [`Surface::Cylinder`] (the face sense flips when the spline's normal points inward);
//! * a surface that is circles about one axis one way and circles through that axis's planes the other, on one ring, becomes a
//!   [`Surface::Torus`] (a fillet round a boss or a rim), flipped the same way;
//! * a surface whose control net is coplanar (so the surface is, by the convex-hull property) and whose normal never
//!   flips becomes a [`Surface::Plane`] with the spline's own normal as the plane's, so a face's `sense` and its loops
//!   keep their meaning (the reader derives pcurves from the edges and the surface afterwards, so no (u, v) data of
//!   the spline survives to be converted).
//!
//! Tolerance: `TOL_REL` (1e-8) of the size of the geometry, with a floor of 1 mm for the scale (so sub-millimetre parts
//! are judged at 1e-8 mm). Everything is all-or-nothing: a curve that misses the tolerance anywhere, rational with a
//! non-positive weight, degenerate or non-monotone stays a spline.
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Vec3};
use std::f64::consts::TAU;

/// Relative tolerance of every recognition.
pub const TOL_REL: f64 = 1e-8;

fn positive(w: &Option<Vec<f64>>) -> bool {
    w.as_ref().map_or(true, |w| w.iter().all(|&x| x.is_finite() && x > 0.0))
}

fn extent(pts: &[Vec3]) -> f64 {
    let mut lo = pts[0];
    let mut hi = pts[0];
    for p in pts {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (hi - lo).len()
}

/// The line a spline is, when its control points are collinear and monotone along it.
pub fn spline_as_line(bs: &BSplineCurve) -> Option<Curve> {
    let n = bs.ctrl.len();
    if n < 2 || !positive(&bs.weights) || bs.ctrl.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return None;
    }
    let a = bs.ctrl[0];
    let b = bs.ctrl[n - 1];
    let len = a.dist(b);
    let size = extent(&bs.ctrl);
    let tol = TOL_REL * size.max(1.0);
    // zero length, or out and back (a closed spline): not one straight edge
    if len < 1e-3 * size || len < 10.0 * tol {
        return None;
    }
    let d = (b - a) / len;
    let mut prev = 0.0f64;
    for &p in &bs.ctrl {
        let v = p - a;
        let t = v.dot(d);
        if (v - d * t).len() > tol || t < prev - tol || t > len + tol {
            return None;
        }
        prev = prev.max(t);
    }
    Some(Curve::Line { o: a, d })
}

/// The circle (or arc) a spline lies on, when all its samples are within tolerance of one circle and it runs one way
/// for at most one turn. The circle's normal is the direction of travel (counter-clockwise about it).
pub fn spline_as_circle(bs: &BSplineCurve) -> Option<Curve> {
    if bs.ctrl.len() < 3 || !positive(&bs.weights) {
        return None;
    }
    let size = extent(&bs.ctrl);
    let tol = TOL_REL * size.max(1.0);
    if !(size > 10.0 * tol) {
        return None;
    }
    let (t0, t1) = bs.domain();
    if !(t1 > t0) {
        return None;
    }
    let at = |i: usize, n: usize| bs.eval(t0 + (t1 - t0) * (i as f64 / n as f64));
    // the circle through three samples a third of the domain apart (the end of a full circle is its start, so the
    // thirds keep the three points distinct)
    let (a, b, c) = (at(0, 3), at(1, 3), at(2, 3));
    let (ab, ac) = (b - a, c - a);
    let nrm = ab.cross(ac);
    let nl = nrm.len();
    if nl < 1e-6 * ab.len() * ac.len() || !nl.is_finite() {
        return None;
    }
    let n = nrm / nl;
    let (d1, d2) = (ab.len2(), ac.len2());
    let o = a + (nrm.cross(ab) * d2 + ac.cross(nrm) * d1) / (2.0 * nl * nl);
    let r = a.dist(o);
    // an arc of huge radius is a straight edge that failed the line test: leave it to the spline
    if !(r > 10.0 * tol) || r > 1e4 * size {
        return None;
    }
    let f = frame_from(o, n, a - o);
    let spans = bs.knots.windows(2).filter(|w| w[1] > w[0]).count().max(1);
    let m = (16 * spans).clamp(64, 4096);
    let mut prev = 0.0f64;
    let mut sweep = 0.0f64;
    for i in 0..=m {
        let p = at(i, m);
        let v = p - o;
        if (v.len() - r).abs() > tol || v.dot(n).abs() > tol {
            return None;
        }
        let ang = v.dot(f.y).atan2(v.dot(f.x));
        // unwrap against the previous angle: the step must be forward (counter-clockwise about the normal)
        let mut step = ang - prev;
        if i > 0 {
            while step < -std::f64::consts::PI {
                step += TAU;
            }
            while step > std::f64::consts::PI {
                step -= TAU;
            }
            // a step backwards by more than the tolerance's worth of angle
            if step < -2.0 * tol / r {
                return None;
            }
            sweep += step.max(0.0);
        }
        prev = ang;
    }
    if sweep > TAU + 1e-6 || sweep < 1e-6 {
        return None;
    }
    Some(Curve::Circle { frame: f, r })
}

/// A spline as line or circle (`None` when it is neither).
pub fn analytic_curve(bs: &BSplineCurve) -> Option<Curve> {
    spline_as_line(bs).or_else(|| spline_as_circle(bs))
}

/// The plane a B-spline surface is, when its control net is coplanar and its normal keeps one side. The plane's normal
/// is the surface's own `Su x Sv`.
pub fn surface_as_plane(s: &BSplineSurface) -> Option<Surface> {
    if s.ctrl.len() < 4 || !positive(&s.weights) || s.ctrl.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return None;
    }
    let size = extent(&s.ctrl);
    let tol = TOL_REL * size.max(1.0);
    if !(size > 10.0 * tol) {
        return None;
    }
    // the plane through the best-spread triple of control points: first, farthest from it, then the one making the
    // largest triangle
    let p0 = s.ctrl[0];
    let mut p1 = p0;
    for &p in &s.ctrl {
        if p.dist(p0) > p1.dist(p0) {
            p1 = p;
        }
    }
    let mut best = Vec3::new(0.0, 0.0, 0.0);
    for &p in &s.ctrl {
        let c = (p1 - p0).cross(p - p0);
        if c.len() > best.len() {
            best = c;
        }
    }
    // a collinear net (a degenerate surface) has no plane
    if best.len() < 1e-6 * size * size {
        return None;
    }
    let mut nrm = best.normalize();
    if s.ctrl.iter().any(|&p| (p - p0).dot(nrm).abs() > tol) {
        return None;
    }
    // orientation: the spline's own normal must not change side anywhere (a fold) and gives the plane's sense
    let ((u0, u1), (v0, v1)) = s.domain();
    let k = 8usize;
    let mut side = 0.0f64;
    for i in 0..=k {
        for j in 0..=k {
            let (u, v) = (u0 + (u1 - u0) * i as f64 / k as f64, v0 + (v1 - v0) * j as f64 / k as f64);
            let q = s.normal(u, v);
            if q.len() < 0.5 {
                continue; // a pole with no usable normal
            }
            let d = q.dot(nrm);
            if d.abs() < 0.5 {
                return None;
            }
            if side == 0.0 {
                side = d.signum();
            } else if d.signum() != side {
                return None;
            }
        }
    }
    if side == 0.0 {
        return None;
    }
    if side < 0.0 {
        nrm = -nrm;
    }
    let xref = s.ctrl[(s.nu - 1) * s.nv] - s.ctrl[0];
    let xref = if xref.len() > 10.0 * tol { xref } else { p1 - p0 };
    Some(Surface::Plane { frame: frame_from(p0, nrm, xref) })
}

/// The circular cylinder a B-spline surface is (a lofted or extruded circle written as a rational surface): one
/// parameter direction runs round a circle, the surface stays on the cylinder of that circle along the other. Returns
/// the surface and whether its own normal `Su x Sv` points inward (the face's sense must then be flipped, since the
/// cylinder's natural normal is outward).
pub fn surface_as_cylinder(s: &BSplineSurface) -> Option<(Surface, bool)> {
    if s.ctrl.len() < 4 || !positive(&s.weights) {
        return None;
    }
    let sf = cylinder_of(s).or_else(|| cylinder_of(&s.transposed()))?;
    orient(s, sf)
}

/// Circle along u at v0, straight along v: the cylinder (axis from the v direction), judged on `s` as given.
fn cylinder_of(s: &BSplineSurface) -> Option<Surface> {
    let size = extent(&s.ctrl);
    let tol = TOL_REL * size.max(1.0);
    let ((u0, u1), (v0, v1)) = s.domain();
    let Some(Curve::Circle { frame, r }) = spline_as_circle(&s.iso_v(v0)) else { return None };
    let um = 0.5 * (u0 + u1);
    let axis = s.eval(um, v1) - s.eval(um, v0);
    if axis.len() < 10.0 * tol {
        return None;
    }
    let axis = axis.normalize();
    // the axis is normal to the circle's plane (an oblique extrusion is an elliptic cylinder)
    if axis.cross(frame.z).len() > 1e-7 {
        return None;
    }
    let (ku, kv) = (24usize, 8usize);
    for i in 0..=ku {
        for j in 0..=kv {
            let (u, v) = (u0 + (u1 - u0) * i as f64 / ku as f64, v0 + (v1 - v0) * j as f64 / kv as f64);
            let w = s.eval(u, v) - frame.o;
            let radial = w - axis * w.dot(axis);
            if (radial.len() - r).abs() > tol {
                return None;
            }
        }
    }
    Some(Surface::Cylinder { frame: frame_from(frame.o, axis, frame.x), r })
}

/// The outward direction of a cylinder or torus at the point `p` of it (`None` for another surface or a degenerate point).
fn outward(sf: &Surface, p: Vec3) -> Option<Vec3> {
    let (frame, major) = match sf {
        Surface::Cylinder { frame, .. } => (frame, 0.0),
        Surface::Torus { frame, major, .. } => (frame, *major),
        _ => return None,
    };
    let w = p - frame.o;
    let radial = w - frame.z * w.dot(frame.z);
    if radial.len() < 1e-12 {
        return None;
    }
    // cylinder: straight out from the axis; torus: out from the ring
    let d = if major == 0.0 { radial } else { p - (frame.o + radial.normalize() * major) };
    if d.len() < 1e-12 {
        return None;
    }
    Some(d.normalize())
}

/// Set the face sense flip of a recognised cylinder or torus from `s`'s own normal (it must point one way everywhere).
fn orient(s: &BSplineSurface, sf: Surface) -> Option<(Surface, bool)> {
    let ((u0, u1), (v0, v1)) = s.domain();
    let k = 8usize;
    let mut side = 0.0f64;
    for i in 0..=k {
        for j in 0..=k {
            let (u, v) = (u0 + (u1 - u0) * i as f64 / k as f64, v0 + (v1 - v0) * j as f64 / k as f64);
            let q = s.normal(u, v);
            if q.len() < 0.5 {
                continue;
            }
            let Some(out) = outward(&sf, s.eval(u, v)) else { continue };
            let d = q.dot(out);
            if d.abs() < 0.5 {
                return None;
            }
            if side == 0.0 {
                side = d.signum();
            } else if d.signum() != side {
                return None;
            }
        }
    }
    if side == 0.0 {
        return None;
    }
    Some((sf, side < 0.0))
}

/// The torus a B-spline surface is (a fillet round a boss or a rim, written as a rational surface): one parameter direction
/// runs round circles about one axis (the parallels), the other round circles whose planes hold the axis and whose centres
/// lie on one circle about it (the meridians), and every point is at the minor radius from that ring. Returns the surface and
/// whether the spline's own normal points into the tube (the face sense flips).
pub fn surface_as_torus(s: &BSplineSurface) -> Option<(Surface, bool)> {
    if s.ctrl.len() < 9 || !positive(&s.weights) {
        return None;
    }
    let sf = torus_of(s).or_else(|| torus_of(&s.transposed()))?;
    orient(s, sf)
}

fn torus_of(s: &BSplineSurface) -> Option<Surface> {
    let size = extent(&s.ctrl);
    let tol = TOL_REL * size.max(1.0);
    let ((u0, u1), (v0, v1)) = s.domain();
    let Some(Curve::Circle { frame: fa, .. }) = spline_as_circle(&s.iso_v(v0)) else { return None };
    let Some(Curve::Circle { frame: fb, r }) = spline_as_circle(&s.iso_u(0.5 * (u0 + u1))) else { return None };
    let n = fa.z;
    let wb = fb.o - fa.o;
    let h0 = wb.dot(n);
    let perp = wb - n * h0;
    let big = perp.len();
    // the meridian circle's plane holds the axis; the ring is a proper one (radius above the tube's)
    if fb.z.dot(n).abs() > 1e-7 || big < 10.0 * tol || big <= r * (1.0 + 1e-6) || fb.z.dot(perp.normalize()).abs() > 1e-7 {
        return None;
    }
    let o = fa.o + n * h0;
    let (ku, kv) = (16usize, 16usize);
    for i in 0..=ku {
        for j in 0..=kv {
            let (u, v) = (u0 + (u1 - u0) * i as f64 / ku as f64, v0 + (v1 - v0) * j as f64 / kv as f64);
            let w = s.eval(u, v) - o;
            let h = w.dot(n);
            let rho = (w - n * h).len();
            if ((rho - big).hypot(h) - r).abs() > tol {
                return None;
            }
        }
    }
    Some(Surface::Torus { frame: frame_from(o, n, perp), major: big, minor: r })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    /// The nine-point rational quadratic of a full circle of radius `r` about the z axis at height `z`.
    fn full_circle(r: f64, z: f64) -> BSplineCurve {
        let w = FRAC_1_SQRT_2;
        let ctrl = vec![v(r, 0.0, z), v(r, r, z), v(0.0, r, z), v(-r, r, z), v(-r, 0.0, z), v(-r, -r, z), v(0.0, -r, z), v(r, -r, z), v(r, 0.0, z)];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        BSplineCurve::new_rational(2, knots, ctrl, weights).unwrap()
    }

    fn quarter(r: f64) -> BSplineCurve {
        BSplineCurve::new_rational(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(r, 0.0, 0.0), v(r, r, 0.0), v(0.0, r, 0.0)], vec![1.0, FRAC_1_SQRT_2, 1.0]).unwrap()
    }

    #[test]
    fn collinear_cubic_is_a_line() {
        let bs = BSplineCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], vec![v(1.0, 2.0, 3.0), v(2.0, 3.0, 4.0), v(4.0, 5.0, 6.0), v(11.0, 12.0, 13.0)]).unwrap();
        let Some(Curve::Line { o, d }) = spline_as_line(&bs) else { panic!("not a line") };
        assert!(o.dist(v(1.0, 2.0, 3.0)) < 1e-12);
        assert!((d - v(1.0, 1.0, 1.0).normalize()).len() < 1e-12);
    }

    #[test]
    fn slightly_curved_spline_stays() {
        // 1e-5 of bow on a 10 mm edge: far outside 1e-8
        let bs = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(5.0, 1e-5, 0.0), v(10.0, 0.0, 0.0)]).unwrap();
        assert!(spline_as_line(&bs).is_none());
        // 1e-10 of bow is rounding: a line
        let bs = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(5.0, 1e-10, 0.0), v(10.0, 0.0, 0.0)]).unwrap();
        assert!(spline_as_line(&bs).is_some());
    }

    #[test]
    fn degenerate_and_doubling_back_splines_stay() {
        let zero = BSplineCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v(1.0, 1.0, 1.0), v(1.0, 1.0, 1.0)]).unwrap();
        assert!(spline_as_line(&zero).is_none());
        // out and back along one line: not monotone
        let back = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(4.0, 0.0, 0.0)]).unwrap();
        assert!(spline_as_line(&back).is_none());
        let closed = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(0.0, 0.0, 0.0)]).unwrap();
        assert!(spline_as_line(&closed).is_none());
        // zero-length circle candidate
        let tiny = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(1.0, 1.0, 1.0); 3]).unwrap();
        assert!(analytic_curve(&tiny).is_none());
    }

    #[test]
    fn rational_full_circle_is_a_circle() {
        let Some(Curve::Circle { frame, r }) = spline_as_circle(&full_circle(7.5, 2.0)) else { panic!("not a circle") };
        assert!((r - 7.5).abs() < 1e-9);
        assert!(frame.o.dist(v(0.0, 0.0, 2.0)) < 1e-9);
        assert!(frame.z.dist(v(0.0, 0.0, 1.0)) < 1e-9, "counter-clockwise spline: normal +z, got {:?}", frame.z);
    }

    #[test]
    fn rational_arc_keeps_its_sense() {
        let Some(Curve::Circle { frame, r }) = spline_as_circle(&quarter(3.0)) else { panic!("not a circle") };
        assert!((r - 3.0).abs() < 1e-9 && frame.z.dist(v(0.0, 0.0, 1.0)) < 1e-9);
        // the same arc run the other way: normal -z, and the curve point at the start is the same
        let rev = quarter(3.0).reversed();
        let Some(Curve::Circle { frame: fr, .. }) = spline_as_circle(&rev) else { panic!("not a circle") };
        assert!(fr.z.dist(v(0.0, 0.0, -1.0)) < 1e-9);
        let c = Curve::Circle { frame: fr, r: 3.0 };
        let t = c.closest_param(v(0.0, 3.0, 0.0));
        let t2 = c.closest_param(v(3.0, 0.0, 0.0));
        let mut dt = t2 - t;
        while dt < 0.0 {
            dt += TAU;
        }
        assert!((dt - FRAC_PI_2).abs() < 1e-9, "forward quarter turn from the arc start to its end, got {dt}");
    }

    #[test]
    fn non_circles_stay() {
        // an ellipse as a rational spline (same net, x scaled)
        let mut e = full_circle(5.0, 0.0);
        for p in &mut e.ctrl {
            p.x *= 1.2;
        }
        assert!(spline_as_circle(&e).is_none());
        // a circle with 1e-6 of out-of-plane wobble
        let mut w = full_circle(5.0, 0.0);
        w.ctrl[2].z += 1e-6;
        assert!(spline_as_circle(&w).is_none());
        // a parabola
        let par = BSplineCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(5.0, 5.0, 0.0), v(10.0, 0.0, 0.0)]).unwrap();
        assert!(spline_as_circle(&par).is_none());
        // a line is not a circle
        let line = BSplineCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)]).unwrap();
        assert!(spline_as_circle(&line).is_none());
    }

    fn bilinear(h: [f64; 4]) -> BSplineSurface {
        BSplineSurface::bilinear(v(0.0, 0.0, h[0]), v(10.0, 0.0, h[1]), v(0.0, 6.0, h[2]), v(10.0, 6.0, h[3]))
    }

    #[test]
    fn coplanar_net_is_a_plane_with_the_surface_normal() {
        let Some(Surface::Plane { frame }) = surface_as_plane(&bilinear([1.0; 4])) else { panic!("not a plane") };
        assert!(frame.o.dist(v(0.0, 0.0, 1.0)) < 1e-12);
        let s = bilinear([1.0; 4]);
        assert!(frame.z.dot(s.normal(0.5, 0.5)) > 1.0 - 1e-12);
        // a tilted plane
        let Some(Surface::Plane { frame }) = surface_as_plane(&bilinear([0.0, 5.0, 3.0, 8.0])) else { panic!("not a plane") };
        let s = bilinear([0.0, 5.0, 3.0, 8.0]);
        assert!(frame.z.dot(s.normal(0.3, 0.7)) > 1.0 - 1e-12);
        // the opposite winding gives the opposite normal
        let t = s.transposed();
        let Some(Surface::Plane { frame: ft }) = surface_as_plane(&t) else { panic!("not a plane") };
        assert!(ft.z.dot(frame.z) < -1.0 + 1e-12);
    }

    #[test]
    fn warped_or_folded_nets_stay() {
        assert!(surface_as_plane(&bilinear([0.0, 0.0, 0.0, 1e-5])).is_none());
        assert!(surface_as_plane(&bilinear([0.0, 0.0, 0.0, 1e-10])).is_some());
        // a net folded over itself (both corners at the same place): the normal flips
        let fold = BSplineSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            3,
            2,
            vec![v(0.0, 0.0, 0.0), v(0.0, 4.0, 0.0), v(10.0, 0.0, 0.0), v(10.0, 4.0, 0.0), v(-5.0, 0.0, 0.0), v(-5.0, 4.0, 0.0)],
            None,
        )
        .unwrap();
        assert!(surface_as_plane(&fold).is_none());
        // a collinear net has no plane
        let col = BSplineSurface::bilinear(v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(2.0, 0.0, 0.0), v(3.0, 0.0, 0.0));
        assert!(surface_as_plane(&col).is_none());
    }

    /// A cylinder of radius `r` and height `h` as a rational surface, u round the circle, v along the axis.
    fn cyl_surface(r: f64, h: f64) -> BSplineSurface {
        let w = FRAC_1_SQRT_2;
        let c = full_circle(r, 0.0);
        let mut ctrl = vec![];
        let mut ws = vec![];
        for (p, &wi) in c.ctrl.iter().zip(c.weights.as_ref().unwrap()) {
            ctrl.push(*p);
            ctrl.push(*p + v(0.0, 0.0, h));
            ws.push(wi);
            ws.push(wi);
        }
        let _ = w;
        BSplineSurface::new(2, 1, c.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], 9, 2, ctrl, Some(ws)).unwrap()
    }

    #[test]
    fn rational_cylinder_surface_is_a_cylinder() {
        let Some((Surface::Cylinder { frame, r }, flip)) = surface_as_cylinder(&cyl_surface(4.0, 9.0)) else { panic!("not a cylinder") };
        assert!((r - 4.0).abs() < 1e-9 && frame.z.dist(v(0.0, 0.0, 1.0)) < 1e-9 && frame.o.dist(v(0.0, 0.0, 0.0)) < 1e-9);
        // u runs counter-clockwise, v up: Su x Sv points outward
        assert!(!flip);
        // transposed: the other parameter order, normal inward
        let Some((Surface::Cylinder { r, .. }, flip)) = surface_as_cylinder(&cyl_surface(4.0, 9.0).transposed()) else { panic!("not a cylinder") };
        assert!((r - 4.0).abs() < 1e-9 && flip);
    }

    #[test]
    fn cones_and_barrels_are_not_cylinders() {
        let mut s = cyl_surface(4.0, 9.0);
        for i in 0..9 {
            s.ctrl[2 * i + 1] = s.ctrl[2 * i + 1] * 0.8 + v(0.0, 0.0, 1.8);
        }
        assert!(surface_as_cylinder(&s).is_none(), "a cone is not a cylinder");
        let mut e = cyl_surface(4.0, 9.0);
        for p in &mut e.ctrl {
            p.x *= 1.1;
        }
        assert!(surface_as_cylinder(&e).is_none(), "an elliptic cylinder is not a circular one");
    }

    /// A torus (ring radius `big`, tube radius `r`) as a 9 x 9 rational net: u round the axis, v round the tube.
    fn torus_surface(big: f64, r: f64) -> BSplineSurface {
        let c = full_circle(1.0, 0.0);
        let wu = c.weights.clone().unwrap();
        let mut ctrl = vec![];
        let mut ws = vec![];
        for (i, pu) in c.ctrl.iter().enumerate() {
            for (j, pv) in c.ctrl.iter().enumerate() {
                let rho = big + r * pv.x;
                ctrl.push(v(rho * pu.x, rho * pu.y, r * pv.y));
                ws.push(wu[i] * wu[j]);
            }
        }
        BSplineSurface::new(2, 2, c.knots.clone(), c.knots.clone(), 9, 9, ctrl, Some(ws)).unwrap()
    }

    #[test]
    fn rational_torus_surface_is_a_torus() {
        let Some((Surface::Torus { frame, major, minor }, flip)) = surface_as_torus(&torus_surface(13.0, 2.0)) else { panic!("not a torus") };
        assert!((major - 13.0).abs() < 1e-9 && (minor - 2.0).abs() < 1e-9 && frame.z.dot(v(0.0, 0.0, 1.0)).abs() > 1.0 - 1e-12 && frame.o.len() < 1e-9);
        assert!(!flip);
        let Some((Surface::Torus { major, minor, .. }, flip)) = surface_as_torus(&torus_surface(13.0, 2.0).transposed()) else { panic!("not a torus") };
        assert!((major - 13.0).abs() < 1e-9 && (minor - 2.0).abs() < 1e-9 && flip);
    }

    #[test]
    fn non_tori_stay() {
        // a tube of elliptic section
        let mut e = torus_surface(13.0, 2.0);
        for p in &mut e.ctrl {
            p.z *= 1.2;
        }
        assert!(surface_as_torus(&e).is_none());
        // a cylinder is not a torus (and a torus is not a cylinder)
        assert!(surface_as_torus(&cyl_surface(4.0, 9.0)).is_none());
        assert!(surface_as_cylinder(&torus_surface(13.0, 2.0)).is_none());
        // a spindle (tube radius above the ring radius)
        assert!(surface_as_torus(&torus_surface(1.5, 2.0)).is_none());
    }
}
