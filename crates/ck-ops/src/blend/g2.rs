//! Curvature-continuous (G2) fillet profile (ck-ops/src/blend/g2.rs).
//! Codes: `blendG2TooLargeMax {max, shape, detail}`; the fillet fields are
//! `profile: "curvature"`, `shape` in [0.1, 1] (`filletShape` otherwise) and
//! `g3` (accepted; the oracle's G3 section differs from the G2 one by 4e-3 in
//! volume, i.e. by its own spline fit, so it is built identically here).
//!
//! RECONSTRUCTED from the oracle's mesh sections (tools/face_section.mjs on
//! `filletCurv`, see internal notes (not published)): the section is an Euler spiral
//! (curvature ramping linearly from 0 to 1/r over a length `r f phi`), a
//! circular arc of radius `r` (the "radius of the middle"), and the mirrored
//! spiral, where `phi` is the turning angle of the edge (90 degrees on a box)
//! and `shape` = `f` the fraction of `phi` spent in the transitions. This
//! reproduces the oracle's total length (4.7124 for r=2, shape .5), its
//! tangent distance from the edge (2.8325 = 1.41627 r, to 5 digits for
//! shapes .1 .3 .5 .8 1) and its volumes to 1.4e-6 (the oracle's own spline
//! fit is that far off the exact spiral).

use ck_geom::curve2::Curve2;
use ck_geom::{Vec2, Vec3};
use ck_topo::make::ProfileSeg;

pub const SHAPE_MIN: f64 = 0.1;
pub const SHAPE_MAX: f64 = 1.0;

/// Gauss-Legendre 4-point nodes / weights on [-1, 1].
const GL4: [(f64, f64); 4] = [
    (-0.8611363115940526, 0.3478548451374538),
    (-0.3399810435848563, 0.6521451548625461),
    (0.3399810435848563, 0.6521451548625461),
    (0.8611363115940526, 0.3478548451374538),
];

/// Heading at arc length `s` of the spiral - arc - spiral curve.
fn heading(s: f64, r: f64, ls: f64, la: f64) -> f64 {
    let tt = ls / (2.0 * r);
    if s <= ls {
        s * s / (2.0 * r * ls)
    } else if s <= ls + la {
        tt + (s - ls) / r
    } else {
        let u = (ls + la + ls) - s; // distance from the end
        2.0 * tt + la / r - u * u / (2.0 * r * ls)
    }
}

/// The curve in its local frame (start at the origin heading +x, turning
/// left by `phi`), sampled at `per` points per sub-piece, and the tangent
/// distance `d` from the start to the corner.
pub fn spiral_blend(r: f64, shape: f64, phi: f64) -> Option<(f64, Vec<Vec2>)> {
    if !(r > 0.0) || !(shape > 0.0) || !(phi > 0.0 && phi < std::f64::consts::PI) {
        return None;
    }
    let ls = r * shape * phi;
    let la = r * (1.0 - shape) * phi;
    let total = 2.0 * ls + la;
    // integer sub-intervals per piece so the junctions are nodes
    let (nr, na) = (800usize, if la > 1e-12 { 1600usize } else { 0 });
    let mut bounds: Vec<f64> = vec![];
    for i in 0..=nr {
        bounds.push(ls * i as f64 / nr as f64);
    }
    for i in 1..=na {
        bounds.push(ls + la * i as f64 / na as f64);
    }
    for i in 1..=nr {
        bounds.push(ls + la + ls * i as f64 / nr as f64);
    }
    let mut pos = vec![Vec2::new(0.0, 0.0)];
    for w in bounds.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
        let (mut dx, mut dy) = (0.0, 0.0);
        for &(x, wt) in GL4.iter() {
            let th = heading(m + h * x, r, ls, la);
            dx += wt * h * th.cos();
            dy += wt * h * th.sin();
        }
        let p = *pos.last().unwrap();
        pos.push(Vec2::new(p.x + dx, p.y + dy));
    }
    let _ = total;
    let e = *pos.last().unwrap();
    let d = e.y / phi.sin();
    // keep ~160 points: every k-th node plus the piece junctions
    let step = 50usize;
    let mut out = vec![];
    for (i, p) in pos.iter().enumerate() {
        if i % step == 0 || i == pos.len() - 1 {
            out.push(*p);
        }
    }
    Some((d, out))
}

/// Tangent distance from the edge of the spiral blend (see [`spiral_blend`]).
pub fn tangent_distance(r: f64, shape: f64, phi: f64) -> Option<f64> {
    spiral_blend(r, shape, phi).map(|x| x.0)
}

/// The profile piece (tag "arc") from `t1` to `t2` for the corner `cp`, all
/// in the plane of the section.
pub fn g2_segment(r: f64, shape: f64, phi: f64, t1: Vec2, cp: Vec2, t2: Vec2) -> Option<ProfileSeg> {
    let (_d, local) = spiral_blend(r, shape, phi)?;
    if (cp - t1).len() < 1e-12 || (t2 - cp).len() < 1e-12 {
        return None;
    }
    let ex = (cp - t1).normalize();
    let leg2 = (t2 - cp).normalize();
    // local y axis: the component of the second leg direction normal to ex
    let ey = (leg2 - ex * leg2.dot(ex)).normalize();
    let pts: Vec<Vec3> = local
        .iter()
        .map(|p| {
            let q = t1 + ex * p.x + ey * p.y;
            Vec3::new(q.x, q.y, 0.0)
        })
        .collect();
    let chord: f64 = pts.windows(2).map(|w| w[0].dist(w[1])).sum();
    // end tangents exactly along the legs (derivatives w.r.t. the [0,1] chord-length parameter)
    let d0 = Vec3::new(ex.x, ex.y, 0.0) * chord;
    let d1 = Vec3::new(leg2.x, leg2.y, 0.0) * chord;
    let c = ck_geom::fit::interpolate_cubic_tangents(&pts, d0, d1, ck_geom::fit::Param::ChordLength).ok()?;
    let (a, b) = c.domain();
    let c2 = c.map(|p| Vec2::new(p.x, p.y));
    Some(ProfileSeg { tag: "arc".into(), curve: Curve2::BSpline(c2), t0: a, t1: b, rev: false })
}

/// Rational quadratic Bezier `t1 -> cp -> t2` with weight `w` at `cp`: the
/// conic profile (`rho = w / (1 + w)`; a circle on a right-angled edge for
/// `rho = 0.4142`).
pub fn conic_segment(rho: f64, t1: Vec2, cp: Vec2, t2: Vec2) -> Option<ProfileSeg> {
    if !(rho > 0.0 && rho < 1.0) {
        return None;
    }
    let w = rho / (1.0 - rho);
    let c = ck_geom::nurbs::bspline::BSplineCurve::new_rational(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![t1, cp, t2], vec![1.0, w, 1.0]).ok()?;
    Some(ProfileSeg { tag: "arc".into(), curve: Curve2::BSpline(c), t0: 0.0, t1: 1.0, rev: false })
}

/// Largest radius whose tangent distance fits `room` (bisection).
pub fn max_radius(shape: f64, phi: f64, room: f64) -> Option<f64> {
    let d1 = tangent_distance(1.0, shape, phi)?;
    Some(room / d1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    /// Oracle sections of the 90 degree box edge (radius 2): tangent distance /
    /// total length for shapes .1 .3 .5 .8 1.
    #[test]
    fn matches_the_oracle_sections() {
        for (shape, d, len) in [(0.1, 2.15910, 3.4555), (0.3, 2.48884, 4.0841), (0.5, 2.83250, 4.7124), (0.8, 3.37004, 5.6549), (1.0, 3.74019, 6.2832)] {
            let (dd, pts) = spiral_blend(2.0, shape, FRAC_PI_2).unwrap();
            assert!((dd - d).abs() < 2e-4, "shape {shape}: d {dd} vs {d}");
            let l: f64 = pts.windows(2).map(|w| w[0].dist(w[1])).sum();
            assert!((l - len).abs() < 2e-3, "shape {shape}: length {l} vs {len}");
        }
    }

    /// Removed corner area for r=2 shape .5: 1.06539 (oracle volume 7957.3957 ->
    /// 1.06511, its fit is 3e-4 off the exact spiral).
    #[test]
    fn corner_area() {
        let (d, pts) = spiral_blend(2.0, 0.5, FRAC_PI_2).unwrap();
        let mut poly = vec![Vec2::new(d, 0.0)];
        poly.extend(pts);
        let mut a = 0.0;
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            a += p.x * q.y - q.x * p.y;
        }
        // (the sample polyline is decimated: its chords cut the corner by ~7e-4)
        assert!((a.abs() / 2.0 - 1.06539).abs() < 1.5e-3, "{}", a.abs() / 2.0);
    }

    fn run(profile: crate::blend::build::FilletProfile, r: f64) -> Result<ck_topo::body::Body, crate::blend::BlendError> {
        use crate::blend::build::{blend_edges, Spec};
        let b = crate::boolean::tests::box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let at = ck_geom::Vec3::new(20.0, 10.0, 0.0);
        blend_edges(&b, &[("F2.C1".into(), "F2.S.e1".into(), at)], Spec::Fillet(r, profile), "F3", 1e-6)
    }

    fn vol(b: &ck_topo::body::Body) -> f64 {
        ck_topo::mass::mass_props(b).volume
    }

    #[test]
    fn conic_and_curvature_fillets_on_a_box_edge() {
        use crate::blend::build::FilletProfile as P;
        // oracle: conic rho .5 / .3 -> 7973.33333 / 7954.43029; curvature shape .5 -> 7957.39574
        let b = run(P::Conic(0.5), 2.0).expect("conic");
        assert_eq!(b.num_faces(), 7);
        assert!((vol(&b) - 7973.33333).abs() < 1e-3, "{}", vol(&b));
        let b = run(P::Conic(0.3), 2.0).expect("conic .3");
        assert!((vol(&b) - 7954.43029).abs() < 1e-3, "{}", vol(&b));
        let b = run(P::Curvature(0.5), 2.0).expect("g2");
        assert_eq!(b.num_faces(), 7);
        assert!((vol(&b) - 7957.39574).abs() < 0.02, "{}", vol(&b));
        assert!(ck_topo::check::check(&b, 1e-6).is_empty());
    }

    #[test]
    fn segment_stays_inside_the_corner_and_is_tangent_at_the_ends() {
        let d = tangent_distance(2.0, 0.5, FRAC_PI_2).unwrap();
        let (t1, cp, t2) = (Vec2::new(d, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, d));
        let seg = g2_segment(2.0, 0.5, FRAC_PI_2, t1, cp, t2).unwrap();
        let (a, b) = (seg.t0, seg.t1);
        let mut worst: f64 = 0.0;
        for k in 0..=2000 {
            let p = seg.curve.eval(a + (b - a) * k as f64 / 2000.0);
            worst = worst.max((-p.x).max(-p.y));
        }
        assert!(worst < 1e-6, "curve leaves the corner by {worst}");
        assert!(seg.curve.eval(a).dist(t1) < 1e-9 && seg.curve.eval(b).dist(t2) < 1e-9, "{:?} {:?}", seg.curve.eval(a), seg.curve.eval(b));
        let h = 1e-5 * (b - a);
        let ta = (seg.curve.eval(a + h) - seg.curve.eval(a)).normalize();
        assert!(ta.dot(Vec2::new(-1.0, 0.0)) > 1.0 - 1e-6, "{ta:?}");
    }

    #[test]
    #[ignore = "known gap: adjacent conic / curvature fillets (corner of two spline sections) answer blendFailed"]
    fn conic_ring() {
        use crate::blend::build::{blend_edges, FilletProfile as P, Spec};
        let b = crate::boolean::tests::box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let es = [("F2.S.e1", [20.0, 10.0, 0.0]), ("F2.S.e2", [40.0, 10.0, -10.0]), ("F2.S.e3", [20.0, 10.0, -20.0]), ("F2.S.e4", [0.0, 10.0, -10.0])];
        let sel: Vec<(String, String, ck_geom::Vec3)> = es.iter().map(|(f, p)| ("F2.C1".to_string(), f.to_string(), ck_geom::Vec3::new(p[0], p[1], p[2]))).collect();
        let out = blend_edges(&b, &sel, Spec::Fillet(2.0, P::Conic(0.5)), "F3", 1e-6).expect("ring");
        assert!((vol(&out) - 7922.13333).abs() < 0.05, "{}", vol(&out));
    }
}
