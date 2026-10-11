//! Differential analysis: curve curvature/torsion/Frenet frames and surface
//! principal/gaussian/mean curvature.
//!
//! Evidence: kernel.strings key run `singular n k1 k2 d1 d2 gaussian mean
//! notFound` = the `curvatureAt` reply record. Oracle probes (flange part,
//! `{"type":"curvatureAt","tag":"main","points":[{face,p}]}`):
//! - bore cylinder R=25 (face F4.S.e1@F1.1) at [1.5,0,25]:
//!   n=[0,0,-1] (towards the axis), k1=0, k2=-0.04, d1=[1,0,0],
//!   d2=[0,-1,0], gaussian=0, mean=-0.02
//! - fillet torus (F10.B0@F1.1) at [15,-20,0]: n=[0,-1,0], k1=0.05,
//!   k2=-1/3, d1=[0,0,-1], d2=[1,0,0]
//! - plane: k1=k2=0, d1/d2 still orthonormal tangents.
//!
//! Hence: curvature is positive where the surface bends AWAY from the
//! normal (convex w.r.t. n): k = −II(d)/I(d) with II built from n;
//! k1 >= k2; d2 = n × d1; mean = (k1+k2)/2; gaussian = k1·k2.

use crate::surface::SurfDerivs;
use crate::{Vec2, Vec3};

/// Curvature of a 3D curve from C', C'': |C'×C''| / |C'|³.
pub fn curvature(d1: Vec3, d2: Vec3) -> f64 {
    let s = d1.len();
    if s < 1e-300 {
        return 0.0;
    }
    d1.cross(d2).len() / (s * s * s)
}

/// Curvature vector κ·N (points to the centre of curvature).
pub fn curvature_vector(d1: Vec3, d2: Vec3) -> Vec3 {
    let s2 = d1.len2();
    if s2 < 1e-300 {
        return Vec3::ZERO;
    }
    (d2 - d1 * (d1.dot(d2) / s2)) / s2
}

/// Torsion (C'×C'')·C''' / |C'×C''|².
pub fn torsion(d1: Vec3, d2: Vec3, d3: Vec3) -> f64 {
    let c = d1.cross(d2);
    let l2 = c.len2();
    if l2 < 1e-300 {
        return 0.0;
    }
    c.dot(d3) / l2
}

/// Signed curvature of a 2D curve (positive = turning left / CCW).
pub fn curvature2(d1: Vec2, d2: Vec2) -> f64 {
    let s = d1.len();
    if s < 1e-300 {
        return 0.0;
    }
    d1.cross(d2) / (s * s * s)
}

/// Frenet–Serret frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrenetFrame {
    pub t: Vec3,
    pub n: Vec3,
    pub b: Vec3,
    pub curvature: f64,
    pub torsion: f64,
}

/// Frenet frame from `[C, C', C'', C''']` (at least C'..C''). At
/// inflections (κ = 0) `n`/`b` are completed with an arbitrary perpendicular.
pub fn frenet(d: &[Vec3]) -> FrenetFrame {
    let (d1, d2) = (d[1], d[2]);
    let d3 = d.get(3).copied().unwrap_or(Vec3::ZERO);
    let t = d1.normalize();
    let kv = curvature_vector(d1, d2);
    let n = if kv.len() > 1e-14 { kv.normalize() } else { crate::any_perp(t) };
    let b = t.cross(n);
    FrenetFrame { t, n, b, curvature: curvature(d1, d2), torsion: torsion(d1, d2, d3) }
}

/// Rotation-minimising frame propagation (double reflection, Wang et al.
/// 2008): given points/tangents along a curve and a start normal, returns
/// the propagated normals.
pub fn rmf(points: &[Vec3], tangents: &[Vec3], n0: Vec3) -> Vec<Vec3> {
    let mut out = Vec::with_capacity(points.len());
    if points.is_empty() {
        return out;
    }
    let t0 = tangents[0].normalize();
    let mut r = (n0 - t0 * n0.dot(t0)).normalize();
    out.push(r);
    for i in 0..points.len() - 1 {
        let v1 = points[i + 1] - points[i];
        let c1 = v1.len2();
        if c1 < 1e-300 {
            out.push(r);
            continue;
        }
        let ti = tangents[i].normalize();
        let rl = r - v1 * (2.0 / c1 * v1.dot(r));
        let tl = ti - v1 * (2.0 / c1 * v1.dot(ti));
        let t1 = tangents[i + 1].normalize();
        let v2 = t1 - tl;
        let c2 = v2.len2();
        r = if c2 < 1e-300 { rl } else { rl - v2 * (2.0 / c2 * v2.dot(rl)) };
        out.push(r.normalize());
    }
    out
}

/// The `curvatureAt` record (minus `p`, which the caller supplies).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfCurvature {
    pub n: Vec3,
    pub k1: f64,
    pub k2: f64,
    pub d1: Vec3,
    pub d2: Vec3,
    pub gaussian: f64,
    pub mean: f64,
    /// Degenerate tangent plane (Su × Sv ≈ 0): pole, apex, collapsed edge.
    pub singular: bool,
}

/// Fundamental forms (E,F,G) and (L,M,N) w.r.t. unit normal `n`.
pub fn fundamental_forms(s: &SurfDerivs, n: Vec3) -> ([f64; 3], [f64; 3]) {
    (
        [s.du.dot(s.du), s.du.dot(s.dv), s.dv.dot(s.dv)],
        [s.duu.dot(n), s.duv.dot(n), s.dvv.dot(n)],
    )
}

/// Normal curvature along tangent direction `dir` (oracle sign convention).
pub fn normal_curvature(s: &SurfDerivs, n: Vec3, du: f64, dv: f64) -> f64 {
    let ([e, f, g], [l, m, nn]) = fundamental_forms(s, n);
    let i = e * du * du + 2.0 * f * du * dv + g * dv * dv;
    if i.abs() < 1e-300 {
        return 0.0;
    }
    -(l * du * du + 2.0 * m * du * dv + nn * dv * dv) / i
}

/// Principal curvatures at a point. `flip` reverses the natural normal
/// Su×Sv (face sense reversed); curvature signs follow the used normal.
pub fn surface_curvature(s: &SurfDerivs, flip: bool) -> SurfCurvature {
    let cr = s.du.cross(s.dv);
    let scale = s.du.len().max(s.dv.len()).powi(2);
    let singular = scale == 0.0 || cr.len() <= 1e-12 * scale;
    let mut n = cr.normalize();
    if flip {
        n = -n;
    }
    if singular {
        return SurfCurvature { n, k1: 0.0, k2: 0.0, d1: Vec3::ZERO, d2: Vec3::ZERO, gaussian: 0.0, mean: 0.0, singular };
    }
    let ([e, f, g], [l, m, nn]) = fundamental_forms(s, n);
    // oracle convention: II' = -II
    let (l, m, nn) = (-l, -m, -nn);
    let det = e * g - f * f;
    // shape operator S = I⁻¹·II' (2x2, not symmetric in general)
    let a = (g * l - f * m) / det;
    let b = (g * m - f * nn) / det;
    let c = (e * m - f * l) / det;
    let d = (e * nn - f * m) / det;
    let mean = (a + d) / 2.0;
    let disc = (((a - d) / 2.0).powi(2) + b * c).max(0.0).sqrt();
    let (k1, k2) = (mean + disc, mean - disc);
    let d1 = if disc <= 1e-12 * (1.0 + mean.abs()) {
        // umbilic: any direction is principal. GUESS: use Su.
        s.du.normalize()
    } else {
        // eigenvector of S for k1 in (du, dv)
        // sign pinned by oracle curvatureAt on concave cylinders / tori
        // (k1 on the v direction): d1 = -dv there, i.e. (-b, a - k1)
        let (x1, y1) = (-b, a - k1);
        let (x2, y2) = (k1 - d, c);
        let (du, dv) = if x1 * x1 + y1 * y1 >= x2 * x2 + y2 * y2 { (x1, y1) } else { (x2, y2) };
        (s.du * du + s.dv * dv).normalize()
    };
    let d2 = n.cross(d1);
    SurfCurvature { n, k1, k2, d1, d2, gaussian: k1 * k2, mean, singular }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::Curve;
    use crate::surface::Surface;
    use crate::testutil::*;
    use crate::{frame_from, Frame};

    #[test]
    fn helix_curvature_torsion() {
        // helix (r cos t, r sin t, c t): κ = r/(r²+c²), τ = c/(r²+c²)
        let (r, c, t) = (2.0f64, 0.5f64, 0.7f64);
        let d1 = Vec3::new(-r * t.sin(), r * t.cos(), c);
        let d2 = Vec3::new(-r * t.cos(), -r * t.sin(), 0.0);
        let d3 = Vec3::new(r * t.sin(), -r * t.cos(), 0.0);
        let f = frenet(&[Vec3::ZERO, d1, d2, d3]);
        assert!(close(f.curvature, r / (r * r + c * c), 1e-14));
        assert!(close(f.torsion, c / (r * r + c * c), 1e-14));
        assert!(close(f.t.cross(f.n).dot(f.b), 1.0, 1e-14));
    }

    #[test]
    fn circle_curvature_and_2d() {
        let c = Curve::circle(Vec3::ZERO, Vec3::Z, 4.0);
        let d = c.derivs(1.0, 2);
        assert!(close(curvature(d[1], d[2]), 0.25, 1e-14));
        assert!(vclose(curvature_vector(d[1], d[2]), -d[0] / 16.0, 1e-14));
        assert!(close(curvature2(Vec2::new(0., 1.), Vec2::new(-1., 0.)), 1.0, 1e-15));
    }

    #[test]
    fn oracle_bore_cylinder() {
        // bore R=25 about the x axis; face normal points to the axis.
        let s = Surface::Cylinder { frame: frame_from(Vec3::ZERO, Vec3::X, Vec3::Z), r: 25.0 };
        let (u, v) = s.closest_param(Vec3::new(1.5, 0., 25.));
        let k = surface_curvature(&s.derivs2(u, v), true);
        assert!(vclose(k.n, Vec3::new(0., 0., -1.), 1e-14));
        assert!(close(k.k1, 0.0, 1e-14) && close(k.k2, -0.04, 1e-14));
        assert!(close(k.mean, -0.02, 1e-14) && close(k.gaussian, 0.0, 1e-14));
        assert!(close(k.d1.x.abs(), 1.0, 1e-14));
        assert!(vclose(k.d2, k.n.cross(k.d1), 1e-14));
    }

    #[test]
    fn oracle_fillet_torus() {
        // axis x through [15,0,0], R=23, r=3; point [15,-20,0] on the inner
        // equator; face normal [0,-1,0] = reversed natural normal.
        let fr = Frame { o: Vec3::new(15., 0., 0.), x: Vec3::Y, y: Vec3::Z, z: Vec3::X };
        let s = Surface::Torus { frame: fr, major: 23.0, minor: 3.0 };
        let (u, v) = s.closest_param(Vec3::new(15., -20., 0.));
        assert!(vclose(s.eval(u, v), Vec3::new(15., -20., 0.), 1e-12));
        let k = surface_curvature(&s.derivs2(u, v), true);
        assert!(vclose(k.n, Vec3::new(0., -1., 0.), 1e-12));
        assert!(close(k.k1, 0.05, 1e-12), "{}", k.k1);
        assert!(close(k.k2, -1.0 / 3.0, 1e-12), "{}", k.k2);
        assert!(close(k.d1.z.abs(), 1.0, 1e-12));
        assert!(close(k.gaussian, -0.016666666666666663, 1e-12));
        assert!(close(k.mean, -0.14166666666666666, 1e-12));
    }

    #[test]
    fn sphere_and_plane_umbilic_and_pole() {
        let s = Surface::Sphere { frame: Frame::identity(), r: 2.0 };
        let k = surface_curvature(&s.derivs2(0.3, 0.4), false);
        assert!(close(k.k1, 0.5, 1e-12) && close(k.k2, 0.5, 1e-12) && !k.singular);
        let pole = surface_curvature(&s.derivs2(0.3, std::f64::consts::FRAC_PI_2), false);
        assert!(pole.singular);
        let p = Surface::Plane { frame: Frame::identity() };
        let k = surface_curvature(&p.derivs2(1.0, 2.0), false);
        assert_eq!((k.k1, k.k2), (0.0, 0.0));
        assert!(vclose(k.d2, Vec3::Y, 1e-15));
        // normal curvature of a cylinder along its circumference
        let c = Surface::Cylinder { frame: Frame::identity(), r: 5.0 };
        let sd = c.derivs2(0.0, 0.0);
        assert!(close(normal_curvature(&sd, sd.normal(), 1.0, 0.0), 0.2, 1e-14));
    }

    #[test]
    fn rmf_on_circle_keeps_binormal() {
        let c = Curve::circle(Vec3::ZERO, Vec3::Z, 1.0);
        let n = 64;
        let ts: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64 * 3.0).collect();
        let pts: Vec<Vec3> = ts.iter().map(|&t| c.eval(t)).collect();
        let tans: Vec<Vec3> = ts.iter().map(|&t| c.tangent(t)).collect();
        let rs = rmf(&pts, &tans, Vec3::Z);
        for r in rs {
            assert!(vclose(r, Vec3::Z, 1e-12));
        }
    }
}
