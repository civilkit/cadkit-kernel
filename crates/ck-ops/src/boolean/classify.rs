//! Classification (ck-ops/src/boolean/classify.rs): point-in-face (uv
//! polygon test, periodic-aware), fragment sample points, and fragment
//! vs. solid classification (In / Out / On-same / On-opposite).
//!
//! Solid containment uses ck-topo's generalised winding number
//! (`region::winding_number`); "on" is decided first by an explicit
//! point-on-face test so coplanar overlaps are resolved by normals.

use ck_geom::curve::Curve;
use ck_geom::pcurve::unwrap_near;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};
use ck_topo::region::winding_number;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Where {
    In,
    On,
    Out,
}

/// Fragment (or face) relative to the other solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    In,
    Out,
    /// On the other solid's boundary, outward normals agree.
    OnSame,
    /// On the other solid's boundary, outward normals opposite.
    OnOpp,
}

/// Samples per curve piece for uv polygons.
pub fn samples_for(c: &Curve) -> usize {
    match c {
        Curve::Line { .. } => 1,
        Curve::Circle { .. } | Curve::Ellipse { .. } => 24,
        Curve::BSpline(b) => (b.ctrl.len() * 4).clamp(8, 128),
        Curve::Conic { .. } | Curve::Sinusoid { .. } => 32,
    }
}

/// uv of a 3D point, unwrapped near `prev`.
pub fn uv_near(s: &Surface, p: Vec3, prev: Option<Vec2>) -> Vec2 {
    let (mut u, mut v) = s.closest_param(p);
    if let Some(q) = prev {
        let (pu, pv) = s.periods();
        if let Some(per) = pu {
            u = unwrap_near(u, q.x, per);
        }
        if let Some(per) = pv {
            v = unwrap_near(v, q.y, per);
        }
    }
    Vec2::new(u, v)
}

/// uv polygon of a loop given as directed curve pieces `(curve, ta, tb)`.
pub fn loop_polygon(s: &Surface, pieces: &[(&Curve, f64, f64)]) -> Vec<Vec2> {
    let mut out = vec![];
    let mut prev: Option<Vec2> = None;
    for &(c, ta, tb) in pieces {
        let n = samples_for(c);
        for k in 0..n {
            let uv = uv_near(s, c.eval(ta + (tb - ta) * k as f64 / n as f64), prev);
            prev = Some(uv);
            out.push(uv);
        }
    }
    out
}

/// Shoelace area (positive = counter-clockwise in uv).
pub fn poly_area(p: &[Vec2]) -> f64 {
    let n = p.len();
    let mut a = 0.0;
    for i in 0..n {
        let (x, y) = (p[i], p[(i + 1) % n]);
        a += x.x * y.y - y.x * x.y;
    }
    0.5 * a
}

fn crossings(p: Vec2, poly: &[Vec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, c) = (poly[i], poly[(i + 1) % n]);
        if (a.y > p.y) != (c.y > p.y) && p.x < a.x + (p.y - a.y) * (c.x - a.x) / (c.y - a.y) {
            inside = !inside;
        }
    }
    inside
}

/// Point-in-polygon with periodic shifts (true if inside for some shift).
pub fn in_polygon(s: &Surface, uv: Vec2, poly: &[Vec2]) -> bool {
    let (pu, pv) = s.periods();
    let su: Vec<f64> = match pu {
        Some(p) => vec![0.0, -p, p, -2.0 * p, 2.0 * p],
        None => vec![0.0],
    };
    let sv: Vec<f64> = match pv {
        Some(p) => vec![0.0, -p, p, -2.0 * p, 2.0 * p],
        None => vec![0.0],
    };
    su.iter().any(|&a| sv.iter().any(|&b| crossings(Vec2::new(uv.x + a, uv.y + b), poly)))
}

/// Even-odd over loops (each loop tested with periodic shifts).
pub fn in_loops(s: &Surface, uv: Vec2, loops: &[Vec<Vec2>]) -> bool {
    loops.iter().filter(|l| in_polygon(s, uv, l)).count() % 2 == 1
}

/// Directed pieces of the loops of a body face.
pub fn face_loops_uv(b: &Body, f: FaceId) -> Vec<Vec<Vec2>> {
    let face = b.face(f);
    face.loops
        .iter()
        .map(|&l| {
            let pieces: Vec<(&Curve, f64, f64)> = b
                .lp(l)
                .coedges
                .iter()
                .map(|&c| {
                    let (ta, tb) = b.coedge_params(c);
                    (&b.edge(b.coedge(c).edge).curve, ta, tb)
                })
                .collect();
            loop_polygon(&face.surface, &pieces)
        })
        .collect()
}

/// Is `p` (assumed near the face's surface) inside / on the boundary of /
/// outside face `f`?
pub fn point_in_face(b: &Body, f: FaceId, p: Vec3, tol: f64) -> Where {
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        if super::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= tol {
            return Where::On;
        }
    }
    let face = b.face(f);
    if face.surface.distance(p) > tol * 10.0 {
        return Where::Out;
    }
    let loops = face_loops_uv(b, f);
    if loops.iter().all(|l| l.len() < 3) {
        return Where::In; // closed surface without boundary (full sphere/torus)
    }
    let (u, v) = face.surface.closest_param(p);
    if in_loops(&face.surface, Vec2::new(u, v), &loops) {
        Where::In
    } else {
        Where::Out
    }
}

/// Classify point `q` (on a fragment with outward normal `n`) against
/// solid `other`.
pub fn classify_point(other: &Body, q: Vec3, n: Vec3, tol: f64) -> Class {
    for g in other.face_ids() {
        let s = &other.face(g).surface;
        let (u, v, cp) = s.closest_point(q);
        if cp.dist(q) > tol * 10.0 {
            continue;
        }
        if point_in_face(other, g, q, tol) == Where::Out {
            continue;
        }
        let ng = other.outward_normal(g, u, v);
        return if ng.dot(n) > 0.0 { Class::OnSame } else { Class::OnOpp };
    }
    let faces: Vec<FaceId> = other.face_ids().collect();
    if faces.is_empty() {
        return Class::Out;
    }
    if winding_number(other, &faces, q) > 0.5 {
        Class::In
    } else {
        Class::Out
    }
}

/// Outward normal of a (surface, sense) pair at the closest point to `p`.
pub fn outward(s: &Surface, sense: bool, p: Vec3) -> Vec3 {
    let (u, v) = s.closest_param(p);
    let n = s.normal(u, v);
    if sense {
        n
    } else {
        -n
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body};

    #[test]
    fn faces_and_solids() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let c0 = a.find_face("F2.C0").unwrap();
        assert_eq!(point_in_face(&a, c0, Vec3::new(5., 0., -5.), 1e-6), Where::In);
        assert_eq!(point_in_face(&a, c0, Vec3::new(0., 0., -5.), 1e-6), Where::On);
        assert_eq!(point_in_face(&a, c0, Vec3::new(45., 0., -5.), 1e-6), Where::Out);
        assert_eq!(classify_point(&a, Vec3::new(5., 5., -5.), Vec3::X, 1e-6), Class::In);
        assert_eq!(classify_point(&a, Vec3::new(50., 5., -5.), Vec3::X, 1e-6), Class::Out);
        assert_eq!(classify_point(&a, Vec3::new(5., 0., -5.), -Vec3::Y, 1e-6), Class::OnSame);
        assert_eq!(classify_point(&a, Vec3::new(5., 0., -5.), Vec3::Y, 1e-6), Class::OnOpp);
        // periodic face
        let c = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let s = c.find_face("F4.S.e1").unwrap();
        for ang in [0.1f64, 1.0, 3.0, 6.2] {
            let p = Vec3::new(20.0 + 5.0 * ang.cos(), 4.0, -10.0 - 5.0 * ang.sin());
            assert_eq!(point_in_face(&c, s, p, 1e-6), Where::In, "{ang}");
        }
        assert_eq!(point_in_face(&c, s, Vec3::new(25.0, 12.0, -10.0), 1e-6), Where::Out);
    }
}
