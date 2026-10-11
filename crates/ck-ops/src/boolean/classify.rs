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
/// solid `other` (one-shot; prefer [`Solid`] for repeated queries).
pub fn classify_point(other: &Body, q: Vec3, n: Vec3, tol: f64) -> Class {
    Solid::new(other, tol).classify(q, n)
}

/// A body prepared for repeated point classification: face uv polygons,
/// face boxes and a BVH, so each query is an "on" test against the few
/// faces near the point plus ray parity against the faces the ray's box
/// path meets.
pub struct Solid<'a> {
    pub body: &'a Body,
    tol: f64,
    faces: Vec<FaceId>,
    loops: Vec<Vec<Vec<Vec2>>>,
    edges: Vec<Vec<ck_topo::body::EdgeId>>,
    bvh: crate::bvh::Bvh,
}

/// Ray directions tried in turn (irrational-ish, away from the axes so a
/// ray seldom grazes an axis-aligned edge or face).
const RAYS: [[f64; 3]; 6] = [
    [0.5773502691896258, 0.6123724356957945, 0.5400617248673217],
    [-0.3511234415883917, 0.8164965809277260, 0.4580392070833400],
    [0.7071067811865475, -0.2236067977499790, 0.6708203932499369],
    [-0.6, -0.48, 0.64],
    [0.1414213562373095, 0.4242640687119285, -0.8944271909999159],
    [-0.8017837257372732, 0.2672612419124244, -0.5345224838248488],
];

impl<'a> Solid<'a> {
    pub fn new(body: &'a Body, tol: f64) -> Solid<'a> {
        let faces: Vec<FaceId> = body.face_ids().collect();
        let loops = faces.iter().map(|&f| face_loops_uv(body, f)).collect();
        let edges = faces.iter().map(|&f| body.face_edges(f)).collect();
        let boxes: Vec<crate::bvh::Aabb> = faces
            .iter()
            .map(|&f| {
                let bb = crate::bvh::face_box(body, f);
                // sampled boxes under-cover curved faces by < 0.5% of their size
                bb.padded(tol * 10.0 + 0.02 * bb.diagonal())
            })
            .collect();
        Solid { body, tol, faces, loops, edges, bvh: crate::bvh::Bvh::build(&boxes) }
    }

    /// Point `p` (near face `i`'s surface) inside / on / outside face `i`.
    fn in_face(&self, i: usize, p: Vec3) -> Where {
        let b = self.body;
        for &e in &self.edges[i] {
            let ed = b.edge(e);
            if super::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= self.tol {
                return Where::On;
            }
        }
        let s = &b.face(self.faces[i]).surface;
        if s.distance(p) > self.tol * 10.0 {
            return Where::Out;
        }
        if self.loops[i].iter().all(|l| l.len() < 3) {
            return Where::In;
        }
        let (u, v) = s.closest_param(p);
        if in_loops(s, Vec2::new(u, v), &self.loops[i]) {
            Where::In
        } else {
            Where::Out
        }
    }

    /// Classify point `q` (with outward normal `n` of the fragment it
    /// samples).
    pub fn classify(&self, q: Vec3, n: Vec3) -> Class {
        let (b, tol) = (self.body, self.tol);
        if self.faces.is_empty() {
            return Class::Out;
        }
        let near = crate::bvh::Aabb::new(q, q).padded(tol * 10.0);
        for i in self.bvh.query_box(&near) {
            let g = self.faces[i];
            let (u, v, cp) = b.face(g).surface.closest_point(q);
            if cp.dist(q) > tol * 10.0 || self.in_face(i, q) == Where::Out {
                continue;
            }
            let ng = b.outward_normal(g, u, v);
            return if ng.dot(n) > 0.0 { Class::OnSame } else { Class::OnOpp };
        }
        for d in RAYS {
            if let Some(inside) = self.ray_parity(q, Vec3::new(d[0], d[1], d[2])) {
                return if inside { Class::In } else { Class::Out };
            }
        }
        // every ray ambiguous (or a face without a closed-form line
        // intersection): generalised winding number
        if winding_number(b, &self.faces, q) > 0.5 {
            Class::In
        } else {
            Class::Out
        }
    }

    /// Parity of the crossings of the ray `q + t d` (t > 0); None when the
    /// ray grazes a face, passes through an edge, or meets a surface
    /// without a closed-form line intersection.
    fn ray_parity(&self, q: Vec3, d: Vec3) -> Option<bool> {
        let b = self.body;
        let mut count = 0usize;
        let mut seen = vec![];
        for (i, _) in self.bvh.query_ray(q, d, 0.0, f64::INFINITY) {
            if seen.contains(&i) {
                continue;
            }
            seen.push(i);
            let s = &b.face(self.faces[i]).surface;
            let mut ts = crate::intersect::cs::line_params(q, d, s, self.tol)?;
            ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mut prev: Option<f64> = None;
            for t in ts {
                if t.abs() <= self.tol * 10.0 {
                    return None; // q on this surface (outside the face): ambiguous
                }
                if t < 0.0 {
                    continue;
                }
                if prev.map_or(false, |p| (t - p).abs() <= self.tol * 10.0) {
                    return None; // double root: tangent
                }
                prev = Some(t);
                let p = q + d * t;
                match self.in_face(i, p) {
                    Where::On => return None,
                    Where::Out => {}
                    Where::In => {
                        let (u, v) = s.closest_param(p);
                        let nn = s.normal(u, v).normalize();
                        if nn.dot(d).abs() < 1e-3 {
                            return None; // grazing
                        }
                        count += 1;
                    }
                }
            }
        }
        Some(count % 2 == 1)
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
