//! Point containment against a body (ck-ops/src/contain.rs): a tolerance-
//! padded face BVH, lazily built per-face cells, a face-level point test and
//! the solid classifier (parity of up to 7 fixed-direction rays, falling back
//! to the nearest face when every ray is ambiguous).
//!
//! Wasm provenance (kernel.wasm): f2668 `Containment::new`, f2669 live face
//! ids, f1428 `cell` (OnceCell init, state byte 2 = initialising / 3 =
//! done), f2667 `uv_tol`, f2430 `point_in_face`, f2670 `classify_point`.
//!
//! GUESS markers: the original answers the face-level question with its own
//! pcurve walk (`f3768`/`f3798` curve queries on the loop coedges) and a uv
//! tolerance (`f2667`, 2 % of the uv box); here the same three-way answer
//! (inside / on the boundary / outside) comes from the polygon + 3D edge
//! distance test of `boolean::classify`. The ray hit's "tangent" flag
//! (hit record byte +0x38) is taken as |n·d| < 1e-9.
//! Nothing in the crate calls this yet (`boolean::classify` has its own
//! winding-number classifier); it is checked against that one in the tests.

use crate::boolean::classify::{face_loops_uv, Where};
use crate::boolean::imprint::dist_in_range;
use crate::bvh::{face_bvh, Aabb, Bvh};
use crate::intersect::cs;
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::Vec2;
use ck_topo::body::{Body, FaceId};
use std::cell::OnceCell;

/// Answer of [`Containment::classify_point`] (wasm codes 0 / 1 / 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Loc {
    In,
    Out,
    /// within tolerance of the boundary
    On,
}

/// One ray/face crossing ([`Containment::ray_hits`]).
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    pub t: f64,
    pub point: Vec3,
    pub face: FaceId,
    pub entering: bool,
    pub tangent: bool,
}

/// The seven ray directions (kernel.wasm data at 0x80f4d8).
pub const RAYS: [[f64; 3]; 7] = [
    [0.5773502691896258, 0.5773502691896257, 0.577350269189626],
    [0.2672612419124244, -0.5345224838248488, 0.8017837257372732],
    [-0.8017837257372731, 0.2672612419124244, 0.5345224838248488],
    [0.4082482904638631, 0.8164965809277261, -0.408248290463863],
    [-0.3713906763541037, -0.5570860145311556, -0.7427813527082074],
    [0.9128709291752769, -0.3651483716701107, 0.1825741858350554],
    [-0.1561737618886061, 0.9370425713316364, 0.3123475237772121],
];

/// Per-face data built on first use (f1428): the uv loops of the face.
struct Cell {
    loops: Vec<Vec<Vec2>>,
}

pub struct Containment<'a> {
    body: &'a Body,
    tol: f64,
    /// live faces (f2669), index = BVH item = cell index
    faces: Vec<FaceId>,
    bbox: Aabb,
    bvh: Bvh,
    cells: Vec<OnceCell<Option<Cell>>>,
}

impl<'a> Containment<'a> {
    /// f2668: pad every face box by `tol`, build the BVH.
    pub fn new(body: &'a Body, tol: f64) -> Self {
        let (bvh, faces) = face_bvh(body, tol);
        let bbox = (0..bvh.len()).fold(Aabb::EMPTY, |a, i| a.union(&bvh.item_box(i)));
        let cells = faces.iter().map(|_| OnceCell::new()).collect();
        Containment { body, tol, faces, bbox, bvh, cells }
    }

    pub fn tol(&self) -> f64 {
        self.tol
    }

    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// f1428: the face's cell, built once; `None` when the face has no
    /// usable boundary polygon.
    fn cell(&self, i: usize) -> Option<&Cell> {
        self.cells[i]
            .get_or_init(|| {
                let loops = face_loops_uv(self.body, self.faces[i]);
                if loops.iter().any(|l| l.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())) {
                    None
                } else {
                    Some(Cell { loops })
                }
            })
            .as_ref()
    }

    /// Does the (u, v) lie inside the face's uv region (f3488)?
    fn contains_uv(&self, i: usize, u: f64, v: f64) -> bool {
        let Some(c) = self.cell(i) else { return false };
        let face = self.body.face(self.faces[i]);
        // closed surface without boundary (full sphere / torus): everywhere
        if c.loops.iter().all(|l| l.len() < 3) {
            return true;
        }
        crate::boolean::classify::in_face_loops(self.body, self.faces[i], Vec2::new(u, v), &c.loops)
    }

    /// f2430: is the surface point (u, v) of face `i` inside the face,
    /// on its boundary (within `tol`) or outside?
    pub fn point_in_face(&self, i: usize, u: f64, v: f64, tol: f64) -> Where {
        if self.cell(i).is_none() {
            return Where::Out;
        }
        let f = self.faces[i];
        let face = self.body.face(f);
        let p = face.surface.eval(u, v);
        for e in self.body.face_edges(f) {
            let ed = self.body.edge(e);
            if dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= tol {
                return Where::On;
            }
        }
        if self.contains_uv(i, u, v) {
            Where::In
        } else {
            Where::Out
        }
    }

    /// Every crossing of the ray `o + t d` (t in [0, tmax]) with the faces, sorted by t. Hits on a shared edge (two faces
    /// within 1e-7 in t) are merged. Faces the ray runs inside are skipped.
    pub fn ray_hits(&self, o: Vec3, d: Vec3, tmax: f64, tol: f64) -> Vec<RayHit> {
        let line = Curve::line(o, d);
        let mut out: Vec<RayHit> = vec![];
        for (fi, _) in self.bvh.query_ray(o, d, -tol, tmax) {
            let f = self.faces[fi];
            let surf = &self.body.face(f).surface;
            let res = cs::intersect(&line, (-tol, tmax), surf, tol);
            if !res.on.is_empty() {
                continue;
            }
            for h in &res.hits {
                if h.t < -tol || h.t > tmax || self.point_in_face(fi, h.uv.0, h.uv.1, tol) == Where::Out {
                    continue;
                }
                let n = self.body.outward_normal(f, h.uv.0, h.uv.1);
                let nd = n.dot(d);
                out.push(RayHit { t: h.t.max(0.0), point: h.p, face: f, entering: nd < 0.0, tangent: nd.abs() < 1e-9 });
            }
        }
        out.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
        let mut merged: Vec<RayHit> = vec![];
        for h in out {
            if let Some(l) = merged.last() {
                if (h.t - l.t).abs() < 1e-7 && h.entering == l.entering {
                    continue;
                }
            }
            merged.push(h);
        }
        merged
    }

    /// Index of face `f` in [`Containment::faces`].
    pub fn face_index(&self, f: FaceId) -> Option<usize> {
        self.faces.iter().position(|&g| g == f)
    }

    /// Nearest point of face `i` (interior or boundary edge) to `q`: `(distance, point)`.
    pub fn face_nearest(&self, i: usize, q: Vec3) -> Option<(f64, Vec3)> {
        let f = self.faces[i];
        let mut best: Option<(f64, Vec3)> = None;
        let (u, v, p) = self.body.face(f).surface.closest_point(q);
        let dist = p.dist(q);
        if dist.is_finite() && self.point_in_face(i, u, v, self.tol) != Where::Out {
            best = Some((dist, p));
        }
        for e in self.body.face_edges(f) {
            let ed = self.body.edge(e);
            let (t, dd) = dist_in_range(&ed.curve, (ed.t0, ed.t1), q);
            if dd.is_finite() && best.map_or(true, |b| dd < b.0) {
                best = Some((dd, ed.curve.eval(t)));
            }
        }
        best
    }

    /// Nearest point of the boundary to `q`: `(distance, face, point)`. Face interiors (closest point inside the trim)
    /// and every boundary edge are searched; faces are visited by box distance so far faces are never touched.
    pub fn nearest(&self, q: Vec3) -> Option<(f64, FaceId, Vec3)> {
        let mut order: Vec<(f64, usize)> = (0..self.faces.len())
            .map(|i| {
                let b = self.bvh.item_box(i);
                let c = |v: f64, lo: f64, hi: f64| if v < lo { lo - v } else if v > hi { v - hi } else { 0.0 };
                let (dx, dy, dz) = (c(q.x, b.min.x, b.max.x), c(q.y, b.min.y, b.max.y), c(q.z, b.min.z, b.max.z));
                ((dx * dx + dy * dy + dz * dz).sqrt(), i)
            })
            .collect();
        order.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut best: Option<(f64, FaceId, Vec3)> = None;
        for (lb, i) in order {
            if best.map_or(false, |b| lb > b.0) {
                break;
            }
            let f = self.faces[i];
            let surf = &self.body.face(f).surface;
            let (u, v, p) = surf.closest_point(q);
            let dist = p.dist(q);
            if dist.is_finite() && best.map_or(true, |b| dist < b.0) && self.point_in_face(i, u, v, self.tol) != Where::Out {
                best = Some((dist, f, p));
            }
            for e in self.body.face_edges(f) {
                let ed = self.body.edge(e);
                let (t, dd) = dist_in_range(&ed.curve, (ed.t0, ed.t1), q);
                if dd.is_finite() && best.map_or(true, |b| dd < b.0) {
                    best = Some((dd, f, ed.curve.eval(t)));
                }
            }
        }
        best
    }

    /// f2670: where is `q` relative to the solid?
    pub fn classify_point(&self, q: Vec3, tol: f64) -> Loc {
        let (lo, hi) = (self.bbox.min, self.bbox.max);
        if !(lo.x - tol <= q.x && q.x <= tol + hi.x && lo.y - tol <= q.y && q.y <= tol + hi.y && lo.z - tol <= q.z && q.z <= tol + hi.z) {
            return Loc::Out;
        }
        let len = if self.bbox.is_empty() {
            1.0
        } else {
            let (dx, dy, dz) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
            let diag = (dx * dx + dy * dy + dz * dz).sqrt();
            diag + diag + 1.0
        };
        'rays: for r in RAYS {
            let d = Vec3::new(r[0], r[1], r[2]);
            let line = Curve::line(q, d);
            let mut count = 0u32;
            for (fi, _) in self.bvh.query_ray(q, d, -tol, len) {
                let surf = &self.body.face(self.faces[fi]).surface;
                let res = cs::intersect(&line, (-tol, len), surf, tol);
                if !res.on.is_empty() {
                    continue 'rays; // ray runs in the surface: ambiguous
                }
                for h in &res.hits {
                    let w = self.point_in_face(fi, h.uv.0, h.uv.1, tol);
                    if h.t.abs() <= tol {
                        if w != Where::Out {
                            return Loc::On;
                        }
                    } else if h.t >= 0.0 {
                        match w {
                            Where::In => {
                                let n = surf.normal(h.uv.0, h.uv.1);
                                if n.dot(d).abs() < 1e-9 {
                                    continue 'rays; // tangent hit
                                }
                                count += 1;
                            }
                            Where::On => continue 'rays,
                            Where::Out => {}
                        }
                    }
                }
            }
            return if count % 2 == 1 { Loc::In } else { Loc::Out };
        }
        // every ray ambiguous: nearest face whose uv region holds the foot point
        let mut best: Option<(f64, usize, f64, f64, Vec3)> = None;
        for i in 0..self.faces.len() {
            let surf = &self.body.face(self.faces[i]).surface;
            let (u, v, p) = surf.closest_point(q);
            let dist = p.dist(q);
            if !self.contains_uv(i, u, v) {
                continue;
            }
            if best.map_or(true, |b| dist < b.0) {
                best = Some((dist, i, u, v, p));
            }
        }
        let Some((dist, i, u, v, p)) = best else { return Loc::Out };
        if dist <= tol {
            return Loc::On;
        }
        let n = self.body.outward_normal(self.faces[i], u, v);
        if 0.0 <= n.z * (q.z - p.z) + n.x * (q.x - p.x) + n.y * (q.y - p.y) {
            Loc::Out
        } else {
            Loc::In
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::classify::{classify_point as winding_class, Class};
    use crate::boolean::tests::{box_body, cyl_body};

    /// Same three-way answer as the winding-number classifier on a grid.
    fn agree(b: &Body, lo: f64, hi: f64) -> (usize, usize) {
        let c = Containment::new(b, 1e-6);
        let (mut n, mut bad) = (0, 0);
        let steps = 7;
        for i in 0..=steps {
            for j in 0..=steps {
                for k in 0..=steps {
                    let f = |t: usize| lo + (hi - lo) * (t as f64 + 0.37) / (steps as f64 + 0.74);
                    let q = Vec3::new(f(i), f(j), f(k));
                    let want = match winding_class(b, q, Vec3::X, 1e-6) {
                        Class::In => Loc::In,
                        Class::Out => Loc::Out,
                        _ => Loc::On,
                    };
                    n += 1;
                    if c.classify_point(q, 1e-6) != want {
                        bad += 1;
                    }
                }
            }
        }
        (n, bad)
    }

    #[test]
    fn box_inside_outside_on() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let c = Containment::new(&b, 1e-6);
        assert_eq!(c.classify_point(Vec3::new(5., 5., -5.), 1e-6), Loc::In);
        assert_eq!(c.classify_point(Vec3::new(50., 5., -5.), 1e-6), Loc::Out);
        assert_eq!(c.classify_point(Vec3::new(5., 0., -5.), 1e-6), Loc::On);
        assert_eq!(c.classify_point(Vec3::new(5., 25., -5.), 1e-6), Loc::Out);
        let c0 = b.find_face("F2.C0").unwrap();
        let i = c.faces().iter().position(|&f| f == c0).unwrap();
        let uv = |p: Vec3| b.face(c0).surface.closest_param(p);
        let at = |p: Vec3| c.point_in_face(i, uv(p).0, uv(p).1, 1e-6);
        assert_eq!(at(Vec3::new(5., 0., -5.)), Where::In);
        assert_eq!(at(Vec3::new(0., 0., -5.)), Where::On);
        assert_eq!(at(Vec3::new(45., 0., -5.)), Where::Out);
    }

    #[test]
    fn agrees_with_winding_classifier() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let (n, bad) = agree(&b, -10.0, 50.0);
        assert_eq!(bad, 0, "box: {bad}/{n}");
        let c = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let (n, bad) = agree(&c, -2.0, 30.0);
        assert_eq!(bad, 0, "cylinder: {bad}/{n}");
    }
}
