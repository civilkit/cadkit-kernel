//! Axis-aligned bounding-volume hierarchy over faces / edges / triangles
//! (ck-ops/src/bvh.rs).
//!
//! Used by the boolean imprint (candidate face pairs between two bodies),
//! containment and interference queries. Clean implementation, not a
//! decompilation: binary tree, median split on the longest centroid axis,
//! `LEAF` items per leaf.
//! GUESS: the original's leaf size / split heuristic are unknown; results
//! (sets of hits / pairs) do not depend on them.

use ck_core::linalg::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId};

/// Axis-aligned box. `EMPTY` has min = +inf, max = -inf.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: Vec3 { x: f64::INFINITY, y: f64::INFINITY, z: f64::INFINITY },
        max: Vec3 { x: f64::NEG_INFINITY, y: f64::NEG_INFINITY, z: f64::NEG_INFINITY },
    };

    pub fn new(min: Vec3, max: Vec3) -> Aabb {
        Aabb { min, max }
    }
    pub fn from_points(pts: &[Vec3]) -> Aabb {
        let mut b = Aabb::EMPTY;
        for &p in pts {
            b.add_point(p);
        }
        b
    }
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }
    pub fn add_point(&mut self, p: Vec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    pub fn union(&self, o: &Aabb) -> Aabb {
        Aabb { min: self.min.min(o.min), max: self.max.max(o.max) }
    }
    /// Grow by `d` on every side.
    pub fn padded(&self, d: f64) -> Aabb {
        let v = Vec3::new(d, d, d);
        Aabb { min: self.min - v, max: self.max + v }
    }
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }
    pub fn extent(&self) -> Vec3 {
        self.max - self.min
    }
    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            self.extent().len()
        }
    }
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }
    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y && p.z >= self.min.z && p.z <= self.max.z
    }
    /// Slab test for the ray `o + t d`, t in [t0, t1]; returns the entry
    /// parameter interval clipped to [t0, t1].
    pub fn ray_hit(&self, o: Vec3, d: Vec3, t0: f64, t1: f64) -> Option<(f64, f64)> {
        let (mut lo, mut hi) = (t0, t1);
        for i in 0..3 {
            let (oi, di, mn, mx) = (o.get(i), d.get(i), self.min.get(i), self.max.get(i));
            if di.abs() < 1e-300 {
                if oi < mn || oi > mx {
                    return None;
                }
                continue;
            }
            let inv = 1.0 / di;
            let (mut a, mut b) = ((mn - oi) * inv, (mx - oi) * inv);
            if a > b {
                std::mem::swap(&mut a, &mut b);
            }
            lo = lo.max(a);
            hi = hi.min(b);
            if lo > hi {
                return None;
            }
        }
        Some((lo, hi))
    }
}

const LEAF: usize = 4;

#[derive(Debug, Clone)]
struct Node {
    bbox: Aabb,
    /// leaf: items[start..start+count]; inner: children (left, right)
    start: usize,
    count: usize,
    left: usize,
    right: usize,
}

/// BVH over item boxes. Item ids are the indices into the `boxes` slice
/// given to [`Bvh::build`].
#[derive(Debug, Clone)]
pub struct Bvh {
    nodes: Vec<Node>,
    items: Vec<usize>,
    boxes: Vec<Aabb>,
}

impl Bvh {
    pub fn build(boxes: &[Aabb]) -> Bvh {
        let mut bvh = Bvh { nodes: vec![], items: (0..boxes.len()).collect(), boxes: boxes.to_vec() };
        if !boxes.is_empty() {
            bvh.build_rec(0, boxes.len());
        }
        bvh
    }

    pub fn len(&self) -> usize {
        self.boxes.len()
    }
    pub fn is_empty(&self) -> bool {
        self.boxes.is_empty()
    }
    pub fn item_box(&self, i: usize) -> Aabb {
        self.boxes[i]
    }
    /// Root box (EMPTY for an empty tree).
    pub fn bbox(&self) -> Aabb {
        self.nodes.first().map(|n| n.bbox).unwrap_or(Aabb::EMPTY)
    }

    fn build_rec(&mut self, start: usize, count: usize) -> usize {
        let mut bb = Aabb::EMPTY;
        let mut cb = Aabb::EMPTY;
        for &i in &self.items[start..start + count] {
            bb = bb.union(&self.boxes[i]);
            cb.add_point(self.boxes[i].center());
        }
        let id = self.nodes.len();
        self.nodes.push(Node { bbox: bb, start, count, left: usize::MAX, right: usize::MAX });
        if count <= LEAF {
            return id;
        }
        let e = cb.extent();
        let axis = if e.x >= e.y && e.x >= e.z {
            0
        } else if e.y >= e.z {
            1
        } else {
            2
        };
        let boxes = &self.boxes;
        self.items[start..start + count].sort_by(|&a, &b| {
            boxes[a].center().get(axis).partial_cmp(&boxes[b].center().get(axis)).unwrap_or(std::cmp::Ordering::Equal)
        });
        let half = count / 2;
        let l = self.build_rec(start, half);
        let r = self.build_rec(start + half, count - half);
        let n = &mut self.nodes[id];
        n.left = l;
        n.right = r;
        n.count = 0;
        id
    }

    fn is_leaf(&self, n: usize) -> bool {
        self.nodes[n].left == usize::MAX
    }

    /// Items whose box overlaps `q` (ascending ids).
    pub fn query_box(&self, q: &Aabb) -> Vec<usize> {
        let mut out = vec![];
        if self.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![0];
        while let Some(n) = stack.pop() {
            let node = &self.nodes[n];
            if !node.bbox.overlaps(q) {
                continue;
            }
            if self.is_leaf(n) {
                for &i in &self.items[node.start..node.start + node.count] {
                    if self.boxes[i].overlaps(q) {
                        out.push(i);
                    }
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        out.sort_unstable();
        out
    }

    /// Items whose box is hit by the ray `o + t d`, t ∈ [t0, t1], sorted by
    /// box entry parameter: `(item, t_enter)`.
    pub fn query_ray(&self, o: Vec3, d: Vec3, t0: f64, t1: f64) -> Vec<(usize, f64)> {
        let mut out = vec![];
        if self.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![0];
        while let Some(n) = stack.pop() {
            let node = &self.nodes[n];
            if node.bbox.ray_hit(o, d, t0, t1).is_none() {
                continue;
            }
            if self.is_leaf(n) {
                for &i in &self.items[node.start..node.start + node.count] {
                    if let Some((a, _)) = self.boxes[i].ray_hit(o, d, t0, t1) {
                        out.push((i, a));
                    }
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        out.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal).then(a.0.cmp(&b.0)));
        out
    }

    /// All pairs (i in self, j in other) with overlapping boxes, sorted.
    pub fn overlap_pairs(&self, other: &Bvh) -> Vec<(usize, usize)> {
        let mut out = vec![];
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![(0usize, 0usize)];
        while let Some((a, b)) = stack.pop() {
            let (na, nb) = (&self.nodes[a], &other.nodes[b]);
            if !na.bbox.overlaps(&nb.bbox) {
                continue;
            }
            match (self.is_leaf(a), other.is_leaf(b)) {
                (true, true) => {
                    for &i in &self.items[na.start..na.start + na.count] {
                        for &j in &other.items[nb.start..nb.start + nb.count] {
                            if self.boxes[i].overlaps(&other.boxes[j]) {
                                out.push((i, j));
                            }
                        }
                    }
                }
                (true, false) => {
                    stack.push((a, nb.left));
                    stack.push((a, nb.right));
                }
                (false, true) => {
                    stack.push((na.left, b));
                    stack.push((na.right, b));
                }
                (false, false) => {
                    // descend the larger box first
                    if na.bbox.diagonal() >= nb.bbox.diagonal() {
                        stack.push((na.left, b));
                        stack.push((na.right, b));
                    } else {
                        stack.push((a, nb.left));
                        stack.push((a, nb.right));
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }

    /// Self-overlap pairs (i < j).
    pub fn self_pairs(&self) -> Vec<(usize, usize)> {
        self.overlap_pairs(self).into_iter().filter(|(i, j)| i < j).collect()
    }
}

// ------------------------------------------------------------ B-rep boxes

/// Box of an edge: its end vertices plus curve samples (conics: exact
/// extreme points are not needed — boxes are padded by the caller's tol).
pub fn edge_box(b: &Body, e: EdgeId) -> Aabb {
    let ed = b.edge(e);
    let n = match ed.curve {
        ck_geom::curve::Curve::Line { .. } => 1,
        _ => 32,
    };
    let mut bb = Aabb::from_points(&ed.curve.sample(ed.t0, ed.t1, n));
    bb.add_point(b.vertex(ed.v0).p);
    bb.add_point(b.vertex(ed.v1).p);
    bb
}

/// Box of a face: union of its edge boxes, plus surface samples over the
/// uv box of its boundary (catches bulging analytic / spline faces whose
/// interior leaves the boundary hull, e.g. a sphere cap).
pub fn face_box(b: &Body, f: FaceId) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for e in b.face_edges(f) {
        bb = bb.union(&edge_box(b, e));
    }
    let face = b.face(f);
    if !matches!(face.surface, ck_geom::surface::Surface::Plane { .. }) {
        // uv bounds from the loops
        let (mut ulo, mut uhi, mut vlo, mut vhi) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &l in &face.loops {
            let mut prev = None;
            for &c in &b.lp(l).coedges {
                let (ta, tb) = b.coedge_params(c);
                for k in 0..=8 {
                    let uv = b.coedge_uv(c, ta + (tb - ta) * k as f64 / 8.0, prev);
                    prev = Some(uv);
                    ulo = ulo.min(uv.x);
                    uhi = uhi.max(uv.x);
                    vlo = vlo.min(uv.y);
                    vhi = vhi.max(uv.y);
                }
            }
        }
        if ulo.is_finite() && vlo.is_finite() {
            let n = 12;
            for i in 0..=n {
                for j in 0..=n {
                    let u = ulo + (uhi - ulo) * i as f64 / n as f64;
                    let v = vlo + (vhi - vlo) * j as f64 / n as f64;
                    bb.add_point(face.surface.eval(u, v));
                }
            }
        }
    }
    bb
}

/// BVH over the live faces of a body; returns (tree, face id per item).
pub fn face_bvh(b: &Body, pad: f64) -> (Bvh, Vec<FaceId>) {
    let ids: Vec<FaceId> = b.face_ids().collect();
    let boxes: Vec<Aabb> = ids.iter().map(|&f| face_box(b, f).padded(pad)).collect();
    (Bvh::build(&boxes), ids)
}

/// BVH over the live edges of a body.
pub fn edge_bvh(b: &Body, pad: f64) -> (Bvh, Vec<EdgeId>) {
    let ids: Vec<EdgeId> = b.edge_ids().collect();
    let boxes: Vec<Aabb> = ids.iter().map(|&e| edge_box(b, e).padded(pad)).collect();
    (Bvh::build(&boxes), ids)
}

/// BVH over triangles given as index triples into `pos`.
pub fn triangle_bvh(pos: &[Vec3], tris: &[[u32; 3]]) -> Bvh {
    let boxes: Vec<Aabb> =
        tris.iter().map(|t| Aabb::from_points(&[pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]])).collect();
    Bvh::build(&boxes)
}

/// Möller–Trumbore ray/triangle: parameter t (> 0) of the hit.
pub fn ray_triangle(o: Vec3, d: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-300 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    if !(-1e-12..=1.0 + 1e-12).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < -1e-12 || u + v > 1.0 + 1e-12 {
        return None;
    }
    let t = e2.dot(q) * inv;
    if t > 0.0 {
        Some(t)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box(c: Vec3, h: f64) -> Aabb {
        Aabb::new(c - Vec3::new(h, h, h), c + Vec3::new(h, h, h))
    }

    #[test]
    fn query_matches_brute_force() {
        // 10x10x3 grid of small boxes
        let mut boxes = vec![];
        for i in 0..10 {
            for j in 0..10 {
                for k in 0..3 {
                    boxes.push(unit_box(Vec3::new(i as f64, j as f64 * 1.3, k as f64 * 2.0), 0.4));
                }
            }
        }
        let t = Bvh::build(&boxes);
        let q = Aabb::new(Vec3::new(2.2, 1.0, -1.0), Vec3::new(5.1, 4.0, 2.5));
        let brute: Vec<usize> = (0..boxes.len()).filter(|&i| boxes[i].overlaps(&q)).collect();
        assert_eq!(t.query_box(&q), brute);
        assert!(!brute.is_empty());
        // ray along x at y=1.3, z=0
        let hits = t.query_ray(Vec3::new(-5.0, 1.3, 0.0), Vec3::X, 0.0, f64::INFINITY);
        assert_eq!(hits.len(), 10);
        assert!(hits.windows(2).all(|w| w[0].1 <= w[1].1));
        assert!((hits[0].1 - 4.6).abs() < 1e-12);
    }

    #[test]
    fn pairs_match_brute_force() {
        let a: Vec<Aabb> = (0..40).map(|i| unit_box(Vec3::new(i as f64 * 0.7, 0.0, 0.0), 0.5)).collect();
        let b: Vec<Aabb> = (0..25).map(|i| unit_box(Vec3::new(i as f64 * 1.1 + 0.3, 0.6, 0.0), 0.2)).collect();
        let (ta, tb) = (Bvh::build(&a), Bvh::build(&b));
        let mut brute = vec![];
        for i in 0..a.len() {
            for j in 0..b.len() {
                if a[i].overlaps(&b[j]) {
                    brute.push((i, j));
                }
            }
        }
        assert_eq!(ta.overlap_pairs(&tb), brute);
        let sp = ta.self_pairs();
        assert!(sp.contains(&(0, 1)) && !sp.contains(&(0, 2)));
    }

    #[test]
    fn ray_tri() {
        let t = ray_triangle(Vec3::new(0.2, 0.2, 5.0), -Vec3::Z, Vec3::ZERO, Vec3::X, Vec3::Y).unwrap();
        assert!((t - 5.0).abs() < 1e-15);
        assert!(ray_triangle(Vec3::new(0.8, 0.8, 5.0), -Vec3::Z, Vec3::ZERO, Vec3::X, Vec3::Y).is_none());
    }
}
