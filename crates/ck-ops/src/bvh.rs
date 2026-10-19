//! Axis-aligned bounding-volume hierarchy over faces / edges / triangles
//! (ck-ops/src/bvh.rs).
//!
//! Used by the boolean imprint (candidate face pairs between two bodies),
//! containment and interference queries. The build and the pair traversal
//! are decoded from the original (f2659 / f2660 build, f2661 pairs; the
//! ORDER of the pairs decides the order of the boolean's cut edges and so
//! the hole-loop order / loop start edges of imprinted faces):
//! - items = boxes with min <= max on every axis, in input order;
//!   centroids = (min + max) * 0.5;
//! - node box = union of its item boxes; < 5 items -> leaf;
//! - split axis = longest CENTROID-bounds axis (ties -> lower axis);
//! - centroid extent > 0: SAH over 12 bins (bin = min(trunc_sat((c - lo) /
//!   (hi - lo) * 12), 11), bins grow by item boxes), cost(split k) =
//!   SA(left) * n_l + SA(right) * n_r for k = 1..11 with both sides
//!   non-empty, first strict minimum; items partitioned in place (an item
//!   with bin >= k is swapped with the last unprocessed one); a split at
//!   either end falls back to the median n/2;
//! - zero extent or no finite cost: stable sort by centroid[axis]
//!   (total order), median n/2;
//! - children are pushed as a consecutive pair (left, left + 1).
//! Pairs (f2661): stack of node pairs from (root, root), popped from the
//! top; both inner -> split the one with the larger box surface area (A on
//! ties); children pushed left then right (so the RIGHT child is visited
//! first); leaf x leaf emits (a, b) for a in A-leaf order, b in B-leaf
//! order whose padded boxes overlap.

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

#[derive(Debug, Clone)]
struct Node {
    bbox: Aabb,
    /// leaf: items[start..start+count]; inner: children (left, left + 1)
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

/// Surface area as the original computes it (f3917): 2 (dx dz + (dx dy + dy dz)), 0 when empty
/// (the association decides SAH ties: oracle costs 50444.867930432294 vs
/// 50444.86793043229 for mirror-symmetric splits).
fn area(b: &Aabb) -> f64 {
    if !(b.min.x <= b.max.x && b.min.y <= b.max.y && b.min.z <= b.max.z) {
        return 0.0;
    }
    let (dx, dy, dz) = (b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z);
    let a = dx * dz + (dx * dy + dy * dz);
    a + a
}

fn valid(b: &Aabb) -> bool {
    b.min.x <= b.max.x && b.min.y <= b.max.y && b.min.z <= b.max.z
}

impl Bvh {
    pub fn build(boxes: &[Aabb]) -> Bvh {
        let items: Vec<usize> = (0..boxes.len()).filter(|&i| valid(&boxes[i])).collect();
        let mut bvh = Bvh { nodes: vec![], items, boxes: boxes.to_vec() };
        if !bvh.items.is_empty() {
            let cent: Vec<Vec3> = boxes.iter().map(|b| (b.min + b.max) * 0.5).collect();
            bvh.nodes.push(Node { bbox: Aabb::EMPTY, start: 0, count: 0, left: usize::MAX, right: usize::MAX });
            let n = bvh.items.len();
            bvh.build_rec(0, 0, n, &cent);
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

    fn is_leaf(&self, n: usize) -> bool {
        self.nodes[n].left == usize::MAX
    }

    fn build_rec(&mut self, node: usize, start: usize, end: usize, cent: &[Vec3]) {
        let mut bb = Aabb::EMPTY;
        let mut cb = Aabb::EMPTY;
        for &i in &self.items[start..end] {
            bb = bb.union(&self.boxes[i]);
            cb.add_point(cent[i]);
        }
        self.nodes[node].bbox = bb;
        let n = end - start;
        if n < 5 {
            self.nodes[node].start = start;
            self.nodes[node].count = n;
            return;
        }
        let (dx, dy, dz) =
            if valid(&cb) { (cb.max.x - cb.min.x, cb.max.y - cb.min.y, cb.max.z - cb.min.z) } else { (0.0, 0.0, 0.0) };
        let a = if dy >= dz { 1 } else { 2 };
        let axis = if dx >= dy {
            if dx >= dz {
                0
            } else {
                a
            }
        } else {
            a
        };
        let (lo, hi) = (cb.min.get(axis), cb.max.get(axis));
        let median = start + n / 2;
        let bin = |c: f64| -> usize {
            let x = (c - lo) / (hi - lo) * 12.0;
            // i32.trunc_sat_f64_u
            let b = if x.is_nan() || x <= -1.0 { 0 } else if x >= 4294967296.0 { u32::MAX } else { x as u32 };
            (b as usize).min(11)
        };
        let mut mid = median;
        let mut sorted = true;
        if hi > lo {
            let mut bins = [Aabb::EMPTY; 12];
            let mut counts = [0usize; 12];
            for &i in &self.items[start..end] {
                let k = bin(cent[i].get(axis));
                bins[k] = bins[k].union(&self.boxes[i]);
                counts[k] += 1;
            }
            let mut best = f64::INFINITY;
            let mut best_k = 0usize;
            for k in 1..12 {
                let (mut lb, mut rb) = (Aabb::EMPTY, Aabb::EMPTY);
                let (mut nl, mut nr) = (0usize, 0usize);
                for j in 0..k {
                    lb = lb.union(&bins[j]);
                    nl += counts[j];
                }
                for j in k..12 {
                    rb = rb.union(&bins[j]);
                    nr += counts[j];
                }
                if nl == 0 || nr == 0 {
                    continue;
                }
                let cost = area(&lb) * nl as f64 + area(&rb) * nr as f64;
                if cost < best {
                    best = cost;
                    best_k = k;
                }
            }
            if best.is_finite() {
                sorted = false;
                let (mut l, mut c) = (0usize, n);
                while l < c {
                    let v = self.items[start + l];
                    if bin(cent[v].get(axis)) < best_k {
                        l += 1;
                    } else {
                        c -= 1;
                        self.items.swap(start + l, start + c);
                    }
                }
                mid = start + l;
            }
        }
        if sorted {
            self.items[start..end].sort_by(|&a, &b| cent[a].get(axis).total_cmp(&cent[b].get(axis)));
            mid = median;
        }
        if mid == start || mid == end {
            mid = median;
        }
        let left = self.nodes.len();
        for _ in 0..2 {
            self.nodes.push(Node { bbox: Aabb::EMPTY, start: 0, count: 0, left: usize::MAX, right: usize::MAX });
        }
        self.nodes[node].left = left;
        self.nodes[node].right = left + 1;
        self.build_rec(left, start, mid, cent);
        self.build_rec(left + 1, mid, end, cent);
    }

    /// Items whose box overlaps `q` (ascending ids).
    /// Whether any item box overlaps `q` (the early-exit form of [`Bvh::query_box`]).
    pub fn any_overlap(&self, q: &Aabb) -> bool {
        if self.nodes.is_empty() {
            return false;
        }
        let mut stack = vec![0];
        while let Some(n) = stack.pop() {
            let node = &self.nodes[n];
            if !node.bbox.overlaps(q) {
                continue;
            }
            if self.is_leaf(n) {
                if self.items[node.start..node.start + node.count].iter().any(|&i| self.boxes[i].overlaps(q)) {
                    return true;
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        false
    }

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

    /// All pairs (i in self, j in other) whose boxes overlap, sorted.
    pub fn overlap_pairs(&self, other: &Bvh) -> Vec<(usize, usize)> {
        let mut out = self.overlap_pairs_ordered(other, 0.0);
        out.sort_unstable();
        out
    }

    /// Overlapping pairs in the original's traversal order (f2661), boxes
    /// compared with slack `pad`.
    pub fn overlap_pairs_ordered(&self, other: &Bvh, pad: f64) -> Vec<(usize, usize)> {
        let ov = |a: &Aabb, b: &Aabb| {
            a.min.x <= pad + b.max.x
                && b.min.x <= pad + a.max.x
                && a.min.y <= pad + b.max.y
                && b.min.y <= pad + a.max.y
                && a.min.z <= pad + b.max.z
                && b.min.z <= pad + a.max.z
        };
        let mut out = vec![];
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![(0usize, 0usize)];
        while let Some((a, b)) = stack.pop() {
            let (na, nb) = (&self.nodes[a], &other.nodes[b]);
            if !ov(&na.bbox, &nb.bbox) {
                continue;
            }
            match (self.is_leaf(a), other.is_leaf(b)) {
                (true, true) => {
                    for &i in &self.items[na.start..na.start + na.count] {
                        for &j in &other.items[nb.start..nb.start + nb.count] {
                            if ov(&self.boxes[i], &other.boxes[j]) {
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
                    if area(&na.bbox) >= area(&nb.bbox) {
                        stack.push((na.left, b));
                        stack.push((na.right, b));
                    } else {
                        stack.push((a, nb.left));
                        stack.push((a, nb.right));
                    }
                }
            }
        }
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
    let mut bb = curve_range_box(&ed.curve, ed.t0, ed.t1);
    bb.add_point(b.vertex(ed.v0).p);
    bb.add_point(b.vertex(ed.v1).p);
    bb
}

/// Box of the curve over `[t0, t1]`: samples plus (conics) the exact
/// per-axis extreme points inside the range.
pub fn curve_range_box(curve: &ck_geom::curve::Curve, t0: f64, t1: f64) -> Aabb {
    let n = match curve {
        ck_geom::curve::Curve::Line { .. } => 1,
        _ => 32,
    };
    let mut bb = Aabb::from_points(&curve.sample(t0, t1, n));
    let conic = match curve {
        ck_geom::curve::Curve::Circle { frame, r } => Some((frame, *r, *r)),
        ck_geom::curve::Curve::Ellipse { frame, a, b } => Some((frame, *a, *b)),
        _ => None,
    };
    if let Some((fr, a, b)) = conic {
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        for i in 0..3 {
            let t = (fr.y.get(i) * b).atan2(fr.x.get(i) * a);
            for k in -3..=3 {
                let tt = t + k as f64 * std::f64::consts::PI;
                if tt >= lo && tt <= hi {
                    bb.add_point(curve.eval(tt));
                }
            }
        }
    }
    bb
}

/// A box sure to hold `curve` over [t0, t1], for lossless rejection (no point of the curve lies outside it; the
/// sampled [`curve_range_box`] can miss a bulge between samples). Lines from the end points, circles and ellipses
/// from the end points and the per-axis extrema inside the range (the whole conic for a range past two turns),
/// B-splines from the control points of the knot spans the range touches (convex hull property, positive weights
/// only). `None` when no cheap bound exists (other curves, non-finite ranges).
pub fn curve_hull_box(curve: &ck_geom::curve::Curve, t0: f64, t1: f64) -> Option<Aabb> {
    use ck_geom::curve::Curve;
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    if !(lo.is_finite() && hi.is_finite()) {
        return None;
    }
    let bb = match curve {
        Curve::Line { .. } => Aabb::from_points(&[curve.eval(lo), curve.eval(hi)]),
        Curve::Circle { .. } | Curve::Ellipse { .. } => {
            let (frame, a, b) = match curve {
                Curve::Circle { frame, r } => (frame, *r, *r),
                Curve::Ellipse { frame, a, b } => (frame, *a, *b),
                _ => unreachable!(),
            };
            let mut bb = Aabb::from_points(&[curve.eval(lo), curve.eval(hi)]);
            if hi - lo >= std::f64::consts::TAU || lo < -std::f64::consts::TAU || hi > 2.0 * std::f64::consts::TAU {
                let e = Vec3::new(frame.x.x.abs() * a + frame.y.x.abs() * b, frame.x.y.abs() * a + frame.y.y.abs() * b, frame.x.z.abs() * a + frame.y.z.abs() * b);
                bb = Aabb::new(frame.o - e, frame.o + e);
            } else {
                // coordinate i is extreme where tan t = (y_i b) / (x_i a): every such t in the range, all branches
                for i in 0..3 {
                    let t = (frame.y.get(i) * b).atan2(frame.x.get(i) * a);
                    for k in -4..=4 {
                        let tt = t + k as f64 * std::f64::consts::PI;
                        if tt >= lo && tt <= hi {
                            bb.add_point(curve.eval(tt));
                        }
                    }
                }
            }
            bb
        }
        Curve::BSpline(s) => {
            if s.weights.as_ref().is_some_and(|w| w.iter().any(|&x| !(x > 0.0))) || s.ctrl.len() <= s.degree {
                return None;
            }
            let p = s.degree;
            let mut bb = Aabb::EMPTY;
            for k in p..s.ctrl.len() {
                let (a, b) = (s.knots[k], s.knots[k + 1]);
                if a < b && b >= lo && a <= hi {
                    for &q in &s.ctrl[k - p..=k] {
                        bb.add_point(q);
                    }
                }
            }
            if bb.is_empty() {
                // a range outside the knot vector: the whole control polygon
                bb = Aabb::from_points(&s.ctrl);
            }
            bb
        }
        _ => return None,
    };
    let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    (finite(bb.min) && finite(bb.max)).then_some(bb)
}

/// Finer boxes that together hold `curve` over [t0, t1] (same guarantee as [`curve_hull_box`]): one per knot span
/// the range touches for a B-spline (the control points of that span), arcs of at most an eighth of a turn for
/// circles and ellipses, the one box of a line. `None` when no cheap bound exists.
pub fn curve_span_boxes(curve: &ck_geom::curve::Curve, t0: f64, t1: f64) -> Option<Vec<Aabb>> {
    use ck_geom::curve::Curve;
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    match curve {
        Curve::BSpline(s) => {
            curve_hull_box(curve, lo, hi)?;
            let p = s.degree;
            let mut out = vec![];
            for k in p..s.ctrl.len() {
                let (a, b) = (s.knots[k], s.knots[k + 1]);
                if a < b && b >= lo && a <= hi {
                    out.push(Aabb::from_points(&s.ctrl[k - p..=k]));
                }
            }
            if out.is_empty() {
                out.push(Aabb::from_points(&s.ctrl));
            }
            Some(out)
        }
        Curve::Circle { .. } | Curve::Ellipse { .. } if hi - lo < std::f64::consts::TAU && lo >= -std::f64::consts::TAU && hi <= 2.0 * std::f64::consts::TAU => {
            let n = ((hi - lo) / std::f64::consts::FRAC_PI_4).ceil().max(1.0) as usize;
            (0..n).map(|i| curve_hull_box(curve, lo + (hi - lo) * i as f64 / n as f64, if i + 1 == n { hi } else { lo + (hi - lo) * (i + 1) as f64 / n as f64 })).collect()
        }
        _ => curve_hull_box(curve, lo, hi).map(|b| vec![b]),
    }
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
            // a full revolution bounded only by its meridian seam (u constant
            // along the loop, e.g. a revolved sphere): the face spans the period
            let (pu, pv) = face.surface.periods();
            if let Some(per) = pu {
                if uhi - ulo < 1e-9 * per {
                    uhi = ulo + per;
                }
            }
            if let Some(per) = pv {
                if vhi - vlo < 1e-9 * per {
                    vhi = vlo + per;
                }
            }
            let n = 12;
            let us: Vec<f64> = (0..=n).map(|i| ulo + (uhi - ulo) * i as f64 / n as f64).collect();
            let vs: Vec<f64> = (0..=n).map(|j| vlo + (vhi - vlo) * j as f64 / n as f64).collect();
            for p in face.surface.eval_grid(&us, &vs) {
                bb.add_point(p);
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

    #[test]
    fn hull_box_holds_the_curve() {
        use ck_geom::curve::Curve;
        let n = Vec3::new(0.3, -0.5, 0.8).normalize();
        let x = n.cross(Vec3::new(1.0, 0.2, 0.0)).normalize();
        let pts: Vec<Vec3> = (0..9).map(|i| { let t = i as f64; Vec3::new(t, (t * 1.3).sin() * 4.0, (t * 0.7).cos() * 3.0) }).collect();
        let spline = Curve::BSpline(ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).unwrap());
        let (s0, s1) = spline.range();
        let cases = [
            (Curve::Line { o: Vec3::new(1.0, 2.0, 3.0), d: n }, -2.0, 5.0),
            (Curve::circle_x(Vec3::new(1.0, -2.0, 0.5), n, x, 3.0), -1.0, 2.5),
            (Curve::circle_x(Vec3::new(1.0, -2.0, 0.5), n, x, 3.0), 4.0, 9.0),
            (Curve::circle_x(Vec3::new(0.0, 0.0, 0.0), n, x, 2.0), -20.0, 20.0),
            (Curve::ellipse(Vec3::new(2.0, 1.0, -1.0), n, x, 5.0, 2.0), 0.3, 5.9),
            (spline.clone(), s0, s1),
            (spline.clone(), s0 + 0.31 * (s1 - s0), s0 + 0.47 * (s1 - s0)),
        ];
        for (c, t0, t1) in cases {
            let bb = curve_hull_box(&c, t0, t1).unwrap().padded(1e-12);
            let spans: Vec<Aabb> = curve_span_boxes(&c, t0, t1).unwrap().iter().map(|b| b.padded(1e-12)).collect();
            for k in 0..=4000 {
                let p = c.eval(t0 + (t1 - t0) * k as f64 / 4000.0);
                assert!(bb.contains(p), "{} {t0} {t1}: {p:?} outside {bb:?}", c.kind());
                assert!(spans.iter().any(|b| b.contains(p)), "{} {t0} {t1}: {p:?} outside the span boxes", c.kind());
            }
        }
    }

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

    /// Oracle (instrumented f2660 / f2661, regen of a 60×60×5 front box cut
    /// throughBoth by five r1 circles at x = −25..15 step 10, y = 30 +
    /// 0.01·i): the two BVHs' item boxes as the original built them, its
    /// leaves and its pair order.
    #[test]
    fn oracle_build_and_pair_order() {
        // the boxes exactly as the original's dump printed them (SAH bins
        // are sensitive to the last bits)
        let bx = |v: &[f64; 6]| Aabb::new(Vec3::new(v[0], v[1], v[2]), Vec3::new(v[3], v[4], v[5]));
        let a: Vec<Aabb> = [
            [-30.000011, -0.000011, -0.000011, 30.000011, 60.000011, 0.000011],
            [-30.000011, -0.000011, 4.999989, 30.000011, 60.000011, 5.000011],
            [-30.000011, -0.000011, -0.000011, 30.000011, 0.000011, 5.000011],
            [29.999989, -0.000011, -0.000011, 30.000011, 60.000011, 5.000011],
            [-30.000011, 59.999989, -0.000011, 30.000011, 60.000011, 5.000011],
            [-30.000011, -0.000011, -0.000011, -29.999989, 60.000011, 5.000011],
        ].iter().map(bx).collect();
        let b: Vec<Aabb> = [
            [-26.000011, 28.999989, -125.10400038815723, -23.999989, 31.000011, -125.10397838815723],
            [-26.000011, 28.999989, 125.10397838815723, -23.999989, 31.000011, 125.10400038815723],
            [-26.000011, 28.999989, -125.10400038815723, -23.999989, 31.000011, 125.10400038815723],
            [-16.000011, 29.009989, -125.10400038815723, -13.999989000000001, 31.010011000000002, -125.10397838815723],
            [-16.000011, 29.009989, 125.10397838815723, -13.999989000000001, 31.010011000000002, 125.10400038815723],
            [-16.000011, 29.009989, -125.10400038815723, -13.999989000000001, 31.010011000000002, 125.10400038815723],
            [-6.000011, 29.019989, -125.10400038815723, -3.999989, 31.020011, -125.10397838815723],
            [-6.000011, 29.019989, 125.10397838815723, -3.999989, 31.020011, 125.10400038815723],
            [-6.000011, 29.019989, -125.10400038815723, -3.999989, 31.020011, 125.10400038815723],
            [3.999989, 29.029989, -125.10400038815723, 6.000011, 31.030011000000002, -125.10397838815723],
            [3.999989, 29.029989, 125.10397838815723, 6.000011, 31.030011000000002, 125.10400038815723],
            [3.999989, 29.029989, -125.10400038815723, 6.000011, 31.030011000000002, 125.10400038815723],
            [13.999989000000001, 29.039989, -125.10400038815723, 16.000011, 31.040011, -125.10397838815723],
            [13.999989000000001, 29.039989, 125.10397838815723, 16.000011, 31.040011, 125.10400038815723],
            [13.999989000000001, 29.039989, -125.10400038815723, 16.000011, 31.040011, 125.10400038815723],
        ].iter().map(bx).collect();
        let (ta, tb) = (Bvh::build(&a), Bvh::build(&b));
        let leaves = |t: &Bvh| -> Vec<Vec<usize>> {
            let mut out = vec![];
            let mut st = vec![0usize];
            while let Some(n) = st.pop() {
                let nd = &t.nodes[n];
                if nd.left == usize::MAX {
                    out.push(t.items[nd.start..nd.start + nd.count].to_vec());
                } else {
                    st.push(nd.right);
                    st.push(nd.left);
                }
            }
            out
        };
        assert_eq!(leaves(&ta), vec![vec![5], vec![2], vec![4, 1, 0, 3]]);
        assert_eq!(leaves(&tb), vec![vec![0, 6, 3], vec![9, 12], vec![5, 2, 8], vec![11, 14], vec![7, 1, 4], vec![13, 10]]);
        assert_eq!(
            ta.overlap_pairs_ordered(&tb, 1e-6),
            vec![(1, 11), (1, 14), (0, 11), (0, 14), (1, 5), (1, 2), (1, 8), (0, 5), (0, 2), (0, 8)]
        );
    }

    #[test]
    fn ray_tri() {
        let t = ray_triangle(Vec3::new(0.2, 0.2, 5.0), -Vec3::Z, Vec3::ZERO, Vec3::X, Vec3::Y).unwrap();
        assert!((t - 5.0).abs() < 1e-15);
        assert!(ray_triangle(Vec3::new(0.8, 0.8, 5.0), -Vec3::Z, Vec3::ZERO, Vec3::X, Vec3::Y).is_none());
    }

}
