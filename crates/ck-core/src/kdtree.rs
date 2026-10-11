//! 3D kd-tree spatial index (ck-core/src/kdtree.rs).
//!
//! The path is pinned by rs_paths.txt / a panic location in kernel.strings
//! (next to ck-ops boolean/assemble.rs and ck-topo make.rs — i.e. used for
//! vertex/edge merging during boolean assembly and body building). The
//! original's internal layout is not recovered; this is a clean
//! implementation: implicit balanced tree over a permutation of point
//! indices, split on the axis of largest extent at the median, small
//! leaves scanned linearly.
//!
//! Query results are point indices into the slice the tree was built from.
//! Ties are broken by lower index so results are deterministic.

use crate::linalg::Vec3;

const LEAF: usize = 8;

#[derive(Debug, Clone)]
struct Node {
    /// range into `perm`
    lo: u32,
    hi: u32,
    /// split axis (0..3), or 3 for a leaf
    axis: u8,
    split: f64,
    /// child node indices (left = coords <= split)
    left: u32,
    right: u32,
}

#[derive(Debug, Clone)]
pub struct KdTree {
    pts: Vec<Vec3>,
    perm: Vec<u32>,
    nodes: Vec<Node>,
}

impl KdTree {
    /// Build from points (copied). O(n log n).
    pub fn build(points: &[Vec3]) -> KdTree {
        let mut t = KdTree { pts: points.to_vec(), perm: (0..points.len() as u32).collect(), nodes: Vec::new() };
        if !points.is_empty() {
            t.build_rec(0, points.len());
        }
        t
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    pub fn point(&self, i: usize) -> Vec3 {
        self.pts[i]
    }

    pub fn points(&self) -> &[Vec3] {
        &self.pts
    }

    fn build_rec(&mut self, lo: usize, hi: usize) -> u32 {
        let id = self.nodes.len() as u32;
        self.nodes.push(Node { lo: lo as u32, hi: hi as u32, axis: 3, split: 0.0, left: 0, right: 0 });
        if hi - lo <= LEAF {
            return id;
        }
        let (mut mn, mut mx) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &i in &self.perm[lo..hi] {
            mn = mn.min(self.pts[i as usize]);
            mx = mx.max(self.pts[i as usize]);
        }
        let ext = mx - mn;
        let axis = if ext.x >= ext.y && ext.x >= ext.z {
            0
        } else if ext.y >= ext.z {
            1
        } else {
            2
        };
        if !(ext.get(axis) > 0.0) {
            return id; // all coincident (or NaN): keep as a leaf
        }
        let mid = (lo + hi) / 2;
        let pts = &self.pts;
        self.perm[lo..hi].select_nth_unstable_by(mid - lo, |&a, &b| {
            pts[a as usize]
                .get(axis)
                .partial_cmp(&pts[b as usize].get(axis))
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(&b))
        });
        let split = self.pts[self.perm[mid] as usize].get(axis);
        let l = self.build_rec(lo, mid);
        let r = self.build_rec(mid, hi);
        let n = &mut self.nodes[id as usize];
        n.axis = axis as u8;
        n.split = split;
        n.left = l;
        n.right = r;
        id
    }

    /// Nearest point to `q`: `(index, distance)`. `None` when empty.
    pub fn nearest(&self, q: Vec3) -> Option<(usize, f64)> {
        let r = self.k_nearest(q, 1);
        r.first().copied()
    }

    /// The `k` nearest points, ascending by distance: `(index, distance)`.
    pub fn k_nearest(&self, q: Vec3, k: usize) -> Vec<(usize, f64)> {
        let mut best: Vec<(f64, u32)> = Vec::with_capacity(k + 1);
        if k == 0 || self.nodes.is_empty() {
            return Vec::new();
        }
        self.knn_rec(0, q, k, &mut best);
        best.into_iter().map(|(d2, i)| (i as usize, d2.sqrt())).collect()
    }

    fn knn_rec(&self, ni: u32, q: Vec3, k: usize, best: &mut Vec<(f64, u32)>) {
        let n = &self.nodes[ni as usize];
        if n.axis == 3 {
            for &i in &self.perm[n.lo as usize..n.hi as usize] {
                let d2 = self.pts[i as usize].dist2(q);
                if best.len() == k {
                    let w = best[k - 1];
                    if d2 > w.0 || (d2 == w.0 && i > w.1) {
                        continue;
                    }
                }
                let pos = best.partition_point(|&(bd, bi)| bd < d2 || (bd == d2 && bi < i));
                best.insert(pos, (d2, i));
                best.truncate(k);
            }
            return;
        }
        let d = q.get(n.axis as usize) - n.split;
        let (near, far) = if d <= 0.0 { (n.left, n.right) } else { (n.right, n.left) };
        self.knn_rec(near, q, k, best);
        if best.len() < k || d * d <= best[best.len() - 1].0 {
            self.knn_rec(far, q, k, best);
        }
    }

    /// All points with `|p − q| <= r`, ascending by distance (ties by index).
    pub fn within(&self, q: Vec3, r: f64) -> Vec<(usize, f64)> {
        let mut out: Vec<(f64, u32)> = Vec::new();
        if !self.nodes.is_empty() && r >= 0.0 {
            self.radius_rec(0, q, r * r, &mut out);
        }
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then(a.1.cmp(&b.1)));
        out.into_iter().map(|(d2, i)| (i as usize, d2.sqrt())).collect()
    }

    fn radius_rec(&self, ni: u32, q: Vec3, r2: f64, out: &mut Vec<(f64, u32)>) {
        let n = &self.nodes[ni as usize];
        if n.axis == 3 {
            for &i in &self.perm[n.lo as usize..n.hi as usize] {
                let d2 = self.pts[i as usize].dist2(q);
                if d2 <= r2 {
                    out.push((d2, i));
                }
            }
            return;
        }
        let d = q.get(n.axis as usize) - n.split;
        // left holds coords <= split, right holds coords >= split
        if d <= 0.0 || d * d <= r2 {
            self.radius_rec(n.left, q, r2, out);
        }
        if d >= 0.0 || d * d <= r2 {
            self.radius_rec(n.right, q, r2, out);
        }
    }

    /// Merge map for coincident points (vertex welding): `rep[i]` is the
    /// smallest index `j` whose point lies within `tol` of point `i`'s
    /// representative chain (greedy, in index order).
    pub fn weld(&self, tol: f64) -> Vec<usize> {
        let mut rep: Vec<usize> = (0..self.pts.len()).collect();
        for i in 0..self.pts.len() {
            if rep[i] != i {
                continue;
            }
            for (j, _) in self.within(self.pts[i], tol) {
                if j > i && rep[j] == j {
                    rep[j] = i;
                }
            }
        }
        rep
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(n: usize, seed: u64) -> Vec<Vec3> {
        let mut s = seed;
        let mut r = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (s >> 11) as f64 / (1u64 << 53) as f64 * 100.0 - 50.0
        };
        (0..n).map(|_| Vec3::new(r(), r(), r())).collect()
    }

    fn brute(p: &[Vec3], q: Vec3) -> Vec<(usize, f64)> {
        let mut v: Vec<(usize, f64)> = p.iter().enumerate().map(|(i, x)| (i, x.dist2(q))).collect();
        v.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap().then(a.0.cmp(&b.0)));
        v.into_iter().map(|(i, d)| (i, d.sqrt())).collect()
    }

    #[test]
    fn empty_and_single() {
        let t = KdTree::build(&[]);
        assert!(t.nearest(Vec3::ZERO).is_none());
        assert!(t.within(Vec3::ZERO, 1.0).is_empty());
        let t = KdTree::build(&[Vec3::X]);
        assert_eq!(t.nearest(Vec3::ZERO), Some((0, 1.0)));
    }

    #[test]
    fn matches_brute_force() {
        let p = cloud(2000, 7);
        let t = KdTree::build(&p);
        for q in cloud(200, 99) {
            let b = brute(&p, q);
            assert_eq!(t.nearest(q).unwrap().0, b[0].0);
            let k = t.k_nearest(q, 10);
            assert_eq!(k.iter().map(|x| x.0).collect::<Vec<_>>(), b[..10].iter().map(|x| x.0).collect::<Vec<_>>());
            let w = t.within(q, 12.0);
            let bw: Vec<usize> = b.iter().filter(|x| x.1 <= 12.0).map(|x| x.0).collect();
            assert_eq!(w.iter().map(|x| x.0).collect::<Vec<_>>(), bw);
        }
        assert_eq!(t.k_nearest(Vec3::ZERO, 5000).len(), 2000);
    }

    #[test]
    fn duplicates_and_weld() {
        let mut p = vec![Vec3::ZERO; 20];
        p.push(Vec3::new(1e-10, 0.0, 0.0));
        p.push(Vec3::new(5.0, 0.0, 0.0));
        p.push(Vec3::new(5.0, 1e-12, 0.0));
        let t = KdTree::build(&p);
        assert_eq!(t.within(Vec3::ZERO, 1e-9).len(), 21);
        let rep = t.weld(1e-9);
        assert!(rep[..21].iter().all(|&r| r == 0));
        assert_eq!(rep[21], 21);
        assert_eq!(rep[22], 21);
        // nearest tie → lowest index
        assert_eq!(t.nearest(Vec3::new(0.0, 0.0, 1.0)).unwrap().0, 0);
    }
}
