//! Constrained Delaunay triangulation in 2D (ck-tess/src/cdt.rs).
//!
//! Ported from the author's own zero-dep mesher in sectionprops-rs
//! (`src/mesher/cdt.rs` + `predicates.rs`, MIT), which is "written from the
//! technique" (Lawson flips, Sloan segment recovery, Shewchuk-style
//! adaptive predicates).
//!
//! Evidence the original used the same predicate family: kernel.wasm
//! functions f3300 (orient2d) and f3311 (incircle) inside the ck-tess range
//! carry the error-bound constants 3.3306690738754716e-16 (ccwerrboundA)
//! and 1.1102230246251577e-15 (iccerrboundA).
//!
//! On top of the ported core: [`triangulate`] (points + constrained
//! segments -> classified triangulation, inside = odd constrained-crossing
//! depth from the super triangle) and interior Steiner insertion used by
//! face refinement.
//!
//! A constrained Delaunay triangulation with local edit operations, the data structure under the mesher.
//!
//! Triangles are counter-clockwise with `n[k]` the neighbour across the edge opposite vertex `k` (that edge runs from `v[(k+1)%3]` to
//! `v[(k+2)%3]`) and `c[k]` marking that edge as a constrained segment. Vertices 0..3 are the corners of an enclosing super-triangle so every
//! real vertex is interior and every real edge has two triangles. Points go in by splitting a triangle or an edge and then restoring the
//! Delaunay property with Lawson flips; a segment goes in by flipping the edges it crosses (S. W. Sloan, "A fast algorithm for generating
//! constrained Delaunay triangulations", 1993) and is then never flipped. All sign decisions use exact predicates.
#![allow(dead_code)]

/// Exact-sign predicates (orient2d / incircle).
pub mod predicates {
    //! Exact-sign geometric predicates: `orient2d` and `incircle`.
    //!
    //! Each tries a plain floating-point evaluation first and accepts it when its magnitude clears a forward error bound; otherwise it redoes the
    //! determinant exactly with floating-point expansions (a value is a sum of non-overlapping doubles, so sums and products of doubles are
    //! exact; the sign of the whole is the sign of its largest component). The error bounds are the standard ones for these two determinants
    //! (from the analysis in J. R. Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates", 1997);
    //! the code below is written from the technique, not copied from the published implementation.
    //!
    //! WHY exact: library sections are rectangles and arcs, so cocircular and collinear point sets are the common case, not the corner case.
    //! An inconsistent in-circle answer on cocircular points can make edge flipping loop; an exact answer cannot.

    type Pt = [f64; 2];
    const EPS: f64 = 1.1102230246251565e-16; // 2^-53

    #[inline]
    fn two_sum(a: f64, b: f64) -> (f64, f64) {
        let x = a + b;
        let bv = x - a;
        let av = x - bv;
        (x, (a - av) + (b - bv))
    }
    #[inline]
    fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
        let x = a + b;
        (x, b - (x - a))
    }
    #[inline]
    fn two_diff(a: f64, b: f64) -> (f64, f64) { two_sum(a, -b) }
    #[inline]
    fn two_prod(a: f64, b: f64) -> (f64, f64) {
        let x = a * b;
        (x, a.mul_add(b, -x))
    }

    /// An expansion: non-overlapping components in increasing magnitude.
    type Exp = Vec<f64>;

    fn exp_from2(x: f64, y: f64) -> Exp { if y == 0.0 { vec![x] } else { vec![y, x] } }

    fn grow(e: &[f64], b: f64) -> Exp {
        let mut h = Vec::with_capacity(e.len() + 1);
        let mut q = b;
        for &c in e {
            let (s, t) = two_sum(q, c);
            q = s;
            if t != 0.0 { h.push(t); }
        }
        if q != 0.0 || h.is_empty() { h.push(q); }
        h
    }

    fn add(e: &[f64], f: &[f64]) -> Exp {
        let mut r = e.to_vec();
        for &c in f { r = grow(&r, c); }
        r
    }

    fn neg(e: &[f64]) -> Exp { e.iter().map(|x| -x).collect() }

    fn scale(e: &[f64], b: f64) -> Exp {
        let mut h = Vec::with_capacity(2 * e.len());
        let (mut q, hh) = two_prod(e[0], b);
        if hh != 0.0 { h.push(hh); }
        for &c in &e[1..] {
            let (t1, t0) = two_prod(c, b);
            let (s, hh) = two_sum(q, t0);
            if hh != 0.0 { h.push(hh); }
            let (q2, hh2) = fast_two_sum(t1, s);
            if hh2 != 0.0 { h.push(hh2); }
            q = q2;
        }
        if q != 0.0 || h.is_empty() { h.push(q); }
        h
    }

    fn mul(e: &[f64], f: &[f64]) -> Exp {
        let mut r: Exp = vec![0.0];
        for &c in f { r = add(&r, &scale(e, c)); }
        r
    }

    fn sign(e: &[f64]) -> f64 {
        for &c in e.iter().rev() { if c != 0.0 { return c; } }
        0.0
    }

    /// Positive when `a`, `b`, `c` turn counter-clockwise, negative clockwise, zero exactly collinear.
    pub fn orient2d(a: Pt, b: Pt, c: Pt) -> f64 {
        let l = (a[0] - c[0]) * (b[1] - c[1]);
        let r = (a[1] - c[1]) * (b[0] - c[0]);
        let det = l - r;
        let bound = 3.3306690738754716e-16 * (l.abs() + r.abs());
        if det > bound || -det > bound { return det; }
        if l == 0.0 && r == 0.0 { return 0.0; }
        let acx = exp_from2_diff(a[0], c[0]);
        let acy = exp_from2_diff(a[1], c[1]);
        let bcx = exp_from2_diff(b[0], c[0]);
        let bcy = exp_from2_diff(b[1], c[1]);
        let d = add(&mul(&acx, &bcy), &neg(&mul(&acy, &bcx)));
        sign(&d)
    }

    fn exp_from2_diff(a: f64, b: f64) -> Exp { let (x, y) = two_diff(a, b); exp_from2(x, y) }

    /// Positive when `d` lies strictly inside the circle through counter-clockwise `a`, `b`, `c`, negative outside, zero exactly on it.
    pub fn incircle(a: Pt, b: Pt, c: Pt, d: Pt) -> f64 {
        let (adx, ady) = (a[0] - d[0], a[1] - d[1]);
        let (bdx, bdy) = (b[0] - d[0], b[1] - d[1]);
        let (cdx, cdy) = (c[0] - d[0], c[1] - d[1]);
        let (bdxcdy, cdxbdy) = (bdx * cdy, cdx * bdy);
        let alift = adx * adx + ady * ady;
        let (cdxady, adxcdy) = (cdx * ady, adx * cdy);
        let blift = bdx * bdx + bdy * bdy;
        let (adxbdy, bdxady) = (adx * bdy, bdx * ady);
        let clift = cdx * cdx + cdy * cdy;
        let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
        let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift + (cdxady.abs() + adxcdy.abs()) * blift + (adxbdy.abs() + bdxady.abs()) * clift;
        let bound = (10.0 + 96.0 * EPS) * EPS * permanent;
        if det > bound || -det > bound { return det; }
        // exact
        let ex = |p: f64, q: f64| exp_from2_diff(p, q);
        let (adx, ady) = (ex(a[0], d[0]), ex(a[1], d[1]));
        let (bdx, bdy) = (ex(b[0], d[0]), ex(b[1], d[1]));
        let (cdx, cdy) = (ex(c[0], d[0]), ex(c[1], d[1]));
        let lift = |x: &Exp, y: &Exp| add(&mul(x, x), &mul(y, y));
        let (al, bl, cl) = (lift(&adx, &ady), lift(&bdx, &bdy), lift(&cdx, &cdy));
        let m1 = add(&mul(&bdx, &cdy), &neg(&mul(&cdx, &bdy)));
        let m2 = add(&mul(&cdx, &ady), &neg(&mul(&adx, &cdy)));
        let m3 = add(&mul(&adx, &bdy), &neg(&mul(&bdx, &ady)));
        let t = add(&add(&mul(&al, &m1), &mul(&bl, &m2)), &mul(&cl, &m3));
        sign(&t)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        #[test]
        fn exact_on_degenerate_inputs() {
            // collinear points that a naive evaluation misjudges
            let a = [0.1, 0.1];
            let b = [0.3, 0.3];
            let c = [0.7, 0.7];
            assert_eq!(orient2d(a, b, c), 0.0);
            // cocircular: unit-circle points
            assert_eq!(incircle([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]), 0.0);
            assert!(incircle([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -0.999]) > 0.0);
            assert!(incircle([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.001]) < 0.0);
            // nearly collinear with a tiny but nonzero turn
            assert!(orient2d([0.0, 0.0], [1e15, 1e15], [2e15, 2e15 + 0.25]) > 0.0);
        }
    }

}

use self::predicates::{incircle, orient2d};
use std::collections::VecDeque;

pub type Pt = [f64; 2];
pub const NONE: u32 = u32::MAX;
/// Region value of a triangle not yet classified.
pub const UNSET: i32 = -2;

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub v: [u32; 3],
    pub n: [u32; 3],
    pub c: [bool; 3],
    pub region: i32,
    pub alive: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Loc {
    /// Strictly inside the triangle.
    In(u32),
    /// On edge `k` of the triangle.
    Edge(u32, usize),
    /// On vertex index `k` of the triangle.
    Vertex(u32, usize),
    /// The walk toward the point would cross constrained edge `k` of the triangle.
    Blocked(u32, usize),
}

pub struct Cdt {
    pub pts: Vec<Pt>,
    pub tris: Vec<Tri>,
    pub vt: Vec<u32>,
    free: Vec<u32>,
    /// Triangles created or changed since the caller last drained this list.
    pub touched: Vec<u32>,
    rng: u64,
    /// Squared cosine of the apex angle that bounds the diametral lens (see `encroaches`).
    pub lens: f64,
}

fn dot(a: Pt, b: Pt, c: Pt) -> f64 { (a[0] - c[0]) * (b[0] - c[0]) + (a[1] - c[1]) * (b[1] - c[1]) }

impl Cdt {
    /// An empty triangulation whose super-triangle encloses `bounds = (min_x, min_y, max_x, max_y)` with a wide margin.
    pub fn new(bounds: (f64, f64, f64, f64)) -> Cdt {
        let (cx, cy) = (0.5 * (bounds.0 + bounds.2), 0.5 * (bounds.1 + bounds.3));
        let r = 100.0 * (bounds.2 - bounds.0).max(bounds.3 - bounds.1).max(1e-12);
        let pts = vec![[cx - 2.0 * r, cy - r], [cx + 2.0 * r, cy - r], [cx, cy + 2.0 * r]];
        let t = Tri { v: [0, 1, 2], n: [NONE; 3], c: [false; 3], region: UNSET, alive: true };
        Cdt { pts, tris: vec![t], vt: vec![0, 0, 0], free: vec![], touched: vec![], rng: 0x9E3779B97F4A7C15, lens: 0.0 }
    }

    pub fn is_super(&self, v: u32) -> bool { v < 3 }

    fn rand(&mut self) -> usize {
        self.rng ^= self.rng << 13; self.rng ^= self.rng >> 7; self.rng ^= self.rng << 17;
        (self.rng >> 33) as usize
    }

    fn alloc(&mut self, t: Tri) -> u32 {
        if let Some(i) = self.free.pop() { self.tris[i as usize] = t; i } else { self.tris.push(t); (self.tris.len() - 1) as u32 }
    }

    pub fn touch(&mut self, t: u32) { self.touched.push(t); }

    pub fn orient(&self, a: u32, b: u32, c: u32) -> f64 { orient2d(self.pts[a as usize], self.pts[b as usize], self.pts[c as usize]) }

    /// Makes the neighbour's pointer for the edge `(a, b)` of a new triangle (seen from the neighbour as `(b, a)`) point at `t`.
    fn fix_back(&mut self, nb: u32, a: u32, b: u32, t: u32) {
        if nb == NONE { return; }
        let tr = &mut self.tris[nb as usize];
        for i in 0..3 {
            if tr.v[(i + 1) % 3] == b && tr.v[(i + 2) % 3] == a { tr.n[i] = t; return; }
        }
        panic!("fix_back: edge not found");
    }

    /// Index of the edge `(a, b)` (in that direction) in triangle `t`.
    fn edge_idx(&self, t: u32, a: u32, b: u32) -> Option<usize> {
        let tr = &self.tris[t as usize];
        (0..3).find(|&i| tr.v[(i + 1) % 3] == a && tr.v[(i + 2) % 3] == b)
    }

    /// The triangles around vertex `v` in counter-clockwise order, each with the index of `v` in it.
    pub fn fan(&self, v: u32) -> Vec<(u32, usize)> {
        let t0 = self.vt[v as usize];
        let mut out = vec![];
        let mut t = t0;
        loop {
            let tr = &self.tris[t as usize];
            let k = (0..3).find(|&i| tr.v[i] == v).expect("vt points at a triangle without the vertex");
            out.push((t, k));
            let nx = tr.n[(k + 1) % 3];
            if nx == NONE || nx == t0 { break; }
            t = nx;
            if out.len() > self.tris.len() { panic!("fan loops"); }
        }
        out
    }

    /// The triangle holding the edge between `u` and `v` (either direction) and the index of that edge in it.
    pub fn find_edge(&self, u: u32, v: u32) -> Option<(u32, usize)> {
        for (t, k) in self.fan(u) {
            let tr = &self.tris[t as usize];
            if tr.v[(k + 1) % 3] == v { return Some((t, (k + 2) % 3)); }
            if tr.v[(k + 2) % 3] == v { return Some((t, (k + 1) % 3)); }
        }
        None
    }

    fn mark_edge(&mut self, t: u32, k: usize) {
        self.tris[t as usize].c[k] = true;
        let tr = self.tris[t as usize];
        let nb = tr.n[k];
        if nb != NONE {
            let (a, b) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            let j = self.edge_idx(nb, b, a).unwrap();
            self.tris[nb as usize].c[j] = true;
        }
    }

    // ------------------------------------------------------------------------------------------------------------------------------
    // point location
    // ------------------------------------------------------------------------------------------------------------------------------

    /// Walks from `start` to the triangle containing `p`. With `stop_at_constrained` the walk reports `Blocked` instead of crossing a segment.
    pub fn locate(&mut self, p: Pt, start: u32, stop_at_constrained: bool) -> Loc {
        let mut t = if (start as usize) < self.tris.len() && self.tris[start as usize].alive { start } else { self.any_alive() };
        let mut steps = 0usize;
        loop {
            steps += 1;
            if steps > 3 * self.tris.len() + 64 {
                return self.locate_brute(p);
            }
            let tr = self.tris[t as usize];
            let k0 = self.rand() % 3;
            let mut moved = false;
            let mut zeros = [0usize; 3];
            let mut nz = 0;
            for dk in 0..3 {
                let k = (k0 + dk) % 3;
                let (a, b) = (self.pts[tr.v[(k + 1) % 3] as usize], self.pts[tr.v[(k + 2) % 3] as usize]);
                let o = orient2d(a, b, p);
                if o < 0.0 {
                    if stop_at_constrained && tr.c[k] { return Loc::Blocked(t, k); }
                    let nb = tr.n[k];
                    if nb == NONE { return Loc::In(t); }
                    t = nb;
                    moved = true;
                    break;
                } else if o == 0.0 {
                    zeros[nz] = k;
                    nz += 1;
                }
            }
            if moved { continue; }
            return match nz {
                0 => Loc::In(t),
                1 => Loc::Edge(t, zeros[0]),
                _ => Loc::Vertex(t, 3 - zeros[0] - zeros[1]),
            };
        }
    }

    fn any_alive(&self) -> u32 { self.tris.iter().position(|t| t.alive).unwrap() as u32 }

    fn locate_brute(&self, p: Pt) -> Loc {
        for (i, tr) in self.tris.iter().enumerate() {
            if !tr.alive { continue; }
            let mut zeros = vec![];
            let mut ok = true;
            for k in 0..3 {
                let o = orient2d(self.pts[tr.v[(k + 1) % 3] as usize], self.pts[tr.v[(k + 2) % 3] as usize], p);
                if o < 0.0 { ok = false; break; }
                if o == 0.0 { zeros.push(k); }
            }
            if ok {
                return match zeros.len() { 0 => Loc::In(i as u32), 1 => Loc::Edge(i as u32, zeros[0]), _ => Loc::Vertex(i as u32, 3 - zeros[0] - zeros[1]) };
            }
        }
        Loc::In(self.any_alive())
    }

    // ------------------------------------------------------------------------------------------------------------------------------
    // edits
    // ------------------------------------------------------------------------------------------------------------------------------

    /// Swaps the diagonal of the quadrilateral across edge `k` of triangle `t`. The caller guarantees the quadrilateral is convex.
    fn flip(&mut self, t: u32, k: usize) {
        let tr = self.tris[t as usize];
        let (w, u, v) = (tr.v[k], tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
        let s = tr.n[k];
        let ts = self.tris[s as usize];
        let j = self.edge_idx(s, v, u).unwrap();
        let x = ts.v[j];
        let (a_n, a_c) = (tr.n[(k + 2) % 3], tr.c[(k + 2) % 3]); // across (w,u)
        let (b_n, b_c) = (tr.n[(k + 1) % 3], tr.c[(k + 1) % 3]); // across (v,w)
        let (c_n, c_c) = (ts.n[(j + 1) % 3], ts.c[(j + 1) % 3]); // across (u,x)
        let (d_n, d_c) = (ts.n[(j + 2) % 3], ts.c[(j + 2) % 3]); // across (x,v)
        let region = tr.region;
        self.tris[t as usize] = Tri { v: [w, u, x], n: [c_n, s, a_n], c: [c_c, false, a_c], region, alive: true };
        self.tris[s as usize] = Tri { v: [w, x, v], n: [d_n, b_n, t], c: [d_c, b_c, false], region, alive: true };
        self.fix_back(c_n, u, x, t); // (u,x) now belongs to t
        self.fix_back(b_n, v, w, s); // (v,w) now belongs to s
        self.vt[w as usize] = t; self.vt[u as usize] = t; self.vt[x as usize] = t; self.vt[v as usize] = s;
        self.touch(t); self.touch(s);
    }

    /// Restores the Delaunay property around the listed edges, flipping non-constrained edges whose opposite vertex is inside the circle.
    fn legalize(&mut self, mut stack: Vec<(u32, usize)>) {
        while let Some((t, k)) = stack.pop() {
            let tr = self.tris[t as usize];
            if !tr.alive || tr.c[k] { continue; }
            let s = tr.n[k];
            if s == NONE { continue; }
            let (u, v) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            let j = self.edge_idx(s, v, u).unwrap();
            let x = self.tris[s as usize].v[j];
            let inside = incircle(self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize], self.pts[x as usize]);
            if inside > 0.0 {
                self.flip(t, k);
                // t = (w,u,x), s = (w,x,v): the four outer edges
                stack.push((t, 2)); stack.push((t, 0)); stack.push((s, 0)); stack.push((s, 1));
            }
        }
    }

    /// Inserts the point strictly inside triangle `t`; returns the new vertex index.
    pub fn insert_in_tri(&mut self, t: u32, p: Pt) -> u32 {
        let pi = self.pts.len() as u32;
        self.pts.push(p);
        self.vt.push(0);
        let tr = self.tris[t as usize];
        let (a, b, c) = (tr.v[0], tr.v[1], tr.v[2]);
        let (n0, n1, n2) = (tr.n[0], tr.n[1], tr.n[2]);
        let (c0, c1, c2) = (tr.c[0], tr.c[1], tr.c[2]);
        let r = tr.region;
        let t1 = self.alloc(Tri { v: [b, c, pi], n: [NONE; 3], c: [false; 3], region: r, alive: true });
        let t2 = self.alloc(Tri { v: [c, a, pi], n: [NONE; 3], c: [false; 3], region: r, alive: true });
        self.tris[t as usize] = Tri { v: [a, b, pi], n: [t1, t2, n2], c: [false, false, c2], region: r, alive: true };
        self.tris[t1 as usize] = Tri { v: [b, c, pi], n: [t2, t, n0], c: [false, false, c0], region: r, alive: true };
        self.tris[t2 as usize] = Tri { v: [c, a, pi], n: [t, t1, n1], c: [false, false, c1], region: r, alive: true };
        self.fix_back(n0, b, c, t1);
        self.fix_back(n1, c, a, t2);
        self.vt[a as usize] = t; self.vt[b as usize] = t; self.vt[c as usize] = t1; self.vt[pi as usize] = t;
        self.touch(t); self.touch(t1); self.touch(t2);
        self.legalize(vec![(t, 2), (t1, 2), (t2, 2)]);
        pi
    }

    /// Splits edge `k` of triangle `t` (and the triangle across it) at `p`, which is taken to lie on the edge whether or not the floating
    /// point coordinates are exactly on it. A constrained edge becomes two constrained edges.
    pub fn insert_on_edge(&mut self, t: u32, k: usize, p: Pt) -> u32 {
        let pi = self.pts.len() as u32;
        self.pts.push(p);
        self.vt.push(0);
        let tr = self.tris[t as usize];
        let (w, u, v) = (tr.v[k], tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
        let s = tr.n[k];
        assert!(s != NONE, "edge split on the hull");
        let ts = self.tris[s as usize];
        let j = self.edge_idx(s, v, u).unwrap();
        let x = ts.v[j];
        let c_uv = tr.c[k];
        let (au_n, au_c) = (tr.n[(k + 2) % 3], tr.c[(k + 2) % 3]); // across (w,u)
        let (bv_n, bv_c) = (tr.n[(k + 1) % 3], tr.c[(k + 1) % 3]); // across (v,w)
        let (dxv_n, dxv_c) = (ts.n[(j + 2) % 3], ts.c[(j + 2) % 3]); // across (x,v)
        let (cux_n, cux_c) = (ts.n[(j + 1) % 3], ts.c[(j + 1) % 3]); // across (u,x)
        let (rt, rs) = (tr.region, ts.region);
        let t2 = self.alloc(Tri { v: [w, pi, v], n: [NONE; 3], c: [false; 3], region: rt, alive: true });
        let s2 = self.alloc(Tri { v: [x, pi, u], n: [NONE; 3], c: [false; 3], region: rs, alive: true });
        let t1 = t;
        let s1 = s;
        self.tris[t1 as usize] = Tri { v: [w, u, pi], n: [s2, t2, au_n], c: [c_uv, false, au_c], region: rt, alive: true };
        self.tris[t2 as usize] = Tri { v: [w, pi, v], n: [s1, bv_n, t1], c: [c_uv, bv_c, false], region: rt, alive: true };
        self.tris[s1 as usize] = Tri { v: [x, v, pi], n: [t2, s2, dxv_n], c: [c_uv, false, dxv_c], region: rs, alive: true };
        self.tris[s2 as usize] = Tri { v: [x, pi, u], n: [t1, cux_n, s1], c: [c_uv, cux_c, false], region: rs, alive: true };
        // note: edge indices above follow "k is opposite vertex k": t1 = (w,u,p): opp w = (u,p); t2 = (w,p,v): opp w = (p,v);
        // s1 = (x,v,p): opp x = (v,p); s2 = (x,p,u): opp x = (p,u)
        self.fix_back(bv_n, v, w, t2);
        self.fix_back(cux_n, u, x, s2);
        self.vt[w as usize] = t1; self.vt[u as usize] = t1; self.vt[v as usize] = t2; self.vt[x as usize] = s1; self.vt[pi as usize] = t1;
        self.touch(t1); self.touch(t2); self.touch(s1); self.touch(s2);
        self.legalize(vec![(t1, 2), (t2, 1), (s1, 2), (s2, 1)]);
        pi
    }

    /// Inserts a point wherever it falls (no constraints involved); returns its vertex index, or the existing vertex it coincides with.
    pub fn insert_point(&mut self, p: Pt, hint: u32) -> u32 {
        match self.locate(p, hint, false) {
            Loc::In(t) => self.insert_in_tri(t, p),
            Loc::Edge(t, k) => self.insert_on_edge(t, k, p),
            Loc::Vertex(t, k) => self.tris[t as usize].v[k],
            Loc::Blocked(..) => unreachable!(),
        }
    }

    /// Makes the segment `a`-`b` an edge of the triangulation and marks it constrained. A vertex lying exactly on the open segment splits it
    /// there. Errors when another constrained segment crosses it.
    pub fn insert_segment(&mut self, a: u32, b: u32) -> Result<(), String> {
        if a == b { return Ok(()); }
        if let Some((t, k)) = self.find_edge(a, b) {
            self.mark_edge(t, k);
            return Ok(());
        }
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        let dir = [pb[0] - pa[0], pb[1] - pa[1]];
        // the wedge at `a` that holds the direction to `b`
        let mut start = None;
        for (t, k) in self.fan(a) {
            let tr = self.tris[t as usize];
            let (p, q) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            let o1 = self.orient(a, p, b);
            let o2 = self.orient(a, q, b);
            let along = |m: u32| { let pm = self.pts[m as usize]; (pm[0] - pa[0]) * dir[0] + (pm[1] - pa[1]) * dir[1] > 0.0 };
            if o1 == 0.0 && along(p) && !self.is_super(p) { self.insert_segment(a, p)?; return self.insert_segment(p, b); }
            if o2 == 0.0 && along(q) && !self.is_super(q) { self.insert_segment(a, q)?; return self.insert_segment(q, b); }
            if o1 > 0.0 && o2 < 0.0 { start = Some((t, k)); break; }
        }
        let (t, k) = start.ok_or_else(|| "no wedge toward segment end".to_string())?;
        // collect the crossed edges, each as (right, left) of the directed segment
        let mut crossed: Vec<(u32, u32)> = vec![];
        let tr = self.tris[t as usize];
        let (mut right, mut left) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
        let mut cur = t;
        let mut cur_edge = k; // edge opposite `a`
        loop {
            let trc = self.tris[cur as usize];
            if trc.c[cur_edge] { return Err("segments cross".into()); }
            crossed.push((right, left));
            let nb = trc.n[cur_edge];
            let j = self.edge_idx(nb, left, right).unwrap();
            let r = self.tris[nb as usize].v[j];
            if r == b { break; }
            let s = self.orient(a, b, r);
            if s == 0.0 {
                // a vertex exactly on the segment: split there
                if self.is_super(r) { return Err("super vertex on segment".into()); }
                // the already collected crossing is stale once we split, recover each half afresh
                self.insert_segment(a, r)?;
                return self.insert_segment(r, b);
            }
            if s > 0.0 {
                left = r;
                cur_edge = (j + 1) % 3; // edge (right, r) in nb: nb = (r, left_old... ) see below
                // nb vertices: [.., ..] with v[j] = r, v[(j+1)%3] = left_old, v[(j+2)%3] = right: edge (right, r) is opposite left_old = (j+1)%3
            } else {
                right = r;
                cur_edge = (j + 2) % 3; // edge (r, left) is opposite right_old = (j+2)%3
            }
            cur = nb;
        }
        // flip the crossed edges out
        let mut queue: VecDeque<(u32, u32)> = crossed.into_iter().collect();
        let mut fresh: Vec<(u32, u32)> = vec![];
        let mut guard = 0usize;
        let limit = 100 + 20 * queue.len() * queue.len();
        while let Some((u, v)) = queue.pop_front() {
            guard += 1;
            if guard > limit { return Err("segment recovery did not converge".into()); }
            let (t, k) = self.find_edge(u, v).ok_or("crossed edge vanished")?;
            let tr = self.tris[t as usize];
            let (eu, ev) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            let w = tr.v[k];
            let s = tr.n[k];
            let j = self.edge_idx(s, ev, eu).unwrap();
            let x = self.tris[s as usize].v[j];
            let (o1, o2) = (self.orient(w, x, eu), self.orient(w, x, ev));
            if !((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) {
                queue.push_back((u, v));
                continue;
            }
            if tr.c[k] { return Err("segments cross".into()); }
            self.flip(t, k);
            let (sw, sx) = (self.orient(a, b, w), self.orient(a, b, x));
            if (sw > 0.0 && sx < 0.0) || (sw < 0.0 && sx > 0.0) { queue.push_back((w, x)); } else { fresh.push((w, x)); }
        }
        let (t, k) = self.find_edge(a, b).ok_or("segment not recovered")?;
        self.mark_edge(t, k);
        let mut stack = vec![];
        for (u, v) in fresh {
            if let Some((t, k)) = self.find_edge(u, v) {
                stack.push((t, k));
                let tr = self.tris[t as usize];
                let nb = tr.n[k];
                if nb != NONE { let j = self.edge_idx(nb, tr.v[(k + 2) % 3], tr.v[(k + 1) % 3]).unwrap(); stack.push((nb, j)); }
            }
        }
        self.legalize(stack);
        Ok(())
    }

    /// Marks every unclassified triangle reachable from `t` without crossing a constrained edge.
    pub fn flood(&mut self, t: u32, region: i32) {
        if self.tris[t as usize].region != UNSET { return; }
        let mut stack = vec![t];
        self.tris[t as usize].region = region;
        while let Some(u) = stack.pop() {
            let tr = self.tris[u as usize];
            for k in 0..3 {
                if tr.c[k] { continue; }
                let nb = tr.n[k];
                if nb != NONE && self.tris[nb as usize].region == UNSET {
                    self.tris[nb as usize].region = region;
                    stack.push(nb);
                }
            }
        }
    }

    /// Gives every live triangle reachable from `t` without crossing a constrained edge the region `region`, whatever region it had (dead
    /// triangles, region -1, stay dead).
    pub fn reflood(&mut self, t: u32, region: i32) {
        if self.tris[t as usize].region < 0 || self.tris[t as usize].region == region { return; }
        let mut stack = vec![t];
        self.tris[t as usize].region = region;
        while let Some(u) = stack.pop() {
            let tr = self.tris[u as usize];
            for k in 0..3 {
                if tr.c[k] { continue; }
                let nb = tr.n[k];
                if nb != NONE && self.tris[nb as usize].region >= 0 && self.tris[nb as usize].region != region {
                    self.tris[nb as usize].region = region;
                    stack.push(nb);
                }
            }
        }
    }

    /// The circumcentre and circumradius of triangle `t`.
    pub fn circumcircle(&self, t: u32) -> (Pt, f64) {
        let tr = &self.tris[t as usize];
        let (a, b, c) = (self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize]);
        let (bx, by, cx, cy) = (b[0] - a[0], b[1] - a[1], c[0] - a[0], c[1] - a[1]);
        let d = 2.0 * (bx * cy - by * cx);
        let (b2, c2) = (bx * bx + by * by, cx * cx + cy * cy);
        let (ux, uy) = ((cy * b2 - by * c2) / d, (bx * c2 - cx * b2) / d);
        ([a[0] + ux, a[1] + uy], (ux * ux + uy * uy).sqrt())
    }

    /// True when `p` is strictly inside the diametral circle of segment `u`-`v`.
    /// Whether `p` encroaches upon the segment `u`-`v`: it lies in the segment's diametral lens, the points that see it under an angle of at
    /// least `180 - 2 * min_angle` degrees (the diametral circle itself, 90 degrees, when `lens` is 0).
    pub fn encroaches(&self, p: Pt, u: u32, v: u32) -> bool {
        let (a, b) = (self.pts[u as usize], self.pts[v as usize]);
        let d = dot(a, b, p);
        if d >= 0.0 { return false; }
        if self.lens == 0.0 { return true; }
        let (ax, ay, bx, by) = (a[0] - p[0], a[1] - p[1], b[0] - p[0], b[1] - p[1]);
        d * d >= self.lens * (ax * ax + ay * ay) * (bx * bx + by * by)
    }
}


// ---------------------------------------------------------------------------------------------------------------------------------------
// ck-tess layer: PSLG triangulation with parity classification + interior Steiner points
// ---------------------------------------------------------------------------------------------------------------------------------------

/// Region of a triangle inside the PSLG after [`Cdt::classify_parity`].
pub const INSIDE: i32 = 1;
/// Region of a triangle outside (or in a hole).
pub const OUTSIDE: i32 = -1;

impl Cdt {
    /// Classifies every live triangle by the parity of constrained edges crossed on the way from the super triangle: odd = inside.
    /// Works for any number of nested loops regardless of their orientation.
    pub fn classify_parity(&mut self) {
        for t in self.tris.iter_mut() {
            t.region = UNSET;
        }
        // start: a live triangle touching a super vertex
        let start = match (0..self.tris.len()).find(|&i| self.tris[i].alive && self.tris[i].v.iter().any(|&v| v < 3)) {
            Some(s) => s as u32,
            None => return,
        };
        let mut depth: Vec<u32> = vec![u32::MAX; self.tris.len()];
        let mut cur: VecDeque<u32> = VecDeque::new();
        let mut next: VecDeque<u32> = VecDeque::new();
        depth[start as usize] = 0;
        cur.push_back(start);
        let mut d = 0u32;
        while !cur.is_empty() {
            while let Some(t) = cur.pop_front() {
                let tr = self.tris[t as usize];
                for k in 0..3 {
                    let nb = tr.n[k];
                    if nb == NONE || !self.tris[nb as usize].alive || depth[nb as usize] != u32::MAX {
                        continue;
                    }
                    if tr.c[k] {
                        // crossing a segment: next depth level (claimed lazily so a same-depth path wins)
                        next.push_back(nb);
                    } else {
                        depth[nb as usize] = d;
                        cur.push_back(nb);
                    }
                }
            }
            d += 1;
            for t in next.drain(..) {
                if depth[t as usize] == u32::MAX {
                    depth[t as usize] = d;
                    cur.push_back(t);
                }
            }
        }
        for (i, t) in self.tris.iter_mut().enumerate() {
            if !t.alive {
                continue;
            }
            let has_super = t.v.iter().any(|&v| v < 3);
            t.region = if depth[i] != u32::MAX && depth[i] % 2 == 1 && !has_super { INSIDE } else { OUTSIDE };
        }
    }

    /// Live triangles with `region == INSIDE`, as vertex index triples (super vertices excluded, indices are into `pts`).
    pub fn inside_tris(&self) -> Vec<(u32, [u32; 3])> {
        let mut out = vec![];
        for (i, t) in self.tris.iter().enumerate() {
            if t.alive && t.region == INSIDE {
                out.push((i as u32, t.v));
            }
        }
        out
    }

    /// Inserts a Steiner point strictly inside an INSIDE triangle (no constrained edge is touched). Returns the new vertex, or None when
    /// the point is not strictly inside an INSIDE triangle (on an edge, a vertex, or outside).
    pub fn insert_interior(&mut self, p: Pt, hint: u32) -> Option<u32> {
        match self.locate(p, hint, false) {
            Loc::In(t) if self.tris[t as usize].region == INSIDE => Some(self.insert_in_tri(t, p)),
            Loc::Edge(t, k) if self.tris[t as usize].region == INSIDE && !self.tris[t as usize].c[k] => {
                let nb = self.tris[t as usize].n[k];
                if nb != NONE && self.tris[nb as usize].region == INSIDE {
                    Some(self.insert_on_edge(t, k, p))
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

/// Result of [`triangulate`].
pub struct Triangulation {
    pub cdt: Cdt,
    /// Map from caller point index to CDT vertex index (duplicates collapse onto one vertex).
    pub vmap: Vec<u32>,
}

/// Triangulates the PSLG `pts` + constrained `segs` (pairs of indices into `pts`) and classifies triangles by parity.
pub fn triangulate(pts: &[Pt], segs: &[(usize, usize)]) -> Result<Triangulation, String> {
    if pts.len() < 3 {
        return Err("fewer than 3 points".into());
    }
    let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        if !(p[0].is_finite() && p[1].is_finite()) {
            return Err("non-finite point".into());
        }
        b = (b.0.min(p[0]), b.1.min(p[1]), b.2.max(p[0]), b.3.max(p[1]));
    }
    let mut cdt = Cdt::new(b);
    let mut vmap = Vec::with_capacity(pts.len());
    let mut hint = 0u32;
    for &p in pts {
        let v = cdt.insert_point(p, hint);
        hint = cdt.vt[v as usize];
        vmap.push(v);
    }
    for &(a, b) in segs {
        let (va, vb) = (vmap[a], vmap[b]);
        if va != vb {
            cdt.insert_segment(va, vb)?;
        }
    }
    cdt.classify_parity();
    Ok(Triangulation { cdt, vmap })
}

#[cfg(test)]
mod layer_tests {
    use super::*;

    fn area(c: &Cdt) -> f64 {
        c.inside_tris()
            .iter()
            .map(|(_, v)| {
                let (a, b, d) = (c.pts[v[0] as usize], c.pts[v[1] as usize], c.pts[v[2] as usize]);
                0.5 * ((b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0]))
            })
            .sum()
    }

    #[test]
    fn square_with_hole() {
        let pts = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0], [1.0, 1.0], [1.0, 3.0], [3.0, 3.0], [3.0, 1.0]];
        let segs = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4)];
        let t = triangulate(&pts, &segs).unwrap();
        let n = t.cdt.inside_tris().len();
        // n_pts + 2*holes - 2 = 8 + 2 - 2
        assert_eq!(n, 8);
        assert!((area(&t.cdt) - 12.0).abs() < 1e-12);
    }

    #[test]
    fn regular_polygon_fan_count() {
        let n = 35;
        let pts: Vec<Pt> = (0..n).map(|i| {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            [a.cos(), a.sin()]
        }).collect();
        let segs: Vec<(usize, usize)> = (0..n).map(|i| (i, (i + 1) % n)).collect();
        let t = triangulate(&pts, &segs).unwrap();
        assert_eq!(t.cdt.inside_tris().len(), n - 2);
    }

    #[test]
    fn steiner_inside() {
        let pts = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let segs = [(0, 1), (1, 2), (2, 3), (3, 0)];
        let mut t = triangulate(&pts, &segs).unwrap();
        assert!(t.cdt.insert_interior([1.0, 1.5], 0).is_some());
        assert!(t.cdt.insert_interior([9.0, 1.5], 0).is_none());
        assert_eq!(t.cdt.inside_tris().len(), 4);
        assert!((area(&t.cdt) - 16.0).abs() < 1e-12);
    }
}
