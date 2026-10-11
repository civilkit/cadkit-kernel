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

/// Triangle record (the original's 0x20-byte slot: v[3], n[3], c[3], alive, inside flag).
#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub v: [u32; 3],
    pub n: [u32; 3],
    pub c: [bool; 3],
    pub region: i32,
    pub alive: bool,
}

/// Where a point falls (original f3299/f3303): inside a triangle, on its edge `k`, or on (within `eps` of) an existing vertex id.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Loc {
    In(u32),
    Edge(u32, usize),
    Vertex(u32),
}

/// Constrained Delaunay triangulation. A line-by-line port of the original's cdt.rs (kernel.wasm f3295..f3312): deterministic
/// last-triangle-hint walk, point snap tolerance `eps`, Lawson flips from an explicit stack, Sloan segment recovery with a FIFO queue
/// and a swap-until-stable pass over the new edges.
pub struct Cdt {
    pub eps: f64,
    pub pts: Vec<Pt>,
    pub tris: Vec<Tri>,
    pub vt: Vec<u32>,
    /// Last triangle created by an insertion: the start of the next point location (f3303).
    pub hint: u32,
    /// Count of segments that could not be recovered (original: field at +0x30).
    pub failed: u32,
}

impl Cdt {
    /// An empty triangulation whose super-triangle encloses `bounds = (min_x, min_y, max_x, max_y)` (original f3312, R = 32 * diagonal).
    pub fn new(bounds: (f64, f64, f64, f64), eps: f64) -> Cdt {
        let (minx, miny, maxx, maxy) = bounds;
        let cx = (minx + maxx) * 0.5;
        let cy = (miny + maxy) * 0.5;
        let mut d = ((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny)).sqrt();
        if d.is_nan() || d <= 1e-9 {
            d = 1e-9;
        }
        let r = d * 0.5 * 64.0;
        let lo = cy - r;
        let r2 = r + r;
        let pts = vec![[cx - r2, lo], [cx + r2, lo], [cx, cy + r2]];
        let t = Tri { v: [0, 1, 2], n: [NONE; 3], c: [false; 3], region: 0, alive: true };
        Cdt { eps, pts, tris: vec![t], vt: vec![0, 0, 0], hint: 0, failed: 0 }
    }

    pub fn is_super(&self, v: u32) -> bool { v < 3 }

    fn push_tri(&mut self, t: Tri) -> u32 {
        self.tris.push(t);
        (self.tris.len() - 1) as u32
    }

    pub fn orient(&self, a: u32, b: u32, c: u32) -> f64 { orient2d(self.pts[a as usize], self.pts[b as usize], self.pts[c as usize]) }

    /// Makes the neighbour's pointer for the edge `(a, b)` of a new triangle (seen from the neighbour as `(b, a)`) point at `t`
    /// (original: the first of n[0..3] equal to the old triangle id is overwritten; same effect for a consistent mesh).
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

    fn pos(tr: &Tri, u: u32) -> usize {
        if tr.v[0] == u { 0 } else if tr.v[1] == u { 1 } else { 2 }
    }

    /// The triangle holding the edge between `u` and `v` and the index of that edge in it (original f3278): fan walk from `vt[u]`,
    /// preferring the triangle where the edge runs `u -> v`.
    pub fn find_edge(&self, u: u32, v: u32) -> Option<(u32, usize)> {
        let start = self.vt[u as usize];
        if start == NONE { return None; }
        let mut cur = start;
        let mut cnt = 0u32;
        loop {
            if cur as usize >= self.tris.len() { panic!("find_edge: index out of range"); }
            let tr = &self.tris[cur as usize];
            let k = Self::pos(tr, u);
            let (k1, k2) = ((k + 1) % 3, (k + 2) % 3);
            if tr.v[k1] == v { return Some((cur, k2)); }
            let nxt = tr.n[k1];
            if tr.v[k2] == v {
                let mut res = (cur, k1);
                if nxt != NONE {
                    let nb = &self.tris[nxt as usize];
                    if nb.v[1] == u && nb.v[2] == v { res = (nxt, 0); }
                    else if nb.v[2] == u && nb.v[0] == v { res = (nxt, 1); }
                    else if nb.v[0] == u && nb.v[1] == v { res = (nxt, 2); }
                }
                return Some(res);
            }
            if nxt != NONE && nxt != start && cnt < 0xfff {
                cnt += 1;
                cur = nxt;
            } else {
                break;
            }
        }
        let mut cur = start;
        let mut cnt = 0u32;
        loop {
            let tr = &self.tris[cur as usize];
            let k = Self::pos(tr, u);
            let (k1, k2) = ((k + 1) % 3, (k + 2) % 3);
            if tr.v[k1] == v { return Some((cur, k2)); }
            if tr.v[k2] == v { return Some((cur, k1)); }
            let nxt = tr.n[k2];
            if nxt != NONE && nxt != start && cnt < 0xfff {
                cnt += 1;
                cur = nxt;
            } else {
                return None;
            }
        }
    }

    /// Marks edge `k` of triangle `t` (and the twin) constrained (original f3306).
    fn mark_edge(&mut self, t: u32, k: usize) {
        self.tris[t as usize].c[k] = true;
        let tr = self.tris[t as usize];
        let nb = tr.n[k];
        if nb != NONE {
            let j = (0..3).find(|&j| self.tris[nb as usize].n[j] == t);
            if let Some(j) = j { self.tris[nb as usize].c[j] = true; }
        }
    }

    fn mark_segment(&mut self, a: u32, b: u32) {
        if let Some((t, k)) = self.find_edge(a, b) { self.mark_edge(t, k); }
    }

    // ------------------------------------------------------------------------------------------------------------------------------
    // point location
    // ------------------------------------------------------------------------------------------------------------------------------

    /// Classifies `p` known to lie in triangle `t` (original f3299): snap to a vertex within `eps`, else exact-zero edge tests, else inside.
    fn classify(&self, t: u32, p: Pt) -> Loc {
        let tr = self.tris[t as usize];
        for k in 0..3 {
            let q = self.pts[tr.v[k] as usize];
            let d = ((q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1])).sqrt();
            if !(self.eps < d) { return Loc::Vertex(tr.v[k]); }
        }
        let (a, b, c) = (self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize]);
        if orient2d(b, c, p) == 0.0 { return Loc::Edge(t, 0); }
        if orient2d(c, a, p) == 0.0 { return Loc::Edge(t, 1); }
        if orient2d(a, b, p) == 0.0 { return Loc::Edge(t, 2); }
        Loc::In(t)
    }

    /// Walks from the hint triangle to the one holding `p` (original f3303): the first edge tested rotates 1,2,0,1,... per step.
    pub fn locate(&self, p: Pt) -> Loc {
        let mut t = self.hint;
        let mut first = 0usize;
        let cap = 4 * self.tris.len() + 0x40;
        let mut steps = 0usize;
        'walk: loop {
            if steps >= cap { break; }
            steps += 1;
            if t as usize >= self.tris.len() { panic!("locate: index out of range"); }
            let tr = self.tris[t as usize];
            let e1 = if first != 2 { first + 1 } else { 0 };
            for i in 0..3 {
                let e = (e1 + i) % 3;
                let (a, b) = (self.pts[tr.v[(e + 1) % 3] as usize], self.pts[tr.v[(e + 2) % 3] as usize]);
                if orient2d(a, b, p) < 0.0 {
                    let nb = tr.n[e];
                    if nb == NONE { break 'walk; }
                    t = nb;
                    first = e1;
                    continue 'walk;
                }
            }
            return self.classify(t, p);
        }
        // original f3301: brute-force scan in triangle order
        for (i, tr) in self.tris.iter().enumerate() {
            if !tr.alive { continue; }
            let (a, b, c) = (self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize]);
            if orient2d(b, c, p) >= 0.0 && orient2d(c, a, p) >= 0.0 && orient2d(a, b, p) >= 0.0 {
                return self.classify(i as u32, p);
            }
        }
        Loc::In(0)
    }

    // ------------------------------------------------------------------------------------------------------------------------------
    // edits
    // ------------------------------------------------------------------------------------------------------------------------------

    /// Swaps the diagonal of the quadrilateral across edge `k` of triangle `t` (original f3309). Returns `(t, s)`.
    fn flip(&mut self, t: u32, k: usize) -> (u32, u32) {
        let tr = self.tris[t as usize];
        let (w, u, v) = (tr.v[k], tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
        let s = tr.n[k];
        let ts = self.tris[s as usize];
        let j = (0..3).find(|&j| ts.n[j] == t).expect("flip: neighbour does not point back");
        let x = ts.v[j];
        let (a_n, a_c) = (tr.n[(k + 2) % 3], tr.c[(k + 2) % 3]); // across (w,u)
        let (b_n, b_c) = (tr.n[(k + 1) % 3], tr.c[(k + 1) % 3]); // across (v,w)
        let (c_n, c_c) = (ts.n[(j + 1) % 3], ts.c[(j + 1) % 3]); // across (u,x)
        let (d_n, d_c) = (ts.n[(j + 2) % 3], ts.c[(j + 2) % 3]); // across (x,v)
        self.tris[t as usize] = Tri { v: [w, u, x], n: [c_n, s, a_n], c: [c_c, false, a_c], region: tr.region, alive: true };
        self.tris[s as usize] = Tri { v: [w, x, v], n: [d_n, b_n, t], c: [d_c, b_c, false], region: ts.region, alive: true };
        self.fix_back(c_n, u, x, t); // (u,x) now belongs to t
        self.fix_back(b_n, v, w, s); // (v,w) now belongs to s
        self.vt[w as usize] = t; self.vt[u as usize] = t; self.vt[x as usize] = t; self.vt[v as usize] = s;
        (t, s)
    }

    /// Restores the Delaunay property from an explicit stack (original f3298): pop, flip if the opposite vertex is strictly inside the
    /// circle, then push `(t,0)` and `(s,0)`.
    fn legalize(&mut self, mut stack: Vec<(u32, usize)>) {
        let mut budget = 1_000_001i64;
        while let Some((t, k)) = stack.pop() {
            budget -= 1;
            if budget <= 0 { return; }
            let tr = self.tris[t as usize];
            if !tr.alive || tr.c[k] { continue; }
            let s = tr.n[k];
            if s == NONE { continue; }
            let j = match (0..3).find(|&j| self.tris[s as usize].n[j] == t) { Some(j) => j, None => continue };
            let x = self.tris[s as usize].v[j];
            let inside = incircle(self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize], self.pts[x as usize]);
            if inside > 0.0 {
                let (a, b) = self.flip(t, k);
                stack.push((a, 0));
                stack.push((b, 0));
            }
        }
    }

    fn add_vertex(&mut self, p: Pt) -> u32 {
        let pi = self.pts.len() as u32;
        self.pts.push(p);
        self.vt.push(NONE);
        pi
    }

    /// Inserts the point strictly inside triangle `t` (original f3305); returns the new vertex index.
    pub fn insert_in_tri(&mut self, t: u32, p: Pt) -> u32 {
        let pi = self.add_vertex(p);
        let tr = self.tris[t as usize];
        let (a, b, c) = (tr.v[0], tr.v[1], tr.v[2]);
        let (n0, n1, n2) = (tr.n[0], tr.n[1], tr.n[2]);
        let (c0, c1, c2) = (tr.c[0], tr.c[1], tr.c[2]);
        let r = tr.region;
        let t1 = self.push_tri(Tri { v: [b, c, pi], n: [NONE; 3], c: [false; 3], region: r, alive: true });
        let t2 = self.push_tri(Tri { v: [c, a, pi], n: [NONE; 3], c: [false; 3], region: r, alive: true });
        self.tris[t as usize] = Tri { v: [a, b, pi], n: [t1, t2, n2], c: [false, false, c2], region: r, alive: true };
        self.tris[t1 as usize] = Tri { v: [b, c, pi], n: [t2, t, n0], c: [false, false, c0], region: r, alive: true };
        self.tris[t2 as usize] = Tri { v: [c, a, pi], n: [t, t1, n1], c: [false, false, c1], region: r, alive: true };
        self.fix_back(n0, b, c, t1);
        self.fix_back(n1, c, a, t2);
        self.vt[a as usize] = t; self.vt[b as usize] = t; self.vt[c as usize] = t1; self.vt[pi as usize] = t;
        self.hint = t;
        self.legalize(vec![(t, 2), (t1, 2), (t2, 2)]);
        pi
    }

    /// Splits edge `k` of triangle `t` (and the triangle across it) at `p` (original f3295). A constrained edge becomes two.
    pub fn insert_on_edge(&mut self, t: u32, k: usize, p: Pt) -> u32 {
        let pi = self.add_vertex(p);
        let tr = self.tris[t as usize];
        let (w, u, v) = (tr.v[k], tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
        let s = tr.n[k];
        let c_uv = tr.c[k];
        let (au_n, au_c) = (tr.n[(k + 2) % 3], tr.c[(k + 2) % 3]); // across (w,u)
        let (bv_n, bv_c) = (tr.n[(k + 1) % 3], tr.c[(k + 1) % 3]); // across (v,w)
        let rt = tr.region;
        if s == NONE {
            let t2 = self.push_tri(Tri { v: [w, pi, v], n: [NONE; 3], c: [false; 3], region: rt, alive: true });
            self.tris[t as usize] = Tri { v: [w, u, pi], n: [NONE, t2, au_n], c: [c_uv, false, au_c], region: rt, alive: true };
            self.tris[t2 as usize] = Tri { v: [w, pi, v], n: [NONE, bv_n, t], c: [c_uv, bv_c, false], region: rt, alive: true };
            self.fix_back(bv_n, v, w, t2);
            self.vt[w as usize] = t; self.vt[u as usize] = t; self.vt[v as usize] = t2; self.vt[pi as usize] = t;
            self.hint = t;
            self.legalize(vec![(t, 2), (t2, 1)]);
            return pi;
        }
        let ts = self.tris[s as usize];
        let j = (0..3).find(|&j| ts.n[j] == t).expect("insert_on_edge: neighbour does not point back");
        let x = ts.v[j];
        let (dxv_n, dxv_c) = (ts.n[(j + 2) % 3], ts.c[(j + 2) % 3]); // across (x,v)
        let (cux_n, cux_c) = (ts.n[(j + 1) % 3], ts.c[(j + 1) % 3]); // across (u,x)
        let rs = ts.region;
        let t2 = self.push_tri(Tri { v: [w, pi, v], n: [NONE; 3], c: [false; 3], region: rt, alive: true });
        let s2 = self.push_tri(Tri { v: [x, pi, u], n: [NONE; 3], c: [false; 3], region: rs, alive: true });
        let t1 = t;
        let s1 = s;
        self.tris[t1 as usize] = Tri { v: [w, u, pi], n: [s2, t2, au_n], c: [c_uv, false, au_c], region: rt, alive: true };
        self.tris[t2 as usize] = Tri { v: [w, pi, v], n: [s1, bv_n, t1], c: [c_uv, bv_c, false], region: rt, alive: true };
        self.tris[s1 as usize] = Tri { v: [x, v, pi], n: [t2, s2, dxv_n], c: [c_uv, false, dxv_c], region: rs, alive: true };
        self.tris[s2 as usize] = Tri { v: [x, pi, u], n: [t1, cux_n, s1], c: [c_uv, cux_c, false], region: rs, alive: true };
        self.fix_back(bv_n, v, w, t2);
        self.fix_back(cux_n, u, x, s2);
        self.vt[w as usize] = t1; self.vt[u as usize] = t1; self.vt[v as usize] = t2; self.vt[x as usize] = s1; self.vt[pi as usize] = t1;
        self.hint = t1;
        self.legalize(vec![(s1, 2), (s2, 1), (t1, 2), (t2, 1)]);
        pi
    }

    /// Inserts a point wherever it falls; returns its vertex index, or the existing vertex it snaps to (original f3310).
    pub fn insert_point(&mut self, p: Pt) -> u32 {
        match self.locate(p) {
            Loc::In(t) => self.insert_in_tri(t, p),
            Loc::Edge(t, k) => self.insert_on_edge(t, k, p),
            Loc::Vertex(v) => v,
        }
    }

    /// Recovers segment `a`-`b` as a constrained edge (original f3307): a work stack of sub-segments; the crossed edges are flipped out
    /// FIFO (Sloan), then the new edges are swapped to Delaunay in passes.
    pub fn insert_segment(&mut self, a0: u32, b0: u32) {
        let mut work: Vec<(u32, u32)> = vec![(a0, b0)];
        let mut iters = 0u32;
        while let Some((a, b)) = work.pop() {
            iters += 1;
            if a == b || iters >= 100_001 { continue; }
            if let Some((t, k)) = self.find_edge(a, b) {
                self.mark_edge(t, k);
                continue;
            }
            match self.segment_walk(a, b) {
                Walk::Split(m) => { work.push((m, b)); work.push((a, m)); }
                Walk::Fail => { self.failed += 1; }
                Walk::Recovered => {}
            }
        }
    }

    fn dot_ab(&self, a: u32, b: u32, p: u32) -> f64 {
        let (pa, pb, pp) = (self.pts[a as usize], self.pts[b as usize], self.pts[p as usize]);
        (pb[0] - pa[0]) * (pp[0] - pa[0]) + (pb[1] - pa[1]) * (pp[1] - pa[1])
    }

    fn segment_walk(&mut self, a: u32, b: u32) -> Walk {
        // --- 1. the wedge at `a` holding the direction to `b`: CCW fan first (with the on-segment vertex test), then CW
        let start = self.vt[a as usize];
        let mut wedge: Option<(u32, usize)> = None;
        let mut cur = start;
        let mut cnt = 0u32;
        loop {
            let tr = self.tris[cur as usize];
            let k = Self::pos(&tr, a);
            let (p, q) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            let op = self.orient(a, b, p);
            let oq = self.orient(a, b, q);
            if op == 0.0 && p != b && self.dot_ab(a, b, p) > 0.0 { return Walk::Split(p); }
            if oq == 0.0 && q != b && self.dot_ab(a, b, q) > 0.0 { return Walk::Split(q); }
            if op < 0.0 && oq > 0.0 { wedge = Some((cur, k)); break; }
            let nxt = tr.n[(k + 1) % 3];
            if nxt != NONE && nxt != start && cnt < 0xfff { cnt += 1; cur = nxt; } else { break; }
        }
        if wedge.is_none() {
            let mut cur = start;
            let mut cnt = 0u32;
            loop {
                let tr = self.tris[cur as usize];
                let k = Self::pos(&tr, a);
                let (p, q) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
                if self.orient(a, b, p) < 0.0 && self.orient(a, b, q) > 0.0 { wedge = Some((cur, k)); break; }
                let nxt = tr.n[(k + 2) % 3];
                if nxt != NONE && nxt != start && cnt < 0xfff { cnt += 1; cur = nxt; } else { break; }
            }
        }
        let (mut cur, mut k) = match wedge { Some(w) => w, None => return Walk::Fail };

        // --- 2. walk across the triangles the segment crosses, recording the crossed edges
        let mut crossed: Vec<(u32, u32)> = vec![];
        let mut count = 1u32;
        loop {
            let tr = self.tris[cur as usize];
            let (p, q) = (tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
            if tr.c[k] {
                // crosses another constrained edge: split both at the intersection point
                let (pa, pb, pp, pq) = (self.pts[a as usize], self.pts[b as usize], self.pts[p as usize], self.pts[q as usize]);
                let den = (pb[0] - pa[0]) * (pq[1] - pp[1]) - (pb[1] - pa[1]) * (pq[0] - pp[0]);
                let mut s = if den == 0.0 { 0.5 } else { ((pp[0] - pa[0]) * (pq[1] - pp[1]) - (pp[1] - pa[1]) * (pq[0] - pp[0])) / den };
                s = if 0.0 <= s { s } else { 0.0 };
                s = if s <= 1.0 { s } else { 1.0 };
                let pt = [pa[0] + (pb[0] - pa[0]) * s, pa[1] + (pb[1] - pa[1]) * s];
                let m = self.insert_point(pt);
                if m == a || m == b { return Walk::Fail; }
                return Walk::Split(m);
            }
            crossed.push((p, q));
            let nb = tr.n[k];
            if nb == NONE { return Walk::Fail; }
            let j = (0..3).find(|&j| self.tris[nb as usize].n[j] == cur).expect("segment walk: neighbour does not point back");
            let r = self.tris[nb as usize].v[j];
            if r == b { break; }
            let o = self.orient(a, b, r);
            if o == 0.0 { return Walk::Split(r); }
            k = if o > 0.0 { (j + 1) % 3 } else { (j + 2) % 3 };
            cur = nb;
            count += 1;
            if count == 1_000_001 { return Walk::Fail; }
        }

        // --- 3. flip the crossed edges out, FIFO
        let mut queue: VecDeque<(u32, u32)> = crossed.into_iter().collect();
        let mut fresh: Vec<(u32, u32)> = vec![];
        let mut popped = 0u32;
        while let Some((u, v)) = queue.pop_front() {
            popped += 1;
            if popped == 200_000 { return Walk::Fail; }
            let mut requeue = true;
            if let Some((t, k)) = self.find_edge(u, v) {
                let tr = self.tris[t as usize];
                let s = tr.n[k];
                if s != NONE {
                    let j = (0..3).find(|&j| self.tris[s as usize].n[j] == t).expect("flip phase: neighbour does not point back");
                    let x = self.tris[s as usize].v[j];
                    let (w, eu, ev) = (tr.v[k], tr.v[(k + 1) % 3], tr.v[(k + 2) % 3]);
                    if self.orient(w, eu, x) > 0.0 && self.orient(x, ev, w) > 0.0 {
                        self.flip(t, k);
                        let (fw, fx) = (self.orient(a, b, w), self.orient(a, b, x));
                        let (gw, gx) = (self.orient(w, x, a), self.orient(w, x, b));
                        if 0.0 <= fw * fx || 0.0 <= gw * gx { fresh.push((w, x)); } else { queue.push_back((w, x)); }
                        requeue = false;
                    }
                }
            }
            if requeue { queue.push_back((u, v)); }
        }
        self.mark_segment(a, b);

        // --- 4. swap the new edges to Delaunay, pass after pass
        if !fresh.is_empty() {
            let mut pass = 0u32;
            loop {
                let mut changed = false;
                for e in 0..fresh.len() {
                    let (u, v) = fresh[e];
                    if (u == a && v == b) || (u == b && v == a) { continue; }
                    let (t, k) = match self.find_edge(u, v) { Some(r) => r, None => continue };
                    let tr = self.tris[t as usize];
                    if tr.c[k] { continue; }
                    let s = tr.n[k];
                    if s == NONE { continue; }
                    let j = (0..3).find(|&j| self.tris[s as usize].n[j] == t).expect("swap phase: neighbour does not point back");
                    let x = self.tris[s as usize].v[j];
                    let inside = incircle(self.pts[tr.v[0] as usize], self.pts[tr.v[1] as usize], self.pts[tr.v[2] as usize], self.pts[x as usize]);
                    if inside > 0.0 {
                        let w = tr.v[k];
                        self.flip(t, k);
                        fresh[e] = (w, x);
                        changed = true;
                    }
                }
                if !changed || pass > 0x3e { break; }
                pass += 1;
            }
        }
        self.mark_segment(a, b);
        Walk::Recovered
    }
}

enum Walk {
    Recovered,
    /// Re-queue the segment as `(a, m)` and `(m, b)`.
    Split(u32),
    Fail,
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

    /// Inserts a Steiner point inside or on an edge of an INSIDE triangle (original f3302). Returns the new vertex, or None when the
    /// point snaps to a vertex or lands in/beside only OUTSIDE triangles.
    pub fn insert_interior(&mut self, p: Pt) -> Option<u32> {
        match self.locate(p) {
            Loc::Edge(t, k) => {
                if self.tris[t as usize].region != INSIDE {
                    let nb = self.tris[t as usize].n[k];
                    if nb == NONE || self.tris[nb as usize].region != INSIDE { return None; }
                }
                Some(self.insert_on_edge(t, k, p))
            }
            Loc::Vertex(_) => None,
            Loc::In(t) => {
                if self.tris[t as usize].region != INSIDE { return None; }
                Some(self.insert_in_tri(t, p))
            }
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
    let mut mx = 0.0f64;
    for p in pts {
        mx = mx.max(p[0].abs()).max(p[1].abs());
    }
    let eps = mx.max(1e-9) * 1e-13;
    let mut cdt = Cdt::new(b, eps);
    let mut vmap = Vec::with_capacity(pts.len());
    for &p in pts {
        vmap.push(cdt.insert_point(p));
    }
    for &(a, b) in segs {
        cdt.insert_segment(vmap[a], vmap[b]);
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
        assert!(t.cdt.insert_interior([1.0, 1.5]).is_some());
        assert!(t.cdt.insert_interior([9.0, 1.5]).is_none());
        assert_eq!(t.cdt.inside_tris().len(), 4);
        assert!((area(&t.cdt) - 16.0).abs() < 1e-12);
    }
}

#[cfg(test)]
mod perm_probe {
    use super::*;
    #[test]
    fn probe() {
        let p = [[-20.0, -10.0], [-20.0, 10.0], [20.0, 10.0], [20.0, -10.0]];
        let mut perms = vec![];
        for a in 0..4 { for b in 0..4 { for c in 0..4 { for d in 0..4 {
            let v = [a, b, c, d];
            let mut s = v; s.sort();
            if s == [0, 1, 2, 3] { perms.push(v); }
        }}}}
        for v in perms {
            let pts: Vec<Pt> = v.iter().map(|&i| p[i]).collect();
            let segs: Vec<(usize, usize)> = (0..4).map(|i| (i, (i + 1) % 4)).collect();
            let t = triangulate(&pts, &segs).unwrap();
            let lab = |cv: u32| -> usize { let q = t.cdt.pts[cv as usize]; (0..4).find(|&i| p[i] == q).unwrap_or(9) };
            let r: Vec<[usize; 3]> = t.cdt.inside_tris().iter().map(|(_, vv)| [lab(vv[0]), lab(vv[1]), lab(vv[2])]).collect();
            eprintln!("PERM {:?} -> {:?}", v, r);
        }
    }
}
