//! Quadric mesh decimation with a hard deviation bound (`meshDecimate`, `opts.decimate` of `meshToSolid`).
//!
//! Garland-Heckbert edge collapse: every vertex carries the (area weighted) sum of the squared-distance quadrics of the
//! planes of its triangles; the edge of least quadric error collapses next, to the position that minimises the error (the
//! linear solve, else the better of both ends and the midpoint). What makes it usable for scans:
//!
//! * **Topology is never broken.** The link condition (the two ends share exactly the opposite corners of the triangles
//!   on the edge), no collapse that leaves a tetrahedron or a lone triangle, no face turned over or squeezed to a sliver.
//!   Every edge of the result is used by at most the number of triangles it was used by, so a manifold input gives a
//!   manifold output with the same orientation. Boundary edges (open meshes) and, with `keepAngleDeg`, crease edges form a
//!   feature graph: a vertex on it only moves along it, a vertex where it branches or ends never moves.
//! * **Sheets are not pulled together.** A scan has places where two layers of the surface are a hair apart (a tyre and its
//!   wheel arch). A plain deviation check does not see a collapse that brings them together, so each collapse measures the
//!   distance of the new triangles to every triangle that does not share a vertex with them (a uniform grid of the live
//!   triangles, triangle-triangle distance); it is refused when that distance would drop below 0.8 x the smaller of the
//!   distance it had and `tol`, and a result with a crossing pair of triangles that the input did not have is rejected.
//! * **The deviation is bounded, not hoped for.** Every current vertex keeps the original vertices that were merged into it;
//!   a collapse is accepted only when all the original vertices of the vertices it touches stay within the internal
//!   tolerance (0.85 x `tol`) of the new triangles. The result is then measured independently, both ways (original
//!   vertices and triangle centroids to the result, the result's to the original, BVH); when the true maximum exceeds
//!   `tol` the run is repeated with a tighter internal tolerance, and when that cannot be reached the mesh is returned
//!   unchanged (`stoppedBy: "verification"`).
//!
//! Deterministic: no hashing order and no threads; ties go to the lower vertex numbers.

use crate::deviation::{dist2_tri, Bvh};
use crate::request::mesh_of;
use crate::Refusal;
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Options (`opts.decimate` / the `meshDecimate` opts). Lengths in model units (mm).
#[derive(Debug, Clone)]
pub struct DecimateOpts {
    /// The largest distance of the result from the input surface (required, > 0).
    pub tol: f64,
    /// Stop at this many triangles (default: as far as `tol` allows).
    pub target_triangles: Option<usize>,
    /// Edges whose neighbouring faces meet at more than this angle (degrees) are kept as creases (default: none).
    pub keep_angle_deg: Option<f64>,
}

impl DecimateOpts {
    pub fn new(tol: f64) -> DecimateOpts {
        DecimateOpts { tol, target_triangles: None, keep_angle_deg: None }
    }
    /// From the JSON object (`tol`, `targetTriangles`, `keepAngleDeg`); a missing or bad `tol` is NaN (refused later).
    pub fn from_json(j: &Json) -> DecimateOpts {
        DecimateOpts {
            tol: j.get_f64("tol").unwrap_or(f64::NAN),
            target_triangles: j.get_f64("targetTriangles").filter(|v| v.is_finite() && *v >= 1.0).map(|v| (v as usize).max(4)),
            keep_angle_deg: j.get_f64("keepAngleDeg").filter(|v| v.is_finite() && *v > 0.0 && *v < 180.0),
        }
    }
}

/// What was done, and the measured deviation.
#[derive(Debug, Clone, Default)]
pub struct DecimateReport {
    pub input_triangles: usize,
    pub input_vertices: usize,
    pub triangles: usize,
    pub vertices: usize,
    pub collapses: usize,
    pub tol: f64,
    /// The tolerance the collapses were held to in the run that was kept.
    pub internal_tol: f64,
    /// Runs made (more than one when the measured deviation made a tighter run necessary).
    pub attempts: usize,
    /// `"target"`, `"tolerance"` (no further collapse stays within `tol`) or `"verification"` (the mesh is unchanged).
    pub stopped_by: &'static str,
    pub boundary_edges: usize,
    pub crease_edges: usize,
    pub watertight: bool,
    /// Collapses refused, by reason (last run).
    pub refused_topology: usize,
    pub refused_flip: usize,
    pub refused_deviation: usize,
    pub refused_clearance: usize,
    /// Measured (independently of the collapse bookkeeping) distance of result and input, both ways.
    pub dev_max: f64,
    pub dev_rms: f64,
    pub dev_samples: usize,
    pub dev_input_to_result: f64,
    pub dev_result_to_input: f64,
    /// Crossing triangle pairs (not sharing a vertex) in the result, and in the input when the result has any.
    pub self_intersections: usize,
    pub input_self_intersections: usize,
}

impl DecimateReport {
    pub fn json(&self) -> Json {
        let mut o = Obj::new()
            .put("triangles", self.triangles as f64)
            .put("vertices", self.vertices as f64)
            .put("inputTriangles", self.input_triangles as f64)
            .put("inputVertices", self.input_vertices as f64)
            .put("collapses", self.collapses as f64)
            .put("tol", self.tol)
            .put("internalTol", self.internal_tol)
            .put("attempts", self.attempts as f64)
            .put("stoppedBy", self.stopped_by)
            .put("watertight", self.watertight)
            .put("boundaryEdges", self.boundary_edges as f64)
            .put("creaseEdges", self.crease_edges as f64)
            .put(
                "refused",
                Obj::new()
                    .put("topology", self.refused_topology as f64)
                    .put("flip", self.refused_flip as f64)
                    .put("deviation", self.refused_deviation as f64)
                    .put("clearance", self.refused_clearance as f64)
                    .build(),
            )
            .put(
                "deviation",
                Obj::new()
                    .put("max", self.dev_max)
                    .put("rms", self.dev_rms)
                    .put("samples", self.dev_samples as f64)
                    .put("inputToResult", self.dev_input_to_result)
                    .put("resultToInput", self.dev_result_to_input)
                    .build(),
            )
            .put("selfIntersections", self.self_intersections as f64);
        if self.self_intersections > 0 {
            o = o.put("inputSelfIntersections", self.input_self_intersections as f64);
        }
        o.build()
    }
}

pub struct Decimated {
    pub pts: Vec<Vec3>,
    pub tris: Vec<[u32; 3]>,
    pub report: DecimateReport,
}

// ------------------------------------------------------------------ geometry

fn same(a: Vec3, b: Vec3) -> bool {
    a.x == b.x && a.y == b.y && a.z == b.z
}

fn tri_box(t: &[Vec3; 3]) -> (Vec3, Vec3) {
    (t[0].min(t[1]).min(t[2]), t[0].max(t[1]).max(t[2]))
}

fn box_gap2(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> f64 {
    let g = |alo: f64, ahi: f64, blo: f64, bhi: f64| (blo - ahi).max(alo - bhi).max(0.0);
    let (x, y, z) = (g(a.0.x, a.1.x, b.0.x, b.1.x), g(a.0.y, a.1.y, b.0.y, b.1.y), g(a.0.z, a.1.z, b.0.z, b.1.z));
    x * x + y * y + z * z
}

/// Does the segment `p`-`q` pierce the triangle (Moller-Trumbore on the segment)?
fn seg_tri_hit(p: Vec3, q: Vec3, t: &[Vec3; 3]) -> bool {
    let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
    let d = q - p;
    let pv = d.cross(e2);
    let det = e1.dot(pv);
    if det.abs() < 1e-300 {
        return false;
    }
    let inv = 1.0 / det;
    let s = p - t[0];
    let u = s.dot(pv) * inv;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let qv = s.cross(e1);
    let v = d.dot(qv) * inv;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    let s_par = e2.dot(qv) * inv;
    (0.0..=1.0).contains(&s_par)
}

fn tris_cross(t: &[Vec3; 3], u: &[Vec3; 3]) -> bool {
    (0..3).any(|i| seg_tri_hit(t[i], t[(i + 1) % 3], u) || seg_tri_hit(u[i], u[(i + 1) % 3], t))
}

/// Squared distance of the segments `p1`-`q1` and `p2`-`q2` (Ericson 5.1.9).
fn seg_seg_d2(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> f64 {
    let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
    let (a, e, f) = (d1.len2(), d2.len2(), d2.dot(r));
    let eps = 1e-300;
    let (s, t);
    if a <= eps && e <= eps {
        return r.len2();
    }
    if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= eps {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let den = a * e - b * b;
            let mut s0 = if den > 1e-30 * a * e { ((b * f - c * e) / den).clamp(0.0, 1.0) } else { 0.0 };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    ((p1 + d1 * s) - (p2 + d2 * t)).len2()
}

/// Squared distance of two triangles (0 when they cross).
fn tri_tri_d2(t: &[Vec3; 3], u: &[Vec3; 3]) -> f64 {
    if tris_cross(t, u) {
        return 0.0;
    }
    let mut m = f64::INFINITY;
    for i in 0..3 {
        m = m.min(dist2_tri(t[i], u[0], u[1], u[2])).min(dist2_tri(u[i], t[0], t[1], t[2]));
        for j in 0..3 {
            m = m.min(seg_seg_d2(t[i], t[(i + 1) % 3], u[j], u[(j + 1) % 3]));
        }
    }
    m
}

// ------------------------------------------------------------------ grid of live triangles

struct Grid {
    lo: Vec3,
    inv: f64,
    n: [i64; 3],
    cells: Vec<Vec<u32>>,
    stamp: Vec<u32>,
    cur: u32,
}

impl Grid {
    fn new(lo: Vec3, hi: Vec3, cell: f64, ntris: usize) -> Grid {
        let e = hi - lo;
        let mut cell = cell.max(1e-12);
        loop {
            let dims = [(e.x / cell).floor() as i64 + 1, (e.y / cell).floor() as i64 + 1, (e.z / cell).floor() as i64 + 1];
            if dims[0].saturating_mul(dims[1]).saturating_mul(dims[2]) <= 3_000_000 {
                return Grid { lo, inv: 1.0 / cell, n: dims, cells: vec![vec![]; (dims[0] * dims[1] * dims[2]) as usize], stamp: vec![0; ntris], cur: 0 };
            }
            cell *= 1.5;
        }
    }
    fn range(&self, a: Vec3, b: Vec3) -> ([i64; 3], [i64; 3]) {
        let f = |v: f64, lo: f64, n: i64| (((v - lo) * self.inv).floor() as i64).clamp(0, n - 1);
        ([f(a.x, self.lo.x, self.n[0]), f(a.y, self.lo.y, self.n[1]), f(a.z, self.lo.z, self.n[2])], [f(b.x, self.lo.x, self.n[0]), f(b.y, self.lo.y, self.n[1]), f(b.z, self.lo.z, self.n[2])])
    }
    fn each_cell(&self, a: Vec3, b: Vec3, mut f: impl FnMut(usize)) {
        let (i0, i1) = self.range(a, b);
        for x in i0[0]..=i1[0] {
            for y in i0[1]..=i1[1] {
                for z in i0[2]..=i1[2] {
                    f(((x * self.n[1] + y) * self.n[2] + z) as usize);
                }
            }
        }
    }
    fn insert(&mut self, t: u32, bx: (Vec3, Vec3)) {
        let mut ids = vec![];
        self.each_cell(bx.0, bx.1, |c| ids.push(c));
        for c in ids {
            self.cells[c].push(t);
        }
    }
    fn remove(&mut self, t: u32, bx: (Vec3, Vec3)) {
        let mut ids = vec![];
        self.each_cell(bx.0, bx.1, |c| ids.push(c));
        for c in ids {
            if let Some(i) = self.cells[c].iter().position(|&x| x == t) {
                self.cells[c].swap_remove(i);
            }
        }
    }
    /// Triangles registered in the cells of the box, each once.
    fn query(&mut self, a: Vec3, b: Vec3, out: &mut Vec<u32>) {
        out.clear();
        self.cur += 1;
        let cur = self.cur;
        let mut ids = vec![];
        self.each_cell(a, b, |c| ids.push(c));
        for c in ids {
            for &t in &self.cells[c] {
                if self.stamp[t as usize] != cur {
                    self.stamp[t as usize] = cur;
                    out.push(t);
                }
            }
        }
        out.sort_unstable();
    }
}

/// Crossing triangle pairs that share no vertex (up to `limit`).
fn count_crossings(pts: &[Vec3], tris: &[[u32; 3]], limit: usize) -> usize {
    if tris.is_empty() {
        return 0;
    }
    let tv: Vec<[Vec3; 3]> = tris.iter().map(|t| [pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]]).collect();
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    let mut e2 = 0.0;
    for t in &tv {
        let (l, h) = tri_box(t);
        lo = lo.min(l);
        hi = hi.max(h);
        e2 += (t[1] - t[0]).len2();
    }
    let mean = (e2 / tv.len() as f64).sqrt();
    let mut g = Grid::new(lo, hi, 2.0 * mean, tris.len());
    let boxes: Vec<(Vec3, Vec3)> = tv.iter().map(tri_box).collect();
    for (i, b) in boxes.iter().enumerate() {
        g.insert(i as u32, *b);
    }
    let (mut n, mut cand) = (0, vec![]);
    for i in 0..tv.len() {
        g.query(boxes[i].0, boxes[i].1, &mut cand);
        for &j in &cand {
            let j = j as usize;
            if j <= i || tris[i].iter().any(|v| tris[j].contains(v)) || box_gap2(boxes[i], boxes[j]) > 0.0 {
                continue;
            }
            if tris_cross(&tv[i], &tv[j]) {
                n += 1;
                if n >= limit {
                    return n;
                }
            }
        }
    }
    n
}

// ------------------------------------------------------------------ quadrics

type Quad = [f64; 10];

fn plane_quad(n: Vec3, d: f64, w: f64) -> Quad {
    [n.x * n.x, n.x * n.y, n.x * n.z, n.x * d, n.y * n.y, n.y * n.z, n.y * d, n.z * n.z, n.z * d, d * d].map(|v| v * w)
}

fn quad_eval(q: &Quad, p: Vec3) -> f64 {
    let v = q[0] * p.x * p.x + 2.0 * q[1] * p.x * p.y + 2.0 * q[2] * p.x * p.z + 2.0 * q[3] * p.x + q[4] * p.y * p.y + 2.0 * q[5] * p.y * p.z + 2.0 * q[6] * p.y + q[7] * p.z * p.z + 2.0 * q[8] * p.z + q[9];
    v.max(0.0)
}

/// The minimiser of the quadric when its 3x3 part is well conditioned.
fn quad_opt(q: &Quad) -> Option<Vec3> {
    let (a, b, c, d, e, f) = (q[0], q[1], q[2], q[4], q[5], q[7]);
    let tr = a + d + f;
    if !(tr > 0.0) {
        return None;
    }
    let det = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c);
    let s = tr / 3.0;
    if det.abs() < 1e-7 * s * s * s {
        return None;
    }
    let (r0, r1, r2) = (-q[3], -q[6], -q[8]);
    let x = (r0 * (d * f - e * e) - b * (r1 * f - e * r2) + c * (r1 * e - d * r2)) / det;
    let y = (a * (r1 * f - e * r2) - r0 * (b * f - e * c) + c * (b * r2 - r1 * c)) / det;
    let z = (a * (d * r2 - r1 * e) - b * (b * r2 - r1 * c) + r0 * (b * e - d * c)) / det;
    let p = Vec3::new(x, y, z);
    p.is_finite().then_some(p)
}

fn add_quad(a: &mut Quad, b: &Quad) {
    for i in 0..10 {
        a[i] += b[i];
    }
}

// ------------------------------------------------------------------ the decimator

struct Entry {
    cost: f64,
    a: u32,
    b: u32,
    va: u32,
    vb: u32,
}
impl PartialEq for Entry {
    fn eq(&self, o: &Entry) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Entry) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    // BinaryHeap is a max heap: the least cost (then the lower vertex numbers) is the greatest
    fn cmp(&self, o: &Entry) -> Ordering {
        o.cost.total_cmp(&self.cost).then(o.a.cmp(&self.a)).then(o.b.cmp(&self.b))
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Why {
    Stale,
    Topology,
    Flip,
    Deviation,
    Clearance,
}

struct Params<'b> {
    /// The input surface: new vertices and triangle centroids must stay within `tol_int` of it.
    orig_bvh: &'b Bvh,
    /// Internal bound on the distance of an original vertex from the new triangles.
    tol_int: f64,
    /// Sheet clearance: distances below this (to triangles not touching the collapse) are guarded.
    clear: f64,
    clear_factor: f64,
    target: usize,
    eps_cost: f64,
}

struct Dec<'a> {
    orig: &'a [Vec3],
    pts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    alive: Vec<bool>,
    vt: Vec<Vec<u32>>,
    vdead: Vec<bool>,
    q: Vec<Quad>,
    samples: Vec<Vec<u32>>,
    ver: Vec<u32>,
    /// Feature graph (boundary and crease edges): neighbours of each vertex.
    feat: Vec<Vec<u32>>,
    live: usize,
    grid: Grid,
    mark: Vec<u32>,
    mark_cur: u32,
    heap: BinaryHeap<Entry>,
    collapses: usize,
    refused: [usize; 4],
}

fn tri_pts(pts: &[Vec3], t: &[u32; 3]) -> [Vec3; 3] {
    [pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]]
}

impl<'a> Dec<'a> {
    fn new(orig: &'a [Vec3], nv: usize, tris0: &[[u32; 3]], feat: Vec<Vec<u32>>, p: &Params<'_>, mean_edge: f64) -> Dec<'a> {
        let mut vt = vec![vec![]; nv];
        for (i, t) in tris0.iter().enumerate() {
            for &v in t {
                vt[v as usize].push(i as u32);
            }
        }
        let mut q = vec![[0.0; 10]; nv];
        for t in tris0 {
            let [a, b, c] = tri_pts(orig, t);
            let n = (b - a).cross(c - a);
            let l = n.len();
            if l > 0.0 {
                let nn = n / l;
                let qq = plane_quad(nn, -nn.dot(a), 0.5 * l);
                for &v in t {
                    add_quad(&mut q[v as usize], &qq);
                }
            }
        }
        let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for t in tris0 {
            let (l, h) = tri_box(&tri_pts(orig, t));
            lo = lo.min(l);
            hi = hi.max(h);
        }
        let pad = Vec3::new(p.clear, p.clear, p.clear);
        let mut grid = Grid::new(lo - pad, hi + pad, 2.0 * mean_edge, tris0.len());
        for (i, t) in tris0.iter().enumerate() {
            grid.insert(i as u32, tri_box(&tri_pts(orig, t)));
        }
        Dec {
            orig,
            pts: orig[..nv].to_vec(),
            tris: tris0.to_vec(),
            alive: vec![true; tris0.len()],
            vt,
            vdead: vec![false; nv],
            q,
            samples: {
                // the original vertices, and each original triangle's centroid with its first corner
                let mut sm: Vec<Vec<u32>> = (0..nv as u32).map(|i| vec![i]).collect();
                for (i, t) in tris0.iter().enumerate() {
                    sm[t[0] as usize].push((nv + i) as u32);
                }
                sm
            },
            ver: vec![0; nv],
            feat,
            live: tris0.len(),
            grid,
            mark: vec![0; nv],
            mark_cur: 0,
            heap: BinaryHeap::new(),
            collapses: 0,
            refused: [0; 4],
        }
    }

    fn class(&self, v: u32) -> u8 {
        match self.feat[v as usize].len() {
            0 => 0,
            2 => 1,
            _ => 2,
        }
    }

    /// Where the edge may collapse to, best quadric error first (empty: the feature rules forbid it).
    fn candidates(&self, a: u32, b: u32, p: &Params<'_>) -> ([(f64, Vec3); 4], usize) {
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        let mut q = self.q[a as usize];
        add_quad(&mut q, &self.q[b as usize]);
        let len2 = pa.dist2(pb);
        let mut c: [(f64, Vec3); 4] = [(0.0, pa); 4];
        let mut n = 0;
        let mut push = |pos: Vec3, n: &mut usize| {
            c[*n] = (quad_eval(&q, pos) + p.eps_cost * len2, pos);
            *n += 1;
        };
        let (ca, cb) = (self.class(a), self.class(b));
        let ce = self.feat[a as usize].contains(&b);
        if ca == 0 && cb == 0 {
            if let Some(o) = quad_opt(&q) {
                let mid = (pa + pb) * 0.5;
                if o.dist2(mid) <= len2 {
                    push(o, &mut n);
                }
            }
            push(pa, &mut n);
            push(pb, &mut n);
            push((pa + pb) * 0.5, &mut n);
        } else if ce {
            if ca == 2 && cb == 2 {
                return (c, 0);
            }
            let (fa, fb) = (&self.feat[a as usize], &self.feat[b as usize]);
            if ca == 1 && cb == 1 && fa.iter().find(|&&x| x != b) == fb.iter().find(|&&x| x != a) {
                return (c, 0); // a crease loop of three would close on itself
            }
            if ca == 2 {
                push(pa, &mut n);
            } else if cb == 2 {
                push(pb, &mut n);
            } else {
                push(pa, &mut n);
                push(pb, &mut n);
            }
        } else if ca != 0 && cb != 0 {
            return (c, 0);
        } else if ca != 0 {
            push(pa, &mut n);
        } else {
            push(pb, &mut n);
        }
        for i in 1..n {
            let mut j = i;
            while j > 0 && c[j - 1].0 > c[j].0 {
                c.swap(j - 1, j);
                j -= 1;
            }
        }
        (c, n)
    }

    fn ring(&self, v: u32) -> Vec<u32> {
        let mut r: Vec<u32> = self.vt[v as usize].iter().flat_map(|&t| self.tris[t as usize]).filter(|&x| x != v).collect();
        r.sort_unstable();
        r.dedup();
        r
    }

    fn push_edge(&mut self, a: u32, b: u32, p: &Params<'_>) {
        let (a, b) = if a < b { (a, b) } else { (b, a) };
        let (c, n) = self.candidates(a, b, p);
        if n > 0 {
            self.heap.push(Entry { cost: c[0].0, a, b, va: self.ver[a as usize], vb: self.ver[b as usize] });
        }
    }

    /// Try the collapse of edge `a`-`b`; `None`: done.
    fn try_collapse(&mut self, a: u32, b: u32, p: &Params<'_>) -> Option<Why> {
        if self.vdead[a as usize] || self.vdead[b as usize] {
            return Some(Why::Stale);
        }
        let tab: Vec<u32> = self.vt[a as usize].iter().copied().filter(|&t| self.tris[t as usize].contains(&b)).collect();
        if tab.is_empty() || tab.len() > 2 {
            return Some(Why::Stale);
        }
        let (cands, nc) = self.candidates(a, b, p);
        if nc == 0 {
            return Some(Why::Topology);
        }
        // link condition
        let (ra, rb) = (self.ring(a), self.ring(b));
        let mut opp: Vec<u32> = tab.iter().map(|&t| *self.tris[t as usize].iter().find(|&&v| v != a && v != b).unwrap()).collect();
        opp.sort_unstable();
        let common: Vec<u32> = ra.iter().copied().filter(|v| *v != b && rb.contains(v)).collect();
        if common != opp {
            return Some(Why::Topology);
        }
        if tab.len() == 2 && self.vt[opp[0] as usize].len() == 3 && self.vt[opp[1] as usize].len() == 3 {
            return Some(Why::Topology); // a tetrahedron
        }
        if tab.len() == 1 && self.vt[a as usize].len() + self.vt[b as usize].len() == 2 {
            return Some(Why::Topology); // a lone triangle
        }
        let star: Vec<u32> = self.vt[a as usize].iter().chain(self.vt[b as usize].iter()).copied().filter(|t| !tab.contains(t)).collect();
        let mut ring: Vec<u32> = ra.iter().chain(rb.iter()).copied().filter(|&v| v != a && v != b).collect();
        ring.sort_unstable();
        ring.dedup();
        let mut last = Why::Flip;
        for ci in 0..nc {
            let pos = cands[ci].1;
            match self.check_position(a, b, pos, &tab, &star, &ring, p) {
                None => {
                    self.apply(a, b, pos, &tab, &star, p);
                    return None;
                }
                Some(w) => last = w,
            }
        }
        Some(last)
    }

    fn check_position(&mut self, a: u32, b: u32, pos: Vec3, tab: &[u32], star: &[u32], ring: &[u32], p: &Params<'_>) -> Option<Why> {
        let sub = |pts: &[Vec3], v: u32| if v == a || v == b { pos } else { pts[v as usize] };
        // flips and slivers
        let mut newtris: Vec<[Vec3; 3]> = Vec::with_capacity(star.len());
        for &t in star {
            let c = self.tris[t as usize];
            let old = tri_pts(&self.pts, &c);
            let new = [sub(&self.pts, c[0]), sub(&self.pts, c[1]), sub(&self.pts, c[2])];
            let (n0, n1) = ((old[1] - old[0]).cross(old[2] - old[0]), (new[1] - new[0]).cross(new[2] - new[0]));
            let (l0, l1) = (n0.len(), n1.len());
            let q = |l: f64, t: &[Vec3; 3]| l / (t[1] - t[0]).len2().max((t[2] - t[1]).len2()).max((t[0] - t[2]).len2()).max(1e-300);
            if !(l1 > 0.0) || n1.dot(n0) <= 0.05 * l0 * l1 || (q(l1, &new) < 1e-3 && q(l1, &new) < q(l0, &old)) {
                return Some(Why::Flip);
            }
            newtris.push(new);
        }
        // deviation of the original vertices of everything the collapse touches
        let t2 = p.tol_int * p.tol_int;
        let near = |s: Vec3, tl: &[[Vec3; 3]]| tl.iter().any(|t| dist2_tri(s, t[0], t[1], t[2]) <= t2);
        for &s in self.samples[a as usize].iter().chain(self.samples[b as usize].iter()) {
            if !near(self.orig[s as usize], &newtris) {
                return Some(Why::Deviation);
            }
        }
        for &x in ring {
            let tl: Vec<[Vec3; 3]> = self.vt[x as usize]
                .iter()
                .filter(|t| !tab.contains(t))
                .map(|&t| {
                    let c = self.tris[t as usize];
                    [sub(&self.pts, c[0]), sub(&self.pts, c[1]), sub(&self.pts, c[2])]
                })
                .collect();
            for &s in &self.samples[x as usize] {
                let o = self.orig[s as usize];
                if same(o, self.pts[x as usize]) {
                    continue;
                }
                if !near(o, &tl) {
                    return Some(Why::Deviation);
                }
            }
        }
        // sheets: triangles that do not touch the collapse must not come closer
        if p.clear > 0.0 && !newtris.is_empty() {
            self.mark_cur += 1;
            let mc = self.mark_cur;
            for &v in ring.iter().chain([a, b].iter()) {
                self.mark[v as usize] = mc;
            }
            let (mut lo, mut hi) = tri_box(&newtris[0]);
            for t in &newtris {
                let (l, h) = tri_box(t);
                lo = lo.min(l);
                hi = hi.max(h);
            }
            let pad = Vec3::new(p.clear, p.clear, p.clear);
            let mut cand = vec![];
            self.grid.query(lo - pad, hi + pad, &mut cand);
            let c2 = p.clear * p.clear;
            let nb: Vec<(Vec3, Vec3)> = newtris.iter().map(tri_box).collect();
            let old: Vec<[Vec3; 3]> = star.iter().chain(tab.iter()).map(|&t| tri_pts(&self.pts, &self.tris[t as usize])).collect();
            for u in cand {
                let uc = self.tris[u as usize];
                if !self.alive[u as usize] || uc.iter().any(|&v| self.mark[v as usize] == mc) {
                    continue;
                }
                let ut = tri_pts(&self.pts, &uc);
                let ub = tri_box(&ut);
                let mut dn = f64::INFINITY;
                for (i, t) in newtris.iter().enumerate() {
                    if box_gap2(nb[i], ub) < c2.min(dn) {
                        dn = dn.min(tri_tri_d2(t, &ut));
                    }
                }
                if dn < c2 {
                    let mut d_old = c2;
                    for t in &old {
                        d_old = d_old.min(tri_tri_d2(t, &ut));
                    }
                    if dn < p.clear_factor * p.clear_factor * d_old.min(c2) {
                        return Some(Why::Clearance);
                    }
                }
            }
        }
        // the new surface must stay near the input too (a spike is within reach of the vertices it covers but not of the surface)
        if p.orig_bvh.dist(pos) > p.tol_int {
            return Some(Why::Deviation);
        }
        for t in &newtris {
            if p.orig_bvh.dist((t[0] + t[1] + t[2]) / 3.0) > p.tol_int {
                return Some(Why::Deviation);
            }
        }
        None
    }

    fn apply(&mut self, a: u32, b: u32, pos: Vec3, tab: &[u32], star: &[u32], p: &Params<'_>) {
        let (keep, gone) = if self.samples[a as usize].len() > self.samples[b as usize].len() { (a, b) } else { (b, a) };
        for &t in star.iter().chain(tab.iter()) {
            let bx = tri_box(&tri_pts(&self.pts, &self.tris[t as usize]));
            self.grid.remove(t, bx);
        }
        for &t in tab {
            self.alive[t as usize] = false;
            self.live -= 1;
            let c = *self.tris[t as usize].iter().find(|&&v| v != a && v != b).unwrap();
            self.vt[c as usize].retain(|&x| x != t);
            self.vt[keep as usize].retain(|&x| x != t);
            self.vt[gone as usize].retain(|&x| x != t);
        }
        for &t in star {
            let tri = &mut self.tris[t as usize];
            let mut moved = false;
            for v in tri.iter_mut() {
                if *v == gone {
                    *v = keep;
                    moved = true;
                }
            }
            if moved {
                self.vt[keep as usize].push(t);
            }
        }
        self.vt[gone as usize].clear();
        self.vdead[gone as usize] = true;
        self.pts[keep as usize] = pos;
        let qg = self.q[gone as usize];
        add_quad(&mut self.q[keep as usize], &qg);
        let sg = std::mem::take(&mut self.samples[gone as usize]);
        self.samples[keep as usize].extend(sg);
        self.ver[keep as usize] += 1;
        // feature graph
        let fg = std::mem::take(&mut self.feat[gone as usize]);
        for x in fg {
            if x == keep {
                continue;
            }
            self.feat[x as usize].retain(|&y| y != gone);
            if !self.feat[x as usize].contains(&keep) {
                self.feat[x as usize].push(keep);
            }
            if !self.feat[keep as usize].contains(&x) {
                self.feat[keep as usize].push(x);
            }
        }
        self.feat[keep as usize].retain(|&y| y != gone && y != keep);
        for &t in &self.vt[keep as usize].clone() {
            let bx = tri_box(&tri_pts(&self.pts, &self.tris[t as usize]));
            self.grid.insert(t, bx);
        }
        self.collapses += 1;
        // new queue entries: the edges of the merged vertex and the edges opposite it
        let ring = self.ring(keep);
        for &x in &ring {
            self.push_edge(keep, x, p);
        }
        let ts = self.vt[keep as usize].clone();
        for t in ts {
            let c = self.tris[t as usize];
            let (x, y) = match c.iter().position(|&v| v == keep) {
                Some(0) => (c[1], c[2]),
                Some(1) => (c[2], c[0]),
                _ => (c[0], c[1]),
            };
            self.push_edge(x, y, p);
        }
    }

    fn run(&mut self, p: &Params<'_>) {
        let mut edges: Vec<(u32, u32)> = vec![];
        for t in &self.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push(if a < b { (a, b) } else { (b, a) });
            }
        }
        edges.sort_unstable();
        edges.dedup();
        for (a, b) in edges {
            self.push_edge(a, b, p);
        }
        while self.live > p.target {
            let Some(e) = self.heap.pop() else { break };
            if self.vdead[e.a as usize] || self.vdead[e.b as usize] || self.ver[e.a as usize] != e.va || self.ver[e.b as usize] != e.vb {
                continue;
            }
            match self.try_collapse(e.a, e.b, p) {
                None | Some(Why::Stale) => {}
                Some(Why::Topology) => self.refused[0] += 1,
                Some(Why::Flip) => self.refused[1] += 1,
                Some(Why::Deviation) => self.refused[2] += 1,
                Some(Why::Clearance) => self.refused[3] += 1,
            }
        }
    }

    /// The live triangles on compact vertex numbers.
    fn compact(&self) -> (Vec<Vec3>, Vec<[u32; 3]>) {
        let mut map = vec![u32::MAX; self.pts.len()];
        let (mut pts, mut tris) = (vec![], vec![]);
        for (i, t) in self.tris.iter().enumerate() {
            if !self.alive[i] {
                continue;
            }
            let mut r = [0u32; 3];
            for k in 0..3 {
                let v = t[k] as usize;
                if map[v] == u32::MAX {
                    map[v] = pts.len() as u32;
                    pts.push(self.pts[v]);
                }
                r[k] = map[v];
            }
            tris.push(r);
        }
        (pts, tris)
    }
}

// ------------------------------------------------------------------ entry points

fn bad(msg: impl Into<String>, hint: &str) -> Refusal {
    Refusal { code: "meshBadInput", message: msg.into(), hint: hint.to_string(), data: vec![] }
}

fn samples_of(pts: &[Vec3], tris: &[[u32; 3]]) -> Vec<Vec3> {
    let mut s: Vec<Vec3> = Vec::with_capacity(pts.len() + tris.len());
    let mut used = vec![false; pts.len()];
    for t in tris {
        for &v in t {
            if !used[v as usize] {
                used[v as usize] = true;
                s.push(pts[v as usize]);
            }
        }
        s.push((pts[t[0] as usize] + pts[t[1] as usize] + pts[t[2] as usize]) / 3.0);
    }
    s
}

/// Decimate a welded triangle mesh (manifold; open boundaries are kept as feature lines). See the module docs.
pub fn decimate(pts: &[Vec3], tris: &[[u32; 3]], o: &DecimateOpts) -> Result<Decimated, Refusal> {
    if !(o.tol.is_finite() && o.tol > 0.0) {
        return Err(bad("opts.decimate.tol (mm) is required and must be a positive number", "the largest distance the decimated mesh may have from the original, e.g. 0.05 for a scan with 0.02 noise"));
    }
    let mut rep = DecimateReport { input_triangles: tris.len(), input_vertices: pts.len(), tol: o.tol, ..Default::default() };
    let eu = crate::edge_use(tris);
    if !eu.nonmanifold.is_empty() {
        let list: Vec<Json> = eu.nonmanifold.iter().take(20).map(|&(h, _)| {
            let m = (pts[crate::he_from(tris, h) as usize] + pts[crate::he_to(tris, h) as usize]) * 0.5;
            Json::Arr(vec![Json::Num(m.x), Json::Num(m.y), Json::Num(m.z)])
        }).collect();
        return Err(Refusal {
            code: "meshNonManifold",
            message: format!("{} edges are used by more than two triangles: the mesh cannot be decimated", eu.nonmanifold.len()),
            hint: "separate the bodies that touch along these edges, or remove internal faces".into(),
            data: vec![("nonManifoldEdges", Json::Num(eu.nonmanifold.len() as f64)), ("positions", Json::Arr(list))],
        });
    }
    rep.boundary_edges = eu.open.len();
    rep.watertight = eu.open.is_empty();
    // feature graph: boundary edges, creases
    let nv = pts.len();
    let mut feat: Vec<Vec<u32>> = vec![vec![]; nv];
    let mut add = |a: u32, b: u32, feat: &mut Vec<Vec<u32>>| {
        if !feat[a as usize].contains(&b) {
            feat[a as usize].push(b);
            feat[b as usize].push(a);
        }
    };
    for &h in &eu.open {
        add(crate::he_from(tris, h), crate::he_to(tris, h), &mut feat);
    }
    let nb_edges = feat.iter().map(|f| f.len()).sum::<usize>() / 2;
    if let Some(deg) = o.keep_angle_deg {
        let cmin = deg.to_radians().cos();
        let nrm: Vec<Vec3> = tris.iter().map(|t| { let n = crate::tri_cross(pts, t); n.try_normalize().unwrap_or(Vec3::new(0.0, 0.0, 0.0)) }).collect();
        for h in 0..tris.len() * 3 {
            let g = eu.twin[h];
            if g != usize::MAX && h < g {
                let (n1, n2) = (nrm[h / 3], nrm[g / 3]);
                if n1.dot(n2) < cmin {
                    add(crate::he_from(tris, h), crate::he_to(tris, h), &mut feat);
                }
            }
        }
    }
    rep.crease_edges = feat.iter().map(|f| f.len()).sum::<usize>() / 2 - nb_edges;
    let keep_all = |rep: &mut DecimateReport, why: &'static str| {
        rep.triangles = tris.len();
        rep.vertices = rep.input_vertices;
        rep.stopped_by = why;
    };
    let target = o.target_triangles.unwrap_or(4).max(4);
    if tris.len() <= target || tris.len() < 8 {
        keep_all(&mut rep, "target");
        return Ok(Decimated { pts: pts.to_vec(), tris: tris.to_vec(), report: rep });
    }
    let mean_edge = {
        let s: f64 = tris.iter().map(|t| pts[t[0] as usize].dist2(pts[t[1] as usize])).sum();
        (s / tris.len() as f64).sqrt()
    };
    // the input, for the independent measurement
    let orig_tris: Vec<[Vec3; 3]> = tris.iter().map(|t| tri_pts(pts, t)).collect();
    let orig_samples = samples_of(pts, tris);
    let orig_bvh = Bvh::new(orig_tris);
    let orig_ext: Vec<Vec3> = pts.iter().copied().chain(tris.iter().map(|t| (pts[t[0] as usize] + pts[t[1] as usize] + pts[t[2] as usize]) / 3.0)).collect();
    let mut tol_int = 0.85 * o.tol;
    let mut factor = 0.8;
    for attempt in 1..=5 {
        rep.attempts = attempt;
        let params = Params { orig_bvh: &orig_bvh, tol_int, clear: o.tol, clear_factor: factor, target, eps_cost: 1e-8 * mean_edge * mean_edge };
        let mut d = Dec::new(&orig_ext, pts.len(), tris, feat.clone(), &params, mean_edge);
        d.run(&params);
        let (rp, rt) = d.compact();
        // measurement, both ways
        let res_bvh = Bvh::new(rt.iter().map(|t| tri_pts(&rp, t)).collect());
        let (mut mx, mut s2, mut n) = (0.0f64, 0.0, 0usize);
        let (mut m_in, mut m_res) = (0.0f64, 0.0f64);
        for &s in &orig_samples {
            let x = res_bvh.dist(s);
            m_in = m_in.max(x);
            s2 += x * x;
            n += 1;
        }
        for s in samples_of(&rp, &rt) {
            let x = orig_bvh.dist(s);
            m_res = m_res.max(x);
            s2 += x * x;
            n += 1;
        }
        mx = mx.max(m_in).max(m_res);
        let crossings = count_crossings(&rp, &rt, 1000);
        let in_cross = if crossings > 0 { count_crossings(pts, tris, 100_000) } else { 0 };
        let eu2 = crate::edge_use(&rt);
        let topo_ok = eu2.nonmanifold.is_empty() && eu2.open.len() == eu.open.len();
        rep.collapses = d.collapses;
        rep.refused_topology = d.refused[0];
        rep.refused_flip = d.refused[1];
        rep.refused_deviation = d.refused[2];
        rep.refused_clearance = d.refused[3];
        rep.dev_max = mx;
        rep.dev_rms = if n > 0 { (s2 / n as f64).sqrt() } else { 0.0 };
        rep.dev_samples = n;
        rep.dev_input_to_result = m_in;
        rep.dev_result_to_input = m_res;
        rep.self_intersections = crossings;
        rep.input_self_intersections = in_cross;
        rep.internal_tol = tol_int;
        if mx <= o.tol && crossings <= in_cross && topo_ok {
            rep.triangles = rt.len();
            rep.vertices = rp.len();
            rep.stopped_by = if rt.len() <= target { "target" } else { "tolerance" };
            return Ok(Decimated { pts: rp, tris: rt, report: rep });
        }
        if mx > o.tol {
            tol_int *= (0.9 * o.tol / mx).clamp(0.25, 0.8);
        } else {
            tol_int *= 0.7;
        }
        factor = 0.95;
    }
    // nothing within the bound: the input itself is within it
    rep.collapses = 0;
    rep.dev_max = 0.0;
    rep.dev_rms = 0.0;
    rep.dev_input_to_result = 0.0;
    rep.dev_result_to_input = 0.0;
    rep.self_intersections = 0;
    rep.input_self_intersections = 0;
    keep_all(&mut rep, "verification");
    Ok(Decimated { pts: pts.to_vec(), tris: tris.to_vec(), report: rep })
}

/// In place on the welded mesh of `meshToSolid` (`opts.decimate`).
pub fn apply(pts: &mut Vec<Vec3>, tris: &mut Vec<[u32; 3]>, o: &DecimateOpts) -> Result<DecimateReport, Refusal> {
    let r = decimate(pts, tris, o)?;
    *pts = r.pts;
    *tris = r.tris;
    Ok(r.report)
}

/// The `meshDecimate` request: weld, decimate, the mesh and the report.
pub fn run(msg: &Json) -> Result<Decimated, Refusal> {
    let (pos, idx) = mesh_of(msg)?;
    let opts = msg.get("opts");
    let o = opts.and_then(|m| m.get("decimate").or(Some(m))).map(DecimateOpts::from_json).unwrap_or_else(|| DecimateOpts::new(f64::NAN));
    let weld_tol = opts.and_then(|m| m.get_f64("weldTol")).filter(|v| *v > 0.0 && v.is_finite());
    if pos.len() % 3 != 0 {
        return Err(bad(format!("{} position values is not a multiple of 3", pos.len()), "send x, y, z per vertex"));
    }
    let nv = pos.len() / 3;
    let raw: Vec<Vec3> = (0..nv).map(|i| Vec3::new(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2])).collect();
    if let Some(i) = raw.iter().position(|p| !p.is_finite()) {
        return Err(bad(format!("vertex {i} is not a finite point"), "remove NaN / infinite coordinates"));
    }
    let corners: Vec<u32> = match idx {
        Some(ix) => {
            if ix.len() % 3 != 0 {
                return Err(bad(format!("{} indices is not a multiple of 3", ix.len()), "send three vertex indices per triangle"));
            }
            if let Some(&b) = ix.iter().find(|&&i| i as usize >= nv) {
                return Err(bad(format!("index {b} is out of range ({nv} vertices)"), "indices count from 0"));
            }
            ix
        }
        None => {
            if nv % 3 != 0 {
                return Err(bad(format!("{nv} vertices of a triangle soup is not a multiple of 3"), "send nine values per triangle, or indices"));
            }
            (0..nv as u32).collect()
        }
    };
    if corners.is_empty() {
        return Err(Refusal { code: "meshDegenerate", message: "the mesh has no triangles".into(), hint: "send at least one triangle".into(), data: vec![] });
    }
    let (lo, hi) = crate::bbox(&raw);
    let diag = (hi - lo).len();
    if !(diag > 0.0) || !(diag < 1e12) {
        return Err(bad("the coordinates are degenerate or beyond 1e12", "send coordinates in model units (mm)"));
    }
    let (wp, map) = crate::weld(&raw, weld_tol.unwrap_or(1e-6 * diag), lo);
    let mut tris: Vec<[u32; 3]> = corners.chunks_exact(3).map(|t| [map[t[0] as usize], map[t[1] as usize], map[t[2] as usize]]).filter(|w| w[0] != w[1] && w[1] != w[2] && w[2] != w[0]).collect();
    crate::drop_duplicates(&mut tris);
    if tris.is_empty() {
        return Err(Refusal { code: "meshDegenerate", message: "no usable triangle after welding".into(), hint: "check opts.weldTol".into(), data: vec![] });
    }
    let mut r = decimate(&wp, &tris, &o)?;
    r.report.input_triangles = corners.len() / 3;
    r.report.input_vertices = nv;
    Ok(r)
}
