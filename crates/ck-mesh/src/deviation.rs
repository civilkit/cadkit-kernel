//! Sampled two-sided distance between the converted body and the input mesh: points of the mesh (corners and triangle
//! centroids) to a fine tessellation of the body, and points of that tessellation to the mesh. A Hausdorff estimate: it
//! misses a deviation between samples and it includes the chord error of the tessellation (`chord`, reported).

use crate::Closed;
use ck_geom::Vec3;
use ck_topo::body::Body;

#[derive(Debug, Clone, Default)]
pub struct Deviation {
    /// Largest and root-mean-square distance over all samples, both directions.
    pub max: f64,
    pub rms: f64,
    /// Mesh to body and body to mesh separately.
    pub max_mesh_to_body: f64,
    pub max_body_to_mesh: f64,
    pub samples: usize,
    /// Chord height of the body tessellation used (its own error is part of the numbers).
    pub chord: f64,
    /// Where the largest deviation is.
    pub at: Vec3,
}

/// Closest point on triangle (Ericson, Real-Time Collision Detection 5.1.5): squared distance.
fn dist2_tri(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.len2();
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return bp.len2();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).len2();
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return cp.len2();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).len2();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).len2();
    }
    let den = 1.0 / (va + vb + vc);
    let (v, w) = (vb * den, vc * den);
    (p - (a + ab * v + ac * w)).len2()
}

/// A bounding-volume tree over triangles for nearest-point queries.
pub struct Bvh {
    tris: Vec<[Vec3; 3]>,
    nodes: Vec<Node>,
    order: Vec<u32>,
}

struct Node {
    lo: Vec3,
    hi: Vec3,
    /// Leaf: first index into `order` and count; inner: children.
    a: u32,
    b: u32,
    leaf: bool,
}

impl Bvh {
    pub fn new(tris: Vec<[Vec3; 3]>) -> Bvh {
        let mut order: Vec<u32> = (0..tris.len() as u32).collect();
        let cent: Vec<Vec3> = tris.iter().map(|t| (t[0] + t[1] + t[2]) / 3.0).collect();
        let mut nodes = vec![];
        fn build(tris: &[[Vec3; 3]], cent: &[Vec3], order: &mut [u32], off: u32, nodes: &mut Vec<Node>) -> u32 {
            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = -lo;
            for &i in order.iter() {
                for p in tris[i as usize] {
                    lo = lo.min(p);
                    hi = hi.max(p);
                }
            }
            let id = nodes.len() as u32;
            nodes.push(Node { lo, hi, a: off, b: order.len() as u32, leaf: true });
            if order.len() > 8 {
                let e = hi - lo;
                let ax = if e.x >= e.y && e.x >= e.z { 0 } else if e.y >= e.z { 1 } else { 2 };
                let m = order.len() / 2;
                order.select_nth_unstable_by(m, |&x, &y| cent[x as usize].get(ax).total_cmp(&cent[y as usize].get(ax)));
                let (l, r) = order.split_at_mut(m);
                let a = build(tris, cent, l, off, nodes);
                let b = build(tris, cent, r, off + m as u32, nodes);
                nodes[id as usize] = Node { lo, hi, a, b, leaf: false };
            }
            id
        }
        if !tris.is_empty() {
            build(&tris, &cent, &mut order, 0, &mut nodes);
        }
        Bvh { tris, nodes, order }
    }

    fn box_d2(n: &Node, p: Vec3) -> f64 {
        let d = |v: f64, lo: f64, hi: f64| if v < lo { lo - v } else if v > hi { v - hi } else { 0.0 };
        let (x, y, z) = (d(p.x, n.lo.x, n.hi.x), d(p.y, n.lo.y, n.hi.y), d(p.z, n.lo.z, n.hi.z));
        x * x + y * y + z * z
    }

    /// Distance from `p` to the nearest triangle.
    pub fn dist(&self, p: Vec3) -> f64 {
        if self.nodes.is_empty() {
            return f64::INFINITY;
        }
        let mut best = f64::INFINITY;
        let mut stack = vec![0u32];
        while let Some(i) = stack.pop() {
            let n = &self.nodes[i as usize];
            if Self::box_d2(n, p) >= best {
                continue;
            }
            if n.leaf {
                for &t in &self.order[n.a as usize..(n.a + n.b) as usize] {
                    let [a, b, c] = self.tris[t as usize];
                    best = best.min(dist2_tri(p, a, b, c));
                }
            } else {
                let (ca, cb) = (&self.nodes[n.a as usize], &self.nodes[n.b as usize]);
                let (da, db) = (Self::box_d2(ca, p), Self::box_d2(cb, p));
                if da < db {
                    stack.push(n.b);
                    stack.push(n.a);
                } else {
                    stack.push(n.a);
                    stack.push(n.b);
                }
            }
        }
        best.sqrt()
    }
}

/// Up to `cap` sample points of a triangle set: corners and centroids, evenly strided.
fn samples(tris: &[[Vec3; 3]], cap: usize) -> Vec<Vec3> {
    let mut all: Vec<Vec3> = Vec::with_capacity(tris.len() * 2);
    for t in tris {
        all.push(t[0]);
        all.push((t[0] + t[1] + t[2]) / 3.0);
    }
    if all.len() <= cap {
        return all;
    }
    let step = all.len() as f64 / cap as f64;
    (0..cap).map(|i| all[(i as f64 * step) as usize]).collect()
}

/// Deviation of `body` from the mesh `m`, from up to 20 000 samples each way.
pub fn measure(body: &Body, m: &Closed) -> Deviation {
    let mesh_tris: Vec<[Vec3; 3]> = m.tris.iter().map(|t| [m.pts[t[0] as usize], m.pts[t[1] as usize], m.pts[t[2] as usize]]).collect();
    let (lo, hi) = body.bbox();
    let mut tol = ck_tess::curvature::Tol::for_bbox(lo, hi);
    // a quarter of the viewer's chord: the tessellation's own error stays small next to the fit's
    tol.chord *= 0.25;
    let bm = ck_tess::mesh::body_mesh(body, &tol);
    let p = &bm.positions;
    let body_tris: Vec<[Vec3; 3]> = (0..p.len() / 9)
        .map(|i| {
            let v = |k: usize| Vec3::new(p[9 * i + 3 * k] as f64, p[9 * i + 3 * k + 1] as f64, p[9 * i + 3 * k + 2] as f64);
            [v(0), v(1), v(2)]
        })
        .collect();
    let mut d = Deviation { chord: tol.chord, ..Default::default() };
    let (mut sum2, mut n) = (0.0, 0usize);
    let bb = Bvh::new(body_tris.clone());
    for s in samples(&mesh_tris, 20_000) {
        let x = bb.dist(s);
        sum2 += x * x;
        n += 1;
        if x > d.max_mesh_to_body {
            d.max_mesh_to_body = x;
            if x > d.max {
                d.max = x;
                d.at = s;
            }
        }
    }
    let mb = Bvh::new(mesh_tris);
    for s in samples(&body_tris, 20_000) {
        let x = mb.dist(s);
        sum2 += x * x;
        n += 1;
        if x > d.max_body_to_mesh {
            d.max_body_to_mesh = x;
            if x > d.max {
                d.max = x;
                d.at = s;
            }
        }
    }
    d.samples = n;
    d.rms = if n > 0 { (sum2 / n as f64).sqrt() } else { 0.0 };
    d
}
