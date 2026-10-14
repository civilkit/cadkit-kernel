//! Shared read-only geometry for the rule engine: face facts, the display
//! triangulation with a BVH, named edges, and cached ray samples.

use crate::bvh::{ray_triangle, triangle_bvh, Bvh};
use ck_core::linalg::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId};
use std::cell::OnceCell;

/// One outward-oriented triangle of the display mesh.
#[derive(Clone, Copy)]
pub struct Tri {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    /// Outward unit normal (winding, flipped to agree with the vertex normals).
    pub n: Vec3,
    pub area: f64,
    /// Index into [`Ctx::names`] / `facts.faces`.
    pub face: usize,
}

/// A surface sample: point, outward normal, face index.
#[derive(Clone, Copy)]
pub struct Sample {
    pub p: Vec3,
    pub n: Vec3,
    pub face: usize,
}

/// Local thickness measured inward from a sample.
#[derive(Clone, Copy)]
pub struct WallHit {
    pub src: usize,
    pub hit: usize,
    pub t: f64,
    pub p: Vec3,
}

/// A named edge chain (wire numbering) with the faces it joins.
pub struct NamedEdge {
    pub key: String,
    pub first: EdgeId,
    pub faces: Vec<FaceId>,
}

struct Mesh {
    tris: Vec<Tri>,
    bvh: Bvh,
    failed: Vec<usize>,
}

pub struct Ctx<'a> {
    pub body: &'a Body,
    /// Persistent face names in `face_ids()` order.
    pub names: Vec<String>,
    pub fids: Vec<FaceId>,
    pub n_samples: usize,
    mesh: OnceCell<Mesh>,
    samples: OnceCell<Vec<Sample>>,
    walls: OnceCell<Vec<WallHit>>,
    edges: OnceCell<Vec<NamedEdge>>,
    stats: OnceCell<(Vec<f64>, Vec<Vec3>, f64)>,
}

fn v3(p: &[f32], i: usize) -> Vec3 {
    Vec3::new(p[i] as f64, p[i + 1] as f64, p[i + 2] as f64)
}

impl<'a> Ctx<'a> {
    pub fn new(body: &'a Body, n_samples: usize) -> Ctx<'a> {
        let fids: Vec<FaceId> = body.face_ids().collect();
        let names = fids.iter().map(|&f| body.face_id_str(f)).collect();
        Ctx {
            body,
            names,
            fids,
            n_samples,
            mesh: OnceCell::new(),
            samples: OnceCell::new(),
            walls: OnceCell::new(),
            edges: OnceCell::new(),
            stats: OnceCell::new(),
        }
    }

    fn mesh(&self) -> &Mesh {
        self.mesh.get_or_init(|| {
            let t = ck_tess::mesh::tessellate(self.body);
            let m = &t.mesh;
            let mut tris = Vec::with_capacity(m.tri_face.len());
            for (k, &fi) in m.tri_face.iter().enumerate() {
                if 9 * k + 9 > m.positions.len() || 9 * k + 9 > m.normals.len() {
                    break;
                }
                let (a, b, c) = (v3(&m.positions, 9 * k), v3(&m.positions, 9 * k + 3), v3(&m.positions, 9 * k + 6));
                let cr = (b - a).cross(c - a);
                let area = 0.5 * cr.len();
                if area < 1e-14 || fi < 0 || fi as usize >= self.fids.len() {
                    continue;
                }
                let mut n = cr.normalize();
                let vn = v3(&m.normals, 9 * k) + v3(&m.normals, 9 * k + 3) + v3(&m.normals, 9 * k + 6);
                if n.dot(vn) < 0.0 {
                    n = -n;
                }
                tris.push(Tri { a, b, c, n, area, face: fi as usize });
            }
            let pos: Vec<Vec3> = tris.iter().flat_map(|t| [t.a, t.b, t.c]).collect();
            let idx: Vec<[u32; 3]> = (0..tris.len() as u32).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect();
            let bvh = triangle_bvh(&pos, &idx);
            Mesh { tris, bvh, failed: m.failed.clone() }
        })
    }

    fn stats(&self) -> &(Vec<f64>, Vec<Vec3>, f64) {
        self.stats.get_or_init(|| {
            let n = self.fids.len();
            let (mut area, mut cen) = (vec![0.0; n], vec![Vec3::ZERO; n]);
            let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
            for t in self.tris() {
                area[t.face] += t.area;
                cen[t.face] = cen[t.face] + (t.a + t.b + t.c) * (t.area / 3.0);
                for p in [t.a, t.b, t.c] {
                    lo = lo.min(p);
                    hi = hi.max(p);
                }
            }
            for i in 0..n {
                if area[i] > 0.0 {
                    cen[i] = cen[i] * (1.0 / area[i]);
                }
            }
            let diag = if lo.is_finite() && hi.is_finite() { (hi - lo).len() } else { 0.0 };
            (area, cen, diag.max(1e-9))
        })
    }

    /// Volume of each lump (`lump_ids()` order) from the outward-oriented mesh
    /// (divergence theorem); negative = inside-out. Voids subtract.
    pub fn lump_volumes(&self) -> Vec<f64> {
        let mut fv = vec![0.0; self.fids.len()];
        for t in self.tris() {
            fv[t.face] += (t.a + t.b + t.c).dot(t.n) * t.area / 9.0;
        }
        let idx: std::collections::HashMap<FaceId, usize> = self.fids.iter().enumerate().map(|(i, &f)| (f, i)).collect();
        self.body
            .lump_ids()
            .map(|l| ck_topo::mass::lump_faces(self.body, l).iter().filter_map(|f| idx.get(f)).map(|&i| fv[i]).sum())
            .collect()
    }

    /// Faces whose triangulation failed, if the mesh has been built (else empty).
    pub fn skipped_if_built(&self) -> &[usize] {
        self.mesh.get().map_or(&[], |m| &m.failed)
    }

    /// Bounding-box diagonal of the mesh (mm).
    pub fn diag(&self) -> f64 {
        self.stats().2
    }

    /// Mesh area of face `i`.
    pub fn area(&self, i: usize) -> f64 {
        self.stats().0[i]
    }

    /// Area centroid of face `i`'s mesh.
    pub fn centroid(&self, i: usize) -> Vec3 {
        self.stats().1[i]
    }

    pub fn tris(&self) -> &[Tri] {
        &self.mesh().tris
    }

    /// Face indices whose triangulation failed (the geometric rules cannot see them).
    pub fn skipped(&self) -> &[usize] {
        &self.mesh().failed
    }

    /// Nearest hit of the ray `o + t d` (t in (0, tmax]): (t, triangle index).
    pub fn cast(&self, o: Vec3, d: Vec3, tmax: f64) -> Option<(f64, usize)> {
        let m = self.mesh();
        let mut best: Option<(f64, usize)> = None;
        for (i, tin) in m.bvh.query_ray(o, d, 0.0, tmax) {
            if let Some((bt, _)) = best {
                if tin > bt {
                    break;
                }
            }
            let t = &m.tris[i];
            if let Some(h) = ray_triangle(o, d, t.a, t.b, t.c) {
                if h <= tmax && best.map_or(true, |(bt, _)| h < bt) {
                    best = Some((h, i));
                }
            }
        }
        best
    }

    /// Deterministic, area-proportional surface samples (at least one per face).
    pub fn samples(&self) -> &[Sample] {
        self.samples.get_or_init(|| {
            let tris = self.tris();
            let mut by_face: Vec<Vec<usize>> = vec![vec![]; self.fids.len()];
            for (i, t) in tris.iter().enumerate() {
                by_face[t.face].push(i);
            }
            let total: f64 = tris.iter().map(|t| t.area).sum::<f64>().max(1e-12);
            let mut out = vec![];
            for (fi, ts) in by_face.iter().enumerate() {
                if ts.is_empty() {
                    continue;
                }
                let fa: f64 = ts.iter().map(|&i| tris[i].area).sum();
                let k = ((self.n_samples as f64 * fa / total).ceil() as usize).clamp(1, 600);
                let mut cum = Vec::with_capacity(ts.len());
                let mut acc = 0.0;
                for &i in ts {
                    acc += tris[i].area;
                    cum.push(acc);
                }
                let mut ti = 0;
                for j in 0..k {
                    let target = (j as f64 + 0.5) / k as f64 * fa;
                    while ti + 1 < ts.len() && cum[ti] < target {
                        ti += 1;
                    }
                    let (mut u, mut v) = ((0.5 + 0.7548776662466927 * j as f64).fract(), (0.5 + 0.5698402909980532 * j as f64).fract());
                    if u + v > 1.0 {
                        (u, v) = (1.0 - u, 1.0 - v);
                    }
                    let t = &tris[ts[ti]];
                    // pulled 10% towards the centroid so a sample never sits on a mesh edge
                    // (a ray along a wall would graze the neighbouring triangles)
                    let (u, v) = (0.0333333 + 0.9 * u, 0.0333333 + 0.9 * v);
                    out.push(Sample { p: t.a + (t.b - t.a) * u + (t.c - t.a) * v, n: t.n, face: fi });
                }
            }
            out
        })
    }

    /// Inward rays from every sample: the distance to the exit surface,
    /// kept where that surface faces along the ray (within ~45 degrees).
    pub fn walls(&self) -> &[WallHit] {
        self.walls.get_or_init(|| {
            let tris = self.tris();
            let eps = 1e-6 * self.diag();
            let mut out = vec![];
            for s in self.samples() {
                let d = -s.n;
                if let Some((t, i)) = self.cast(s.p + d * eps, d, self.diag() * 2.0) {
                    let h = &tris[i];
                    if h.n.dot(d) > 0.7 {
                        out.push(WallHit { src: s.face, hit: h.face, t: t + eps, p: s.p });
                    }
                }
            }
            out
        })
    }

    /// Named edge chains (same naming as `body.edges[].key`).
    pub fn edges(&self) -> &[NamedEdge] {
        self.edges.get_or_init(|| {
            let b = self.body;
            let mut out = vec![];
            for (seq, chain) in b.wire_numbered().0 {
                let e = chain[0];
                if b.is_split_rail(e) {
                    continue;
                }
                let mut fs: Vec<FaceId> = b.edge_faces(e);
                let mut names: Vec<String> = fs.iter().map(|&f| b.face_id_str(f)).collect();
                names.sort();
                names.dedup();
                if names.is_empty() {
                    continue;
                }
                let key = if names.len() >= 2 { ck_topo::body::edge_key(&names[0], &names[1], seq) } else { format!("{}#{}", names[0], seq) };
                fs.dedup();
                out.push(NamedEdge { key, first: e, faces: fs });
            }
            out
        })
    }

    /// Wire key of the chain that contains edge `e` (any member).
    pub fn edge_key_of(&self, e: EdgeId) -> Option<String> {
        let b = self.body;
        for (seq, chain) in b.wire_numbered().0 {
            if chain.contains(&e) {
                let ne = self.edges().iter().find(|n| n.first == chain[0])?;
                let _ = seq;
                return Some(ne.key.clone());
            }
        }
        None
    }

    pub fn face_index(&self, f: FaceId) -> usize {
        self.fids.iter().position(|&x| x == f).unwrap_or(0)
    }

    /// Height range of the mesh along unit `up`.
    pub fn height_range(&self, up: Vec3) -> (f64, f64) {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for t in self.tris() {
            for p in [t.a, t.b, t.c] {
                let h = p.dot(up);
                lo = lo.min(h);
                hi = hi.max(h);
            }
        }
        (lo, hi)
    }
}
