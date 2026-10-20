//! Kernel body from a baked scene: every SketchUp face becomes one planar face (with its holes) of a single body,
//! vertices welded by position and edges shared between neighbouring faces. This is the same kind of body the other
//! importers build (planar faces on `ck-topo`), so tessellation, inspection and the viewer treat it like any imported
//! body. A SketchUp model is generally a surface model, not a solid: the body is closed only where the model is.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Materials are not part of `ck-topo`: they travel beside the body as [`SkpBody::face_materials`].

use crate::error::{SkpError, R};
use crate::scene::{PolyFace, Scene, SceneMaterial};
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceName};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct BodyOptions {
    /// Vertices closer than this (mm) are welded.
    pub weld_tol_mm: f64,
    /// Most faces accepted (the kernel's per-face work makes very large imports slow).
    pub max_faces: usize,
}

impl Default for BodyOptions {
    fn default() -> Self {
        BodyOptions { weld_tol_mm: 1e-4, max_faces: 50_000 }
    }
}

#[derive(Debug)]
pub struct SkpBody {
    pub body: Body,
    /// Wire id of every face (`skp.F<n>`), in creation order, and its [`Scene::materials`] index.
    pub face_ids: Vec<String>,
    pub face_materials: Vec<usize>,
    pub materials: Vec<SceneMaterial>,
    pub skipped_faces: usize,
    /// Edges used by exactly one face (open boundary) and by more than two (non manifold).
    pub open_edges: usize,
    pub non_manifold_edges: usize,
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

struct Weld {
    tol: f64,
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
    pts: Vec<[f64; 3]>,
}

impl Weld {
    fn key(&self, p: [f64; 3]) -> (i64, i64, i64) {
        ((p[0] / self.tol).floor() as i64, (p[1] / self.tol).floor() as i64, (p[2] / self.tol).floor() as i64)
    }
    /// Index of the welded point near `p`, adding it when none is within the tolerance.
    fn find_or_add(&mut self, p: [f64; 3]) -> u32 {
        let k = self.key(p);
        for dx in -1..=1i64 {
            for dy in -1..=1i64 {
                for dz in -1..=1i64 {
                    if let Some(v) = self.cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
                        for &i in v {
                            let q = self.pts[i as usize];
                            let d2 = (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) + (q[2] - p[2]).powi(2);
                            if d2 <= self.tol * self.tol {
                                return i;
                            }
                        }
                    }
                }
            }
        }
        let i = self.pts.len() as u32;
        self.pts.push(p);
        self.cells.entry(k).or_default().push(i);
        i
    }
}

fn signed_area_about(pts: &[[f64; 3]], n: Vec3) -> f64 {
    // area vector of the polygon dotted with n
    let mut a = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        a = a + Vec3::new(p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]);
    }
    dot(a, n) * 0.5
}

/// Build the body from `scene` (baked with `keep_polygons`).
pub fn build_body(scene: &Scene, opt: &BodyOptions) -> R<SkpBody> {
    let polys: &[PolyFace] = &scene.polygons;
    if polys.is_empty() {
        return Err(SkpError::new("body.empty", "the scene has no faces (was it baked with keep_polygons?)"));
    }
    if polys.len() > opt.max_faces {
        return Err(SkpError::new("limit.faces", format!("{} faces exceed the import limit of {}; hide or delete parts of the model, or raise the limit", polys.len(), opt.max_faces)));
    }
    let mut weld = Weld { tol: opt.weld_tol_mm.max(1e-9), cells: HashMap::new(), pts: vec![] };
    let mut body = Body::new();
    let lump = body.add_lump();
    let shell = body.add_shell(lump);
    let mut verts: HashMap<u32, ck_topo::body::VertexId> = HashMap::new();
    let mut edges: HashMap<(u32, u32), ck_topo::body::EdgeId> = HashMap::new();
    let mut edge_uses: HashMap<(u32, u32), u32> = HashMap::new();
    let mut face_ids = vec![];
    let mut face_materials = vec![];
    let mut skipped = 0usize;

    for pf in polys {
        // weld and clean the loops
        let mut rings: Vec<Vec<u32>> = vec![];
        for l in &pf.loops {
            let mut r: Vec<u32> = Vec::with_capacity(l.len());
            for p in l {
                if !p.iter().all(|x| x.is_finite()) {
                    continue;
                }
                let i = weld.find_or_add(*p);
                if r.last() != Some(&i) {
                    r.push(i);
                }
            }
            while r.len() > 1 && r[0] == r[r.len() - 1] {
                r.pop();
            }
            if r.len() >= 3 {
                rings.push(r);
            } else if rings.is_empty() {
                break; // the outer loop is unusable
            }
        }
        if rings.is_empty() {
            skipped += 1;
            continue;
        }
        let ring_pts = |r: &Vec<u32>| -> Vec<[f64; 3]> { r.iter().map(|&i| weld.pts[i as usize]).collect() };
        // plane normal: the stored front normal, else Newell
        let mut n = Vec3::new(pf.normal[0], pf.normal[1], pf.normal[2]);
        let outer_pts = ring_pts(&rings[0]);
        let nw = {
            let mut a = Vec3::new(0.0, 0.0, 0.0);
            for i in 0..outer_pts.len() {
                let (p, q) = (outer_pts[i], outer_pts[(i + 1) % outer_pts.len()]);
                a = a + Vec3::new((p[1] - q[1]) * (p[2] + q[2]), (p[2] - q[2]) * (p[0] + q[0]), (p[0] - q[0]) * (p[1] + q[1]));
            }
            a
        };
        let nwl = nw.len();
        if !(n.len() > 0.5) {
            if nwl < 1e-12 {
                skipped += 1;
                continue;
            }
            n = nw * (1.0 / nwl);
        } else {
            n = n * (1.0 / n.len());
        }
        // orient: outer counter-clockwise about n, holes clockwise
        let mut fixed: Vec<Vec<u32>> = vec![];
        for (k, r) in rings.iter().enumerate() {
            let a = signed_area_about(&ring_pts(r), n);
            let mut r = r.clone();
            if (k == 0 && a < 0.0) || (k > 0 && a > 0.0) {
                r.reverse();
            }
            fixed.push(r);
        }
        let o = Vec3::new(outer_pts[0][0], outer_pts[0][1], outer_pts[0][2]);
        let fname = format!("F{}", face_ids.len());
        let face = body.add_face(shell, Surface::plane(o, n), true, FaceName::other("skp", &fname));
        for ring in &fixed {
            let lp = body.add_loop(face);
            for k in 0..ring.len() {
                let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                let (lo, hi, rev) = if a < b { (a, b, false) } else { (b, a, true) };
                *edge_uses.entry((lo, hi)).or_insert(0) += 1;
                let eid = match edges.get(&(lo, hi)) {
                    Some(&e) => e,
                    None => {
                        let mut vid = |i: u32, body: &mut Body| -> ck_topo::body::VertexId {
                            *verts.entry(i).or_insert_with(|| {
                                let p = weld.pts[i as usize];
                                body.add_vertex(Vec3::new(p[0], p[1], p[2]))
                            })
                        };
                        let va = vid(lo, &mut body);
                        let vb = vid(hi, &mut body);
                        let (pa, pb) = (weld.pts[lo as usize], weld.pts[hi as usize]);
                        let (pa, pb) = (Vec3::new(pa[0], pa[1], pa[2]), Vec3::new(pb[0], pb[1], pb[2]));
                        let len = (pb - pa).len();
                        let e = body.add_edge(Curve::line_through(pa, pb), 0.0, len, va, vb);
                        edges.insert((lo, hi), e);
                        e
                    }
                };
                body.add_coedge(lp, eid, rev, None);
            }
        }
        face_ids.push(format!("skp.{fname}"));
        face_materials.push(pf.material);
    }
    if face_ids.is_empty() {
        return Err(SkpError::new("body.empty", "no usable face (all were degenerate)"));
    }
    let open_edges = edge_uses.values().filter(|&&c| c == 1).count();
    let non_manifold_edges = edge_uses.values().filter(|&&c| c > 2).count();
    ck_topo::region::rebuild_lumps(&mut body);
    Ok(SkpBody { body, face_ids, face_materials, materials: scene.materials.clone(), skipped_faces: skipped, open_edges, non_manifold_edges })
}
