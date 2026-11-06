//! `curvatureMap` mesh: a triangle soup with per-vertex principal curvatures
//! (k1, k2), for the false-colour overlay.
//!
//! Wire shape (oracle): one body (`id: null`) `{faces:[ids], mesh:{positions,
//! normals (f32, 9/tri), k (f32, 6/tri = k1,k2 per vertex), triFace (i32, face
//! index into `faces`)}}` + `failed:[]`; buffers 0..3 in that order.
//!
//! RECONSTRUCTED, not bit-exact: the original meshes every face on a dense
//! adaptive grid (64x64 cells on a planar rectangle, 50-250k triangles per
//! curved face); here the display tessellation is reused, and non-planar
//! faces are subdivided 4x4 in uv (exact surface points / normals / curvature
//! at every new vertex). Curvature values per vertex are exact
//! ([`ck_geom::analysis::surface_curvature`], sign convention of
//! `curvatureAt`). Face order = wire order (the oracle's order is not
//! understood).

use crate::face::FaceMesh;
use crate::mesh::Tess;
use ck_geom::analysis::surface_curvature;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};

#[derive(Debug, Clone, Default)]
pub struct CurvMap {
    pub faces: Vec<FaceId>,
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub k: Vec<f32>,
    pub tri_face: Vec<i32>,
    /// Wire face indices that failed to triangulate.
    pub failed: Vec<usize>,
}

struct V {
    p: Vec3,
    n: Vec3,
    k: [f64; 2],
}

fn vertex(s: &Surface, sense: bool, uv: Vec2) -> V {
    let d = s.derivs2(uv.x, uv.y);
    let c = surface_curvature(&d, !sense);
    let n = if sense { d.normal() } else { -d.normal() };
    V { p: d.p, n, k: if c.singular { [0.0, 0.0] } else { [c.k1, c.k2] } }
}

/// The curvature map of a tessellated body.
pub fn curvature_map(body: &Body, tess: &Tess) -> CurvMap {
    let mut out = CurvMap::default();
    let fids: Vec<FaceId> = body.face_ids().collect();
    for (wi, fm) in tess.faces.iter().enumerate() {
        let Some(fm) = fm else {
            out.failed.push(wi);
            continue;
        };
        let f = fids[wi];
        let face = body.face(f);
        let planar = matches!(face.surface, Surface::Plane { .. });
        let fi = out.faces.len() as i32;
        out.faces.push(f);
        let (pu, pv) = face.surface.periods();
        let wrap = |uv: Vec2| -> Vec2 {
            let mut e = uv;
            if let Some(p) = pu {
                if e.x.is_finite() {
                    e.x = e.x.rem_euclid(p);
                }
            }
            if let Some(p) = pv {
                if e.y.is_finite() {
                    e.y = e.y.rem_euclid(p);
                }
            }
            e
        };
        let eval = |uv: Vec2| vertex(&face.surface, face.sense, wrap(uv));
        for t in &fm.tris {
            let uvs = [fm.uv[t[0] as usize], fm.uv[t[1] as usize], fm.uv[t[2] as usize]];
            emit(&mut out, fi, &uvs, if planar { 0 } else { 2 }, &eval, fm);
        }
    }
    out
}

/// Emit triangle `uvs` subdivided `levels` times (4^levels triangles).
fn emit(out: &mut CurvMap, fi: i32, uvs: &[Vec2; 3], levels: u32, eval: &dyn Fn(Vec2) -> V, _fm: &FaceMesh) {
    if levels == 0 {
        for uv in uvs {
            let v = eval(*uv);
            out.positions.extend([v.p.x as f32, v.p.y as f32, v.p.z as f32]);
            out.normals.extend([v.n.x as f32, v.n.y as f32, v.n.z as f32]);
            out.k.extend([v.k[0] as f32, v.k[1] as f32]);
        }
        out.tri_face.push(fi);
        return;
    }
    let m = |a: Vec2, b: Vec2| (a + b) * 0.5;
    let (ab, bc, ca) = (m(uvs[0], uvs[1]), m(uvs[1], uvs[2]), m(uvs[2], uvs[0]));
    for t in [[uvs[0], ab, ca], [ab, uvs[1], bc], [ca, bc, uvs[2]], [ab, bc, ca]] {
        emit(out, fi, &t, levels - 1, eval, _fm);
    }
}
