//! Mixed vertices: a 3-valent vertex of planar faces whose three edges are all selected for one
//! constant-radius fillet, two convex and one concave (the rim of a pocket where its inner corner
//! edge meets the top face).
//!
//! The rolling ball cannot touch all three faces at once there, so the blend is built in the order
//! in which the surfaces really depend on each other (and the answer no longer depends on the order
//! of the selection):
//!
//! 1. the concave edge is filleted first (a cylinder that fills the inner corner). Its blend face
//!    ends in the top face along a circular arc, tangent to the two rim edges;
//! 2. the two rim edges and that arc form one tangent chain of convex edges and are filleted
//!    together: the ball rolls along the rims, around the filled corner (a torus patch of major
//!    radius `2 r` about the axis of the corner cylinder) and along the other rim.
//!
//! This is the surface structure OCCT's `BRepFilletAPI_MakeFillet` produces (two cylinders, a torus,
//! the corner cylinder): volume agreement with it to the precision of the reference (corpus cases
//! `pocket_mixed_*` in tools/blend_corpus.mjs).
//!
//! Blend names keep the selection numbering: `B<k>` is the blend of pick `k`; the arcs, which no pick
//! asked for, are numbered after the picks.

use super::build::{blend_edges, find_edge, FilletProfile, Spec};
use super::BlendError;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, Role};

/// `Some(true)` for a concave straight edge between two planar faces, `Some(false)` for a convex one.
pub(super) fn line_edge_concave(b: &Body, e: EdgeId) -> Option<bool> {
    let ed = b.edge(e);
    if !matches!(ed.curve, Curve::Line { .. }) {
        return None;
    }
    let fs = b.edge_faces(e);
    if fs.len() != 2 || fs.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
        return None;
    }
    let d = (b.vertex(ed.v1).p - b.vertex(ed.v0).p).normalize();
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0])?;
    let tdir = if b.coedge(co).rev { -d } else { d };
    let s = b.planar_normal(fs[0]).cross(tdir).dot(b.planar_normal(fs[1]));
    if s.abs() < 1e-9 {
        return None;
    }
    Some(s > 0.0)
}

/// Fillet of `edges` (radius `r`) when the selection holds mixed vertices (see the module doc);
/// `None` when it does not (the caller uses the ordinary path).
pub fn fillet_mixed(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    let n = edges.len();
    let eids: Vec<EdgeId> = edges.iter().map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect::<Option<_>>()?;
    let conc: Vec<bool> = eids.iter().map(|&e| line_edge_concave(b, e)).collect::<Option<_>>()?;
    if !conc.iter().any(|&c| c) || conc.iter().all(|&c| c) {
        return None;
    }
    // mixed vertices: all three edges selected, two convex and one concave
    let mut wall = vec![false; n];
    let mut tops: Vec<(usize, FaceId)> = vec![]; // (concave pick, the face the two convex edges share)
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 3 {
            continue;
        }
        let picks: Vec<usize> = (0..n).filter(|&i| inc.contains(&eids[i])).collect();
        if picks.len() != 3 || picks.iter().filter(|&&i| conc[i]).count() != 1 {
            continue;
        }
        let cv: Vec<usize> = picks.iter().copied().filter(|&i| !conc[i]).collect();
        let w = picks.iter().copied().find(|&i| conc[i])?;
        let (f0, f1) = (b.edge_faces(eids[cv[0]]), b.edge_faces(eids[cv[1]]));
        let common: Vec<FaceId> = f0.iter().copied().filter(|f| f1.contains(f)).collect();
        if common.len() != 1 {
            continue;
        }
        wall[w] = true;
        tops.push((w, common[0]));
    }
    if tops.is_empty() {
        // concave-concave-convex: the foot of a tower on a wall. The convex edge is filleted first (with every other
        // convex edge); its band ends on the wall W, and the wall edges of the tower plus that arc form a tangent chain
        // of concave edges that is filleted next (torus of major radius r_convex + r_concave round the band's end)
        for v in b.vertex_ids() {
            let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
            if inc.len() != 3 {
                continue;
            }
            let picks: Vec<usize> = (0..n).filter(|&i| inc.contains(&eids[i])).collect();
            if picks.len() != 3 || picks.iter().filter(|&&i| conc[i]).count() != 2 {
                continue;
            }
            let cx = picks.iter().copied().find(|&i| !conc[i])?;
            let cc: Vec<usize> = picks.iter().copied().filter(|&i| conc[i]).collect();
            let (f0, f1) = (b.edge_faces(eids[cc[0]]), b.edge_faces(eids[cc[1]]));
            let common: Vec<FaceId> = f0.iter().copied().filter(|f| f1.contains(f)).collect();
            if common.len() != 1 || b.edge_faces(eids[cx]).contains(&common[0]) {
                continue;
            }
            tops.push((cx, common[0]));
        }
        if tops.is_empty() {
            return None;
        }
        for i in 0..n {
            wall[i] = !conc[i];
        }
    }
    // a failure of the two-stage build is not final: the ordinary builder may still manage the selection
    // one radius for the concave edges of mixed vertices, one for everything else (the rim ball rolls round the
    // corner cylinder: torus of major radius r_wall + r_rim)
    let same = |v: Vec<f64>| v.first().copied().filter(|x| v.iter().all(|y| (y - x).abs() <= 1e-12 * (1.0 + x.abs())));
    let rw = same((0..n).filter(|&i| wall[i]).map(|i| radii[i]).collect())?;
    let rr = same((0..n).filter(|&i| !wall[i]).map(|i| radii[i]).collect())?;
    run(b, edges, &wall, &tops, [rw, rr], feature, tol).ok().map(Ok)
}

fn run(b: &Body, edges: &[(String, String, Vec3)], wall: &[bool], tops: &[(usize, FaceId)], r: [f64; 2], feature: &str, tol: f64) -> Result<Body, BlendError> {
    let n = edges.len();
    let spec_a = Spec::Fillet(r[0], FilletProfile::Circular);
    let spec_b = Spec::Fillet(r[1], FilletProfile::Circular);
    // stage A: the concave edges of mixed vertices
    let a_idx: Vec<usize> = (0..n).filter(|&i| wall[i]).collect();
    let pre = format!("{feature}_a");
    let sel_a: Vec<(String, String, Vec3)> = a_idx.iter().map(|&i| edges[i].clone()).collect();
    let body1 = blend_edges(b, &sel_a, spec_a, &pre, tol)?;
    // stage B: everything else, plus the arc where the corner cylinder meets the top face
    let mut b_idx: Vec<usize> = (0..n).filter(|&i| !wall[i]).collect();
    let mut sel_b: Vec<(String, String, Vec3)> = b_idx.iter().map(|&i| edges[i].clone()).collect();
    let mut next = n;
    for &(w, top) in tops {
        let top_name = b.face_id_str(top);
        let pos = a_idx.iter().position(|&i| i == w).unwrap();
        let blend_name = ck_topo::body::FaceName::blend(&pre, pos as u32).id();
        let (Some(ft), Some(fbl)) = (body1.find_face(&top_name), body1.find_face(&blend_name)) else {
            return Err(BlendError::code("blendFailed"));
        };
        let arc = body1.edge_ids().find(|&e| {
            let fs = body1.edge_faces(e);
            fs.len() == 2 && fs.contains(&ft) && fs.contains(&fbl)
        });
        let Some(arc) = arc else { return Err(BlendError::code("blendFailed")) };
        let ed = body1.edge(arc);
        let at = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        sel_b.push((top_name, blend_name, at));
        b_idx.push(next);
        next += 1;
    }
    let mut out = super::build::blend_edges_tangent_chain(&body1, &sel_b, spec_b, feature, tol)?;
    // blend names follow the selection: stage A's `B<j>` is pick `a_idx[j]`, stage B's `B<j>` is `b_idx[j]`
    let renames: Vec<(usize, u32)> = out
        .face_ids()
        .filter_map(|f| {
            let nm = &out.face(f).name;
            match nm.role {
                Role::Blend(k) if nm.feature == pre => Some((f.idx(), a_idx[k as usize] as u32)),
                Role::Blend(k) if nm.feature == feature => Some((f.idx(), b_idx[k as usize] as u32)),
                _ => None,
            }
        })
        .collect();
    for (fi, k) in renames {
        out.faces[fi].name.feature = feature.to_string();
        out.faces[fi].name.role = Role::Blend(k);
    }
    Ok(out)
}
