//! Rim corners on curved outlines: a concave wall edge between a plane and a cylinder (the inner corner where a
//! radial flank meets a hub) whose top and / or bottom rim edges are filleted in the same feature.
//!
//! The rolling ball of the rim cannot touch the flank, the hub and the top at once. Built in the order in which the
//! surfaces depend on each other, as [`super::mixed`] does for planar pockets:
//!
//! 1. the concave wall edges first: an analytic cylinder (radius `r`, axis parallel to the edge at distance `r` from
//!    the plane and `R + r` from the hub's axis), ending flush on the top and bottom planes in a circular arc of
//!    radius `r` that is tangent to both rim edges at that vertex;
//! 2. the rim edges plus those arcs as one tangent chain of convex edges: along the arc the ball runs on a torus of
//!    major radius `2 r` and minor radius `r` about the wall fillet's axis, which ends the wall fillet at `H - r`
//!    with four tangent circular edges (wall fillet, the two rim fillets, the top plane).
//!
//! Where only one rim edge at a wall end is selected (the bottom of a petal whose hub edge is left sharp) the arc's
//! torus runs on past the unselected side and is trimmed by that wall.
//!
//! This is the corner a third-party CAD kernel and the reference engine both build for the petal ring
//! (`docs/blend-reference-petal.md`).

use super::build::find_edge;
use super::BlendError;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId};

thread_local! {
    /// While set, a concave straight edge between a plane and a cylinder parallel to it is built by the analytic
    /// line builder (an exact cylinder) instead of the free-form path.
    static WALL: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Sets [`WALL`] for its lifetime.
struct WallGuard(bool);

impl WallGuard {
    fn new() -> WallGuard {
        WallGuard(WALL.with(|c| c.replace(true)))
    }
}

impl Drop for WallGuard {
    fn drop(&mut self) {
        WALL.with(|c| c.set(self.0));
    }
}

/// Lets the analytic line builder take the wall lines (see [`WALL`]) until the guard is dropped.
pub(super) fn wall_guard() -> impl Drop {
    WallGuard::new()
}

pub(super) fn wall_enabled() -> bool {
    WALL.with(|c| c.get())
}

/// Outward unit normal of face `f` at (its point nearest) `p`.
fn normal_at(b: &Body, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = b.face(f).surface.closest_param(p);
    b.outward_normal(f, u, v).normalize()
}

/// `Some(true)` for a concave edge, `Some(false)` for a convex one (sign of the dihedral at the edge's midpoint);
/// `None` for a smooth or non-manifold edge.
pub(super) fn edge_concave(b: &Body, e: EdgeId) -> Option<bool> {
    let ed = b.edge(e);
    let fs = b.edge_faces(e);
    if fs.len() != 2 || ed.v0 == ed.v1 {
        return None;
    }
    let tm = 0.5 * (ed.t0 + ed.t1);
    let pm = ed.curve.eval(tm);
    let d = ed.curve.tangent(tm).try_normalize()?;
    let d = if ed.t1 >= ed.t0 { d } else { -d };
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0])?;
    let tdir = if b.coedge(co).rev { -d } else { d };
    let s = normal_at(b, fs[0], pm).cross(tdir).dot(normal_at(b, fs[1], pm));
    if s.abs() < 1e-9 {
        return None;
    }
    Some(s > 0.0)
}

/// A concave straight edge between a plane and a convex cylinder (material inside it) whose axis is parallel to the
/// edge: the wall edge of a radial flank meeting a hub.
pub(super) fn is_wall_line(b: &Body, e: EdgeId) -> bool {
    let ed = b.edge(e);
    let Curve::Line { d, .. } = ed.curve else { return false };
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return false;
    }
    let dir = d.normalize();
    let pm = (b.vertex(ed.v0).p + b.vertex(ed.v1).p) * 0.5;
    let kinds: Vec<u8> = fs
        .iter()
        .map(|&f| match &b.face(f).surface {
            Surface::Plane { .. } => 0,
            Surface::Cylinder { frame, .. } if frame.z.normalize().cross(dir).len() < 1e-9 => {
                let rel = (pm - frame.o).reject(frame.z.normalize());
                if normal_at(b, f, pm).dot(rel) > 0.0 { 1 } else { 2 }
            }
            _ => 2,
        })
        .collect();
    kinds.contains(&0) && kinds.contains(&1) && edge_concave(b, e) == Some(true)
}

/// While [`WALL`] is set: the edge is a [`is_wall_line`] the analytic line builder takes.
pub(super) fn analytic_wall_edge(b: &Body, e: EdgeId) -> bool {
    wall_enabled() && is_wall_line(b, e)
}

/// The rolling ball of a concave fillet of radius `r` at the point `pm` of a wall line between `faces` (one plane,
/// one cylinder parallel to the line): `(centre, contact on faces[0], contact on faces[1])`, all in the plane through
/// `pm` normal to the line.
pub(super) fn wall_section(b: &Body, faces: [FaceId; 2], pm: Vec3, dir: Vec3, r: f64) -> Option<(Vec3, Vec3, Vec3)> {
    let (ip, ic) = match (&b.face(faces[0]).surface, &b.face(faces[1]).surface) {
        (Surface::Plane { .. }, Surface::Cylinder { .. }) => (0, 1),
        (Surface::Cylinder { .. }, Surface::Plane { .. }) => (1, 0),
        _ => return None,
    };
    let Surface::Cylinder { frame, r: rc } = &b.face(faces[ic]).surface else { return None };
    let ax = frame.z.normalize();
    if ax.cross(dir).len() > 1e-9 {
        return None;
    }
    let np = b.planar_normal(faces[ip]).normalize();
    // centre: on the plane offset by r (outward: the ball is outside the material), at R + r from the axis
    let w = np.cross(dir).normalize();
    let q0 = (pm + np * r - frame.o).reject(ax);
    let rr = rc + r;
    let bq = q0.dot(w);
    let disc = bq * bq - (q0.dot(q0) - rr * rr);
    if !(disc >= 0.0) {
        return None;
    }
    let s = disc.sqrt();
    // the root nearer the edge
    let u = if (-bq + s).abs() <= (-bq - s).abs() { -bq + s } else { -bq - s };
    let c = pm + np * r + w * u;
    let along = (c - frame.o).dot(ax);
    let radial = (c - frame.o).reject(ax);
    if radial.len() < 1e-12 {
        return None;
    }
    let tc = frame.o + ax * along + radial.normalize() * *rc;
    let tp = c - np * r;
    // the ball must sit next to the edge, not on the far side of the hub
    if tc.dist(pm) > 4.0 * r || tp.dist(pm) > 4.0 * r {
        return None;
    }
    Some(if ip == 0 { (c, tp, tc) } else { (c, tc, tp) })
}

/// The face at vertex `v` that is not one of `e`'s own faces (the face an edge ends on), when there is exactly one.
fn end_face(b: &Body, e: EdgeId, v: ck_topo::body::VertexId) -> Option<FaceId> {
    let own = b.edge_faces(e);
    let mut ends: Vec<FaceId> = vec![];
    for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
        for f in b.edge_faces(x) {
            if !own.contains(&f) && !ends.contains(&f) {
                ends.push(f);
            }
        }
    }
    (ends.len() == 1).then(|| ends[0])
}

/// [`super::mixed::fillet_mixed`] for selections on curved outlines: the wall edges are [`is_wall_line`]s, the rim
/// edges any convex lines or circles. `None` when the selection has no such corner (the caller goes on with the
/// ordinary path); a failed build is also `None`, so the ordinary builder still gets its try.
pub fn fillet_mixed_curved(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    let n = edges.len();
    let eids: Vec<EdgeId> = edges.iter().map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect::<Option<_>>()?;
    let conc: Vec<Option<bool>> = eids.iter().map(|&e| edge_concave(b, e)).collect();
    let mut wall = vec![false; n];
    let mut tops: Vec<(usize, FaceId)> = vec![];
    for (w, &e) in eids.iter().enumerate() {
        if conc[w] != Some(true) || !(is_wall_line(b, e) || super::mixed::line_edge_concave(b, e) == Some(true)) {
            continue;
        }
        let ed = b.edge(e);
        for v in [ed.v0, ed.v1] {
            let inc: Vec<EdgeId> = b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).collect();
            if inc.len() != 3 {
                continue;
            }
            let Some(ef) = end_face(b, e, v) else { continue };
            let rims: Vec<usize> = (0..n).filter(|&i| i != w && inc.contains(&eids[i])).collect();
            // one or both rim edges at this end selected, convex, on the end face
            if rims.is_empty() || !rims.iter().all(|&i| conc[i] == Some(false) && b.edge_faces(eids[i]).contains(&ef)) {
                continue;
            }
            wall[w] = true;
            tops.push((w, ef));
        }
    }
    if tops.is_empty() {
        return None;
    }
    // every straight concave edge between planes is built with the walls (a vertex of three concave edges is a
    // sphere of stage A; in the rim chain it would be missed)
    for i in 0..n {
        if conc[i] == Some(true) && super::mixed::line_edge_concave(b, eids[i]) == Some(true) && radii[i] == radii[tops[0].0] {
            wall[i] = true;
        }
    }
    let same = |v: Vec<f64>| v.first().copied().filter(|x| v.iter().all(|y| (y - x).abs() <= 1e-12 * (1.0 + x.abs())));
    let rw = same((0..n).filter(|&i| wall[i]).map(|i| radii[i]).collect())?;
    let rr = same((0..n).filter(|&i| !wall[i]).map(|i| radii[i]).collect()).unwrap_or(rw);
    let _g = WallGuard::new();
    match super::mixed::run(b, edges, &wall, &tops, [rw, rr], feature, tol) {
        Ok(out) => Some(Ok(out)),
        Err(e) => {
            super::diag::dbg_fail(&format!("rim corner build: {} {}", e.code, e.detail.unwrap_or_default()));
            None
        }
    }
}
