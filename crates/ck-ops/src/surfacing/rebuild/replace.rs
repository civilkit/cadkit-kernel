//! Swap a face's surface for the rebuilt one (edges/pcurves kept).

use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, FaceId};
use ck_geom::surface::Surface;

/// Replace `f`'s surface in place; the face keeps its name, sense, loops
/// and edge pcurves (the rebuild preserves the uv parameterisation).
pub fn replace_face(b: &mut Body, f: FaceId, surface: Surface) {
    b.face_mut(f).surface = surface;
    // the refit net has its own parameter range (0..spans): pcurves given in
    // the old surface's uv no longer apply; coedge_uv falls back to
    // projecting the edge onto the new surface
    for l in b.face(f).loops.clone() {
        for c in b.lp(l).coedges.clone() {
            b.coedge_mut(c).pcurve = None;
        }
    }
}

/// Lift the free boundary edges of `faces` onto their (edited) surface: each edge sample keeps its (u, v) on the surface `before` the edit. A control point edit of a face trimmed inside its
/// net (a circle on a square net) moves the surface under the boundary, and the boundary edge no longer lies on the face;
/// a thickened or knitted copy would then be an invalid solid. Each edge that deviates more than `tol` from the surface
/// (and is used by this one face only, so no neighbour is left behind) is refitted through the closest surface points of
/// its samples, its end vertices move with it. Edges shared with another face keep their curve.
pub fn lift_free_edges(b: &mut Body, faces: &[FaceId], before: &Body, tol: f64) {
    use crate::surfacing::knit::BodyExt;
    use ck_geom::curve::Curve;
    use ck_geom::fit::{interpolate, Param};
    use ck_geom::Vec3;
    const N: usize = 64;
    for &f in faces {
        let surface = b.face(f).surface.clone();
        let old = before.face(f).surface.clone();
        let mut edges = vec![];
        for l in b.face(f).loops.clone() {
            for &c in &b.lp(l).coedges {
                if !b.coedge(c).alive {
                    continue;
                }
                let e = b.coedge(c).edge;
                let live = b.edge_coedges(e).iter().filter(|&&k| b.coedge(k).alive).count();
                if live == 1 && !edges.contains(&e) {
                    edges.push(e);
                }
            }
        }
        for e in edges {
            let ed = b.edge(e).clone();
            let samples: Vec<Vec3> = (0..=N).map(|k| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / N as f64)).collect();
            let mut hint = None;
            let mut dev: f64 = 0.0;
            let lifted: Vec<Vec3> = samples
                .iter()
                .map(|&p| {
                    // the trim stays where it is in the surface's own (u, v): the point keeps its parameters
                    let (u, v, _) = match hint {
                        Some(h) => old.closest_point_near(p, h),
                        None => old.closest_point(p),
                    };
                    hint = Some((u, v));
                    let q = surface.eval(u, v);
                    dev = dev.max(q.dist(p));
                    q
                })
                .collect();
            if dev <= tol {
                continue;
            }
            let mut pts = lifted;
            if samples[0].dist(samples[N]) < 1e-9 {
                pts[N] = pts[0];
            }
            let Ok(c) = interpolate(&pts, 3, Param::ChordLength) else { continue };
            let (t0, t1) = c.domain();
            let (v0, v1) = (ed.v0, ed.v1);
            let em = b.edge_mut(e);
            em.curve = Curve::BSpline(c);
            em.t0 = t0;
            em.t1 = t1;
            b.vertex_mut(v0).p = pts[0];
            b.vertex_mut(v1).p = pts[N];
        }
    }
}
