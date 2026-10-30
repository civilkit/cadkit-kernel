//! Sanity checks on a shelled body: walls that meet inside the cavity.
//!
//! The cavity of a shell is the body offset inward. Where two of its offset surfaces touch (the
//! offset of a hole is tangent to the offset of a wall) or cross (it overshoots a wall by a hair),
//! the cut leaves a cavity pinched to zero width, or two overlapping offset patches. The result
//! then passes the topology checks although the wall is not `t` thick there. These checks look at
//! the loops of the result's faces: a planar face never has two loops (or two coplanar faces never
//! have boundaries) that meet without sharing an edge.

use crate::boolean::imprint::dist_in_range;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, LoopId};

const SAMPLES: usize = 2048;

struct Lp {
    face: FaceId,
    edges: Vec<EdgeId>,
    /// Sample points of every edge, and the box around all of them.
    pts: Vec<Vec<Vec3>>,
    lo: Vec3,
    hi: Vec3,
}

fn plane_key(b: &Body, f: FaceId) -> Option<(Vec3, f64)> {
    match &b.face(f).surface {
        Surface::Plane { frame } => {
            let n = if b.face(f).sense { frame.z } else { -frame.z };
            Some((n, frame.o.dot(n)))
        }
        _ => None,
    }
}

fn sample_loop(b: &Body, face: FaceId, l: LoopId) -> Lp {
    let (mut lo, mut hi) = (Vec3::new(f64::MAX, f64::MAX, f64::MAX), Vec3::new(f64::MIN, f64::MIN, f64::MIN));
    let (mut edges, mut pts) = (vec![], vec![]);
    for &c in &b.lp(l).coedges {
        let e = b.coedge(c).edge;
        let ed = b.edge(e);
        let n = if matches!(ed.curve, ck_geom::curve::Curve::Line { .. }) { 2 } else { SAMPLES };
        let p: Vec<Vec3> = (0..=n).map(|k| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64)).collect();
        for q in &p {
            lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
            hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
        }
        edges.push(e);
        pts.push(p);
    }
    Lp { face, edges, pts, lo, hi }
}

fn boxes_near(a: &Lp, c: &Lp, eps: f64) -> bool {
    a.lo.x <= c.hi.x + eps && c.lo.x <= a.hi.x + eps && a.lo.y <= c.hi.y + eps && c.lo.y <= a.hi.y + eps && a.lo.z <= c.hi.z + eps && c.lo.z <= a.hi.z + eps
}

/// The closest approach of two loops is below `eps` (exact distance from samples of one to the
/// curves of the other).
fn loops_meet(b: &Body, a: &Lp, c: &Lp, eps: f64) -> bool {
    for (i, &ea) in a.edges.iter().enumerate() {
        for (j, &ec) in c.edges.iter().enumerate() {
            if ea == ec {
                continue;
            }
            let ed = b.edge(ec);
            let (lo, hi) = c.pts[j].iter().fold((c.pts[j][0], c.pts[j][0]), |(lo, hi), q| (Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z)), Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z))));
            for p in &a.pts[i] {
                if p.x < lo.x - eps || p.x > hi.x + eps || p.y < lo.y - eps || p.y > hi.y + eps || p.z < lo.z - eps || p.z > hi.z + eps {
                    continue;
                }
                if dist_in_range(&ed.curve, (ed.t0, ed.t1), *p).1 < eps {
                    return true;
                }
            }
        }
    }
    false
}

/// A description of the first place where the boundaries of planar faces meet without sharing an
/// edge (two loops of one face, or two coplanar faces), `None` when there is none. `eps` is the
/// smallest cavity width that counts as open.
pub fn pinched_wall(b: &Body, eps: f64) -> Option<String> {
    let mut loops: Vec<(Lp, (Vec3, f64))> = vec![];
    for f in b.face_ids() {
        let Some(key) = plane_key(b, f) else { continue };
        for &l in &b.face(f).loops {
            if b.lp(l).alive && !b.lp(l).coedges.is_empty() {
                loops.push((sample_loop(b, f, l), key));
            }
        }
    }
    for i in 0..loops.len() {
        for j in i + 1..loops.len() {
            let ((a, ka), (c, kc)) = (&loops[i], &loops[j]);
            if ka.0.dot(kc.0) < 1.0 - 1e-9 || (ka.1 - kc.1).abs() > 1e-6 * (1.0 + ka.1.abs()) || !boxes_near(a, c, eps) {
                continue;
            }
            if a.face != c.face {
                // faces that share an edge are neighbours (split faces), not a pinch
                let fa = b.face_edges(a.face);
                if b.face_edges(c.face).iter().any(|e| fa.contains(e)) {
                    continue;
                }
            }
            if loops_meet(b, a, c, eps) || loops_meet(b, c, a, eps) {
                return Some(format!("{} and {}", b.face_id_str(a.face), b.face_id_str(c.face)));
            }
        }
    }
    None
}
