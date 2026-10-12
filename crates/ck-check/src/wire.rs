// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/wire.rs
//! Loop closure / chaining and uv self-intersection of planar faces.

use crate::{face_ent, Issue};
use ck_geom::surface::Surface;
use ck_geom::Vec2;
use ck_topo::body::Body;

pub fn check(b: &Body, tol: f64, out: &mut Vec<Issue>) {
    for f in b.face_ids() {
        let face = b.face(f);
        let planar = matches!(face.surface, Surface::Plane { .. });
        if face.loops.is_empty() {
            // A closed analytic face (full sphere/torus) may legitimately have none.
            if planar {
                out.push(Issue { code: "FACE_NO_LOOP", entity: face_ent(b, f), detail: "planar face without a loop".into() });
            }
            continue;
        }
        for &l in &face.loops {
            if !b.lp(l).alive {
                continue;
            }
            let cs: &Vec<_> = &b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
            if cs.is_empty() {
                out.push(Issue { code: "LOOP_EMPTY", entity: face_ent(b, f), detail: "loop has no edges".into() });
                continue;
            }
            for i in 0..cs.len() {
                let (c, n) = (cs[i], cs[(i + 1) % cs.len()]);
                let (ve, vs) = (b.coedge_end(c), b.coedge_start(n));
                if ve != vs && b.vertex(ve).p.dist(b.vertex(vs).p) > tol {
                    out.push(Issue {
                        code: "LOOP_OPEN",
                        entity: face_ent(b, f),
                        detail: format!("coedge {i} end != coedge {} start", (i + 1) % cs.len()),
                    });
                }
            }
        }
        if planar {
            self_intersection(b, f, out);
        }
    }
}

/// Adaptive uv polyline of one coedge: subdivide (min 2 levels, max 12) until
/// the parametric midpoint lies within `sag` of the chord. Pushes the start
/// point and interior points, not the end point (the next coedge's start).
#[allow(clippy::too_many_arguments)]
fn sample(b: &Body, c: ck_topo::body::CoedgeId, t0: f64, p0: Vec2, t1: f64, p1: Vec2, depth: u32, sag: f64, out: &mut Vec<Vec2>) {
    let tm = 0.5 * (t0 + t1);
    let pm = b.coedge_uv(c, tm, Some(p0));
    let d = p1 - p0;
    let l = d.len();
    let dev = if l < 1e-14 { (pm - p0).len() } else { d.cross(pm - p0).abs() / l };
    // a midpoint beyond the chord ends (cusp / overshoot) also needs refining
    let along = if l < 1e-14 { 0.0 } else { (pm - p0).dot(d) / (l * l) };
    let flat = dev <= sag && (0.0..=1.0).contains(&along);
    if depth >= 12 || (depth >= 2 && flat) {
        out.push(p0);
        out.push(pm);
        return;
    }
    sample(b, c, t0, p0, tm, pm, depth + 1, sag, out);
    sample(b, c, tm, pm, t1, p1, depth + 1, sag, out);
}

/// Polylines (uv) of each loop of a planar face. Within one loop, report
/// crossings between non-adjacent segments (`LOOP_SELF_INTERSECT`); between
/// different loops of the face (outer vs hole) report `LOOP_CROSS` only for a
/// true transversal crossing. Sampling is adaptive (chord sag below a tiny
/// fraction of the face size) and the crossing tolerance is scaled to that
/// sag, so thin annuli / near-concentric loops are not flagged.
fn self_intersection(b: &Body, f: ck_topo::body::FaceId, out: &mut Vec<Issue>) {
    // size of the face in uv, from the vertices (sets the sag tolerance)
    let (mut lo, mut hi) = (Vec2::new(f64::MAX, f64::MAX), Vec2::new(f64::MIN, f64::MIN));
    let mut curved = false;
    for &l in &b.face(f).loops {
        if !b.lp(l).alive {
            continue;
        }
        for &c in b.lp(l).coedges.iter().filter(|&&c| b.coedge(c).alive) {
            let (t0, t1) = b.coedge_params(c);
            for k in 0..=4 {
                let p = b.coedge_uv(c, t0 + (t1 - t0) * k as f64 / 4.0, None);
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
            curved |= !matches!(b.edge(b.coedge(c).edge).curve, ck_geom::curve::Curve::Line { .. });
        }
    }
    let size = (hi - lo).len().max(1e-9);
    let sag = if curved { (size * 1e-5).max(1e-9) } else { f64::INFINITY };
    let eps = if curved { sag.max(1e-6) } else { 1e-6 };
    let mut polys: Vec<Vec<Vec2>> = vec![];
    for &l in &b.face(f).loops {
        if !b.lp(l).alive {
            continue;
        }
        let mut pts: Vec<Vec2> = vec![];
        let cs: Vec<_> = b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
        for &c in &cs {
            let (t0, t1) = b.coedge_params(c);
            let p0 = b.coedge_uv(c, t0, pts.last().copied());
            let p1 = b.coedge_uv(c, t1, Some(p0));
            if sag.is_infinite() {
                // straight edges: the end points are the whole polyline
                pts.push(p0);
            } else {
                sample(b, c, t0, p0, t1, p1, 0, sag, &mut pts);
            }
        }
        polys.push(pts);
    }
    let seg = |p: &Vec<Vec2>, i: usize| (p[i], p[(i + 1) % p.len()]);
    for (a, pa) in polys.iter().enumerate() {
        for (c, pc) in polys.iter().enumerate().skip(a) {
            for i in 0..pa.len() {
                let s = seg(pa, i);
                let (sl, sh) = (Vec2::new(s.0.x.min(s.1.x) - eps, s.0.y.min(s.1.y) - eps), Vec2::new(s.0.x.max(s.1.x) + eps, s.0.y.max(s.1.y) + eps));
                for j in 0..pc.len() {
                    if a == c && (j <= i || j == i + 1 || (i == 0 && j == pa.len() - 1)) {
                        continue;
                    }
                    let t = seg(pc, j);
                    if t.0.x.min(t.1.x) > sh.x || t.0.x.max(t.1.x) < sl.x || t.0.y.min(t.1.y) > sh.y || t.0.y.max(t.1.y) < sl.y {
                        continue;
                    }
                    if proper_cross(s.0, s.1, t.0, t.1, eps) {
                        out.push(Issue {
                            code: if a == c { "LOOP_SELF_INTERSECT" } else { "LOOP_CROSS" },
                            entity: face_ent(b, f),
                            detail: format!("uv loops {a}/{c} cross at segments {i}/{j}"),
                        });
                        return;
                    }
                }
            }
        }
    }
}

/// True when the segments cross strictly inside both. Endpoints that touch
/// the other segment (shared vertices, keyhole bridges that run back over each
/// other, pinch points where a loop revisits a vertex) are not crossings: the
/// test is on signed *distances* so float noise at a shared vertex cannot
/// flip a sign.
fn proper_cross(p: Vec2, q: Vec2, r: Vec2, s: Vec2, eps: f64) -> bool {
    let dist = |a: Vec2, b: Vec2, c: Vec2| {
        let l = (b - a).len();
        if l < 1e-12 { 0.0 } else { (b - a).cross(c - a) / l }
    };
    let (d1, d2, d3, d4) = (dist(r, s, p), dist(r, s, q), dist(p, q, r), dist(p, q, s));
    (d1 < -eps && d2 > eps || d1 > eps && d2 < -eps) && (d3 < -eps && d4 > eps || d3 > eps && d4 < -eps)
}

#[cfg(test)]
mod tests {
    use super::proper_cross;
    use ck_geom::Vec2;
    fn v(x: f64, y: f64) -> Vec2 {
        Vec2::new(x, y)
    }
    #[test]
    fn real_crossing() {
        assert!(proper_cross(v(0., 0.), v(2., 2.), v(0., 2.), v(2., 0.), 1e-6));
    }
    #[test]
    fn shared_vertex_with_noise_is_not_a_crossing() {
        // two segments meeting at (42, ~1e-15) from opposite sides of the x axis
        assert!(!proper_cross(v(42.0, 1e-15), v(41.657, 1.722), v(41.657, -1.722), v(42.0, 0.0), 1e-6));
    }
    #[test]
    fn overlapping_bridge_is_not_a_crossing() {
        assert!(!proper_cross(v(0., 0.), v(5., 0.), v(5., 0.), v(0., 0.), 1e-6));
    }
}
