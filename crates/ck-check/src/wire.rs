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

/// Polylines (uv) of each loop of a planar face; report crossings between
/// non-adjacent segments of the same loop and between different loops.
fn self_intersection(b: &Body, f: ck_topo::body::FaceId, out: &mut Vec<Issue>) {
    let mut polys: Vec<Vec<Vec2>> = vec![];
    for &l in &b.face(f).loops {
        if !b.lp(l).alive {
            continue;
        }
        let mut pts: Vec<Vec2> = vec![];
        for &c in b.lp(l).coedges.iter().filter(|&&c| b.coedge(c).alive) {
            let (t0, t1) = b.coedge_params(c);
            let n = 8;
            for k in 0..n {
                let t = t0 + (t1 - t0) * k as f64 / n as f64;
                let near = pts.last().copied();
                pts.push(b.coedge_uv(c, t, near));
            }
        }
        polys.push(pts);
    }
    let seg = |p: &Vec<Vec2>, i: usize| (p[i], p[(i + 1) % p.len()]);
    for (a, pa) in polys.iter().enumerate() {
        for (c, pc) in polys.iter().enumerate().skip(a) {
            for i in 0..pa.len() {
                for j in 0..pc.len() {
                    if a == c && (j <= i || j == i + 1 || (i == 0 && j == pa.len() - 1)) {
                        continue;
                    }
                    let (s, t) = (seg(pa, i), seg(pc, j));
                    if proper_cross(s.0, s.1, t.0, t.1) {
                        out.push(Issue {
                            code: "LOOP_SELF_INTERSECT",
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
fn proper_cross(p: Vec2, q: Vec2, r: Vec2, s: Vec2) -> bool {
    let dist = |a: Vec2, b: Vec2, c: Vec2| {
        let l = (b - a).len();
        if l < 1e-12 { 0.0 } else { (b - a).cross(c - a) / l }
    };
    let (d1, d2, d3, d4) = (dist(r, s, p), dist(r, s, q), dist(p, q, r), dist(p, q, s));
    let eps = 1e-6;
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
        assert!(proper_cross(v(0., 0.), v(2., 2.), v(0., 2.), v(2., 0.)));
    }
    #[test]
    fn shared_vertex_with_noise_is_not_a_crossing() {
        // two segments meeting at (42, ~1e-15) from opposite sides of the x axis
        assert!(!proper_cross(v(42.0, 1e-15), v(41.657, 1.722), v(41.657, -1.722), v(42.0, 0.0)));
    }
    #[test]
    fn overlapping_bridge_is_not_a_crossing() {
        assert!(!proper_cross(v(0., 0.), v(5., 0.), v(5., 0.), v(0., 0.)));
    }
}
