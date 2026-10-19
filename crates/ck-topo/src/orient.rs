//! Orientation: outward normals, loop orientation (outer CCW / inner CW
//! about the outward normal), shell orientation fixing.

use crate::body::{Body, FaceId, LoopId, ShellId};
use crate::mass::{signed_volume, GL};
use ck_geom::Vec2;

/// Reverse a face: loops reversed (order + coedge directions), sense
/// toggled. Keeps the "CCW about outward normal" invariant.
pub fn flip_face(b: &mut Body, f: FaceId) {
    let loops = b.faces[f.idx()].loops.clone();
    for l in loops {
        b.loops[l.idx()].coedges.reverse();
        for c in b.loops[l.idx()].coedges.clone() {
            b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
        }
    }
    b.faces[f.idx()].sense = !b.faces[f.idx()].sense;
}

/// Signed uv area of a loop with respect to the surface's natural
/// orientation (positive = CCW in uv).
pub fn loop_uv_area(b: &Body, l: LoopId) -> f64 {
    let mut a = 0.0;
    let mut prev: Option<Vec2> = None;
    for &c in &b.lp(l).coedges {
        let (ta, tb) = b.coedge_params(c);
        let n = 8;
        for k in 0..n {
            let (p, q) = (ta + (tb - ta) * k as f64 / n as f64, ta + (tb - ta) * (k + 1) as f64 / n as f64);
            let (m, h) = ((p + q) * 0.5, (q - p) * 0.5);
            for &(x, w) in GL.iter() {
                let t = m + h * x;
                let (uv, d) = match &b.coedge(c).pcurve {
                    Some(pc) => {
                        let d = pc.derivs(t, 1);
                        (d[0], d[1])
                    }
                    None => {
                        let e = 1e-6 * (tb - ta).abs().max(1e-9);
                        let u0 = b.coedge_uv(c, t - e, prev);
                        let u1 = b.coedge_uv(c, t + e, Some(u0));
                        prev = Some(u1);
                        ((u0 + u1) * 0.5, (u1 - u0) / (2.0 * e))
                    }
                };
                a += 0.5 * w * h * uv.cross(d);
            }
        }
    }
    a
}

/// Is the face's loop set consistent with its sense (outer loop CCW about
/// the outward normal, holes CW)? Only meaningful for faces whose uv
/// loops are closed (planes, or periodic faces with seams).
pub fn loops_consistent(b: &Body, f: FaceId) -> bool {
    let sg = if b.face(f).sense { 1.0 } else { -1.0 };
    let total: f64 = b.face(f).loops.iter().map(|&l| loop_uv_area(b, l)).sum();
    total * sg > 0.0
}

/// Planar faces: put the largest |area| loop first, make it CCW about the
/// outward normal and every other loop CW. Returns true if anything changed.
pub fn fix_planar_loops(b: &mut Body, f: FaceId) -> bool {
    let loops = b.faces[f.idx()].loops.clone();
    if loops.is_empty() {
        return false;
    }
    let sg = if b.faces[f.idx()].sense { 1.0 } else { -1.0 };
    let areas: Vec<f64> = loops.iter().map(|&l| loop_uv_area(b, l) * sg).collect();
    let outer = (0..loops.len()).max_by(|&i, &j| areas[i].abs().partial_cmp(&areas[j].abs()).unwrap()).unwrap();
    let mut changed = false;
    if outer != 0 {
        b.faces[f.idx()].loops.swap(0, outer);
        changed = true;
    }
    for (i, &l) in loops.iter().enumerate() {
        let want_pos = i == outer;
        if (areas[i] > 0.0) != want_pos {
            reverse_loop(b, l);
            changed = true;
        }
    }
    changed
}

pub fn reverse_loop(b: &mut Body, l: LoopId) {
    b.loops[l.idx()].coedges.reverse();
    for c in b.loops[l.idx()].coedges.clone() {
        b.coedges[c.idx()].rev = !b.coedges[c.idx()].rev;
    }
}

/// Make all faces of a shell consistently oriented (each manifold edge
/// traversed oppositely by its two coedges) and outward (positive signed
/// volume). Returns the number of faces flipped.
/// `signed_volume(b, faces) < 0.0`, decided by the cheap quick volume when the faces close up (every edge of
/// them has its two coedges among them, so the volume does not depend on the reference point) and that volume
/// is clearly away from zero; else by the full integral.
fn volume_is_negative(b: &Body, faces: &[FaceId]) -> bool {
    let mut inset = vec![false; b.faces.len()];
    for &f in faces {
        inset[f.idx()] = true;
    }
    let closed = faces.iter().all(|&f| {
        b.face_edges(f).into_iter().all(|e| {
            let cs = b.edge_coedges(e);
            cs.len() == 2 && cs.iter().all(|&c| inset[b.coedge_face(c).idx()])
        })
    });
    if closed {
        if let Some(v) = ck_topo_mass_quick(b, faces) {
            return v < 0.0;
        }
    }
    signed_volume(b, faces) < 0.0
}

fn ck_topo_mass_quick(b: &Body, faces: &[FaceId]) -> Option<f64> {
    crate::mass::signed_volume_quick_margin(b, faces)
}

pub fn orient_shell(b: &mut Body, s: ShellId) -> usize {
    let faces: Vec<FaceId> = b.shell(s).faces.iter().copied().filter(|&f| b.face(f).alive).collect();
    let mut flips = 0;
    let mut seen = vec![false; b.faces.len()];
    for &seed in &faces {
        if seen[seed.idx()] {
            continue;
        }
        seen[seed.idx()] = true;
        let mut stack = vec![seed];
        while let Some(f) = stack.pop() {
            for e in b.face_edges(f) {
                let cs = b.edge_coedges(e);
                if cs.len() != 2 {
                    continue;
                }
                let (c0, c1) = (cs[0], cs[1]);
                let (f0, f1) = (b.coedge_face(c0), b.coedge_face(c1));
                if f0 == f1 {
                    continue;
                }
                let other = if f0 == f { f1 } else { f0 };
                if seen[other.idx()] || !faces.contains(&other) {
                    continue;
                }
                if b.coedge(c0).rev == b.coedge(c1).rev {
                    flip_face(b, other);
                    flips += 1;
                }
                seen[other.idx()] = true;
                stack.push(other);
            }
        }
    }
    if volume_is_negative(b, &faces) {
        for &f in &faces {
            flip_face(b, f);
        }
        flips += faces.len();
    }
    flips
}

/// Orient every live shell; returns total face flips.
pub fn orient_body(b: &mut Body) -> usize {
    let shells: Vec<ShellId> = b.shell_ids().collect();
    shells.into_iter().map(|s| orient_shell(b, s)).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check;
    use crate::make::tests::box_body;
    use crate::mass::mass_props;

    #[test]
    fn built_faces_are_consistent() {
        let b = box_body();
        for f in b.face_ids() {
            assert!(loops_consistent(&b, f), "{}", b.face_id_str(f));
        }
    }

    #[test]
    fn orient_repairs_flipped_faces() {
        let mut b = box_body();
        flip_face(&mut b, FaceId(2));
        flip_face(&mut b, FaceId(0));
        assert!(!check(&b, 1e-9).is_empty());
        let n = orient_body(&mut b);
        assert!(n == 2 || n == 10, "{n}"); // seed face may be a flipped one
        assert!(check(&b, 1e-9).is_empty());
        assert!((mass_props(&b).volume - 8000.0).abs() < 1e-9);
        // fully inside-out -> all flipped back
        for f in 0..6 {
            flip_face(&mut b, FaceId(f));
        }
        assert!(mass_props(&b).volume < 0.0);
        orient_body(&mut b);
        assert!((mass_props(&b).volume - 8000.0).abs() < 1e-9);
        assert_eq!(b.planar_normal(FaceId(0)).y, -1.0);
    }

    #[test]
    fn planar_loop_fix() {
        let mut b = box_body();
        let l0 = b.face(FaceId(1)).loops[0];
        reverse_loop(&mut b, l0);
        assert!(!loops_consistent(&b, FaceId(1)));
        assert!(fix_planar_loops(&mut b, FaceId(1)));
        assert!(loops_consistent(&b, FaceId(1)));
    }
}
