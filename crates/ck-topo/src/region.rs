//! Point-in-solid classification and lump separation.
//!
//! Classification uses the generalised winding number
//! w(q) = 1/(4π) ∮∮ (p-q)·N / |p-q|³ dA over all faces, evaluated with the
//! exact face integrator of mass.rs (refined until stable). Robust to
//! seams / poles and needs no ray-surface intersection.

use crate::body::{Body, FaceId, LumpId, ShellId};
use crate::mass::{face_integral, signed_volume};
use ck_geom::Vec3;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Containment {
    Inside,
    Outside,
    On,
}

/// Winding number of `faces` about `q` (≈1 inside, 0 outside, ½ on).
pub fn winding_number(b: &Body, faces: &[FaceId], q: Vec3) -> f64 {
    let at = |res: usize| -> f64 {
        faces
            .iter()
            .map(|&f| {
                face_integral::<1>(b, f, res, &|p, n| {
                    let d = p - q;
                    let r2 = d.len2();
                    if r2 < 1e-30 {
                        [0.0]
                    } else {
                        [d.dot(n) / (r2 * r2.sqrt())]
                    }
                })[0]
            })
            .sum::<f64>()
            / (4.0 * PI)
    };
    let mut prev = at(1);
    let mut res = 2;
    while res <= 16 {
        let w = at(res);
        if (w - prev).abs() < 1e-3 {
            return w;
        }
        prev = w;
        res *= 2;
    }
    prev
}

/// Distance from `q` to the nearest face surface point that is (roughly)
/// on the face: sampled projection check via the winding of neighbours is
/// avoided — this uses surface closest points and accepts them when the
/// projected point is close to the face's own vertices/edges bbox.
fn near_boundary(b: &Body, q: Vec3, tol: f64) -> bool {
    let (lo, hi) = b.bbox();
    let pad = tol.max(1e-9);
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        let (_, _, p) = s.closest_point(q);
        if p.dist(q) <= tol
            && p.x >= lo.x - pad
            && p.y >= lo.y - pad
            && p.z >= lo.z - pad
            && p.x <= hi.x + pad
            && p.y <= hi.y + pad
            && p.z <= hi.z + pad
        {
            return true;
        }
    }
    false
}

/// Classify `q` against the whole body.
pub fn classify(b: &Body, q: Vec3, tol: f64) -> Containment {
    let faces: Vec<FaceId> = b.face_ids().collect();
    let w = winding_number(b, &faces, q);
    if (w - 0.5).abs() < 0.3 && near_boundary(b, q, tol) {
        return Containment::On;
    }
    if w > 0.5 {
        Containment::Inside
    } else {
        Containment::Outside
    }
}

/// Connected face components (faces linked through shared edges).
pub fn components(b: &Body) -> Vec<Vec<FaceId>> {
    let mut seen = vec![false; b.faces.len()];
    let mut out = vec![];
    for f in b.face_ids() {
        if seen[f.idx()] {
            continue;
        }
        seen[f.idx()] = true;
        let mut comp = vec![];
        let mut stack = vec![f];
        while let Some(g) = stack.pop() {
            comp.push(g);
            for h in b.face_neighbors(g) {
                if !seen[h.idx()] {
                    seen[h.idx()] = true;
                    stack.push(h);
                }
            }
        }
        comp.sort();
        out.push(comp);
    }
    out
}

/// Rebuild shells/lumps from connectivity: each connected component is a
/// shell; components with negative signed volume are voids and join the
/// smallest positive shell that contains them. Lump order follows the
/// lowest face id of each outer shell (keeps wire `lump` numbering stable).
pub fn rebuild_lumps(b: &mut Body) {
    let comps = components(b);
    for s in &mut b.shells {
        s.alive = false;
        s.faces.clear();
    }
    for l in &mut b.lumps {
        l.alive = false;
        l.shells.clear();
    }
    let vols: Vec<f64> = comps.iter().map(|c| signed_volume(b, c)).collect();
    let mut shell_of: Vec<Option<ShellId>> = vec![None; comps.len()];
    let mut lump_of_shell: Vec<(ShellId, LumpId, f64, usize)> = vec![];
    for (i, c) in comps.iter().enumerate() {
        if vols[i] >= 0.0 {
            let l = b.add_lump();
            let s = b.add_shell(l);
            for &f in c {
                b.faces[f.idx()].shell = s;
            }
            b.shells[s.idx()].faces = c.clone();
            shell_of[i] = Some(s);
            lump_of_shell.push((s, l, vols[i], i));
        }
    }
    for (i, c) in comps.iter().enumerate() {
        if vols[i] >= 0.0 {
            continue;
        }
        let probe = b.vertex(b.coedge_start(b.lp(b.face(c[0]).loops[0]).coedges[0])).p;
        let host = lump_of_shell
            .iter()
            .filter(|(_, _, _, j)| winding_number(b, &comps[*j], probe) > 0.5)
            .min_by(|a, b2| a.2.partial_cmp(&b2.2).unwrap())
            .map(|x| x.1);
        let l = host.unwrap_or_else(|| b.add_lump());
        let s = b.add_shell(l);
        for &f in c {
            b.faces[f.idx()].shell = s;
        }
        b.shells[s.idx()].faces = c.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::tests::{box_body, rect};
    use crate::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    use ck_geom::{Frame, Vec2};

    #[test]
    fn classify_box() {
        let b = box_body(); // x 0..40, y 0..10, z -20..0
        assert_eq!(classify(&b, Vec3::new(20.0, 5.0, -10.0), 1e-6), Containment::Inside);
        assert_eq!(classify(&b, Vec3::new(39.9, 9.9, -0.1), 1e-6), Containment::Inside);
        assert_eq!(classify(&b, Vec3::new(41.0, 5.0, -10.0), 1e-6), Containment::Outside);
        assert_eq!(classify(&b, Vec3::new(20.0, 10.0, -10.0), 1e-6), Containment::On);
    }

    #[test]
    fn classify_holed_cylinder_part() {
        let fr = Frame::identity();
        let circ = ProfileLoop::new(vec![ProfileSeg::circle("c", Vec2::new(20.0, 10.0), 5.0).reversed()]);
        let p = Profile::new(fr, vec![ProfileRegion { outer: rect(["a", "b", "c", "d"], 0.0, 0.0, 40.0, 20.0), holes: vec![circ] }]);
        let b = extrude("F2", &p, ExtrudeOpts::blind(10.0)).unwrap();
        assert_eq!(classify(&b, Vec3::new(20.0, 10.0, 5.0), 1e-6), Containment::Outside);
        assert_eq!(classify(&b, Vec3::new(20.0, 16.0, 5.0), 1e-6), Containment::Inside);
    }

    #[test]
    fn lumps_from_components() {
        let fr = Frame::identity();
        let p = Profile::new(
            fr,
            vec![
                ProfileRegion { outer: rect(["a0", "a1", "a2", "a3"], 0.0, 0.0, 10.0, 10.0), holes: vec![] },
                ProfileRegion { outer: rect(["b0", "b1", "b2", "b3"], 20.0, 0.0, 30.0, 10.0), holes: vec![] },
            ],
        );
        let mut b = extrude("F2", &p, ExtrudeOpts::blind(5.0)).unwrap();
        assert_eq!(components(&b).len(), 2);
        rebuild_lumps(&mut b);
        assert_eq!(b.num_lumps(), 2);
        assert_eq!(b.lump_index(b.face_lump(FaceId(7))), 1);
    }
}
