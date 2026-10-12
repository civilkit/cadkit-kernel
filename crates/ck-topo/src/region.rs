//! Point-in-solid classification and lump separation.
//!
//! Classification uses the generalised winding number
//! w(q) = 1/(4π) ∮∮ (p-q)·N / |p-q|³ dA over all faces, evaluated with the
//! exact face integrator of mass.rs (refined until stable). Robust to
//! seams / poles and needs no ray-surface intersection.

use crate::body::{Body, EdgeId, FaceId, LumpId, ShellId, VertexId};
use ck_geom::curve::Curve;
use ck_geom::pcurve::unwrap_near;
use ck_geom::Vec2;
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
///
/// Near the boundary (closer than 1% of the bbox diagonal) the winding
/// number's quadrature is unreliable (it misclassified points 1e-3 from an
/// edge), so the side is read off the NEAREST boundary feature instead:
/// a face interior point by its outward normal, an edge point by the sum
/// of its faces' normals, a vertex by the angle-weighted normal sum of its
/// face corners (the pseudo-normal test of Bærentzen & Aanæs, exact for
/// polyhedra and locally for smooth faces). Far from it the winding number
/// is accurate and decides.
pub fn classify(b: &Body, q: Vec3, tol: f64) -> Containment {
    let faces: Vec<FaceId> = b.face_ids().collect();
    if faces.is_empty() {
        return Containment::Outside;
    }
    let (lo, hi) = b.bbox();
    let diag = (hi - lo).len();
    if let Some((d, side)) = nearest_feature(b, q) {
        if d <= tol {
            return Containment::On;
        }
        // (a body with a long multi-span spline face -- a screw sweep -- trusts
        // the nearest feature at any distance: its winding number costs
        // seconds per face and the closest-point search is global)
        let long_spline = || b.face_ids().any(|f| b.face(f).surface.is_long_sweep());
        if d < 0.01 * diag || !diag.is_finite() || long_spline() {
            return if side < 0.0 { Containment::Inside } else { Containment::Outside };
        }
    }
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

/// uv polygons of a face's loops from the coedge pcurves, each kept
/// continuous across periodic wraps.
fn face_uv_loops(b: &Body, f: FaceId) -> Vec<Vec<Vec2>> {
    let face = b.face(f);
    let (pu, pv) = face.surface.periods();
    let mut out = vec![];
    for &l in &face.loops {
        let mut poly: Vec<Vec2> = vec![];
        for &c in &b.lp(l).coedges {
            let (ta, tb) = b.coedge_params(c);
            let n = match b.edge(b.coedge(c).edge).curve {
                Curve::Line { .. } => 2,
                _ => 32,
            };
            for k in 0..n {
                let mut uv = b.coedge_uv(c, ta + (tb - ta) * k as f64 / n as f64, poly.last().copied());
                if let Some(prev) = poly.last() {
                    if let Some(p) = pu {
                        uv.x = unwrap_near(uv.x, prev.x, p);
                    }
                    if let Some(p) = pv {
                        uv.y = unwrap_near(uv.y, prev.y, p);
                    }
                }
                poly.push(uv);
            }
        }
        out.push(poly);
    }
    out
}

fn crossings(p: Vec2, poly: &[Vec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, c) = (poly[i], poly[(i + 1) % n]);
        if (a.y > p.y) != (c.y > p.y) && p.x < a.x + (p.y - a.y) * (c.x - a.x) / (c.y - a.y) {
            inside = !inside;
        }
    }
    inside
}

/// Is surface point `uv` inside face `f` (even-odd over its loops, each
/// tested at periodic shifts)? Faces without loops (full sphere / torus)
/// cover their whole surface.
fn in_face(b: &Body, f: FaceId, uv: Vec2) -> bool {
    let loops = face_uv_loops(b, f);
    if loops.iter().all(|l| l.len() < 3) {
        return true;
    }
    let (pu, pv) = b.face(f).surface.periods();
    let shifts = |p: Option<f64>| p.map_or(vec![0.0], |p| vec![0.0, -p, p, -2.0 * p, 2.0 * p]);
    let (su, sv) = (shifts(pu), shifts(pv));
    let hit = |l: &Vec<Vec2>| su.iter().any(|&a| sv.iter().any(|&c| crossings(Vec2::new(uv.x + a, uv.y + c), l)));
    loops.iter().filter(|l| hit(l)).count() % 2 == 1
}

/// Outward normal of face `f` at the surface point closest to `p`.
fn normal_at(b: &Body, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = b.face(f).surface.closest_param(p);
    b.outward_normal(f, u, v).normalize()
}

/// Closest point of `q` on edge `e` within its range: (distance, point,
/// end vertex if the range end is closest).
fn edge_closest(b: &Body, e: EdgeId, q: Vec3) -> (f64, Vec3, Option<VertexId>) {
    let ed = b.edge(e);
    let (t0, t1) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
    let mut t = ed.curve.closest_param(q);
    if let Some(p) = ed.curve.period() {
        t = t0 + (t - t0).rem_euclid(p);
    }
    if t >= t0 && t <= t1 {
        let p = ed.curve.eval(t);
        let (pa, pb) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
        let d = p.dist(q);
        // an interior point that coincides with an end counts as the vertex
        let at_end = (p.dist(pa) < 1e-12 * (1.0 + pa.len())).then_some(ed.v0)
            .or((p.dist(pb) < 1e-12 * (1.0 + pb.len())).then_some(ed.v1));
        return (d, p, at_end);
    }
    let (da, db) = (b.vertex(ed.v0).p.dist(q), b.vertex(ed.v1).p.dist(q));
    let v = if da <= db { ed.v0 } else { ed.v1 };
    (da.min(db), b.vertex(v).p, Some(v))
}

/// Angle-weighted pseudo-normal of vertex `v`: Σ over face corners at `v`
/// of (corner angle) × (face outward normal at v).
fn vertex_pseudo_normal(b: &Body, v: VertexId) -> Vec3 {
    let p = b.vertex(v).p;
    let mut n = Vec3::ZERO;
    // degenerate corners (a pole between two seam coedges has angle 0):
    // plain sum of the incident faces' normals
    let mut plain = Vec3::ZERO;
    for f in b.face_ids() {
        let face = b.face(f);
        for &l in &face.loops {
            let cs = &b.lp(l).coedges;
            for (i, &c) in cs.iter().enumerate() {
                if b.coedge_end(c) != v {
                    continue;
                }
                let c2 = cs[(i + 1) % cs.len()];
                let tan = |c: crate::body::CoedgeId, at_start: bool| {
                    let ce = b.coedge(c);
                    let ed = b.edge(ce.edge);
                    let (ta, tb) = b.coedge_params(c);
                    let t = if at_start { ta } else { tb };
                    let d = ed.curve.deriv(t).normalize();
                    if ce.rev {
                        -d
                    } else {
                        d
                    }
                };
                let (din, dout) = (-tan(c, false), tan(c2, true));
                let ang = din.dot(dout).clamp(-1.0, 1.0).acos();
                let nf = normal_at(b, f, p);
                n = n + nf * ang;
                plain = plain + nf;
            }
        }
    }
    if n.len() < 1e-9 {
        plain
    } else {
        n
    }
}

/// Nearest boundary feature of `q`: (distance, side) with side < 0 inside,
/// > 0 outside (dot of q − p with the feature's (pseudo-)normal).
fn nearest_feature(b: &Body, q: Vec3) -> Option<(f64, f64)> {
    #[derive(Clone, Copy)]
    enum Feat {
        Face(FaceId),
        Edge(EdgeId),
        Vertex(VertexId),
    }
    let mut best: Option<(f64, Vec3, Feat)> = None;
    let mut take = |d: f64, p: Vec3, f: Feat| {
        if best.map_or(true, |(bd, _, _)| d < bd) {
            best = Some((d, p, f));
        }
    };
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        let (u, v, p) = s.closest_point(q);
        if in_face(b, f, Vec2::new(u, v)) {
            take(p.dist(q), p, Feat::Face(f));
        }
    }
    for e in b.edge_ids() {
        if b.edge_faces(e).is_empty() {
            continue;
        }
        let (d, p, end) = edge_closest(b, e, q);
        // prefer the edge (or vertex) on exact ties with a face point
        let d = d * (1.0 - 1e-12);
        take(d, p, end.map_or(Feat::Edge(e), Feat::Vertex));
    }
    let (d, p, feat) = best?;
    let n = match feat {
        Feat::Face(f) => normal_at(b, f, p),
        Feat::Edge(e) => b.edge_faces(e).iter().fold(Vec3::ZERO, |a, &f| a + normal_at(b, f, p)),
        Feat::Vertex(v) => vertex_pseudo_normal(b, v),
    };
    Some((d, (q - p).dot(n)))
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
    // a lone component gets its own lump whatever its orientation (a void
    // shell with no host also opens a fresh lump): skip the volume integral
    let vols: Vec<f64> = if comps.len() == 1 { vec![0.0] } else { comps.iter().map(|c| signed_volume(b, c)).collect() };
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
    fn classify_near_boundary() {
        use Containment::*;
        let b = box_body(); // x 0..40, y 0..10, z -20..0
        let fr = Frame::identity();
        let circ = ProfileLoop::new(vec![ProfileSeg::circle("c", Vec2::new(20.0, 10.0), 5.0).reversed()]);
        let p = Profile::new(fr, vec![ProfileRegion { outer: rect(["a", "b", "c", "d"], 0.0, 0.0, 40.0, 20.0), holes: vec![circ] }]);
        let h = extrude("F2", &p, ExtrudeOpts::blind(10.0)).unwrap(); // hole r5 at (20,10), z 0..10
        for d in [1e-2, 1e-3, 1e-4, 1e-5, 1e-6 * 5.0] {
            // box: near faces, an edge, a corner
            assert_eq!(classify(&b, Vec3::new(20.0, 10.0 - d, -10.0), 1e-6), Inside, "face in {d}");
            assert_eq!(classify(&b, Vec3::new(20.0, 10.0 + d, -10.0), 1e-6), Outside, "face out {d}");
            assert_eq!(classify(&b, Vec3::new(40.0 - d, 10.0 - d, -10.0), 1e-6), Inside, "edge in {d}");
            assert_eq!(classify(&b, Vec3::new(40.0 + d, 10.0 + d, -10.0), 1e-6), Outside, "edge out {d}");
            assert_eq!(classify(&b, Vec3::new(40.0 + d, 10.0 - d, -10.0), 1e-6), Outside, "edge side {d}");
            assert_eq!(classify(&b, Vec3::new(40.0 - d, 10.0 - d, -d), 1e-6), Inside, "corner in {d}");
            assert_eq!(classify(&b, Vec3::new(40.0 + d, 10.0 + d, d), 1e-6), Outside, "corner out {d}");
            // holed block: near the cylinder wall and its rim
            assert_eq!(classify(&h, Vec3::new(25.0 + d, 10.0, 5.0), 1e-6), Inside, "wall in {d}");
            assert_eq!(classify(&h, Vec3::new(25.0 - d, 10.0, 5.0), 1e-6), Outside, "wall out {d}");
            assert_eq!(classify(&h, Vec3::new(20.0, 15.0 + d, 10.0 - d), 1e-6), Inside, "rim in {d}");
            assert_eq!(classify(&h, Vec3::new(20.0, 15.0 - d, 10.0 - d), 1e-6), Outside, "rim out {d}");
        }
        // sphere r10 (one face, seam + poles)
        let semi = ProfileLoop::new(vec![
            ProfileSeg::arc("e1", Vec2::ZERO, Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)),
            ProfileSeg::line("e2", Vec2::new(0.0, 10.0), Vec2::new(0.0, -10.0)),
        ]);
        let sp = crate::make::revolve::revolve(
            "F2",
            &Profile::new(fr, vec![ProfileRegion { outer: semi, holes: vec![] }]),
            Vec3::ZERO,
            Vec3::Y,
            0.0,
            std::f64::consts::TAU,
        )
        .unwrap();
        for (d, dir) in [(1e-3, Vec3::new(1.0, 2.0, 3.0)), (1e-5, Vec3::new(-1.0, 0.0, 0.0)), (1e-4, Vec3::new(0.0, 1.0, 1e-9))] {
            let u = dir.normalize();
            assert_eq!(classify(&sp, u * (10.0 - d), 1e-6), Inside, "sphere in {d}");
            assert_eq!(classify(&sp, u * (10.0 + d), 1e-6), Outside, "sphere out {d}");
        }
        assert_eq!(classify(&b, Vec3::new(40.0, 10.0, -10.0), 1e-6), On);
        assert_eq!(classify(&b, Vec3::new(40.0, 10.0, 0.0), 1e-6), On);
        assert_eq!(classify(&h, Vec3::new(25.0, 10.0, 5.0), 1e-6), On);
        assert_eq!(classify(&b, Vec3::new(20.0, 10.0 + 5e-7, -10.0), 1e-6), On);
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
