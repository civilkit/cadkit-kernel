//! End conditions of a selection (what a tool does at the end of a blended edge): flush, run-on, mitre and
//! overhang-corner vertices, and the clipping solids they need.

use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use super::{model::*, rounds::*};

/// Reflex-corner pairs of the selection, see [`EndConditions::mitre`]: two selected straight convex edges of
/// planar faces that share a vertex and exactly one face, each of whose end face at the vertex leans
/// toward the material (the other edge's wall).
pub(super) fn mitre_pairs(b: &Body, sel: &[EdgeId], equal_chamfer: bool) -> Vec<(Vec3, Vec3, Vec3)> {
    let convex_line = |e: EdgeId| -> bool {
        super::mixed::line_edge_concave(b, e) == Some(false)
    };
    let away = |e: EdgeId, v: ck_topo::body::VertexId| -> Vec3 {
        let ed = b.edge(e);
        let other = if ed.v0 == v { ed.v1 } else { ed.v0 };
        (b.vertex(other).p - b.vertex(v).p).normalize()
    };
    // the end face of `e` at `v`: the one face at the vertex not on the edge; it must lean toward the material
    let leans = |e: EdgeId, v: ck_topo::body::VertexId| -> bool {
        let efs = b.edge_faces(e);
        let mut ends: Vec<FaceId> = vec![];
        for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
            for f in b.edge_faces(x) {
                if !efs.contains(&f) && !ends.contains(&f) {
                    ends.push(f);
                }
            }
        }
        ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(-away(e, v)) < -1e-9
    };
    let concave_line = |e: EdgeId| -> bool { super::mixed::line_edge_concave(b, e) == Some(true) };
    let mut out = vec![];
    for (i, &a) in sel.iter().enumerate() {
        for &c in &sel[i + 1..] {
            let both_convex = convex_line(a) && convex_line(c);
            let both_concave = concave_line(a) && concave_line(c);
            if a == c || !(both_convex || both_concave) {
                continue;
            }
            let (ea, ec) = (b.edge(a), b.edge(c));
            for v in [ea.v0, ea.v1] {
                if ea.v0 == ea.v1 || !(ec.v0 == v || ec.v1 == v) {
                    continue;
                }
                let (fa, fc) = (b.edge_faces(a), b.edge_faces(c));
                if fa.iter().filter(|f| fc.contains(f)).count() != 1 {
                    continue;
                }
                if both_convex {
                    if !leans(a, v) || !leans(c, v) {
                        // an equal-distance chamfer chain that turns by a slight angle at a convex corner of its common
                        // face (a rounded outline's straight run into a skewed one): the tools reach past the vertex
                        // into the material of the other edge's wall and over-cut it; each keeps its own side of the
                        // mitre plane instead, which is the exact meeting of the two chamfer planes when both walls
                        // stand at one angle to the common face
                        if !(equal_chamfer && skew_convex_pair(b, a, c, v)) {
                            continue;
                        }
                    }
                } else {
                    // two concave edges (a cove round the foot of a post) that meet at an OUTSIDE corner: the
                    // third edge of the vertex, between the two walls, is convex and not filleted
                    let others: Vec<EdgeId> = b.edge_ids().filter(|&x| x != a && x != c && (b.edge(x).v0 == v || b.edge(x).v1 == v)).collect();
                    if others.len() != 1 || sel.contains(&others[0]) || !convex_line(others[0]) {
                        continue;
                    }
                }
                let (da, dc) = (away(a, v), away(c, v));
                let p = b.vertex(v).p;
                out.push((p, da, dc));
                out.push((p, dc, da));
            }
        }
    }
    out
}

/// Two straight convex edges of planar faces at `v` that share one face, leave `v` at an angle other than a straight
/// continuation or a right angle, and whose other faces make the same angle with the shared face (so the mitre
/// of two equal-distance chamfers is the bisector plane); the vertex has just these two edges and a third, unselected.
fn skew_convex_pair(b: &Body, a: EdgeId, c: EdgeId, v: ck_topo::body::VertexId) -> bool {
    let away = |e: EdgeId| -> Vec3 {
        let ed = b.edge(e);
        let other = if ed.v0 == v { ed.v1 } else { ed.v0 };
        (b.vertex(other).p - b.vertex(v).p).normalize()
    };
    let (da, dc) = (away(a), away(c));
    let cs = da.dot(dc);
    if cs < -1.0 + 1e-6 || cs.abs() < 1e-6 || cs > 1.0 - 1e-6 {
        return false;
    }
    let valence = b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).count();
    if valence != 3 {
        return false;
    }
    let (fa, fc) = (b.edge_faces(a), b.edge_faces(c));
    let Some(&shared) = fa.iter().find(|f| fc.contains(f)) else { return false };
    let (Some(&oa), Some(&oc)) = (fa.iter().find(|&&f| f != shared), fc.iter().find(|&&f| f != shared)) else { return false };
    if [shared, oa, oc].iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
        return false;
    }
    let ns = b.planar_normal(shared);
    (ns.dot(b.planar_normal(oa)) - ns.dot(b.planar_normal(oc))).abs() < 1e-6
}

/// How a concave blend's tool is trimmed where its contact line would leave a face that is narrower than the blend
/// (see [`overflow_clip`]).
pub(super) enum OverflowClip {
    /// keep the half-space `(p - o) . n >= 0`
    Half(Vec3, Vec3),
    /// cut away this solid (the air inside a concave cylinder)
    Cut(Body),
    /// keep only this solid (the material inside a convex cylinder)
    Keep(Body),
}

impl OverflowClip {
    pub(super) fn apply(&self, tool: &Body, tol: f64) -> Result<Body, BlendError> {
        let op = match self {
            OverflowClip::Half(o, n) => return clip_halfspace(tool, *o, *n, tol),
            OverflowClip::Cut(c) => (c, Op::Cut),
            OverflowClip::Keep(c) => (c, Op::Intersect),
        };
        crate::boolean::boolean(tool, op.0, op.1, tol).map(|r| r.body).map_err(|_| BlendError::code("blendFailed"))
    }
}

/// A concave blend whose contact point `t` (reached from the edge point `a` across face `f`) lies past the far edge of
/// `f`: the face `g` beyond that edge, when the edge is convex and `g` is a plane perpendicular to `f` or a cylinder
/// whose axis is normal to `f`, so that `g` continued past its edge is the same surface. The blend then rolls off `f`
/// and is trimmed by `g` continued (the material is added only on the material side of `g`'s surface; `g` grows down to
/// the blend face), which is what a large fillet in a narrow ledge or over a low step does. `None` otherwise.
pub(super) fn overflow_clip(b: &Body, f: FaceId, e: EdgeId, a: Vec3, t: Vec3, bb: (Vec3, Vec3), tol: f64) -> Option<OverflowClip> {
    use crate::boolean::classify::{point_in_face, Where};
    let on_f = |q: Vec3| match &b.face(f).surface {
        Surface::Plane { .. } => q,
        sf => sf.closest_point(q).2,
    };
    let inside = |s: f64| point_in_face(b, f, on_f(a + (t - a) * s), tol) != Where::Out;
    const N: usize = 64;
    let k = (1..=N).find(|&k| !inside(k as f64 / N as f64))?;
    let (mut lo, mut hi) = ((k - 1) as f64 / N as f64, k as f64 / N as f64);
    if !inside(lo) {
        return None;
    }
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if inside(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let q = on_f(a + (t - a) * lo);
    let size = (bb.1 - bb.0).len();
    let (e2, d) = b
        .face_edges(f)
        .into_iter()
        // (not a seam of `f`)
        .filter(|&x| x != e && b.edge_faces(x).iter().any(|&h| h != f))
        .map(|x| {
            let ed = b.edge(x);
            (x, crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0.min(ed.t1), ed.t0.max(ed.t1)), q).1)
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if d > 1e-6 * (1.0 + size) {
        return None;
    }
    let fs = b.edge_faces(e2);
    let g = *fs.iter().find(|&&x| x != f)?;
    if fs.len() != 2 {
        return None;
    }
    let nrm = |h: FaceId| {
        let (u, v) = b.face(h).surface.closest_param(q);
        b.outward_normal(h, u, v).normalize()
    };
    let (nf, ng) = (nrm(f), nrm(g));
    let leg = t - a;
    let leg = (leg - nf * leg.dot(nf)).normalize();
    // convex edge: g faces the side the contact runs off to
    if !(ng.dot(leg) > 1e-6) {
        return None;
    }
    // wherever along the blended edge the contact leaves `f`, it leaves across `g`: the in-face line from the edge
    // point to the contact meets `g`'s surface inside `g` (else `g` continued would trim the blend where some other
    // face bounds `f`, or where `f` is wide enough)
    let reach = (t - a).len();
    let near_g = |p: Vec3| {
        point_in_face(b, g, p, tol) != Where::Out
            || b.face_edges(g).into_iter().any(|x| {
                let ed = b.edge(x);
                crate::boolean::imprint::near_in_range(&ed.curve, (ed.t0.min(ed.t1), ed.t0.max(ed.t1)), p, 1e-6 * (1.0 + size))
            })
    };
    let first_hit = |p: Vec3, d: Vec3| -> Option<f64> {
        match &b.face(g).surface {
            Surface::Plane { frame } => {
                let den = d.dot(ng);
                (den.abs() > 1e-12).then(|| (frame.o - p).dot(ng) / den)
            }
            Surface::Cylinder { frame, r } => {
                let ax = frame.z.normalize();
                let perp = |v: Vec3| v - ax * v.dot(ax);
                let (w, dd) = (perp(p - frame.o), perp(d));
                let (qa, qb, qc) = (dd.dot(dd), 2.0 * w.dot(dd), w.dot(w) - r * r);
                let disc = qb * qb - 4.0 * qa * qc;
                if qa < 1e-24 || disc < -1e-9 * (1.0 + r * r) {
                    return None;
                }
                let sq = disc.max(0.0).sqrt();
                let (s0, s1) = ((-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa));
                Some(if s0 >= -1e-9 { s0 } else { s1 })
            }
            _ => None,
        }
    };
    {
        let ed = b.edge(e);
        let (t0, t1) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
        let tm = 0.5 * (t0 + t1);
        let sgn = if nrm_at(b, f, ed.curve.eval(tm)).cross(ed.curve.tangent(tm)).dot(leg) >= 0.0 { 1.0 } else { -1.0 };
        for i in 0..9 {
            let s = t0 + (t1 - t0) * (i as f64 + 0.5) / 9.0;
            let p = ed.curve.eval(s);
            let d = nrm_at(b, f, p).cross(ed.curve.tangent(s)).normalize() * sgn;
            if point_in_face(b, f, on_f(p + d * reach), tol) != Where::Out {
                continue;
            }
            let h = first_hit(p, d)?;
            if !(h > -1e-9 && h < reach) || !near_g(p + d * h) {
                return None;
            }
        }
    }
    match &b.face(g).surface {
        // (the plane's own point: `q` is found only to the face test's tolerance)
        Surface::Plane { frame } if ng.dot(nf).abs() < 1e-9 => Some(OverflowClip::Half(frame.o, -ng)),
        Surface::Cylinder { frame, r } if frame.z.normalize().cross(nf).len() < 1e-9 => {
            let rel = q - frame.o;
            let rad = rel - frame.z * rel.dot(frame.z) / frame.z.dot(frame.z);
            let big = 4.0 * size + 4.0 * r + 10.0;
            let h = rel.dot(frame.z.normalize());
            let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), *r)]);
            // seam away from the contact
            let prof = Profile::new(ck_geom::frame_from(frame.o, frame.z, -rad), vec![ProfileRegion { outer: lp, holes: vec![] }]);
            let cyl = extrude("clip", &prof, ExtrudeOpts { s0: h - big, s1: h + big, dir: None, draft: 0.0 }).ok()?;
            Some(if ng.dot(rad) < 0.0 { OverflowClip::Cut(cyl) } else { OverflowClip::Keep(cyl) })
        }
        _ => None,
    }
}

/// The outward normal of face `f` at its point nearest `p`.
fn nrm_at(b: &Body, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = b.face(f).surface.closest_param(p);
    b.outward_normal(f, u, v).normalize()
}

/// `tool` cut down to the half-space `(p - o) . n >= 0`.
pub(super) fn clip_halfspace(tool: &Body, o: Vec3, n: Vec3, tol: f64) -> Result<Body, BlendError> {
    let (lo, hi) = tool.bbox();
    let big = 4.0 * (hi - lo).len() + 10.0;
    let u = if n.x.abs() < 0.9 { Vec3::X.cross(n) } else { Vec3::Y.cross(n) }.normalize();
    let w = n.cross(u);
    let pt = |a: f64, b: f64, c: f64| o + n * (a * big) + u * (b * big) + w * (c * big);
    // (corner loop, outward normal)
    let quads: [([Vec3; 4], Vec3); 6] = [
        ([pt(0., -1., -1.), pt(0., 1., -1.), pt(0., 1., 1.), pt(0., -1., 1.)], -n),
        ([pt(1., -1., -1.), pt(1., 1., -1.), pt(1., 1., 1.), pt(1., -1., 1.)], n),
        ([pt(0., -1., -1.), pt(1., -1., -1.), pt(1., -1., 1.), pt(0., -1., 1.)], -u),
        ([pt(0., 1., -1.), pt(1., 1., -1.), pt(1., 1., 1.), pt(0., 1., 1.)], u),
        ([pt(0., -1., -1.), pt(1., -1., -1.), pt(1., 1., -1.), pt(0., 1., -1.)], -w),
        ([pt(0., -1., 1.), pt(1., -1., 1.), pt(1., 1., 1.), pt(0., 1., 1.)], w),
    ];
    let polys: Vec<(Vec<Vec3>, FaceName)> = quads
        .iter()
        .enumerate()
        .map(|(i, (q, out))| {
            let nw = (q[1] - q[0]).cross(q[2] - q[0]);
            let mut v = q.to_vec();
            if nw.dot(*out) < 0.0 {
                v.reverse();
            }
            (v, FaceName::other("clip", &format!("k{i}")))
        })
        .collect();
    let block = super::vertex::polyhedron(&polys);
    crate::boolean::boolean(tool, &block, Op::Intersect, tol).map(|r| r.body).map_err(|_| BlendError::code("blendFailed"))
}

/// The planar face that ends an edge at vertex `v` (the edge runs on in direction `out` past it), when it is the only
/// other face there, the body ends at it (its outward normal points along `out`) and its plane is not
/// perpendicular to the edge: (a point on the plane, outward normal).
pub(super) fn oblique_end_plane(b: &Body, v: ck_topo::body::VertexId, faces: &[FaceId], out: Vec3) -> Option<(Vec3, Vec3)> {
    let mut ends: Vec<FaceId> = vec![];
    for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
        for f in b.edge_faces(x) {
            if !faces.contains(&f) && !ends.contains(&f) {
                ends.push(f);
            }
        }
    }
    if ends.len() != 1 || !matches!(b.face(ends[0]).surface, Surface::Plane { .. }) {
        return None;
    }
    let n = b.planar_normal(ends[0]).normalize();
    (n.dot(out) > 1e-6 && n.cross(out).len() > 1e-6).then(|| (b.vertex(v).p, n))
}

/// The planar faces that end an edge at vertex `v` (the edge runs on in direction `out` past it), when at least one of
/// them is oblique to the edge: one face as [`oblique_end_plane`], or two that meet at `v` along a convex edge (a rib's
/// slanted free face drawn to the corner of a step: the slanted face and the step's face end the edge together). The
/// body ends at each of them (outward normal along `out`): (a point on the planes, outward normals).
pub(super) fn oblique_end_planes(b: &Body, v: ck_topo::body::VertexId, faces: &[FaceId], out: Vec3) -> Option<(Vec3, Vec<Vec3>)> {
    if let Some((p, n)) = oblique_end_plane(b, v, faces, out) {
        return Some((p, vec![n]));
    }
    let at: Vec<EdgeId> = b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).collect();
    let mut ends: Vec<FaceId> = vec![];
    for &x in &at {
        for f in b.edge_faces(x) {
            if !faces.contains(&f) && !ends.contains(&f) {
                ends.push(f);
            }
        }
    }
    if ends.len() != 2 || ends.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
        return None;
    }
    let between = at.iter().copied().find(|&x| {
        let fs = b.edge_faces(x);
        fs.len() == 2 && fs.contains(&ends[0]) && fs.contains(&ends[1])
    })?;
    if super::mixed::line_edge_concave(b, between) != Some(false) {
        return None;
    }
    let ns: Vec<Vec3> = ends.iter().map(|&f| b.planar_normal(f).normalize()).collect();
    if ns.iter().any(|n| n.dot(out) <= 1e-6) || ns.iter().all(|n| n.cross(out).len() <= 1e-6) {
        return None;
    }
    Some((b.vertex(v).p, ns))
}

/// The outward normal of the planar face that ends an edge at a reflex corner at vertex `v` (the edge runs on in
/// direction `out` past it): it is the only other face there and it leans toward the material ahead (its outward
/// normal points back against `out`), like the wall of a petal that a hub arc's rim runs into.
pub(super) fn reflex_end_plane(b: &Body, v: ck_topo::body::VertexId, faces: &[FaceId], out: Vec3) -> Option<Vec3> {
    let mut ends: Vec<FaceId> = vec![];
    for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
        for f in b.edge_faces(x) {
            if !faces.contains(&f) && !ends.contains(&f) {
                ends.push(f);
            }
        }
    }
    if ends.len() != 1 || !matches!(b.face(ends[0]).surface, Surface::Plane { .. }) {
        return None;
    }
    let n = b.planar_normal(ends[0]).normalize();
    (n.dot(out) < -1e-6).then_some(n)
}

/// The solid of the cylinder that bounds an edge at its end vertex `v` (the edge runs
/// on in direction `od` past `v`), when that end meets exactly one more face, a
/// cylinder whose material lies inside it and ahead of the edge (a boss the edge runs
/// into): the tool must not cut into it, so it is cut by this solid.
pub(super) fn end_cylinder_solid(b: &Body, v: ck_topo::body::VertexId, od: Vec3, faces: &[FaceId], bb: (Vec3, Vec3)) -> Option<Body> {
    let mut ends: Vec<FaceId> = vec![];
    for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
        for f in b.edge_faces(x) {
            if !faces.contains(&f) && !ends.contains(&f) {
                ends.push(f);
            }
        }
    }
    if ends.len() != 1 {
        return None;
    }
    let Surface::Cylinder { frame, r } = &b.face(ends[0]).surface else { return None };
    let pv = b.vertex(v).p;
    let (u, w) = b.face(ends[0]).surface.closest_param(pv);
    let n = b.outward_normal(ends[0], u, w).normalize();
    let rel = pv - frame.o;
    let rad = rel - frame.z * rel.dot(frame.z);
    // material is ahead of the edge (the face leans toward it) and inside the cylinder (normal points away from the axis)
    if n.dot(od) > -1e-9 || n.dot(rad) <= 0.0 {
        return None;
    }
    let big = 4.0 * (bb.1 - bb.0).len() + 4.0 * rad.len() + 10.0;
    let t = rel.dot(frame.z);
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), *r)]);
    // the clip's seam on the side away from the vertex: the face's own frame may put it right at the vertex (an arc
    // that starts there), where the tool's end meets the cylinder, and the cut then loses the cylinder face
    let prof = Profile::new(ck_geom::frame_from(frame.o, frame.z, -rad), vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude("clip", &prof, ExtrudeOpts { s0: t - big, s1: t + big, dir: None, draft: 0.0 }).ok()
}

/// Vertices where two of the selected edges continue each other tangentially (see [`TANGENT_FLUSH`]).
pub(super) fn tangent_flush_vertices(b: &Body, sel: &[EdgeId]) -> Vec<Vec3> {
    let mut flush: Vec<Vec3> = vec![];
    {
        for &e in sel {
            let ed = b.edge(e);
            if ed.v0 == ed.v1 {
                continue;
            }
            for v in [ed.v0, ed.v1] {
                let d = ed.curve.tangent(if ed.v0 == v { ed.t0 } else { ed.t1 });
                if d.len() < 1e-12 {
                    continue;
                }
                let tangent_mate = sel.iter().copied().filter(|&x| x != e).any(|x| {
                    let ex = b.edge(x);
                    if ex.v0 != v && ex.v1 != v {
                        return false;
                    }
                    let dx = ex.curve.tangent(if ex.v0 == v { ex.t0 } else { ex.t1 });
                    dx.len() > 1e-12 && d.normalize().cross(dx.normalize()).len() < 1e-6
                });
                let p = b.vertex(v).p;
                if tangent_mate && !flush.iter().any(|q| q.dist(p) < 1e-9) {
                    flush.push(p);
                }
            }
        }
    }
    flush
}

/// End vertices of the selected edges where the outline goes on tangentially into an edge that is not selected (a
/// flank picked without the round petal end it runs into: the continuing edge shares one face with it and leaves the
/// vertex in the same direction). A convex fillet tool stops flush there: run on, it would cut into the rim of the
/// unselected edge and leave an overlapping end on the junction.
pub(super) fn open_tangent_ends(b: &Body, sel: &[EdgeId]) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = vec![];
    for &e in sel {
        let ed = b.edge(e);
        if ed.v0 == ed.v1 {
            continue;
        }
        let fe = b.edge_faces(e);
        // the end arc of a wall fillet (super::torus_corner) whose other rim is not selected runs on past it by design
        if fe.iter().any(|&f| matches!(b.face(f).name.role, Role::Blend(_))) {
            continue;
        }
        for v in [ed.v0, ed.v1] {
            let d = ed.curve.tangent(if ed.v0 == v { ed.t0 } else { ed.t1 });
            if d.len() < 1e-12 {
                continue;
            }
            let open = b.edge_ids().filter(|&x| x != e && !sel.contains(&x) && (b.edge(x).v0 == v || b.edge(x).v1 == v)).any(|x| {
                let ex = b.edge(x);
                let fx = b.edge_faces(x);
                if fx.len() != 2 || fx.iter().filter(|f| fe.contains(f)).count() != 1 {
                    return false;
                }
                let dx = ex.curve.tangent(if ex.v0 == v { ex.t0 } else { ex.t1 });
                dx.len() > 1e-12 && d.normalize().cross(dx.normalize()).len() < 1e-6
            });
            let p = b.vertex(v).p;
            if open && !out.iter().any(|q| q.dist(p) < 1e-9) {
                out.push(p);
            }
        }
    }
    out
}

/// The original vertices (with `true` when material lies behind the end face, so
/// a tool may run on past it) where two or more of the `sel` edges meet and not
/// all of them are concave straight edges of planar faces (three of those get the
/// sphere patch). An earlier tool of the feature shortens the later edge there; a
/// concave tool runs on to the original vertex.
pub(super) fn junction_vertices(b: &Body, sel: &[EdgeId], tol: f64) -> EndConditions {
    let concave = |e: EdgeId| -> bool {
        let ed = b.edge(e);
        if !matches!(ed.curve, Curve::Line { .. }) {
            return false;
        }
        let fs = b.edge_faces(e);
        if fs.len() != 2 || fs.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
            return false;
        }
        let d = (b.vertex(ed.v1).p - b.vertex(ed.v0).p).normalize();
        let Some(co) = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0]) else { return false };
        let tdir = if b.coedge(co).rev { -d } else { d };
        b.planar_normal(fs[0]).cross(tdir).dot(b.planar_normal(fs[1])) > 1e-9
    };
    let mut out: Vec<(Vec3, bool)> = vec![];
    let mut flush: Vec<Vec3> = vec![];
    if TANGENT_FLUSH.with(|c| c.get()) {
        flush = tangent_flush_vertices(b, sel);
    }
    for &e in sel {
        let ed = b.edge(e);
        if matches!(ed.curve, Curve::Line { .. }) {
            let efs = b.edge_faces(e);
            for (v, other) in [(ed.v0, ed.v1), (ed.v1, ed.v0)] {
                let outd = (b.vertex(v).p - b.vertex(other).p).normalize();
                let mut ends: Vec<FaceId> = vec![];
                for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                    for f in b.edge_faces(x) {
                        if !efs.contains(&f) && !ends.contains(&f) {
                            ends.push(f);
                        }
                    }
                }
                if ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(outd) < -1e-9 {
                    flush.push(b.vertex(v).p);
                }
            }
        }
        for v in [ed.v0, ed.v1] {
            let at: Vec<EdgeId> = sel.iter().copied().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).collect();
            let p = b.vertex(v).p;
            if at.len() < 2 || (at.len() == 3 && at.iter().all(|&x| concave(x))) || out.iter().any(|(q, _)| q.dist(p) <= tol) {
                continue;
            }
            // the end face of the concave edge here: the face at the vertex not on the edge
            let Some(&ce) = at.iter().find(|&&x| concave(x)) else { continue };
            let cd = b.edge(ce);
            let other = if cd.v0 == v { b.vertex(cd.v1).p } else { b.vertex(cd.v0).p };
            let out_dir = (p - other).normalize();
            let on_edge = b.edge_faces(ce);
            let mut ends: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !on_edge.contains(&f) && !ends.contains(&f) {
                        ends.push(f);
                    }
                }
            }
            // the faces other than the concave edge's own: the plane perpendicular to it decides
            let behind = ends.iter().filter(|&&f| matches!(b.face(f).surface, Surface::Plane { .. })).any(|&f| {
                let n = b.planar_normal(f);
                n.cross(out_dir).len() < 1e-6 && n.dot(out_dir) < 0.0
            });
            out.push((p, behind));
        }
    }
    EndConditions { ext: out, flush, mitre: vec![], patch: vec![] }
}

/// Both faces border a blend face of `feature` in `b` (their shared edge was
/// consumed by a tool of this feature).
pub(super) fn blended_by_earlier_tool(b: &Body, fa: &str, fb: &str, feature: &str) -> bool {
    let (Some(f1), Some(f2)) = (b.find_face(fa), b.find_face(fb)) else { return false };
    b.face_ids().any(|g| {
        let n = &b.face(g).name;
        n.feature == feature && matches!(n.role, Role::Blend(_)) && {
            let nb = b.face_neighbors(g);
            nb.contains(&f1) && nb.contains(&f2)
        }
    })
}
