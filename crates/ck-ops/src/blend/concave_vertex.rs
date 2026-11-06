//! A concave vertex of three equal fillets over plane / cylinder supports, one of whose edges is free-form (two
//! round rods that meet along a groin, standing on a plane): the rolling ball touches all three faces at once, so the
//! vertex gets the sphere patch `V<k>` the three strips are tangent to.
//!
//! The ball centre `c` is at distance `r` on the void side of each of the three faces (Newton on the signed
//! distances). The ball touches the strip of each edge along the circle in the plane through `c` perpendicular to the
//! strip's centre curve. The build is staged:
//!
//! 0. the corner block minus the ball is added ([`super::corner::sphere_block_minus_ball`]); its three planes through
//!    `c` are then faces the strips end on, and its sphere patch is the one the strips are tangent to along the
//!    block's arcs;
//! 1. the free-form edges are filleted, their chain ends flush on the block's plane (the usual end handling of a
//!    concave chain);
//! 2. the other selected edges follow, through the rim-corner build or the ordinary path (wall lines analytic).
//!
//! Any failure returns `None`, and the ordinary path (which refuses the branching selection) runs as before.

use super::build::{find_edge, FilletProfile, Spec};
use super::corner::sphere_block_minus_ball;
use super::diag::dbg;
use super::rounds::{blend_edges_inner, EDGE_RADII};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role, VertexId};

/// A face as a signed distance to the void side.
#[derive(Clone, Copy)]
enum Sup {
    Plane { o: Vec3, n: Vec3 },
    Cyl { o: Vec3, a: Vec3, r: f64, s: f64 },
}

impl Sup {
    /// Signed distance of `p` from the face (positive on the void side) and its unit gradient.
    fn sd(&self, p: Vec3) -> (f64, Vec3) {
        match *self {
            Sup::Plane { o, n } => ((p - o).dot(n), n),
            Sup::Cyl { o, a, r, s } => {
                let d = p - o;
                let perp = d - a * d.dot(a);
                let l = perp.len().max(1e-12);
                (s * (l - r), perp / l * s)
            }
        }
    }
}

fn support(b: &Body, f: FaceId, at: Vec3) -> Option<Sup> {
    let (u, v, _) = b.face(f).surface.closest_point(at);
    let nrm = b.outward_normal(f, u, v);
    match &b.face(f).surface {
        Surface::Plane { frame } => Some(Sup::Plane { o: frame.o, n: b.planar_normal(f) }),
        Surface::Cylinder { frame, r } => {
            let d = at - frame.o;
            let perp = d - frame.z * d.dot(frame.z);
            let s = if perp.dot(nrm) >= 0.0 { 1.0 } else { -1.0 };
            Some(Sup::Cyl { o: frame.o, a: frame.z, r: *r, s })
        }
        _ => None,
    }
}

/// The edge leaves `v` along the returned unit direction.
fn away(b: &Body, e: EdgeId, v: VertexId) -> Option<Vec3> {
    let ed = b.edge(e);
    let p = b.vertex(v).p;
    let dt = 1e-3 * (ed.t1 - ed.t0);
    let q = if ed.v0 == v { ed.curve.eval(ed.t0 + dt) } else { ed.curve.eval(ed.t1 - dt) };
    (q - p).try_normalize()
}

/// The edge is concave (the material wraps more than a half turn about it) at the vertex `v`.
fn concave_at(b: &Body, e: EdgeId, v: VertexId) -> Option<bool> {
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return None;
    }
    let p = b.vertex(v).p;
    let ed = b.edge(e);
    let w = away(b, e, v)?;
    let d = if ed.v0 == v { w } else { -w };
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0])?;
    let tdir = if b.coedge(co).rev { -d } else { d };
    let nrm = |f: FaceId| {
        let (u, vv, _) = b.face(f).surface.closest_point(p);
        b.outward_normal(f, u, vv)
    };
    let s = nrm(fs[0]).cross(tdir).dot(nrm(fs[1]));
    if s.abs() < 1e-9 {
        return None;
    }
    Some(s > 0.0)
}

/// The edge is built by the free-form path (a wall line between a plane and a parallel cylinder is analytic).
fn free_edge(b: &Body, e: EdgeId) -> bool {
    super::freeform::needs_freeform(b, e) && !super::torus_corner::is_wall_line(b, e)
}

/// One concave vertex of three selected edges.
struct Triple {
    c: Vec3,
    /// Per edge: its direction away from the vertex at the ball centre (perpendicular to the strip's end plane).
    u: [Vec3; 3],
    /// Per edge `i`: the block's outer plane opposite to it.
    outer: [(Vec3, Vec3); 3],
}

fn triples(b: &Body, eids: &[EdgeId], r: f64) -> Vec<Triple> {
    let mut out = vec![];
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 3 || inc.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1) {
            continue;
        }
        if !inc.iter().all(|e| eids.contains(e)) || !inc.iter().any(|&e| free_edge(b, e)) {
            continue;
        }
        if !inc.iter().all(|&e| concave_at(b, e, v) == Some(true)) {
            continue;
        }
        let mut faces: Vec<FaceId> = vec![];
        for &e in &inc {
            for f in b.edge_faces(e) {
                if !faces.contains(&f) {
                    faces.push(f);
                }
            }
        }
        if faces.len() != 3 {
            continue;
        }
        let vp = b.vertex(v).p;
        let Some(sups) = faces.iter().map(|&f| support(b, f, vp)).collect::<Option<Vec<Sup>>>() else { continue };
        // the ball centre: r on the void side of all three faces, near the vertex
        let mut sum = Vec3::ZERO;
        for s in &sups {
            sum = sum + s.sd(vp).1;
        }
        let mut c = vp + sum * r;
        let mut ok = false;
        for _ in 0..60 {
            let (mut g, mut f) = ([Vec3::ZERO; 3], [0.0; 3]);
            for (i, s) in sups.iter().enumerate() {
                let (d, gr) = s.sd(c);
                g[i] = gr;
                f[i] = r - d;
            }
            let Some(dx) = crate::intersect::solve3(g, f) else { break };
            c = c + dx;
            if dx.len() < 1e-13 * (1.0 + c.len()) {
                ok = true;
                break;
            }
        }
        if !ok {
            continue;
        }
        // per edge i (faces other than face i): the strip's end plane and the block's planes
        let mut u = [Vec3::ZERO; 3];
        let mut outer = [(Vec3::ZERO, Vec3::ZERO); 3];
        let mut good = true;
        for (i, &e) in inc.iter().enumerate() {
            let opp = faces.iter().position(|f| !b.edge_faces(e).contains(f));
            let Some(oi) = opp else {
                good = false;
                break;
            };
            let (ga, gb): (Vec<Vec3>, Vec<Vec3>) = {
                let mut it = (0..3).filter(|&k| k != oi);
                let (k0, k1) = (it.next().unwrap(), it.next().unwrap());
                (vec![sups[k0].sd(c).1], vec![sups[k1].sd(c).1])
            };
            let Some(mut t) = ga[0].cross(gb[0]).try_normalize() else {
                good = false;
                break;
            };
            let Some(w) = away(b, e, v) else {
                good = false;
                break;
            };
            if t.dot(w) < 0.0 {
                t = -t;
            }
            // the centre must lie beyond the vertex along every edge
            if (c - vp).dot(t) <= 1e-9 {
                good = false;
                break;
            }
            u[i] = t;
            let g = sups[oi].sd(c).1;
            let a = c - g * r;
            outer[i] = (-g, a - g * (r + 1.0));
        }
        if !good {
            continue;
        }
        out.push(Triple { c, u, outer });
    }
    out
}

/// `Some(result)` when the selection holds a concave vertex of this kind and the staged build was tried.
pub fn try_fillet(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Option<Result<Body, BlendError>> {
    if edges.len() < 3 || EDGE_RADII.with(|c| c.borrow().is_some()) {
        return None;
    }
    let eids: Vec<EdgeId> = edges.iter().map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect::<Option<_>>()?;
    let ts = triples(b, &eids, r);
    if ts.is_empty() {
        return None;
    }
    let res = staged(b, edges, &eids, &ts, r, feature, tol);
    match res {
        Ok(x) => Some(Ok(x)),
        Err(e) => {
            dbg(&format!("concave vertex build failed: {} {:?}", e.code, e.detail));
            None
        }
    }
}

fn staged(b: &Body, edges: &[(String, String, Vec3)], eids: &[EdgeId], ts: &[Triple], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let n = edges.len();
    let free: Vec<usize> = (0..n).filter(|&i| free_edge(b, eids[i])).collect();
    let rest: Vec<usize> = (0..n).filter(|i| !free.contains(i)).collect();
    if free.is_empty() {
        return Err(BlendError::code("blendUnsupported"));
    }
    let pre = format!("{feature}_g");
    dbg(&format!("concave vertex: {} triple(s), free edges {:?}", ts.len(), free));
    // stage 0: the corner blocks minus the balls. Their planes through the ball centres are faces the strips end on.
    let mut cur = b.clone();
    for (k, t) in ts.iter().enumerate() {
        let name = FaceName::other(feature, &format!("Vc{k}"));
        let tool = sphere_block_minus_ball(t.c, t.u, t.outer, r, feature, &name)?;
        let (next, _) = super::apply::apply_tool(&cur, &tool, Op::Union, &name, vec![], tol)?;
        cur = next;
    }
    if super::diag::on() {
        dbg(&format!("concave vertex: stage 0 volume +{:.4}", super::validity::volume(&cur) - super::validity::volume(b)));
    }
    // stage 1: the free-form edges
    let sel1: Vec<(String, String, Vec3)> = free.iter().map(|&i| edges[i].clone()).collect();
    cur = super::freeform::fillet_freeform(&cur, &sel1, r, &pre, tol)?;
    if super::diag::on() {
        dbg(&format!("concave vertex: stage 1 volume +{:.4}", super::validity::volume(&cur) - super::validity::volume(b)));
    }
    // stage 2: everything else
    if !rest.is_empty() {
        let sel2: Vec<(String, String, Vec3)> = rest.iter().map(|&i| edges[i].clone()).collect();
        let radii = vec![r; sel2.len()];
        cur = match super::mixed::fillet_mixed(&cur, &sel2, &radii, feature, tol) {
            Some(Ok(x)) => x,
            other => {
                dbg(&format!("concave vertex: stage 2 mixed build: {:?}", other.map(|r| r.map(|_| ()))));
                blend_edges_inner(&cur, &sel2, Spec::Fillet(r, FilletProfile::Circular), feature, tol)?
            }
        };
    }
    if super::diag::on() {
        dbg(&format!("concave vertex: stage 2 volume +{:.4}", super::validity::volume(&cur) - super::validity::volume(b)));
    }
    // blend names follow the selection: stage 1's `B<j>` is pick `free[j]`, stage 2's `B<j>` is pick `rest[j]`; the
    // sphere patches continue the numbering of the vertex blends the ordinary path made
    let mut renames: Vec<(usize, u32)> = vec![];
    let mut spare = n as u32;
    let mut next_v = 0usize;
    for f in cur.face_ids() {
        if let Role::Other(o) = &cur.face(f).name.role {
            if cur.face(f).name.feature == feature {
                if let Some(k) = o.strip_prefix('V').and_then(|x| x.parse::<usize>().ok()) {
                    next_v = next_v.max(k + 1);
                }
            }
        }
        let nm = &cur.face(f).name;
        let Role::Blend(k) = nm.role else { continue };
        let list = if nm.feature == pre {
            &free
        } else if nm.feature == feature {
            &rest
        } else {
            continue;
        };
        renames.push((f.idx(), list.get(k as usize).map_or_else(|| { spare += 1; spare - 1 }, |&i| i as u32)));
    }
    for (fi, k) in renames {
        cur.faces[fi].name.feature = feature.to_string();
        cur.faces[fi].name.role = Role::Blend(k);
    }
    for f in cur.face_ids().collect::<Vec<_>>() {
        if let Role::Other(o) = &cur.face(f).name.role {
            if cur.face(f).name.feature == feature {
                if let Some(k) = o.strip_prefix("Vc").and_then(|x| x.parse::<usize>().ok()) {
                    cur.faces[f.idx()].name.role = Role::Other(format!("V{}", next_v + k));
                }
            }
        }
    }
    Ok(cur)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_on};
    use crate::boolean::{boolean, Op as BOp};
    use ck_geom::Frame;
    use ck_topo::mass::mass_props;

    /// A plate with two cylindrical rods standing half out of it: rod A (radius 8, along z) and rod B (radius 6, along
    /// x) that ends inside A. The groin where B meets A is a spline; at its two ends it meets the plate top in a
    /// concave vertex of three edges (plate / A, plate / B, A / B).
    fn rods() -> Body {
        let plate = box_body("F2", 0.0, 0.0, 60.0, 40.0, 0.0, 10.0);
        let fa = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
        let a = cyl_on("F3", fa, 20.0, 10.0, 8.0, -35.0, -5.0);
        let fb = Frame { o: Vec3::ZERO, x: Vec3::Y, y: Vec3::Z, z: Vec3::X };
        let bb = cyl_on("F4", fb, 10.0, -20.0, 6.0, 20.0, 55.0);
        let u = boolean(&plate, &a, BOp::Union, 1e-6).unwrap().body;
        boolean(&u, &bb, BOp::Union, 1e-6).unwrap().body
    }

    /// The picks of the edges at the vertices where the groin meets the plate top.
    fn picks(b: &Body) -> Vec<(String, String, Vec3)> {
        let mut out = vec![];
        let mut seen: Vec<EdgeId> = vec![];
        for v in b.vertex_ids() {
            let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
            if inc.len() != 3 || !inc.iter().any(|&e| !matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. })) {
                continue;
            }
            if !inc.iter().all(|&e| concave_at(b, e, v) == Some(true)) {
                continue;
            }
            for e in inc {
                if seen.contains(&e) {
                    continue;
                }
                seen.push(e);
                let fs = b.edge_faces(e);
                let ed = b.edge(e);
                out.push((b.face_id_str(fs[0]), b.face_id_str(fs[1]), ed.curve.eval(0.5 * (ed.t0 + ed.t1))));
            }
        }
        out
    }

    #[test]
    fn groin_vertices_get_their_sphere_patches() {
        let b = rods();
        assert!(ck_topo::check::check(&b, 1e-6).is_empty());
        let sel = picks(&b);
        assert_eq!(sel.len(), 5, "{sel:?}");
        let v0 = mass_props(&b).volume;
        let out = super::super::build::fillet(&b, &sel, 1.0, "F9", 1e-6).unwrap();
        let spheres: Vec<_> = out.face_ids().filter(|&f| matches!(out.face(f).surface, Surface::Sphere { .. })).collect();
        assert_eq!(spheres.len(), 2);
        // the ball touches the plate top (y = 10), rod A (radius 8 about x = 20) and rod B (radius 6 about z = -20)
        let cx = 20.0 + 80f64.sqrt();
        let cz = 48f64.sqrt();
        let mut zs = vec![];
        for f in spheres {
            let Surface::Sphere { frame, r } = &out.face(f).surface else { unreachable!() };
            assert!((r - 1.0).abs() < 1e-9);
            assert!((frame.o.x - cx).abs() < 1e-6 && (frame.o.y - 11.0).abs() < 1e-6, "{:?}", frame.o);
            zs.push(frame.o.z);
        }
        zs.sort_by(|p, q| p.total_cmp(q));
        assert!((zs[0] + 20.0 + cz).abs() < 1e-6 && (zs[1] + 20.0 - cz).abs() < 1e-6, "{zs:?}");
        // material was added: the strips along the six edges and the two corners (each at most the pocket of the box
        // about the ball), and the body is still a valid solid
        let dv = mass_props(&out).volume - v0;
        assert!(dv > 0.0 && dv < 60.0, "{dv}");
        assert!(ck_topo::check::check(&out, 1e-3).is_empty(), "{:?}", ck_topo::check::check(&out, 1e-3));
    }
}
