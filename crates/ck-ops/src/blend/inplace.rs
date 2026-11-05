//! PROTOTYPE (not wired into the default build path): an IN-PLACE blend corner.
//!
//! Every other blend here is a swept tool applied by boolean. This file builds one corner without a tool, on the
//! topology edit layer of `ck-topo`, to find out what that costs: a concave vertical edge between two planar walls
//! whose top outline was rounded earlier (the rim fillet of a body made of two walls meets in a mitre edge at the
//! concave corner). The vertical fillet then ends against the rim fillet, and the rolling ball turns the corner on a
//! torus:
//!
//! ```text
//!   before                               after
//!   W1 | E | W2   vertical edge E        W1 | L1 | F | L2 | W2     F: cylinder (the vertical fillet)
//!   R1 | M | R2   mitre M (rim)          R1 | T1 | T | T2 | R2     T: torus (the rim ball round the corner)
//!   bottom corner Vb                     bottom arc Ab (F | bottom plane)
//! ```
//!
//! The walls, the bottom plane, the top plane and the two rim cylinders are trimmed along the contact curves (a
//! shared edge is moved, so both faces it bounds follow), the mitre and the old vertical edge are removed, and the two
//! new faces are stitched in by chaining their loops against the loops they meet. Nothing is intersected: all contact
//! curves are circles and lines known in closed form.
//!
//! Scope. Wall 1 is a plane. Wall 2 is a plane perpendicular to it (exercised: sweep of 32 radius pairs against the
//! other order of the same blends) or a vertical cylinder with a torus rim fillet (the geometry is written down but
//! UNTESTED: no body the engine builds reaches it, see docs/blend-inplace-prototype.md). The pattern is a closed
//! 4-valent top vertex and a 3-valent bottom vertex; anything else returns `None` and the ordinary builders run.
//!
//! Enabled by `CK_BLEND_INPLACE=1` in [`super::rounds::blend_edges`] (single-edge selections only).

use super::diag::dbg;
use super::BlendError;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, FaceName, LoopId, VertexId};
use ck_topo::make::plane_pcurve;

/// `CK_BLEND_INPLACE=1`
pub(super) fn enabled() -> bool {
    std::env::var_os("CK_BLEND_INPLACE").is_some_and(|v| v == "1")
}

/// What the recogniser found around the concave edge.
struct Setup {
    e: EdgeId,
    w: [FaceId; 2],
    /// bottom plane, top plane, the two rim cylinders
    bottom: FaceId,
    top: FaceId,
    rim: [FaceId; 2],
    vb: VertexId,
    vt: VertexId,
    vtop: VertexId,
    /// bottom edges (wall i | bottom), rim edges at the wall (wall i | rim i), top edges (rim i | top), mitre
    eb: [EdgeId; 2],
    eh: [EdgeId; 2],
    eq: [EdgeId; 2],
    m: EdgeId,
    up: Vec3,
    /// height of the rim fillet's start above the bottom, of the top plane
    h_c: f64,
    h_top: f64,
}

/// Trace why the recogniser gave up (`CK_BLEND_DEBUG`).
fn nope<T>(line: u32) -> Option<T> {
    dbg(&format!("in-place corner: not the pattern (inplace.rs:{line})"));
    None
}

fn edges_at(b: &Body, v: VertexId) -> Vec<EdgeId> {
    b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect()
}

fn other_end(b: &Body, e: EdgeId, v: VertexId) -> VertexId {
    if b.edge(e).v0 == v {
        b.edge(e).v1
    } else {
        b.edge(e).v0
    }
}

fn recognise(b: &Body, e: EdgeId) -> Option<Setup> {
    let ed = b.edge(e);
    if !matches!(ed.curve, Curve::Line { .. }) {
        return nope(line!());
    }
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return nope(line!());
    }
    // wall 0 is the plane; wall 1 a plane or a cylinder along the edge
    let mut w = [fs[0], fs[1]];
    if !matches!(b.face(w[0]).surface, Surface::Plane { .. }) {
        w.swap(0, 1);
    }
    if !matches!(b.face(w[0]).surface, Surface::Plane { .. }) || !matches!(b.face(w[1]).surface, Surface::Plane { .. } | Surface::Cylinder { .. }) {
        return nope(line!());
    }
    // which end is the rim end: four edges, one of them between two other faces
    let mut ends = [(ed.v0, edges_at(b, ed.v0)), (ed.v1, edges_at(b, ed.v1))];
    ends.sort_by_key(|(_, es)| std::cmp::Reverse(es.len()));
    let ((vt, et), (vb, eb_all)) = (&ends[0], &ends[1]);
    if et.len() != 4 || eb_all.len() != 3 {
        let kinds = |es: &Vec<EdgeId>| es.iter().map(|&x| format!("e{} {} {:?}", x.idx(), b.edge(x).curve.kind(), b.edge_faces(x).iter().map(|&f| format!("{}:{}", b.face_id_str(f), b.face(f).surface.kind())).collect::<Vec<_>>())).collect::<Vec<_>>();
        dbg(&format!("in-place corner: top end {:?}, bottom end {:?}", kinds(et), kinds(eb_all)));
        return nope(line!());
    }
    let (vt, vb) = (*vt, *vb);
    let faces_of = |x: EdgeId| b.edge_faces(x);
    let mut eh = [None; 2];
    let mut m = None;
    for &x in et.iter().filter(|&&x| x != e) {
        let f = faces_of(x);
        if f.len() != 2 {
            return nope(line!());
        }
        match (f.contains(&w[0]), f.contains(&w[1])) {
            (true, false) => eh[0] = Some(x),
            (false, true) => eh[1] = Some(x),
            (false, false) => m = Some(x),
            _ => return None,
        }
    }
    let (eh, m) = ([eh[0].or_else(|| nope(line!()))?, eh[1].or_else(|| nope(line!()))?], m.or_else(|| nope(line!()))?);
    let rim = [faces_of(eh[0]).into_iter().find(|f| *f != w[0]).or_else(|| nope(line!()))?, faces_of(eh[1]).into_iter().find(|f| *f != w[1]).or_else(|| nope(line!()))?];
    if !(faces_of(m).len() == 2 && faces_of(m).contains(&rim[0]) && faces_of(m).contains(&rim[1])) {
        return nope(line!());
    }
    if !matches!(b.face(rim[0]).surface, Surface::Cylinder { .. }) {
        return nope(line!());
    }
    let rim1_ok = match &b.face(w[1]).surface {
        Surface::Plane { .. } => matches!(b.face(rim[1]).surface, Surface::Cylinder { .. }),
        _ => matches!(b.face(rim[1]).surface, Surface::Torus { .. }),
    };
    if !rim1_ok {
        return nope(line!());
    }
    // bottom end: two wall edges in one plane
    let mut eb = [None; 2];
    let mut bottom = None;
    for &x in eb_all.iter().filter(|&&x| x != e) {
        let f = faces_of(x);
        if f.len() != 2 {
            return nope(line!());
        }
        let i = if f.contains(&w[0]) { 0 } else if f.contains(&w[1]) { 1 } else { return None };
        eb[i] = Some(x);
        let p = f.into_iter().find(|f| *f != w[i]).or_else(|| nope(line!()))?;
        if bottom.is_some_and(|q| q != p) {
            return nope(line!());
        }
        bottom = Some(p);
    }
    let (eb, bottom) = ([eb[0].or_else(|| nope(line!()))?, eb[1].or_else(|| nope(line!()))?], bottom.or_else(|| nope(line!()))?);
    // top end of the mitre: the two rim cylinders meet the top plane there
    let vtop = other_end(b, m, vt);
    let etop = edges_at(b, vtop);
    if etop.len() != 3 {
        return nope(line!());
    }
    let mut eq = [None; 2];
    let mut top = None;
    for &x in etop.iter().filter(|&&x| x != m) {
        let f = faces_of(x);
        if f.len() != 2 {
            return nope(line!());
        }
        let i = if f.contains(&rim[0]) { 0 } else if f.contains(&rim[1]) { 1 } else { return None };
        eq[i] = Some(x);
        let p = f.into_iter().find(|f| *f != rim[i]).or_else(|| nope(line!()))?;
        if top.is_some_and(|q| q != p) {
            return nope(line!());
        }
        top = Some(p);
    }
    let (eq, top) = ([eq[0].or_else(|| nope(line!()))?, eq[1].or_else(|| nope(line!()))?], top.or_else(|| nope(line!()))?);
    for f in [bottom, top] {
        if !matches!(b.face(f).surface, Surface::Plane { .. }) {
            return nope(line!());
        }
    }
    let (pb, pt, ptop) = (b.vertex(vb).p, b.vertex(vt).p, b.vertex(vtop).p);
    let up = (pt - pb).try_normalize().or_else(|| nope(line!()))?;
    let (h_c, h_top) = ((pt - pb).dot(up), (ptop - pb).dot(up));
    // the planes are perpendicular to the edge, the walls parallel to it
    for f in [bottom, top] {
        if b.planar_normal(f).cross(up).len() > 1e-9 {
            return nope(line!());
        }
    }
    if b.planar_normal(w[0]).dot(up).abs() > 1e-9 {
        return nope(line!());
    }
    if let Surface::Cylinder { frame, .. } = &b.face(w[1]).surface {
        if frame.z.cross(up).len() > 1e-9 {
            return nope(line!());
        }
    }
    Some(Setup { e, w, bottom, top, rim, vb, vt, vtop, eb, eh, eq, m, up, h_c, h_top })
}

/// The point `p` lies inside the parameter range of edge `x`, away from its end `v` (the end `p` replaces).
fn moves_inside(b: &Body, x: EdgeId, v: VertexId, p: Vec3) -> bool {
    let ed = b.edge(x);
    let mut t = ed.curve.closest_param(p);
    if matches!(ed.curve, Curve::Circle { .. }) {
        let tau = std::f64::consts::TAU;
        while t < ed.t0 - 1e-12 {
            t += tau;
        }
        while t > ed.t0 + tau {
            t -= tau;
        }
    }
    let (lo, hi) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
    let eps = 1e-6;
    let _ = v;
    t > lo + eps && t < hi - eps
}

/// Arc of the circle about `c` (axis chosen so the sweep from `a` to `b` is positive and below pi).
fn arc(c: Vec3, a: Vec3, b: Vec3) -> (Curve, f64, f64) {
    let (x, y) = (a - c, b - c);
    let r = x.len();
    let n = x.cross(y).normalize();
    let sweep = (x.dot(y) / (r * y.len())).clamp(-1.0, 1.0).acos();
    (Curve::circle_x(c, n, x, r), 0.0, sweep)
}

fn param(c: &Curve, p: Vec3) -> f64 {
    c.closest_param(p)
}

/// Move end `old` of edge `e` to vertex `nv` at point `p`.
fn retarget(b: &mut Body, e: EdgeId, old: VertexId, nv: VertexId, p: Vec3) {
    let mut t = param(&b.edge(e).curve, p);
    if matches!(b.edge(e).curve, Curve::Circle { .. }) {
        // keep the parameter in the edge's own turn
        let (t0, tau) = (b.edge(e).t0, std::f64::consts::TAU);
        while t < t0 - 1e-12 {
            t += tau;
        }
        while t > t0 + tau {
            t -= tau;
        }
    }
    let ed = &mut b.edges[e.idx()];
    if ed.v0 == old {
        ed.v0 = nv;
        ed.t0 = t;
    } else {
        ed.v1 = nv;
        ed.t1 = t;
    }
}

fn plane_frame(b: &Body, f: FaceId) -> Option<Frame> {
    match &b.face(f).surface {
        Surface::Plane { frame } => Some(*frame),
        _ => None,
    }
}

fn pc_on(b: &Body, f: FaceId, curve: &Curve) -> Option<Curve2> {
    plane_frame(b, f).and_then(|fr| plane_pcurve(&fr, curve))
}

/// A new edge from vertex `va` to `vb` along `curve[t0, t1]`.
fn new_edge(b: &mut Body, curve: Curve, t0: f64, t1: f64, va: VertexId, vb: VertexId) -> EdgeId {
    b.add_edge(curve, t0, t1, va, vb)
}

/// Replace the coedge of `old` in loop `lp` by coedges of `news` (in order), each oriented to chain with the previous
/// coedge end; the edges named must connect. Returns false when they do not chain.
fn replace_in_loop(b: &mut Body, lp: LoopId, old: EdgeId, news: &[EdgeId], pcs: &[Option<Curve2>]) -> bool {
    let cs = b.lp(lp).coedges.clone();
    let Some(i) = cs.iter().position(|&c| b.coedge(c).edge == old) else { return false };
    let prev = cs[(i + cs.len() - 1) % cs.len()];
    let mut cur = b.coedge_end(prev);
    let mut made = vec![];
    for (k, &ne) in news.iter().enumerate() {
        let rev = if b.edge(ne).v0 == cur {
            cur = b.edge(ne).v1;
            false
        } else if b.edge(ne).v1 == cur {
            cur = b.edge(ne).v0;
            true
        } else {
            return false;
        };
        b.coedges.push(ck_topo::body::Coedge { edge: ne, rev, lp, pcurve: pcs[k].clone(), alive: true });
        let c = CoedgeId(b.coedges.len() as u32 - 1);
        b.edges[ne.idx()].coedges.push(c);
        made.push(c);
    }
    // the old coedge goes away; the loop must close on the next coedge
    let next = cs[(i + 1) % cs.len()];
    if cur != b.coedge_start(next) && cs.len() > 1 {
        return false;
    }
    b.coedges[cs[i].idx()].alive = false;
    let mut out = cs.clone();
    out.splice(i..=i, made);
    b.loops[lp.idx()].coedges = out;
    true
}

/// Put `arc` into the outer loop of planar face `f` between the two coedges that now end / start at the vertices `ends`.
fn insert_arc(b: &mut Body, f: FaceId, arc: EdgeId, ends: [VertexId; 2]) -> bool {
    let lp = outer_loop(b, f);
    let cs = b.lp(lp).coedges.clone();
    let Some(i) = cs.iter().position(|&c| ends.contains(&b.coedge_end(c))) else { return false };
    let rev = b.edge(arc).v1 == b.coedge_end(cs[i]);
    let pcurve = pc_on(b, f, &b.edge(arc).curve);
    b.coedges.push(ck_topo::body::Coedge { edge: arc, rev, lp, pcurve, alive: true });
    let c = CoedgeId(b.coedges.len() as u32 - 1);
    b.edges[arc.idx()].coedges.push(c);
    b.loops[lp.idx()].coedges.insert(i + 1, c);
    true
}

fn outer_loop(b: &Body, f: FaceId) -> LoopId {
    b.face(f).loops[0]
}

/// Chain `edges` into the loop of a new face: the first edge is traversed forward when `first_forward` (against the
/// direction its other face uses), the rest follow end to start. False when the loop does not close.
fn stitch_face(b: &mut Body, f: FaceId, edges: &[EdgeId], first_forward: bool, pcs: Vec<Option<Curve2>>) -> bool {
    let lp = b.add_loop(f);
    let mut cur;
    let mut used = vec![false; edges.len()];
    let e0 = edges[0];
    let mut rev = !first_forward;
    cur = if rev { b.edge(e0).v0 } else { b.edge(e0).v1 };
    b.add_coedge(lp, e0, rev, pcs[0].clone());
    used[0] = true;
    for _ in 1..edges.len() {
        let Some(k) = (0..edges.len()).find(|&k| !used[k] && (b.edge(edges[k]).v0 == cur || b.edge(edges[k]).v1 == cur)) else { return false };
        rev = b.edge(edges[k]).v1 == cur && b.edge(edges[k]).v0 != cur;
        cur = if rev { b.edge(edges[k]).v0 } else { b.edge(edges[k]).v1 };
        b.add_coedge(lp, edges[k], rev, pcs[k].clone());
        used[k] = true;
    }
    cur == b.coedge_start(b.lp(lp).coedges[0])
}

/// Fillet the concave vertical edge `e` with radius `r` in place. `None`: not the pattern this prototype knows.
pub(super) fn concave_under_rim(b: &Body, e: EdgeId, r: f64, feature: &str) -> Option<Result<Body, BlendError>> {
    let s = recognise(b, e).or_else(|| nope(line!()))?;
    let fail = |m: &str| Some(Err(BlendError::detail("blendUnsupported", &format!("in-place corner: {m}"))));
    let up = s.up;
    let vbp = b.vertex(s.vb).p;
    let pt = b.vertex(s.vt).p;
    if (pt - vbp).dot(up) <= 0.0 {
        return nope(line!());
    }
    // outward normals at the edge: wall 0 is a plane; wall 1 a plane or a vertical cylinder (its axis point at the bottom level)
    let n1 = b.planar_normal(s.w[0]);
    let hub: Option<(Vec3, f64)> = match &b.face(s.w[1]).surface {
        Surface::Cylinder { frame, r } => Some((frame.o + frame.z * (vbp - frame.o).dot(frame.z), *r)),
        _ => None,
    };
    let n2 = match hub {
        None => b.planar_normal(s.w[1]),
        Some((h, _)) => (vbp - h).try_normalize().or_else(|| nope(line!()))?,
    };
    // concave: the direction into the first wall (left of its coedge about the outward normal) is not against the second normal
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == s.w[0]).or_else(|| nope(line!()))?;
    let t = b.edge(e).curve.tangent(0.5 * (b.edge(e).t0 + b.edge(e).t1)).normalize();
    let tdir = if b.coedge(co).rev { -t } else { t };
    if n1.cross(tdir).dot(n2) < 0.0 {
        return nope(line!()); // convex
    }
    // the rim ball radius, from the height of the rim fillet; the rim faces must carry it
    let rt = s.h_top - s.h_c;
    if rt <= 0.0 {
        return nope(line!());
    }
    match (&b.face(s.rim[0]).surface, &b.face(s.rim[1]).surface) {
        (Surface::Cylinder { r: r1, .. }, Surface::Cylinder { r: r2, .. }) if (r1 - rt).abs() < 1e-6 && (r2 - rt).abs() < 1e-6 => {}
        (Surface::Cylinder { r: r1, .. }, Surface::Torus { minor, .. }) if (r1 - rt).abs() < 1e-6 && (minor - rt).abs() < 1e-6 => {}
        _ => return nope(line!()),
    }
    // fillet axis at the bottom level: distance r from the plane, r from the second wall (a plane or the hub cylinder)
    let a0 = match hub {
        None => {
            if n1.dot(n2).abs() > 1e-9 {
                return fail("walls are not perpendicular");
            }
            vbp + (n1 + n2) * r
        }
        Some((h, rh)) => {
            // on the plane offset by r, at distance rh + r from the hub axis, on the side of the first wall's face
            let w = n1.cross(up).normalize();
            let away = {
                let o = other_end(b, s.eb[0], s.vb);
                (b.vertex(o).p - vbp).dot(w)
            };
            let w = if away < 0.0 { -w } else { w };
            let q0 = vbp + n1 * r - h;
            let (bq, cq) = (q0.dot(w), q0.len2() - (rh + r) * (rh + r));
            let disc = bq * bq - cq;
            if disc < 0.0 {
                return fail("the fillet does not fit between the wall and the hub");
            }
            let roots = [-bq - disc.sqrt(), -bq + disc.sqrt()];
            let sroot = roots.into_iter().filter(|x| *x > 1e-9).fold(f64::INFINITY, f64::min);
            if !sroot.is_finite() {
                return fail("no fillet axis on the wall side");
            }
            vbp + n1 * r + w * sroot
        }
    };
    let rr = r + rt;
    let foot_b = match hub {
        None => [a0 - n1 * r, a0 - n2 * r],
        Some((h, rh)) => [a0 - n1 * r, h + (a0 - h) * (rh / (rh + r))],
    };
    // outward direction of the second wall at its contact point
    let n2f = match hub {
        None => n2,
        Some((h, _)) => (foot_b[1] - h).normalize(),
    };
    let foot_c = [foot_b[0] + up * s.h_c, foot_b[1] + up * s.h_c];
    let cen = [foot_c[0] - n1 * rt, foot_c[1] - n2f * rt];
    let q = [cen[0] + up * rt, cen[1] + up * rt];
    // the rim faces carry the ball path through these points
    let on_axis = |fr: &Frame, p: Vec3| {
        let d = p - fr.o;
        (d - fr.z * d.dot(fr.z)).len()
    };
    if let Surface::Cylinder { frame, .. } = &b.face(s.rim[0]).surface {
        if on_axis(frame, cen[0]) > 1e-6 {
            return nope(line!());
        }
    }
    match &b.face(s.rim[1]).surface {
        Surface::Cylinder { frame, .. } if on_axis(frame, cen[1]) > 1e-6 => return nope(line!()),
        Surface::Torus { frame, major, .. } if (on_axis(frame, cen[1]) - major).abs() > 1e-6 => return nope(line!()),
        _ => {}
    }
    // every trimmed edge keeps a positive length
    let spans = [(s.eb[0], s.vb, foot_b[0]), (s.eb[1], s.vb, foot_b[1]), (s.eh[0], s.vt, foot_c[0]), (s.eh[1], s.vt, foot_c[1]), (s.eq[0], s.vtop, q[0]), (s.eq[1], s.vtop, q[1])];
    if !spans.iter().all(|&(x, v, p)| moves_inside(b, x, v, p)) {
        return Some(Err(BlendError::code("blendOverflow")));
    }
    dbg(&format!("in-place corner: axis {:?} rim ball radius {rt:.4}, torus major {rr:.4}", a0));
    let mut o = b.clone();
    // vertices
    let (pb, pt_, pq) = ([o.add_vertex(foot_b[0]), o.add_vertex(foot_b[1])], [o.add_vertex(foot_c[0]), o.add_vertex(foot_c[1])], [o.add_vertex(q[0]), o.add_vertex(q[1])]);
    // move the shared edges
    retarget(&mut o, s.eb[0], s.vb, pb[0], foot_b[0]);
    retarget(&mut o, s.eb[1], s.vb, pb[1], foot_b[1]);
    retarget(&mut o, s.eh[0], s.vt, pt_[0], foot_c[0]);
    retarget(&mut o, s.eh[1], s.vt, pt_[1], foot_c[1]);
    retarget(&mut o, s.eq[0], s.vtop, pq[0], q[0]);
    retarget(&mut o, s.eq[1], s.vtop, pq[1], q[1]);
    // new edges
    let line = |a: Vec3, c: Vec3| (Curve::line(a, c - a), 0.0, (c - a).len());
    let (l1c, l1a, l1b) = line(foot_b[0], foot_c[0]);
    let l1 = new_edge(&mut o, l1c, l1a, l1b, pb[0], pt_[0]);
    let (l2c, l2a, l2b) = line(foot_b[1], foot_c[1]);
    let l2 = new_edge(&mut o, l2c, l2a, l2b, pb[1], pt_[1]);
    let (cc, ca, cb) = arc(a0, foot_b[0], foot_b[1]);
    let ab = new_edge(&mut o, cc, ca, cb, pb[0], pb[1]);
    let (cc, ca, cb) = arc(a0 + up * s.h_c, foot_c[0], foot_c[1]);
    let at = new_edge(&mut o, cc, ca, cb, pt_[0], pt_[1]);
    let (cc, ca, cb) = arc(cen[0], foot_c[0], q[0]);
    let t1 = new_edge(&mut o, cc, ca, cb, pt_[0], pq[0]);
    let (cc, ca, cb) = arc(cen[1], foot_c[1], q[1]);
    let t2 = new_edge(&mut o, cc, ca, cb, pt_[1], pq[1]);
    let (cc, ca, cb) = arc(a0 + up * s.h_top, q[0], q[1]);
    let tt = new_edge(&mut o, cc, ca, cb, pq[0], pq[1]);
    // loops of the old faces
    let pcs = |o: &Body, f: FaceId, c: &Curve| pc_on(o, f, c);
    let cur_of = |o: &Body, x: EdgeId| o.edge(x).curve.clone();
    let mut ok = true;
    for (i, wi) in s.w.iter().enumerate() {
        let lp = outer_loop(&o, *wi);
        let ne = if i == 0 { l1 } else { l2 };
        let pc = pcs(&o, *wi, &cur_of(&o, ne));
        ok &= replace_in_loop(&mut o, lp, s.e, &[ne], &[pc]);
    }
    // the bottom and top planes lose a corner: the arc joins the two trimmed edges
    ok &= insert_arc(&mut o, s.bottom, ab, pb);
    ok &= insert_arc(&mut o, s.top, tt, pq);
    // rim cylinders: the quarter circle replaces the mitre
    for (i, &ri) in s.rim.iter().enumerate() {
        let lp = outer_loop(&o, ri);
        ok &= replace_in_loop(&mut o, lp, s.m, &[if i == 0 { t1 } else { t2 }], &[None]);
    }
    if !ok {
        return Some(Err(BlendError::detail("blendFailed", "in-place corner: a loop does not close")));
    }
    // the mitre and the vertical edge are gone
    for x in [s.m, s.e] {
        for c in o.edge_coedges(x) {
            o.coedges[c.idx()].alive = false;
        }
        o.edges[x.idx()].alive = false;
    }
    // new faces: F (the vertical fillet, normal toward its axis) and T (the rim ball round the corner)
    let shell = o.face(s.w[0]).shell;
    let Curve::Circle { frame: fz, .. } = Curve::circle(a0, up, 1.0) else { return nope(line!()) };
    let f_face = o.add_face(shell, Surface::Cylinder { frame: fz, r }, false, FaceName::blend(feature, 0));
    let tz = Frame { o: a0 + up * s.h_c, ..fz };
    let t_face = o.add_face(shell, Surface::Torus { frame: tz, major: rr, minor: rt }, true, FaceName::other(feature, "RB0"));
    // F: L1 against W1's direction, then the rest by chaining
    let w1_dir_fwd = {
        let c = o.lp(outer_loop(&o, s.w[0])).coedges.iter().copied().find(|&c| o.coedge(c).edge == l1).unwrap();
        !o.coedge(c).rev
    };
    let ok_f = stitch_face(&mut o, f_face, &[l1, at, l2, ab], !w1_dir_fwd, vec![None, None, None, None]);
    // T: T1 against R1's direction
    let r1_dir_fwd = {
        let c = o.lp(outer_loop(&o, s.rim[0])).coedges.iter().copied().find(|&c| o.coedge(c).edge == t1).unwrap();
        !o.coedge(c).rev
    };
    let ok_t = stitch_face(&mut o, t_face, &[t1, tt, t2, at], !r1_dir_fwd, vec![None, None, None, None]);
    if !(ok_f && ok_t) {
        return Some(Err(BlendError::detail("blendFailed", "in-place corner: the new faces do not close")));
    }
    ck_topo::edit::gc(&mut o);
    ck_topo::region::rebuild_lumps(&mut o);
    Some(Ok(o))
}

/// Entry from the selection driver: a single concave edge pick whose end meets a rim fillet.
pub(super) fn try_pick(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str) -> Option<Result<Body, BlendError>> {
    if edges.len() != 1 {
        return None;
    }
    let e = super::find::find_edge(b, &edges[0].0, &edges[0].1, edges[0].2)?;
    concave_under_rim(b, e, r, feature)
}
