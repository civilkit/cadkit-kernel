//! Blend construction (ck-ops/src/blend/build.rs) — FIRST PASS: the
//! simplest case only, a constant-radius fillet along a straight edge
//! between two planar faces (convex or concave), built as a boolean with
//! a "corner prism" tool whose arc face becomes the blend face `B<k>`.
//!
//! Oracle evidence (probes on box F2, fillet feature F3):
//! - `fillet r=2` on `F2.C1|F2.S.e1`: faces `C0, S.e3` (untouched, original
//!   order), `C1, S.e1` (the edge's faces, ref order), `S.e4, S.e2` (end
//!   faces at the edge's start, end vertex), `F3.B0` (cylinder, meta
//!   `{feature:"F3"}`); end-face edges are circle arcs c = axis point,
//!   n = edge direction (sign from the end face), r = 2.
//! - two edges → `B0`, `B1` in ref order; touched faces in first-touch order.
//! - errors: radius 0 → `invalidValue`; no / unknown edge → `missingEdge`;
//!   radius 30 → `blendOverflow` with detail
//!   `"F3.B0: spring leaves its face away from its edges"`; the body is
//!   left unchanged in all error cases.
//! - GUESS/known mismatch: the oracle blends IN PLACE (blend/inplace.rs):
//!   surviving edges keep their arena number in the wire key `#n`, split
//!   pieces and new edges are appended (fillet probe: #0..#4, #6, #8..#11,
//!   #14, #16..#19). This boolean-based pass renumbers edges in face-walk
//!   order; face ids, order, geometry and massProps match.

use super::BlendError;
use crate::boolean::assemble::{build, select};
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::imprint::imprint;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// Edge between the two named faces closest to `at` (fillet `edges[]` ref).
pub fn find_edge(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let (f, g) = (b.find_face(fa)?, b.find_face(fb)?);
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && fs.contains(&f) && fs.contains(&g)
        })
        .min_by(|&x, &y| {
            let d = |e: EdgeId| {
                let ed = b.edge(e);
                crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
            };
            d(x).partial_cmp(&d(y)).unwrap()
        })
}

/// Constant-radius fillets of `edges` (face-name pairs + pick point, in
/// ref order), blend faces `B0, B1, …` of `feature`.
///
/// Each pair is first normalised so that the face whose loop runs the edge
/// FORWARD comes first (oracle box probes, bracket F12: selection
/// `["F2.S.e8","F2.S.e9"]` lists `F12.B0|F2.S.e9` before `…|F2.S.e8`, and
/// the touched faces come e9, e8). The result is written in the original's
/// IN-PLACE arena layout ([`in_place_layout`]), so wire `#n` keep the
/// input's numbers.
pub fn fillet(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Fillet(r), feature, tol)
}

/// Equal-distance chamfers (`Chamfer(d, d)` in the trace) of `edges`.
pub fn chamfer(b: &Body, edges: &[(String, String, Vec3)], d: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Chamfer(d), feature, tol)
}

/// What one blend cuts: a rolling-ball fillet of radius `r` or an
/// equal-distance chamfer of leg `d`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Spec {
    Fillet(f64),
    Chamfer(f64),
}

impl Spec {
    fn size(self) -> f64 {
        match self {
            Spec::Fillet(r) | Spec::Chamfer(r) => r,
        }
    }
}

/// Shared fillet / chamfer driver (see [`fillet`]).
pub fn blend_edges(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let r = spec.size();
    if !(r > 0.0) || !r.is_finite() {
        return Err(BlendError::code("invalidValue"));
    }
    if edges.is_empty() {
        return Err(BlendError::code("missingEdge"));
    }
    if matches!(spec, Spec::Chamfer(_)) && edges.len() > 1 {
        // Oracle (box probes): chamfers run in ROUNDS. An edge's round is
        // the number of earlier-selected edges in its connected component
        // (shared end vertices): the first edge of every chain goes in one
        // op (the fillet's single-op face order), the second of every chain
        // in the next, and so on; blend names stay selection-indexed.
        let fwd: Vec<(String, String, Vec3)> = edges.iter().map(|(fa, fb, at)| forward_first(b, fa, fb, *at)).collect();
        let ends: Vec<[usize; 2]> = fwd
            .iter()
            .map(|(fa, fb, at)| {
                let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
                Ok([b.edge(e).v0.idx(), b.edge(e).v1.idx()])
            })
            .collect::<Result<_, BlendError>>()?;
        let n = edges.len();
        let mut comp: Vec<usize> = (0..n).collect();
        for i in 0..n {
            for j in 0..i {
                if ends[i].iter().any(|v| ends[j].contains(v)) {
                    let (ci, cj) = (comp[i], comp[j]);
                    for c in comp.iter_mut() {
                        if *c == ci {
                            *c = cj;
                        }
                    }
                }
            }
        }
        let level: Vec<usize> = (0..n).map(|i| (0..i).filter(|&j| comp[j] == comp[i]).count()).collect();
        let mut out = b.clone();
        for lv in 0..=*level.iter().max().unwrap() {
            let idx: Vec<u32> = (0..n).filter(|&i| level[i] == lv).map(|i| i as u32).collect();
            let sel: Vec<(String, String, Vec3)> = idx.iter().map(|&i| fwd[i as usize].clone()).collect();
            out = blend_edges_at(&out, &sel, spec, feature, tol, &idx)?;
        }
        return Ok(out);
    }
    let idx: Vec<u32> = (0..edges.len() as u32).collect();
    blend_edges_at(b, edges, spec, feature, tol, &idx)
}

/// One blend op over `edges`; `idx[k]` is the blend number (`B<idx>`) of edge k.
fn blend_edges_at(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64, idx: &[u32]) -> Result<Body, BlendError> {
    let edges: Vec<(String, String, Vec3)> = edges.iter().map(|(fa, fb, at)| forward_first(b, fa, fb, *at)).collect();
    // blend references on the INPUT body (for the in-place numbering)
    let mut refs: Vec<BlendRef> = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        let ed = b.edge(e);
        refs.push(BlendRef {
            name: FaceName::blend(feature, idx[k]).id(),
            faces: [fa.clone(), fb.clone()],
            ends: [b.vertex(ed.v0).p, b.vertex(ed.v1).p],
        });
    }
    let mut out = b.clone();
    let mut touched: Vec<String> = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(&out, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        let (f1, f2) = (out.find_face(fa).unwrap(), out.find_face(fb).unwrap());
        let (body, hint) = blend_edge_hint(&out, e, [f1, f2], spec, feature, idx[k], tol)?;
        for h in hint {
            if !touched.contains(&h) {
                touched.push(h);
            }
        }
        out = body;
    }
    if edges.len() == 1 {
        return Ok(in_place_layout(b, &out, &refs, tol));
    }
    // one blend op over all edges: untouched faces, touched faces in
    // first-touch order, blend faces (probe fil2)
    let base = |id: &str| FaceName::parse(id).map(|mut n| {
        n.region = None;
        n.id()
    });
    let rank = |f: FaceId| -> (usize, usize) {
        let name = &out.face(f).name;
        if name.feature == feature && matches!(name.role, Role::Blend(_)) {
            return (2, 0);
        }
        match touched.iter().position(|t| base(t) == base(&name.id())) {
            Some(p) => (1, p),
            None => (0, 0),
        }
    };
    let mut order: Vec<FaceId> = out.face_ids().collect();
    order.sort_by_key(|&f| rank(f)); // stable: keeps relative order within ties
    let out = crate::boolean::assemble::rebuild_in_order(&out, &order);
    Ok(in_place_layout(b, &out, &refs, tol))
}

/// `(fa, fb)` with the face whose coedge runs the selected edge forward
/// first (unchanged when the edge is not found).
pub fn forward_first(b: &Body, fa: &str, fb: &str, at: Vec3) -> (String, String, Vec3) {
    let keep = (fa.to_string(), fb.to_string(), at);
    let Some(e) = find_edge(b, fa, fb, at) else { return keep };
    let fwd = b.edge_coedges(e).into_iter().find(|&c| !b.coedge(c).rev).map(|c| b.coedge_face(c));
    match (fwd, b.find_face(fb)) {
        (Some(f), Some(g)) if f == g => (fb.to_string(), fa.to_string(), at),
        _ => keep,
    }
}

/// One selected blend: its face id, the edge's two faces (forward face
/// first) and end points (v0, v1) on the input body.
#[derive(Debug, Clone)]
pub struct BlendRef {
    pub name: String,
    pub faces: [String; 2],
    pub ends: [Vec3; 2],
}

/// Re-lay `out` (the blended body) in the original's in-place arena, so
/// [`Body::wire_numbered`] gives the oracle `#n` (bracket F11 / F12):
/// - an input edge that survives unchanged keeps its slot;
/// - an input edge the blend trims is split in parameter order (in
///   ascending input slot order): its first piece keeps the slot, each
///   further piece takes the next new slot; pieces the blend removed stay
///   as dead slots (F11: C0|S.e9 #2 trimmed at its end keeps #2, C0|S.e8
///   #1 trimmed at its start dies and its kept piece is #76, S.e10|S.e9 #6
///   trimmed at both ends -> #80 kept, #81 dead);
/// - removed input edges stay dead slots;
/// - then per blend (selection order) its edges: spring on the forward
///   face, spring on the other face, end arc at v0, end arc at v1.
pub fn in_place_layout(input: &Body, out: &Body, refs: &[BlendRef], tol: f64) -> Body {
    let (in_slots, n_in) = input.wire_numbered();
    let out_chains = out.wire_chains();
    let names = |b: &Body, e: EdgeId| {
        let mut v: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
        v.sort();
        v
    };
    let ends = |b: &Body, ch: &[EdgeId]| (b.vertex(b.edge(ch[0]).v0).p, b.vertex(b.edge(*ch.last().unwrap()).v1).p);
    let near = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let on = |b: &Body, ch: &[EdgeId], p: Vec3| {
        ch.iter().any(|&e| {
            let ed = b.edge(e);
            crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= tol * 10.0
        })
    };
    let mut slot_of: Vec<Option<usize>> = vec![None; out_chains.len()];
    let mut used_in = vec![false; in_slots.len()];
    // 1. unchanged edges
    for (k, ch) in out_chains.iter().enumerate() {
        let (p, q) = ends(out, ch);
        let nm = names(out, ch[0]);
        let mid = out.chain_mid(ch);
        for (i, (s, ich)) in in_slots.iter().enumerate() {
            if used_in[i] || names(input, ich[0]) != nm {
                continue;
            }
            let (a, b2) = ends(input, ich);
            if ((near(a, p) && near(b2, q)) || (near(a, q) && near(b2, p))) && near(input.chain_mid(ich), mid) {
                slot_of[k] = Some(*s);
                used_in[i] = true;
                break;
            }
        }
    }
    // 2. trimmed edges, split in parameter order
    let mut next = n_in;
    for (i, (s, ich)) in in_slots.iter().enumerate() {
        if used_in[i] || ich.len() != 1 {
            continue;
        }
        let ie = input.edge(ich[0]);
        let nm = names(input, ich[0]);
        let pieces: Vec<usize> = (0..out_chains.len())
            .filter(|&k| slot_of[k].is_none() && names(out, out_chains[k][0]) == nm && {
                let (p, q) = ends(out, &out_chains[k]);
                on(input, ich, p) && on(input, ich, q) && on(input, ich, out.chain_mid(&out_chains[k]))
            })
            .collect();
        if pieces.is_empty() {
            continue;
        }
        let param = |p: Vec3| crate::boolean::imprint::dist_in_range(&ie.curve, (ie.t0, ie.t1), p).0;
        let mut cuts: Vec<f64> = vec![];
        for &k in &pieces {
            let (p, q) = ends(out, &out_chains[k]);
            for t in [param(p), param(q)] {
                let slack = crate::intersect::param_slack(&ie.curve, t, tol * 10.0);
                if t > ie.t0 + slack && t < ie.t1 - slack && !cuts.iter().any(|&c| (c - t).abs() <= slack) {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(|a, b| a.total_cmp(b));
        let mut bounds = vec![ie.t0];
        bounds.extend(cuts);
        bounds.push(ie.t1);
        for (j, w) in bounds.windows(2).enumerate() {
            let slot = if j == 0 { *s } else {
                next += 1;
                next - 1
            };
            let tm = 0.5 * (w[0] + w[1]);
            if let Some(&k) = pieces.iter().find(|&&k| {
                let t = param(out.chain_mid(&out_chains[k]));
                t > w[0] && t < w[1] && slot_of[k].is_none()
            }) {
                let _ = tm;
                slot_of[k] = Some(slot);
            }
        }
        used_in[i] = true;
    }
    // 3. blend edges per blend
    for r in refs {
        let mut mine: Vec<(usize, usize, f64)> = vec![];
        for (k, ch) in out_chains.iter().enumerate() {
            if slot_of[k].is_some() {
                continue;
            }
            let nm = names(out, ch[0]);
            if !nm.contains(&r.name) {
                continue;
            }
            let other = nm.iter().find(|n| **n != r.name).cloned().unwrap_or_default();
            let (p, q) = ends(out, ch);
            let d0 = p.dist(r.ends[0]).min(q.dist(r.ends[0]));
            let d1 = p.dist(r.ends[1]).min(q.dist(r.ends[1]));
            let rank = if other == r.faces[0] {
                0
            } else if other == r.faces[1] {
                1
            } else if d0 <= d1 {
                2
            } else {
                3
            };
            mine.push((rank, k, d0.min(d1)));
        }
        mine.sort_by(|a, b| a.0.cmp(&b.0).then(a.2.total_cmp(&b.2)));
        for (_, k, _) in mine {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    // 4. anything else, in out order
    for k in 0..out_chains.len() {
        if slot_of[k].is_none() {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    // ---- write the arena: slot order, dead placeholders, seams last
    let mut res = Body::new();
    let lump = res.add_lump();
    let shell = res.add_shell(lump);
    let mut vmap: Vec<Option<ck_topo::body::VertexId>> = vec![None; out.vertices.len()];
    for v in out.vertex_ids() {
        vmap[v.idx()] = Some(res.add_vertex(out.vertex(v).p));
    }
    let mut by_slot: Vec<Option<usize>> = vec![None; next];
    for (k, s) in slot_of.iter().enumerate() {
        by_slot[s.unwrap()] = Some(k);
    }
    let mut emap: Vec<Option<EdgeId>> = vec![None; out.edges.len()];
    let dummy = res.add_vertex(Vec3::ZERO);
    res.vertices[dummy.idx()].alive = false;
    for s in 0..next {
        match by_slot[s] {
            Some(k) => {
                for &e in &out_chains[k] {
                    let ed = out.edge(e);
                    emap[e.idx()] =
                        Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
                }
            }
            None => {
                let d = res.add_edge(Curve::Line { o: Vec3::ZERO, d: Vec3::X }, 0.0, 0.0, dummy, dummy);
                res.edges[d.idx()].alive = false;
            }
        }
    }
    for e in out.edge_ids() {
        if emap[e.idx()].is_none() {
            let ed = out.edge(e);
            emap[e.idx()] = Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
        }
    }
    for f in out.face_ids() {
        let face = out.face(f);
        let nf = res.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
        for &l in &face.loops {
            let nl = res.add_loop(nf);
            for &c in &out.lp(l).coedges {
                let ce = out.coedge(c).clone();
                res.add_coedge(nl, emap[ce.edge.idx()].unwrap(), ce.rev, ce.pcurve);
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut res);
    res
}

#[cfg(test)]
fn dbg_fail(m: &str) {
    eprintln!("blend fail: {m}");
}
#[cfg(not(test))]
fn dbg_fail(_: &str) {}

/// One straight edge between two planar faces.
pub fn fillet_line_edge(b: &Body, e: EdgeId, faces: [FaceId; 2], r: f64, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    blend_edge_hint(b, e, faces, Spec::Fillet(r), feature, k, tol).map(|x| x.0)
}

/// As [`fillet_line_edge`], also returning the touched-face order hint
/// (edge faces, then end faces).
fn blend_edge_hint(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    spec: Spec,
    feature: &str,
    k: u32,
    tol: f64,
) -> Result<(Body, Vec<String>), BlendError> {
    let ed = b.edge(e).clone();
    let r = spec.size();
    // Geometry at the working point P0 (line: edge midpoint; circle: its
    // start vertex, which is also the faces' seam point).
    let (p0, p1, dir, n1, n2, circ): (Vec3, Vec3, Vec3, Vec3, Vec3, Option<(Vec3, Vec3)>) = match ed.curve {
        Curve::Line { .. } => {
            for f in faces {
                if !matches!(b.face(f).surface, Surface::Plane { .. }) {
                    return Err(BlendError::code("blendUnsupported"));
                }
            }
            let p0 = b.vertex(ed.v0).p;
            let p1 = b.vertex(ed.v1).p;
            let dir = (p1 - p0) / p0.dist(p1);
            (p0, p1, dir, b.planar_normal(faces[0]), b.planar_normal(faces[1]), None)
        }
        _ => {
            // circle between coaxial plane / cylinder / cone faces
            let Some((c, axis, rad)) = ed.curve.circle_info() else { return Err(BlendError::code("blendUnsupported")) };
            let axis = axis.normalize();
            let p0 = ed.curve.eval(ed.t0);
            let coaxial = |f: FaceId| match &b.face(f).surface {
                Surface::Plane { frame } => frame.z.cross(axis).len() < 1e-9,
                Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => {
                    frame.z.cross(axis).len() < 1e-9 && (frame.o - c).cross(axis).len() < 1e-7 * (1.0 + rad)
                }
                _ => false,
            };
            if !coaxial(faces[0]) || !coaxial(faces[1]) || !(rad > 0.0) {
                return Err(BlendError::code("blendUnsupported"));
            }
            let nrm = |f: FaceId| {
                let (u, v) = b.face(f).surface.closest_param(p0);
                b.outward_normal(f, u, v).normalize()
            };
            let dir = ed.curve.tangent(ed.t0).normalize();
            (p0, p0, dir, nrm(faces[0]), nrm(faces[1]), Some((c, axis)))
        }
    };
    let len = p0.dist(p1);
    let c12 = n1.dot(n2);
    if (1.0 + c12).abs() < 1e-9 || (1.0 - c12).abs() < 1e-9 {
        return Err(BlendError::code("blendUnsupported")); // tangent / folded faces
    }
    // convex iff the direction into F1 (left of its coedge about the
    // outward normal) points against n2
    let pm = if circ.is_some() { p0 } else { (p0 + p1) * 0.5 };
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]).ok_or_else(|| BlendError::code("missingEdge"))?;
    let tdir = if b.coedge(co).rev { -dir } else { dir };
    let convex = n1.cross(tdir).dot(n2) < 0.0;
    let s = if convex { -1.0 } else { 1.0 };
    // contact points on the two faces, the ball centre (fillet) and the
    // chamfer legs (in-face directions away from the edge: n x coedge dir)
    let off = (n1 + n2) * (s * r / (1.0 + c12));
    let (ta, tb) = match spec {
        Spec::Fillet(_) => (pm + off - n1 * (s * r), pm + off - n2 * (s * r)),
        Spec::Chamfer(_) => (pm + n1.cross(tdir).normalize() * r, pm - n2.cross(tdir).normalize() * r),
    };
    // contact (spring) lines must stay on their faces
    let name = FaceName::blend(feature, k);
    for (f, t) in [(faces[0], ta), (faces[1], tb)] {
        if point_in_face(b, f, t, tol) == Where::Out {
            return Err(BlendError::detail("blendOverflow", &format!("{}: spring leaves its face away from its edges", name.id())));
        }
    }
    // tool profile in the plane perpendicular to dir through p0 - dir*margin
    // (line) or in the meridian plane through P0 (circle: revolved about the
    // axis). convex: the tool pierces the end faces (the cut trims it);
    // concave: GUESS end faces perpendicular to the edge, the tool ends
    // flush with them
    let margin = if convex && circ.is_none() { r.max(len * 0.05) + 1.0 } else { 0.0 };
    let o = p0 - dir * margin;
    let fr = match circ {
        None => Frame::from_normal(o, dir),
        Some((c, axis)) => {
            let rad = (p0 - c).normalize();
            ck_geom::frame_from(c, rad.cross(axis), rad)
        }
    };
    let uv = |p: Vec3| {
        let q = fr.world_to_uv(p);
        Vec2::new(q.x, q.y)
    };
    let (cp, t1, t2) = (uv(p0), uv(ta), uv(tb));
    let cc = uv(p0 + off);
    let lp = if matches!(spec, Spec::Chamfer(_)) {
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), ProfileSeg::line("arc", t1, t2)])
    } else {
        let short_ccw = (t1 - cc).cross(t2 - cc) > 0.0;
        let arc = if short_ccw {
            ProfileSeg::arc("arc", cc, t1, t2)
        } else if !convex {
            // concave: the tool's cap arcs ARE the end arcs; the oracle runs
            // them from the forward face's spring to the other one, n = (s1 -
            // c) x (s2 - c) (bracket F12 on S.e8|S.e9 along +x: n = -x)
            let x = (t1 - cc).normalize();
            let (a, b) = (t1 - cc, t2 - cc);
            let sweep = -(a.cross(b)).atan2(a.dot(b));
            ProfileSeg { tag: "arc".into(), curve: ck_geom::curve2::Curve2::Circle { c: cc, x, r: cc.dist(t1), ccw: false }, t0: 0.0, t1: sweep, rev: false }
        } else {
            ProfileSeg::arc("arc", cc, t2, t1).reversed()
        };
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), arc])
    };
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]).normalized();
    let made = match circ {
        None => extrude(feature, &prof, ExtrudeOpts { s0: 0.0, s1: len + 2.0 * margin, dir: None, draft: 0.0 }),
        Some((c, axis)) => ck_topo::make::revolve::revolve(feature, &prof, c, axis, 0.0, std::f64::consts::TAU),
    };
    let mut tool = made.map_err(|e| {
        dbg_fail(&format!("tool: {e}"));
        BlendError::code("blendFailed")
    })?;
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
    let op = if convex { Op::Cut } else { Op::Union };
    let arr = imprint(b, &tool, tol);
    let mut faces_out = select(&arr, [b, &tool], op).map_err(|e| {
        dbg_fail(&format!("select: {e:?}"));
        BlendError::code("blendFailed")
    })?;
    // order: untouched; edge faces (ref order), end faces (start, end);
    // other touched faces; blend
    let ends: Vec<FaceId> = [ed.v0, ed.v1]
        .iter()
        .flat_map(|&v| {
            b.edge_ids()
                .filter(move |&x| b.edge(x).v0 == v || b.edge(x).v1 == v)
                .flat_map(|x| b.edge_faces(x))
                .filter(|f| !faces.contains(f))
                .collect::<Vec<_>>()
        })
        .collect();
    let mut hint: Vec<String> = faces.iter().map(|&f| b.face_id_str(f)).collect();
    for f in ends {
        let id = b.face_id_str(f);
        if !hint.contains(&id) {
            hint.push(id);
        }
    }
    for f in &mut faces_out {
        if f.group == 1 {
            let mut base = f.name.clone();
            base.region = None;
            let pos = hint.iter().position(|h| FaceName::parse(h).map(|mut x| {
                x.region = None;
                x
            }) == Some(base.clone()));
            f.key = (1, pos.unwrap_or(hint.len()), f.key.1 * 1000 + f.key.2);
        } else if f.group == 4 {
            f.key = (1, hint.len() + 1, f.key.1);
        }
        // walk items follow the face key (edge numbering = face-walk order)
        let key = f.key;
        for w in &mut f.walk {
            w.0 = key;
        }
    }
    faces_out.sort_by_key(|f| f.key);
    if !faces_out.iter().any(|f| f.name == name) {
        dbg_fail("no blend face in result");
        return Err(BlendError::code("blendFailed"));
    }
    let body = build(&arr, &faces_out);
    let bad = ck_topo::check::check(&body, tol)
        .into_iter()
        .any(|i| matches!(i, ck_topo::check::Issue::OpenEdge(_) | ck_topo::check::Issue::BrokenLoop(..)));
    if bad {
        dbg_fail(&format!("check: {:?}", ck_topo::check::check(&body, tol)));
        return Err(BlendError::code("blendFailed"));
    }
    Ok((body, hint))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    fn the_box() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }
    fn ids(b: &Body) -> Vec<String> {
        b.face_ids().map(|f| b.face_id_str(f)).collect()
    }
    fn mass(b: &Body, vol: f64, area: f64, com: [f64; 3]) {
        let m = mass_props(b);
        assert!((m.volume - vol).abs() < 1e-9 * vol, "{} vs {vol}", m.volume);
        assert!((m.area - area).abs() < 1e-9 * area, "{} vs {area}", m.area);
        assert!((m.com - Vec3::new(com[0], com[1], com[2])).len() < 1e-8, "{:?}", m.com);
    }
    fn e(a: &str, b: &str, at: [f64; 3]) -> (String, String, Vec3) {
        (a.into(), b.into(), Vec3::new(at[0], at[1], at[2]))
    }

    #[test]
    fn oracle_fillet_one_edge() {
        let r = fillet(&the_box(), &[e("F2.C1", "F2.S.e1", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert!(ck_topo::check::check(&r, 1e-6).is_empty());
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e3", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F3.B0"]);
        mass(&r, 7965.6637061435895, 2763.946891450771, [20.000000000000004, 4.980372983260395, -10.041179705330975]);
        let b0 = r.find_face("F3.B0").unwrap();
        assert_eq!(r.face(b0).surface.kind(), "cylinder");
        // end-face arcs: c = (0,8,-2) / (40,8,-2), r = 2
        let arcs: Vec<(Vec3, f64)> = r
            .face_edges(b0)
            .into_iter()
            .filter_map(|x| r.edge(x).curve.circle_info().map(|(c, _, rr)| (c, rr)))
            .collect();
        assert_eq!(arcs.len(), 2);
        for (c, rr) in arcs {
            assert!((rr - 2.0).abs() < 1e-12);
            assert!((c.y - 8.0).abs() < 1e-12 && (c.z + 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn oracle_fillet_two_edges_and_vertical() {
        let r = fillet(
            &the_box(),
            &[e("F2.C1", "F2.S.e1", [20., 10., 0.]), e("F2.C1", "F2.S.e3", [20., 10., -20.])],
            2.0,
            "F3",
            1e-6,
        )
        .unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F2.S.e3", "F3.B0", "F3.B1"]);
        mass(&r, 7931.327412287184, 2727.8937829015417, [20., 4.960576028002486, -10.]);
        let r = fillet(&the_box(), &[e("F2.S.e1", "F2.S.e2", [40., 5., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.S.e3", "F2.S.e4", "F2.S.e1", "F2.S.e2", "F2.C0", "F2.C1", "F3.B0"]);
        mass(&r, 7991.4159265358985, 2789.6991118430774, [19.978996631231546, 5., -10.010261751083915]);
    }

    #[test]
    fn cylinder_top_circle() {
        let cyl = crate::boolean::tests::cyl_body("F2", 5.0, 3.0, 10.0, 0.0, 10.0);
        let top = cyl.face_ids().find(|&f| cyl.face_id_str(f) == "F2.C1").unwrap();
        let side = cyl.face_ids().find(|&f| matches!(cyl.face(f).surface, Surface::Cylinder { .. })).unwrap();
        let (a, bb) = (cyl.face_id_str(top), cyl.face_id_str(side));
        let at = Vec3::new(15., 10., -3.);
        let v0 = mass_props(&cyl).volume;
        let r = fillet(&cyl, &[(a.clone(), bb.clone(), at)], 2.0, "F3", 1e-6).unwrap();
        let pi = std::f64::consts::PI;
        // torus ring removed: 2*pi*(R - r*(4-pi)/(3*(4-pi)... ) simply compare to oracle
        assert!((mass_props(&r).volume - 3090.0668137679036).abs() < 1e-6, "{} {}", mass_props(&r).volume, v0);
        let _ = pi;
        for e in r.edge_ids() {
            let ed = r.edge(e);
            eprintln!("EDGE {:?} {} t=({:?},{:?}) v={:?}", r.edge_faces(e).iter().map(|&f| r.face_id_str(f)).collect::<Vec<_>>(), ed.curve.kind(), ed.t0, ed.t1, r.vertex(ed.v0).p);
        }
        let r = chamfer(&cyl, &[(a, bb, at)], 1.0, "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - 3111.223924605093).abs() < 1e-6);
    }

    #[test]
    fn oracle_fillet_errors() {
        let b = the_box();
        let one = [e("F2.C1", "F2.S.e1", [20., 10., 0.])];
        assert_eq!(fillet(&b, &one, 0.0, "F3", 1e-6).unwrap_err(), BlendError::code("invalidValue"));
        assert_eq!(fillet(&b, &[], 2.0, "F3", 1e-6).unwrap_err(), BlendError::code("missingEdge"));
        assert_eq!(
            fillet(&b, &[e("F2.C1", "F2.S.e9", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap_err(),
            BlendError::code("missingEdge")
        );
        assert_eq!(
            fillet(&b, &one, 30.0, "F3", 1e-6).unwrap_err(),
            BlendError::detail("blendOverflow", "F3.B0: spring leaves its face away from its edges")
        );
    }

    #[test]
    fn concave_fillet_adds_material() {
        // L-shape: box with a step cut → concave edge at the step
        let a = the_box();
        let tool = box_body("F4", 0.0, 0.0, 20.0, 20.0, 5.0, 11.0);
        let l = crate::boolean::boolean(&a, &tool, Op::Cut, 1e-6).unwrap().body;
        let v0 = mass_props(&l).volume;
        // concave edge between the step floor (y=5) and the step wall (x=20)
        let floor = l.face_ids().find(|&f| l.face_id_str(f).starts_with("F4.C0")).unwrap();
        let wall = l.face_ids().find(|&f| l.face_id_str(f) == "F4.S.e2").unwrap();
        let (fa, fb) = (l.face_id_str(floor), l.face_id_str(wall));
        let r = fillet(&l, &[(fa, fb, Vec3::new(20., 5., -10.))], 1.0, "F5", 1e-6).unwrap();
        let v = mass_props(&r).volume;
        let add = (1.0 - std::f64::consts::PI / 4.0) * 20.0;
        assert!((v - v0 - add).abs() < 1e-9 * v, "{v} {v0}");
    }
}
