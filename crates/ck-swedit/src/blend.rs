//! Fillets and chamfers read back from their strip faces.
//!
//! Every strip face (a cylinder or torus for a fillet, a plane or cone for a chamfer) lies between two support
//! faces: for a fillet the two neighbours it is tangent to, for a chamfer the two neighbours it shares its longest
//! edges with. The sharp edge the supports would meet at is reconstructed geometrically (the pick point `at` of our
//! edge selection), the supports are named by the faces' names in the rebuilt document; `ck-ops` resolves the pick
//! by the supports' features and the point even when the names shifted.

use crate::{dim_note, Cx};
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, s, vec3};
use ck_recog::model::{Kind, Model};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{EdgeId, FaceId};
use std::collections::HashSet;

/// The two faces lie on one surface (kind and parameters).
pub(crate) fn same_surface(x: &Kind, y: &Kind) -> bool {
    let c = |a: f64, b: f64| (a - b).abs() < 1e-6 * (1.0 + a.abs());
    match (x, y) {
        (Kind::Plane { n: n1, d: d1 }, Kind::Plane { n: n2, d: d2 }) => (*n1 - *n2).len() < 1e-7 && c(*d1, *d2),
        (Kind::Cyl { o: o1, a: a1, r: r1, .. }, Kind::Cyl { o: o2, a: a2, r: r2, .. }) => a1.cross(*a2).len() < 1e-7 && (*o2 - *o1).cross(*a1).len() < 1e-5 * (1.0 + r1) && c(*r1, *r2),
        (Kind::Cone { o: o1, a: a1, half: h1, .. }, Kind::Cone { o: o2, a: a2, half: h2, .. }) => a1.cross(*a2).len() < 1e-7 && (*o2 - *o1).cross(*a1).len() < 1e-5 && c(h1.abs(), h2.abs()),
        (Kind::Sphere { c: c1, r: r1, .. }, Kind::Sphere { c: c2, r: r2, .. }) => (*c1 - *c2).len() < 1e-5 && c(*r1, *r2),
        (Kind::Torus { c: c1, a: a1, major: m1, minor: n1, .. }, Kind::Torus { c: c2, a: a2, major: m2, minor: n2, .. }) => {
            (*c1 - *c2).len() < 1e-5 && a1.cross(*a2).len() < 1e-7 && c(*m1, *m2) && c(*n1, *n2)
        }
        _ => false,
    }
}

/// Outward normal of face `f` at the point of it nearest `p`.
fn normal_near(m: &Model, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = m.b.face(f).surface.closest_param(p);
    m.b.outward_normal(f, u, v)
}

fn point_near(m: &Model, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = m.b.face(f).surface.closest_param(p);
    m.b.face(f).surface.eval(u, v)
}

/// The two support faces of a strip: neighbours outside `own` ranked by `score` (sum over shared edges). A neighbour
/// a later feature made (no name in the rebuilt document yet: a later blend that trimmed the strip, a cut through
/// it) is no support, unless that leaves fewer than two.
fn supports(cx: &Cx, f: FaceId, own: &HashSet<FaceId>, score: impl Fn(EdgeId, FaceId) -> f64) -> Vec<(FaceId, EdgeId, f64)> {
    let all = supports_of(cx.m, f, own, &score, &|_| true);
    let known = supports_of(cx.m, f, own, &score, &|g| cx.lookup(g).is_some());
    if known.len() >= 2 || known.len() >= all.len() {
        known
    } else {
        all
    }
}

fn supports_of(m: &Model, f: FaceId, own: &HashSet<FaceId>, score: &dyn Fn(EdgeId, FaceId) -> f64, keep: &dyn Fn(FaceId) -> bool) -> Vec<(FaceId, EdgeId, f64)> {
    let mut acc: Vec<(FaceId, EdgeId, f64)> = vec![];
    for e in m.edges(f) {
        let Some(g) = m.across(e, f) else { continue };
        if own.contains(&g) || !keep(g) {
            continue;
        }
        let sc = score(e, g);
        if sc <= 0.0 {
            continue;
        }
        match acc.iter_mut().find(|a| a.0 == g) {
            Some(a) => {
                a.2 += sc;
                if m.b.edge_length(e) > m.b.edge_length(a.1) {
                    a.1 = e;
                }
            }
            None => acc.push((g, e, sc)),
        }
    }
    acc.sort_by(|x, y| y.2.partial_cmp(&x.2).unwrap());
    acc
}

/// Point of the sharp edge between planes `a` and `b` nearest `g`.
fn plane_plane_corner(m: &Model, a: FaceId, b: FaceId, g: Vec3) -> Option<Vec3> {
    let (Kind::Plane { n: na, d: da }, Kind::Plane { n: nb, d: db }) = (m.kind(a).clone(), m.kind(b).clone()) else { return None };
    // g + s na + t nb on both planes
    let (aa, ab, bb) = (na.dot(na), na.dot(nb), nb.dot(nb));
    let det = aa * bb - ab * ab;
    if det.abs() < 1e-9 {
        return None;
    }
    let (ra, rb) = (da - na.dot(g), db - nb.dot(g));
    let s_ = (ra * bb - rb * ab) / det;
    let t = (aa * rb - ab * ra) / det;
    Some(g + na * s_ + nb * t)
}

/// Point of the sharp edge between a plane and a cylinder (a circle) nearest `g`.
fn plane_cyl_corner(m: &Model, a: FaceId, b: FaceId, g: Vec3) -> Option<Vec3> {
    let (plane, cyl) = match (m.kind(a), m.kind(b)) {
        (Kind::Plane { .. }, Kind::Cyl { .. }) => (a, b),
        (Kind::Cyl { .. }, Kind::Plane { .. }) => (b, a),
        _ => return None,
    };
    let (Kind::Plane { n: np, d: dp }, Kind::Cyl { o, a: ax, r, .. }) = (m.kind(plane).clone(), m.kind(cyl).clone()) else { return None };
    let p = g - np * (np.dot(g) - dp);
    let foot = o + ax * (p - o).dot(ax);
    let v = p - foot;
    if v.len() < 1e-9 {
        return None;
    }
    Some(foot + v.normalize() * r)
}

struct Pick {
    a: FaceId,
    b: FaceId,
    at: Vec3,
    /// Radius, or the chamfer distance on `a`.
    param: f64,
    /// Chamfer distance on `b` (equal to `param` for a fillet).
    param2: f64,
}

/// The pick of a spline strip tangent to two plane / cylinder supports that are not strips of this feature (a fillet
/// along the groin of two cylinders): the radius is that of the round strips and corner spheres next to it; `None`
/// when the patch is a corner of the strips beside it.
fn spline_pick(cx: &Cx, f: FaceId, own: &HashSet<FaceId>) -> Result<Option<Pick>, String> {
    let m = cx.m;
    let tangent = |e: EdgeId, g: FaceId| {
        let p = m.mid(e);
        let (nf, ng) = (normal_near(m, f, p), normal_near(m, g, p));
        // (the spline of an imported file may carry the opposite orientation: the direction is not compared)
        nf.cross(ng).len() < 3e-3
    };
    let sup = supports(cx, f, own, |e, g| if tangent(e, g) { m.b.edge_length(e) } else { 0.0 });
    let round = |g: FaceId| matches!(m.kind(g), Kind::Plane { .. } | Kind::Cyl { .. });
    if sup.len() != 2 || !sup.iter().all(|x| round(x.0)) {
        return Ok(None);
    }
    let r = m.edges(f).into_iter().filter_map(|e| m.across(e, f)).filter(|g| own.contains(g)).find_map(|g| match m.kind(g) {
        Kind::Cyl { r, .. } | Kind::Sphere { r, .. } => Some(*r),
        Kind::Torus { minor, .. } => Some(*minor),
        _ => None,
    });
    let Some(r) = r else { return Ok(None) };
    let (a, b) = (sup[0].0, sup[1].0);
    // the ball centre off a rail point, then the nearest point of both supports (alternating projections)
    let pa = m.mid(sup[0].1);
    let c = pa + normal_near(m, a, pa) * r;
    let mut q = c;
    for _ in 0..40 {
        q = point_near(m, a, q);
        q = point_near(m, b, q);
    }
    let tol = 1e-6 * (1.0 + q.len());
    if point_near(m, a, q).dist(q) > tol || point_near(m, b, q).dist(q) > tol {
        return Ok(None);
    }
    Ok(Some(Pick { a, b, at: q, param: r, param2: r }))
}

fn fillet_pick(cx: &Cx, f: FaceId, own: &HashSet<FaceId>) -> Result<Option<Pick>, String> {
    let m = cx.m;
    let (r, centre): (f64, Box<dyn Fn(Vec3) -> Vec3>) = match m.kind(f).clone() {
        Kind::Cyl { o, a, r, .. } => (r, Box::new(move |p: Vec3| o + a * (p - o).dot(a))),
        Kind::Torus { c, a, major, minor, .. } => (
            minor,
            Box::new(move |p: Vec3| {
                let q = p - a * (p - c).dot(a);
                let v = q - c;
                if v.len() < 1e-9 {
                    c
                } else {
                    c + v.normalize() * major
                }
            }),
        ),
        Kind::Sphere { .. } => return Ok(None),
        // a blend between two non-coaxial rounds is a spline patch: a corner of the strips next to it. A spline strip
        // that rests on two supports of its own (the groin where two round rods meet) is a fillet edge itself
        Kind::Other(_) if matches!(m.b.face(f).surface, ck_geom::surface::Surface::BSpline(_)) => return spline_pick(cx, f, own),
        k => return Err(format!("fillet face is a {}", k.name())),
    };
    // supports: the neighbours the strip is tangent to along their shared edge
    let sup = supports(cx, f, own, |e, g| {
        let p = m.mid(e);
        let (nf, ng) = (normal_near(m, f, p), normal_near(m, g, p));
        if nf.cross(ng).len() < 1e-3 && nf.dot(ng) > 0.0 {
            m.b.edge_length(e)
        } else {
            0.0
        }
    });
    if sup.len() >= 2 && matches!(m.kind(f), Kind::Torus { .. }) && sup.iter().take(2).any(|x| !cx.lookup(x.0).is_some()) {
        // a torus between a support and a face a later feature made (a corner the later blend rounds) is a corner
        // patch of the strips next to it, which carry the edge picks
        return Ok(None);
    }
    if sup.len() < 2 && matches!(m.kind(f), Kind::Torus { .. }) && !sup.is_empty() {
        // a torus resting on one support, its other tangent neighbours all strips of this same fillet (a round of the
        // rim of a concave edge round: the vertex where a concave round meets the rounds of the faces it ends on) is
        // a corner patch of those strips
        let tangent_own = |g: FaceId| {
            m.edges(f).into_iter().any(|e| {
                m.across(e, f) == Some(g) && {
                    let p = m.mid(e);
                    let (nf, ng) = (normal_near(m, f, p), normal_near(m, g, p));
                    nf.cross(ng).len() < 1e-3 && nf.dot(ng) > 0.0
                }
            })
        };
        let others = m.edges(f).into_iter().filter_map(|e| m.across(e, f)).filter(|g| *g != sup[0].0);
        if others.clone().all(|g| own.contains(&g) && tangent_own(g)) {
            return Ok(None);
        }
    }
    if sup.len() < 2 {
        return Err(format!(
            "fillet strip with {} tangent support(s) (the other support is gone from the final body: the round overflowed onto a face it is not tangent to, or a later feature consumed the support)",
            sup.len()
        ));
    }
    let (a, ea) = (sup[0].0, sup[0].1);
    let b = sup[1].0;
    let pa = m.mid(ea);
    let c = centre(pa);
    let pb = point_near(m, b, c);
    let (na, nb) = (normal_near(m, a, pa), normal_near(m, b, pb));
    let k = 1.0 + na.dot(nb);
    if k < 0.05 {
        return Err("fillet between nearly tangent supports".into());
    }
    // the sharp edge nearest the ball's centre: on both support surfaces (plane / plane, plane / cylinder), else
    // from the normals
    let on_both = |q: Vec3| point_near(m, a, q).dist(q) < 1e-6 * (1.0 + q.len()) && point_near(m, b, q).dist(q) < 1e-6 * (1.0 + q.len());
    let at = plane_plane_corner(m, a, b, c).or_else(|| plane_cyl_corner(m, a, b, c)).filter(|&q| on_both(q)).unwrap_or(c - (na + nb) * (r / k));
    Ok(Some(Pick { a, b, at, param: r, param2: r }))
}

/// The supports of a planar chamfer strip: two plane neighbours whose intersection line runs in the strip's plane,
/// both meeting the strip along edges parallel to that line (the rails); the pair with the longest rails. A cap
/// across the strip's end (a plane the strip meets at its short edges) never qualifies.
fn plane_rails(cx: &Cx, f: FaceId, own: &HashSet<FaceId>) -> Vec<(FaceId, EdgeId, FaceId, EdgeId)> {
    let m = cx.m;
    let Kind::Plane { n: ns, .. } = m.kind(f).clone() else { return vec![] };
    // neighbours (known faces first, as for the other supports) with their line edges on the strip
    let mut nb: Vec<(FaceId, Vec3, Vec<(EdgeId, Vec3)>)> = vec![];
    for e in m.edges(f) {
        let Some(g) = m.across(e, f) else { continue };
        if own.contains(&g) || !cx.lookup(g).is_some() {
            continue;
        }
        let Kind::Plane { n, .. } = m.kind(g).clone() else { continue };
        let (p0, p1) = (m.p0(e), m.p1(e));
        if !matches!(m.curve(e), ck_geom::curve::Curve::Line { .. }) || p0.dist(p1) < 1e-9 {
            continue;
        }
        let d = (p1 - p0).normalize();
        match nb.iter_mut().find(|x| x.0 == g) {
            Some(x) => x.2.push((e, d)),
            None => nb.push((g, n, vec![(e, d)])),
        }
    }
    let mut cands: Vec<(f64, FaceId, EdgeId, FaceId, EdgeId)> = vec![];
    for i in 0..nb.len() {
        for j in i + 1..nb.len() {
            let c = nb[i].1.cross(nb[j].1);
            if c.len() < 1e-6 {
                continue;
            }
            let dir = c.normalize();
            if dir.dot(ns).abs() > 1e-6 {
                continue;
            }
            let rails = |k: usize| -> Option<(f64, EdgeId)> {
                let rs: Vec<&(EdgeId, Vec3)> = nb[k].2.iter().filter(|(_, d)| d.cross(dir).len() < 1e-6).collect();
                let e = rs.iter().max_by(|x, y| m.b.edge_length(x.0).partial_cmp(&m.b.edge_length(y.0)).unwrap())?.0;
                Some((rs.iter().map(|x| m.b.edge_length(x.0)).sum(), e))
            };
            let (Some((la, ea)), Some((lb, eb))) = (rails(i), rails(j)) else { continue };
            cands.push((la + lb, nb[i].0, ea, nb[j].0, eb));
        }
    }
    cands.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
    cands.into_iter().map(|b| (b.1, b.2, b.3, b.4)).collect()
}

fn chamfer_pick(cx: &Cx, f: FaceId, own: &HashSet<FaceId>, want: Option<f64>) -> Result<Option<Pick>, String> {
    let m = cx.m;
    match m.kind(f) {
        Kind::Plane { .. } | Kind::Cone { .. } => {}
        k => return Err(format!("chamfer face is a {}", k.name())),
    }
    let dist_of = |a: FaceId, ea: EdgeId, b: FaceId, eb: EdgeId| (point_near(m, b, m.mid(ea)).dist(m.mid(ea)), point_near(m, a, m.mid(eb)).dist(m.mid(eb)));
    let rails = plane_rails(cx, f, own);
    // the longest rails, unless a later cut left a ledge of the body alongside the strip: a pair of supports at the
    // distance the file lists is then the one the chamfer was made between
    let mut pick = rails.first().copied();
    if let Some(want) = want {
        let hit = |(a, ea, b, eb): &(FaceId, EdgeId, FaceId, EdgeId)| {
            let (da, db) = dist_of(*a, *ea, *b, *eb);
            (da - want).abs() < 1e-4 * (1.0 + want) || (db - want).abs() < 1e-4 * (1.0 + want)
        };
        if !pick.as_ref().map_or(false, |p| hit(p)) {
            if let Some(p) = rails.iter().find(|p| hit(p)) {
                pick = Some(*p);
            }
        }
    }
    let (a, ea, b, eb) = match pick {
        Some(p) => p,
        None => {
            let sup = supports(cx, f, own, |e, _| m.b.edge_length(e));
            if sup.len() < 2 {
                return Err(format!("chamfer strip with {} support(s)", sup.len()));
            }
            (sup[0].0, sup[0].1, sup[1].0, sup[1].1)
        }
    };
    let g = m.mid(ea);
    let on_both = |q: Vec3| point_near(m, a, q).dist(q) < 1e-6 * (1.0 + q.len()) && point_near(m, b, q).dist(q) < 1e-6 * (1.0 + q.len());
    let at = plane_plane_corner(m, a, b, g)
        .or_else(|| plane_cyl_corner(m, a, b, g))
        .filter(|&q| on_both(q))
        .ok_or_else(|| format!("chamfer between a {} and a {} that do not meet near the strip", m.kind(a).name(), m.kind(b).name()))?;
    // equal-distance chamfer: each rail is as far from the other support's surface
    let (da, db) = (point_near(m, b, m.mid(ea)).dist(m.mid(ea)), point_near(m, a, m.mid(eb)).dist(m.mid(eb)));
    Ok(Some(Pick { a, b, at, param: da, param2: db }))
}

/// Read a fillet (`chamfer` false) or chamfer back from its strip faces and emit one feature per distinct size.
pub fn blend(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], chamfer: bool) -> Result<String, String> {
    let own: HashSet<FaceId> = faces.iter().copied().collect();
    let mut picks: Vec<Pick> = vec![];
    let mut strip: Vec<FaceId> = vec![]; // the strip face of every pick
    let mut corners = 0usize;
    for &g in faces {
        let p = if chamfer { chamfer_pick(cx, g, &own, crate::dim(cx, f, "D1"))? } else { fillet_pick(cx, g, &own)? };
        match p {
            Some(p) => {
                picks.push(p);
                strip.push(g);
            }
            None => corners += 1,
        }
    }
    if picks.is_empty() {
        return Err("no strip face".into());
    }
    // group by size (a chamfer with two distances: the first face of every pick carries `distance`)
    let same = |x: f64, y: f64| (x - y).abs() < 1e-6 * (1.0 + x.abs());
    let mut groups: Vec<(f64, f64, Vec<usize>)> = vec![];
    for (i, p) in picks.iter().enumerate() {
        match groups.iter_mut().find(|(r, r2, _)| same(*r, p.param) && same(*r2, p.param2)) {
            Some((_, _, v)) => v.push(i),
            None => groups.push((p.param, p.param2, vec![i])),
        }
    }
    // the supports as the document rebuilt so far names them: the faces of its edge at the pick point (a support
    // flush with another feature's face is one face of the rebuilt body, under one of the two names; a face a
    // boolean split carries a region suffix), else the names the read-back gave them
    let prefix = ck_recog::rebuild::rebuild(&cx.tree.feats).ok().and_then(|(b, _)| b);
    let pm = prefix.as_ref().map(Model::new);
    let rebuilt_names = |p: &Pick| -> Option<(String, String)> {
        let (pb, pm) = (prefix.as_ref()?, pm.as_ref()?);
        let tol = 1e-6 * (1.0 + p.at.len());
        for e in pb.edge_ids() {
            let fs = pb.edge_faces(e);
            if fs.len() != 2 {
                continue;
            }
            let ed = pb.edge(e);
            if ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p.at).1 > tol {
                continue;
            }
            let (sa, sb) = (same_surface(cx.m.kind(p.a), pm.kind(fs[0])), same_surface(cx.m.kind(p.b), pm.kind(fs[1])));
            let (ta, tb) = (same_surface(cx.m.kind(p.a), pm.kind(fs[1])), same_surface(cx.m.kind(p.b), pm.kind(fs[0])));
            if sa && sb {
                return Some((pb.face_id_str(fs[0]), pb.face_id_str(fs[1])));
            }
            if ta && tb {
                return Some((pb.face_id_str(fs[1]), pb.face_id_str(fs[0])));
            }
        }
        None
    };
    let mut ids = vec![];
    for (param, param2, idx) in &groups {
        let mut edges: Vec<Json> = vec![];
        for &i in idx {
            let p = &picks[i];
            let (na, nb) = match rebuilt_names(p) {
                Some(x) => x,
                None => (cx.name_of(p.a)?, cx.name_of(p.b)?),
            };
            edges.push(Obj::new().put("faces", arr(vec![s(&na), s(&nb)])).put("at", vec3(p.at)).build());
        }
        let id = cx.tree.ids.next();
        let o = Obj::new().put("type", if chamfer { "chamfer" } else { "fillet" }).put("id", id.as_str());
        let o = if !chamfer {
            o.put("radius", n(*param))
        } else if same(*param, *param2) {
            o.put("chamferType", "equal").put("distance", n(*param))
        } else {
            o.put("chamferType", "distances").put("distance", n(*param)).put("distance2", n(*param2))
        };
        cx.tree.push(o.put("edges", arr(edges)).build());
        // the blend names its faces by the order of its edges
        for (k, &i) in idx.iter().enumerate() {
            cx.names.insert(strip[i], format!("{id}.B{k}"));
        }
        ids.push(id);
    }
    // corner patches: a face of the first blend (a pick on them resolves by feature and position)
    for &g in faces {
        cx.names.entry(g).or_insert_with(|| format!("{}.B0", ids[0]));
    }
    let what = if chamfer { "distance" } else { "radius" };
    let mut note = dim_note(cx, f, "D1", what, groups[0].0);
    if chamfer && !same(groups[0].0, groups[0].1) {
        note.push_str(&format!(" / {:.4}", groups[0].1));
    }
    note.push_str(&format!("; {} edge(s)", picks.len()));
    if groups.len() > 1 {
        note.push_str(&format!(", {} sizes", groups.len()));
    }
    if corners > 0 {
        note.push_str(&format!(", {corners} corner patch(es)"));
    }
    Ok(note)
}
