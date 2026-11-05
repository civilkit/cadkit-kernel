//! Loft features (boss and cut), read back from the faces the feature left on the final body.
//!
//! A SolidWorks loft leaves its lateral faces (one per profile piece, B-spline, plane or ruled analytic surface;
//! `FACE_ID_2001` kind 74) and, where a section is not on earlier material, a planar end cap (kind 77). Nothing of
//! the sketches is decoded: the sections are the boundary of the lateral faces.
//!
//! * the edges of the lateral faces that border another face (a cap, a face of another feature) form closed loops;
//!   every loop is planar, and the loops in one plane are one section (the largest is the outline, the others its
//!   inner loops: a hollow loft). Two section planes make a two-section loft; one plane whose lateral faces close
//!   at a single vertex is a loft to a point (apex) section. SolidWorks splits the section curves where the lateral
//!   faces meet, so the sections come with matching piece counts and the kernel's start matching pairs them;
//! * the rails (edges between two lateral faces): straight rails are a plain loft (ruled between two sections). A
//!   curved rail is read as an end constraint when every lateral face is a cubic Bezier in the loft direction whose
//!   first (last) control row leaves its section along the section normal: `startNormal` (`endNormal`) with the
//!   tension `3 |P1 - P0| / L` (the kernel's rails are parametrised by the chord `L` between the sections'
//!   centroids). Any other curved rail (guide curves, tangency to faces, more than two sections) is not read back;
//! * section pieces: lines, arcs and circles, and non-rational B-splines (as `cvspline` with the trimmed curve's
//!   control points and knots); anything else is not read back.
//!
//! The loft is emitted as two sketches (standard plane with an offset when the section normal is a principal axis,
//! else a frame plane) and a `loft` feature (`op` add / cut). Lateral faces are named `<loft>.S.<entity>` by the
//! piece of the first section they start from, caps `<loft>.C0` / `<loft>.C1`. The rebuild is checked against the
//! import by the caller; a loft whose surfaces our loft does not reproduce falls back there.

use crate::{Cx, Fr};
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::{arr, n, s, Sk};
use ck_recog::model::Kind;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{EdgeId, FaceId};
use std::collections::HashSet;

/// `FACE_ID_2001` kind code of a loft's end cap (sample census: a plane).
pub const KIND_LOFT_CAP: i64 = 77;

/// The SolidWorks loft types: `Some(cut)`.
pub fn loft_kind(f: &SwFeature) -> Option<bool> {
    let (k, name) = (f.kind.as_str(), f.name.as_str());
    match k {
        "Loft" | "Blend" | "LoftBoss" => Some(name.starts_with("Cut")),
        "LoftCut" | "BlendCut" | "Cut-Loft" | "CutLoft" => Some(true),
        _ => None,
    }
}

/// The dispatch hook: `None` when the feature is no loft.
pub fn rebuild(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Option<Result<String, String>> {
    let cut = loft_kind(f)?;
    if f.name.contains("Thin") || f.kind.contains("Thin") {
        return Some(Err("thin loft".into()));
    }
    Some(loft(cx, f, faces, cut))
}

/// An edge of a loop in travel order.
#[derive(Debug, Clone, Copy)]
struct Oe {
    e: EdgeId,
    fwd: bool,
}

struct Loop {
    edges: Vec<Oe>,
    /// Unit normal (Newell) and plane offset.
    nrm: Vec3,
    d: f64,
    area: f64,
    /// Mean of the piece start points.
    cen: Vec3,
}

struct Section {
    outer: Loop,
    holes: Vec<Loop>,
}

fn samples(cx: &Cx, oe: Oe, k: usize) -> Vec<Vec3> {
    let ed = cx.m.b.edge(oe.e);
    let mut v: Vec<Vec3> = (0..k).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / k as f64)).collect();
    v[0] = cx.m.p0(oe.e);
    if !oe.fwd {
        v = (1..=k).map(|i| ed.curve.eval(ed.t1 - (ed.t1 - ed.t0) * i as f64 / k as f64)).collect();
        v[0] = cx.m.p1(oe.e);
    }
    v
}

fn start(cx: &Cx, oe: Oe) -> Vec3 {
    if oe.fwd { cx.m.p0(oe.e) } else { cx.m.p1(oe.e) }
}

fn end(cx: &Cx, oe: Oe) -> Vec3 {
    if oe.fwd { cx.m.p1(oe.e) } else { cx.m.p0(oe.e) }
}

/// Closed loops of the boundary edges (chained by their end points).
fn loops(cx: &Cx, bnd: &[EdgeId], tol: f64) -> Result<Vec<Loop>, String> {
    let mut free: Vec<EdgeId> = bnd.to_vec();
    let mut out = vec![];
    while let Some(e0) = free.pop() {
        let mut es = vec![Oe { e: e0, fwd: true }];
        let first = cx.m.p0(e0);
        let mut cur = cx.m.p1(e0);
        let mut guard = 0;
        while cur.dist(first) > tol {
            guard += 1;
            if guard > 10_000 {
                return Err("section loop does not close".into());
            }
            let Some(i) = free.iter().position(|&e| cx.m.p0(e).dist(cur) <= tol || cx.m.p1(e).dist(cur) <= tol) else {
                return Err("the boundary of the loft's faces is not a set of closed loops".into());
            };
            let e = free.swap_remove(i);
            let fwd = cx.m.p0(e).dist(cur) <= tol;
            let oe = Oe { e, fwd };
            cur = end(cx, oe);
            es.push(oe);
        }
        let pts: Vec<Vec3> = es.iter().flat_map(|&oe| samples(cx, oe, 16)).collect();
        let mut nv = Vec3::ZERO;
        for i in 0..pts.len() {
            nv = nv + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        let area = nv.len() / 2.0;
        if area < tol * tol {
            return Err("a section loop encloses no area".into());
        }
        let nrm = nv.normalize();
        let d = nrm.dot(pts[0]);
        let size = pts.iter().map(|p| p.dist(pts[0])).fold(0.0, f64::max);
        if pts.iter().any(|p| (nrm.dot(*p) - d).abs() > 1e-6 * (1.0 + size)) {
            return Err("a section loop is not planar".into());
        }
        let cen = es.iter().map(|&oe| start(cx, oe)).fold(Vec3::ZERO, |a, b| a + b) / es.len() as f64;
        out.push(Loop { edges: es, nrm, d, area, cen });
    }
    Ok(out)
}

/// The loops grouped by plane: one section per plane, its largest loop the outline.
fn sections(ls: Vec<Loop>, tol: f64) -> Vec<Section> {
    let mut groups: Vec<Vec<Loop>> = vec![];
    for l in ls {
        match groups.iter_mut().find(|g| {
            let o = &g[0];
            o.nrm.cross(l.nrm).len() < 1e-6 && (l.cen - o.cen).dot(o.nrm).abs() < tol * 10.0
        }) {
            Some(g) => g.push(l),
            None => groups.push(vec![l]),
        }
    }
    groups
        .into_iter()
        .map(|mut g| {
            g.sort_by(|a, b| b.area.total_cmp(&a.area));
            let outer = g.remove(0);
            Section { outer, holes: g }
        })
        .collect()
}

fn straight(cx: &Cx, e: EdgeId) -> bool {
    let ed = cx.m.b.edge(e);
    if matches!(ed.curve, Curve::Line { .. }) {
        return true;
    }
    let ps: Vec<Vec3> = (0..=16).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 16.0)).collect();
    let (a, z) = (ps[0], ps[16]);
    let l = a.dist(z);
    l > 0.0 && ps.iter().all(|p| (*p - a).cross(z - a).len() / l < 1e-7 * (1.0 + l))
}

/// The lateral face is ruled between its two section edges (plane, cylinder, cone, or a B-spline of degree 1 with
/// two rows in one direction).
fn ruled(cx: &Cx, f: FaceId) -> bool {
    match &cx.m.b.face(f).surface {
        Surface::BSpline(b) => (b.deg_u == 1 && b.nu == 2) || (b.deg_v == 1 && b.nv == 2),
        _ => matches!(cx.m.kind(f), Kind::Plane { .. } | Kind::Cyl { .. } | Kind::Cone { .. }),
    }
}

/// End tangents of a cubic loft: per end the tension `3 |C1 - C0| / L` when every rail (edge between two lateral
/// faces) is a single cubic span (a Hermite rail of the loft parameter, the chord `L` between the sections'
/// centroids) whose first (last) control leg runs along the section normal; `None` for an end without that
/// constraint. Without rails (one closed lateral face) the face's control rows in the loft direction stand in for
/// them. `Err` when a rail is no such cubic.
fn tensions(cx: &Cx, lat: &[FaceId], rails: &[EdgeId], s0: &Loop, s1: &Loop) -> Result<[Option<f64>; 2], String> {
    let l = s0.cen.dist(s1.cen);
    let on = |p: Vec3, sl: &Loop| (sl.nrm.dot(p) - sl.d).abs() < 1e-6 * (1.0 + l);
    let mut rows: Vec<[Vec3; 4]> = vec![];
    for &e in rails {
        let ed = cx.m.b.edge(e);
        let Curve::BSpline(c) = ed.curve.segment(ed.t0, ed.t1) else { return Err("a curved rail that is no B-spline".into()) };
        let even = c.weights.as_ref().map_or(true, |w| w.iter().all(|x| (x - w[0]).abs() < 1e-12));
        if c.degree != 3 || c.ctrl.len() != 4 || !even {
            return Err("a rail that is no single cubic span (guide curves or more sections)".into());
        }
        rows.push([c.ctrl[0], c.ctrl[1], c.ctrl[2], c.ctrl[3]]);
    }
    if rails.is_empty() {
        for &f in lat {
            let Surface::BSpline(b) = &cx.m.b.face(f).surface else { return Err("a curved lateral face that is no B-spline".into()) };
            if b.weights.as_ref().map_or(false, |w| w.iter().any(|x| (x - w[0]).abs() > 1e-12)) {
                return Err("a rational lateral face without rails".into());
            }
            // the rows of the loft direction: index (i, j) -> ctrl[i * nv + j]
            let along_v = b.deg_v == 3 && b.nv == 4 && (0..b.nu).all(|i| on(b.ctrl[i * b.nv], s0) && on(b.ctrl[i * b.nv + 3], s1) || on(b.ctrl[i * b.nv], s1) && on(b.ctrl[i * b.nv + 3], s0));
            let along_u = b.deg_u == 3 && b.nu == 4 && (0..b.nv).all(|j| on(b.ctrl[j], s0) && on(b.ctrl[3 * b.nv + j], s1) || on(b.ctrl[j], s1) && on(b.ctrl[3 * b.nv + j], s0));
            if along_v {
                rows.extend((0..b.nu).map(|i| [0, 1, 2, 3].map(|k| b.ctrl[i * b.nv + k])));
            } else if along_u {
                rows.extend((0..b.nv).map(|j| [0, 1, 2, 3].map(|k| b.ctrl[k * b.nv + j])));
            } else {
                return Err(format!("lateral face {} is no single cubic span between the sections (guide curves or more sections)", cx.m.b.face_id_str(f)));
            }
        }
    }
    let mut out: [Vec<Option<f64>>; 2] = [vec![], vec![]];
    for r in rows {
        let r = if on(r[0], s0) && on(r[3], s1) {
            r
        } else if on(r[3], s0) && on(r[0], s1) {
            [r[3], r[2], r[1], r[0]]
        } else {
            return Err("a rail that does not run from one section to the other".into());
        };
        for (end, (p0, p1), sl) in [(0, (r[0], r[1]), s0), (1, (r[3], r[2]), s1)] {
            let d = p1 - p0;
            let t = if d.len() > 1e-9 * (1.0 + l) && d.cross(sl.nrm).len() < 1e-6 * d.len() { Some(3.0 * d.len() / l) } else { None };
            out[end].push(t);
        }
    }
    let mut res = [None, None];
    for end in 0..2 {
        let v = &out[end];
        if v.iter().all(|t| t.is_none()) {
            continue;
        }
        let Some(t0) = v[0] else { return Err("an end constraint along the normal on part of the section only".into()) };
        if v.iter().any(|t| t.map_or(true, |t| (t - t0).abs() > 1e-4 * t0)) {
            return Err("an end constraint with an uneven tension (tangent to faces, or a weight per piece)".into());
        }
        res[end] = Some(t0);
    }
    if res.iter().all(|t| t.is_none()) {
        return Err("curved rails without an end constraint along the section normal (guide curves)".into());
    }
    Ok(res)
}

/// One sketch of a section: entities in loop order; `names` gets (edge, entity) for every piece.
fn section_sketch(cx: &mut Cx, sec: &Section, names: &mut Vec<(EdgeId, String)>) -> Result<String, String> {
    let at = sec.outer.cen;
    let fr = cx.frame(sec.outer.nrm, at);
    if fr.z.cross(sec.outer.nrm).len() > 1e-9 {
        return Err("sketch frame does not match the section plane".into());
    }
    let sid = cx.tree.ids.next();
    let mut sk = Sk::new(sid.clone(), fr.plane.clone());
    let mut pts: Vec<((f64, f64), String)> = vec![];
    for l in std::iter::once(&sec.outer).chain(sec.holes.iter()) {
        for &oe in &l.edges {
            let e = piece(cx, &fr, &mut sk, &mut pts, oe)?;
            names.push((oe.e, e));
        }
    }
    cx.tree.push(sk.json());
    Ok(sid)
}

fn pt(fr: &Fr, sk: &mut Sk, pts: &mut Vec<((f64, f64), String)>, p: Vec3) -> String {
    let q = fr.xy(p);
    if let Some((_, id)) = pts.iter().find(|(r, _)| (r.0 - q.0).hypot(r.1 - q.1) < 1e-9) {
        return id.clone();
    }
    let id = sk.point(q.0, q.1);
    pts.push((q, id.clone()));
    id
}

/// What a section piece is: a B-spline edge that runs on a straight line or a circle (a loft's own rational
/// pieces, an exporter's spline approximation) is that line or arc.
enum Shape {
    Line,
    /// Centre and radius.
    Arc(Vec3, f64),
    Spline(ck_geom::nurbs::bspline::BSplineCurve),
}

fn shape(cx: &Cx, oe: Oe, z: Vec3) -> Result<Shape, String> {
    let ed = cx.m.b.edge(oe.e);
    match &ed.curve {
        Curve::Line { .. } => return Ok(Shape::Line),
        Curve::Circle { frame, r } => {
            if frame.z.cross(z).len() > 1e-7 {
                return Err("a section arc is not in the section plane".into());
            }
            return Ok(Shape::Arc(frame.o, *r));
        }
        Curve::BSpline(_) => {}
        other => return Err(format!("a section piece of kind {}", other.kind())),
    }
    let Curve::BSpline(c) = ed.curve.segment(ed.t0, ed.t1) else { unreachable!() };
    let ps: Vec<Vec3> = (0..=32).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 32.0)).collect();
    let size = ps.iter().map(|p| p.dist(ps[0])).fold(0.0, f64::max);
    let tol = 1e-7 * (1.0 + size);
    let (a, b) = (ps[0], ps[32]);
    if a.dist(b) > tol && ps.iter().all(|p| (*p - a).cross(b - a).len() / a.dist(b) < tol) {
        return Ok(Shape::Line);
    }
    // a circle through three samples (the circumcentre in the section plane)
    let (p, q, r) = (ps[0], ps[11], ps[22]);
    let (u, v) = (q - p, r - p);
    let w = u.cross(v);
    if w.len() > tol * size && w.normalize().cross(z).len() < 1e-7 {
        let c = p + (w.cross(u) * v.dot(v) + v.cross(w) * u.dot(u)) / (2.0 * w.dot(w));
        let rad = c.dist(p);
        if ps.iter().all(|x| (x.dist(c) - rad).abs() < tol) {
            return Ok(Shape::Arc(c, rad));
        }
    }
    if c.weights.as_ref().map_or(false, |w| w.iter().any(|x| (x - 1.0).abs() > 1e-12)) {
        return Err("a rational spline section piece (no line or arc)".into());
    }
    Ok(Shape::Spline(c))
}

/// The sketch entity of one section piece.
fn piece(cx: &Cx, fr: &Fr, sk: &mut Sk, pts: &mut Vec<((f64, f64), String)>, oe: Oe) -> Result<String, String> {
    let (a, b) = (start(cx, oe), end(cx, oe));
    match shape(cx, oe, fr.z)? {
        Shape::Line => {
            let (pa, pb) = (pt(fr, sk, pts, a), pt(fr, sk, pts, b));
            Ok(sk.line(&pa, &pb, false))
        }
        Shape::Arc(c, r) => {
            let pc = pt(fr, sk, pts, c);
            if a.dist(b) < 1e-9 * (1.0 + r) {
                return Ok(sk.circle(&pc, r));
            }
            let (pa, pb) = (pt(fr, sk, pts, a), pt(fr, sk, pts, b));
            let (qa, qc, qm) = (fr.xy(a), fr.xy(c), fr.xy(cx.m.mid(oe.e)));
            let ccw = (qa.0 - qc.0) * (qm.1 - qc.1) - (qa.1 - qc.1) * (qm.0 - qc.0) > 0.0;
            Ok(if ccw { sk.arc(&pc, &pa, &pb) } else { sk.arc(&pc, &pb, &pa) })
        }
        Shape::Spline(c) => {
            let ids: Vec<String> = c.ctrl.iter().map(|&p| pt(fr, sk, pts, p)).collect();
            let id = format!("cv{}", sk.entities.len() + 1);
            sk.entities.push(
                Obj::new()
                    .put("type", "cvspline")
                    .put("id", id.as_str())
                    .put("pts", arr(ids.iter().map(|x| s(x)).collect()))
                    .put("degree", n(c.degree as f64))
                    .put("knots", arr(c.knots.iter().map(|&k| n(k)).collect()))
                    .build(),
            );
            Ok(id)
        }
    }
}

pub fn loft(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let own: Vec<FaceId> = faces.iter().copied().filter(|&g| cx.tag_of(g) == Some(f.id)).collect();
    let cap = |cx: &Cx, g: FaceId| matches!(cx.m.kind(g), Kind::Plane { .. }) && cx.tags.get(g.idx()).map_or(false, |v| v.iter().any(|t| t.0 == f.id && t.1 == KIND_LOFT_CAP));
    let caps: Vec<FaceId> = own.iter().copied().filter(|&g| cap(cx, g)).collect();
    let lat: Vec<FaceId> = own.iter().copied().filter(|g| !caps.contains(g)).collect();
    if lat.is_empty() {
        return Err("no lateral faces".into());
    }
    let lset: HashSet<FaceId> = lat.iter().copied().collect();
    let (lo, hi) = cx.m.b.bbox();
    let tol = 1e-6 * (1.0 + lo.dist(hi));
    // boundary edges (to a face that is no lateral face of the loft) and rails (between two lateral faces)
    let mut bnd: Vec<EdgeId> = vec![];
    let mut rails: Vec<EdgeId> = vec![];
    for &g in &lat {
        for e in cx.m.edges(g) {
            match cx.m.across(e, g) {
                Some(h) if lset.contains(&h) => {
                    if g.idx() < h.idx() {
                        rails.push(e);
                    }
                }
                _ => bnd.push(e),
            }
        }
    }
    bnd.sort_by_key(|e| e.idx());
    bnd.dedup();
    let secs = sections(loops(cx, &bnd, tol)?, tol);
    // a point (apex) section: the rails meet at one vertex off the section
    let mut apex: Option<Vec3> = None;
    match secs.len() {
        2 => {}
        1 => {
            let s0 = &secs[0].outer;
            let ends: Vec<Vec3> = rails.iter().flat_map(|&e| [cx.m.p0(e), cx.m.p1(e)]).filter(|p| (s0.nrm.dot(*p) - s0.d).abs() > tol * 10.0).collect();
            let Some(&a) = ends.first() else { return Err("one section and no apex".into()) };
            if ends.iter().any(|p| p.dist(a) > tol * 10.0) {
                return Err("one section and the lateral faces do not close at a point".into());
            }
            apex = Some(a);
        }
        k => return Err(format!("{k} section planes (a loft through more than two sections is not read back)")),
    }
    if secs.iter().any(|s| s.holes.len() != secs[0].holes.len()) {
        return Err("sections with different numbers of inner loops".into());
    }
    // order: a section without a cap (on earlier material) first, else the lower one along the loft's dominant axis
    let mut secs = secs;
    if secs.len() == 2 {
        let capped = |sec: &Section| caps.iter().any(|&c| {
            let p = cx.m.b.vertex(cx.m.b.edge(cx.m.b.face_edges(c)[0]).v0).p;
            (sec.outer.nrm.dot(p) - sec.outer.d).abs() < tol * 10.0
        });
        let (c0, c1) = (capped(&secs[0]), capped(&secs[1]));
        let d = secs[1].outer.cen - secs[0].outer.cen;
        let k = (0..3).max_by(|&i, &j| ck_recog::model::comp(d, i).abs().total_cmp(&ck_recog::model::comp(d, j).abs())).unwrap_or(0);
        if (c0 && !c1) || (c0 == c1 && ck_recog::model::comp(d, k) < 0.0) {
            secs.swap(0, 1);
        }
    }
    let all_straight = rails.iter().all(|&e| straight(cx, e));
    let mut ten = [None, None];
    let ruled_faces = lat.iter().all(|&g| ruled(cx, g));
    if apex.is_none() && !(all_straight && (ruled_faces || !rails.is_empty())) {
        ten = tensions(cx, &lat, &rails, &secs[0].outer, &secs[1].outer)?;
    }
    if apex.is_some() && !all_straight {
        return Err("a loft to a point with curved rails (end constraints at the apex)".into());
    }
    // emit
    let mut names0: Vec<(EdgeId, String)> = vec![];
    let mut names1: Vec<(EdgeId, String)> = vec![];
    let s0 = section_sketch(cx, &secs[0], &mut names0)?;
    let s1 = if let Some(a) = apex {
        let fr = cx.frame(secs[0].outer.nrm, a);
        let sid = cx.tree.ids.next();
        let mut sk = Sk::new(sid.clone(), fr.plane.clone());
        let (x, y) = fr.xy(a);
        let p = sk.point(x, y);
        sk.point_entity(&p);
        cx.tree.push(sk.json());
        sid
    } else {
        section_sketch(cx, &secs[1], &mut names1)?
    };
    let lid = cx.tree.ids.next();
    let mut o = Obj::new().put("type", "loft").put("id", lid.as_str()).put("op", if cut { "cut" } else { "add" }).put("sketches", arr(vec![s(&s0), s(&s1)]));
    if let Some(t) = ten[0] {
        o = o.put("startNormal", n(t));
    }
    if let Some(t) = ten[1] {
        o = o.put("endNormal", n(t));
    }
    cx.tree.push(o.build());
    // names: a lateral face by the piece of the first section it starts from; caps by their section
    for (e, ent) in &names0 {
        if let Some(g) = cx.m.b.edge_faces(*e).into_iter().find(|g| lset.contains(g)) {
            cx.names.insert(g, format!("{lid}.S.{ent}"));
        }
    }
    for &c in &caps {
        let p = cx.m.b.vertex(cx.m.b.edge(cx.m.b.face_edges(c)[0]).v0).p;
        let k = if (secs[0].outer.nrm.dot(p) - secs[0].outer.d).abs() < tol * 10.0 { "C0" } else { "C1" };
        cx.names.insert(c, format!("{lid}.{k}"));
    }
    let shape = if apex.is_some() {
        "to a point".to_string()
    } else {
        let pieces = |s: &Section| s.outer.edges.len();
        format!("{} -> {} piece(s)", pieces(&secs[0]), pieces(&secs[1]))
    };
    let ends = match ten {
        [None, None] => "ruled".to_string(),
        [a, b] => format!("normal to the sections, tension {} / {}", a.map_or("-".into(), |t| format!("{t:.4}")), b.map_or("-".into(), |t| format!("{t:.4}"))),
    };
    Ok(format!(
        "{} loft, 2 sections ({shape}{}), {} lateral face(s), {} cap(s), {ends}",
        if cut { "cut" } else { "boss" },
        if secs[0].holes.is_empty() { String::new() } else { format!(", {} inner loop(s)", secs[0].holes.len()) },
        lat.len(),
        caps.len()
    ))
}
