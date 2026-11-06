//! Sweeps (boss and cut) and domes, read back as the extrude or revolve they are geometrically.
//!
//! * a **sweep along a straight path** leaves the faces of an extrusion along the path (planes, cylinders and
//!   extrusion surfaces sharing one direction): read back by the extrude reader (`draft::plain_or_drafted`);
//! * a **sweep along a line and an arc** (a planar path, the profile square to it where the two meet, the usual
//!   tooth-milling cut of a gear) leaves surfaces of revolution about the arc's axis and surfaces of extrusion along
//!   the line: the arc piece is read back by the revolve reader as a partial revolve from the profile plane, and the
//!   line piece is the same profile sketch extruded along the line (`line_piece`);
//! * a **dome on a circular face** leaves one surface of revolution (a sphere, or an ellipsoid of revolution) about
//!   the face's axis: read back by the revolve reader (`revolve::revolve`), which closes the profile to the axis;
//! * a **loft** the loft reader cannot take apart (its section loops cut into by later features) whose faces are
//!   those of a revolve or an extrude is read back as that (`loft_as`).
//!
//! Anything else (a path of several arcs or a spline, a dome on a non-circular face, a loft through free-form
//! sections) stays unsupported. The rebuild is checked against the import by the caller like any other step.

use crate::Cx;
use ck_json::{Json, Obj};
use ck_recog::emit::n;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_recog::model::Kind;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;

/// The SolidWorks sweep types: `Some(cut)`.
pub fn sweep_kind(f: &SwFeature) -> Option<bool> {
    match f.kind.as_str() {
        "Sweep" => Some(f.name.starts_with("Cut")),
        "SweepCut" | "Cut-Sweep" => Some(true),
        _ => None,
    }
}

/// The dispatch hook: `None` when the feature is no sweep or dome.
pub fn rebuild(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Option<Result<String, String>> {
    if f.kind == "Dome" {
        return Some(dome(cx, f, faces));
    }
    let cut = sweep_kind(f)?;
    if f.name.contains("Thin") || f.kind.contains("Thin") {
        return Some(Err("thin sweep".into()));
    }
    Some(sweep(cx, f, faces, cut))
}

/// What a read-back may change, to undo it when it fails (`restore`).
pub(crate) type Saved = (usize, usize, usize, std::collections::HashMap<FaceId, String>);

pub(crate) fn save(cx: &Cx) -> Saved {
    (cx.tree.ids.0, cx.tree.feats.len(), cx.notes.len(), cx.names.clone())
}

/// Undo what a failed read-back emitted.
pub(crate) fn restore(cx: &mut Cx, saved: &Saved) {
    cx.tree.ids.0 = saved.0;
    cx.tree.feats.truncate(saved.1);
    cx.notes.truncate(saved.2);
    cx.names = saved.3.clone();
}

/// The axis of a face on a surface of revolution.
fn rev_axis(cx: &Cx, g: FaceId) -> Option<(Vec3, Vec3)> {
    match cx.m.kind(g) {
        Kind::Cyl { o, a, .. } | Kind::Cone { o, a, .. } => Some((*o, *a)),
        Kind::Torus { c, a, .. } => Some((*c, *a)),
        Kind::Other(_) => match &cx.m.b.face(g).surface {
            Surface::Revolution { axis_o, axis, .. } => Some((*axis_o, axis.normalize())),
            _ => None,
        },
        _ => None,
    }
}

/// The direction of a face on a surface of extrusion (a plane has none of its own).
fn ext_dir(cx: &Cx, g: FaceId) -> Option<Vec3> {
    match cx.m.kind(g) {
        Kind::Cyl { a, .. } => Some(*a),
        Kind::Other(_) => match &cx.m.b.face(g).surface {
            Surface::Extrusion { dir, .. } => Some(dir.normalize()),
            _ => None,
        },
        _ => None,
    }
}

/// A sweep: along a straight path it is an extrude of its profile; along a path of a line and an arc (a planar path,
/// the profile carried square to it) it is an extrude and a revolve of the same profile that meet where the path
/// turns from one to the other. The faces split into the two groups by their surfaces (surfaces of extrusion along
/// one direction and planes along it; surfaces of revolution about one axis and planes square to it), and each group
/// is read back by its own reader; the revolve spans from the half plane of the edges the groups share to a little
/// past the faces' angular extent (where the tool leaves the material).
fn sweep(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let saved = save(cx);
    let straight = match crate::draft::plain_or_drafted(cx, f, faces, cut) {
        Ok(n) => return Ok(format!("sweep along a straight path as an extrude: {n}")),
        Err(e) => e,
    };
    restore(cx, &saved);
    let why = |e: String| format!("sweep: as an extrude along a straight path: {straight}; along a line and an arc: {e}");
    // the axis of the arc and the direction of the line
    let Some((ao, a)) = faces.iter().find_map(|&g| rev_axis(cx, g)) else { return Err(why("no surface of revolution".into())) };
    let on_axis = |cx: &Cx, g: FaceId| rev_axis(cx, g).map_or(false, |(o, d)| d.cross(a).len() < 1e-7 && ck_recog::model::dist_to_line(o, ao, a) < crate::TOL * (1.0 + ao.len()));
    // (from a surface of extrusion, else from a plane along the line: square to the axis and to its normal; a plane
    // through the axis is an end cap of the arc piece)
    let through_axis = |n: Vec3, dd: f64| (n.dot(ao) - dd).abs() < crate::TOL * (1.0 + ao.len());
    let along_plane = |cx: &Cx, g: FaceId| match cx.m.kind(g) {
        Kind::Plane { n, d: dd } if n.cross(a).len() > 1e-7 && !through_axis(*n, *dd) => a.cross(*n).try_normalize(),
        _ => None,
    };
    let Some(d) = faces.iter().filter(|&&g| !on_axis(cx, g)).find_map(|&g| ext_dir(cx, g)).or_else(|| faces.iter().find_map(|&g| along_plane(cx, g))) else {
        return Err(why("no surface of extrusion or plane along a line off the arc's axis".into()));
    };
    if d.dot(a).abs() > 1e-7 {
        return Err(why("the line of the path is not square to the arc's axis (no planar path)".into()));
    }
    // (a plane both along the line and square to the axis goes by the side of the profile plane it lies on)
    let (mut rev, mut ext, mut either) = (vec![], vec![], vec![]);
    for &g in faces {
        if on_axis(cx, g) {
            rev.push(g);
        } else if ext_dir(cx, g).map_or(false, |e| e.cross(d).len() < 1e-7) {
            ext.push(g);
        } else if let Kind::Plane { n, .. } = cx.m.kind(g) {
            let (along, square) = (n.dot(d).abs() < 1e-7, n.cross(a).len() < 1e-7);
            if along && square {
                either.push(g);
            } else if along {
                ext.push(g);
            } else if square {
                rev.push(g);
            } else if n.cross(d).len() < 1e-7 {
                // the end cap of the line piece (it ends in the material)
                ext.push(g);
            } else {
                return Err(why(format!("plane {} fits neither the line nor the arc", cx.m.b.face_id_str(g))));
            }
        } else {
            return Err(why(format!("face {} ({}) fits neither the line nor the arc", cx.m.b.face_id_str(g), cx.m.kind(g).name())));
        }
    }
    if rev.is_empty() || ext.is_empty() {
        return Err(why("the faces are not split between a line and an arc".into()));
    }
    // where the path turns: a point of an edge between the two groups, off the axis
    let mut junction = None;
    for &g in &rev {
        for e in cx.m.edges(g) {
            if cx.m.across(e, g).map_or(false, |h| ext.contains(&h)) {
                let p = cx.m.mid(e);
                if ck_recog::model::dist_to_line(p, ao, a) > 1e-6 {
                    junction = Some(p);
                }
            }
        }
    }
    let Some(p0) = junction else { return Err(why("the line's and the arc's faces share no edge".into())) };
    let side: f64 = ext.iter().flat_map(|&g| crate::face_points(cx.m, g)).map(|q| (q - p0).dot(d)).sum();
    let tol = crate::TOL * (1.0 + p0.len());
    for g in either {
        let ds: Vec<f64> = crate::face_points(cx.m, g).into_iter().map(|q| (q - p0).dot(d) * side.signum()).collect();
        if ds.iter().all(|&t| t > -tol) {
            ext.push(g);
        } else if ds.iter().all(|&t| t < tol) {
            rev.push(g);
        } else {
            return Err(why(format!("plane {} lies on both sides of the profile plane", cx.m.b.face_id_str(g))));
        }
    }
    let (lo, hi) = cx.m.b.bbox();
    let rho = ck_recog::model::dist_to_line(p0, ao, a);
    let extra = ((0.05 * lo.dist(hi) + 1.0) / rho).min(0.5);
    let opts = crate::revolve::RevOpts { start: Some((p0, extra)), ..Default::default() };
    let r = match crate::revolve::revolve_with(cx, f, &rev, cut, &opts) {
        Ok(r) => r,
        Err(e) => {
            restore(cx, &saved);
            return Err(why(format!("arc piece: {e}")));
        }
    };
    // a cut's profile runs on past the material along its end tangents: a spline end is extended as the spline itself
    // (the tool's edge between a spline and a straight extension would lie on the surface it cuts, which the boolean
    // refuses)
    if cut {
        if let Some(sid) = cx.tree.feats.last().and_then(|j| j.get_str("sketch")).map(String::from) {
            if let Some(sk) = cx.tree.feats.iter_mut().rev().find(|j| j.get_str("id") == Some(sid.as_str())) {
                extend_spline_ends(sk);
            }
        }
    }
    match line_piece(cx, &rev, &ext, p0, d * side.signum(), opts.normal.get(), cut, lo.dist(hi)) {
        Ok(e) => Ok(format!("sweep along a line and an arc: arc piece {r}; line piece {e}")),
        Err(e) => {
            restore(cx, &saved);
            Err(why(format!("line piece: {e}")))
        }
    }
}

/// How far the line piece's extrude reaches back into the arc piece (mm).
const OVERLAP: f64 = 1e-3;

/// The line piece of a sweep: the arc piece's profile sketch (the last revolve emitted) extruded from the profile
/// plane along `w` to the far end of the line piece's faces (a cap there: exactly to it; else on past them, where the
/// tool has left the material). A face of it is named by the profile entity of the arc piece's face across the
/// profile plane.
#[allow(clippy::too_many_arguments)]
fn line_piece(cx: &mut Cx, rev: &[FaceId], ext: &[FaceId], p0: Vec3, w: Vec3, normal: Option<Vec3>, cut: bool, size: f64) -> Result<String, String> {
    let nz = normal.ok_or("no profile sketch")?;
    let (rid, sid) = {
        let last = cx.tree.feats.last().ok_or("no revolve")?;
        (last.get_str("id").ok_or("no revolve id")?.to_string(), last.get_str("sketch").ok_or("no revolve sketch")?.to_string())
    };
    let caps: Vec<FaceId> = ext.iter().copied().filter(|&g| matches!(cx.m.kind(g), Kind::Plane { n, .. } if n.cross(w).len() < 1e-7)).collect();
    let far = ext.iter().flat_map(|&g| crate::face_points(cx.m, g)).map(|q| (q - p0).dot(w)).fold(0.0, f64::max);
    let depth = if caps.is_empty() { far + 0.05 * size + 1.0 } else { far };
    if far < 1e-6 {
        return Err("the line piece's faces do not leave the profile plane".into());
    }
    let xid = cx.tree.ids.next();
    let flip = (w.dot(nz) < 0.0) != cut;
    // (the extrude starts a hair behind the profile plane, inside the arc piece's tool: its start cap would lie on the
    // wall the arc piece left there, a coplanar face the boolean refuses; the overlap departs from the arc piece's
    // tool by `OVERLAP^2 / 2r`, far below the modelling tolerance)
    let start = Json::Arr(vec![n(p0.x), n(p0.y), n(p0.z)]);
    let mut o = Obj::new()
        .put("type", "extrude")
        .put("id", xid.as_str())
        .put("op", if cut { "cut" } else { "add" })
        .put("sketch", sid.as_str())
        .put("start", "fromObject")
        .put("startRef", start)
        .put("startOffset", n(-OVERLAP))
        .put("end", "blind")
        .put("depth", n(depth + OVERLAP));
    if flip {
        o = o.put("flip", true);
    }
    cx.tree.push(o.build());
    let pre = format!("{rid}.S.");
    for &g in ext {
        if caps.contains(&g) {
            cx.names.insert(g, format!("{xid}.C1"));
            continue;
        }
        let ent = cx.m.edges(g).into_iter().find_map(|e| {
            let h = cx.m.across(e, g).filter(|h| rev.contains(h))?;
            cx.names.get(&h).and_then(|n| n.strip_prefix(&pre)).map(|s| s.to_string())
        });
        cx.names.insert(g, format!("{xid}.S.{}", ent.unwrap_or_else(|| "e1".into())));
    }
    Ok(format!("extrude of the same profile, depth {depth:.4}{}", if caps.is_empty() { " (past the material)" } else { " to its end cap" }))
}

/// A dome: its faces revolved about the axis of the circular face it sat on (the normal of its rim circles, which
/// runs through the centre of a spherical dome).
fn dome(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let own: Vec<FaceId> = faces.iter().copied().filter(|&g| cx.tag_of(g) == Some(f.id)).collect();
    let mut base: Option<Vec3> = None;
    for &g in &own {
        for e in cx.m.edges(g) {
            if cx.m.across(e, g).map_or(true, |h| own.contains(&h)) {
                continue;
            }
            let Curve::Circle { frame, .. } = &cx.m.b.edge(e).curve else { return Err("dome: a rim piece that is no circle (a dome on a face that is no disc)".into()) };
            match base {
                Some(b) if b.cross(frame.z).len() > 1e-7 => return Err("dome: rim circles of different axes".into()),
                _ => base = Some(frame.z),
            }
        }
    }
    let opts = crate::revolve::RevOpts { sphere_axis: base, ..Default::default() };
    crate::revolve::revolve_with(cx, f, faces, false, &opts).map(|n| format!("dome as a revolve: {n}")).map_err(|e| format!("dome: {e}"))
}

/// A loft whose sections the loft reader cannot take from its faces' boundary (later features cut into it) but whose
/// faces are those of a revolve (coaxial circles: a cone or a cylinder between planes square to its axis) or of an
/// extrude (equal sections): read back as that (`loft.rs` calls this when its own reading fails).
pub(crate) fn loft_as(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool, why: String) -> Result<String, String> {
    let saved = save(cx);
    let r = match crate::revolve::revolve(cx, f, faces, cut) {
        Ok(n) => return Ok(format!("loft as a revolve ({why}): {n}")),
        Err(e) => e,
    };
    restore(cx, &saved);
    match crate::draft::plain_or_drafted(cx, f, faces, cut) {
        Ok(n) => Ok(format!("loft as an extrude ({why}): {n}")),
        Err(e) => {
            restore(cx, &saved);
            Err(format!("{why}; as a revolve: {r}; as an extrude: {e}"))
        }
    }
}

type P2 = (f64, f64);

/// The polar form (blossom) of span `span` of a B-spline at `args` (de Boor's triangle, one argument per level).
fn blossom(ctrl: &[P2], knots: &[f64], p: usize, span: usize, args: &[f64]) -> P2 {
    let mut d: Vec<P2> = (0..=p).map(|j| ctrl[span - p + j]).collect();
    for r in 1..=p {
        let u = args[r - 1];
        for j in (r..=p).rev() {
            let i = span - p + j;
            let a = (u - knots[i]) / (knots[i + p + 1 - r] - knots[i]);
            d[j] = ((1.0 - a) * d[j - 1].0 + a * d[j].0, (1.0 - a) * d[j - 1].1 + a * d[j].1);
        }
    }
    d[p]
}

/// A clamped B-spline carried on past its end by `delta` in parameter (its last polynomial piece continued): the
/// last `p + 1` knots move to the new end and the control points they govern are the piece's polar values there.
fn extend_end(ctrl: &[P2], knots: &[f64], p: usize, delta: f64) -> (Vec<P2>, Vec<f64>) {
    let n = ctrl.len();
    let b = knots[knots.len() - 1];
    let mut span = n - 1;
    while knots[span] >= b {
        span -= 1;
    }
    let mut nk: Vec<f64> = knots[..n].to_vec();
    nk.extend(std::iter::repeat(b + delta).take(p + 1));
    let mut out = ctrl.to_vec();
    for (i, slot) in out.iter_mut().enumerate().skip(n - p) {
        *slot = blossom(ctrl, knots, p, span, &nk[i + 1..i + p + 1]);
    }
    (out, nk)
}

fn extend_start(ctrl: &[P2], knots: &[f64], p: usize, delta: f64) -> (Vec<P2>, Vec<f64>) {
    let (a, b) = (knots[0], knots[knots.len() - 1]);
    let rk: Vec<f64> = knots.iter().rev().map(|k| a + b - k).collect();
    let rc: Vec<P2> = ctrl.iter().rev().copied().collect();
    let (c, k) = extend_end(&rc, &rk, p, delta);
    (c.into_iter().rev().collect(), k.iter().rev().map(|x| a + b - x).collect())
}

/// In a sketch: a `cvspline` end met by a line along its end tangent (the straight run-on of a cut's profile) becomes
/// the spline continued as far as the line went, and the line goes. Returns how many ends were extended.
fn extend_spline_ends(sk: &mut Json) -> usize {
    let num = |j: &Json, k: &str| j.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
    let pts: Vec<(String, P2)> = sk.get_arr("points").unwrap_or(&[]).iter().map(|p| (p.get_str("id").unwrap_or("").to_string(), (num(p, "x"), num(p, "y")))).collect();
    let at = |id: &str| pts.iter().find(|p| p.0 == id).map(|p| p.1);
    let ents: Vec<Json> = sk.get_arr("entities").unwrap_or(&[]).to_vec();
    let lines: Vec<(String, String, String)> = ents
        .iter()
        .filter(|e| e.get_str("type") == Some("line") && e.get("construction").and_then(|c| c.as_bool()) != Some(true))
        .filter_map(|e| Some((e.get_str("id")?.to_string(), e.get_str("p1")?.to_string(), e.get_str("p2")?.to_string())))
        .collect();
    let d2 = |a: P2, b: P2| (a.0 - b.0).hypot(a.1 - b.1);
    let (mut drop, mut moved, mut added): (Vec<String>, Vec<(String, P2)>, Vec<(String, P2)>) = (vec![], vec![], vec![]);
    let mut new_ents = ents.clone();
    let mut count = 0;
    for e in new_ents.iter_mut().filter(|e| e.get_str("type") == Some("cvspline")) {
        let ids: Vec<String> = e.get_arr("pts").unwrap_or(&[]).iter().filter_map(|v| v.as_str().map(String::from)).collect();
        let Some(mut ctrl) = ids.iter().map(|i| at(i)).collect::<Option<Vec<P2>>>() else { continue };
        let mut knots: Vec<f64> = e.get_arr("knots").unwrap_or(&[]).iter().filter_map(|v| v.as_f64()).collect();
        let p = num(e, "degree") as usize;
        if p == 0 || ctrl.len() <= p || knots.len() != ctrl.len() + p + 1 {
            continue;
        }
        let (mut first, mut last) = (ids[0].clone(), ids[ids.len() - 1].clone());
        let mut changed = false;
        for at_end in [false, true] {
            let (q, nb) = if at_end { (ctrl[ctrl.len() - 1], ctrl[ctrl.len() - 2]) } else { (ctrl[0], ctrl[1]) };
            let t = (q.0 - nb.0, q.1 - nb.1);
            let lt = t.0.hypot(t.1);
            let run = lines.iter().filter(|l| !drop.contains(&l.0)).find_map(|(lid, a, b)| {
                let (pa, pb) = (at(a)?, at(b)?);
                let (far, fid) = if d2(pa, q) < 1e-6 { (pb, b) } else if d2(pb, q) < 1e-6 { (pa, a) } else { return None };
                let v = (far.0 - q.0, far.1 - q.1);
                let lv = v.0.hypot(v.1);
                let along = lt > 1e-12 && lv > 1e-9 && (t.0 * v.1 - t.1 * v.0).abs() < 1e-6 * lt * lv && t.0 * v.0 + t.1 * v.1 > 0.0;
                along.then(|| (lid.clone(), fid.clone(), lv))
            });
            let Some((lid, fid, lv)) = run else { continue };
            // the end speed of a clamped spline: p |P_end - P_next| / (knot span next to the end)
            let (c, k) = if at_end {
                let kk = knots.iter().rev().copied().find(|&k| k < knots[knots.len() - 1]).unwrap_or(knots[0]);
                extend_end(&ctrl, &knots, p, lv / (p as f64 * lt / (knots[knots.len() - 1] - kk)))
            } else {
                let kk = knots.iter().copied().find(|&k| k > knots[0]).unwrap_or(knots[knots.len() - 1]);
                extend_start(&ctrl, &knots, p, lv / (p as f64 * lt / (kk - knots[0])))
            };
            ctrl = c;
            knots = k;
            let end_pt = if at_end { ctrl[ctrl.len() - 1] } else { ctrl[0] };
            moved.push((fid.clone(), end_pt));
            if at_end {
                last = fid;
            } else {
                first = fid;
            }
            drop.push(lid);
            changed = true;
            count += 1;
        }
        if !changed {
            continue;
        }
        // the interior control points are new points (the old ones stay, unused)
        let mut nids = vec![Json::Str(first)];
        for &c in &ctrl[1..ctrl.len() - 1] {
            let id = format!("xs{}", pts.len() + added.len() + 1);
            added.push((id.clone(), c));
            nids.push(Json::Str(id));
        }
        nids.push(Json::Str(last));
        e.set("pts", Json::Arr(nids));
        e.set("knots", Json::Arr(knots.iter().map(|&k| n(k)).collect()));
    }
    if count == 0 {
        return 0;
    }
    new_ents.retain(|e| !e.get_str("id").map_or(false, |i| drop.iter().any(|d| d == i)));
    let mut new_pts: Vec<Json> = sk.get_arr("points").unwrap_or(&[]).to_vec();
    for pj in new_pts.iter_mut() {
        if let Some((_, q)) = pj.get_str("id").and_then(|i| moved.iter().find(|m| m.0 == i)) {
            let q = *q;
            pj.set("x", n(q.0));
            pj.set("y", n(q.1));
        }
    }
    for (id, q) in added {
        new_pts.push(Obj::new().put("id", id.as_str()).put("x", n(q.0)).put("y", n(q.1)).build());
    }
    sk.set("points", Json::Arr(new_pts));
    sk.set("entities", Json::Arr(new_ents));
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(ctrl: &[P2], knots: &[f64], p: usize, t: f64) -> P2 {
        let mut span = p;
        while span + 1 < ctrl.len() && knots[span + 1] <= t {
            span += 1;
        }
        blossom(ctrl, knots, p, span, &vec![t; p])
    }

    #[test]
    fn extended_spline_keeps_its_old_part_and_continues_it() {
        let ctrl = [(0.0, 0.0), (1.0, 2.0), (2.5, 2.0), (3.0, 1.0), (4.0, 0.0)];
        let knots = [0.0, 0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0, 1.0];
        let (ce, ke) = extend_end(&ctrl, &knots, 3, 0.3);
        let (cs, ks) = extend_start(&ctrl, &knots, 3, 0.2);
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let (a, b, c) = (eval(&ctrl, &knots, 3, t), eval(&ce, &ke, 3, t), eval(&cs, &ks, 3, t));
            assert!((a.0 - b.0).hypot(a.1 - b.1) < 1e-12 && (a.0 - c.0).hypot(a.1 - c.1) < 1e-12, "t {t}: {a:?} {b:?} {c:?}");
        }
        // the new ends lie on the last (first) polynomial piece carried on
        let span = 4;
        let past = blossom(&ctrl, &knots, 3, span, &[1.3; 3]);
        let end = ce[ce.len() - 1];
        assert!((past.0 - end.0).hypot(past.1 - end.1) < 1e-12, "{past:?} {end:?}");
        let before = blossom(&ctrl, &knots, 3, 3, &[-0.2; 3]);
        assert!((before.0 - cs[0].0).hypot(before.1 - cs[0].1) < 1e-12, "{before:?} {:?}", cs[0]);
    }
}
