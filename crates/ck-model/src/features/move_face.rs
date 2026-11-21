//! `moveFace`: push / pull faces, the neighbours following.
//!
//! Wire shape (GUI handles, docs/spec-agent-interface.md "Direct manipulation"):
//!
//! ```json
//! {"type":"moveFace","id":"F7","faces":["F2.C1"],"mode":"offset","distance":2.5}
//! {"type":"moveFace","id":"F8","faces":["F2.S.e1"],"mode":"translate","translate":[3,0,0]}
//! ```
//!
//! * `faces`: persistent face names (a face split into `<id>~k` pieces is named by
//!   its base id). Required (`noFaces`), every name must resolve in the solid
//!   (`missingReference`).
//! * `mode: "offset"` (default): every face moves along its OUTWARD normal by
//!   `distance` (`flip` negates it); positive adds material. Planes slide along
//!   their normal, cylinders / cones / spheres / tori change radius
//!   (a hole wall with a positive distance shrinks the hole).
//! * `mode: "translate"`: the faces move rigidly by `translate:[x,y,z]` (numbers or
//!   `"=expr"`). Only planar faces are supported: a plane moves by the component of
//!   the vector along its outward normal (the sliding part changes nothing).
//! * The unselected neighbours keep their surfaces and are extended (or trimmed) to
//!   meet the moved faces: vertices are re-solved on the incident surfaces, edges follow
//!   (ck-ops `offset_body` with distance 0 for the unselected faces). Topology, face
//!   order and persistent names are unchanged.
//! * A selected plane with an unselected coplanar neighbour across an edge (one piece of a split face) is a step: the
//!   piece alone rises (union with its prism) or sinks (cut by it) and a wall `<id>.S.<name of the piece across>` appears along
//!   the split line, as Move Face does in Onshape / SolidWorks (`step_move`). The moved face keeps its name. Refused
//!   (`moveFaceFailed` with `maxSize`): a sink as deep as or deeper than the body under the piece, a rise into other material,
//!   a result that splits the body. Refused (`moveFaceUnsupported`): such a piece selected together with other faces (move one
//!   piece at a time). Naming the base id (or every piece) moves them all, as before.
//! * Errors (the model is unchanged on any): `nothingToCut` (no solid),
//!   `noFaces`, `missingReference`, `invalidValue` (non-finite), `moveFaceUnsupported
//!   {detail}` (a surface kind the move cannot do: free-form, translate mode on a
//!   curved face, a face of a surface body), `moveFaceFailed {detail}` (the neighbours
//!   cannot meet the moved faces: an edge collapses or reverses, the result is not
//!   valid or the volume changed the wrong way). A distance of 0 is a no-op. An offset that fails at its
//!   distance but moves at a smaller one carries `maxSize` (distance, the sign kept), see max_size.rs.

use super::{Brep, TOL};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_topo::body::{Body, FaceId};

pub(super) fn failed(detail: impl Into<String>) -> FeatureError {
    FeatureError::new("moveFaceFailed").with("detail", Json::Str(detail.into()))
}
pub(super) fn unsupported(detail: impl Into<String>) -> FeatureError {
    FeatureError::new("moveFaceUnsupported").with("detail", Json::Str(detail.into()))
}

pub(super) fn volume(b: &Body) -> f64 {
    let all: Vec<FaceId> = b.face_ids().collect();
    ck_topo::mass::signed_volume(b, &all)
}

/// Resolve the face names (dedup, split families by base id).
pub(super) fn resolve(br: &Brep, names: &[String]) -> Result<Vec<FaceId>, FeatureError> {
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new(codes::NOTHING_TO_CUT))?;
    let mut ids: Vec<FaceId> = vec![];
    for n in names {
        if let Some(g) = body.find_face(n) {
            if !ids.contains(&g) {
                ids.push(g);
            }
            continue;
        }
        if br.surfs.iter().any(|(_, s)| s.find_face(n).is_some()) {
            return Err(unsupported(format!("{n} belongs to a surface body")));
        }
        let base = super::lip::base_id(n);
        let fam: Vec<FaceId> = body.face_ids().filter(|&g| super::lip::base_id(&body.face_id_str(g)) == base).collect();
        if fam.is_empty() {
            return Err(FeatureError::new(codes::MISSING_REFERENCE).with("ref", Json::Str(n.clone())));
        }
        for g in fam {
            if !ids.contains(&g) {
                ids.push(g);
            }
        }
    }
    Ok(ids)
}

/// Outward distance each selected face moves by (`None` = not supported).
fn face_distance(b: &Body, f: FaceId, translate: Option<ck_geom::Vec3>, offset: f64) -> Result<f64, FeatureError> {
    let face = b.face(f);
    let name = b.face_id_str(f);
    match (&face.surface, translate) {
        (Surface::Plane { frame }, Some(t)) => {
            let n = if face.sense { frame.z } else { -frame.z };
            Ok(t.dot(n))
        }
        (_, Some(_)) => Err(unsupported(format!("translate mode moves planar faces only; {name} is a {} (use mode offset)", face.surface.kind()))),
        (Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. } | Surface::Extrusion { .. }, None) => Ok(offset),
        (s, None) => Err(unsupported(format!("{name} is a {} face; planes, cylinders, cones, spheres, tori and extruded walls can be moved", s.kind()))),
    }
}

/// A selected plane that moves while an unselected plane lies flush with it across a shared edge (the other piece of a split face,
/// two coplanar faces a boolean left apart): the neighbour cannot follow, so the edge would need a wall of its own, which this
/// feature does not build. Refused up front with the way out, instead of a broken edge after the solve.
fn flush_neighbour(b: &Body, ids: &[FaceId], dist: &[f64]) -> Option<(FaceId, FaceId)> {
    let plane = |f: FaceId| match &b.face(f).surface {
        Surface::Plane { frame } => Some((frame.o, if b.face(f).sense { frame.z } else { -frame.z })),
        _ => None,
    };
    for (k, &f) in ids.iter().enumerate() {
        if dist[k].abs() < 1e-12 {
            continue;
        }
        let Some((o, n)) = plane(f) else { continue };
        for e in b.face_edges(f) {
            for g in b.edge_faces(e) {
                if g == f || ids.contains(&g) {
                    continue;
                }
                let Some((o2, n2)) = plane(g) else { continue };
                if n.dot(n2) > 1.0 - 1e-9 && n.dot(o2 - o).abs() < 1e-7 {
                    return Some((f, g));
                }
            }
        }
    }
    None
}

/// The outline of the plane face `f` as a closed profile on the face's own plane (outer loop CCW, holes CW), with the face's
/// outward normal. Segments are tagged by the face across the edge (`F2_C1p1`), so a wall built from it keeps the same name for
/// as long as the face next to it does. `None` when the face is not a plane or its loops do not close.
pub(super) fn face_profile(b: &Body, f: FaceId) -> Option<(ck_topo::make::Profile, ck_geom::Vec3)> {
    use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let Surface::Plane { frame } = &b.face(f).surface else { return None };
    let n_out = if b.face(f).sense { frame.z } else { -frame.z };
    let mut loops: Vec<ProfileLoop> = vec![];
    for &l in &b.face(f).loops {
        let mut segs = vec![];
        for &c in &b.lp(l).coedges {
            let co = b.coedge(c);
            let ed = b.edge(co.edge);
            let across = b.edge_faces(co.edge).into_iter().find(|&g| g != f)?;
            let tag: String = b.face_id_str(across).chars().map(|ch| if ch == '.' { '_' } else if ch == '~' { 'p' } else { ch }).collect();
            let curve = ck_topo::make::plane_pcurve(frame, &ed.curve)
                .or_else(|| ck_geom::pcurve::pcurve_of(&b.face(f).surface, &ed.curve, ed.t0, ed.t1, TOL).ok())?;
            segs.push(ProfileSeg { tag, curve, t0: ed.t0, t1: ed.t1, rev: co.rev });
        }
        loops.push(ProfileLoop::new(segs));
    }
    if loops.is_empty() {
        return None;
    }
    let outer_at = (0..loops.len()).max_by(|&a, &c| loops[a].signed_area().abs().total_cmp(&loops[c].signed_area().abs()))?;
    let outer = loops.remove(outer_at);
    Some((Profile::new(*frame, vec![ProfileRegion { outer, holes: loops }]).normalized(), n_out))
}

/// Heights `(s0, s1)` along the profile normal of a run of `d` from the face along its outward normal.
pub(super) fn run_heights(profile_z: ck_geom::Vec3, n_out: ck_geom::Vec3, d: f64) -> (f64, f64) {
    let up = n_out.dot(profile_z) > 0.0;
    match (up, d > 0.0) {
        (true, true) => (0.0, d),
        (true, false) => (d, 0.0),
        (false, true) => (-d, 0.0),
        (false, false) => (0.0, -d),
    }
}

/// The plane piece `f` swept along its outward normal by `d` (`d` > 0 out of the body, `d` < 0 into it) as a solid whose first
/// end sits on the face: the tool of a step move. Walls are tagged by the face across the edge (`F2_C1p1`), so a wall keeps the
/// same name for as long as the piece next to it does. `None` when the face is not a plane or its loops do not close.
fn face_prism(b: &Body, f: FaceId, d: f64, fid: &str) -> Option<Body> {
    use ck_topo::make::{extrude, ExtrudeOpts};
    let (profile, n_out) = face_profile(b, f)?;
    let (s0, s1) = run_heights(profile.frame.z, n_out, d);
    extrude(fid, &profile, ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).ok()
}

/// Move the one plane piece `f` of a split face by `d` along its outward normal while its flush neighbours stay: the half rises
/// (union with its prism) or sinks (cut by it) and a wall appears along the split line, as in Onshape's and SolidWorks' Move
/// Face. The moved face keeps its name; the new walls are named `<fid>.S.<name of the piece across>`.
fn step_move(body: &Body, f: FaceId, d: f64, fid: &str) -> Result<Body, FeatureError> {
    let shown = body.face_id_str(f);
    let prism = face_prism(body, f, d, fid).ok_or_else(|| unsupported(format!("{shown} cannot be swept: its outline is not made of closed loops on one plane")))?;
    apply_prism(body, f, &prism, d, fid)
}

/// Join (`d` > 0) or cut (`d` < 0) the tool `prism` (a run of the plane face `f` along its outward normal by `d`, built with
/// feature id `fid`): the far cap of the tool becomes the moved face and takes the name of `f`. Checks the volume, the cap, the
/// validity and the lump count; the model is unchanged on error (`moveFaceFailed`).
pub(super) fn apply_prism(body: &Body, f: FaceId, prism: &Body, d: f64, fid: &str) -> Result<Body, FeatureError> {
    use ck_ops::boolean::{boolean, Op};
    let name = body.face(f).name.clone();
    let shown = body.face_id_str(f);
    let n_out = body.planar_normal(f);
    let o_out = match &body.face(f).surface {
        Surface::Plane { frame } => frame.o,
        _ => return Err(unsupported(format!("{shown} is not flat"))),
    };
    let vp = volume(prism);
    let (v0, out_op) = (volume(body), if d > 0.0 { Op::Union } else { Op::Cut });
    let r = boolean(body, prism, out_op, TOL).map_err(|e| failed(format!("the step of {shown} could not be built ({e})")))?;
    let mut nb = r.body;
    let want = if d > 0.0 { v0 + vp } else { v0 - vp };
    let got = volume(&nb);
    if r.mismatch.is_some() || r.no_effect || (got - want).abs() > 1e-5 * v0.abs().max(1.0) + 1e-6 * vp {
        return Err(failed(if d > 0.0 {
            format!("{shown} cannot rise by {d}: something above it is in the way")
        } else {
            format!("{shown} cannot sink by {}: the body is not that thick under it (a move through it would cut the piece away)", -d)
        }));
    }
    // the moved face: the one cap of the tool left on the outside of the body, facing the same way as the old face, `d` from it
    let cap = nb.face_ids().find(|&g| {
        let nm = &nb.face(g).name;
        nm.feature == fid
            && matches!(nm.role, ck_topo::body::Role::Cap(_))
            && matches!(nb.face(g).surface, Surface::Plane { .. })
            && nb.planar_normal(g).dot(n_out) > 1.0 - 1e-9
            && match &nb.face(g).surface {
                Surface::Plane { frame } => ((frame.o - o_out).dot(n_out) - d).abs() < 1e-6 * d.abs().max(1.0),
                _ => false,
            }
    });
    let Some(cap) = cap else {
        return Err(failed(format!("{shown} would reach the far side of the body: nothing is left under it to move to")));
    };
    nb.faces[cap.idx()].name = name;
    super::renumber_duplicates(&mut nb);
    let before = ck_check::validate(body, TOL).len();
    let issues = ck_check::validate(&nb, TOL);
    if issues.len() > before {
        let i = &issues[0];
        return Err(failed(format!("the result is not valid ({} {})", i.code, i.entity)));
    }
    if nb.lump_ids().count() != body.lump_ids().count() {
        return Err(failed(format!("moving {shown} by {d} would split the body in pieces")));
    }
    Ok(nb)
}

/// Rebuild the pcurves `offset_body` dropped (exact for the analytic pairs; the
/// periodic branch follows the old pcurve when there was one).
fn restore_pcurves(out: &mut Body, src: &Body) {
    use ck_geom::Vec2;
    for c in 0..out.coedges.len() {
        let cid = ck_topo::body::CoedgeId(c as u32);
        if !out.coedge(cid).alive {
            continue;
        }
        let f = out.coedge_face(cid);
        let ed = out.edge(out.coedge(cid).edge).clone();
        let surf = out.face(f).surface.clone();
        let new = ck_geom::pcurve::exact_pcurve(&surf, &ed.curve, TOL * 10.0)
            .or_else(|| ck_geom::pcurve::pcurve_of(&surf, &ed.curve, ed.t0, ed.t1, TOL).ok());
        let Some(mut pc) = new else { continue };
        if let Some(old) = &src.coedge(cid).pcurve {
            let (pu, pv) = surf.periods();
            let t = 0.5 * (ed.t0 + ed.t1);
            let (a, b) = (old.eval(t), pc.eval(t));
            let du = pu.map_or(0.0, |p| ((a.x - b.x) / p).round() * p);
            let dv = pv.map_or(0.0, |p| ((a.y - b.y) / p).round() * p);
            if du != 0.0 || dv != 0.0 {
                pc = shift(&pc, Vec2::new(du, dv));
            }
        }
        out.coedges[c].pcurve = Some(pc);
    }
}

fn shift(pc: &ck_geom::curve2::Curve2, d: ck_geom::Vec2) -> ck_geom::curve2::Curve2 {
    use ck_geom::curve2::Curve2;
    match pc {
        Curve2::Line { o, d: dir } => Curve2::Line { o: *o + d, d: *dir },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        other => other.clone(),
    }
}

/// Round faces (fillets, cylinders) tangent to a pushed planar face travel with it so the
/// blend stays tangent: they translate rigidly by the plane's displacement. Planes
/// tangent across an edge stay (they slide), and a round face reached with two
/// different displacements is left to the plain extension.
fn carried_faces(body: &Body, ids: &[FaceId], dist: &[f64]) -> Vec<(FaceId, ck_geom::Vec3)> {
    const TANGENT_DOT: f64 = 0.999999;
    let mut out: Vec<(FaceId, ck_geom::Vec3)> = vec![];
    let mut clash: Vec<FaceId> = vec![];
    let round = |f: FaceId| matches!(body.face(f).surface, Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. });
    for (k, &f) in ids.iter().enumerate() {
        let Surface::Plane { frame } = &body.face(f).surface else { continue };
        if dist[k] == 0.0 {
            continue;
        }
        let n = if body.face(f).sense { frame.z } else { -frame.z };
        let t = n.scale(dist[k]);
        let mut stack = vec![f];
        let mut seen = vec![f];
        while let Some(h) = stack.pop() {
            for e in body.face_edges(h) {
                let m = body.edge_mid(e);
                for g in body.edge_faces(e) {
                    if seen.contains(&g) || ids.contains(&g) || !round(g) {
                        continue;
                    }
                    let nrm = |x: FaceId| {
                        let (u, v) = body.face(x).surface.closest_param(m);
                        body.outward_normal(x, u, v)
                    };
                    if nrm(h).dot(nrm(g)) <= TANGENT_DOT {
                        continue;
                    }
                    seen.push(g);
                    match out.iter().find(|(x, _)| *x == g) {
                        Some((_, t0)) if (*t0 - t).len() > 1e-12 => clash.push(g),
                        Some(_) => {}
                        None => out.push((g, t)),
                    }
                    stack.push(g);
                }
            }
        }
    }
    out.retain(|(g, _)| !clash.contains(g));
    out
}

/// A moved cylinder wall whose neighbour across an edge is a torus blend (the fillet at its foot) that is not moved
/// with it: the torus would keep its major radius while the wall's radius changes, a body slightly off. Refused
/// like any edge the offset cannot rebuild (the fillet would have to be carried or reshaped with the wall).
fn base_torus_conflict(body: &Body, ids: &[FaceId], dist: &[f64], carried: &[(FaceId, ck_geom::Vec3)]) -> Option<String> {
    const TANGENT_DOT: f64 = 0.999999;
    for (k, &f) in ids.iter().enumerate() {
        if dist[k] == 0.0 || !matches!(body.face(f).surface, Surface::Cylinder { .. }) {
            continue;
        }
        for e in body.face_edges(f) {
            let m = body.edge_mid(e);
            for g in body.edge_faces(e) {
                if g == f || ids.contains(&g) || carried.iter().any(|(c, _)| *c == g) || !matches!(body.face(g).surface, Surface::Torus { .. }) {
                    continue;
                }
                let nrm = |x: FaceId| {
                    let (u, v) = body.face(x).surface.closest_param(m);
                    body.outward_normal(x, u, v)
                };
                if nrm(f).dot(nrm(g)).abs() > TANGENT_DOT {
                    return Some(format!("edge between {} and {}: edge curve type", body.face_id_str(f), body.face_id_str(g)));
                }
            }
        }
    }
    None
}

/// Move `ids` of `body` by the per-face outward distances `dist` (aligned with
/// `ids`; unselected faces stay, tangent round neighbours of a pushed plane travel with it
/// when that gives a valid body). The pure geometric core, also used by the tests.
pub fn move_faces(body: &Body, ids: &[FaceId], dist: &[f64]) -> Result<Body, FeatureError> {
    // a closed-spline wall carries a two-lap profile: cut it to one lap per face first
    let unrolled = ck_ops::offset::single_lap_body(body);
    let body = &unrolled;
    let carried = carried_faces(body, ids, dist);
    if let Some(msg) = base_torus_conflict(body, ids, dist, &carried) {
        return Err(super::offset_unsupported_hint(unsupported(msg.clone()), &msg));
    }
    if carried.is_empty() {
        return attempt(body, ids, dist, &[]);
    }
    match attempt(body, ids, dist, &carried) {
        Ok(b) => Ok(b),
        Err(first) => attempt(body, ids, dist, &[]).map_err(|_| first),
    }
}

fn attempt(body: &Body, ids: &[FaceId], dist: &[f64], carried: &[(FaceId, ck_geom::Vec3)]) -> Result<Body, FeatureError> {
    use ck_ops::offset::FaceMove;
    let mv = |f: FaceId| {
        if let Some(i) = ids.iter().position(|&g| g == f) {
            FaceMove::Offset(dist[i])
        } else if let Some((_, t)) = carried.iter().find(|(g, _)| *g == f) {
            FaceMove::Shift(*t)
        } else {
            FaceMove::Offset(0.0)
        }
    };
    let mut out = ck_ops::offset::move_faces_body(body, &mv).map_err(|e| {
        let detail = e.detail.clone().unwrap_or_default();
        let fe = match e.code {
            "offsetUnsupported" => super::offset_unsupported_hint(unsupported(detail.clone()), &detail),
            _ => failed(detail),
        };
        match &e.limit {
            Some(l) => super::offset_limit_params(fe, l),
            None => fe,
        }
    })?;
    // geometry sanity: finite, edges inside their parameter range
    for e in out.edge_ids() {
        let ed = out.edge(e);
        let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) || !(ed.t1 - ed.t0 > 0.0) {
            return Err(failed("an edge degenerated"));
        }
        if let Curve::Line { .. } = ed.curve {
            if (ed.curve.eval(ed.t1) - ed.curve.eval(ed.t0)).len() < 1e-9 {
                return Err(failed("an edge shrank to nothing"));
            }
        }
    }
    restore_pcurves(&mut out, body);
    let before = ck_check::validate(body, TOL).len();
    let issues = ck_check::validate(&out, TOL);
    if issues.len() > before {
        let i = &issues[0];
        return Err(failed(format!("the result is not valid ({} {})", i.code, i.entity)));
    }
    // the volume must move the way the faces do
    let (v0, v1) = (volume(body), volume(&out));
    let grow = dist.iter().all(|&d| d >= 0.0);
    let shrink = dist.iter().all(|&d| d <= 0.0);
    let slack = 1e-6 * v0.abs().max(1.0);
    if !(v1.is_finite()) || (v0 > 0.0 && v1 <= 0.0) || (grow && v1 < v0 - slack) || (shrink && v1 > v0 + slack) {
        return Err(failed(format!("the volume went from {v0:.3} to {v1:.3}")));
    }
    Ok(out)
}

pub fn exec_move_face(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let names: Vec<String> = f
        .raw
        .get("faces")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    if names.is_empty() {
        return Err(FeatureError::new("noFaces"));
    }
    let mode = f.raw.get_str("mode").unwrap_or("offset");
    let translate = match mode {
        "offset" => None,
        "translate" => Some(super::move_body::vec3(state, f.raw.get("translate"))?.ok_or_else(|| {
            FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str("translate mode needs translate:[x,y,z]".into()))
        })?),
        m => return Err(FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(format!("unknown mode {m} (offset | translate)")))),
    };
    let mut offset = 0.0;
    if translate.is_none() {
        offset = state.num(f, "distance", 0.0)?;
        if !offset.is_finite() {
            return Err(FeatureError::new(codes::INVALID_VALUE));
        }
        if crate::sketch::jtruthy(f.raw.get("flip")) {
            offset = -offset;
        }
    }
    let ids = resolve(br, &names)?;
    let body = br.body.as_ref().unwrap();
    let dist: Vec<f64> = ids.iter().map(|&g| face_distance(body, g, translate, offset)).collect::<Result<_, _>>()?;
    if dist.iter().all(|&d| d.abs() < 1e-12) {
        return Ok(());
    }
    if let Some((pf, g)) = flush_neighbour(body, &ids, &dist) {
        let (a, c) = (body.face_id_str(pf), body.face_id_str(g));
        if ids.len() != 1 {
            return Err(unsupported(format!(
                "{a} lies flush with {c} (the other piece of a split face): move one piece at a time, or name every piece (or the face they were split from) to move them together"
            )));
        }
        let step = |d: f64| step_move(body, pf, d, &f.id);
        return match step(dist[0]) {
            Ok(b) => {
                br.body = Some(b);
                Ok(())
            }
            Err(e) if e.code == "moveFaceFailed" => {
                let asked = dist[0].abs();
                let max = super::max_size::largest_that_builds(asked, None, |v| step(v.copysign(dist[0])).is_ok());
                let given = state.num(f, "distance", 0.0)?;
                // the distance field is the offset; for translate mode the vector is not a single number
                Err(match (max, translate.is_none()) {
                    (Some(m), true) => super::max_size::with_max_size(e, "distance", m.copysign(given), "mm"),
                    _ => e,
                })
            }
            Err(e) => Err(e),
        };
    }
    let out = match move_faces(body, &ids, &dist) {
        Ok(b) => b,
        // an offset that cannot be made this far: the largest distance that moves (same sign as asked)
        Err(e) if e.code == "moveFaceFailed" && translate.is_none() => {
            let named = e.params.iter().find(|(k, _)| k == "maxThickness").and_then(|(_, v)| v.as_f64()).filter(|m| *m > 0.0);
            let asked = offset.abs();
            let max = super::max_size::largest_that_builds(asked, named, |v| {
                let d: Vec<f64> = dist.iter().map(|x| x * v / asked).collect();
                move_faces(body, &ids, &d).is_ok()
            });
            let given = state.num(f, "distance", 0.0)?;
            return Err(match max {
                Some(m) => super::max_size::with_max_size(e, "distance", m.copysign(given), "mm"),
                None => e,
            });
        }
        Err(e) => return Err(e),
    };
    br.body = Some(out);
    Ok(())
}
