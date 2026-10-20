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
//! * Errors (the model is unchanged on any): `nothingToCut` (no solid),
//!   `noFaces`, `missingReference`, `invalidValue` (non-finite), `moveFaceUnsupported
//!   {detail}` (a surface kind the move cannot do: free-form, translate mode on a
//!   curved face, a face of a surface body), `moveFaceFailed {detail}` (the neighbours
//!   cannot meet the moved faces: an edge collapses or reverses, the result is not
//!   valid or the volume changed the wrong way). A distance of 0 is a no-op.

use super::{Brep, TOL};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_topo::body::{Body, FaceId};

fn failed(detail: impl Into<String>) -> FeatureError {
    FeatureError::new("moveFaceFailed").with("detail", Json::Str(detail.into()))
}
fn unsupported(detail: impl Into<String>) -> FeatureError {
    FeatureError::new("moveFaceUnsupported").with("detail", Json::Str(detail.into()))
}

fn volume(b: &Body) -> f64 {
    let all: Vec<FaceId> = b.face_ids().collect();
    ck_topo::mass::signed_volume(b, &all)
}

/// Resolve the face names (dedup, split families by base id).
fn resolve(br: &Brep, names: &[String]) -> Result<Vec<FaceId>, FeatureError> {
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

/// Move `ids` of `body` by the per-face outward distances `dist` (aligned with
/// `ids`; unselected faces stay, tangent round neighbours of a pushed plane travel with it
/// when that gives a valid body). The pure geometric core, also used by the tests.
pub fn move_faces(body: &Body, ids: &[FaceId], dist: &[f64]) -> Result<Body, FeatureError> {
    // a closed-spline wall carries a two-lap profile: cut it to one lap per face first
    let unrolled = ck_ops::offset::single_lap_body(body);
    let body = &unrolled;
    let carried = carried_faces(body, ids, dist);
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
    let out = move_faces(body, &ids, &dist)?;
    br.body = Some(out);
    Ok(())
}
