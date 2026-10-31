//! Extrude / cut "To object" and "From object" (docs/spec-agent-interface.md, "To object / From object").
//!
//! * `end:"toObject"` + `endRef`: the extrusion ends at a reference, read again on every regen so the end follows it:
//!   a face (`"F2.C1"` or `{kind:"face",id}`: its plane), a plane (`{kind:"std",id}`, `{kind:"feature",id}` of a Plane
//!   feature earlier in the timeline), a point (`{kind:"sketchPoint",sketch,point}`, `{kind:"vertex",faces,at}`,
//!   `{kind:"point",at}` or `[x,y,z]`: the plane through it parallel to the sketch) or a body (`{kind:"body",name}`: the
//!   tool runs until it meets that body's surface). `endOffset` (expression, mm) moves a plane end along the extrusion
//!   (positive = further); a body end takes none.
//! * `start:"fromObject"` + `startRef` (+ `startOffset`): the extrusion starts at a plane, flat face or point instead of
//!   the sketch plane; the profile is carried along the extrusion direction onto it. Works with `end` `blind` (depth from
//!   the start; a slanted start gives a parallel slanted end) and `toObject`.
//! * A plane at an angle to the sketch is reached exactly: the tool is trimmed along it (as `upToFace`).
//! * Strict: a reference that cannot be found is `missingReference {field, hint}`; one that gives no end
//!   (`endRefNoEnd`), lies behind the start (`endRefBehind`), a body the tool never meets or does not cover it
//!   (`endRefMissed`, `endRefOutside`), a curved face (`upToFaceUnsupported`) are refusals with a `hint`. There is no
//!   blind fallback and no re-find by position (a face id is matched exactly, or as the one face of its `~k` family).

use super::extrude::{profile_samples, resolve_face, EndClip, EndExtras, END_REF_RELINKED, UP_TO_FACE_UNSUPPORTED};
use super::End;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::Frame;
use ck_geom::Vec3;
use ck_json::Json;
use ck_topo::body::Body;
use ck_topo::make::Profile;

/// The reference gives no end: a plane that holds the extrusion direction.
pub const END_REF_NO_END: &str = "endRefNoEnd";
/// The end lies behind the sketch (or the start) for some part of the profile.
pub const END_REF_BEHIND: &str = "endRefBehind";
/// A body end the tool never meets.
pub const END_REF_MISSED: &str = "endRefMissed";
/// A body end that does not cover the whole profile.
pub const END_REF_OUTSIDE: &str = "endRefOutside";

/// The tool runs until it meets the named body: what `Brep::trim_to_body` needs.
#[derive(Debug, Clone)]
pub struct ToBody {
    pub name: String,
    /// A point on the start plane and its normal: the kept piece is the one that touches it.
    pub start: (Vec3, Vec3),
    /// A point on the far cap of the long tool, and the unit extrusion direction.
    pub far: Vec3,
    pub dir: Vec3,
}

/// Where the swept profile starts and how long it is, and the trims it needs.
pub struct ObjPlan {
    /// Distance along the extrusion from the sketch plane to where the profile is swept from.
    pub s0: f64,
    pub len: f64,
    pub end_clip: Option<EndClip>,
    pub start_clip: Option<EndClip>,
    pub to_body: Option<ToBody>,
}

enum Target {
    /// A point on the plane, its normal (unit, either side).
    Plane(Vec3, Vec3),
    Body(String),
}

fn hint(e: FeatureError, h: &str) -> FeatureError {
    e.with("hint", Json::Str(h.into()))
}

fn lost(field: &str, h: &str) -> FeatureError {
    hint(FeatureError::new(codes::MISSING_REFERENCE).with("field", Json::Str(field.into())), h)
}

fn invalid(field: &str, detail: &str, h: &str) -> FeatureError {
    hint(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into())), h)
}

fn json_pt(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    if a.len() != 3 {
        return None;
    }
    Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?))
}

/// Whether the feature takes this path, checking the `start` field: `toObject`, or any end with `start:"fromObject"`
/// (which then must be `blind` or `toObject`). `start` absent, null or `"sketch"` is the sketch plane.
pub fn wanted(f: &Feature, end: End) -> Result<bool, FeatureError> {
    let from = match f.raw.get("start") {
        None | Some(Json::Null) => false,
        Some(Json::Str(s)) if s == "sketch" => false,
        Some(Json::Str(s)) if s == "fromObject" => true,
        Some(v) => {
            return Err(invalid("start", &format!("unknown start {}", ck_json::to_string(v)), "start is \"sketch\" (the default) or \"fromObject\" with a startRef"));
        }
    };
    if from && !matches!(end, End::Blind | End::ToObject) {
        return Err(invalid("start", "From object works with a blind or To object end", "set end to \"blind\" (depth from the start) or \"toObject\", or drop start"));
    }
    Ok(from || end == End::ToObject)
}

/// Resolves `field` (`endRef` / `startRef`) to a plane or a body. `normal`: the sketch normal (the plane through a point).
fn target(state: &RegenState, f: &Feature, field: &str, body: Option<&Body>, normal: Vec3, extras: &mut EndExtras) -> Result<Target, FeatureError> {
    let what = if field == "endRef" { "end" } else { "start" };
    let usage = format!(
        "give {field}: a face id, or {{kind:\"face\"|\"std\"|\"feature\", id}}, {{kind:\"sketchPoint\", sketch, point}}, {{kind:\"vertex\", faces, at}}, {{kind:\"point\", at}} or {{kind:\"body\", name}}"
    );
    let r = match f.raw.get(field) {
        None | Some(Json::Null) => return Err(lost(field, &usage)),
        Some(r) => r,
    };
    if let Some(p) = json_pt(r) {
        return Ok(Target::Plane(p, normal));
    }
    let kind = r.get("kind").and_then(|k| k.as_str());
    let id = || r.get("id").and_then(|v| v.as_str()).filter(|s| !s.is_empty());
    let face_id = match (r, kind) {
        (Json::Str(s), _) if !s.is_empty() => Some(s.as_str()),
        (_, Some("face")) => Some(id().ok_or_else(|| lost(field, &usage))?),
        _ => None,
    };
    if let Some(fid) = face_id {
        let Some(b) = body else {
            return Err(lost(field, &format!("there is no body yet, so the face {fid} cannot be found; pick a plane or a point as the {what}")).with("face", Json::Str(fid.into())));
        };
        let Some((face, relinked)) = resolve_face(b, fid, None) else {
            return Err(lost(field, &format!("the face {fid} no longer exists; pick the {what} again (a lost face is not looked for by position)")).with("face", Json::Str(fid.into())));
        };
        if relinked {
            extras.warnings.push(FeatureError::new(END_REF_RELINKED).with("from", Json::Str(fid.into())).with("to", Json::Str(b.face_id_str(face))));
        }
        let fc = b.face(face);
        let ck_geom::surface::Surface::Plane { frame } = &fc.surface else {
            return Err(hint(
                FeatureError::new(UP_TO_FACE_UNSUPPORTED).with("field", Json::Str(field.into())).with("face", Json::Str(fid.into())),
                &format!("the {what} must be flat: pick a flat face, a plane or a point{}", if field == "endRef" { ", or the body itself to stop at its curved surface" } else { "" }),
            ));
        };
        return Ok(Target::Plane(frame.o, frame.z));
    }
    match kind {
        Some("std") => {
            let p = id().and_then(crate::StdPlane::from_wire).ok_or_else(|| lost(field, "a standard plane is front, top or right"))?;
            let fr: Frame = p.frame();
            Ok(Target::Plane(fr.o, fr.z))
        }
        Some("feature") => {
            let pid = id().ok_or_else(|| lost(field, &usage))?;
            if let Some(fr) = state.plane(pid) {
                return Ok(Target::Plane(fr.o, fr.z));
            }
            let later = state.part.features.iter().any(|x| x.id == pid && x.type_name == "plane");
            let h = if later {
                format!("the plane {pid} comes later in the timeline than this feature or did not build; move it earlier, or pick another {what}")
            } else {
                format!("the plane {pid} no longer exists; pick the {what} again")
            };
            Err(lost(field, &h).with("plane", Json::Str(pid.into())))
        }
        Some("sketchPoint") => {
            let (sk, pt) = (r.get("sketch").and_then(|v| v.as_str()).unwrap_or(""), r.get("point").and_then(|v| v.as_str()).unwrap_or(""));
            let p = state.sketch(sk).and_then(|s| s.sketch.point_xy(pt).map(|xy| s.frame.o + s.frame.x * xy[0] + s.frame.y * xy[1]));
            p.map(|p| Target::Plane(p, normal)).ok_or_else(|| lost(field, &format!("the point {pt} of sketch {sk} no longer exists; pick the {what} again")))
        }
        Some("point") => r.get("at").and_then(json_pt).map(|p| Target::Plane(p, normal)).ok_or_else(|| lost(field, &usage)),
        Some("vertex") => {
            let faces: Vec<String> = r.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect()).unwrap_or_default();
            let at = r.get("at").and_then(json_pt);
            let gone = || lost(field, &format!("the corner where the faces {} meet no longer exists; pick the {what} again", faces.join(", ")));
            if faces.is_empty() || at.is_none() {
                return Err(lost(field, &usage));
            }
            let b = body.ok_or_else(gone)?;
            corner(b, &faces, at.unwrap()).map(|p| Target::Plane(p, normal)).ok_or_else(gone)
        }
        Some("body") => {
            let name = r.get("name").and_then(|v| v.as_str()).filter(|s| !s.is_empty()).ok_or_else(|| lost(field, &usage))?;
            if field != "endRef" {
                return Err(invalid(field, "a start cannot be a body", "start at a plane, a flat face or a point"));
            }
            Ok(Target::Body(name.to_string()))
        }
        _ => Err(lost(field, &usage)),
    }
}

/// The vertex every named face has, nearest `at`. The faces are matched by name only (no position fallback).
fn corner(b: &Body, faces: &[String], at: Vec3) -> Option<Vec3> {
    let mut cands: Vec<ck_topo::body::VertexId> = vec![];
    for (i, name) in faces.iter().enumerate() {
        let (f, _) = resolve_face(b, name, None)?;
        let vs: Vec<_> = b.face_edges(f).into_iter().flat_map(|e| [b.edge(e).v0, b.edge(e).v1]).collect();
        if i == 0 {
            cands = vs;
        } else {
            cands.retain(|v| vs.contains(v));
        }
    }
    cands.into_iter().map(|v| b.vertex(v).p).min_by(|x, y| x.dist(at).total_cmp(&y.dist(at)))
}

/// Distance along `d` from each sample to the plane `(p0, m)`; `None` when the plane holds the direction.
fn reach(pts: &[Vec3], d: Vec3, p0: Vec3, m: Vec3) -> Option<(Vec<f64>, f64)> {
    let c = d.dot(m);
    (c.abs() >= 1e-9).then(|| (pts.iter().map(|q| (p0 - *q).dot(m) / c).collect(), c))
}

fn range(v: &[f64]) -> (f64, f64) {
    v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, &t| (a.0.min(t), a.1.max(t)))
}

const FLAT: f64 = 1e-7;

/// Plans a To object / From object extrusion of `profile` (in the sketch frame `fr`, direction `-fr.z` when `flip`).
#[allow(clippy::too_many_arguments)]
pub fn plan(state: &RegenState, f: &Feature, fr: &Frame, profile: &Profile, flip: bool, end: End, depth: f64, body: Option<&Body>, extras: &mut EndExtras) -> Result<ObjPlan, FeatureError> {
    let d = if flip { -fr.z } else { fr.z };
    let pts = profile_samples(profile);
    let no_end = |field: &str| {
        hint(
            FeatureError::new(END_REF_NO_END).with("field", Json::Str(field.into())),
            "the plane runs along the extrusion, so it gives no end; pick a plane, face or point across the direction",
        )
    };
    // the start: distances from the sketch plane along d, and the start plane
    let from = super::has_start(f);
    let (ss, start_plane): (Vec<f64>, Option<(Vec3, Vec3, f64)>) = if from {
        let Target::Plane(ps, ms) = target(state, f, "startRef", body, fr.z, extras)? else { unreachable!() };
        let off = state.num(f, "startOffset", 0.0)?;
        let (t, c) = reach(&pts, d, ps, ms).ok_or_else(|| no_end("startRef"))?;
        (t.into_iter().map(|x| x + off).collect(), Some((ps + d * off, ms, c)))
    } else {
        (vec![0.0; pts.len()], None)
    };
    let (s0, s1) = range(&ss);
    let start_clip = start_plane.filter(|_| s1 - s0 > FLAT).map(|(p0, m, c)| EndClip { p0, w: if c > 0.0 { -m } else { m } });
    let behind = || {
        hint(
            FeatureError::new(END_REF_BEHIND).with("field", Json::Str("endRef".into())),
            "the end lies behind the start for some of the profile; flip the direction, or pick an end ahead of the whole profile",
        )
    };
    // the end
    let (te, end_plane): (Vec<f64>, Option<(Vec3, Vec3, f64)>) = if end == End::ToObject {
        match target(state, f, "endRef", body, fr.z, extras)? {
            Target::Body(name) => {
                if state.num(f, "endOffset", 0.0)? != 0.0 {
                    return Err(invalid("endOffset", "a body end takes no offset", "drop endOffset, or end at a face or plane of the body with an offset"));
                }
                let Some(b) = body else {
                    return Err(lost("endRef", &format!("there is no body named {name} yet; pick the end again")).with("name", Json::Str(name)));
                };
                let o = profile.frame.o + d * s0;
                let len = super::extrude::through_len(o, b) + (s1 - s0);
                let start = match start_plane {
                    Some((p0, m, _)) => (p0, m),
                    None => (o, fr.z),
                };
                let to_body = ToBody { name, start, far: o + d * len, dir: d };
                return Ok(ObjPlan { s0, len, end_clip: None, start_clip, to_body: Some(to_body) });
            }
            Target::Plane(p0, m) => {
                let off = state.num(f, "endOffset", 0.0)?;
                let (t, c) = reach(&pts, d, p0, m).ok_or_else(|| no_end("endRef"))?;
                (t.into_iter().map(|x| x + off).collect(), Some((p0 + d * off, m, c)))
            }
        }
    } else {
        // blind from the start: the end is the start plane moved by `depth`
        (ss.iter().map(|s| s + depth).collect(), start_plane.map(|(p0, m, c)| (p0 + d * depth, m, c)))
    };
    if te.iter().zip(&ss).any(|(e, s)| e - s <= 1e-9) {
        return Err(behind());
    }
    let (e0, e1) = range(&te);
    let end_clip = end_plane.filter(|_| e1 - e0 > FLAT).map(|(p0, m, c)| EndClip { p0, w: if c > 0.0 { m } else { -m } });
    Ok(ObjPlan { s0, len: e1 - s0, end_clip, start_clip, to_body: None })
}
