//! `helix` curves and `sweep` (ck-model/src/features/sweep.rs).
//!
//! helix (oracle: hex-bolt-m8x40 F11): the first circle of the sketch gives
//! centre / radius; output `curves[] = ["F11", {pts, axis:false, t0:0,
//! t1:angle, helix:{o,a,x,y,k,r,angle}}]` (ck_ops::helix pins the fields);
//! a sketch without a circle -> `helixNeedsCircle` (kernel.strings).
//! sweep: TODO(rebuild) — ck-ops `sweep` is a stub.

use super::{Brep, DefinedBy, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::sketch::SketchEntity;
use crate::Feature;
use ck_json::{Json, Obj};
use ck_ops::helix::{helix, HelixParams};

pub const HELIX_NEEDS_CIRCLE: &str = "helixNeedsCircle";

fn v3(p: ck_core::linalg::Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

/// `helix`.
pub fn exec_helix(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Helix { defined_by, left_hand, reverse, .. } = &f.kind else { unreachable!() };
    let sk = f.raw.get("sketch").and_then(|s| s.as_str()).and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let (c, r) = sk
        .sketch
        .entities
        .iter()
        .find_map(|e| match e {
            SketchEntity::Circle { c, r, .. } => sk.sketch.point_xy(c).map(|p| (p, *r)),
            _ => None,
        })
        .ok_or_else(|| FeatureError::new(HELIX_NEEDS_CIRCLE))?;
    let fr = sk.frame;
    let cw = fr.o + fr.x * c[0] + fr.y * c[1];
    let p = HelixParams {
        defined_by: match defined_by {
            DefinedBy::PitchRevs => ck_ops::helix::DefinedBy::PitchRevs,
            _ => ck_ops::helix::DefinedBy::parse(f.raw.get("definedBy").and_then(|v| v.as_str()).unwrap_or("")),
        },
        pitch: state.num(f, "pitch", 1.0)?,
        revolutions: state.num(f, "revolutions", 10.0)?,
        height: state.num(f, "height", 10.0)?,
        start_angle: state.num(f, "startAngle", 0.0)?,
        left_hand: *left_hand,
        reverse: *reverse,
    };
    let h = helix(&fr, cw, r, &p).map_err(|e| FeatureError::new(&e))?;
    let hj = Obj::new()
        .put("o", v3(h.o))
        .put("a", v3(h.a))
        .put("x", v3(h.x))
        .put("y", v3(h.y))
        .put("k", h.k)
        .put("r", h.r)
        .put("angle", h.angle)
        .build();
    let cj = Obj::new()
        .put("pts", Json::Arr(h.pts().into_iter().map(v3).collect()))
        .put("axis", false)
        .put("t0", 0.0)
        .put("t1", h.angle)
        .put("helix", hj)
        .build();
    state.result.curves.retain(|e| e.as_arr().and_then(|a| a.first()).and_then(|j| j.as_str()) != Some(f.id.as_str()));
    state.result.curves.push(Json::Arr(vec![Json::Str(f.id.clone()), cj]));
    br.helices.push((f.id.clone(), h));
    Ok(())
}
