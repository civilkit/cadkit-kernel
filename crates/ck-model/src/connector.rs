//! Mate Connector (`type: "connector"`) feature: named coordinate frame
//! `(origin, x, y, z)` anchored to a face, circle/cylinder, vertex, edge,
//! plane, another connector, or explicit frame, with optional local offset,
//! Euler rotation, Z-flip, Z-rotation, and `frozenFrame` survival.

use crate::expr;
use crate::regen::{self, codes, FeatureExec, FeatureError, RegenError, RegenState};
use crate::sketch::jtruthy;
use crate::{Feature, GeomRef};
use ck_core::linalg::{Frame, Mat3, Vec3};
use ck_json::Json;
use ck_ops::sweep::PathSeg;

fn eval_vec3_expr(j: Option<&Json>, vars: &expr::Vars) -> Result<Option<Vec3>, FeatureError> {
    let Some(arr) = j.and_then(|v| v.as_arr()) else { return Ok(None) };
    if arr.len() != 3 {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let x = expr::eval_field(Some(&arr[0]), vars, 0.0).map_err(FeatureError::new)?;
    let y = expr::eval_field(Some(&arr[1]), vars, 0.0).map_err(FeatureError::new)?;
    let z = expr::eval_field(Some(&arr[2]), vars, 0.0).map_err(FeatureError::new)?;
    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    Ok(Some(Vec3::new(x, y, z)))
}

fn parse_vec3_num(j: Option<&Json>) -> Option<Vec3> {
    let a = j?.as_arr()?;
    if a.len() != 3 {
        return None;
    }
    let (x, y, z) = (a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?);
    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
        return None;
    }
    Some(Vec3::new(x, y, z))
}

fn faces_of(j: &Json) -> Vec<String> {
    j.get("faces")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|f| f.as_str().map(|s| s.to_string())).collect())
        .unwrap_or_default()
}

fn up_frame_x(z: Vec3) -> Vec3 {
    if z.y.abs() > 0.99 {
        Vec3::X.reject(z).try_normalize().unwrap_or(Vec3::X)
    } else {
        Vec3::Y.cross(z).try_normalize().unwrap_or(Vec3::X)
    }
}

fn anchor_label(anchor: &Json) -> Option<String> {
    anchor
        .get("id")
        .or_else(|| anchor.get("face"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .or_else(|| {
            let fs = faces_of(anchor);
            if fs.is_empty() { None } else { Some(fs.join("+")) }
        })
}

fn resolve_anchor(state: &RegenState, exec: &mut dyn FeatureExec, anchor: &Json) -> Option<Frame> {
    let kind = anchor.get("kind").and_then(|v| v.as_str())?;
    match kind {
        "face" => {
            let id = anchor.get("id").or_else(|| anchor.get("face")).and_then(|v| v.as_str())?;
            exec.face_connector_frame(state, id).or_else(|| exec.face_frame(state, id))
        }
        "circle" => {
            if let Some(id) = anchor.get("face").or_else(|| anchor.get("id")).and_then(|v| v.as_str()) {
                return exec.face_connector_frame(state, id);
            }
            let faces = faces_of(anchor);
            if faces.is_empty() {
                return None;
            }
            let at = parse_vec3_num(anchor.get("at"));
            match exec.edge_seg(state, &faces, at)? {
                PathSeg::Arc { frame, .. } => Some(frame),
                _ => None,
            }
        }
        "vertex" => {
            let faces = faces_of(anchor);
            if faces.is_empty() {
                return None;
            }
            let at = parse_vec3_num(anchor.get("at"));
            let o = exec.vertex_point(state, &faces, at)?;
            if let Some(first) = faces.first().and_then(|f| exec.face_connector_frame(state, f)) {
                Some(Frame { o, x: first.x, y: first.y, z: first.z })
            } else {
                Some(Frame { o, x: Vec3::X, y: Vec3::Y, z: Vec3::Z })
            }
        }
        "edge" => {
            let faces = faces_of(anchor);
            if faces.is_empty() {
                return None;
            }
            let at = parse_vec3_num(anchor.get("at"));
            match exec.edge_seg(state, &faces, at)? {
                PathSeg::Line { a, b } => {
                    let o = (a + b).scale(0.5);
                    let z = (b - a).try_normalize()?;
                    let x = up_frame_x(z);
                    let y = z.cross(x);
                    Some(Frame { o, x, y, z })
                }
                PathSeg::Arc { frame, .. } => Some(frame),
                _ => None,
            }
        }
        "std" | "feature" => {
            let r = GeomRef::from_json(anchor)?;
            regen::resolve_plane(state, exec, &r)
        }
        "connector" => {
            let id = anchor.get("id").and_then(|v| v.as_str())?;
            state.connector(id)
        }
        "frame" => {
            let o = parse_vec3_num(anchor.get("origin").or_else(|| anchor.get("o"))).unwrap_or(Vec3::ZERO);
            let x_raw = parse_vec3_num(anchor.get("xDir").or_else(|| anchor.get("x")))?;
            let x = x_raw.try_normalize()?;
            if let Some(y_raw) = parse_vec3_num(anchor.get("yDir").or_else(|| anchor.get("y"))) {
                let y = y_raw.reject(x).try_normalize()?;
                let z = x.cross(y);
                Some(Frame { o, x, y, z })
            } else if let Some(z_raw) = parse_vec3_num(anchor.get("zDir").or_else(|| anchor.get("z"))) {
                let z = z_raw.reject(x).try_normalize()?;
                let y = z.cross(x);
                Some(Frame { o, x, y, z })
            } else {
                None
            }
        }
        _ => None,
    }
}

fn apply_modifiers(state: &RegenState, f: &Feature, mut fr: Frame) -> Result<Frame, FeatureError> {
    // 1. Local offset [dx, dy, dz]
    if let Some(off) = eval_vec3_expr(f.raw.get("offset"), &state.vars)? {
        fr.o = fr.o + fr.x.scale(off.x) + fr.y.scale(off.y) + fr.z.scale(off.z);
    }
    // 2. Local Euler rotations [rx, ry, rz] (degrees)
    if let Some(rot) = eval_vec3_expr(f.raw.get("rotate"), &state.vars)? {
        if rot.x.abs() > 0.0 {
            let m = Mat3::rotation(fr.x, rot.x.to_radians());
            fr.y = m.mul_vec(fr.y).normalize();
            fr.z = m.mul_vec(fr.z).normalize();
        }
        if rot.y.abs() > 0.0 {
            let m = Mat3::rotation(fr.y, rot.y.to_radians());
            fr.x = m.mul_vec(fr.x).normalize();
            fr.z = m.mul_vec(fr.z).normalize();
        }
        if rot.z.abs() > 0.0 {
            let m = Mat3::rotation(fr.z, rot.z.to_radians());
            fr.x = m.mul_vec(fr.x).normalize();
            fr.y = m.mul_vec(fr.y).normalize();
        }
    }
    // 3. Flip Z axis (negate x and z to preserve right-handedness)
    if jtruthy(f.raw.get("flip")) {
        fr.x = -fr.x;
        fr.z = -fr.z;
    }
    // 4. Rotate about local Z by rotateZ degrees (e.g. 90, 180, 270)
    if f.raw.get("rotateZ").is_some() {
        let rz = state.num(f, "rotateZ", 0.0)?;
        if !rz.is_finite() {
            return Err(FeatureError::new(codes::INVALID_VALUE));
        }
        if rz.abs() > 0.0 {
            let m = Mat3::rotation(fr.z, rz.to_radians());
            fr.x = m.mul_vec(fr.x).normalize();
            fr.y = m.mul_vec(fr.y).normalize();
        }
    }
    Ok(fr)
}

/// Execute a `connector` feature: resolve its anchor frame (or fall back to
/// `frozenFrame` with a `detachedReference` warning), apply local offset /
/// rotation / flip / rotateZ, and record it in `state.result.connectors`.
pub fn exec_connector(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<(), FeatureError> {
    let anchor = f.raw.get("anchor");
    let frame = match anchor.and_then(|a| resolve_anchor(state, exec, a)) {
        Some(base) => apply_modifiers(state, f, base)?,
        None => match regen::parse_frozen_frame(&f.raw) {
            Some(frozen) => {
                let label = anchor.and_then(anchor_label);
                state.result.warnings.push(RegenError {
                    feature: f.id.clone(),
                    error: regen::detached_ref_warning(label.as_deref()),
                });
                frozen
            }
            None => return Err(FeatureError::new(codes::MISSING_REFERENCE)),
        },
    };
    let name = if f.name.is_empty() { f.id.as_str() } else { f.name.as_str() };
    state.result.put_connector(&f.id, name, frame);
    Ok(())
}
