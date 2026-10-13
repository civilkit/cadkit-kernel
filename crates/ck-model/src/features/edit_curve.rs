//! `editCurve` (ck-model/src/features/edit_curve.rs): rebuild the picked
//! curves as one B-spline and shape it by its control points (ck-ops
//! `curve_edit`). Payload (client defaults): `curves[]` refs, `method`
//! `count | tolerance`, `degree` (5), `count`, `tolerance` (0.01),
//! `keepEnds` (true), `keepTangents`, `keepCorners`, `entries[]`, `planar`,
//! `plane`.
//!
//! Check order (oracle): no `curves` -> `editCurveNoInput`; a ref that does
//! not resolve -> `editCurveInputMissing {curve}`; then the ck-ops codes
//! (`editCurveDegree`, `editCurveNotChain`, `editCurveNotSmooth`,
//! `editCurveTooFewPoints`, `editCurveTooManyPoints`, `editCurveWeight`,
//! `editCurveNoPlane`).
//!
//! Output (oracle): `curves[] = [id, {pts, axis:false, t0:0, t1:1}]` and
//! `markers[] = [id, {degree, count, knots, cvs, weights, base, frames,
//! deviation, edited, devs, corners, entries}]`. Point references of entries
//! (`ref`) are not resolved: such an entry is skipped.

use super::curve_ref::ref_samples;
use super::pattern::plane_frame;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef};
use ck_core::linalg::Vec3;
use ck_json::Json;
use ck_ops::curve_edit::{edit_curve, EditError, EditOpts, EntryIn, PieceIn};

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

fn fe(e: EditError) -> FeatureError {
    let mut f = FeatureError::new(e.code);
    for (k, v) in e.params {
        // numeric parameters go out as numbers when they parse, like the UI placeholders expect
        f = f.with(k, Json::Str(v));
    }
    f
}

pub fn exec_edit_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let refs = f.raw.get("curves").and_then(|c| c.as_arr()).unwrap_or(&[]);
    if refs.is_empty() {
        return Err(FeatureError::new("editCurveNoInput"));
    }
    let mut pieces = vec![];
    for r in refs {
        match ref_samples(br, state, r) {
            Some(pts) => pieces.push(PieceIn { pts }),
            None => {
                let name = r.get_str("entity").or_else(|| r.get_str("id")).unwrap_or("?").to_string();
                return Err(FeatureError::new("editCurveInputMissing").with("curve", Json::Str(name)));
            }
        }
    }
    let flag = |k: &str, default: bool| match f.raw.get(k) {
        None | Some(Json::Null) => default,
        Some(_) => crate::sketch::jtruthy(f.raw.get(k)),
    };
    let degree = state.num(f, "degree", 5.0)?.round().max(0.0) as usize;
    let by_tolerance = f.raw.get_str("method") == Some("tolerance");
    let count = state.num(f, "count", (degree + 1) as f64)?.round().max(0.0) as usize;
    let tolerance = state.num(f, "tolerance", 0.01)?;
    let mut entries = vec![];
    for e in f.raw.get("entries").and_then(|a| a.as_arr()).unwrap_or(&[]) {
        let num = |k: &str| crate::expr::eval_field(e.get(k), &state.vars, 0.0).unwrap_or(0.0);
        entries.push(EntryIn {
            cv: e.get("cv").and_then(|c| c.as_f64()).unwrap_or(0.0).max(0.0) as usize,
            of: e.get("of").and_then(|c| c.as_f64()).map(|c| c as usize),
            curve_frame: e.get_str("frame") == Some("curve"),
            d: Vec3::new(num("dx"), num("dy"), num("dz")),
            weight: e.get("weight").and_then(|w| w.as_f64()),
            disabled: crate::sketch::jtruthy(e.get("disabled")),
            ref_point: None,
            ref_missing: e.get("ref").map_or(false, |r| !matches!(r, Json::Null)),
        });
    }
    let planar = flag("planar", false);
    let plane = f
        .raw
        .get("plane")
        .and_then(GeomRef::from_json)
        .and_then(|r| plane_frame(br, state, &r))
        .map(|fr| (fr.o, fr.z));
    let opts = EditOpts {
        degree,
        by_tolerance,
        count,
        tolerance,
        keep_ends: flag("keepEnds", true),
        keep_tangents: flag("keepTangents", false),
        keep_corners: flag("keepCorners", false),
        entries,
        plane,
        planar,
    };
    let out = edit_curve(&pieces, &opts).map_err(fe)?;
    let _ = codes::MISSING_REFERENCE;
    let arr = |v: &[Vec3]| Json::Arr(v.iter().map(|p| v3(*p)).collect());
    state.result.curves.retain(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
    state.result.curves.push(Json::Arr(vec![
        Json::Str(f.id.clone()),
        Json::Obj(vec![
            ("pts".into(), arr(&out.pts)),
            ("axis".into(), Json::Bool(false)),
            ("t0".into(), Json::Num(0.0)),
            ("t1".into(), Json::Num(1.0)),
        ]),
    ]));
    let ents: Vec<Json> = out
        .entries
        .iter()
        .map(|e| {
            let mut o = vec![("cv".to_string(), Json::Num(e.cv as f64))];
            if let Some(b) = e.before {
                o.push(("before".into(), v3(b)));
            }
            o.push(("applied".into(), Json::Bool(e.applied)));
            Json::Obj(o)
        })
        .collect();
    let marker = Json::Obj(vec![
        ("degree".into(), Json::Num(out.degree as f64)),
        ("count".into(), Json::Num(out.cvs.len() as f64)),
        ("knots".into(), Json::Arr(out.knots.iter().map(|k| Json::Num(*k)).collect())),
        ("cvs".into(), arr(&out.cvs)),
        ("weights".into(), Json::Arr(out.weights.iter().map(|k| Json::Num(*k)).collect())),
        ("base".into(), arr(&out.base)),
        ("frames".into(), Json::Arr(out.frames.iter().map(|fr| Json::Arr(fr.iter().map(|v| v3(*v)).collect())).collect())),
        ("deviation".into(), Json::Num((out.deviation * 1e6).round() / 1e6)),
        ("edited".into(), Json::Num((out.edited * 1e6).round() / 1e6)),
        ("devs".into(), Json::Arr(out.devs.iter().map(|d| Json::Arr(d.iter().map(|x| Json::Num(*x)).collect())).collect())),
        ("corners".into(), Json::Num(out.corners as f64)),
        ("entries".into(), Json::Arr(ents)),
    ]);
    state.result.markers.retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|j| j.as_str()) != Some(f.id.as_str()));
    state.result.markers.push(Json::Arr(vec![Json::Str(f.id.clone()), marker]));
    Ok(())
}
