//! `projectCurve` (ck-model/src/features/project_curve.rs): project a sketch
//! (or a planar curve feature) perpendicular to its plane onto faces — a 3D
//! curve on the surface (groove path, guide curve). Faces stay unsplit.
//!
//! Payload: `sketch` | `curve` (feature id of an earlier curve: editCurve,
//! helix, ...), `faces[]` (face names; empty = every face of the body),
//! `degree` (3), `tolerance` (0.001), `direction` (`both` | `forward` |
//! `reverse` — along / against the sketch plane normal).
//!
//! Check order (oracle): no `sketch` and no `curve` → `projectCurveNoInput`;
//! unresolved `sketch` → `missingSketch`; unresolved `curve` →
//! `missingReference`; no body or no live faces → `projectCurveNoFaces`;
//! a non-planar `curve` → `projectCurvePlane`; degree outside 2..=9 →
//! `editCurveDegree {min:2,max:9}`; no orthogonal hit on the picked faces →
//! `projectCurveMisses`; the projection lands on several faces / several
//! disjoint pieces → `projectCurveSeveral {count, points}`.
//!
//! Output (oracle): `curves[] = [id, {pts, axis:false, t0:0, t1:1}]` and
//! `markers[] = [id, {degree, count, deviation, length, pieces, corners,
//! closed}]`. `pieces` = number of source entities that projected; `corners`
//! = sharp bends in the projected polyline; `closed` = ends coincide.

use super::curve_ref::sketch_entity_samples;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::fit::{approximate_tol, Param};
use ck_json::Json;
use ck_ops::boolean::classify::{point_in_face, Where};
use ck_topo::body::{Body, FaceId};

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

/// Orthogonal projection of `p` onto face `fid` along unit `dir` (the sketch
/// plane normal, signed). `None` when the ray misses the face or lands
/// outside the trim.
fn project_point(body: &Body, fid: FaceId, p: Vec3, dir: Vec3) -> Option<Vec3> {
    let face = body.face(fid);
    match &face.surface {
        ck_geom::surface::Surface::Plane { frame } => {
            let denom = dir.dot(frame.z);
            if denom.abs() < 1e-12 {
                return None;
            }
            // t >= 0: only cast along dir, not against it (each dir is tried separately)
            let t = (frame.o - p).dot(frame.z) / denom;
            if t < -1e-9 {
                return None;
            }
            let q = p + dir * t.max(0.0);
            (point_in_face(body, fid, q, 1e-5) != Where::Out).then_some(q)
        }
        _ => {
            // non-planar: march a short way along dir, snap to the surface
            const STEPS: usize = 128;
            let span = body.bbox();
            let max_t = (span.1 - span.0).len().max(1.0);
            let mut prev_d = face.surface.distance(p);
            if prev_d < 1e-9 && point_in_face(body, fid, p, 1e-5) != Where::Out {
                return Some(p);
            }
            for i in 1..=STEPS {
                let t = max_t * i as f64 / STEPS as f64;
                let q = p + dir * t;
                let d = face.surface.distance(q);
                if d < 1e-9 {
                    return (point_in_face(body, fid, q, 1e-5) != Where::Out).then_some(q);
                }
                if prev_d > 1e-6 && d <= 1e-6 {
                    let (mut lo, mut hi) = (max_t * (i - 1) as f64 / STEPS as f64, t);
                    for _ in 0..30 {
                        let mid = 0.5 * (lo + hi);
                        if face.surface.distance(p + dir * mid) <= 1e-6 {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    let q = p + dir * (0.5 * (lo + hi));
                    return (face.surface.distance(q) < 1e-5 && point_in_face(body, fid, q, 1e-5) != Where::Out).then_some(q);
                }
                prev_d = d;
            }
            None
        }
    }
}

/// Project one polyline onto one face along one direction. Returns every
/// maximal run of consecutive hits (oracle splits at misses).
fn project_runs(body: &Body, fid: FaceId, src: &[Vec3], dir: Vec3) -> Vec<Vec<Vec3>> {
    let mut runs: Vec<Vec<Vec3>> = Vec::new();
    let mut cur: Vec<Vec3> = Vec::new();
    for &p in src {
        match project_point(body, fid, p, dir) {
            Some(q) => cur.push(q),
            None => {
                if !cur.is_empty() {
                    runs.push(std::mem::take(&mut cur));
                }
            }
        }
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    runs
}

/// Sharp bends in the projected polyline (direction change > ~30°).
fn count_corners(pts: &[Vec3]) -> usize {
    let mut n = 0;
    for w in pts.windows(3) {
        let a = (w[1] - w[0]).try_normalize();
        let b = (w[2] - w[1]).try_normalize();
        if let (Some(a), Some(b)) = (a, b) {
            if a.dot(b) < 0.866 {
                n += 1;
            }
        }
    }
    n
}

fn poly_len(pts: &[Vec3]) -> f64 {
    pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// Is every sample within `tol` of one plane? Returns that plane when yes.
fn fit_plane(pts: &[Vec3]) -> Option<Frame> {
    if pts.len() < 3 {
        return None;
    }
    let o = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    let mut n = Vec3::ZERO;
    for w in pts.windows(2) {
        n = n + (w[1] - o).cross(w[0] - o);
    }
    if n.len() < 1e-12 {
        return None;
    }
    let n = n.try_normalize()?;
    let max_d = pts.iter().map(|p| (*p - o).dot(n).abs()).fold(0.0, f64::max);
    if max_d > 1e-4 {
        return None;
    }
    let z = n;
    let x = (pts[1] - pts[0]).try_normalize().unwrap_or(Vec3::X);
    let y = z.cross(x).try_normalize().unwrap_or(Vec3::Y);
    let x = y.cross(z).try_normalize()?;
    Some(Frame { o, x, y, z })
}

pub fn exec_project_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let sketch = f.raw.get_str("sketch");
    let curve = f.raw.get_str("curve");
    if sketch.is_none() && curve.is_none() {
        return Err(FeatureError::new("projectCurveNoInput"));
    }
    // source: one polyline per sketch entity (chained), or the curve feature's pts
    let (pieces, plane): (Vec<Vec3>, Frame) = if let Some(sk_id) = sketch {
        let sk = state.sketch(sk_id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let mut all: Vec<Vec3> = vec![];
        let mut entities = 0;
        for e in &sk.sketch.entities {
            if let Some(pts) = sketch_entity_samples(sk, e.id()) {
                if !all.is_empty() {
                    if pts.first().map_or(false, |p| all.last().map_or(false, |q| p.dist(*q) < 1e-9)) {
                        all.extend_from_slice(&pts[1..]);
                    } else {
                        all.extend_from_slice(&pts);
                    }
                } else {
                    all.extend_from_slice(&pts);
                }
                entities += 1;
            }
        }
        if entities == 0 || all.len() < 2 {
            return Err(FeatureError::new("projectCurveMisses"));
        }
        let _ = entities;
        (all, sk.frame)
    } else {
        let id = curve.unwrap();
        let c = state
            .result
            .curves
            .iter()
            .find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id))
            .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
        let pts_json = c.as_arr().and_then(|a| a.get(1)).and_then(|o| o.get("pts")).and_then(|p| p.as_arr());
        let Some(pts_json) = pts_json else {
            return Err(FeatureError::new(codes::MISSING_REFERENCE));
        };
        let pts: Vec<Vec3> = pts_json
            .iter()
            .filter_map(|p| {
                let a = p.as_arr()?;
                Some(Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?))
            })
            .collect();
        if pts.len() < 2 {
            return Err(FeatureError::new("projectCurveMisses"));
        }
        let frame = fit_plane(&pts).ok_or_else(|| FeatureError::new("projectCurvePlane"))?;
        (pts, frame)
    };
    let normal = plane.z.try_normalize().ok_or_else(|| FeatureError::new("projectCurveMisses"))?;

    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("projectCurveNoFaces"))?;
    let face_names: Vec<String> = f
        .raw
        .get("faces")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    let mut faces: Vec<FaceId> = Vec::new();
    if face_names.is_empty() {
        faces.extend(body.face_ids());
    } else {
        for name in &face_names {
            faces.push(body.find_face(name).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?);
        }
    }
    if faces.is_empty() {
        return Err(FeatureError::new("projectCurveNoFaces"));
    }

    let degree = state.num(f, "degree", 3.0)?.round() as usize;
    if !(2..=9).contains(&degree) {
        return Err(FeatureError::new("editCurveDegree").with("min", Json::Num(2.0)).with("max", Json::Num(9.0)));
    }
    let tolerance = state.num(f, "tolerance", 0.001)?.max(1e-9);
    let direction = f.raw.get_str("direction").unwrap_or("both");

    let dirs: &[Vec3] = match direction {
        "forward" => &[normal],
        "reverse" => &[-normal],
        _ => &[normal, -normal],
    };

    // For each (face, direction) that fully covers the source, record a hit.
    // A full hit on exactly one (face, direction) is the result; more is Several.
    let mut full: Vec<Vec<Vec3>> = vec![];
    let mut partial_runs: Vec<Vec<Vec3>> = vec![];
    for &d in dirs {
        for &fid in &faces {
            for run in project_runs(body, fid, &pieces, d) {
                if run.len() == pieces.len() && run.len() >= 2 {
                    full.push(run);
                } else if run.len() >= 2 {
                    partial_runs.push(run);
                }
            }
        }
    }
    if full.len() == 1 {
        let hits = full.pop().unwrap();
        let pieces_n = if sketch.is_some() {
            state
                .sketch(sketch.unwrap())
                .map(|sk| sk.sketch.entities.iter().filter(|e| sketch_entity_samples(sk, e.id()).is_some()).count())
                .unwrap_or(1)
        } else {
            1
        };
        let corners = count_corners(&hits);
        let closed = hits.first().map_or(false, |a| hits.last().map_or(false, |b| a.dist(*b) < 1e-6));
        let length = poly_len(&hits);
        let curve = approximate_tol(&hits, degree, tolerance, Param::ChordLength)
            .map_err(|_| FeatureError::new("projectCurveFailed").with("detail", Json::Str("fit failed".into())))?;
        let (u0, u1) = curve.domain();
        let n_out = hits.len().clamp(16, 512);
        let pts: Vec<Vec3> = (0..=n_out).map(|i| curve.eval(u0 + (u1 - u0) * i as f64 / n_out as f64)).collect();
        let count = curve.ctrl.len();
        let deviation = ck_geom::fit::max_deviation(&curve, &hits);

        state
            .result
            .curves
            .retain(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
        state.result.curves.push(Json::Arr(vec![
            Json::Str(f.id.clone()),
            Json::Obj(vec![
                ("pts".into(), Json::Arr(pts.iter().map(|p| v3(*p)).collect())),
                ("axis".into(), Json::Bool(false)),
                ("t0".into(), Json::Num(0.0)),
                ("t1".into(), Json::Num(1.0)),
            ]),
        ]));
        state
            .result
            .markers
            .retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
        state.result.markers.push(Json::Arr(vec![
            Json::Str(f.id.clone()),
            Json::Obj(vec![
                ("degree".into(), Json::Num(degree as f64)),
                ("count".into(), Json::Num(count as f64)),
                ("deviation".into(), Json::Num(deviation)),
                ("length".into(), Json::Num(length)),
                ("pieces".into(), Json::Num(pieces_n as f64)),
                ("corners".into(), Json::Num(corners as f64)),
                ("closed".into(), Json::Bool(closed)),
            ]),
        ]));
        return Ok(());
    }
    if full.len() > 1 {
        // several faces / directions each got the whole curve
        let points: Vec<Json> = full.iter().flat_map(|h| [h[0], h[h.len() - 1]]).map(v3).collect();
        return Err(
            FeatureError::new("projectCurveSeveral").with("count", Json::Num(full.len() as f64)).with("points", Json::Arr(points)),
        );
    }
    if !partial_runs.is_empty() {
        // the curve leaves the face / crosses several faces: report every run
        let points: Vec<Json> = partial_runs.iter().flat_map(|h| [h[0], h[h.len() - 1]]).map(v3).collect();
        return Err(FeatureError::new("projectCurveSeveral")
            .with("count", Json::Num(partial_runs.len() as f64))
            .with("points", Json::Arr(points)));
    }
    Err(FeatureError::new("projectCurveMisses"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regen::{regen, RegenOpts, RegenResult};
    use crate::Part;

    fn run(src: &str) -> RegenResult {
        let part = Part::parse(src).expect("parse");
        regen(&part, &RegenOpts::default(), &mut Brep::new())
    }

    fn err_code(r: RegenResult) -> String {
        assert_eq!(r.errors.len(), 1, "one error: {:?}", r.errors);
        r.errors[0].error.code.clone()
    }

    const BOX_LINE: &str = r#"{
      "format": "ck-part",
      "features": [
        { "type": "sketch", "id": "F1", "plane": { "kind": "std", "id": "front" },
          "points": [{ "id": "p1", "x": 0, "y": 0 }, { "id": "p2", "x": 40, "y": 0 },
                     { "id": "p3", "x": 40, "y": 20 }, { "id": "p4", "x": 0, "y": 20 }],
          "entities": [
            { "type": "line", "id": "e1", "p1": "p1", "p2": "p2" },
            { "type": "line", "id": "e2", "p1": "p2", "p2": "p3" },
            { "type": "line", "id": "e3", "p1": "p3", "p2": "p4" },
            { "type": "line", "id": "e4", "p1": "p4", "p2": "p1" }],
          "constraints": [] },
        { "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10 },
        { "type": "plane", "id": "F6", "mode": "offset", "base": { "kind": "std", "id": "front" }, "offset": 15 },
        { "type": "sketch", "id": "F5", "plane": { "kind": "feature", "id": "F6" },
          "points": [{ "id": "a", "x": 5, "y": 5 }, { "id": "b", "x": 30, "y": 15 }],
          "entities": [{ "type": "line", "id": "e1", "p1": "a", "p2": "b" }],
          "constraints": [] }
      ]
    }"#;

    fn with_project(extra: &str) -> String {
        // BOX_LINE ends with `      ]\n    }` — insert the feature before the array closes
        let mut s = BOX_LINE.to_string();
        let close = s.rfind("]").expect("features array close");
        s.insert_str(close, &format!(",\n        {extra}"));
        s
    }

    #[test]
    fn no_input() {
        let r = run(&with_project(r#"{ "type": "projectCurve", "id": "F7" }"#));
        assert_eq!(err_code(r), "projectCurveNoInput");
    }

    #[test]
    fn misses_when_forward_only() {
        let r = run(&with_project(
            r#"{ "type": "projectCurve", "id": "F7", "sketch": "F5", "faces": ["F2.C1"], "direction": "forward" }"#,
        ));
        assert_eq!(err_code(r), "projectCurveMisses");
    }

    #[test]
    fn hits_top_face_reverse() {
        let r = run(&with_project(
            r#"{ "type": "projectCurve", "id": "F7", "sketch": "F5", "faces": ["F2.C1"], "direction": "reverse" }"#,
        ));
        assert!(r.errors.is_empty(), "errors: {:?}", r.errors);
        let curve = r
            .curves
            .iter()
            .find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some("F7"))
            .expect("curve F7");
        let pts = curve.as_arr().and_then(|a| a.get(1)).and_then(|o| o.get("pts")).and_then(|p| p.as_arr()).unwrap();
        assert!(pts.len() >= 2);
        // all projected points sit on the top face plane z = 10
        for p in pts {
            let a = p.as_arr().unwrap();
            assert!((a[2].as_f64().unwrap() - 10.0).abs() < 1e-6, "z should be 10");
        }
        let marker = r
            .markers
            .iter()
            .find(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some("F7"))
            .expect("marker");
        let obj = marker.as_arr().and_then(|a| a.get(1)).unwrap();
        assert_eq!(obj.get("closed").and_then(|c| c.as_bool()), Some(false));
    }

    #[test]
    fn several_when_empty_faces() {
        // empty faces = every body face: both top and bottom receive the projection
        let r = run(&with_project(r#"{ "type": "projectCurve", "id": "F7", "sketch": "F5", "faces": [] }"#));
        assert_eq!(err_code(r), "projectCurveSeveral");
    }

    #[test]
    fn missing_sketch() {
        let r = run(&with_project(
            r#"{ "type": "projectCurve", "id": "F7", "sketch": "F9", "faces": ["F2.C1"] }"#,
        ));
        assert_eq!(err_code(r), "missingSketch");
    }

    #[test]
    fn degree_bounds() {
        let r = run(&with_project(
            r#"{ "type": "projectCurve", "id": "F7", "sketch": "F5", "faces": ["F2.C1"], "degree": 1 }"#,
        ));
        assert_eq!(err_code(r), "editCurveDegree");
    }
}
