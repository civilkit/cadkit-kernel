//! Sketch-backed halves of `refCurve` (and, later, `convert`): they read the
//! solved sketches a regen leaves in its snapshot.

use ck_core::linalg::{Frame, Vec3};
use ck_geom::convert::curve_to_nurbs;
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_model::regen::SketchResult;
use ck_model::{Sketch, SketchEntity};
use std::f64::consts::TAU;

/// Sweep of an arc in (0, 2pi] (client `tn()`).
fn norm_sweep(mut o: f64) -> f64 {
    while o <= 1e-12 {
        o += TAU;
    }
    while o > TAU + 1e-12 {
        o -= TAU;
    }
    o
}

fn to3(f: &Frame, p: [f64; 2]) -> Vec3 {
    f.o + f.x * p[0] + f.y * p[1]
}

/// 3D curve of sketch entity `id` on the sketch plane, with its parameter
/// range and closedness. `None` for unknown / unsupported entities (points,
/// interpolating splines, degenerate geometry).
pub fn entity_curve(sk: &Sketch, frame: &Frame, id: &str) -> Option<(Curve, f64, f64, bool)> {
    let e = sk.entity(id)?;
    let pt = |p: &str| sk.point_xy(p);
    let local = |c: [f64; 2], x: Vec3, y: Vec3| Frame { o: to3(frame, c), x, y, z: x.cross(y) };
    Some(match e {
        SketchEntity::Line { p1, p2, .. } => {
            let (a, b) = (to3(frame, pt(p1)?), to3(frame, pt(p2)?));
            (Curve::line_through(a, b), 0.0, (b - a).len(), false)
        }
        SketchEntity::Circle { c, r, .. } => {
            (Curve::Circle { frame: local(pt(c)?, frame.x, frame.y), r: r.abs() }, 0.0, TAU, true)
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (cc, a, b) = (pt(c)?, pt(p1)?, pt(p2)?);
            let a0 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
            let sw = norm_sweep((b[1] - cc[1]).atan2(b[0] - cc[0]) - a0);
            let r = (a[0] - cc[0]).hypot(a[1] - cc[1]);
            (Curve::Circle { frame: local(cc, frame.x, frame.y), r }, a0, a0 + sw, (sw - TAU).abs() < 1e-9)
        }
        SketchEntity::Ellipse { c, p, r, .. } => {
            let (cc, pp) = (pt(c)?, pt(p)?);
            let u = [pp[0] - cc[0], pp[1] - cc[1]];
            let lu = u[0].hypot(u[1]);
            if lu < 1e-12 {
                return None;
            }
            let x = frame.x * (u[0] / lu) + frame.y * (u[1] / lu);
            let y = frame.x * (-u[1] / lu) + frame.y * (u[0] / lu);
            (Curve::Ellipse { frame: local(cc, x, y), a: lu, b: r.abs() }, 0.0, TAU, true)
        }
        SketchEntity::CvSpline { pts, degree, knots, closed, .. } => {
            let p = (*degree as usize).max(1);
            let ctrl: Vec<Vec3> = pts.iter().filter_map(|q| pt(q)).map(|q| to3(frame, q)).collect();
            if *closed || ctrl.len() < p + 1 {
                return None;
            }
            let k = match knots {
                Some(k) if k.len() == ctrl.len() + p + 1 => k.clone(),
                _ => {
                    let spans = ctrl.len() - p;
                    let mut k = vec![0.0; p + 1];
                    for i in 1..spans {
                        k.push(i as f64 / spans as f64);
                    }
                    k.extend(std::iter::repeat(1.0).take(p + 1));
                    k
                }
            };
            let b = BSplineCurve::new(p, k, ctrl).ok()?;
            let (t0, t1) = b.domain();
            (Curve::BSpline(b), t0, t1, false)
        }
        SketchEntity::Spline { .. } | SketchEntity::Point { .. } | SketchEntity::Text { .. } => return None,
    })
}

/// Exact NURBS of sketch entity `id` (see [`entity_curve`]); also `closed`.
pub fn entity_nurbs(sk: &Sketch, frame: &Frame, id: &str) -> Option<(BSplineCurve, bool)> {
    let (c, t0, t1, closed) = entity_curve(sk, frame, id)?;
    Some((curve_to_nurbs(&c, t0, t1), closed))
}

/// `{kind:"sketchCurve", sketch, entity}` of `refCurve`.
pub fn ref_curve(sketches: &[SketchResult], r: &ck_json::Json) -> ck_json::Json {
    let found = (|| {
        let s = sketches.iter().find(|s| Some(s.id.as_str()) == r.get_str("sketch"))?;
        entity_nurbs(&s.sketch, &s.frame, r.get_str("entity")?)
    })();
    match found {
        Some((c, closed)) => ck_ops::ref_curve::curve_reply(&c, closed),
        None => ck_ops::ref_curve::missing_reference(),
    }
}

/// `convert` with `{kind:"sketchCurve", sketch, entity}`: the entity
/// re-projected into the target frame.
pub fn convert(sketches: &[SketchResult], msg: &ck_json::Json) -> ck_json::Json {
    use ck_json::{Json, Obj};
    let r = msg.get("ref").unwrap_or(&Json::Null);
    let (Some(f), Some(found)) = (msg.get("frame").and_then(ck_ops::convert_proj::frame_of), (|| {
        let s = sketches.iter().find(|s| Some(s.id.as_str()) == r.get_str("sketch"))?;
        entity_curve(&s.sketch, &s.frame, r.get_str("entity")?)
    })()) else {
        return Json::Null;
    };
    let (c, t0, t1, closed) = found;
    let from = Obj::new()
        .put("kind", "sketchCurve")
        .put("sketch", r.get_str("sketch").unwrap_or(""))
        .put("entity", r.get_str("entity").unwrap_or(""))
        .build();
    let items: Vec<_> = ck_ops::convert_proj::project_curve(&c, t0, t1, closed, &f)
        .map(|ent| ck_ops::convert_proj::Item { ent, from })
        .into_iter()
        .collect();
    let skipped = 1 - items.len();
    ck_ops::convert_proj::finish(msg.get("sketch").unwrap_or(&Json::Null), items, skipped)
}
