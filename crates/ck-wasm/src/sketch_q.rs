//! Sketch-backed halves of `refCurve` (and, later, `convert`): they read the
//! solved sketches a regen leaves in its snapshot.

use ck_core::linalg::{Frame, Vec3};
use ck_geom::convert::{conic_arc, curve_to_nurbs};
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

/// Exact NURBS of sketch entity `id`, lifted onto the sketch plane. Returns
/// (curve, closed). `None` for unknown / unsupported entities (points,
/// interpolating splines, degenerate geometry).
pub fn entity_nurbs(sk: &Sketch, frame: &Frame, id: &str) -> Option<(BSplineCurve, bool)> {
    let e = sk.entity(id)?;
    let pt = |p: &str| sk.point_xy(p);
    let local = |c: [f64; 2], x: Vec3, y: Vec3| Frame { o: to3(frame, c), x, y, z: x.cross(y) };
    Some(match e {
        SketchEntity::Line { p1, p2, .. } => {
            let (a, b) = (to3(frame, pt(p1)?), to3(frame, pt(p2)?));
            let c = Curve::line_through(a, b);
            (curve_to_nurbs(&c, 0.0, (b - a).len()), false)
        }
        SketchEntity::Circle { c, r, .. } => {
            let f = local(pt(c)?, frame.x, frame.y);
            (conic_arc(&f, r.abs(), r.abs(), 0.0, TAU), true)
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (cc, a, b) = (pt(c)?, pt(p1)?, pt(p2)?);
            let a0 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
            let sw = norm_sweep((b[1] - cc[1]).atan2(b[0] - cc[0]) - a0);
            let r = (a[0] - cc[0]).hypot(a[1] - cc[1]);
            let f = local(cc, frame.x, frame.y);
            (conic_arc(&f, r, r, a0, a0 + sw), (sw - TAU).abs() < 1e-9)
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
            let f = local(cc, x, y);
            (conic_arc(&f, lu, r.abs(), 0.0, TAU), true)
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
            (BSplineCurve::new(p, k, ctrl).ok()?, false)
        }
        SketchEntity::Spline { .. } | SketchEntity::Point { .. } => return None,
    })
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
