//! Curve references as dense sample polylines (input of `editCurve`).
//!
//! Reference forms (client `curveRef`): `{kind:"sketchCurve", sketch, entity}`,
//! `{kind:"edge", faces, at}` (a body edge) and `{kind:"curve", id}` (the
//! display polyline of an earlier curve feature: editCurve, helix, ...).

use super::surfaces::pick_edge;
use super::Brep;
use crate::regen::{RegenState, SketchResult};
use crate::SketchEntity;
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_json::Json;
use std::f64::consts::TAU;

/// Samples of a curved piece (a line needs only its ends).
const CURVED: usize = 1024;
const LINE: usize = 16;

fn norm_sweep(mut o: f64) -> f64 {
    while o <= 1e-12 {
        o += TAU;
    }
    while o > TAU + 1e-12 {
        o -= TAU;
    }
    o
}

/// Samples of sketch entity `id` on the sketch plane (`None`: unknown entity,
/// a point, or degenerate geometry).
pub fn sketch_entity_samples(sk: &SketchResult, id: &str) -> Option<Vec<Vec3>> {
    let frame = &sk.frame;
    let s = &sk.sketch;
    let to3 = |p: [f64; 2]| frame.o + frame.x * p[0] + frame.y * p[1];
    let pt = |p: &str| s.point_xy(p).map(to3);
    let e = s.entity(id)?;
    let sample = |n: usize, f: &dyn Fn(f64) -> Vec3| (0..=n).map(|i| f(i as f64 / n as f64)).collect::<Vec<_>>();
    match e {
        SketchEntity::Line { p1, p2, .. } => {
            let (a, b) = (pt(p1)?, pt(p2)?);
            if a.dist(b) < 1e-12 {
                return None;
            }
            Some(sample(LINE, &|t| a + (b - a) * t))
        }
        SketchEntity::Circle { c, r, .. } => {
            let c = pt(c)?;
            let r = r.abs();
            Some(sample(CURVED, &|t| c + frame.x * (r * (TAU * t).cos()) + frame.y * (r * (TAU * t).sin())))
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (cc, a, b) = (s.point_xy(c)?, s.point_xy(p1)?, s.point_xy(p2)?);
            let a0 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
            let sw = norm_sweep((b[1] - cc[1]).atan2(b[0] - cc[0]) - a0);
            let r = (a[0] - cc[0]).hypot(a[1] - cc[1]);
            let c3 = to3(cc);
            let n = ((CURVED as f64 * sw / TAU).ceil() as usize).max(64);
            Some(sample(n, &|t| {
                let ang = a0 + sw * t;
                c3 + frame.x * (r * ang.cos()) + frame.y * (r * ang.sin())
            }))
        }
        SketchEntity::Ellipse { c, p, r, .. } => {
            let (cc, pp) = (s.point_xy(c)?, s.point_xy(p)?);
            let u = [pp[0] - cc[0], pp[1] - cc[1]];
            let lu = u[0].hypot(u[1]);
            if lu < 1e-12 {
                return None;
            }
            let x = frame.x * (u[0] / lu) + frame.y * (u[1] / lu);
            let y = frame.x * (-u[1] / lu) + frame.y * (u[0] / lu);
            let c3 = to3(cc);
            let r = r.abs();
            Some(sample(CURVED, &|t| c3 + x * (lu * (TAU * t).cos()) + y * (r * (TAU * t).sin())))
        }
        SketchEntity::CvSpline { pts, degree, knots, closed, .. } => {
            let p = (*degree as usize).max(1);
            let ctrl: Vec<Vec3> = pts.iter().filter_map(|q| pt(q)).collect();
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
            let b = ck_geom::nurbs::bspline::BSplineCurve::new(p, k, ctrl).ok()?;
            let (a, z) = b.domain();
            Some(sample(CURVED, &|t| b.eval(a + (z - a) * t)))
        }
        SketchEntity::Spline { pts, closed, .. } => {
            let mut p3: Vec<Vec3> = pts.iter().filter_map(|q| pt(q)).collect();
            if *closed && !p3.is_empty() {
                p3.push(p3[0]);
            }
            let sp = ck_ops::spline_path::SplinePath::new(&p3, None, None)?;
            let len = sp.length();
            Some(sample(CURVED, &|t| sp.eval(len * t)))
        }
        SketchEntity::Point { .. } => None,
    }
}

/// Samples of the curve a `curves[]` entry refers to; `None` when it does not
/// resolve.
pub fn ref_samples(br: &Brep, state: &RegenState, r: &Json) -> Option<Vec<Vec3>> {
    match r.get_str("kind")? {
        "sketchCurve" => {
            let sk = state.sketch(r.get_str("sketch")?)?;
            sketch_entity_samples(sk, r.get_str("entity")?)
        }
        "edge" => {
            let faces: Vec<String> = r.get("faces")?.as_arr()?.iter().filter_map(|f| f.as_str().map(str::to_string)).collect();
            let at = r.get("at").and_then(|a| a.as_arr()).and_then(|a| {
                (a.len() == 3).then(|| Vec3::new(a[0].as_f64().unwrap_or(0.0), a[1].as_f64().unwrap_or(0.0), a[2].as_f64().unwrap_or(0.0)))
            });
            let (slot, e) = pick_edge(br, &faces, at).ok()?;
            let b = br.slot_body(slot)?;
            let ed = b.edge(e);
            let n = if matches!(ed.curve, Curve::Line { .. }) { LINE } else { CURVED };
            Some((0..=n).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / n as f64)).collect())
        }
        "curve" => {
            let id = r.get_str("id")?;
            let c = state.result.curves.iter().find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id))?;
            let pts = c.as_arr()?.get(1)?.get("pts")?.as_arr()?;
            let v: Vec<Vec3> = pts
                .iter()
                .filter_map(|p| {
                    let a = p.as_arr()?;
                    Some(Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?))
                })
                .collect();
            (v.len() >= 2).then_some(v)
        }
        _ => None,
    }
}
