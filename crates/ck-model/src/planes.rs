//! Reference-plane modes driven by points, lines, curves and faces
//! (`threePoints`, `parallel`, `linePoint`, `normal`, `normalCurve`,
//! `curve`, `tangent`) and the `axis` feature, plus the axis-reference
//! resolver shared with `angle` / `perpendicular`.
//!
//! Frame conventions are pinned against the original kernel (see
//! `plane_modes_match_oracle` and `tools/plane_cmp.mjs`): a plane's `flip`
//! negates `x` and `z` (`normal` negates `z` and `y` only), the "up = Y"
//! frame rule is `x = Y × z` (`X` when degenerate).
//!
//! Known deviations (correct instead of identical): `normalCurve` /
//! `tangent` with a point project to the true nearest point (the original
//! lands on a curve end for arcs); `tangent`-to-line on a cylinder puts
//! the origin at the foot of the axis origin on the line.

use crate::regen::{codes, FaceSurf, FeatureExec, FeatureError, RegenState};
use crate::sketch::SketchEntity;
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_json::Json;
use ck_ops::sweep::PathSeg;
use std::f64::consts::TAU;

pub const MISSING_PATH: &str = "missingPath";
pub const PLANE_POINTS_IN_LINE: &str = "planePointsInLine";
pub const PLANE_POINT_ON_LINE: &str = "planePointOnLine";
pub const PLANE_NOT_TANGENT: &str = "planeNotTangent";
pub const PLANES_PARALLEL: &str = "planesParallel";
pub const CURVES_NOT_PLANAR: &str = "curvesNotPlanar";
pub const CURVES_STRAIGHT: &str = "curvesStraight";
pub const FACE_NOT_CYLINDRICAL: &str = "faceNotCylindrical";

fn err(code: &str) -> FeatureError {
    FeatureError::new(code)
}
fn missing() -> FeatureError {
    err(codes::MISSING_REFERENCE)
}

fn kind_of(j: &Json) -> Option<&str> {
    j.get("kind").and_then(|k| k.as_str())
}

fn vec3_of(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    if a.len() != 3 {
        return None;
    }
    Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?))
}

fn faces_of(j: &Json) -> Vec<String> {
    j.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|f| f.as_str().map(|s| s.to_string())).collect()).unwrap_or_default()
}

fn world(fr: &Frame, p: [f64; 2]) -> Vec3 {
    fr.o + fr.x * p[0] + fr.y * p[1]
}

/// `x = Y × z`, or `X` when `z` is along Y: the frame used for planes
/// normal to a line / sphere.
fn up_frame_x(z: Vec3) -> Vec3 {
    Vec3::Y.cross(z).try_normalize().unwrap_or(Vec3::X)
}

fn flipped(fr: Frame) -> Frame {
    Frame { o: fr.o, x: -fr.x, y: fr.y, z: -fr.z }
}

// ---- point references ----

/// World position of a point reference (`sketchPoint` or `vertex`).
fn resolve_point(state: &RegenState, exec: &mut dyn FeatureExec, j: &Json) -> Option<Vec3> {
    match kind_of(j)? {
        "sketchPoint" => {
            let sk = state.sketch(j.get("sketch")?.as_str()?)?;
            let p = sk.sketch.point_xy(j.get("point")?.as_str()?)?;
            Some(world(&sk.frame, p))
        }
        "vertex" => {
            let faces = faces_of(j);
            if faces.is_empty() {
                return None;
            }
            exec.vertex_point(state, &faces, j.get("at").and_then(vec3_of))
        }
        _ => None,
    }
}

fn points_of(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Vec<Vec3> {
    let Some(a) = f.raw.get("points").and_then(|p| p.as_arr()) else { return vec![] };
    a.iter().map_while(|j| resolve_point(state, exec, j)).collect()
}

// ---- curve references ----

/// A referenced curve: a path piece plus the orientation hint of the sketch
/// it lives in (frame axes; origin unused).
#[derive(Clone)]
pub struct Piece {
    pub seg: PathSeg,
    pub hint: Option<Frame>,
    pub circle: bool,
    /// Outward normal of a planar face the (edge) curve bounds.
    pub cap_normal: Option<Vec3>,
}

fn entity_piece(sk: &crate::regen::SketchResult, id: &str) -> Option<Piece> {
    let fr = sk.frame;
    let s = &sk.sketch;
    let hint = Some(fr);
    match s.entity(id)? {
        SketchEntity::Line { p1, p2, .. } => {
            let (a, b) = (s.point_xy(p1)?, s.point_xy(p2)?);
            Some(Piece { seg: PathSeg::Line { a: world(&fr, a), b: world(&fr, b) }, hint, circle: false, cap_normal: None })
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (c, a, b) = (s.point_xy(c)?, s.point_xy(p1)?, s.point_xy(p2)?);
            let (cw, aw) = (world(&fr, c), world(&fr, a));
            let r = (aw - cw).len();
            let x = (aw - cw).try_normalize()?;
            let frame = Frame { o: cw, x, y: fr.z.cross(x), z: fr.z };
            let ang = ((b[1] - c[1]).atan2(b[0] - c[0]) - (a[1] - c[1]).atan2(a[0] - c[0])).rem_euclid(TAU);
            Some(Piece { seg: PathSeg::Arc { frame, r, a0: 0.0, a1: if ang < 1e-12 { TAU } else { ang } }, hint, circle: false, cap_normal: None })
        }
        SketchEntity::Circle { c, r, .. } => {
            let c = s.point_xy(c)?;
            let frame = Frame { o: world(&fr, c), x: fr.x, y: fr.y, z: fr.z };
            Some(Piece { seg: PathSeg::Arc { frame, r: *r, a0: 0.0, a1: TAU }, hint, circle: true, cap_normal: None })
        }
        _ => None,
    }
}

fn resolve_curve(state: &RegenState, exec: &mut dyn FeatureExec, j: &Json) -> Option<Vec<Piece>> {
    match kind_of(j)? {
        "sketchCurve" => {
            let sk = state.sketch(j.get("sketch")?.as_str()?)?;
            Some(vec![entity_piece(sk, j.get("entity")?.as_str()?)?])
        }
        "curve" => {
            let seg = crate::features::curve_ref::curve_path(state, j.get("id")?.as_str()?)?;
            Some(vec![Piece { seg, hint: None, circle: false, cap_normal: None }])
        }
        "edge" => {
            let seg = exec.edge_seg(state, &faces_of(j), j.get("at").and_then(vec3_of))?;
            let circle = matches!(&seg, PathSeg::Arc { a0, a1, .. } if (a1 - a0).abs() > TAU - 1e-9);
            let cap_normal = faces_of(j).iter().find_map(|f| exec.face_frame(state, f)).map(|fr| fr.z);
            Some(vec![Piece { seg, hint: None, circle, cap_normal }])
        }
        _ => None,
    }
}

fn curves_of(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<Vec<Piece>, FeatureError> {
    let a = f.raw.get("curves").and_then(|c| c.as_arr()).ok_or_else(missing)?;
    let mut out = vec![];
    for j in a {
        out.extend(resolve_curve(state, exec, j).ok_or_else(missing)?);
    }
    if out.is_empty() {
        return Err(missing());
    }
    Ok(out)
}

fn arc_pt(f: &Frame, r: f64, t: f64) -> Vec3 {
    f.o + (f.x * t.cos() + f.y * t.sin()) * r
}

/// Point at the start / end of a piece and the unit travel direction there.
fn end_state(seg: &PathSeg, at_end: bool) -> Option<(Vec3, Vec3)> {
    match seg {
        PathSeg::Line { a, b } => {
            let d = (*b - *a).try_normalize()?;
            Some((if at_end { *b } else { *a }, d))
        }
        PathSeg::Arc { frame, r, a0, a1 } => {
            let t = if at_end { *a1 } else { *a0 };
            let s = if a1 >= a0 { 1.0 } else { -1.0 };
            Some((arc_pt(frame, *r, t), (frame.y * t.cos() - frame.x * t.sin()) * s))
        }
        PathSeg::Spline { sp, rev } => {
            // the end of the walked curve: parameter 0 (or its length when reversed)
            let u = if at_end != *rev { sp.length() } else { 0.0 };
            let t = sp.d1(u).try_normalize()?;
            Some((sp.eval(u), if *rev { -t } else { t }))
        }
        _ => None,
    }
}

/// Nearest point of a piece to `p` with the travel direction there.
fn project_on(seg: &PathSeg, p: Vec3) -> Option<(Vec3, Vec3)> {
    match seg {
        PathSeg::Line { a, b } => {
            let d = *b - *a;
            let l = d.len();
            let u = d.try_normalize()?;
            let t = ((p - *a).dot(u)).clamp(0.0, l);
            Some((*a + u * t, u))
        }
        PathSeg::Arc { frame, r, a0, a1 } => {
            let q = p - frame.o;
            let (qx, qy) = (q.dot(frame.x), q.dot(frame.y));
            if (qx * qx + qy * qy).sqrt() < 1e-9 * r.max(1.0) {
                return None;
            }
            let ang = qy.atan2(qx);
            let (lo, hi) = if a1 >= a0 { (*a0, *a1) } else { (*a1, *a0) };
            let span = hi - lo;
            // angle within [lo, lo + 2π), clamp into the swept range
            let mut t = lo + (ang - lo).rem_euclid(TAU);
            if span < TAU - 1e-9 && t > hi {
                // nearer end wins
                let (e0, e1) = (arc_pt(frame, *r, lo), arc_pt(frame, *r, hi));
                t = if e0.dist(p) <= e1.dist(p) { lo } else { hi };
            }
            let s = if a1 >= a0 { 1.0 } else { -1.0 };
            Some((arc_pt(frame, *r, t), (frame.y * t.cos() - frame.x * t.sin()) * s))
        }
        _ => None,
    }
}

fn seg_len(seg: &PathSeg) -> f64 {
    match seg {
        PathSeg::Line { a, b } => a.dist(*b),
        PathSeg::Arc { r, a0, a1, .. } => r * (a1 - a0).abs(),
        PathSeg::Spline { sp, .. } => sp.length(),
        _ => 0.0,
    }
}

fn seg_samples(seg: &PathSeg) -> Vec<Vec3> {
    match seg {
        PathSeg::Line { a, b } => vec![*a, a.lerp(*b, 0.5), *b],
        PathSeg::Arc { frame, r, a0, a1 } => (0..=16).map(|i| arc_pt(frame, *r, a0 + (a1 - a0) * i as f64 / 16.0)).collect(),
        PathSeg::Spline { sp, .. } => (0..=64).map(|i| sp.eval(sp.length() * i as f64 / 64.0)).collect(),
        _ => vec![],
    }
}

fn seg_mid(seg: &PathSeg) -> Vec3 {
    match seg {
        PathSeg::Line { a, b } => a.lerp(*b, 0.5),
        PathSeg::Arc { frame, r, a0, a1 } => arc_pt(frame, *r, 0.5 * (a0 + a1)),
        PathSeg::Spline { sp, .. } => sp.eval(0.5 * sp.length()),
        _ => Vec3::ZERO,
    }
}

// ---- axis references ----

/// Axis `(point, unit direction)` of an axis reference: a world axis, a
/// straight edge (origin = its start), a sketch line (origin = its first
/// point) or an `axis` feature.
pub fn resolve_axis(state: &RegenState, exec: &mut dyn FeatureExec, r: &crate::GeomRef) -> Option<(Vec3, Vec3)> {
    use crate::GeomRef;
    let (o, d) = match r {
        GeomRef::Axis(_) => crate::features::pattern::axis_of(r)?,
        GeomRef::Feature(id) => state.axes.iter().find(|a| &a.0 == id).map(|a| a.1)?,
        GeomRef::Other(j) => match kind_of(j)? {
            "edge" => {
                let e = crate::EdgeRef::from_json(j)?;
                exec.edge_line(state, &e.faces, e.at)?
            }
            "sketchLine" => {
                let sk = state.sketch(j.get("sketch")?.as_str()?)?;
                match entity_piece(sk, j.get("entity")?.as_str()?)?.seg {
                    PathSeg::Line { a, b } => (a, b - a),
                    _ => return None,
                }
            }
            _ => return None,
        },
        _ => return None,
    };
    Some((o, d.try_normalize()?))
}

/// Plane `(unit normal, offset)` of a plane reference (std / plane feature /
/// planar face).
fn plane_of_ref(state: &RegenState, exec: &mut dyn FeatureExec, j: &Json) -> Option<Frame> {
    let r = crate::GeomRef::from_json(j)?;
    crate::regen::resolve_plane(state, exec, &r)
}

/// Line of intersection of two planes; origin is the point nearest the
/// world origin, direction `z1 × z2`.
fn plane_intersection(a: Frame, b: Frame) -> Result<(Vec3, Vec3), FeatureError> {
    let c = a.z.cross(b.z);
    if c.len() < 1e-9 {
        return Err(err(PLANES_PARALLEL));
    }
    let (d1, d2, k) = (a.z.dot(a.o), b.z.dot(b.o), a.z.dot(b.z));
    let den = 1.0 - k * k;
    let p = a.z * ((d1 - d2 * k) / den) + b.z * ((d2 - d1 * k) / den);
    Ok((p, c.normalize()))
}

/// The `axis` feature: `refs` = one axis-like reference (world axis, edge,
/// sketch line, cylindrical face) or two planes (their intersection line).
pub fn exec_axis(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<(), FeatureError> {
    let refs: Vec<Json> = f.raw.get("refs").and_then(|r| r.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
    let line = match refs.len() {
        1 => {
            let j = &refs[0];
            if kind_of(j) == Some("face") {
                let id = j.get("id").and_then(|v| v.as_str()).ok_or_else(missing)?;
                match exec.face_surf(state, id).ok_or_else(missing)? {
                    FaceSurf::Cylinder { o, a, .. } => (o, a),
                    _ => return Err(err(FACE_NOT_CYLINDRICAL)),
                }
            } else {
                let r = crate::GeomRef::from_json(j).ok_or_else(missing)?;
                resolve_axis(state, exec, &r).ok_or_else(missing)?
            }
        }
        2 => {
            let a = plane_of_ref(state, exec, &refs[0]).ok_or_else(missing)?;
            let b = plane_of_ref(state, exec, &refs[1]).ok_or_else(missing)?;
            plane_intersection(a, b)?
        }
        _ => return Err(missing()),
    };
    state.axes.retain(|a| a.0 != f.id);
    state.axes.push((f.id.clone(), line));
    Ok(())
}

// ---- modes ----

fn base_frame(state: &RegenState, exec: &mut dyn FeatureExec, base: &Option<crate::GeomRef>) -> Result<Frame, FeatureError> {
    base.as_ref().and_then(|b| crate::regen::resolve_plane(state, exec, b)).ok_or_else(missing)
}

pub fn three_points(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature, flip: bool) -> Result<Frame, FeatureError> {
    let p = points_of(state, exec, f);
    if p.len() < 3 {
        return Err(missing());
    }
    let (x, v) = (p[1] - p[0], p[2] - p[0]);
    let z = x.cross(v);
    if z.len() <= 1e-9 * x.len() * v.len() || x.len() < 1e-12 {
        return Err(err(PLANE_POINTS_IN_LINE));
    }
    let (x, z) = (x.normalize(), z.normalize());
    let fr = Frame { o: p[0], x, y: z.cross(x), z };
    Ok(if flip { flipped(fr) } else { fr })
}

pub fn parallel(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature, base: &Option<crate::GeomRef>, flip: bool) -> Result<Frame, FeatureError> {
    let b = base_frame(state, exec, base)?;
    let p = *points_of(state, exec, f).first().ok_or_else(missing)?;
    let fr = Frame { o: b.o + b.z * (p - b.o).dot(b.z), ..b };
    Ok(if flip { flipped(fr) } else { fr })
}

pub fn line_point(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature, axis: &Option<crate::GeomRef>, flip: bool) -> Result<Frame, FeatureError> {
    let (o, d) = axis.as_ref().and_then(|a| resolve_axis(state, exec, a)).ok_or_else(missing)?;
    let p = *points_of(state, exec, f).first().ok_or_else(missing)?;
    let foot = o + d * (p - o).dot(d);
    let z = d.cross(p - foot);
    if z.len() < 1e-9 * (1.0 + (p - o).len()) {
        return Err(err(PLANE_POINT_ON_LINE));
    }
    let z = z.normalize();
    let fr = Frame { o: foot, x: d, y: z.cross(d), z };
    Ok(if flip { flipped(fr) } else { fr })
}

/// `normal`: plane normal to a sketch path at its start (or end).
pub fn normal(state: &RegenState, f: &Feature, flip: bool) -> Result<Frame, FeatureError> {
    let id = f.raw.get("path").and_then(|p| p.as_str()).ok_or_else(|| err(MISSING_PATH))?;
    let at_end = f.raw.get("atEnd").and_then(|b| b.as_bool()).unwrap_or(false);
    let (o, t, x) = match state.sketch(id) {
        Some(sk) => {
            let chain = sketch_chain(sk).ok_or_else(|| err(crate::features::sweep::PATH_INVALID))?;
            let seg = if at_end { chain.last() } else { chain.first() }.ok_or_else(|| err(MISSING_PATH))?;
            let (o, t) = end_state(seg, at_end).ok_or_else(|| err(MISSING_PATH))?;
            (o, t, sk.frame.z.reject(t).try_normalize().unwrap_or_else(|| up_frame_x(t)))
        }
        None => {
            // a sketch3d spline path: no sketch plane, the "up = Y" frame rule
            let j = state.result.sketches3.iter().find(|s| s.0 == id).map(|s| &s.1).ok_or_else(|| err(MISSING_PATH))?;
            let chain = crate::features::sweep::sketch3_path(j)?;
            let seg = chain.first().ok_or_else(|| err(MISSING_PATH))?;
            let (o, t) = end_state(seg, at_end).ok_or_else(|| err(MISSING_PATH))?;
            (o, t, up_frame_x(t))
        }
    };
    let z = if flip { -t } else { t };
    Ok(Frame { o, x, y: z.cross(x), z })
}

/// Connected chain of the non-construction path pieces of a sketch, walked
/// from a free end (a lone circle is its own chain).
fn sketch_chain(sk: &crate::regen::SketchResult) -> Option<Vec<PathSeg>> {
    let mut pieces = vec![];
    for e in &sk.sketch.entities {
        if e.construction() {
            continue;
        }
        match e {
            SketchEntity::Point { .. } => {}
            SketchEntity::Line { .. } | SketchEntity::Arc { .. } | SketchEntity::Circle { .. } => pieces.push(entity_piece(sk, e.id())?.seg),
            _ => return None,
        }
    }
    if pieces.is_empty() {
        return None;
    }
    if pieces.len() == 1 {
        return Some(pieces);
    }
    let near = |a: Vec3, b: Vec3| a.dist(b) < 1e-6;
    let mut start = None;
    'find: for (i, p) in pieces.iter().enumerate() {
        for (flip, q) in [(false, p.start()), (true, p.end())] {
            if !pieces.iter().enumerate().any(|(j, o)| j != i && (near(o.start(), q) || near(o.end(), q))) {
                start = Some((i, flip));
                break 'find;
            }
        }
    }
    let (i0, flip0) = start.unwrap_or((0, false));
    let mut used = vec![false; pieces.len()];
    used[i0] = true;
    let mut chain = vec![if flip0 { pieces[i0].reversed() } else { pieces[i0].clone() }];
    while chain.len() < pieces.len() {
        let tip = chain.last().unwrap().end();
        let (j, flip) = pieces.iter().enumerate().filter(|(j, _)| !used[*j]).find_map(|(j, p)| {
            if near(p.start(), tip) {
                Some((j, false))
            } else if near(p.end(), tip) {
                Some((j, true))
            } else {
                None
            }
        })?;
        used[j] = true;
        chain.push(if flip { pieces[j].reversed() } else { pieces[j].clone() });
    }
    Some(chain)
}

/// `normalCurve`: plane normal to a curve at its start / end / the point
/// nearest a picked point.
pub fn normal_curve(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature, flip: bool) -> Result<Frame, FeatureError> {
    let pieces = curves_of(state, exec, f)?;
    let pc = &pieces[0];
    let at_end = f.raw.get("atEnd").and_then(|b| b.as_bool()).unwrap_or(false);
    let pts = points_of(state, exec, f);
    let (o, t) = match pts.first() {
        Some(p) => project_on(&pc.seg, *p),
        None => end_state(&pc.seg, at_end),
    }
    .ok_or_else(missing)?;
    let x = match &pc.seg {
        PathSeg::Arc { frame, .. } => frame.z,
        _ => up_frame_x(t),
    };
    let fr = Frame { o, x, y: t.cross(x), z: t };
    Ok(if flip { flipped(fr) } else { fr })
}

/// `curve`: plane of one or more planar curves (offset along its normal).
pub fn curve_plane(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature, flip: bool) -> Result<Frame, FeatureError> {
    let pieces = curves_of(state, exec, f)?;
    let offset = state.num(f, "offset", 0.0)?;
    let samples: Vec<Vec3> = pieces.iter().flat_map(|p| seg_samples(&p.seg)).collect();
    // orientation: a circular curve's own plane, else the sketch's, else a fit
    let circ = pieces.iter().find(|p| matches!(p.seg, PathSeg::Arc { .. }));
    let mut fr = match (circ, pieces[0].hint) {
        (Some(c), Some(h)) if c.hint.is_some() => Frame { o: Vec3::ZERO, x: h.x, y: h.y, z: h.z },
        (Some(Piece { seg: PathSeg::Arc { frame, .. }, cap_normal, .. }), _) => {
            let n = match cap_normal {
                Some(n) if n.dot(frame.z).abs() > 1.0 - 1e-9 => *n,
                _ => frame.z,
            };
            Frame::from_normal_x(Vec3::ZERO, n, frame.x)
        }
        (_, Some(h)) => Frame { o: Vec3::ZERO, x: h.x, y: h.y, z: h.z },
        _ => {
            let n = fit_normal(&samples).ok_or_else(|| err(CURVES_STRAIGHT))?;
            Frame::from_normal(Vec3::ZERO, n)
        }
    };
    // all samples must lie in the plane through the first sample
    let o0 = samples[0];
    let dev = samples.iter().map(|p| (*p - o0).dot(fr.z).abs()).fold(0.0, f64::max);
    if dev > 1e-6 * (1.0 + samples.iter().map(|p| p.dist(o0)).fold(0.0, f64::max)) {
        let dev = plane_dev(&samples).unwrap_or(dev);
        return Err(err(CURVES_NOT_PLANAR).with("dev", Json::Num((dev * 1e6).round() / 1e6)));
    }
    // origin: length-weighted centroid (an arc / circle counts as its centre)
    let (mut sum, mut w) = (Vec3::ZERO, 0.0);
    for p in &pieces {
        let l = seg_len(&p.seg);
        let c = match &p.seg {
            PathSeg::Arc { frame, .. } => frame.o,
            seg => seg_mid(seg),
        };
        sum = sum + c * l;
        w += l;
    }
    fr.o = if w > 0.0 { sum * (1.0 / w) } else { o0 };
    if flip {
        fr = flipped(fr);
    }
    fr.o = fr.o + fr.z * offset;
    Ok(fr)
}

/// Least-squares plane normal of a point cloud (smallest covariance axis).
fn fit_normal(pts: &[Vec3]) -> Option<Vec3> {
    let n = pts.len() as f64;
    let c = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) * (1.0 / n);
    // the best of the pairwise-cross candidates (robust, no eigen solver)
    let mut best: Option<(f64, Vec3)> = None;
    for a in pts {
        for b in pts {
            let z = (*a - c).cross(*b - c);
            let l = z.len();
            if best.map_or(true, |(bl, _)| l > bl) {
                best = Some((l, z));
            }
        }
    }
    let (l, z) = best?;
    (l > 1e-12).then(|| z.normalize())
}

/// Maximum distance of the points from their best-fit plane.
fn plane_dev(pts: &[Vec3]) -> Option<f64> {
    let n = fit_normal(pts)?;
    let c = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) * (1.0 / pts.len() as f64);
    Some(pts.iter().map(|p| (*p - c).dot(n).abs()).fold(0.0, f64::max))
}

/// `tangent`: plane touching a curved face (to a point, parallel to a plane
/// or through a line).
pub fn tangent(state: &RegenState, exec: &mut dyn FeatureExec, f: &Feature, base: &Option<crate::GeomRef>, axis: &Option<crate::GeomRef>, flip: bool) -> Result<Frame, FeatureError> {
    let face = f.raw.get("face").and_then(|v| v.as_str()).ok_or_else(missing)?;
    let surf = exec.face_surf(state, face).ok_or_else(missing)?;
    let second = f.raw.get("second").and_then(|b| b.as_bool()).unwrap_or(false);
    let to = f.raw.get("tangentTo").and_then(|v| v.as_str()).unwrap_or("point");
    let not_tangent = || err(PLANE_NOT_TANGENT);
    let fr = match to {
        "plane" => {
            let b = base_frame(state, exec, base)?;
            let oriented = if second { flipped(b) } else { b };
            let z = oriented.z;
            match surf {
                FaceSurf::Sphere { c, r, .. } => Frame { o: c + z * r, ..oriented },
                FaceSurf::Cylinder { o, a, r, .. } => {
                    if a.dot(b.z).abs() > 1e-9 {
                        return Err(not_tangent());
                    }
                    // axis point nearest the world origin
                    let c = o - a * o.dot(a);
                    Frame { o: c + z * r, ..oriented }
                }
                _ => return Err(not_tangent()),
            }
        }
        "line" => {
            let (lo, ld) = axis.as_ref().and_then(|a| resolve_axis(state, exec, a)).ok_or_else(missing)?;
            let (c, r, axial) = match surf {
                FaceSurf::Sphere { c, r, .. } => (c, r, None),
                FaceSurf::Cylinder { o, a, r, .. } => (o, r, Some(a)),
                _ => return Err(not_tangent()),
            };
            if let Some(a) = axial {
                if a.cross(ld).len() > 1e-9 {
                    return Err(not_tangent());
                }
            }
            let foot = lo + ld * (c - lo).dot(ld);
            let w = foot - c;
            let dist = w.len();
            if dist <= r * (1.0 + 1e-12) {
                return Err(not_tangent());
            }
            let u = w * (1.0 / dist);
            let s = (1.0 - (r / dist) * (r / dist)).max(0.0).sqrt() * if second { -1.0 } else { 1.0 };
            let z = (u * (r / dist) + u.cross(ld) * s).normalize();
            Frame { o: foot, x: ld, y: z.cross(ld), z }
        }
        _ => {
            let p = match points_of(state, exec, f).first() {
                Some(p) => *p,
                None => f.raw.get("at").and_then(vec3_of).ok_or_else(missing)?,
            };
            match surf {
                FaceSurf::Sphere { c, r, x } => {
                    let z = (p - c).try_normalize().unwrap_or(x);
                    Frame { o: c + z * r, x: up_frame_x(z), y: z, z }.fix_y()
                }
                FaceSurf::Cylinder { o, a, r, x } => {
                    let q = p - o;
                    let h = q.dot(a);
                    let z = (q - a * h).try_normalize().unwrap_or(x);
                    Frame { o: o + a * h + z * r, x: a.cross(z), y: a, z }
                }
                _ => return Err(not_tangent()),
            }
        }
    };
    Ok(if flip { flipped(fr) } else { fr })
}

trait FixY {
    fn fix_y(self) -> Frame;
}
impl FixY for Frame {
    /// Recompute `y = z × x`.
    fn fix_y(mut self) -> Frame {
        self.y = self.z.cross(self.x);
        self
    }
}

#[cfg(test)]
mod tests {
    use crate::regen::{regen, NoGeometry, RegenOpts};
    use crate::Part;
    use ck_core::linalg::{Frame, Vec3};

    const SK: &str = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"p0","x":0,"y":0},{"id":"p1","x":10,"y":0},{"id":"p2","x":0,"y":10},{"id":"p3","x":5,"y":5},{"id":"p4","x":20,"y":0},{"id":"p5","x":3,"y":4}],"entities":[{"id":"e1","type":"line","p1":"p0","p2":"p1"},{"id":"e2","type":"line","p1":"p1","p2":"p2"},{"id":"e4","type":"line","p1":"p3","p2":"p5"}],"constraints":[]},
        {"type":"sketch","id":"PL","plane":{"kind":"std","id":"front"},"points":[{"id":"p0","x":0,"y":0},{"id":"p1","x":10,"y":0},{"id":"p2","x":10,"y":10}],"entities":[{"id":"e1","type":"line","p1":"p0","p2":"p1"},{"id":"e2","type":"line","p1":"p1","p2":"p2"}],"constraints":[]},
        {"type":"sketch","id":"S2","plane":{"kind":"std","id":"right"},"points":[{"id":"p0","x":1,"y":2},{"id":"p1","x":7,"y":3},{"id":"p2","x":-4,"y":9}],"entities":[],"constraints":[]},
        {"type":"sketch","id":"PA","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":1,"y":2},{"id":"a","x":10,"y":2},{"id":"b","x":1,"y":10}],"entities":[{"id":"e1","type":"arc","c":"c","p1":"a","p2":"b"}],"constraints":[]},
        {"type":"sketch","id":"PC","plane":{"kind":"std","id":"right"},"points":[{"id":"c","x":3,"y":4}],"entities":[{"id":"e1","type":"circle","c":"c","r":5}],"constraints":[]}"#;

    fn plane(tail: &str) -> (Option<Frame>, Vec<String>) {
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{SK},{tail}]}}"#);
        let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::default(), &mut NoGeometry);
        (r.planes.iter().find(|p| p.0 == "P").map(|p| p.1), r.errors.iter().map(|e| e.error.code.clone()).collect())
    }
    fn p(tail: &str) -> Frame {
        let (f, e) = plane(tail);
        f.unwrap_or_else(|| panic!("{e:?}"))
    }
    fn near(a: Vec3, b: [f64; 3]) -> bool {
        (a - Vec3::new(b[0], b[1], b[2])).len() < 1e-9
    }
    fn is(f: Frame, o: [f64; 3], x: [f64; 3], z: [f64; 3]) {
        assert!(near(f.o, o) && near(f.x, x) && near(f.z, z), "{f:?}");
    }
    fn sp(s: &str, i: u32) -> String {
        format!(r#"{{"kind":"sketchPoint","sketch":"{s}","point":"p{i}"}}"#)
    }

    /// Frames probed against the original kernel (tools/plane_cmp.mjs).
    #[test]
    fn point_modes_match_oracle() {
        let (s, c) = (0.5, 0.75f64.sqrt());
        let _ = (s, c);
        let pts = |a: &str, b: &str, c: &str| format!(r#""points":[{a},{b},{c}]"#);
        // threePoints: origin p0, x toward p1, z = x x (p2 - p0)
        let f = p(&format!(r#"{{"type":"plane","id":"P","mode":"threePoints",{}}}"#, pts(&sp("S1", 1), &sp("S1", 0), &sp("S1", 2))));
        is(f, [10., 0., 0.], [-1., 0., 0.], [0., 0., -1.]);
        let f = p(&format!(r#"{{"type":"plane","id":"P","mode":"threePoints","flip":true,{}}}"#, pts(&sp("S1", 0), &sp("S1", 1), &sp("S1", 2))));
        is(f, [0., 0., 0.], [-1., 0., 0.], [0., 0., -1.]);
        let f = p(&format!(r#"{{"type":"plane","id":"P","mode":"threePoints",{}}}"#, pts(&sp("S2", 0), &sp("S2", 1), &sp("S2", 2))));
        is(f, [0., 2., -1.], [0., 0.1643989873053573, -0.9863939238321437], [1., 0., 0.]);
        assert_eq!(plane(&format!(r#"{{"type":"plane","id":"P","mode":"threePoints",{}}}"#, pts(&sp("S1", 0), &sp("S1", 1), &sp("S1", 4)))).1, vec!["planePointsInLine"]);
        assert_eq!(plane(r#"{"type":"plane","id":"P","mode":"threePoints","points":[]}"#).1, vec!["missingReference"]);
        // parallel: base frame through the point (offset ignored)
        let f = p(&format!(r#"{{"type":"plane","id":"P","mode":"parallel","base":{{"kind":"std","id":"top"}},"points":[{}],"offset":9}}"#, sp("S2", 1)));
        is(f, [0., 3., 0.], [1., 0., 0.], [0., 1., 0.]);
        let f = p(&format!(r#"{{"type":"plane","id":"P","mode":"parallel","flip":true,"base":{{"kind":"std","id":"front"}},"points":[{}]}}"#, sp("S1", 3)));
        is(f, [0., 0., 0.], [-1., 0., 0.], [0., 0., -1.]);
        // linePoint: origin = foot of the point on the axis, x = axis
        let lp = |axis: &str, pt: String, extra: &str| format!(r#"{{"type":"plane","id":"P","mode":"linePoint","axis":{axis},"points":[{pt}]{extra}}}"#);
        let f = p(&lp(r#"{"kind":"axis","axis":"y"}"#, sp("S2", 1), ""));
        is(f, [0., 3., 0.], [0., 1., 0.], [-1., 0., 0.]);
        let f = p(&lp(r#"{"kind":"axis","axis":"x"}"#, sp("S1", 2), r#","flip":true"#));
        is(f, [0., 0., 0.], [-1., 0., 0.], [0., 0., -1.]);
        // sketch-line axis: origin = the line's first point
        let f = p(&lp(r#"{"kind":"sketchLine","sketch":"S1","entity":"e4"}"#, sp("S2", 1), ""));
        is(f, [0.2, 2.6, 0.], [-0.8944271909999159, -0.4472135954999579, 0.], [0.44630369914433676, -0.8926073982886735, -0.06375767130633382]);
        assert_eq!(plane(&lp(r#"{"kind":"axis","axis":"y"}"#, sp("S1", 2), "")).1, vec!["planePointOnLine"]);
        // oblique axis keeps x on the axis' positive side; perpendicular normalises y
        let f = p(r#"{"type":"plane","id":"P","mode":"angle","base":{"kind":"std","id":"top"},"axis":{"kind":"sketchLine","sketch":"S1","entity":"e4"},"angle":30}"#);
        is(f, [5., 5., 0.], [-0.9732050807568877, -0.053589838486224534, -0.22360679774997902], [0.05358983848622454, 0.892820323027551, -0.447213595499958]);
        let f = p(r#"{"type":"plane","id":"P","mode":"perpendicular","base":{"kind":"std","id":"top"},"axis":{"kind":"sketchLine","sketch":"S1","entity":"e4"}}"#);
        is(f, [5., 5., 0.], [-0.8944271909999159, -0.4472135954999579, 0.], [0., 0., -1.]);
    }

    #[test]
    fn axis_feature_matches_oracle() {
        let planes = r#"{"type":"plane","id":"A1","mode":"offset","base":{"kind":"std","id":"top"},"offset":5},{"type":"plane","id":"A2","mode":"offset","base":{"kind":"std","id":"front"},"offset":3}"#;
        let ax = |refs: &str, mode: &str| format!(r#"{planes},{{"type":"axis","id":"A","refs":[{refs}]}},{{"type":"plane","id":"P","mode":"{mode}","base":{{"kind":"std","id":"top"}},"axis":{{"kind":"feature","id":"A"}},"angle":20}}"#);
        // two planes: line z1 x z2, origin nearest the world origin
        let f = p(&ax(r#"{"kind":"feature","id":"A1"},{"kind":"feature","id":"A2"}"#, "angle"));
        is(f, [0., 5., 3.], [1., 0., 0.], [0., 0.9396926207859084, 0.3420201433256687]);
        let f = p(&ax(r#"{"kind":"feature","id":"A2"},{"kind":"feature","id":"A1"}"#, "angle"));
        is(f, [0., 5., 3.], [-1., 0., 0.], [0., 0.9396926207859084, -0.3420201433256687]);
        let f = p(&ax(r#"{"kind":"std","id":"top"},{"kind":"std","id":"front"}"#, "angle"));
        is(f, [0., 0., 0.], [1., 0., 0.], [0., 0.9396926207859084, 0.3420201433256687]);
        assert_eq!(plane(&ax(r#"{"kind":"std","id":"top"},{"kind":"std","id":"top"}"#, "angle")).1, vec!["planesParallel", "missingReference"]);
        assert_eq!(plane(&ax(r#"{"kind":"std","id":"top"}"#, "angle")).1, vec!["missingReference", "missingReference"]);
        assert_eq!(plane(&ax("", "angle")).1, vec!["missingReference", "missingReference"]);
        // a single sketch line / world axis
        let f = p(&ax(r#"{"kind":"axis","axis":"z"}"#, "angle"));
        is(f, [0., 0., 0.], [0., 0., 1.], [-0.3420201433256687, 0.9396926207859084, 0.]);
    }

    #[test]
    fn path_and_curve_modes_match_oracle() {
        let s = |p: &str| p.to_string();
        let _ = s;
        // normal: z = path tangent at the start/end, x = sketch normal; flip negates z and y
        let n = |extra: &str, path: &str| p(&format!(r#"{{"type":"plane","id":"P","mode":"normal","path":"{path}"{extra}}}"#));
        is(n("", "PL"), [0., 0., 0.], [0., 0., 1.], [1., 0., 0.]);
        is(n(r#","atEnd":true"#, "PL"), [10., 10., 0.], [0., 0., 1.], [0., 1., 0.]);
        is(n(r#","atEnd":true"#, "PA"), [1., 0., -11.], [0., 1., 0.], [-1., 0., 0.]);
        let f = n(r#","flip":true"#, "PL");
        assert!(near(f.x, [0., 0., 1.]) && near(f.y, [0., 1., 0.]) && near(f.z, [-1., 0., 0.]), "{f:?}");
        assert_eq!(plane(r#"{"type":"plane","id":"P","mode":"normal"}"#).1, vec!["missingPath"]);
        // normalCurve: x = Y x z for lines, the curve normal for arcs / circles
        let nc = |e: &str, extra: &str| {
            let (s, e) = e.split_once('.').unwrap();
            p(&format!(r#"{{"type":"plane","id":"P","mode":"normalCurve","curves":[{{"kind":"sketchCurve","sketch":"{s}","entity":"{e}"}}]{extra}}}"#))
        };
        is(nc("PL.e2", ""), [10., 0., 0.], [1., 0., 0.], [0., 1., 0.]);
        is(nc("PL.e2", r#","atEnd":true"#), [10., 10., 0.], [1., 0., 0.], [0., 1., 0.]);
        is(nc("PL.e1", ""), [0., 0., 0.], [0., 0., -1.], [1., 0., 0.]);
        is(nc("PA.e1", ""), [10., 0., -2.], [0., 1., 0.], [0., 0., -1.]);
        is(nc("PC.e1", ""), [0., 4., -8.], [1., 0., 0.], [0., 1., 0.]);
        let f = nc("PL.e2", r#","flip":true"#);
        assert!(near(f.x, [-1., 0., 0.]) && near(f.z, [0., -1., 0.]), "{f:?}");
        // curve: the sketch frame at the circle centre / weighted centroid, offset along z
        let cv = |curves: &str, extra: &str| p(&format!(r#"{{"type":"plane","id":"P","mode":"curve","curves":[{curves}]{extra}}}"#));
        let c = |s: &str, e: &str| format!(r#"{{"kind":"sketchCurve","sketch":"{s}","entity":"{e}"}}"#);
        is(cv(&c("PC", "e1"), r#","offset":5"#), [5., 4., -3.], [0., 0., -1.], [1., 0., 0.]);
        is(cv(&c("PA", "e1"), ""), [1., 0., -2.], [1., 0., 0.], [0., 1., 0.]);
        is(cv(&c("S1", "e1"), ""), [5., 0., 0.], [1., 0., 0.], [0., 0., 1.]);
        is(cv(&format!("{},{}", c("PL", "e1"), c("PL", "e2")), ""), [7.5, 2.5, 0.], [1., 0., 0.], [0., 0., 1.]);
        let f = cv(&c("PC", "e1"), r#","flip":true"#);
        assert!(near(f.x, [0., 0., 1.]) && near(f.z, [-1., 0., 0.]), "{f:?}");
        assert_eq!(plane(&format!(r#"{{"type":"plane","id":"P","mode":"curve","curves":[{},{}]}}"#, c("S1", "e1"), c("PC", "e1"))).1, vec!["curvesNotPlanar"]);
        assert_eq!(plane(r#"{"type":"plane","id":"P","mode":"curve","curves":[]}"#).1, vec!["missingReference"]);
    }
}
