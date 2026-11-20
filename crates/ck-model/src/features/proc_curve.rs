//! Procedural curve features (ck-model/src/features/proc_curve.rs): `offsetCurve`, `compositeCurve` and
//! `projectCurve` with `procedural: true`. Each keeps its curve as a ck-geom procedural definition
//! (`Curve::Offset`, `Curve::Composite`, `Curve::Projected`) on the [`Brep`] (`proc_curves`, by feature id) and
//! publishes it like every other curve feature: `curves[] = [id, {pts, axis:false, t0:0, t1:1, procedural}]`
//! (`pts` evaluated on the definition) and a marker `[id, {kind, length, closed, pieces, corners, continuity,
//! degree, count, deviation, ...}]` whose `degree` / `count` / `deviation` describe the B-spline the definition
//! converts to within `tolerance` (`Curve::to_nurbs_within`; a definition that does not convert is refused).
//!
//! Every consumer of a curve feature (`{kind:"curve", id}`: sweep / pipe / curvepattern paths, sweep guides,
//! `plane` mode `curve`, `editCurve`, `projectCurve`, and these features themselves) reads it the way it reads a
//! B-spline curve feature, through `curves[]`. Two read the definition itself: a path whose pieces are all lines
//! and arcs is swept along those pieces exactly ([`exact_path`]), and a later procedural feature builds on the
//! definition (an offset of a composite, a projection of an offset), not on its polyline.
//!
//! Input (all three): `curves` (ordered curve references: `{kind:"sketchCurve", sketch, entity}`, `{kind:"edge",
//! faces, at}`, `{kind:"curve", id}`), else `curve` (an earlier curve feature), else `sketch` (its non-construction
//! curves, walked into one chain). Pieces are turned to run end to end; ends further apart than `gapTolerance`
//! (1e-5 mm) are refused. A curve feature that is not procedural (helix, editCurve, a plain projectCurve) enters as
//! its exact helix or as a cubic B-spline through its points.

use super::curve_ref::curve_pts;
use super::project_curve::plane_normal;
use super::surfaces::pick_edge;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef, SketchEntity};
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::procedural::{Continuity, OffsetRef};
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_ops::boolean::classify::{point_in_face_cached, LoopCache, Where};
use ck_ops::sweep::PathSeg;
use std::f64::consts::TAU;

/// The definition of a procedural curve feature over its range.
#[derive(Debug, Clone, PartialEq)]
pub struct ProcCurve {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
}

/// One input piece: `c` over `[t0, t1]`.
#[derive(Debug, Clone)]
struct Piece {
    c: Curve,
    t0: f64,
    t1: f64,
}

impl Piece {
    fn start(&self) -> Vec3 {
        self.c.eval(self.t0)
    }
    fn end(&self) -> Vec3 {
        self.c.eval(self.t1)
    }
    fn reversed(&self) -> Piece {
        let (a, b) = (self.c.reversed_param(self.t1), self.c.reversed_param(self.t0));
        Piece { c: self.c.reversed(), t0: a.min(b), t1: a.max(b) }
    }
}

/// Where the input came from: a sketch carries its plane normal.
struct Source {
    pieces: Vec<Piece>,
    sketch_normal: Option<Vec3>,
}

const DEFAULT_GAP: f64 = 1e-5;

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

fn hint(e: FeatureError, h: &str) -> FeatureError {
    e.with("hint", Json::Str(h.into()))
}

fn invalid(field: &str, detail: &str, h: &str) -> FeatureError {
    hint(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into())), h)
}

/// Is `projectCurve` asked for the procedural projection?
pub fn is_procedural(f: &Feature) -> bool {
    crate::sketch::jtruthy(f.raw.get("procedural"))
}

/// A numeric field: number, `=expression` or numeric string; absent is the default.
fn num(state: &RegenState, f: &Feature, key: &str, default: f64) -> Result<f64, FeatureError> {
    match f.raw.get(key) {
        Some(Json::Str(s)) if !s.trim_start().starts_with('=') => match s.trim().parse::<f64>() {
            Ok(v) => Ok(v),
            Err(_) => state.num(f, key, default),
        },
        _ => state.num(f, key, default),
    }
}

fn json_vec(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    (a.len() == 3).then(|| Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?)))?
}

// ---- input --------------------------------------------------------------------------------------------------

/// A cubic B-spline of `f` over `range`, within 1e-6 of the size.
fn fit_fn(f: &dyn Fn(f64) -> Vec3, range: (f64, f64), size: f64) -> Option<Curve> {
    let tol = (1e-6 * size).max(1e-7);
    let (b, _) = ck_geom::approx::approximate_curve_fn(f, range, 3, tol, &[]).ok()?;
    Some(Curve::BSpline(b))
}

/// The exact curves of one sketch entity (`None`: a point, text, a closed control-point spline or degenerate).
fn sketch_entity(sk: &crate::regen::SketchResult, e: &SketchEntity) -> Option<Piece> {
    let fr = &sk.frame;
    let s = &sk.sketch;
    let to3 = |p: [f64; 2]| fr.o + fr.x * p[0] + fr.y * p[1];
    let pt = |p: &str| s.point_xy(p).map(to3);
    match e {
        SketchEntity::Line { p1, p2, .. } => {
            let (a, b) = (pt(p1)?, pt(p2)?);
            let l = a.dist(b);
            (l > 1e-12).then(|| Piece { c: Curve::line(a, b - a), t0: 0.0, t1: l })
        }
        SketchEntity::Circle { c, r, .. } => {
            let r = r.abs();
            let c3 = pt(c)?;
            (r > 1e-12).then(|| Piece { c: Curve::circle_x(c3, fr.z, fr.x, r), t0: 0.0, t1: TAU })
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (cc, a, b) = (s.point_xy(c)?, s.point_xy(p1)?, s.point_xy(p2)?);
            let (c3, a3) = (to3(cc), to3(a));
            let r = c3.dist(a3);
            if r < 1e-12 {
                return None;
            }
            let a0 = (a[1] - cc[1]).atan2(a[0] - cc[0]);
            let mut sw = (b[1] - cc[1]).atan2(b[0] - cc[0]) - a0;
            while sw <= 1e-12 {
                sw += TAU;
            }
            while sw > TAU + 1e-12 {
                sw -= TAU;
            }
            Some(Piece { c: Curve::circle_x(c3, fr.z, a3 - c3, r), t0: 0.0, t1: sw })
        }
        SketchEntity::Ellipse { c, p, r, .. } => {
            let (c3, p3) = (pt(c)?, pt(p)?);
            let lu = c3.dist(p3);
            (lu > 1e-12 && r.abs() > 1e-12).then(|| Piece { c: Curve::ellipse(c3, fr.z, p3 - c3, lu, r.abs()), t0: 0.0, t1: TAU })
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
                    k.extend((1..spans).map(|i| i as f64 / spans as f64));
                    k.extend(std::iter::repeat(1.0).take(p + 1));
                    k
                }
            };
            let b = ck_geom::nurbs::bspline::BSplineCurve::new(p, k, ctrl).ok()?;
            let (t0, t1) = b.domain();
            Some(Piece { c: Curve::BSpline(b), t0, t1 })
        }
        SketchEntity::Spline { pts, closed, .. } => {
            let mut p3: Vec<Vec3> = pts.iter().filter_map(|q| pt(q)).collect();
            if *closed && !p3.is_empty() {
                p3.push(p3[0]);
            }
            let sp = ck_ops::spline_path::SplinePath::new(&p3, None, None)?;
            let len = sp.length();
            let c = fit_fn(&|s| sp.eval(s), (0.0, len), len)?;
            let (t0, t1) = c.range();
            Some(Piece { c, t0, t1 })
        }
        SketchEntity::Point { .. } | SketchEntity::Text { .. } => None,
    }
}

/// An earlier curve feature as pieces: its definition (a procedural feature, flattened when it is a composite), its
/// helix, or a cubic B-spline through its points.
fn feature_curve(br: &Brep, state: &RegenState, id: &str) -> Option<Vec<Piece>> {
    if let Some((_, pc)) = br.proc_curves.iter().rev().find(|(i, _)| i == id) {
        if let Curve::Composite(cc) = &pc.curve {
            if pc.t0 <= 0.0 && pc.t1 >= cc.range().1 {
                return Some(cc.segs.iter().map(|g| Piece { c: g.curve.clone(), t0: g.t0, t1: g.t1 }).collect());
            }
        }
        return Some(vec![Piece { c: pc.curve.clone(), t0: pc.t0, t1: pc.t1 }]);
    }
    if let Some((_, h)) = br.helices.iter().rev().find(|(i, _)| i == id) {
        let h = *h;
        let size = h.r + h.k.abs() * h.angle;
        let c = fit_fn(&|t| h.eval(t), (0.0, h.angle), size)?;
        let (t0, t1) = c.range();
        return Some(vec![Piece { c, t0, t1 }]);
    }
    let pts = curve_pts(state, id)?;
    let step = pts.len().div_ceil(200).max(1);
    let mut p: Vec<Vec3> = pts.iter().step_by(step).copied().collect();
    let last = *pts.last()?;
    if p.last().map_or(true, |q| q.dist(last) > 1e-9) {
        p.push(last);
    }
    let sp = ck_ops::spline_path::SplinePath::new(&p, None, None)?;
    let len = sp.length();
    let c = fit_fn(&|s| sp.eval(s), (0.0, len), len)?;
    let (t0, t1) = c.range();
    Some(vec![Piece { c, t0, t1 }])
}

/// One curve reference (`sketchCurve`, `edge`, `curve`) as pieces.
fn ref_pieces(br: &Brep, state: &RegenState, r: &Json) -> Result<Option<Vec<Piece>>, FeatureError> {
    Ok(match r.get_str("kind") {
        Some("sketchCurve") => {
            let Some(sk) = r.get_str("sketch").and_then(|s| state.sketch(s)) else { return Ok(None) };
            let Some(e) = r.get_str("entity").and_then(|id| sk.sketch.entity(id)) else { return Ok(None) };
            sketch_entity(sk, e).map(|p| vec![p])
        }
        Some("edge") => {
            let faces: Vec<String> = r.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|f| f.as_str().map(str::to_string)).collect()).unwrap_or_default();
            if faces.is_empty() {
                return Ok(None);
            }
            let at = r.get("at").and_then(json_vec);
            // an unknown face name is the structured missingReference of the face pick
            let (slot, e) = pick_edge(br, &faces, at)?;
            let Some(b) = br.slot_body(slot) else { return Ok(None) };
            let ed = b.edge(e);
            Some(vec![Piece { c: ed.curve.clone(), t0: ed.t0, t1: ed.t1 }])
        }
        Some("curve") => r.get_str("id").and_then(|id| feature_curve(br, state, id)),
        _ => None,
    })
}

/// Pieces in the order given, each turned to start where the previous one ends; a joint further apart than `gap`
/// is `<prefix>Gap`.
fn orient_ordered(mut ps: Vec<Piece>, gap: f64, prefix: &str) -> Result<Vec<Piece>, FeatureError> {
    if ps.len() >= 2 {
        let (s0, e0) = (ps[0].start(), ps[0].end());
        let near = |q: Vec3| ps[1].start().dist(q).min(ps[1].end().dist(q));
        if near(s0) < near(e0) {
            ps[0] = ps[0].reversed();
        }
    }
    for i in 1..ps.len() {
        let tail = ps[i - 1].end();
        if ps[i].end().dist(tail) < ps[i].start().dist(tail) {
            ps[i] = ps[i].reversed();
        }
        let d = ps[i].start().dist(tail);
        if !(d <= gap) {
            return Err(hint(
                FeatureError::new(&format!("{prefix}Gap"))
                    .with("index", Json::Num(i as f64))
                    .with("gap", Json::Num(d))
                    .with("tolerance", Json::Num(gap))
                    .with("at", v3(tail)),
                "the curves must run end to end in the order listed: put them in chain order, close the gap in the geometry, or raise `gapTolerance` to at least the gap",
            ));
        }
    }
    Ok(ps)
}

/// Unordered pieces (a sketch) walked into one chain from a free end; pieces left over are `<prefix>NotChain`.
fn walk_chain(mut ps: Vec<Piece>, gap: f64, prefix: &str) -> Result<Vec<Piece>, FeatureError> {
    let n = ps.len();
    let meets = |a: Vec3, ps: &[Piece], skip: usize| ps.iter().enumerate().any(|(j, p)| j != skip && (p.start().dist(a) <= gap || p.end().dist(a) <= gap));
    let mut first = None;
    'find: for i in 0..n {
        if !meets(ps[i].start(), &ps, i) {
            first = Some((i, false));
            break 'find;
        }
        if !meets(ps[i].end(), &ps, i) {
            first = Some((i, true));
            break 'find;
        }
    }
    let (i0, rev) = first.unwrap_or((0, false));
    let p0 = ps.remove(i0);
    let mut chain = vec![if rev { p0.reversed() } else { p0 }];
    loop {
        let tail = chain.last().unwrap().end();
        let mut best: Option<(f64, usize, bool)> = None;
        for (j, p) in ps.iter().enumerate() {
            for (d, r) in [(p.start().dist(tail), false), (p.end().dist(tail), true)] {
                if d <= gap && best.map_or(true, |b| d < b.0) {
                    best = Some((d, j, r));
                }
            }
        }
        let Some((_, j, r)) = best else { break };
        let p = ps.remove(j);
        chain.push(if r { p.reversed() } else { p });
    }
    if !ps.is_empty() {
        let ends: Vec<Json> = [chain[0].start(), chain.last().unwrap().end()].into_iter().map(v3).collect();
        return Err(hint(
            FeatureError::new(&format!("{prefix}NotChain")).with("count", Json::Num((chain.len()) as f64)).with("left", Json::Num(ps.len() as f64)).with("points", Json::Arr(ends)),
            "the sketch curves do not form one connected chain: connect their ends (coincident points), make the extra curves construction geometry, or list the curves to use in `curves`",
        ));
    }
    Ok(chain)
}

/// The input of a procedural curve feature (see the module doc); `prefix` names its error codes.
fn source(br: &Brep, state: &RegenState, f: &Feature, prefix: &str) -> Result<Source, FeatureError> {
    let gap = num(state, f, "gapTolerance", DEFAULT_GAP)?;
    if !(gap.is_finite() && gap >= 0.0) {
        return Err(invalid("gapTolerance", "gapTolerance is a distance (mm), 0 or more", "leave it out for 1e-5 mm"));
    }
    let no_input = || {
        hint(
            FeatureError::new(&format!("{prefix}NoInput")),
            "give `curves` (curve references in chain order), `curve` (an earlier curve feature) or `sketch` (a sketch whose curves form one chain)",
        )
    };
    let refs = f.raw.get("curves").and_then(|c| c.as_arr()).filter(|a| !a.is_empty());
    if let Some(refs) = refs {
        let mut ps = vec![];
        for (i, r) in refs.iter().enumerate() {
            match ref_pieces(br, state, r)? {
                Some(p) => ps.extend(p),
                None => {
                    let name = r.get_str("entity").or_else(|| r.get_str("id")).unwrap_or("?").to_string();
                    return Err(hint(
                        FeatureError::new(codes::MISSING_REFERENCE).with("curve", Json::Str(name)).with("index", Json::Num(i as f64)),
                        "the curve reference does not resolve: a sketch curve needs `sketch` and `entity` (not a point or construction-only text), an edge `faces` (and `at`), a curve feature an `id` of an earlier curve feature",
                    ));
                }
            }
        }
        return Ok(Source { pieces: orient_ordered(ps, gap, prefix)?, sketch_normal: None });
    }
    if let Some(id) = f.raw.get_str("curve") {
        let ps = feature_curve(br, state, id).ok_or_else(|| {
            hint(FeatureError::new(codes::MISSING_REFERENCE).with("curve", Json::Str(id.into())), "`curve` names an earlier curve feature (helix, editCurve, projectCurve, offsetCurve, compositeCurve)")
        })?;
        return Ok(Source { pieces: orient_ordered(ps, gap, prefix)?, sketch_normal: None });
    }
    if let Some(id) = f.raw.get_str("sketch") {
        let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let ps: Vec<Piece> = sk.sketch.entities.iter().filter(|e| !e.construction()).filter_map(|e| sketch_entity(sk, e)).collect();
        if ps.is_empty() {
            return Err(no_input());
        }
        return Ok(Source { pieces: walk_chain(ps, gap, prefix)?, sketch_normal: sk.frame.z.try_normalize() });
    }
    Err(no_input())
}

/// One curve given in a field of another feature (the profile and path of `sweepSurface`): a string is an earlier
/// curve feature, else a sketch (its curves walked into one chain); an object is one curve reference; an array is
/// curve references in chain order. Returns the curve over its range (a composite for several pieces, the piece
/// itself for one) and the sketch normal for a sketch input. `None` for an absent or empty field.
pub(crate) fn field_curve(br: &Brep, state: &RegenState, j: Option<&Json>, gap: f64, prefix: &str) -> Result<Option<(Curve, f64, f64, Option<Vec3>)>, FeatureError> {
    let unresolved = |name: String, i: usize| {
        hint(
            FeatureError::new(codes::MISSING_REFERENCE).with("curve", Json::Str(name)).with("index", Json::Num(i as f64)),
            "the curve reference does not resolve: a sketch curve needs `sketch` and `entity`, an edge `faces` (and `at`), a curve feature an `id` of an earlier curve feature",
        )
    };
    let (ps, normal) = match j {
        None | Some(Json::Null) => return Ok(None),
        Some(Json::Str(id)) => {
            if let Some(ps) = feature_curve(br, state, id) {
                (orient_ordered(ps, gap, prefix)?, None)
            } else {
                let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH).with("sketch", Json::Str(id.clone())))?;
                let ps: Vec<Piece> = sk.sketch.entities.iter().filter(|e| !e.construction()).filter_map(|e| sketch_entity(sk, e)).collect();
                if ps.is_empty() {
                    return Ok(None);
                }
                (walk_chain(ps, gap, prefix)?, sk.frame.z.try_normalize())
            }
        }
        Some(Json::Arr(refs)) => {
            if refs.is_empty() {
                return Ok(None);
            }
            let mut ps = vec![];
            for (i, r) in refs.iter().enumerate() {
                match ref_pieces(br, state, r)? {
                    Some(p) => ps.extend(p),
                    None => return Err(unresolved(r.get_str("entity").or_else(|| r.get_str("id")).unwrap_or("?").to_string(), i)),
                }
            }
            (orient_ordered(ps, gap, prefix)?, None)
        }
        Some(r) => match ref_pieces(br, state, r)? {
            Some(p) => {
                // a sketch curve reference keeps its sketch's normal
                let n = (r.get_str("kind") == Some("sketchCurve")).then(|| r.get_str("sketch").and_then(|s| state.sketch(s)).and_then(|sk| sk.frame.z.try_normalize())).flatten();
                (orient_ordered(p, gap, prefix)?, n)
            }
            None => return Err(unresolved(r.get_str("entity").or_else(|| r.get_str("id")).unwrap_or("?").to_string(), 0)),
        },
    };
    let (c, t0, t1) = single(chain_curve(ps, gap, prefix)?);
    Ok(Some((c, t0, t1, normal)))
}

/// A numeric field as the procedural curve features read it (number, `=expression` or numeric string).
pub(crate) fn num_field(state: &RegenState, f: &Feature, key: &str, default: f64) -> Result<f64, FeatureError> {
    num(state, f, key, default)
}

/// One curve of the chain: the piece itself, or the composite of the pieces.
fn chain_curve(ps: Vec<Piece>, gap: f64, prefix: &str) -> Result<ProcCurve, FeatureError> {
    let scale = 1.0 + ps.iter().map(|p| p.start().len().max(p.end().len())).fold(0.0, f64::max);
    let c = Curve::composite(ps.into_iter().map(|p| (p.c, p.t0, p.t1)).collect(), gap.max(1e-9 * scale))
        .map_err(|e| hint(FeatureError::new(&format!("{prefix}Failed")).with("detail", Json::Str(e)), "the chain could not be joined into one curve"))?;
    let (t0, t1) = c.range();
    Ok(ProcCurve { curve: c, t0, t1 })
}

/// A composite of one piece is that piece over its own range (exact closed forms stay reachable).
fn single(pc: ProcCurve) -> (Curve, f64, f64) {
    match pc.curve {
        Curve::Composite(cc) if cc.segs.len() == 1 => {
            let g = cc.segs.into_iter().next().unwrap();
            (g.curve, g.t0, g.t1)
        }
        c => (c, pc.t0, pc.t1),
    }
}

/// Samples of the input, for a plane normal.
fn chain_samples(ps: &[Piece]) -> Vec<Vec3> {
    ps.iter().flat_map(|p| p.c.sample(p.t0, p.t1, 64)).collect()
}

// ---- output -------------------------------------------------------------------------------------------------

/// Parameters of the joints of `c` inside `(t0, t1)`: of a composite, and of the composite an offset or a projection
/// is taken of (they run at its parameter, so its speed jumps at the same places).
fn breaks(c: &Curve, t0: f64, t1: f64) -> Vec<f64> {
    match c {
        Curve::Composite(cc) => cc.breaks().into_iter().filter(|&b| b > t0 && b < t1).collect(),
        Curve::Offset { base, .. } | Curve::Projected { base, .. } => breaks(base, t0, t1),
        _ => vec![],
    }
}

/// Arc length (the curve splits its own quadrature at the joints).
fn length(pc: &ProcCurve) -> f64 {
    pc.curve.length_between(pc.t0, pc.t1)
}

/// The display polyline: every piece between joints sampled on its own (a line by its ends).
fn display(pc: &ProcCurve) -> Vec<Vec3> {
    let mut ks = vec![pc.t0];
    ks.extend(breaks(&pc.curve, pc.t0, pc.t1));
    ks.push(pc.t1);
    let mut pts = vec![pc.curve.eval(pc.t0)];
    for w in ks.windows(2) {
        let mid = 0.5 * (w[0] + w[1]);
        let straight = match &pc.curve {
            Curve::Line { .. } => true,
            Curve::Composite(cc) => matches!(cc.segs[cc.locate(mid).0].curve, Curve::Line { .. }),
            Curve::Offset { base, .. } | Curve::Projected { base, .. } => match &**base {
                Curve::Composite(cc) => matches!(cc.segs[cc.locate(mid).0].curve, Curve::Line { .. }) && matches!(&pc.curve, Curve::Projected { surf, dir: Some(_), .. } if matches!(**surf, Surface::Plane { .. })),
                Curve::Line { .. } => matches!(&pc.curve, Curve::Projected { surf, dir: Some(_), .. } if matches!(**surf, Surface::Plane { .. })),
                _ => false,
            },
            _ => false,
        };
        let n = if straight { 1 } else { 128 };
        for j in 1..=n {
            pts.push(pc.curve.eval(w[0] + (w[1] - w[0]) * j as f64 / n as f64));
        }
    }
    pts
}

fn cont_name(c: Continuity) -> &'static str {
    match c {
        Continuity::C0 => "C0",
        Continuity::G1 => "G1",
        Continuity::C1 => "C1",
        Continuity::G2 => "G2",
        Continuity::C2 => "C2",
    }
}

/// Certify the definition (its B-spline within `tol`), publish `curves[]` and the marker, keep the definition.
fn publish(br: &mut Brep, state: &mut RegenState, f: &Feature, pc: ProcCurve, kind: &str, prefix: &str, tol: f64, extra: Vec<(String, Json)>) -> Result<(), FeatureError> {
    let (b, bound) = pc.curve.to_nurbs_within(pc.t0, pc.t1, tol).map_err(|e| {
        hint(
            FeatureError::new(&format!("{prefix}Failed")).with("detail", Json::Str(e)).with("tolerance", Json::Num(tol)),
            "the curve could not be converted to a B-spline within `tolerance`: raise `tolerance`, or check that the definition has a solution along the whole curve",
        )
    })?;
    let pts = display(&pc);
    let length = length(&pc);
    let closed = pc.curve.eval(pc.t0).dist(pc.curve.eval(pc.t1)) <= 1e-6 * (1.0 + length);
    let (pieces, corners, cont) = match &pc.curve {
        Curve::Composite(cc) => {
            (cc.segs.len(), cc.joints.iter().filter(|j| !j.is_smooth()).count(), cc.min_continuity())
        }
        _ => (1, 0, Continuity::C2),
    };
    let id = f.id.clone();
    state.result.curves.retain(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(id.as_str()));
    state.result.curves.push(Json::Arr(vec![
        Json::Str(id.clone()),
        Json::Obj(vec![
            ("pts".into(), Json::Arr(pts.iter().map(|p| v3(*p)).collect())),
            ("axis".into(), Json::Bool(false)),
            ("t0".into(), Json::Num(0.0)),
            ("t1".into(), Json::Num(1.0)),
            ("procedural".into(), Json::Str(kind.into())),
        ]),
    ]));
    let mut m = vec![
        ("kind".to_string(), Json::Str(kind.into())),
        ("length".into(), Json::Num(length)),
        ("closed".into(), Json::Bool(closed)),
        ("pieces".into(), Json::Num(pieces as f64)),
        ("corners".into(), Json::Num(corners as f64)),
        ("continuity".into(), Json::Str(cont_name(cont).into())),
        ("degree".into(), Json::Num(b.degree as f64)),
        ("count".into(), Json::Num(b.ctrl.len() as f64)),
        ("deviation".into(), Json::Num(bound)),
    ];
    m.extend(extra);
    state.result.markers.retain(|x| x.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(id.as_str()));
    state.result.markers.push(Json::Arr(vec![Json::Str(id.clone()), Json::Obj(m)]));
    br.proc_curves.retain(|(i, _)| *i != id);
    br.proc_curves.push((id, pc));
    Ok(())
}

fn tolerance(state: &RegenState, f: &Feature) -> Result<f64, FeatureError> {
    let tol = num(state, f, "tolerance", 0.001)?;
    if !(tol.is_finite() && tol > 0.0) {
        return Err(invalid("tolerance", "tolerance is the largest distance (mm) of the B-spline from the definition, above 0", "leave it out for 0.001 mm"));
    }
    Ok(tol)
}

// ---- compositeCurve -----------------------------------------------------------------------------------------

/// `compositeCurve`: the input chain as one curve.
pub fn exec_composite_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let src = source(br, state, f, "compositeCurve")?;
    let tol = tolerance(state, f)?;
    let gap = num(state, f, "gapTolerance", DEFAULT_GAP)?;
    let pc = chain_curve(src.pieces, gap, "compositeCurve")?;
    publish(br, state, f, pc, "composite", "compositeCurve", tol, vec![])
}

// ---- offsetCurve --------------------------------------------------------------------------------------------

/// Unit tangent jump between the end of `a` and the start of `b` (a corner of the chain).
fn corner_at(ps: &[Piece]) -> Option<(usize, Vec3)> {
    for i in 1..ps.len() {
        let (ta, tb) = (ps[i - 1].c.tangent(ps[i - 1].t1), ps[i].c.tangent(ps[i].t0));
        if ta.dot(tb) < 1.0 - 1e-9 {
            return Some((i, ps[i].start()));
        }
    }
    // a closed chain: the joint of its end and its start
    if let (Some(a), Some(b)) = (ps.last(), ps.first()) {
        let scale = 1.0 + a.end().len();
        if ps.len() > 1 && a.end().dist(b.start()) <= 1e-9 * scale && a.c.tangent(a.t1).dot(b.c.tangent(b.t0)) < 1.0 - 1e-9 {
            return Some((0, b.start()));
        }
    }
    // a tangent discontinuity inside a B-spline piece (a knot of full multiplicity)
    for p in ps {
        if let Curve::BSpline(b) = &p.c {
            let deg = b.degree.max(1);
            let mut ks = b.knots.clone();
            ks.dedup_by(|x, y| (*x - *y).abs() < 1e-14);
            for k in ks.into_iter().filter(|&k| k > p.t0 && k < p.t1) {
                let mult = b.knots.iter().filter(|&&x| (x - k).abs() < 1e-14).count();
                if mult >= deg {
                    let h = 1e-7 * (p.t1 - p.t0);
                    let (tl, tr) = (p.c.tangent(k - h), p.c.tangent(k + h));
                    if tl.dot(tr) < 1.0 - 1e-9 {
                        return Some((0, p.c.eval(k)));
                    }
                }
            }
        }
    }
    None
}

/// The ways the offset of `base` goes wrong (ck-geom offset.rs): a zero tangent or a tangent along the normal (no
/// offset direction), a distance beyond the radius of curvature (the offset reverses), and for a curve in the
/// offset plane an offset that runs closer than the distance to another part of the curve (self-intersecting).
fn check_offset(off: &Curve, base: &Curve, t0: f64, t1: f64, dist: f64, plane: Option<Vec3>) -> Result<(), FeatureError> {
    let n = 256;
    let hint_dir = "give the offset plane with `plane` or `normal` so that it is not perpendicular to the curve anywhere, or offset on a support `face`";
    let hint_fold = "use a smaller distance (below the smallest radius of curvature on the side the offset moves to), or offset to the other side";
    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let c = base.derivs(t, 2);
        let p = off.derivs(t, 1);
        let at = base.eval(t);
        if !(c[1].len() > 1e-12) || (p[0].dist(c[0]) - dist.abs()).abs() > 1e-6 * (1.0 + dist.abs()) {
            return Err(hint(
                FeatureError::new("offsetCurveDirection")
                    .with("detail", Json::Str(format!("offset direction undefined at parameter {t}: zero tangent, or tangent parallel to the offset-plane normal")))
                    .with("at", v3(at)),
                hint_dir,
            ));
        }
        if !(p[1].dot(c[1]) > 0.0) {
            let k = c[1].cross(c[2]).len() / c[1].len().powi(3);
            let radius = if k > 1e-300 { 1.0 / k } else { f64::INFINITY };
            let mut e = FeatureError::new("offsetCurveFold")
                .with("detail", Json::Str(format!("offset distance {dist} exceeds the radius of curvature near parameter {t}: the offset reverses (self-intersecting)")))
                .with("at", v3(at))
                .with("distance", Json::Num(dist));
            if radius.is_finite() {
                e = e.with("radius", Json::Num(radius));
            }
            return Err(hint(e, hint_fold));
        }
    }
    // global: a planar curve offset in its own plane must not come closer than |dist| to another part of it
    if let Some(nrm) = plane {
        let pts = base.sample(t0, t1, 64);
        let o = pts[0];
        let flat = pts.iter().all(|q| (*q - o).dot(nrm).abs() <= 1e-6 * (1.0 + q.dist(o)));
        if flat && dist != 0.0 {
            for i in 0..=128 {
                let t = t0 + (t1 - t0) * i as f64 / 128.0;
                let q = off.eval(t);
                let (_, foot) = base.closest_point(q);
                let d = foot.dist(q);
                if d < dist.abs() * (1.0 - 1e-3) {
                    return Err(hint(
                        FeatureError::new("offsetCurveFold")
                            .with("detail", Json::Str(format!("the offset runs {d:.6} from another part of the curve, closer than the distance {} (self-intersecting offset)", dist.abs())))
                            .with("at", v3(base.eval(t)))
                            .with("distance", Json::Num(dist)),
                        hint_fold,
                    ));
                }
            }
        }
    }
    Ok(())
}

/// `offsetCurve`: the input chain offset by `distance` in a plane (`plane`, `normal`, else the sketch plane or the
/// plane of the curve) or in the tangent planes of a support `face`.
pub fn exec_offset_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let src = source(br, state, f, "offsetCurve")?;
    let tol = tolerance(state, f)?;
    let gap = num(state, f, "gapTolerance", DEFAULT_GAP)?;
    let dist = match f.raw.get("distance") {
        None | Some(Json::Null) => return Err(invalid("distance", "distance is required", "give the offset distance in mm (positive: to the side of C' x N, see the docs)")),
        _ => num(state, f, "distance", 0.0)?,
    };
    if !dist.is_finite() || dist == 0.0 {
        return Err(invalid("distance", "distance is a finite distance (mm), not 0", "a zero offset is the curve itself: use compositeCurve to copy it"));
    }
    if let Some((index, at)) = corner_at(&src.pieces) {
        return Err(hint(
            FeatureError::new("offsetCurveCorner")
                .with("detail", Json::Str("the curve has a tangent discontinuity: its offset has a gap and is not one curve".into()))
                .with("index", Json::Num(index as f64))
                .with("at", v3(at)),
            "round the corner (a sketch fillet) or offset the smooth pieces separately",
        ));
    }
    // the offset reference
    let mut extra: Vec<(String, Json)> = vec![("distance".into(), Json::Num(dist))];
    let mut d = dist;
    let (dir, plane) = if let Some(face) = f.raw.get_str("face") {
        let slot = br.slot_of_face(face).ok_or_else(|| br.missing_face(face))?;
        let body = br.slot_body(slot).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
        let fid = body.find_face(face).ok_or_else(|| crate::refs::missing_face([body], face))?;
        let fc = body.face(fid);
        extra.push(("face".into(), Json::Str(face.into())));
        // the sign follows the face's outward normal, not the surface's natural one
        if !fc.sense {
            d = -dist;
        }
        (OffsetRef::Surface(Box::new(fc.surface.clone())), None)
    } else {
        let n = if let Some(j) = f.raw.get("normal").filter(|j| !matches!(j, Json::Null)) {
            Some(json_vec(j).ok_or_else(|| invalid("normal", "normal is a direction [x, y, z]", "give three numbers"))?)
        } else if let Some(r) = f.raw.get("plane").filter(|j| !matches!(j, Json::Null)) {
            let fr = GeomRef::from_json(r).and_then(|g| super::pattern::plane_frame(br, state, &g)).ok_or_else(|| {
                hint(FeatureError::new(codes::MISSING_REFERENCE).with("field", Json::Str("plane".into())), "`plane` is a standard plane, a plane feature or a planar face")
            })?;
            Some(fr.z)
        } else {
            src.sketch_normal.or_else(|| plane_normal(&chain_samples(&src.pieces)))
        };
        let Some(n) = n else {
            return Err(hint(
                FeatureError::new("offsetCurvePlane").with("detail", Json::Str("the curve does not lie in one plane (or is straight), so the offset plane is not defined".into())),
                "give the offset plane with `plane` or `normal`, or a support `face`",
            ));
        };
        let Some(n) = n.try_normalize() else {
            return Err(hint(FeatureError::new("offsetCurveDirection").with("detail", Json::Str("offset plane normal is zero".into())), "give a non-zero `normal`"));
        };
        extra.push(("normal".into(), v3(n)));
        (OffsetRef::Normal(n), Some(n))
    };
    let (base_c, t0, t1) = single(chain_curve(src.pieces, gap, "offsetCurve")?);
    let off = match &dir {
        OffsetRef::Normal(n) => Curve::offset(base_c.clone(), d, *n),
        OffsetRef::Surface(s) => Curve::offset_on(base_c.clone(), d, (**s).clone()),
    }
    .map_err(|e| hint(FeatureError::new("offsetCurveDirection").with("detail", Json::Str(e)), "give a non-zero `normal`"))?;
    check_offset(&off, &base_c, t0, t1, d, plane)?;
    // the definition, or its closed form where there is one (the offset of a line, of a circle in its plane)
    let off = match &off {
        Curve::Offset { base, dist, dir } => ck_geom::procedural::offset_exact(base, *dist, dir).unwrap_or(off.clone()),
        _ => off,
    };
    publish(br, state, f, ProcCurve { curve: off, t0, t1 }, "offset", "offsetCurve", tol, extra)
}

// ---- projectCurve (procedural) ------------------------------------------------------------------------------

/// `projectCurve` with `procedural: true`: the input chain projected onto ONE face, to the closest point
/// (`along: "normal"`) or along a direction (`along: [x,y,z]`; default the sketch plane normal or the plane of the
/// curve, else the closest point).
pub fn exec_project_curve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() && br.surfs.is_empty() {
        return Err(FeatureError::new("projectCurveNoFaces"));
    }
    let src = source(br, state, f, "projectCurve")?;
    let tol = tolerance(state, f)?;
    let gap = num(state, f, "gapTolerance", DEFAULT_GAP)?;
    let faces: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    let face = match faces.as_slice() {
        [one] => *one,
        _ => {
            return Err(invalid(
                "faces",
                &format!("a procedural projection goes onto one face; {} given", faces.len()),
                "name exactly one face in `faces`, or leave `procedural` out to project onto several faces as one fitted B-spline",
            ))
        }
    };
    let slot = br.slot_of_face(face).ok_or_else(|| br.missing_face(face))?;
    let body = br.slot_body(slot).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let fid = body.find_face(face).ok_or_else(|| crate::refs::missing_face([body], face))?;
    let surf: Surface = body.face(fid).surface.clone();
    let along = match f.raw.get("along") {
        None | Some(Json::Null) => src.sketch_normal.or_else(|| plane_normal(&chain_samples(&src.pieces))),
        Some(Json::Str(s)) if s == "normal" => None,
        Some(j) => match json_vec(j).and_then(|v| v.try_normalize()) {
            Some(v) => Some(v),
            None => return Err(invalid("along", "along is \"normal\" (closest point) or a non-zero direction [x, y, z]", "give \"normal\" or three numbers")),
        },
    };
    let (base_c, t0, t1) = single(chain_curve(src.pieces, gap, "projectCurve")?);
    let pc = Curve::projected(base_c, surf, along).map_err(|e| hint(FeatureError::new("projectCurveFailed").with("detail", Json::Str(e)), "give a non-zero `along` direction"))?;
    // the projection must exist along the whole curve and stay inside the trimmed face
    let misses = |at: Vec3, why: &str| {
        hint(
            FeatureError::new("projectCurveMisses").with("face", Json::Str(face.into())).with("at", v3(at)).with("detail", Json::Str(why.into())),
            "the projection must land on the face along the whole curve: pick the face it falls on, move the curve over the face, or change `along`",
        )
    };
    if !ck_geom::procedural::is_defined_on(&pc, t0, t1) {
        return Err(misses(pc.eval(t0), "the line along the direction misses the surface for part of the curve"));
    }
    let mut cache = LoopCache::new();
    let n = 64;
    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let p = pc.eval(t);
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(misses(p, "the projection has no solution here"));
        }
        if point_in_face_cached(body, fid, p, 1e-5, &mut cache) == Where::Out {
            return Err(misses(p, "the projected curve leaves the face here"));
        }
    }
    let mut extra = vec![("face".to_string(), Json::Str(face.into())), ("mode".into(), Json::Str(if along.is_some() { "direction" } else { "normal" }.into()))];
    if let Some(a) = along {
        extra.push(("along".into(), v3(a)));
    }
    publish(br, state, f, ProcCurve { curve: pc, t0, t1 }, "projected", "projectCurve", tol, extra)
}

// ---- consumers ----------------------------------------------------------------------------------------------

/// A procedural curve feature whose pieces are all lines and circle arcs as exact sweep path pieces (`None`
/// otherwise: the path is then the spline through its points, like any curve feature).
pub fn exact_path(br: &Brep, id: &str) -> Option<Vec<PathSeg>> {
    let (_, pc) = br.proc_curves.iter().rev().find(|(i, _)| i == id)?;
    let pieces: Vec<(&Curve, f64, f64)> = match &pc.curve {
        Curve::Composite(cc) => cc.segs.iter().map(|g| (&g.curve, g.t0, g.t1)).collect(),
        c => vec![(c, pc.t0, pc.t1)],
    };
    pieces
        .into_iter()
        .map(|(c, a, b)| match c {
            Curve::Line { .. } => Some(PathSeg::Line { a: c.eval(a), b: c.eval(b) }),
            Curve::Circle { frame, r } => Some(PathSeg::Arc { frame: *frame, r: *r, a0: a, a1: b }),
            _ => None,
        })
        .collect()
}
