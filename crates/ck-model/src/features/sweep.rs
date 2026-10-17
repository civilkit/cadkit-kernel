//! `helix` curves and `sweep` (ck-model/src/features/sweep.rs).
//!
//! helix (oracle: hex-bolt-m8x40 F11): the first circle of the sketch gives
//! centre / radius; output `curves[] = ["F11", {pts, axis:false, t0:0,
//! t1:angle, helix:{o,a,x,y,k,r,angle}}]` (ck_ops::helix pins the fields);
//! a sketch without a circle -> `helixNeedsCircle` (kernel.strings).
//! sweep (oracle probes, see ck_ops::sweep): `path` is `{kind:"curve", id}`
//! (a helix feature) or `{kind:"sketch", id}` (an open sketch chain of
//! lines / arcs); anything else -> `missingPath`. Check order: path ->
//! profile (`noClosedProfile`) -> `nothingToCut` -> geometry codes.
//! A `{kind:"sketch"}` path may also name a `sketch3d` holding one `spline` entity (natural cubic
//! spline, end tangents from `tangent` constraints to lines): the section is carried along it by a
//! rotation-minimizing frame (ck_ops::sweep::sweep_spline).
//! `cut` sweeps are not flipped (unlike extrude). GUESS: `twist`, `guides`
//! (guide.rs) and `surface` sweeps are not reconstructed yet.

use super::extrude::{sketch_profile, todo, Combine, Tool};
use super::{Brep, DefinedBy, FeatureKind, Op};
use crate::GeomRef;
use ck_core::linalg::Vec3;
use ck_geom::Frame;
use ck_ops::sweep::PathSeg;
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
    let FeatureKind::Helix { left_hand, reverse, .. } = &f.kind else { unreachable!() };
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
        // wire values are pitchRevs / heightRevs / heightPitch (the FeatureKind
        // enum's PitchHeight/RevsHeight names never match), so parse the raw field
        defined_by: ck_ops::helix::DefinedBy::parse(f.raw.get("definedBy").and_then(|v| v.as_str()).unwrap_or("")),
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

pub const MISSING_PATH: &str = "missingPath";
pub const PATH_INVALID: &str = ck_ops::sweep::PATH_INVALID;

fn world(fr: &Frame, p: [f64; 2]) -> Vec3 {
    fr.o + fr.x * p[0] + fr.y * p[1]
}

/// Open sketch -> connected chain of path pieces (construction entities are
/// skipped). The chain is walked from one of its two free ends; ck-ops
/// orients it towards the profile.
fn sketch_path(sk: &crate::regen::SketchResult) -> Result<Vec<PathSeg>, FeatureError> {
    let bad = || FeatureError::new(PATH_INVALID);
    let fr = sk.frame;
    let mut pieces: Vec<PathSeg> = vec![];
    for e in &sk.sketch.entities {
        if e.construction() {
            continue;
        }
        match e {
            SketchEntity::Line { p1, p2, .. } => {
                let (a, b) = (sk.sketch.point_xy(p1).ok_or_else(bad)?, sk.sketch.point_xy(p2).ok_or_else(bad)?);
                pieces.push(PathSeg::Line { a: world(&fr, a), b: world(&fr, b) });
            }
            SketchEntity::Arc { c, p1, p2, .. } => {
                let (c, a, b) = (sk.sketch.point_xy(c).ok_or_else(bad)?, sk.sketch.point_xy(p1).ok_or_else(bad)?, sk.sketch.point_xy(p2).ok_or_else(bad)?);
                let (cw, aw) = (world(&fr, c), world(&fr, a));
                let r = (aw - cw).len();
                let x = (aw - cw).try_normalize().ok_or_else(bad)?;
                let frame = Frame { o: cw, x, y: fr.z.cross(x), z: fr.z };
                let ang = (b[1] - c[1]).atan2(b[0] - c[0]) - (a[1] - c[1]).atan2(a[0] - c[0]);
                let ang = ang.rem_euclid(std::f64::consts::TAU);
                pieces.push(PathSeg::Arc { frame, r, a0: 0.0, a1: if ang < 1e-12 { std::f64::consts::TAU } else { ang } });
            }
            SketchEntity::Point { .. } => {}
            _ => return Err(bad()),
        }
    }
    if pieces.is_empty() {
        return Err(bad());
    }
    // walk the chain from a free end
    let near = |a: Vec3, b: Vec3| a.dist(b) < 1e-6;
    let mut start = None;
    'find: for (i, p) in pieces.iter().enumerate() {
        for (flip, q) in [(false, p.start()), (true, p.end())] {
            let shared = pieces.iter().enumerate().any(|(j, o)| j != i && (near(o.start(), q) || near(o.end(), q)));
            if !shared {
                start = Some((i, flip));
                break 'find;
            }
        }
    }
    let (i0, flip0) = start.ok_or_else(bad)?;
    let mut used = vec![false; pieces.len()];
    used[i0] = true;
    let mut chain = vec![if flip0 { pieces[i0].reversed() } else { pieces[i0].clone() }];
    while chain.len() < pieces.len() {
        let tip = chain.last().unwrap().end();
        let mut found = None;
        for (j, p) in pieces.iter().enumerate() {
            if used[j] {
                continue;
            }
            if near(p.start(), tip) {
                found = Some((j, false));
                break;
            }
            if near(p.end(), tip) {
                found = Some((j, true));
                break;
            }
        }
        let (j, flip) = found.ok_or_else(bad)?;
        used[j] = true;
        chain.push(if flip { pieces[j].reversed() } else { pieces[j].clone() });
    }
    Ok(chain)
}

/// A `sketch3d` path: one `spline` entity (natural cubic spline through its points, the curve the
/// client draws), swept with the section carried along it by Frenet frames.
pub(crate) fn sketch3_path(j: &Json) -> Result<Vec<PathSeg>, FeatureError> {
    let bad = || FeatureError::new(PATH_INVALID);
    let sk = j.get("sketch").unwrap_or(j);
    let all = sk.get_arr("entities").unwrap_or(&[]);
    let live = |e: &&Json| e.get("construction").and_then(|c| c.as_bool()) != Some(true);
    let splines: Vec<&Json> = all.iter().filter(live).filter(|e| e.get_str("type") == Some("spline")).collect();
    let [e] = splines[..] else { return Err(bad()) };
    if e.get("closed").and_then(|c| c.as_bool()) == Some(true) {
        return Err(bad());
    }
    // (the only other entities a path may hold: points and the lines that give the spline end tangents)
    let eid = e.get_str("id").unwrap_or("");
    let tangent_line = |l: &Json| {
        sk.get_arr("constraints").unwrap_or(&[]).iter().any(|c| {
            let refs: Vec<&str> = c.get_arr("refs").unwrap_or(&[]).iter().filter_map(|r| r.as_str()).collect();
            c.get_str("type") == Some("tangent") && refs.len() == 2 && refs.contains(&eid) && refs.contains(&l.get_str("id").unwrap_or("\u{0}"))
        })
    };
    if all.iter().filter(live).any(|o| o.get_str("id") != Some(eid) && o.get_str("type") != Some("point") && !(o.get_str("type") == Some("line") && tangent_line(o))) {
        return Err(bad());
    }
    let points = sk.get_arr("points").ok_or_else(bad)?;
    let point = |id: &str| -> Option<Vec3> {
        let p = points.iter().find(|p| p.get_str("id") == Some(id))?;
        Some(Vec3::new(p.get_f64("x").unwrap_or(0.0), p.get_f64("y").unwrap_or(0.0), p.get_f64("z").unwrap_or(0.0)))
    };
    let ids: Vec<&str> = e.get_arr("pts").ok_or_else(bad)?.iter().filter_map(|i| i.as_str()).collect();
    let pts: Vec<Vec3> = ids.iter().map(|id| point(id)).collect::<Option<_>>().ok_or_else(bad)?;
    // `tangent` constraints against a line fix the end tangents (client `$n`): the line's contact
    // end point minus its far end point, the contact being the spline's first / last point
    let (mut t0, mut t1) = (None, None);
    let (first, last) = (ids.first().copied().unwrap_or(""), ids.last().copied().unwrap_or(""));
    for c in sk.get_arr("constraints").unwrap_or(&[]) {
        let refs: Vec<&str> = c.get_arr("refs").unwrap_or(&[]).iter().filter_map(|r| r.as_str()).collect();
        if c.get_str("type") != Some("tangent") || refs.len() != 2 || !refs.contains(&eid) {
            continue;
        }
        let other = if refs[0] == eid { refs[1] } else { refs[0] };
        let Some(line) = sk.get_arr("entities").unwrap_or(&[]).iter().find(|l| l.get_str("id") == Some(other) && l.get_str("type") == Some("line")) else { continue };
        let (Some(a), Some(b)) = (line.get_str("p1"), line.get_str("p2")) else { continue };
        for (l, r) in [(a, b), (b, a)] {
            let (Some(pl), Some(pr)) = (point(l), point(r)) else { continue };
            if l == first {
                t0 = Some(pl - pr);
            } else if l == last {
                t1 = Some(pl - pr);
            }
        }
    }
    let sp = ck_ops::spline_path::SplinePath::new(&pts, t0, t1).ok_or_else(bad)?;
    Ok(vec![PathSeg::Spline { sp: std::sync::Arc::new(sp), rev: false }])
}

/// `sweep`.
pub fn exec_sweep(br: &Brep, state: &mut RegenState, f: &Feature) -> Result<Tool, FeatureError> {
    let FeatureKind::Sweep { op, path, surface, .. } = &f.kind else { unreachable!() };
    let sk = f
        .raw
        .get("sketch")
        .and_then(|s| s.as_str())
        .and_then(|s| state.sketch(s))
        .ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let missing = || FeatureError::new(MISSING_PATH);
    let segs = match path.as_ref().ok_or_else(missing)? {
        GeomRef::Curve(id) => {
            match br.helices.iter().rev().find(|(i, _)| i == id).map(|(_, h)| *h) {
                Some(h) => vec![PathSeg::Helix { h, rev: false }],
                // any other curve feature (editCurve, projectCurve): swept along a spline through its points
                None => vec![super::curve_ref::curve_path(state, id).ok_or_else(missing)?],
            }
        }
        GeomRef::Other(j) if j.get("kind").and_then(|k| k.as_str()) == Some("sketch") => {
            let id = j.get("id").and_then(|v| v.as_str()).ok_or_else(missing)?;
            match state.sketch(id) {
                Some(sk) => sketch_path(sk)?,
                None => sketch3_path(state.result.sketches3.iter().find(|s| s.0 == id).map(|s| &s.1).ok_or_else(missing)?)?,
            }
        }
        _ => return Err(missing()),
    };
    let profile = sketch_profile(sk)?;
    let combine = if *op == Op::Cut { Combine::Cut } else { Combine::Add };
    if combine == Combine::Cut && br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    if *surface {
        return Err(FeatureError::new(&todo("surface sweep (open profile -> sheet body)")));
    }
    // `twist` is in degrees (f626: twist * pi / 180)
    let twist = state.num(f, "twist", 0.0)?.to_radians();
    if let Some(gs) = f.raw.get("guides").and_then(|g| g.as_arr()).filter(|g| !g.is_empty()) {
        // guides: curve refs (`{kind:"sketchCurve"|"edge"|"curve", ...}`); a ref that does not resolve
        // is `sweepGuideMissing`, a twist with guides `sweepGuidesTwist` (oracle probes)
        let mut polys = vec![];
        for g in gs {
            polys.push(super::curve_ref::ref_samples(br, state, g).ok_or_else(|| FeatureError::new("sweepGuideMissing"))?);
        }
        if twist != 0.0 {
            return Err(FeatureError::new("sweepGuidesTwist"));
        }
        let mode = ck_ops::guide::GuideMode::parse(f.raw.get_str("guideMode"));
        let g = ck_ops::guide::guided_sweep(&f.id, &profile, &segs, &polys, mode).map_err(|e| {
            let fe = FeatureError::new(&e.code);
            match e.detail {
                Some(d) => fe.with("detail", Json::Str(d)),
                None => fe,
            }
        })?;
        // marker `{guides: [contact point on the profile, ..]}`
        let contacts = Json::Arr(g.contacts.iter().map(|p| v3(*p)).collect());
        state.result.markers.retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|j| j.as_str()) != Some(f.id.as_str()));
        state.result.markers.push(Json::Arr(vec![Json::Str(f.id.clone()), Json::Obj(vec![("guides".into(), contacts)])]));
        return Ok(Tool { body: g.body, combine, recipe: None });
    }
    let body = ck_ops::sweep::sweep(&f.id, &profile, &segs, twist).map_err(|e| FeatureError::new(&e))?;
    Ok(Tool { body, combine, recipe: None })
}
