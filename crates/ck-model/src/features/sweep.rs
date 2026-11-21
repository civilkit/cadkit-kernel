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

/// A path refusal with its plain message and params (`message`, `supported`, `path` = piece types, `joint`, `at`, `hint`).
fn path_err(code: &str, why: &str, hint: &str, joint: Option<usize>, at: Option<Vec3>, pieces: &[PathSeg]) -> FeatureError {
    let kinds: Vec<&str> = pieces.iter().map(ck_ops::sweep::kind_name).collect();
    FeatureError::new(&ck_ops::sweep::path_problem(code, why, hint, joint, at, &kinds))
}

/// Pieces (with a label each: the entity id) -> one chain walked from a free end. Refuses, naming the place: a
/// closed loop, a branch, pieces that are not connected to the chain.
fn chain_pieces(pieces: Vec<PathSeg>, labels: &[String], what: &str) -> Result<Vec<PathSeg>, FeatureError> {
    let near = |a: Vec3, b: Vec3| a.dist(b) < 1e-6;
    let hint_open = format!("make the {what} one open chain: its pieces meet end to end at shared points, with no branch and no loop");
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
    let Some((i0, flip0)) = start else {
        return Err(path_err(
            PATH_INVALID,
            &format!("the {what} is a closed loop of {} pieces (every end meets another piece), so it has no start", pieces.len()),
            "a sweep path is open: remove one piece or leave a gap at the end where the sweep should start",
            None,
            Some(pieces[0].start()),
            &pieces,
        ));
    };
    let mut used = vec![false; pieces.len()];
    used[i0] = true;
    let mut chain = vec![if flip0 { pieces[i0].reversed() } else { pieces[i0].clone() }];
    let mut order = vec![i0];
    while chain.len() < pieces.len() {
        let tip = chain.last().unwrap().end();
        let hits: Vec<(usize, bool)> = pieces
            .iter()
            .enumerate()
            .filter(|(j, _)| !used[*j])
            .flat_map(|(j, p)| [(near(p.start(), tip), j, false), (near(p.end(), tip), j, true)])
            .filter(|(m, _, _)| *m)
            .map(|(_, j, f)| (j, f))
            .collect();
        let Some(&(j, flip)) = hits.first() else {
            let left: Vec<&str> = (0..pieces.len()).filter(|j| !used[*j]).map(|j| labels[j].as_str()).collect();
            return Err(path_err(
                PATH_INVALID,
                &format!("the {what} breaks after piece {} ({}) at the marked point: the pieces {} do not connect to it (nothing within 1e-6 mm of its end)", chain.len() - 1, labels[*order.last().unwrap()], left.join(", ")),
                &format!("move the end points together so the {what} is one chain (the sketch solver can do it with coincident constraints); to join pieces that are further apart than 1e-6 mm, use a compositeCurve with a gapTolerance and sweep along it"),
                Some(chain.len() - 1),
                Some(tip),
                &pieces,
            ));
        };
        if hits.iter().map(|h| h.0).collect::<std::collections::BTreeSet<_>>().len() > 1 {
            return Err(path_err(
                PATH_INVALID,
                &format!("the {what} branches at piece {} ({}): more than one piece starts at its end", chain.len() - 1, labels[*order.last().unwrap()]),
                &hint_open,
                Some(chain.len() - 1),
                Some(tip),
                &pieces,
            ));
        }
        used[j] = true;
        order.push(j);
        chain.push(if flip { pieces[j].reversed() } else { pieces[j].clone() });
    }
    Ok(chain)
}

/// Open sketch -> connected chain of path pieces (construction entities are
/// skipped). The chain is walked from one of its two free ends; ck-ops
/// orients it towards the profile.
fn sketch_path(sk: &crate::regen::SketchResult) -> Result<Vec<PathSeg>, FeatureError> {
    let bad = || path_err(PATH_INVALID, "a point of the path sketch is missing", "check the sketch's points", None, None, &[]);
    let fr = sk.frame;
    let mut pieces: Vec<PathSeg> = vec![];
    let mut labels: Vec<String> = vec![];
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
            SketchEntity::Point { .. } => continue,
            other => {
                return Err(path_err(
                    PATH_INVALID,
                    &format!("the path sketch holds a {} ({}): a sketch path is made of lines and arcs", other.type_name(), other.id()),
                    "replace it by lines and arcs, mark it as construction geometry, or draw the path as a sketch3d spline",
                    None,
                    None,
                    &pieces,
                ))
            }
        }
        labels.push(e.id().to_string());
    }
    if pieces.is_empty() {
        return Err(path_err(PATH_INVALID, "the path sketch has no lines or arcs", "draw the path as lines and arcs in the sketch", None, None, &[]));
    }
    chain_pieces(pieces, &labels, "path sketch")
}

/// A `sketch3d` path: either one `spline` entity (natural cubic spline through its points, the curve the
/// client draws, swept with the section carried along it by rotation-minimizing frames), or a connected chain of
/// `line` and `arc` entities (every joint tangent, or a corner between two lines).
pub(crate) fn sketch3_path(j: &Json) -> Result<Vec<PathSeg>, FeatureError> {
    let bad = || path_err(PATH_INVALID, "the 3D sketch path could not be read: a point it refers to is missing", "check that every entity's points exist in the sketch", None, None, &[]);
    let sk = j.get("sketch").unwrap_or(j);
    let all = sk.get_arr("entities").unwrap_or(&[]);
    let live = |e: &&Json| e.get("construction").and_then(|c| c.as_bool()) != Some(true);
    let splines: Vec<&Json> = all.iter().filter(live).filter(|e| e.get_str("type") == Some("spline")).collect();
    let types: Vec<String> = all.iter().filter(live).filter(|e| e.get_str("type") != Some("point")).map(|e| format!("{} {}", e.get_str("type").unwrap_or("?"), e.get_str("id").unwrap_or("?"))).collect();
    let unsupported = |why: String, hint: &str| path_err(PATH_INVALID, &why, hint, None, None, &[]);
    let points = sk.get_arr("points").ok_or_else(bad)?;
    let point = |id: &str| -> Option<Vec3> {
        let p = points.iter().find(|p| p.get_str("id") == Some(id))?;
        Some(Vec3::new(p.get_f64("x").unwrap_or(0.0), p.get_f64("y").unwrap_or(0.0), p.get_f64("z").unwrap_or(0.0)))
    };
    if splines.is_empty() {
        // a chain of lines and arcs
        let mut pieces = vec![];
        let mut labels = vec![];
        for e in all.iter().filter(live) {
            let id = e.get_str("id").unwrap_or("?").to_string();
            let pt = |k: &str| e.get_str(k).and_then(&point);
            match e.get_str("type") {
                Some("point") => continue,
                Some("line") => match (pt("p1"), pt("p2")) {
                    (Some(a), Some(b)) => pieces.push(PathSeg::Line { a, b }),
                    _ => return Err(unsupported(format!("the line {id} of the 3D sketch has a point that is not in the sketch"), "check the line's points")),
                },
                Some("arc") => {
                    let (Some(c), Some(a), Some(b)) = (pt("c"), pt("p1"), pt("p2")) else {
                        return Err(unsupported(format!("the arc {id} of the 3D sketch has a point that is not in the sketch"), "check the arc's centre and end points"));
                    };
                    let (u, w) = (a - c, b - c);
                    let (Some(x), Some(z)) = (u.try_normalize(), u.cross(w).try_normalize()) else {
                        return Err(unsupported(format!("the arc {id} of the 3D sketch is degenerate (its end points and centre are on one line)"), "move the arc's centre off the line through its end points"));
                    };
                    let ang = u.cross(w).len().atan2(u.dot(w));
                    pieces.push(PathSeg::Arc { frame: Frame { o: c, x, y: z.cross(x), z }, r: u.len(), a0: 0.0, a1: ang });
                }
                Some(t) => {
                    return Err(unsupported(
                        format!("the 3D sketch holds a {t} ({id}): a 3D sketch path is one spline, or a chain of lines and arcs ({})", types.join(", ")),
                        "draw the path as one spline, or as lines and arcs, and mark the other entities as construction",
                    ))
                }
                None => return Err(bad()),
            }
            labels.push(id);
        }
        if pieces.is_empty() {
            return Err(unsupported("the 3D sketch holds no spline, line or arc to sweep along".into(), "draw the path in the sketch"));
        }
        return chain_pieces(pieces, &labels, "3D sketch path");
    }
    let [e] = splines[..] else {
        return Err(unsupported(
            format!("the 3D sketch holds {} splines ({}): a single spline path is supported, and splines cannot be joined to each other or to lines and arcs", splines.len(), types.join(", ")),
            "draw the whole path as one spline through all its points (a spline through the end points of the pieces), or replace the splines by lines and arcs",
        ));
    };
    if e.get("closed").and_then(|c| c.as_bool()) == Some(true) {
        return Err(unsupported(format!("the spline {} is closed: a sweep path is open", e.get_str("id").unwrap_or("?")), "open the spline (clear `closed`) so it has a start and an end"));
    }
    // (the only other entities a path may hold: points and the lines that give the spline end tangents)
    let eid = e.get_str("id").unwrap_or("");
    let tangent_line = |l: &Json| {
        sk.get_arr("constraints").unwrap_or(&[]).iter().any(|c| {
            let refs: Vec<&str> = c.get_arr("refs").unwrap_or(&[]).iter().filter_map(|r| r.as_str()).collect();
            c.get_str("type") == Some("tangent") && refs.len() == 2 && refs.contains(&eid) && refs.contains(&l.get_str("id").unwrap_or("\u{0}"))
        })
    };
    if let Some(o) = all.iter().filter(live).find(|o| o.get_str("id") != Some(eid) && o.get_str("type") != Some("point") && !(o.get_str("type") == Some("line") && tangent_line(o))) {
        return Err(unsupported(
            format!("the 3D sketch mixes the spline {eid} with a {} ({}): a spline path cannot be joined to lines or arcs, except a line tangent to the spline's end that only sets the end direction ({})", o.get_str("type").unwrap_or("?"), o.get_str("id").unwrap_or("?"), types.join(", ")),
            "draw the whole path as one spline, or make it lines and arcs only (mark helper entities as construction)",
        ));
    }
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
    let sp = ck_ops::spline_path::SplinePath::new(&pts, t0, t1).ok_or_else(|| {
        path_err(PATH_INVALID, &format!("the spline {eid} has {} points and no usable curve through them (it needs at least 2 distinct points)", pts.len()), "give the spline two or more distinct points", None, None, &[])
    })?;
    Ok(vec![PathSeg::Spline { sp: std::sync::Arc::new(sp), rev: false }])
}

/// The pieces of a sweep / path-pattern `path`: `{kind:"curve", id}` (a helix, or any other curve feature as a
/// spline through its points) or `{kind:"sketch", id}` (an open chain of a sketch, or a `sketch3d` spline).
/// No path, or one that does not resolve, is `missingPath`.
pub(crate) fn path_segs(br: &Brep, state: &RegenState, path: Option<&GeomRef>) -> Result<Vec<PathSeg>, FeatureError> {
    let missing = || FeatureError::new(MISSING_PATH);
    Ok(match path.ok_or_else(missing)? {
        GeomRef::Curve(id) => {
            match br.helices.iter().rev().find(|(i, _)| i == id).map(|(_, h)| *h) {
                Some(h) => vec![PathSeg::Helix { h, rev: false }],
                // a procedural curve of lines and arcs: those pieces exactly
                None if super::proc_curve::exact_path(br, id).is_some() => super::proc_curve::exact_path(br, id).unwrap(),
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
    })
}

/// The gap up to which the pieces of `segs` count as joined: the widest gap between neighbouring pieces. A path
/// whose pieces meet closer than `JOIN_TOL` keeps that; a larger gap exists only where the path came from a
/// `compositeCurve` built with a `gapTolerance` that allowed it (a sketch chain is walked at 1e-6), so the sweep
/// accepts what the composite accepted.
pub(crate) fn path_join_tol(segs: &[PathSeg]) -> f64 {
    let gap = segs.windows(2).map(|w| w[0].end().dist(w[1].start())).fold(0.0_f64, f64::max);
    ck_ops::sweep::JOIN_TOL.max(gap * (1.0 + 1e-9))
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
    let segs = path_segs(br, state, path.as_ref())?;
    let profile = sketch_profile(sk)?;
    let combine = Combine::of(*op);
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
    let join_tol = path_join_tol(&segs);
    let body = ck_ops::sweep::sweep_joined(&f.id, &profile, &segs, twist, join_tol).map_err(|e| FeatureError::new(&e))?;
    Ok(Tool { body, combine, recipe: None })
}
