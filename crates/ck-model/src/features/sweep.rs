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
    let FeatureKind::Helix { defined_by, left_hand, reverse, .. } = &f.kind else { unreachable!() };
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
        defined_by: match defined_by {
            DefinedBy::PitchRevs => ck_ops::helix::DefinedBy::PitchRevs,
            _ => ck_ops::helix::DefinedBy::parse(f.raw.get("definedBy").and_then(|v| v.as_str()).unwrap_or("")),
        },
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

/// `sweep`.
pub fn exec_sweep(br: &Brep, state: &RegenState, f: &Feature) -> Result<Tool, FeatureError> {
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
            let h = br.helices.iter().rev().find(|(i, _)| i == id).map(|(_, h)| *h).ok_or_else(missing)?;
            vec![PathSeg::Helix { h, rev: false }]
        }
        GeomRef::Other(j) if j.get("kind").and_then(|k| k.as_str()) == Some("sketch") => {
            let id = j.get("id").and_then(|v| v.as_str()).ok_or_else(missing)?;
            sketch_path(state.sketch(id).ok_or_else(missing)?)?
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
    if f.raw.get("guides").and_then(|g| g.as_arr()).map_or(false, |g| !g.is_empty()) {
        return Err(FeatureError::new(&todo("sweep guides")));
    }
    // `twist` is in degrees (f626: twist * pi / 180)
    let twist = state.num(f, "twist", 0.0)?.to_radians();
    let body = ck_ops::sweep::sweep(&f.id, &profile, &segs, twist).map_err(|e| FeatureError::new(&e))?;
    // Guard: ck-ops booleans cannot yet assemble a cut against the spline
    // faces of a screw sweep. Probe (ck_ops::sweep::tests::cut_timing, a
    // trapezoid groove swept 0.25 turn into a bored tube): the SSI curves and
    // the cut-plane arcs are all found (rails are within 3e-7 of the screw),
    // but the plane face z=0 of the tube is not re-looped around the groove
    // opening: the fragment inside the opening is kept (open edge on the
    // bore arc) and the face splits `C0~1` / `C0~2` -> booleanFailed. On a
    // 7-19 turn tool the marching also takes minutes. Fail fast with a TODO
    // code (the pre-sweep behaviour) until that is fixed in ck-ops/boolean.
    if SPLINE_TOOL_NEEDS_BOOLEAN && br.body.is_some() && body.face_ids().any(|f| matches!(body.face(f).surface, ck_geom::surface::Surface::BSpline(_))) {
        return Err(FeatureError::new(&todo("boolean with a swept spline (screw) tool")));
    }
    Ok(Tool { body, combine, recipe: None })
}

/// Set to `false` once ck-ops booleans handle helical spline tools.
const SPLINE_TOOL_NEEDS_BOOLEAN: bool = true;
