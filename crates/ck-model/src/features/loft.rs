//! `loft` (ck-model/src/features/loft.rs): sketches -> ck-ops loft.
//!
//! Check order (oracle probes):
//! 1. every `sketches` entry names a regenerated sketch (`missingSketch`);
//! 2. fewer than two sketches -> `loftProfiles`;
//! 3. per sketch, in order: a solid loft needs a region (a non-finite point
//!    is `invalidValue {point, detail, hint}` first, see
//!    [`crate::features::extrude::finite_points`]; none ->
//!    `noClosedProfile`, several -> `loftProfile`; islands are ignored);
//!    a surface loft takes one region (closed profile) or, with no region,
//!    the sketch's curves as ONE open chain (anything else ->
//!    `loftProfile`; an empty sketch too);
//! 4. ck-ops: mixed open / closed (`loftProfileOpen`), all profiles in one
//!    plane (`loftProfiles`), `loftProfileNotPlanar`, `loftFailed`;
//!    the `nothingToCut` of a solid `cut` without a body comes after the
//!    profile checks (and before the geometry).
//!
//! `op` is ignored for `surface: true`. A solid loft joins the body like an
//! extrude (`op: "cut"` subtracts).
//!
//! A sketch holding exactly ONE point entity (no other non-construction
//! entity) is a point (apex) section: only the first and/or last sketch may
//! be one; the loft converges to that point (no cap there; guides, surface
//! lofts and a point in the middle are `loftFailed`).
//!
//! `centreLine {sketch}`: the loft follows a planar open chain (hulls, bottles, ducts, handles); see
//! [`check_centre_line`] and ck-ops `loft::centre` (a moving frame carries the sections; a straight chain is a
//! no-op). Refusals: `loftCentreLineInvalid`, `Kink`, `Reach`, `Tight`, `Apex`, `Guides`, each with `detail` + `hint`.
//!
//! Wire: `markers[] = [id, {loops, starts, guided, tension}]` with `loops`
//! = per sketch, 16 samples per profile piece in sketch order
//! (`{p: [x,y,z], f: arc fraction}`), `starts` = the matched start point of
//! every profile, `guided: false`, `tension: []`. A surface loft registers
//! as a sheet (`surfaces[]`), not in `body`.

use super::{Brep, FeatureKind, Op};
use crate::regen::{codes, FeatureError, RegenState, SketchResult};
use crate::sketch::profile::{open_chain, Seg};
use crate::Feature;
use ck_geom::{Frame, Vec3};
use ck_json::Json;
use ck_ops::loft::{loft, LoftError, LoftOpts, ProfileIn, SegIn};
use ck_topo::make::curve2_to_3d;

pub const LOFT_PROFILE: &str = "loftProfile";

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

fn err(e: LoftError) -> FeatureError {
    let mut fe = FeatureError::new(&e.code);
    // `loftFailed` never leaves with empty params: a missing detail / hint gets a generic one
    let failed = e.code == ck_ops::loft::FAILED;
    let detail = e.detail.or_else(|| failed.then(|| "the loft could not be built from these sections".to_string()));
    let hint = e.hint.or_else(|| failed.then(|| "simplify the sections (the same number of points in every section, no nearly-coincident points) or add intermediate sections".to_string()));
    if let Some(d) = detail {
        fe = fe.with("detail", Json::Str(d));
    }
    if let Some(h) = hint {
        fe = fe.with("hint", Json::Str(h));
    }
    fe
}

/// World-space pieces of a sketch loop / chain.
fn segs_in(sk: &SketchResult, segs: &[Seg]) -> Result<Vec<SegIn>, FeatureError> {
    let mut out = vec![];
    for s in segs {
        let ps = super::extrude::profile_seg(s)?;
        let curve = curve2_to_3d(&sk.frame, &ps.curve, Vec3::new(0.0, 0.0, 0.0));
        out.push(SegIn { tag: ps.tag, curve, t0: ps.t0, t1: ps.t1, rev: ps.rev });
    }
    Ok(out)
}

/// A sketch that holds exactly one point entity (and no other real curve) is a
/// point (apex) section: its world position.
fn point_of(sk: &SketchResult) -> Option<Vec3> {
    use crate::sketch::SketchEntity;
    let mut pts = sk.sketch.entities.iter().filter_map(|e| match e {
        SketchEntity::Point { p, .. } => Some(Some(p)),
        e if e.construction() => None,
        _ => Some(None),
    });
    let first = pts.next()??;
    if pts.next().is_some() {
        return None;
    }
    let [x, y] = sk.sketch.point_xy(first)?;
    Some(sk.frame.o + sk.frame.x * x + sk.frame.y * y)
}

/// The profile of one sketch (see the module header for the rules).
fn profile_of(sk: &SketchResult, surface: bool) -> Result<(ProfileIn, Vec<Seg>), FeatureError> {
    super::extrude::finite_points(sk)?;
    let regions = &sk.profiles.regions;
    if regions.len() > 1 {
        return Err(FeatureError::new(LOFT_PROFILE));
    }
    if regions.is_empty() {
        if let Some(apex) = point_of(sk) {
            // a point section: no pieces, the apex is the frame origin (see ck_ops::loft)
            return Ok((ProfileIn { frame: Frame { o: apex, ..sk.frame }, closed: true, segs: vec![] }, vec![]));
        }
    }
    if let Some(r) = regions.first() {
        let segs = &r.outer;
        return Ok((ProfileIn { frame: sk.frame, closed: true, segs: segs_in(sk, segs)? }, segs.clone()));
    }
    if !surface {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    let chain = open_chain(&sk.sketch).ok_or_else(|| FeatureError::new(LOFT_PROFILE))?;
    Ok((ProfileIn { frame: sk.frame, closed: false, segs: segs_in(sk, &chain)? }, chain))
}

/// `{p, f}` samples of one profile (see [`ck_ops::loft::marker_samples`]).
fn loop_samples(p: &ProfileIn) -> Json {
    if ck_ops::loft::is_point_profile(p) {
        return Json::Arr(vec![Json::Obj(vec![("p".into(), v3(p.frame.o)), ("f".into(), Json::Num(0.0))])]);
    }
    Json::Arr(
        ck_ops::loft::marker_samples(p)
            .into_iter()
            .map(|(pt, f)| Json::Obj(vec![("p".into(), v3(pt)), ("f".into(), Json::Num(f))]))
            .collect(),
    )
}

/// The guide curves of `guides[]` as polylines. Items: `{sketch}` (an open
/// chain of sketch curves; a bare id string is the same), with `influence`
/// (`next` default; `sharp` / `global` are refused: `loftFailed`) and
/// `tangency` (edge guides only). Edge and curve-feature guides are refused
/// (`loftGuideInvalid`); an unusable sketch is `loftGuideInvalid`.
fn guides_of(state: &RegenState, f: &Feature) -> Result<Vec<ck_ops::loft::guides::GuideIn>, FeatureError> {
    let Some(arr) = f.raw.get("guides").and_then(|g| g.as_arr()) else { return Ok(vec![]) };
    let bad = |detail: &str| FeatureError::new("loftGuideInvalid").with("detail", Json::Str(detail.into()));
    let mut out = vec![];
    for g in arr {
        let id = match g {
            Json::Str(s) => Some(s.as_str()),
            _ => g.get_str("sketch"),
        };
        if let Some(inf) = g.get_str("influence") {
            if inf != "next" {
                return Err(FeatureError::new(ck_ops::loft::FAILED).with("detail", Json::Str(format!("guide influence '{inf}' is not supported"))).with("hint", Json::Str("use influence 'next' (the default)".into())));
            }
        }
        let Some(id) = id else { return Err(bad("only sketch guides are supported")) };
        let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let chain = open_chain(&sk.sketch).ok_or_else(|| bad("the guide sketch is not one open chain of curves"))?;
        let mut pts: Vec<Vec3> = vec![];
        for s in segs_in(sk, &chain)? {
            let Some(g) = ck_ops::loft::stations::geom_of(&s.curve, s.t0, s.t1, s.rev) else { return Err(bad("unsupported guide curve")) };
            for k in 0..=32 {
                if k == 0 && !pts.is_empty() {
                    continue;
                }
                pts.push(g.at(k as f64 / 32.0));
            }
        }
        out.push(ck_ops::loft::guides::GuideIn { pts });
    }
    Ok(out)
}

/// `startNormal` / `endNormal`: the tension (a number), `true` = 1.
fn tension(f: &Feature, key: &str) -> Option<f64> {
    match f.raw.get(key)? {
        Json::Num(t) if *t > 0.0 => Some(*t),
        Json::Bool(true) => Some(1.0),
        _ => None,
    }
}

/// `centreLine`: a planar sketch chain (the spine) the loft follows (ck-ops `loft::centre`: a moving frame along the
/// spine carries the sections; a STRAIGHT centre line, slanted or not, leaves the loft unchanged). Here: the sketch
/// must be one open chain of supported curves (`loftCentreLineInvalid`) without a corner between its entities
/// (`loftCentreLineKink`). ck-ops adds `loftCentreLineReach` (the chain does not reach a profile),
/// `loftCentreLineTight {detail}` (bends tighter than the sections allow), `loftCentreLineApex` and
/// `loftCentreLineGuides` (a curved centre line with a point end section / with guides).
fn check_centre_line(state: &RegenState, f: &Feature) -> Result<Option<ck_ops::loft::centre::Spine>, FeatureError> {
    let Some(c) = f.raw.get("centreLine").filter(|c| !matches!(c, Json::Null)) else { return Ok(None) };
    let bad = |code: &str, detail: &str, hint: &str| FeatureError::new(code).with("detail", Json::Str(detail.into())).with("hint", Json::Str(hint.into()));
    let Some(id) = c.get_str("sketch") else {
        return Err(bad("loftCentreLineInvalid", "only sketch centre lines are supported", "draw the centre line as an open chain in a sketch and select that sketch"));
    };
    let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let chain = open_chain(&sk.sketch).ok_or_else(|| bad("loftCentreLineInvalid", "the centre line is not one open chain of curves", "use a sketch with one open chain of lines, arcs or splines"))?;
    let mut pts: Vec<Vec3> = vec![];
    let mut joints: Vec<usize> = vec![];
    for s in segs_in(sk, &chain)? {
        let Some(g) = ck_ops::loft::stations::geom_of(&s.curve, s.t0, s.t1, s.rev) else {
            return Err(bad("loftCentreLineInvalid", "unsupported centre line curve", "use lines, arcs or splines in the centre line sketch"));
        };
        if !pts.is_empty() {
            joints.push(pts.len().saturating_sub(1));
        }
        for k in 0..=CENTRE_SAMPLES {
            if k == 0 && !pts.is_empty() {
                continue;
            }
            pts.push(g.at(k as f64 / CENTRE_SAMPLES as f64));
        }
    }
    let Some(sp) = ck_ops::loft::centre::Spine::new(&ck_ops::loft::centre::CentreIn { pts, joints, normal: sk.frame.z }) else {
        return Err(bad("loftCentreLineInvalid", "the centre line has no length", "draw a centre line with at least one curve of non-zero length"));
    };
    if !sp.kinks.is_empty() {
        return Err(bad("loftCentreLineKink", "the centre line has a corner: its pieces must run on tangentially", "round the corner (fillet the sketch) or split the loft at the corner"));
    }
    Ok(Some(sp))
}

/// Samples per sketch entity of the centre line.
const CENTRE_SAMPLES: usize = 128;

/// `tension[]` of the marker: one entry per end constraint (`p` the station centroid,
/// `d` the unit direction into the loft, `reach` a quarter of the loft length: RECONSTRUCTED
/// from the oracle's `{end,p,d,reach,tension}`; its `p` / `reach` differ in the low digits).
fn tension_markers(f: &Feature, out: &ck_ops::loft::LoftOut) -> Vec<Json> {
    let mut v = vec![];
    let len = out.end_centroids[0].dist(out.end_centroids[1]);
    for (i, (name, d, key)) in [("start", out.ends.d0, "startNormal"), ("end", out.ends.d1, "endNormal")].into_iter().enumerate() {
        let Some(d) = d else { continue };
        let t = d.len();
        let u = if i == 0 { d / t } else { -(d / t) };
        v.push(Json::Obj(vec![
            ("end".into(), Json::Str(name.into())),
            ("p".into(), v3(out.end_centroids[i])),
            ("d".into(), v3(u)),
            ("reach".into(), Json::Num(len / 4.0)),
            ("tension".into(), Json::Num(tension(f, key).unwrap_or(1.0))),
        ]));
    }
    v
}

/// `loft`.
pub fn exec_loft(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Loft { op, sketches, surface } = &f.kind else { unreachable!() };
    let mut sks = vec![];
    for r in sketches {
        sks.push(state.sketch(r.as_str()).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?.clone());
    }
    if sks.len() < 2 {
        return Err(FeatureError::new(ck_ops::loft::PROFILES));
    }
    let mut profiles = vec![];
    let mut loops = vec![];
    for sk in &sks {
        let (p, _segs) = profile_of(sk, *surface)?;
        loops.push(loop_samples(&p));
        profiles.push(p);
    }
    if !*surface && *op == Op::Cut && br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let guides = guides_of(state, f)?;
    let guided = !guides.is_empty();
    let centre = check_centre_line(state, f)?;
    // tangency to the adjacent faces needs profiles made of body edges
    for k in ["startTangent", "endTangent", "startContinuity", "endContinuity"] {
        if f.raw.get(k).map_or(false, |v| !matches!(v, Json::Null | Json::Bool(false))) {
            return Err(FeatureError::new("loftTangentEdges"));
        }
    }
    let connectors: Vec<Option<f64>> = f
        .raw
        .get("connectors")
        .and_then(|c| c.as_arr())
        .map(|a| a.iter().map(|x| x.as_f64()).collect())
        .unwrap_or_default();
    let opts = LoftOpts { surface: *surface, guides, start_normal: tension(f, "startNormal"), end_normal: tension(f, "endNormal"), connectors, apex_tangent: f.raw.get("apexTangent").and_then(|v| v.as_f64()).filter(|t| t.is_finite()), centre };
    let out = loft(&f.id, &profiles, &opts).map_err(err)?;
    let marker = Json::Obj(vec![
        ("loops".into(), Json::Arr(loops)),
        ("starts".into(), Json::Arr(out.starts.iter().map(|p| v3(*p)).collect())),
        ("guided".into(), Json::Bool(guided)),
        ("tension".into(), Json::Arr(tension_markers(f, &out))),
    ]);
    state.result.markers.retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|j| j.as_str()) != Some(f.id.as_str()));
    state.result.markers.push(Json::Arr(vec![Json::Str(f.id.clone()), marker]));
    if *surface {
        br.surfs.retain(|(id, _)| id != &f.id);
        br.surfs.push((f.id.clone(), out.body));
        return Ok(());
    }
    let combine = if *op == Op::Cut { super::extrude::Combine::Cut } else { super::extrude::Combine::Add };
    br.apply(&f.id, super::extrude::Tool { body: out.body, combine, recipe: None })
}
