//! `loft` (ck-model/src/features/loft.rs): sketches -> ck-ops loft.
//!
//! Check order (oracle probes):
//! 1. every `sketches` entry names a regenerated sketch (`missingSketch`);
//! 2. fewer than two sketches -> `loftProfiles`;
//! 3. per sketch, in order: a solid loft needs a region (none ->
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
//! Wire: `markers[] = [id, {loops, starts, guided, tension}]` with `loops`
//! = per sketch, 16 samples per profile piece in sketch order
//! (`{p: [x,y,z], f: arc fraction}`), `starts` = the matched start point of
//! every profile, `guided: false`, `tension: []`. A surface loft registers
//! as a sheet (`surfaces[]`), not in `body`.

use super::{Brep, FeatureKind, Op};
use crate::regen::{codes, FeatureError, RegenState, SketchResult};
use crate::sketch::profile::{open_chain, Seg};
use crate::Feature;
use ck_geom::Vec3;
use ck_json::Json;
use ck_ops::loft::{loft, LoftError, LoftOpts, ProfileIn, SegIn};
use ck_topo::make::curve2_to_3d;

pub const LOFT_PROFILE: &str = "loftProfile";

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

fn err(e: LoftError) -> FeatureError {
    let mut fe = FeatureError::new(&e.code);
    if let Some(d) = e.detail {
        fe = fe.with("detail", Json::Str(d));
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

/// The profile of one sketch (see the module header for the rules).
fn profile_of(sk: &SketchResult, surface: bool) -> Result<(ProfileIn, Vec<Seg>), FeatureError> {
    let regions = &sk.profiles.regions;
    if regions.len() > 1 {
        return Err(FeatureError::new(LOFT_PROFILE));
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
    Json::Arr(
        ck_ops::loft::marker_samples(p)
            .into_iter()
            .map(|(pt, f)| Json::Obj(vec![("p".into(), v3(pt)), ("f".into(), Json::Num(f))]))
            .collect(),
    )
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
    let out = loft(&f.id, &profiles, &LoftOpts { surface: *surface }).map_err(err)?;
    let marker = Json::Obj(vec![
        ("loops".into(), Json::Arr(loops)),
        ("starts".into(), Json::Arr(out.starts.iter().map(|p| v3(*p)).collect())),
        ("guided".into(), Json::Bool(false)),
        ("tension".into(), Json::Arr(vec![])),
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
