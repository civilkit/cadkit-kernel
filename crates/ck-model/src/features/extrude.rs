//! Extrude and revolve of sketch profiles (`extrude` / `revolve` features).
//!
//! Sketch `profiles` (sketch::profile, the oracle's region/loop travel order)
//! become a ck-topo [`Profile`]; ck-topo builds the swept lump(s).
//!
//! Pinned by oracle probes (one sketch + one feature on an empty body):
//! * extrude check order: `depth`/`draft` expressions (`badExpression`) ->
//!   closed profile (`noClosedProfile`) -> `end` (`unknownEnd {end}`) ->
//!   references / body (`missingReference` for upToFace / offsetFromFace /
//!   upToPoint, `nothingToCut` for upToNext and for `op:"cut"`);
//! * `depth <= 0` is clamped to `1e-6` (box max `1e-06`); `flip` extrudes
//!   along -normal; `mid` is symmetric (`flip` ignored);
//! * `through` on an empty body = blind 1000 (flip honoured), `throughBoth`
//!   = ±1000; GUESS: with a body the length comes from its extent (needs
//!   booleans anyway);
//! * `op` other than `"cut"` behaves as `"add"`;
//! * `draft` (degrees) tapers inward, `draftOut` outward; a mid-plane
//!   drafted extrude is two halves UNITED by a boolean in the oracle
//!   (`boolean Unite (6 × 6 faces)` trace) -> needs ck-ops;
//! * revolve check order: `angle` expression -> `axis` (a LINE entity of the
//!   sketch named by the bare string, construction or not; anything else
//!   incl. `{kind:"axis"}` -> `missingAxis`) -> profile -> `|angle| ~ 0`
//!   (`invalidValue`) -> `nothingToCut`; angle in degrees, `>= 360` (also
//!   400) is a full revolve, negative angles revolve the other way; `end`
//!   is ignored except `mid` (symmetric); `flip` reverses the direction.

use super::{End, FeatureKind, Op};
use crate::regen::{codes, FeatureError, RegenState, SketchResult};
use crate::sketch::profile::{Geom, Seg};
use crate::sketch::SketchEntity;
use crate::Feature;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::{Vec2, Vec3};
use ck_json::Json;
use ck_topo::body::Body;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

/// `through` / `throughBoth` length on an empty body (oracle box).
pub const THROUGH_EMPTY: f64 = 1000.0;
/// Smallest blind depth (oracle: `depth <= 0` -> box height `1e-06`).
pub const MIN_DEPTH: f64 = 1e-6;

/// What a body-making feature asks of the body it is applied to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Combine {
    Add,
    Cut,
}

fn v2(p: [f64; 2]) -> Vec2 {
    Vec2::new(p[0], p[1])
}

/// Interpolating spline -> piecewise cubic B-spline over `[0, reps·L]`
/// (closed splines are unrolled `reps` times so trimmed pieces may wrap).
fn interp_to_bspline(s: &crate::sketch::spline::InterpSpline) -> Option<Curve2> {
    let reps = if s.closed { 2 } else { 1 };
    let l = s.length();
    let mut knots = vec![0.0; 4];
    let mut ctrl = vec![v2(s.at(0.0, 0))];
    for r in 0..reps {
        for i in 0..s.h.len() {
            let (ta, tb) = (s.knots[i], s.knots[i + 1]);
            let b = tb + r as f64 * l;
            let h = tb - ta;
            let (p0, p3) = (v2(s.at(ta, 0)), v2(s.at(tb, 0)));
            // one-sided derivatives inside the span
            let (d0, d1) = (v2(s.at(ta + h * 1e-12, 1)), v2(s.at(tb - h * 1e-12, 1)));
            ctrl.push(p0 + d0 * (h / 3.0));
            ctrl.push(p3 - d1 * (h / 3.0));
            ctrl.push(p3);
            knots.extend([b, b, b]);
        }
    }
    *knots.last_mut()? = reps as f64 * l;
    knots.push(reps as f64 * l);
    BSplineCurve::new(3, knots, ctrl).ok().map(Curve2::BSpline)
}

/// One profile segment (a possibly split sketch-entity piece).
pub fn profile_seg(s: &Seg) -> Result<ProfileSeg, FeatureError> {
    let (ta, tb) = (s.curve.ta, s.curve.tb);
    let (lo, hi, rev) = if tb < ta { (tb, ta, true) } else { (ta, tb, false) };
    let (curve, t0, t1) = match &s.curve.geom {
        Geom::Line { p, d } => {
            let len = d[0].hypot(d[1]);
            (Curve2::line_through(v2(*p), v2([p[0] + d[0], p[1] + d[1]])), lo * len, hi * len)
        }
        // GUESS: circles/arcs keep the sketch angle parameter (x̂ = +X), so
        // full circles start at c + r·X̂ like the oracle's cap edges.
        Geom::Circle { c, r } => (Curve2::Circle { c: v2(*c), x: Vec2::X, r: *r, ccw: true }, lo, hi),
        Geom::Ellipse { c, u, v } => {
            let (a, b) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
            (Curve2::Ellipse { c: v2(*c), x: v2(*u).normalize(), a, b, ccw: true }, lo, hi)
        }
        Geom::Spline(sp) => {
            let c = interp_to_bspline(sp).ok_or_else(|| FeatureError::new(codes::NO_CLOSED_PROFILE))?;
            let (lo, hi) = if sp.closed && lo < 0.0 { (lo + sp.length(), hi + sp.length()) } else { (lo, hi) };
            (c, lo, hi)
        }
        Geom::BSpline { cv, degree, knots } => {
            let c = BSplineCurve::new(*degree, knots.clone(), cv.iter().map(|p| v2(*p)).collect())
                .map_err(|_| FeatureError::new(codes::NO_CLOSED_PROFILE))?;
            (Curve2::BSpline(c), lo, hi)
        }
    };
    Ok(ProfileSeg { tag: s.tag.clone(), curve, t0, t1, rev })
}

/// The ck-topo profile of a regenerated sketch (`noClosedProfile` when the
/// sketch has no region).
pub fn sketch_profile(sk: &SketchResult) -> Result<Profile, FeatureError> {
    if sk.profiles.regions.is_empty() {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    let lp = |l: &Vec<Seg>| -> Result<ProfileLoop, FeatureError> {
        Ok(ProfileLoop::new(l.iter().map(profile_seg).collect::<Result<_, _>>()?))
    };
    let mut regions = vec![];
    for r in &sk.profiles.regions {
        regions.push(ProfileRegion { outer: lp(&r.outer)?, holes: r.holes.iter().map(lp).collect::<Result<_, _>>()? });
    }
    Ok(Profile::new(sk.frame, regions))
}

/// `extrudeRegionNone {regions}`: the `regions` selection of an extrude /
/// revolve matched no closed region (empty list, or ids that are no region's
/// outer boundary). Never falls back to "all regions".
pub const REGION_NONE: &str = "extrudeRegionNone";
/// `extrudeRegionUnknown {ids}`: `regions` names ids that are no entity of the sketch.
pub const REGION_UNKNOWN: &str = "extrudeRegionUnknown";

/// The profile of the sketch restricted to the feature's optional `regions`
/// selection (docs/spec-agent-interface.md, "Region selection").
///
/// `regions` is a list of sketch entity ids. A region is selected when any
/// segment of its OUTER loop carries one of the ids (so a GUI can pass
/// whatever the user clicked; a split entity piece or a shared entity can
/// select several regions). A selected region keeps all its hole loops.
/// Regions nested inside a selected one (an island in its hole) are separate
/// regions and stay out unless named themselves. Absent `regions` = every
/// region (unchanged behaviour). The kept regions keep sketch order, so one
/// selected region is named exactly as if it were alone in the sketch (no
/// `~k`); several get `~1..~n` over the selection.
pub fn selected_profile(sk: &SketchResult, f: &Feature) -> Result<Profile, FeatureError> {
    let Some(sel) = f.raw.get("regions") else { return sketch_profile(sk) };
    let ids: Vec<String> = sel
        .as_arr()
        .and_then(|a| a.iter().map(|v| v.as_str().map(str::to_string)).collect::<Option<_>>())
        .ok_or_else(|| FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("regions".into())))?;
    let unknown: Vec<Json> = ids.iter().filter(|i| sk.sketch.entity(i).is_none()).map(|i| Json::Str(i.clone())).collect();
    if !unknown.is_empty() {
        return Err(FeatureError::new(REGION_UNKNOWN).with("ids", Json::Arr(unknown)));
    }
    let full = sketch_profile(sk)?;
    let hit = |l: &[Seg]| l.iter().any(|s| ids.contains(&s.tag));
    let mut regions: Vec<ProfileRegion> =
        full.regions.iter().zip(&sk.profiles.regions).filter(|(_, r)| hit(&r.outer)).map(|(p, _)| p.clone()).collect();
    // ids that bound no region's outer loop but a hole loop select that hole's filled disc (no holes)
    let matched: Vec<&String> = sk.profiles.regions.iter().filter(|r| hit(&r.outer)).flat_map(|r| r.outer.iter().map(|s| &s.tag)).collect();
    let unmatched: Vec<&String> = ids.iter().filter(|i| !matched.contains(i)).collect();
    for (p, r) in full.regions.iter().zip(&sk.profiles.regions) {
        for (hp, h) in p.holes.iter().zip(&r.holes) {
            if h.iter().any(|s| unmatched.contains(&&s.tag)) {
                let outer = if hp.signed_area() < 0.0 { hp.reversed() } else { hp.clone() };
                let reg = ProfileRegion { outer, holes: vec![] };
                if !regions.contains(&reg) {
                    regions.push(reg);
                }
            }
        }
    }
    if regions.is_empty() {
        return Err(FeatureError::new(REGION_NONE).with("regions", Json::Arr(ids.into_iter().map(Json::Str).collect())));
    }
    Ok(Profile::new(sk.frame, regions))
}

/// Splits a profile into (regions that are not islands, islands): an island is a region whose outer
/// boundary is made of the same sketch entities as a hole of another region of the profile. `None`
/// when there is no such pair.
pub fn split_islands(p: &Profile) -> Option<(Profile, Profile)> {
    let key = |l: &ProfileLoop| -> Vec<String> {
        let mut k: Vec<String> = l.segs.iter().map(|s| s.tag.clone()).collect();
        k.sort_unstable();
        k
    };
    let holes: Vec<Vec<String>> = p.regions.iter().flat_map(|r| r.holes.iter().map(key)).collect();
    let (isl, base): (Vec<_>, Vec<_>) = p.regions.iter().cloned().partition(|r| holes.contains(&key(&r.outer)));
    if isl.is_empty() || base.is_empty() {
        return None;
    }
    Some((Profile::new(p.frame, base), Profile::new(p.frame, isl)))
}

/// How a feature's tool body was swept (kept so mirror / patterns can
/// rebuild it in place of reflecting a finished body).
#[derive(Debug, Clone)]
pub enum Recipe {
    Extrude { profile: Profile, opts: ExtrudeOpts },
    Revolve { profile: Profile, o: Vec3, d: Vec3, a0: f64, a1: f64 },
}

/// A swept tool body plus how it combines with the current body.
pub struct Tool {
    pub body: Body,
    pub combine: Combine,
    pub recipe: Option<Recipe>,
}

/// `through` length with a body (oracle probe: a flipped add `through`
/// from a plane 2000 below a 40×10×20 box gave a 2060.95 tool).
/// GUESS: distance from the sketch origin to the body box centre + box
/// diagonal + 10 (matches that probe to 3e-6).
pub fn through_len(frame_o: Vec3, body: &Body) -> f64 {
    let (lo, hi) = crate::features::body_box(body);
    let c = (lo + hi) * 0.5;
    frame_o.dist(c) + lo.dist(hi) + 10.0
}

fn sketch_of<'a>(state: &'a RegenState, f: &Feature) -> Result<&'a SketchResult, FeatureError> {
    f.raw
        .get("sketch")
        .and_then(|s| s.as_str())
        .and_then(|s| state.sketch(s))
        .ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))
}

fn topo_err(e: String) -> FeatureError {
    // ck-topo error strings are the oracle codes where pinned
    // (draftTooLarge, draftUnsupported, draftFailed); others are GUESSes.
    FeatureError::new(&e)
}

/// `extrude`. `has_body`: whether a body exists (cut / upToNext targets).
pub fn exec_extrude(state: &RegenState, f: &Feature, body: Option<&Body>) -> Result<Tool, FeatureError> {
    let has_body = body.is_some();
    let FeatureKind::Extrude { op, end, flip, draft_out, .. } = &f.kind else { unreachable!() };
    // Oracle: a cut goes AGAINST the sketch normal unless `flip`.
    let flip = &(*flip ^ (*op == Op::Cut));
    let sk = sketch_of(state, f)?;
    let depth = state.num(f, "depth", 0.0)?;
    let draft = state.num(f, "draft", 0.0)?;
    let profile = selected_profile(sk, f)?;
    let depth = depth.max(MIN_DEPTH);
    let combine = if *op == Op::Cut { Combine::Cut } else { Combine::Add };
    let mut o = match end {
        End::Blind if *flip => ExtrudeOpts::flipped(depth),
        End::Blind => ExtrudeOpts::blind(depth),
        End::Mid => ExtrudeOpts::mid(depth),
        // upToNext with a body: a `through` tool, trimmed by the caller
        // (`Brep::up_to_next`); without one: nothingToCut (oracle).
        End::UpToNext if !has_body => return Err(FeatureError::new(codes::NOTHING_TO_CUT)),
        End::Through | End::ThroughBoth | End::UpToNext => {
            let l = body.map_or(THROUGH_EMPTY, |b| through_len(sk.frame.o, b));
            match end {
                End::ThroughBoth => ExtrudeOpts::mid(2.0 * l),
                _ if *flip => ExtrudeOpts::flipped(l),
                _ => ExtrudeOpts::blind(l),
            }
        }
        End::UpToFace | End::OffsetFromFace | End::UpToPoint => {
            // GUESS: the face/point reference is resolved against the body;
            // with no body every reference is missing (oracle).
            if !has_body {
                return Err(FeatureError::new(codes::MISSING_REFERENCE));
            }
            return Err(FeatureError::new(&todo("extrude up to a reference")));
        }
        End::Unknown => {
            let e = f.raw.get("end").cloned().unwrap_or(Json::Null);
            return Err(FeatureError::new(UNKNOWN_END).with("end", e));
        }
    };
    if combine == Combine::Cut && !has_body {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    if draft != 0.0 {
        o.draft = draft.abs().to_radians() * if *draft_out { -1.0 } else { 1.0 };
        if *end == End::Mid || *end == End::ThroughBoth {
            // oracle: two drafted halves united (boolean Unite) — needs ck-ops
            return Err(FeatureError::new(&todo("mid-plane drafted extrude (boolean unite)")));
        }
    }
    // GUESS (not an oracle option): `twist` degrees over the full depth about
    // the axis through `twistCentre` (sketch xy, default the sketch origin).
    let twist = state.num(f, "twist", 0.0)?;
    if twist != 0.0 {
        if o.draft != 0.0 || matches!(end, End::UpToNext) {
            return Err(FeatureError::new("draftUnsupported"));
        }
        let c = f.raw.get("twistCentre").and_then(|c| c.as_arr()).filter(|a| a.len() == 2);
        let centre = c.map_or(Vec2::ZERO, |a| Vec2::new(a[0].as_f64().unwrap_or(0.0), a[1].as_f64().unwrap_or(0.0)));
        let body = ck_ops::sweep::twist_extrude(&f.id, &profile, o, twist.to_radians(), centre).map_err(topo_err)?;
        return Ok(Tool { body, combine, recipe: None });
    }
    let build = |p: &Profile| -> Result<Body, FeatureError> {
        if o.draft != 0.0 {
            super::draft::drafted_extrude(&f.id, p, o)
        } else {
            extrude(&f.id, p, o).map_err(topo_err)
        }
    };
    let body = match f.raw.get("regions").and_then(|_| split_islands(&profile)) {
        // a selected region that fills a hole of another selected one: sweep the two apart and unite
        // them, so the shared wall disappears and the caps merge (one lump, not two touching ones)
        Some((base, islands)) => {
            let a = build(&base)?;
            let b = build(&islands)?;
            let out = ck_ops::boolean::boolean(&a, &b, ck_ops::boolean::Op::Union, 1e-6).map_err(|e| FeatureError::new(&e))?;
            out.body
        }
        None => build(&profile)?,
    };
    Ok(Tool { body, combine, recipe: Some(Recipe::Extrude { profile, opts: o }) })
}

pub const UNKNOWN_END: &str = "unknownEnd";
/// upToNext found no face to stop at (oracle code).
pub const UP_TO_NEXT_NONE: &str = "upToNextNone";
/// upToNext add whose tool never meets the body (oracle code).
pub const UP_TO_NEXT_OPEN: &str = "upToNextOpen";
pub const MISSING_AXIS: &str = "missingAxis";

/// Error code for paths that need not-yet-rebuilt operations (never produced
/// by the oracle).
pub fn todo(what: &str) -> String {
    format!("TODO(rebuild): {what} needs ck-ops")
}

/// Revolve axis: a line entity of the sketch, in world coordinates.
fn sketch_axis(sk: &SketchResult, axis: &Json) -> Option<(Vec3, Vec3)> {
    let id = axis.as_str()?;
    let SketchEntity::Line { p1, p2, .. } = sk.sketch.entity(id)? else { return None };
    let (a, b) = (sk.sketch.point_xy(p1)?, sk.sketch.point_xy(p2)?);
    let fr = &sk.frame;
    let w = |p: [f64; 2]| fr.o + fr.x * p[0] + fr.y * p[1];
    let (a, b) = (w(a), w(b));
    let d = (b - a).try_normalize()?;
    Some((a, d))
}

/// `revolve`.
pub fn exec_revolve(state: &RegenState, f: &Feature, has_body: bool) -> Result<Tool, FeatureError> {
    let FeatureKind::Revolve { op, end, flip, .. } = &f.kind else { unreachable!() };
    let sk = sketch_of(state, f)?;
    let angle = state.num(f, "angle", 360.0)?;
    let (o, d) = f
        .raw
        .get("axis")
        .and_then(|a| sketch_axis(sk, a))
        .ok_or_else(|| FeatureError::new(MISSING_AXIS))?;
    let profile = selected_profile(sk, f)?;
    // pinned: a region on both sides of the axis -> profileCrossesAxis (no params)
    if super::surface_extrude::crosses_axis(&profile, o, d) {
        return Err(FeatureError::new(super::surface_extrude::PROFILE_CROSSES_AXIS));
    }
    // GUESS: threshold for a zero angle (1e-12 deg is rejected).
    if angle.abs() < 1e-9 {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let combine = if *op == Op::Cut { Combine::Cut } else { Combine::Add };
    if combine == Combine::Cut && !has_body {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let mut a = angle.to_radians();
    if a.abs() >= TAU {
        a = TAU * a.signum();
    }
    if *flip {
        a = -a;
    }
    let (a0, a1) = if *end == End::Mid { (-a / 2.0, a / 2.0) } else { (0.0, a) };
    // GUESS: a negative sweep revolves about the reversed axis (keeps C0 in
    // the sketch plane).
    let (d, a0, a1) = if a1 < a0 { (-d, -a0, -a1) } else { (d, a0, a1) };
    let (a0, a1) = (a0.min(a1), a0.max(a1));
    let body = revolve(&f.id, &profile, o, d, a0, a1).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Revolve { profile, o, d, a0, a1 }) })
}
