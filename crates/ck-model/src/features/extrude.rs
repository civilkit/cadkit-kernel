//! Extrude and revolve of sketch profiles (`extrude` / `revolve` features).
//!
//! Sketch `profiles` (sketch::profile, the oracle's region/loop travel order)
//! become a ck-topo [`Profile`]; ck-topo builds the swept lump(s).
//!
//! Pinned by oracle probes (one sketch + one feature on an empty body):
//! * extrude check order: `depth`/`draft` expressions (`badExpression`) ->
//!   non-finite sketch point (`invalidValue {point, detail, hint}`, see
//!   [`finite_points`]) -> closed profile (`noClosedProfile`) -> `end` (`unknownEnd {end}`) ->
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
    /// `newBody`: the tool becomes its own lump, not united with what it touches or crosses.
    Separate,
}

impl Combine {
    /// How a feature with this operation combines with the body.
    pub fn of(op: super::Op) -> Combine {
        match op {
            super::Op::Add => Combine::Add,
            super::Op::Cut => Combine::Cut,
            super::Op::NewBody => Combine::Separate,
        }
    }
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

/// `invalidValue {point, detail, hint}`: a sketch point holds a non-finite
/// coordinate (NaN / infinity, e.g. a kit script computing a fractional power
/// of a negative number). A loop of such points finds no region, which would
/// otherwise surface as a misleading `noClosedProfile` (loft: `loftProfile`).
pub fn finite_points(sk: &SketchResult) -> Result<(), FeatureError> {
    let Some(p) = sk.sketch.points.iter().find(|p| !p.x.is_finite() || !p.y.is_finite()) else { return Ok(()) };
    let (axis, v) = if !p.x.is_finite() { ("x", p.x) } else { ("y", p.y) };
    let shown = if v.is_nan() { "NaN".to_string() } else { v.to_string() };
    Err(FeatureError::new(codes::INVALID_VALUE)
        .with("point", Json::Str(p.id.clone()))
        .with("detail", Json::Str(format!("sketch point {} has a non-finite {} ({})", p.id, axis, shown)))
        .with("hint", Json::Str(
            "a sketch coordinate must be finite; check the script or expression that places this point (for example a fractional power of a negative number yields NaN)".into(),
        )))
}

/// The ck-topo profile of a regenerated sketch (`noClosedProfile` when the
/// sketch has no region).
pub fn sketch_profile(sk: &SketchResult) -> Result<Profile, FeatureError> {
    finite_points(sk)?;
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

/// Length of an outer-loop segment.
fn seg_len(s: &Seg) -> f64 {
    s.sample(32).windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).sum()
}

/// Which regions (`.0`, by index into `sk.profiles.regions`) and fills (`.1`, `sk.profiles.fills`) the entity
/// ids select: those whose OUTER loop carries one of the ids.
///
/// Regions: every one the id touches (a shared edge or a split entity selects all of them).
/// Fills (the faces of a ring cut by petal outlines ending on it): an id touching a face and a pocket of it
/// (a face whose entities shared with other faces are a strict subset of the other's: a petal outline, and
/// the ring minus the petals) goes to one of the two so each can be picked alone: to the face the id is
/// mostly drawn on (longest length of the entity on its outer loop), on a tie to the pocket.
fn faces_for_ids(sk: &SketchResult, ids: &[String]) -> (Vec<bool>, Vec<bool>) {
    use std::collections::BTreeSet;
    let (rs, fs) = (&sk.profiles.regions, &sk.profiles.fills);
    let mut reg = vec![false; rs.len()];
    let mut fill = vec![false; fs.len()];
    let near = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]) <= 1e-6;
    // entities of a fill's outer loop that border another fill
    let shared: Vec<BTreeSet<&str>> = fs
        .iter()
        .enumerate()
        .map(|(i, f)| {
            f.outer
                .iter()
                .filter(|s| fs.iter().enumerate().any(|(j, g)| j != i && g.outer.iter().any(|t| t.tag == s.tag && near(s.a, t.b) && near(s.b, t.a))))
                .map(|s| s.tag.as_str())
                .collect()
        })
        .collect();
    for id in ids {
        for (i, r) in rs.iter().enumerate() {
            reg[i] |= r.outer.iter().any(|s| &s.tag == id);
        }
        let hits: Vec<usize> = (0..fs.len()).filter(|&i| fs[i].outer.iter().any(|s| &s.tag == id)).collect();
        let cover = |i: usize| fs[i].outer.iter().filter(|s| &s.tag == id).map(seg_len).sum::<f64>();
        let mut drop = vec![false; fs.len()];
        for &host in &hits {
            for &pocket in &hits {
                if shared[pocket].len() < shared[host].len() && shared[pocket].is_subset(&shared[host]) {
                    if cover(host) > cover(pocket) * (1.0 + 1e-6) + 1e-9 {
                        drop[pocket] = true;
                    } else {
                        drop[host] = true;
                    }
                }
            }
        }
        for &i in &hits {
            fill[i] |= !drop[i];
        }
    }
    (reg, fill)
}

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
    // the regions the ids touch, then the fills of subdivided components (a ring cut by petal outlines)
    let (preg, pfill) = faces_for_ids(sk, &ids);
    let lp = |l: &Vec<Seg>| -> Result<ProfileLoop, FeatureError> {
        Ok(ProfileLoop::new(l.iter().map(profile_seg).collect::<Result<_, _>>()?))
    };
    let mut regions: Vec<ProfileRegion> = vec![];
    let cands: Vec<&crate::sketch::profile::Region> = sk.profiles.regions.iter().chain(sk.profiles.fills.iter()).collect();
    let picked: Vec<bool> = preg.iter().chain(pfill.iter()).copied().collect();
    for (r, _) in cands.iter().zip(&picked).filter(|(_, p)| **p) {
        regions.push(ProfileRegion { outer: lp(&r.outer)?, holes: r.holes.iter().map(lp).collect::<Result<_, _>>()? });
    }
    // ids that bound no region's outer loop but a hole loop select that hole's filled disc (no holes)
    let matched: Vec<&String> = cands.iter().zip(&picked).filter(|(_, p)| **p).flat_map(|(r, _)| r.outer.iter().map(|s| &s.tag)).collect();
    let fill_tags: Vec<&String> = sk.profiles.fills.iter().flat_map(|r| r.outer.iter().map(|s| &s.tag)).collect();
    let unmatched: Vec<&String> = ids.iter().filter(|i| !matched.contains(i) && !fill_tags.contains(i)).collect();
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
    exec_extrude_ex(state, f, body, &mut EndExtras::default())
}

/// `extrude` with the reference end conditions' side channel (see [`EndExtras`]).
pub fn exec_extrude_ex(state: &RegenState, f: &Feature, body: Option<&Body>, extras: &mut EndExtras) -> Result<Tool, FeatureError> {
    let has_body = body.is_some();
    let FeatureKind::Extrude { op, end, flip, draft_out, .. } = &f.kind else { unreachable!() };
    // Oracle: a cut goes AGAINST the sketch normal unless `flip`.
    let flip = &(*flip ^ (*op == Op::Cut));
    let sk = sketch_of(state, f)?;
    let depth = state.size(f, "depth", 0.0)?;
    let draft = state.num(f, "draft", 0.0)?;
    let (mut profile, thin_on) = super::thin::feature_profile(state, sk, f)?;
    let depth = depth.max(MIN_DEPTH);
    let combine = Combine::of(*op);
    let object = super::extrude_object::wanted(f, *end)?;
    let mut o = if object {
        // To object / From object: the profile is carried to where the sweep starts; the trims are the caller's
        let p = super::extrude_object::plan(state, f, &sk.frame, &profile, *flip, *end, depth, body, extras)?;
        let d = if *flip { -sk.frame.z } else { sk.frame.z };
        profile.frame.o = profile.frame.o + d * p.s0;
        extras.clip = p.end_clip;
        extras.start_clip = p.start_clip;
        extras.to_body = p.to_body;
        let l = p.len.max(MIN_DEPTH);
        if *flip {
            ExtrudeOpts::flipped(l)
        } else {
            ExtrudeOpts::blind(l)
        }
    } else {
        match end {
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
                // The reference is resolved against the body (persistent face names); with no body every face
                // reference is missing (oracle). A reference that cannot be reached falls back to a blind
                // extrude of `depth` plus an `endConditionFallback` warning (unless `strictEnd`).
                let planned = match extras.force_fallback.take() {
                    Some(e) => Err(e),
                    None => plan_end(state, f, &sk.frame, &profile, *flip, *end, body, extras),
                };
                match planned {
                    Ok(p) => {
                        extras.clip = p.clip;
                        let l = p.depth.max(MIN_DEPTH);
                        if *flip {
                            ExtrudeOpts::flipped(l)
                        } else {
                            ExtrudeOpts::blind(l)
                        }
                    }
                    Err(e) if !strict_end(f) && is_unreachable(&e.code) => {
                        extras.warnings.push(fallback_warning(*end, &e, depth));
                        if *flip {
                            ExtrudeOpts::flipped(depth)
                        } else {
                            ExtrudeOpts::blind(depth)
                        }
                    }
                    Err(e) => return Err(e),
                }
            }
            End::ToObject => unreachable!("toObject takes the object path"),
            End::Unknown => {
                let e = f.raw.get("end").cloned().unwrap_or(Json::Null);
                return Err(FeatureError::new(UNKNOWN_END).with("end", e));
            }
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
        if extras.clip.is_some() || extras.start_clip.is_some() || extras.to_body.is_some() {
            return Err(FeatureError::new(codes::INVALID_VALUE)
                .with("detail", Json::Str("a twist cannot end on a slanted plane or a body".into()))
                .with("hint", Json::Str("drop the twist, or end on a plane parallel to the sketch".into())));
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
    let body = match f.raw.get("regions").filter(|_| !thin_on).and_then(|_| split_islands(&profile)) {
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
    let recipe = if extras.clip.is_some() || extras.start_clip.is_some() || extras.to_body.is_some() { None } else { Some(Recipe::Extrude { profile, opts: o }) };
    Ok(Tool { body, combine, recipe })
}

pub const UNKNOWN_END: &str = "unknownEnd";
/// `upToFaceBehind` / `upToFaceOutside` / `upToFaceUnsupported` / `upToPointBehind`: oracle codes
/// (kernel.strings) for an end reference the extrusion cannot reach.
pub const UP_TO_FACE_BEHIND: &str = "upToFaceBehind";
pub const UP_TO_FACE_OUTSIDE: &str = "upToFaceOutside";
pub const UP_TO_FACE_UNSUPPORTED: &str = "upToFaceUnsupported";
pub const UP_TO_POINT_BEHIND: &str = "upToPointBehind";
/// Boolean that trims a slanted end failed (REBUILD-ONLY code, always downgraded).
pub const END_CLIP_FAILED: &str = "endClipFailed";
/// Warning (REBUILD-ONLY): the end reference was unreachable, the feature ran as a blind extrude of
/// `depth`. Params: `end`, `reason` (the error code that was downgraded), its own params, `depth`.
pub const END_FALLBACK: &str = "endConditionFallback";
/// Warning (REBUILD-ONLY): the face id of `endRef` no longer exists, the face was re-found by its
/// `~k` family or its `at` anchor. Params `from`, `to`.
pub const END_REF_RELINKED: &str = "endRefRelinked";

/// Codes of an end reference that can be downgraded to a fallback (never `missingReference`).
pub fn is_unreachable(code: &str) -> bool {
    matches!(code, UP_TO_FACE_BEHIND | UP_TO_FACE_OUTSIDE | UP_TO_FACE_UNSUPPORTED | UP_TO_POINT_BEHIND | END_CLIP_FAILED)
}

fn strict_end(f: &Feature) -> bool {
    matches!(f.raw.get("strictEnd"), Some(Json::Bool(true)))
}

pub fn fallback_warning(end: End, e: &FeatureError, depth: f64) -> FeatureError {
    let name = match end {
        End::UpToFace => "upToFace",
        End::OffsetFromFace => "offsetFromFace",
        End::UpToPoint => "upToPoint",
        _ => "",
    };
    let mut w = FeatureError::new(END_FALLBACK)
        .with("end", Json::Str(name.into()))
        .with("reason", Json::Str(e.code.clone()))
        .with("fallback", Json::Str("blind".into()))
        .with("depth", Json::Num(depth));
    for (k, v) in &e.params {
        w = w.with(k, v.clone());
    }
    w
}

/// Side channel of [`exec_extrude_ex`]: warnings to report, the trim a slanted end needs, and a
/// forced fallback (set by the caller when the trim itself failed).
#[derive(Default)]
pub struct EndExtras {
    pub warnings: Vec<FeatureError>,
    pub clip: Option<EndClip>,
    pub force_fallback: Option<FeatureError>,
    /// From object on a slanted plane: the trim before the start (the cap most towards `w` takes the new face).
    pub start_clip: Option<EndClip>,
    /// To object on a body: the long tool is cut back to where it meets the body.
    pub to_body: Option<super::extrude_object::ToBody>,
}

/// A slanted end plane: the tool is cut by the half-space beyond `p0` (towards `w`).
#[derive(Debug, Clone, Copy)]
pub struct EndClip {
    pub p0: Vec3,
    /// Unit normal pointing to the side that is removed.
    pub w: Vec3,
}

struct EndPlan {
    depth: f64,
    clip: Option<EndClip>,
}

fn bad_ref(field: &str) -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE).with("field", Json::Str(field.into()))
}

fn json_pt(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    if a.len() != 3 {
        return None;
    }
    Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?))
}

fn strip_region(n: &ck_topo::body::FaceName) -> String {
    let mut m = n.clone();
    m.region = None;
    m.id()
}

/// Persistent-name lookup of a face: the exact id; else the unique face of the same `~k` family
/// (`F4.S.e1` finds `F4.S.e1~2` when a profile gained regions, and vice versa); else, with an `at`
/// anchor, the planar face through that point. `Some((face, relinked))`.
pub fn resolve_face(body: &Body, id: &str, at: Option<Vec3>) -> Option<(ck_topo::body::FaceId, bool)> {
    if let Some(f) = body.find_face(id) {
        return Some((f, false));
    }
    let base = id.split('~').next().unwrap_or(id);
    let fam: Vec<_> = body.face_ids().filter(|&f| strip_region(&body.face(f).name) == base).collect();
    if fam.len() == 1 {
        return Some((fam[0], true));
    }
    let at = at?;
    let pool: Vec<_> = if fam.is_empty() { body.face_ids().collect() } else { fam };
    // the anchor must pick exactly one planar face (a corner or an edge point is ambiguous)
    let hits: Vec<_> = pool
        .into_iter()
        .filter(|&f| {
            let face = body.face(f);
            matches!(face.surface, ck_geom::surface::Surface::Plane { .. })
                && face.surface.distance(at) <= 1e-4
                && ck_ops::boolean::classify::point_in_face(body, f, at, 1e-5) != ck_ops::boolean::classify::Where::Out
        })
        .collect();
    (hits.len() == 1).then(|| (hits[0], true))
}

/// World points along the profile boundaries (end points and interior samples).
pub(super) fn profile_samples(p: &Profile) -> Vec<Vec3> {
    let mut out = vec![];
    for r in &p.regions {
        for l in std::iter::once(&r.outer).chain(r.holes.iter()) {
            for s in &l.segs {
                for k in 0..=8 {
                    let t = s.t0 + (s.t1 - s.t0) * k as f64 / 8.0;
                    let q = s.curve.eval(t);
                    out.push(p.frame.o + p.frame.x * q.x + p.frame.y * q.y);
                }
            }
        }
    }
    out
}

/// [`vertex_of`] without the relink warnings (a pattern's To Object vertex).
pub(crate) fn vertex_near(body: &Body, faces: &[String], at: Vec3) -> Option<Vec3> {
    if faces.is_empty() {
        return None;
    }
    vertex_of(body, faces, at, &mut EndExtras::default())
}

/// A vertex shared by the named faces, nearest `at` (persistent: faces by name, the vertex by position).
fn vertex_of(body: &Body, faces: &[String], at: Vec3, extras: &mut EndExtras) -> Option<Vec3> {
    let mut cands: Vec<ck_topo::body::VertexId> = vec![];
    for (i, name) in faces.iter().enumerate() {
        let (f, re) = resolve_face(body, name, Some(at))?;
        if re {
            extras.warnings.push(FeatureError::new(END_REF_RELINKED).with("from", Json::Str(name.clone())).with("to", Json::Str(body.face_id_str(f))));
        }
        let mut vs = vec![];
        for e in body.face_edges(f) {
            vs.push(body.edge(e).v0);
            vs.push(body.edge(e).v1);
        }
        if i == 0 {
            cands = vs;
        } else {
            cands.retain(|v| vs.contains(v));
        }
    }
    cands.into_iter().map(|v| body.vertex(v).p).min_by(|a, b| a.dist(at).total_cmp(&b.dist(at)))
}

/// Where a reference end condition stops: the end plane's distance along the extrusion and, when the
/// plane is slanted against the sketch, the trim it needs.
///
/// * `upToFace`: `endRef` = `{"kind":"face","id":"F2.C1","at":[x,y,z]}` (or the bare id). Planar faces only
///   (`upToFaceUnsupported` otherwise); the face must lie ahead of the whole profile
///   (`upToFaceBehind`) and cover it along the extrusion (`upToFaceOutside`).
/// * `offsetFromFace`: the same face, its plane moved by `offset` along the face's outward normal
///   (negative = into the material); no coverage test.
/// * `upToPoint`: `endRef` = `[x,y,z]`, `{"kind":"point","at":[x,y,z]}` or `{"kind":"vertex","faces":[..],"at":[..]}`;
///   the end plane is parallel to the sketch through the point (`upToPointBehind`).
#[allow(clippy::too_many_arguments)]
fn plan_end(
    state: &RegenState,
    f: &Feature,
    fr: &ck_core::linalg::Frame,
    profile: &Profile,
    flip: bool,
    end: End,
    body: Option<&Body>,
    extras: &mut EndExtras,
) -> Result<EndPlan, FeatureError> {
    let r = f.raw.get("endRef").filter(|v| !matches!(v, Json::Null)).ok_or_else(|| bad_ref("endRef"))?;
    let d = if flip { -fr.z } else { fr.z };
    let pts = profile_samples(profile);
    let mut face = None;
    // (a point on the end plane, its normal)
    let (p0, mm): (Vec3, Vec3) = if end == End::UpToPoint {
        let kind = r.get("kind").and_then(|k| k.as_str());
        let pt = if let Some(p) = json_pt(r) {
            p
        } else if let (Some("point"), Some(p)) = (kind, r.get("at").and_then(json_pt)) {
            p
        } else if kind == Some("vertex") {
            let faces: Vec<String> =
                r.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect()).unwrap_or_default();
            let at = r.get("at").and_then(json_pt).unwrap_or(Vec3::ZERO);
            if faces.is_empty() {
                return Err(bad_ref("endRef"));
            }
            vertex_of(body.ok_or_else(|| bad_ref("endRef"))?, &faces, at, extras).ok_or_else(|| bad_ref("endRef"))?
        } else {
            return Err(bad_ref("endRef"));
        };
        (pt, fr.z)
    } else {
        let body = body.ok_or_else(|| bad_ref("endRef"))?;
        let (id, at) = match r {
            Json::Str(s) => (s.clone(), None),
            _ => (r.get("id").and_then(|v| v.as_str()).ok_or_else(|| bad_ref("endRef"))?.to_string(), r.get("at").and_then(json_pt)),
        };
        let (fid, relinked) = resolve_face(body, &id, at).ok_or_else(|| bad_ref("endRef").with("face", Json::Str(id.clone())))?;
        if relinked {
            extras.warnings.push(FeatureError::new(END_REF_RELINKED).with("from", Json::Str(id.clone())).with("to", Json::Str(body.face_id_str(fid))));
        }
        let fc = body.face(fid);
        let ck_geom::surface::Surface::Plane { frame } = &fc.surface else {
            return Err(FeatureError::new(UP_TO_FACE_UNSUPPORTED).with("face", Json::Str(id)));
        };
        let out = if fc.sense { frame.z } else { -frame.z };
        let mut p0 = frame.o;
        if end == End::OffsetFromFace {
            p0 = p0 + out * state.num(f, "offset", 0.0)?;
        }
        face = Some(fid);
        (p0, out)
    };
    let behind = || FeatureError::new(if end == End::UpToPoint { UP_TO_POINT_BEHIND } else { UP_TO_FACE_BEHIND });
    let c = d.dot(mm);
    if c.abs() < 1e-9 {
        return Err(behind());
    }
    let ts: Vec<f64> = pts.iter().map(|q| (p0 - *q).dot(mm) / c).collect();
    let (tmin, tmax) = ts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, &t| (a.0.min(t), a.1.max(t)));
    if tmin <= 1e-9 {
        return Err(behind());
    }
    if let (End::UpToFace, Some(fid), Some(body)) = (end, face, body) {
        for (q, t) in pts.iter().zip(&ts) {
            if ck_ops::boolean::classify::point_in_face(body, fid, *q + d * *t, 1e-5) == ck_ops::boolean::classify::Where::Out {
                return Err(FeatureError::new(UP_TO_FACE_OUTSIDE).with("face", Json::Str(body.face_id_str(fid))));
            }
        }
    }
    let slanted = tmax - tmin > 1e-7;
    let clip = slanted.then(|| EndClip { p0, w: if c > 0.0 { mm } else { -mm } });
    Ok(EndPlan { depth: tmax, clip })
}

/// upToNext found no face to stop at (oracle code).
pub const UP_TO_NEXT_NONE: &str = "upToNextNone";
/// upToNext add whose tool never meets the body (oracle code).
pub const UP_TO_NEXT_OPEN: &str = "upToNextOpen";
pub const MISSING_AXIS: &str = "missingAxis";

/// Error for a feature or option that is not built yet: the stable code `notImplemented` with the
/// name of the feature or option in `what` (see `FeatureError::new`).
pub fn todo(what: &str) -> String {
    format!("notImplemented;what={what}")
}

/// Revolve axis, in world coordinates: a line entity id of the profile's own sketch, or
/// `{"sketch": <sketch id>, "id": <line id>}` for a line of another sketch (entity ids restart in
/// each sketch, so the owner is named).
pub(super) fn sketch_axis(state: &RegenState, sk: &SketchResult, axis: &Json) -> Option<(Vec3, Vec3)> {
    let (owner, id) = match axis {
        Json::Str(id) => (sk, id.as_str()),
        _ => (state.sketch(axis.get("sketch")?.as_str()?)?, axis.get("id")?.as_str()?),
    };
    let SketchEntity::Line { p1, p2, .. } = owner.sketch.entity(id)? else { return None };
    let (a, b) = (owner.sketch.point_xy(p1)?, owner.sketch.point_xy(p2)?);
    let fr = &owner.frame;
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
        .and_then(|a| sketch_axis(state, sk, a))
        .ok_or_else(|| FeatureError::new(MISSING_AXIS))?;
    let (profile, _) = super::thin::feature_profile(state, sk, f)?;
    // pinned: a region on both sides of the axis -> profileCrossesAxis (no params)
    if super::surface_extrude::crosses_axis(&profile, o, d) {
        return Err(FeatureError::new(super::surface_extrude::PROFILE_CROSSES_AXIS));
    }
    // GUESS: threshold for a zero angle (1e-12 deg is rejected).
    if angle.abs() < 1e-9 {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let combine = Combine::of(*op);
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

/// A rectangular block: `size` `[x, y, z]` (or `width` / `depth` / `height`) about `center` (default the origin), edges along the
/// world axes, with no sketch. Its faces are `<id>.S.e1` .. `<id>.S.e4` (the sides, -Y, +X, +Y, -X) and the caps `<id>.C0` (-Z) and
/// `<id>.C1` (+Z), as an extruded rectangle names them.
pub fn exec_box(state: &RegenState, f: &Feature, has_body: bool) -> Result<Tool, FeatureError> {
    let FeatureKind::Box { op } = &f.kind else { unreachable!() };
    let bad = |detail: &str| FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()));
    let c = super::move_body::vec3(state, f.raw.get("center"))?.unwrap_or(Vec3::ZERO);
    let size = match super::move_body::vec3(state, f.raw.get("size"))? {
        Some(v) => v,
        None => Vec3::new(state.num(f, "width", 0.0)?, state.num(f, "depth", 0.0)?, state.num(f, "height", 0.0)?),
    };
    if !(size.x.is_finite() && size.y.is_finite() && size.z.is_finite() && size.x > 0.0 && size.y > 0.0 && size.z > 0.0) {
        return Err(bad("each side of a box is above 0 (size [x, y, z], or width, depth and height)"));
    }
    let combine = Combine::of(*op);
    if combine == Combine::Cut && !has_body {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let frame = ck_core::linalg::Frame { o: Vec3::new(c.x, c.y, c.z - size.z / 2.0), x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
    let (hx, hy) = (size.x / 2.0, size.y / 2.0);
    let p = [Vec2::new(-hx, -hy), Vec2::new(hx, -hy), Vec2::new(hx, hy), Vec2::new(-hx, hy)];
    let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let profile = Profile::new(frame, vec![ProfileRegion { outer, holes: vec![] }]);
    let opts = ExtrudeOpts { s0: 0.0, s1: size.z, dir: None, draft: 0.0 };
    let body = extrude(&f.id, &profile, opts.clone()).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Extrude { profile, opts }) })
}

/// A ball of `radius` about `center` (poles along +Y): the revolve of a half disc, built directly so it needs no sketch.
/// The one face is `<id>.S.e1`, a true sphere surface.
pub fn exec_sphere(state: &RegenState, f: &Feature, has_body: bool) -> Result<Tool, FeatureError> {
    let FeatureKind::Sphere { op } = &f.kind else { unreachable!() };
    let bad = |detail: &str| FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()));
    let c = super::move_body::vec3(state, f.raw.get("center"))?.unwrap_or(Vec3::ZERO);
    let r = state.num(f, "radius", 0.0)?;
    if !(r.is_finite() && r > 0.0) {
        return Err(bad("the radius of a sphere is above 0"));
    }
    let combine = Combine::of(*op);
    if combine == Combine::Cut && !has_body {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let frame = ck_core::linalg::Frame { o: c, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
    let half = ProfileLoop::new(vec![
        ProfileSeg::arc("e1", Vec2::ZERO, Vec2::new(0.0, -r), Vec2::new(0.0, r)),
        ProfileSeg::line("e2", Vec2::new(0.0, r), Vec2::new(0.0, -r)),
    ]);
    let profile = Profile::new(frame, vec![ProfileRegion { outer: half, holes: vec![] }]);
    let body = revolve(&f.id, &profile, c, Vec3::Y, 0.0, TAU).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Revolve { profile, o: c, d: Vec3::Y, a0: 0.0, a1: TAU }) })
}
