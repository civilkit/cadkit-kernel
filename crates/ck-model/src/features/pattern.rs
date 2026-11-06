//! Mirror / linear pattern / circular pattern of seed FEATURES.
//!
//! Pinned by oracle probes (bracket F10 = mirror of the gusset F8 in the
//! right plane, body:false):
//! * the seed is the feature's TOOL body (the swept gusset, 7 faces), not the
//!   faces it left on the result: trace `boolean Unite (25 × 7 faces)`;
//! * copies are renamed `@<feature>.<k>` like pattern instances
//!   (`F8.S.e9@F10.1`, `F8.C0@F10.1`): a mirrored cap keeps its cap index
//!   (the mirrored `F8.C0` still names the copy of C0).
//! * errors: `noSeeds` (empty list), `missingReference` (unknown seed,
//!   missing dir / plane); count <= 1 -> nothing happens;
//! * one boolean per seed; patterns append all of a seed's instances into
//!   one tool (`boolean Subtract (7 × 6 faces)` for 2 copies of a 3-face
//!   hole); a mirrored/patterned cut that removes nothing is NOT a
//!   `cutNoEffect` error;
//! * pattern transforms: ck-ops `boolean::apply` (lpattern k·spacing·dir;
//!   cpattern `equal` step = angle/count at 360°, angle/(count−1) otherwise).
//!
//! ck-topo's `Body::transform` is rigid-only, so a mirrored copy is rebuilt
//! by sweeping the REFLECTED profile (frame o,x,y reflected, z = x'×y' — the
//! sweep then runs along −z' over the negated height range, which swaps the
//! low/high caps back).

use super::extrude::{Combine, Recipe, Tool};
use super::{pattern_place, thread, Brep, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef, Ref};
use ck_core::linalg::{Frame, Plane, Vec3};
use ck_core::xform::Xform;
use ck_topo::body::{Body, Role};
use ck_topo::make::{extrude, revolve::revolve, ExtrudeOpts, Profile};

/// World frame of a plane reference (std / plane feature / planar face).
pub fn plane_frame(br: &Brep, state: &RegenState, r: &GeomRef) -> Option<Frame> {
    match r {
        GeomRef::Std(p) => Some(p.frame()),
        GeomRef::Feature(id) => state.plane(id),
        GeomRef::Face(id) => crate::refs::face_frame(br.body.as_ref()?, id),
        _ => None,
    }
}

/// Pattern axis `(origin, unit direction)`. `{kind:"axis", axis:"x|y|z"}`
/// = world axes through the origin. GUESS: other references unsupported.
pub fn axis_of(r: &GeomRef) -> Option<(Vec3, Vec3)> {
    match r {
        GeomRef::Axis(a) => match a.as_str() {
            "x" => Some((Vec3::ZERO, Vec3::X)),
            "y" => Some((Vec3::ZERO, Vec3::Y)),
            "z" => Some((Vec3::ZERO, Vec3::Z)),
            _ => None,
        },
        _ => None,
    }
}

fn reflect_frame(m: &Xform, f: &Frame) -> Frame {
    let (x, y) = (m.apply_vec(f.x), m.apply_vec(f.y));
    Frame { o: m.apply_point(f.o), x, y, z: x.cross(y) }
}

/// Swap the `C0` / `C1` names of an extruded body and restore the seed's
/// face order (C0 before C1 per region; pinned: mirrored copy lists
/// `F7.C0@F5.1, F7.C1@F5.1, F7.S.e1@F5.1`).
fn swap_caps(b: &mut Body) -> Body {
    for f in b.faces.iter_mut() {
        if !f.alive {
            continue;
        }
        if let Role::Cap(k) = f.name.role {
            if k <= 1 {
                f.name.role = Role::Cap(1 - k);
            }
        }
    }
    // within each region the caps are adjacent: swap each C1,C0 pair back
    let mut order: Vec<ck_topo::body::FaceId> = b.face_ids().collect();
    for i in 1..order.len() {
        let (p, q) = (&b.face(order[i - 1]).name, &b.face(order[i]).name);
        if p.role == Role::Cap(1) && q.role == Role::Cap(0) && p.region == q.region {
            order.swap(i - 1, i);
        }
    }
    ck_ops::boolean::assemble::rebuild_in_order(b, &order)
}

/// Mirror image of a seed tool in the plane `pl`, rebuilt from its recipe.
pub fn mirror_tool(feature: &str, recipe: &Recipe, pl: &Plane) -> Result<Body, String> {
    let m = Xform::mirror(pl);
    match recipe {
        Recipe::Extrude { profile, opts } => {
            let p = Profile::new(reflect_frame(&m, &profile.frame), profile.regions.clone());
            let o = ExtrudeOpts { s0: -opts.s1, s1: -opts.s0, dir: opts.dir.map(|d| -m.apply_vec(d)), draft: opts.draft };
            let mut b = extrude(feature, &p, o)?;
            Ok(swap_caps(&mut b))
        }
        Recipe::Revolve { profile, o, d, a0, a1 } => {
            // M·R(d, a) = R(−M d, a)·M
            let p = Profile::new(reflect_frame(&m, &profile.frame), profile.regions.clone());
            revolve(feature, &p, m.apply_point(*o), -m.apply_vec(*d), *a0, *a1)
        }
    }
}

/// Exact mirror image of a body in the plane `pl` (ck-topo transforms are
/// rigid-only). Geometry is rebuilt on RIGHT-handed frames so every
/// surface/curve evaluates exactly to the image of the original:
/// planes / extrusions / revolutions / B-splines keep their parameters and
/// flip `sense`; cylinders, cones, spheres, tori take frame (Mx, My, −Mz)
/// with v -> −v in their pcurves (cones also negate the semi-angle) and keep
/// `sense`; conic edges take frame (Mx, My, Mx×My). Every loop is reversed
/// (a reflection turns CCW loops CW about the image normal).
pub fn reflect_body(b: &Body, pl: &Plane) -> Result<Body, String> {
    use ck_geom::curve::Curve;
    use ck_geom::curve2::Curve2;
    use ck_geom::surface::Surface;
    use ck_geom::Vec2;
    let m = Xform::mirror(pl);
    let (p, v) = (|q: Vec3| m.apply_point(q), |d: Vec3| m.apply_vec(d));
    let rh = |f: &Frame| {
        let (x, y) = (v(f.x), v(f.y));
        Frame { o: p(f.o), x, y, z: x.cross(y) }
    };
    let curve = |c: &Curve| -> Curve {
        match c {
            Curve::Line { o, d } => Curve::Line { o: p(*o), d: v(*d) },
            Curve::Circle { frame, r } => Curve::Circle { frame: rh(frame), r: *r },
            Curve::Ellipse { frame, a, b } => Curve::Ellipse { frame: rh(frame), a: *a, b: *b },
            Curve::BSpline(s) => Curve::BSpline(s.map(|q| p(q))),
            // GUESS: analytic carrier frames transform like ellipse frames
            Curve::Conic { frame, p, e } => Curve::Conic { frame: rh(frame), p: *p, e: *e },
            Curve::Sinusoid { frame, amp, freq, phase } => {
                Curve::Sinusoid { frame: rh(frame), amp: *amp, freq: *freq, phase: *phase }
            }
        }
    };
    let flip_v = |c: &Curve2| -> Curve2 {
        let f = |q: Vec2| Vec2::new(q.x, -q.y);
        match c {
            Curve2::Line { o, d } => Curve2::Line { o: f(*o), d: f(*d) },
            Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: f(*c), x: f(*x), r: *r, ccw: !ccw },
            Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: f(*c), x: f(*x), a: *a, b: *b, ccw: !ccw },
            Curve2::BSpline(s) => Curve2::BSpline(s.map(f)),
        }
    };
    let mut out = b.clone();
    for vx in &mut out.vertices {
        vx.p = p(vx.p);
    }
    for e in &mut out.edges {
        e.curve = curve(&e.curve);
    }
    let mut vflip = vec![false; out.faces.len()];
    for (i, f) in out.faces.iter_mut().enumerate() {
        let lh = |fr: &Frame| Frame { o: p(fr.o), x: v(fr.x), y: v(fr.y), z: -v(fr.z) };
        let (s, flip_sense, fv) = match &f.surface {
            Surface::Plane { frame } => (Surface::Plane { frame: rh(frame) }, true, false),
            Surface::Cylinder { frame, r } => (Surface::Cylinder { frame: lh(frame), r: *r }, false, true),
            Surface::Cone { frame, r, semi_angle } => (Surface::Cone { frame: lh(frame), r: *r, semi_angle: -semi_angle }, false, true),
            Surface::Sphere { frame, r } => (Surface::Sphere { frame: lh(frame), r: *r }, false, true),
            Surface::Torus { frame, major, minor } => (Surface::Torus { frame: lh(frame), major: *major, minor: *minor }, false, true),
            Surface::Extrusion { profile, dir } => (Surface::Extrusion { profile: Box::new(curve(profile)), dir: v(*dir) }, true, false),
            // M·R(a, u) = R(−M a, u)·M
            Surface::Revolution { profile, axis_o, axis } => {
                (Surface::Revolution { profile: Box::new(curve(profile)), axis_o: p(*axis_o), axis: -v(*axis) }, true, false)
            }
            Surface::BSpline(s) => {
                let mut s = s.clone();
                s.ctrl.iter_mut().for_each(|q| *q = p(*q));
                (Surface::BSpline(s), true, false)
            }
            Surface::Screw { profile, axis_o, axis, rise, urange, vrange } => (
                Surface::Screw { profile: Box::new(curve(profile)), axis_o: p(*axis_o), axis: -v(*axis), rise: -*rise, urange: *urange, vrange: *vrange },
                true,
                false,
            ),
            Surface::Offset { .. } => return Err(super::extrude::todo("mirror of an offset surface")),
        };
        f.surface = s;
        if flip_sense {
            f.sense = !f.sense;
        }
        vflip[i] = fv;
    }
    for l in 0..out.loops.len() {
        let face = out.loops[l].face.idx();
        let mut cs = out.loops[l].coedges.clone();
        cs.reverse();
        for &c in &cs {
            let ce = &mut out.coedges[c.idx()];
            ce.rev = !ce.rev;
            if vflip[face] {
                ce.pcurve = ce.pcurve.as_ref().map(&flip_v);
            }
        }
        out.loops[l].coedges = cs;
    }
    Ok(out)
}


/// Does the pattern / mirror carry the threads of its seeds along (`carryThreads`, default true)?
fn carry_threads(f: &Feature) -> bool {
    !matches!(f.raw.get("carryThreads"), Some(ck_json::Json::Bool(false)))
}

/// Split `seeds` into the geometry seeds and the thread entries to carry: the explicit thread
/// seeds, plus (with `auto`) the threads sitting on a faces of a geometry seed
/// (`thread::host_of(face)` is the seed). Unknown thread ids give a warning.
fn thread_sources(state: &mut RegenState, f: &Feature, seeds: &[Ref], auto: bool) -> (Vec<Ref>, Vec<ck_json::Json>) {
    let is_thread = |id: &str| state.part.features.iter().any(|g| g.id == id && g.type_name == "thread");
    let (mut geo, mut explicit) = (vec![], vec![]);
    for s in seeds {
        if is_thread(s.as_str()) {
            explicit.push(s.as_str().to_string());
        } else {
            geo.push(s.clone());
        }
    }
    let mut out: Vec<ck_json::Json> = vec![];
    for (id, j) in &state.result.threads {
        let host = j.get_str("face").map_or("", thread::host_of);
        let hit = explicit.iter().any(|e| e == id) || (auto && geo.iter().any(|g| g.as_str() == host));
        if hit {
            out.push(j.clone());
        }
    }
    for e in &explicit {
        if !out.iter().any(|j| j.get_str("id") == Some(e.as_str())) {
            state.result.warnings.push(crate::regen::RegenError {
                feature: f.id.clone(),
                error: FeatureError::new(thread::THREAD_NOT_CARRIED)
                    .with("thread", ck_json::Json::Str(e.clone()))
                    .with("reason", ck_json::Json::Str("the thread feature produced no entry (it failed or is suppressed)".into()))
                    .with("hint", ck_json::Json::Str(format!("thread {e} is a seed of {} but has nothing to copy", f.id))),
            });
        }
    }
    (geo, out)
}

/// Carry every source thread onto every instance (`xs[k]` is instance `k + 1`).
fn carry_all(br: &mut Brep, state: &mut RegenState, f: &Feature, srcs: &[ck_json::Json], xs: &[Xform], rebuild: bool, mirrored: bool) {
    for (k, x) in xs.iter().enumerate() {
        let label = format!("{}.{}", f.id, k + 1);
        for src in srcs {
            if let Err(e) = thread::carry(br, state, src, x, &label, &f.id, rebuild, mirrored) {
                state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: e });
            }
        }
    }
}

fn seed_tools<'a>(br: &'a Brep, seeds: &[Ref]) -> Result<Vec<(&'a str, &'a Tool)>, FeatureError> {
    if seeds.is_empty() {
        return Err(FeatureError::new(NO_SEEDS));
    }
    let mut out = vec![];
    for s in seeds {
        if let Some((id, t)) = br.tools.iter().rev().find(|(id, _)| id == s.as_str()) {
            out.push((id.as_str(), &**t));
            continue;
        }
        // a mirror seed: its copies, one tool per seed of the mirror (`<id>#<k>`, the latest regeneration's)
        let pre = format!("{}#", s.as_str());
        let mut copies: Vec<(&str, &Tool)> = vec![];
        for (id, t) in br.tools.iter().rev() {
            let Some(k) = id.strip_prefix(&pre) else { continue };
            if k == "0" {
                copies.push((id.as_str(), &**t));
                break;
            }
            copies.push((id.as_str(), &**t));
        }
        if copies.is_empty() {
            return Err(FeatureError::new(codes::MISSING_REFERENCE));
        }
        copies.reverse();
        out.extend(copies);
    }
    Ok(out)
}

/// `mirror`.
pub fn exec_mirror(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Mirror { seeds, plane, body } = &f.kind else { unreachable!() };
    let fr = plane.as_ref().and_then(|p| plane_frame(br, state, p)).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let pl = Plane::new(fr.o, fr.z);
    state.result.warnings.retain(|w| w.feature != f.id);
    let auto = carry_threads(f);
    let mx = Xform::mirror(&pl);
    if *body {
        // Pinned (probe): the whole current body reflected, renamed
        // `@<feature>.1`, one `boolean Unite (n × n faces)`; seeds ignored.
        // GUESS: no body -> missingReference.
        let b = br.body.as_ref().ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
        let mut copy = reflect_body(b, &pl).map_err(|e| FeatureError::new(&e))?;
        copy.push_instance(&format!("{}.1", f.id));
        // every thread of the body is mirrored with it (the copy's faces exist after the combine)
        let srcs: Vec<ck_json::Json> = if auto { state.result.threads.iter().map(|t| t.1.clone()).collect() } else { vec![] };
        // Pinned: a copy clear of the body is appended with NO boolean span
        // (reflected cone 30 off the plane); touching copies are united.
        // GUESS: "clear" = disjoint bounding boxes (exact bounds ± tol).
        let (alo, ahi) = super::body_box(b);
        let (blo, bhi) = super::body_box(&copy);
        let t = super::TOL;
        let apart = alo.x > bhi.x + t || blo.x > ahi.x + t || alo.y > bhi.y + t || blo.y > ahi.y + t || alo.z > bhi.z + t || blo.z > ahi.z + t;
        if apart {
            let mut nb = b.clone();
            nb.append(&copy);
            br.body = Some(nb);
        } else {
            br.combine(&copy, Combine::Add, false)?;
        }
        carry_all(br, state, f, &srcs, &[mx], false, true);
        return Ok(());
    }
    if seeds.is_empty() {
        return Err(FeatureError::new(NO_SEEDS));
    }
    let (geo, srcs) = thread_sources(state, f, seeds, auto);
    let mut copies = vec![];
    for (id, t) in seed_tools(br, &geo).or_else(|e| if geo.is_empty() && !seeds.is_empty() { Ok(vec![]) } else { Err(e) })? {
        let mut b = match t.recipe.as_ref() {
            Some(recipe) => mirror_tool(id, recipe, &pl).map_err(|e| FeatureError::new(&e))?,
            // A tool made without a recipe (the tool body of a targeted join / cut / intersect on a named body): reflect the body itself,
            // as `body: true` does, faces keep their names.
            None => reflect_body(&t.body, &pl).map_err(|e| FeatureError::new(&e))?,
        };
        // GUESS: every seed's copy is instance 1
        b.push_instance(&format!("{}.1", f.id));
        copies.push((b, t.combine));
    }
    // Pinned: one boolean per seed; a mirrored cut that misses is silent.
    for (b, c) in &copies {
        br.combine(b, *c, false)?;
    }
    // the copies are the mirror's own tools (`<id>#<k>`, one per seed in order), so a later mirror or pattern can name
    // this mirror as a seed and copy its copies, each with its own boss / cut combine
    for (k, (b, c)) in copies.into_iter().enumerate() {
        br.tools.push((format!("{}#{k}", f.id), std::rc::Rc::new(Tool { body: b, combine: c, recipe: None })));
    }
    carry_all(br, state, f, &srcs, &[mx], true, true);
    Ok(())
}

/// Seeds of a pattern feature (`curvepattern` is not a typed kind: its seeds come from the raw DOM).
fn seeds_of(f: &Feature) -> Vec<Ref> {
    match &f.kind {
        FeatureKind::LPattern { seeds, .. } | FeatureKind::CPattern { seeds, .. } => seeds.clone(),
        _ => f.raw.get("seeds").and_then(|v| v.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str().map(|s| Ref(s.to_string()))).collect()).unwrap_or_default(),
    }
}

/// Middle of what is patterned: the centre of the box of the seeds' tools (None when no seed has a tool).
fn seeds_mid(br: &Brep, seeds: &[Ref]) -> Option<Vec3> {
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    let mut any = false;
    for s in seeds {
        if let Some((_, t)) = br.tools.iter().rev().find(|(id, _)| id == s.as_str()) {
            let (a, b) = super::body_box(&t.body);
            lo = Vec3::new(lo.x.min(a.x), lo.y.min(a.y), lo.z.min(a.z));
            hi = Vec3::new(hi.x.max(b.x), hi.y.max(b.y), hi.z.max(b.z));
            any = true;
        }
    }
    any.then(|| (lo + hi) * 0.5)
}

/// Seeds and instance transforms of an `lpattern` / `cpattern` / `curvepattern` (`xs[k]` is instance `k + 1`).
fn pattern_xforms(br: &mut Brep, state: &RegenState, f: &Feature) -> Result<(Vec<Ref>, Vec<Xform>), FeatureError> {
    let seeds = seeds_of(f);
    let xs = match &f.kind {
        FeatureKind::LPattern { .. } => {
            let to = |k: &str| f.raw.get(k).and_then(|v| v.as_str()) == Some("toObject");
            let mid = if to("distanceType") || to("distanceType2") { seeds_mid(br, &seeds) } else { None };
            let a = pattern_place::row(br, state, f, "", mid, count_of)?.ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            let b = pattern_place::row(br, state, f, "2", mid, count_of)?;
            if let Some(b) = b.as_ref().filter(|b| b.count > 1 && a.count > 1) {
                if a.d.cross(b.d).len() < 1e-9 {
                    return Err(FeatureError::new(codes::INVALID_VALUE)
                        .with("field", ck_json::Json::Str("dir2".into()))
                        .with("detail", ck_json::Json::Str("the second direction runs along the first: pick a direction across it".into())));
                }
            }
            if a.count as f64 * b.as_ref().map_or(1.0, |b| b.count.max(1) as f64) > MAX_INSTANCES {
                return Err(FeatureError::new(codes::INVALID_VALUE));
            }
            pattern_place::grid(&a, b.as_ref())
        }
        FeatureKind::CPattern { axis, equal, reverse, .. } => {
            let count = state.num(f, "count", 0.0)?;
            let angle = state.num(f, "angle", 360.0)?;
            let (o, d) = axis.as_ref().and_then(|a| crate::planes::resolve_axis(state, br, a)).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            ck_ops::boolean::apply::circular_instances(o, d, angle, count_of(count)?, *equal, *reverse)
        }
        _ if f.type_name == "curvepattern" => pattern_place::path_instances(br, state, f, seeds_mid(br, &seeds).unwrap_or(Vec3::ZERO), count_of)?,
        _ => unreachable!(),
    };
    Ok((seeds, xs))
}

/// A thread feature that comes AFTER a pattern / mirror of its host (the thread was added to a
/// patterned hole): carry it onto that pattern's instances too, as if it had come first. The
/// patterns run in history order, so a pattern of the pattern picks up the new instance threads.
pub fn carry_into_earlier(br: &mut Brep, state: &mut RegenState, tf: &Feature) {
    let part = state.part;
    let Some(pos) = part.features.iter().position(|g| g.id == tf.id) else { return };
    let mut fresh: Vec<String> = vec![tf.id.clone()];
    for p in &part.features[..pos] {
        if p.suppressed || !carry_threads(p) {
            continue;
        }
        let (seeds, xs, mirror) = match &p.kind {
            FeatureKind::LPattern { .. } | FeatureKind::CPattern { .. } => match pattern_xforms(br, state, p) {
                Ok((s, xs)) => (s, xs, false),
                Err(_) => continue,
            },
            FeatureKind::Mirror { seeds, plane, body: false } => {
                let Some(fr) = plane.as_ref().and_then(|g| plane_frame(br, state, g)) else { continue };
                (seeds.clone(), vec![Xform::mirror(&Plane::new(fr.o, fr.z))], true)
            }
            _ if p.type_name == "curvepattern" => match pattern_xforms(br, state, p) {
                Ok((s, xs)) => (s, xs, false),
                Err(_) => continue,
            },
            _ => continue,
        };
        let srcs: Vec<ck_json::Json> = state
            .result
            .threads
            .iter()
            .filter(|(id, j)| fresh.contains(id) && seeds.iter().any(|s| s.as_str() == j.get_str("face").map_or("", thread::host_of)))
            .map(|t| t.1.clone())
            .collect();
        if srcs.is_empty() {
            continue;
        }
        carry_all(br, state, p, &srcs, &xs, true, mirror);
        for src in &srcs {
            for k in 0..xs.len() {
                fresh.push(format!("{}@{}.{}", src.get_str("id").unwrap_or(""), p.id, k + 1));
            }
        }
    }
}

/// `lpattern` / `cpattern` / `curvepattern`.
pub fn exec_pattern(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if f.type_name == "curvepattern" {
        // the path is checked first (an empty curvepattern is `missingPath`, as the oracle answers)
        super::sweep::path_segs(br, state, pattern_place::path_ref(f).as_ref())?;
    }
    if seeds_of(f).is_empty() {
        return Err(FeatureError::new(NO_SEEDS));
    }
    let (seeds, xs) = pattern_xforms(br, state, f)?;
    state.result.warnings.retain(|w| w.feature != f.id);
    let (geo, srcs) = thread_sources(state, f, &seeds, carry_threads(f));
    let tools: Vec<(Body, Combine)> = if xs.is_empty() || geo.is_empty() { vec![] } else { seed_tools(br, &geo)?.into_iter().map(|(_, t)| (t.body.clone(), t.combine)).collect() };
    // the body before the copies, for the check of copies that change nothing (only cloned when there are copies)
    let before = if tools.is_empty() { None } else { br.body.clone() };
    // Pinned: per seed ONE boolean with all its instances (`boolean Subtract
    // (7 × 6 faces)` for 2 copies of a 3-face hole), seeds in order.
    // Instances are appended while their box is clear of the tool so far,
    // else united (stator F14: `boolean Unite (50 × 10 faces) → 60 faces`
    // before `boolean Subtract (168 × 70 faces)`).
    let mut all = Body::new();
    let mut last = Combine::Add;
    let mut overlap = false;
    for (seed, c) in &tools {
        let mut tool: Option<Body> = None;
        let spans0 = br.spans.len();
        for (k, x) in xs.iter().enumerate() {
            if ck_core::budget::expired() {
                return Err(FeatureError::new(crate::regen::codes::FEATURE_BUDGET));
            }
            let inst = ck_ops::boolean::apply::instance(seed, x, &f.id, k + 1);
            tool = Some(match tool {
                None => inst,
                Some(t) => accumulate(br, t, inst)?,
            });
        }
        let tool = tool.unwrap_or_default();
        let mut united = br.spans.len() > spans0;
        br.combine(&tool, *c, false)?;
        // the pattern's own tool is the seed AND its copies, so a pattern of this pattern copies the whole row
        // (a union made only for this record is not a step of the feature: its span is dropped)
        let spans = br.spans.len();
        let whole = accumulate(br, seed.clone(), tool)?;
        united |= br.spans.len() > spans;
        br.spans.truncate(spans);
        // copies that overlap each other (or the seed): the united volume falls short of one seed's volume per copy
        if united && !overlap {
            let (v1, vw) = (ck_topo::mass::mass_props(seed).volume.abs(), ck_topo::mass::mass_props(&whole).volume.abs());
            overlap = vw < v1 * (xs.len() + 1) as f64 * (1.0 - 1e-7) - 1e-9;
        }
        all.append(&whole);
        last = *c;
    }
    // the pattern's own tool (seeds of later patterns): the seeds and every instance
    if !tools.is_empty() {
        br.tools.push((f.id.clone(), std::rc::Rc::new(Tool { body: all, combine: last, recipe: None })));
        copy_warnings(br, state, f, &tools, &xs, before.as_ref(), overlap);
    }
    // threads of the seeds follow onto every instance (cosmetic entries, modeled grooves again)
    carry_all(br, state, f, &srcs, &xs, true, false);
    Ok(())
}

/// Copies that changed nothing (a cut that misses, a boss inside the material).
pub const COPIES_NO_EFFECT: &str = "patternCopiesNoEffect";
/// Boss copies that do not touch the body: each is a body of its own.
pub const COPIES_APART: &str = "patternCopiesApart";
/// Copies that overlap each other or the seed.
pub const COPIES_OVERLAP: &str = "patternCopiesOverlap";

/// Copy numbers of feature `fid` a face name (suffix chain `@F9.1@F12.3`, or an absorbed face's whole id) carries.
fn copy_numbers(name: &str, fid: &str, out: &mut Vec<usize>) {
    for tok in name.split('@').skip(1) {
        if let Some(k) = tok.strip_prefix(fid).and_then(|r| r.strip_prefix('.')).and_then(|r| r.parse::<usize>().ok()) {
            out.push(k);
        }
    }
}

fn json_list(v: &[usize]) -> ck_json::Json {
    ck_json::Json::Arr(v.iter().map(|&k| ck_json::Json::Num(k as f64)).collect())
}

/// The warnings of a built pattern, found cheaply from the result's face names: a copy that left no face (and no
/// absorbed face name) changed nothing, unless it is a cut whose box meets the body and that cuts material away
/// whole (then an exact intersection decides); a lump made only of copies is copies apart from the body.
fn copy_warnings(br: &Brep, state: &mut RegenState, f: &Feature, tools: &[(Body, Combine)], xs: &[Xform], before: Option<&Body>, overlap: bool) {
    let Some(body) = br.body.as_ref() else { return };
    let n = xs.len();
    let mut present = vec![false; n + 1];
    let mut ks = vec![];
    for fid in body.face_ids() {
        let face = body.face(fid);
        ks.clear();
        copy_numbers(&face.name.suffix, &f.id, &mut ks);
        for a in &face.absorbed {
            copy_numbers(a, &f.id, &mut ks);
        }
        for &k in &ks {
            if k <= n {
                present[k] = true;
            }
        }
    }
    let warn = |state: &mut RegenState, e: FeatureError| state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: e });
    let mut missed: Vec<usize> = vec![];
    for k in 1..=n {
        if present[k] {
            continue;
        }
        // a faceless cut copy whose box meets the body may still have taken material (it swallowed a whole lump)
        let took = before.is_some_and(|b| {
            let (blo, bhi) = super::body_box(b);
            tools.iter().filter(|(_, c)| *c == Combine::Cut).any(|(seed, _)| {
                let inst = ck_ops::boolean::apply::instance(seed, &xs[k - 1], &f.id, k);
                let (lo, hi) = super::body_box(&inst);
                let t = super::TOL;
                let apart = lo.x > bhi.x + t || blo.x > hi.x + t || lo.y > bhi.y + t || blo.y > hi.y + t || lo.z > bhi.z + t || blo.z > hi.z + t;
                !apart
                    && ck_ops::boolean::boolean(b, &inst, ck_ops::boolean::Op::Intersect, super::TOL)
                        .map_or(false, |o| ck_topo::mass::mass_props(&o.body).volume.abs() > 1e-9)
            })
        });
        if !took {
            missed.push(k);
        }
    }
    if !missed.is_empty() {
        warn(
            state,
            FeatureError::new(COPIES_NO_EFFECT)
                .with("copies", json_list(&missed))
                .with("of", ck_json::Json::Num(n as f64))
                .with("hint", ck_json::Json::Str(format!("{} of the {n} copies change nothing: they miss the body (or lie inside its material). Check the count, spacing, direction and where the copies land", missed.len()))),
        );
    }
    // boss copies standing alone
    if tools.iter().any(|(_, c)| *c == Combine::Add) {
        let mut apart: Vec<usize> = vec![];
        for l in body.lump_ids() {
            let mut lump: Vec<usize> = vec![];
            let mut only = true;
            for fid in ck_topo::mass::lump_faces(body, l) {
                ks.clear();
                copy_numbers(&body.face(fid).name.suffix, &f.id, &mut ks);
                if ks.is_empty() {
                    only = false;
                    break;
                }
                lump.extend(ks.iter().copied());
            }
            if only {
                apart.extend(lump);
            }
        }
        apart.sort_unstable();
        apart.dedup();
        if !apart.is_empty() {
            warn(
                state,
                FeatureError::new(COPIES_APART)
                    .with("copies", json_list(&apart))
                    .with("hint", ck_json::Json::Str(format!("{} copies do not touch the body, so each stands as a body of its own. Check the count, spacing and direction", apart.len()))),
            );
        }
    }
    if overlap {
        let mut e = FeatureError::new(COPIES_OVERLAP).with("hint", ck_json::Json::Str("copies overlap each other: the spacing is smaller than what is patterned. Raise the spacing or lower the count".into()));
        // a one-direction linear pattern: the width of what is patterned along the direction is the least spacing that keeps copies apart
        if let (FeatureKind::LPattern { dir: Some(d), .. }, true) = (&f.kind, f.raw.get("dir2").map_or(true, |v| matches!(v, ck_json::Json::Null))) {
            if let Some((_, d)) = axis_of(d) {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                let d = Vec3::new(d.x.abs(), d.y.abs(), d.z.abs());
                for (seed, _) in tools {
                    let (a, b) = super::body_box(seed);
                    lo = lo.min(a.dot(d));
                    hi = hi.max(b.dot(d));
                }
                let w = ((hi - lo) * 1e6).round() / 1e6;
                e = e.with("minSpacing", ck_json::Json::Num(w));
                e.params.retain(|p| p.0 != "hint");
                e = e.with("hint", ck_json::Json::Str(format!("copies overlap each other: what is patterned is {w} mm wide along the direction, so a spacing of {w} mm or more keeps them apart")));
            }
        }
        warn(state, e);
    }
}

/// Pinned: count 0 / 1 / negative -> no copies and no error (no boolean
/// span); 2.6 -> 3. GUESS: round half away from zero.
fn count_of(c: f64) -> Result<usize, FeatureError> {
    if !c.is_finite() || c < 1.0 {
        return Ok(1);
    }
    // robustness: an absurd count would allocate/loop forever (wasm trap) -> structured error
    if c > MAX_INSTANCES {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    Ok(c.round() as usize)
}

/// Upper bound on pattern instances (far above any real part).
const MAX_INSTANCES: f64 = 10_000.0;

/// Mirror / pattern with an empty `seeds` list (oracle code).
pub const NO_SEEDS: &str = "noSeeds";

/// Add `b` to the tool `t`: appended when their boxes are clear (GUESS:
/// exact bounds ± tol, the `mirror body:true` rule), else `boolean Unite`.
pub fn accumulate(br: &mut Brep, mut t: Body, b: Body) -> Result<Body, FeatureError> {
    let (alo, ahi) = super::body_box(&t);
    let (blo, bhi) = super::body_box(&b);
    let e = super::TOL;
    let apart = alo.x > bhi.x + e || blo.x > ahi.x + e || alo.y > bhi.y + e || blo.y > ahi.y + e || alo.z > bhi.z + e || blo.z > ahi.z + e;
    if apart {
        t.append(&b);
        return Ok(t);
    }
    let out = ck_ops::boolean::boolean(&t, &b, ck_ops::boolean::Op::Union, super::TOL).map_err(|e| FeatureError::new(&e))?;
    if let Some(m) = &out.mismatch {
        return Err(super::volume_mismatch(m));
    }
    br.spans.push(super::boolean_span(ck_ops::boolean::Op::Union, t.num_faces(), b.num_faces(), out.body.num_faces()));
    Ok(out.body)
}
