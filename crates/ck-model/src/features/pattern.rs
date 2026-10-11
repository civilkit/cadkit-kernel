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
use super::{Brep, FeatureKind};
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

fn seed_tools<'a>(br: &'a Brep, seeds: &[Ref]) -> Result<Vec<(&'a str, &'a Tool)>, FeatureError> {
    if seeds.is_empty() {
        return Err(FeatureError::new(NO_SEEDS));
    }
    seeds
        .iter()
        .map(|s| {
            br.tools
                .iter()
                .rev()
                .find(|(id, _)| id == s.as_str())
                .map(|(id, t)| (id.as_str(), t))
                .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))
        })
        .collect()
}

/// `mirror`.
pub fn exec_mirror(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Mirror { seeds, plane, body } = &f.kind else { unreachable!() };
    let fr = plane.as_ref().and_then(|p| plane_frame(br, state, p)).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let pl = Plane::new(fr.o, fr.z);
    if *body {
        // Pinned (probe): the whole current body reflected, renamed
        // `@<feature>.1`, one `boolean Unite (n × n faces)`; seeds ignored.
        // GUESS: no body -> missingReference.
        let b = br.body.as_ref().ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
        let mut copy = reflect_body(b, &pl).map_err(|e| FeatureError::new(&e))?;
        copy.push_instance(&format!("{}.1", f.id));
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
            return Ok(());
        }
        return br.combine(&copy, Combine::Add, false);
    }
    let mut copies = vec![];
    for (id, t) in seed_tools(br, seeds)? {
        let recipe = t.recipe.as_ref().ok_or_else(|| FeatureError::new(&super::extrude::todo("mirror of a tool without a recipe")))?;
        let mut b = mirror_tool(id, recipe, &pl).map_err(|e| FeatureError::new(&e))?;
        // GUESS: every seed's copy is instance 1
        b.push_instance(&format!("{}.1", f.id));
        copies.push((b, t.combine));
    }
    // Pinned: one boolean per seed; a mirrored cut that misses is silent.
    for (b, c) in copies {
        br.combine(&b, c, false)?;
    }
    Ok(())
}

/// `lpattern` / `cpattern`.
pub fn exec_pattern(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if let FeatureKind::LPattern { seeds, .. } | FeatureKind::CPattern { seeds, .. } = &f.kind {
        if seeds.is_empty() {
            return Err(FeatureError::new(NO_SEEDS));
        }
    }
    let (seeds, xs) = match &f.kind {
        FeatureKind::LPattern { seeds, dir, reverse, .. } => {
            let spacing = state.num(f, "spacing", 0.0)?;
            let count = state.num(f, "count", 0.0)?;
            let (_, d) = dir.as_ref().and_then(axis_of).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            (seeds, ck_ops::boolean::apply::linear_instances(d, spacing, count_of(count)?, *reverse))
        }
        FeatureKind::CPattern { seeds, axis, equal, reverse, .. } => {
            let count = state.num(f, "count", 0.0)?;
            let angle = state.num(f, "angle", 360.0)?;
            let (o, d) = axis.as_ref().and_then(axis_of).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            (seeds, ck_ops::boolean::apply::circular_instances(o, d, angle, count_of(count)?, *equal, *reverse))
        }
        _ => unreachable!(),
    };
    let tools: Vec<(Body, Combine)> = if xs.is_empty() { vec![] } else { seed_tools(br, seeds)?.into_iter().map(|(_, t)| (t.body.clone(), t.combine)).collect() };
    // Pinned: per seed ONE boolean with all its instances (`boolean Subtract
    // (7 × 6 faces)` for 2 copies of a 3-face hole), seeds in order.
    for (seed, c) in &tools {
        let mut tool = Body::new();
        for (k, x) in xs.iter().enumerate() {
            tool.append(&ck_ops::boolean::apply::instance(seed, x, &f.id, k + 1));
        }
        br.combine(&tool, *c, false)?;
    }
    Ok(())
}

/// Pinned: count 0 / 1 / negative -> no copies and no error (no boolean
/// span); 2.6 -> 3. GUESS: round half away from zero.
fn count_of(c: f64) -> Result<usize, FeatureError> {
    if !c.is_finite() || c < 1.0 {
        return Ok(1);
    }
    Ok(c.round() as usize)
}

/// Mirror / pattern with an empty `seeds` list (oracle code).
pub const NO_SEEDS: &str = "noSeeds";
