//! Shell (hollow) a solid: offset every face inward by the wall thickness
//! and cut the offset cavity out of the body.
//!
//! Semantics pinned by oracle probes (box, opened cap / side, thickness
//! too large, outward):
//! - result = body - cavity; the cavity is the body offset by -t on every
//!   kept face. An opened (removed) face is pushed OUTWARD instead, so the
//!   cavity runs out through it and the face survives as a rim ring
//!   (unsuffixed name; every other face is split `~1` outer / `~2` inner by
//!   the boolean).
//! - `outward`: the wall grows outside - base = body offset by +t (opened
//!   faces stay put), cavity = the original body.
//! - errors: t <= 0 -> invalidValue; an offset edge shrinking / reversing ->
//!   shellTooThick{detail}; opened face not planar -> shellUnsupported;
//!   offset errors (offsetUnsupported / offsetTooLarge) pass through; a
//!   boolean failure -> shellFailed.
//!
//! The original (shell.rs:156, 18 KB) builds an "opening prism" per opened
//! face and verifies flush openings by volume; this version gets the same
//! topology by overshooting the opened face instead.

use crate::boolean::{boolean, Op};
use crate::offset::{offset_body, OffsetError, OffsetLimit};
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceId};

#[derive(Debug, Clone, PartialEq)]
pub struct ShellError {
    pub code: String,
    pub detail: Option<String>,
    /// For `shellTooThick` / `shellFailed`: the smallest radius (or gap) found and the thickness that would work.
    pub limit: Option<OffsetLimit>,
}

impl ShellError {
    fn code(c: &str) -> ShellError {
        ShellError { code: c.into(), detail: None, limit: None }
    }
    fn detail(c: &str, d: impl Into<String>) -> ShellError {
        ShellError { code: c.into(), detail: Some(d.into()), limit: None }
    }
}

fn from_offset(e: OffsetError) -> ShellError {
    if e.is_degenerate() {
        ShellError { code: "shellTooThick".into(), detail: e.detail, limit: e.limit }
    } else {
        ShellError { code: e.code.into(), detail: e.detail, limit: e.limit }
    }
}

/// Why a cavity cut failed, in numbers: the tightest curved face for this wall thickness.
/// `shellTooThick` with the radius and the thickness that would work when a face curves more
/// tightly than the wall is thick (the cavity folds over), else `shellFailed` saying that the
/// curvature is not the limit.
fn diagnose_cut_failure(body: &Body, is_open: &dyn Fn(FaceId) -> bool, t: f64, outward: bool, why: Option<&str>) -> ShellError {
    // parallel planar faces facing away from each other: the wall between them
    // must fit both thicknesses (one if a face of the pair is opened)
    if !outward {
        let planes: Vec<(FaceId, ck_geom::Vec3, ck_geom::Vec3)> = body
            .face_ids()
            .filter_map(|f| match &body.face(f).surface {
                Surface::Plane { frame } => Some((f, if body.face(f).sense { frame.z } else { -frame.z }, frame.o)),
                _ => None,
            })
            .collect();
        let mut narrow: Option<(f64, f64, String, String)> = None;
        for (i, &(f, nf, of)) in planes.iter().enumerate() {
            for &(g, ng, og) in &planes[i + 1..] {
                if nf.dot(ng) > -0.999 || (is_open(f) && is_open(g)) {
                    continue;
                }
                let gap = (of - og).dot(nf);
                let allowed = if is_open(f) || is_open(g) { gap } else { 0.5 * gap };
                if gap > 0.0 && t >= allowed * 0.999 && narrow.as_ref().map_or(true, |x| allowed < x.0) {
                    narrow = Some((allowed, gap, body.face_id_str(f), body.face_id_str(g)));
                }
            }
        }
        if let Some((allowed, gap, a, b)) = narrow {
            let limit = OffsetLimit { kind: "thin", min_radius: gap, max_thickness: (0.95 * allowed * 100.0).floor() / 100.0, face: Some(a.clone()) };
            let detail = format!("{a} and {b} are only {gap:.2} mm apart, too close for a wall of {t:.2} mm: use at most {:.2} mm", limit.max_thickness);
            return ShellError { code: "shellTooThick".into(), detail: Some(detail), limit: Some(limit) };
        }
    }
    let mut tightest: Option<(f64, String)> = None;
    for f in body.face_ids() {
        if is_open(f) {
            continue;
        }
        let face = body.face(f);
        // distance along the natural normal the cavity (inward) or the base (outward) moves
        let d = if outward { t } else { -t };
        let nat = if face.sense { d } else { -d };
        let r = match &face.surface {
            Surface::Cylinder { r, .. } | Surface::Sphere { r, .. } if nat < 0.0 => Some(*r),
            Surface::Torus { minor, .. } if nat < 0.0 => Some(*minor),
            Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Sphere { .. } | Surface::Torus { .. } | Surface::Cone { .. } => None,
            _ => crate::offset::face_min_radius(body, f, nat),
        };
        if let Some(r) = r {
            if tightest.as_ref().map_or(true, |(x, _)| r < *x) {
                tightest = Some((r, body.face_id_str(f)));
            }
        }
    }
    match tightest {
        Some((r, face)) if r <= t * 1.001 => {
            let limit = OffsetLimit { kind: "curvature", min_radius: r, max_thickness: (0.8 * r * 100.0).floor() / 100.0, face: Some(face.clone()) };
            let detail = crate::offset::limit_message(&face, t, &limit);
            ShellError { code: "shellTooThick".into(), detail: Some(detail), limit: Some(limit) }
        }
        Some((r, face)) => ShellError {
            code: "shellFailed".into(),
            detail: Some(format!(
                "the cavity could not be cut out of the shape, and not because the wall is too thick for its curvature ({face} has a smallest radius of {r:.2} mm against a wall of {t:.2} mm): this is a limit of the shell operation here, so a different thickness may not help; try opening another face or changing the shape slightly{}",
                why.map_or(String::new(), |w| format!(" (the boolean reported: {w})"))
            )),
            limit: Some(OffsetLimit { kind: "curvature", min_radius: r, max_thickness: 0.0, face: Some(face) }),
        },
        None => ShellError::code("shellFailed"),
    }
}

/// Hollow `body` with wall `thickness`, removing the `open` faces. A failure that is about the
/// wall being too thick for the shape (curvature, a narrow gap, an edge that vanishes) always
/// carries the thickness that would work (`limit.max_thickness`): where the failure itself does
/// not name one it is found by halving the wall until the shell succeeds.
pub fn shell(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Result<Body, ShellError> {
    match shell_once(body, open, thickness, outward, tol) {
        Err(mut e)
            if matches!(e.code.as_str(), "shellTooThick" | "shellFailed" | "offsetTooLarge")
                && e.limit.as_ref().map_or(true, |l| l.max_thickness <= 0.0)
                && (outward || near_pole_cuts(body, thickness, 6.0 * thickness).is_empty()) =>
        {
            if let Some(ok) = largest_working(body, open, thickness, outward, tol) {
                let l = e.limit.take();
                let max = floor_hundredth(0.95 * ok);
                e.limit = Some(match l {
                    Some(l) => OffsetLimit { max_thickness: max, ..l },
                    None => OffsetLimit { kind: "thin", min_radius: 2.0 * ok, max_thickness: max, face: None },
                });
                match e.detail.as_deref() {
                    None => e.detail = Some(format!("a wall of {thickness:.2} mm does not fit this shape: use at most {max:.2} mm")),
                    Some(d) if d.starts_with("walls touch") || d.starts_with(INVALID_RESULT) => e.detail = Some(format!("{d}: use at most {max:.2} mm")),
                    _ => {}
                }
            }
            Err(e)
        }
        // a thickness the failure names must itself build (a valid body): else the largest below it that does, or none
        Err(mut e)
            if matches!(e.code.as_str(), "shellTooThick" | "shellFailed" | "offsetTooLarge")
                && e.limit.as_ref().is_some_and(|l| l.max_thickness > 0.0 && l.max_thickness < thickness) =>
        {
            let named = e.limit.as_ref().map_or(0.0, |l| l.max_thickness);
            if shell_once(body, open, named, outward, tol).is_err() {
                let max = largest_working(body, open, named, outward, tol).map_or(0.0, |ok| floor_hundredth(0.95 * ok));
                if let Some(l) = e.limit.as_mut() {
                    l.max_thickness = max;
                }
                let note = if max > 0.0 {
                    format!("; a wall of {named:.2} mm does not build here either: use at most {max:.2} mm")
                } else {
                    format!("; a wall of {named:.2} mm does not build here either, nor does a thinner one")
                };
                e.detail = Some(e.detail.take().unwrap_or_default() + &note);
            }
            Err(e)
        }
        r => r,
    }
}

/// Detail of the refusal when the hollowed body does not validate; `shell` appends the thickness that works.
const INVALID_RESULT: &str = "the hollowed body is not valid (the offset surfaces do not close into a clean wall on this shape)";

/// Detail of the refusal when two walls meet inside the cavity; `shell` appends the thickness that works.
const WALLS_MEET: &str = "walls touch: two parts of the shape are too close for this wall, so the cavity would be pinched shut or its offset surfaces overlap";

fn floor_hundredth(x: f64) -> f64 {
    (x * 100.0).floor() / 100.0
}

/// The largest wall below `thickness` (to a sixteenth of it, a 256th when nothing larger works) that still
/// shells: bisection on the failing thickness. `None` when not even that works.
fn largest_working(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Option<f64> {
    let (mut lo, mut hi) = (0.0f64, thickness);
    // (a failing shell can take seconds itself, and the refusal waits for the whole search: no new try starts after
    // SEARCH_MS of the host clock, so a refusal comes back with the best wall found so far, or none)
    const SEARCH_MS: f64 = 3000.0;
    let started = ck_core::budget::mark();
    // (four halvings; up to eight when nothing worked yet, e.g. a thin web far below the wall asked for)
    for i in 0..8 {
        if i >= 4 && lo > 0.0 {
            break;
        }
        if i > 0 && ck_core::budget::since(started) > SEARCH_MS {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if shell_once(body, open, mid, outward, tol).is_ok() {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo > 0.0).then_some(lo)
}

/// One shell attempt at exactly this wall, without the search for a wall that works (`shell` does that on a
/// failure). Callers that search themselves use this to keep each try to one shell.
pub fn shell_once(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Result<Body, ShellError> {
    let cuts = if outward { vec![] } else { near_pole_cuts(body, thickness, 6.0 * thickness) };
    if cuts.is_empty() {
        return shell_try(body, open, thickness, outward, tol, 6.0);
    }

    // The cavity of a point-ended loft is cut off by a plane some wall thicknesses short of the
    // apex. The nearer the plane, the less cavity is lost, but the cut-off front must still have
    // room for the wall (a hull section thinner than twice the wall has no cavity): try planes
    // from near to far.
    let mut first: Option<ShellError> = None;
    for factor in [4.0, 6.0, 8.0, 10.0, 12.0] {
        match shell_try(body, open, thickness, outward, tol, factor) {
            Ok(b) => {
                if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                    eprintln!("shell: pole cut at {factor} wall thicknesses worked");
                }
                return Ok(b);
            }
            Err(e) if matches!(e.code.as_str(), "invalidValue" | "shellUnsupported") => return Err(e),
            Err(e) => {
                if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                    eprintln!("shell: pole cut at {factor} wall thicknesses failed: {} {:?}", e.code, e.detail);
                }
                // report the failure that names the largest wall that works (a curvature limit), else the first
                let better = |a: &ShellError| if a.detail.as_deref().is_some_and(|d| d.contains("too blunt")) { 1e6 } else { a.limit.as_ref().map_or(0.0, |l| l.max_thickness) };
                if first.as_ref().map_or(true, |f| better(&e) > better(f)) {
                    first = Some(e);
                }
            }
        }
    }
    Err(first.unwrap())
}

/// An opened face and a closed neighbour that continues it smoothly across a shared edge (the rim of an opened
/// face that was rounded or tangent-blended before the shell): the offset surfaces of the two are parallel
/// (the opening moves outward, the wall inward) and never meet, so the cavity has no edge there. Returns the
/// names of the pair.
fn rim_tangent_to_open_face(body: &Body, open: &[FaceId]) -> Option<(String, String)> {
    for &f in open {
        for e in body.face_edges(f) {
            let fs = body.edge_faces(e);
            if fs.len() != 2 {
                continue;
            }
            let g = if fs[0] == f { fs[1] } else { fs[0] };
            if g == f || open.contains(&g) {
                continue;
            }
            let ed = body.edge(e);
            let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
            let nrm = |h: FaceId| {
                let sf = &body.face(h).surface;
                let (u, v, _) = sf.closest_point(p);
                sf.normal(u, v).try_normalize()
            };
            if let (Some(a), Some(b)) = (nrm(f), nrm(g)) {
                if a.cross(b).len() < 0.02 {
                    return Some((body.face_id_str(f), body.face_id_str(g)));
                }
            }
        }
    }
    None
}

/// Does `s` contain, through every point, a straight line along `n` (a wall standing square on a plane of normal `n`)?
fn ruled_along(s: &Surface, n: Vec3) -> bool {
    match s {
        Surface::Plane { frame } => frame.z.dot(n).abs() < 1e-9,
        Surface::Cylinder { frame, .. } => frame.z.cross(n).len() < 1e-9,
        Surface::Extrusion { dir, .. } => dir.try_normalize().is_some_and(|d| d.cross(n).len() < 1e-9),
        _ => false,
    }
}

/// Edges shorter than this in the outline of an opened face are slivers: the micrometre arcs a boolean leaves where a
/// cylinder touches the plane along a line (a boss tangent to the opened face).
const SLIVER: f64 = 1e-5;

/// The outline of planar face `f` of `b` as a profile on its plane's frame, sliver edges dropped. Each edge of the outline is
/// moved `grow(edge)` away from the face, in its plane (0 keeps it; only lines and arcs can move). The outer loop is the one
/// enclosing the most area. Gaps left by a dropped sliver or a moved edge are closed where the two pieces run through a
/// common point: two pieces of one arc, or two lines (extended to their crossing).
fn face_outline(b: &Body, f: FaceId, grow: &dyn Fn(ck_topo::body::EdgeId) -> f64) -> Result<ck_topo::make::Profile, String> {
    use ck_geom::curve2::Curve2;
    use ck_geom::Vec2;
    use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let face = b.face(f);
    let Surface::Plane { frame } = &face.surface else { return Err("the opened face is not planar".into()) };
    let mut loops: Vec<(ProfileLoop, Vec<f64>)> = vec![];
    for &l in &face.loops {
        let (mut segs, mut moves) = (vec![], vec![]);
        for (k, &c) in b.lp(l).coedges.iter().enumerate() {
            if !b.coedge(c).alive {
                continue;
            }
            let e = b.coedge(c).edge;
            if b.edge_length(e) < SLIVER {
                continue;
            }
            let ed = b.edge(e);
            let pc = match &b.coedge(c).pcurve {
                Some(pc) => pc.clone(),
                None => ck_geom::pcurve::exact_pcurve(&face.surface, &ed.curve, 1e-7).ok_or("an edge of the opened face has no plane image")?,
            };
            let (ta, tb) = b.coedge_params(c);
            segs.push(ProfileSeg { tag: format!("r{k}"), curve: pc, t0: ta.min(tb), t1: ta.max(tb), rev: tb < ta });
            moves.push(grow(e));
        }
        if !segs.is_empty() {
            loops.push((ProfileLoop::new(segs), moves));
        }
    }
    if loops.is_empty() {
        return Err("the opened face has no outline".into());
    }
    let k = (0..loops.len()).max_by(|&i, &j| loops[i].0.signed_area().abs().total_cmp(&loops[j].0.signed_area().abs())).unwrap();
    let outer_k = loops.remove(k);
    loops.insert(0, outer_k);
    let mut done: Vec<ProfileLoop> = vec![];
    for (i, (lp, moves)) in loops.into_iter().enumerate() {
        // the region on the left of travel: the outer loop runs counter-clockwise, holes clockwise
        let a = lp.signed_area();
        let (mut segs, moves) = if (i == 0) == (a > 0.0) {
            (lp.segs, moves)
        } else {
            (lp.segs.iter().rev().map(|s| s.reversed()).collect::<Vec<_>>(), moves.into_iter().rev().collect())
        };
        // move edges away from the face (to the right of travel)
        for (s, &d) in segs.iter_mut().zip(&moves) {
            if d == 0.0 {
                continue;
            }
            let tm = 0.5 * (s.t0 + s.t1);
            let tt = s.travel_tangent(tm);
            if !(tt.len() > 1e-12) {
                return Err("a degenerate edge in the outline".into());
            }
            let tt = tt.normalize();
            let right = Vec2::new(tt.y, -tt.x);
            s.curve = match &s.curve {
                Curve2::Line { o, d: dir } => Curve2::Line { o: *o + right * d, d: *dir },
                Curve2::Circle { c, x, r, ccw } => {
                    let out = (s.curve.eval(tm) - *c).dot(right) > 0.0;
                    let r2 = if out { r + d } else { r - d };
                    if r2 <= 1e-9 {
                        return Err("an arc of the outline shrinks to nothing".into());
                    }
                    Curve2::Circle { c: *c, x: *x, r: r2, ccw: *ccw }
                }
                _ => return Err("only lines and arcs of the outline can be moved".into()),
            };
        }
        // close the gaps: the next piece starts where the previous one ends
        let m = segs.len();
        for i in 0..m {
            let j = (i + m - 1) % m;
            let end = segs[j].end();
            if segs[i].start().dist(end) <= 1e-9 {
                continue;
            }
            if let (Curve2::Line { o: o1, d: d1 }, Curve2::Line { o: o2, d: d2 }) = (segs[j].curve.clone(), segs[i].curve.clone()) {
                let den = d1.cross(d2);
                if den.abs() > 1e-9 * d1.len() * d2.len() {
                    let u = (o2 - o1).cross(d2) / den;
                    let v = (o2 - o1).cross(d1) / den;
                    let pj = &mut segs[j];
                    if pj.rev {
                        pj.t0 = u;
                    } else {
                        pj.t1 = u;
                    }
                    let si = &mut segs[i];
                    if si.rev {
                        si.t1 = v;
                    } else {
                        si.t0 = v;
                    }
                    continue;
                }
            }
            let s = &mut segs[i];
            let old = s.t_start();
            let mut tp = s.curve.closest_param(end);
            if matches!(s.curve, Curve2::Circle { .. }) {
                tp += std::f64::consts::TAU * ((old - tp) / std::f64::consts::TAU).round();
            }
            if s.curve.eval(tp).dist(end) < 1e-9 && (tp - old).abs() < 1e-3 {
                if s.rev {
                    s.t1 = tp;
                } else {
                    s.t0 = tp;
                }
            }
        }
        if segs.iter().any(|s| !(s.t1 > s.t0)) {
            return Err("an edge of the moved outline reverses".into());
        }
        done.push(ProfileLoop::new(segs));
    }
    let outer = done.remove(0);
    let profile = Profile::new(*frame, vec![ProfileRegion { outer, holes: done }]);
    profile.validate().map_err(|e| format!("the outline of the opened face does not close: {e}"))?;
    Ok(profile)
}

fn profile_area(p: &ck_topo::make::Profile) -> f64 {
    p.regions.iter().map(|r| r.outer.signed_area().abs() - r.holes.iter().map(|h| h.signed_area().abs()).sum::<f64>()).sum()
}

/// A convex round along the rim of an opened face that is no larger than the wall (a fillet of the rim, radius <= t): its
/// inward offset collapses, and the cavity has a sharp corner there, the same as for the body with that edge left sharp
/// (the points at least `t` inside are the same). Returns, for such a body, that body with the rim edges sharp again and
/// its opened faces: the body united with a slab of the opened faces' outlines moved out by each round's radius (the walls
/// below the rounds then reach the opened plane), as deep as the largest round. `None` when no rim round is that small.
fn sharpen_small_rims(body: &Body, open: &[FaceId], t: f64, tol: f64) -> Result<Option<(Body, Vec<FaceId>)>, String> {
    use ck_topo::make::{extrude, ExtrudeOpts};
    // (round radius per rim edge)
    let small = |f: FaceId, e: ck_topo::body::EdgeId| -> Option<f64> {
        let fs = body.edge_faces(e);
        let g = *fs.iter().find(|&&g| g != f)?;
        if fs.len() != 2 || open.contains(&g) || body.edge_length(e) < SLIVER {
            return None;
        }
        let face = body.face(g);
        // convex: the outward normal points away from the round's axis / spine (natural normal, sense set)
        let r = match &face.surface {
            Surface::Cylinder { r, .. } if face.sense => *r,
            Surface::Torus { minor, .. } if face.sense => *minor,
            _ => return None,
        };
        let ed = body.edge(e);
        let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let nrm = |h: FaceId| {
            let sf = &body.face(h).surface;
            let (u, v, _) = sf.closest_point(p);
            sf.normal(u, v).try_normalize()
        };
        let tangent = matches!((nrm(f), nrm(g)), (Some(a), Some(b)) if a.cross(b).len() < 0.02);
        (tangent && r <= t * (1.0 + 1e-9)).then_some(r)
    };
    // (opened face, its outline moved out, slab depth)
    let mut outlines = vec![];
    let mut added_bound = 0.0;
    for &f in open {
        let rims: Vec<(ck_topo::body::EdgeId, f64)> = body.face_edges(f).into_iter().filter_map(|e| small(f, e).map(|r| (e, r))).collect();
        let Some(depth) = rims.iter().map(|x| x.1).reduce(f64::max) else { continue };
        let grow = |e: ck_topo::body::EdgeId| rims.iter().find(|x| x.0 == e).map_or(0.0, |x| x.1);
        let profile = face_outline(body, f, &grow)?;
        // (the corner a round of radius r removes has the section (1 - pi/4) r^2: at most that along the moved outline,
        // which is at most twice as long as the rim)
        let perimeter: f64 = rims.iter().map(|&(e, _)| body.edge_length(e)).sum::<f64>() * 2.0 + 1.0;
        added_bound += (1.0 - std::f64::consts::FRAC_PI_4) * depth * depth * perimeter;
        outlines.push((f, profile, depth));
    }
    if outlines.is_empty() {
        return Ok(None);
    }
    let vol = |b: &Body| ck_topo::mass::mass_props(b).volume;
    let vb = vol(body);
    let mut last = String::new();
    // the slab reaches past the rounds into the walls (its sides then lie on them), else just to where the rounds end
    let mut found = None;
    for factor in [1.25, 1.0] {
        let mut sharp = body.clone();
        let mut ok = true;
        for (f, profile, depth) in &outlines {
            let d = factor * depth;
            // into the body: against the outward normal
            let inward = if body.face(*f).sense { ExtrudeOpts { s0: -d, s1: 0.0, dir: None, draft: 0.0 } } else { ExtrudeOpts { s0: 0.0, s1: d, dir: None, draft: 0.0 } };
            match extrude("shellSharp", profile, inward).and_then(|slab| boolean(&sharp, &slab, Op::Union, tol)) {
                Ok(b) => sharp = b.body,
                Err(e) => {
                    last = e;
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            continue;
        }
        ck_topo::edit::merge_coplanar(&mut sharp, 1e-9);
        let added = vol(&sharp) - vb;
        let issues = ck_check::validate(&sharp, 1e-6).len();
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("shell (sharp rim, slab x{factor}): adds {added:.4} mm3 (at most {added_bound:.4}), {} faces, {issues} issues", sharp.num_faces());
        }
        // the slab may only fill the corners the rounds removed (not a pocket next to the rim)
        if !(added > 0.0 && added <= added_bound) || issues > ck_check::validate(body, 1e-6).len() {
            last = format!("the slab adds {added:.3} mm3 with {issues} issues");
            continue;
        }
        found = Some(sharp);
        break;
    }
    let Some(sharp) = found else { return Err(format!("the rim could not be made sharp for the cavity: {last}")) };
    let planes: Vec<(Vec3, f64)> = open.iter().filter_map(|&f| plane_of(body, f)).collect();
    let open_s: Vec<FaceId> = sharp
        .face_ids()
        .filter(|&g| plane_of(&sharp, g).is_some_and(|q| planes.iter().any(|pl| q.0.dot(pl.0) > 1.0 - 1e-9 && (q.1 - pl.1).abs() < 1e-6 * (1.0 + pl.1.abs()))))
        .collect();
    if open_s.len() != open.len() {
        return Err(format!("the sharpened body has {} faces on the opened planes, not {}", open_s.len(), open.len()));
    }
    Ok(Some((sharp, open_s)))
}

/// Shell with an opened face that a neighbour meets tangentially: a rounded rim (a round of the opened face's edges), or a
/// boss cylinder whose top line lies in the opened plane. The offset of the opening moves out and that of the tangent face
/// inward, and the two never meet, so the cavity is built differently: every face is offset inward (the opened one too, which
/// keeps the tangency and so the body's topology), and a prism of the closed cavity's opened face, from its plane to past the
/// body's face, opens the cavity through it. Next to a round the wall then ends in a flat ring of the width of the round. Cut
/// in two steps (the offset cavity, then the prism). A rim round no larger than the wall collapses in the inward offset: the
/// cavity is then that of the body with the rim sharp ([`sharpen_small_rims`]), and the wall ends in the round itself.
///
/// The prism is the true opening only where the cavity's walls at its outline stand square on the opened plane or continue it
/// tangentially: a wall meeting it at another angle (a draft) refuses. The result is checked by volume: body - closed cavity
/// - opened area x t.
fn shell_rounded_rim(body: &Body, open: &[FaceId], t: f64, tol: f64) -> Result<Body, ShellError> {
    use ck_topo::make::{extrude, ExtrudeOpts};
    let fail = |m: String| ShellError::detail("shellFailed", m);
    let sharp = sharpen_small_rims(body, open, t, tol).map_err(fail)?;
    let (src, src_open) = match &sharp {
        Some((b, o)) => (b, o.clone()),
        None => (body, open.to_vec()),
    };
    let cavity = offset_body(src, &|_| -t).map_err(from_offset)?;
    let cut = {
        // face k of the cavity is the offset of face k: those pairs never intersect (only when the cavity is the body's own)
        let _skip = sharp.is_none().then(|| crate::boolean::imprint::skip_pairs(body.face_ids().zip(cavity.face_ids()).filter(|(a, b)| a == b).collect()));
        boolean(body, &cavity, Op::Cut, tol)
    };
    let mut cur = cut.map_err(|e| fail(format!("the cavity could not be cut out of the shape: {e}")))?.body;
    let mut lid = 0.0;
    for &f in &src_open {
        // (the offset keeps the face order: face f of the cavity is the offset of the opened face)
        let face = cavity.face(f);
        let Surface::Plane { .. } = &face.surface else { return Err(fail("the opened face is not planar".into())) };
        let n = plane_of(&cavity, f).unwrap().0;
        let mut square = false;
        for e in cavity.face_edges(f) {
            if cavity.edge_length(e) < SLIVER {
                continue;
            }
            let ed = cavity.edge(e);
            let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
            for g in cavity.edge_faces(e) {
                if g == f {
                    continue;
                }
                let gs = &cavity.face(g).surface;
                let (u, v, _) = gs.closest_point(p);
                let tangent = gs.normal(u, v).try_normalize().is_some_and(|m| m.cross(n).len() < 0.02);
                if ruled_along(gs, n) {
                    square = true;
                } else if !tangent {
                    return Err(fail(format!("{} meets the opened face at an angle (neither square nor tangent), so the opening cannot be cut as a prism", src.face_id_str(g))));
                }
            }
        }
        let profile = face_outline(&cavity, f, &|_| 0.0).map_err(fail)?;
        lid += profile_area(&profile);
        // heights run along the plane frame's z, from the cavity's face (the face's outward side is +z when its sense is
        // set). Below a round the cavity bulges past the outline, so the prism starts a little below the face; along a wall
        // standing square on it the prism starts on the face, flush with the wall
        let (below, above) = (if square { 0.0 } else { 0.25 * t }, t + t.max(1.0));
        let o = if face.sense { ExtrudeOpts { s0: -below, s1: above, dir: None, draft: 0.0 } } else { ExtrudeOpts { s0: -above, s1: below, dir: None, draft: 0.0 } };
        let prism = extrude("shellRim", &profile, o).map_err(|e| fail(format!("the opening could not be built: {e}")))?;
        cur = boolean(&cur, &prism, Op::Cut, tol).map_err(|e| fail(format!("the opening could not be cut: {e}")))?.body;
    }
    cur.number_split_faces();
    if !ck_check::validate(&cur, 1e-6).is_empty() && ck_check::validate(&cur, 1e-6).len() > ck_check::validate(body, 1e-6).len() {
        return Err(fail("the hollowed body is not valid".into()));
    }
    let vol = |b: &Body| ck_topo::mass::mass_props(b).volume;
    let (vb, vc, vo) = (vol(body), vol(&cavity), vol(&cur));
    let want = vb - vc - lid * t;
    if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
        eprintln!("shell (tangent opening): body {vb:.4} cavity {vc:.4} lid {:.4} -> want {want:.4} got {vo:.4}", lid * t);
    }
    if !((vo - want).abs() <= 1e-5 * vb) {
        return Err(fail(format!("the hollowed body has the wrong volume ({vo:.3} mm3 against {want:.3} mm3)")));
    }
    Ok(cur)
}

fn shell_try(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64, cut_factor: f64) -> Result<Body, ShellError> {
    if !(thickness > 0.0) || !thickness.is_finite() {
        return Err(ShellError::code("invalidValue"));
    }
    for &f in open {
        if !matches!(body.face(f).surface, Surface::Plane { .. } | Surface::BSpline(_)) {
            return Err(ShellError::detail("shellUnsupported", format!("opened face {} is not planar", body.face_id_str(f))));
        }
    }
    // A loft ended by a point section has a pole: the surface collapses to the apex, where its
    // offset is not a surface of the same kind (the offset folds over itself and the inner
    // surfaces only close at a new apex). The cavity is therefore cut off by a plane across the
    // loft, `reach_of_pole` short of the apex: the wall stays solid beyond it (the cavity there
    // is a sliver narrower than the wall), and the cavity ends flush on that plane.
    let reach_of_pole = cut_factor * thickness;
    let near_poles = near_pole_cuts(body, thickness, reach_of_pole);
    if outward && !near_poles.is_empty() {
        return Err(ShellError::detail(
            "offsetUnsupported",
            format!(
                "{} runs into the apex of a point-ended loft: an outward shell of an apex is not supported. Cut the tip off further away (more than {reach_of_pole:.1} mm from the apex for this wall) or end the loft with a small closed section instead of a point",
                near_poles[0].face
            ),
        ));
    }
    if let Some((open_face, neighbour)) = rim_tangent_to_open_face(body, open) {
        if !outward && near_poles.is_empty() {
            match shell_rounded_rim(body, open, thickness, tol) {
                Ok(b) => return Ok(b),
                Err(e) => {
                    if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                        eprintln!("shell (tangent opening) failed: {:?}", e.detail);
                    }
                }
            }
        }
        return Err(ShellError::detail(
            "shellUnsupported",
            format!(
                "the opened face {open_face} meets {neighbour} tangentially (a rounded rim, or a curved face touching it along a line) and the wall could not be cut flush on it. Shell first and round the rim afterwards, or leave the edges around the opened face unrounded"
            ),
        ));
    }
    let is_open = |f: FaceId| open.contains(&f);
    let t = thickness;
    // closed-spline walls carry a two-lap profile: cut it to one lap per face / edge first
    let unrolled = crate::offset::single_lap_body(body);
    let body = &unrolled;
    // fitted offset surfaces / edges of free-form faces are accurate to ~1e-5,
    // so the cut welds at a matching tolerance
    let freeform = body.face_ids().any(|f| matches!(body.face(f).surface, Surface::BSpline(_) | Surface::Screw { .. } | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Offset { .. }));
    let tol = if freeform { tol.max(1e-3) } else { tol };
    let dbg_t = std::env::var_os("CK_OFFSET_DEBUG").is_some().then(std::time::Instant::now);
    // How far the cavity overshoots an opened face. The overshoot region lies
    // outside the body, so any positive value gives the same result; a short
    // one keeps the free-form neighbours' continuation past their domain
    // (second-order, so neighbouring fillets and walls drift apart with the
    // square of the distance) inside the fit tolerance for thick walls.
    let ov = (0.25 * t).max(t.min(10.0 * tol));
    // volume of the body beyond the cutting plane(s): the cavity lost to the cut is part of it
    let mut tail_vol = 0.0;
    let (base, cavity, trim_pairs) = if outward {
        let base = offset_body(body, &|f| if is_open(f) { 0.0 } else { t }).map_err(from_offset)?;
        let cav = offset_body(body, &|f| if is_open(f) { ov } else { 0.0 }).map_err(from_offset)?;
        (base, cav, None)
    } else if near_poles.is_empty() {
        let cav = offset_body(body, &|f| if is_open(f) { ov } else { -t }).map_err(from_offset)?;
        (body.clone(), cav, None)
    } else {
        // the open faces and the cut planes, as (outward normal, plane constant)
        let planes: Vec<(Vec3, f64)> = open
            .iter()
            .filter_map(|&f| plane_of(body, f))
            .collect();
        let mut front = body.clone();
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("shell: {} pole cut(s): {:?}", near_poles.len(), near_poles.iter().map(|c| (c.n, c.p)).collect::<Vec<_>>());
        }
        let mut caps: Vec<(Vec3, f64)> = vec![];
        for c in &near_poles {
            let (b, n) = trim_behind(&front, c.p, c.n, tol).map_err(|e| {
                ShellError::detail("shellFailed", format!("the tip of the point-ended loft could not be cut off for the cavity: {e}"))
            })?;
            front = b;
            caps.push((n, c.p.dot(n)));
            if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
                eprintln!("shell: trimmed front has {} faces, {} vertices, bbox {:?}", front.num_faces(), front.vertex_ids().count(), front.bbox());
            }
        }
        tail_vol = ck_topo::mass::mass_props(body).volume - ck_topo::mass::mass_props(&front).volume;
        let find = |pl: &(Vec3, f64)| front.face_ids().find(|&f| plane_of(&front, f).is_some_and(|q| q.0.dot(pl.0) > 1.0 - 1e-9 && (q.1 - pl.1).abs() < 1e-6 * (1.0 + pl.1.abs())));
        let open_f: Vec<FaceId> = planes.iter().filter_map(find).collect();
        let cap_f: Vec<FaceId> = caps.iter().filter_map(find).collect();
        let cav = offset_body(&front, &|f| if open_f.contains(&f) { ov } else if cap_f.contains(&f) { 0.0 } else { -t }).map_err(from_offset)?;
        // faces that survive the trim keep their names: they pair with the cavity face of the same name
        let pairs: Vec<(FaceId, FaceId)> = body
            .face_ids()
            .filter_map(|a| {
                let m: Vec<FaceId> = cav.face_ids().filter(|&b| cav.face(b).name == body.face(a).name && !cap_f.contains(&b)).collect();
                (m.len() == 1).then(|| (a, m[0]))
            })
            .collect();
        (body.clone(), cav, Some(pairs))
    };
    if let Some(t) = dbg_t {
        eprintln!("shell: offset {:.1}s", t.elapsed().as_secs_f64());
    }
    let dbg_t = std::env::var_os("CK_OFFSET_DEBUG").is_some().then(std::time::Instant::now);
    // the cavity keeps the body's face order: face k of the cavity is the
    // offset of face k, parallel to it, so those pairs never intersect
    let _skip = crate::boolean::imprint::skip_pairs(match trim_pairs {
        Some(p) => p,
        None => base.face_ids().zip(cavity.face_ids()).filter(|(a, b)| a == b).collect(),
    });
    let coplanar_failed = || {
        coplanar_open_pair(body, open).map(|(f, g)| {
            ShellError::detail(
            "shellUnsupported",
            format!(
                "the opened faces {} and {} lie on one plane and share an edge (the two end caps of a 180 degree revolve): a shell opened through such a split face is not supported; revolve a full turn and cut the body in half (so the opening is one face), then shell",
                body.face_id_str(f),
                body.face_id_str(g)
            ),
            )
        })
    };
    let out = boolean(&base, &cavity, Op::Cut, tol).map_err(|e| {
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("shell: cavity cut failed: {e}");
        }
        coplanar_failed().unwrap_or_else(|| diagnose_cut_failure(body, &is_open, t, outward, Some(&e)))
    })?;
    if let Some(t) = dbg_t {
        eprintln!("shell: cavity cut {:.1}s", t.elapsed().as_secs_f64());
    }
    if out.no_effect || out.body.num_faces() == 0 {
        return Err(coplanar_failed().unwrap_or_else(|| diagnose_cut_failure(body, &is_open, t, outward, None)));
    }
    let mut b = out.body;
    b.number_split_faces();
    // Offset surfaces that cross or touch inside the cavity leave a wall that is not `t` thick (or a
    // cavity pinched shut) in a body that still passes the topology checks: refuse it.
    if !outward {
        let crossing = ck_check::loop_issues(&b, 1e-6).into_iter().find(|i| matches!(i.code, "LOOP_CROSS" | "LOOP_SELF_INTERSECT"));
        let touching = if crossing.is_none() { crate::shell_check::pinched_wall(&b, 2e-4) } else { None };
        if crossing.is_some() || touching.is_some() {
            let at = crossing.map(|i| i.entity).or(touching).unwrap_or_default();
            return Err(ShellError::detail("shellTooThick", format!("{WALLS_MEET} (at {at})")));
        }
    }
    // A hollowed body that fails validation where the solid passed (an offset edge off its faces, a loop that
    // does not close) is not returned as a result: refused, with what failed and, via `shell`, a wall that builds.
    if near_poles.is_empty() {
        let issues = ck_check::validate(&b, 1e-6);
        if !issues.is_empty() && issues.len() > ck_check::validate(body, 1e-6).len() {
            return Err(ShellError::detail(
                "shellFailed",
                format!("{INVALID_RESULT}: {} {} ({} issues in all)", issues[0].code, issues[0].detail, issues.len()),
            ));
        }
    }
    if !near_poles.is_empty() {
        // a cavity cut off short of an apex has to come out clean, else a plane further away is tried
        let wall = ck_topo::mass::mass_props(&b).volume;
        if tail_vol > 0.05 * wall {
            return Err(ShellError::detail(
                "offsetUnsupported",
                format!(
                    "the apex of the point-ended loft is too blunt to shell: the cavity can only be built up to {cut_factor:.0} wall thicknesses short of it, which would leave {tail_vol:.0} mm3 of solid in place of cavity. Cut the tip off (so that the body stays more than {:.0} mm from the apex) or end the loft with a small closed section instead of a point",
                    6.0 * thickness
                ),
            ));
        }
        let issues = ck_check::validate(&b, 1e-6);
        if !issues.is_empty() {
            return Err(ShellError::detail("shellFailed", format!("the cavity cut off short of the apex left an invalid wall: {} {}", issues[0].code, issues[0].detail)));
        }
    }
    // never hand back a body less valid than the one shelled (a cavity vertex placed off its edge where a hole
    // crosses a curved wall off its axis, solidcore #1528)
    let issues = ck_check::validate(&b, 1e-6);
    if !issues.is_empty() && issues.len() > ck_check::validate(body, 1e-6).len() {
        let i = &issues[0];
        return Err(ShellError::detail(
            "shellFailed",
            format!(
                "the shell came out as an invalid body ({} at {}: {}): this is a limit of the shell operation for this shape, not a wrong input; a hole or cut-out that crosses a curved wall off its axis is the usual cause: shell first and add the hole after, or centre the hole on the curved wall's axis",
                i.code, i.entity, i.detail
            ),
        ));
    }
    Ok(b)
}

/// Two opened planar faces that share an edge and lie on one plane (same outward normal): the start and end caps of a
/// 180 degree revolve. Their offsets are the same plane and the cavity cut is not built for that opening.
fn coplanar_open_pair(b: &Body, open: &[FaceId]) -> Option<(FaceId, FaceId)> {
    for &f in open {
        for e in b.face_edges(f) {
            let fs = b.edge_faces(e);
            let Some(&g) = fs.iter().find(|&&g| g != f) else { continue };
            if fs.len() != 2 || !open.contains(&g) {
                continue;
            }
            if let (Some((nf, cf)), Some((ng, cg))) = (plane_of(b, f), plane_of(b, g)) {
                if nf.dot(ng) > 1.0 - 1e-12 && (cf - cg).abs() < 1e-9 * (1.0 + cf.abs()) {
                    return Some((f.min(g), f.max(g)));
                }
            }
        }
    }
    None
}

/// A plane across a point-ended loft that cuts off its apex: the side the apex is on is dropped.
struct PoleCut {
    face: String,
    /// Unit normal, pointing to the apex.
    n: Vec3,
    /// A point on the plane.
    p: Vec3,
}

/// Outward normal and plane constant of a planar face.
fn plane_of(body: &Body, f: FaceId) -> Option<(Vec3, f64)> {
    match &body.face(f).surface {
        Surface::Plane { frame } => {
            let n = if body.face(f).sense { frame.z } else { -frame.z };
            Some((n, frame.o.dot(n)))
        }
        _ => None,
    }
}

/// The apexes of point-ended lofts that lie within `trigger` of a corner of the body (or inside it),
/// each with the plane that cuts it off `reach` before the apex.
fn near_pole_cuts(body: &Body, t: f64, reach: f64) -> Vec<PoleCut> {
    let trigger = 6.0 * t;
    // (apex, face name, sum of apex - ring centre over the faces meeting there): a cone split into
    // halves has an off-axis centre per half, the sum of both is on the axis
    let mut found: Vec<(Vec3, String, Vec3)> = vec![];
    for f in body.face_ids() {
        let face = body.face(f);
        let Surface::BSpline(_) = &face.surface else { continue };
        let s = &face.surface;
        let ((u0, u1), (v0, v1)) = s.domain();
        for (side, on) in crate::offset::pole_sides(s).into_iter().enumerate() {
            if !on {
                continue;
            }
            // the ring of surface points a fraction `fr` of the domain away from the collapsed side
            let ring = |fr: f64| -> Vec3 {
                let mut c = Vec3::ZERO;
                let n = 24;
                for k in 0..n {
                    let t = k as f64 / n as f64;
                    c = c + match side {
                        0 => s.eval(u0 + fr * (u1 - u0), v0 + t * (v1 - v0)),
                        1 => s.eval(u1 - fr * (u1 - u0), v0 + t * (v1 - v0)),
                        2 => s.eval(u0 + t * (u1 - u0), v0 + fr * (v1 - v0)),
                        _ => s.eval(u0 + t * (u1 - u0), v1 - fr * (v1 - v0)),
                    };
                }
                c * (1.0 / n as f64)
            };
            let apex = match side {
                0 => s.eval(u0, 0.5 * (v0 + v1)),
                1 => s.eval(u1, 0.5 * (v0 + v1)),
                2 => s.eval(0.5 * (u0 + u1), v0),
                _ => s.eval(0.5 * (u0 + u1), v1),
            };
            let nearest = body.vertex_ids().map(|v| body.vertex(v).p.dist(apex)).fold(f64::INFINITY, f64::min);
            if nearest >= trigger {
                continue;
            }
            let Some(centre) = [0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.5, 0.8].into_iter().map(ring).find(|c| c.dist(apex) >= reach) else { continue };
            let dir = (apex - centre).normalize();
            match found.iter_mut().find(|e| e.0.dist(apex) < 1e-6 * (1.0 + apex.len())) {
                Some(e) => e.2 = e.2 + dir,
                None => found.push((apex, body.face_id_str(f), dir)),
            }
        }
    }
    found
        .into_iter()
        .filter_map(|(apex, face, sum)| {
            let n = sum.try_normalize()?;
            Some(PoleCut { face, n, p: apex - n * reach })
        })
        .collect()
}

/// `body` with everything on the apex side of the plane (through `p`, normal `n`) cut away.
fn trim_behind(body: &Body, p: Vec3, n0: Vec3, tol: f64) -> Result<(Body, Vec3), String> {
    // a plane square across the axis of a cone coincides with the iso-lines of its fitted surface
    // and the intersection can come out empty: a slightly tilted plane is the same cut for our purpose
    let a = if n0.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let side = n0.cross(a).normalize();
    let mut last = String::new();
    for deg in [0.0f64, 3.0, -3.0, 7.0] {
        let r = deg.to_radians();
        let n = (n0 * r.cos() + side * r.sin()).normalize();
        match trim_plane(body, p, n, tol) {
            Ok(b) => return Ok((b, n)),
            Err(e) => last = e,
        }
    }
    Err(last)
}

fn trim_plane(body: &Body, p: Vec3, n: Vec3, tol: f64) -> Result<Body, String> {
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let (lo, hi) = body.bbox();
    let half = 2.0 * (hi - lo).len() + 10.0;
    let a = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (a - n * a.dot(n)).normalize();
    let frame = ck_geom::Frame { o: p, x, y: n.cross(x), z: n };
    let c = [ck_geom::Vec2::new(-half, -half), ck_geom::Vec2::new(half, -half), ck_geom::Vec2::new(half, half), ck_geom::Vec2::new(-half, half)];
    let tags = ["t1", "t2", "t3", "t4"];
    let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], c[i], c[(i + 1) % 4])).collect());
    let slab = extrude("shellTrim", &Profile::new(frame, vec![ProfileRegion { outer, holes: vec![] }]), ExtrudeOpts::blind(2.0 * half))?;
    let r = boolean(body, &slab, Op::Cut, tol)?;
    if r.no_effect || r.body.num_faces() == 0 || r.body.num_faces() <= body.num_faces() {
        return Err("the cutting plane does not cross the body".into());
    }
    Ok(r.body)
}
