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
fn diagnose_cut_failure(body: &Body, is_open: &dyn Fn(FaceId) -> bool, t: f64, outward: bool) -> ShellError {
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
                "the cavity could not be cut out of the shape, and not because the wall is too thick for its curvature ({face} has a smallest radius of {r:.2} mm against a wall of {t:.2} mm): this is a limit of the shell operation here, so a different thickness may not help; try opening another face or changing the shape slightly"
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
                    Some(d) if d.starts_with("walls touch") => e.detail = Some(format!("{d}: use at most {max:.2} mm")),
                    _ => {}
                }
            }
            Err(e)
        }
        r => r,
    }
}

/// Detail of the refusal when two walls meet inside the cavity; `shell` appends the thickness that works.
const WALLS_MEET: &str = "walls touch: two parts of the shape are too close for this wall, so the cavity would be pinched shut or its offset surfaces overlap";

fn floor_hundredth(x: f64) -> f64 {
    (x * 100.0).floor() / 100.0
}

/// The largest wall below `thickness` (to a sixteenth of it, a 256th when nothing larger works) that still
/// shells: bisection on the failing thickness. `None` when not even that works.
fn largest_working(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Option<f64> {
    let (mut lo, mut hi) = (0.0f64, thickness);
    // (four halvings; up to eight when nothing worked yet, e.g. a thin web far below the wall asked for)
    for i in 0..8 {
        if i >= 4 && lo > 0.0 {
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

fn shell_once(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Result<Body, ShellError> {
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
    let out = boolean(&base, &cavity, Op::Cut, tol).map_err(|e| {
        if std::env::var_os("CK_OFFSET_DEBUG").is_some() {
            eprintln!("shell: cavity cut failed: {e}");
        }
        diagnose_cut_failure(body, &is_open, t, outward)
    })?;
    if let Some(t) = dbg_t {
        eprintln!("shell: cavity cut {:.1}s", t.elapsed().as_secs_f64());
    }
    if out.no_effect || out.body.num_faces() == 0 {
        return Err(diagnose_cut_failure(body, &is_open, t, outward));
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
    Ok(b)
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
