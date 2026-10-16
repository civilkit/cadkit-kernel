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
                "the cavity could not be cut; curvature is not the limit ({face} has a smallest radius of {r:.2} mm against a wall of {t:.2} mm): try a slightly different thickness or open another face"
            )),
            limit: Some(OffsetLimit { kind: "curvature", min_radius: r, max_thickness: 0.0, face: Some(face) }),
        },
        None => ShellError::code("shellFailed"),
    }
}

/// Hollow `body` with wall `thickness`, removing the `open` faces.
pub fn shell(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Result<Body, ShellError> {
    if !(thickness > 0.0) || !thickness.is_finite() {
        return Err(ShellError::code("invalidValue"));
    }
    for &f in open {
        if !matches!(body.face(f).surface, Surface::Plane { .. }) {
            return Err(ShellError::detail("shellUnsupported", format!("opened face {} is not planar", body.face_id_str(f))));
        }
    }
    // A loft ended by a point section has a pole: the surface collapses to the apex, where its
    // offset is not a surface of the same kind (the offset folds over itself and the inner
    // surfaces only close at a new apex). That corner is not solved yet: say so, with the way out.
    for f in body.face_ids() {
        let face = body.face(f);
        if matches!(face.surface, Surface::BSpline(_)) && !crate::offset::pole_points(&face.surface).is_empty() {
            return Err(ShellError::detail(
                "offsetUnsupported",
                format!(
                    "{} comes to a point (a loft ended by a point section): the offset of an apex is not supported, so this body cannot be shelled. End the loft with a small closed section instead of a point (a blunt end shells and thickens normally)",
                    body.face_id_str(f)
                ),
            ));
        }
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
    let (base, cavity) = if outward {
        let base = offset_body(body, &|f| if is_open(f) { 0.0 } else { t }).map_err(from_offset)?;
        let cav = offset_body(body, &|f| if is_open(f) { ov } else { 0.0 }).map_err(from_offset)?;
        (base, cav)
    } else {
        let cav = offset_body(body, &|f| if is_open(f) { ov } else { -t }).map_err(from_offset)?;
        (body.clone(), cav)
    };
    if let Some(t) = dbg_t {
        eprintln!("shell: offset {:.1}s", t.elapsed().as_secs_f64());
    }
    let dbg_t = std::env::var_os("CK_OFFSET_DEBUG").is_some().then(std::time::Instant::now);
    // the cavity keeps the body's face order: face k of the cavity is the
    // offset of face k, parallel to it, so those pairs never intersect
    let _skip = crate::boolean::imprint::skip_pairs(base.face_ids().zip(cavity.face_ids()).filter(|(a, b)| a == b).collect());
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
    Ok(b)
}
