//! The analytic builder: one planar / cylindrical / conical edge blended by a swept tool (extruded or revolved
//! section) applied by boolean.

use super::BlendError;
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use super::{apply::*, diag::*, ends::*, model::*, rounds::*};

pub(super) fn chamfer_only(spec: Spec) -> bool {
    matches!(spec, Spec::Chamfer(..))
}

/// One straight edge between two planar faces.
pub fn fillet_line_edge(b: &Body, e: EdgeId, faces: [FaceId; 2], r: f64, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    blend_edge_hint(b, e, faces, Spec::Fillet(r, FilletProfile::Circular), feature, k, tol, false, &EndConditions::default()).map(|x| x.0)
}

/// A planar face the circle edge `e` stands on: the plane contains the circle, is normal to its axis, and the
/// face covers the edge's start point in its interior (a wall that only touches a plate along a line: the edge
/// is the foot of the wall, not a free rim). Not one of the edge's own faces.
pub(super) fn foot_plane(b: &Body, e: EdgeId, faces: [FaceId; 2], tol: f64) -> Option<FaceId> {
    let ed = b.edge(e);
    let (c, axis, rad) = ed.curve.circle_info()?;
    let axis = axis.normalize();
    let p0 = ed.curve.eval(ed.t0.min(ed.t1));
    b.face_ids().find(|&g| {
        if faces.contains(&g) {
            return false;
        }
        let Surface::Plane { frame } = &b.face(g).surface else { return false };
        frame.z.cross(axis).len() < 1e-9
            && (p0 - frame.o).dot(frame.z).abs() <= 1e-7 * (1.0 + rad)
            && (c - p0).dot(frame.z).abs() <= 1e-7 * (1.0 + rad)
            && point_in_face(b, g, p0, tol) != Where::Out
    })
}

/// As [`fillet_line_edge`], also returning the touched-face order hint
/// (edge faces, then end faces).
///
/// A circular fillet on the foot of a wall that stands on a plane is the concave fillet between the wall and that
/// plane (tangent to both, adding material), not a round-over of the wall's own base edge: tried first with each of
/// the edge's faces in turn; the ordinary blend of the edge's two faces is the fallback.
pub(super) fn blend_edge_hint(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    spec: Spec,
    feature: &str,
    k: u32,
    tol: f64,
    ring: bool,
    junc: &EndConditions,
) -> Result<(Body, Vec<String>), BlendError> {
    if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) {
        if let Some(plane) = foot_plane(b, e, faces, tol) {
            for cand in [faces[0], faces[1]] {
                if !matches!(b.face(cand).surface, Surface::Cone { .. } | Surface::Cylinder { .. }) {
                    continue;
                }
                match blend_edge_hint_inner(b, e, [cand, plane], spec, feature, k, tol, ring, junc) {
                    Ok(x) => {
                        if super::diag::on() {
                            eprintln!("foot blend on {} and {} ok", b.face_id_str(cand), b.face_id_str(plane));
                        }
                        return Ok(x);
                    }
                    Err(err) => dbg_fail(&format!("foot blend on {} and {} failed: {}", b.face_id_str(cand), b.face_id_str(plane), err.code)),
                }
            }
        }
    }
    blend_edge_hint_inner(b, e, faces, spec, feature, k, tol, ring, junc)
}

pub(super) fn blend_edge_hint_inner(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    spec: Spec,
    feature: &str,
    k: u32,
    tol: f64,
    ring: bool,
    junc: &EndConditions,
) -> Result<(Body, Vec<String>), BlendError> {
    let ed = b.edge(e).clone();
    let r = spec.size();
    let chamfer = matches!(spec, Spec::Chamfer(..));
    // `ext`: chamfer through curved geometry (a cylinder / lofted cone face,
    // a tangent edge, a spline edge that is a circle). The tool then reaches
    // OUT of the part past both legs, so no tool face is coincident with a
    // body face of a different surface type.
    let mut ext = false;
    // angle swept by a circle edge (from its start point, ccw about the axis)
    let mut sweep = 0.0f64;
    // Geometry at the working point P0 (line: edge midpoint; circle: its
    // start vertex, which is also the faces' seam point).
    let (p0, p1, dir, n1, n2, circ): (Vec3, Vec3, Vec3, Vec3, Vec3, Option<(Vec3, Vec3)>) = match ed.curve {
        Curve::Line { .. } => {
            let p0 = b.vertex(ed.v0).p;
            let p1 = b.vertex(ed.v1).p;
            let dir = (p1 - p0) / p0.dist(p1);
            let pm = (p0 + p1) * 0.5;
            for f in faces {
                match &b.face(f).surface {
                    Surface::Plane { .. } => {}
                    Surface::Cylinder { frame, .. } if chamfer && frame.z.cross(dir).len() < 1e-9 => ext = true,
                    // a flank meeting a hub (super::torus_corner): exact section from `wall_section`
                    Surface::Cylinder { .. } if !chamfer && super::torus_corner::analytic_wall_edge(b, e) => {}
                    other => {
                        dbg_fail(&format!("line edge on {} face", other.kind()));
                        return Err(BlendError::code("blendUnsupported"));
                    }
                }
            }
            let nrm = |f: FaceId| match &b.face(f).surface {
                Surface::Plane { .. } => b.planar_normal(f),
                sf => {
                    let (u, v) = sf.closest_param(pm);
                    b.outward_normal(f, u, v).normalize()
                }
            };
            (p0, p1, dir, nrm(faces[0]), nrm(faces[1]), None)
        }
        _ => {
            // circle between coaxial plane / cylinder / cone faces
            let (c, axis, rad, span) = match ed.curve.circle_info() {
                Some((c, axis, rad)) => (c, axis.normalize(), rad, (ed.t1 - ed.t0).abs()),
                None => match chamfer.then(|| fit_circle(&ed.curve, ed.t0.min(ed.t1), ed.t0.max(ed.t1))).flatten() {
                    Some(x) => {
                        ext = true;
                        x
                    }
                    None => {
                        dbg_fail(&format!("non-circle edge {}", ed.curve.kind()));
                        return Err(BlendError::code("blendUnsupported"));
                    }
                },
            };
            sweep = if ring { std::f64::consts::TAU } else { span };
            let t_start = ed.t0.min(ed.t1);
            let p0 = ed.curve.eval(t_start);
            let mut spline_face = false;
            let mut coaxial = |f: FaceId| match &b.face(f).surface {
                Surface::Plane { frame } => frame.z.cross(axis).len() < 1e-9,
                Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => {
                    frame.z.cross(axis).len() < 1e-9 && (frame.o - c).cross(axis).len() < 1e-7 * (1.0 + rad)
                }
                Surface::BSpline(_) if chamfer => {
                    let ok = bspline_coaxial_cone(&b.face(f).surface, c, axis);
                    spline_face |= ok;
                    ok
                }
                _ => false,
            };
            if !coaxial(faces[0]) || !coaxial(faces[1]) || !(rad > 0.0) {
                dbg_fail(&format!("circle edge not coaxial: {} {}", b.face(faces[0]).surface.kind(), b.face(faces[1]).surface.kind()));
                return Err(BlendError::code("blendUnsupported"));
            }
            ext |= spline_face;
            let nrm = |f: FaceId| {
                let (u, v) = b.face(f).surface.closest_param(p0);
                b.outward_normal(f, u, v).normalize()
            };
            let dir = ed.curve.tangent(t_start).normalize();
            (p0, p0, dir, nrm(faces[0]), nrm(faces[1]), Some((c, axis)))
        }
    };
    let len = p0.dist(p1);
    let c12 = n1.dot(n2);
    let tangent = (1.0 - c12).abs() < 1e-9;
    // turning angle of the edge; a CHORD fillet is the circular fillet whose
    // arc has that chord width
    let phi = c12.clamp(-1.0, 1.0).acos();
    let (spec, r) = match spec {
        Spec::Fillet(w, FilletProfile::Chord) if phi > 1e-9 && phi < std::f64::consts::PI - 1e-9 => {
            let rr = w / (2.0 * (0.5 * phi).sin());
            (Spec::Fillet(rr, FilletProfile::Circular), rr)
        }
        _ => (spec, r),
    };
    // `chamfer_legs_on_face`: the first leg belongs to the named face, whichever side of the pick it is on
    let swap_first = CHAMFER_FIRST.with(|c| {
        c.borrow().as_ref().map_or(false, |(name, flip)| {
            let base = |f: FaceId| {
                let mut n = b.face(f).name.clone();
                n.region = None;
                n
            };
            (base(faces[1]) == *name) ^ *flip
        })
    });
    // chamfer legs on the first / second face
    let (ch1, ch2) = match spec {
        Spec::Chamfer(a, ChamferLeg::Dist(b)) => if swap_first { (b, a) } else { (a, b) },
        Spec::Chamfer(a, ChamferLeg::Angle(al) | ChamferLeg::AngleRev(al)) => {
            // triangle corner / first contact / second contact: law of sines with the interior angle
            let psi = std::f64::consts::PI - phi;
            let den = (al + psi).sin();
            if !(al > 0.0) {
                return Err(BlendError::code("invalidValue"));
            }
            if !(den > 1e-9) {
                return Err(BlendError::detail("blendSize", &format!("{}: no blend fits between the faces", FaceName::blend(feature, k).id())));
            }
            let x = a * al.sin() / den;
            if matches!(spec, Spec::Chamfer(_, ChamferLeg::AngleRev(_))) != swap_first {
                (x, a)
            } else {
                (a, x)
            }
        }
        _ => (r, r),
    };
    let r = r.max(ch1).max(ch2);
    // curvature-continuous profile: the contact points lie `g2d` from the edge
    let g2d = match spec {
        Spec::Fillet(_, FilletProfile::Curvature(sh)) => super::g2::tangent_distance(r, sh, phi).ok_or_else(|| BlendError::code("blendUnsupported"))?,
        _ => 0.0,
    };
    if (1.0 + c12).abs() < 1e-9 || (tangent && !(ext && circ.is_none())) {
        dbg_fail("tangent / folded faces");
        return Err(BlendError::code("blendUnsupported")); // tangent / folded faces
    }
    // convex iff the direction into F1 (left of its coedge about the
    // outward normal) points against n2
    let pm = if circ.is_some() { p0 } else { (p0 + p1) * 0.5 };
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]).ok_or_else(|| BlendError::code("missingEdge"))?;
    let tdir = if b.coedge(co).rev { -dir } else { dir };
    let up = |f: FaceId, q: Vec3| -> Vec3 {
        // on-face point nearest `q` (planes are exact already)
        match &b.face(f).surface {
            Surface::Plane { .. } => q,
            sf => sf.closest_point(q).2,
        }
    };
    let (leg1, leg2) = (n1.cross(tdir).normalize(), -n2.cross(tdir).normalize());
    let convex = if tangent {
        // G1 edge: convex iff a curved face falls away from the shared
        // tangent plane on the leg side (the cut then removes a sliver)
        let away = |f: FaceId, n: Vec3, leg: Vec3| {
            let q = pm + leg * r;
            !matches!(b.face(f).surface, Surface::Plane { .. }) && (q - up(f, q)).dot(n) > 1e-12 * (1.0 + r)
        };
        let cv = away(faces[0], n1, leg1) || away(faces[1], n2, leg2);
        if !cv {
            dbg_fail("concave tangent edge");
            return Err(BlendError::code("blendUnsupported"));
        }
        true
    } else {
        n1.cross(tdir).dot(n2) < 0.0
    };
    let s = if convex { -1.0 } else { 1.0 };
    // contact points on the two faces, the ball centre (fillet) and the
    // chamfer legs (in-face directions away from the edge: n x coedge dir)
    let off = if tangent { Vec3::ZERO } else { (n1 + n2) * (s * r / (1.0 + c12)) };
    let (ta, tb) = match spec {
        Spec::Fillet(_, FilletProfile::Curvature(_)) => (up(faces[0], pm + leg1 * g2d), up(faces[1], pm + leg2 * g2d)),
        Spec::Fillet(..) => (pm + off - n1 * (s * r), pm + off - n2 * (s * r)),
        Spec::Chamfer(..) => (up(faces[0], pm + leg1 * ch1), up(faces[1], pm + leg2 * ch2)),
    };
    // a plane meeting a cylinder along the line: the ball centre is at r from the plane and R + r from the axis
    let (off, ta, tb) = if circ.is_none() && !chamfer && faces.iter().any(|&f| matches!(b.face(f).surface, Surface::Cylinder { .. })) {
        match super::torus_corner::wall_section(b, faces, pm, dir, r) {
            Some((c, a, bb)) if (!convex || super::torus_corner::is_convex_wall_line(b, e)) && matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) => (c - pm, a, bb),
            _ => {
                dbg_fail("wall line: no exact section");
                return Err(BlendError::code("blendUnsupported"));
            }
        }
    } else {
        (off, ta, tb)
    };
    // a concave arc that runs into a planar wall which is not radial (its plane does not contain the axis, e.g. the
    // side of a bar a boss joins at an angle): the swept tool runs on past that end and is clipped by the wall's
    // plane, so the blend ends on the wall instead of stopping short of it
    let mut circ_clips: Vec<(Vec3, Vec3)> = vec![];
    let mut circ_oblique = [false; 2];
    if !convex && sweep > 0.0 && sweep < std::f64::consts::TAU - 1e-9 {
        if let Some((_, _)) = circ {
            let (t_lo, t_hi) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
            let (vs, ve) = if ed.t0 <= ed.t1 { (ed.v0, ed.v1) } else { (ed.v1, ed.v0) };
            for (i, (v, out)) in [(vs, -ed.curve.tangent(t_lo).normalize()), (ve, ed.curve.tangent(t_hi).normalize())].into_iter().enumerate() {
                if let Some((pv, n)) = oblique_end_plane(b, v, &faces, out) {
                    circ_clips.push((pv, -n));
                    circ_oblique[i] = true;
                }
            }
        }
    }
    // which end of the edge meets a curved face or an oblique wall (named in the hint of a refusal)
    let cyl_end = |v, od| convex && circ.is_none() && end_cylinder_solid(b, v, od, &faces, (Vec3::ZERO, Vec3::ZERO)).is_some();
    let ends_curved = if circ.is_some() { circ_oblique } else { [cyl_end(ed.v0, -dir), cyl_end(ed.v1, dir)] };
    let curved_end = match ends_curved {
        [true, true] => Some("both"),
        [true, false] => Some("start"),
        [false, true] => Some("end"),
        _ => None,
    };
    // contact (spring) lines must stay on their faces
    let name = FaceName::blend(feature, k);
    // (a contact point that runs along an end face which is perpendicular to the
    // edge up to the data precision -- sketch coordinates carry 4 decimals, so an
    // "exactly radial" end line of a clip is off by 1e-5 -- lies on that face's
    // boundary line within `ptol`, not outside the face)
    let ptol = (10.0 * tol).max(2e-5 * (b.bbox().1 - b.bbox().0).len()).min(0.25 * r);
    // (a section whose start end is clipped by an oblique wall is checked half way along the arc: its contact on the
    // face the wall bounds leaves that face through the wall by design)
    let rot = |p: Vec3| -> Vec3 {
        match circ {
            Some((c, axis)) if circ_oblique[0] => {
                let (v, al) = (p - c, 0.5 * sweep);
                c + v * al.cos() + axis.cross(v) * al.sin() + axis * (axis.dot(v) * (1.0 - al.cos()))
            }
            _ => p,
        }
    };
    let (pm_c, ta_c, tb_c) = (rot(pm), rot(ta), rot(tb));
    // a concave fillet whose contact runs off a narrow face over its convex far edge: trimmed by the next face continued
    let mut overflow: Vec<OverflowClip> = vec![];
    for (f, t) in [(faces[0], ta_c), (faces[1], tb_c)] {
        let on_boundary = || {
            b.face_edges(f).into_iter().any(|e2| {
                let x = b.edge(e2);
                crate::boolean::imprint::near_in_range(&x.curve, (x.t0, x.t1), t, ptol)
            })
        };
        if point_in_face(b, f, t, tol) == Where::Out && !on_boundary() {
            if !convex && !tangent && matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) {
                if let Some(c) = overflow_clip(b, f, e, pm_c, t, b.bbox(), tol) {
                    if super::diag::on() {
                        eprintln!("{}: contact leaves {} past its far edge; trimmed by the next face continued", name.id(), b.face_id_str(f));
                    }
                    overflow.push(c);
                    continue;
                }
            }
            if let Spec::Fillet(_, FilletProfile::Curvature(sh)) = spec {
                // the largest radius whose contact points still lie on their faces
                let fits = |rr: f64| -> bool {
                    let Some(d) = super::g2::tangent_distance(rr, sh, phi) else { return false };
                    [(faces[0], leg1), (faces[1], leg2)].iter().all(|&(f, leg)| point_in_face(b, f, up(f, pm + leg * d), tol) != Where::Out)
                };
                let (mut lo, mut hi) = (0.0, r);
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    if fits(mid) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let mut e = BlendError::detail("blendG2TooLargeMax", &format!("{}: spring ends out of order", name.id()));
                e.params = vec![("max".into(), format!("{lo:.2}")), ("shape".into(), format!("{sh}"))];
                return Err(e);
            }
            dbg_fail(&format!("spring leaves face {} at {:?} (edge {:?} t=({:.4},{:.4}) corev {}, p0 {:?}, dir {:?}, leg1 {:?} leg2 {:?}, circ {})", b.face_id_str(f), t, e, ed.t0, ed.t1, b.coedge(co).rev, p0, dir, leg1, leg2, circ.is_some()));
            // the largest size whose contact points (linear in the size about the section's edge point) still lie on
            // their faces, and where along the edge the failing section is
            let fits = |rr: f64| -> bool {
                [(faces[0], ta_c), (faces[1], tb_c)].iter().all(|&(f, t)| {
                    let q = up(f, pm_c + (t - pm_c) * (rr / r));
                    point_in_face(b, f, q, tol) != Where::Out
                })
            };
            let mut err = BlendError::detail("blendOverflow", &format!("{}: spring leaves its face away from its edges", name.id()));
            if curved_end.is_some() && r > 0.0 && fits(r * 1e-3) {
                let (mut lo, mut hi) = (r * 1e-3, r);
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    if fits(mid) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                err.params.push(("max".into(), format!("{:.4}", lo * 0.999)));
            }
            if let Some(end) = curved_end {
                err.params.push(("curvedEnd".into(), end.into()));
            }
            return Err(err);
        }
    }
    // tool profile in the plane perpendicular to dir through p0 - dir*margin
    // (line) or in the meridian plane through P0 (circle: revolved about the
    // axis). convex: the tool pierces the end faces (the cut trims it);
    // concave: GUESS end faces perpendicular to the edge, the tool ends
    // flush with them
    let margin = if convex && circ.is_none() { r.max(len * 0.05) + 1.0 } else { 0.0 };
    // concave: an end at a junction vertex runs on past the shortened edge end
    // to just inside the wall behind the original vertex
    let (mut m0, mut m1) = (margin, margin);
    if convex && circ.is_none() && !chamfer {
        // the tool pierces an end only if the body ends there; an end face that
        // leans toward the material (the edge runs into a reflex corner, e.g. a
        // pocket rim edge into the pocket wall) bounds the tool flush
        let flush_end = |v: ck_topo::body::VertexId, out: Vec3| -> bool {
            let mut ends: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !faces.contains(&f) && !ends.contains(&f) {
                        ends.push(f);
                    }
                }
            }
            ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(out) < -1e-9
        };
        let near = |q: Vec3, p: Vec3, o: Vec3| (q - p).cross(dir).len() <= 1e-6 && q.dist(p) < q.dist(o);
        // the edge goes on past the vertex as a smooth (G1) edge of the same
        // line: nothing there to round, so the tool stops at the vertex
        // (e.g. a slot cut through a rounded end leaves the rim edge between
        // the slot walls, continued by the tangent rim of the round)
        let smooth_run_on = |v: ck_topo::body::VertexId, out: Vec3| -> bool {
            let pv = b.vertex(v).p;
            b.edge_ids().filter(|&x| x != e && (b.edge(x).v0 == v || b.edge(x).v1 == v)).any(|x| {
                let ex = b.edge(x);
                let away = if ex.v0 == v { ex.curve.tangent(ex.t0) } else { -ex.curve.tangent(ex.t1) };
                if away.len() < 1e-12 || away.normalize().cross(out).len() > 1e-6 || away.normalize().dot(out) < 0.5 {
                    return false;
                }
                let fs = b.edge_faces(x);
                if fs.len() != 2 {
                    return false;
                }
                let nrm = |f: FaceId| {
                    let (u, w) = b.face(f).surface.closest_param(pv);
                    b.outward_normal(f, u, w).normalize()
                };
                (1.0 - nrm(fs[0]).dot(nrm(fs[1]))).abs() < 1e-9
            })
        };
        if flush_end(ed.v0, -dir) || smooth_run_on(ed.v0, -dir) || junc.flush.iter().any(|&q| near(q, p0, p1)) {
            m0 = 0.0;
        }
        if flush_end(ed.v1, dir) || smooth_run_on(ed.v1, dir) || junc.flush.iter().any(|&q| near(q, p1, p0)) {
            m1 = 0.0;
        }
    } else if convex && circ.is_none() && !junc.flush.is_empty() {
        // chamfer: only the ends where a tangent chain goes on stop flush
        let near = |q: Vec3, p: Vec3, o: Vec3| (q - p).cross(dir).len() <= 1e-6 && q.dist(p) < q.dist(o);
        if junc.flush.iter().any(|&q| near(q, p0, p1)) {
            m0 = 0.0;
        }
        if junc.flush.iter().any(|&q| near(q, p1, p0)) {
            m1 = 0.0;
        }
    }
    // an end that runs into the blend faces of an earlier feature (the convex rim of a tower after its wall coves were rounded):
    // the tool must not reach past the plane of the wall those blends are tangent to
    if convex && circ.is_none() {
        for (v, od, which) in [(ed.v0, -dir, 0usize), (ed.v1, dir, 1usize)] {
            let cur = if which == 0 { m0 } else { m1 };
            if cur == 0.0 {
                continue;
            }
            let pv = b.vertex(v).p;
            let mut end_faces: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !faces.contains(&f) && !end_faces.contains(&f) {
                        end_faces.push(f);
                    }
                }
            }
            if end_faces.is_empty() || !end_faces.iter().all(|&f| matches!(b.face(f).name.role, Role::Blend(_))) {
                continue;
            }
            let mut best: Option<f64> = None;
            for &f in &end_faces {
                for g in b.face_neighbors(f) {
                    if let Surface::Plane { frame } = &b.face(g).surface {
                        let n = b.planar_normal(g);
                        if n.cross(od).len() < 1e-6 {
                            let t = (frame.o - pv).dot(od);
                            if t > -1e-9 && t < cur && best.map_or(true, |x| t < x) {
                                best = Some(t.max(0.0));
                            }
                        }
                    }
                }
            }
            if let Some(t) = best {
                if which == 0 { m0 = t } else { m1 = t }
            }
        }
    }
    // a flush end vertex of the selection that now lies inside the edge: an earlier tool of this feature lengthened the
    // edge past it (at a reflex rim corner the hub tool's flush end face lies in the flank's plane and merges into it,
    // so the flank's rim edge runs on into the hub). The tool still ends at that vertex, not at the edge's new end:
    // each tool is the same solid whatever order the edges are applied in.
    if convex && circ.is_none() && !chamfer {
        for &q in &junc.flush {
            if (q - p0).cross(dir).len() > 1e-6 {
                continue;
            }
            let t = (q - p0).dot(dir);
            if t > 1e-6 && t < 0.5 * len {
                m0 = -t;
            } else if t < len - 1e-6 && t > 0.5 * len {
                m1 = -(len - t);
            } else {
                continue;
            }
            if super::diag::on() {
                eprintln!("{}: tool ends at the flush vertex {q:?}, {t:.6} along an edge of length {len:.6}", FaceName::blend(feature, k).id());
            }
        }
    }
    // reflex corner shared with another selected edge: run on past the vertex, clip to the bisector plane
    let mut clips: Vec<(Vec3, Vec3)> = vec![];
    if circ.is_none() {
        let mm = if convex { margin } else { r.max(len * 0.05) + 1.0 };
        for &(q, da, db) in &junc.mitre {
            let n = da - db;
            if n.len() < 1e-9 {
                continue;
            }
            if q.dist(p0) < 1e-6 && (da - dir).len() < 1e-6 {
                m0 = mm;
                clips.push((q, n.normalize()));
            } else if q.dist(p1) < 1e-6 && (da + dir).len() < 1e-6 {
                m1 = mm;
                clips.push((q, n.normalize()));
            }
        }
    }
    if !convex && circ.is_none() && !chamfer {
        // per end, the junction vertex it was shortened from: the one on the edge's line nearest that end (beyond it,
        // or already on the edge when an earlier tool lengthened it). Another junction further along the same line,
        // the corner of a second boss in line with this one, is no end of this edge: running on to it would bridge
        // the gap between the bosses
        let (mut e0, mut e1): (Option<(f64, bool)>, Option<(f64, bool)>) = (None, None);
        for &(q, behind) in &junc.ext {
            if (q - p0).cross(dir).len() > 1e-6 {
                continue;
            }
            let (t0, t1) = ((p0 - q).dot(dir), (q - p1).dot(dir));
            if t0.abs() < t1.abs() {
                if e0.map_or(true, |x| t0.abs() < x.0.abs()) {
                    e0 = Some((t0, behind));
                }
            } else if e1.map_or(true, |x| t1.abs() < x.0.abs()) {
                e1 = Some((t1, behind));
            }
        }
        let (e0, e1) = (e0.filter(|x| x.0 > 1e-6), e1.filter(|x| x.0 > 1e-6));
        let more = |behind: bool| if behind { 0.25 * r } else { 0.0 };
        if let Some((t, behind)) = e0 {
            m0 = m0.max(t + more(behind));
        }
        if let Some((t, behind)) = e1 {
            m1 = m1.max(t + more(behind));
        }
    }
    // a concave straight edge that runs into an oblique planar end face (a rib's free face slanting across the corner
    // the rib stands in): the tool runs on past that end and is clipped by the end face's plane, so the blend ends on
    // it (as for a concave arc above) instead of on a plane square to the edge
    if !convex && circ.is_none() && !chamfer {
        for (v, od, which) in [(ed.v0, -dir, 0usize), (ed.v1, dir, 1usize)] {
            if (if which == 0 { m0 } else { m1 }) > 0.0 {
                continue;
            }
            if let Some((pv, ns)) = oblique_end_planes(b, v, &faces, od) {
                let mm = r.max(len * 0.05) + 1.0;
                if which == 0 {
                    m0 = mm;
                } else {
                    m1 = mm;
                }
                clips.extend(ns.into_iter().map(|n| (pv, -n)));
            }
        }
    }
    let o = p0 - dir * m0;
    let fr = match circ {
        None => Frame::from_normal(o, dir),
        Some((c, axis)) => {
            let rad = (p0 - c).normalize();
            ck_geom::frame_from(c, rad.cross(axis), rad)
        }
    };
    let uv = |p: Vec3| {
        let q = fr.world_to_uv(p);
        Vec2::new(q.x, q.y)
    };
    let (cp, t1, t2) = (uv(p0), uv(ta), uv(tb));
    let cc = uv(p0 + off);
    let lp = if ext {
        // chord t1-t2 plus an outer rim a margin beyond both faces
        let k = r;
        let dn = |n: Vec3| uv(p0 + n) - cp;
        let (d1, d2) = (dn(n1), dn(n2));
        let mut pts = vec![t1, t2, t2 + d2 * k];
        if !tangent {
            pts.push(cp + (d1 + d2).normalize() * (k * 1.5));
        }
        pts.push(t1 + d1 * k);
        let tags = ["arc", "o1", "o2", "o3", "o4"];
        let n = pts.len();
        ProfileLoop::new((0..n).map(|i| ProfileSeg::line(tags[i], pts[i], pts[(i + 1) % n])).collect())
    } else if let Some(seg) = match spec {
        Spec::Fillet(_, FilletProfile::Conic(rho)) => super::g2::conic_segment(rho, t1, cp, t2),
        Spec::Fillet(_, FilletProfile::Curvature(sh)) => super::g2::g2_segment(r, sh, phi, t1, cp, t2),
        _ => None,
    } {
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), seg])
    } else if matches!(spec, Spec::Chamfer(..)) {
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), ProfileSeg::line("arc", t1, t2)])
    } else {
        let short_ccw = (t1 - cc).cross(t2 - cc) > 0.0;
        let arc = if short_ccw {
            ProfileSeg::arc("arc", cc, t1, t2)
        } else if !convex {
            // concave: the tool's cap arcs ARE the end arcs; the oracle runs
            // them from the forward face's spring to the other one, n = (s1 -
            // c) x (s2 - c) (bracket F12 on S.e8|S.e9 along +x: n = -x)
            let x = (t1 - cc).normalize();
            let (a, b) = (t1 - cc, t2 - cc);
            let sweep = -(a.cross(b)).atan2(a.dot(b));
            ProfileSeg { tag: "arc".into(), curve: ck_geom::curve2::Curve2::Circle { c: cc, x, r: cc.dist(t1), ccw: false }, t0: 0.0, t1: sweep, rev: false }
        } else {
            ProfileSeg::arc("arc", cc, t2, t1).reversed()
        };
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), arc])
    };
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]).normalized();
    let made = match circ {
        None => extrude(feature, &prof, ExtrudeOpts { s0: 0.0, s1: len + m0 + m1, dir: None, draft: 0.0 }),
        Some((c, axis)) => {
            // a partial arc sweeps only its own angle (convex: a margin more at
            // both ends so the tool pierces the adjacent faces; concave:
            // flush, like the straight edge); a full circle is one turn
            use std::f64::consts::TAU;
            let span = sweep;
            let rad = (p0 - c).len();
            let (a0, a1) = if span >= TAU - 1e-9 {
                (0.0, TAU)
            } else {
                let m = if convex { (r.max(span * rad * 0.05) + 1.0) / rad } else { 0.0 };
                let mo = (r.max(span * rad * 0.05) + 1.0) / rad;
                // an end that runs on into another arc of the same circle (a
                // split ring) stops flush: that arc is cut by its own tool
                let ends = [p0, ed.curve.eval(ed.t0.max(ed.t1))];
                let flush = |q: Vec3| (ext && (continues_circle(b, e, q, c, axis, rad) || meets_own_blend(b, e, q, feature, tol))) || junc.flush.iter().any(|f| f.dist(q) < 1e-6 * (1.0 + rad));
                let (mut m0, mut m1) = (
                    if circ_oblique[0] { mo } else if flush(ends[0]) { 0.0 } else { m },
                    if circ_oblique[1] { mo } else if flush(ends[1]) { 0.0 } else { m },
                );
                // a convex arc that runs into a reflex corner (a hub arc's rim into the wall of a petal whose rim is
                // not rounded): the only other face there is a plane that leans toward the material, so the tool must
                // not run on into the wall. It ends on that plane: flush when the plane is radial (it contains the
                // axis), else run on and clipped by it (while the clip plane cannot reach the tool's other end).
                if convex && !chamfer && span + m0 + m1 < TAU {
                    let (t_lo, t_hi) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
                    let (vs, ve) = if ed.t0 <= ed.t1 { (ed.v0, ed.v1) } else { (ed.v1, ed.v0) };
                    let outs = [(vs, -ed.curve.tangent(t_lo).normalize()), (ve, ed.curve.tangent(t_hi).normalize())];
                    for (i, (v, out)) in outs.into_iter().enumerate() {
                        let mi = if i == 0 { m0 } else { m1 };
                        if mi == 0.0 || circ_oblique[i] {
                            continue;
                        }
                        let Some(n) = reflex_end_plane(b, v, &faces, out) else { continue };
                        let pv = b.vertex(v).p;
                        let radial = n.dot(axis).abs() < 1e-9 && (pv - c).dot(n).abs() < 1e-7 * (1.0 + rad);
                        if radial || span + m0 + m1 >= std::f64::consts::PI {
                            if i == 0 { m0 = 0.0 } else { m1 = 0.0 }
                        } else {
                            circ_clips.push((pv, n));
                        }
                    }
                }
                if span + m0 + m1 >= TAU { (0.0, TAU) } else { (-m0, span + m1) }
            };
            ck_topo::make::revolve::revolve(feature, &prof, c, axis, a0, a1)
        }
    };
    let mut tool = made.map_err(|e| {
        dbg_fail(&format!("tool: {e}"));
        BlendError::code("blendFailed")
    })?;
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
    for (o, n) in clips {
        tool = clip_halfspace(&tool, o, n, tol)?;
    }
    for (o, n) in circ_clips {
        tool = clip_halfspace(&tool, o, n, tol)?;
    }
    for c in &overflow {
        tool = c.apply(&tool, tol)?;
    }
    // an overhang corner's vertex: the tool must not run on past it (a convex tool's margin
    // would cut material on the far side of the vertex outside the zone, where the corner
    // rebuild does not repair it); the zone swap trims the stripe to distance r afterwards
    if circ.is_none() {
        for &qv in &junc.patch {
            if p0.dist(qv) <= 1e-6 && m0 > 0.0 {
                tool = clip_halfspace(&tool, qv, dir, tol)?;
            }
            if p1.dist(qv) <= 1e-6 && m1 > 0.0 {
                tool = clip_halfspace(&tool, qv, -dir, tol)?;
            }
        }
    }
    // an end that runs into a convex cylinder (a joined boss): the tool ends on its surface
    if convex && circ.is_none() {
        for (v, od, m) in [(ed.v0, -dir, m0), (ed.v1, dir, m1)] {
            if m > 0.0 {
                if let Some(cyl) = end_cylinder_solid(b, v, od, &faces, tool.bbox()) {
                    tool = crate::boolean::boolean(&tool, &cyl, Op::Cut, tol).map(|r| r.body).map_err(|_| BlendError::code("blendFailed"))?;
                }
            }
        }
    }
    let op = if convex { Op::Cut } else { Op::Union };
    // order: untouched; edge faces (ref order), end faces (start, end);
    // other touched faces; blend
    let ends: Vec<FaceId> = [ed.v0, ed.v1]
        .iter()
        .flat_map(|&v| {
            b.edge_ids()
                .filter(move |&x| b.edge(x).v0 == v || b.edge(x).v1 == v)
                .flat_map(|x| b.edge_faces(x))
                .filter(|f| !faces.contains(f))
                .collect::<Vec<_>>()
        })
        .collect();
    let mut hint: Vec<String> = faces.iter().map(|&f| b.face_id_str(f)).collect();
    for f in ends {
        let id = b.face_id_str(f);
        if !hint.contains(&id) {
            hint.push(id);
        }
    }
    if matches!(spec, Spec::Fillet(_, FilletProfile::Conic(_) | FilletProfile::Curvature(_))) {
        // the spline section touches the two faces tangentially along its end rulings (their
        // contact lines are the edges it shares with the tool's flat legs): no marching
        let blend_f = tool.face_ids().find(|&f| tool.face(f).name == name);
        let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> { (Some(g) == blend_f && faces.contains(&f)).then(Vec::new) };
        return apply_tool_hooked(b, &tool, op, &name, hint, tol, Some(&hook));
    }
    apply_tool(b, &tool, op, &name, hint, tol)
}

/// Circle through a curve's samples on `[t0, t1]`: `(centre, axis, radius,
/// swept angle)`, the axis oriented so increasing `t` runs counter-clockwise.
/// `None` unless every sample lies on that circle (1e-6 relative).
pub(super) fn fit_circle(c: &Curve, t0: f64, t1: f64) -> Option<(Vec3, Vec3, f64, f64)> {
    const N: usize = 48;
    let p = |i: usize| c.eval(t0 + (t1 - t0) * i as f64 / N as f64);
    let (a, m, d) = (p(0), p(N / 3), p(2 * N / 3));
    let (u, v) = (m - a, d - a);
    let w = u.cross(v);
    let w2 = w.dot(w);
    if w2 < 1e-24 {
        return None;
    }
    let ctr = a + (v.cross(w) * u.dot(u) + w.cross(u) * v.dot(v)) / (2.0 * w2);
    let axis = w.normalize();
    let rad = a.dist(ctr);
    if !(rad > 0.0) {
        return None;
    }
    let (mut ang, mut prev) = (0.0, a - ctr);
    for i in 1..=N {
        let q = p(i) - ctr;
        if (q.len() - rad).abs() > 1e-6 * rad || q.dot(axis).abs() > 1e-6 * rad {
            return None;
        }
        ang += prev.cross(q).dot(axis).atan2(prev.dot(q));
        prev = q;
    }
    (ang > 1e-6).then_some((ctr, axis, rad, ang.min(std::f64::consts::TAU)))
}

/// Is this B-spline face a cone / cylinder about the axis `(c, axis)`
/// (radius linear in height over a sample grid, 1e-6 relative)? A lofted
/// side between two coaxial circles is one.
pub(super) fn bspline_coaxial_cone(sf: &Surface, c: Vec3, axis: Vec3) -> bool {
    let ((u0, u1), (v0, v1)) = sf.domain();
    if ![u0, u1, v0, v1].iter().all(|x| x.is_finite()) {
        return false;
    }
    let mut hr: Vec<(f64, f64)> = vec![];
    for i in 0..=8 {
        for j in 0..=8 {
            let q = sf.eval(u0 + (u1 - u0) * i as f64 / 8.0, v0 + (v1 - v0) * j as f64 / 8.0) - c;
            let h = q.dot(axis);
            hr.push((h, (q - axis * h).len()));
        }
    }
    let (lo, hi) = hr.iter().fold((f64::MAX, f64::MIN), |(l, h), x| (l.min(x.0), h.max(x.0)));
    let scale = hr.iter().map(|x| x.1.max(x.0.abs())).fold(0.0, f64::max);
    if !(hi - lo > 1e-6 * scale) {
        return false;
    }
    let a = hr.iter().find(|x| x.0 == lo).unwrap();
    let z = hr.iter().find(|x| x.0 == hi).unwrap();
    let slope = (z.1 - a.1) / (z.0 - a.0);
    hr.iter().all(|x| (a.1 + slope * (x.0 - a.0) - x.1).abs() < 1e-6 * scale)
}

/// Do the two faces of `e` meet tangentially (G1) along it?
pub(super) fn is_tangent_edge(b: &Body, e: EdgeId) -> bool {
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return false;
    }
    let ed = b.edge(e);
    let pm = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
    let n = |f: FaceId| {
        let (u, v) = b.face(f).surface.closest_param(pm);
        b.outward_normal(f, u, v).normalize()
    };
    (1.0 - n(fs[0]).dot(n(fs[1]))).abs() < 1e-9
}

/// Does another edge of `b` leave the point `q` on the same circle
/// `(c, axis, rad)` as edge `e` (a ring split in several edges)?
pub(super) fn continues_circle(b: &Body, e: EdgeId, q: Vec3, c: Vec3, axis: Vec3, rad: f64) -> bool {
    b.edge_ids().filter(|&x| x != e).any(|x| {
        let ed = b.edge(x);
        let (t0, t1) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
        if ed.curve.eval(t0).dist(q) > 1e-7 * (1.0 + rad) && ed.curve.eval(t1).dist(q) > 1e-7 * (1.0 + rad) {
            return false;
        }
        let fit = ed.curve.circle_info().map(|(c2, a2, r2)| (c2, a2.normalize(), r2)).or_else(|| fit_circle(&ed.curve, t0, t1).map(|x| (x.0, x.1, x.2)));
        fit.map_or(false, |(c2, a2, r2)| (c2 - c).len() < 1e-7 * (1.0 + rad) && a2.cross(axis).len() < 1e-7 && (r2 - rad).abs() < 1e-7 * (1.0 + rad))
    })
}

/// Does another edge of `b` leave `q` along a face blended by `feature`
/// earlier in this feature (the ring's previous arc, already chamfered: its blend
/// face or the flush end wall the tool left)?
pub(super) fn meets_own_blend(b: &Body, e: EdgeId, q: Vec3, feature: &str, tol: f64) -> bool {
    b.edge_ids().filter(|&x| x != e).any(|x| {
        let ed = b.edge(x);
        (b.vertex(ed.v0).p.dist(q) <= tol.max(1e-7) * 10.0 || b.vertex(ed.v1).p.dist(q) <= tol.max(1e-7) * 10.0)
            && b.edge_faces(x).iter().any(|&f| {
                let n = &b.face(f).name;
                n.feature == feature && matches!(n.role, Role::Blend(_) | Role::Cap(_))
            })
    })
}
