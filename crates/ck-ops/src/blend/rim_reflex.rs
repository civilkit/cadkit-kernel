//! Rim fillets at a reflex corner of a curved outline: a straight rim edge (top plane / radial flank plane) and a rim
//! arc (top plane / hub cylinder) that meet where the flank leaves the hub, the concave wall edge between them left
//! sharp (the rim-only selection of a petal ring).
//!
//! The two rim fillets run on past the corner until they meet: in the corner square (at most `r` from both walls)
//! the result is the higher of the two fillet surfaces, so the two blend faces meet along their intersection curve,
//! which runs from the corner of the two top contact lines down to the wall edge at `H - r`. No corner face (this is
//! the reference engine's result; `docs/blend-reference-petal.md`).
//!
//! Built with the swept tools: both rim tools stop flush at the corner vertex (the flank tool on the plane normal to
//! the flank, the hub tool on the radial plane, which is the flank's own plane), and a wedge tool then removes what is
//! above both fillet surfaces in the corner: the flank tool run on past the corner, intersected with the hub tool run
//! on past it (its section widened beyond the hub wall so that the wedge reaches the flank tool's flush end), clipped
//! to the hub side of the flank tool's end plane. Every face of the wedge that is not a fillet surface lies on a face
//! the earlier tools left (top plane, the two flush end planes), so the cut only adds the intersection curve.

use super::build::{apply_tool_hooked, clip_halfspace};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// One reflex rim corner of a selection.
pub(super) struct Reflex {
    /// the corner vertex
    pub(super) v: Vec3,
    /// selection indices of the straight rim edge and of the rim arc
    pub(super) line: usize,
    pub(super) arc: usize,
    /// unit direction along the straight edge away from the corner
    da: Vec3,
    /// outward normals of the top plane and of the flank plane
    nt: Vec3,
    na: Vec3,
    /// the arc's circle: centre and unit axis, and the hub radius
    c: Vec3,
    axis: Vec3,
    rh: f64,
}

fn normal_at(b: &Body, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = b.face(f).surface.closest_param(p);
    b.outward_normal(f, u, v).normalize()
}

/// The reflex rim corners of the selection `sel` (see the module doc).
pub(super) fn corners(b: &Body, sel: &[EdgeId]) -> Vec<Reflex> {
    let mut out = vec![];
    for (i, &a) in sel.iter().enumerate() {
        let ea = b.edge(a);
        if !matches!(ea.curve, Curve::Line { .. }) || super::torus_corner::edge_concave(b, a) != Some(false) {
            continue;
        }
        for (j, &c) in sel.iter().enumerate() {
            if i == j || super::torus_corner::edge_concave(b, c) != Some(false) {
                continue;
            }
            let ec = b.edge(c);
            let Some((cc, axis, rad)) = ec.curve.circle_info() else { continue };
            let axis = axis.normalize();
            for v in [ea.v0, ea.v1] {
                if !(ec.v0 == v || ec.v1 == v) {
                    continue;
                }
                let (fa, fc) = (b.edge_faces(a), b.edge_faces(c));
                let common: Vec<FaceId> = fa.iter().copied().filter(|f| fc.contains(f)).collect();
                if fa.len() != 2 || fc.len() != 2 || common.len() != 1 {
                    continue;
                }
                let top = common[0];
                let flank = if fa[0] == top { fa[1] } else { fa[0] };
                let hub = if fc[0] == top { fc[1] } else { fc[0] };
                let (Surface::Plane { .. }, Surface::Plane { .. }) = (&b.face(top).surface, &b.face(flank).surface) else { continue };
                let Surface::Cylinder { frame: hf, r: rh } = &b.face(hub).surface else { continue };
                let p = b.vertex(v).p;
                let nt = b.planar_normal(top).normalize();
                let na = b.planar_normal(flank).normalize();
                let hz = hf.z.normalize();
                // the arc lies on the top plane, about the hub's axis; the hub is convex (material inside)
                if hz.cross(nt).len() > 1e-9 || hz.cross(axis).len() > 1e-9 || (cc - hf.o).cross(hz).len() > 1e-7 * (1.0 + rad) || (rad - rh).abs() > 1e-7 * (1.0 + rad) {
                    continue;
                }
                let radial = (p - hf.o).reject(hz);
                let nh = normal_at(b, hub, p);
                if radial.len() < 1e-9 || nh.dot(radial) <= 0.0 {
                    continue;
                }
                // the flank is radial (its plane holds the hub axis) and runs outward from the hub at the corner
                let other = if ea.v0 == v { ea.v1 } else { ea.v0 };
                let da = (b.vertex(other).p - p).normalize();
                if na.dot(hz).abs() > 1e-9 || na.dot(hf.o - p).abs() > 1e-7 * (1.0 + rh) || (da - radial.normalize()).len() > 1e-7 {
                    continue;
                }
                // the vertex: exactly the two rim edges and one concave wall edge, not selected
                let inc: Vec<EdgeId> = b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).collect();
                let walls: Vec<EdgeId> = inc.iter().copied().filter(|&x| x != a && x != c).collect();
                if inc.len() != 3 || walls.len() != 1 || sel.contains(&walls[0]) || super::torus_corner::edge_concave(b, walls[0]) != Some(true) {
                    continue;
                }
                out.push(Reflex { v: p, line: i, arc: j, da, nt, na, c: cc, axis, rh: *rh });
            }
        }
    }
    out
}

/// The section loop of a convex fillet of radius `r` between two faces with outward normals `n1`, `n2` at the edge
/// point `p`, in the frame `fr`'s plane; `wide`: the leg along `n2`'s face is replaced by a step `k` beyond that face
/// (the section then reaches past the wall into the air). Returns the loop with its fillet arc tagged `arc`.
fn section(fr: &Frame, p: Vec3, n1: Vec3, n2: Vec3, r: f64, wide: Option<f64>) -> ProfileLoop {
    let uv = |q: Vec3| {
        let w = fr.world_to_uv(q);
        Vec2::new(w.x, w.y)
    };
    let c12 = n1.dot(n2);
    let c = p - (n1 + n2) * (r / (1.0 + c12));
    let (t1, t2) = (c + n1 * r, c + n2 * r);
    let (cp, u1, u2, cc) = (uv(p), uv(t1), uv(t2), uv(c));
    let arc = if (u1 - cc).cross(u2 - cc) > 0.0 { ProfileSeg::arc("arc", cc, u1, u2) } else { ProfileSeg::arc("arc", cc, u2, u1).reversed() };
    match wide {
        None => ProfileLoop::new(vec![ProfileSeg::line("l2", u2, cp), ProfileSeg::line("l1", cp, u1), arc]),
        Some(k) => {
            let (e1, e2) = (uv(t2 + n2 * k), uv(p + n2 * k));
            ProfileLoop::new(vec![ProfileSeg::line("w1", u2, e1), ProfileSeg::line("w2", e1, e2), ProfileSeg::line("l1", e2, u1), arc])
        }
    }
}

fn name_arc(tool: &mut Body, name: &FaceName) {
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
}

/// Cuts the corner wedge of `rc` (fillet radius `r`) from `body`, whose two rim tools stopped flush at the corner;
/// `k_line` / `k_arc` are the blend numbers of the two rim edges.
pub(super) fn cut_wedge(body: &Body, rc: &Reflex, r: f64, feature: &str, k_line: u32, k_arc: u32, tol: f64) -> Result<Body, BlendError> {
    // the tools' run-on lengths only have to cover the corner square; a boolean that does not close for one choice
    // (a classification ray through the crease's end) is tried again with others
    let mut first = None;
    for (ki, ka) in [(2.0, 3.0), (1.5, 2.5), (2.5, 3.5)] {
        match cut_wedge_once(body, rc, r, feature, k_line, k_arc, tol, ki, ka) {
            Ok(out) => return Ok(out),
            Err(e) => {
                first.get_or_insert(e);
            }
        }
    }
    Err(first.unwrap())
}

#[allow(clippy::too_many_arguments)]
fn cut_wedge_once(body: &Body, rc: &Reflex, r: f64, feature: &str, k_line: u32, k_arc: u32, tol: f64, ki: f64, ka: f64) -> Result<Body, BlendError> {
    let fail = |m: &str| {
        super::diag::dbg_fail(&format!("reflex rim corner at ({:.3},{:.3},{:.3}): {m}", rc.v.x, rc.v.y, rc.v.z));
        BlendError::code("blendFailed")
    };
    let (name_l, name_a) = (FaceName::blend(feature, k_line), FaceName::blend(feature, k_arc));
    // the flank tool, run on `ki r` past the corner (into the hub) and r back along the flank
    let o = rc.v - rc.da * (ki * r);
    let fr = Frame::from_normal(o, rc.da);
    let prof = Profile::new(fr, vec![ProfileRegion { outer: section(&fr, rc.v, rc.nt, rc.na, r, None), holes: vec![] }]).normalized();
    let mut tf = extrude(feature, &prof, ExtrudeOpts { s0: 0.0, s1: (ki + 1.0) * r, dir: None, draft: 0.0 }).map_err(|e| fail(&format!("flank tool: {e}")))?;
    name_arc(&mut tf, &name_l);
    // the hub tool, run on past the corner both ways, its section widened r beyond the hub wall
    // (swept from its own start meridian, `al` before the corner, so that the torus's seam is not inside the wedge)
    let al = ka * r / (rc.rh - r).max(r);
    let rot = |v: Vec3, t: f64| v * t.cos() + rc.axis.cross(v) * t.sin() + rc.axis * (rc.axis.dot(v) * (1.0 - t.cos()));
    let v0 = rc.c + rot(rc.v - rc.c, -al);
    let rad = (v0 - rc.c).reject(rc.axis).normalize();
    let frh = ck_geom::frame_from(rc.c, rad.cross(rc.axis), rad);
    let profh = Profile::new(frh, vec![ProfileRegion { outer: section(&frh, v0, rc.nt, rad, r, Some(r)), holes: vec![] }]).normalized();
    let mut th = ck_topo::make::revolve::revolve(feature, &profh, rc.c, rc.axis, 0.0, 2.0 * al).map_err(|e| fail(&format!("hub tool: {e}")))?;
    name_arc(&mut th, &name_a);
    // the two fillet surfaces meet along the crease, which is known in closed form; near its top end both surfaces
    // are tangent to the top plane (and to each other), where marching their intersection is unreliable
    let crease = crease_curve(rc, r).ok_or_else(|| fail("crease fit"))?;
    let (is_cyl, is_tor) = (|s: &Surface| flank_cylinder(rc, r, s), |s: &Surface| hub_torus(rc, r, s));
    let hook_tools = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> { (is_cyl(&tf.face(f).surface) && is_tor(&th.face(g).surface)).then(|| vec![crease.clone()]) };
    let (wedge, _) = super::build::apply_tool_hooked(&tf, &th, Op::Intersect, &name_l, vec![], tol, Some(&hook_tools)).map_err(|e| fail(&format!("wedge: {}", e.code)))?;
    let wedge = clip_halfspace(&wedge, rc.v, -rc.da, tol).map_err(|_| fail("wedge clip"))?;
    if super::diag::on() {
        eprintln!(
            "reflex wedge at ({:.3},{:.3},{:.3}): faces {:?}",
            rc.v.x,
            rc.v.y,
            rc.v.z,
            wedge.face_ids().map(|f| format!("{} {}", wedge.face_id_str(f), wedge.face(f).surface.kind())).collect::<Vec<_>>()
        );
    }
    let hook_cut = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> {
        let (sf, sg) = (&body.face(f).surface, &wedge.face(g).surface);
        if (is_cyl(sf) && is_tor(sg)) || (is_tor(sf) && is_cyl(sg)) {
            Some(vec![crease.clone()])
        } else if is_tor(sf) && is_tor(sg) {
            // the hub fillet and the wedge's piece of it: one surface, they only share the edge on the flank plane
            Some(vec![])
        } else {
            None
        }
    };
    // the wedge's piece of the hub fillet merges into the hub fillet face
    let old = crate::boolean::assemble::MERGE_TORI.with(|c| c.replace(true));
    let out = apply_tool_hooked(body, &wedge, Op::Cut, &name_l, vec![], tol, Some(&hook_cut)).map(|x| x.0);
    crate::boolean::assemble::MERGE_TORI.with(|c| c.set(old));
    out
}

/// The flank fillet's cylinder of `rc` (radius `r`).
fn flank_cylinder(rc: &Reflex, r: f64, s: &Surface) -> bool {
    let Surface::Cylinder { frame, r: rr } = s else { return false };
    let axis_pt = rc.v - (rc.nt + rc.na) * r;
    (rr - r).abs() <= 1e-9 * (1.0 + r) && frame.z.normalize().cross(rc.da).len() < 1e-9 && (frame.o - axis_pt).reject(rc.da).len() <= 1e-7 * (1.0 + r)
}

/// The hub fillet's torus of `rc` (radius `r`).
fn hub_torus(rc: &Reflex, r: f64, s: &Surface) -> bool {
    let Surface::Torus { frame, major, minor } = s else { return false };
    let ctr = rc.c - rc.nt * r;
    (minor - r).abs() <= 1e-9 * (1.0 + r) && (major - (rc.rh - r)).abs() <= 1e-7 * (1.0 + rc.rh) && frame.z.normalize().cross(rc.axis).len() < 1e-9 && frame.o.dist(ctr) <= 1e-7 * (1.0 + rc.rh)
}

/// The crease of the corner: the points at distance `d` from both walls (the flank plane and the hub cylinder) on the
/// common fillet profile, from the wall edge at `H - r` (`d = 0`) to the corner of the two top contact lines
/// (`d = r`); parameterised by the profile angle, `d = r (1 - cos a)`, height `H - r + r sin a`.
fn crease_curve(rc: &Reflex, r: f64) -> Option<Curve> {
    const N: usize = 32;
    let pts: Vec<Vec3> = (0..=N)
        .map(|i| {
            let a = std::f64::consts::FRAC_PI_2 * i as f64 / N as f64;
            let d = r * (1.0 - a.cos());
            let along = ((rc.rh - d) * (rc.rh - d) - d * d).max(0.0).sqrt();
            rc.c + rc.da * along - rc.na * d + rc.nt * (r * a.sin() - r)
        })
        .collect();
    ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).ok().map(Curve::BSpline)
}
