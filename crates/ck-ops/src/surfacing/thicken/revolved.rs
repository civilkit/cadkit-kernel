//! Thicken of a revolved sheet: one sphere, cone or torus face whose boundary is the whole
//! (u, v) rectangle of its surface (the sheet `surfaceRevolve` makes from one arc or line, full
//! turn or partial). The wall is the region between the meridian curve and its offset, closed by
//! the two normal segments at its ends, revolved through the sheet's angle: the layers are the
//! surface and its concentric offset, the walls are the cones / planes the normals sweep and, for a
//! partial turn, the two meridian planes. The general layer builder translates rim edges by one
//! normal per face, which is wrong for a doubly curved face (a full turn was refused at the seam, a
//! partial turn came back invalid with the wrong volume).

use super::{cone_sign, face_normal, layer_suffix, Side, ThickenError};
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, FaceId, FaceName, Role};
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::{PI, TAU};

/// Start and span of the angles in `a` (radians, any range) on the circle: the complement of the largest
/// gap between neighbours. `None` for fewer than two angles.
fn angle_range(a: &[f64]) -> Option<(f64, f64)> {
    let mut s: Vec<f64> = a.iter().map(|x| x.rem_euclid(TAU)).collect();
    if s.len() < 2 {
        return None;
    }
    s.sort_by(f64::total_cmp);
    let (mut gap, mut start) = (s[0] + TAU - s[s.len() - 1], s[0]);
    for w in s.windows(2) {
        if w[1] - w[0] > gap {
            gap = w[1] - w[0];
            start = w[1];
        }
    }
    Some((start, TAU - gap))
}

/// The arc about `c` from `a` to `b` through `m`, as a profile segment.
fn arc3(tag: &str, c: Vec2, a: Vec2, m: Vec2, b: Vec2) -> ProfileSeg {
    let ang = |p: Vec2| (p.y - c.y).atan2(p.x - c.x);
    let (ta, tm, tb) = (ang(a), ang(m), ang(b));
    if (tm - ta).rem_euclid(TAU) < (tb - ta).rem_euclid(TAU) {
        ProfileSeg::arc(tag, c, a, b)
    } else {
        ProfileSeg::arc(tag, c, b, a).reversed()
    }
}

/// `None` when the sheet is not one full-rectangle sphere / cone / torus face (the general path handles it).
pub(super) fn thicken_revolved(sheet: &Body, feature: &str, thickness: f64, side: Side) -> Option<Result<Body, ThickenError>> {
    let faces: Vec<FaceId> = sheet.face_ids().collect();
    let [f] = faces[..] else { return None };
    let face = sheet.face(f);
    let (frame, kind) = match &face.surface {
        Surface::Sphere { frame, .. } | Surface::Cone { frame, .. } | Surface::Torus { frame, .. } => (*frame, &face.surface),
        _ => return None,
    };
    // boundary samples: angles about the axis and the meridian parameter v
    let mut us: Vec<f64> = vec![];
    let mut vs: Vec<f64> = vec![];
    let mut seam = false;
    for e in sheet.face_edges(f) {
        seam |= sheet.is_seam(e);
        let ed = sheet.edge(e);
        for k in 0..=32 {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 32.0);
            let q = p - frame.o;
            let (x, y, z) = (q.dot(frame.x), q.dot(frame.y), q.dot(frame.z));
            let rho = x.hypot(y);
            if rho > 1e-9 * (1.0 + q.len()) {
                us.push(y.atan2(x));
            }
            vs.push(match kind {
                Surface::Sphere { r, .. } => (z / r).clamp(-1.0, 1.0).asin(),
                Surface::Cone { .. } => z,
                Surface::Torus { major, .. } => z.atan2(rho - major),
                _ => unreachable!(),
            });
        }
    }
    let (u0, du) = if seam { (0.0, TAU) } else { angle_range(&us)? };
    let (v0, v1) = match kind {
        Surface::Torus { .. } => {
            let (s, w) = angle_range(&vs)?;
            // the parameter range of the tube circle that holds the samples (atan2 runs over -pi..pi)
            let s = if s > PI { s - TAU } else { s };
            (s, s + w)
        }
        _ => (vs.iter().copied().fold(f64::INFINITY, f64::min), vs.iter().copied().fold(f64::NEG_INFINITY, f64::max)),
    };
    if !(du > 1e-6 && v1 - v0 > 1e-9) {
        return None;
    }
    // the face must be the whole rectangle: its area against the patch's
    let patch = match kind {
        Surface::Sphere { r, .. } => r * r * du * (v1.sin() - v0.sin()),
        Surface::Cone { r, semi_angle, .. } => {
            let t = semi_angle.tan();
            let rho = |v: f64| r + v * t;
            // area of a frustum strip, |rho| integrated along the slant
            du / semi_angle.cos() * if rho(v0) * rho(v1) >= 0.0 { (0.5 * (rho(v0) + rho(v1)) * (v1 - v0)).abs() } else { return None }
        }
        Surface::Torus { major, minor, .. } => minor * du * (major * (v1 - v0) + minor * (v1.sin() - v0.sin())),
        _ => unreachable!(),
    };
    let area = ck_topo::mass::face_integral_vpoly::<1>(sheet, f, 1, &|_, n| [n.len()])[0].abs();
    if !(patch > 0.0 && (area - patch).abs() <= 1e-6 * patch) {
        return None;
    }

    // the layers as offsets along the face's thicken normal (see `surface_offset`): sphere and torus offset
    // along the outward surface normal, a cone along its oriented normal
    let layers: Vec<(f64, bool)> = match side {
        Side::Front => vec![(0.0, true), (thickness, false)],
        Side::Back => vec![(0.0, true), (-thickness, false)],
        Side::Both => vec![(-thickness * 0.5, false), (thickness * 0.5, false)],
    };
    let sgn = if matches!(kind, Surface::Cone { .. }) { cone_sign(sheet, f) } else { 1.0 };
    // meridian plane at u0: 2D (radial, axial) about the axis
    let rad = frame.x * u0.cos() + frame.y * u0.sin();
    let pf = Frame { o: frame.o, x: rad, y: frame.z, z: rad.cross(frame.z) };
    let to2 = |p: Vec3| Vec2::new((p - pf.o).dot(pf.x), (p - pf.o).dot(pf.y));
    let pt = |v: f64, s: f64| {
        let n = kind.normal(u0, v).normalize() * sgn;
        to2(kind.eval(u0, v) + n * s)
    };
    let centre = match kind {
        Surface::Sphere { .. } => Some(Vec2::new(0.0, 0.0)),
        Surface::Torus { major, .. } => Some(Vec2::new(*major, 0.0)),
        _ => None,
    };
    let too_large = || Some(Err(ThickenError::detail("offsetTooLarge", "the offset passes the surface's centre of curvature")));
    let vm = 0.5 * (v0 + v1);
    let mut segs: Vec<ProfileSeg> = vec![];
    // layer curves: the first runs v0 -> v1, the second back
    let curve = |li: usize, s: f64| -> Option<(ProfileSeg, Vec2, Vec2)> {
        let (a, m, b) = (pt(v0, s), pt(vm, s), pt(v1, s));
        let tag = format!("L{li}");
        match centre {
            Some(c) => {
                // passing the centre turns the offset arc inside out
                if (m - c).dot(pt(vm, 0.0) - c) <= 1e-9 || m.dist(c) < 1e-9 {
                    return None;
                }
                Some((arc3(&tag, c, a, m, b), a, b))
            }
            None => {
                if a.dist(b) < 1e-9 {
                    return None;
                }
                Some((ProfileSeg::line(&tag, a, b), a, b))
            }
        }
    };
    let Some((c0, a0, a1)) = curve(0, layers[0].0) else { return too_large() };
    let Some((c1, b0, b1)) = curve(1, layers[1].0) else { return too_large() };
    // a profile point across the axis would revolve through it
    if [a0, a1, b0, b1, pt(vm, layers[0].0), pt(vm, layers[1].0)].iter().any(|p| p.x < -1e-9) {
        return Some(Err(ThickenError::hint(
            "offsetTooLarge",
            "the thickened wall would cross the axis of the revolved sheet",
            "use a smaller thickness, or thicken towards the other side",
        )));
    }
    segs.push(c0);
    segs.push(ProfileSeg::line("w1", a1, b1));
    segs.push(c1.reversed());
    segs.push(ProfileSeg::line("w0", b0, a0));
    // an end on the axis (a pole) leaves a connector along the axis; a zero-length one is dropped
    segs.retain(|s| {
        let (p, q) = (s.curve.eval(s.t0), s.curve.eval(s.t1));
        p.dist(q) > 1e-12 || s.t1 - s.t0 > 1e-9
    });
    let profile = Profile::new(pf, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized();
    let mut out = match ck_topo::make::revolve::revolve(feature, &profile, frame.o, frame.z, 0.0, du) {
        Ok(b) => b,
        Err(e) => return Some(Err(ThickenError::detail("thickenFailed", format!("the revolved wall could not be built: {e}")))),
    };
    // names as the general path gives them: the layers keep the sheet face's name (`~k` by layer), the rest are walls
    let n = face_normal(sheet, f);
    let mut w = 0u32;
    for g in out.face_ids().collect::<Vec<_>>() {
        let name = match &out.face(g).name.role {
            Role::Side(t) if t == "L0" || t == "L1" => {
                let li = if t == "L0" { 0 } else { 1 };
                let mut nm = face.name.clone();
                nm.region = Some(layer_suffix(&layers, li, n));
                nm
            }
            _ => {
                let nm = FaceName { feature: feature.into(), role: Role::Other(format!("W{w}")), region: None, suffix: String::new() };
                w += 1;
                nm
            }
        };
        out.faces[g.idx()].name = name;
    }
    Some(Ok(out))
}
