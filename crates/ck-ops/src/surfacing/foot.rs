//! Sinking the foot of a thickened wall into the body it is added to.
//!
//! The wall of a thickened cone sheet is an exact normal offset of the sheet, so its base is a conical ring normal to the
//! sheet. When the sheet starts on a plane of the body the wall is added to, the ring's inner circle stands on that plane
//! and its outer edge hovers a fraction of the wall thickness above it: a thin wedge between ring and plane stays open
//! and the foot edge is a convex corner of the wall, not the cone / plane edge a person expects (and cannot be filleted as
//! such). The usual workflow is to let the wall start inside the body. [`sink_feet`] does this for the wall: it adds the
//! wedge, continued a little below the plane, as a revolved skirt on the same cone, so the union with the body makes the
//! wall meet the plane in an ordinary concave cone / plane edge.
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::Op;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// How far below the plane the skirt reaches (mm).
const SINK: f64 = 0.3;
/// Steepest base ring that counts as a wedge over the plane: tan 30 degrees.
const MAX_SLOPE: f64 = 0.5774;

/// A circle of the wall's base ring: full-circle centre, axis, radius, total swept angle of the ring's edges at that radius.
struct Ring {
    c: Vec3,
    axis: Vec3,
    r: f64,
    p: Vec3,
    span: f64,
}

fn rings_of(wall: &Body, f: FaceId) -> Vec<Ring> {
    let mut out: Vec<Ring> = vec![];
    for e in wall.face_edges(f) {
        let ed = wall.edge(e);
        let Some((c, axis, r)) = ed.curve.circle_info() else { continue };
        let span = (ed.t1 - ed.t0).abs();
        match out.iter_mut().find(|q| (q.r - r).abs() < 1e-9 && (q.c - c).len() < 1e-7 * (1.0 + r)) {
            Some(q) => q.span += span,
            None => out.push(Ring { c, axis: axis.normalize(), r, p: ed.curve.eval(ed.t0.min(ed.t1)), span }),
        }
    }
    out
}

/// `wall` with the wedge under each base ring that stands on a planar face of `host` added as a skirt (see module docs).
/// Walls that do not start on a plane of the host come back unchanged, as does a wall for which the skirt fails.
pub fn sink_feet(wall: &Body, host: &Body, feature: &str, tol: f64) -> Body {
    let mut acc = wall.clone();
    // base rings already handled, as (low radius, high radius)
    let mut done: Vec<(f64, f64)> = vec![];
    for f in wall.face_ids() {
        if !matches!(wall.face(f).surface, Surface::Cone { .. }) {
            continue;
        }
        let rs = rings_of(wall, f);
        if rs.len() != 2 || rs[0].axis.cross(rs[1].axis).len() > 1e-9 {
            continue;
        }
        for (low, high) in [(0, 1), (1, 0)] {
            let key = (rs[low].r, rs[high].r);
            if done.iter().any(|d| (d.0 - key.0).abs() < 1e-9 && (d.1 - key.1).abs() < 1e-9) {
                continue;
            }
            // a ring split into halves by the sheet's seam: its faces together must go all the way round
            let total: f64 = wall.face_ids().filter(|&g| matches!(wall.face(g).surface, Surface::Cone { .. })).map(|g| {
                let q = rings_of(wall, g);
                if q.len() == 2 && q.iter().any(|x| (x.r - key.0).abs() < 1e-9) && q.iter().any(|x| (x.r - key.1).abs() < 1e-9) { q[0].span } else { 0.0 }
            }).sum();
            if total < std::f64::consts::TAU - 1e-6 {
                continue;
            }
            if let Some(sk) = skirt(wall, host, &rs[low], &rs[high], feature, tol) {
                done.push(key);
                match crate::boolean::boolean(&acc, &sk, Op::Union, tol) {
                    Ok(r) => acc = r.body,
                    Err(e) => {
                        if std::env::var_os("CK_FOOT_DEBUG").is_some() {
                            eprintln!("foot: skirt union failed: {e}");
                        }
                    }
                }
            }
        }
    }
    acc
}

fn skirt(wall: &Body, host: &Body, low: &Ring, high: &Ring, feature: &str, tol: f64) -> Option<Body> {
    let axis = low.axis;
    if (high.c - low.c).cross(axis).len() > 1e-7 * (1.0 + high.r) {
        return None;
    }
    // the plane of the host the low circle stands on
    let plane = host.face_ids().find(|&g| {
        let Surface::Plane { frame } = &host.face(g).surface else { return false };
        frame.z.cross(axis).len() < 1e-9
            && (low.p - frame.o).dot(frame.z).abs() <= 1e-7 * (1.0 + low.r)
            && point_in_face(host, g, low.p, tol) != Where::Out
    })?;
    let Surface::Plane { frame: pf } = &host.face(plane).surface else { return None };
    let np = {
        let (u, v) = host.face(plane).surface.closest_param(high.p);
        host.outward_normal(plane, u, v).normalize()
    };
    let hover = (high.p - pf.o).dot(np);
    if hover <= 1e-6 * (1.0 + high.r) || hover > MAX_SLOPE * (high.r - low.r).abs() {
        return None;
    }
    // the wall face on the other side of the hovering edge
    let wall_face = wall.face_ids().find(|&g| {
        let at = |rr: f64| {
            wall.face_edges(g).into_iter().any(|e| wall.edge(e).curve.circle_info().is_some_and(|(c, _, r)| (r - rr).abs() < 1e-9 && (c - high.c).len() < 1e-7 * (1.0 + r)))
        };
        matches!(wall.face(g).surface, Surface::Cone { .. } | Surface::Cylinder { .. }) && at(high.r) && !at(low.r)
    })?;
    let rd = {
        let v = high.p - high.c;
        (v - axis * v.dot(axis)).normalize()
    };
    let nw = {
        let (u, v) = wall.face(wall_face).surface.closest_param(high.p);
        wall.outward_normal(wall_face, u, v).normalize()
    };
    let (nwx, nwy) = (nw.dot(rd), nw.dot(np));
    let mut t = (-nwy, nwx);
    if t.1 > 0.0 {
        t = (-t.0, -t.1);
    }
    if t.1 > -1e-6 || (nw - rd * nwx - np * nwy).len() > 1e-6 {
        return None;
    }
    // the wall line meets the plane beyond the hovering edge (away from the ring's inner circle)
    let sc = hover / -t.1;
    let cx = high.r + sc * t.0;
    if cx <= 0.0 || (cx - high.r) * (high.r - low.r) <= 0.0 {
        return None;
    }
    let o = high.c + axis * ((pf.o - high.c).dot(axis));
    let p3 = |x: f64, y: f64| o + rd * x + np * y;
    let fr = ck_geom::frame_from(high.c, rd.cross(axis), rd);
    let uv = |p: Vec3| {
        let q = fr.world_to_uv(p);
        Vec2::new(q.x, q.y)
    };
    let sd = SINK / -t.1;
    let pts = [p3(low.r, 0.0), p3(high.r, hover), p3(cx + t.0 * sd, -SINK), p3(low.r, -SINK)];
    let uvs: Vec<Vec2> = pts.iter().map(|&p| uv(p)).collect();
    let n = uvs.len();
    let lp = ProfileLoop::new((0..n).map(|i| ProfileSeg::line(["l1", "l2", "l3", "l4"][i], uvs[i], uvs[(i + 1) % n])).collect());
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]).normalized();
    ck_topo::make::revolve::revolve(feature, &prof, high.c, axis, 0.0, std::f64::consts::TAU).ok()
}
