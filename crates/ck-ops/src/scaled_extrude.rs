//! `scaled_extrude` (ck-ops/src/scaled_extrude.rs): extrude a planar profile along its normal while it
//! scales linearly about a point of the sketch plane (the extrude feature's `endScale`).
//!
//! Every section at height `s` is the profile scaled by `k(s)` about the axis through `centre` (parallel to
//! the normal); `k` is 1 at the end nearest the sketch plane and `endScale` at the far end. So each profile
//! edge and its scaled copy bound a ruled side face:
//! * a line gives a plane (the two lines are parallel);
//! * a circle / arc about the scaling centre gives an exact right cone;
//! * any other arc (circle about another point, ellipse) and splines give a ruled rational B-spline: scaling is an
//!   affine map, so the control net of the far section is the near one scaled (an oblique cone is exact, as a
//!   rational quadratic arc swept straight to its scaled copy).
//!
//! Names and numbering follow the plain extrude (`F.S.<tag>`, caps `F.C0` low end / `F.C1` high end, rails first
//! then the C0 and C1 edge of every segment). A mid-plane extrude is the two halves (scale 1 at the sketch plane,
//! `endScale` at each end), united: the side face of an entity is then two faces, numbered `~1` / `~2`.

use crate::boolean::{boolean, Op};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::convert::curve2_to_nurbs;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::Body;
use ck_topo::make::{
    build_sweep, curve2_to_3d, extrude, plane_line_pc, sense_of, Caps, ExtrudeOpts, LoopGeom, Profile, ProfileLoop, ProfileSeg, SegGeom, SideGeom,
    VertGeom,
};

/// Smallest accepted end scale: below this the far section is a point (an apex), which this build refuses.
pub const MIN_SCALE: f64 = 1e-6;

/// Extrude `profile` over `[o.s0, o.s1]` (along the sketch normal) scaling to `end_scale` at the far end.
/// `centre`: the scaling point in sketch coordinates. `end_scale == 1` is the plain extrude.
pub fn scaled_extrude(feature: &str, profile: &Profile, o: ExtrudeOpts, end_scale: f64, centre: Vec2) -> Result<Body, String> {
    if !end_scale.is_finite() || end_scale <= 0.0 {
        return Err("invalidValue".into());
    }
    if o.draft != 0.0 {
        return Err("draftUnsupported".into());
    }
    if (end_scale - 1.0).abs() < 1e-12 {
        return extrude(feature, profile, o);
    }
    if end_scale < MIN_SCALE {
        return Err("scaleTooSmall".into());
    }
    profile.validate()?;
    let n = profile.frame.z;
    if o.dir.map_or(false, |d| d.normalize().cross(n).len() > 1e-9) {
        return Err("scaleUnsupported".into());
    }
    let (s0, s1) = (o.s0.min(o.s1), o.s0.max(o.s1));
    if s1 - s0 < 1e-9 {
        return Err("zeroDepth".into());
    }
    if s0 < -1e-12 && s1 > 1e-12 {
        let lower = ruled(feature, profile, s0, 0.0, end_scale, 1.0, centre)?;
        let upper = ruled(feature, profile, 0.0, s1, 1.0, end_scale, centre)?;
        let mut out = boolean(&lower, &upper, Op::Union, 1e-6)?.body;
        out.number_split_faces();
        return Ok(out);
    }
    if s1 <= 1e-12 {
        ruled(feature, profile, s0, s1, end_scale, 1.0, centre)
    } else {
        ruled(feature, profile, s0, s1, 1.0, end_scale, centre)
    }
}

/// `p` scaled by `k` about `c`.
fn about(p: Vec2, c: Vec2, k: f64) -> Vec2 {
    c + (p - c) * k
}

/// A sketch curve scaled by `k` about `c`: a line keeps its direction (its parameter is scaled by `k`), a circle
/// keeps its angle parameter. `None` for curves that are scaled as B-splines.
fn scale_curve2(cv: &Curve2, c: Vec2, k: f64) -> Option<Curve2> {
    match cv {
        Curve2::Line { o, d } => Some(Curve2::Line { o: about(*o, c, k), d: *d }),
        Curve2::Circle { c: cc, x, r, ccw } => Some(Curve2::Circle { c: about(*cc, c, k), x: *x, r: r * k, ccw: *ccw }),
        _ => None,
    }
}

fn is_line(c: &Curve2) -> bool {
    matches!(c, Curve2::Line { .. })
}

/// One ruled build from the section scaled by `kb` at height `s0` to the one scaled by `kt` at `s1`.
fn ruled(feature: &str, profile: &Profile, s0: f64, s1: f64, kb: f64, kt: f64, centre: Vec2) -> Result<Body, String> {
    let fr = profile.frame;
    let n = fr.z;
    let h = s1 - s0;
    let mut regions = vec![];
    let mut caps = vec![];
    for reg in &profile.regions {
        let mut lgs = vec![];
        for pl in reg.loops() {
            lgs.push(ruled_loop(&fr, pl, s0, s1, kb, kt, centre, h)?);
        }
        regions.push(lgs);
        caps.push(Caps { c0: Frame { o: fr.o + n * s0, ..fr }, c1: Frame { o: fr.o + n * s1, ..fr } });
    }
    Ok(build_sweep(feature, profile, regions, Some(&caps), true, false))
}

#[allow(clippy::too_many_arguments)]
fn ruled_loop(fr: &Frame, pl: &ProfileLoop, s0: f64, s1: f64, kb: f64, kt: f64, c: Vec2, h: f64) -> Result<LoopGeom, String> {
    let m = pl.segs.len();
    let n = fr.z;
    let p3 = |p: Vec2, s: f64| fr.o + fr.x * p.x + fr.y * p.y + n * s;
    let mut verts = vec![];
    for sg in &pl.segs {
        let v = sg.start();
        let (a, b) = (p3(about(v, c, kb), s0), p3(about(v, c, kt), s1));
        let len = a.dist(b);
        if !(len > 1e-12) {
            return Err("scaleDegenerate".into());
        }
        verts.push(VertGeom { bottom: a, top: b, rail: Some((Curve::line_through(a, b), 0.0, len)) });
    }
    let mut segs = vec![];
    for (i, s) in pl.segs.iter().enumerate() {
        let (vs, ve) = if s.rev { ((i + 1) % m, i) } else { (i, (i + 1) % m) };
        let tm_e = 0.5 * (s.t0 + s.t1);
        let tan2 = s.travel_tangent(tm_e);
        let outward = (fr.x * tan2.x + fr.y * tan2.y).cross(n);
        segs.push(ruled_seg(fr, s, &verts[vs], &verts[ve], s0, s1, kb, kt, c, h, outward, &p3)?);
    }
    Ok(LoopGeom { verts, segs })
}

fn rail_line(v: &VertGeom) -> (Vec3, Vec3, f64) {
    let (Curve::Line { o, d }, _, len) = v.rail.as_ref().unwrap() else { unreachable!() };
    (*o, *d, *len)
}

#[allow(clippy::too_many_arguments)]
fn ruled_seg(
    fr: &Frame,
    s: &ProfileSeg,
    rail_t0: &VertGeom,
    rail_t1: &VertGeom,
    s0: f64,
    s1: f64,
    kb: f64,
    kt: f64,
    c: Vec2,
    h: f64,
    outward: Vec3,
    p3: &dyn Fn(Vec2, f64) -> Vec3,
) -> Result<SegGeom, String> {
    let n = fr.z;
    let bc = scale_curve2(&s.curve, c, kb);
    let tc = scale_curve2(&s.curve, c, kt);
    // trims of the scaled copies: a line's parameter is arc length, so it scales; angles do not
    let (kbt, ktt) = if is_line(&s.curve) { (kb, kt) } else { (1.0, 1.0) };
    let (bt0, bt1, tt0, tt1) = (s.t0 * kbt, s.t1 * kbt, s.t0 * ktt, s.t1 * ktt);
    if let (Some(bc), Some(tc)) = (&bc, &tc) {
        let bottom3 = curve2_to_3d(fr, bc, n * s0);
        let top3 = curve2_to_3d(fr, tc, n * s1);
        match (bc, tc) {
            (Curve2::Line { .. }, Curve2::Line { .. }) => {
                let (Curve::Line { o: bo, d: bd }, Curve::Line { o: to, d: td }) = (&bottom3, &top3) else { unreachable!() };
                let x = (bottom3.eval(bt1) - bottom3.eval(bt0)).normalize();
                let w = *to - *bo;
                let y = (w - x * w.dot(x)).normalize();
                let pf = Frame { o: *bo, x, y, z: x.cross(y) };
                let surface = Surface::Plane { frame: pf };
                let pc_bottom = plane_line_pc(&pf, *bo, *bd);
                let pc_top = plane_line_pc(&pf, *to, *td);
                let rl = |v: &VertGeom| {
                    let (o, d, _) = rail_line(v);
                    plane_line_pc(&pf, o, d)
                };
                let tm = 0.5 * (bt0 + bt1);
                let sense = sense_of(&surface, pc_bottom.eval(tm), outward);
                let side = SideGeom { surface, sense, pc_bottom, pc_top, pc_rail_t0: Some(rl(rail_t0)), pc_rail_t1: Some(rl(rail_t1)) };
                return Ok(SegGeom { bottom: (bottom3, bt0, bt1), top: (top3, tt0, tt1), cap0_pc: Some(bc.clone()), cap1_pc: Some(tc.clone()), side: Some(side) });
            }
            (Curve2::Circle { c: c0, .. }, Curve2::Circle { r: r1, c: c1, .. }) if c0.dist(c) <= 1e-9 * (1.0 + c0.len()) && c1.dist(c) <= 1e-9 * (1.0 + c1.len()) => {
                let Curve2::Circle { r: r0, .. } = bc else { unreachable!() };
                let Curve::Circle { frame: cf, .. } = &bottom3 else { unreachable!() };
                let x = cf.x;
                let y = n.cross(x);
                let su = if cf.y.dot(y) > 0.0 { 1.0 } else { -1.0 };
                let frame = Frame { o: cf.o, x, y, z: n };
                let surface = Surface::Cone { frame, r: *r0, semi_angle: ((r1 - r0) / h).atan() };
                let pc_bottom = Curve2::Line { o: Vec2::ZERO, d: Vec2::new(su, 0.0) };
                let pc_top = Curve2::Line { o: Vec2::new(0.0, h), d: Vec2::new(su, 0.0) };
                let rl = |tb: f64, tt: f64, v: &VertGeom| {
                    let len = rail_line(v).2.max(1e-300);
                    Curve2::Line { o: Vec2::new(su * tb, 0.0), d: Vec2::new(su * (tt - tb) / len, h / len) }
                };
                let tm = 0.5 * (bt0 + bt1);
                let sense = sense_of(&surface, pc_bottom.eval(tm), outward);
                let side = SideGeom {
                    surface,
                    sense,
                    pc_bottom,
                    pc_top,
                    pc_rail_t0: Some(rl(bt0, tt0, rail_t0)),
                    pc_rail_t1: Some(rl(bt1, tt1, rail_t1)),
                };
                return Ok(SegGeom { bottom: (bottom3, bt0, bt1), top: (top3, tt0, tt1), cap0_pc: Some(bc.clone()), cap1_pc: Some(tc.clone()), side: Some(side) });
            }
            _ => {}
        }
    }
    // free-form: the section as a (rational) B-spline, ruled to its scaled copy
    let nb = curve2_to_nurbs(&s.curve, s.t0, s.t1);
    let bottom = nb.map(|p| p3(about(p, c, kb), s0));
    let top = nb.map(|p| p3(about(p, c, kt), s1));
    let (d0, d1) = bottom.domain();
    let nu = nb.ctrl.len();
    let mut ctrl = Vec::with_capacity(2 * nu);
    let mut wts = Vec::with_capacity(2 * nu);
    for i in 0..nu {
        ctrl.push(bottom.ctrl[i]);
        ctrl.push(top.ctrl[i]);
        let w = nb.weights.as_ref().map_or(1.0, |w| w[i]);
        wts.push(w);
        wts.push(w);
    }
    let rational = wts.iter().any(|w| (w - 1.0).abs() > 1e-14);
    let surf = BSplineSurface::new(nb.degree, 1, nb.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], nu, 2, ctrl, if rational { Some(wts) } else { None })
        .map_err(|e| format!("scaleSurface;{e}"))?;
    let surface = Surface::BSpline(Box::new(surf));
    let pc_bottom = Curve2::Line { o: Vec2::ZERO, d: Vec2::new(1.0, 0.0) };
    let pc_top = Curve2::Line { o: Vec2::new(0.0, 1.0), d: Vec2::new(1.0, 0.0) };
    let rl = |u: f64, v: &VertGeom| {
        let len = rail_line(v).2.max(1e-300);
        Curve2::Line { o: Vec2::new(u, 0.0), d: Vec2::new(0.0, 1.0 / len) }
    };
    let sense = sense_of(&surface, Vec2::new(0.5 * (d0 + d1), 0.0), outward);
    let side = SideGeom { surface, sense, pc_bottom, pc_top, pc_rail_t0: Some(rl(d0, rail_t0)), pc_rail_t1: Some(rl(d1, rail_t1)) };
    Ok(SegGeom {
        bottom: (Curve::BSpline(bottom), d0, d1),
        top: (Curve::BSpline(top), d0, d1),
        cap0_pc: Some(Curve2::BSpline(nb.map(|p| about(p, c, kb)))),
        cap1_pc: Some(Curve2::BSpline(nb.map(|p| about(p, c, kt)))),
        side: Some(side),
    })
}
