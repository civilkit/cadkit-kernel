//! Extended mate kinds (perpendicular, tangent, lock, symmetric, width,
//! hinge, slot, screw, gear, rackPinion, limits, and a working `angle`)
//! plus the degrees-of-freedom report.
//!
//! Nothing here is pinned by the oracle (it knows five kinds); the kinds are
//! built on the same one-mover Gauss-Seidel solver as the oracle kinds
//! ([`super::mate`]): every geometric mate lowers to constraint records
//! ([`Rec`]) of its mover. Kinds that couple two motions (gear, rackPinion,
//! screw, limits) cannot be written as records of one body; they run as a
//! "post" step after the mover's records: a direct rigid correction of the
//! follower (and of the driver when a drive `angle` is given), measured
//! against the placements both components had when the mate was created.
//!
//! The DOF report differentiates every mate's residual numerically (central
//! differences over a 6-vector twist of each floating component) and reads
//! the rank and null space off the eigen decomposition of `JᵀJ`.

use super::mate::{comp_box, find_ref, knorm, plane_normal, rows, MateRec, Rec, Row, Xf, MATE_CONFLICT, MATE_FIXED, MATE_GEOMETRY, MATE_SAME_BODY};
use super::{Brep, FeatureKind, MateKind as K};
use crate::regen::{self, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{wrap_pi, Vec3};
use ck_json::{Json, Obj};
use ck_topo::body::Body;

/// Extra data of an extended mate.
#[derive(Debug, Clone, Default)]
pub struct Ext {
    /// Second face on the mover side (`a2`; width, hinge stop, ...).
    pub m2: Option<String>,
    /// Second face on the target side (`b2`).
    pub t2: Option<String>,
    /// Mirror / helper face (`c`).
    pub c: Option<String>,
    /// Target component (the driver for gear / rackPinion / screw).
    pub tc: usize,
    /// `angle` in radians: the target angle of an `angle` mate; the drive
    /// rotation of gear / rackPinion / screw.
    pub angle: Option<f64>,
    pub ratio: f64,
    pub pitch: f64,
    pub radius: f64,
    pub min: Option<f64>,
    pub max: Option<f64>,
    /// `limits` measure the angle instead of the distance.
    pub of_angle: bool,
    /// Rack travel direction when the rack is a plane (`dir`).
    pub dir: Option<Vec3>,
    /// Roles were swapped because the natural follower is fixed.
    pub swapped: bool,
    /// Side of the plane the contact is on (`±1`, fixed at creation; `flip`
    /// already applied) for tangent / one-wall slot.
    pub side: f64,
    pub creation: Option<Creation>,
}

/// Placements and axes at the moment the mate was created.
#[derive(Debug, Clone, Copy)]
pub struct Creation {
    pub pm0: Xf,
    pub pt0: Xf,
    /// Axis (point, unit direction) of the moving / target face.
    pub ax_m: (Vec3, Vec3),
    pub ax_t: (Vec3, Vec3),
}

// ------------------------------------------------------------ geometry

#[derive(Debug, Clone, Copy)]
pub enum G {
    Plane { o: Vec3, n: Vec3 },
    Cyl { o: Vec3, d: Vec3, r: f64 },
    Cone { o: Vec3, d: Vec3 },
    Sphere { c: Vec3, r: f64 },
}

impl G {
    fn dir(&self) -> Option<Vec3> {
        match *self {
            G::Plane { n, .. } => Some(n),
            G::Cyl { d, .. } | G::Cone { d, .. } => Some(d),
            G::Sphere { .. } => None,
        }
    }
    fn pt(&self) -> Vec3 {
        match *self {
            G::Plane { o, .. } | G::Cyl { o, .. } | G::Cone { o, .. } => o,
            G::Sphere { c, .. } => c,
        }
    }
    fn is_axis(&self) -> bool {
        matches!(self, G::Cyl { .. } | G::Cone { .. })
    }
}

pub fn geo(b: &Body, id: &str) -> Option<G> {
    use ck_geom::surface::Surface;
    let f = find_ref(b, id)?;
    match &b.face(f).surface {
        Surface::Plane { frame } => Some(G::Plane { o: frame.o, n: plane_normal(frame, b.face(f).sense) }),
        Surface::Cylinder { frame, r } => Some(G::Cyl { o: frame.o, d: knorm(frame.z), r: *r }),
        Surface::Cone { frame, .. } => Some(G::Cone { o: frame.o, d: knorm(frame.z) }),
        Surface::Sphere { frame, r } => Some(G::Sphere { c: frame.o, r: *r }),
        _ => None,
    }
}

fn gerr(detail: &str) -> FeatureError {
    FeatureError::new(MATE_GEOMETRY).with("detail", Json::Str(detail.to_string()))
}

fn sgn(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else {
        1.0
    }
}

pub fn inv(x: &Xf) -> Xf {
    let r = x.r;
    let rt = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]];
    let it = Xf { r: rt, t: [0.0; 3] };
    let t = it.vec(Vec3::new(x.t[0], x.t[1], x.t[2]));
    Xf { r: rt, t: [-t.x, -t.y, -t.z] }
}

fn axis_rot(o: Vec3, d: Vec3, ang: f64) -> Xf {
    let mut x = Xf::rotation(d, ang);
    let ro = x.vec(o);
    x.t = [o.x - ro.x, o.y - ro.y, o.z - ro.z];
    x
}

/// Rotation of `p` relative to `p0` about the world direction `d`
/// (wrapped to `(-π, π]`).
fn dtheta(p: &Xf, p0: &Xf, d: Vec3) -> f64 {
    let dd = p.compose(&inv(p0));
    let e = d.any_perp().normalize();
    let re = dd.vec(e);
    d.dot(e.cross(re)).atan2(e.dot(re))
}

/// Displacement of the point `o` along `d` caused by `p ∘ p0⁻¹`.
fn dz(p: &Xf, p0: &Xf, o: Vec3, d: Vec3) -> f64 {
    let dd = p.compose(&inv(p0));
    d.dot(dd.point(o) - o)
}

/// Current axis of a face axis recorded at creation.
fn cur_axis(p: &Xf, p0: &Xf, ax: (Vec3, Vec3)) -> (Vec3, Vec3) {
    let dd = p.compose(&inv(p0));
    (dd.point(ax.0), dd.vec(ax.1).normalize())
}

fn reflect_xf(o: Vec3, n: Vec3) -> Xf {
    // p ↦ p − 2((p − o)·n) n
    let r = [
        1.0 - 2.0 * n.x * n.x,
        -2.0 * n.x * n.y,
        -2.0 * n.x * n.z,
        -2.0 * n.y * n.x,
        1.0 - 2.0 * n.y * n.y,
        -2.0 * n.y * n.z,
        -2.0 * n.z * n.x,
        -2.0 * n.z * n.y,
        1.0 - 2.0 * n.z * n.z,
    ];
    let m = Xf { r, t: [0.0; 3] };
    let mo = m.vec(o);
    Xf { r, t: [o.x - mo.x, o.y - mo.y, o.z - mo.z] }
}

// ------------------------------------------------------------- records

fn two_axes_concentric(gm: G, gt: G) -> Result<Vec<Rec>, FeatureError> {
    match (gm, gt) {
        (a, b) if a.is_axis() && b.is_axis() => {
            let (p, d) = (a.pt(), a.dir().unwrap());
            let (q, u) = (b.pt(), b.dir().unwrap());
            Ok(vec![Rec::Dir { a: d, b: u, s: sgn(d.dot(u)) }, Rec::PtLine { p, q, u }])
        }
        _ => Err(gerr("concentric faces must be cylindrical or conical")),
    }
}

/// Records of an extended mate for its mover (the body is at the current
/// pose).
pub fn recs_ext(asm: &super::mate::Assembly, b: &Body, m: &MateRec) -> Result<Vec<Rec>, FeatureError> {
    let e = m.ext.as_deref().ok_or_else(|| gerr("no extension"))?;
    let g = |id: &str| geo(b, id).ok_or_else(|| FeatureError::new(MATE_GEOMETRY));
    let gm = g(&m.moving)?;
    let gt = g(&m.target)?;
    let flip = m.flip;
    Ok(match m.kind {
        K::Perpendicular => match (gm, gt) {
            (a, t) if a.dir().is_some() && t.dir().is_some() => {
                let (da, dt) = (a.dir().unwrap(), t.dir().unwrap());
                if matches!(a, G::Plane { .. }) == matches!(t, G::Plane { .. }) {
                    vec![Rec::DirDot { a: da, b: dt, c: 0.0 }]
                } else {
                    // an axis perpendicular to a plane lies along its normal
                    vec![Rec::Dir { a: da, b: dt, s: sgn(da.dot(dt)) }]
                }
            }
            _ => return Err(gerr("perpendicular needs planes or axes")),
        },
        K::Angle => match (gm.dir(), gt.dir()) {
            (Some(a), Some(t)) => {
                let th = e.angle.unwrap_or(0.0);
                let th = if flip { std::f64::consts::PI - th } else { th };
                vec![Rec::DirDot { a, b: t, c: th.cos() }]
            }
            _ => return Err(gerr("angle needs planes or axes")),
        },
        K::Tangent => tangent(gm, gt, flip, if e.side == 0.0 { 1.0 } else { e.side })?,
        K::Lock => {
            let cr = e.creation.as_ref().ok_or_else(|| gerr("lock without a creation pose"))?;
            let rel = inv(&cr.pt0).compose(&cr.pm0);
            let want = asm.components[e.tc].place.compose(&rel);
            let cur = asm.components[m.mover].place;
            let (ex, ey) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
            let o = Vec3::new(cur.t[0], cur.t[1], cur.t[2]);
            let w = Vec3::new(want.t[0], want.t[1], want.t[2]);
            vec![
                Rec::Dir { a: cur.vec(ex), b: want.vec(ex), s: 1.0 },
                Rec::Dir { a: cur.vec(ey), b: want.vec(ey), s: 1.0 },
                Rec::PtPlane { p: o, q: w, n: ex, d: 0.0 },
                Rec::PtPlane { p: o, q: w, n: ey, d: 0.0 },
                Rec::PtPlane { p: o, q: w, n: Vec3::new(0.0, 0.0, 1.0), d: 0.0 },
            ]
        }
        K::Symmetric => {
            let cid = e.c.as_deref().ok_or_else(|| FeatureError::new(regen::codes::MISSING_REFERENCE))?;
            let G::Plane { o: oc, n: nc } = g(cid)? else { return Err(gerr("symmetric: c must be a planar face")) };
            let mir = reflect_xf(oc, nc);
            match (gm, gt) {
                (G::Plane { o: p, n: a }, G::Plane { o: q, n: t }) => {
                    let (mq, mt) = (mir.point(q), mir.vec(t));
                    vec![Rec::Dir { a, b: mt, s: 1.0 }, Rec::PtPlane { p, q: mq, n: mt, d: 0.0 }]
                }
                (G::Cyl { o: p, d: a, .. }, G::Cyl { o: q, d: t, .. }) => {
                    let (mq, mt) = (mir.point(q), mir.vec(t));
                    vec![Rec::Dir { a, b: mt, s: sgn(a.dot(mt)) }, Rec::PtLine { p, q: mq, u: mt }]
                }
                (G::Sphere { c: p, .. }, G::Sphere { c: q, .. }) => {
                    let mq = mir.point(q);
                    (0..3)
                        .map(|i| {
                            let mut n = [0.0; 3];
                            n[i] = 1.0;
                            Rec::PtPlane { p, q: mq, n: Vec3::from_array(n), d: 0.0 }
                        })
                        .collect()
                }
                _ => return Err(gerr("symmetric needs two planes, two cylinders or two spheres")),
            }
        }
        K::Width => {
            let (m2, t2) = (e.m2.as_deref(), e.t2.as_deref());
            let (Some(m2), Some(t2)) = (m2, t2) else { return Err(FeatureError::new(regen::codes::MISSING_REFERENCE)) };
            let (G::Plane { o: p1, n: n1 }, G::Plane { o: p2, n: n2 }) = (gm, g(m2)?) else { return Err(gerr("width: a and a2 must be planar")) };
            let (G::Plane { o: q1, n: t1 }, G::Plane { o: q2, n: t2n }) = (gt, g(t2)?) else { return Err(gerr("width: b and b2 must be planar")) };
            if n1.dot(n2).abs() < 1.0 - 1e-6 || t1.dot(t2n).abs() < 1.0 - 1e-6 {
                return Err(gerr("width: the two faces of a side must be parallel"));
            }
            let (mid_m, mid_t) = ((p1 + p2).scale(0.5), (q1 + q2).scale(0.5));
            vec![Rec::Dir { a: n1, b: t1, s: sgn(n1.dot(t1)) }, Rec::PtPlane { p: mid_m, q: mid_t, n: t1, d: 0.0 }]
        }
        K::Hinge => {
            let mut v = two_axes_concentric(gm, gt)?;
            if let (Some(m2), Some(t2)) = (e.m2.as_deref(), e.t2.as_deref()) {
                let (G::Plane { o: p, .. }, G::Plane { o: q, n }) = (g(m2)?, g(t2)?) else { return Err(gerr("hinge: a2 and b2 (axial stop) must be planar")) };
                v.push(Rec::PtPlane { p, q, n, d: m.distance });
            }
            v
        }
        K::Slot => {
            let G::Cyl { o: p, d, r } = gm else { return Err(gerr("slot: a must be the pin (cylindrical face)")) };
            let G::Plane { o: q, n } = gt else { return Err(gerr("slot: b must be a planar slot wall")) };
            let mut v = vec![Rec::DirDot { a: d, b: n, c: 0.0 }];
            match e.t2.as_deref() {
                Some(t2) => {
                    let G::Plane { o: q2, .. } = g(t2)? else { return Err(gerr("slot: b2 must be a planar slot wall")) };
                    v.push(Rec::PtPlane { p, q: (q + q2).scale(0.5), n, d: 0.0 });
                }
                None => {
                    let side = if e.side == 0.0 { 1.0 } else { e.side };
                    v.push(Rec::PtPlane { p, q, n, d: side * r });
                }
            }
            v
        }
        K::Screw => two_axes_concentric(gm, gt)?,
        K::Gear => {
            if !gm.is_axis() || !gt.is_axis() {
                return Err(gerr("gear: a and b must be cylindrical faces (the gear axes)"));
            }
            vec![]
        }
        K::RackPinion => {
            // mover = the rack (a or b follows the driver), target = the pinion
            if !matches!(gt, G::Cyl { .. }) {
                return Err(gerr("rackPinion: a must be the pinion (cylindrical face)"));
            }
            if !(gm.is_axis() || (matches!(gm, G::Plane { .. }) && e.dir.is_some())) {
                return Err(gerr("rackPinion: b must be a cylindrical rack or a planar face with `dir`"));
            }
            vec![]
        }
        K::Limits => {
            if e.of_angle {
                if gm.dir().is_none() || gt.dir().is_none() {
                    return Err(gerr("limits (angle) needs planes or axes"));
                }
            } else if !matches!((gm, gt), (G::Plane { .. }, G::Plane { .. })) {
                return Err(gerr("limits (distance) needs two planar faces"));
            }
            vec![]
        }
        _ => return Err(FeatureError::new(MATE_GEOMETRY)),
    })
}

/// Side of the plane (`+1` along its normal) the other face starts on.
pub fn side_of(gm: G, gt: G) -> f64 {
    match (gm, gt) {
        (G::Cyl { o: p, .. }, G::Plane { o: q, n }) | (G::Sphere { c: p, .. }, G::Plane { o: q, n }) => sgn(n.dot(p - q)),
        (G::Plane { o: p, n }, G::Cyl { o: q, .. }) | (G::Plane { o: p, n }, G::Sphere { c: q, .. }) => sgn(n.dot(q - p)),
        _ => 1.0,
    }
}

fn tangent(gm: G, gt: G, flip: bool, side: f64) -> Result<Vec<Rec>, FeatureError> {
    // flip: internal contact (radius difference) instead of external (sum)
    let dist = |r1: f64, r2: f64| if flip { (r1 - r2).abs() } else { r1 + r2 };
    Ok(match (gm, gt) {
        (G::Cyl { o: p, d, r }, G::Plane { o: q, n }) => vec![Rec::DirDot { a: d, b: n, c: 0.0 }, Rec::PtPlane { p, q, n, d: side * r }],
        (G::Plane { o: p, n }, G::Cyl { o: q, d, r }) => vec![Rec::DirDot { a: n, b: d, c: 0.0 }, Rec::PlanePt { p, n, q, d: side * r }],
        (G::Sphere { c: p, r }, G::Plane { o: q, n }) => vec![Rec::PtPlane { p, q, n, d: side * r }],
        (G::Plane { o: p, n }, G::Sphere { c: q, r }) => vec![Rec::PlanePt { p, n, q, d: side * r }],
        (G::Cyl { o: p, d, r: r1 }, G::Cyl { o: q, d: u, r: r2 }) => {
            let want = dist(r1, r2);
            let contact = if want < 1e-9 { Rec::PtLine { p, q, u } } else { Rec::LineDist { p, q, u, d: want } };
            vec![Rec::Dir { a: d, b: u, s: sgn(d.dot(u)) }, contact]
        }
        (G::Sphere { c: p, r: r1 }, G::Sphere { c: q, r: r2 }) => {
            let want = dist(r1, r2);
            if want < 1e-9 {
                (0..3)
                    .map(|i| {
                        let mut n = [0.0; 3];
                        n[i] = 1.0;
                        Rec::PtPlane { p, q, n: Vec3::from_array(n), d: 0.0 }
                    })
                    .collect()
            } else {
                vec![Rec::PtDist { p, q, d: want }]
            }
        }
        (G::Sphere { c: p, r: r1 }, G::Cyl { o: q, d: u, r: r2 }) => {
            let want = dist(r1, r2);
            vec![if want < 1e-9 { Rec::PtLine { p, q, u } } else { Rec::LineDist { p, q, u, d: want } }]
        }
        (G::Cyl { .. }, G::Sphere { .. }) => return Err(gerr("tangent: put the sphere in a and the cylinder in b")),
        _ => return Err(gerr("tangent supports plane, cylinder and sphere faces")),
    })
}

// ------------------------------------------------------------ execution

fn comp_scale(asm: &super::mate::Assembly, b: &Body, k: usize) -> (Vec3, f64) {
    match comp_box(b, &asm.components[k].range) {
        Some((lo, hi)) => ((lo + hi).scale(0.5), lo.dist(hi) * 0.5),
        None => (Vec3::ZERO, 0.0),
    }
}

/// `mate` of an extended kind.
pub fn exec_ext(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Mate { kind, a, b, flip, a2, b2, c, .. } = &f.kind else { unreachable!() };
    let s = |r: &Option<super::Ref>| r.as_ref().map(|r| r.as_str().to_string());
    let (ra, rb) = (s(a), s(b));
    let ca = ra.as_deref().and_then(|r| br.component_of(r));
    let cb = rb.as_deref().and_then(|r| br.component_of(r));
    let (Some(ca), Some(cb)) = (ca, cb) else { return Err(FeatureError::new(regen::codes::MISSING_REFERENCE)) };
    if ca == cb {
        return Err(FeatureError::new(MATE_SAME_BODY));
    }
    let (ra, rb) = (ra.unwrap(), rb.unwrap());
    let (ra2, rb2, rc) = (s(a2), s(b2), s(c));
    for r in [&ra2, &rb2, &rc].into_iter().flatten() {
        if br.component_of(r).is_none() {
            return Err(FeatureError::new(regen::codes::MISSING_REFERENCE));
        }
    }
    let (fa, fb) = (br.asm.components[ca].fixed, br.asm.components[cb].fixed);
    if fa && fb {
        return Err(FeatureError::new(MATE_FIXED));
    }
    let present = |k: &str| f.raw.get(k).is_some();
    let opt = |k: &str, state: &RegenState| -> Result<Option<f64>, FeatureError> {
        if present(k) {
            Ok(Some(state.num(f, k, 0.0)?)).and_then(|v: Option<f64>| match v {
                Some(x) if !x.is_finite() => Err(FeatureError::new(regen::codes::BAD_EXPRESSION)),
                v => Ok(v),
            })
        } else {
            Ok(None)
        }
    };
    let distance = state.num(f, "distance", 0.0)?;
    let angle = opt("angle", state)?.map(f64::to_radians);
    let ratio = state.num(f, "ratio", 1.0)?;
    let pitch = state.num(f, "pitch", 1.0)?;
    let radius = state.num(f, "radius", 1.0)?;
    let min = opt("min", state)?;
    let max = opt("max", state)?;
    let of_angle = f.raw.get("of").and_then(|v| v.as_str()) == Some("angle");
    let dir = f.raw.get("dir").and_then(Json::as_vec3).map(Vec3::from_array).and_then(|v| v.try_normalize());
    if matches!(kind, K::Gear) && !(ratio.is_finite() && ratio > 0.0) {
        return Err(gerr("gear: ratio must be positive"));
    }
    if matches!(kind, K::RackPinion) && !(radius.is_finite() && radius > 0.0) {
        return Err(gerr("rackPinion: radius must be positive"));
    }
    if matches!(kind, K::Screw) && !pitch.is_finite() {
        return Err(gerr("screw: bad pitch"));
    }
    if matches!(kind, K::Limits) {
        if min.is_none() && max.is_none() {
            return Err(gerr("limits: give min and/or max"));
        }
        if let (Some(lo), Some(hi)) = (min, max) {
            if lo > hi {
                return Err(gerr("limits: min is above max"));
            }
        }
    }
    let coupling = matches!(kind, K::Gear | K::RackPinion | K::Screw);
    // roles
    let (mover, moving, target, tc, m2, t2, swapped) = if coupling {
        // driver = a, follower = b; the follower must float
        if fb {
            if matches!(kind, K::RackPinion) {
                return Err(FeatureError::new(MATE_FIXED).with("detail", Json::Str("rackPinion: the rack (b) must float".into())));
            }
            (ca, ra, rb, cb, ra2, rb2, true)
        } else {
            (cb, rb, ra, ca, rb2, ra2, false)
        }
    } else if fa {
        (cb, rb, ra, ca, rb2, ra2, false)
    } else {
        (ca, ra, rb, cb, ra2, rb2, false)
    };
    let mut ext = Ext {
        m2,
        t2,
        c: rc,
        tc,
        angle,
        ratio: if swapped && matches!(kind, K::Gear) { 1.0 / ratio } else { ratio },
        pitch,
        radius,
        min,
        max,
        of_angle,
        dir,
        swapped,
        side: 0.0,
        creation: None,
    };
    let body = br.body.as_ref().unwrap();
    {
        let gm = geo(body, &moving).ok_or_else(|| gerr("unsupported face geometry"))?;
        let gt = geo(body, &target).ok_or_else(|| gerr("unsupported face geometry"))?;
        let axis = |g: G| g.dir().map(|d| (g.pt(), d)).unwrap_or((g.pt(), Vec3::new(0.0, 0.0, 1.0)));
        ext.side = side_of(gm, gt) * if *flip { -1.0 } else { 1.0 };
        if !coupling {
            ext.creation = Some(Creation { pm0: br.asm.components[mover].place, pt0: br.asm.components[tc].place, ax_m: axis(gm), ax_t: axis(gt) });
        }
    }
    let m = MateRec { id: f.id.clone(), kind: *kind, mover, moving, target, distance, flip: *flip, ext: Some(Box::new(ext)) };
    // validate the geometry before keeping it
    recs_ext(&br.asm, body, &m)?;
    // snapshot for a clean rollback on conflict
    let saved_body = br.body.clone();
    let saved_places: Vec<Xf> = br.asm.components.iter().map(|c| c.place).collect();
    br.asm.mates.push(m);
    let mut r = super::mate::solve_all(br, mover);
    if r.is_ok() && coupling {
        // couplings are measured from the pose the other mates of this one
        // (hinge-like records) left behind: capture it, then solve again
        let (mi, ai) = (br.asm.mates.len() - 1, br.asm.mates[br.asm.mates.len() - 1].clone());
        let b = br.body.as_ref().unwrap();
        let (gm, gt) = (geo(b, &ai.moving), geo(b, &ai.target));
        if let (Some(gm), Some(gt)) = (gm, gt) {
            let axis = |g: G| g.dir().map(|d| (g.pt(), d)).unwrap_or((g.pt(), Vec3::new(0.0, 0.0, 1.0)));
            let cr = Creation { pm0: br.asm.components[mover].place, pt0: br.asm.components[tc].place, ax_m: axis(gm), ax_t: axis(gt) };
            br.asm.mates[mi].ext.as_mut().unwrap().creation = Some(cr);
            r = super::mate::solve_all(br, mover);
        } else {
            r = Err(FeatureError::new(MATE_GEOMETRY));
        }
    }
    if let Err(mut err) = r {
        let new = br.asm.mates.pop().unwrap();
        br.body = saved_body;
        for (c, p) in br.asm.components.iter_mut().zip(saved_places) {
            c.place = p;
        }
        let with = record_conflict(br, &new, tc);
        if err.code == MATE_CONFLICT {
            err = err.with("mates", Json::Arr(std::iter::once(new.id.clone()).chain(with.iter().cloned()).map(Json::Str).collect()));
        }
        return Err(err);
    }
    super::mate::update_dof(br);
    Ok(())
}

/// Remember a rejected mate and the earlier mates on its components.
pub fn record_conflict(br: &mut Brep, new: &MateRec, tc: usize) -> Vec<String> {
    let touch = [new.mover, tc];
    let with: Vec<String> = br
        .asm
        .mates
        .iter()
        .filter(|o| touch.contains(&o.mover) || o.ext.as_ref().is_some_and(|x| touch.contains(&x.tc)) || br.component_of(&o.target).is_some_and(|c| touch.contains(&c)))
        .map(|o| o.id.clone())
        .collect();
    br.asm.conflicts.push((new.id.clone(), with.clone(), new.ext.is_some()));
    with
}

fn move_comp(br: &mut Brep, k: usize, x: &Xf) {
    let range = br.asm.components[k].range.clone();
    x.range(br.body.as_mut().unwrap(), &range);
    let c = &mut br.asm.components[k];
    c.place = x.compose(&c.place);
}

fn is_identity(x: &Xf) -> bool {
    let i = Xf::IDENTITY;
    x.r.iter().zip(i.r.iter()).all(|(a, b)| (a - b).abs() < 1e-11) && x.t.iter().all(|t| t.abs() < 1e-9)
}

/// Post steps of the mates whose mover is `k`; `true` when anything moved.
pub fn apply_posts(br: &mut Brep, k: usize) -> Result<bool, FeatureError> {
    let mut moved = false;
    for mi in 0..br.asm.mates.len() {
        let m = br.asm.mates[mi].clone();
        if m.mover != k || m.ext.is_none() {
            continue;
        }
        moved |= match m.kind {
            K::Gear | K::RackPinion | K::Screw => couple(br, &m)?,
            K::Limits => limit(br, &m)?,
            _ => false,
        };
    }
    Ok(moved)
}

fn couple(br: &mut Brep, m: &MateRec) -> Result<bool, FeatureError> {
    let e = m.ext.as_deref().unwrap();
    // (the first solve of a new coupling runs before its creation pose exists)
    let Some(cr) = e.creation else { return Ok(false) };
    let (drv, fol) = (e.tc, m.mover);
    let pd = br.asm.components[drv].place;
    let mut changed = false;
    match m.kind {
        K::Gear => {
            let (ad, af) = (cur_axis(&pd, &cr.pt0, cr.ax_t), cur_axis(&br.asm.components[fol].place, &cr.pm0, cr.ax_m));
            let ds = sgn(cr.ax_t.1.dot(cr.ax_m.1));
            let k = if m.flip { 1.0 } else { -1.0 } * ds / e.ratio;
            let target_f;
            if let Some(phi) = e.angle {
                if !e.swapped {
                    if br.asm.components[drv].fixed {
                        return Err(FeatureError::new(MATE_FIXED));
                    }
                    let cur = dtheta(&pd, &cr.pt0, cr.ax_t.1);
                    let diff = wrap_pi(phi - cur);
                    if diff.abs() > 1e-10 {
                        move_comp(br, drv, &axis_rot(ad.0, ad.1, diff));
                        changed = true;
                    }
                    target_f = k * phi;
                } else {
                    // the natural follower is fixed: the drive turns `a` itself
                    target_f = phi;
                }
            } else {
                target_f = k * dtheta(&pd, &cr.pt0, cr.ax_t.1);
            }
            let cur = dtheta(&br.asm.components[fol].place, &cr.pm0, cr.ax_m.1);
            let diff = wrap_pi(target_f - cur);
            if diff.abs() > 1e-10 {
                move_comp(br, fol, &axis_rot(af.0, af.1, diff));
                changed = true;
            }
        }
        K::RackPinion => {
            let u = match geo_dir(br, &m.moving, e) {
                Some(u) => u,
                None => return Err(gerr("rackPinion: rack direction")),
            };
            let k = if m.flip { -1.0 } else { 1.0 } * e.radius;
            let ad = cur_axis(&pd, &cr.pt0, cr.ax_t);
            let theta = if let Some(phi) = e.angle {
                if br.asm.components[drv].fixed {
                    return Err(FeatureError::new(MATE_FIXED));
                }
                let cur = dtheta(&pd, &cr.pt0, cr.ax_t.1);
                let diff = wrap_pi(phi - cur);
                if diff.abs() > 1e-10 {
                    move_comp(br, drv, &axis_rot(ad.0, ad.1, diff));
                    changed = true;
                }
                phi
            } else {
                dtheta(&pd, &cr.pt0, cr.ax_t.1)
            };
            let want = k * theta;
            let cur = dz(&br.asm.components[fol].place, &cr.pm0, cr.ax_m.0, u);
            let diff = want - cur;
            if diff.abs() > 1e-9 {
                move_comp(br, fol, &Xf::translation(u.scale(diff)));
                changed = true;
            }
        }
        K::Screw => {
            if let Some(phi) = e.angle {
                let sg = if e.swapped { -1.0 } else { 1.0 };
                let lh = if m.flip { -1.0 } else { 1.0 };
                let (o, d) = cr.ax_t;
                let mut s = axis_rot(o, d, sg * phi);
                let z = sg * lh * e.pitch * phi / std::f64::consts::TAU;
                s.t = [s.t[0] + d.x * z, s.t[1] + d.y * z, s.t[2] + d.z * z];
                let dref = pd.compose(&inv(&cr.pt0));
                let want = dref.compose(&s).compose(&cr.pm0);
                let delta = want.compose(&inv(&br.asm.components[fol].place));
                if !is_identity(&delta) {
                    move_comp(br, fol, &delta);
                    changed = true;
                }
            }
        }
        _ => {}
    }
    Ok(changed)
}

/// Travel direction of a rack (unit): the cylinder axis, or `dir`.
fn geo_dir(br: &Brep, moving: &str, e: &Ext) -> Option<Vec3> {
    if let Some(d) = e.dir {
        return Some(d);
    }
    // the rack is the follower: its face is `moving`
    let g = geo(br.body.as_ref()?, moving)?;
    g.dir().filter(|_| g.is_axis())
}

fn measure(b: &Body, m: &MateRec, e: &Ext) -> Option<(f64, Vec3, Vec3, Vec3)> {
    let gm = geo(b, &m.moving)?;
    let gt = geo(b, &m.target)?;
    if e.of_angle {
        let (a, t) = (gm.dir()?, gt.dir()?);
        let ang = a.dot(t).clamp(-1.0, 1.0).acos();
        Some((ang, a, t, gm.pt()))
    } else {
        let (G::Plane { o: p, .. }, G::Plane { o: q, n }) = (gm, gt) else { return None };
        Some((n.dot(p - q), n, n, p))
    }
}

fn limit(br: &mut Brep, m: &MateRec) -> Result<bool, FeatureError> {
    let e = m.ext.as_deref().unwrap();
    let b = br.body.as_ref().unwrap();
    let Some((v, a, t, pivot)) = measure(b, m, e) else { return Err(FeatureError::new(MATE_GEOMETRY)) };
    let (lo, hi) = if e.of_angle { (e.min.map(f64::to_radians), e.max.map(f64::to_radians)) } else { (e.min, e.max) };
    let want = match (lo, hi) {
        (Some(l), _) if v < l - 1e-9 => l,
        (_, Some(h)) if v > h + 1e-9 => h,
        _ => return Ok(false),
    };
    let x = if e.of_angle {
        // rotating `a` about a × t by α closes the angle by α
        let mut k = a.cross(t);
        if k.len() < 1e-9 {
            k = a.any_perp();
        }
        axis_rot(pivot, k.normalize(), v - want)
    } else {
        Xf::translation(t.scale(want - v))
    };
    move_comp(br, m.mover, &x);
    Ok(true)
}

// -------------------------------------------------------------- report

/// Per-mate residual template at the current pose.
enum Tm {
    Recs(Vec<Rec>, bool),
    Mirror(Vec<Rec>, Xf),
    Couple,
    /// Records plus a coupling (screw).
    Both(Vec<Rec>),
    Limit(Option<f64>, Option<f64>),
}

fn twist(c: Vec3, col: usize, h: f64, l: f64) -> Xf {
    if col < 3 {
        let mut ax = [0.0; 3];
        ax[col] = 1.0;
        axis_rot(c, Vec3::from_array(ax), h / l)
    } else {
        let mut t = [0.0; 3];
        t[col - 3] = h;
        Xf { t, ..Xf::IDENTITY }
    }
}

fn map_rec(r: &Rec, fm: &Xf, ft: &Xf, mirror: Option<&Xf>, keep_n: bool) -> Rec {
    // the mirror conjugates the target motion: M ∘ ft ∘ M
    let ftt = match mirror {
        Some(mi) => mi.compose(ft).compose(mi),
        None => *ft,
    };
    let (mp, mv) = (|p: Vec3| fm.point(p), |v: Vec3| fm.vec(v));
    let (tp, tv) = (|p: Vec3| ftt.point(p), |v: Vec3| ftt.vec(v));
    match *r {
        Rec::Dir { a, b, s } => Rec::Dir { a: mv(a), b: tv(b), s },
        Rec::PtPlane { p, q, n, d } => Rec::PtPlane { p: mp(p), q: tp(q), n: if keep_n { n } else { tv(n) }, d },
        Rec::PtLine { p, q, u } => Rec::PtLine { p: mp(p), q: tp(q), u: tv(u) },
        Rec::DirDot { a, b, c } => Rec::DirDot { a: mv(a), b: tv(b), c },
        Rec::PtDist { p, q, d } => Rec::PtDist { p: mp(p), q: tp(q), d },
        Rec::LineDist { p, q, u, d } => Rec::LineDist { p: mp(p), q: tp(q), u: tv(u), d },
        Rec::PlanePt { p, n, q, d } => Rec::PlanePt { p: mp(p), n: mv(n), q: tp(q), d },
    }
}

fn eval_recs(recs: &[Rec], l: f64) -> Vec<f64> {
    let mut out = vec![];
    for r in recs {
        let mut rs: Vec<Row> = vec![];
        rows(r, &Xf::IDENTITY.r, Vec3::ZERO, Vec3::ZERO, &mut rs);
        let ang = matches!(r, Rec::Dir { .. } | Rec::DirDot { .. });
        out.extend(rs.iter().map(|x| if ang { x.r * l } else { x.r }));
    }
    out
}

fn residual(asm: &super::mate::Assembly, b: &Body, m: &MateRec, tm: &Tm, dm: &Xf, dt: &Xf, l: f64) -> Vec<f64> {
    match tm {
        Tm::Recs(recs, keep_n) => eval_recs(&recs.iter().map(|r| map_rec(r, dm, dt, None, *keep_n)).collect::<Vec<_>>(), l),
        Tm::Mirror(recs, mi) => eval_recs(&recs.iter().map(|r| map_rec(r, dm, dt, Some(mi), false)).collect::<Vec<_>>(), l),
        Tm::Both(recs) => {
            let mut v = eval_recs(&recs.iter().map(|r| map_rec(r, dm, dt, None, false)).collect::<Vec<_>>(), l);
            v.extend(residual(asm, b, m, &Tm::Couple, dm, dt, l));
            v
        }
        Tm::Couple => {
            let e = m.ext.as_deref().unwrap();
            let cr = e.creation.unwrap();
            let (drv, fol) = (e.tc, m.mover);
            let pd = dt.compose(&asm.components[drv].place);
            let pf = dm.compose(&asm.components[fol].place);
            let mut out = vec![];
            match m.kind {
                K::Gear => {
                    let ds = sgn(cr.ax_t.1.dot(cr.ax_m.1));
                    let k = if m.flip { 1.0 } else { -1.0 } * ds / e.ratio;
                    out.push((dtheta(&pf, &cr.pm0, cr.ax_m.1) - k * dtheta(&pd, &cr.pt0, cr.ax_t.1)) * l);
                    if let (Some(phi), false) = (e.angle, e.swapped) {
                        out.push(wrap_pi(dtheta(&pd, &cr.pt0, cr.ax_t.1) - phi) * l);
                    }
                }
                K::RackPinion => {
                    let u = geo_dir_body(b, &m.moving, e).unwrap_or(Vec3::new(1.0, 0.0, 0.0));
                    let k = if m.flip { -1.0 } else { 1.0 } * e.radius;
                    out.push(dz(&pf, &cr.pm0, cr.ax_m.0, u) - k * dtheta(&pd, &cr.pt0, cr.ax_t.1));
                    if let Some(phi) = e.angle {
                        out.push(wrap_pi(dtheta(&pd, &cr.pt0, cr.ax_t.1) - phi) * l);
                    }
                }
                K::Screw => {
                    let lh = if m.flip { -1.0 } else { 1.0 };
                    let d = cr.ax_t.1;
                    let rel_z = dz(&pf, &cr.pm0, cr.ax_m.0, d) - dz(&pd, &cr.pt0, cr.ax_t.0, d);
                    let rel_t = wrap_pi(dtheta(&pf, &cr.pm0, d) - dtheta(&pd, &cr.pt0, d));
                    out.push(rel_z - lh * e.pitch * rel_t / std::f64::consts::TAU);
                    if let Some(phi) = e.angle {
                        let sg = if e.swapped { -1.0 } else { 1.0 };
                        out.push(wrap_pi(rel_t - sg * phi) * l);
                    }
                }
                _ => {}
            }
            out
        }
        Tm::Limit(lo, hi) => {
            let e = m.ext.as_deref().unwrap();
            // the measured value under the motion: transform the face data
            let (Some(gm), Some(gt)) = (geo(b, &m.moving), geo(b, &m.target)) else { return vec![] };
            let v = if e.of_angle {
                let (a, t) = (dm.vec(gm.dir().unwrap()), dt.vec(gt.dir().unwrap()));
                a.dot(t).clamp(-1.0, 1.0).acos()
            } else {
                let (G::Plane { o: p, .. }, G::Plane { o: q, n }) = (gm, gt) else { return vec![] };
                dt.vec(n).dot(dm.point(p) - dt.point(q))
            };
            let mut out = vec![];
            if let Some(l0) = lo {
                if v <= l0 + 1e-7 {
                    out.push((v - l0) * if e.of_angle { l } else { 1.0 });
                }
            }
            if let Some(h0) = hi {
                if v >= h0 - 1e-7 {
                    out.push((v - h0) * if e.of_angle { l } else { 1.0 });
                }
            }
            out
        }
    }
}

fn geo_dir_body(b: &Body, moving: &str, e: &Ext) -> Option<Vec3> {
    if let Some(d) = e.dir {
        return Some(d);
    }
    let g = geo(b, moving)?;
    g.dir().filter(|_| g.is_axis())
}

/// Symmetric eigen decomposition (cyclic Jacobi): eigenvalues and the
/// eigenvectors as columns of the returned matrix.
fn jacobi(a: &mut Vec<Vec<f64>>) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = a.len();
    let mut v = vec![vec![0.0; n]; n];
    for i in 0..n {
        v[i][i] = 1.0;
    }
    for _ in 0..60 {
        let mut off = 0.0;
        for i in 0..n {
            for j in i + 1..n {
                off += a[i][j] * a[i][j];
            }
        }
        if off < 1e-26 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q].abs() < 1e-300 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = sgn(theta) / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (akp, akq) = (a[k][p], a[k][q]);
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p][k], a[q][k]);
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for k in 0..n {
                    let (vkp, vkq) = (v[k][p], v[k][q]);
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    ((0..n).map(|i| a[i][i]).collect(), v)
}

fn gram(j: &[Vec<f64>], n: usize) -> Vec<Vec<f64>> {
    let mut a = vec![vec![0.0; n]; n];
    for r in j {
        for i in 0..n {
            if r[i] == 0.0 {
                continue;
            }
            for k in 0..n {
                a[i][k] += r[i] * r[k];
            }
        }
    }
    a
}

fn rank_of(a: &[Vec<f64>]) -> usize {
    let mut m = a.to_vec();
    let (ev, _) = jacobi(&mut m);
    let mx = ev.iter().cloned().fold(0.0f64, f64::max);
    ev.iter().filter(|&&x| x > 1e-12 * mx.max(1e-6)).count()
}

/// The `assembly` report, plus the joint DOF written into `components[]`
/// when the assembly has extended mates. `force`: report even without them.
pub fn finalize(br: &mut Brep, force: bool) -> Option<Json> {
    let has_ext = br.asm.mates.iter().any(|m| m.ext.is_some()) || br.asm.conflicts.iter().any(|c| c.2);
    if !has_ext && !force {
        return None;
    }
    let body = br.body.as_ref()?;
    let asm = &br.asm;
    let n = asm.components.len();
    if n == 0 {
        return None;
    }
    // scene length for the rotation columns
    let mut lscale = 1.0f64;
    let mut centre = vec![Vec3::ZERO; n];
    for k in 0..n {
        let (c, s) = comp_scale(asm, body, k);
        centre[k] = c;
        lscale = lscale.max(s);
    }
    let col_of: Vec<Option<usize>> = {
        let mut next = 0;
        asm.components
            .iter()
            .map(|c| {
                if c.fixed {
                    None
                } else {
                    next += 1;
                    Some(next - 1)
                }
            })
            .collect()
    };
    let free = col_of.iter().flatten().count();
    let ncols = free * 6;
    // templates
    let mut tms: Vec<Option<Tm>> = vec![];
    for m in &asm.mates {
        let recs = if m.ext.is_some() { recs_ext(asm, body, m) } else { super::mate::mate_recs(body, m) };
        let tm = match (m.kind, recs) {
            (K::Gear | K::RackPinion | K::Screw, Ok(r)) => {
                // screw keeps its concentric records
                Some(if r.is_empty() { Tm::Couple } else { Tm::Both(r) })
            }
            (K::Limits, Ok(_)) => {
                let e = m.ext.as_deref().unwrap();
                Some(Tm::Limit(e.min.map(|v| if e.of_angle { v.to_radians() } else { v }), e.max.map(|v| if e.of_angle { v.to_radians() } else { v })))
            }
            (K::Symmetric, Ok(r)) => {
                let e = m.ext.as_deref().unwrap();
                match e.c.as_deref().and_then(|c| geo(body, c)) {
                    Some(G::Plane { o, n }) => Some(Tm::Mirror(r, reflect_xf(o, n))),
                    _ => None,
                }
            }
            (K::Lock, Ok(r)) => Some(Tm::Recs(r, true)),
            (_, Ok(r)) => Some(Tm::Recs(r, false)),
            _ => None,
        };
        tms.push(tm);
    }
    // numeric Jacobian, row blocks in mate order
    let mut jrows: Vec<Vec<f64>> = vec![];
    let mut blocks: Vec<(usize, usize)> = vec![];
    let h = 1e-6;
    for (mi, m) in asm.mates.iter().enumerate() {
        let start = jrows.len();
        let Some(tm) = &tms[mi] else {
            blocks.push((start, start));
            continue;
        };
        let tc = match &m.ext {
            Some(e) => e.tc,
            None => target_comp(br, m).unwrap_or(usize::MAX),
        };
        let id = Xf::IDENTITY;
        let base = residual(asm, body, m, tm, &id, &id, lscale);
        let mut cols: Vec<Vec<f64>> = vec![vec![0.0; base.len()]; ncols];
        for (who, is_m) in [(m.mover, true), (tc, false)] {
            let Some(Some(ci)) = col_of.get(who).copied() else { continue };
            for k in 0..6 {
                let (xp, xm) = (twist(centre[who], k, h, lscale), twist(centre[who], k, -h, lscale));
                let (rp, rm) = if is_m {
                    (residual(asm, body, m, tm, &xp, &id, lscale), residual(asm, body, m, tm, &xm, &id, lscale))
                } else {
                    (residual(asm, body, m, tm, &id, &xp, lscale), residual(asm, body, m, tm, &id, &xm, lscale))
                };
                if rp.len() != base.len() || rm.len() != base.len() {
                    continue;
                }
                for r in 0..base.len() {
                    cols[ci * 6 + k][r] += (rp[r] - rm[r]) / (2.0 * h);
                }
            }
        }
        for r in 0..base.len() {
            jrows.push((0..ncols).map(|c| cols[c][r]).collect());
        }
        blocks.push((start, jrows.len()));
    }
    let truncated = ncols > 180;
    let mut comp_rep: Vec<(usize, usize, usize)> = vec![(0, 0, 0); n]; // dof, trans, rot
    let (mut tot_t, mut tot_r, mut tot_d) = (0usize, 0usize, 0usize);
    let mut redundant: Vec<String> = vec![];
    if !truncated {
        let a = gram(&jrows, ncols);
        let mut am = a.clone();
        let (ev, vec_) = jacobi(&mut am);
        let mx = ev.iter().cloned().fold(0.0f64, f64::max);
        let null: Vec<usize> = (0..ncols).filter(|&i| ev[i] <= 1e-12 * mx.max(1e-6)).collect();
        // rotation part of each null vector, per component
        let sub_rank = |cols: &[usize], vs: &[usize]| -> usize {
            // rank of the (cols × vs) block of the null basis
            if vs.is_empty() || cols.is_empty() {
                return 0;
            }
            let mut g = vec![vec![0.0; vs.len()]; vs.len()];
            for &c in cols {
                for (i, &vi) in vs.iter().enumerate() {
                    for (j, &vj) in vs.iter().enumerate() {
                        g[i][j] += vec_[c][vi] * vec_[c][vj];
                    }
                }
            }
            rank_of(&g)
        };
        for k in 0..n {
            if let Some(ci) = col_of[k] {
                let all: Vec<usize> = (ci * 6..ci * 6 + 6).collect();
                let rotc: Vec<usize> = (ci * 6..ci * 6 + 3).collect();
                let d = sub_rank(&all, &null);
                let r = sub_rank(&rotc, &null).min(d);
                comp_rep[k] = (d, d - r, r);
            }
        }
        tot_d = null.len();
        let allrot: Vec<usize> = (0..free).flat_map(|c| c * 6..c * 6 + 3).collect();
        tot_r = sub_rank(&allrot, &null).min(tot_d);
        tot_t = tot_d - tot_r;
        // redundancy: rank gained by each mate vs its own rank
        if ncols <= 72 {
            let mut acc = vec![vec![0.0; ncols]; ncols];
            let mut prev = 0usize;
            for (mi, m) in asm.mates.iter().enumerate() {
                let (s, e) = blocks[mi];
                if s == e {
                    continue;
                }
                let g = gram(&jrows[s..e], ncols);
                let alone = rank_of(&g);
                for i in 0..ncols {
                    for k in 0..ncols {
                        acc[i][k] += g[i][k];
                    }
                }
                let now = rank_of(&acc);
                if now - prev < alone {
                    redundant.push(m.id.clone());
                }
                prev = now;
            }
        }
    }
    // joint DOF into components[] (extended assemblies only)
    if has_ext && !truncated {
        for (k, c) in br.asm.components.iter_mut().enumerate() {
            c.dof = if c.fixed { 0 } else { comp_rep[k].0 as u32 };
        }
    }
    let asm = &br.asm;
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| asm.components[a].id.cmp(&asm.components[b].id));
    let comps: Vec<Json> = order
        .into_iter()
        .map(|k| {
            let c = &asm.components[k];
            let (d, t, r) = comp_rep[k];
            Obj::new()
                .put("id", c.id.as_str())
                .put("fixed", Json::Bool(c.fixed))
                .put("dof", if truncated { c.dof as f64 } else { d as f64 })
                .put("translational", t as f64)
                .put("rotational", r as f64)
                .build()
        })
        .collect();
    let conflicts: Vec<Json> = asm
        .conflicts
        .iter()
        .map(|(id, with, _)| Obj::new().put("mate", id.as_str()).put("with", Json::Arr(with.iter().map(|s| Json::Str(s.clone())).collect())).build())
        .collect();
    let over = !redundant.is_empty() || !asm.conflicts.is_empty();
    let mut o = Obj::new()
        .put("dof", Obj::new().put("total", tot_d as f64).put("translational", tot_t as f64).put("rotational", tot_r as f64).build())
        .put("components", Json::Arr(comps))
        .put("overConstrained", Json::Bool(over))
        .put("redundant", Json::Arr(redundant.into_iter().map(Json::Str).collect()))
        .put("conflicts", Json::Arr(conflicts));
    if truncated {
        o = o.put("truncated", Json::Bool(true));
    }
    Some(o.build())
}

/// Component of the target face of an oracle-kind mate.
fn target_comp(br: &Brep, m: &MateRec) -> Option<usize> {
    br.component_of(&m.target)
}
