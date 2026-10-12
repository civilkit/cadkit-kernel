//! `sweep` (ck-ops/src/sweep.rs): sweep a planar profile along a path.
//!
//! Codes (kernel.strings): sweepPathInvalid, sweepProfileNotNormal,
//! sweepSelfIntersects, sweepFailed (guide codes live in guide.rs).
//!
//! Reconstructed from oracle probes plus the packet reading of f2892 / f626:
//! * the original reduces the path to a RIGID MOTION when it can (all lines
//!   along one direction -> translation; arcs of one circle / a helix ->
//!   rotation about an axis, with a rise along the axis for a helix) and
//!   builds the solid with ck-topo's sweep core (`build_sweep`): the same
//!   face / edge numbering as extrude / revolve (`C0` = the cap at the
//!   profile, `C1` = the moved copy; `S.<tag>` per profile entity; rails
//!   first, then the C0/C1 edges per segment);
//! * the path direction is irrelevant: the end of the path nearest to the
//!   profile starts the sweep (probe: a path drawn away from the profile
//!   gives the mirror image of a path drawn towards it); the path may start
//!   anywhere (the profile is moved by the motion, not placed on the path);
//! * `sweepProfileNotNormal`: the path tangent at the start must be
//!   perpendicular to the profile plane, `| |t·n| - 1 | <= 5e-5` (checked for
//!   lines and arcs; for a helix the check uses the rotational part of the
//!   motion, a profile in an axial plane is accepted at any pitch);
//! * a translation / rotation whose axis lies in the profile plane is
//!   delegated to ck-topo extrude / revolve (oracle pinned); a screw motion
//!   (helix) is built here: side faces are B-spline surfaces
//!   `S(φ, t) = M(φ) c(t)` (interpolated, `PER_REV` rows per revolution)
//!   except lines parallel to the axis, which are cylinders; rails are the
//!   B-spline vertex orbits, taken from the same interpolation so that the
//!   edges lie exactly on both faces.
//! * GUESS: general (non rigid) paths are not reconstructed -> sweepFailed.

use crate::boolean::{boolean, Op};
use crate::helix::Helix;
use ck_core::xform::Xform;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::fit::{averaged_knots, interpolate_with};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::Body;
use ck_topo::make::revolve::{revolve, rot_curve, rot_frame, rotation};
use ck_topo::make::{
    build_sweep_with, curve2_to_3d, extrude, sense_of, Caps, ExtrudeOpts, LoopGeom, Profile, ProfileLoop, ProfileRegion, ProfileSeg, SegGeom, SideGeom,
    VertGeom,
};

pub const PATH_INVALID: &str = "sweepPathInvalid";
pub const PROFILE_NOT_NORMAL: &str = "sweepProfileNotNormal";
pub const SELF_INTERSECTS: &str = "sweepSelfIntersects";
pub const FAILED: &str = "sweepFailed";

/// `| |t·n| - 1 |` above this: the profile is not normal to the path.
const NORMAL_TOL: f64 = 5e-5;
/// Rows per revolution of the interpolated screw surfaces / rails.
const PER_REV: f64 = 48.0;
const TOL: f64 = 1e-9;
/// Degree of the rotation-direction interpolation.
const DEG_U: usize = 5;

/// One piece of a sweep path (world coordinates).
#[derive(Debug, Clone)]
pub enum PathSeg {
    Line { a: Vec3, b: Vec3 },
    /// Circle arc `p(θ) = frame.o + r (x cos θ + y sin θ)`, θ from `a0` to
    /// `a1` (`a1 < a0`: clockwise about `frame.z`).
    Arc { frame: Frame, r: f64, a0: f64, a1: f64 },
    /// A helix feature; `rev` walks it from its end to its start.
    Helix { h: Helix, rev: bool },
}

impl PathSeg {
    pub fn start(&self) -> Vec3 {
        match self {
            PathSeg::Line { a, .. } => *a,
            PathSeg::Arc { frame, r, a0, .. } => arc_pt(frame, *r, *a0),
            PathSeg::Helix { h, rev } => h.eval(if *rev { h.angle } else { 0.0 }),
        }
    }
    pub fn end(&self) -> Vec3 {
        self.reversed().start()
    }
    pub fn reversed(&self) -> PathSeg {
        match self {
            PathSeg::Line { a, b } => PathSeg::Line { a: *b, b: *a },
            PathSeg::Arc { frame, r, a0, a1 } => PathSeg::Arc { frame: *frame, r: *r, a0: *a1, a1: *a0 },
            PathSeg::Helix { h, rev } => PathSeg::Helix { h: *h, rev: !rev },
        }
    }
    /// Unit direction of travel at the start (a helix: its rotational part,
    /// see the module header).
    fn start_dir(&self) -> Option<Vec3> {
        match self {
            PathSeg::Line { a, b } => (*b - *a).try_normalize(),
            PathSeg::Arc { frame, a0, a1, .. } => {
                let s = if a1 >= a0 { 1.0 } else { -1.0 };
                Some((frame.y * a0.cos() - frame.x * a0.sin()) * s)
            }
            PathSeg::Helix { h, rev } => {
                let t = if *rev { h.angle } else { 0.0 };
                let d = h.x * (-t.sin()) + h.y * t.cos();
                Some(if *rev { -d } else { d })
            }
        }
    }
}

fn arc_pt(f: &Frame, r: f64, t: f64) -> Vec3 {
    f.o + (f.x * t.cos() + f.y * t.sin()) * r
}

/// A rigid motion of space: translation, or rotation by `angle` about the
/// axis (`o`, unit `a`) with a rise `rise` along `a` (a screw).
#[derive(Debug, Clone, Copy)]
pub enum Motion {
    Translate(Vec3),
    Screw { o: Vec3, a: Vec3, angle: f64, rise: f64 },
}

impl Motion {
    fn inverse(self) -> Motion {
        match self {
            Motion::Translate(v) => Motion::Translate(-v),
            Motion::Screw { o, a, angle, rise } => Motion::Screw { o, a, angle: -angle, rise: -rise },
        }
    }
}

/// The rigid motion of a (oriented) path, None when the path is not one.
pub fn path_motion(path: &[PathSeg]) -> Option<Motion> {
    let first = path.first()?;
    match first {
        PathSeg::Line { .. } => {
            let d0 = first.start_dir()?;
            let mut v = Vec3::ZERO;
            for s in path {
                let PathSeg::Line { a, b } = s else { return None };
                let d = (*b - *a).try_normalize()?;
                if d.dot(d0) < 0.999999999 {
                    return None;
                }
                v = v + (*b - *a);
            }
            Some(Motion::Translate(v))
        }
        PathSeg::Arc { frame, r, .. } => {
            // consecutive arcs of one circle, all in the same sense
            let mut total = 0.0;
            let sense = |a0: f64, a1: f64| if a1 >= a0 { 1.0 } else { -1.0 };
            let s0 = if let PathSeg::Arc { a0, a1, .. } = first { sense(*a0, *a1) } else { 1.0 };
            for s in path {
                let PathSeg::Arc { frame: f, r: rr, a0, a1 } = s else { return None };
                if (f.o - frame.o).len() > 1e-7 || f.z.dot(frame.z) < 0.999999999 || (rr - r).abs() > 1e-7 {
                    return None;
                }
                if sense(*a0, *a1) != s0 {
                    return None;
                }
                total += a1 - a0;
            }
            Some(Motion::Screw { o: frame.o, a: frame.z, angle: total, rise: 0.0 })
        }
        PathSeg::Helix { h, rev } => {
            if path.len() != 1 {
                return None;
            }
            // p(θ) rotates by σθ about `a` (σ = +1 when y = a × x) and rises k θ
            let sigma = if h.a.cross(h.x).dot(h.y) >= 0.0 { 1.0 } else { -1.0 };
            let m = Motion::Screw { o: h.o, a: h.a, angle: sigma * h.angle, rise: h.k * h.angle };
            Some(if *rev { m.inverse() } else { m })
        }
    }
}

/// Sweep `profile` along `path` (a chain of connected pieces).
/// `twist`: total rotation (radians) of the profile about the path while it
/// travels (oracle probe: a straight path becomes a screw about the path
/// line, rotating by +twist about the travel direction).
pub fn sweep(feature: &str, profile: &Profile, path: &[PathSeg], twist: f64) -> Result<Body, String> {
    profile.validate()?;
    if path.is_empty() {
        return Err(PATH_INVALID.into());
    }
    // the end of the path nearest to the profile starts the sweep
    let q = profile_ref_point(profile);
    let ps = path[0].start();
    let pe = path[path.len() - 1].end();
    let oriented: Vec<PathSeg> = if pe.dist(q) < ps.dist(q) - 1e-9 {
        path.iter().rev().map(|s| s.reversed()).collect()
    } else {
        path.to_vec()
    };
    let t0 = oriented[0].start_dir().ok_or_else(|| PATH_INVALID.to_string())?;
    let n = profile.frame.z;
    if (t0.dot(n).abs() - 1.0).abs() > NORMAL_TOL {
        return Err(PROFILE_NOT_NORMAL.into());
    }
    // oracle: a path of several pieces is swept piece by piece and the
    // pieces united with mitre joints (strings 'mitre', 'half-space block',
    // 'the pieces did not unite'; f2329 / f2894 / f2331)
    let Some(motion) = path_motion(&oriented) else {
        return sweep_pieces(feature, profile, &oriented, twist);
    };
    if twist != 0.0 {
        let Motion::Translate(v) = motion else {
            return Err("TODO(rebuild): twisted sweep along a curved path".to_string());
        };
        let len = v.len();
        if len < TOL {
            return Err(PATH_INVALID.into());
        }
        return sweep_motion(feature, profile, Motion::Screw { o: oriented[0].start(), a: v * (1.0 / len), angle: twist, rise: len });
    }
    sweep_motion(feature, profile, motion)
}

/// Extrude `profile` along its normal over `[o.s0, o.s1]` while rotating it
/// by `twist` radians (total, about the axis through `centre`, a point of the
/// sketch plane in sketch coordinates, parallel to the normal). The rotation
/// is zero at the sketch plane (s = 0), so a flipped / mid extrude twists on
/// either side of it. Side faces are screw (helicoidal) surfaces: exact
/// cylinders for circles about the axis, interpolated B-splines otherwise.
/// Names follow the plain extrude (`F.S.<tag>`, caps `F.C0` / `F.C1`; a closed
/// off-axis loop is split into `~1` / `~2` halves like the general sweeps).
pub fn twist_extrude(feature: &str, profile: &Profile, o: ExtrudeOpts, twist: f64, centre: Vec2) -> Result<Body, String> {
    profile.validate()?;
    let fr = profile.frame;
    let (s0, s1) = (o.s0.min(o.s1), o.s0.max(o.s1));
    let depth = s1 - s0;
    if depth < TOL {
        return Err("zeroDepth".into());
    }
    if o.dir.map_or(false, |d| d.cross(fr.z).len() > 1e-9) {
        return Err("twistUnsupported".into());
    }
    if o.draft != 0.0 {
        return Err("draftUnsupported".into());
    }
    if twist.abs() < 1e-12 {
        return extrude(feature, profile, o);
    }
    let a = fr.z;
    let axis_pt = fr.o + fr.x * centre.x + fr.y * centre.y;
    // start the screw at s0: shift the profile there and pre-rotate it by the
    // twist accumulated between the sketch plane and s0
    let phi0 = twist * s0 / depth;
    let rot0 = rotation(axis_pt, a, phi0);
    let shifted = rot_frame(&rot0, &Frame { o: fr.o + a * s0, ..fr });
    let pr = Profile::new(shifted, profile.regions.clone());
    let o_axis = axis_pt + a * s0;
    if twist < 0.0 {
        screw_with(feature, &pr, o_axis, -a, -twist, -depth, false)
    } else {
        screw_with(feature, &pr, o_axis, a, twist, depth, false)
    }
}

/// Mean of the profile's vertices (world).
fn profile_ref_point(p: &Profile) -> Vec3 {
    let f = &p.frame;
    let (mut s, mut k) = (Vec3::ZERO, 0.0);
    for r in &p.regions {
        for l in r.loops() {
            for g in &l.segs {
                let v = g.start();
                s = s + f.o + f.x * v.x + f.y * v.y;
                k += 1.0;
            }
        }
    }
    if k == 0.0 {
        f.o
    } else {
        s * (1.0 / k)
    }
}

/// Build the swept solid for a rigid `motion`.
pub fn sweep_motion(feature: &str, profile: &Profile, motion: Motion) -> Result<Body, String> {
    let fr = profile.frame;
    match motion {
        Motion::Translate(v) => {
            let len = v.len();
            if len < TOL {
                return Err(PATH_INVALID.into());
            }
            let d = v * (1.0 / len);
            let (s0, s1) = if d.dot(fr.z) >= 0.0 { (0.0, len) } else { (-len, 0.0) };
            extrude(feature, profile, ExtrudeOpts { s0, s1, dir: Some(d), draft: 0.0 })
        }
        Motion::Screw { o, a, angle, rise } => {
            if angle.abs() < TOL {
                return Err(PATH_INVALID.into());
            }
            let (a, angle, rise) = if angle < 0.0 { (-a, -angle, -rise) } else { (a, angle, rise) };
            let in_plane = fr.z.dot(a).abs() < TOL && fr.z.dot(o - fr.o).abs() < 1e-6;
            if rise.abs() < TOL && in_plane {
                return revolve(feature, profile, o, a, 0.0, angle);
            }
            screw(feature, profile, o, a, angle, rise)
        }
    }
}

// ------------------------------------------------------------- screw sweep

/// `M(φ)`: rotation by `φ` about (`o`, `a`) plus the rise `rise φ / angle`.
#[derive(Clone, Copy)]
struct Screw {
    o: Vec3,
    a: Vec3,
    angle: f64,
    rise: f64,
}

impl Screw {
    fn xf(&self, phi: f64) -> Xform {
        let r = rotation(self.o, self.a, phi);
        Xform::new(r.r, r.t + self.a * (self.rise * phi / self.angle))
    }
    fn pt(&self, phi: f64, p: Vec3) -> Vec3 {
        self.xf(phi).apply_point(p)
    }
    /// Velocity of the motion at `p` (per radian).
    fn vel(&self, p: Vec3) -> Vec3 {
        self.a.cross(p - self.o) + self.a * (self.rise / self.angle)
    }
}

fn scaled_knots(u: &[f64], p: usize) -> Vec<f64> {
    let (u0, u1) = (u[0], u[u.len() - 1]);
    let un: Vec<f64> = u.iter().map(|x| (x - u0) / (u1 - u0)).collect();
    averaged_knots(&un, p).into_iter().map(|k| u0 + k * (u1 - u0)).collect()
}

/// Orbit of `p` under the screw as a cubic B-spline over φ ∈ [0, angle].
fn orbit(sc: &Screw, p: Vec3, phis: &[f64]) -> Result<BSplineCurve, String> {
    orbit_deg(sc, p, phis, DEG_U)
}

fn orbit_deg(sc: &Screw, p: Vec3, phis: &[f64], deg: usize) -> Result<BSplineCurve, String> {
    let pts: Vec<Vec3> = phis.iter().map(|&f| sc.pt(f, p)).collect();
    interpolate_with(&pts, deg, phis, scaled_knots(phis, deg))
}

/// Surface through `grid[i][j]` (i along φ, j along the profile parameter).
fn grid_surface(grid: &[Vec<Vec3>], u: &[f64], v: &[f64], pu: usize, pv: usize) -> Result<BSplineSurface, String> {
    let (nu, nv) = (grid.len(), grid[0].len());
    let (pu, pv) = (pu.min(nu - 1), pv.min(nv - 1));
    let ku = scaled_knots(u, pu);
    let kv = scaled_knots(v, pv);
    let mut r = vec![vec![Vec3::ZERO; nv]; nu];
    for j in 0..nv {
        let col: Vec<Vec3> = (0..nu).map(|i| grid[i][j]).collect();
        let c = interpolate_with(&col, pu, u, ku.clone())?;
        for i in 0..nu {
            r[i][j] = c.ctrl[i];
        }
    }
    let mut ctrl = Vec::with_capacity(nu * nv);
    for row in r.iter() {
        let c = interpolate_with(row, pv, v, kv.clone())?;
        ctrl.extend(c.ctrl);
    }
    BSplineSurface::new(pu, pv, ku, kv, nu, nv, ctrl, None)
}

fn screw(feature: &str, profile: &Profile, o: Vec3, a: Vec3, angle: f64, rise: f64) -> Result<Body, String> {
    screw_with(feature, profile, o, a, angle, rise, true)
}

/// [`screw`] with the cap-face numbering choice (`caps_last`: general
/// sweeps list sides first; the twisted extrude keeps the extrude order).
fn screw_with(feature: &str, profile: &Profile, o: Vec3, a: Vec3, angle: f64, rise: f64, caps_last: bool) -> Result<Body, String> {
    let a = a.normalize();
    let split = split_closed(profile, &|g: &ProfileSeg| on_axis_circle(&profile.frame, g, o, a));
    let profile = &split;
    let sc = Screw { o, a, angle, rise };
    let pf = &profile.frame;
    let n_rows = ((PER_REV * angle / std::f64::consts::TAU).ceil() as usize).max(8);
    let phis: Vec<f64> = (0..=n_rows).map(|i| angle * i as f64 / n_rows as f64).collect();
    let xf_end = sc.xf(angle);
    let world = |p: Vec2| pf.o + pf.x * p.x + pf.y * p.y;

    // sweep sense: velocity of the motion at a profile vertex vs plane normal
    let mut sigma = 0.0;
    'outer: for reg in &profile.regions {
        for l in reg.loops() {
            for s in &l.segs {
                let v = sc.vel(world(s.start())).dot(pf.z);
                if v.abs() > 1e-9 {
                    sigma = v;
                    break 'outer;
                }
            }
        }
    }
    if sigma == 0.0 {
        return Err(PROFILE_NOT_NORMAL.into());
    }

    let mut regions = vec![];
    let mut caps = vec![];
    for reg in &profile.regions {
        let mut lgs = vec![];
        for pl in reg.loops() {
            let m = pl.segs.len();
            let mut verts = vec![];
            let mut orbits: Vec<Option<BSplineCurve>> = vec![];
            for s in &pl.segs {
                let base = world(s.start());
                let top = xf_end.apply_point(base);
                let rad = (base - sc.o) - a * (base - sc.o).dot(a);
                let rail = if rad.len() < TOL {
                    // on the axis: the orbit is the axis line
                    let d = a * (rise / angle);
                    orbits.push(None);
                    Some((Curve::Line { o: base, d }, 0.0, angle))
                } else {
                    let b = orbit(&sc, base, &phis)?;
                    orbits.push(Some(b.clone()));
                    Some((Curve::BSpline(b), 0.0, angle))
                };
                verts.push(VertGeom { bottom: base, top, rail });
            }
            let mut segs = vec![];
            for (i, s) in pl.segs.iter().enumerate() {
                let base = curve2_to_3d(pf, &s.curve, Vec3::ZERO);
                let top = rot_curve(&xf_end, &base);
                let (vs, ve) = if s.rev { ((i + 1) % m, i) } else { (i, (i + 1) % m) };
                let side = side_geom(&sc, &phis, &base, s.t0, s.t1, &verts[vs], &verts[ve]).transpose()?.map(|mut g| {
                    let tm = 0.5 * (s.t0 + s.t1);
                    let t2 = s.travel_tangent(tm);
                    let outward = (pf.x * t2.x + pf.y * t2.y).cross(pf.z);
                    g.sense = sense_of(&g.surface, g.pc_bottom.eval(tm), outward);
                    g
                });
                segs.push(SegGeom {
                    bottom: (base, s.t0, s.t1),
                    top: (top, s.t0, s.t1),
                    cap0_pc: Some(s.curve.clone()),
                    cap1_pc: Some(s.curve.clone()),
                    side,
                });
            }
            lgs.push(LoopGeom { verts, segs });
        }
        regions.push(lgs);
        caps.push(Caps { c0: *pf, c1: rot_frame(&xf_end, pf) });
    }
    // general sweeps list the side faces before the caps (oracle)
    Ok(build_sweep_with(feature, profile, regions, Some(&caps), sigma > 0.0, false, caps_last))
}

/// A closed circle centred on the screw axis (and normal to it): the screw
/// maps it onto itself, so its side face is an exact cylinder.
fn on_axis_circle(fr: &Frame, g: &ProfileSeg, o: Vec3, a: Vec3) -> bool {
    let Curve2::Circle { c, .. } = &g.curve else { return false };
    let w = fr.o + fr.x * c.x + fr.y * c.y;
    g.is_closed() && (w - o).cross(a).len() < 1e-9 && fr.z.cross(a).len() < 1e-9
}

/// The oracle's general sweeps cut a closed single-entity loop (a circle)
/// into two half pieces of one entity: faces `S.<tag>~1`, `S.<tag>~2`
/// (numbered by `Body::number_split_faces`), two rails, two C0 / C1 edges.
fn split_closed(p: &Profile, keep: &dyn Fn(&ProfileSeg) -> bool) -> Profile {
    let half = |l: &ProfileLoop| -> ProfileLoop {
        if l.segs.len() != 1 || !l.segs[0].is_closed() || keep(&l.segs[0]) {
            return l.clone();
        }
        let s = &l.segs[0];
        let mid = 0.5 * (s.t0 + s.t1);
        let a = ProfileSeg { t1: mid, ..s.clone() };
        let b = ProfileSeg { t0: mid, ..s.clone() };
        // travel order: reversed entities walk from t1 down to t0
        ProfileLoop::new(if s.rev { vec![b, a] } else { vec![a, b] })
    };
    let regions = p.regions.iter().map(|r| ProfileRegion { outer: half(&r.outer), holes: r.holes.iter().map(&half).collect() }).collect();
    Profile::new(p.frame, regions)
}

/// Side surface of one profile segment (curve `c` over [t0, t1]).
/// Parameters: u = φ (rotation angle, 0..angle), v = t (profile parameter),
/// except cylinders (u = angle about the axis, v = height).
fn side_geom(sc: &Screw, phis: &[f64], c: &Curve, t0: f64, t1: f64, _va: &VertGeom, _vb: &VertGeom) -> Option<Result<SideGeom, String>> {
    let (pa, pb) = (c.eval(t0), c.eval(t1));
    let angle = sc.angle;
    let a = sc.a;
    // lines parallel to the axis -> cylinder
    if let Curve::Line { o: lo, d } = c {
        let radial = |p: Vec3| (p - sc.o) - a * (p - sc.o).dot(a);
        let (ra, rb) = (radial(pa), radial(pb));
        if d.cross(a).len() < 1e-9 && ra.len() > TOL && (ra.len() - rb.len()).abs() < 1e-9 {
            let x = ra.normalize();
            let frame = Frame { o: sc.o, x, y: a.cross(x), z: a };
            let f0 = (*lo - sc.o).dot(a);
            let f1 = d.dot(a);
            let slope = sc.rise / angle;
            let surface = Surface::Cylinder { frame, r: ra.len() };
            return Some(Ok(SideGeom {
                surface,
                sense: true,
                pc_bottom: Curve2::Line { o: Vec2::new(0.0, f0), d: Vec2::new(0.0, f1) },
                pc_top: Curve2::Line { o: Vec2::new(angle, f0 + sc.rise), d: Vec2::new(0.0, f1) },
                pc_rail_t0: Some(Curve2::Line { o: Vec2::new(0.0, f0 + f1 * t0), d: Vec2::new(1.0, slope) }),
                pc_rail_t1: Some(Curve2::Line { o: Vec2::new(0.0, f0 + f1 * t1), d: Vec2::new(1.0, slope) }),
            }));
        }
    }
    // circle about the axis -> cylinder (u = angle about the axis, v = height)
    if let Curve::Circle { frame: cf, r } = c {
        let radial = (cf.o - sc.o).cross(a).len();
        if radial < 1e-9 && cf.z.cross(a).len() < 1e-9 {
            let su = if cf.z.dot(a) > 0.0 { 1.0 } else { -1.0 };
            let frame = Frame { o: cf.o, x: cf.x, y: a.cross(cf.x), z: a };
            let slope = sc.rise / angle;
            let surface = Surface::Cylinder { frame, r: *r };
            return Some(Ok(SideGeom {
                surface,
                sense: true,
                pc_bottom: Curve2::Line { o: Vec2::ZERO, d: Vec2::new(su, 0.0) },
                pc_top: Curve2::Line { o: Vec2::new(angle, sc.rise), d: Vec2::new(su, 0.0) },
                pc_rail_t0: Some(Curve2::Line { o: Vec2::new(su * t0, 0.0), d: Vec2::new(1.0, slope) }),
                pc_rail_t1: Some(Curve2::Line { o: Vec2::new(su * t1, 0.0), d: Vec2::new(1.0, slope) }),
            }));
        }
    }
    // v samples along the profile curve
    let (vs, pv): (Vec<f64>, usize) = match c {
        Curve::Line { .. } => (vec![t0, t1], 1),
        _ => {
            let span = (t1 - t0).abs();
            let per = match c {
                Curve::Circle { .. } | Curve::Ellipse { .. } => std::f64::consts::FRAC_PI_2 / 12.0,
                _ => span / 24.0,
            };
            let n = ((span / per).ceil() as usize).clamp(8, 400);
            ((0..=n).map(|j| t0 + (t1 - t0) * j as f64 / n as f64).collect(), 3)
        }
    };
    let qs: Vec<Vec3> = vs.iter().map(|&t| c.eval(t)).collect();
    let grid: Vec<Vec<Vec3>> = phis.iter().map(|&f| qs.iter().map(|&q| sc.pt(f, q)).collect()).collect();
    let surf = match grid_surface(&grid, phis, &vs, DEG_U, pv) {
        Ok(s) => s,
        Err(e) => return Some(Err(e)),
    };
    Some(Ok(SideGeom {
        surface: Surface::BSpline(Box::new(surf)),
        sense: true,
        pc_bottom: Curve2::Line { o: Vec2::new(0.0, 0.0), d: Vec2::new(0.0, 1.0) },
        pc_top: Curve2::Line { o: Vec2::new(angle, 0.0), d: Vec2::new(0.0, 1.0) },
        pc_rail_t0: Some(Curve2::Line { o: Vec2::new(0.0, t0), d: Vec2::new(1.0, 0.0) }),
        pc_rail_t1: Some(Curve2::Line { o: Vec2::new(0.0, t1), d: Vec2::new(1.0, 0.0) }),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helix::{helix, HelixParams};
    use ck_topo::check::check;

    fn top() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
    }

    fn trap() -> Profile {
        let p = [Vec2::new(3.3, -0.5), Vec2::new(4.5, -0.3), Vec2::new(4.5, 0.3), Vec2::new(3.3, 0.5)];
        let tags = ["e5", "e6", "e7", "e8"];
        let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        Profile::new(top(), vec![ProfileRegion { outer, holes: vec![] }])
    }

    fn helix_path(revs: f64, rev: bool) -> Vec<PathSeg> {
        let fr = Frame::identity();
        let p = HelixParams { pitch: 2.0, revolutions: revs, reverse: rev, ..Default::default() };
        vec![PathSeg::Helix { h: helix(&fr, Vec3::ZERO, 4.0, &p).unwrap(), rev: false }]
    }

    #[test]
    fn helix_trapezoid_matches_oracle() {
        // oracle (3 revolutions, pitch 2, r 4): faces S.e5..S.e8, C0, C1
        let b = sweep("F4", &trap(), &helix_path(3.0, false), 0.0).unwrap();
        assert!(check(&b, 1e-6).is_empty(), "{:?}", check(&b, 1e-6));
        let ids: Vec<_> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        assert_eq!(ids, ["F4.S.e5", "F4.S.e6", "F4.S.e7", "F4.S.e8", "F4.C0", "F4.C1"]);
        let kinds: Vec<_> = b.face_ids().map(|f| b.face(f).surface.kind()).collect();
        assert_eq!(kinds, ["spline", "cylinder", "spline", "cylinder", "plane", "plane"]);
    }

    #[test]
    fn orbit_accuracy() {
        let sc = Screw { o: Vec3::ZERO, a: Vec3::Z, angle: 3.0 * std::f64::consts::TAU, rise: 6.0 };
        let p = Vec3::new(4.5, 0.0, 0.3);
        for (deg, per_rev) in [(3usize, 48.0), (5, 16.0), (5, 24.0), (5, 32.0), (5, 48.0), (7, 16.0), (7, 24.0)] {
            let n = (per_rev * 3.0) as usize;
            let phis: Vec<f64> = (0..=n).map(|i| sc.angle * i as f64 / n as f64).collect();
            let c = orbit_deg(&sc, p, &phis, deg).unwrap();
            let mut worst: f64 = 0.0;
            for k in 0..=4000 {
                let u = sc.angle * k as f64 / 4000.0;
                worst = worst.max(c.eval(u).dist(sc.pt(u, p)));
            }
            eprintln!("deg {deg} per_rev {per_rev}: max deviation {worst:e}");
        }
    }

    fn vol(b: &Body) -> f64 {
        ck_topo::mass::mass_props(b).volume
    }

    /// A circle profile in the meridian plane swept `revs` turns: a long spline face with a curved
    /// (multi-span) section, the shape of a coil spring.
    fn coil(revs: f64) -> Body {
        let fr = Frame { o: Vec3::new(30.0, 0.0, 0.0), x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
        let c = ProfileLoop::new(vec![ProfileSeg::circle("c", Vec2::ZERO, 2.0)]);
        let prof = Profile::new(fr, vec![ProfileRegion { outer: c, holes: vec![] }]);
        let h = HelixParams { pitch: 7.0, revolutions: revs, ..Default::default() };
        let path = vec![PathSeg::Helix { h: helix(&Frame::identity(), Vec3::ZERO, 30.0, &h).unwrap(), rev: false }];
        sweep("F1", &prof, &path, 0.0).expect("coil sweep")
    }

    #[test]
    fn long_spline_face_integrals_use_exact_rules() {
        // the polynomial-degree Gauss rules (one panel per span, 5 points across the section)
        // must reproduce the generic composite rule on every face
        let b = coil(6.0);
        let r = ck_topo::mass::ref_point_of(&b);
        for f in b.face_ids() {
            let g = ck_topo::mass::face_integral::<1>(&b, f, 1, &|p, n| [(p - r).dot(n) / 3.0])[0];
            let e = ck_topo::mass::face_integral_poly::<1>(&b, f, 1, &|p, n| [(p - r).dot(n) / 3.0])[0];
            assert!((g - e).abs() <= 1e-9 * g.abs().max(1.0), "{} {g} vs {e}", b.face_id_str(f));
        }
        // closed form: A * 2 pi R * N
        let want = std::f64::consts::PI * 4.0 * std::f64::consts::TAU * 30.0 * 6.0;
        assert!((vol(&b) - want).abs() / want < 3e-5, "{} vs {want}", vol(&b));
    }

    #[test]
    fn twist_extrude_keeps_volume_and_names() {
        let rect = ProfileLoop::new(
            [(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)]
                .iter()
                .enumerate()
                .map(|(i, &(x, y))| {
                    let (x2, y2) = [(40.0, 0.0), (40.0, 20.0), (0.0, 20.0), (0.0, 0.0)][i];
                    ProfileSeg::line(&format!("e{}", i + 1), Vec2::new(x, y), Vec2::new(x2, y2))
                })
                .collect(),
        );
        let circ = |tag: &str, c: Vec2, r: f64| ProfileLoop::new(vec![ProfileSeg::circle(tag, c, r)]);
        let cases = [
            (Profile::new(top(), vec![ProfileRegion { outer: rect, holes: vec![] }]), 8000.0),
            (
                Profile::new(
                    top(),
                    vec![ProfileRegion { outer: circ("e7", Vec2::new(5.0, 3.0), 10.0), holes: vec![ProfileLoop::new(vec![ProfileSeg::circle("e8", Vec2::new(8.0, 5.0), 3.0).reversed()])] }],
                ),
                std::f64::consts::PI * 91.0 * 10.0,
            ),
        ];
        for (p, want) in &cases {
            for deg in [90.0f64, -200.0, 471.6] {
                let b = twist_extrude("F2", p, ExtrudeOpts::blind(10.0), deg.to_radians(), Vec2::ZERO).unwrap();
                assert!(check(&b, 1e-6).is_empty(), "{deg}: {:?}", check(&b, 1e-6));
                assert!((vol(&b) - want).abs() < 1e-4 * want, "{deg}: {} vs {want}", vol(&b));
                let ids: Vec<_> = b.face_ids().map(|f| b.face_id_str(f)).collect();
                assert_eq!(&ids[..2], ["F2.C0", "F2.C1"], "{ids:?}");
                assert!(ids[2..].iter().all(|i| i.starts_with("F2.S.e")), "{ids:?}");
            }
        }
    }

    #[test]
    fn twist_of_circle_about_its_axis_is_a_cylinder() {
        let l = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::ZERO, 5.0)]);
        let p = Profile::new(top(), vec![ProfileRegion { outer: l, holes: vec![] }]);
        let b = twist_extrude("F2", &p, ExtrudeOpts::mid(8.0), 1.0, Vec2::ZERO).unwrap();
        let ids: Vec<_> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        assert_eq!(ids, ["F2.C0", "F2.C1", "F2.S.e1"]);
        assert_eq!(b.face(b.face_ids().nth(2).unwrap()).surface.kind(), "cylinder");
        assert!(check(&b, 1e-6).is_empty());
        assert!((vol(&b) - std::f64::consts::PI * 25.0 * 8.0).abs() < 1e-6);
    }

    #[test]
    fn profile_not_normal() {
        let l = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)),
            ProfileSeg::line("e2", Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0)),
            ProfileSeg::line("e3", Vec2::new(1.0, 1.0), Vec2::new(0.0, 1.0)),
            ProfileSeg::line("e4", Vec2::new(0.0, 1.0), Vec2::new(0.0, 0.0)),
        ]);
        let p = Profile::new(Frame::identity(), vec![ProfileRegion { outer: l, holes: vec![] }]);
        let path = [PathSeg::Line { a: Vec3::ZERO, b: Vec3::new(0.0, 20.0, 0.0) }];
        assert_eq!(sweep("F4", &p, &path, 0.0).unwrap_err(), PROFILE_NOT_NORMAL);
    }

    /// Groove (swept trapezoid, `REVS` turns of pitch 2, r 4) cut into a solid
    /// shank cylinder r 4.0 spanning z in [ZLO, ZHI]: `ZLO=-10 ZHI=8` keeps the
    /// whole groove inside (a pocket), `ZHI=3` lets it run out of the top end.
    /// `SSDBG=1` prints the boolean's phases and open edges.
    #[test]
    #[ignore]
    fn screw_cut_shank() {
        use crate::boolean::{boolean, Op};
        use ck_topo::make::extrude;
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d);
        let circ = |tag: &str, r: f64| ProfileLoop::new(vec![ProfileSeg::circle(tag, Vec2::ZERO, r)]);
        let prof = Profile::new(Frame::identity(), vec![ProfileRegion { outer: circ("e1", 4.0), holes: vec![] }]);
        let (zlo, zhi) = (env("ZLO", -10.0), env("ZHI", 8.0));
        let shank = extrude("F2", &prof, ExtrudeOpts { s0: zlo, s1: zhi, dir: None, draft: 0.0 }).unwrap();
        let tool = sweep("F13", &trap(), &helix_path(env("REVS", 3.0), false), 0.0).unwrap();
        let t = std::time::Instant::now();
        let r = boolean(&shank, &tool, Op::Cut, 1e-6);
        eprintln!("cut {:?} in {:?}", r.as_ref().map(|o| o.body.num_faces()), t.elapsed());
        if let Ok(o) = r {
            eprintln!("issues {:?}", check(&o.body, 1e-6));
            let (v0, v1) = (ck_topo::mass::mass_props(&shank).volume, ck_topo::mass::mass_props(&o.body).volume);
            let mut tot = ck_geom::Vec3::ZERO;
            for f in o.body.face_ids() {
                let a = ck_topo::mass::face_integral::<3>(&o.body, f, 1, &|_, n| [n.x, n.y, n.z]);
                let ar = ck_topo::mass::face_integral::<1>(&o.body, f, 1, &|_, n| [n.len()])[0];
                let v = ck_geom::Vec3::new(a[0], a[1], a[2]);
                tot = tot + v;
                eprintln!("face {} {} area {:.4} vecarea {:?}", o.body.face_id_str(f), o.body.face(f).surface.kind(), ar, v);
            }
            eprintln!("sum vecarea {:?}", tot);
            eprintln!("volume {v0:.4} -> {v1:.4} (removed {:.4}); tool {:.4}", v0 - v1, ck_topo::mass::mass_props(&tool).volume);
        }
    }

    /// Point classification against a multi-turn screw tool along the line
    /// x = -4 (the groove is centred on z = 1, 3, 5 ...).
    #[test]
    #[ignore]
    fn screw_tool_classify() {
        use crate::boolean::classify::Solid;
        let tool = sweep("F13", &trap(), &helix_path(4.0, false), 0.0).unwrap();
        let t = std::time::Instant::now();
        let solid = Solid::new(&tool, 1e-6);
        eprintln!("solid built in {:?}", t.elapsed());
        for i in 0..=70 {
            let z = 0.1 * i as f64;
            let t = std::time::Instant::now();
            let c = solid.classify(Vec3::new(-4.0, 0.01, z), Vec3::new(-1.0, 0.0, 0.0));
            eprintln!("z {z:.1} {c:?} {:?}", t.elapsed());
        }
    }

    #[test]
    #[ignore]
    fn screw_tool_rays() {
        use crate::boolean::classify::point_in_face;
        let tool = sweep("F13", &trap(), &helix_path(4.0, false), 0.0).unwrap();
        let q = Vec3::new(-4.0, 0.01, 2.3);
        let d = Vec3::new(0.5773502691896258, 0.6123724356957945, 0.5400617248673217);
        for f in tool.face_ids() {
            let s = &tool.face(f).surface;
            let ts = crate::intersect::cs::line_params(q, d, s, 1e-6);
            let mut ts = ts.unwrap_or_default();
            ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let inf: Vec<_> = ts.iter().filter(|&&t| t > 0.0).map(|&t| (format!("{t:.4}"), format!("{:?}", point_in_face(&tool, f, q + d * t, 1e-6)))).collect();
            eprintln!("face {} {} roots(t>0) {:?}", tool.face_id_str(f), s.kind(), inf);
        }
    }

    /// Diagnostic (slow, run with `REVS=0.25 cargo test -p ck-ops --profile fast
    /// cut_timing -- --ignored --nocapture`): groove swept into a bored tube.
    /// Currently `booleanFailed`; prints the SSI timings and the open edges of
    /// the assembled result (see the guard in ck-model features/sweep.rs).
    #[test]
    #[ignore]
    fn cut_timing() {
        use crate::boolean::{boolean, Op};
        use ck_topo::make::extrude;
        let circ = |tag: &str, r: f64| ProfileLoop::new(vec![ProfileSeg::circle(tag, Vec2::ZERO, r)]);
        let prof = Profile::new(Frame::identity(), vec![ProfileRegion { outer: circ("e1", 6.0), holes: vec![circ("e2", 3.4)] }]);
        let tube = extrude("F11", &prof, ExtrudeOpts::blind(8.0)).unwrap();
        let revs: f64 = std::env::var("REVS").ok().and_then(|s| s.parse().ok()).unwrap_or(0.25);
        let tool = sweep("F4", &trap(), &helix_path(revs, false), 0.0).unwrap();
        // SSI of every tool face with the bore
        let bore = tube.face_ids().map(|f| tube.face(f).surface.clone()).find(|s| matches!(s, Surface::Cylinder { r, .. } if (*r - 3.4).abs() < 1e-9)).unwrap();
        let (lo, hi) = tool.bbox();
        for f in tool.face_ids() {
            let t = std::time::Instant::now();
            let r = crate::intersect::ss::intersect(&tool.face(f).surface, &bore, Some(crate::intersect::ss::Bounds { lo, hi }), 1e-6);
            eprintln!("ssi {} -> {} in {:?}", tool.face_id_str(f), r.curves().len(), t.elapsed());
        }
        let t = std::time::Instant::now();
        let r = boolean(&tube, &tool, Op::Cut, 1e-6);
        eprintln!("cut {:?} in {:?}", r.as_ref().map(|o| o.body.num_faces()), t.elapsed());
        if r.is_err() {
            use crate::boolean::{assemble, imprint};
            let a1 = crate::boolean::apply::relocate_seams(&tube, &tool, crate::boolean::classify::Class::In, 1e-6);
            let b1 = crate::boolean::apply::relocate_seams(&tool, &tube, crate::boolean::classify::Class::Out, 1e-6);
            let arr = imprint::imprint(&a1, &b1, 1e-6);
            eprintln!("imprint: interacting {}", arr.interacting_pairs);
            match assemble::select(&arr, [&a1, &b1], Op::Cut) {
                Err(e) => eprintln!("select failed: {e}"),
                Ok(faces) => {
                    let body = assemble::build(&arr, &faces);
                    eprintln!("built {} faces; issues {:?}", body.num_faces(), check(&body, 1e-6));
                    for e in body.edge_ids() {
                        let ed = body.edge(e);
                        let fs: Vec<_> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
                        let kind = match &ed.curve {
                            Curve::Line { .. } => "line",
                            Curve::Circle { .. } => "circle",
                            Curve::BSpline(_) => "bspline",
                            _ => "other",
                        };
                        eprintln!(
                            "  e{} {} t[{:.4},{:.4}] {:?} -> {:?} faces {:?} coedges {}",
                            e.idx(),
                            kind,
                            ed.t0,
                            ed.t1,
                            body.vertex(ed.v0).p,
                            body.vertex(ed.v1).p,
                            fs,
                            body.edge_coedges(e).len()
                        );
                    }
                }
            }
        }
    }
}

// ------------------------------------------------------- multi-piece paths

fn motion_xf(m: Motion) -> Xform {
    match m {
        Motion::Translate(v) => Xform::translation(v),
        Motion::Screw { o, a, angle, rise } => Screw { o, a, angle, rise }.xf(angle),
    }
}

/// Largest distance between sampled profile points (an upper bound for any
/// chord of the section).
fn profile_extent(p: &Profile) -> f64 {
    let mut pts: Vec<Vec2> = vec![];
    for r in &p.regions {
        for l in r.loops() {
            for g in &l.segs {
                pts.extend(g.curve.sample(g.t0.min(g.t1), g.t0.max(g.t1), 12));
            }
        }
    }
    let mut d: f64 = 0.0;
    for a in &pts {
        for b in &pts {
            d = d.max(a.dist(*b));
        }
    }
    d
}

/// Unit end direction of a piece.
fn end_dir(s: &PathSeg) -> Option<Vec3> {
    s.reversed().start_dir().map(|d| -d)
}

/// Big block on one side of the plane through `p` with normal `m`
/// (`positive`: the side `m` points to).
fn half_space(p: Vec3, m: Vec3, positive: bool, h: f64) -> Result<Body, String> {
    let x = if m.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (x - m * x.dot(m)).normalize();
    let frame = Frame { o: p, x, y: m.cross(x), z: m };
    let c = [Vec2::new(-h, -h), Vec2::new(h, -h), Vec2::new(h, h), Vec2::new(-h, h)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line("h", c[i], c[(i + 1) % 4])).collect());
    let prof = Profile::new(frame, vec![ProfileRegion { outer: lp, holes: vec![] }]);
    let (s0, s1) = if positive { (0.0, h) } else { (-h, 0.0) };
    extrude("H", &prof, ExtrudeOpts { s0, s1, dir: None, draft: 0.0 })
}

/// Sweep along a path that is not one rigid motion: every piece is swept
/// separately (the section carried along by the previous pieces), the pieces
/// are united. Tangent joints (G1) meet in coincident caps; corner joints
/// between straight pieces are mitred: both pieces are overshot and clipped
/// by the bisector plane. Side faces keep the profile entity tag with a
/// `~k` piece number (`F3.S.e1~2` = entity `e1` on the 2nd piece); the first
/// cap stays `C0`, the last `C1`, interior caps disappear in the union.
fn sweep_pieces(feature: &str, profile: &Profile, path: &[PathSeg], twist: f64) -> Result<Body, String> {
    if twist != 0.0 {
        return Err("TODO(rebuild): twisted sweep along a multi-piece path".to_string());
    }
    if profile.regions.len() != 1 {
        return Err("TODO(rebuild): multi-region profile swept along a multi-piece path".to_string());
    }
    let n = path.len();
    if path.iter().any(|s| matches!(s, PathSeg::Helix { .. })) {
        return Err(PATH_INVALID.into());
    }
    let ext = profile_extent(profile);
    // joint i joins piece i and i + 1: None = tangent, Some(turn angle) = mitre
    let mut joints: Vec<Option<f64>> = vec![];
    for i in 0..n.saturating_sub(1) {
        let (d1, d2) = (end_dir(&path[i]).ok_or(PATH_INVALID)?, path[i + 1].start_dir().ok_or(PATH_INVALID)?);
        if path[i].end().dist(path[i + 1].start()) > 1e-6 {
            return Err(PATH_INVALID.into());
        }
        let c = d1.dot(d2).clamp(-1.0, 1.0);
        if c > 1.0 - 1e-9 {
            joints.push(None);
        } else {
            let th = c.acos();
            if th > 2.9 || !matches!(path[i], PathSeg::Line { .. }) || !matches!(path[i + 1], PathSeg::Line { .. }) {
                return Err("TODO(rebuild): mitre joint other than line-line (turn < 166 deg)".to_string());
            }
            joints.push(Some(th));
        }
    }
    let mut fr = profile.frame;
    let mut acc: Option<Body> = None;
    for (i, seg) in path.iter().enumerate() {
        let m_start = i > 0 && joints[i - 1].is_some();
        let m_end = i + 1 < n && joints[i].is_some();
        let motion = path_motion(std::slice::from_ref(seg)).ok_or(PATH_INVALID)?;
        // mitred ends: overshoot (straight pieces only, checked above)
        let e_s = if m_start { ext * ((joints[i - 1].unwrap() / 2.0).tan() + 0.5) } else { 0.0 };
        let e_e = if m_end { ext * ((joints[i].unwrap() / 2.0).tan() + 0.5) } else { 0.0 };
        let mut body = if m_start || m_end {
            let PathSeg::Line { a, b } = seg else { unreachable!() };
            let d = (*b - *a).normalize();
            let pf = Profile::new(Frame { o: fr.o - d * e_s, ..fr }, profile.regions.clone());
            sweep_motion(feature, &pf, Motion::Translate(*b - *a + d * (e_s + e_e)))?
        } else {
            sweep_motion(feature, &Profile::new(fr, profile.regions.clone()), motion)?
        };
        let h = 4.0 * (ext + e_s + e_e + seg_len(seg));
        if m_start {
            let (d1, d2) = (end_dir(&path[i - 1]).unwrap(), seg.start_dir().unwrap());
            let m = (d1 + d2).normalize();
            body = boolean(&body, &half_space(seg.start(), m, true, h)?, Op::Intersect, 1e-6)?.body;
        }
        if m_end {
            let (d1, d2) = (end_dir(seg).unwrap(), path[i + 1].start_dir().unwrap());
            let m = (d1 + d2).normalize();
            body = boolean(&body, &half_space(seg.end(), m, false, h)?, Op::Intersect, 1e-6)?.body;
        }
        for f in body.faces.iter_mut() {
            if matches!(f.name.role, ck_topo::body::Role::Side(_)) && f.name.region.is_none() {
                f.name.region = Some(i as u32 + 1);
            }
        }
        acc = Some(match acc {
            None => body,
            Some(a) => boolean(&a, &body, Op::Union, 1e-6).map_err(|_| "the pieces did not unite".to_string())?.body,
        });
        // carry the section to the end of the piece (and round the corner)
        let x = motion_xf(motion);
        fr = rot_frame(&x, &fr);
        if let (true, Some(th)) = (m_end, joints.get(i).copied().flatten()) {
            let (d1, d2) = (end_dir(seg).unwrap(), path[i + 1].start_dir().unwrap());
            let axis = d1.cross(d2).normalize();
            let mut r = rotation(seg.end(), axis, th);
            if r.apply_vec(d1).dist(d2) > 1e-6 {
                r = rotation(seg.end(), axis, -th);
            }
            fr = rot_frame(&r, &fr);
        }
    }
    acc.ok_or_else(|| PATH_INVALID.to_string())
}

fn seg_len(s: &PathSeg) -> f64 {
    match s {
        PathSeg::Line { a, b } => a.dist(*b),
        PathSeg::Arc { r, a0, a1, .. } => r * (a1 - a0).abs(),
        PathSeg::Helix { .. } => 0.0,
    }
}

#[cfg(test)]
mod piece_tests {
    use super::*;
    use ck_topo::check::check;
    use ck_topo::mass::mass_props;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// Section in the XY plane (normal +Z) at the origin.
    fn frame_z() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }
    }
    fn circle(r: f64) -> Profile {
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), r)]);
        Profile::new(frame_z(), vec![ProfileRegion { outer: lp, holes: vec![] }])
    }
    fn square(h: f64) -> Profile {
        let c = [Vec2::new(-h, -h), Vec2::new(h, -h), Vec2::new(h, h), Vec2::new(-h, h)];
        let tags = ["e1", "e2", "e3", "e4"];
        let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], c[i], c[(i + 1) % 4])).collect());
        Profile::new(frame_z(), vec![ProfileRegion { outer: lp, holes: vec![] }])
    }
    fn line(a: [f64; 3], b: [f64; 3]) -> PathSeg {
        PathSeg::Line { a: Vec3::new(a[0], a[1], a[2]), b: Vec3::new(b[0], b[1], b[2]) }
    }

    /// line - quarter arc - line (tangent joints), in the XZ plane.
    fn elbow(l1: f64, ra: f64, l2: f64) -> Vec<PathSeg> {
        // up +Z, turn towards +X about the arc centre (ra, 0, l1)
        let fr = Frame { o: Vec3::new(ra, 0.0, l1), x: -Vec3::X, y: Vec3::Z, z: Vec3::Y };
        vec![
            line([0.0, 0.0, 0.0], [0.0, 0.0, l1]),
            PathSeg::Arc { frame: fr, r: ra, a0: 0.0, a1: FRAC_PI_2 },
            line([ra, 0.0, l1 + ra], [ra + l2, 0.0, l1 + ra]),
        ]
    }

    #[test]
    fn tangent_elbow_circle() {
        let (r, l1, ra, l2) = (2.0, 10.0, 6.0, 8.0);
        let b = sweep("F3", &circle(r), &elbow(l1, ra, l2), 0.0).unwrap();
        assert!(check(&b, 1e-6).is_empty(), "{:?}", check(&b, 1e-6));
        let want = PI * r * r * (l1 + l2 + ra * FRAC_PI_2);
        let v = mass_props(&b).volume;
        assert!((v - want).abs() < 1e-3 * want, "{v} vs {want}");
        let ids: Vec<_> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        assert!(ids.contains(&"F3.S.e1~1".to_string()) && ids.contains(&"F3.S.e1~2".to_string()) && ids.contains(&"F3.S.e1~3".to_string()), "{ids:?}");
        assert!(ids.contains(&"F3.C0".to_string()) && ids.contains(&"F3.C1".to_string()), "{ids:?}");
        assert_eq!(ids.len(), 5, "{ids:?}");
    }

    #[test]
    fn tangent_elbow_square_reversed_path() {
        let (h, l1, ra, l2) = (1.5, 10.0, 6.0, 8.0);
        let rev: Vec<PathSeg> = elbow(l1, ra, l2).iter().rev().map(|s| s.reversed()).collect();
        let b = sweep("F3", &square(h), &rev, 0.0).unwrap();
        assert!(check(&b, 1e-6).is_empty(), "{:?}", check(&b, 1e-6));
        let want = 4.0 * h * h * (l1 + l2 + ra * FRAC_PI_2);
        let v = mass_props(&b).volume;
        assert!((v - want).abs() < 1e-3 * want, "{v} vs {want}");
    }

    #[test]
    fn mitre_corner_square() {
        // L path: up 10 then along +X 8, square section half-width h:
        // mitred volume = area * (centre-line length) (the inner/outer excess cancel)
        let h = 1.5;
        let path = vec![line([0.0, 0.0, 0.0], [0.0, 0.0, 10.0]), line([0.0, 0.0, 10.0], [8.0, 0.0, 10.0])];
        let b = sweep("F3", &square(h), &path, 0.0).unwrap();
        assert!(check(&b, 1e-6).is_empty(), "{:?}", check(&b, 1e-6));
        let want = 4.0 * h * h * 18.0;
        let v = mass_props(&b).volume;
        assert!((v - want).abs() < 1e-3 * want, "{v} vs {want}");
    }

    #[test]
    fn mitre_corner_circle_oblique() {
        let r = 1.0;
        let th = 2.2_f64; // heading change of the second leg
        let p1 = Vec3::new(0.0, 0.0, 10.0);
        let p2 = p1 + Vec3::new(th.sin(), 0.0, th.cos()) * 8.0;
        let path = vec![line([0.0, 0.0, 0.0], [0.0, 0.0, 10.0]), PathSeg::Line { a: p1, b: p2 }];
        let b = sweep("F3", &circle(r), &path, 0.0).unwrap();
        assert!(check(&b, 1e-6).is_empty(), "{:?}", check(&b, 1e-6));
        // mitred volume = pi r^2 (L1 + L2) (inner and outer wedges are equal for a symmetric mitre)
        let want = PI * r * r * 18.0;
        let v = mass_props(&b).volume;
        assert!((v - want).abs() < 2e-3 * want, "{v} vs {want}");
    }
}
