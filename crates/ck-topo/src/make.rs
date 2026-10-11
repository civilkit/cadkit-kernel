//! Primitive construction: extrude (blind / mid / flipped, optional draft)
//! and revolve (make/revolve.rs) of planar profiles.
//!
//! Pinned against oracle regens (see README "make" section):
//! - faces: per region `C0, C1, S.<tag>…` (outer loop segments in travel
//!   order, then each hole loop). C0 = the cap at the LOW end of the sweep
//!   (extrude: min height along the sketch normal, even when flipped;
//!   revolve: start angle), C1 = high end. Multi-region profiles add `~k`
//!   to cap ids and get one lump per region.
//! - edges, per loop: first one rail per loop vertex (vertex i = travel
//!   start of segment i; faces = seg i, seg i-1), then for each segment its
//!   C0 edge and C1 edge. Rails run C0 -> C1. Cap edges keep the sketch
//!   ENTITY direction/parameterisation (not the loop travel direction).
//! - closed single-entity loops (circles) give a seam rail (not exported);
//!   360° revolves have no caps and the profile curves become seams;
//!   profile vertices on the revolve axis give no rail, segments lying on
//!   the axis give no face (partial revolve: one C0|C1 edge in their slot).

pub mod revolve;

use crate::body::{Body, FaceName, VertexId};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use std::f64::consts::TAU;

pub type MakeResult = Result<Body, String>;

// ---------------------------------------------------------------- profile

/// One oriented profile segment: a sketch entity curve (entity direction,
/// trimmed to [t0,t1]) traversed reversed when `rev`.
#[derive(Debug, Clone, PartialEq)]
pub struct ProfileSeg {
    pub tag: String,
    pub curve: Curve2,
    pub t0: f64,
    pub t1: f64,
    pub rev: bool,
}

impl ProfileSeg {
    /// Line entity from `a` to `b` (t = arc length).
    pub fn line(tag: &str, a: Vec2, b: Vec2) -> ProfileSeg {
        ProfileSeg { tag: tag.into(), curve: Curve2::line_through(a, b), t0: 0.0, t1: a.dist(b), rev: false }
    }
    /// Arc entity: counter-clockwise about `c` from `p1` to `p2` (the
    /// sketch `{type:"arc", c, p1, p2}` convention); t = angle from `p1`.
    pub fn arc(tag: &str, c: Vec2, p1: Vec2, p2: Vec2) -> ProfileSeg {
        let r = c.dist(p1);
        let x = (p1 - c).normalize();
        let curve = Curve2::Circle { c, x, r, ccw: true };
        let mut t1 = curve.closest_param(p2);
        if t1 <= 1e-12 {
            t1 += TAU;
        }
        ProfileSeg { tag: tag.into(), curve, t0: 0.0, t1, rev: false }
    }
    /// Full circle entity (starts at c + r·X̂, as the oracle's edges do).
    pub fn circle(tag: &str, c: Vec2, r: f64) -> ProfileSeg {
        ProfileSeg { tag: tag.into(), curve: Curve2::circle(c, r), t0: 0.0, t1: TAU, rev: false }
    }
    pub fn reversed(&self) -> ProfileSeg {
        ProfileSeg { rev: !self.rev, ..self.clone() }
    }
    /// Entity parameter at the travel start / end.
    pub fn t_start(&self) -> f64 {
        if self.rev {
            self.t1
        } else {
            self.t0
        }
    }
    pub fn t_end(&self) -> f64 {
        if self.rev {
            self.t0
        } else {
            self.t1
        }
    }
    pub fn start(&self) -> Vec2 {
        self.curve.eval(self.t_start())
    }
    pub fn end(&self) -> Vec2 {
        self.curve.eval(self.t_end())
    }
    /// Unit tangent in travel direction at entity parameter t.
    pub fn travel_tangent(&self, t: f64) -> Vec2 {
        let d = self.curve.tangent(t);
        if self.rev {
            -d
        } else {
            d
        }
    }
    pub fn is_closed(&self) -> bool {
        self.start().dist(self.end()) < 1e-9
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct ProfileLoop {
    pub segs: Vec<ProfileSeg>,
}

impl ProfileLoop {
    pub fn new(segs: Vec<ProfileSeg>) -> ProfileLoop {
        ProfileLoop { segs }
    }
    /// Signed area (CCW > 0), by Green's theorem on the exact curves.
    pub fn signed_area(&self) -> f64 {
        let mut a = 0.0;
        for s in &self.segs {
            let (ta, tb) = (s.t_start(), s.t_end());
            let n = 16;
            for k in 0..n {
                let (p, q) = (ta + (tb - ta) * k as f64 / n as f64, ta + (tb - ta) * (k + 1) as f64 / n as f64);
                let (m, h) = ((p + q) * 0.5, (q - p) * 0.5);
                for &(x, w) in crate::mass::GL.iter() {
                    let t = m + h * x;
                    let d = s.curve.derivs(t, 1);
                    a += 0.5 * w * h * d[0].cross(d[1]);
                }
            }
        }
        a
    }
    /// Reverse travel. GUESS (ordering): the reversed loop keeps segment 0
    /// first (`[s0', s(n-1)', …, s1']`) — the oracle's own profile builder
    /// decides the order, callers should pass loops already oriented.
    pub fn reversed(&self) -> ProfileLoop {
        let mut segs: Vec<ProfileSeg> = self.segs.iter().rev().map(|s| s.reversed()).collect();
        segs.rotate_right(1);
        ProfileLoop { segs }
    }
    /// Max gap between consecutive segment ends.
    pub fn max_gap(&self) -> f64 {
        let n = self.segs.len();
        (0..n).map(|i| self.segs[i].end().dist(self.segs[(i + 1) % n].start())).fold(0.0, f64::max)
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct ProfileRegion {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
}

impl ProfileRegion {
    pub fn loops(&self) -> impl Iterator<Item = &ProfileLoop> {
        std::iter::once(&self.outer).chain(self.holes.iter())
    }
}

/// Closed planar profile on a sketch frame (2D coordinates = frame x/y).
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    pub frame: Frame,
    pub regions: Vec<ProfileRegion>,
}

impl Profile {
    pub fn new(frame: Frame, regions: Vec<ProfileRegion>) -> Profile {
        Profile { frame, regions }
    }
    /// Make outer loops CCW and holes CW (reversing where needed).
    pub fn normalized(mut self) -> Profile {
        for r in &mut self.regions {
            if r.outer.signed_area() < 0.0 {
                r.outer = r.outer.reversed();
            }
            for h in &mut r.holes {
                if h.signed_area() > 0.0 {
                    *h = h.reversed();
                }
            }
        }
        self
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.regions.is_empty() {
            return Err("emptyProfile".into()); // GUESS: error string
        }
        for r in &self.regions {
            for l in r.loops() {
                if l.segs.is_empty() || l.max_gap() > 1e-6 {
                    return Err("openProfile".into()); // GUESS: error string
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------- 2D -> 3D

/// Sketch 2D curve -> 3D curve on `frame`, translated by `off`.
pub fn curve2_to_3d(frame: &Frame, c: &Curve2, off: Vec3) -> Curve {
    let pv = |p: Vec2| frame.o + frame.x * p.x + frame.y * p.y + off;
    let vv = |p: Vec2| frame.x * p.x + frame.y * p.y;
    match c {
        Curve2::Line { o, d } => Curve::Line { o: pv(*o), d: vv(*d) },
        Curve2::Circle { c, x, r, ccw } => {
            let y = if *ccw { x.perp() } else { -x.perp() };
            let (x3, y3) = (vv(*x), vv(y));
            Curve::Circle { frame: Frame { o: pv(*c), x: x3, y: y3, z: x3.cross(y3) }, r: *r }
        }
        Curve2::Ellipse { c, x, a, b, ccw } => {
            let y = if *ccw { x.perp() } else { -x.perp() };
            let (x3, y3) = (vv(*x), vv(y));
            Curve::Ellipse { frame: Frame { o: pv(*c), x: x3, y: y3, z: x3.cross(y3) }, a: *a, b: *b }
        }
        Curve2::BSpline(b) => Curve::BSpline(b.map(|p| pv(p))),
    }
}

/// pcurve of a 3D line on a plane frame (exact, same parameter).
pub fn plane_line_pc(fr: &Frame, o: Vec3, d: Vec3) -> Curve2 {
    let q = o - fr.o;
    Curve2::Line { o: Vec2::new(q.dot(fr.x), q.dot(fr.y)), d: Vec2::new(d.dot(fr.x), d.dot(fr.y)) }
}

/// Exact pcurve of a line or circle lying in a plane (None otherwise).
pub fn plane_pcurve(fr: &Frame, c: &Curve) -> Option<Curve2> {
    match c {
        Curve::Line { o, d } => Some(plane_line_pc(fr, *o, *d)),
        Curve::Circle { frame, r } => {
            let q = frame.o - fr.o;
            let c2 = Vec2::new(q.dot(fr.x), q.dot(fr.y));
            let x2 = Vec2::new(frame.x.dot(fr.x), frame.x.dot(fr.y));
            let ccw = frame.z.dot(fr.z) > 0.0;
            Some(Curve2::Circle { c: c2, x: x2.normalize(), r: *r, ccw })
        }
        Curve::Ellipse { frame, a, b } => {
            let q = frame.o - fr.o;
            let c2 = Vec2::new(q.dot(fr.x), q.dot(fr.y));
            let x2 = Vec2::new(frame.x.dot(fr.x), frame.x.dot(fr.y));
            let ccw = frame.z.dot(fr.z) > 0.0;
            Some(Curve2::Ellipse { c: c2, x: x2.normalize(), a: *a, b: *b, ccw })
        }
        Curve::BSpline(_) => None,
    }
}

// ---------------------------------------------------------------- sweep core

/// Side surface + exact pcurves of its boundary pieces.
pub(crate) struct SideGeom {
    pub surface: Surface,
    pub sense: bool,
    pub pc_bottom: Curve2,
    pub pc_top: Curve2,
    /// pcurves of the rails through the entity's t0 / t1 ends (None at a
    /// pole / missing rail).
    pub pc_rail_t0: Option<Curve2>,
    pub pc_rail_t1: Option<Curve2>,
}

pub(crate) struct SegGeom {
    pub bottom: (Curve, f64, f64),
    pub top: (Curve, f64, f64),
    pub cap0_pc: Option<Curve2>,
    pub cap1_pc: Option<Curve2>,
    pub side: Option<SideGeom>,
}

pub(crate) struct VertGeom {
    pub bottom: Vec3,
    pub top: Vec3,
    pub rail: Option<(Curve, f64, f64)>,
}

pub(crate) struct LoopGeom {
    pub verts: Vec<VertGeom>,
    pub segs: Vec<SegGeom>,
}

pub(crate) struct Caps {
    pub c0: Frame,
    pub c1: Frame,
}

/// Assemble topology for a profile sweep. `sigma_pos`: sweep direction is
/// along +sketch-normal at the profile (decides loop orientation).
pub(crate) fn build_sweep(
    feature: &str,
    profile: &Profile,
    regions: Vec<Vec<LoopGeom>>,
    caps: Option<&[Caps]>,
    sigma_pos: bool,
    closed: bool,
) -> Body {
    let mut b = Body::new();
    let multi = profile.regions.len() > 1;
    for (ri, (preg, lgs)) in profile.regions.iter().zip(regions).enumerate() {
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let rtag = if multi { Some(ri as u32 + 1) } else { None };
        let cap_faces = caps.map(|cs| {
            let c = &cs[ri.min(cs.len() - 1)];
            let f0 = b.add_face(shell, Surface::Plane { frame: c.c0 }, !sigma_pos, FaceName::cap(feature, 0, rtag));
            let f1 = b.add_face(shell, Surface::Plane { frame: c.c1 }, sigma_pos, FaceName::cap(feature, 1, rtag));
            (f0, f1)
        });
        let ploops: Vec<&ProfileLoop> = preg.loops().collect();
        // side faces first (face order: caps, then all sides of the region)
        let mut side_faces = vec![];
        for (pl, lg) in ploops.iter().zip(lgs.iter()) {
            let mut fs = vec![];
            for (s, g) in pl.segs.iter().zip(lg.segs.iter()) {
                fs.push(g.side.as_ref().map(|sg| {
                    b.add_face(shell, sg.surface.clone(), sg.sense, FaceName::side(feature, &s.tag))
                }));
            }
            side_faces.push(fs);
        }
        for (li, (pl, lg)) in ploops.iter().zip(lgs.into_iter()).enumerate() {
            let m = pl.segs.len();
            // vertices
            let mut vb: Vec<VertexId> = vec![];
            let mut vt: Vec<VertexId> = vec![];
            for v in &lg.verts {
                let a = b.add_vertex(v.bottom);
                vb.push(a);
                vt.push(if closed || v.rail.is_none() { a } else { b.add_vertex(v.top) });
            }
            // rails
            let mut re = vec![];
            for (i, v) in lg.verts.iter().enumerate() {
                re.push(v.rail.as_ref().map(|(c, t0, t1)| b.add_edge(c.clone(), *t0, *t1, vb[i], vt[i])));
            }
            // bottom / top edges
            let mut be = vec![];
            let mut te = vec![];
            for (i, (s, g)) in pl.segs.iter().zip(lg.segs.iter()).enumerate() {
                let (si, ei) = if s.rev { ((i + 1) % m, i) } else { (i, (i + 1) % m) };
                if g.side.is_none() && closed {
                    be.push(None);
                    te.push(None);
                    continue;
                }
                let e0 = b.add_edge(g.bottom.0.clone(), g.bottom.1, g.bottom.2, vb[si], vb[ei]);
                be.push(Some(e0));
                if closed || g.side.is_none() {
                    te.push(Some(e0));
                } else {
                    te.push(Some(b.add_edge(g.top.0.clone(), g.top.1, g.top.2, vt[si], vt[ei])));
                }
            }
            // side loops
            for (i, (s, g)) in pl.segs.iter().zip(lg.segs.iter()).enumerate() {
                let (Some(f), Some(sg)) = (side_faces[li][i], g.side.as_ref()) else { continue };
                let j = (i + 1) % m;
                let (pc_end, pc_start) =
                    if s.rev { (&sg.pc_rail_t0, &sg.pc_rail_t1) } else { (&sg.pc_rail_t1, &sg.pc_rail_t0) };
                let mut list = vec![(be[i].unwrap(), s.rev, Some(sg.pc_bottom.clone()))];
                if let Some(r) = re[j] {
                    list.push((r, false, pc_end.clone()));
                }
                list.push((te[i].unwrap(), !s.rev, Some(sg.pc_top.clone())));
                if let Some(r) = re[i] {
                    list.push((r, true, pc_start.clone()));
                }
                if !sigma_pos {
                    list = list.into_iter().rev().map(|(e, r, p)| (e, !r, p)).collect();
                }
                let lp = b.add_loop(f);
                for (e, r, p) in list {
                    b.add_coedge(lp, e, r, p);
                }
            }
            // cap loops
            if let Some((f0, f1)) = cap_faces {
                let fwd1: Vec<_> =
                    (0..m).map(|i| (te[i].unwrap(), pl.segs[i].rev, lg.segs[i].cap1_pc.clone())).collect();
                let fwd0: Vec<_> =
                    (0..m).map(|i| (be[i].unwrap(), pl.segs[i].rev, lg.segs[i].cap0_pc.clone())).collect();
                let flip = |l: Vec<_>| -> Vec<_> { l.into_iter().rev().map(|(e, r, p): (_, bool, _)| (e, !r, p)).collect() };
                let (l0, l1) = if sigma_pos { (flip(fwd0), fwd1) } else { (fwd0, flip(fwd1)) };
                let lp0 = b.add_loop(f0);
                for (e, r, p) in l0 {
                    b.add_coedge(lp0, e, r, p);
                }
                let lp1 = b.add_loop(f1);
                for (e, r, p) in l1 {
                    b.add_coedge(lp1, e, r, p);
                }
            }
        }
    }
    b
}

/// Numerical sense: does the natural normal at `uv` agree with `outward`?
pub(crate) fn sense_of(s: &Surface, uv: Vec2, outward: Vec3) -> bool {
    s.normal(uv.x, uv.y).dot(outward) >= 0.0
}

// ---------------------------------------------------------------- extrude

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtrudeOpts {
    /// Low / high heights along `dir` (s0 < s1). Blind = (0, d); flipped
    /// blind = (-d, 0); mid = (-d/2, d/2).
    pub s0: f64,
    pub s1: f64,
    /// Sweep direction (unit); None = sketch normal. Must not lie in the
    /// sketch plane; it is flipped to have a positive normal component.
    pub dir: Option<Vec3>,
    /// Draft angle in radians (> 0 tapers inward away from the sketch
    /// plane; < 0 = outward, the `draftOut` checkbox).
    pub draft: f64,
}

impl ExtrudeOpts {
    pub fn blind(depth: f64) -> ExtrudeOpts {
        ExtrudeOpts { s0: 0.0, s1: depth, dir: None, draft: 0.0 }
    }
    pub fn flipped(depth: f64) -> ExtrudeOpts {
        ExtrudeOpts { s0: -depth, s1: 0.0, dir: None, draft: 0.0 }
    }
    pub fn mid(depth: f64) -> ExtrudeOpts {
        ExtrudeOpts { s0: -depth / 2.0, s1: depth / 2.0, dir: None, draft: 0.0 }
    }
}

/// Extrude a profile into one lump per region.
pub fn extrude(feature: &str, profile: &Profile, o: ExtrudeOpts) -> MakeResult {
    profile.validate()?;
    let fr = profile.frame;
    let n = fr.z;
    let mut d = o.dir.unwrap_or(n).normalize();
    let dn = d.dot(n);
    if dn.abs() < 1e-9 {
        return Err("extrudeDirectionInPlane".into()); // GUESS: error string
    }
    if dn < 0.0 {
        d = -d;
    }
    let (s0, s1) = (o.s0.min(o.s1), o.s0.max(o.s1));
    if s1 - s0 < 1e-9 {
        return Err("zeroDepth".into()); // GUESS: error string
    }
    let skew = (d - n).len() > 1e-12;
    let tan_a = o.draft.tan();
    if o.draft != 0.0 && (skew || (s0 < -1e-12 && s1 > 1e-12)) {
        return Err("draftUnsupported".into());
    }
    let mut regions = vec![];
    let mut caps = vec![];
    for reg in &profile.regions {
        let mut lgs = vec![];
        for pl in reg.loops() {
            let (bot, vbot) = offset_loop(pl, -s0.abs() * tan_a)?;
            let (top, vtop) = offset_loop(pl, -s1.abs() * tan_a)?;
            lgs.push(extrude_loop(&fr, d, s0, s1, pl, &bot, &vbot, &top, &vtop));
        }
        regions.push(lgs);
        let c0 = Frame { o: fr.o + d * s0, ..fr };
        let c1 = Frame { o: fr.o + d * s1, ..fr };
        caps.push(Caps { c0, c1 });
    }
    let mut body = build_sweep(feature, profile, regions, Some(&caps), true, false);
    if o.draft != 0.0 {
        orient_draft_edges(&mut body);
    }
    Ok(body)
}

/// The oracle's drafted extrude re-derives plane/plane edges by
/// intersection (a draft op on the side faces), which fixes their
/// direction (pinned by the 10° draft probe):
/// - C0 edges: n_C0 × n_side; C1 edges: same direction as the C0 edge of
///   that side, i.e. (−n_C1) × n_side;
/// - side/side rails: n_a × n_b with (a, b) the face ids sorted bytewise.
fn orient_draft_edges(b: &mut Body) {
    use crate::body::Role;
    for e in b.edge_ids().collect::<Vec<_>>() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !matches!(b.edge(e).curve, Curve::Line { .. }) {
            continue;
        }
        if !fs.iter().all(|&f| matches!(b.face(f).surface, Surface::Plane { .. })) {
            continue;
        }
        let role = |f: crate::body::FaceId| b.face(f).name.role.clone();
        let want = match (role(fs[0]), role(fs[1])) {
            (Role::Cap(k), _) => cap_dir(b, fs[0], fs[1], k),
            (_, Role::Cap(k)) => cap_dir(b, fs[1], fs[0], k),
            _ => {
                let (mut a, mut c) = (fs[0], fs[1]);
                if b.face_id_str(a) > b.face_id_str(c) {
                    std::mem::swap(&mut a, &mut c);
                }
                b.planar_normal(a).cross(b.planar_normal(c))
            }
        };
        let Curve::Line { d, .. } = b.edge(e).curve else { continue };
        if d.dot(want) < 0.0 {
            crate::edit::reverse_line_edge(b, e);
        }
    }
}

fn cap_dir(b: &Body, cap: crate::body::FaceId, side: crate::body::FaceId, k: u32) -> Vec3 {
    let nc = b.planar_normal(cap);
    let n0 = if k == 0 { nc } else { -nc };
    n0.cross(b.planar_normal(side))
}

#[allow(clippy::too_many_arguments)]
fn extrude_loop(
    fr: &Frame,
    d: Vec3,
    s0: f64,
    s1: f64,
    pl: &ProfileLoop,
    bot: &[(Curve2, f64, f64)],
    vbot: &[Vec2],
    top: &[(Curve2, f64, f64)],
    vtop: &[Vec2],
) -> LoopGeom {
    let m = pl.segs.len();
    let h = s1 - s0;
    let p3 = |p: Vec2, s: f64| fr.o + fr.x * p.x + fr.y * p.y + d * s;
    let mut verts = vec![];
    for i in 0..m {
        let (a, b) = (p3(vbot[i], s0), p3(vtop[i], s1));
        let len = a.dist(b);
        verts.push(VertGeom { bottom: a, top: b, rail: Some((Curve::line_through(a, b), 0.0, len)) });
    }
    let skew = (d - fr.z).len() > 1e-12;
    let mut segs = vec![];
    for (i, s) in pl.segs.iter().enumerate() {
        let (bc, bt0, bt1) = &bot[i];
        let (tc, tt0, tt1) = &top[i];
        let bottom3 = curve2_to_3d(fr, bc, d * s0);
        let top3 = curve2_to_3d(fr, tc, d * s1);
        // entity-start / end vertex indices
        let (vs, ve) = if s.rev { ((i + 1) % m, i) } else { (i, (i + 1) % m) };
        let rail_t0 = &verts[vs];
        let rail_t1 = &verts[ve];
        let tm = 0.5 * (bt0 + bt1);
        let tan2 = s.travel_tangent(tm);
        let outward = (fr.x * tan2.x + fr.y * tan2.y).cross(fr.z);
        let side = match (bc, tc) {
            (Curve2::Line { .. }, Curve2::Line { .. }) => {
                let (Curve::Line { o: bo, d: bd }, Curve::Line { o: to, d: td }) = (&bottom3, &top3) else {
                    unreachable!()
                };
                // oracle (box probe, sign of zero): x = entity end − start
                // in 3D, not the sketch direction mapped through the frame
                let x = (bottom3.eval(*bt1) - bottom3.eval(*bt0)).normalize();
                let w = *to - *bo;
                let y = (w - x * w.dot(x)).normalize();
                let pf = Frame { o: *bo, x, y, z: x.cross(y) };
                let surface = Surface::Plane { frame: pf };
                let pc_bottom = plane_line_pc(&pf, *bo, *bd);
                let pc_top = plane_line_pc(&pf, *to, *td);
                let rl = |v: &VertGeom| {
                    let (Curve::Line { o, d }, _, _) = v.rail.as_ref().unwrap() else { unreachable!() };
                    plane_line_pc(&pf, *o, *d)
                };
                let sense = sense_of(&surface, pc_bottom.eval(tm), outward);
                Some(SideGeom {
                    surface,
                    sense,
                    pc_bottom,
                    pc_top,
                    pc_rail_t0: Some(rl(rail_t0)),
                    pc_rail_t1: Some(rl(rail_t1)),
                })
            }
            (Curve2::Circle { r: r0, .. }, Curve2::Circle { r: r1, .. }) if !skew => {
                let Curve::Circle { frame: cf, .. } = &bottom3 else { unreachable!() };
                let z = d;
                let x = cf.x;
                let y = z.cross(x);
                let su = if cf.y.dot(y) > 0.0 { 1.0 } else { -1.0 };
                let frame = Frame { o: cf.o, x, y, z };
                let surface = if (r1 - r0).abs() < 1e-12 {
                    Surface::Cylinder { frame, r: *r0 }
                } else {
                    Surface::Cone { frame, r: *r0, semi_angle: ((r1 - r0) / h).atan() }
                };
                let pc_bottom = Curve2::Line { o: Vec2::ZERO, d: Vec2::new(su, 0.0) };
                let pc_top = Curve2::Line { o: Vec2::new(0.0, h), d: Vec2::new(su, 0.0) };
                let rl = |tb: f64, tt: f64, v: &VertGeom| {
                    let len = v.rail.as_ref().unwrap().2.max(1e-300);
                    Curve2::Line { o: Vec2::new(su * tb, 0.0), d: Vec2::new(su * (tt - tb) / len, h / len) }
                };
                let sense = sense_of(&surface, pc_bottom.eval(tm), outward);
                Some(SideGeom {
                    surface,
                    sense,
                    pc_bottom,
                    pc_top,
                    pc_rail_t0: Some(rl(*bt0, *tt0, rail_t0)),
                    pc_rail_t1: Some(rl(*bt1, *tt1, rail_t1)),
                })
            }
            _ => {
                // general: linear extrusion surface, uv = (t, s - s0)
                let surface = Surface::Extrusion { profile: Box::new(bottom3.clone()), dir: d };
                let pc_bottom = Curve2::Line { o: Vec2::ZERO, d: Vec2::new(1.0, 0.0) };
                let pc_top = Curve2::Line { o: Vec2::new(0.0, h), d: Vec2::new(1.0, 0.0) };
                let rl = |tb: f64, v: &VertGeom| {
                    let len = v.rail.as_ref().unwrap().2.max(1e-300);
                    Curve2::Line { o: Vec2::new(tb, 0.0), d: Vec2::new(0.0, h / len) }
                };
                let sense = sense_of(&surface, pc_bottom.eval(tm), outward);
                Some(SideGeom {
                    surface,
                    sense,
                    pc_bottom,
                    pc_top,
                    pc_rail_t0: Some(rl(*bt0, rail_t0)),
                    pc_rail_t1: Some(rl(*bt1, rail_t1)),
                })
            }
        };
        segs.push(SegGeom {
            bottom: (bottom3, *bt0, *bt1),
            top: (top3, *tt0, *tt1),
            cap0_pc: Some(bc.clone()),
            cap1_pc: Some(tc.clone()),
            side,
        });
    }
    LoopGeom { verts, segs }
}

/// Offset a closed loop by `delta` along its outward normal (travel
/// tangent rotated clockwise). Returns entity-direction curves with their
/// new trims, and the new loop vertices (vertex i = travel start of seg i).
pub(crate) fn offset_loop(pl: &ProfileLoop, delta: f64) -> Result<(Vec<(Curve2, f64, f64)>, Vec<Vec2>), String> {
    let m = pl.segs.len();
    if delta == 0.0 {
        return Ok((
            pl.segs.iter().map(|s| (s.curve.clone(), s.t0, s.t1)).collect(),
            pl.segs.iter().map(|s| s.start()).collect(),
        ));
    }
    let out_n = |s: &ProfileSeg, t: f64| {
        let tt = s.travel_tangent(t);
        Vec2::new(tt.y, -tt.x)
    };
    let mut curves = vec![];
    for s in &pl.segs {
        let c = match &s.curve {
            Curve2::Line { o, d } => {
                let nn = out_n(s, s.t0);
                Curve2::Line { o: *o + nn * delta, d: *d }
            }
            Curve2::Circle { c, x, r, ccw } => {
                let tm = 0.5 * (s.t0 + s.t1);
                let radial = (s.curve.eval(tm) - *c).normalize();
                let sg = radial.dot(out_n(s, tm)).signum();
                let r2 = r + delta * sg;
                if r2 <= 1e-9 {
                    return Err("draftTooLarge".into());
                }
                Curve2::Circle { c: *c, x: *x, r: r2, ccw: *ccw }
            }
            _ => return Err("draftUnsupported".into()),
        };
        curves.push(c);
    }
    // vertices
    let mut verts = vec![];
    for i in 0..m {
        let (pi, ni) = ((i + m - 1) % m, i);
        let (sp, sn) = (&pl.segs[pi], &pl.segs[ni]);
        let v = sn.start();
        let (tp, tn) = (sp.travel_tangent(sp.t_end()), sn.travel_tangent(sn.t_start()));
        let guess_n = out_n(sn, sn.t_start());
        if m == 1 || tp.cross(tn).abs() < 1e-9 && tp.dot(tn) > 0.0 {
            verts.push(v + guess_n * delta);
            continue;
        }
        let guess = v + (out_n(sp, sp.t_end()) + guess_n) * (0.5 * delta);
        let cands = intersect2(&curves[pi], &curves[ni]);
        let best = cands.into_iter().min_by(|a, b| a.dist(guess).partial_cmp(&b.dist(guess)).unwrap());
        verts.push(best.ok_or_else(|| "draftFailed".to_string())?);
    }
    // trims
    let mut out = vec![];
    for (i, (s, c)) in pl.segs.iter().zip(curves).enumerate() {
        let (ps, pe) = (verts[i], verts[(i + 1) % m]);
        let (pa, pb) = if s.rev { (pe, ps) } else { (ps, pe) }; // entity start / end
        let (t0, t1) = match &c {
            Curve2::Line { .. } => (c.closest_param(pa), c.closest_param(pb)),
            _ => {
                if s.is_closed() {
                    (s.t0, s.t1)
                } else {
                    let a = ck_geom::pcurve::unwrap_near(c.closest_param(pa), s.t0, TAU);
                    let mut b = ck_geom::pcurve::unwrap_near(c.closest_param(pb), s.t1, TAU);
                    if b <= a {
                        b += TAU;
                    }
                    (a, b)
                }
            }
        };
        out.push((c, t0, t1));
    }
    Ok((out, verts))
}

/// Intersections of two 2D lines/circles.
fn intersect2(a: &Curve2, b: &Curve2) -> Vec<Vec2> {
    match (a, b) {
        (Curve2::Line { .. }, Curve2::Line { .. }) => {
            ck_geom::curve2::line_line(a, b).map(|(t, _)| vec![a.eval(t)]).unwrap_or_default()
        }
        (Curve2::Line { o, d }, Curve2::Circle { c, r, .. }) | (Curve2::Circle { c, r, .. }, Curve2::Line { o, d }) => {
            let dd = d.normalize();
            let q = *o - *c;
            let bq = q.dot(dd);
            let disc = bq * bq - (q.len2() - r * r);
            if disc < -1e-12 {
                return vec![];
            }
            let s = disc.max(0.0).sqrt();
            vec![*o + dd * (-bq - s), *o + dd * (-bq + s)]
        }
        (Curve2::Circle { c: c0, r: r0, .. }, Curve2::Circle { c: c1, r: r1, .. }) => {
            let dv = *c1 - *c0;
            let dl = dv.len();
            if dl < 1e-12 {
                return vec![];
            }
            let a = (r0 * r0 - r1 * r1 + dl * dl) / (2.0 * dl);
            let h2 = r0 * r0 - a * a;
            if h2 < -1e-12 {
                return vec![];
            }
            let h = h2.max(0.0).sqrt();
            let e = dv / dl;
            let m = *c0 + e * a;
            vec![m + e.perp() * h, m - e.perp() * h]
        }
        _ => vec![],
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::check::check;

    /// Oracle std planes (sketches[].frame of probe regens):
    /// top = {x:[1,0,0], y:[0,0,-1], z:[0,1,0]}, front = identity.
    pub fn top_frame() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
    }
    pub fn rect(tags: [&str; 4], x0: f64, y0: f64, x1: f64, y1: f64) -> ProfileLoop {
        let p = [Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)];
        ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect())
    }
    pub fn box_body() -> Body {
        // probe: top-plane rect (0,0)-(40,20), blind 10 -> volume 8000
        let prof = Profile::new(
            top_frame(),
            vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![] }],
        );
        extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap()
    }

    fn face_ids(b: &Body) -> Vec<String> {
        b.face_ids().map(|f| b.face_id_str(f)).collect()
    }
    fn keys(b: &Body) -> Vec<String> {
        let j = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        j.as_arr().unwrap().iter().map(|e| e.get_str("key").unwrap().to_string()).collect()
    }

    #[test]
    fn box_matches_oracle_names_and_wire() {
        let b = box_body();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(face_ids(&b), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"]);
        assert_eq!(
            keys(&b),
            [
                "F2.S.e1|F2.S.e4#0",
                "F2.S.e1|F2.S.e2#1",
                "F2.S.e2|F2.S.e3#2",
                "F2.S.e3|F2.S.e4#3",
                "F2.C0|F2.S.e1#4",
                "F2.C1|F2.S.e1#5",
                "F2.C0|F2.S.e2#6",
                "F2.C1|F2.S.e2#7",
                "F2.C0|F2.S.e3#8",
                "F2.C1|F2.S.e3#9",
                "F2.C0|F2.S.e4#10",
                "F2.C1|F2.S.e4#11"
            ]
        );
        // exact oracle wire text for faces
        let fj = ck_json::to_string(&b.faces_json());
        assert_eq!(
            fj,
            r#"[{"id":"F2.C0","kind":"plane","planar":true,"meta":{"feature":"F2"},"normal":[0,-1,0]},{"id":"F2.C1","kind":"plane","planar":true,"meta":{"feature":"F2"},"normal":[0,1,0]},{"id":"F2.S.e1","kind":"plane","planar":true,"meta":{"feature":"F2","entity":"e1"},"normal":[0,0,1]},{"id":"F2.S.e2","kind":"plane","planar":true,"meta":{"feature":"F2","entity":"e2"},"normal":[1,0,0]},{"id":"F2.S.e3","kind":"plane","planar":true,"meta":{"feature":"F2","entity":"e3"},"normal":[0,0,-1]},{"id":"F2.S.e4","kind":"plane","planar":true,"meta":{"feature":"F2","entity":"e4"},"normal":[-1,0,0]}]"#
        );
        // oracle: first edges
        let ej = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        let e0 = ck_json::to_string(&ej.as_arr().unwrap()[0]);
        assert_eq!(e0, r#"{"pts":[[0,0,0],[0,10,0]],"faces":["F2.S.e1","F2.S.e4"],"key":"F2.S.e1|F2.S.e4#0","mid":[0,5,0],"line":true}"#);
        let e8 = ck_json::to_string(&ej.as_arr().unwrap()[8]);
        assert_eq!(e8, r#"{"pts":[[40,0,-20],[0,0,-20]],"faces":["F2.C0","F2.S.e3"],"key":"F2.C0|F2.S.e3#8","mid":[20,0,-20],"line":true}"#);
    }

    #[test]
    fn cylinder_matches_oracle() {
        // probe: top plane, circle c=(5,3) r=10 tag e7, blind 10
        let prof = Profile::new(
            top_frame(),
            vec![ProfileRegion {
                outer: ProfileLoop::new(vec![ProfileSeg::circle("e7", Vec2::new(5.0, 3.0), 10.0)]),
                holes: vec![],
            }],
        );
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(face_ids(&b), ["F2.C0", "F2.C1", "F2.S.e7"]);
        assert_eq!(keys(&b), ["F2.C0|F2.S.e7#0", "F2.C1|F2.S.e7#1"]);
        let fj = ck_json::to_string(&b.faces_json().as_arr().unwrap()[2]);
        assert_eq!(fj, r#"{"id":"F2.S.e7","kind":"cylinder","planar":false,"meta":{"feature":"F2","entity":"e7"}}"#);
        let ej = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        let e0 = &ej.as_arr().unwrap()[0];
        // oracle: "circle":{"c":[5,0,-3],"n":[0,1,0],"r":10}, starts at [15,0,-3]
        assert_eq!(ck_json::to_string(e0.get("circle").unwrap()), r#"{"c":[5,0,-3],"n":[0,1,0],"r":10}"#);
        let p0 = e0.get_arr("pts").unwrap()[0].as_vec3().unwrap();
        assert_eq!(p0, [15.0, 0.0, -3.0]);
        let mid = e0.get("mid").unwrap().as_vec3().unwrap();
        assert!((mid[0] + 5.0).abs() < 1e-12 && (mid[2] + 3.0).abs() < 1e-12);
    }

    #[test]
    fn holes_and_regions_order() {
        // probe box3: front plane; outer e1..e4, holes [circle e5, rect h4,h3,h2,h1 (CW)]
        let fr = Frame::identity();
        let hole = ProfileLoop::new(vec![
            ProfileSeg::line("h4", Vec2::new(5.0, 15.0), Vec2::new(5.0, 5.0)).reversed(),
            ProfileSeg::line("h3", Vec2::new(15.0, 15.0), Vec2::new(5.0, 15.0)).reversed(),
            ProfileSeg::line("h2", Vec2::new(15.0, 5.0), Vec2::new(15.0, 15.0)).reversed(),
            ProfileSeg::line("h1", Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0)).reversed(),
        ]);
        let circ = ProfileLoop::new(vec![ProfileSeg::circle("e5", Vec2::new(30.0, 10.0), 4.0).reversed()]);
        let prof = Profile::new(
            fr,
            vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![circ, hole] }],
        );
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(
            face_ids(&b),
            ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4", "F2.S.e5", "F2.S.h4", "F2.S.h3", "F2.S.h2", "F2.S.h1"]
        );
        let k = keys(&b);
        assert_eq!(k[12], "F2.C0|F2.S.e5#12");
        assert_eq!(k[13], "F2.C1|F2.S.e5#13");
        assert_eq!(k[14], "F2.S.h1|F2.S.h4#14");
        assert_eq!(k[15], "F2.S.h3|F2.S.h4#15");
        assert_eq!(k[18], "F2.C0|F2.S.h4#18");
        assert_eq!(k[25], "F2.C1|F2.S.h1#25");
        assert_eq!(k.len(), 26);
        // oracle: hole side normal h4 = [1,0,0]; C0 normal [0,0,-1]
        let fj = b.faces_json();
        assert_eq!(fj.as_arr().unwrap()[7].get("normal").unwrap().as_vec3().unwrap(), [1.0, 0.0, 0.0]);
        // oracle: cap edge C0|h4 runs in entity direction [5,15,0] -> [5,5,0]
        let ej = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        let pts = ej.as_arr().unwrap()[18].get_arr("pts").unwrap().to_vec();
        assert_eq!(pts[0].as_vec3().unwrap(), [5.0, 15.0, 0.0]);
        // two regions -> ~k caps and lumps
        let prof2 = Profile::new(
            fr,
            vec![
                ProfileRegion { outer: rect(["a0", "a1", "a2", "a3"], 0.0, 0.0, 10.0, 10.0), holes: vec![] },
                ProfileRegion { outer: rect(["b0", "b1", "b2", "b3"], 20.0, 0.0, 30.0, 10.0), holes: vec![] },
            ],
        );
        let b2 = extrude("F2", &prof2, ExtrudeOpts::blind(5.0)).unwrap();
        let ids = face_ids(&b2);
        assert_eq!(ids[0], "F2.C0~1");
        assert_eq!(ids[6], "F2.C0~2");
        let fj2 = b2.faces_json();
        let f6 = &fj2.as_arr().unwrap()[6];
        assert_eq!(f6.get("lump").unwrap().as_f64(), Some(1.0));
        assert_eq!(keys(&b2)[16], "F2.C0~2|F2.S.b0#16");
    }

    #[test]
    fn flip_mid_and_draft() {
        let prof = Profile::new(
            Frame::identity(),
            vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![] }],
        );
        // oracle flip: C0 at z=-10 with normal -z; rail #0 [0,0,-10]->[0,0,0]
        let b = extrude("F2", &prof, ExtrudeOpts::flipped(10.0)).unwrap();
        let ej = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        assert_eq!(ck_json::to_string(ej.as_arr().unwrap()[0].get("pts").unwrap()), "[[0,0,-10],[0,0,0]]");
        assert_eq!(ck_json::to_string(ej.as_arr().unwrap()[4].get("pts").unwrap()), "[[0,0,-10],[40,0,-10]]");
        // draft 10° inward: oracle top vertex [1.7632698068918753, 1.7632698068918755, 9.99999999993...]
        let mut o = ExtrudeOpts::blind(10.0);
        o.draft = 10f64.to_radians();
        let b = extrude("F2", &prof, o).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        let fj = b.faces_json();
        let n = fj.as_arr().unwrap()[2].get("normal").unwrap().as_vec3().unwrap();
        assert!((n[1] + 0.984807753012208).abs() < 1e-12 && (n[2] - 0.17364817766693033).abs() < 1e-12, "{n:?}");
        let ej = b.edges_json(&|b, e| b.sample_edge(e, 0.01));
        let p = ej.as_arr().unwrap()[5].get_arr("pts").unwrap()[0].as_vec3().unwrap();
        assert!((p[0] - 38.23673019310812).abs() < 1e-9 && (p[1] - 1.7632698068918755).abs() < 1e-9);
    }
}
