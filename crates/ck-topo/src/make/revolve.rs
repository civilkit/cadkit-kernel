//! Revolve a planar profile about an axis lying in the profile plane.
//!
//! Pinned by oracle probes (front-plane rectangles about the sketch y axis):
//! - rotation is right-handed about the axis direction; C0 lies in the
//!   profile plane at the start angle, C1 at the end angle; a 360° revolve
//!   has no caps.
//! - side surfaces: line ⟂ axis → plane, line ∥ axis → cylinder, oblique
//!   line → cone, arc centred on the axis → sphere, other arcs/circles →
//!   torus; everything else → surface of revolution.
//! - rails are circles with n = axis direction, starting at the profile
//!   vertex (x = radial direction of the vertex).

use super::{build_sweep, curve2_to_3d, sense_of, Caps, LoopGeom, MakeResult, Profile, SegGeom, SideGeom, VertGeom};
use crate::trig;
use ck_core::linalg::Mat3;
use ck_core::xform::Xform;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use std::f64::consts::TAU;

const TOL: f64 = 1e-9;

/// Revolve `profile` about the axis (`axis_o`, unit `axis`) from angle `a0`
/// to `a1` (radians, a0 < a1; |a1-a0| >= 2π is a full revolve).
pub fn revolve(feature: &str, profile: &Profile, axis_o: Vec3, axis: Vec3, a0: f64, a1: f64) -> MakeResult {
    profile.validate()?;
    let a = axis.normalize();
    let (a0, a1) = (a0.min(a1), a0.max(a1));
    let sweep = a1 - a0;
    if sweep < 1e-9 {
        return Err("zeroAngle".into()); // GUESS: error string
    }
    let closed = sweep >= TAU - 1e-9;
    let sweep = if closed { TAU } else { sweep };
    if profile.frame.z.dot(a).abs() > 1e-9 || profile.frame.z.dot(axis_o - profile.frame.o).abs() > 1e-6 {
        return Err("revolveAxisNotInPlane".into()); // GUESS: error string
    }
    // Oracle (cone 120° probe, bit-level): the C1 cap and every C1 edge /
    // vertex are the sketch geometry rotated by the Rodrigues matrix built
    // with the kernel's own sin/cos (trig.rs), applied column-wise; rotated
    // frames keep x, y normalised and z = R·z as is (STEP writes x × y, the
    // wire normal is ±R·z). An unrotated start (a0 = 0) keeps the sketch
    // geometry bit-for-bit (applying an identity matrix would flip -0s).
    let r0 = (a0 != 0.0).then(|| rotation(axis_o, a, a0));
    let r1 = rotation(axis_o, a, a0 + sweep);
    let rot_pt = |r: &Option<Xform>, p: Vec3| r.as_ref().map_or(p, |x| x.apply_point(p));
    let fr0 = r0.as_ref().map_or(profile.frame, |x| rot_frame(x, &profile.frame));
    let fr1 = rot_frame(&r1, &profile.frame);
    let ax = Axis { o: axis_o, a };

    // sweep sense: w = a × r at any off-axis profile point vs plane normal
    let mut sigma = 0.0;
    'outer: for reg in &profile.regions {
        for l in reg.loops() {
            for s in &l.segs {
                for k in 0..5 {
                    let t = s.t0 + (s.t1 - s.t0) * k as f64 / 4.0;
                    let p = curve2_to_3d(&fr0, &s.curve, Vec3::ZERO).eval(t);
                    let r = ax.radial(p);
                    if r.len() > 1e-6 {
                        sigma = a.cross(r).dot(fr0.z);
                        break 'outer;
                    }
                }
            }
        }
    }
    if sigma == 0.0 {
        return Err("revolveDegenerate".into()); // GUESS: error string
    }
    let sigma_pos = sigma > 0.0;

    let mut regions = vec![];
    let mut caps = vec![];
    for reg in &profile.regions {
        let mut lgs = vec![];
        for pl in reg.loops() {
            let m = pl.segs.len();
            let mut verts = vec![];
            let pf = &profile.frame;
            for s in &pl.segs {
                let st = s.start();
                let base = pf.o + pf.x * st.x + pf.y * st.y;
                let p = rot_pt(&r0, base);
                let top = if closed { p } else { r1.apply_point(base) };
                verts.push(VertGeom { bottom: p, top, rail: ax.rail(p, sweep) });
            }
            let mut segs = vec![];
            for (i, s) in pl.segs.iter().enumerate() {
                let base = curve2_to_3d(pf, &s.curve, Vec3::ZERO);
                let bottom = r0.as_ref().map_or_else(|| base.clone(), |x| rot_curve(x, &base));
                let top = if closed { bottom.clone() } else { rot_curve(&r1, &base) };
                let (vs, ve) = if s.rev { ((i + 1) % m, i) } else { (i, (i + 1) % m) };
                let side = side_geom(&ax, &bottom, s.t0, s.t1, sweep, &verts[vs], &verts[ve]).map(|mut g| {
                    let tm = 0.5 * (s.t0 + s.t1);
                    let t2 = s.travel_tangent(tm);
                    let outward = (fr0.x * t2.x + fr0.y * t2.y).cross(fr0.z);
                    g.sense = sense_of(&g.surface, g.pc_bottom.eval(tm), outward);
                    g
                });
                segs.push(SegGeom {
                    bottom: (bottom, s.t0, s.t1),
                    top: (top, s.t0, s.t1),
                    cap0_pc: Some(s.curve.clone()),
                    cap1_pc: Some(s.curve.clone()),
                    side,
                });
            }
            lgs.push(LoopGeom { verts, segs });
        }
        regions.push(lgs);
        caps.push(Caps { c0: fr0, c1: fr1 });
    }
    Ok(build_sweep(feature, profile, regions, if closed { None } else { Some(&caps) }, sigma_pos, closed))
}

/// Rotation by `ang` about the axis (`o`, unit `a`): ck-core's Rodrigues
/// matrix (`Mat3::rotation` formula) with the kernel's sin/cos.
pub(crate) fn rotation(o: Vec3, a: Vec3, ang: f64) -> Xform {
    let (s, c) = (trig::sin(ang), trig::cos(ang));
    let t = 1.0 - c;
    let (x, y, z) = (a.x, a.y, a.z);
    let r = Mat3::from_rows(
        Vec3::new(t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        Vec3::new(t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        Vec3::new(t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    );
    Xform::new(r, o - r.mul_vec(o))
}

/// Rotated frame: x, y re-normalised, z rotated as is (see `revolve`).
pub(crate) fn rot_frame(r: &Xform, f: &Frame) -> Frame {
    Frame { o: r.apply_point(f.o), x: r.apply_vec(f.x).normalize(), y: r.apply_vec(f.y).normalize(), z: r.apply_vec(f.z) }
}

/// Rotated curve. GUESS: conic frames are treated like cap frames
/// (x, y re-normalised); only line / plane data is pinned.
pub(crate) fn rot_curve(r: &Xform, c: &Curve) -> Curve {
    match c {
        Curve::Line { o, d } => Curve::Line { o: r.apply_point(*o), d: r.apply_vec(*d) },
        Curve::Circle { frame, r: rad } => Curve::Circle { frame: rot_frame(r, frame), r: *rad },
        Curve::Ellipse { frame, a, b } => Curve::Ellipse { frame: rot_frame(r, frame), a: *a, b: *b },
        Curve::BSpline(b) => Curve::BSpline(b.map(|p| r.apply_point(p))),
    }
}

struct Axis {
    o: Vec3,
    a: Vec3,
}

impl Axis {
    fn z(&self, p: Vec3) -> f64 {
        (p - self.o).dot(self.a)
    }
    fn foot(&self, p: Vec3) -> Vec3 {
        self.o + self.a * self.z(p)
    }
    fn radial(&self, p: Vec3) -> Vec3 {
        p - self.foot(p)
    }
    /// Frame at the axis foot of `p` with x = radial of `p` (or `xr`).
    fn frame_at(&self, origin: Vec3, xr: Vec3) -> Frame {
        let x = xr.normalize();
        Frame { o: origin, x, y: self.a.cross(x), z: self.a }
    }
    fn rail(&self, p: Vec3, sweep: f64) -> Option<(Curve, f64, f64)> {
        let r = self.radial(p);
        let rho = r.len();
        if rho < TOL {
            return None;
        }
        Some((Curve::Circle { frame: self.frame_at(self.foot(p), r), r: rho }, 0.0, sweep))
    }
}

/// Side surface for one profile segment (bottom = profile at start angle).
/// pcurves use u = rotation angle in [0, sweep] (planes: polar uv).
fn side_geom(ax: &Axis, c: &Curve, t0: f64, t1: f64, sweep: f64, v_t0: &VertGeom, v_t1: &VertGeom) -> Option<SideGeom> {
    let (pa, pb) = (c.eval(t0), c.eval(t1));
    let (ra, rb) = (ax.radial(pa), ax.radial(pb));
    // radial reference direction: any off-axis point of the segment
    let xr = {
        let mut x = Vec3::ZERO;
        for k in 0..=8 {
            let r = ax.radial(c.eval(t0 + (t1 - t0) * k as f64 / 8.0));
            if r.len() > x.len() {
                x = r;
            }
        }
        x
    };
    if xr.len() < TOL {
        return None; // segment on the axis: no face
    }
    let xr = xr.normalize();
    // linear v-map v(t) = f0 + f1 t and the surface
    enum Kind {
        Polar { rho0: f64, rho1: f64 },
        Ang { f0: f64, f1: f64 },
    }
    let (surface, kind) = match c {
        Curve::Line { o, d } => {
            let (za, zb) = (ax.z(pa), ax.z(pb));
            let (rha, rhb) = (ra.dot(xr), rb.dot(xr));
            if (za - zb).abs() < TOL {
                let fr = ax.frame_at(ax.foot(pa), xr);
                let rho0 = (*o - fr.o).dot(fr.x);
                (Surface::Plane { frame: fr }, Kind::Polar { rho0, rho1: d.dot(fr.x) })
            } else if (rha - rhb).abs() < TOL {
                let fr = ax.frame_at(ax.o, xr);
                (Surface::Cylinder { frame: fr, r: rha }, Kind::Ang { f0: ax.z(*o), f1: d.dot(ax.a) })
            } else {
                let fr = ax.frame_at(ax.foot(pa), xr);
                let tan = (rhb - rha) / (zb - za);
                (
                    Surface::Cone { frame: fr, r: rha, semi_angle: tan.atan() },
                    Kind::Ang { f0: ax.z(*o) - za, f1: d.dot(ax.a) },
                )
            }
        }
        Curve::Circle { frame: cf, r } => {
            let cc = cf.o;
            let rc = ax.radial(cc);
            let (o, x, major) = if rc.len() < TOL { (cc, xr, 0.0) } else { (ax.foot(cc), rc.normalize(), rc.len()) };
            let fr = ax.frame_at(o, x);
            // φ(p) = atan2(z, ρ - major) about the circle centre
            let phi = |p: Vec3| {
                let q = p - o;
                q.dot(ax.a).atan2(q.dot(x) - major)
            };
            let tm = 0.5 * (t0 + t1);
            let (pm, dm) = (c.eval(tm), c.deriv(tm));
            let q = pm - o;
            let (rr, zz) = (q.dot(x) - major, q.dot(ax.a));
            let s = if rr * dm.dot(ax.a) - zz * dm.dot(x) >= 0.0 { 1.0 } else { -1.0 };
            let f0 = phi(pm) - s * tm;
            let surf = if major == 0.0 {
                Surface::Sphere { frame: fr, r: *r }
            } else {
                Surface::Torus { frame: fr, major, minor: *r }
            };
            (surf, Kind::Ang { f0, f1: s })
        }
        _ => (
            Surface::Revolution { profile: Box::new(c.clone()), axis_o: ax.o, axis: ax.a },
            Kind::Ang { f0: 0.0, f1: 1.0 },
        ),
    };
    let (cs, sn) = (trig::cos(sweep), trig::sin(sweep));
    let (pc_bottom, pc_top, rail_pc): (Curve2, Curve2, Box<dyn Fn(f64) -> Curve2>) = match kind {
        Kind::Polar { rho0, rho1 } => (
            Curve2::Line { o: Vec2::new(rho0, 0.0), d: Vec2::new(rho1, 0.0) },
            Curve2::Line { o: Vec2::new(rho0 * cs, rho0 * sn), d: Vec2::new(rho1 * cs, rho1 * sn) },
            Box::new(move |t: f64| Curve2::Circle { c: Vec2::ZERO, x: Vec2::X, r: rho0 + rho1 * t, ccw: true }),
        ),
        Kind::Ang { f0, f1 } => (
            Curve2::Line { o: Vec2::new(0.0, f0), d: Vec2::new(0.0, f1) },
            Curve2::Line { o: Vec2::new(sweep, f0), d: Vec2::new(0.0, f1) },
            Box::new(move |t: f64| Curve2::Line { o: Vec2::new(0.0, f0 + f1 * t), d: Vec2::new(1.0, 0.0) }),
        ),
    };
    Some(SideGeom {
        surface,
        sense: true,
        pc_bottom,
        pc_top,
        pc_rail_t0: v_t0.rail.as_ref().map(|_| rail_pc(t0)),
        pc_rail_t1: v_t1.rail.as_ref().map(|_| rail_pc(t1)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check;
    use crate::make::{ProfileLoop, ProfileRegion, ProfileSeg};

    fn rect_prof(x0: f64, x1: f64) -> Profile {
        let p = [Vec2::new(x0, 0.0), Vec2::new(x1, 0.0), Vec2::new(x1, 5.0), Vec2::new(x0, 5.0)];
        let tags = ["e1", "e2", "e3", "e4"];
        let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }])
    }
    fn ids(b: &crate::body::Body) -> Vec<String> {
        b.face_ids().map(|f| b.face_id_str(f)).collect()
    }
    fn wire(b: &crate::body::Body) -> Vec<ck_json::Json> {
        b.edges_json(&|b, e| b.sample_edge(e, 0.01)).as_arr().unwrap().to_vec()
    }

    #[test]
    fn rev360_matches_oracle() {
        let b = revolve("F2", &rect_prof(10.0, 20.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(ids(&b), ["F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"]);
        let kinds: Vec<_> = b.face_ids().map(|f| crate::body::wire_kind(&b.face(f).surface)).collect();
        assert_eq!(kinds, ["plane", "cylinder", "plane", "cylinder"]);
        let w = wire(&b);
        let keys: Vec<_> = w.iter().map(|e| e.get_str("key").unwrap().to_string()).collect();
        assert_eq!(keys, ["F2.S.e1|F2.S.e4#0", "F2.S.e1|F2.S.e2#1", "F2.S.e2|F2.S.e3#2", "F2.S.e3|F2.S.e4#3"]);
        assert_eq!(ck_json::to_string(w[0].get("circle").unwrap()), r#"{"c":[0,0,0],"n":[0,1,0],"r":10}"#);
        assert_eq!(w[0].get_arr("pts").unwrap()[0].as_vec3().unwrap(), [10.0, 0.0, 0.0]);
        // oracle normals: e1 [0,-1,0], e3 [0,1,0]
        let fj = b.faces_json();
        assert_eq!(fj.as_arr().unwrap()[0].get("normal").unwrap().as_vec3().unwrap(), [0.0, -1.0, 0.0]);
    }

    #[test]
    fn rev90_matches_oracle() {
        let b = revolve("F2", &rect_prof(10.0, 20.0), Vec3::ZERO, Vec3::Y, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(ids(&b), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"]);
        let fj = b.faces_json();
        // oracle: C0 normal [0,0,1], C1 normal [-1,0,-6.1e-17]
        assert_eq!(fj.as_arr().unwrap()[0].get("normal").unwrap().as_vec3().unwrap(), [0.0, 0.0, 1.0]);
        let n1 = fj.as_arr().unwrap()[1].get("normal").unwrap().as_vec3().unwrap();
        assert!((n1[0] + 1.0).abs() < 1e-15 && n1[2].abs() < 1e-15);
        let w = wire(&b);
        assert_eq!(w.len(), 12);
        assert_eq!(w[5].get_str("key"), Some("F2.C1|F2.S.e1#5"));
        let p = w[0].get_arr("pts").unwrap().last().unwrap().as_vec3().unwrap();
        assert!(p[0].abs() < 1e-12 && (p[2] + 10.0).abs() < 1e-12); // ends at [6e-16,0,-10]
    }

    #[test]
    fn on_axis_segments() {
        // oracle revax90: e4 on the axis -> no face, C0|C1#8 edge last
        let b = revolve("F2", &rect_prof(0.0, 20.0), Vec3::ZERO, Vec3::Y, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(ids(&b), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2", "F2.S.e3"]);
        let keys: Vec<_> = wire(&b).iter().map(|e| e.get_str("key").unwrap().to_string()).collect();
        assert_eq!(
            keys,
            [
                "F2.S.e1|F2.S.e2#0",
                "F2.S.e2|F2.S.e3#1",
                "F2.C0|F2.S.e1#2",
                "F2.C1|F2.S.e1#3",
                "F2.C0|F2.S.e2#4",
                "F2.C1|F2.S.e2#5",
                "F2.C0|F2.S.e3#6",
                "F2.C1|F2.S.e3#7",
                "F2.C0|F2.C1#8"
            ]
        );
        let b = revolve("F2", &rect_prof(0.0, 20.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(ids(&b), ["F2.S.e1", "F2.S.e2", "F2.S.e3"]);
        assert_eq!(wire(&b).len(), 2);
    }

    #[test]
    fn sphere_torus_cone_kinds() {
        let fr = Frame::identity();
        let c = Vec2::ZERO;
        let semi = ProfileLoop::new(vec![
            ProfileSeg::arc("e1", c, Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)),
            ProfileSeg::line("e2", Vec2::new(0.0, 10.0), Vec2::new(0.0, -10.0)),
        ]);
        let p = Profile::new(fr, vec![ProfileRegion { outer: semi, holes: vec![] }]);
        let b = revolve("F2", &p, Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(ids(&b), ["F2.S.e1"]);
        assert_eq!(crate::body::wire_kind(&b.face(crate::body::FaceId(0)).surface), "sphere");
        assert_eq!(b.wire_edges().len(), 0);

        let tor = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(20.0, 0.0), 5.0)]);
        let p = Profile::new(fr, vec![ProfileRegion { outer: tor, holes: vec![] }]);
        let b = revolve("F2", &p, Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        assert_eq!(crate::body::wire_kind(&b.face(crate::body::FaceId(0)).surface), "torus");

        let tri = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(5.0, 0.0), Vec2::new(15.0, 0.0)),
            ProfileSeg::line("e2", Vec2::new(15.0, 0.0), Vec2::new(5.0, 10.0)),
            ProfileSeg::line("e3", Vec2::new(5.0, 10.0), Vec2::new(5.0, 0.0)),
        ]);
        let p = Profile::new(fr, vec![ProfileRegion { outer: tri, holes: vec![] }]);
        let b = revolve("F2", &p, Vec3::ZERO, Vec3::Y, 0.0, 120f64.to_radians()).unwrap();
        assert!(check(&b, 1e-9).is_empty(), "{:?}", check(&b, 1e-9));
        let kinds: Vec<_> = b.face_ids().map(|f| crate::body::wire_kind(&b.face(f).surface)).collect();
        assert_eq!(kinds, ["plane", "plane", "plane", "cone", "cylinder"]);
        // oracle C1 normal [-0.8660254037844387, 0, 0.4999999999999998]
        let n1 = b.faces_json().as_arr().unwrap()[1].get("normal").unwrap().as_vec3().unwrap();
        assert!((n1[0] + 0.8660254037844387).abs() < 1e-12 && (n1[2] - 0.5).abs() < 1e-12, "{n1:?}");
    }
}
