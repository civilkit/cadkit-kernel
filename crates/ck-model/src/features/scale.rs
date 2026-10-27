//! `scale`: scale one or more bodies about a point, the same factor on every axis or one per axis.
//!
//! ```json
//! {"type":"scale","id":"F9","factor":2,"at":[0,0,0],"bodies":[{"body":"Shaft"}]}
//! {"type":"scale","id":"F9","factor":[1,1.09,1],"at":[177,0,165],"bodies":[{"body":"Shaft"}]}
//! ```
//!
//! * `factor`: one number (uniform) or `[sx, sy, sz]` along the world axes (`factors` is read as a synonym); every
//!   factor is a number or an expression and must be finite and above zero (a negative one would mirror the body).
//! * `at` (`origin` too): the fixed point; without one `about` says: `"origin"`, or `"centroid"` (the default, as
//!   the builder writes it): the centre of mass of the selection. The map is `x -> at + F (x - at)`.
//! * the selection is the one of `moveBody` (`bodies`, `body`, `target`; none: every lump of the solid body).
//!
//! The body is changed in place and every entity keeps its name. The geometry stays exact: a plane stays a plane; a
//! cylinder or cone stretched along its own axis (and by one factor across it) stays one; a sphere or torus under a
//! uniform factor stays one; B-spline faces and edges map their control points; a circle becomes an ellipse where the
//! section is stretched. What cannot be kept exact is refused as `invalidValue` with a `detail` and a `hint`, and
//! the body is left as it was: a cylinder or cone stretched across its axis (an elliptic cylinder), a sphere or torus
//! under a non-uniform factor, and a revolution, extrusion, screw or offset surface (uniform or not, not yet).

use super::move_body::{select, vec3};
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Mat3};
use ck_core::xform::Xform;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_json::Json;
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId};

fn invalid(detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()))
}

fn refuse(detail: &str, hint: &str) -> FeatureError {
    invalid(detail).with("hint", Json::Str(hint.into()))
}

const HINT_ACROSS: &str = "scale it by one factor on all three axes, or only along the axis of the round faces (a cylinder stretched along its own axis stays exact)";

fn num(state: &RegenState, j: Option<&Json>) -> Result<f64, FeatureError> {
    crate::expr::eval_field(j, &state.vars, f64::NAN).map_err(FeatureError::new)
}

/// `factor` / `factors`: one number or three.
fn factors(state: &RegenState, raw: &Json) -> Result<[f64; 3], FeatureError> {
    let j = raw.get("factors").filter(|j| j.as_arr().is_some()).or_else(|| raw.get("factor"));
    let f = match j {
        Some(Json::Arr(a)) if a.len() == 3 => [num(state, a.first())?, num(state, a.get(1))?, num(state, a.get(2))?],
        Some(Json::Arr(_)) => return Err(invalid("a scale factor needs one number or three")),
        Some(Json::Null) | None => return Err(invalid("scale needs a factor")),
        Some(one) => {
            let s = num(state, Some(one))?;
            [s, s, s]
        }
    };
    if !f.iter().all(|v| v.is_finite()) {
        return Err(invalid("non-finite scale factor"));
    }
    if f.iter().any(|v| *v <= 0.0) {
        return Err(refuse("a scale factor must be above zero", "a negative factor would mirror the body: use mirror"));
    }
    Ok(f)
}

/// 2 x 2 linear map of a face's (u, v), applied to its pcurves.
#[derive(Clone, Copy)]
struct Uv {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

impl Uv {
    const ID: Uv = Uv { a: 1.0, b: 0.0, c: 0.0, d: 1.0 };
    fn diag(u: f64, v: f64) -> Uv {
        Uv { a: u, b: 0.0, c: 0.0, d: v }
    }
    fn apply(&self, q: Vec2) -> Vec2 {
        Vec2::new(self.a * q.x + self.b * q.y, self.c * q.x + self.d * q.y)
    }
    fn is_identity(&self) -> bool {
        self.a == 1.0 && self.b == 0.0 && self.c == 0.0 && self.d == 1.0
    }
    /// The same factor on both parameters (a circle stays a circle).
    fn similarity(&self) -> Option<f64> {
        (self.b.abs() <= 1e-12 * self.a.abs() && self.c.abs() <= 1e-12 * self.a.abs() && same(self.a, self.d)).then_some(self.a)
    }
}

fn same(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs())
}

fn orthogonal(a: Vec3, b: Vec3) -> bool {
    a.dot(b).abs() <= 1e-9 * a.len() * b.len()
}

/// The map: linear part `m`, and the factor of a uniform scale (`None` when the axes differ).
struct Map {
    x: Xform,
    uniform: Option<f64>,
}

impl Map {
    fn p(&self, q: Vec3) -> Vec3 {
        self.x.apply_point(q)
    }
    fn v(&self, d: Vec3) -> Vec3 {
        self.x.apply_vec(d)
    }
    /// The frame whose axes are the unit images of the given frame's, if it stays orthogonal.
    fn frame(&self, f: &Frame) -> Option<(Frame, f64, f64)> {
        let (x, y) = (self.v(f.x), self.v(f.y));
        if !orthogonal(x, y) {
            return None;
        }
        let (kx, ky) = (x.len(), y.len());
        let (xh, yh) = (x * (1.0 / kx), y * (1.0 / ky));
        Some((Frame { o: self.p(f.o), x: xh, y: yh, z: xh.cross(yh) }, kx, ky))
    }
}

/// An edge curve mapped, with the factor its parameter scales by (a line's is its length factor).
fn map_curve(c: &Curve, m: &Map) -> Result<(Curve, f64), FeatureError> {
    Ok(match c {
        Curve::Line { o, d } => {
            let d2 = m.v(*d);
            let l = d2.len();
            (Curve::Line { o: m.p(*o), d: d2 * (1.0 / l) }, l)
        }
        Curve::Circle { frame, r } => {
            let (fr, kx, ky) = m.frame(frame).ok_or_else(|| refuse("a circle edge would be sheared", HINT_ACROSS))?;
            if same(kx, ky) {
                (Curve::Circle { frame: fr, r: r * kx }, 1.0)
            } else {
                (Curve::Ellipse { frame: fr, a: r * kx, b: r * ky }, 1.0)
            }
        }
        Curve::Ellipse { frame, a, b } => {
            let (fr, kx, ky) = m.frame(frame).ok_or_else(|| refuse("an ellipse edge would be sheared", HINT_ACROSS))?;
            (Curve::Ellipse { frame: fr, a: a * kx, b: b * ky }, 1.0)
        }
        Curve::BSpline(s) => (Curve::BSpline(s.map(|q| m.p(q))), 1.0),
        Curve::Conic { .. } | Curve::Sinusoid { .. } => {
            let k = m.uniform.ok_or_else(|| refuse("a conic or sinusoid edge needs one factor on all axes", HINT_ACROSS))?;
            let t = c.transform(|q| m.p(q), |d| m.v(d));
            (t, if matches!(c, Curve::Sinusoid { .. }) { k } else { 1.0 })
        }
    })
}

/// A pcurve under the face's (u, v) map; `lam` is the factor its edge's parameter scales by (the pcurve is read at the
/// edge's parameter, so a line edge that grows takes its pcurve's speed down by the same factor).
fn map_pcurve(c: &Curve2, uv: &Uv, lam: f64) -> Result<Curve2, FeatureError> {
    let bad = || refuse("a pcurve would change shape", HINT_ACROSS);
    Ok(match c {
        Curve2::Line { o, d } => Curve2::Line { o: uv.apply(*o), d: uv.apply(*d) * (1.0 / lam) },
        Curve2::BSpline(s) if lam == 1.0 => Curve2::BSpline(s.map(|q| uv.apply(q))),
        Curve2::Circle { c, x, r, ccw } if lam == 1.0 => {
            let k = uv.similarity().ok_or_else(bad)?;
            Curve2::Circle { c: uv.apply(*c), x: *x, r: r * k, ccw: *ccw }
        }
        Curve2::Ellipse { c, x, a, b, ccw } if lam == 1.0 => {
            let k = uv.similarity().ok_or_else(bad)?;
            Curve2::Ellipse { c: uv.apply(*c), x: *x, a: a * k, b: b * k, ccw: *ccw }
        }
        _ => return Err(bad()),
    })
}

/// A surface mapped, with the (u, v) map of its pcurves.
fn map_surface(s: &Surface, m: &Map) -> Result<(Surface, Uv), FeatureError> {
    // a round face keeps its section when the two directions across the axis are scaled alike and stay square to it
    let round = |frame: &Frame| -> Result<(Frame, f64, f64), FeatureError> {
        let (x, y, z) = (m.v(frame.x), m.v(frame.y), m.v(frame.z));
        let ok = orthogonal(x, y) && orthogonal(x, z) && orthogonal(y, z) && same(x.len(), y.len());
        if !ok {
            return Err(refuse("a round face (cylinder or cone) would be stretched across its axis", HINT_ACROSS));
        }
        let k = x.len();
        Ok((Frame { o: m.p(frame.o), x: x * (1.0 / k), y: y * (1.0 / k), z: z * (1.0 / z.len()) }, k, z.len()))
    };
    let uniform = |what: &str| m.uniform.ok_or_else(|| refuse(&format!("a {what} needs one factor on all axes"), HINT_ACROSS));
    Ok(match s {
        Surface::Plane { frame } => {
            // any plane stays a plane: x, then the image normal, then y; (u, v) maps by the image of the old x and y
            let (x, y) = (m.v(frame.x), m.v(frame.y));
            let xh = x * (1.0 / x.len());
            let zh = x.cross(y);
            let zh = zh * (1.0 / zh.len());
            let yh = zh.cross(xh);
            let uv = Uv { a: xh.dot(x), b: xh.dot(y), c: yh.dot(x), d: yh.dot(y) };
            (Surface::Plane { frame: Frame { o: m.p(frame.o), x: xh, y: yh, z: zh } }, uv)
        }
        Surface::Cylinder { frame, r } => {
            let (fr, k, sz) = round(frame)?;
            (Surface::Cylinder { frame: fr, r: r * k }, Uv::diag(1.0, sz))
        }
        Surface::Cone { frame, r, semi_angle } => {
            let (fr, k, sz) = round(frame)?;
            (Surface::Cone { frame: fr, r: r * k, semi_angle: (semi_angle.tan() * k / sz).atan() }, Uv::diag(1.0, sz))
        }
        Surface::Sphere { frame, r } => {
            let k = uniform("sphere")?;
            let (fr, _, _) = m.frame(frame).ok_or_else(|| refuse("a sphere would be sheared", HINT_ACROSS))?;
            (Surface::Sphere { frame: fr, r: r * k }, Uv::ID)
        }
        Surface::Torus { frame, major, minor } => {
            let k = uniform("torus")?;
            let (fr, _, _) = m.frame(frame).ok_or_else(|| refuse("a torus would be sheared", HINT_ACROSS))?;
            (Surface::Torus { frame: fr, major: major * k, minor: minor * k }, Uv::ID)
        }
        Surface::BSpline(b) => {
            let mut b = b.clone();
            b.ctrl.iter_mut().for_each(|q| *q = m.p(*q));
            (Surface::BSpline(b), Uv::ID)
        }
        Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Screw { .. } | Surface::Offset { .. } => {
            return Err(refuse(
                "this face is a revolution, extrusion, screw or offset surface, whose scaling is not built yet",
                "rebuild the body from a sketch at the size you need, or scale a body made of planes, cylinders, cones, spheres and free-form faces",
            ))
        }
    })
}

/// Scale the entities of whole lumps (every vertex, edge and face of `faces` belongs to them alone). All is checked
/// before anything changes: a refused face leaves the body as it was.
fn scale_faces(b: &mut Body, faces: &[FaceId], m: &Map) -> Result<(), FeatureError> {
    let mut vs = vec![false; b.vertices.len()];
    let mut es: Vec<Option<(Curve, f64)>> = vec![None; b.edges.len()];
    let mut surfaces = Vec::with_capacity(faces.len());
    let mut coedges: Vec<(CoedgeId, FaceId, Curve2)> = vec![];
    let mut edge_ids: Vec<EdgeId> = vec![];
    for &f in faces {
        surfaces.push(map_surface(&b.face(f).surface, m)?);
        for &l in &b.face(f).loops.clone() {
            for &c in &b.lp(l).coedges.clone() {
                let e = b.coedge(c).edge;
                if es[e.idx()].is_none() {
                    let ed = b.edge(e);
                    es[e.idx()] = Some(map_curve(&ed.curve, m)?);
                    vs[ed.v0.idx()] = true;
                    vs[ed.v1.idx()] = true;
                    edge_ids.push(e);
                }
            }
        }
    }
    // pcurves, now that every edge's parameter factor is known
    for (&f, (_, uv)) in faces.iter().zip(&surfaces) {
        for &l in &b.face(f).loops.clone() {
            for &c in &b.lp(l).coedges.clone() {
                let Some(pc) = &b.coedge(c).pcurve else { continue };
                let lam = es[b.coedge(c).edge.idx()].as_ref().map_or(1.0, |e| e.1);
                if uv.is_identity() && lam == 1.0 {
                    continue;
                }
                coedges.push((c, f, map_pcurve(pc, uv, lam)?));
            }
        }
    }
    // ---- apply
    let kmax = m.v(Vec3::X).len().max(m.v(Vec3::Y).len()).max(m.v(Vec3::Z).len());
    for (i, v) in b.vertices.iter_mut().enumerate() {
        if vs[i] {
            v.p = m.p(v.p);
            v.tol *= kmax;
        }
    }
    for e in edge_ids {
        let (curve, lam) = es[e.idx()].take().unwrap();
        let ed = &mut b.edges[e.idx()];
        ed.curve = curve;
        ed.t0 *= lam;
        ed.t1 *= lam;
        ed.tol *= kmax;
    }
    for (c, _, pc) in coedges {
        b.coedges[c.idx()].pcurve = Some(pc);
    }
    for (&f, (s, _)) in faces.iter().zip(surfaces) {
        b.faces[f.idx()].surface = s;
    }
    Ok(())
}

/// The fixed point when none is given (`about: "centroid"`, the default): the centre of mass of the selected solids, or
/// the centre of the box of the selected surface bodies when there is no solid.
fn centroid(br: &Brep, solid_faces: &[FaceId], surf_hit: &[usize]) -> Vec3 {
    if let (Some(b), false) = (&br.body, solid_faces.is_empty()) {
        let mut c = ck_ops::boolean::assemble::rebuild_in_order(b, solid_faces);
        ck_topo::region::rebuild_lumps(&mut c);
        let com = ck_topo::mass::mass_props(&c).com;
        if com.x.is_finite() && com.y.is_finite() && com.z.is_finite() {
            return com;
        }
    }
    let boxes = surf_hit.iter().map(|&i| br.surfs[i].1.bbox());
    let (lo, hi) = boxes.fold((None, None), |(lo, hi): (Option<Vec3>, Option<Vec3>), (a, b)| (Some(lo.map_or(a, |l| l.min(a))), Some(hi.map_or(b, |h| h.max(b)))));
    match (lo, hi) {
        (Some(l), Some(h)) => (l + h) * 0.5,
        _ => Vec3::ZERO,
    }
}

pub fn exec_scale(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() && br.surfs.is_empty() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let fac = factors(state, &f.raw)?;
    let (solid_faces, surf_hit) = select(br, f)?;
    let at = match vec3(state, f.raw.get("at"))? {
        Some(a) => a,
        None => match vec3(state, f.raw.get("origin"))? {
            Some(a) => a,
            None if f.raw.get_str("about") == Some("origin") => Vec3::ZERO,
            None => centroid(br, &solid_faces, &surf_hit),
        },
    };
    if fac == [1.0, 1.0, 1.0] {
        return Ok(());
    }
    let lin = Mat3 { c: [Vec3::new(fac[0], 0.0, 0.0), Vec3::new(0.0, fac[1], 0.0), Vec3::new(0.0, 0.0, fac[2])] };
    let x = Xform { r: lin, t: at - (Xform { r: lin, t: Vec3::ZERO }).apply_vec(at) };
    let uniform = (same(fac[0], fac[1]) && same(fac[1], fac[2])).then_some(fac[0]);
    let m = Map { x, uniform };
    // every selected body is checked before the first changes
    let mut trial: Vec<Body> = vec![];
    let work = |b: &Body, faces: &[FaceId]| -> Result<Body, FeatureError> {
        let mut c = b.clone();
        scale_faces(&mut c, faces, &m)?;
        Ok(c)
    };
    let solid = match (&br.body, solid_faces.is_empty()) {
        (Some(b), false) => Some(work(b, &solid_faces)?),
        _ => None,
    };
    for &i in &surf_hit {
        let b = &br.surfs[i].1;
        let faces: Vec<FaceId> = b.face_ids().collect();
        trial.push(work(b, &faces)?);
    }
    if let Some(s) = solid {
        br.body = Some(s);
    }
    for (i, b) in surf_hit.into_iter().zip(trial) {
        br.surfs[i].1 = b;
    }
    Ok(())
}
