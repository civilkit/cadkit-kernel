//! Vertex blend of three equal fillets at a convex corner of planar faces
//! (the oracle's `F3.V0`, a sphere patch). RECONSTRUCTED from box probes
//! (`filletCorner3`), not a port of corner.rs (f2419 / f2420 ...).
//!
//! Geometry: the rolling ball touching the three planes sits at `c` (offset
//! planes meet there); the rounded corner is the part of the body within
//! `r` of the inner corner, i.e. the three edge cylinders and, where all
//! three offsets are positive, the ball. The mitre left by three sequential
//! edge fillets keeps too much material there; cutting the "corner block
//! minus ball" tool removes exactly the difference:
//!
//! * block `H` = the corner region bounded by the three face planes pushed
//!   OUT by a margin and the three planes through `c` perpendicular to the
//!   edges (the block's inner side), a hexahedron;
//! * tool `T = H - ball(c, r)`.
//!
//! `T`'s inner planes pass through the ball's centre (great circles), and
//! the ball is tangent to the edge cylinders along the circles those planes
//! cut; the block is pushed out so nothing but the ball touches the body's
//! planes.

use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, VertexId};

/// The three selected edges meeting at a convex 3-valent vertex of planar
/// faces: the vertex, its three edges (ascending ids) and the face opposite
/// to each edge (the one not containing it).
pub struct Corner {
    pub vertex: VertexId,
    pub edges: [EdgeId; 3],
    pub faces: [FaceId; 3],
    /// All three edges are concave (the void side of the vertex is a trihedral
    /// octant): the corner is filled with material (union), not cut.
    pub concave: bool,
}

/// Corner at the vertex shared by exactly the three `selected` edges, or
/// `None` (not 3-valent / not planar / not straight).
pub fn find_corner(b: &Body, selected: &[EdgeId]) -> Vec<Corner> {
    let mut out = vec![];
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 3 || !inc.iter().all(|e| selected.contains(e)) {
            continue;
        }
        if inc.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1 || !matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. })) {
            continue;
        }
        // the three faces at the vertex, each planar
        let mut faces: Vec<FaceId> = vec![];
        for &e in &inc {
            for f in b.edge_faces(e) {
                if !faces.contains(&f) {
                    faces.push(f);
                }
            }
        }
        if faces.len() != 3 || faces.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
            continue;
        }
        let opposite = |e: EdgeId| faces.iter().copied().find(|f| !b.edge_faces(e).contains(f));
        let (Some(f0), Some(f1), Some(f2)) = (opposite(inc[0]), opposite(inc[1]), opposite(inc[2])) else { continue };
        // convexity: along edge i the face opposite to it leans toward the
        // body (convex, n.u < 0) or toward the void (concave, n.u > 0)
        let vp = b.vertex(v).p;
        let mut sgn = [0i32; 3];
        for (i, (&e, &f)) in inc.iter().zip([f0, f1, f2].iter()).enumerate() {
            let ed = b.edge(e);
            let other = if ed.v0 == v { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
            let d = b.planar_normal(f).dot((other - vp).normalize());
            sgn[i] = if d > 1e-9 { 1 } else if d < -1e-9 { -1 } else { 0 };
        }
        let concave = sgn.iter().all(|&s| s > 0);
        if !concave && !sgn.iter().all(|&s| s < 0) {
            continue; // mixed or degenerate vertex: no spherical corner
        }
        out.push(Corner { vertex: v, edges: [inc[0], inc[1], inc[2]], faces: [f0, f1, f2], concave });
    }
    out
}

/// The tool `H - ball` for `corner` (see the module doc), its sphere face
/// named `name`. `Err(blendUnsupported)` for a concave / degenerate corner.
pub fn sphere_corner_tool(b: &Body, corner: &Corner, r: f64, feature: &str, name: &FaceName, tol: f64) -> Result<Body, BlendError> {
    let vp = b.vertex(corner.vertex).p;
    let n: Vec<Vec3> = corner.faces.iter().map(|&f| b.planar_normal(f)).collect();
    // ball centre: r below each of the three planes
    // concave: the ball sits on the void side (r above each plane), the tool
    // is the block MINUS the ball, added to the body
    let sg = if corner.concave { 1.0 } else { -1.0 };
    let c = crate::intersect::solve3([n[0], n[1], n[2]], [sg * r, sg * r, sg * r]).map(|d| vp + d).ok_or_else(|| BlendError::code("blendUnsupported"))?;
    // edge directions away from the vertex
    let mut u = [Vec3::ZERO; 3];
    for (i, &e) in corner.edges.iter().enumerate() {
        let ed = b.edge(e);
        let other = if ed.v0 == corner.vertex { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
        u[i] = (other - vp).normalize();
        // the ball centre lies along every edge of the vertex
        if (c - vp).dot(u[i]) <= 0.0 {
            return Err(BlendError::code("blendUnsupported"));
        }
    }
    // the corner block: out(i) = plane of the face opposite to edge i, pushed
    // out by `m`; inn(i) = plane through c perpendicular to edge i
    let m = r + 1.0;
    // concave: the block is pushed into the material instead (normal -n)
    let out_pl = |i: usize| (n[i] * -sg, vp - n[i] * (sg * m)); // (outward normal, point)
    let inn_pl = |i: usize| (u[i], c);
    let mut verts: Vec<Vec3> = vec![];
    for mask in 0..8usize {
        let pl: Vec<(Vec3, Vec3)> = (0..3).map(|i| if mask >> i & 1 == 1 { inn_pl(i) } else { out_pl(i) }).collect();
        let p = crate::intersect::solve3([pl[0].0, pl[1].0, pl[2].0], [pl[0].0.dot(pl[0].1), pl[1].0.dot(pl[1].1), pl[2].0.dot(pl[2].1)])
            .ok_or_else(|| BlendError::code("blendUnsupported"))?;
        verts.push(p);
    }
    // The block minus the ball octant at its corner `c = verts[7]`, built by
    // hand (a revolved sphere has no usable boundary for the boolean): the
    // corner vertex is replaced by the three points `a[i]` where the ball
    // touches the body's faces; the inner faces get a quarter-circle notch
    // and the octant becomes the (cavity-facing) sphere patch.
    let cc = verts[7];
    let e: Vec<Vec3> = (0..3).map(|i| (verts[7 ^ (1 << i)] - cc).normalize()).collect();
    let a: Vec<Vec3> = (0..3).map(|i| cc + e[i] * r).collect();
    let mut tb = ToolBuilder::new();
    let ids_v: Vec<usize> = (0..8).map(|mask| if mask == 7 { usize::MAX } else { tb.vertex(verts[mask]) }).collect();
    let ids_a: Vec<usize> = a.iter().map(|&p| tb.vertex(p)).collect();
    // outer faces: the quads of the vertices with bit i clear
    for i in 0..3 {
        let quad: Vec<usize> = (0..8usize).filter(|mask| mask >> i & 1 == 0).map(|mask| ids_v[mask]).collect();
        let segs = tb.cycle_by_angle(&quad, n[i] * -sg);
        tb.plane_face(n[i] * -sg, segs, FaceName::side(feature, &format!("k{}0", i)));
    }
    // inner faces: A_j, mask_j, opposite, mask_k, A_k and the arc back
    for i in 0..3 {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        let mj = 7 ^ (1 << j);
        let mk = 7 ^ (1 << k);
        let opp = 7 ^ (1 << j) ^ (1 << k);
        let pts = [ids_a[j], ids_v[mj], ids_v[opp], ids_v[mk], ids_a[k]];
        let mut segs: Vec<Seg> = (0..4).map(|t| Seg { a: pts[t], b: pts[t + 1], arc: None }).collect();
        segs.push(Seg { a: pts[4], b: pts[0], arc: Some((cc, r)) });
        tb.plane_face(u[i], segs, FaceName::side(feature, &format!("k{}1", i)));
    }
    // the sphere patch: the three arcs; outward normal toward the centre
    let segs = vec![
        Seg { a: ids_a[0], b: ids_a[1], arc: Some((cc, r)) },
        Seg { a: ids_a[1], b: ids_a[2], arc: Some((cc, r)) },
        Seg { a: ids_a[2], b: ids_a[0], arc: Some((cc, r)) },
    ];
    tb.sphere_face(cc, r, segs, name.clone());
    Ok(tb.finish())
}

/// Give the coedges of the sphere patch `name` exact straight pcurves again
/// (the boolean refits them by projection, which is off by ~1e-4 near the
/// pole). Valid for the orthogonal corner, where [`sphere_corner_tool`]
/// puts the pole at a patch corner so every arc is a meridian or the equator.
pub fn fix_sphere_pcurves(body: &mut Body, name: &str) {
    let Some(f) = body.find_face(name) else { return };
    let Surface::Sphere { frame, r } = body.face(f).surface.clone() else { return };
    let c = frame.o;
    let uo = |p: Vec3| (p - c).dot(frame.y).atan2((p - c).dot(frame.x));
    let is_pole = |p: Vec3| (p - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
    let mut fixes: Vec<(usize, ck_geom::curve2::Curve2)> = vec![];
    for &l in &body.face(f).loops {
        for &cid in &body.lp(l).coedges {
            let ed = body.edge(body.coedge(cid).edge);
            if !matches!(ed.curve, Curve::Circle { .. }) {
                return;
            }
            let (pa, pb) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
            let (mut ua, mut ub) = (uo(pa), uo(pb));
            if is_pole(pa) {
                ua = ub;
            } else if is_pole(pb) {
                ub = ua;
            }
            let v = |p: Vec3| ((p - c) / r).dot(frame.z).clamp(-1.0, 1.0).asin();
            let mut du = ub - ua;
            while du > std::f64::consts::PI {
                du -= std::f64::consts::TAU;
            }
            while du < -std::f64::consts::PI {
                du += std::f64::consts::TAU;
            }
            let th = ed.t1 - ed.t0;
            let d = ck_geom::Vec2::new(du / th, (v(pb) - v(pa)) / th);
            let o = ck_geom::Vec2::new(ua, v(pa)) - d * ed.t0;
            fixes.push((cid.idx(), ck_geom::curve2::Curve2::Line { o, d }));
        }
    }
    for (i, pc) in fixes {
        body.coedges[i].pcurve = Some(pc);
    }
}

/// One directed loop segment of [`ToolBuilder`]: a line, or an arc of the
/// circle through both ends about `centre` (the shorter way).
#[derive(Clone)]
struct Seg {
    a: usize,
    b: usize,
    arc: Option<(Vec3, f64)>,
}

/// Minimal B-rep builder for the corner tool: planar faces with line / arc
/// edges and one spherical patch, pcurves left to the projection fallback.
struct ToolBuilder {
    body: Body,
    verts: Vec<(Vec3, VertexId)>,
    edges: Vec<((usize, usize, bool), EdgeId)>,
    shell: ck_topo::body::ShellId,
}

impl ToolBuilder {
    fn new() -> ToolBuilder {
        let mut body = Body::new();
        let lump = body.add_lump();
        let shell = body.add_shell(lump);
        ToolBuilder { body, verts: vec![], edges: vec![], shell }
    }
    fn vertex(&mut self, p: Vec3) -> usize {
        let v = self.body.add_vertex(p);
        self.verts.push((p, v));
        self.verts.len() - 1
    }
    /// The quad `idx` as segments counter-clockwise about `normal`.
    fn cycle_by_angle(&self, idx: &[usize], normal: Vec3) -> Vec<Seg> {
        let pts: Vec<Vec3> = idx.iter().map(|&i| self.verts[i].0).collect();
        let ctr = pts.iter().fold(Vec3::ZERO, |acc, &p| acc + p) / pts.len() as f64;
        let ex = (pts[0] - ctr).normalize();
        let ey = normal.cross(ex);
        let mut order: Vec<usize> = (0..idx.len()).collect();
        order.sort_by(|&x, &y| {
            let ang = |i: usize| (pts[i] - ctr).dot(ey).atan2((pts[i] - ctr).dot(ex));
            ang(x).total_cmp(&ang(y))
        });
        (0..order.len()).map(|t| Seg { a: idx[order[t]], b: idx[order[(t + 1) % order.len()]], arc: None }).collect()
    }
    fn seg_points(&self, s: &Seg) -> Vec<Vec3> {
        let (pa, pb) = (self.verts[s.a].0, self.verts[s.b].0);
        match s.arc {
            None => vec![pa],
            Some((c, r)) => {
                let (ea, eb) = ((pa - c).normalize(), (pb - c).normalize());
                let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
                let ez = ea.cross(eb).normalize();
                (0..8).map(|k| c + (ea * (th * k as f64 / 8.0).cos() + ez.cross(ea) * (th * k as f64 / 8.0).sin()) * r).collect()
            }
        }
    }
    /// Reverse a loop (segments and order).
    fn reversed(segs: Vec<Seg>) -> Vec<Seg> {
        segs.into_iter().rev().map(|s| Seg { a: s.b, b: s.a, arc: s.arc }).collect()
    }
    fn edge(&mut self, s: &Seg) -> (EdgeId, bool) {
        let key = (s.a.min(s.b), s.a.max(s.b), s.arc.is_some());
        let rev = s.a > s.b;
        if let Some((_, e)) = self.edges.iter().find(|(k, _)| *k == key) {
            return (*e, rev);
        }
        // edge direction low index -> high index
        let (lo, hi) = (key.0, key.1);
        let (pa, pb) = (self.verts[lo].0, self.verts[hi].0);
        let (curve, t1) = match s.arc {
            None => {
                let len = pa.dist(pb);
                (Curve::Line { o: pa, d: (pb - pa) / len }, len)
            }
            Some((c, r)) => {
                let (ea, eb) = ((pa - c).normalize(), (pb - c).normalize());
                let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
                let ez = ea.cross(eb).normalize();
                (Curve::Circle { frame: Frame { o: c, x: ea, y: ez.cross(ea), z: ez }, r }, th)
            }
        };
        let e = self.body.add_edge(curve, 0.0, t1, self.verts[lo].1, self.verts[hi].1);
        self.edges.push((key, e));
        (e, rev)
    }
    fn plane_face(&mut self, normal: Vec3, mut segs: Vec<Seg>, name: FaceName) {
        // counter-clockwise about `normal`
        let mut area = Vec3::ZERO;
        let pts: Vec<Vec3> = segs.iter().flat_map(|s| self.seg_points(s)).collect();
        for i in 0..pts.len() {
            area = area + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        if area.dot(normal) < 0.0 {
            segs = ToolBuilder::reversed(segs);
        }
        let frame = ck_geom::frame_from(pts[0], normal, pts[1] - pts[0]);
        let f = self.body.add_face(self.shell, Surface::Plane { frame }, true, name);
        let lp = self.body.add_loop(f);
        for s in &segs {
            let (e, rev) = self.edge(s);
            let pc = ck_topo::make::plane_pcurve(&frame, &self.body.edge(e).curve.clone());
            self.body.add_coedge(lp, e, rev, pc);
        }
    }
    fn sphere_face(&mut self, c: Vec3, r: f64, mut segs: Vec<Seg>, name: FaceName) {
        // outward normal of the tool here points at the centre; the loop runs
        // counter-clockwise about it
        let pts: Vec<Vec3> = segs.iter().map(|s| self.verts[s.a].0).collect();
        let ctr = pts.iter().fold(Vec3::ZERO, |acc, &p| acc + p) / pts.len() as f64;
        let nrm = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
        if nrm.dot(c - ctr) < 0.0 {
            segs = ToolBuilder::reversed(segs);
        }
        // frame: the pole at the first patch corner (so the two arcs through
        // it are meridians and the opposite arc is the equator: all straight
        // in uv when the corner is orthogonal)
        let axis = (pts[0] - c).normalize();
        let frame = ck_geom::frame_from(c, axis, pts[1] - c);
        let f = self.body.add_face(self.shell, Surface::Sphere { frame, r }, false, name);
        let lp = self.body.add_loop(f);
        let uv_of = |p: Vec3, other_u: f64| {
            let d = (p - c) / r;
            let v = d.dot(frame.z).clamp(-1.0, 1.0).asin();
            let u = if v.abs() > std::f64::consts::FRAC_PI_2 - 1e-9 { other_u } else { d.dot(frame.y).atan2(d.dot(frame.x)) };
            ck_geom::Vec2::new(u, v)
        };
        let orthogonal = (pts[1] - c).normalize().dot((pts[0] - c).normalize()).abs() < 1e-9;
        for s in &segs {
            let (e, rev) = self.edge(s);
            let pc = if orthogonal {
                let ed = self.body.edge(e).clone();
                let (pa, pb) = (self.body.vertex(ed.v0).p, self.body.vertex(ed.v1).p);
                let uo = |p: Vec3| ((p - c).dot(frame.y)).atan2((p - c).dot(frame.x));
                let (ua, ub) = (uo(pa), uo(pb));
                let pole_a = (pa - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
                let pole_b = (pb - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
                let (ua, ub) = if pole_a { (ub, ub) } else if pole_b { (ua, ua) } else { (ua, ub) };
                let (a, bb) = (uv_of(pa, ua), uv_of(pb, ub));
                // equator arcs: u runs the short way
                let mut du = bb.x - a.x;
                while du > std::f64::consts::PI {
                    du -= std::f64::consts::TAU;
                }
                while du < -std::f64::consts::PI {
                    du += std::f64::consts::TAU;
                }
                let th = ed.t1 - ed.t0;
                Some(ck_geom::curve2::Curve2::Line { o: a, d: ck_geom::Vec2::new(du / th, (bb.y - a.y) / th) })
            } else {
                None
            };
            self.body.add_coedge(lp, e, rev, pc);
        }
    }
    fn finish(mut self) -> Body {
        ck_topo::region::rebuild_lumps(&mut self.body);
        self.body
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::build::{apply_tool, blend_edges, find_edge, Spec};
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    #[test]
    fn box_corner_sphere() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let sel = [
            ("F2.C1".to_string(), "F2.S.e1".to_string(), Vec3::new(20., 10., 0.)),
            ("F2.C1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 10., -10.)),
            ("F2.S.e1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 5., 0.)),
        ];
        let mitred = blend_edges(&b, &sel, Spec::Fillet(2.0, crate::blend::build::FilletProfile::Circular), "F3", 1e-6).unwrap();
        let ids: Vec<EdgeId> = sel.iter().map(|(a, bb, at)| find_edge(&b, a, bb, *at).unwrap()).collect();
        let corners = find_corner(&b, &ids);
        assert_eq!(corners.len(), 1);
        let name = FaceName::other("F3", "V0");
        let tool = sphere_corner_tool(&b, &corners[0], 2.0, "F3", &name, 1e-6).unwrap();
        assert!(ck_topo::check::check(&tool, 1e-6).is_empty());
        // block 5^3 minus the ball octant
        assert!((mass_props(&tool).volume - (125.0 - std::f64::consts::PI * 4.0 / 3.0)).abs() < 1e-9);
        let (mut r, _) = apply_tool(&mitred, &tool, Op::Cut, &name, vec![], 1e-6).unwrap();
        fix_sphere_pcurves(&mut r, &name.id());
        assert!(ck_topo::check::check(&r, 1e-6).is_empty());
        let faces: Vec<String> = r.face_ids().map(|f| r.face_id_str(f)).collect();
        assert_eq!(faces[faces.len() - 4..], ["F3.B0", "F3.B1", "F3.B2", "F3.V0"]);
        // oracle volume of filletCorner3 (exact: 8000 - 64(1-pi/4)4/4... see tools/brep_cases.mjs)
        assert!((mass_props(&r).volume - 7941.250719950454).abs() < 1e-6, "{}", mass_props(&r).volume);
    }
}
