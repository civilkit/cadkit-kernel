//! Vertical corner edges of an extruded free-form profile: a round (an exact cylinder) or an equal chamfer (a plane) built in place
//! on the edge between the walls of two links of the profile (a spline wall and a plane wall, two spline walls, two plane walls).
//!
//! At each end of the edge, with the rim curves `C_a`, `C_b` of the two walls and their inward offsets `K(t) = C(t) - rho n(t)`:
//! the ball centre `c` is where the two offsets meet, the contact points are `C_a(t_a)`, `C_b(t_b)`, and the cylinder's axis runs
//! through `c` along the extrusion. A chamfer cuts at arc length `d` along each wall. The two rim edges are shortened to the contact
//! points, the seam edge is replaced by one seam edge on each wall, and the new face closes the gap; the arc (or chord) on each cap
//! becomes an edge of the cap's loop. What is left is a loop of splines, lines and arcs the rim builder (`spline_rim.rs`) then rounds.

use super::spline_rim::{find_ring, nm, offset_meet, offset_terms, refuse, Miss, Ring, Sec, R};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, FaceName, VertexId};

/// The ring through the cap loop at `v` whose seam there is `s`.
fn ring_at(b: &Body, v: VertexId, s: EdgeId) -> Option<Ring> {
    for x in b.edge_ids() {
        if x == s || b.edge_coedges(x).is_empty() || !(b.edge(x).v0 == v || b.edge(x).v1 == v) {
            continue;
        }
        if let Ok(ring) = find_ring(b, x) {
            if ring.seam.get(&v) == Some(&s) {
                return Some(ring);
            }
        }
    }
    None
}

pub(super) fn is_vertical(b: &Body, s: EdgeId) -> bool {
    let sd = b.edge(s);
    if !matches!(sd.curve, Curve::Line { .. }) || sd.v0 == sd.v1 || b.edge_faces(s).len() != 2 {
        return false;
    }
    [sd.v0, sd.v1].iter().all(|&v| ring_at(b, v, s).is_some())
}

/// One end of the vertical edge.
struct EndGeom {
    v: VertexId,
    cap: FaceId,
    ea: EdgeId,
    eb: EdgeId,
    ta: f64,
    tb: f64,
    pa: Vec3,
    pb: Vec3,
    /// ball centre (round), at cap level
    c: Vec3,
    na: Vec3,
    nb: Vec3,
    cos_th: f64,
    cap_frame: Frame,
}

fn end_geom(b: &Body, v: VertexId, s: EdgeId, fa: FaceId, size: f64, chamfer: bool) -> R<EndGeom> {
    let Some(ring) = ring_at(b, v, s) else { return nm(line!()) };
    let dir = ring.dir;
    let find = |face: FaceId| ring.links.iter().position(|l| b.coedge_face(l.c_wall) == face && (b.edge(l.e).v0 == v || b.edge(l.e).v1 == v));
    let faces = b.edge_faces(s);
    let fb = if faces[0] == fa { faces[1] } else { faces[0] };
    let (Some(ka), Some(kb)) = (find(fa), find(fb)) else { return nm(line!()) };
    let (la, lb) = (&ring.links[ka], &ring.links[kb]);
    let (ea, eb) = (la.e, lb.e);
    let (eda, edb) = (b.edge(ea), b.edge(eb));
    let (hia, hib) = (eda.v1 == v, edb.v1 == v);
    let (ta0, tb0) = (if hia { eda.t1 } else { eda.t0 }, if hib { edb.t1 } else { edb.t0 });
    let (_, na, _, ga, _) = offset_terms(&eda.curve, ta0, dir, la.sig);
    let (_, nb, _, gb, _) = offset_terms(&edb.curve, tb0, dir, lb.sig);
    let (tga, tgb) = (ga.normalize() * if hia { -1.0 } else { 1.0 }, gb.normalize() * if hib { -1.0 } else { 1.0 });
    if (na - nb).len() < 1e-6 || tgb.dot(na) > -1e-9 || tga.dot(nb) > -1e-9 {
        return nm(line!());
    }
    let cos_th = na.dot(nb).clamp(-1.0, 1.0);
    let (ia, ib) = (if hia { -1.0 } else { 1.0 }, if hib { -1.0 } else { 1.0 });
    let sa = Sec { c: &eda.curve, dir, sig: la.sig, u_in: ring.u_in, r: size, chamfer: false };
    let sb = Sec { c: &edb.curve, dir, sig: lb.sig, u_in: ring.u_in, r: size, chamfer: false };
    let (ta, tb, c) = if chamfer {
        // the point at arc length `size` from the corner along each wall
        let walk = |ed: &ck_topo::body::Edge, t0: f64, hi: bool| -> R<f64> {
            let total = ed.curve.length_between(ed.t0, ed.t1);
            if size >= total * (1.0 - 1e-6) {
                return refuse(&format!("the chamfer {size} does not fit: a wall of this corner is only {total:.3} long"), "use a smaller size");
            }
            // bisect on the parameter: length from the corner grows away from it
            let (mut near, mut far) = (t0, if hi { ed.t0 } else { ed.t1 });
            for _ in 0..80 {
                let m = 0.5 * (near + far);
                let len = if hi { ed.curve.length_between(m, t0) } else { ed.curve.length_between(t0, m) };
                if len > size {
                    far = m;
                } else {
                    near = m;
                }
            }
            Ok(0.5 * (near + far))
        };
        (walk(eda, ta0, hia)?, walk(edb, tb0, hib)?, Vec3::ZERO)
    } else {
        let (ta, tb) = offset_meet(&sa, ta0, ia, &sb, tb0, ib, cos_th)?;
        (ta, tb, sa.top(ta).0)
    };
    for (ed, t, hi) in [(eda, ta, hia), (edb, tb, hib)] {
        let ok = if hi { t > ed.t0 + 1e-6 && t < ed.t1 } else { t < ed.t1 - 1e-6 && t > ed.t0 };
        if !ok {
            return refuse(&format!("the {} {size} does not fit: a wall of this corner is shorter than the corner needs", if chamfer { "chamfer" } else { "round" }), "use a smaller size");
        }
    }
    let pa = eda.curve.eval(ta);
    let pb = edb.curve.eval(tb);
    let Surface::Plane { frame } = b.face(ring.cap).surface.clone() else { return nm(line!()) };
    // the normals at the contact points (they differ from the corner's on a curved wall)
    let (na2, nb2) = if chamfer { (na, nb) } else { ((pa - c).normalize(), (pb - c).normalize()) };
    let cos2 = na2.dot(nb2).clamp(-1.0, 1.0);
    let _ = cos_th;
    Ok(EndGeom { v, cap: ring.cap, ea, eb, ta, tb, pa, pb, c, na: na2, nb: nb2, cos_th: cos2, cap_frame: frame })
}

/// Round (`chamfer` false, radius) or chamfer (equal distance) the vertical edges `list` = (edge, size, pick index).
/// Returns the arcs / chords made on the caps (they are rim edges of the new loops).
pub(super) fn round(b: &mut Body, list: &[(EdgeId, f64, u32)], chamfer: bool, feature: &str) -> R<Vec<EdgeId>> {
    let mut made: Vec<EdgeId> = vec![];
    for &(s, size, idx) in list {
        let sd = b.edge(s).clone();
        let Curve::Line { o: so, d: sdir } = sd.curve else { return nm(line!()) };
        let faces = b.edge_faces(s);
        if faces.len() != 2 {
            return nm(line!());
        }
        let (fa, fb) = (faces[0], faces[1]);
        let ends = [end_geom(b, sd.v0, s, fa, size, chamfer)?, end_geom(b, sd.v1, s, fa, size, chamfer)?];
        let shell = b.face(fa).shell;
        let mut va = vec![];
        let mut vb = vec![];
        let mut arcs = vec![];
        for g in &ends {
            let cap_loop = b.face(g.cap).loops.iter().copied().find(|&l| b.lp(l).coedges.iter().any(|&c| b.coedge(c).edge == g.ea)).ok_or(Miss::NotMine)?;
            let pos_of = |e: EdgeId| b.lp(cap_loop).coedges.iter().position(|&c| b.coedge(c).edge == e);
            let (pa_pos, pb_pos) = (pos_of(g.ea).ok_or(Miss::NotMine)?, pos_of(g.eb).ok_or(Miss::NotMine)?);
            let ca: CoedgeId = b.lp(cap_loop).coedges[pa_pos];
            let a_first = b.coedge_end(ca) == g.v;
            let (v_a, v_b) = (b.add_vertex(g.pa), b.add_vertex(g.pb));
            for (e, t, vn) in [(g.ea, g.ta, v_a), (g.eb, g.tb, v_b)] {
                let ed = &mut b.edges[e.idx()];
                if ed.v1 == g.v {
                    ed.v1 = vn;
                    ed.t1 = t;
                } else {
                    ed.v0 = vn;
                    ed.t0 = t;
                }
            }
            let zc = g.na.cross(g.nb).normalize();
            let phi = g.cos_th.acos();
            let arc = if chamfer {
                let dv = g.pb - g.pa;
                b.add_edge(Curve::Line { o: g.pa, d: dv.normalize() }, 0.0, dv.len(), v_a, v_b)
            } else {
                let fr = Frame { o: g.c, x: g.na, y: zc.cross(g.na), z: zc };
                b.add_edge(Curve::Circle { frame: fr, r: size }, 0.0, phi, v_a, v_b)
            };
            let curve = b.edge(arc).curve.clone();
            let pc = ck_topo::make::plane_pcurve(&g.cap_frame, &curve);
            let nc = b.add_coedge(cap_loop, arc, !a_first, pc);
            b.loops[cap_loop.idx()].coedges.pop();
            let at = if a_first { pa_pos + 1 } else { pb_pos + 1 };
            b.loops[cap_loop.idx()].coedges.insert(at, nc);
            va.push(v_a);
            vb.push(v_b);
            arcs.push(arc);
            made.push(arc);
        }
        // the seam edges on each wall
        let off_a = ends[0].pa - b.vertex(ends[0].v).p;
        let off_b = ends[0].pb - b.vertex(ends[0].v).p;
        let sa_e = b.add_edge(Curve::Line { o: so + off_a, d: sdir }, sd.t0, sd.t1, va[0], va[1]);
        let sb_e = b.add_edge(Curve::Line { o: so + off_b, d: sdir }, sd.t0, sd.t1, vb[0], vb[1]);
        for (face, ne, off, tpar) in [(fa, sa_e, off_a, ends[0].ta), (fb, sb_e, off_b, ends[0].tb)] {
            let c: CoedgeId = b.edge_coedges(s).into_iter().find(|&c| b.coedge_face(c) == face).ok_or(Miss::NotMine)?;
            b.edges[s.idx()].coedges.retain(|&x| x != c);
            b.coedges[c.idx()].edge = ne;
            b.edges[ne.idx()].coedges.push(c);
            let new_pc = match (&b.coedge(c).pcurve, &b.face(face).surface) {
                (Some(Curve2::Line { o, d }), Surface::Extrusion { .. }) => Some(Curve2::Line { o: Vec2::new(tpar, o.y), d: *d }),
                (Some(Curve2::Line { o, d }), Surface::Plane { frame }) => Some(Curve2::Line { o: *o + Vec2::new(off.dot(frame.x), off.dot(frame.y)), d: *d }),
                _ => return nm(line!()),
            };
            b.coedges[c.idx()].pcurve = new_pc;
        }
        // the new face
        let polygon = [ends[0].pa, ends[0].pb, ends[1].pb, ends[1].pa];
        let surface: Surface;
        let pcs: Vec<Option<Curve2>>;
        let mid_dir: Vec3;
        if chamfer {
            let chord = (ends[0].pb - ends[0].pa).normalize();
            let mut n = chord.cross(sdir).normalize();
            if n.dot(ends[0].na + ends[0].nb) < 0.0 {
                n = -n;
            }
            mid_dir = n;
            let fr = Frame { o: ends[0].pa, x: chord, y: n.cross(chord), z: n };
            surface = Surface::Plane { frame: fr };
            pcs = [arcs[0], sb_e, arcs[1], sa_e].iter().map(|&e| ck_topo::make::plane_pcurve(&fr, &b.edge(e).curve.clone())).collect();
        } else {
            let g = &ends[0];
            let zc = g.na.cross(g.nb).normalize();
            let phi = g.cos_th.acos();
            let fr = Frame { o: g.c, x: g.na, y: zc.cross(g.na), z: zc };
            surface = Surface::Cylinder { frame: fr, r: size };
            let sz = sdir.dot(zc);
            let hs = (b.edge(sa_e).curve.eval(sd.t0) - fr.o).dot(zc);
            let h_arc1 = (ends[1].c - g.c).dot(zc);
            mid_dir = (g.pa + g.pb) * 0.5 - g.c;
            pcs = vec![
                Some(Curve2::Line { o: Vec2::new(0.0, 0.0), d: Vec2::new(1.0, 0.0) }),
                Some(Curve2::Line { o: Vec2::new(phi, hs - sd.t0 * sz), d: Vec2::new(0.0, sz) }),
                Some(Curve2::Line { o: Vec2::new(0.0, h_arc1), d: Vec2::new(1.0, 0.0) }),
                Some(Curve2::Line { o: Vec2::new(0.0, hs - sd.t0 * sz), d: Vec2::new(0.0, sz) }),
            ];
        }
        let mut area = Vec3::ZERO;
        for i in 0..4 {
            area = area + polygon[i].cross(polygon[(i + 1) % 4]);
        }
        let ccw = area.dot(mid_dir) > 0.0;
        let fid = b.add_face(shell, surface, true, FaceName::blend(feature, idx));
        let lp = b.add_loop(fid);
        let seq: [(EdgeId, bool, usize); 4] = if ccw { [(arcs[0], false, 0), (sb_e, false, 1), (arcs[1], true, 2), (sa_e, true, 3)] } else { [(sa_e, false, 3), (arcs[1], false, 2), (sb_e, true, 1), (arcs[0], true, 0)] };
        for (e, rev, k) in seq {
            b.add_coedge(lp, e, rev, pcs[k].clone());
        }
        b.edges[s.idx()].alive = false;
        for g in &ends {
            b.vertices[g.v.idx()].alive = false;
        }
    }
    Ok(made)
}
