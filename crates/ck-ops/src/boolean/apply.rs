//! Apply (ck-ops/src/boolean/apply.rs): the boolean entry point plus the
//! pattern-instance helpers that feed it (instances are transformed copies
//! of a seed body, renamed `@<feature>.<k>`, unioned into the target).
//!
//! Pattern rules pinned by oracle probes (box seed `F2`, pattern `F3`):
//! - lpattern: instance k (1..count-1) = translation k·spacing·dir
//!   (`reverse` negates dir).
//! - cpattern: rotation about the axis by k·step (right-hand rule,
//!   `reverse` negates). `equal`: step = angle/count when angle = 360,
//!   else angle/(count-1); not `equal`: step = angle. count includes the
//!   seed. (count 3, 90°, equal → 45° steps; not equal → 90°; 360 equal →
//!   120°.)
//! - instance faces: `F2.S.e1@F3.1`, `@F3.2`; the result is rebuilt in
//!   face-walk edge order even for disjoint copies: `F2.S.e1|F2.S.e4` is
//!   #9 / #21 / #33 (12 edges per box, walk C0, C1, S.e1 → 4+4+2).

use super::assemble::{build, select};
use super::imprint::imprint;
use super::classify::{Class, Solid};
use super::Op;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_topo::body::Body;
use ck_topo::check::{check, Issue};

#[derive(Debug, Clone, PartialEq)]
pub struct BoolOut {
    pub body: Body,
    /// Cut / intersect that changed nothing (feature warning `cutNoEffect`).
    pub no_effect: bool,
}

/// `a op b`. Errors: `booleanFailed` (topology could not be assembled).
///
/// Fallback ladder: marched intersection curves meet their neighbours only
/// to ~1e-6, so a configuration whose edge endpoints sit within a few
/// model tolerances of each other (a sliver where a cylinder grazes a
/// torus) can leave the arrangement open at `tol`. On `booleanFailed` the
/// operation is retried with the welding tolerance raised 10x and 100x
/// (the vertex pool, edge de-duplication and trims all follow `tol`); the
/// first result that assembles into a closed manifold wins.
pub fn boolean(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    let first = boolean_at(a, b, op, tol);
    if !matches!(&first, Err(e) if e == "booleanFailed") {
        return first;
    }
    for k in [10.0, 100.0] {
        super::bool_dbg!("retrying with tolerance {:e}", tol * k);
        if let Ok(out) = boolean_at(a, b, op, tol * k) {
            return Ok(out);
        }
    }
    first
}

fn boolean_at(a: &Body, b: &Body, op: Op, tol: f64) -> Result<BoolOut, String> {
    if b.num_faces() == 0 {
        return Ok(BoolOut { body: a.clone(), no_effect: op != Op::Union });
    }
    if a.num_faces() == 0 {
        return Ok(match op {
            Op::Union => BoolOut { body: b.clone(), no_effect: false },
            _ => BoolOut { body: Body::new(), no_effect: true },
        });
    }
    // (results keep the caller's bodies for the no-effect returns)
    let (a0, b0) = (a, b);
    let a1 = relocate_seams(a0, b0, dropped(op, 0), tol);
    let b1 = relocate_seams(b0, a0, dropped(op, 1), tol);
    let (a, b) = (&a1, &b1);
    let arr = imprint(a, b, tol);
    if arr.interacting_pairs == 0 {
        // no face interaction: containment decides
        let pa = first_point(a);
        let pb = first_point(b);
        let a_in_b = ck_topo::region::classify(b, pa, tol) == ck_topo::region::Containment::Inside;
        let b_in_a = ck_topo::region::classify(a, pb, tol) == ck_topo::region::Containment::Inside;
        match op {
            Op::Cut if !a_in_b && !b_in_a => return Ok(BoolOut { body: a0.clone(), no_effect: true }),
            Op::Intersect if !a_in_b && !b_in_a => return Ok(BoolOut { body: Body::new(), no_effect: true }),
            _ => {} // nested: the general path (void shells / swallowed bodies)
        }
    }
    let faces = select(&arr, [a, b], op)?;
    // nothing removed from A and nothing added from B → unchanged
    let a_faces = a.num_faces();
    // (a face split by an imprint and rejoined is listed with the merged
    // faces but walked as untouched)
    let whole_a = |f: &super::assemble::OutFace| f.group == 0 || (f.group == 4 && f.walk.len() == 1 && f.walk[0].0 .0 == 0);
    let untouched_a = faces.iter().filter(|f| whole_a(f)).count();
    if op == Op::Cut && untouched_a == a_faces && faces.iter().all(|f| whole_a(f)) {
        return Ok(BoolOut { body: a0.clone(), no_effect: true });
    }
    let body = build(&arr, &faces);
    let issues = check(&body, tol);
    let bad_issues: Vec<_> = issues
        .into_iter()
        .filter(|i| matches!(i, Issue::OpenEdge(_) | Issue::NonManifoldEdge(..) | Issue::BrokenLoop(..) | Issue::BadPairing(_)))
        .collect();
    if !bad_issues.is_empty() {
        super::bool_dbg!("arrangement:\n{}", arr.dump());
        for i in &bad_issues {
            if let Issue::OpenEdge(e) | Issue::NonManifoldEdge(e, _) | Issue::BadPairing(e) = i {
                let ed = body.edge(*e);
                let faces: Vec<_> = body.edge_coedges(*e).iter().map(|&c| body.face_id_str(body.coedge_face(c))).collect();
                super::bool_dbg!("  bad edge {:?}: {} t[{:.5},{:.5}] p0 {:?} p1 {:?} faces {:?}", e, ed.curve.kind(), ed.t0, ed.t1, ed.curve.eval(ed.t0), ed.curve.eval(ed.t1), faces);
            }
        }
        super::bool_dbg!("assembled body invalid: {} issues, first {:?}", bad_issues.len(), &bad_issues[..bad_issues.len().min(6)]);
        return Err("booleanFailed".into());
    }
    Ok(BoolOut { body, no_effect: false })
}

/// The class of the other body's region whose faces `op` discards on side
/// `side` (see `assemble::select`'s keep rule).
fn dropped(op: Op, side: usize) -> Class {
    match (op, side) {
        (Op::Union, _) | (Op::Cut, 0) => Class::In,
        (Op::Intersect, _) | (Op::Cut, _) => Class::Out,
    }
}

/// Move the seam of every cylinder / cone face whose two boundary circles
/// both lie in the region the boolean discards (through holes: the tool's
/// caps) to the angle where the boolean's closed intersection circles
/// start (`imprint::closed_start`, 0.62·2π of the face frame). The
/// oracle has no seams, so such a hole is bounded by single closed circles;
/// with the seam there our intersection circles are not split at it and
/// export (and tessellate) as one edge. Faces whose circles survive keep
/// their seam (their circles start at c + r·x).
pub fn relocate_seams(b: &Body, other: &Body, drop: Class, tol: f64) -> Body {
    use ck_geom::curve::Curve;
    use ck_geom::pcurve::{exact_pcurve, pcurve_of};
    use ck_geom::surface::Surface;
    use std::f64::consts::TAU;
    let mut out = b.clone();
    let solid = Solid::new(other, tol);
    let (t0, _) = super::imprint::closed_start(TAU);
    let candidate = |f: ck_topo::body::FaceId| -> Option<(ck_geom::Frame, Vec<ck_topo::body::EdgeId>, Vec<ck_topo::body::EdgeId>)> {
        let face = b.face(f);
        let frame = match &face.surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => *frame,
            _ => return None,
        };
        if face.loops.len() != 1 {
            return None;
        }
        let cs = b.lp(face.loops[0]).coedges.clone();
        let mut seams = vec![];
        let mut circles = vec![];
        for &c in &cs {
            let e = b.coedge(c).edge;
            if b.is_seam(e) {
                if !seams.contains(&e) {
                    seams.push(e);
                }
            } else {
                circles.push(e);
            }
        }
        if seams.len() != 1 || circles.len() != 2 {
            return None;
        }
        let full = |e: ck_topo::body::EdgeId| {
            let ed = b.edge(e);
            matches!(ed.curve, Curve::Circle { .. }) && ed.v0 == ed.v1 && ((ed.t1 - ed.t0) - TAU).abs() < 1e-9
        };
        if !circles.iter().all(|&e| full(e)) {
            return None;
        }
        let in_drop = circles.iter().all(|&e| {
            let p = b.edge_mid(e);
            solid.classify(p, frame.z) == drop
        });
        if !in_drop {
            return None;
        }
        Some((frame, seams, circles))
    };
    let cands: Vec<_> = b.face_ids().filter_map(|f| candidate(f).map(|c| (f, c))).collect();
    let ids: Vec<ck_topo::body::FaceId> = cands.iter().map(|c| c.0).collect();
    // a face sharing a circle (so its seam end vertex) with a seamed
    // cylinder / cone that keeps its seam cannot move its circle starts
    let mut ok: Vec<bool> = vec![true; cands.len()];
    loop {
        let mut changed = false;
        for (i, (f, c)) in cands.iter().enumerate() {
            if !ok[i] {
                continue;
            }
            let blocked = c.2.iter().any(|&e| {
                b.edge_faces(e).into_iter().any(|g| {
                    g != *f && matches!(b.face(g).surface, Surface::Cylinder { .. } | Surface::Cone { .. }) && ids.iter().position(|&x| x == g).map_or(true, |k| !ok[k])
                })
            });
            if blocked {
                ok[i] = false;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    for (i, (f, (frame, seams, circles))) in cands.into_iter().enumerate() {
        if !ok[i] {
            continue;
        }
        let s = seams[0];
        let radial = frame.x * t0.cos() + frame.y * t0.sin();
        for &e in &circles {
            let ed = b.edge(e).clone();
            let Curve::Circle { frame: cf, r } = ed.curve else { continue };
            let target = cf.o + radial * r;
            let mut t = ed.curve.closest_param(target);
            // same start convention as imprint: the run [t, t + 2π]
            t = t0 + (t - t0 - TAU * ((t - t0) / TAU).round());
            out.edges[e.idx()].t0 = t;
            out.edges[e.idx()].t1 = t + TAU;
            out.vertices[ed.v0.idx()].p = ed.curve.eval(t);
        }
        let se = b.edge(s).clone();
        let (pa, pb) = (out.vertex(se.v0).p, out.vertex(se.v1).p);
        let len = pa.dist(pb);
        out.edges[s.idx()].curve = Curve::Line { o: pa, d: (pb - pa) / len };
        out.edges[s.idx()].t0 = 0.0;
        out.edges[s.idx()].t1 = len;
        // pcurves of the touched faces (cylinder loop kept uv-continuous)
        let mut faces: Vec<ck_topo::body::FaceId> = vec![f];
        for &e in &circles {
            for g in b.edge_faces(e) {
                if !faces.contains(&g) {
                    faces.push(g);
                }
            }
        }
        for g in faces {
            let surf = out.face(g).surface.clone();
            let (pu, pv) = surf.periods();
            for l in out.face(g).loops.clone() {
                let mut prev_end: Option<ck_geom::Vec2> = None;
                for c in out.lp(l).coedges.clone() {
                    let ce = out.coedge(c).clone();
                    let ed = out.edge(ce.edge).clone();
                    let pc = exact_pcurve(&surf, &ed.curve, tol * 10.0).or_else(|| pcurve_of(&surf, &ed.curve, ed.t0, ed.t1, tol).ok());
                    let pc = pc.map(|mut pc| {
                        let (ta, tb) = if ce.rev { (ed.t1, ed.t0) } else { (ed.t0, ed.t1) };
                        if let Some(prev) = prev_end {
                            let st = pc.eval(ta);
                            let du = pu.map_or(0.0, |p| ((prev.x - st.x) / p).round() * p);
                            let dv = pv.map_or(0.0, |p| ((prev.y - st.y) / p).round() * p);
                            if du != 0.0 || dv != 0.0 {
                                pc = super::assemble::shift2(&pc, ck_geom::Vec2::new(du, dv));
                            }
                        }
                        prev_end = Some(pc.eval(tb));
                        pc
                    });
                    out.coedges[c.idx()].pcurve = pc;
                }
            }
        }
    }
    out
}

fn first_point(b: &Body) -> Vec3 {
    b.vertex_ids().next().map(|v| b.vertex(v).p).unwrap_or(Vec3::ZERO)
}

// ------------------------------------------------------------ patterns

/// lpattern instance transforms (k = 1..count-1).
pub fn linear_instances(dir: Vec3, spacing: f64, count: usize, reverse: bool) -> Vec<Xform> {
    let d = dir.normalize() * if reverse { -1.0 } else { 1.0 };
    (1..count.max(1)).map(|k| Xform::translation(d * (spacing * k as f64))).collect()
}

/// cpattern step in degrees.
pub fn circular_step(angle_deg: f64, count: usize, equal: bool) -> f64 {
    if !equal {
        return angle_deg;
    }
    // GUESS: "full circle" test exact to 1e-9 degrees
    if (angle_deg.abs() - 360.0).abs() < 1e-9 {
        angle_deg / count.max(1) as f64
    } else {
        angle_deg / (count.max(2) - 1) as f64
    }
}

/// cpattern instance transforms about the axis through `o` along `axis`.
pub fn circular_instances(o: Vec3, axis: Vec3, angle_deg: f64, count: usize, equal: bool, reverse: bool) -> Vec<Xform> {
    let step = circular_step(angle_deg, count, equal) * if reverse { -1.0 } else { 1.0 };
    (1..count.max(1)).map(|k| Xform::rotation(o, axis, (step * k as f64).to_radians())).collect()
}

/// Transformed, renamed copy of `seed` (`@<feature>.<k>`, k from 1).
pub fn instance(seed: &Body, x: &Xform, feature: &str, k: usize) -> Body {
    let mut b = seed.clone();
    b.transform(x);
    b.push_instance(&format!("{}.{}", feature, k));
    b
}

/// Union every instance of `seed` into `target` (instances in order).
pub fn apply_instances(target: &Body, seed: &Body, xs: &[Xform], feature: &str, tol: f64) -> Result<Body, String> {
    let mut out = target.clone();
    for (i, x) in xs.iter().enumerate() {
        let inst = instance(seed, x, feature, i + 1);
        out = boolean(&out, &inst, Op::Union, tol)?.body;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, rect_tags};
    use ck_topo::mass::mass_props;

    /// (id, normal for planar faces)
    fn faces(b: &Body) -> Vec<(String, Option<[f64; 3]>)> {
        b.face_ids()
            .map(|f| {
                let n = matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. })
                    .then(|| b.planar_normal(f).to_array().map(|x| if x.abs() < 1e-12 { 0.0 } else { x }));
                (b.face_id_str(f), n)
            })
            .collect()
    }

    fn assert_faces(b: &Body, want: &[(&str, Option<[f64; 3]>)]) {
        let got = faces(b);
        let g: Vec<&str> = got.iter().map(|x| x.0.as_str()).collect();
        let w: Vec<&str> = want.iter().map(|x| x.0).collect();
        assert_eq!(g, w);
        for (x, y) in got.iter().zip(want) {
            assert_eq!(x.1, y.1, "{}", x.0);
        }
    }

    fn assert_mass(b: &Body, vol: f64, area: f64, com: [f64; 3]) {
        let m = mass_props(b);
        assert!((m.volume - vol).abs() < 1e-9 * vol, "volume {} vs {}", m.volume, vol);
        assert!((m.area - area).abs() < 1e-9 * area, "area {} vs {}", m.area, area);
        for i in 0..3 {
            assert!((m.com.get(i) - com[i]).abs() < 1e-9 * 40.0, "com {:?} vs {:?}", m.com, com);
        }
    }

    fn valid(b: &Body) {
        let issues = check(b, 1e-6);
        assert!(issues.is_empty(), "{issues:?}");
    }

    const NX: Option<[f64; 3]> = Some([1., 0., 0.]);
    const MX: Option<[f64; 3]> = Some([-1., 0., 0.]);
    const NY: Option<[f64; 3]> = Some([0., 1., 0.]);
    const MY: Option<[f64; 3]> = Some([0., -1., 0.]);
    const NZ: Option<[f64; 3]> = Some([0., 0., 1.]);
    const MZ: Option<[f64; 3]> = Some([0., 0., -1.]);

    fn the_box() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }

    #[test]
    fn oracle_box_minus_through_cylinder() {
        // probe box_cyl: extrude cut throughBoth of circle (20,10) r5
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.S.e1", NZ),
                ("F2.S.e2", NX),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.C0", MY),
                ("F2.C1", NY),
                ("F4.S.e1", None),
            ],
        );
        assert_mass(&r.body, 7214.601836602553, 2957.0796326794866, [20., 5., -10.]);
        // the hole faces inward
        let f = r.body.find_face("F4.S.e1").unwrap();
        assert!(r.body.outward_normal(f, 0.0, 0.0).dot(ck_geom::Vec3::X) < 0.0);
        // edge keys: oracle has the 12 box edges + two circles C0|F4, C1|F4
        let wire: Vec<String> = r
            .body
            .wire_chains()
            .iter()
            .map(|ch| {
                let fs = r.body.edge_faces(ch[0]);
                let mut ids: Vec<String> = fs.iter().map(|&f| r.body.face_id_str(f)).collect();
                ids.sort();
                ids.join("|")
            })
            .collect();
        assert_eq!(wire.len(), 14);
        assert!(wire.contains(&"F2.C0|F4.S.e1".to_string()) && wire.contains(&"F2.C1|F4.S.e1".to_string()));
        // oracle edge order (#0..#13): face-walk order of the output faces
        assert_eq!(
            wire,
            [
                "F2.C0|F2.S.e1",
                "F2.S.e1|F2.S.e2",
                "F2.C1|F2.S.e1",
                "F2.S.e1|F2.S.e4",
                "F2.C0|F2.S.e2",
                "F2.S.e2|F2.S.e3",
                "F2.C1|F2.S.e2",
                "F2.C0|F2.S.e3",
                "F2.S.e3|F2.S.e4",
                "F2.C1|F2.S.e3",
                "F2.C0|F2.S.e4",
                "F2.C1|F2.S.e4",
                "F2.C0|F4.S.e1",
                "F2.C1|F4.S.e1"
            ]
        );
    }

    #[test]
    fn oracle_blind_pocket() {
        // probe blindcut: circle (20,10) r5, cut flipped depth 4 from C0
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, 0.0, 4.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.C1", NY),
                ("F2.S.e1", NZ),
                ("F2.S.e2", NX),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.C0", MY),
                ("F4.C1", MY),
                ("F4.S.e1", None),
            ],
        );
        assert_mass(&r.body, 7685.840734641021, 2925.6637061435913, [20., 5.122625205051293, -10.]);
        // circle edges start at c + r·x (oracle pts[0] = [25, 0, -10])
        let c0 = r.body.find_face("F2.C0").unwrap();
        let circ = r.body.face_edges(c0).into_iter().find(|&e| r.body.edge(e).curve.kind() == "circle").unwrap();
        let p = r.body.vertex(r.body.edge(circ).v0).p;
        assert!((p - Vec3::new(25., 0., -10.)).len() < 1e-9, "{p:?}");
    }

    #[test]
    fn oracle_cut_no_effect() {
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -4.0, 0.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        assert!(r.no_effect);
        assert_eq!(r.body, the_box());
    }

    #[test]
    fn oracle_slot_split() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 15.0, -5.0, 25.0, 25.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Cut, 1e-6).unwrap();
        valid(&r.body);
        let mut got: Vec<String> = faces(&r.body).into_iter().map(|x| x.0).collect();
        got.sort();
        let mut want = vec![
            "F2.S.e2", "F2.S.e4", "F2.C0~1", "F2.C0~2", "F2.C1~1", "F2.C1~2", "F2.S.e1~1", "F2.S.e1~2", "F2.S.e3~2",
            "F2.S.e3~1", "F4.S.s2", "F4.S.s4",
        ];
        want.sort();
        assert_eq!(got, want);
        assert_eq!(r.body.num_lumps(), 2);
        assert_mass(&r.body, 6000.0, 2600.0, [20., 5., -10.]);
        // ~1 = the x < 15 piece
        let f = r.body.find_face("F2.C0~1").unwrap();
        let x: f64 = r.body.face_edges(f).iter().map(|&e| r.body.edge_mid(e).x).fold(f64::MIN, f64::max);
        assert!(x <= 15.0 + 1e-9);
        // lump 0 holds the first face (oracle: F2.S.e2, the x > 25 piece)
        let first = r.body.face_ids().next().unwrap();
        assert_eq!(r.body.face_id_str(first), "F2.S.e2");
        assert_eq!(r.body.lump_index(r.body.face_lump(first)), 0);
    }

    #[test]
    fn oracle_union_overlapping_box() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 5.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.C1", NY),
                ("F2.S.e1", NZ),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.S.e2", NX),
                ("F4.S.s2", NX),
                ("F4.C1", NY),
                ("F4.S.s1", NZ),
                ("F4.S.s3", MZ),
                ("F2.C0", MY),
            ],
        );
        assert_mass(&r.body, 8500.0, 3100.0, [21.470588235294116, 4.852941176470589, -10.]);
    }

    #[test]
    fn oracle_union_flush_box() {
        let tool = rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 10.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        valid(&r.body);
        assert_faces(
            &r.body,
            &[
                ("F2.S.e1", NZ),
                ("F2.S.e3", MZ),
                ("F2.S.e4", MX),
                ("F2.S.e2~2", NX),
                ("F2.S.e2~1", NX),
                ("F4.S.s2", NX),
                ("F4.S.s1", NZ),
                ("F4.S.s3", MZ),
                ("F2.C0", MY),
                ("F2.C1", NY),
            ],
        );
        assert_mass(&r.body, 9000.0, 3200.0, [22.777777777777775, 5., -10.]);
    }

    #[test]
    fn intersect_box_cylinder() {
        let tool = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let r = boolean(&the_box(), &tool, Op::Intersect, 1e-6).unwrap();
        valid(&r.body);
        let m = mass_props(&r.body);
        let v = std::f64::consts::PI * 25.0 * 10.0;
        assert!((m.volume - v).abs() < 1e-9 * v, "{}", m.volume);
    }

    #[test]
    fn disjoint_union_appends() {
        let tool = box_body("F4", 50.0, 0.0, 60.0, 10.0, 0.0, 5.0);
        let r = boolean(&the_box(), &tool, Op::Union, 1e-6).unwrap();
        assert_eq!(r.body.num_lumps(), 2);
        assert_eq!(r.body.num_faces(), 12);
    }

    #[test]
    fn pattern_transforms_match_oracle() {
        // probe: box (10,0)-(14,2) on top, cpattern about y, count 3
        let rot = circular_instances(Vec3::ZERO, Vec3::Y, 90.0, 3, true, false);
        let p = rot[0].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(7.0710678118654755, 1.5, -7.071067811865475)).len() < 1e-14);
        let p = rot[1].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(6.123233995736766e-16, 1.5, -10.)).len() < 1e-14);
        let rot = circular_instances(Vec3::ZERO, Vec3::Y, 360.0, 3, true, false);
        let p = rot[1].apply_point(Vec3::new(10., 1.5, 0.));
        assert!((p - Vec3::new(-5.000000000000004, 1.5, 8.660254037844386)).len() < 1e-14);
        assert_eq!(circular_step(90.0, 3, false), 90.0);
        let rev = circular_instances(Vec3::ZERO, Vec3::Y, 90.0, 3, true, true);
        assert!((rev[0].apply_point(Vec3::new(10., 1.5, 0.)) - Vec3::new(7.0710678118654755, 1.5, 7.071067811865475)).len() < 1e-14);
        let lin = linear_instances(Vec3::Z, 5.0, 3, true);
        assert_eq!(lin.len(), 2);
        assert_eq!(lin[1].apply_point(Vec3::new(10., 1.5, 0.)), Vec3::new(10., 1.5, -10.));
        // instances unioned: disjoint copies are appended, renamed
        let seed = box_body("F2", 10.0, 0.0, 14.0, 2.0, 0.0, 3.0);
        let lin = linear_instances(Vec3::Z, 5.0, 3, false);
        let out = apply_instances(&seed, &seed, &lin, "F3", 1e-6).unwrap();
        let ids: Vec<String> = out.face_ids().map(|f| out.face_id_str(f)).filter(|s| s.starts_with("F2.S.e1")).collect();
        assert_eq!(ids, ["F2.S.e1", "F2.S.e1@F3.1", "F2.S.e1@F3.2"]);
        // oracle edges #9 / #21 = F2.S.e1|F2.S.e4 of the seed / copy 1
        let wire = out.wire_edges();
        let mut f9: Vec<String> = out.edge_faces(wire[9]).iter().map(|&f| out.face_id_str(f)).collect();
        f9.sort();
        assert_eq!(f9, ["F2.S.e1", "F2.S.e4"]);
        let e = wire[21];
        let mut fs: Vec<String> = out.edge_faces(e).iter().map(|&f| out.face_id_str(f)).collect();
        fs.sort();
        assert_eq!(fs, ["F2.S.e1@F3.1", "F2.S.e4@F3.1"]);
        assert_eq!(out.edge_mid(e), Vec3::new(10., 1.5, 5.));
    }
}

#[cfg(test)]
mod oracle_edges {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, rect_tags};

    /// Wire edge `key` (sorted face pair + `#n`) and `mid`, oracle replies
    /// of the probes (tools/q.mjs regen of box F2 + feature F4).
    #[rustfmt::skip]
    const ORACLE: &[(&str, &[(&str, [f64; 3])])] = &[
        ("blindcut", &[
            ("F2.C1|F2.S.e1#0", [20.0, 10.0, 0.0]),
            ("F2.C1|F2.S.e2#1", [40.0, 10.0, -10.0]),
            ("F2.C1|F2.S.e3#2", [20.0, 10.0, -20.0]),
            ("F2.C1|F2.S.e4#3", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e1#4", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2#5", [40.0, 5.0, 0.0]),
            ("F2.S.e1|F2.S.e4#6", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e2#7", [40.0, 0.0, -10.0]),
            ("F2.S.e2|F2.S.e3#8", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e3#9", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#10", [0.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#11", [0.0, 0.0, -10.0]),
            ("F2.C0|F4.S.e1#12", [15.0, 0.0, -10.0]),
            ("F4.C1|F4.S.e1#13", [15.0, 4.0, -10.0]),
        ]),
        ("slot", &[
            ("F2.C0~2|F2.S.e2#0", [40.0, 0.0, -10.0]),
            ("F2.S.e2|F2.S.e3~2#1", [40.0, 5.0, -20.0]),
            ("F2.C1~2|F2.S.e2#2", [40.0, 10.0, -10.0]),
            ("F2.S.e1~2|F2.S.e2#3", [40.0, 5.0, 0.0]),
            ("F2.C0~1|F2.S.e4#4", [0.0, 0.0, -10.0]),
            ("F2.S.e1~1|F2.S.e4#5", [0.0, 5.0, 0.0]),
            ("F2.C1~1|F2.S.e4#6", [0.0, 10.0, -10.0]),
            ("F2.S.e3~1|F2.S.e4#7", [0.0, 5.0, -20.0]),
            ("F2.C0~1|F2.S.e3~1#8", [7.5, 0.0, -20.0]),
            ("F2.C0~1|F4.S.s4#9", [15.0, 0.0, -10.0]),
            ("F2.C0~1|F2.S.e1~1#10", [7.5, 0.0, 0.0]),
            ("F2.C0~2|F2.S.e3~2#11", [32.5, 0.0, -20.0]),
            ("F2.C0~2|F2.S.e1~2#12", [32.5, 0.0, 0.0]),
            ("F2.C0~2|F4.S.s2#13", [25.0, 0.0, -10.0]),
            ("F2.C1~1|F2.S.e1~1#14", [7.5, 10.0, 0.0]),
            ("F2.C1~1|F4.S.s4#15", [15.0, 10.0, -10.0]),
            ("F2.C1~1|F2.S.e3~1#16", [7.5, 10.0, -20.0]),
            ("F2.C1~2|F2.S.e1~2#17", [32.5, 10.0, 0.0]),
            ("F2.C1~2|F2.S.e3~2#18", [32.5, 10.0, -20.0]),
            ("F2.C1~2|F4.S.s2#19", [25.0, 10.0, -10.0]),
            ("F2.S.e1~1|F4.S.s4#20", [15.0, 5.0, 0.0]),
            ("F2.S.e1~2|F4.S.s2#21", [25.0, 5.0, 0.0]),
            ("F2.S.e3~2|F4.S.s2#22", [25.0, 5.0, -20.0]),
            ("F2.S.e3~1|F4.S.s4#23", [15.0, 5.0, -20.0]),
        ]),
        ("union", &[
            ("F2.C1|F2.S.e1#0", [20.0, 10.0, 0.0]),
            ("F2.C1|F2.S.e2#1", [40.0, 10.0, -10.0]),
            ("F2.C1|F2.S.e3#2", [20.0, 10.0, -20.0]),
            ("F2.C1|F2.S.e4#3", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e1#4", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2#5", [40.0, 5.0, 0.0]),
            ("F2.S.e1|F2.S.e4#6", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e3#7", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#8", [0.0, 5.0, -20.0]),
            ("F2.S.e2|F2.S.e3#9", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#10", [0.0, 0.0, -10.0]),
            ("F2.C0|F2.S.e2#11", [40.0, 0.0, -17.5]),
            ("F2.C0|F2.S.e2#12", [40.0, 0.0, -2.5]),
            ("F2.S.e2|F4.S.s1#13", [40.0, 2.5, -5.0]),
            ("F2.S.e2|F4.C1#14", [40.0, 5.0, -10.0]),
            ("F2.S.e2|F4.S.s3#15", [40.0, 2.5, -15.0]),
            ("F2.C0|F4.S.s2#16", [50.0, 0.0, -10.0]),
            ("F4.S.s2|F4.S.s3#17", [50.0, 2.5, -15.0]),
            ("F4.C1|F4.S.s2#18", [50.0, 5.0, -10.0]),
            ("F4.S.s1|F4.S.s2#19", [50.0, 2.5, -5.0]),
            ("F2.C0|F4.S.s3#20", [45.0, 0.0, -15.0]),
            ("F2.C0|F4.S.s1#21", [45.0, 0.0, -5.0]),
            ("F4.C1|F4.S.s1#22", [45.0, 5.0, -5.0]),
            ("F4.C1|F4.S.s3#23", [45.0, 5.0, -15.0]),
        ]),
        ("unionflush", &[
            ("F2.C0|F2.S.e1#0", [20.0, 0.0, 0.0]),
            ("F2.S.e1|F2.S.e2~2#1", [40.0, 5.0, 0.0]),
            ("F2.C1|F2.S.e1#2", [20.0, 10.0, 0.0]),
            ("F2.S.e1|F2.S.e4#3", [0.0, 5.0, 0.0]),
            ("F2.C0|F2.S.e3#4", [20.0, 0.0, -20.0]),
            ("F2.S.e3|F2.S.e4#5", [0.0, 5.0, -20.0]),
            ("F2.C1|F2.S.e3#6", [20.0, 10.0, -20.0]),
            ("F2.S.e2~1|F2.S.e3#7", [40.0, 5.0, -20.0]),
            ("F2.C0|F2.S.e4#8", [0.0, 0.0, -10.0]),
            ("F2.C1|F2.S.e4#9", [0.0, 10.0, -10.0]),
            ("F2.C0|F2.S.e2~1#10", [40.0, 0.0, -17.5]),
            ("F2.C0|F2.S.e2~2#11", [40.0, 0.0, -2.5]),
            ("F2.C1|F2.S.e2~2#12", [40.0, 10.0, -2.5]),
            ("F2.C1|F2.S.e2~1#13", [40.0, 10.0, -17.5]),
            ("F2.S.e2~2|F4.S.s1#14", [40.0, 5.0, -5.0]),
            ("F2.S.e2~1|F4.S.s3#15", [40.0, 5.0, -15.0]),
            ("F2.C0|F4.S.s2#16", [50.0, 0.0, -10.0]),
            ("F4.S.s2|F4.S.s3#17", [50.0, 5.0, -15.0]),
            ("F2.C1|F4.S.s2#18", [50.0, 10.0, -10.0]),
            ("F4.S.s1|F4.S.s2#19", [50.0, 5.0, -5.0]),
            ("F2.C0|F4.S.s3#20", [45.0, 0.0, -15.0]),
            ("F2.C0|F4.S.s1#21", [45.0, 0.0, -5.0]),
            ("F2.C1|F4.S.s1#22", [45.0, 10.0, -5.0]),
            ("F2.C1|F4.S.s3#23", [45.0, 10.0, -15.0]),
        ]),
    ];

    #[test]
    fn edge_keys_and_mids_match_oracle() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let tool = |n: &str| match n {
            "blindcut" => (cyl_body("F4", 20.0, 10.0, 5.0, 0.0, 4.0), Op::Cut),
            "slot" => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 15.0, -5.0, 25.0, 25.0, -1.0, 11.0), Op::Cut),
            "union" => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 5.0), Op::Union),
            _ => (rect_tags("F4", ["s1", "s2", "s3", "s4"], 30.0, 5.0, 50.0, 15.0, 0.0, 10.0), Op::Union),
        };
        for (name, want) in ORACLE {
            let (t, op) = tool(name);
            let b = boolean(&a, &t, op, 1e-6).unwrap().body;
            let wire = b.wire_edges();
            assert_eq!(wire.len(), want.len(), "{name}");
            for (i, (&e, (key, mid))) in wire.iter().zip(want.iter()).enumerate() {
                let fs = b.edge_faces(e);
                let k = ck_topo::body::edge_key(&b.face_id_str(fs[0]), &b.face_id_str(fs[1]), i);
                assert_eq!(&k, key, "{name}");
                assert!((b.edge_mid(e) - Vec3::new(mid[0], mid[1], mid[2])).len() < 1e-9, "{name} {key}");
            }
        }
    }
}

#[cfg(test)]
mod oracle_cylinders {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body, cyl_on, top};
    use ck_geom::Frame;
    use ck_topo::mass::mass_props;

    /// (probe, oracle face ids, oracle edge keys)
    #[rustfmt::skip]
    const ORACLE: &[(&str, &[&str], &[&str])] = &[
        ("edgehole", &["F2.S.e1", "F2.S.e3", "F2.S.e4", "F2.C0", "F2.C1", "F2.S.e2~2", "F2.S.e2~1", "F4.S.e1"],
            &["F2.C0|F2.S.e1#0", "F2.S.e1|F2.S.e2~2#1", "F2.C1|F2.S.e1#2", "F2.S.e1|F2.S.e4#3", "F2.C0|F2.S.e3#4", "F2.S.e3|F2.S.e4#5", "F2.C1|F2.S.e3#6", "F2.S.e2~1|F2.S.e3#7", "F2.C0|F2.S.e4#8", "F2.C1|F2.S.e4#9", "F2.C0|F2.S.e2~1#10", "F2.C0|F4.S.e1#11", "F2.C0|F2.S.e2~2#12", "F2.C1|F2.S.e2~2#13", "F2.C1|F4.S.e1#14", "F2.C1|F2.S.e2~1#15", "F2.S.e2~2|F4.S.e1#16", "F2.S.e2~1|F4.S.e1#17"]),
        ("boss", &["F2.S.e1", "F2.S.e3", "F2.S.e4", "F2.C1", "F2.S.e2~2", "F2.S.e2~1", "F4.C1", "F4.S.e1", "F2.C0"],
            &["F2.C0|F2.S.e1#0", "F2.S.e1|F2.S.e2~2#1", "F2.C1|F2.S.e1#2", "F2.S.e1|F2.S.e4#3", "F2.C0|F2.S.e3#4", "F2.S.e3|F2.S.e4#5", "F2.C1|F2.S.e3#6", "F2.S.e2~1|F2.S.e3#7", "F2.C0|F2.S.e4#8", "F2.C1|F2.S.e4#9", "F2.C0|F2.S.e2~1#10", "F2.C0|F2.S.e2~2#11", "F2.C1|F2.S.e2~2#12", "F2.C1|F4.S.e1#13", "F2.C1|F2.S.e2~1#14", "F2.S.e2~2|F4.S.e1#15", "F2.S.e2~1|F4.S.e1#16", "F4.C1|F4.S.e1#17", "F2.C0|F4.S.e1#18"]),
        ("fronthole", &["F2.C0", "F2.C1", "F2.S.e2", "F2.S.e4", "F2.S.e1", "F2.S.e3", "F4.S.e1"],
            &["F2.C0|F2.S.e4#0", "F2.C0|F2.S.e3#1", "F2.C0|F2.S.e2#2", "F2.C0|F2.S.e1#3", "F2.C1|F2.S.e1#4", "F2.C1|F2.S.e2#5", "F2.C1|F2.S.e3#6", "F2.C1|F2.S.e4#7", "F2.S.e2|F2.S.e3#8", "F2.S.e1|F2.S.e2#9", "F2.S.e1|F2.S.e4#10", "F2.S.e3|F2.S.e4#11", "F2.S.e1|F4.S.e1#12", "F2.S.e3|F4.S.e1#13"]),
        ("cylcross", &["F2.C0", "F2.C1", "F2.S.e1", "F4.S.e1"],
            &["F2.C0|F2.S.e1#0", "F2.C1|F2.S.e1#1", "F2.S.e1|F4.S.e1#2", "F2.S.e1|F4.S.e1#3"]),
    ];

    fn case(name: &str) -> (Body, Body, Op, f64, f64, [f64; 3]) {
        let bx = || box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        match name {
            // cut throughBoth of circle (40,10) r5 on the box's x=40 edge
            "edgehole" => (bx(), cyl_body("F4", 40.0, 10.0, 5.0, -1.0, 11.0), Op::Cut, 7607.300918301279, 2778.539816339745, [19.07711705162717, 5., -10.]),
            // boss circle (38,10) r5, blind 15, overhanging the x=40 face
            "boss" => (bx(), cyl_body("F4", 38.0, 10.0, 5.0, 0.0, 15.0), Op::Union, 8590.86743798146, 3020.989738109662, [21.312693341198823, 5.342834193869538, -10.]),
            // front-plane circle (20,5) r3 cut throughBoth (along z)
            "fronthole" => (bx(), cyl_on("F4", Frame::identity(), 20.0, 5.0, 3.0, -25.0, 5.0), Op::Cut, 7434.513322353839, 3120.4424506661585, [20., 5., -10.]),
            // cylinder r10 h30 crossed by a front-plane hole r4 at y=15
            _ => (cyl_on("F2", top(), 0.0, 0.0, 10.0, 0.0, 30.0), cyl_on("F4", Frame::identity(), 0.0, 15.0, 4.0, -15.0, 15.0), Op::Cut, 8439.998272825493, 2892.5020588265716, [0., 15., 0.]),
        }
    }

    #[test]
    fn faces_edges_mass_match_oracle() {
        for (name, faces, keys) in ORACLE {
            let (a, b, op, vol, area, com) = case(name);
            let r = boolean(&a, &b, op, 1e-6).unwrap_or_else(|e| panic!("{name}: {e}"));
            let issues = check(&r.body, 1e-6);
            assert!(issues.is_empty(), "{name}: {issues:?}");
            let got: Vec<String> = r.body.face_ids().map(|f| r.body.face_id_str(f)).collect();
            assert_eq!(got, *faces, "{name}");
            // oracle edges = chains of our wire edges through seam points
            let wire: Vec<_> = crate::boolean::assemble::wire_chains(&r.body).into_iter().map(|c| c[0]).collect();
            let ks: Vec<String> = wire
                .iter()
                .enumerate()
                .map(|(i, &e)| {
                    let fs = r.body.edge_faces(e);
                    ck_topo::body::edge_key(&r.body.face_id_str(fs[0]), &r.body.face_id_str(fs[1]), i)
                })
                .collect();
            let mut ks_sorted: Vec<String> = ks.iter().map(|k| k.split('#').next().unwrap().to_string()).collect();
            let mut want_sorted: Vec<String> = keys.iter().map(|k| k.split('#').next().unwrap().to_string()).collect();
            ks_sorted.sort();
            want_sorted.sort();
            assert_eq!(ks_sorted, want_sorted, "{name} edge sets");
            let m = mass_props(&r.body);
            // marched SSI (cylcross) is a fitted curve: looser tolerance
            let rel = if *name == "cylcross" { 1e-6 } else { 1e-9 };
            assert!((m.volume - vol).abs() <= rel * vol, "{name} volume {} vs {vol}", m.volume);
            assert!((m.area - area).abs() <= rel * area, "{name} area {} vs {area}", m.area);
            assert!((m.com - Vec3::new(com[0], com[1], com[2])).len() <= rel * 40.0, "{name} com {:?}", m.com);
            if *name != "cylcross" {
                assert_eq!(ks, *keys, "{name} edge order");
            }
        }
    }
}

#[cfg(test)]
mod flange_tests {
    use super::*;
    use crate::boolean::tests::top;
    use ck_geom::{Frame, Vec2, Vec3};
    use ck_topo::mass::mass_props;
    use ck_topo::make::{extrude, revolve::revolve, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    /// Hex prism (corner radius 7.5055, across flats 13) + revolved chamfer
    /// triangle cut (flangeBolt F4+F6 / flangeNut F4 shape).
    #[test]
    fn hex_prism_minus_revolved_chamfer_cone() {
        let front = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
        let r = 7.505553499465135;
        let p: Vec<Vec2> = (0..6).map(|i| { let a = std::f64::consts::FRAC_PI_3 * i as f64 + std::f64::consts::FRAC_PI_2; Vec2::new(r * a.cos(), r * a.sin()) }).collect();
        let lp = ProfileLoop::new((0..6).map(|i| ProfileSeg::line(&format!("e{}", i + 7), p[i], p[(i + 1) % 6])).collect());
        let prof = Profile::new(front, vec![ProfileRegion { outer: lp, holes: vec![] }]);
        let hex = extrude("F4", &prof, ExtrudeOpts { s0: 0.0, s1: -5.3, dir: None, draft: 0.0 }).unwrap();
        let q = [Vec2::new(6.11, 5.3), Vec2::new(8.5, 3.920132856636794), Vec2::new(8.5, 5.3)];
        let tl = ProfileLoop::new((0..3).map(|i| ProfileSeg::line(&format!("e{}", i + 4), q[i], q[(i + 1) % 3])).collect());
        let tp = Profile::new(top(), vec![ProfileRegion { outer: tl, holes: vec![] }]);
        let tool = revolve("F6", &tp, Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, std::f64::consts::TAU).unwrap();
        let r = boolean(&hex, &tool, Op::Cut, 1e-6);
        let r = r.unwrap();
        assert_eq!(r.body.num_faces(), 9);
        let (vh, vt, vr) = (mass_props(&hex).volume, mass_props(&tool).volume, mass_props(&r.body).volume);
        assert!(vr < vh && vr > vh - vt, "{vr} {vh} {vt}");
    }
}
