//! Faces of revolved profiles that reach the axis at a pole (a teardrop: sphere arc + cone to an apex) must mesh watertight.
use ck_geom::{Frame, Vec2, Vec3};
use ck_tess::mesh::tessellate;
use ck_topo::body::Body;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

/// BOSL2 onion: sphere arc from the south pole to the tangent point, a cone to the apex on the axis, the axis back down.
fn onion(r: f64, ang_deg: f64, cap: f64, fr: Frame, axis: Vec3, rot: usize, rev: bool) -> Body {
    let a = ang_deg.to_radians();
    let tip_y = r / a.sin();
    let tr = v(r * a.cos(), r * a.sin());
    let (top, dx) = if cap > 0.0 && cap < tip_y { (cap, (tip_y - cap) * a.tan()) } else { (tip_y, 0.0) };
    let mut segs = vec![ProfileSeg::arc("e1", v(0.0, 0.0), v(0.0, -r), tr), ProfileSeg::line("e2", tr, v(dx, top))];
    if dx > 0.0 {
        segs.push(ProfileSeg::line("e3", v(dx, top), v(0.0, top)));
        segs.push(ProfileSeg::line("ax", v(0.0, top), v(0.0, -r)));
    } else {
        segs.push(ProfileSeg::line("ax", v(dx, top), v(0.0, -r)));
    }
    if rev {
        segs = segs.iter().rev().map(|s| s.reversed()).collect();
    }
    let n = segs.len();
    segs.rotate_left(rot % n);
    let p = Profile::new(fr, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]);
    revolve("F2", &p, Vec3::ZERO, axis, 0.0, TAU).unwrap()
}

/// Unmeshed faces, and faces owning mesh edges that only one triangle uses (positions welded at 1e-4).
fn open_faces(b: &Body) -> Vec<String> {
    let t = tessellate(b);
    let mut out: Vec<String> = b.face_ids().filter(|f| t.faces[f.idx()].is_none()).map(|f| format!("{} unmeshed", b.face_id_str(f))).collect();
    let m = &t.mesh;
    let key = |k: usize| [m.positions[k], m.positions[k + 1], m.positions[k + 2]].map(|x| (x as f64 * 1e4).round() as i64);
    let mut used: std::collections::HashMap<([i64; 3], [i64; 3]), (u32, i32)> = Default::default();
    for (k, &fi) in m.tri_face.iter().enumerate() {
        let c: Vec<[i64; 3]> = (0..3).map(|j| key(9 * k + 3 * j)).collect();
        for j in 0..3 {
            let (a, bb) = (c[j], c[(j + 1) % 3]);
            if a == bb {
                continue;
            }
            let e = if a < bb { (a, bb) } else { (bb, a) };
            used.entry(e).or_insert((0, fi)).0 += 1;
        }
    }
    let mut per_face: std::collections::BTreeMap<i32, usize> = Default::default();
    for (_, (n, fi)) in used {
        if n == 1 {
            *per_face.entry(fi).or_default() += 1;
        }
    }
    let ids: Vec<_> = b.face_ids().collect();
    for (fi, n) in per_face {
        out.push(format!("{}: {} open mesh edges", b.face_id_str(ids[fi as usize]), n));
    }
    out
}

fn frames() -> Vec<(&'static str, Frame, Vec3)> {
    let f = |x: Vec3, y: Vec3| Frame { o: Vec3::ZERO, x, y, z: x.cross(y) };
    vec![
        ("xy", Frame::identity(), Vec3::Y),
        ("front", f(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)), Vec3::new(0.0, 0.0, 1.0)),
        ("top", f(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)), Vec3::Y),
        ("right", f(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)), Vec3::new(0.0, 0.0, 1.0)),
    ]
}

/// A partial revolve of a profile that reaches the axis at a point: a cone wedge (triangle profile, apex on the axis) and a
/// sphere wedge (half disc). The loop of the curved face crosses the apex / pole from one meridian to the other; for a
/// wedge wider than half a turn the far meridian was unwrapped one period over and the face meshed its complement (the
/// crash fuzz's `case_1714` and `case_4935`: 77 % of a 237-degree cone, open mesh edges on the base). Every wedge meshes
/// closed, and the curved face's mesh area is within 2 % of its exact area.
#[test]
fn wedges_through_a_pole_mesh_closed() {
    let tri = || ProfileLoop::new(vec![ProfileSeg::line("e1", v(10.0, 0.0), v(0.0, 10.0)), ProfileSeg::line("e2", v(0.0, 10.0), v(0.0, 0.0)), ProfileSeg::line("e3", v(0.0, 0.0), v(10.0, 0.0))]);
    let half = || ProfileLoop::new(vec![ProfileSeg::arc("e1", v(0.0, 0.0), v(0.0, -10.0), v(0.0, 10.0)), ProfileSeg::line("e2", v(0.0, 10.0), v(0.0, -10.0))]);
    let mut all = vec![];
    for (name, lp) in [("cone", tri()), ("sphere", half())] {
        // (a cone wedge of 355 degrees or more whose loop starts at the apex still meshes nothing: a separate, older
        // failure of the strip window, not of the pole crossing; see docs/crash-fuzz.md)
        for deg in [90.0, 179.0, 180.0, 181.0, 200.0, 237.04, 270.0, 300.0, 350.0] {
            for rev in [false, true] {
                let lp = if rev { ProfileLoop::new(lp.segs.iter().rev().map(|s| s.reversed()).collect()) } else { lp.clone() };
                let p = Profile::new(Frame::identity(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
                let b = revolve("F2", &p, Vec3::ZERO, Vec3::Y, 0.0, f64::to_radians(deg)).unwrap();
                let t = tessellate(&b);
                let m = &t.mesh;
                let mut issues: Vec<String> = ck_check::tessellation::check_mesh_why(&b, &m.positions, &m.tri_face, &m.failed, &m.failed_why)
                    .into_iter()
                    .map(|i| format!("{} {}: {}", i.code, i.entity, i.detail))
                    .collect();
                for f in b.face_ids() {
                    let Some(fm) = &t.faces[f.idx()] else { continue };
                    let mesh: f64 = fm.tris.iter().map(|q| 0.5 * (fm.pts[q[1] as usize] - fm.pts[q[0] as usize]).cross(fm.pts[q[2] as usize] - fm.pts[q[0] as usize]).len()).sum();
                    let exact = ck_topo::mass::face_integral::<1>(&b, f, 1, &|_, n| [n.len()])[0].abs();
                    if (mesh - exact).abs() > 0.02 * exact {
                        issues.push(format!("{}: mesh area {mesh:.3} of {exact:.3}", b.face_id_str(f)));
                    }
                }
                all.push(format!("{name} {deg} rev{rev}: {issues:?}"));
            }
        }
    }
    assert!(all.iter().all(|l| l.ends_with("[]")), "\n{}", all.join("\n"));
}

#[test]
fn onion_variants() {
    let mut all = vec![];
    for (name, fr, ax) in frames() {
        for sgn in [1.0, -1.0] {
            for (r, a, cap) in [(15.0, 45.0, 0.0), (15.0, 30.0, 20.0), (8.0, 60.0, 0.0)] {
                for rev in [false, true] {
                    for rot in 0..4 {
                        let issues = open_faces(&onion(r, a, cap, fr, ax * sgn, rot, rev));
                        all.push(format!("{name} axis {sgn} r{r} ang{a} cap{cap} rev{rev} rot{rot}: {issues:?}"));
                    }
                }
            }
        }
    }
    assert!(all.iter().all(|l| l.ends_with("[]")), "\n{}", all.join("\n"));
}
