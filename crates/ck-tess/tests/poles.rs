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
