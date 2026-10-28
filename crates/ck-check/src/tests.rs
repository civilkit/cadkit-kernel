use super::*;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::FaceId;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

fn box_body() -> Body {
    let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y };
    let p = [Vec2::new(0.0, 0.0), Vec2::new(40.0, 0.0), Vec2::new(40.0, 20.0), Vec2::new(0.0, 20.0)];
    let tags = ["e1", "e2", "e3", "e4"];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap()
}

fn codes(b: &Body) -> Vec<&'static str> {
    let mut c: Vec<_> = validate(b, 1e-7).iter().map(|i| i.code).collect();
    c.sort();
    c.dedup();
    c
}

#[test]
fn good_box() {
    assert!(validate(&box_body(), 1e-7).is_empty());
}

#[test]
fn open_shell() {
    let mut b = box_body();
    ck_topo::edit::delete_face(&mut b, FaceId(0));
    assert!(codes(&b).contains(&"SHELL_OPEN"));
}

#[test]
fn flipped_face() {
    let mut b = box_body();
    ck_topo::orient::flip_face(&mut b, FaceId(2));
    assert!(codes(&b).contains(&"SHELL_ORIENTATION"));
}

/// Patch faces may live in their own single-face shell (fill / knit): the
/// Euler count is per edge-connected component, so a closed box split over
/// two shells is still valid.
#[test]
fn box_split_over_two_shells_is_valid() {
    let mut b = box_body();
    let lump = b.lump_ids().next().unwrap();
    let ns = b.add_shell(lump);
    let f = FaceId(0);
    let old = b.face(f).shell;
    b.shells[old.idx()].faces.retain(|&x| x != f);
    b.shells[ns.idx()].faces.push(f);
    b.faces[f.idx()].shell = ns;
    assert!(validate(&b, 1e-7).is_empty(), "{:?}", validate(&b, 1e-7));
}

#[test]
fn inside_out() {
    let mut b = box_body();
    let fs: Vec<_> = b.face_ids().collect();
    for f in fs {
        ck_topo::orient::flip_face(&mut b, f);
    }
    assert_eq!(codes(&b), vec!["VOLUME_NONPOSITIVE"]);
}

/// A shell whose faces cannot reach each other is still reported: the sweep
/// builder splits components into their own shells (a ring revolve gives an
/// outer and a void shell), but faces joined into one shell from elsewhere
/// (damage, import) keep their `SHELL_DISCONNECTED`.
#[test]
fn unreachable_faces_in_a_shell() {
    let mut b = box_body();
    let lump = b.lump_ids().next().unwrap();
    let ns = b.add_shell(lump);
    // the two caps: edge-disconnected from each other, connected only via the sides
    for f in [FaceId(0), FaceId(1)] {
        let old = b.face(f).shell;
        b.shells[old.idx()].faces.retain(|&x| x != f);
        b.shells[ns.idx()].faces.push(f);
        b.faces[f.idx()].shell = ns;
    }
    assert!(codes(&b).contains(&"SHELL_DISCONNECTED"), "{:?}", codes(&b));
}

fn xz_frame() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}

fn ring(r_out: f64, c_in: Vec2, r_in: f64) -> Body {
    let outer = ProfileLoop::new(vec![ProfileSeg::circle("o", Vec2::new(0.0, 0.0), r_out)]);
    let hole = ProfileLoop::new(vec![ProfileSeg::circle("h", c_in, r_in)]);
    let prof = Profile::new(xz_frame(), vec![ProfileRegion { outer, holes: vec![hole] }]);
    extrude("F2", &prof, ExtrudeOpts::blind(5.0)).unwrap()
}

#[test]
fn thin_annulus_is_valid() {
    // 0.09 wide ring on a radius-23 circle (BenchCAD nozzle)
    assert_eq!(codes(&ring(22.99, Vec2::new(0.0, 0.0), 22.9)), Vec::<&str>::new());
}

#[test]
fn concentric_circles_are_valid() {
    assert_eq!(codes(&ring(10.0, Vec2::new(0.0, 0.0), 4.0)), Vec::<&str>::new());
}

#[test]
fn near_tangent_offset_hole_is_valid() {
    assert_eq!(codes(&ring(10.0, Vec2::new(0.05, 0.0), 9.9)), Vec::<&str>::new());
}

#[test]
fn hole_crossing_outer_loop_is_flagged() {
    let b = ring(10.0, Vec2::new(8.0, 0.0), 5.0);
    assert!(validate(&b, 1e-7).iter().any(|i| i.code == "LOOP_CROSS"), "{:?}", codes(&b));
}

#[test]
fn bow_tie_polygon_is_flagged() {
    let p = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(10.0, 0.0), Vec2::new(0.0, 10.0)];
    let tags = ["e1", "e2", "e3", "e4"];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(xz_frame(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
    if let Ok(b) = extrude("F2", &prof, ExtrudeOpts::blind(5.0)) {
        assert!(validate(&b, 1e-7).iter().any(|i| i.code == "LOOP_SELF_INTERSECT"), "{:?}", codes(&b));
    }
}

/// Two disjoint solids in one body (a multi-solid result) are each judged on
/// their own: valid, and breaking one names its lump.
#[test]
fn two_disjoint_solids_are_valid_per_lump() {
    let mut a = box_body();
    let mut c = box_body();
    c.transform(&ck_core::xform::Xform::translation(Vec3::new(100.0, 0.0, 0.0)));
    a.append(&c);
    assert_eq!(a.num_lumps(), 2);
    assert!(validate(&a, 1e-7).is_empty(), "{:?}", validate(&a, 1e-7));
    // flip the second solid inside out: only lump#1 is reported
    let second: Vec<FaceId> = a.face_ids().filter(|&f| a.lump_index(a.face_lump(f)) == 1).collect();
    for f in second {
        ck_topo::orient::flip_face(&mut a, f);
    }
    let v = validate(&a, 1e-7);
    assert!(v.iter().any(|i| i.code == "VOLUME_NONPOSITIVE" && i.entity == "lump#1"), "{v:?}");
    assert!(!v.iter().any(|i| i.entity == "lump#0"), "{v:?}");
}

/// Two outer shells in one lump are two solids: flagged.
#[test]
fn two_outer_shells_in_one_lump_are_flagged() {
    let mut a = box_body();
    let mut c = box_body();
    c.transform(&ck_core::xform::Xform::translation(Vec3::new(100.0, 0.0, 0.0)));
    a.append(&c);
    // move the second lump's shell into the first lump
    let (l0, l1): (Vec<_>, Vec<_>) = a.lump_ids().partition(|&l| a.lump_index(l) == 0);
    let (l0, l1) = (l0[0], l1[0]);
    let shells = a.lumps[l1.idx()].shells.clone();
    for s in &shells {
        a.shells[s.idx()].lump = l0;
    }
    a.lumps[l0.idx()].shells.extend(shells);
    a.lumps[l1.idx()].shells.clear();
    a.lumps[l1.idx()].alive = false;
    assert!(validate(&a, 1e-7).iter().any(|i| i.code == "SOLID_MULTIPLE_OUTERS"), "{:?}", validate(&a, 1e-7));
}

thread_local! { static NOW: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) }; }
/// A clock that advances 10 ms per reading, so a budget runs out after a known number of reads.
fn ticking() -> f64 {
    NOW.with(|t| {
        let v = t.get() + 10.0;
        t.set(v);
        v
    })
}

#[test]
fn unlimited_run_is_the_plain_validate() {
    let b = box_body();
    let ctl = Ctl::unlimited();
    assert_eq!(validate_ctl(&b, 1e-7, DEFAULT_TOL_CAP, false, &ctl), validate(&b, 1e-7));
    assert!(!ctl.exceeded() && ctl.not_run().is_empty() && ctl.partial().is_empty());
    let names: Vec<_> = ctl.stats().iter().map(|s| s.0).collect();
    assert_eq!(names, ["shell", "loops", "edges", "solids"]);
}

#[test]
fn tiny_budget_stops_the_remaining_checks_and_names_them() {
    ck_core::budget::set_clock(ticking);
    NOW.with(|t| t.set(0.0));
    let b = box_body();
    let ctl = Ctl::new(Some(15.0)); // reads: start 10; shell stage's expired() 20 > 15 + start... see below
    let _ = validate_ctl(&b, 1e-7, DEFAULT_TOL_CAP, false, &ctl);
    assert!(ctl.exceeded());
    let not_run = ctl.not_run();
    assert!(not_run.contains(&"solids"), "{not_run:?}");
    assert!(ctl.stats().len() + not_run.len() >= 4);
}

#[test]
fn quick_leaves_out_crossings_and_solids() {
    let mut b = box_body();
    ck_topo::orient::flip_face(&mut b, FaceId(2));
    let ctl = Ctl::unlimited();
    let q = validate_ctl(&b, 1e-7, DEFAULT_TOL_CAP, true, &ctl);
    assert!(q.iter().any(|i| i.code == "SHELL_ORIENTATION"));
    assert_eq!(ctl.not_run(), ["loopCrossings", "solids"]);
}
