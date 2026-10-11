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

#[test]
fn inside_out() {
    let mut b = box_body();
    let fs: Vec<_> = b.face_ids().collect();
    for f in fs {
        ck_topo::orient::flip_face(&mut b, f);
    }
    assert_eq!(codes(&b), vec!["VOLUME_NONPOSITIVE"]);
}
