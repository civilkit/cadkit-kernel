//! A dome (cylinder + sphere cap ending at the axis pole) must mesh fully.
use ck_geom::{Frame, Vec2, Vec3};
use ck_tess::mesh::tessellate;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

#[test]
fn dome_cap_meshes_to_the_pole() {
    let v = |x: f64, y: f64| Vec2::new(x, y);
    let outer = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(0.0, 0.0), v(42.88, 0.0)),
        ProfileSeg::line("e2", v(42.88, 0.0), v(42.88, 38.86)),
        ProfileSeg::arc("e3", v(0.0, 38.86), v(42.88, 38.86), v(0.0, 81.74)),
        ProfileSeg::line("e4", v(0.0, 81.74), v(0.0, 0.0)),
    ]);
    let p = Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }]);
    let b = revolve("F2", &p, Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap();
    let t = tessellate(&b);
    let missing: Vec<_> = b.face_ids().filter(|f| t.faces[f.idx()].is_none()).map(|f| b.face_id_str(f)).collect();
    assert!(missing.is_empty(), "unmeshed faces {missing:?}");
}
