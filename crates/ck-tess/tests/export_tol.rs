//! `export_tol`: the tolerance of an export mesh from a quality preset and/or an absolute chord height.
use ck_geom::{Frame, Vec2, Vec3};
use ck_tess::curvature::Tol;
use ck_tess::mesh::{body_tol, export_tol};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

fn box_body() -> ck_topo::body::Body {
    let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y };
    let p = [Vec2::new(0.0, 0.0), Vec2::new(40.0, 0.0), Vec2::new(40.0, 20.0), Vec2::new(0.0, 20.0)];
    let tags = ["e1", "e2", "e3", "e4"];
    let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
    extrude("F2", &Profile::new(fr, vec![ProfileRegion { outer, holes: vec![] }]), ExtrudeOpts::blind(10.0)).unwrap()
}

#[test]
fn export_tolerance() {
    let b = box_body();
    let base = body_tol(&b);
    // no options: the viewer's tolerance
    assert_eq!(export_tol(&b, None, None).unwrap(), (base, false));
    assert_eq!(export_tol(&b, Some("normal"), None).unwrap(), (base, false));
    // the presets scale the chord and tighten the angle for fine
    let (coarse, _) = export_tol(&b, Some("coarse"), None).unwrap();
    let (fine, _) = export_tol(&b, Some("fine"), None).unwrap();
    assert!((coarse.chord - 10.0 * base.chord).abs() < 1e-12 && coarse.angle > base.angle);
    assert!((fine.chord - 0.25 * base.chord).abs() < 1e-12 && fine.angle < base.angle);
    // an absolute chord replaces the preset's chord and keeps its angle
    let (t, raised) = export_tol(&b, Some("coarse"), Some(0.25)).unwrap();
    assert_eq!((t.chord, t.angle, raised), (0.25, coarse.angle, false));
    assert_eq!(export_tol(&b, None, Some(0.25)).unwrap(), (Tol { chord: 0.25, angle: base.angle }, false));
    // a chord far below the size of the part is raised, and says so
    let (t, raised) = export_tol(&b, None, Some(1e-30)).unwrap();
    assert!(raised && t.chord > 1e-30);
    // refusals
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(export_tol(&b, None, Some(bad)).unwrap_err().contains("chord_mm"), "{bad}");
    }
    let e = export_tol(&b, Some("ultra"), None).unwrap_err();
    assert!(e.contains("coarse, normal, fine") && e.contains("ultra"), "{e}");
}
