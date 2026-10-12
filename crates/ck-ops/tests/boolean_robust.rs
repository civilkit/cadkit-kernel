//! Boolean robustness regressions minimised from the BenchCAD parts
//! (dome_cap, i_beam, bellows, eyebolt). Each case has a closed-form volume.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::mass::mass_props;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::{PI, TAU};

fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn region(outer: ProfileLoop) -> Vec<ProfileRegion> {
    vec![ProfileRegion { outer, holes: vec![] }]
}
/// Circle `e1` on `frame` extruded from s0 to s1.
fn cyl(feature: &str, frame: Frame, cx: f64, cy: f64, r: f64, s0: f64, s1: f64) -> Body {
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(cx, cy), r)]);
    extrude(feature, &Profile::new(frame, region(lp)), ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
}
fn front() -> Frame {
    Frame::identity()
}
fn assert_valid(b: &Body) {
    let issues = check(b, 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
}

/// Cylinder r with a hemispherical cap (sphere centre on the axis).
fn dome(r: f64, h: f64) -> Body {
    dome_off(r, h, 0.0, 0.0)
}

/// As `dome`, the arc centre displaced by (dx, dy) (a sketch solved to ~1e-4).
fn dome_off(r: f64, h: f64, dx: f64, dy: f64) -> Body {
    let outer = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(0.0, 0.0), v(r, 0.0)),
        ProfileSeg::line("e2", v(r, 0.0), v(r, h)),
        ProfileSeg::arc("e3", v(dx, h + dy), v(r, h), v(0.0, h + r)),
        ProfileSeg::line("e4", v(0.0, h + r), v(0.0, 0.0)),
    ]);
    revolve("F2", &Profile::new(top(), region(outer)), Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, TAU).unwrap()
}

/// Volume of the part of the dome inside a cylinder (r hr, axis parallel to the dome axis at (hx, hy)),
/// by midpoint quadrature over the hole disc (height at radius s is h + sqrt(R^2 - s^2)).
fn dome_column_volume(r: f64, h: f64, hr: f64, hx: f64, hy: f64) -> f64 {
    let n = 400;
    let mut vol = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (hx + hr * (-1.0 + 2.0 * (i as f64 + 0.5) / n as f64), hy + hr * (-1.0 + 2.0 * (j as f64 + 0.5) / n as f64));
            if (x - hx).powi(2) + (y - hy).powi(2) <= hr * hr {
                vol += (2.0 * hr / n as f64).powi(2) * (h + (r * r - x * x - y * y).max(0.0).sqrt());
            }
        }
    }
    vol
}

fn dome_hole(hx: f64, hy: f64) {
    let (r, h, hr) = (42.88, 38.86, 2.12);
    let d = dome(r, h);
    let v0 = mass_props(&d).volume;
    assert!((v0 - (PI * r * r * h + 2.0 / 3.0 * PI * r * r * r)).abs() < 1e-6 * v0);
    let tool = cyl("F4", front(), hx, hy, hr, -200.0, 200.0);
    let out = boolean(&d, &tool, Op::Cut, 1e-6).expect("dome hole cut");
    assert_valid(&out.body);
    // every face meshes, and the mesh reaches the pole (z = -(h + r))
    let t = ck_tess::mesh::tessellate(&out.body);
    let missing: Vec<_> = out.body.face_ids().filter(|f| t.faces[f.idx()].is_none()).map(|f| out.body.face_id_str(f)).collect();
    assert!(missing.is_empty(), "unmeshed faces {missing:?}");
    let zmin = t.faces.iter().flatten().flat_map(|m| m.pts.iter()).map(|p| p.z).fold(f64::INFINITY, f64::min);
    assert!((zmin + (h + r)).abs() < 1e-6, "mesh stops at z={zmin}");
    let want = dome_column_volume(r, h, hr, hx, hy);
    let got = v0 - mass_props(&out.body).volume;
    assert!((got - want).abs() < 2e-3 * want, "({hx},{hy}) removed {got} vs {want}");
}

fn mesh_zmin(b: &Body) -> f64 {
    let t = ck_tess::mesh::tessellate(b);
    let missing: Vec<_> = b.face_ids().filter(|f| t.faces[f.idx()].is_none()).map(|f| b.face_id_str(f)).collect();
    assert!(missing.is_empty(), "unmeshed faces {missing:?}");
    t.faces.iter().flatten().flat_map(|m| m.pts.iter()).map(|p| p.z).fold(f64::INFINITY, f64::min)
}

#[test]
fn dome_meshes_to_the_pole() {
    let z = mesh_zmin(&dome(42.88, 38.86));
    assert!((z + 81.74).abs() < 1e-6, "mesh stops at z={z}");
}

#[test]
fn dome_with_sketch_noise_is_a_sphere() {
    // arc centre 1.3e-4 off the axis (dataset sketches): still a sphere, meshed to the pole
    let b = dome_off(42.88, 38.86, -1.3239440585806916e-4, -1.3239440585806916e-4);
    let kinds: Vec<_> = b.face_ids().map(|f| ck_topo::body::wire_kind(&b.face(f).surface)).collect();
    assert_eq!(kinds, ["plane", "cylinder", "sphere"]);
    let z = mesh_zmin(&b);
    assert!((z + 81.74).abs() < 1e-6, "mesh stops at z={z}");
}

/// Dome (optionally with dataset sketch noise) cut by `holes` through holes parallel to its axis.
fn dome_holes(noisy: bool, holes: &[(f64, f64)]) {
    let (r, h, hr) = (42.88, 38.86, 2.12);
    let n = if noisy { -1.3239440585806916e-4 } else { 0.0 };
    let d = dome_off(r, h, n, n);
    let regions: Vec<ProfileRegion> = holes
        .iter()
        .map(|&(x, y)| ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("e1", v(x, y), hr)]), holes: vec![] })
        .collect();
    let tool = extrude("F6", &Profile::new(front(), regions), ExtrudeOpts { s0: -200.0, s1: 200.0, dir: None, draft: 0.0 }).unwrap();
    let out = boolean(&d, &tool, Op::Cut, 1e-6).expect("dome holes");
    assert_valid(&out.body);
    let want: f64 = holes.iter().map(|&(x, y)| dome_column_volume(r, h, hr, x, y)).sum();
    let got = mass_props(&d).volume - mass_props(&out.body).volume;
    assert!((got - want).abs() < 2e-3 * want, "removed {got} vs {want}");
    let z = mesh_zmin(&out.body);
    assert!((z + (h + r)).abs() < 1e-6, "mesh stops at z={z}");
}

const BOLT_CIRCLE: [(f64, f64); 4] = [(29.5, 0.0), (0.0, 29.5), (-29.5, 0.0), (0.0, -29.5)];

#[test]
fn noisy_dome_one_hole() {
    dome_holes(true, &[(0.0, 29.5)]);
}

#[test]
fn exact_dome_four_holes() {
    dome_holes(false, &BOLT_CIRCLE);
}

#[test]
fn noisy_dome_seam_hole() {
    dome_holes(true, &[(29.5, 0.0)]);
}

#[test]
fn noisy_dome_two_holes() {
    dome_holes(true, &[(29.5, 0.0), (0.0, 29.5)]);
}

#[test]
fn noisy_dome_four_holes() {
    dome_holes(true, &BOLT_CIRCLE);
}

#[test]
fn dome_hole_off_seam() {
    dome_hole(0.0, 29.5);
}

#[test]
fn dome_hole_straddling_seam() {
    dome_hole(29.5, 0.0);
}

/// Point classification against a face with a large circular rim (a flange annulus): with 24 chords per
/// circle the polygon sags 0.3 mm inside the r = 35.1 rim, so a hole arc 0.16 mm inside the rim read as
/// outside the face and the cut came back non-manifold (bellows flange).
#[test]
fn flange_hole_breaking_out_of_the_rim() {
    let (ri, ro, t, hr, bc) = (12.167, 35.1, 4.15, 1.915, 33.759);
    let outer = ProfileLoop::new(vec![
        ProfileSeg::line("e1", v(ri, 0.0), v(ro, 0.0)),
        ProfileSeg::line("e2", v(ro, 0.0), v(ro, t)),
        ProfileSeg::line("e3", v(ro, t), v(ri, t)),
        ProfileSeg::line("e4", v(ri, t), v(ri, 0.0)),
    ]);
    let plate = revolve("F2", &Profile::new(top(), region(outer)), Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), 0.0, TAU).unwrap();
    for deg in [0.0f64, 60.0, 120.0, 200.0] {
        let a = deg.to_radians();
        let (hx, hy) = (bc * a.cos(), bc * a.sin());
        let tool = cyl("F4", front(), hx, hy, hr, -10.0, 10.0);
        let out = boolean(&plate, &tool, Op::Cut, 1e-6).unwrap_or_else(|e| panic!("hole at {deg} deg: {e}"));
        assert_valid(&out.body);
        // removed volume = thickness * area(hole disc inside the annulus)
        let n = 600;
        let mut area = 0.0;
        for i in 0..n {
            for j in 0..n {
                let (x, y) = (hx + hr * (-1.0 + 2.0 * (i as f64 + 0.5) / n as f64), hy + hr * (-1.0 + 2.0 * (j as f64 + 0.5) / n as f64));
                let rr = (x * x + y * y).sqrt();
                if (x - hx).powi(2) + (y - hy).powi(2) <= hr * hr && rr >= ri && rr <= ro {
                    area += (2.0 * hr / n as f64).powi(2);
                }
            }
        }
        let got = mass_props(&plate).volume - mass_props(&out.body).volume;
        assert!((got - area * t).abs() < 2e-3 * area * t, "hole at {deg} deg removed {got} vs {}", area * t);
    }
}

// ---- eyebolt: torus eye unioned with the shank ----

fn right() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::Z, y: Vec3::Y, z: Vec3::new(-1.0, 0.0, 0.0) }
}

/// Torus ring (major 49, minor 14) with its axis along Z through (0, 59.67, 0).
fn eye() -> Body {
    let outer = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(0.0, 108.67), 14.0)]);
    // axis: sketch line y = 59.67, along sketch x (world Z)
    revolve("F12", &Profile::new(right(), region(outer)), Vec3::new(0.0, 59.67, 0.0), Vec3::Z, 0.0, TAU).unwrap()
}

fn shank_parts() -> Vec<Body> {
    let collar = cyl("F2", top(), 0.0, 0.0, 35.0, 0.0, 26.0);
    let neck = {
        let q = [v(0.0, 26.0), v(19.39, 26.0), v(16.0, 31.67), v(0.0, 31.67)];
        let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), q[i], q[(i + 1) % 4])).collect());
        revolve("F7", &Profile::new(front(), region(lp)), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap()
    };
    let rod = cyl("F10", top(), 0.0, 0.0, 18.0, -54.0, 0.0);
    vec![collar, neck, rod]
}

#[test]
fn eyebolt_eye_union_shank() {
    let mut body = shank_parts().remove(0);
    for (i, part) in shank_parts().into_iter().enumerate().skip(1) {
        body = boolean(&body, &part, Op::Union, 1e-6).unwrap_or_else(|e| panic!("shank part {i}: {e}")).body;
    }
    let shank_v = mass_props(&body).volume;
    let joined = boolean(&body, &eye(), Op::Union, 1e-6).expect("eye union shank").body;
    assert_valid(&joined);
    let v = mass_props(&joined).volume;
    // ground truth (CadQuery) of the full eyebolt
    assert!((v - 309589.0).abs() < 2e-3 * 309589.0, "volume {v} (shank {shank_v})");
    let t = ck_tess::mesh::tessellate(&joined);
    let missing: Vec<_> = joined.face_ids().filter(|f| t.faces[f.idx()].is_none()).map(|f| joined.face_id_str(f)).collect();
    assert!(missing.is_empty(), "unmeshed faces {missing:?}");
}
