//! `proc_length` (the length of a procedural curve, used to sample the edges of blend and sweep sheets) has a work budget in
//! Gauss-Kronrod panels. Without it, the iso-curve of a rolling-ball blend far from the origin bisected to depth 30 (a hang),
//! because its speed is a finite difference whose noise grows with the coordinates. Timings are printed (run with --nocapture).
use ck_geom::curve::Curve;
use ck_geom::procsurf::{RollingBall, SweepLaw, SweepSurface};
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use std::time::Instant;

fn blend_edge(off: f64) -> Curve {
    let o = Vec3::new(off, off, off);
    let rb = RollingBall::new(Surface::plane(o, Vec3::Z), Surface::plane(o, Vec3::X), 1.0, 1.0, (3.0, 3.0), Curve::line(o, Vec3::Y), (0.0, 25.0)).unwrap();
    Curve::iso(Surface::Blend(Box::new(rb)), false, 0.0)
}

fn sweep_edge(off: f64) -> Curve {
    let o = Vec3::new(off, off, off);
    let prof = Curve::circle_x(o, Vec3::Y, Vec3::X, 2.0);
    let path = Curve::line(o, Vec3::Y);
    let s = SweepSurface::new(prof, path, SweepLaw::Translate, (0.0, 25.0), (0.0, std::f64::consts::TAU)).unwrap();
    Curve::iso(Surface::Sweep(Box::new(s)), false, 0.0)
}

#[test]
fn blend_edge_length_is_bounded_far_from_the_origin() {
    for off in [0.0, 15.0, 4000.0, 1.0e6, 1.0e9] {
        let e = blend_edge(off);
        let t = Instant::now();
        let len = e.length_between(0.0, 25.0);
        let dt = t.elapsed();
        eprintln!("blend iso length offset {off:e}: {len} in {dt:?}");
        assert!((len - 25.0).abs() <= 1e-6 * 25.0 + 1e-9 * off, "offset {off}: length {len}");
        assert!(dt.as_secs_f64() < 60.0, "offset {off}: {dt:?}");
    }
}

#[test]
fn sweep_edge_length_is_bounded_far_from_the_origin() {
    for off in [0.0, 15.0, 4000.0, 1.0e6, 1.0e9] {
        let e = sweep_edge(off);
        let t = Instant::now();
        let len = e.length_between(0.0, 25.0);
        let dt = t.elapsed();
        eprintln!("sweep iso length offset {off:e}: {len} in {dt:?}");
        assert!((len - 25.0).abs() <= 1e-6 * 25.0 + 1e-9 * off, "offset {off}: length {len}");
        assert!(dt.as_secs_f64() < 60.0, "offset {off}: {dt:?}");
    }
}
