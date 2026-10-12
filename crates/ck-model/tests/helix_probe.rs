//! Diagnostic (ignored): `PART=path UPTO=n cargo test -p ck-model --profile fast
//! --test helix_probe -- --ignored --nocapture` regenerates a part (up to feature n),
//! printing the elapsed time and volume.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::time::Instant;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

#[test]
#[ignore]
fn helix_probe() {
    let path = std::env::var("PART").unwrap();
    let upto: Option<u32> = std::env::var("UPTO").ok().and_then(|s| s.parse().ok());
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts { up_to: upto, ..RegenOpts::default() }, &mut ex);
    eprintln!("regen {:.2}s errors {}", t.elapsed().as_secs_f64(), r.to_json().to_string().chars().take(300).collect::<String>());
    let b = ex.0.body.as_ref().unwrap();
    let t = Instant::now();
    let m = ck_topo::mass::mass_props(b);
    eprintln!("mass_props {:.2}s {:?} lumps {} faces {}", t.elapsed().as_secs_f64(), m, b.num_lumps(), b.face_ids().count());
}

/// Deviation of the (Y-up) sweep faces from the ideal screw tube R=56.25 r=3.52 pitch 17.24.
#[test]
#[ignore]
fn helix_dev() {
    let path = std::env::var("PART").unwrap();
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    let (big_r, rad, k) = (56.25f64, 3.52f64, 17.24 / std::f64::consts::TAU);
    let (mut worst, mut sum, mut n, mut smin, mut smax) = (0.0f64, 0.0, 0, f64::INFINITY, f64::NEG_INFINITY);
    for f in b.face_ids() {
        let ck_geom::surface::Surface::BSpline(bs) = &b.face(f).surface else { continue };
        let ((u0, u1), (v0, v1)) = bs.domain();
        for i in 0..=1200 {
            for j in 0..=48 {
                let p = bs.eval(u0 + (u1 - u0) * i as f64 / 1200.0, v0 + (v1 - v0) * j as f64 / 48.0);
                let rho = p.x.hypot(p.z);
                let alpha = (-p.z).atan2(p.x);
                let mut best = f64::INFINITY;
                for m in -2..16 {
                    let phi = alpha + std::f64::consts::TAU * m as f64;
                    let s = (rho - big_r).hypot(p.y - k * phi) - rad;
                    if s.abs() < best.abs() { best = s; }
                }
                worst = worst.max(best.abs());
                sum += best.abs();
                smin = smin.min(best);
                smax = smax.max(best);
                n += 1;
            }
        }
    }
    eprintln!("sweep faces: max |dev| {worst:.6}, mean {:.6}, signed [{smin:.6}, {smax:.6}] over {n}", sum / n as f64);
}
