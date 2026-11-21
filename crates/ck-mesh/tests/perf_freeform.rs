//! A scan-like free-form mesh (a bumpy, noisy sphere of about 50000 triangles) through the analytic refit: how long each
//! stage takes, and that the work budget of the refit bounds it. Timing prints are for `--nocapture`.
mod common;
use std::f64::consts::PI;
use std::time::Instant;

fn clock() -> f64 {
    thread_local! { static T0: Instant = Instant::now(); }
    T0.with(|t| t.elapsed().as_secs_f64() * 1e3)
}

/// A closed UV blob: radius 25 x (1 + sum of sines) plus noise of `noise` mm, `nlat` x `nlon` quads.
pub fn blob(nlat: usize, nlon: usize, noise: f64) -> (Vec<f64>, Vec<u32>) {
    let mut s = 987654321u64;
    let mut rnd = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0
    };
    let r = |th: f64, ph: f64| 25.0 * (1.0 + 0.08 * (3.0 * th).sin() * (2.0 * ph).cos() + 0.05 * (7.0 * th + ph).sin() + 0.03 * (11.0 * ph).sin() * th.sin());
    let mut p = vec![0.0, 0.0, r(0.0, 0.0)];
    for i in 1..nlat {
        let th = PI * i as f64 / nlat as f64;
        for j in 0..nlon {
            let ph = 2.0 * PI * j as f64 / nlon as f64;
            let rr = r(th, ph) + noise * rnd();
            p.extend([rr * th.sin() * ph.cos(), rr * th.sin() * ph.sin(), rr * th.cos()]);
        }
    }
    p.extend([0.0, 0.0, -r(PI, 0.0)]);
    let ring = |i: usize, j: usize| (1 + (i - 1) * nlon + j % nlon) as u32;
    let south = (p.len() / 3 - 1) as u32;
    let mut ix = vec![];
    for j in 0..nlon {
        ix.extend([0, ring(1, j), ring(1, j + 1)]);
        ix.extend([south, ring(nlat - 1, j + 1), ring(nlat - 1, j)]);
    }
    for i in 1..nlat - 1 {
        for j in 0..nlon {
            let (a, b, c, d) = (ring(i, j), ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j));
            ix.extend([a, d, c, a, c, b]);
        }
    }
    (p, ix)
}

fn run(name: &str, nlat: usize, nlon: usize, noise: f64, o: &ck_mesh::Opts) -> (ck_mesh::Converted, f64) {
    ck_core::budget::set_clock(clock);
    let (p, ix) = blob(nlat, nlon, noise);
    let t = clock();
    let c = ck_mesh::mesh_to_solid(&p, Some(&ix), o).unwrap_or_else(|r| panic!("{name}: {} {}", r.code, r.message));
    let s = (clock() - t) / 1e3;
    eprintln!("{name}: {} triangles, {} faces, {s:.2} s; stages {:?}; analytic {:?}", c.report.input_triangles, c.body.num_faces(), c.report.stages.iter().map(|(k, v)| format!("{k} {:.0}", v)).collect::<Vec<_>>(), c.report.analytic.as_ref().map(|a| (a.applied, a.budget_exhausted, a.fitted_fraction, a.reason.clone())));
    if let Ok(path) = std::env::var("CK_MESH_PERF_OUT") {
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let st: Vec<String> = c.report.stages.iter().map(|(k, v)| format!("{k} {v:.0}")).collect();
            let _ = writeln!(f, "{name}: {} triangles, {} faces, {s:.2} s; stages ms [{}]", c.report.input_triangles, c.body.num_faces(), st.join(", "));
        }
    }
    (c, s)
}

fn with_seconds(s: f64) -> ck_mesh::Opts {
    let mut o = ck_mesh::Opts { max_faces: 250_000, ..Default::default() };
    o.fit.max_seconds = s;
    o
}

/// By hand (CK_MESH_PERF=1): the refit without a budget (what a scan used to cost) and how many work units that is.
#[test]
fn timings_unbounded() {
    if std::env::var("CK_MESH_PERF").is_err() {
        return; // by hand: CK_MESH_PERF=1 (the CI box: --env CK_MESH_PERF=1)
    }
    let (c, s) = run("blob 50k noisy, no budget", 112, 224, 0.05, &with_seconds(0.0));
    let a = c.report.analytic.as_ref().unwrap();
    let refit = c.report.stages.iter().find(|(k, _)| *k == "refit").unwrap().1 / 1e3;
    eprintln!("noisy: {s:.1} s total, refit {refit:.1} s, {} units -> {:.3e} units/s", a.work_units, a.work_units as f64 / refit);
    let (c, s) = run("blob 50k smooth, no budget", 112, 224, 0.0, &with_seconds(0.0));
    let a = c.report.analytic.as_ref().unwrap();
    let refit = c.report.stages.iter().find(|(k, _)| *k == "refit").unwrap().1 / 1e3;
    eprintln!("smooth: {s:.1} s total, refit {refit:.1} s, {} units -> {:.3e} units/s", a.work_units, a.work_units as f64 / refit);
}

/// The default budget: the refit of a noisy 50000 triangle free-form mesh stops in time, stays faceted and valid.
#[test]
fn the_refit_of_a_free_form_scan_is_bounded() {
    let (c, _) = run("blob 50k, 1 s budget", 112, 224, 0.05, &with_seconds(1.0));
    let a = c.report.analytic.as_ref().unwrap();
    let refit = c.report.stages.iter().find(|(k, _)| *k == "refit").unwrap().1 / 1e3;
    assert!(a.budget_exhausted, "the budget should have run out ({} units)", a.work_units);
    assert!(a.fitted_fraction < 0.5);
    assert!(refit < 60.0, "refit took {refit:.1} s on a 1 s budget");
    assert!(ck_check::validate(&c.body, 1e-6).is_empty());
    assert!(c.report.volume_rel_err().abs() < 1e-6, "{}", c.report.volume_rel_err());
    let j = ck_mesh::request::report_json("m", &c.body, &c.report);
    assert_eq!(j.path(&["analytic", "budgetExhausted"]).and_then(|v| v.as_bool()), Some(true));
}

/// The same with no host clock (the node wasm host has none): the work units alone stop it.
#[test]
fn the_work_units_bound_the_refit_without_a_clock() {
    let (p, ix) = blob(112, 224, 0.05);
    let t = Instant::now();
    let c = ck_mesh::mesh_to_solid(&p, Some(&ix), &with_seconds(0.5)).unwrap();
    eprintln!("no clock, 0.5 s of units: {:.2} s for the whole conversion", t.elapsed().as_secs_f64());
    let a = c.report.analytic.as_ref().unwrap();
    assert!(a.budget_exhausted && a.work_units >= (0.5 * ck_mesh::analytic::UNITS_PER_SECOND) as u64, "{} units", a.work_units);
    assert!(c.report.stages.is_empty(), "no clock, no stage times");
    assert!(ck_check::validate(&c.body, 1e-6).is_empty());
}

/// A budget that is plenty changes nothing for a CAD mesh: the fitted faces are the same as without a limit.
#[test]
fn a_cad_mesh_is_fitted_within_the_default_budget() {
    let (p, ix) = uv_sphere_like();
    let c = ck_mesh::mesh_to_solid(&p, Some(&ix), &ck_mesh::Opts::default()).unwrap();
    let a = c.report.analytic.as_ref().unwrap();
    assert!(!a.budget_exhausted);
    assert_eq!(c.body.num_faces(), 1);
    assert!(a.fitted_fraction > 0.99);
}

fn uv_sphere_like() -> (Vec<f64>, Vec<u32>) {
    // the blob without bumps or noise is a sphere
    let (mut p, ix) = blob(60, 120, 0.0);
    for v in p.chunks_mut(3) {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        v.iter_mut().for_each(|x| *x *= 25.0 / l);
    }
    (p, ix)
}

/// By hand (CK_MESH_PERF=1): a 200000 triangle free-form scan with the default budget, stage by stage.
#[test]
fn timings_free_form_200k_default_budget() {
    if std::env::var("CK_MESH_PERF").is_err() {
        return;
    }
    let o = ck_mesh::Opts { max_faces: 250_000, ..Default::default() };
    let (c, s) = run("blob 200k noisy, default budget", 224, 446, 0.05, &o);
    eprintln!("200k: {s:.1} s total");
    assert!(ck_check::validate(&c.body, 1e-6).is_empty());
}

/// By hand (CK_MESH_PERF=1): the 50000 triangle free-form scan with the default budget, stage by stage.
#[test]
fn timings_free_form_50k_default_budget() {
    if std::env::var("CK_MESH_PERF").is_err() {
        return;
    }
    let o = ck_mesh::Opts { max_faces: 250_000, ..Default::default() };
    let (c, s) = run("blob 50k noisy, default budget", 112, 224, 0.05, &o);
    assert!(ck_check::validate(&c.body, 1e-6).is_empty());
    assert!(s < 20.0, "{s:.1} s");
}
