//! Stage 3: robustness. Coarse and fine tessellations, vertex noise, sliver triangles, uniform and non-uniform scaling,
//! rotation, and the faceted fallback; a timing table by triangle count (the 100k row is `--ignored`).
mod common;
use ck_geom::Vec3;
use ck_mesh::{mesh_to_solid, Converted, Opts};
use common::*;

const HOLE: &str = r#"{"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"k","x":5,"y":3}],"entities":[{"id":"pt1","type":"point","p":"k"}],"constraints":[]},
{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}"#;
const FILLETS: &str = r#"{"type":"fillet","id":"F5","radius":5,"edges":[{"faces":["F2.S.e1","F2.S.e2"]},{"faces":["F2.S.e2","F2.S.e3"]},{"faces":["F2.S.e3","F2.S.e4"]},{"faces":["F2.S.e4","F2.S.e1"]}]}"#;
const SPHERE: &str = r#"{"type":"sphere","id":"F1","op":"add","center":[0,0,0],"radius":10}"#;

fn shaft() -> String {
    revolved(&[(0.0, 0.0), (10.0, 0.0), (10.0, 20.0), (6.0, 30.0), (6.0, 40.0), (0.0, 40.0)])
}

fn run(s: &[f64], o: &Opts, what: &str) -> Converted {
    let t = std::time::Instant::now();
    let c = match mesh_to_solid(s, None, o) {
        Ok(c) => c,
        Err(r) => panic!("{what}: refused {}: {}", r.code, r.message),
    };
    let a = c.report.analytic.as_ref();
    eprintln!(
        "{what}: {} triangles -> {} {} in {:.0} ms; faceted {:?}; declined {}; mesh vol {:.4} body {:.4}; dev {:?}",
        s.len() / 9,
        c.body.num_faces(),
        kinds(&c.body),
        t.elapsed().as_secs_f64() * 1e3,
        a.map(|a| a.faceted),
        a.map_or(0, |a| a.declined.len()),
        c.report.mesh_volume,
        c.report.body_volume,
        c.report.deviation.as_ref().map(|d| (d.max, d.rms))
    );
    let issues = ck_check::validate(&c.body, 1e-6);
    assert!(issues.is_empty(), "{what}: validate: {:?}", &issues[..issues.len().min(5)]);
    c
}

fn map(s: &[f64], f: impl Fn(Vec3) -> Vec3) -> Vec<f64> {
    s.chunks(3).flat_map(|c| { let q = f(Vec3::new(c[0], c[1], c[2])); [q.x, q.y, q.z] }).collect()
}

#[test]
fn coarse_and_fine_tessellations() {
    for (src, want) in [
        (format!("{RECT},{HOLE}"), r#"{"cylinder":1,"plane":6}"#),
        (format!("{RECT},{FILLETS}"), r#"{"cylinder":4,"plane":6}"#),
        (shaft(), r#"{"cone":1,"cylinder":2,"plane":2}"#),
        (SPHERE.to_string(), r#"{"sphere":1}"#),
    ] {
        let b = body_of(&src);
        let v = ck_topo::mass::mass_props(&b).volume;
        for (q, f) in [("coarse", 10.0), ("fine", 0.25)] {
            let c = run(&soup(&b, f), &Opts::default(), q);
            assert_eq!(kinds(&c.body), want, "{q}");
            let rel = (c.report.body_volume - v) / v;
            assert!(rel.abs() < 5e-3, "{q} volume rel {rel}");
        }
    }
}

#[test]
fn rotated_and_uniformly_scaled() {
    let b = body_of(&shaft());
    let s = soup(&b, 1.0);
    let ax = Vec3::new(0.3, -0.5, 0.8).normalize();
    let ang: f64 = 1.1;
    let rot = |p: Vec3| p * ang.cos() + ax.cross(p) * ang.sin() + ax * ax.dot(p) * (1.0 - ang.cos()) + Vec3::new(120.0, -40.0, 7.0);
    let c = run(&map(&s, rot), &Opts::default(), "rotated");
    assert_eq!(kinds(&c.body), r#"{"cone":1,"cylinder":2,"plane":2}"#);
    for k in [1e-3, 1e3] {
        let c = run(&map(&s, |p| p * k), &Opts::default(), "scaled");
        assert_eq!(kinds(&c.body), r#"{"cone":1,"cylinder":2,"plane":2}"#, "scale {k}");
    }
}

#[test]
fn non_uniform_scaling_falls_back_to_facets() {
    // stretched along x the hole is an elliptic cylinder: it stays faceted, the box's planes are still merged
    let b = body_of(&format!("{RECT},{HOLE}"));
    let s = soup(&b, 1.0);
    let c = run(&map(&s, |p| Vec3::new(2.0 * p.x, p.y, p.z)), &Opts::default(), "stretched hole");
    let a = c.report.analytic.as_ref().unwrap();
    assert_eq!(a.cylinders, 0);
    assert!(a.faceted >= 8, "faceted {}", a.faceted);
    assert!(c.report.volume_rel_err().abs() < 1e-6);
    // a stretched sphere (an ellipsoid) is faceted too, and valid
    let b = body_of(SPHERE);
    let c = run(&map(&soup(&b, 4.0), |p| Vec3::new(p.x, 1.5 * p.y, p.z)), &Opts::default(), "ellipsoid");
    assert_eq!(c.report.analytic.as_ref().unwrap().spheres, 0);
    assert!(c.report.volume_rel_err().abs() < 1e-6);
}

/// Deterministic noise per position (the same vertex in several triangles moves the same way).
fn noisy(s: &[f64], amp: f64) -> Vec<f64> {
    map(s, |p| {
        let h = |k: u64| {
            let mut x = (p.x * 1e4).round() as i64 as u64 ^ ((p.y * 1e4).round() as i64 as u64).rotate_left(21) ^ ((p.z * 1e4).round() as i64 as u64).rotate_left(42) ^ k;
            x ^= x >> 33;
            x = x.wrapping_mul(0xff51afd7ed558ccd);
            x ^= x >> 33;
            (x % 2001) as f64 / 1000.0 - 1.0
        };
        p + Vec3::new(h(1), h(2), h(3)) * amp
    })
}

#[test]
fn vertex_noise() {
    let b = body_of(&format!("{RECT},{HOLE}"));
    let s = soup(&b, 1.0);
    let diag = 73.0;
    let amp = 1e-4 * diag;
    let n = noisy(&s, amp);
    // default tolerances: the planes no longer merge (their corners are off by the noise) and nothing fits; still a solid
    let c = run(&n, &Opts::default(), "noise, default tolerances");
    assert!(c.body.num_faces() > 7);
    // tolerances above the noise: the faces come back
    let mut o = Opts::default();
    o.plane_tol = Some(4.0 * amp);
    o.angle_tol_deg = 10.0;
    o.fit.fit_tol = Some(4.0 * amp);
    let c = run(&n, &o, "noise, tolerances above it");
    assert_eq!(kinds(&c.body), r#"{"cylinder":1,"plane":6}"#);
}

#[test]
fn sliver_triangles() {
    // every triangle split 1-to-4 at points very near its corners: many needle triangles, shared consistently
    let b = body_of(&format!("{RECT},{HOLE}"));
    let s = soup(&b, 1.0);
    let mut out = vec![];
    let key = |p: Vec3| ((p.x * 1e5).round() as i64, (p.y * 1e5).round() as i64, (p.z * 1e5).round() as i64);
    let near = |a: Vec3, b: Vec3| if key(a) < key(b) { a + (b - a) * 1e-4 } else { b + (a - b) * 1e-4 };
    for t in s.chunks(9) {
        let v: Vec<Vec3> = t.chunks(3).map(|c| Vec3::new(c[0], c[1], c[2])).collect();
        let (a, b2, c) = (v[0], v[1], v[2]);
        let (ab, bc, ca) = (near(a, b2), near(b2, c), near(c, a));
        for tri in [[a, ab, ca], [ab, b2, bc], [ca, bc, c], [ab, bc, ca]] {
            for p in tri {
                out.extend([p.x, p.y, p.z]);
            }
        }
    }
    let f = run(&out, &opts_faceted(), "slivers, faceted");
    assert!(f.report.volume_rel_err().abs() < 1e-6);
    let c = run(&out, &Opts::default(), "slivers");
    let v = ck_topo::mass::mass_props(&b).volume;
    assert!(((c.report.body_volume - v) / v).abs() < 5e-3);
    // the box's 6 planes merge whatever the slivers (the growth passes through them); the hole comes back as a cylinder
    // although its rim points sit on the chords (within the sag)
    let big = c.body.face_ids().filter(|&f| matches!(c.body.face(f).surface, ck_geom::surface::Surface::Plane { .. }) && {
        let es = c.body.face_edges(f);
        let ps: Vec<Vec3> = es.iter().map(|&e| c.body.vertex(c.body.edge(e).v0).p).collect();
        ps.iter().any(|a| ps.iter().any(|b| a.dist(*b) > 20.0))
    }).count();
    assert_eq!(big, 6);
    assert_eq!(f.body.num_faces(), 54);
    assert_eq!(kinds(&c.body), r#"{"cylinder":1,"plane":6}"#);
}

fn timing(src: &str, target: usize) -> (usize, f64, String) {
    let b = body_of(src);
    let (lo, hi) = b.bbox();
    let base = ck_tess::curvature::Tol::for_bbox(lo, hi);
    // find a chord that gives about `target` triangles
    let mut f = 1.0f64;
    let mut s = soup_tol(&b, &base);
    for _ in 0..30 {
        let n = s.len() / 9;
        if (n as f64) > 0.8 * target as f64 && (n as f64) < 1.25 * target as f64 {
            break;
        }
        f *= (n as f64 / target as f64).max(0.05).min(20.0);
        let mut t = base;
        t.chord *= f;
        s = soup_tol(&b, &t);
    }
    let t = std::time::Instant::now();
    let c = mesh_to_solid(&s, None, &Opts::default()).ok().expect("converted");
    (s.len() / 9, t.elapsed().as_secs_f64() * 1e3, kinds(&c.body))
}

#[test]
fn timing_1k_10k() {
    for (name, src) in [("sphere", SPHERE.to_string()), ("shaft", shaft())] {
        for n in [1_000usize, 10_000] {
            let (got, ms, k) = timing(&src, n);
            eprintln!("TIMING {name} {got} triangles {ms:.0} ms {k}");
            assert!(ms < 60_000.0);
        }
    }
}

#[test]
#[ignore = "diagnostic: timing print-out at 100k triangles, no assertion; run by hand"]
fn timing_100k() {
    for (name, src) in [("sphere", SPHERE.to_string()), ("shaft", shaft())] {
        let (got, ms, k) = timing(&src, 100_000);
        eprintln!("TIMING {name} {got} triangles {ms:.0} ms {k}");
    }
    // a scan-like mesh: the fine sphere with noise above the fit tolerance, nothing fits; and the faceted path alone
    let b = body_of(SPHERE);
    let (lo, hi) = b.bbox();
    let mut t = ck_tess::curvature::Tol::for_bbox(lo, hi);
    t.chord *= 0.1;
    let s = noisy(&soup_tol(&b, &t), 2e-3);
    let big = Opts { max_faces: 200_000, analytic: false, ..Default::default() };
    for (what, o) in [("noisy sphere", Opts::default()), ("noisy sphere, faceted only", opts_faceted()), ("noisy sphere, faceted, maxFaces 200k", big)] {
        let t0 = std::time::Instant::now();
        let r = mesh_to_solid(&s, None, &o);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        match r {
            Ok(c) => eprintln!("TIMING {what} {} triangles {ms:.0} ms {} ({:?})", s.len() / 9, kinds(&c.body), c.report.analytic.as_ref().and_then(|a| a.reason.clone())),
            Err(e) => eprintln!("TIMING {what} {} triangles {ms:.0} ms refused {} {}", s.len() / 9, e.code, e.message),
        }
    }
}
