//! Quadric decimation (`ck_mesh::decimate`): noisy procedural scans (sphere, torus, a 120k-triangle blob), a box with
//! creases, determinism, speed, an independent deviation measurement, and `meshToSolid` on the decimated result. The real
//! scan of a toy van is used when `CK_VAN_STL` (or ~/Downloads/interceptor/van_86mm_full_120k.stl) exists.

use ck_geom::Vec3;
use ck_mesh::decimate::{decimate, DecimateOpts};
use ck_mesh::deviation::Bvh;
use std::collections::HashMap;
use std::time::Instant;

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

/// A closed surface of revolution-like grid `nu` x `nv` with poles: `f(u, v) -> point`, u in [0, 1) around, v in (0, 1).
fn pole_grid(nu: usize, nv: usize, f: impl Fn(f64, f64) -> Vec3) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut p = vec![f(0.0, 0.0)];
    for j in 1..nv {
        for i in 0..nu {
            p.push(f(i as f64 / nu as f64, j as f64 / nv as f64));
        }
    }
    p.push(f(0.0, 1.0));
    let south = (p.len() - 1) as u32;
    let at = |i: usize, j: usize| (1 + (j - 1) * nu + i % nu) as u32;
    let mut t = vec![];
    for i in 0..nu {
        t.push([0, at(i + 1, 1), at(i, 1)]);
        for j in 1..nv - 1 {
            t.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            t.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
        t.push([south, at(i, nv - 1), at(i + 1, nv - 1)]);
    }
    (p, t)
}

fn noisy(p: &mut [Vec3], amp: f64, seed: u64) {
    let mut r = Lcg(seed);
    for q in p.iter_mut() {
        *q = *q + Vec3::new(r.next() - 0.5, r.next() - 0.5, r.next() - 0.5) * (2.0 * amp);
    }
}

fn sphere(nu: usize, nv: usize, r: f64, noise: f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut p, t) = pole_grid(nu, nv, |u, v| {
        let (a, b) = (u * std::f64::consts::TAU, v * std::f64::consts::PI);
        Vec3::new(r * b.sin() * a.cos(), r * b.sin() * a.sin(), -r * b.cos())
    });
    noisy(&mut p, noise, 1);
    (p, t)
}

fn blob(nu: usize, nv: usize, noise: f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut p, t) = pole_grid(nu, nv, |u, v| {
        let (a, b) = (u * std::f64::consts::TAU, v * std::f64::consts::PI);
        let r = 20.0 * (1.0 + 0.12 * (3.0 * a).sin() * b.sin().powi(2) + 0.1 * (2.0 * b).cos() + 0.05 * (5.0 * a + 2.0 * b).sin());
        Vec3::new(1.4 * r * b.sin() * a.cos(), r * b.sin() * a.sin(), 0.8 * -r * b.cos())
    });
    noisy(&mut p, noise, 7);
    (p, t)
}

fn torus(nu: usize, nv: usize, big: f64, small: f64, noise: f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut p = vec![];
    for j in 0..nv {
        for i in 0..nu {
            let (a, b) = (i as f64 / nu as f64 * std::f64::consts::TAU, j as f64 / nv as f64 * std::f64::consts::TAU);
            p.push(Vec3::new((big + small * b.cos()) * a.cos(), (big + small * b.cos()) * a.sin(), small * b.sin()));
        }
    }
    let at = |i: usize, j: usize| ((j % nv) * nu + i % nu) as u32;
    let mut t = vec![];
    for j in 0..nv {
        for i in 0..nu {
            t.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            t.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    noisy(&mut p, noise, 3);
    (p, t)
}

/// A box 40 x 30 x 20 as a grid of triangles (`n` per unit of 5 mm), outward oriented, welded, slightly noisy inside faces
/// only by nothing: the faces are flat so only the creases matter.
fn boxy(step: f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (sx, sy, sz) = (40.0, 30.0, 20.0);
    let mut p: Vec<Vec3> = vec![];
    let mut t: Vec<[u32; 3]> = vec![];
    let mut key: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut vid = |q: Vec3, p: &mut Vec<Vec3>| -> u32 {
        let k = ((q.x * 1000.0).round() as i64, (q.y * 1000.0).round() as i64, (q.z * 1000.0).round() as i64);
        *key.entry(k).or_insert_with(|| {
            p.push(q);
            (p.len() - 1) as u32
        })
    };
    // six faces: origin, u axis, v axis (right handed so that u x v is outward)
    let faces = [
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, sy, 0.0), Vec3::new(sx, 0.0, 0.0), sy, sx),
        (Vec3::new(0.0, 0.0, sz), Vec3::new(sx, 0.0, 0.0), Vec3::new(0.0, sy, 0.0), sx, sy),
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(sx, 0.0, 0.0), Vec3::new(0.0, 0.0, sz), sx, sz),
        (Vec3::new(0.0, sy, 0.0), Vec3::new(0.0, 0.0, sz), Vec3::new(sx, 0.0, 0.0), sz, sx),
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, sz), Vec3::new(0.0, sy, 0.0), sz, sy),
        (Vec3::new(sx, 0.0, 0.0), Vec3::new(0.0, sy, 0.0), Vec3::new(0.0, 0.0, sz), sy, sz),
    ];
    for (o, u, v, lu, lv) in faces {
        let (nu, nv) = ((lu / step).round() as usize, (lv / step).round() as usize);
        let un = u / lu;
        let vn = v / lv;
        for i in 0..nu {
            for j in 0..nv {
                let c = |a: usize, b: usize| o + un * (a as f64 * lu / nu as f64) + vn * (b as f64 * lv / nv as f64);
                let (a, b, cc, d) = (vid(c(i, j), &mut p), vid(c(i + 1, j), &mut p), vid(c(i + 1, j + 1), &mut p), vid(c(i, j + 1), &mut p));
                t.push([a, b, cc]);
                t.push([a, cc, d]);
            }
        }
    }
    // orient outward by the volume sign of each face's first triangle against the centre
    let centre = Vec3::new(sx / 2.0, sy / 2.0, sz / 2.0);
    for tri in t.iter_mut() {
        let n = (p[tri[1] as usize] - p[tri[0] as usize]).cross(p[tri[2] as usize] - p[tri[0] as usize]);
        if n.dot(p[tri[0] as usize] - centre) < 0.0 {
            tri.swap(1, 2);
        }
    }
    (p, t)
}

fn volume(p: &[Vec3], t: &[[u32; 3]]) -> f64 {
    t.iter().map(|k| p[k[0] as usize].dot(p[k[1] as usize].cross(p[k[2] as usize])) / 6.0).sum()
}

/// Closed and consistently oriented: every directed edge once, and its reverse once.
fn assert_closed_oriented(t: &[[u32; 3]], what: &str) {
    let mut e: HashMap<(u32, u32), u32> = HashMap::new();
    for k in t {
        for i in 0..3 {
            *e.entry((k[i], k[(i + 1) % 3])).or_default() += 1;
        }
    }
    for (&(a, b), &n) in &e {
        assert_eq!(n, 1, "{what}: directed edge {a}->{b} used {n} times");
        assert_eq!(e.get(&(b, a)), Some(&1), "{what}: edge {a}->{b} has no opposite");
    }
}

/// Independent measurement: random points on the original triangles to the result's triangles (BVH), and a brute force
/// cross-check on a subset. Returns (max, rms).
fn measure(op: &[Vec3], ot: &[[u32; 3]], rp: &[Vec3], rt: &[[u32; 3]], n: usize) -> (f64, f64) {
    let bvh = Bvh::new(rt.iter().map(|k| [rp[k[0] as usize], rp[k[1] as usize], rp[k[2] as usize]]).collect());
    let mut r = Lcg(99);
    let (mut mx, mut s2) = (0.0f64, 0.0);
    for i in 0..n {
        let k = ot[(r.next() * ot.len() as f64) as usize % ot.len()];
        let (mut u, mut v) = (r.next(), r.next());
        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        let (a, b, c) = (op[k[0] as usize], op[k[1] as usize], op[k[2] as usize]);
        let s = a + (b - a) * u + (c - a) * v;
        let d = bvh.dist(s);
        mx = mx.max(d);
        s2 += d * d;
        if i < 60 {
            // brute force on the first few
            let bf = rt.iter().map(|k| point_tri_d(s, rp[k[0] as usize], rp[k[1] as usize], rp[k[2] as usize])).fold(f64::INFINITY, f64::min);
            assert!((bf - d).abs() < 1e-9, "bvh {d} vs brute force {bf}");
        }
    }
    (mx, (s2 / n as f64).sqrt())
}

fn point_tri_d(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    // closest point by projecting and clamping through the three edges (simple, slow)
    let n = (b - a).cross(c - a);
    let nn = n.dot(n);
    let mut best = f64::INFINITY;
    if nn > 0.0 {
        let q = p - n * ((p - a).dot(n) / nn);
        let inside = (b - a).cross(q - a).dot(n) >= 0.0 && (c - b).cross(q - b).dot(n) >= 0.0 && (a - c).cross(q - c).dot(n) >= 0.0;
        if inside {
            best = (p - q).len();
        }
    }
    for (x, y) in [(a, b), (b, c), (c, a)] {
        let d = y - x;
        let t = ((p - x).dot(d) / d.dot(d)).clamp(0.0, 1.0);
        best = best.min((p - (x + d * t)).len());
    }
    best
}

fn soup(p: &[Vec3], t: &[[u32; 3]]) -> (Vec<f64>, Vec<u32>) {
    (p.iter().flat_map(|q| [q.x, q.y, q.z]).collect(), t.iter().flatten().copied().collect())
}

#[test]
fn noisy_sphere() {
    let (p, t) = sphere(160, 80, 20.0, 0.02);
    let t0 = Instant::now();
    let r = decimate(&p, &t, &DecimateOpts { tol: 0.1, target_triangles: Some(3000), keep_angle_deg: None }).unwrap();
    eprintln!("sphere {} -> {} tris in {:?}: {:?}", t.len(), r.tris.len(), t0.elapsed(), r.report);
    assert_closed_oriented(&r.tris, "sphere");
    assert!(r.tris.len() <= 3000 && r.tris.len() > 1500, "{}", r.tris.len());
    let (mx, _) = measure(&p, &t, &r.pts, &r.tris, 20000);
    assert!(mx <= 0.1, "true max {mx}");
    let (v0, v1) = (volume(&p, &t), volume(&r.pts, &r.tris));
    assert!(((v1 - v0) / v0).abs() < 0.03, "volume {v0} -> {v1}");
    assert_eq!(r.report.self_intersections, 0);
}

#[test]
fn noisy_torus() {
    let (p, t) = torus(200, 100, 30.0, 8.0, 0.02);
    let r = decimate(&p, &t, &DecimateOpts { tol: 0.1, target_triangles: Some(4000), keep_angle_deg: None }).unwrap();
    eprintln!("torus {} -> {} tris: {:?}", t.len(), r.tris.len(), r.report);
    assert_closed_oriented(&r.tris, "torus");
    let (mx, _) = measure(&p, &t, &r.pts, &r.tris, 20000);
    assert!(mx <= 0.1, "true max {mx}");
    let (v0, v1) = (volume(&p, &t), volume(&r.pts, &r.tris));
    assert!(((v1 - v0) / v0).abs() < 0.03, "volume {v0} -> {v1}");
    assert!(r.tris.len() < t.len() / 4);
}

#[test]
fn blob_120k_deterministic_fast() {
    let (p, t) = blob(246, 246, 0.015);
    assert!(t.len() > 119_000, "{}", t.len());
    let o = DecimateOpts { tol: 0.1, target_triangles: Some(15_000), keep_angle_deg: None };
    let t0 = Instant::now();
    let r = decimate(&p, &t, &o).unwrap();
    let dt = t0.elapsed();
    eprintln!("blob {} -> {} tris in {:?}: {:?}", t.len(), r.tris.len(), dt, r.report);
    assert_closed_oriented(&r.tris, "blob");
    assert!(r.tris.len() <= 15_000);
    let (mx, rms) = measure(&p, &t, &r.pts, &r.tris, 30000);
    eprintln!("blob independent deviation max {mx:.4} rms {rms:.4}");
    assert!(mx <= 0.1, "true max {mx}");
    let (v0, v1) = (volume(&p, &t), volume(&r.pts, &r.tris));
    assert!(((v1 - v0) / v0).abs() < 0.02, "volume {v0} -> {v1}");
    assert_eq!(r.report.self_intersections, 0);
    if !cfg!(debug_assertions) {
        assert!(dt.as_secs_f64() < 30.0, "120k triangles took {dt:?}");
    }
    let r2 = decimate(&p, &t, &o).unwrap();
    assert_eq!(r.tris, r2.tris, "same input, same triangles");
    assert!(r.pts.iter().zip(&r2.pts).all(|(a, b)| a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits() && a.z.to_bits() == b.z.to_bits()), "same bits");
}

#[test]
fn box_keeps_edges() {
    let (p, t) = boxy(2.5);
    assert_closed_oriented(&t, "box input");
    let o = DecimateOpts { tol: 0.05, target_triangles: Some(12), keep_angle_deg: Some(30.0) };
    let r = decimate(&p, &t, &o).unwrap();
    eprintln!("box {} -> {} tris: {:?}", t.len(), r.tris.len(), r.report);
    assert_closed_oriented(&r.tris, "box");
    // the 8 corners survive exactly and nothing sticks out or in
    for c in [(0.0, 0.0, 0.0), (40.0, 0.0, 0.0), (0.0, 30.0, 0.0), (40.0, 30.0, 0.0), (0.0, 0.0, 20.0), (40.0, 0.0, 20.0), (0.0, 30.0, 20.0), (40.0, 30.0, 20.0)] {
        assert!(r.pts.iter().any(|q| q.dist(Vec3::new(c.0, c.1, c.2)) < 1e-9), "corner {c:?} lost");
    }
    let (mx, _) = measure(&p, &t, &r.pts, &r.tris, 20000);
    assert!(mx < 1e-6, "flat faces must stay flat: {mx}");
    assert!((volume(&r.pts, &r.tris) - 40.0 * 30.0 * 20.0).abs() < 1e-6);
    assert!(r.tris.len() < 60, "{}", r.tris.len());
}

#[test]
fn open_mesh_keeps_its_boundary() {
    let (p, mut t) = sphere(60, 30, 10.0, 0.0);
    // cut a cap: drop the triangles of the first pole
    t.retain(|k| !k.contains(&0));
    let r = decimate(&p, &t, &DecimateOpts { tol: 0.2, target_triangles: Some(200), keep_angle_deg: None }).unwrap();
    assert!(!r.report.watertight && r.report.boundary_edges == 60);
    let (mx, _) = measure(&p, &t, &r.pts, &r.tris, 5000);
    assert!(mx <= 0.2, "{mx}");
    // the boundary ring is still a closed loop of boundary edges
    let mut dir: HashMap<(u32, u32), u32> = HashMap::new();
    for k in &r.tris {
        for i in 0..3 {
            *dir.entry((k[i], k[(i + 1) % 3])).or_default() += 1;
        }
    }
    assert!(dir.values().all(|&n| n == 1));
    assert!(dir.keys().filter(|(a, b)| !dir.contains_key(&(*b, *a))).count() >= 3);
}

#[test]
fn refusals() {
    let (p, t) = sphere(20, 10, 5.0, 0.0);
    assert_eq!(decimate(&p, &t, &DecimateOpts::new(f64::NAN)).err().unwrap().code, "meshBadInput");
    assert_eq!(decimate(&p, &t, &DecimateOpts::new(-1.0)).err().unwrap().code, "meshBadInput");
    // three triangles on one edge
    let p2 = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
    let t2 = vec![[0, 1, 2], [0, 3, 1], [0, 1, 4]];
    assert_eq!(decimate(&p2, &t2, &DecimateOpts::new(0.1)).err().unwrap().code, "meshNonManifold");
}

#[test]
fn decimated_blob_converts() {
    let (p, t) = blob(246, 246, 0.015);
    let (pos, idx) = soup(&p, &t);
    // undecimated: refused as too large (a noisy surface keeps one face per triangle)
    let mut o = ck_mesh::Opts::default();
    o.analytic = false;
    o.deviation = false;
    let e = ck_mesh::mesh_to_solid(&pos, Some(&idx), &o).err().expect("refused");
    assert_eq!(e.code, "meshTooLarge");
    assert!(e.hint.contains("decimate"), "{}", e.hint);
    // with opts.decimate it converts under the default limits
    o.decimate = Some(DecimateOpts { tol: 0.1, target_triangles: Some(15_000), keep_angle_deg: None });
    let t0 = Instant::now();
    let c = ck_mesh::mesh_to_solid(&pos, Some(&idx), &o).ok().expect("converts");
    eprintln!("blob meshToSolid with decimate: {:?}, {} faces, issues {}", t0.elapsed(), c.body.num_faces(), c.report.issues);
    assert!(c.body.num_faces() <= 15_000);
    if c.report.issues != 0 {
        let is = ck_check::validate(&c.body, 1e-6);
        for i in is.iter().take(8) {
            eprintln!("ISSUE {} {} {}", i.code, i.entity, i.detail);
        }
    }
    assert_eq!(c.report.issues, 0);
    let dec = c.report.decimation.as_ref().expect("report");
    assert!(dec.dev_max <= 0.1 && dec.triangles <= 15_000);
    let rel = (c.report.body_volume - volume(&p, &t)).abs() / volume(&p, &t);
    assert!(rel < 0.02, "volume off by {rel}");
}

fn read_stl(path: &std::path::Path) -> (Vec<f64>, Vec<u32>) {
    let b = std::fs::read(path).unwrap();
    let n = u32::from_le_bytes([b[80], b[81], b[82], b[83]]) as usize;
    assert_eq!(b.len(), 84 + 50 * n, "binary STL expected");
    let mut pos = Vec::with_capacity(n * 9);
    for i in 0..n {
        let o = 84 + 50 * i + 12;
        for k in 0..9 {
            pos.push(f32::from_le_bytes([b[o + 4 * k], b[o + 4 * k + 1], b[o + 4 * k + 2], b[o + 4 * k + 3]]) as f64);
        }
    }
    (pos, (0..(n * 3) as u32).collect())
}

fn van_path() -> Option<std::path::PathBuf> {
    let p = std::env::var("CK_VAN_STL").map(std::path::PathBuf::from).ok().or_else(|| std::env::var("HOME").ok().map(|h| std::path::PathBuf::from(h).join("Downloads/interceptor/van_86mm_full_120k.stl")))?;
    p.exists().then_some(p)
}

/// The real scan of a toy van: 120k triangles, noisy, a tyre almost touching its wheel arch. Decimation at 0.05 and 0.1 mm
/// must stay manifold, free of crossings, within the bound, and convert.
#[test]
fn van_scan() {
    let Some(path) = van_path() else {
        eprintln!("van_scan: skipped, no scan file (set CK_VAN_STL to a binary STL)");
        return;
    };
    let (pos, idx) = read_stl(&path);
    for tol in [0.05, 0.1] {
        let mut o = ck_mesh::Opts::default();
        o.analytic = false;
        o.deviation = false;
        o.decimate = Some(DecimateOpts { tol, target_triangles: Some(12_000), keep_angle_deg: None });
        let t0 = Instant::now();
        let c = ck_mesh::mesh_to_solid(&pos, Some(&idx), &o);
        let dt = t0.elapsed();
        match c {
            Ok(c) => {
                let d = c.report.decimation.as_ref().unwrap();
                eprintln!("van tol {tol}: {} tris, max {:.4} rms {:.5}, crossings {} (input {}), meshToSolid {dt:?}, faces {}, issues {}, volume {:.2}", d.triangles, d.dev_max, d.dev_rms, d.self_intersections, d.input_self_intersections, c.body.num_faces(), c.report.issues, c.report.body_volume);
                assert!(d.dev_max <= tol);
                assert_eq!(c.report.issues, 0);
            }
            Err(e) => panic!("van tol {tol}: {} {}", e.code, e.message),
        }
    }
}

/// Probe (run with --ignored --nocapture): the van with no triangle target at two tolerances (the smallest mesh each
/// allows, its measured deviation, conversion time), and the undecimated mesh converted with the limits raised.
#[test]
#[ignore]
fn van_probe() {
    let Some(path) = van_path() else { return };
    let (pos, idx) = read_stl(&path);
    for tol in [0.05, 0.1] {
        let mut o = ck_mesh::Opts::default();
        o.decimate = Some(DecimateOpts::new(tol));
        let t0 = Instant::now();
        let c = ck_mesh::mesh_to_solid(&pos, Some(&idx), &o).ok().expect("converts");
        let d = c.report.decimation.as_ref().unwrap();
        eprintln!("PROBE tol {tol} no target, default opts (analytic on): {} tris (stopped by {}), max {:.4} rms {:.5}, crossings {}, whole meshToSolid {:?}, faces {}, issues {}, volume {:.2}, refused {:?}", d.triangles, d.stopped_by, d.dev_max, d.dev_rms, d.self_intersections, t0.elapsed(), c.body.num_faces(), c.report.issues, c.report.body_volume, (d.refused_topology, d.refused_flip, d.refused_deviation, d.refused_clearance));
    }
    let mut o = ck_mesh::Opts::default();
    o.max_faces = 200_000;
    o.close = Some(Default::default());
    let t0 = Instant::now();
    let c = ck_mesh::mesh_to_solid(&pos, Some(&idx), &o).ok().expect("converts");
    eprintln!("PROBE undecimated 120k, maxFaces 200000, close: {} faces, issues {}, volume {:.2}, {:?}", c.body.num_faces(), c.report.issues, c.report.body_volume, t0.elapsed());
}
