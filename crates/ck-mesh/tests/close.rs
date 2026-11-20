//! Mesh closing (`opts.close`): cracks welded, holes filled, and what cannot be closed refused with the reason.
//! The meshes are generated here (cube, sphere, torus, cylinder, a noisy scan-like sphere).
mod common;
use common::*;
use ck_geom::Vec3;
use ck_mesh::close::CloseOpts;
use ck_mesh::{mesh_to_solid, Converted, Opts, Refusal};
use std::collections::HashSet;
use std::f64::consts::PI;

type Mesh = (Vec<f64>, Vec<u32>);

fn closing(analytic: bool, co: CloseOpts) -> Opts {
    Opts { analytic, close: Some(co), ..Default::default() }
}

fn run(m: &Mesh, o: &Opts) -> Result<Converted, Refusal> {
    mesh_to_solid(&m.0, Some(&m.1), o)
}

fn ok(m: &Mesh, o: &Opts) -> Converted {
    match run(m, o) {
        Ok(c) => c,
        Err(r) => panic!("refused {} ({}): {} {:?}", r.code, r.hint, r.message, r.data),
    }
}

fn refused(m: &Mesh, o: &Opts) -> Refusal {
    match run(m, o) {
        Ok(_) => panic!("expected a refusal"),
        Err(r) => r,
    }
}

fn reason(r: &Refusal) -> String {
    r.data.iter().find(|(k, _)| *k == "reason").and_then(|(_, v)| v.as_str().map(|s| s.to_string())).unwrap_or_default()
}

/// Signed volume of an indexed mesh.
fn volume(m: &Mesh) -> f64 {
    let p = |i: u32| Vec3::new(m.0[3 * i as usize], m.0[3 * i as usize + 1], m.0[3 * i as usize + 2]);
    m.1.chunks(3).map(|t| p(t[0]).dot(p(t[1]).cross(p(t[2]))) / 6.0).sum()
}

fn without(m: &Mesh, drop: &HashSet<usize>) -> Mesh {
    let ix = m.1.chunks(3).enumerate().filter(|(i, _)| !drop.contains(i)).flat_map(|(_, t)| t.iter().copied()).collect();
    (m.0.clone(), ix)
}

fn tris_at(m: &Mesh, v: u32) -> Vec<usize> {
    m.1.chunks(3).enumerate().filter(|(_, t)| t.contains(&v)).map(|(i, _)| i).collect()
}

/// UV sphere: pole, `nlat - 1` rings of `nlon`, pole. Ring i (1-based) vertex j is `1 + (i - 1) * nlon + j`.
fn sphere(r: f64, nlat: usize, nlon: usize) -> Mesh {
    let mut p = vec![0.0, 0.0, r];
    for i in 1..nlat {
        let th = PI * i as f64 / nlat as f64;
        for j in 0..nlon {
            let ph = 2.0 * PI * j as f64 / nlon as f64;
            p.extend([r * th.sin() * ph.cos(), r * th.sin() * ph.sin(), r * th.cos()]);
        }
    }
    p.extend([0.0, 0.0, -r]);
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

fn torus(big: f64, small: f64, nu: usize, nv: usize) -> Mesh {
    let mut p = vec![];
    for i in 0..nu {
        for j in 0..nv {
            let (u, v) = (2.0 * PI * i as f64 / nu as f64, 2.0 * PI * j as f64 / nv as f64);
            let rr = big + small * v.cos();
            p.extend([rr * u.cos(), rr * u.sin(), small * v.sin()]);
        }
    }
    let at = |i: usize, j: usize| ((i % nu) * nv + j % nv) as u32;
    let mut ix = vec![];
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            ix.extend([a, b, c, a, c, d]);
        }
    }
    let mut m = (p, ix);
    if volume(&m) < 0.0 {
        m.1.chunks_mut(3).for_each(|t| t.swap(1, 2));
    }
    m
}

/// Cylinder along z, radius `r`, height `h`, `n` segments, without caps (rings of `n` vertices).
fn tube(r: f64, h: f64, n: usize) -> Mesh {
    let mut p = vec![];
    for z in [0.0, h] {
        for j in 0..n {
            let a = 2.0 * PI * j as f64 / n as f64;
            p.extend([r * a.cos(), r * a.sin(), z]);
        }
    }
    let mut ix = vec![];
    for j in 0..n {
        let (a, b, c, d) = (j as u32, ((j + 1) % n) as u32, (n + (j + 1) % n) as u32, (n + j) as u32);
        ix.extend([a, b, c, a, c, d]);
    }
    (p, ix)
}

/// Cube [0,10]^3 as 6 independent sheets (each its own 4 vertices), every vertex moved by `jitter` (deterministic).
fn sheets(jitter: f64) -> Mesh {
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    let mut q = vec![];
    let mut out = vec![];
    let mut s = 12345u64;
    let mut rnd = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0 * jitter
    };
    for t in ix.chunks(3) {
        for &v in t {
            q.extend([p[3 * v as usize] + rnd(), p[3 * v as usize + 1] + rnd(), p[3 * v as usize + 2] + rnd()]);
            out.push((q.len() / 3 - 1) as u32);
        }
    }
    (q, out)
}

fn cube_missing(face: usize) -> Mesh {
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    without(&(p, ix), &HashSet::from([2 * face, 2 * face + 1]))
}

fn assert_valid(c: &Converted) {
    assert_valid_tol(c, 1e-6);
}

/// Meshes with noise are valid at a tolerance above their noise.
fn assert_valid_tol(c: &Converted, tol: f64) {
    let issues = ck_check::validate(&c.body, tol);
    assert!(issues.is_empty(), "validate: {:?}", &issues[..issues.len().min(5)]);
}

// ---------------------------------------------------------------- default behaviour

#[test]
fn default_still_refuses_open_meshes() {
    let r = refused(&cube_missing(1), &opts_faceted());
    assert_eq!(r.code, "meshNotWatertight");
    assert_eq!(r.get_f64("openEdges"), Some(4.0));
    assert!(r.hint.contains("close"), "{}", r.hint);
    // and a closed mesh gives the same report with or without the option (no `closing` unless asked)
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    let a = mesh_to_solid(&p, Some(&ix), &opts_faceted()).unwrap();
    let b = mesh_to_solid(&p, Some(&ix), &closing(false, CloseOpts::default())).unwrap();
    assert!(a.report.closing.is_none());
    assert_eq!(b.report.closing.as_ref().unwrap().holes.len(), 0);
    assert_eq!(a.body.num_faces(), b.body.num_faces());
    assert_eq!(a.report.body_volume, b.report.body_volume);
}

// ---------------------------------------------------------------- holes

#[test]
fn cube_missing_a_face_gets_a_planar_cap() {
    for face in 0..6 {
        let c = ok(&cube_missing(face), &closing(false, CloseOpts::default()));
        assert_valid(&c);
        let cl = c.report.closing.as_ref().unwrap();
        assert_eq!(cl.holes.len(), 1);
        let h = &cl.holes[0];
        assert_eq!((h.edges, h.planar, h.triangles), (4, true, 2), "{h:?}");
        assert!((h.area - 100.0).abs() < 1e-9);
        assert_eq!(c.body.num_faces(), 6, "the cap merges with the planar region it closes");
        assert!((c.report.body_volume - 1000.0).abs() < 1e-6, "{}", c.report.body_volume);
    }
}

#[test]
fn sphere_one_triangle_and_three_holes() {
    let full = sphere(10.0, 24, 48);
    let vfull = volume(&full);
    // one triangle missing: the 3-edge hole is closed with exactly that triangle
    let m1 = without(&full, &HashSet::from([100]));
    let c = ok(&m1, &closing(false, CloseOpts::default()));
    assert_valid(&c);
    assert!((c.report.body_volume - vfull).abs() < 1e-8 * vfull);
    assert_eq!(c.report.closing.as_ref().unwrap().holes[0].edges, 3);
    // three holes: a triangle, a vertex fan (non-planar 8-gon), two quads
    let fan = tris_at(&full, 1 + 5 * 48 + 7);
    let mut drop: HashSet<usize> = fan.into_iter().collect();
    drop.insert(100);
    drop.extend([1000, 1001, 1002, 1003]);
    let m3 = without(&full, &drop);
    let c = ok(&m3, &closing(false, CloseOpts::default()));
    assert_valid(&c);
    let cl = c.report.closing.as_ref().unwrap();
    assert_eq!(cl.holes.len(), 3, "{:?}", cl.holes);
    assert!(cl.holes.iter().any(|h| !h.planar && h.method == "minArea"), "{:?}", cl.holes);
    assert!((c.report.body_volume - vfull).abs() < 5e-3 * vfull, "{} vs {vfull}", c.report.body_volume);
    assert_eq!(c.report.lumps, 1);
}

#[test]
fn torus_with_a_hole() {
    let full = torus(20.0, 6.0, 40, 20);
    let vfull = volume(&full);
    let m = without(&full, &HashSet::from([410, 411, 412, 413]));
    let c = ok(&m, &closing(false, CloseOpts::default()));
    assert_valid(&c);
    let cl = c.report.closing.as_ref().unwrap();
    assert_eq!(cl.holes.len(), 1);
    assert!((c.report.body_volume - vfull).abs() < 5e-3 * vfull, "{} vs {vfull} {:?}", c.report.body_volume, cl.holes);
    // a hole through the torus' tube is a hole of the mesh, not of the solid: still one lump with the donut's genus
    assert_eq!(c.report.lumps, 1);
}

#[test]
fn cylinder_missing_both_caps_stays_analytic() {
    let (r, h, n) = (8.0, 20.0, 64);
    let m = tube(r, h, n);
    let c = ok(&m, &closing(true, CloseOpts::default()));
    assert_valid(&c);
    let cl = c.report.closing.as_ref().unwrap();
    assert_eq!(cl.holes.len(), 2);
    assert!(cl.holes.iter().all(|h| h.planar && h.edges == n && h.method == "ear"), "{:?}", cl.holes);
    assert_eq!(kinds(&c.body), r#"{"cylinder":1,"plane":2}"#);
    // the mesh is inscribed: volume of the prism of n sides
    let prism = 0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h;
    assert!((c.report.mesh_volume - prism).abs() < 1e-6 * prism, "{} vs {prism}", c.report.mesh_volume);
    let exact = PI * r * r * h;
    assert!((c.report.body_volume - exact).abs() < 2e-3 * exact, "{} vs {exact}", c.report.body_volume);
}

// ---------------------------------------------------------------- gaps

#[test]
fn unwelded_sheets_with_cracks_are_welded() {
    let m = sheets(5e-5);
    // default: refused (the sheets are not welded), with the hint that names the option
    let r = refused(&m, &opts_faceted());
    assert_eq!(r.code, "meshNotWatertight");
    let c = ok(&m, &closing(false, CloseOpts { tol: Some(5e-4), ..Default::default() }));
    assert_valid_tol(&c, 1e-3);
    let cl = c.report.closing.as_ref().unwrap();
    assert_eq!(cl.holes.len(), 0, "welding closes it, nothing is invented: {cl:?}");
    // 36 corners of 12 triangles, 8 cube corners; the default weld (1e-6 of the diagonal) already took a pair
    assert!(cl.merged_vertices >= 26 && cl.merged_vertices <= 28, "{cl:?}");
    assert_eq!(c.body.num_faces(), 6);
    assert!((c.report.body_volume - 1000.0).abs() < 1.0, "{}", c.report.body_volume);
}

#[test]
fn t_junction_cracks_are_stitched() {
    // a cube whose top face is cut into 2 x 2 squares with its own midpoint vertices, the walls not: the midpoints of the
    // top's boundary lie on the walls' open edges, and all vertices are unwelded with 1e-4 offsets
    let s = 10.0;
    let (mut p, mut ix) = cube(Vec3::new(0.0, 0.0, 0.0), s);
    // drop the top face (quad 1: z = s) and build it as 4 squares
    let keep: Vec<u32> = ix.chunks(3).enumerate().filter(|(i, _)| *i != 2 && *i != 3).flat_map(|(_, t)| t.iter().copied()).collect();
    ix = keep;
    let n0 = p.len() / 3;
    let mut grid = vec![];
    for j in 0..3 {
        for i in 0..3 {
            p.extend([s * i as f64 / 2.0 + 1e-4 * ((i + j) % 2) as f64, s * j as f64 / 2.0 - 1e-4 * (i % 2) as f64, s + 1e-4]);
            grid.push((n0 + j * 3 + i) as u32);
        }
    }
    for j in 0..2 {
        for i in 0..2 {
            let (a, b, c, d) = (grid[j * 3 + i], grid[j * 3 + i + 1], grid[(j + 1) * 3 + i + 1], grid[(j + 1) * 3 + i]);
            ix.extend([a, b, c, a, c, d]);
        }
    }
    let m = (p, ix);
    assert_eq!(refused(&m, &opts_faceted()).code, "meshNotWatertight");
    let c = ok(&m, &closing(false, CloseOpts { tol: Some(1e-3), ..Default::default() }));
    assert_valid_tol(&c, 1e-3);
    let cl = c.report.closing.as_ref().unwrap();
    assert!(cl.stitched >= 4, "{cl:?}");
    assert_eq!(cl.holes.len(), 0);
    assert!((c.report.body_volume - 1000.0).abs() < 1.0, "{}", c.report.body_volume);
}

#[test]
fn gap_welding_that_would_make_edges_non_manifold_is_refused() {
    // a closed cube and a second closed cube 1e-4 away, both with an open top... the merge would join three sheets
    // along one edge: three sheets, one gap tolerance
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    let a = without(&(p.clone(), ix.clone()), &HashSet::from([2, 3]));
    let (p2, ix2) = cube(Vec3::new(0.0, 0.0, 1e-4), 10.0);
    let b = without(&(p2, ix2), &HashSet::from([2, 3]));
    let (p3, ix3) = cube(Vec3::new(0.0, 0.0, 2e-4), 10.0);
    let c3 = without(&(p3, ix3), &HashSet::from([2, 3]));
    let mut all = a;
    for m in [b, c3] {
        let off = (all.0.len() / 3) as u32;
        all.0.extend(&m.0);
        all.1.extend(m.1.iter().map(|i| i + off));
    }
    let r = refused(&all, &closing(false, CloseOpts { tol: Some(1e-3), ..Default::default() }));
    assert_eq!(r.code, "meshGapAmbiguous", "{}", r.message);
    assert_eq!(reason(&r), "weldNonManifold");
}

// ---------------------------------------------------------------- refusals

#[test]
fn hole_too_large_is_refused() {
    let full = sphere(10.0, 24, 48);
    // the northern hemisphere's rings 1..11 removed: a 48-edge hole, half the surface
    let drop: HashSet<usize> = full.1.chunks(3).enumerate().filter(|(_, t)| t.iter().all(|&v| v == 0 || (v as usize) <= 11 * 48 + 0)).map(|(i, _)| i).collect();
    let m = without(&full, &drop);
    let r = refused(&m, &closing(false, CloseOpts::default()));
    assert_eq!(r.code, "meshHoleTooLarge", "{}", r.message);
    assert_eq!(reason(&r), "area");
    assert!(!r.hint.is_empty());
    // the same hole is above an edge limit when that is set low, and fills when both limits are raised
    let r = refused(&m, &closing(false, CloseOpts { max_hole_edges: 20, max_hole_area: 10.0, ..Default::default() }));
    assert_eq!((r.code, reason(&r).as_str()), ("meshHoleTooLarge", "edges"));
    let c = ok(&m, &closing(false, CloseOpts { max_hole_area: 10.0, ..Default::default() }));
    assert_valid(&c);
}

#[test]
fn loop_that_crosses_itself_is_refused() {
    // an open box whose two top corners were swapped: the rim is a bow tie
    let (mut p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    for k in 0..3 {
        p.swap(3 * 4 + k, 3 * 5 + k);
    }
    let m = without(&(p, ix), &HashSet::from([2, 3]));
    let r = refused(&m, &closing(false, CloseOpts::default()));
    assert_eq!(r.code, "meshHoleSelfIntersects", "{}", r.message);
    assert_eq!(reason(&r), "loopCrossesItself");
    assert!(r.data.iter().any(|(k, _)| *k == "positions"));
}

#[test]
fn fill_that_cuts_the_mesh_is_refused() {
    // an open box and a closed post that stands through where the cap would be
    let m0 = cube_missing(1);
    let (p2, ix2) = cube(Vec3::new(6.0, 2.0, 2.0), 2.0);
    let (mut p, mut ix) = m0;
    let off = (p.len() / 3) as u32;
    // stretch the post's top vertices above the cap plane
    let mut p2 = p2;
    for k in 0..8 {
        if k & 4 != 0 {
            p2[3 * k + 2] += 12.0;
        }
    }
    p.extend(&p2);
    ix.extend(ix2.iter().map(|i| i + off));
    let r = refused(&(p, ix), &closing(false, CloseOpts::default()));
    assert_eq!(r.code, "meshHoleSelfIntersects", "{}", r.message);
    assert_eq!(reason(&r), "fillIntersectsMesh");
}

#[test]
fn holes_touching_at_a_vertex_are_ambiguous() {
    let full = sphere(10.0, 24, 48);
    let fan = tris_at(&full, 1 + 5 * 48 + 7);
    // two triangles of the fan that share only the vertex
    let common = |a: usize, b: usize| full.1[3 * a..3 * a + 3].iter().filter(|v| full.1[3 * b..3 * b + 3].contains(v)).count();
    let (a, b) = fan.iter().flat_map(|&a| fan.iter().map(move |&b| (a, b))).find(|&(a, b)| a != b && common(a, b) == 1).unwrap();
    let m = without(&full, &HashSet::from([a, b]));
    let r = refused(&m, &closing(false, CloseOpts::default()));
    assert_eq!(r.code, "meshGapAmbiguous", "{}", r.message);
    assert_eq!(reason(&r), "pinchedLoop");
}

#[test]
fn fill_false_only_welds() {
    let m = cube_missing(2);
    let r = refused(&m, &closing(false, CloseOpts { fill: false, ..Default::default() }));
    assert_eq!(r.code, "meshNotWatertight");
}

// ---------------------------------------------------------------- scan-like

#[test]
fn noisy_scan_with_a_hole() {
    let mut full = sphere(30.0, 40, 80);
    let mut s = 99u64;
    for v in full.0.iter_mut() {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        *v += ((s >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0 * 0.01;
    }
    let vfull = volume(&full);
    let mut drop: HashSet<usize> = HashSet::new();
    for dj in 0..4 {
        for di in 0..3 {
            drop.extend(tris_at(&full, 1 + (10 + di) * 80 + 20 + dj));
        }
    }
    let m = without(&full, &drop);
    let o = Opts { analytic: false, close: Some(CloseOpts::default()), ..Default::default() };
    let c = ok(&m, &o);
    assert_valid_tol(&c, 1e-2);
    let cl = c.report.closing.as_ref().unwrap();
    assert_eq!(cl.holes.len(), 1);
    assert!(!cl.holes[0].planar);
    assert!((c.report.body_volume - vfull).abs() < 5e-3 * vfull, "{} vs {vfull}", c.report.body_volume);
}

#[test]
fn hundred_thousand_triangles_with_a_hole() {
    let full = sphere(50.0, 224, 224);
    let n = full.1.len() / 3;
    let vfull = volume(&full);
    let mut drop: HashSet<usize> = HashSet::new();
    for dj in 0..5 {
        for di in 0..4 {
            drop.extend(tris_at(&full, 1 + (100 + di) * 224 + 50 + dj));
        }
    }
    let m = without(&full, &drop);
    let o = Opts { analytic: false, deviation: false, close: Some(CloseOpts::default()), ..Default::default() };
    let _warm = ok(&m, &o);
    let t = std::time::Instant::now();
    let c = ok(&m, &o);
    let dt = t.elapsed().as_secs_f64();
    // the stage alone, on the welded mesh
    let (mut pts, mut tris): (Vec<Vec3>, Vec<[u32; 3]>) = (m.0.chunks(3).map(|q| Vec3::new(q[0], q[1], q[2])).collect(), m.1.chunks(3).map(|t| [t[0], t[1], t[2]]).collect());
    let mut cr = ck_mesh::close::CloseReport::default();
    let t1 = std::time::Instant::now();
    ck_mesh::close::close(&mut pts, &mut tris, &CloseOpts::default(), 100.0, 20, &mut cr).unwrap();
    eprintln!("closing stage alone: {:.3} s", t1.elapsed().as_secs_f64());
    let t0 = std::time::Instant::now();
    let _ = mesh_to_solid(&full.0, Some(&full.1), &Opts { analytic: false, deviation: false, ..Default::default() }).unwrap();
    eprintln!("{n} triangles, hole of {} removed: with closing {:.2} s, closed mesh without the stage {:.2} s", drop.len(), dt, t0.elapsed().as_secs_f64());
    assert_valid(&c);
    assert_eq!(c.report.closing.as_ref().unwrap().holes.len(), 1);
    assert!((c.report.body_volume - vfull).abs() < 5e-3 * vfull);
    assert!(dt < 60.0);
}
