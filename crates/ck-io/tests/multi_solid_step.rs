//! Multi-solid STEP import and the faces the BenchCAD ground-truth files
//! exercise: a sweep face closed on itself (B-spline patch with a seam), a
//! torus fragment bounded by a seam on one side and a real edge on the other.
use ck_io::step::read_step;

fn gt(name: &str) -> String {
    std::fs::read_to_string(format!("{}/../../tools/bench/benchcad/gt/{name}.step", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

#[test]
fn torsion_spring_two_solids_import_with_full_volume() {
    let r = read_step(&gt("torsion_spring_000054")).unwrap();
    assert_eq!(r.solids, 2);
    assert_eq!(r.body.num_lumps(), 2);
    let mp = ck_topo::mass::mass_props(&r.body);
    assert!((mp.volume - 180176.517).abs() / 180176.517 < 1e-4, "volume {}", mp.volume);
    assert!((mp.area - 102528.72).abs() / 102528.72 < 1e-4, "area {}", mp.area);
    let lumps = ck_topo::mass::lump_moments(&r.body);
    assert!(lumps.iter().all(|m| m.volume > 0.0));
    assert!(ck_check::validate(&r.body, 1e-6).is_empty());
    // the closed sweep face tessellates (bbox reaches the coil radius)
    let t = ck_tess::mesh::tessellate(&r.body);
    let (lo, hi) = t.mesh.bbox().unwrap();
    assert!(lo.x < -59.0 && hi.x > 59.0, "{lo:?} {hi:?}");
}

#[test]
#[ignore = "known: the torus eye of this part is dropped from the display mesh (tessellation holes), tracked separately"]
fn eyebolt_torus_fragments_tessellate() {
    let r = read_step(&gt("eyebolt_000265")).unwrap();
    let t = ck_tess::mesh::tessellate(&r.body);
    let (lo, hi) = t.mesh.bbox().unwrap();
    // the torus eye spans x = +-63
    assert!(lo.x < -62.9 && hi.x > 62.9, "{lo:?} {hi:?}");
    assert!(t.faces.iter().all(|f| f.is_some()), "a face failed to tessellate");
}

/// Deviation of the ground-truth coil faces from the ideal screw tube
/// (circle r in the meridian plane swept by the helix): diagnostic.
#[test]
#[ignore]
fn torsion_spring_gt_coil_vs_ideal_tube() {
    use ck_geom::surface::Surface;
    let r = read_step(&gt("torsion_spring_000054")).unwrap();
    let (big_r, rad, k) = (56.25f64, 3.52f64, 17.24 / std::f64::consts::TAU);
    let (mut smin, mut smax) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut worst = 0.0f64;
    let mut sum = 0.0;
    let mut n = 0;
    for f in r.body.face_ids() {
        let Surface::BSpline(bs) = &r.body.face(f).surface else { continue };
        let ((u0, u1), (v0, v1)) = bs.domain();
        for i in 0..=400 {
            for j in 0..=24 {
                let p = bs.eval(u0 + (u1 - u0) * i as f64 / 400.0, v0 + (v1 - v0) * j as f64 / 24.0);
                let rho = p.x.hypot(p.y);
                let alpha = p.y.atan2(p.x);
                let mut best = f64::INFINITY;
                let mut sg = 0.0;
                for m in -2..16 {
                    let phi = alpha + std::f64::consts::TAU * m as f64;
                    let s = (rho - big_r).hypot(p.z - k * phi);
                    if (s - rad).abs() < best { best = (s - rad).abs(); sg = s - rad; }
                }
                worst = worst.max(best);
                sum += best;
                smin = smin.min(sg);
                smax = smax.max(sg);
                n += 1;
            }
        }
    }
    eprintln!("GT coil faces: max |dist to ideal tube| {worst:.5}, mean {:.5}, signed [{smin:.5}, {smax:.5}] over {n} samples", sum / n as f64);
}

/// coil_spring ground truth: one closed rational degree-6 sweep face (976 knot spans along the path,
/// self-overlapping coils: pitch 6 < wire 6.94) + 4 planes. Import, mass, validate and tessellation used
/// to cost 16 + 11 + 13 + 38 s (every closest-point query scanned all spans, closest points across the
/// closing isoline were wrong, the refinement re-tested every triangle each pass).
#[test]
fn coil_spring_gt_imports_and_tessellates_fast() {
    let t = std::time::Instant::now();
    let r = read_step(&gt("coil_spring_000026")).unwrap();
    assert_eq!(r.solids, 1);
    assert_eq!(r.body.face_ids().count(), 5);
    let mp = ck_topo::mass::mass_props(&r.body);
    // dataset volume 89283.67, area 52788.04 (the face integral double-counts the coils' overlap, as OCC does)
    assert!((mp.volume - 89283.67).abs() / 89283.67 < 1e-3, "volume {}", mp.volume);
    assert!((mp.area - 52788.04).abs() / 52788.04 < 1e-3, "area {}", mp.area);
    assert!(ck_check::validate(&r.body, 1e-6).is_empty());
    let tm = ck_tess::mesh::tessellate(&r.body);
    assert!(tm.mesh.failed.is_empty());
    assert!(tm.mesh.tri_face.len() > 30_000, "{} triangles", tm.mesh.tri_face.len());
    assert!(t.elapsed().as_secs_f64() < 120.0, "{:.1} s", t.elapsed().as_secs_f64());
}

/// Timing diagnostic: `cargo test -p ck-io --profile fast --test multi_solid_step coil_spring_gt_probe -- --ignored --nocapture`.
#[test]
#[ignore]
fn coil_spring_gt_probe() {
    let t = std::time::Instant::now();
    let r = read_step(&gt("coil_spring_000026")).unwrap();
    eprintln!("read_step {:.2}s solids {} faces {}", t.elapsed().as_secs_f64(), r.solids, r.body.face_ids().count());
    let t = std::time::Instant::now();
    let mp = ck_topo::mass::mass_props(&r.body);
    eprintln!("mass_props {:.2}s volume {} area {}", t.elapsed().as_secs_f64(), mp.volume, mp.area);
    let t = std::time::Instant::now();
    let v = ck_check::validate(&r.body, 1e-6);
    eprintln!("validate {:.2}s {} issues {:?}", t.elapsed().as_secs_f64(), v.len(), v.iter().take(3).collect::<Vec<_>>());
    let t = std::time::Instant::now();
    let tm = ck_tess::mesh::tessellate(&r.body);
    eprintln!("tessellate {:.2}s tris {}", t.elapsed().as_secs_f64(), tm.mesh.tri_face.len());
}

/// closest-point recovery on the GT coil face: random surface points must project back onto themselves.
#[test]
#[ignore]
fn coil_spring_closest_point_recovery() {
    use ck_geom::surface::Surface;
    let r = read_step(&gt("coil_spring_000026")).unwrap();
    let bs = r.body.face_ids().find_map(|f| match &r.body.face(f).surface { Surface::BSpline(b) => Some(b.clone()), _ => None }).unwrap();
    let ((u0, u1), (v0, v1)) = bs.domain();
    eprintln!("deg {} {} nu {} nv {} domain u {u0}..{u1} v {v0}..{v1} rational {}", bs.deg_u, bs.deg_v, bs.nu, bs.nv, bs.weights.is_some());
    let mut rng = 12345u64;
    let mut next = || { rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); (rng >> 11) as f64 / (1u64 << 53) as f64 };
    let (mut bad, mut worst) = (0, 0.0f64);
    let t = std::time::Instant::now();
    for _ in 0..300 {
        let (u, v) = (u0 + (u1 - u0) * next(), v0 + (v1 - v0) * next());
        let p = bs.eval(u, v);
        let (cu, cv) = bs.closest_param(p);
        let d = bs.eval(cu, cv).dist(p);
        worst = worst.max(d);
        if d > 1e-7 { bad += 1; eprintln!("miss target uv ({u:.4},{v:.3}) got ({cu:.4},{cv:.3}) d {d:.3e}"); }
    }
    eprintln!("closest_param on surface points: {bad}/300 missed (>1e-7), worst {worst:.3e}, {:.3}s", t.elapsed().as_secs_f64());
}

/// Centre line of the ground-truth coil: radius of the section-circle centres against the path parameter.
#[test]
#[ignore]
fn coil_spring_gt_centerline() {
    use ck_geom::surface::Surface;
    let r = read_step(&gt("coil_spring_000026")).unwrap();
    let bs = r.body.face_ids().find_map(|f| match &r.body.face(f).surface { Surface::BSpline(b) => Some(b.clone()), _ => None }).unwrap();
    let ((u0, u1), (v0, v1)) = bs.domain();
    let (mut rmin, mut rmax, mut sum, mut n) = (f64::INFINITY, 0.0f64, 0.0, 0);
    let mut rows = vec![];
    for k in 0..=2400 {
        let v = v0 + (v1 - v0) * k as f64 / 2400.0;
        let c = (0..8).map(|i| bs.eval(u0 + (u1 - u0) * i as f64 / 8.0, v)).fold(ck_geom::Vec3::ZERO, |a, p| a + p) * (1.0 / 8.0);
        let rho = c.x.hypot(c.y);
        rmin = rmin.min(rho);
        rmax = rmax.max(rho);
        sum += rho;
        n += 1;
        rows.push((v, rho, c.z));
    }
    eprintln!("GT centre-line radius: min {rmin:.4} max {rmax:.4} mean {:.4}; wire radius {:.4}", sum / n as f64, {
        let p = bs.eval(u0, v0);
        let c = (0..8).map(|i| bs.eval(u0 + (u1 - u0) * i as f64 / 8.0, v0)).fold(ck_geom::Vec3::ZERO, |a, p| a + p) * (1.0 / 8.0);
        p.dist(c)
    });
    for k in (0..rows.len()).step_by(60) {
        eprintln!("  v {:.1} rho {:.4} z {:.3}", rows[k].0, rows[k].1, rows[k].2);
    }
}

/// Mesh area, triangle count and degenerate-triangle count of a tessellation.
fn mesh_stats(m: &ck_tess::mesh::Mesh) -> (f64, usize) {
    let (mut area, mut tiny) = (0.0, 0);
    for i in 0..m.tri_face.len() {
        let p = |k: usize| ck_geom::Vec3::new(m.positions[i * 9 + k * 3] as f64, m.positions[i * 9 + k * 3 + 1] as f64, m.positions[i * 9 + k * 3 + 2] as f64);
        let (e0, e1, e2) = ((p(1) - p(0)).len(), (p(2) - p(1)).len(), (p(0) - p(2)).len());
        area += 0.5 * (p(1) - p(0)).cross(p(2) - p(0)).len();
        if e0.min(e1).min(e2) < 1e-3 {
            tiny += 1;
        }
    }
    (area, tiny)
}

/// The coil sweep face is a 12-turn closed rational B-spline tube (u around the wire, v along the path). Its mesh
/// used to cover only the two ends (the long seam-spanning triangles in the middle were judged flat and accepted
/// as is: 44% of the area missing), carried ~47k sliver triangles where the grazing end cuts lie 0.15 off the
/// surface, and took 100k triangles. Timing-insensitive checks: full area coverage, a bounded triangle count, no slivers.
#[test]
fn coil_spring_gt_mesh_covers_face_with_bounded_triangles() {
    let r = read_step(&gt("coil_spring_000026")).unwrap();
    let mp = ck_topo::mass::mass_props(&r.body);
    let tm = ck_tess::mesh::tessellate(&r.body);
    assert!(tm.mesh.failed.is_empty());
    let (area, tiny) = mesh_stats(&tm.mesh);
    // dataset area 52788.04; the polygon is inscribed so slightly smaller, but not by a missing half
    assert!(area > 0.99 * mp.area && area < 1.001 * mp.area, "mesh area {area} vs {}", mp.area);
    let n = tm.mesh.tri_face.len();
    assert!((30_000..80_000).contains(&n), "{n} triangles");
    assert!(tiny < 100, "{tiny} sliver triangles");
}

/// Same check on the two-solid torsion spring (swept coil faces + legs): the mesh covers the exact area.
#[test]
fn torsion_spring_gt_mesh_covers_area() {
    let r = read_step(&gt("torsion_spring_000054")).unwrap();
    let mp = ck_topo::mass::mass_props(&r.body);
    let tm = ck_tess::mesh::tessellate(&r.body);
    let (area, tiny) = mesh_stats(&tm.mesh);
    eprintln!("torsion mesh area {area} vs {} tris {} tiny {tiny}", mp.area, tm.mesh.tri_face.len());
    assert!(area > 0.99 * mp.area && area < 1.001 * mp.area, "mesh area {area} vs {}", mp.area);
    assert!(tiny < 100, "{tiny} sliver triangles");
}
