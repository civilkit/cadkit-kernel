//! Kernel body import: planar faces with holes, welded vertices, shared edges; the body tessellates with the kernel mesher.

use ck_skp::body::{build_body, BodyOptions};
use ck_skp::{bake, parse, BakeOptions, Limits};
use std::path::PathBuf;

fn load(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)).unwrap()
}

fn body_of(name: &str) -> (ck_skp::body::SkpBody, ck_skp::Scene) {
    let m = parse(&load(name)).unwrap();
    let s = bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, ..Default::default() }).unwrap();
    let b = build_body(&s, &BodyOptions::default()).unwrap_or_else(|e| panic!("{name}: {e}"));
    (b, s)
}

fn poly_area(l: &[[f64; 3]]) -> f64 {
    let mut a = [0.0; 3];
    for i in 0..l.len() {
        let (p, q) = (l[i], l[(i + 1) % l.len()]);
        a[0] += p[1] * q[2] - p[2] * q[1];
        a[1] += p[2] * q[0] - p[0] * q[2];
        a[2] += p[0] * q[1] - p[1] * q[0];
    }
    0.5 * (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

#[test]
fn face_with_hole_tessellates_to_the_exact_area() {
    let (b, s) = body_of("holes.skp");
    assert_eq!(b.body.num_faces(), 1);
    let f = b.body.face_ids().next().unwrap();
    assert_eq!(b.body.face(f).loops.len(), 2, "outer loop and one hole");
    let expect = poly_area(&s.polygons[0].loops[0]) - poly_area(&s.polygons[0].loops[1]);
    let tess = ck_tess::mesh::tessellate(&b.body);
    let m = &tess.mesh;
    let mut area = 0.0;
    for t in m.positions.chunks_exact(9) {
        let (a, b2, c) = ([t[0] as f64, t[1] as f64, t[2] as f64], [t[3] as f64, t[4] as f64, t[5] as f64], [t[6] as f64, t[7] as f64, t[8] as f64]);
        let (u, v) = ([b2[0] - a[0], b2[1] - a[1], b2[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let cr = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        area += 0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
    }
    assert!((area - expect).abs() < 1e-3 * expect, "tessellated {area} vs {expect}");
}

#[test]
fn shared_edges_are_shared() {
    let (b, _) = body_of("multi_face_shared_edge.skp");
    assert_eq!(b.body.num_faces(), 2);
    // two squares sharing one edge: 6 vertices, 7 edges
    assert_eq!(b.body.vertex_ids().count(), 6);
    assert_eq!(b.body.edge_ids().count(), 7);
    assert_eq!(b.open_edges, 6);
    assert_eq!(b.non_manifold_edges, 0);
}

#[test]
fn all_fixtures_with_faces_become_bodies_that_tessellate() {
    for name in ["chair_and_table.skp", "capilla_quiroz_v17.skp", "SU_File.skp", "legacy_v3_synthetic.skp", "circle.skp", "Untitled.skp", "gondola_v20.skp"] {
        let (b, s) = body_of(name);
        assert_eq!(b.face_ids.len() + b.skipped_faces, s.polygons.len(), "{name}");
        assert_eq!(b.face_ids.len(), b.body.num_faces(), "{name}");
        assert!(b.face_materials.iter().all(|&m| m < b.materials.len()), "{name}");
        // bounding boxes agree (the polygon corners are the mesh corners)
        let (lo, hi) = b.body.bbox();
        let (slo, shi) = s.bbox().unwrap();
        for k in 0..3 {
            let (l, h) = ([lo.x, lo.y, lo.z][k], [hi.x, hi.y, hi.z][k]);
            assert!((l - slo[k]).abs() < 1e-3 && (h - shi[k]).abs() < 1e-3, "{name}: bbox axis {k}: {l} {h} vs {} {}", slo[k], shi[k]);
        }
        eprintln!("{name}: {} faces built, tessellating", b.body.num_faces());
        let t0 = std::time::Instant::now();
        let tess = ck_tess::mesh::tessellate(&b.body);
        eprintln!("{name}: tessellated in {:?}", t0.elapsed());
        let mut per = vec![0usize; b.body.num_faces()];
        for &f in &tess.mesh.tri_face {
            if let Some(c) = per.get_mut(f as usize) {
                *c += 1;
            }
        }
        let empty = per.iter().filter(|&&c| c == 0).count();
        assert!(empty * 50 <= per.len(), "{name}: {empty} of {} faces have no triangles", per.len());
        println!("{name}: {} faces, {} lumps, {} open edges, {} non-manifold, {} triangles", b.body.num_faces(), b.body.num_lumps(), b.open_edges, b.non_manifold_edges, tess.mesh.tri_face.len());
    }
}

#[test]
fn refuses_too_many_faces_and_empty_scenes() {
    let m = parse(&load("gondola_v20.skp")).unwrap();
    let s = bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, ..Default::default() }).unwrap();
    let e = build_body(&s, &BodyOptions { max_faces: 10, ..Default::default() }).unwrap_err();
    assert_eq!(e.code, "limit.faces");
    let m = parse(&load("blank_v17.skp")).unwrap();
    let s = bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, ..Default::default() }).unwrap();
    assert_eq!(build_body(&s, &BodyOptions::default()).unwrap_err().code, "body.empty");
}

/// Regression: a face whose stored normal is not the normal of its corners (what an instance under a non-uniform scale used
/// to produce) made the plane miss the corners, and the kernel mesher refined each quad into thousands of triangles
/// (75 s for 864 faces of a real file). The plane now follows the polygon; the budget is generous for a debug build.
#[test]
fn faces_with_a_tilted_stored_normal_stay_cheap_to_tessellate() {
    use ck_skp::scene::{PolyFace, Scene, SceneMaterial};
    let mut s = Scene::default();
    s.materials.push(SceneMaterial { rgb: [200, 200, 200], opacity: 1.0, double_sided: false, texture: None });
    let n_faces = 200;
    for i in 0..n_faces {
        let (x, y) = ((i % 20) as f64 * 100.0, (i / 20) as f64 * 400.0);
        // a 60 x 250 quad tilted about the x axis; the stored normal is 5.4 degrees off the true one
        let (c, sn) = (0.4793_f64.hypot(0.8786).recip() * 0.8786, 0.4793_f64.hypot(0.8786).recip() * 0.4793);
        let quad = vec![[x, y, 0.0], [x + 60.0, y, 0.0], [x + 60.0, y + 250.0 * c, 250.0 * sn], [x, y + 250.0 * c, 250.0 * sn]];
        let a = 5.4_f64.to_radians();
        let (ct, st) = (a.cos(), a.sin());
        // true normal (0, -sn, c) tilted by `a` about x
        let normal = [0.0, -sn * ct - c * st, c * ct - sn * st];
        s.polygons.push(PolyFace { loops: vec![quad], normal, material: 0, layer: String::new(), path: String::new() });
    }
    let b = build_body(&s, &BodyOptions::default()).unwrap();
    assert_eq!(b.body.num_faces(), n_faces);
    let t0 = std::time::Instant::now();
    let tess = ck_tess::mesh::tessellate(&b.body);
    let dt = t0.elapsed();
    assert_eq!(tess.mesh.tri_face.len(), 2 * n_faces, "one quad is two triangles");
    assert!(dt.as_secs_f64() < 5.0, "tessellation took {dt:?}");
}

/// The cofactor transform keeps normals perpendicular to the transformed plane under a non-uniform scale.
#[test]
fn transform_normal_stays_perpendicular_under_non_uniform_scale() {
    use ck_skp::scene::{transform_normal, transform_point};
    let m = [2.0, 0.3, 0.0, 0.0, 0.5, 0.1, 0.2, 0.0, 3.0, 5.0, -2.0, 1.0, 1.0];
    let (a, b, c) = ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 1.0]);
    let (ta, tb, tc) = (transform_point(&m, a), transform_point(&m, b), transform_point(&m, c));
    let (u, v) = ([tb[0] - ta[0], tb[1] - ta[1], tb[2] - ta[2]], [tc[0] - ta[0], tc[1] - ta[1], tc[2] - ta[2]]);
    // (0,1,-1) is the normal of the plane through a, b, c
    let n = transform_normal(&m, [0.0, 1.0, -1.0]);
    let d = |p: [f64; 3], q: [f64; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    assert!(d(n, u).abs() < 1e-12 && d(n, v).abs() < 1e-12, "{n:?}");
}

/// The three files that took 10 to 75 s before the fix (licensed, so not committed): `SKP_PERF_FILES` is a list of paths
/// separated by `:`; each must import (parse, bake, body, tessellate) in under 2 s.
#[test]
fn slow_corpus_files_import_fast() {
    let Ok(list) = std::env::var("SKP_PERF_FILES") else { return };
    for f in list.split(':').filter(|f| !f.is_empty()) {
        let data = std::fs::read(f).unwrap();
        let t0 = std::time::Instant::now();
        let m = parse(&data).unwrap();
        let s = bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, ..Default::default() }).unwrap();
        let b = build_body(&s, &BodyOptions::default()).unwrap();
        let _ = ck_tess::mesh::tessellate(&b.body);
        let dt = t0.elapsed().as_secs_f64();
        assert!(dt < 2.0, "{f}: {dt:.2}s");
    }
}
