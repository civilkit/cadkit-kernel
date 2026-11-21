//! Times of a conversion near the limit of 200000 triangles, analytic refit included (run by hand, release-like profile):
//! `CK_MESH_PERF=1 cargo-queue test --profile fast -p ck-mesh --test perf_200k -- --nocapture --test-threads=1`.
//! Two meshes: a fine UV sphere (one sphere to fit) and a plate with 100 drilled holes (100 cylinders to fit, planar faces
//! with 100 inner loops). Prints the times; the asserts only pin that the result is right.
mod common;
use common::*;
use std::f64::consts::PI;
use std::time::Instant;

fn uv_sphere(r: f64, nlat: usize, nlon: usize) -> (Vec<f64>, Vec<u32>) {
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

fn time(name: &str, p: &[f64], ix: Option<&[u32]>, o: &ck_mesh::Opts) -> ck_mesh::Converted {
    let t = Instant::now();
    let c = ck_mesh::mesh_to_solid(p, ix, o).unwrap_or_else(|r| panic!("{name}: {} {}", r.code, r.message));
    let s = t.elapsed().as_secs_f64();
    let a = c.report.analytic.as_ref();
    eprintln!(
        "{name}: {} triangles, {} faces, {:.2} s (analytic {}, cylinders {}, spheres {}, faceted {}, deviation {}, units {}, budget exhausted {})",
        c.report.input_triangles,
        c.body.num_faces(),
        s,
        o.analytic,
        a.map_or(0, |a| a.cylinders),
        a.map_or(0, |a| a.spheres),
        a.map_or(0, |a| a.faceted),
        o.deviation,
        a.map_or(0, |a| a.work_units),
        a.is_some_and(|a| a.budget_exhausted)
    );
    c
}

#[test]
fn fine_sphere_200k() {
    if std::env::var("CK_MESH_PERF").is_err() {
        return; // by hand: CK_MESH_PERF=1 (the CI box: --env CK_MESH_PERF=1)
    }
    let (p, ix) = uv_sphere(25.0, 224, 446);
    assert!(ix.len() / 3 <= 200_000, "{}", ix.len() / 3);
    let c = time("sphere, analytic", &p, Some(&ix), &ck_mesh::Opts::default());
    assert_eq!(c.body.num_faces(), 1);
    time("sphere, faceted only", &p, Some(&ix), &ck_mesh::Opts { analytic: false, deviation: false, max_faces: 250_000, ..Default::default() });
}

#[test]
fn plate_with_100_holes_200k() {
    if std::env::var("CK_MESH_PERF").is_err() {
        return; // by hand: CK_MESH_PERF=1 (the CI box: --env CK_MESH_PERF=1)
    }
    let mut pts = vec![];
    let mut ents = vec![];
    for i in 0..10 {
        for j in 0..10 {
            let k = i * 10 + j;
            pts.push(format!(r#"{{"id":"k{k}","x":{},"y":{}}}"#, -67.5 + 15.0 * i as f64, -67.5 + 15.0 * j as f64));
            ents.push(format!(r#"{{"id":"pt{k}","type":"point","p":"k{k}"}}"#));
        }
    }
    let plate = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-80,"y":-80},{"id":"p2","x":80,"y":-80},{"id":"p3","x":80,"y":80},{"id":"p4","x":-80,"y":80}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
    let holes = format!(
        r#"{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}}"#,
        pts.join(","),
        ents.join(",")
    );
    let b = body_of(&format!("{plate},{holes}"));
    assert_eq!(b.num_faces(), 6 + 100);
    // the finest chord whose mesh stays within the limit
    let mut best = None;
    for f in [0.02, 0.01, 0.005, 0.0025, 0.00125, 0.0009, 0.0006] {
        let s = soup(&b, f);
        let n = s.len() / 9;
        eprintln!("chord x {f}: {n} triangles");
        if n > 200_000 {
            break;
        }
        best = Some(s);
    }
    let s = best.expect("a mesh within the limit");
    let c = time("plate with 100 holes, analytic", &s, None, &ck_mesh::Opts::default());
    assert_eq!(c.report.analytic.as_ref().unwrap().cylinders, 100);
    time("plate with 100 holes, faceted only", &s, None, &ck_mesh::Opts { analytic: false, deviation: false, max_faces: 250_000, ..Default::default() });
}
