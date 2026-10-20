//! Orientation option: SketchUp is Z up, the kit is Y up. `Up::Y` maps (x, y, z) -> (x, z, -y), `Up::YFlip` (-x, -z, -y).

use ck_skp::body::{build_body, BodyOptions};
use ck_skp::{bake, parse, BakeOptions, Limits, Scene, Up};
use std::path::PathBuf;

fn load(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(name)).unwrap()
}

fn scene(name: &str, up: Up) -> Scene {
    let m = parse(&load(name)).unwrap();
    bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, up, ..Default::default() }).unwrap()
}

/// Signed volume by the divergence theorem over all triangles (meaningful only for closed parts; rotation keeps it).
fn signed_volume(s: &Scene) -> f64 {
    let mut v = 0.0;
    for p in &s.primitives {
        for t in p.indices.chunks_exact(3) {
            let (a, b, c) = (p.positions[t[0] as usize], p.positions[t[1] as usize], p.positions[t[2] as usize]);
            v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
        }
    }
    v
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() <= 1e-6 * (1.0 + a.abs().max(b.abs())), "{what}: {a} vs {b}");
}

const FILES: [&str; 4] = ["chair_and_table.skp", "gondola_v20.skp", "capilla_quiroz_v17.skp", "holes.skp"];

#[test]
fn matrices_are_proper_rotations_with_the_documented_mapping() {
    assert_eq!(Up::Y.apply([1.0, 2.0, 3.0]), [1.0, 3.0, -2.0]);
    assert_eq!(Up::YFlip.apply([1.0, 2.0, 3.0]), [-1.0, -3.0, -2.0]);
    assert_eq!(Up::Z.apply([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
    for up in [Up::Z, Up::Y, Up::YFlip] {
        let m = up.matrix();
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        close(det, 1.0, "determinant");
        assert_eq!(Up::parse(up.name()), Some(up));
    }
    assert_eq!(Up::parse("x"), None);
}

#[test]
fn rotated_scene_has_the_same_geometry_in_new_axes() {
    for name in FILES {
        let z = scene(name, Up::Z);
        for up in [Up::Y, Up::YFlip] {
            let r = scene(name, up);
            assert_eq!(z.polygons.len(), r.polygons.len(), "{name}");
            assert_eq!(z.triangle_count(), r.triangle_count(), "{name}");
            close(z.area(), r.area(), "area");
            close(signed_volume(&z), signed_volume(&r), "volume");
            // every vertex and normal is the mapped one, order preserved
            for (a, b) in z.polygons.iter().zip(&r.polygons) {
                assert_eq!(a.loops.len(), b.loops.len());
                for (la, lb) in a.loops.iter().zip(&b.loops) {
                    for (p, q) in la.iter().zip(lb) {
                        let e = up.apply(*p);
                        assert!((0..3).all(|k| (e[k] - q[k]).abs() < 1e-9), "{name} vertex");
                    }
                }
                let e = up.apply(a.normal);
                assert!((0..3).all(|k| (e[k] - b.normal[k]).abs() < 1e-9), "{name} normal");
            }
            for (a, b) in z.primitives.iter().zip(&r.primitives) {
                assert_eq!(a.indices, b.indices);
                for (n, m) in a.normals.iter().zip(&b.normals) {
                    let e = up.apply([n[0] as f64, n[1] as f64, n[2] as f64]);
                    assert!((0..3).all(|k| (e[k] - m[k] as f64).abs() < 1e-5), "{name} primitive normal");
                }
            }
        }
    }
}

#[test]
fn bbox_axes_swap() {
    for name in FILES {
        let (zl, zh) = scene(name, Up::Z).bbox().unwrap();
        let (yl, yh) = scene(name, Up::Y).bbox().unwrap();
        // (x, y, z) -> (x, z, -y)
        let want_lo = [zl[0], zl[2], -zh[1]];
        let want_hi = [zh[0], zh[2], -zl[1]];
        for k in 0..3 {
            close(yl[k], want_lo[k], &format!("{name} lo {k}"));
            close(yh[k], want_hi[k], &format!("{name} hi {k}"));
        }
    }
}

#[test]
fn body_keeps_names_counts_and_areas_and_has_the_rotated_bbox() {
    for name in FILES {
        let zs = scene(name, Up::Z);
        let zb = build_body(&zs, &BodyOptions::default()).unwrap();
        for up in [Up::Y, Up::YFlip] {
            let rs = scene(name, up);
            let rb = build_body(&rs, &BodyOptions::default()).unwrap();
            assert_eq!(zb.face_ids, rb.face_ids, "{name}: face names");
            assert_eq!(zb.face_materials, rb.face_materials);
            assert_eq!(zb.body.num_faces(), rb.body.num_faces());
            assert_eq!(zb.body.num_lumps(), rb.body.num_lumps());
            assert_eq!((zb.open_edges, zb.non_manifold_edges, zb.skipped_faces), (rb.open_edges, rb.non_manifold_edges, rb.skipped_faces), "{name}");
            let (lo, hi) = rb.body.bbox();
            let (zlo, zhi) = zb.body.bbox();
            let corners: Vec<[f64; 3]> = (0..8).map(|i| [if i & 1 == 0 { zlo.x } else { zhi.x }, if i & 2 == 0 { zlo.y } else { zhi.y }, if i & 4 == 0 { zlo.z } else { zhi.z }]).map(|c| up.apply(c)).collect();
            for k in 0..3 {
                let (mn, mx) = corners.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, c| (a.0.min(c[k]), a.1.max(c[k])));
                let (l, h) = ([lo.x, lo.y, lo.z][k], [hi.x, hi.y, hi.z][k]);
                assert!((l - mn).abs() < 1e-3 && (h - mx).abs() < 1e-3, "{name} {up:?} axis {k}: {l} {h} vs {mn} {mx}");
            }
        }
    }
}

/// Footprint (extent along the two ground axes) of the vertices in the lowest / highest 5 % of `up_axis`.
fn band_footprints(s: &Scene, up_axis: usize) -> (f64, f64) {
    let (lo, hi) = s.bbox().unwrap();
    let span = hi[up_axis] - lo[up_axis];
    let g: Vec<usize> = (0..3).filter(|&k| k != up_axis).collect();
    let mut b = [[(f64::MAX, f64::MIN); 2]; 2];
    for p in &s.primitives {
        for v in &p.positions {
            let band = if v[up_axis] < lo[up_axis] + 0.05 * span { 0 } else if v[up_axis] > hi[up_axis] - 0.05 * span { 1 } else { continue };
            for (i, &k) in g.iter().enumerate() {
                b[band][i] = (b[band][i].0.min(v[k]), b[band][i].1.max(v[k]));
            }
        }
    }
    let area = |x: [(f64, f64); 2]| (x[0].1 - x[0].0) * (x[1].1 - x[1].0);
    (area(b[0]), area(b[1]))
}

#[test]
fn chair_and_table_stands_with_its_legs_towards_minus_y() {
    // in the file the legs touch the ground (wide footprint at the bottom); the top is the thin chair back
    let (zl, zh) = band_footprints(&scene("chair_and_table.skp", Up::Z), 2);
    assert!(zl > 10.0 * zh, "premise: file is Z up, bottom footprint {zl}, top {zh}");
    let (yl, yh) = band_footprints(&scene("chair_and_table.skp", Up::Y), 1);
    close(yl, zl, "bottom footprint");
    close(yh, zh, "top footprint");
    let (fl, fh) = band_footprints(&scene("chair_and_table.skp", Up::YFlip), 1);
    assert!(fh > 10.0 * fl, "Up::YFlip is upside down relative to Up::Y: low {fl} high {fh}");
}
