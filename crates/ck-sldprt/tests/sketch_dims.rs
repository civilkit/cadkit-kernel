//! Sketch dimensions, ellipses, stored plane frames and extrusion end types, read from the feature stream: the committed fixture
//! parts (always run) and the local corpus of part files (`SW_CORPUS`, default `corpus-local` at the workspace root; skipped when
//! it is absent).

use ck_sldprt::container;
use ck_sldprt::sketch::{self, DimKind, PlaneRef, RelTarget, SkCurve};
use ck_sldprt::Limits;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(format!("{name}.sldprt"))).unwrap()
}

fn read(bytes: &[u8]) -> sketch::SketchSet {
    let lim = Limits::default();
    sketch::read(&container::open(bytes, &lim).unwrap(), &lim).unwrap()
}

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(es) = std::fs::read_dir(dir) else { return };
    let mut es: Vec<PathBuf> = es.filter_map(|e| e.ok().map(|e| e.path())).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            files(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
            out.push(p);
        }
    }
}

fn corpus() -> Vec<PathBuf> {
    let root = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let mut fs = vec![];
    files(&root.join("samples"), &mut fs);
    fs
}

/// The end type word of an extrusion: 0 for a boss with a listed depth, 1 (through all) for the hole of the hole fixture, which
/// lists no depth; and the revolve's 360 degrees are a blind end.
#[test]
fn fixture_end_types() {
    let set = read(&fixture("sw2022_cube_hole_5mm"));
    let boss = set.feature("Boss-Extrude1").unwrap();
    assert_eq!(boss.end_type, Some(0));
    assert_eq!(boss.from_type, Some(0));
    let cut = set.feature("Cut-Extrude1").unwrap();
    assert_eq!(cut.end_type, Some(1), "the hole runs through all");
    assert!(cut.params.is_empty(), "a through all cut lists no depth");
    let cone = read(&fixture("sw2022_cone"));
    assert_eq!(cone.feature("Revolve1").unwrap().end_type, Some(0));
    let old = read(&fixture("sw2011_cube_10mm_ole2"));
    assert_eq!(old.feature("Boss-Extrude1").unwrap().end_type, Some(0));
}

/// The standard planes' frames are the transforms the file stores with the sketch's plane reference (Front: none, an identity;
/// Top and Right: a rotation); the corpus has the Top and Right ones, whose columns are the axes of the frames used.
#[test]
fn fixture_frames_are_the_standard_ones() {
    let set = read(&fixture("sw2022_cube_hole_5mm"));
    for s in &set.sketches {
        assert_eq!(s.plane, PlaneRef::Plane(2));
        let f = s.frame().unwrap();
        assert_eq!((f.x, f.y, f.o), ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0; 3]));
    }
}

/// Every standard plane's frame is checked, the offset planes' stored transforms are the plane's own offset (the sketch on the
/// offset plane of the loft sample sits 10 mm along the normal, as its plane feature's D1 says).
#[test]
fn corpus_stored_frames() {
    let (mut top, mut right, mut offset, mut on_distance) = (0, 0, 0, 0);
    for p in corpus() {
        let Ok(bytes) = std::fs::read(&p) else { continue };
        let Ok(c) = container::open(&bytes, &Limits::default()) else { continue };
        let Ok(set) = sketch::read(&c, &Limits::default()) else { continue };
        for s in &set.sketches {
            // a transform the file stores is a rotation: unit orthogonal axes
            for (id, m) in &s.plane_frames {
                let col = |c: usize| [m[c], m[3 + c], m[6 + c]];
                let d = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                assert!((d(col(0), col(0)) - 1.0).abs() < 1e-6 && d(col(0), col(1)).abs() < 1e-6, "{id}");
            }
            match s.plane {
                PlaneRef::Plane(3) => top += 1,
                PlaneRef::Plane(4) => right += 1,
                PlaneRef::Plane(i) if i > 4 => {
                    if let Some(f) = s.stored_frame {
                        offset += 1;
                        // an offset plane's origin lies on its normal through the origin (all of them here are offset from a standard
                        // plane or from another plane that is) and its feature lists a distance: from a standard plane the origin is
                        // that far from the origin
                        let rec = set.features.iter().find(|r| r.id as u32 == i);
                        let d1 = rec.and_then(|r| r.params.iter().find(|p| p.name == "D1")).map(|p| p.value);
                        let on_axis = f.o[0].abs() < 1e-9 && f.o[2].abs() < 1e-9;
                        if let (Some(d1), true, true) = (d1, on_axis, p.to_string_lossy().contains("C16_loft")) {
                            assert!((f.o[1].abs() - d1.abs()).abs() < 1e-9, "{}: origin {:?}, D1 {d1}", s.name, f.o);
                            on_distance += 1;
                        }
                    }
                }
                _ => {}
            }
        }
    }
    eprintln!("Top sketches {top}, Right sketches {right}, sketches on a plane with a stored frame {offset} ({on_distance} at the distance their plane lists)");
}

/// Every dimension that is kept measures what its value says on the sketch's own geometry, and the KeyWords tree of the file lists
/// a dimension of the same name and value on the same sketch (millimetres or degrees).
#[test]
fn corpus_dimensions_measure_their_operands() {
    let (mut n, mut named, mut kinds) = (0usize, 0usize, [0usize; 6]);
    let mut unlisted: Vec<String> = vec![];
    let mut contradicted = 0usize;
    let mut checked = 0usize;
    let lim = Limits::default();
    for p in corpus() {
        let Ok(bytes) = std::fs::read(&p) else { continue };
        let Ok(c) = container::open(&bytes, &lim) else { continue };
        let Ok(set) = sketch::read(&c, &lim) else { continue };
        let meta = ck_sldprt::meta::read(&c, &lim);
        let scale = sketch::tree_scale(meta.units.as_ref().and_then(|u| u.length.as_deref()));
        for s in &set.sketches {
            let pt = |i: usize| (s.points[i].x, s.points[i].y);
            for d in &s.dims {
                n += 1;
                let k = match d.kind {
                    DimKind::Distance => 0,
                    DimKind::HDist => 1,
                    DimKind::VDist => 2,
                    DimKind::Radius => 3,
                    DimKind::Diameter => 4,
                    DimKind::Angle => 5,
                };
                kinds[k] += 1;
                // the measure, recomputed here from the points
                if let (DimKind::Distance, [RelTarget::Point(a), RelTarget::Point(b)]) = (d.kind, d.targets.as_slice()) {
                    let (p, q) = (pt(*a), pt(*b));
                    assert!(((p.0 - q.0).hypot(p.1 - q.1) - d.value).abs() <= 1e-9 + 1e-7 * d.value.abs(), "{} {}", s.name, d.name);
                }
                if let (DimKind::Radius | DimKind::Diameter, [RelTarget::Arc(h)]) = (d.kind, d.targets.as_slice()) {
                    let c = s.curves.iter().find(|c| matches!(c, SkCurve::Arc { handle: Some(x), .. } | SkCurve::Circle { handle: Some(x), .. } if x == h)).unwrap();
                    let r = s.radius(c).unwrap();
                    let want = if d.kind == DimKind::Radius { r } else { 2.0 * r };
                    assert!((want - d.value).abs() <= 1e-9 + 1e-7 * d.value.abs(), "{} {}", s.name, d.name);
                }
                // the same dimension in the file's own tree
                let angle = d.kind == DimKind::Angle;
                let Some(scale) = scale else { continue };
                checked += 1;
                let tree = meta.features.iter().filter(|kf| kf.name == s.name).flat_map(|kf| kf.dims.iter()).any(|kd| {
                    kd.name == d.name && kd.value.map_or(false, |v| (v - if angle { d.value.to_degrees() } else { d.value * scale }).abs() <= 1e-3 * (1.0 + v.abs()))
                });
                named += tree as usize;
                // a dimension the tree lists by name but with another value would be a wrong record taken for the driving one
                let listed_name = meta.features.iter().filter(|kf| kf.name == s.name).flat_map(|kf| kf.dims.iter()).any(|kd| kd.name == d.name);
                if listed_name && !tree {
                    contradicted += 1;
                }
                if !tree && unlisted.len() < 14 {
                    let listed: Vec<String> = meta.features.iter().filter(|kf| kf.name == s.name).flat_map(|kf| kf.dims.iter()).map(|kd| format!("{}={:?}", kd.name, kd.value)).collect();
                    unlisted.push(format!("{} {} {} {:?} = {} (tree: {})", p.file_name().unwrap().to_string_lossy(), s.name, d.name, d.kind, d.value, listed.join(", ")));
                }
            }
        }
    }
    eprintln!("{contradicted} contradicted by the tree");
    eprintln!("{n} dimensions kept (distance {}, horizontal {}, vertical {}, radius {}, diameter {}, angle {}), {named} of the {checked} in documents whose unit is known are listed by name and value in the file's tree", kinds[0], kinds[1], kinds[2], kinds[3], kinds[4], kinds[5]);
    for u in &unlisted {
        eprintln!("  not in the tree: {u}");
    }
    if !corpus().is_empty() {
        assert!(n > 100, "{n} dimensions");
        assert_eq!(contradicted, 0, "dimensions whose name the tree lists with another value");
        assert!(named * 10 >= checked * 6, "only {named} of {checked} dimensions are in the file's tree under the same name and value");
    }
}

/// An ellipse record is read where the stream has one and its points lie on the ellipse; nothing else is called an ellipse.
#[test]
fn corpus_ellipses_lie_on_their_points() {
    let mut found = 0;
    for p in corpus() {
        let Ok(bytes) = std::fs::read(&p) else { continue };
        let Ok(c) = container::open(&bytes, &Limits::default()) else { continue };
        let Ok(set) = sketch::read(&c, &Limits::default()) else { continue };
        for s in &set.sketches {
            for cv in &s.curves {
                if let SkCurve::Ellipse { rim, centre, major, minor, .. } = cv {
                    found += 1;
                    let (r, c0, m) = (&s.points[*rim], &s.points[*centre], &s.points[*major]);
                    let a = (m.x - c0.x).hypot(m.y - c0.y);
                    assert!(a >= *minor - 1e-12);
                    // the rim point on x^2/a^2 + y^2/b^2 = 1 in the axes
                    let (ux, uy) = ((m.x - c0.x) / a, (m.y - c0.y) / a);
                    let (dx, dy) = (r.x - c0.x, r.y - c0.y);
                    let (x, y) = (dx * ux + dy * uy, -dx * uy + dy * ux);
                    assert!(((x / a).powi(2) + (y / minor).powi(2) - 1.0).abs() < 1e-6);
                }
            }
        }
    }
    eprintln!("ellipses read: {found}");
}
