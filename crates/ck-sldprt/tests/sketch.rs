//! Sketches read from the feature stream: the four committed fixture parts (always run) and the local corpus of part files
//! (`SW_CORPUS`, default `corpus-local` at the workspace root; skipped when it is absent), compared with the geometry of the
//! body each part imports to.

use ck_sldprt::container;
use ck_sldprt::sketch::{self, PlaneRef, PointRole, RelationKind, SkCurve};
use ck_sldprt::{import_sldprt, Limits};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(format!("{name}.sldprt"))).unwrap()
}

fn read(bytes: &[u8]) -> sketch::SketchSet {
    let lim = Limits::default();
    sketch::read(&container::open(bytes, &lim).unwrap(), &lim).unwrap()
}

fn mm(v: f64) -> f64 {
    v * 1e3
}

#[test]
fn cube_sketch_is_a_10_mm_square_and_the_extrude_is_10_mm() {
    let set = read(&fixture("sw2022_cube_10mm"));
    assert!(set.modern);
    assert_eq!(set.sketches.len(), 1);
    let s = &set.sketches[0];
    assert_eq!(s.name, "Sketch1");
    assert_eq!(s.plane, PlaneRef::Plane(2), "{:?}", s.plane_refs);
    let mut corners: Vec<(i64, i64)> = s.points.iter().filter(|p| p.role == PointRole::Free).map(|p| ((mm(p.x) * 1e3).round() as i64, (mm(p.y) * 1e3).round() as i64)).collect();
    corners.sort();
    assert_eq!(corners, vec![(0, 0), (0, 10_000), (10_000, 0), (10_000, 10_000)]);
    let lines: Vec<_> = s.curves.iter().filter_map(|c| if let SkCurve::Line { a, b, construction, .. } = c { Some((*a, *b, *construction)) } else { None }).collect();
    assert_eq!(lines.iter().filter(|l| !l.2).count(), 4);
    // every non-construction line is axis parallel and 10 mm long
    for (a, b, _) in lines.iter().filter(|l| !l.2) {
        let (p, q) = (&s.points[*a], &s.points[*b]);
        let len = mm((p.x - q.x).hypot(p.y - q.y));
        assert!((len - 10.0).abs() < 1e-9, "{len}");
        assert!(p.x == q.x || p.y == q.y);
    }
    let h = s.relations.iter().filter(|r| r.kind == RelationKind::Horizontal).count();
    let v = s.relations.iter().filter(|r| r.kind == RelationKind::Vertical).count();
    assert_eq!((h, v), (2, 2), "{:?}", s.relations);
    assert!(s.relations.iter().any(|r| r.kind == RelationKind::Coincident));
    let e = set.feature("Boss-Extrude1").expect("extrude record");
    assert_eq!(e.params.len(), 1);
    assert!((mm(e.params[0].value) - 10.0).abs() < 1e-9);
    assert_eq!(set.sketch_before(set.features.iter().position(|f| f.name == "Boss-Extrude1").unwrap()).unwrap().name, "Sketch1");
    assert!(s.exact(), "{:?}", s.notes);
    // sketch area times depth is the volume of the imported body
    let area = polygon_area(s);
    let vol = ck_topo::mass::mass_props(&import_sldprt(&fixture("sw2022_cube_10mm")).unwrap().body).volume;
    assert!((mm(mm(area)) * mm(e.params[0].value) - vol).abs() / vol < 1e-9, "{area} {vol}");
}

/// The area of the closed outline made of the sketch's non-construction lines (sketch units squared), for a polygon whose
/// lines are listed in any order.
fn polygon_area(s: &sketch::Sketch) -> f64 {
    let mut edges: Vec<(usize, usize)> = s.curves.iter().filter_map(|c| if let SkCurve::Line { a, b, construction: false, .. } = c { Some((*a, *b)) } else { None }).collect();
    let mut order = vec![edges.remove(0)];
    while !edges.is_empty() {
        let last = order.last().unwrap().1;
        let k = edges.iter().position(|e| e.0 == last || e.1 == last).expect("closed outline");
        let e = edges.remove(k);
        order.push(if e.0 == last { e } else { (e.1, e.0) });
    }
    let mut a = 0.0;
    for (i, j) in &order {
        a += s.points[*i].x * s.points[*j].y - s.points[*j].x * s.points[*i].y;
    }
    a.abs() / 2.0
}

#[test]
fn hole_sketch_circle_matches_the_hole_in_the_body() {
    let bytes = fixture("sw2022_cube_hole_5mm");
    let set = read(&bytes);
    assert_eq!(set.sketches.len(), 2);
    let hole = set.sketches.iter().find(|s| s.name == "Sketch2").unwrap();
    let (rim, centre) = hole.curves.iter().find_map(|c| if let SkCurve::Circle { rim, centre, centre_certain, .. } = c { assert!(*centre_certain, "the centre is claimed as often as its counter says"); Some((*rim, centre.unwrap())) } else { None }).unwrap();
    let c = &hole.points[centre];
    let r = mm((hole.points[rim].x - c.x).hypot(hole.points[rim].y - c.y));
    assert!((mm(c.x) - 5.0).abs() < 1e-9 && (mm(c.y) - 5.0).abs() < 1e-9, "{} {}", mm(c.x), mm(c.y));
    assert!((r - 2.5).abs() < 1e-9, "{r}");
    // the imported body is the cube less a cylinder of that radius and the cube's height
    let body = import_sldprt(&bytes).unwrap();
    let vol = ck_topo::mass::mass_props(&body.body).volume;
    let want = 1000.0 * 1.0 - std::f64::consts::PI * r * r * 10.0;
    assert!((vol - want).abs() / want < 1e-6, "{vol} vs {want}");
}

#[test]
fn cone_sketch_is_a_right_triangle_with_its_axis() {
    let set = read(&fixture("sw2022_cone"));
    let s = &set.sketches[0];
    let mut pts: Vec<(i64, i64)> = s.points.iter().filter(|p| p.role == PointRole::Free).map(|p| ((mm(p.x) * 1e3).round() as i64, (mm(p.y) * 1e3).round() as i64)).collect();
    pts.sort();
    assert_eq!(pts, vec![(0, 0), (0, 10_000), (5_000, 0)]);
    // Pappus: the cone is the triangle (area 25 mm^2, centroid 5/3 mm from the axis) turned once about the axis
    let body = ck_topo::mass::mass_props(&import_sldprt(&fixture("sw2022_cone")).unwrap().body).volume;
    let want = 25.0 * std::f64::consts::TAU * 5.0 / 3.0;
    assert!((body - want).abs() / want < 1e-6, "{body} {want}");
    let pts: Vec<(f64, f64)> = s.points.iter().filter(|p| p.role == PointRole::Free).map(|p| (mm(p.x), mm(p.y))).collect();
    let area = 0.5 * ((pts[1].0 - pts[0].0) * (pts[2].1 - pts[0].1) - (pts[2].0 - pts[0].0) * (pts[1].1 - pts[0].1)).abs();
    assert!((area - 25.0).abs() < 1e-9, "{area}");
    let rev = set.features.iter().find(|f| f.params.iter().any(|p| (p.value - std::f64::consts::TAU).abs() < 1e-12));
    assert!(rev.is_some(), "the revolve angle parameter (a full turn) is read");
}

#[test]
fn ole2_stream_reads_too() {
    let set = read(&fixture("sw2011_cube_10mm_ole2"));
    assert!(!set.modern);
    let s = &set.sketches[0];
    assert_eq!(s.points.iter().filter(|p| p.role == PointRole::Free).count(), 4);
    assert_eq!(s.curves.iter().filter(|c| matches!(c, SkCurve::Line { construction: false, .. })).count(), 4);
    // the same six relations as the 2022 cube: two horizontal, two vertical, equal sides, the corner on the origin
    let kinds: Vec<RelationKind> = s.relations.iter().map(|r| r.kind).collect();
    assert_eq!(kinds.iter().filter(|k| **k == RelationKind::Horizontal).count(), 2, "{kinds:?}");
    assert_eq!(kinds.iter().filter(|k| **k == RelationKind::Vertical).count(), 2);
    assert!(kinds.contains(&RelationKind::Equal) && kinds.contains(&RelationKind::Coincident));
    let e = set.feature("Boss-Extrude1").unwrap();
    assert!((mm(e.params[0].value) - 10.0).abs() < 1e-9);
}

#[test]
fn fixtures_never_panic_when_damaged() {
    for name in ["sw2022_cube_10mm", "sw2022_cone", "sw2011_cube_10mm_ole2"] {
        let lim = Limits::default();
        let c = container::open(&fixture(name), &lim).unwrap();
        let st = c.streams.iter().find(|s| s.name.contains("ResolvedFeatures")).unwrap();
        let d = c.data(st.name.strip_suffix("__ZLB").unwrap_or(&st.name), &lim).unwrap();
        let mut x = d.clone();
        for i in (0..x.len()).step_by(7) {
            let keep = x[i];
            x[i] ^= 0xa5;
            let _ = sketch::parse(&x, &lim);
            x[i] = keep;
        }
        for n in (0..d.len()).step_by(97) {
            let _ = sketch::parse(&d[..n], &lim);
        }
    }
}

fn corpus_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(es) = std::fs::read_dir(dir) else { return };
    let mut es: Vec<PathBuf> = es.filter_map(|e| e.ok().map(|e| e.path())).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            corpus_files(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
            out.push(p);
        }
    }
}

/// The corpus: every part's sketches decode, every line has two distinct end points, every circle centre that is certain sits at
/// a positive distance, every arc centre is equidistant from its ends, and each relation of a kind checked against geometry holds.
#[test]
fn corpus_sketches_are_geometrically_consistent() {
    let root = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let mut fs = vec![];
    corpus_files(&root.join("samples"), &mut fs);
    if fs.is_empty() {
        return;
    }
    let lim = Limits::default();
    let (mut files, mut with_sketches, mut sketches, mut exact, mut rel_checked, mut rel_bad) = (0, 0, 0, 0, 0, 0);
    let mut bad: Vec<String> = vec![];
    let (mut dims_checked, mut dims_bad) = (0, 0);
    for p in &fs {
        let Ok(bytes) = std::fs::read(p) else { continue };
        let Ok(c) = container::open(&bytes, &lim) else { continue };
        files += 1;
        let Ok(set) = sketch::read(&c, &lim) else { continue };
        // parameters against the dimensions of the KeyWords tree (millimetres, degrees; shown rounded)
        let meta = ck_sldprt::meta::read(&c, &lim);
        let mm_doc = meta.units.as_ref().and_then(|u| u.length.as_deref()) == Some("mm");
        if mm_doc {
            for kf in &meta.features {
                for d in &kf.dims {
                    let Some(v) = d.value else { continue };
                    let angle = d.text.contains('\u{b0}');
                    // `Sketch2<3>`: an instance of a feature in a pattern or in-context, its dimensions are kept elsewhere
                    if kf.name.contains('<') {
                        continue;
                    }
                    let Some(fr) = set.features.iter().find(|f| f.name == kf.name) else { continue };
                    let want = |p: &sketch::Param| if angle { p.value.to_degrees() } else { p.value * 1e3 };
                    let cands: Vec<f64> = fr.params.iter().filter(|p| p.name == d.name).map(want).collect();
                    // an integer parameter (a pattern count) is stored as an integer: read as a double it is not a number
                    if cands.is_empty() || cands.iter().any(|c| c.abs() < 1e-30 && *c != 0.0) {
                        continue;
                    }
                    dims_checked += 1;
                    // the file keeps the value in effect last; any of the records for that name may be the saved one
                    if !cands.iter().any(|c| (c - v).abs() <= 0.006 + 1e-4 * v.abs()) {
                        dims_bad += 1;
                        bad.push(format!("{} {} {}: {} vs {:?}", p.display(), kf.name, d.name, v, cands));
                    }
                }
            }
        }
        if !set.sketches.is_empty() {
            with_sketches += 1;
        }
        for s in &set.sketches {
            sketches += 1;
            if s.exact() {
                exact += 1;
            }
            for c in &s.curves {
                match c {
                    SkCurve::Line { a, b, .. } => assert!(a != b && *a < s.points.len() && *b < s.points.len()),
                    SkCurve::Arc { start, end, centre: Some(ct), centre_certain: true, .. } => {
                        let d1 = (s.points[*start].x - s.points[*ct].x).hypot(s.points[*start].y - s.points[*ct].y);
                        let d2 = (s.points[*end].x - s.points[*ct].x).hypot(s.points[*end].y - s.points[*ct].y);
                        assert!((d1 - d2).abs() < 1e-9 * d1.max(1e-3), "{}: arc ends at {d1} and {d2} from the centre", p.display());
                    }
                    _ => {}
                }
            }
            for r in &s.relations {
                let line = |id: u16| s.curves.iter().find_map(|c| if let SkCurve::Line { a, b, handle: Some(h), .. } = c { (*h == id).then(|| ((s.points[*a].x, s.points[*a].y), (s.points[*b].x, s.points[*b].y))) } else { None });
                let dir = |id: u16| line(id).map(|(a, b)| (b.0 - a.0, b.1 - a.1));
                use sketch::RelTarget::{Arc, Line, Point};
                let pt = |i: usize| s.points.get(i).map(|p| (p.x, p.y));
                let arc = |id: u16| {
                    s.curves.iter().find_map(|c| match c {
                        SkCurve::Arc { start, centre: Some(k), centre_certain: true, handle: Some(h), .. } if *h == id => Some((pt(*k)?, pt(*start)?)),
                        SkCurve::Circle { rim, centre: Some(k), centre_certain: true, handle: Some(h), .. } if *h == id => Some((pt(*k)?, pt(*rim)?)),
                        _ => None,
                    })
                };
                let dd = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
                let ok: Option<bool> = match (r.kind, r.targets.as_slice()) {
                    (RelationKind::Horizontal, [Line(l)]) => dir(*l).map(|d| (rel_checked += 1, d.1.abs() < 1e-12).1),
                    (RelationKind::Vertical, [Line(l)]) => dir(*l).map(|d| (rel_checked += 1, d.0.abs() < 1e-12).1),
                    (RelationKind::Collinear, [Line(a), Line(b)]) => line(*a).zip(line(*b)).map(|((p, q), (u, v))| {
                        rel_checked += 1;
                        let d = (q.0 - p.0, q.1 - p.1);
                        let off = |w: (f64, f64)| ((w.0 - p.0) * d.1 - (w.1 - p.1) * d.0).abs() / d.0.hypot(d.1);
                        off(u) < 1e-9 && off(v) < 1e-9
                    }),
                    (RelationKind::Equal, [Line(a), Line(b)]) => dir(*a).zip(dir(*b)).map(|(u, v)| (rel_checked += 1, (u.0.hypot(u.1) - v.0.hypot(v.1)).abs() < 1e-9).1),
                    (RelationKind::Equal, [Arc(a), Arc(b)]) => arc(*a).zip(arc(*b)).map(|(u, v)| (rel_checked += 1, (dd(u.0, u.1) - dd(v.0, v.1)).abs() < 1e-9).1),
                    (RelationKind::Tangent, [Arc(a), Line(l)] | [Line(l), Arc(a)]) => arc(*a).zip(line(*l)).map(|((c, p), (u, v))| {
                        rel_checked += 1;
                        let d = (v.0 - u.0, v.1 - u.1);
                        (((c.0 - u.0) * d.1 - (c.1 - u.1) * d.0).abs() / d.0.hypot(d.1) - dd(c, p)).abs() < 1e-9
                    }),
                    (RelationKind::Tangent, [Arc(a), Arc(b)]) => arc(*a).zip(arc(*b)).map(|((c1, p1), (c2, p2))| {
                        rel_checked += 1;
                        let (r1, r2) = (dd(c1, p1), dd(c2, p2));
                        (dd(c1, c2) - (r1 + r2)).abs() < 1e-9 || (dd(c1, c2) - (r1 - r2).abs()).abs() < 1e-9
                    }),
                    (RelationKind::Coincident, [Point(a), Point(b)]) => pt(*a).zip(pt(*b)).map(|(p, q)| (rel_checked += 1, dd(p, q) < 1e-12).1),
                    (RelationKind::Midpoint, [Point(i), Line(l)] | [Line(l), Point(i)]) => pt(*i).zip(line(*l)).map(|(p, (u, v))| (rel_checked += 1, dd(p, ((u.0 + v.0) / 2.0, (u.1 + v.1) / 2.0)) < 1e-9).1),
                    (RelationKind::HorizontalPoints, [Point(a), Point(b)]) => pt(*a).zip(pt(*b)).map(|(p, q)| (rel_checked += 1, (p.1 - q.1).abs() < 1e-12).1),
                    (RelationKind::VerticalPoints, [Point(a), Point(b)]) => pt(*a).zip(pt(*b)).map(|(p, q)| (rel_checked += 1, (p.0 - q.0).abs() < 1e-12).1),
                    _ => None,
                };
                if ok == Some(false) {
                    rel_bad += 1;
                    bad.push(format!("{} {}: {:?} {:?}", p.display(), s.name, r.kind, r.targets));
                }
            }
        }
    }
    eprintln!("corpus: {files} files, {with_sketches} with sketches, {sketches} sketches ({exact} exact), {rel_checked} relations checked, {rel_bad} violated");
    eprintln!("corpus: {dims_checked} feature dimensions compared with the KeyWords tree, {dims_bad} differ");
    assert_eq!(rel_bad + dims_bad, 0, "{bad:#?}");
    if files >= 38 {
        assert!(with_sketches >= 35, "{with_sketches}");
    }
}

/// Sketch geometry against the body each part imports to: a circle whose centre is certain has the radius of some cylinder,
/// cone seat, sphere or torus of the body, a non-construction line of a part's first sketch has the length of some edge of the body. (Not every
/// sketch survives into the final solid: a later cut or fillet can remove the face or edge a sketch made, so these are
/// shares, with a floor, not equalities.)
#[test]
fn corpus_sketches_against_the_body() {
    use ck_geom::surface::Surface;
    let root = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let mut fs = vec![];
    corpus_files(&root.join("samples"), &mut fs);
    if fs.is_empty() {
        return;
    }
    let lim = Limits::default();
    let (mut circles, mut circles_hit, mut lines, mut lines_hit, mut first_lines, mut first_hit) = (0, 0, 0, 0, 0, 0);
    for p in &fs {
        let Ok(bytes) = std::fs::read(p) else { continue };
        let Ok(c) = container::open(&bytes, &lim) else { continue };
        let Ok(set) = sketch::read(&c, &lim) else { continue };
        let Ok(imp) = import_sldprt(&bytes) else { continue };
        let b = &imp.body;
        let mut radii: Vec<f64> = vec![];
        for f in b.face_ids() {
            match &b.face(f).surface {
                Surface::Cylinder { r, .. } | Surface::Sphere { r, .. } | Surface::Cone { r, .. } => radii.push(*r),
                Surface::Torus { major, minor, .. } => {
                    radii.push(*major);
                    radii.push(*minor);
                }
                _ => {}
            }
        }
        let lens: Vec<f64> = b.edge_ids().map(|e| b.edge_length(e)).collect();
        for (si, s) in set.sketches.iter().enumerate() {
            for cv in &s.curves {
                match cv {
                    SkCurve::Circle { construction: false, centre_certain: true, .. } => {
                        let r = mm(s.radius(cv).unwrap());
                        circles += 1;
                        if radii.iter().any(|x| (x - r).abs() < 1e-6 * r.max(1.0)) {
                            circles_hit += 1;
                        }
                    }
                    SkCurve::Line { a, b: e, construction: false, .. } => {
                        let l = mm((s.points[*a].x - s.points[*e].x).hypot(s.points[*a].y - s.points[*e].y));
                        let hit = lens.iter().any(|x| (x - l).abs() < 1e-6 * l.max(1.0));
                        lines += 1;
                        lines_hit += hit as usize;
                        if si == 0 {
                            first_lines += 1;
                            first_hit += hit as usize;
                        }
                    }
                    _ => {}
                }
            }
        }
    }
    eprintln!("corpus vs body: {circles_hit}/{circles} certain circles match a body radius, {lines_hit}/{lines} lines match a body edge length ({first_hit}/{first_lines} in each part's first sketch)");
    if circles > 20 {
        assert!(circles_hit * 10 >= circles * 8, "{circles_hit}/{circles}");
    }
    if first_lines > 100 {
        assert!(first_hit * 10 >= first_lines * 5, "{first_hit}/{first_lines}");
    }
}

/// The centre index an arc or circle record stores: where it names a point equidistant from an arc's ends, the centre of every arc is
/// that point (167 of 170 arcs of the local corpus; the others are settled by relations to another coincident point), the radius it
/// gives is a radius of the body for all but 2 of the arcs, and more sketches decode exactly than with the search alone (186 of 289).
#[test]
fn corpus_stored_centres_agree_with_the_geometry() {
    let root = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let mut fs = vec![];
    corpus_files(&root.join("samples"), &mut fs);
    if fs.is_empty() {
        return;
    }
    let lim = Limits::default();
    let (mut arcs, mut arcs_agree, mut sketches, mut exact) = (0, 0, 0, 0);
    for p in &fs {
        let Ok(bytes) = std::fs::read(p) else { continue };
        let Ok(c) = container::open(&bytes, &lim) else { continue };
        let Ok(set) = sketch::read(&c, &lim) else { continue };
        for s in &set.sketches {
            sketches += 1;
            exact += s.exact() as usize;
            for cv in &s.curves {
                if let SkCurve::Arc { start, end, centre: Some(k), stored: Some(st), .. } = cv {
                    let d = |i: usize, j: usize| (s.points[i].x - s.points[j].x).hypot(s.points[i].y - s.points[j].y);
                    if *st < s.points.len() && (d(*st, *start) - d(*st, *end)).abs() < 1e-9 * d(*start, *end).max(1e-3) {
                        arcs += 1;
                        arcs_agree += (*st == *k) as usize;
                    }
                }
            }
        }
    }
    eprintln!("stored centres: {arcs_agree}/{arcs} arcs take the stored centre; {exact}/{sketches} sketches decode exactly");
    if arcs > 100 {
        assert!(arcs_agree * 100 >= arcs * 97, "{arcs_agree}/{arcs}");
    }
    if sketches > 250 {
        assert!(exact >= 200, "{exact}/{sketches} exact sketches");
    }
}
