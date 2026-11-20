//! n-sided `fill` over rims that are not flat and whose neighbours are tilted or curved: drafted walls, a rim cut by a
//! cylinder (a saddle-shaped ring), curved (arc) walls; plus refusals of rings with fewer than three edges and the
//! regeneration round trip. The flat prism dish is in `fill_n_sided.rs`.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts, RegenResult};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

fn run(src: &str) -> (RegenResult, Exec) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (res, ex)
}

const R: f64 = 10.0;

fn corner(n: usize, i: usize) -> (f64, f64) {
    let a = std::f64::consts::TAU * i as f64 / n as f64;
    (R * a.cos(), R * a.sin())
}

/// Sketch `S1` on the front plane: a regular `n`-gon, its sides arcs bulging out when `bulge`.
fn outline(n: usize, bulge: bool) -> String {
    let mut pts = String::new();
    let mut ents = String::new();
    for i in 0..n {
        let (x, y) = corner(n, i);
        pts += &format!(r#"{}{{"id":"p{i}","x":{x},"y":{y}}}"#, if i > 0 { "," } else { "" });
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let sep = if i > 0 { "," } else { "" };
        if bulge {
            // the side's circle: centre on the bisector, far on the origin side of the chord
            let (a, b) = (corner(n, i), corner(n, j));
            let (mx, my) = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
            let m = (mx * mx + my * my).sqrt();
            let (ux, uy) = (mx / m, my / m);
            let c = m - 14.0;
            pts += &format!(r#",{{"id":"c{i}","x":{},"y":{}}}"#, ux * c, uy * c);
            ents += &format!(r#"{sep}{{"type":"arc","id":"e{}","c":"c{i}","p1":"p{i}","p2":"p{j}"}}"#, i + 1);
        } else {
            ents += &format!(r#"{sep}{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{j}"}}"#, i + 1);
        }
    }
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}}"#)
}

/// The open dish `F5`: the outline extruded (`draft` degrees), optionally cut by a cylinder (a saddle-shaped top), the
/// top face deleted. Returns the part's feature list (to which the fill is appended) and the fill's `edges` picks.
fn dish_features(outline: &str, draft: f64, saddle: bool) -> (String, String) {
    let mut f = vec![outline.to_string(), format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"S1","end":"blind","depth":10,"draft":{draft}}}"#)];
    let top = if saddle {
        // a cylinder along y, centre 40 above the plane, radius 34: it leaves the dish floor-high in the middle
        f.push(r#"{"type":"sketch","id":"S3","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":-40}],"entities":[{"type":"circle","id":"c1","c":"c","r":34}],"constraints":[]}"#.into());
        f.push(r#"{"type":"extrude","id":"F4","op":"cut","sketch":"S3","end":"throughBoth","depth":0}"#.into());
        "F4.S.c1"
    } else {
        "F2.C1"
    };
    f.push(format!(r#"{{"type":"deleteFace","id":"F5","faces":["{top}"]}}"#));
    let feats = f.join(",");
    let src = format!(r#"{{"format":"ck-part","version":1,"name":"t","features":[{feats}]}}"#);
    let (res, ex) = run(&src);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    let (_, sheet) = ex.0.surfs.iter().find(|(id, _)| id == "F5").expect("the open dish");
    let mut picks = vec![];
    for e in sheet.edge_ids() {
        let cs: Vec<_> = sheet.edge_coedges(e).into_iter().filter(|&c| sheet.coedge(c).alive).collect();
        if cs.len() != 1 {
            continue;
        }
        let ed = sheet.edge(e);
        let m = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let face = sheet.face_id_str(sheet.coedge_face(cs[0]));
        picks.push(format!(r#"{{"faces":["{face}"],"at":[{},{},{}]}}"#, m.x, m.y, m.z));
    }
    (feats, picks.join(","))
}

fn dish(outline: &str, draft: f64, saddle: bool, fill_extra: &str) -> (String, usize) {
    let (feats, picks) = dish_features(outline, draft, saddle);
    let n = picks.matches("faces").count();
    (format!(r#"{{"format":"ck-part","version":1,"name":"t","features":[{feats},{{"type":"fill","id":"F6","edges":[{picks}],{fill_extra}}}]}}"#), n)
}

fn describe(res: &RegenResult) -> Vec<String> {
    res.errors.iter().map(|e| e.error.describe()).collect()
}

/// Outward normal of face `f` at the point of it nearest `p`.
fn normal_at(b: &ck_topo::body::Body, f: ck_topo::body::FaceId, p: ck_core::linalg::Vec3) -> ck_core::linalg::Vec3 {
    let face = b.face(f);
    let (u, v) = face.surface.closest_param(p);
    let n = face.surface.normal(u, v);
    if face.sense {
        n
    } else {
        n * -1.0
    }
}

/// The checks every filled dish has to pass; returns the worst normal angle (rad) along the seams.
fn check_filled(label: &str, n: usize, cont: &str, ex: &Exec) -> f64 {
    let b = ex.0.body.as_ref().unwrap_or_else(|| panic!("{label}: the closed shell is the solid body"));
    assert!(ex.0.surfs.is_empty(), "{label}: no sheet left");
    for e in b.edge_ids() {
        assert_eq!(b.edge_coedges(e).len(), 2, "{label}: every edge has two coedges (a closed shell)");
    }
    for k in 0..n {
        assert!(b.find_face(&format!("F6.N{k}")).is_some(), "{label}: F6.N{k}");
    }
    // the patches deviate from the rim edges by at most the patch tolerance (1e-4), which the check is given
    let issues = ck_check::validate(b, 2e-4);
    assert!(issues.is_empty(), "{label}: {issues:?}");
    assert!(ck_topo::mass::mass_props(b).volume > 0.0, "{label}: positive volume");
    // along every seam between a patch and an older face: the points lie on the patch (boundary within tolerance),
    // and for G1 the two outward normals agree
    let is_patch = |f: ck_topo::body::FaceId| b.face(f).name.feature == "F6";
    let mut worst: f64 = 0.0;
    let mut seams = 0;
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        let (Some(&pf), Some(&of)) = (fs.iter().find(|&&f| is_patch(f)), fs.iter().find(|&&f| !is_patch(f))) else { continue };
        seams += 1;
        let ed = b.edge(e);
        // a seam runs from a ring corner (where two different neighbours meet, so the patch cannot be tangent to
        // both) to the middle of the ring edge, where the patch leaves tangent to its neighbour: judge that end
        let mut ends = [0.0f64; 2];
        for i in 0..=8 {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 8.0);
            let sp = &b.face(pf).surface;
            let (u, v) = sp.closest_param(p);
            let off = sp.eval(u, v).dist(p);
            assert!(off < 2e-4, "{label}: seam point {off} off its patch");
            let ang = normal_at(b, pf, p).dot(normal_at(b, of, p)).clamp(-1.0, 1.0).acos();
            if i == 0 {
                ends[0] = ang;
            } else if i == 8 {
                ends[1] = ang;
            }
        }
        worst = worst.max(ends[0].min(ends[1]));
    }
    assert_eq!(seams, 2 * n, "{label}: two seam edges per ring edge");
    if cont == "G1" {
        assert!(worst < 0.02, "{label}: G1 seam normals {worst} rad apart at the middle of a ring edge");
    }
    worst
}

fn fill_cases(label: &str, outline: impl Fn(usize) -> String, draft: f64, saddle: bool) {
    for n in [3usize, 4, 5, 6] {
        for cont in ["G0", "G1"] {
            let (src, ring) = dish(&outline(n), draft, saddle, &format!(r#""merge":true,"patch":"nSided","continuity":"{cont}""#));
            let (res, ex) = run(&src);
            let l = format!("{label} n={n} {cont}");
            assert!(res.errors.is_empty(), "{l}: {:?}", describe(&res));
            assert!(ring >= 3, "{l}: ring of {ring}");
            let worst = check_filled(&l, ring, cont, &ex);
            eprintln!("{l}: ring {ring}, worst seam angle {worst:.4}");
        }
    }
}

#[test]
fn tilted_wall_dish() {
    fill_cases("drafted prism", |n| outline(n, false), 15.0, false);
}

#[test]
fn curved_wall_dish() {
    // arc sides: cylinder walls, or cones when drafted
    fill_cases("arc walls", |n| outline(n, true), 0.0, false);
    fill_cases("drafted arc walls", |n| outline(n, true), 10.0, false);
}

#[test]
fn saddle_rim_dish() {
    // the rim is cut by a cylinder: not planar, its edges are arcs / spline curves on planar, cylinder and cone walls
    fill_cases("saddle on prism", |n| outline(n, false), 0.0, true);
    fill_cases("saddle on drafted prism", |n| outline(n, false), 12.0, true);
    fill_cases("saddle on arc walls", |n| outline(n, true), 0.0, true);
    fill_cases("saddle on drafted arc walls", |n| outline(n, true), 10.0, true);
}

#[test]
fn regeneration_round_trip_is_identical() {
    for cont in ["G0", "G1"] {
        let (src, _) = dish(&outline(5, true), 10.0, true, &format!(r#""merge":true,"patch":"nSided","continuity":"{cont}""#));
        let (r1, a) = run(&src);
        let (r2, b) = run(&src);
        assert!(r1.errors.is_empty() && r2.errors.is_empty(), "{cont}: {:?}", describe(&r1));
        let (ba, bb) = (a.0.body.as_ref().unwrap(), b.0.body.as_ref().unwrap());
        assert_eq!(ck_topo::codec::encode(ba), ck_topo::codec::encode(bb), "{cont}: two regenerations give the same body");
        // undo and redo: the tree without the fill leaves the open dish, and the full tree again gives the same body
        let cut = src.find(r#",{"type":"fill""#).unwrap();
        let undone = format!("{}]}}", &src[..cut]);
        let (r3, u) = run(&undone);
        assert!(r3.errors.is_empty(), "{cont}: {:?}", describe(&r3));
        assert!(u.0.body.is_none() && !u.0.surfs.is_empty(), "{cont}: undone, the dish is open again");
        let (r4, c) = run(&src);
        assert!(r4.errors.is_empty());
        assert_eq!(ck_topo::codec::encode(ba), ck_topo::codec::encode(c.0.body.as_ref().unwrap()), "{cont}: redo gives the same body");
    }
}

/// A cup over `sketch` (extruded, its top deleted) with a fill whose ring is the cup's rim.
fn cup_fill(sketch: &str, cont: &str) -> (RegenResult, Exec, usize) {
    let (feats, picks) = dish_features(sketch, 0.0, false);
    let n = picks.matches("faces").count();
    let src = format!(r#"{{"format":"ck-part","version":1,"name":"t","features":[{feats},{{"type":"fill","id":"F6","edges":[{picks}],"merge":true,"patch":"nSided","continuity":"{cont}"}}]}}"#);
    let (res, ex) = run(&src);
    (res, ex, n)
}

#[test]
fn a_ring_of_fewer_than_three_edges_is_refused() {
    // one full circle edge
    let circle = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":10}],"constraints":[]}"#;
    // two half-circle arcs
    let two = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0},{"id":"a","x":10,"y":0},{"id":"b","x":-10,"y":0}],"entities":[{"type":"arc","id":"e1","c":"c","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"c","p1":"b","p2":"a"}],"constraints":[]}"#;
    for (label, sk, edges) in [("one circle", circle, 1usize), ("two arcs", two, 2)] {
        for cont in ["G0", "G1"] {
            let (res, ex, n) = cup_fill(sk, cont);
            assert_eq!(n, edges, "{label}: the rim has {edges} edge(s)");
            let codes: Vec<_> = res.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
            assert_eq!(codes, [("F6".to_string(), "fillInvalid".to_string())], "{label} {cont}: {:?}", describe(&res));
            // the dish is left as it was: still an open sheet, no half-built patches
            assert!(ex.0.body.is_none(), "{label} {cont}: no solid appeared");
            let (_, sheet) = ex.0.surfs.iter().find(|(id, _)| id == "F5").expect("the sheet stays");
            assert!(sheet.face_ids().all(|f| sheet.face(f).name.feature != "F6"), "{label} {cont}: no patch faces");
        }
    }
}
