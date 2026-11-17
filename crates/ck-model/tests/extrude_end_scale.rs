//! `endScale` on the extrude feature: the profile scales linearly over the depth to this factor at the far end, about
//! `twistCentre` (default the sketch origin). Analytic volumes of the frusta (V = h/3 (A0 + sqrt(A0 A1) + A1), A1 = A0 s^2),
//! centre of mass and bounding box, mid-plane / flip / cut, circles (right and oblique cones), splines, holes, the refusals
//! with their hints, `endScale` 1 = plain extrude, face names, validity, and a fillet on a frustum edge.
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
    warnings: Vec<String>,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors, warnings: r.warnings.iter().map(|w| w.error.code.clone()).collect() }
}

fn body(o: &Out) -> &ck_topo::body::Body {
    o.brep.body.as_ref().unwrap()
}
fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(body(o)).volume
}
/// A world point in sketch coordinates: (x, y, height above the sketch plane). The xz plane: x right, y = world z, normal -Y.
fn sk(p: ck_geom::Vec3) -> ck_geom::Vec3 {
    ck_geom::Vec3::new(p.x, p.z, -p.y)
}
fn com(o: &Out) -> ck_geom::Vec3 {
    sk(ck_topo::mass::mass_props(body(o)).com)
}
fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-9)
}
fn param(e: &regen::FeatureError, k: &str) -> Option<Json> {
    e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
}
fn check_valid(o: &Out, what: &str) {
    let b = body(o);
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
    assert!(ck_topo::check::check(b, 1e-6).is_empty(), "{what}: {:?}", ck_topo::check::check(b, 1e-6));
}
fn names(o: &Out) -> Vec<String> {
    let b = body(o);
    let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    v.sort();
    v
}
fn ok(o: &Out, what: &str) {
    assert!(o.errors.is_empty(), "{what}: {:?}", o.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
}

/// Frustum volume of a section area `a0` scaled by `s` over height `h`.
fn frustum(a0: f64, s: f64, h: f64) -> f64 {
    let a1 = a0 * s * s;
    h / 3.0 * (a0 + (a0 * a1).sqrt() + a1)
}

fn polygon(id: &str, pts: &[(f64, f64)], tag: usize) -> String {
    let n = pts.len();
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"{id}p{i}","x":{x},"y":{y}}}"#)).collect();
    let e: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"{id}p{i}","p2":"{id}p{}"}}"#, tag + i + 1, (i + 1) % n)).collect();
    format!("{}|{}", p.join(","), e.join(","))
}

/// A sketch on the top plane of one or more closed polygons (ids e1.. in order).
fn sketch_polys(polys: &[&[(f64, f64)]]) -> String {
    let (mut ps, mut es) = (vec![], vec![]);
    let mut tag = 0;
    for (k, pts) in polys.iter().enumerate() {
        let (p, e) = polygon(&format!("k{k}"), pts, tag).split_once('|').map(|(a, b)| (a.to_string(), b.to_string())).unwrap();
        ps.push(p);
        es.push(e);
        tag += pts.len();
    }
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"xz"}},"points":[{}],"entities":[{}],"constraints":[]}}"#,
        ps.join(","),
        es.join(",")
    )
}

fn square(c: (f64, f64), half: f64) -> Vec<(f64, f64)> {
    vec![(c.0 - half, c.1 - half), (c.0 + half, c.1 - half), (c.0 + half, c.1 + half), (c.0 - half, c.1 + half)]
}

fn part(sk: &str, rest: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{rest}]}}"#)
}

fn ext(depth: f64, extra: &str) -> String {
    ext_end("blind", depth, extra)
}

fn ext_end(end: &str, depth: f64, extra: &str) -> String {
    format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"{end}","depth":{depth}{extra}}}"#)
}

fn circle_sketch(cx: f64, cy: f64, r: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"xz"}},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#
    )
}

/// Bounding box in sketch coordinates (x, y, height along the normal).
fn bbox(o: &Out) -> ([f64; 3], [f64; 3]) {
    let (lo, hi) = body(o).bbox();
    ([lo.x, lo.z, -hi.y], [hi.x, hi.z, -lo.y])
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn expect_face_names(o: &Out, want: &[&str]) {
    let n = names(o);
    for w in want {
        assert!(n.contains(&w.to_string()), "{w} missing in {n:?}");
    }
}

#[test]
fn square_frustum() {
    let sq = square((0.0, 0.0), 20.0);
    let o = run(&part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":0.5"#)));
    ok(&o, "frustum");
    assert!(rel(vol(&o), 20.0 / 3.0 * (1600.0 + 800.0 + 400.0)) < 1e-6, "{}", vol(&o));
    assert!(rel(vol(&o), 18666.666666666668) < 1e-6);
    check_valid(&o, "frustum");
    assert_eq!(names(&o), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"]);
    let (lo, hi) = bbox(&o);
    // widest at the sketch plane (+-20), the far end +-10 in a plane 20 away
    assert!(close(lo[0], -20.0, 1e-9) && close(hi[0], 20.0, 1e-9) && close(lo[1], -20.0, 1e-9) && close(hi[1], 20.0, 1e-9), "{lo:?} {hi:?}");
    assert!(close(hi[2] - lo[2], 20.0, 1e-9), "{lo:?} {hi:?}");
    // centre of mass along the axis from the wide end: h (A0 + 2 sqrt(A0A1) + 3 A1) / (4 (A0 + sqrt(A0A1) + A1))
    let zbar = 20.0 * (1600.0 + 1600.0 + 1200.0) / (4.0 * 2800.0);
    let c = com(&o);
    let wide_z = if close(lo[2], 0.0, 1e-9) { 0.0 } else { hi[2] };
    let dir = if close(lo[2], 0.0, 1e-9) { 1.0 } else { -1.0 };
    assert!(close((c.z - wide_z) * dir, zbar, 1e-6), "{} {zbar}", c.z);
    assert!(c.x.abs() < 1e-6 && c.y.abs() < 1e-6);
    // the sides are planes leaning in, the caps keep their size
    let b = body(&o);
    let c1 = b.find_face("F2.C1").unwrap();
    let c0 = b.find_face("F2.C0").unwrap();
    let area = |f| face_area(b, f);
    assert!(rel(area(c0), 1600.0) < 1e-6 && rel(area(c1), 400.0) < 1e-6 || rel(area(c0), 400.0) < 1e-6 && rel(area(c1), 1600.0) < 1e-6);
}

fn face_area(b: &ck_topo::body::Body, f: ck_topo::body::FaceId) -> f64 {
    // the plane cap area from its bounding box (a square cap)
    let (lo, hi) = b.bbox_of_faces(&[f]);
    let (dx, dy, dz) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
    let mut d = [dx, dy, dz];
    d.sort_by(|a, b| a.total_cmp(b));
    d[1] * d[2]
}

#[test]
fn growing_frustum() {
    let sq = square((0.0, 0.0), 20.0);
    let o = run(&part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":2"#)));
    ok(&o, "s=2");
    assert!(rel(vol(&o), frustum(1600.0, 2.0, 20.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "s=2");
    let (lo, hi) = bbox(&o);
    assert!(close(hi[0] - lo[0], 80.0, 1e-9), "{lo:?} {hi:?}");
}

#[test]
fn circle_frustum_is_a_cone() {
    let o = run(&part(&circle_sketch(0.0, 0.0, 15.0), &ext(10.0, r#","endScale":0.4"#)));
    ok(&o, "cone");
    let r1 = 6.0;
    assert!(rel(vol(&o), PI * 10.0 / 3.0 * (225.0 + 15.0 * r1 + r1 * r1)) < 1e-9, "{}", vol(&o));
    check_valid(&o, "cone");
    assert_eq!(names(&o), ["F2.C0", "F2.C1", "F2.S.e1"]);
    let b = body(&o);
    let f = b.find_face("F2.S.e1").unwrap();
    match &b.face(f).surface {
        ck_geom::surface::Surface::Cone { r, semi_angle, .. } => {
            assert!(close(*r, 15.0, 1e-9), "{r}");
            assert!(close(semi_angle.tan(), (6.0 - 15.0) / 10.0, 1e-9), "{semi_angle}");
        }
        s => panic!("side is {}", s.kind()),
    }
}

#[test]
fn circle_about_another_centre_is_an_oblique_cone() {
    // circle (20, 0) r 10 scaled 0.5 about the origin: far circle (10, 0) r 5
    let o = run(&part(&circle_sketch(20.0, 0.0, 10.0), &ext(10.0, r#","endScale":0.5"#)));
    ok(&o, "oblique");
    assert!(rel(vol(&o), frustum(PI * 100.0, 0.5, 10.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "oblique");
    assert_eq!(names(&o), ["F2.C0", "F2.C1", "F2.S.e1"]);
    let (lo, hi) = bbox(&o);
    assert!(close(lo[0], 5.0, 1e-6) && close(hi[0], 30.0, 1e-6), "{lo:?} {hi:?}");
    // explicit centre on the circle's centre: a right cone, the axis stays at x = 20
    let o = run(&part(&circle_sketch(20.0, 0.0, 10.0), &ext(10.0, r#","endScale":0.5,"twistCentre":[20,0]"#)));
    ok(&o, "centred");
    let b = body(&o);
    let f = b.find_face("F2.S.e1").unwrap();
    assert!(matches!(b.face(f).surface, ck_geom::surface::Surface::Cone { .. }), "{}", b.face(f).surface.kind());
    assert!(close(com(&o).x, 20.0, 1e-6));
    check_valid(&o, "centred");
}

#[test]
fn l_shape() {
    let l = [(0.0, 0.0), (30.0, 0.0), (30.0, 10.0), (10.0, 10.0), (10.0, 30.0), (0.0, 30.0)];
    let a0 = 30.0 * 10.0 + 10.0 * 20.0;
    for s in [0.5, 1.5] {
        let o = run(&part(&sketch_polys(&[&l]), &ext(12.0, &format!(r#","endScale":{s}"#))));
        ok(&o, "L");
        assert!(rel(vol(&o), frustum(a0, s, 12.0)) < 1e-6, "{s}: {}", vol(&o));
        check_valid(&o, "L");
        assert_eq!(names(&o).len(), 8);
    }
}

/// com of the swept sections: section at height t is the profile scaled by k(t) about `ctr`; its centroid moves with it.
fn expected_com_xy(c0: (f64, f64), ctr: (f64, f64), a0: f64, s: f64, h: f64) -> (f64, f64, f64) {
    let n = 20000;
    let (mut v, mut mx, mut my, mut mz) = (0.0, 0.0, 0.0, 0.0);
    for i in 0..n {
        let t = (i as f64 + 0.5) / n as f64;
        let k = 1.0 + (s - 1.0) * t;
        let dv = a0 * k * k * h / n as f64;
        v += dv;
        mx += dv * (ctr.0 + k * (c0.0 - ctr.0));
        my += dv * (ctr.1 + k * (c0.1 - ctr.1));
        mz += dv * t * h;
    }
    (mx / v, my / v, mz / v)
}

#[test]
fn off_centre_profile_default_and_explicit_centre() {
    // 20 x 20 square centred (30, 10): default centre = the sketch origin, so the body drifts towards it
    let sq = square((30.0, 10.0), 10.0);
    for (extra, ctr) in [(r#","endScale":0.5"#, (0.0, 0.0)), (r#","endScale":0.5,"twistCentre":[30,10]"#, (30.0, 10.0)), (r#","endScale":0.5,"twistCentre":[40,0]"#, (40.0, 0.0))] {
        let o = run(&part(&sketch_polys(&[&sq]), &ext(10.0, extra)));
        ok(&o, extra);
        assert!(rel(vol(&o), frustum(400.0, 0.5, 10.0)) < 1e-6, "{extra}: {}", vol(&o));
        check_valid(&o, extra);
        let (ex, ey, ez) = expected_com_xy((30.0, 10.0), ctr, 400.0, 0.5, 10.0);
        let c = com(&o);
        assert!(close(c.x, ex, 1e-3) && close(c.y, ey, 1e-3), "{extra}: com {c:?} vs {ex} {ey}");
        let (lo, hi) = bbox(&o);
        // far section corners
        let far = |p: (f64, f64)| (ctr.0 + 0.5 * (p.0 - ctr.0), ctr.1 + 0.5 * (p.1 - ctr.1));
        let (a, b) = (far((20.0, 0.0)), far((40.0, 20.0)));
        assert!(close(lo[0], 20.0f64.min(a.0), 1e-9) && close(hi[0], 40.0f64.max(b.0), 1e-9), "{extra}: {lo:?} {hi:?}");
        assert!(close(lo[1], 0.0f64.min(a.1), 1e-9) && close(hi[1], 20.0f64.max(b.1), 1e-9), "{extra}: {lo:?} {hi:?}");
        let _ = ez;
    }
}

#[test]
fn flip_scales_towards_minus_normal() {
    let sq = square((0.0, 0.0), 20.0);
    let a = run(&part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":0.5"#)));
    let b = run(&part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":0.5,"flip":true"#)));
    ok(&b, "flip");
    assert!(rel(vol(&b), vol(&a)) < 1e-9);
    check_valid(&b, "flip");
    let (la, ha) = bbox(&a);
    let (lb, hb) = bbox(&b);
    // the wide end stays on the sketch plane (z = 0), the body is on the other side of it
    assert!(close(la[2], 0.0, 1e-9) && close(ha[2], 20.0, 1e-9), "{la:?} {ha:?}");
    assert!(close(lb[2], -20.0, 1e-9) && close(hb[2], 0.0, 1e-9), "{lb:?} {hb:?}");
    assert!(close(hb[0] - lb[0], 40.0, 1e-9));
    assert_eq!(names(&a), names(&b));
    // the wide cap is the one on the sketch plane in both
    let wide = |o: &Out| {
        let bd = body(o);
        let f = bd.find_face("F2.C0").unwrap();
        let (l, h) = bd.bbox_of_faces(&[f]);
        h.x - l.x
    };
    assert!(close(wide(&a), 40.0, 1e-9) && close(wide(&b), 20.0, 1e-9), "{} {}", wide(&a), wide(&b));
}

#[test]
fn mid_plane_is_symmetric_about_the_sketch_plane() {
    // scale 1 at the sketch plane, endScale at both ends: two frusta of half the depth
    let sq = square((0.0, 0.0), 20.0);
    let o = run(&part(&sketch_polys(&[&sq]), &ext_end("mid", 20.0, r#","endScale":0.5"#)));
    ok(&o, "mid");
    assert!(rel(vol(&o), 2.0 * frustum(1600.0, 0.5, 10.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "mid");
    let (lo, hi) = bbox(&o);
    assert!(close(lo[2], -10.0, 1e-9) && close(hi[2], 10.0, 1e-9), "{lo:?} {hi:?}");
    assert!(close(hi[0] - lo[0], 40.0, 1e-9));
    let c = com(&o);
    assert!(c.x.abs() < 1e-6 && c.y.abs() < 1e-6 && c.z.abs() < 1e-6, "{c:?}");
    let n = names(&o);
    assert!(n.contains(&"F2.C0".to_string()) && n.contains(&"F2.C1".to_string()), "{n:?}");
    assert_eq!(n.len(), 2 + 8, "{n:?}");
    for e in 1..=4 {
        assert!(n.contains(&format!("F2.S.e{e}~1")) && n.contains(&format!("F2.S.e{e}~2")), "{n:?}");
    }
    // a circle about its own centre: two cones
    let c = run(&part(&circle_sketch(0.0, 0.0, 10.0), &ext_end("mid", 10.0, r#","endScale":0.5"#)));
    ok(&c, "mid circle");
    assert!(rel(vol(&c), 2.0 * frustum(PI * 100.0, 0.5, 5.0)) < 1e-6, "{}", vol(&c));
    check_valid(&c, "mid circle");
}

#[test]
fn cut_subtracts_the_frustum() {
    // a 60 x 60 x 30 block, a frustum cut (20 square, 10 deep, 0.5) from its bottom face going up into it
    let block = square((0.0, 0.0), 30.0);
    let hole = square((0.0, 0.0), 10.0);
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"xz"}},"points":[{}],"entities":[{}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":10,"flip":true,"endScale":0.5}}]}}"#,
        sketch_polys(&[&block]),
        polygon("h", &hole, 0).split_once('|').unwrap().0,
        polygon("h", &hole, 0).split_once('|').unwrap().1,
    );
    let o = run(&src);
    ok(&o, "cut");
    assert!(rel(vol(&o), 60.0 * 60.0 * 30.0 - frustum(400.0, 0.5, 10.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "cut");
    expect_face_names(&o, &["F4.S.e1", "F4.S.e2", "F4.S.e3", "F4.S.e4"]);
    // not flipped: the cut runs away from the block, nothing is removed
    let o = run(&src.replace(r#""depth":10,"flip":true,"#, r#""depth":10,"#));
    assert!(o.errors.iter().all(|e| e.error.code == "cutNoEffect"), "{:?}", o.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    assert!(rel(vol(&o), 60.0 * 60.0 * 30.0) < 1e-9, "{}", vol(&o));
}

#[test]
fn spline_profile() {
    // a closed spline through eight points of an ellipse-like blob, off the origin
    let pts: Vec<(f64, f64)> = (0..8).map(|i| (25.0 + 12.0 * (i as f64 * PI / 4.0).cos(), 5.0 + 8.0 * (i as f64 * PI / 4.0).sin())).collect();
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ids: Vec<String> = (0..8).map(|i| format!(r#""p{i}""#)).collect();
    let sk = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"xz"}},"points":[{}],"entities":[{{"type":"spline","id":"e1","pts":[{}],"closed":true}}],"constraints":[]}}"#,
        ps.join(","),
        ids.join(",")
    );
    let plain = run(&part(&sk, &ext(1.0, "")));
    ok(&plain, "plain spline");
    let a0 = vol(&plain);
    for s in [0.5, 1.6] {
        let o = run(&part(&sk, &ext(10.0, &format!(r#","endScale":{s}"#))));
        ok(&o, "spline");
        assert!(rel(vol(&o), frustum(a0, s, 10.0)) < 1e-5, "{s}: {} vs {}", vol(&o), frustum(a0, s, 10.0));
        check_valid(&o, "spline");
        expect_face_names(&o, &["F2.C0", "F2.C1"]);
    }
}

#[test]
fn profile_with_a_hole_scales_both_loops() {
    let outer = square((0.0, 0.0), 20.0);
    let inner = square((5.0, 0.0), 10.0);
    let a0 = 1600.0 - 400.0;
    let o = run(&part(&sketch_polys(&[&outer, &inner]), &ext(20.0, r#","endScale":0.5"#)));
    ok(&o, "hole");
    assert!(rel(vol(&o), frustum(a0, 0.5, 20.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "hole");
    assert_eq!(names(&o).len(), 2 + 8);
    // the hole's far wall is closer to the axis too: its section scales
    let (lo, hi) = bbox(&o);
    assert!(close(hi[0] - lo[0], 40.0, 1e-9));
}

#[test]
fn tiny_scale_builds_or_refuses_cleanly() {
    let sq = square((0.0, 0.0), 20.0);
    let o = run(&part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":0.01"#)));
    ok(&o, "0.01");
    assert!(rel(vol(&o), frustum(1600.0, 0.01, 20.0)) < 1e-6, "{}", vol(&o));
    check_valid(&o, "0.01");
}

fn err_of(src: &str) -> regen::FeatureError {
    let o = run(src);
    assert!(!o.errors.is_empty(), "no error");
    o.errors[0].error.clone()
}

fn has_hint(e: &regen::FeatureError) {
    match param(e, "hint") {
        Some(Json::Str(h)) => assert!(!h.is_empty()),
        other => panic!("no hint: {other:?} on {}", e.code),
    }
}

#[test]
fn refusals_have_a_hint() {
    let sq = square((0.0, 0.0), 20.0);
    let sk = sketch_polys(&[&sq]);
    for bad in ["0", "-0.5", "1e-9"] {
        let e = err_of(&part(&sk, &ext(20.0, &format!(r#","endScale":{bad}"#))));
        assert_eq!(e.code, "invalidValue", "{bad}");
        assert_eq!(param(&e, "field"), Some(Json::Str("endScale".into())), "{bad}");
        has_hint(&e);
    }
    // not a finite number
    let e = err_of(&part(&sk, &ext(20.0, r#","endScale":"=1/0""#)));
    assert!(e.code == "invalidValue" || e.code == "badExpression", "{}", e.code);
    // draft
    for d in [r#","draft":5"#, r#","draft":5,"draftOut":true"#] {
        let e = err_of(&part(&sk, &ext(20.0, &format!(r#","endScale":0.5{d}"#))));
        assert_eq!(e.code, "draftUnsupported");
        has_hint(&e);
        match param(&e, "hint") {
            Some(Json::Str(h)) => assert!(h.contains("either a draft angle or an end scale"), "{h}"),
            _ => unreachable!(),
        }
        assert!(param(&e, "detail").is_some());
    }
    // twist
    let e = err_of(&part(&sk, &ext(20.0, r#","endScale":0.5,"twist":30"#)));
    assert_eq!(e.code, "draftUnsupported");
    has_hint(&e);
    // end conditions that are not a set distance
    for end in ["through", "throughBoth"] {
        let e = err_of(&part(&sk, &format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"{end}","depth":20,"endScale":0.5}}"#)));
        assert_eq!(e.code, "invalidValue", "{end}");
        has_hint(&e);
    }
}

#[test]
fn up_to_next_is_refused() {
    // a block, then an endScale extrude up to the next face
    let block = square((0.0, 0.0), 30.0);
    let hole = square((0.0, 0.0), 10.0);
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}},
{{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"xz"}},"points":[{}],"entities":[{}],"constraints":[]}},
{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"upToNext","endScale":0.5}}]}}"#,
        sketch_polys(&[&block]),
        polygon("h", &hole, 0).split_once('|').unwrap().0,
        polygon("h", &hole, 0).split_once('|').unwrap().1,
    );
    let e = err_of(&src);
    assert!(e.code == "invalidValue" || e.code == "draftUnsupported", "{}", e.code);
    has_hint(&e);
}

#[test]
fn end_scale_one_is_the_plain_extrude() {
    let l = [(0.0, 0.0), (30.0, 0.0), (30.0, 10.0), (10.0, 10.0), (10.0, 30.0), (0.0, 30.0)];
    let sk = sketch_polys(&[&l]);
    let plain = run(&part(&sk, &ext(12.0, "")));
    let one = run(&part(&sk, &ext(12.0, r#","endScale":1"#)));
    ok(&one, "one");
    assert_eq!(names(&plain), names(&one));
    assert_eq!(body(&plain).num_faces(), body(&one).num_faces());
    assert_eq!(vol(&plain), vol(&one));
    assert_eq!(com(&plain).x, com(&one).x);
    // also with a draft or twist present: endScale 1 changes nothing
    let d = run(&part(&sk, &ext(12.0, r#","draft":5"#)));
    let d1 = run(&part(&sk, &ext(12.0, r#","draft":5,"endScale":1"#)));
    assert!(d.errors.is_empty() && d1.errors.is_empty());
    assert_eq!(vol(&d), vol(&d1));
    // a plain extrude carries no warning about the field, and the scaled one neither
    assert!(!one.warnings.iter().any(|w| w == "unknownField"), "{:?}", one.warnings);
    let s = run(&part(&sk, &ext(12.0, r#","endScale":0.5"#)));
    assert!(!s.warnings.iter().any(|w| w == "unknownField"), "{:?}", s.warnings);
}

#[test]
fn fillet_on_a_frustum_edge() {
    let sq = square((0.0, 0.0), 20.0);
    let base = part(&sketch_polys(&[&sq]), &ext(20.0, r#","endScale":0.5"#));
    let o = run(&base);
    ok(&o, "base");
    let b = body(&o);
    let (cf, sf) = (b.find_face("F2.C1").unwrap(), b.find_face("F2.S.e1").unwrap());
    let ed = b.edge_ids().find(|&e| {
        let fs = b.edge_faces(e);
        fs.len() == 2 && fs.contains(&cf) && fs.contains(&sf)
    });
    let m = b.edge_mid(ed.expect("far rim edge"));
    let src = base.replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F3","radius":1.5,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[{},{},{}]}}]}}]}}"#, m.x, m.y, m.z));
    let f = run(&src);
    ok(&f, "fillet");
    assert!(vol(&f) < vol(&o) && vol(&f) > vol(&o) - 400.0, "{} vs {}", vol(&f), vol(&o));
    check_valid(&f, "fillet");
    // the side seam between two leaning planes can be rounded too
    let (s1, s2) = (b.find_face("F2.S.e1").unwrap(), b.find_face("F2.S.e2").unwrap());
    let ed = b.edge_ids().find(|&e| {
        let fs = b.edge_faces(e);
        fs.len() == 2 && fs.contains(&s1) && fs.contains(&s2)
    });
    let m = b.edge_mid(ed.expect("corner edge"));
    let src = base.replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F3","radius":1.5,"edges":[{{"faces":["F2.S.e1","F2.S.e2"],"at":[{},{},{}]}}]}}]}}"#, m.x, m.y, m.z));
    let f = run(&src);
    ok(&f, "corner fillet");
    check_valid(&f, "corner fillet");
}

/// A D: the upper half circle (radius 10, about `(cx, 0)`) closed by its diameter.
fn d_sketch(cx: f64) -> String {
    let (l, r) = (cx - 10.0, cx + 10.0);
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"xz"}},"points":[{{"id":"c","x":{cx},"y":0}},{{"id":"p1","x":{r},"y":0}},{{"id":"p2","x":{l},"y":0}}],"entities":[{{"type":"arc","id":"e1","c":"c","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p1"}}],"constraints":[]}}"#
    )
}

#[test]
fn arc_and_line_profile() {
    for (cx, ctr) in [(0.0, ""), (15.0, ""), (15.0, r#","twistCentre":[15,0]"#)] {
        let plain = run(&part(&d_sketch(cx), &ext(1.0, "")));
        ok(&plain, "plain D");
        let a0 = vol(&plain);
        for s in [0.5, 1.7] {
            let o = run(&part(&d_sketch(cx), &ext(8.0, &format!(r#","endScale":{s}{ctr}"#))));
            ok(&o, "D");
            assert!(rel(vol(&o), frustum(a0, s, 8.0)) < 1e-6, "cx {cx} s {s}: {} vs {}", vol(&o), frustum(a0, s, 8.0));
            check_valid(&o, "D");
            assert_eq!(names(&o), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2"]);
        }
    }
}

#[test]
fn thin_wall_scales_too() {
    // a 20 x 20 square wall of 2 (inside): the wall ring scales as a whole
    let sq = square((0.0, 0.0), 10.0);
    let plain = run(&part(&sketch_polys(&[&sq]), &ext(1.0, r#","thin":{"thickness":2,"side":"inside"}"#)));
    ok(&plain, "plain thin");
    let a0 = vol(&plain);
    let o = run(&part(&sketch_polys(&[&sq]), &ext(10.0, r#","thin":{"thickness":2,"side":"inside"},"endScale":0.5"#)));
    ok(&o, "thin");
    assert!(rel(vol(&o), frustum(a0, 0.5, 10.0)) < 1e-6, "{} vs {}", vol(&o), frustum(a0, 0.5, 10.0));
    check_valid(&o, "thin");
}

#[test]
fn surface_extrude_refuses_end_scale() {
    let sq = square((0.0, 0.0), 20.0);
    let src = part(&sketch_polys(&[&sq]), r#"{"type":"surfaceExtrude","id":"F2","sketch":"F1","end":"blind","depth":10,"endScale":0.5}"#);
    let e = err_of(&src);
    assert_eq!(e.code, "invalidValue");
    has_hint(&e);
    // endScale 1 is nothing
    let src = part(&sketch_polys(&[&sq]), r#"{"type":"surfaceExtrude","id":"F2","sketch":"F1","end":"blind","depth":10,"endScale":1}"#);
    assert!(run(&src).errors.is_empty());
}
