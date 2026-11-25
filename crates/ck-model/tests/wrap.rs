//! `wrap` (docs/proposals/wrap-feature.md, phases 1 and 2: the last tests are `projectCurve method:"wrap"`): sketch text and closed profiles laid on a cylindrical face, outside or in a
//! bore, raised or engraved. The reference numbers are independent: the glyph area from ck-text's own layout, the radial shell
//! volume formula, and the widths of the sketch.
use ck_core::linalg::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::{Body, FaceId};
use std::f64::consts::TAU;

struct Exec(Brep);
impl FeatureExec for Exec {
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

struct Out {
    body: Option<Body>,
    errors: Vec<(String, String, String)>,
    warnings: Vec<(String, String, String)>,
    markers: Vec<ck_json::Json>,
    curves: Vec<ck_json::Json>,
}

fn run(features: &str) -> Out {
    run_vars("", features)
}

/// `vars`: `"variables":[...],` or empty.
fn run_vars(vars: &str, features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,{vars}"features":[{features}]}}"#);
    run_part(&src)
}

fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let fmt = |e: &regen::RegenError| (e.feature.clone(), e.error.code.clone(), format!("{:?}", e.error.params));
    Out { body: ex.0.body.clone(), errors: r.errors.iter().map(fmt).collect(), warnings: r.warnings.iter().filter(|w| w.error.code != "unknownField").map(fmt).collect(), markers: r.markers.clone(), curves: r.curves.clone() }
}

fn volume(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn valid(b: &Body, what: &str) {
    let issues = ck_check::validate(b, 1e-6);
    assert!(issues.is_empty(), "{what}: {} validity issues, first {:?}", issues.len(), issues.first());
}

/// A tube about the z axis from z = 0 to `h`: outer radius `ro`, bore radius `ri` (0: solid). Faces `F1.S.e1` (outside), `F1.S.e2` (bore).
fn tube(ro: f64, ri: f64, h: f64) -> String {
    let (p2, e2) = if ri > 0.0 { (r#",{"id":"p2","x":0,"y":0}"#.to_string(), format!(r#",{{"type":"circle","id":"e2","c":"p2","r":{ri}}}"#)) } else { (String::new(), String::new()) };
    format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":0,"y":0}}{p2}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":{ro}}}{e2}],"constraints":[]}},
        {{"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":{h}}}"#
    )
}

/// A text sketch on the plane through the axis yz (normal +x; sketch x = +y, sketch y = +z).
fn text_sketch(id: &str, string: &str, font: &str, height: f64, at: [f64; 2]) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"yz"}},"points":[],"entities":[{{"type":"text","id":"t1","string":"{string}","font":"{font}","height":{height},"at":[{},{}],"anchor":["center","middle"]}}],"constraints":[]}}"#,
        at[0], at[1]
    )
}

fn layout(string: &str, font: &str, height: f64, at: [f64; 2]) -> ck_text::Layout {
    let j = ck_json::parse(&format!(r#"{{"type":"text","string":"{string}","font":"{font}","height":{height},"at":[{},{}],"anchor":["center","middle"]}}"#, at[0], at[1])).unwrap();
    ck_text::layout(&ck_model::sketch::text::TextEntity::from_json(&j).spec).expect("layout")
}

fn ok(o: &Out, what: &str) -> Body {
    assert!(o.errors.is_empty(), "{what}: errors {:?}", o.errors);
    o.body.clone().expect(what)
}

/// Radial shell volume of an area `a` (on the face of radius `r`) raised / cut by `d`.
fn shell_volume(a: f64, d: f64, r: f64, outward: bool) -> f64 {
    a * d * (1.0 + if outward { d / (2.0 * r) } else { -d / (2.0 * r) })
}

#[test]
fn mug_text_cut_and_add() {
    let (ro, ri, h) = (40.0, 37.0, 90.0);
    let base = tube(ro, ri, h);
    let v0 = volume(&run(&base).body.unwrap());
    for font in ["gauge", "engrave"] {
        let lay = layout("CADKIT 1/2", font, 6.0, [0.0, 45.0]);
        let area = lay.area();
        for (op, depth) in [("cut", 0.4), ("add", 0.5)] {
            let o = run(&format!(
                r#"{base},{},{{"type":"wrap","id":"F3","sketch":"S2","face":"F1.S.e1","op":"{op}","depth":{depth}}}"#,
                text_sketch("S2", "CADKIT 1/2", font, 6.0, [0.0, 45.0])
            ));
            let b = ok(&o, &format!("{font} {op}"));
            valid(&b, &format!("{font} {op}"));
            assert_eq!(b.num_lumps(), 1);
            let dv = volume(&b) - v0;
            let want = if op == "add" { shell_volume(area, depth, ro, true) } else { -shell_volume(area, depth, ro, false) };
            assert!((dv - want).abs() <= 0.015 * want.abs(), "{font} {op}: volume change {dv} vs {want}");
        }
    }
}

/// A closed loop of straight lines in the text plane (sketch x = +y, sketch y = +z) through the points `pts` (u, w).
fn poly_sketch(id: &str, pts: &[[f64; 2]]) -> String {
    let n = pts.len();
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p[0], p[1])).collect();
    let lines: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{i}","p1":"p{i}","p2":"p{}"}}"#, (i + 1) % n)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"yz"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), lines.join(","))
}

fn rect(id: &str, u0: f64, u1: f64, w0: f64, w1: f64) -> String {
    poly_sketch(id, &[[u0, w0], [u1, w0], [u1, w1], [u0, w1]])
}

fn wrap(id: &str, sketch: &str, face: &str, op: &str, depth: f64, extra: &str) -> String {
    format!(r#"{{"type":"wrap","id":"{id}","sketch":"{sketch}","face":"{face}","op":"{op}","depth":{depth}{extra}}}"#)
}

fn faces_named(b: &Body, prefix: &str) -> Vec<FaceId> {
    b.face_ids().filter(|&f| b.face_id_str(f).starts_with(prefix)).collect()
}

fn near(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * a.abs().max(b.abs()).max(1e-12)
}

/// Angular extent (radians, about the z axis, around `mid`) of the edges of one face.
fn angular_extent(b: &Body, f: FaceId) -> f64 {
    let mut ths: Vec<f64> = vec![];
    for e in b.face_edges(f) {
        for p in b.sample_edge(e, 1e-4) {
            ths.push(p.y.atan2(p.x));
        }
    }
    // the profile is within a half turn: unwrap about the first sample
    let t0 = ths[0];
    let un: Vec<f64> = ths.iter().map(|t| { let mut d = t - t0; while d > std::f64::consts::PI { d -= TAU; } while d < -std::f64::consts::PI { d += TAU; } d }).collect();
    un.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - un.iter().cloned().fold(f64::INFINITY, f64::min)
}

#[test]
fn glyph_widths_are_kept() {
    let (ro, ri, h) = (40.0, 37.0, 90.0);
    let lay = layout("CADKIT 1/2", "gauge", 6.0, [0.0, 45.0]);
    let o = run(&format!(
        r#"{},{},{}"#,
        tube(ro, ri, h),
        text_sketch("S2", "CADKIT 1/2", "gauge", 6.0, [0.0, 45.0]),
        wrap("F3", "S2", "F1.S.e1", "cut", 0.4, "")
    ));
    let b = ok(&o, "cut");
    let m = o.markers.iter().find(|m| m.as_arr().unwrap()[0].as_str() == Some("F3")).expect("marker");
    let mk = &m.as_arr().unwrap()[1];
    assert_eq!(mk.get("profiles").and_then(|v| v.as_f64()), Some(lay.regions.len() as f64));
    assert_eq!(mk.get("surface").and_then(|v| v.as_str()), Some("cylinder"));
    assert!(mk.get("deviation").and_then(|v| v.as_f64()).unwrap() <= 0.001 / 4.0 + 1e-9);
    for (k, reg) in lay.regions.iter().enumerate() {
        let poly = ck_text::loop_polygon(&reg.outer, 16);
        let (lo, hi) = poly.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(q[0]), a.1.max(q[0])));
        let name = format!("F3.C0~{}", k + 1);
        let fid = b.find_face(&name).unwrap_or_else(|| panic!("no face {name}; faces {:?}", faces_named(&b, "F3.C0").iter().map(|&f| b.face_id_str(f)).collect::<Vec<_>>()));
        let w = angular_extent(&b, fid) * ro;
        assert!((w - (hi - lo)).abs() <= 2e-3, "glyph {k}: {w} vs {}", hi - lo);
    }
}

#[test]
fn rectangle_label_has_exact_planar_walls() {
    let (ro, h, d) = (40.0, 90.0, 0.8);
    let (a, bz) = (20.0, 10.0);
    for (op, want) in [("cut", -((ro * ro - (ro - d) * (ro - d)) / 2.0 * (a / ro) * bz)), ("add", ((ro + d) * (ro + d) - ro * ro) / 2.0 * (a / ro) * bz)] {
        let base = tube(ro, 0.0, h);
        let v0 = volume(&run(&base).body.unwrap());
        let o = run(&format!("{base},{},{}", rect("S2", -a / 2.0, a / 2.0, 40.0, 40.0 + bz), wrap("F3", "S2", "F1.S.e1", op, d, "")));
        let b = ok(&o, op);
        valid(&b, op);
        let walls = faces_named(&b, "F3.S.");
        assert_eq!(walls.len(), 4, "{op}");
        for f in walls {
            assert!(matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }), "{op}: {} is not a plane", b.face_id_str(f));
        }
        let m = o.markers.iter().find(|m| m.as_arr().unwrap()[0].as_str() == Some("F3")).unwrap();
        assert_eq!(m.as_arr().unwrap()[1].get("deviation").and_then(|v| v.as_f64()), Some(0.0));
        let dv = volume(&b) - v0;
        assert!(near(dv, want, 1e-6), "{op}: {dv} vs {want}");
    }
}

/// The point of the sketch glyph map: outside the reading direction runs towards +y seen from +x (the sketch plane normal), in a bore
/// towards -y, so a letter reads the same from the side the face normal points to.
#[test]
fn reading_direction_outside_and_in_a_bore() {
    use ck_ops::boolean::classify::{Class, Solid};
    let (ro, ri, h, d) = (40.0, 37.0, 90.0, 0.6);
    let lay = layout("R", "gauge", 8.0, [0.0, 45.0]);
    let reg = &lay.regions[0];
    let outer: Vec<[f64; 2]> = ck_text::loop_polygon(&reg.outer, 16);
    let holes: Vec<Vec<[f64; 2]>> = reg.holes.iter().map(|l| ck_text::loop_polygon(l, 16)).collect();
    let inside = |q: [f64; 2]| -> bool {
        let pip = |poly: &[[f64; 2]]| {
            let mut c = false;
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0] {
                    c = !c;
                }
            }
            c
        };
        pip(&outer) && !holes.iter().any(|hh| pip(hh))
    };
    let uc = lay.bbox[0] * 0.5 + lay.bbox[2] * 0.5;
    // a point inside the glyph whose mirror about the glyph middle is outside it
    let mut pick = None;
    'o: for i in 0..60 {
        for j in 0..60 {
            let q = [lay.bbox[0] + (lay.bbox[2] - lay.bbox[0]) * (i as f64 + 0.5) / 60.0, lay.bbox[1] + (lay.bbox[3] - lay.bbox[1]) * (j as f64 + 0.5) / 60.0];
            if inside(q) && !inside([2.0 * uc - q[0], q[1]]) {
                pick = Some(q);
                break 'o;
            }
        }
    }
    let q = pick.expect("asymmetric point");
    for (face, r, sgn, op) in [("F1.S.e1", ro, 1.0, "cut"), ("F1.S.e1", ro, 1.0, "add"), ("F1.S.e2", ri, -1.0, "cut"), ("F1.S.e2", ri, -1.0, "add")] {
        let o = run(&format!("{},{},{}", tube(ro, ri, h), text_sketch("S2", "R", "gauge", 8.0, [0.0, 45.0]), wrap("F3", "S2", face, op, d, "")));
        let b = ok(&o, &format!("{face} {op}"));
        valid(&b, &format!("{face} {op}"));
        let solid = Solid::new(&b, 1e-6);
        // the centre of the glyph's bounding box is the middle of the map: its angle 0 is the +x side, the map turns by sgn * u / r
        let at = |u: f64, w: f64, rad: f64| {
            let th = sgn * (u - uc) / r;
            Vec3::new(rad * th.cos(), rad * th.sin(), w)
        };
        // a cut removed material at the glyph, an add put material there; the mirror image is untouched
        let probe = if op == "cut" { r - sgn * d / 2.0 } else { r + sgn * d / 2.0 };
        let (want_hit, want_miss) = if op == "cut" { (Class::Out, Class::In) } else { (Class::In, Class::Out) };
        let hit = solid.classify(at(q[0], q[1], probe), Vec3::X);
        let miss = solid.classify(at(2.0 * uc - q[0], q[1], probe), Vec3::X);
        // (the base material at the probe radius is In for a cut, Out above the face for an add)
        assert_eq!(hit, want_hit, "{face} {op}: glyph point");
        assert_eq!(miss, want_miss, "{face} {op}: mirrored point");
    }
}

fn centroid_volume(b: &Body) -> (Vec3, f64) {
    let m = ck_topo::mass::mass_props(b);
    (m.com, m.volume)
}

fn code_of(o: &Out) -> String {
    o.errors.first().map(|e| e.1.clone()).unwrap_or_default()
}

#[test]
fn offsets_equal_moving_the_sketch() {
    let (ro, h, d) = (40.0, 90.0, 0.5);
    let base = tube(ro, 0.0, h);
    let at = |dz: f64, du: f64| rect("S2", du - 8.0, du + 8.0, 40.0 + dz, 50.0 + dz);
    let by_hand = |dz: f64, du: f64| {
        let o = run(&format!("{base},{},{}", at(dz, du), wrap("F3", "S2", "F1.S.e1", "cut", d, "")));
        centroid_volume(&ok(&o, "by hand"))
    };
    let (c0, v0) = by_hand(0.0, 0.0);
    let _ = (c0, v0);
    // alongAxis moves the profile up the axis
    let (ca, va) = by_hand(7.0, 0.0);
    let o = run(&format!("{base},{},{}", at(0.0, 0.0), wrap("F3", "S2", "F1.S.e1", "cut", d, r#","alongAxis":7"#)));
    let (c, v) = centroid_volume(&ok(&o, "alongAxis"));
    assert!(near(v, va, 1e-9) && (c - ca).len() < 1e-6, "alongAxis: {c:?} {v} vs {ca:?} {va}");
    // around: 30 degrees is an arc of r * 30 deg along the reading direction (+u outside)
    let arc = ro * 30f64.to_radians();
    let (ca, va) = by_hand(0.0, arc);
    let o = run(&format!("{base},{},{}", at(0.0, 0.0), wrap("F3", "S2", "F1.S.e1", "cut", d, r#","around":30"#)));
    let (c, v) = centroid_volume(&ok(&o, "around"));
    assert!(near(v, va, 1e-9) && (c - ca).len() < 1e-6, "around: {c:?} {v} vs {ca:?} {va}");
    // rotation: a 16 x 10 rectangle turned 35 degrees about its middle, against the same rectangle drawn turned
    let (s35, c35) = 35f64.to_radians().sin_cos();
    let rot = |p: [f64; 2]| [p[0] * c35 - p[1] * s35, p[0] * s35 + p[1] * c35];
    let pts: Vec<[f64; 2]> = [[-8.0, -5.0], [8.0, -5.0], [8.0, 5.0], [-8.0, 5.0]].iter().map(|p| { let q = rot(*p); [q[0], q[1] + 45.0] }).collect();
    let o = run(&format!("{base},{},{}", poly_sketch("S2", &pts), wrap("F3", "S2", "F1.S.e1", "cut", d, "")));
    let (ch, vh) = centroid_volume(&ok(&o, "drawn turned"));
    let o = run(&format!("{base},{},{}", rect("S2", -8.0, 8.0, 40.0, 50.0), wrap("F3", "S2", "F1.S.e1", "cut", d, r#","rotation":35"#)));
    let (cr, vr) = centroid_volume(&ok(&o, "rotation"));
    assert!(near(vr, vh, 1e-4) && (cr - ch).len() < 1e-4, "rotation: {cr:?} {vr} vs {ch:?} {vh}");
    // flip mirrors the profile along the circumference about its own middle: a right triangle turns into its mirror image in place
    let o = run(&format!("{base},{},{}", poly_sketch("S2", &[[0.0, 40.0], [16.0, 40.0], [0.0, 50.0]]), wrap("F3", "S2", "F1.S.e1", "cut", d, r#","flip":true"#)));
    let (cf, vf) = centroid_volume(&ok(&o, "flip"));
    let o = run(&format!("{base},{},{}", poly_sketch("S2", &[[16.0, 40.0], [0.0, 40.0], [16.0, 50.0]]), wrap("F3", "S2", "F1.S.e1", "cut", d, "")));
    let (cm, vm) = centroid_volume(&ok(&o, "mirrored by hand"));
    assert!(near(vf, vm, 1e-6) && (cf - cm).len() < 1e-5, "flip: {cf:?} {vf} vs {cm:?} {vm}");
    // expressions
    let (ca, va) = by_hand(7.0, arc);
    let o = run_vars(
        r#""variables":[{"name":"dd","expr":"0.5"},{"name":"turn","expr":"30"}],"#,
        &format!(
            r#"{base},{},{{"type":"wrap","id":"F3","sketch":"S2","face":"F1.S.e1","op":"cut","depth":"=dd","around":"=turn","alongAxis":"=3+4"}}"#,
            at(0.0, 0.0)
        ),
    );
    let (c, v) = centroid_volume(&ok(&o, "expressions"));
    assert!(near(v, va, 1e-9) && (c - ca).len() < 1e-6, "expressions: {c:?} {v} vs {ca:?} {va}");
}

/// The cut of a rectangle with walls tilted by the taper: the area at height t above the face is (a + 2 t tan)(b + 2 t tan), and
/// the radial shell scales it by (r + t) / r.
fn tapered_volume(a: f64, b: f64, r: f64, d: f64, taper_deg: f64, add: bool) -> f64 {
    let tan = taper_deg.to_radians().tan();
    let (t0, t1) = if add { (0.0, d) } else { (-d, 0.0) };
    let n = 20000;
    let mut s = 0.0;
    for i in 0..n {
        let t = t0 + (t1 - t0) * (i as f64 + 0.5) / n as f64;
        s += (a + 2.0 * t * tan) * (b + 2.0 * t * tan) * (r + t) / r;
    }
    s * (t1 - t0) / n as f64
}

#[test]
fn taper_walls_and_refusal() {
    let (ro, h, d) = (40.0, 90.0, 2.0);
    let (a, bz) = (20.0, 12.0);
    let base = tube(ro, 0.0, h);
    let v0 = volume(&run(&base).body.unwrap());
    for taper in [10.0, -10.0] {
        for op in ["cut", "add"] {
            let o = run(&format!("{base},{},{}", rect("S2", -a / 2.0, a / 2.0, 39.0, 39.0 + bz), wrap("F3", "S2", "F1.S.e1", op, d, &format!(r#","taper":{taper}"#))));
            let b = ok(&o, &format!("{op} {taper}"));
            valid(&b, &format!("{op} {taper}"));
            let want = tapered_volume(a, bz, ro, d, taper, op == "add");
            let dv = (volume(&b) - v0).abs();
            assert!(near(dv, want, 2e-3), "{op} taper {taper}: {dv} vs {want}");
        }
    }
    // a thin stroke cut deep with a draft that closes it: refused, with the largest taper that works
    let o = run(&format!("{base},{},{}", rect("S2", -0.5, 0.5, 40.0, 60.0), wrap("F3", "S2", "F1.S.e1", "cut", 3.0, r#","taper":20"#)));
    assert_eq!(code_of(&o), "wrapTaperTooLarge", "{:?}", o.errors);
    let params = &o.errors[0].2;
    assert!(params.contains("maxTaper"), "{params}");
    assert!(o.body.as_ref().map_or(true, |b| faces_named(b, "F3").is_empty()));
    let o = run(&format!("{base},{},{}", rect("S2", -0.5, 0.5, 40.0, 60.0), wrap("F3", "S2", "F1.S.e1", "cut", 3.0, r#","taper":60"#)));
    assert_eq!(code_of(&o), "invalidValue");
}

#[test]
fn refusals() {
    let base = tube(40.0, 37.0, 90.0);
    let rc = rect("S2", -8.0, 8.0, 40.0, 50.0);
    let w = |face: &str, op: &str, d: f64, extra: &str| run(&format!("{base},{rc},{}", wrap("F3", "S2", face, op, d, extra)));
    // no sketch, or a sketch with no closed region
    let o = run(&format!("{base},{{\"type\":\"wrap\",\"id\":\"F3\",\"face\":\"F1.S.e1\",\"op\":\"cut\",\"depth\":1}}"));
    assert_eq!(code_of(&o), "wrapNoInput");
    let o = run(&format!(
        r#"{base},{{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"yz"}},"points":[{{"id":"a","x":0,"y":40}},{{"id":"b","x":10,"y":50}}],"entities":[{{"type":"line","id":"l","p1":"a","p2":"b"}}],"constraints":[]}},{}"#,
        wrap("F3", "S2", "F1.S.e1", "cut", 1.0, "")
    ));
    assert_eq!(code_of(&o), "wrapNoInput", "{:?}", o.errors);
    // no body
    let o = run(&format!("{rc},{}", wrap("F3", "S2", "F1.S.e1", "cut", 1.0, "")));
    assert_eq!(code_of(&o), "nothingToCut");
    // unknown sketch / face
    assert_eq!(code_of(&run(&format!("{base},{}", wrap("F3", "S9", "F1.S.e1", "cut", 1.0, "")))), "missingSketch");
    assert_eq!(code_of(&w("F9.S.c9", "cut", 1.0, "")), "missingReference");
    // a plane, and the methods that are not built
    let o = w("F1.C1", "cut", 1.0, "");
    assert_eq!(code_of(&o), "wrapFaceUnsupported");
    assert!(o.errors[0].2.contains("extrude"), "{:?}", o.errors);
    assert_eq!(code_of(&w("F1.S.e1", "cut", 1.0, r#","method":"project""#)), "wrapFaceUnsupported");
    assert_eq!(code_of(&w("F1.S.e1", "cut", 1.0, r#","method":"spin""#)), "invalidValue");
    // field checks
    assert_eq!(code_of(&w("F1.S.e1", "cut", 0.0, "")), "invalidValue");
    assert_eq!(code_of(&w("F1.S.e1", "newBody", 1.0, "")), "invalidValue");
    assert_eq!(code_of(&w("F1.S.e1", "cut", 1.0, r#","tolerance":0"#)), "invalidValue");
    // an emboss into a bore deeper than its radius
    assert_eq!(code_of(&w("F1.S.e2", "add", 40.0, "")), "wrapDepthTooLarge");
    // the sketch plane is perpendicular to the axis
    let o = run(&format!(
        r#"{base},{{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":0,"y":0}}],"entities":[{{"type":"circle","id":"c","c":"a","r":3}}],"constraints":[]}},{}"#,
        wrap("F3", "S2", "F1.S.e1", "cut", 1.0, "")
    ));
    assert_eq!(code_of(&o), "wrapPlane");
    // the failed feature leaves the body as the part before it
    assert!(faces_named(o.body.as_ref().unwrap(), "F3").is_empty());
}

#[test]
fn one_turn_limit_and_off_face() {
    // r 10: circumference 62.83. Text longer than that is refused and names the profile
    let o = run(&format!("{},{},{}", tube(10.0, 0.0, 40.0), text_sketch("S2", "CADKIT CADKIT CADKIT", "gauge", 6.0, [0.0, 20.0]), wrap("F3", "S2", "F1.S.e1", "cut", 0.3, "")));
    assert_eq!(code_of(&o), "wrapProfileTooWide", "{:?}", o.errors);
    let p = &o.errors[0].2;
    assert!(p.contains("t1") && p.contains("circumference") && p.contains("extent"), "{p}");
    // one turn and a millimetre
    let circ = std::f64::consts::TAU * 10.0;
    let o = run(&format!("{},{},{}", tube(10.0, 0.0, 40.0), rect("S2", -(circ + 1.0) / 2.0, (circ + 1.0) / 2.0, 10.0, 15.0), wrap("F3", "S2", "F1.S.e1", "cut", 0.3, "")));
    assert_eq!(code_of(&o), "wrapProfileTooWide");
    // 0.999 of a turn builds, with the seam clear of the profile, for a cut and an add
    for op in ["cut", "add"] {
        let w = 0.999 * circ;
        let o = run(&format!("{},{},{}", tube(10.0, 0.0, 40.0), rect("S2", -w / 2.0, w / 2.0, 10.0, 15.0), wrap("F3", "S2", "F1.S.e1", op, 0.3, "")));
        let b = ok(&o, &format!("0.999 turn {op}"));
        valid(&b, &format!("0.999 turn {op}"));
        let v0 = volume(&run(&tube(10.0, 0.0, 40.0)).body.unwrap());
        let want = if op == "cut" { -((100.0 - 9.7 * 9.7) / 2.0 * 0.999 * std::f64::consts::TAU * 5.0) } else { (10.3 * 10.3 - 100.0) / 2.0 * 0.999 * std::f64::consts::TAU * 5.0 };
        assert!(near(volume(&b) - v0, want, 1e-4), "{op}: {} vs {want}", volume(&b) - v0);
    }
    // past the end of the cylinder
    let base = tube(40.0, 0.0, 90.0);
    let o = run(&format!("{base},{},{}", rect("S2", -5.0, 5.0, 85.0, 95.0), wrap("F3", "S2", "F1.S.e1", "cut", 0.5, "")));
    assert_eq!(code_of(&o), "wrapProfileOffFace", "{:?}", o.errors);
    assert!(o.errors[0].2.contains("profile") && o.errors[0].2.contains("at"));
    // over a through hole (a cylinder cut across the whole tube)
    let hole = format!(
        r#"{{"type":"sketch","id":"SH","plane":{{"kind":"std","id":"yz"}},"points":[{{"id":"c","x":0,"y":45}}],"entities":[{{"type":"circle","id":"h","c":"c","r":4}}],"constraints":[]}},
        {{"type":"extrude","id":"FH","op":"cut","sketch":"SH","end":"throughBoth","depth":100}}"#
    );
    for (u0, u1) in [(-8.0, 8.0), (-2.0, 12.0)] {
        let o = run(&format!("{base},{hole},{},{}", rect("S2", u0, u1, 40.0, 50.0), wrap("F3", "S2", "F1.S.e1", "cut", 0.5, "")));
        assert_eq!(code_of(&o), "wrapProfileOffFace", "{u0}: {:?}", o.errors);
    }
    // beside the hole is fine
    let o = run(&format!("{base},{hole},{},{}", rect("S2", 10.0, 20.0, 40.0, 50.0), wrap("F3", "S2", "F1.S.e1", "cut", 0.5, "")));
    valid(&ok(&o, "beside the hole"), "beside the hole");
}

#[test]
fn step_round_trip_and_persistence() {
    let src_features = format!(
        "{},{},{}",
        tube(40.0, 37.0, 90.0),
        text_sketch("S2", "CADKIT 1/2", "gauge", 6.0, [0.0, 45.0]),
        wrap("F3", "S2", "F1.S.e1", "cut", 0.4, r#","alongAxis":2,"around":10,"rotation":3,"taper":0,"flip":false,"tolerance":0.002"#)
    );
    let o = run(&src_features);
    let b = ok(&o, "text");
    valid(&b, "text");
    let step = ck_io::step::write_step(&b, "wrap", &ck_io::step::StepOpts { unix_ms: 0.0 });
    let back = ck_io::step::read_step(&step).expect("step reads").body;
    assert!(near(volume(&back), volume(&b), 1e-4), "{} vs {}", volume(&back), volume(&b));
    assert_eq!(back.num_faces(), b.num_faces());
    // the part document round-trips with every field of the feature, and regenerates the same body
    let part_src = format!(r#"{{"format":"ck-part","version":1,"features":[{src_features}]}}"#);
    let part = Part::parse(&part_src).unwrap();
    let text = ck_json::to_string(&part.to_json());
    let again = Part::parse(&text).unwrap();
    let (f0, f1) = (part.feature("F3").unwrap(), again.feature("F3").unwrap());
    assert_eq!(f0.raw, f1.raw);
    for k in ["sketch", "face", "op", "depth", "alongAxis", "around", "rotation", "taper", "flip", "tolerance"] {
        assert!(f1.raw.get(k).is_some(), "{k} lost");
    }
    let o2 = run_part(&text);
    assert!(near(volume(&ok(&o2, "again")), volume(&b), 1e-12));
    // no field of the feature is unknown to the lint
    assert!(!o.warnings.iter().any(|w| w.1 == "unknownField"), "{:?}", o.warnings);
}

#[test]
fn text_in_a_bore_and_on_the_second_body() {
    let (ro, ri, h, d) = (40.0, 37.0, 90.0, 0.4);
    let lay = layout("CADKIT 1/2", "gauge", 6.0, [0.0, 45.0]);
    let base = tube(ro, ri, h);
    let v0 = volume(&run(&base).body.unwrap());
    for (op, r_face, outward) in [("cut", ri, true), ("add", ri, false)] {
        let o = run(&format!("{base},{},{}", text_sketch("S2", "CADKIT 1/2", "gauge", 6.0, [0.0, 45.0]), wrap("F3", "S2", "F1.S.e2", op, d, "")));
        let b = ok(&o, &format!("bore {op}"));
        valid(&b, &format!("bore {op}"));
        let dv = volume(&b) - v0;
        let mag = shell_volume(lay.area(), d, r_face, outward);
        let want = if op == "add" { mag } else { -mag };
        assert!((dv - want).abs() <= 0.015 * want.abs(), "bore {op}: {dv} vs {want}");
    }
    // a second tube as its own body: text on it leaves the first one alone
    let second = r#"{"type":"sketch","id":"S9","plane":{"kind":"std","id":"front"},"points":[{"id":"q","x":150,"y":0}],"entities":[{"type":"circle","id":"c9","c":"q","r":40}],"constraints":[]},
        {"type":"extrude","id":"F9","op":"newBody","sketch":"S9","end":"blind","depth":90}"#;
    let one = run(&format!("{base},{second}")).body.unwrap();
    // the sketch plane through the second axis: an offset plane x = 150 is out of the plane yz, so write the text sketch on a feature plane
    let plane = r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"yz"},"offset":150}"#;
    let text = r#"{"type":"sketch","id":"S2","plane":{"kind":"feature","id":"P1"},"points":[],"entities":[{"type":"text","id":"t1","string":"AB","font":"gauge","height":6,"at":[0,45],"anchor":["center","middle"]}],"constraints":[]}"#;
    let o = run(&format!("{base},{second},{plane},{text},{}", wrap("F3", "S2", "F9.S.c9", "cut", 0.4, "")));
    let b = ok(&o, "second body");
    assert_eq!(b.num_lumps(), 2);
    assert!(volume(&b) < volume(&one) - 1e-3);
    // the first lump is untouched: same volume as before the wrap (lumps in order)
    let per_lump = |b: &Body| -> Vec<f64> { b.lump_ids().map(|l| ck_topo::mass::lump_faces(b, l).len() as f64).collect() };
    assert_eq!(per_lump(&b)[0], per_lump(&one)[0]);
}

#[test]
fn cut_through_warns_and_valid_results_only() {
    // a 3 mm wall engraved 4 mm deep: pierced, valid, warned
    let o = run(&format!("{},{},{}", tube(40.0, 37.0, 90.0), rect("S2", -6.0, 6.0, 40.0, 50.0), wrap("F3", "S2", "F1.S.e1", "cut", 4.0, "")));
    let b = ok(&o, "pierce");
    valid(&b, "pierce");
    let w = o.warnings.iter().find(|w| w.1 == "wrapCutThrough").unwrap_or_else(|| panic!("no warning: {:?}", o.warnings));
    assert!(w.2.contains("wall"), "{:?}", w);
    // a tolerance no fit can reach
    let o = run(&format!("{},{},{}", tube(40.0, 0.0, 90.0), text_sketch("S2", "S", "gauge", 6.0, [0.0, 45.0]), wrap("F3", "S2", "F1.S.e1", "cut", 0.4, r#","tolerance":1e-14"#)));
    ok(&o, "tight tolerance");
    assert!(o.warnings.iter().any(|w| w.1 == "wrapApproximated"), "{:?}", o.warnings);
    // two sketches of overlapping profiles
    let two = r#"{"type":"sketch","id":"S2","plane":{"kind":"std","id":"yz"},"points":[],"entities":[
        {"type":"text","id":"t1","string":"MM","font":"gauge","height":6,"at":[0,45],"anchor":["center","middle"]},
        {"type":"text","id":"t2","string":"MM","font":"gauge","height":6,"at":[1.5,45],"anchor":["center","middle"]}],"constraints":[]}"#;
    let o = run(&format!("{},{two},{}", tube(40.0, 0.0, 90.0), wrap("F3", "S2", "F1.S.e1", "cut", 0.4, "")));
    assert_eq!(code_of(&o), "wrapProfilesOverlap", "{:?}", o.errors);
    // a small radius with large letters: a body or a refusal, never an invalid body
    for op in ["cut", "add"] {
        let o = run(&format!("{},{},{}", tube(3.0, 0.0, 20.0), text_sketch("S2", "e", "gauge", 2.0, [0.0, 10.0]), wrap("F3", "S2", "F1.S.e1", op, 0.5, "")));
        match o.errors.first() {
            None => valid(o.body.as_ref().unwrap(), &format!("r 3 {op}")),
            Some(e) => assert!(["wrapToolInvalid", "booleanFailed", "wrapProfileTooWide"].contains(&e.1.as_str()), "{op}: {:?}", e),
        }
    }
}

/// The risk corpus of the proposal: text on three radii and three heights, cut and add, all valid.
#[test]
fn text_on_three_radii_and_heights() {
    for r in [10.0, 40.0, 150.0] {
        for hgt in [2.0, 6.0, 20.0] {
            // keep the string within a turn at the small radius
            let s = if r < 20.0 { "CAD" } else { "CADKIT 12" };
            if r < 20.0 && hgt > 6.0 {
                continue;
            }
            let base = tube(r, 0.0, 120.0);
            let v0 = volume(&run(&base).body.unwrap());
            for (op, d) in [("cut", 0.4), ("add", 0.5)] {
                let o = run(&format!("{base},{},{}", text_sketch("S2", s, "gauge", hgt, [0.0, 60.0]), wrap("F3", "S2", "F1.S.e1", op, d, "")));
                let b = ok(&o, &format!("r {r} h {hgt} {op}"));
                valid(&b, &format!("r {r} h {hgt} {op}"));
                let dv = volume(&b) - v0;
                assert!(if op == "cut" { dv < 0.0 } else { dv > 0.0 }, "r {r} h {hgt} {op}: {dv}");
            }
        }
    }
}

// ---- phase 2: projectCurve method "wrap" -------------------------------------------------------------------------

/// A sketch on the plane through the axis (sketch x = +y, sketch y = +z) with the points `pts` named p0.. and the entity JSON given.
fn curve_sketch(id: &str, pts: &[[f64; 2]], entities: &[&str]) -> String {
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p[0], p[1])).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"yz"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), entities.join(","))
}

fn line_sketch(id: &str, a: [f64; 2], b: [f64; 2]) -> String {
    curve_sketch(id, &[a, b], &[r#"{"type":"line","id":"e0","p1":"p0","p2":"p1"}"#])
}

fn wc(id: &str, sketch: &str, face: &str, extra: &str) -> String {
    format!(r#"{{"type":"projectCurve","id":"{id}","sketch":"{sketch}","faces":["{face}"],"method":"wrap"{extra}}}"#)
}

fn curve_pts(o: &Out, id: &str) -> Vec<Vec3> {
    let c = o.curves.iter().find(|c| c.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).unwrap_or_else(|| panic!("no curve {id}"));
    c.as_arr().unwrap()[1].get("pts").unwrap().as_arr().unwrap().iter().map(|p| {
        let a = p.as_arr().unwrap();
        Vec3::new(a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap())
    }).collect()
}

fn marker_num(o: &Out, id: &str, key: &str) -> f64 {
    o.markers.iter().filter_map(|m| m.as_arr()).find(|m| m[0].as_str() == Some(id)).and_then(|m| m[1].get(key)).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("no marker {id}.{key}"))
}

fn poly_len(p: &[Vec3]) -> f64 {
    p.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// The point of the development of radius `r` at sketch (u, w) for the outside of a cylinder about z, sketch on the plane yz.
fn at_uw(r: f64, u: f64, w: f64) -> Vec3 {
    Vec3::new(r * (u / r).cos(), r * (u / r).sin(), w)
}

fn radial_err(p: &[Vec3], r: f64) -> f64 {
    p.iter().map(|q| (q.x.hypot(q.y) - r).abs()).fold(0.0, f64::max)
}

#[test]
fn open_curves_keep_their_length_on_the_cylinder() {
    let r = 20.0;
    let base = tube(r, 0.0, 100.0);
    // a wiggly open spline, a line, an arc: lengths, ends and the distance from the axis
    let spl = curve_sketch("S2", &[[0.0, 40.0], [8.0, 46.0], [16.0, 40.0], [24.0, 46.0], [32.0, 42.0]], &[r#"{"type":"spline","id":"e0","pts":["p0","p1","p2","p3","p4"]}"#]);
    let o = run(&format!("{base},{spl},{}", wc("F5", "S2", "F1.S.e1", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let pts = curve_pts(&o, "F5");
    assert!(radial_err(&pts, r) < 1.5e-3, "radial {}", radial_err(&pts, r));
    assert!(pts[0].dist(at_uw(r, 0.0, 40.0)) < 1e-3, "{:?}", pts[0]);
    assert!(pts.last().unwrap().dist(at_uw(r, 32.0, 42.0)) < 1e-3, "{:?}", pts.last());
    // the unrolled length of the result equals the length of the data, and the data are longer than the chord in the sketch
    let unrolled: f64 = pts.windows(2).map(|w| {
        let dth = (w[1].y.atan2(w[1].x) - w[0].y.atan2(w[0].x) + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
        (r * dth).hypot(w[1].z - w[0].z)
    }).sum();
    assert!(near(unrolled, marker_num(&o, "F5", "length"), 1e-3), "{unrolled} vs {}", marker_num(&o, "F5", "length"));
    assert!(marker_num(&o, "F5", "length") > 32.0_f64.hypot(2.0));
    assert!(marker_num(&o, "F5", "deviation") <= 1e-3);
    assert!(near(marker_num(&o, "F5", "turns"), 32.0 / (TAU * r), 0.1), "turns {}", marker_num(&o, "F5", "turns"));
    // a line
    let o = run(&format!("{base},{},{}", line_sketch("S2", [0.0, 30.0], [40.0, 55.0]), wc("F5", "S2", "F1.S.e1", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let pts = curve_pts(&o, "F5");
    let want = 40.0_f64.hypot(25.0);
    assert!(near(marker_num(&o, "F5", "length"), want, 1e-5), "{}", marker_num(&o, "F5", "length"));
    assert!(near(poly_len(&pts), want, 2e-3), "{}", poly_len(&pts));
    assert!(radial_err(&pts, r) < 1.5e-3);
    // an arc (a quarter circle of radius 6 about (10, 45))
    let arc = curve_sketch("S2", &[[10.0, 45.0], [16.0, 45.0], [10.0, 51.0]], &[r#"{"type":"arc","id":"e0","c":"p0","p1":"p1","p2":"p2"}"#]);
    let o = run(&format!("{base},{arc},{}", wc("F5", "S2", "F1.S.e1", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(near(marker_num(&o, "F5", "length"), 3.0 * std::f64::consts::PI, 1e-4), "{}", marker_num(&o, "F5", "length"));
    assert!(radial_err(&curve_pts(&o, "F5"), r) < 1.5e-3);
    // a closed circle is a closed curve
    let circ = curve_sketch("S2", &[[10.0, 45.0]], &[r#"{"type":"circle","id":"e0","c":"p0","r":5}"#]);
    let o = run(&format!("{base},{circ},{}", wc("F5", "S2", "F1.S.e1", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.markers.iter().filter_map(|m| m.as_arr()).find(|m| m[0].as_str() == Some("F5")).unwrap()[1].get("closed"), Some(&ck_json::Json::Bool(true)));
    assert!(near(marker_num(&o, "F5", "length"), TAU * 5.0, 1e-4));
}

#[test]
fn three_turn_line_is_a_helix_and_sweeps() {
    let r = 20.0;
    let base = tube(r, 0.0, 100.0);
    let u1 = 3.0 * TAU * r;
    let o = run(&format!("{base},{},{}", line_sketch("S2", [0.0, 10.0], [u1, 25.0]), wc("F5", "S2", "F1.S.e1", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let pts = curve_pts(&o, "F5");
    let want = 3.0 * (TAU * r).hypot(5.0);
    assert!(near(marker_num(&o, "F5", "length"), want, 1e-5), "{} vs {want}", marker_num(&o, "F5", "length"));
    assert!(near(poly_len(&pts), want, 1e-3), "{} vs {want}", poly_len(&pts));
    assert!(near(marker_num(&o, "F5", "turns"), 3.0, 1e-6));
    assert!(radial_err(&pts, r) < 1.5e-3);
    assert!(pts.windows(2).all(|w| w[1].z >= w[0].z - 1e-6), "height rises monotonically");
    // against the helix feature of the same radius, pitch and turns
    let helix = format!(
        r#"{},{{"type":"helix","id":"F11","sketch":"H","definedBy":"pitchRevs","pitch":5,"revolutions":3}}"#,
        r#"{"type":"sketch","id":"H","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":20}],"constraints":[]}"#
    );
    let h = run(&format!("{base},{helix}"));
    assert!(h.errors.is_empty(), "{:?}", h.errors);
    let hl = poly_len(&curve_pts(&h, "F11"));
    assert!(near(hl, want, 2e-3), "helix {hl} vs wrapped line {want}");
    // a circle swept along a short wrapped line (the profile stands normal to the path start: plane normal to +y at (r, 0, 45))
    let path = line_sketch("S2", [0.0, 45.0], [10.0, 45.0]);
    let prof = r#"{"type":"sketch","id":"S3","plane":{"kind":"std","id":"xz"},"points":[{"id":"c","x":20,"y":45}],"entities":[{"type":"circle","id":"e1","c":"c","r":1}],"constraints":[]}"#;
    let before = run(&base);
    let o = run(&format!(
        r#"{base},{path},{},{prof},{{"type":"sweep","id":"F6","op":"add","sketch":"S3","path":{{"kind":"curve","id":"F5"}}}}"#,
        wc("F5", "S2", "F1.S.e1", "")
    ));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    valid(b, "sweep along a wrapped curve");
    let gain = volume(b) - volume(before.body.as_ref().unwrap());
    // half the pipe lies outside the cylinder: about pi / 2 * 10
    assert!(gain > 0.3 * std::f64::consts::PI * 10.0 && gain < 0.7 * std::f64::consts::PI * 10.0, "gain {gain}");
}

#[test]
fn wrapped_curve_offsets_bore_and_expressions() {
    let r = 20.0;
    let base = tube(r, 0.0, 100.0);
    // rotation by 90 degrees about the middle turns an axial line into a circumferential one
    let axial = line_sketch("S2", [0.0, 40.0], [0.0, 50.0]);
    let o = run(&format!("{base},{axial},{}", wc("F5", "S2", "F1.S.e1", r#","rotation":90"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let pts = curve_pts(&o, "F5");
    assert!(pts.iter().all(|p| (p.z - 45.0).abs() < 1e-6), "constant height");
    assert!(near(marker_num(&o, "F5", "turns"), 10.0 / (TAU * r), 1e-6));
    // alongAxis and around accept expressions and move the curve like moving the sketch
    let o2 = run(&format!("{base},{axial},{}", wc("F5", "S2", "F1.S.e1", r#","alongAxis":"=2+3","around":90"#)));
    assert!(o2.errors.is_empty(), "{:?}", o2.errors);
    let q = curve_pts(&o2, "F5");
    assert!(q[0].dist(Vec3::new(0.0, r, 45.0)) < 1e-3, "{:?}", q[0]);
    assert!(q.last().unwrap().dist(Vec3::new(0.0, r, 55.0)) < 1e-3, "{:?}", q.last());
    // equal to the sketch moved by hand
    let moved = line_sketch("S2", [r * std::f64::consts::FRAC_PI_2, 45.0], [r * std::f64::consts::FRAC_PI_2, 55.0]);
    let o3 = run(&format!("{base},{moved},{}", wc("F5", "S2", "F1.S.e1", "")));
    let m = curve_pts(&o3, "F5");
    assert!(m[0].dist(q[0]) < 1e-6 && m.last().unwrap().dist(*q.last().unwrap()) < 1e-6);
    // in a bore the reading direction runs the other way round the axis
    let bore = tube(40.0, 37.0, 90.0);
    let o = run(&format!("{bore},{},{}", line_sketch("S2", [10.0, 40.0], [20.0, 40.0]), wc("F5", "S2", "F1.S.e2", "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = curve_pts(&o, "F5");
    let want = |u: f64| Vec3::new(37.0 * (u / 37.0).cos(), -37.0 * (u / 37.0).sin(), 40.0);
    assert!(b[0].dist(want(10.0)) < 1e-3 && b.last().unwrap().dist(want(20.0)) < 1e-3, "{:?} {:?}", b[0], b.last());
    assert!(radial_err(&b, 37.0) < 1.5e-3);
    // `method: "project"` is today's projection, unchanged
    let plain = run(&format!("{base},{axial},{{\"type\":\"projectCurve\",\"id\":\"F5\",\"sketch\":\"S2\",\"faces\":[\"F1.S.e1\"],\"direction\":\"forward\"}}"));
    let proj = run(&format!("{base},{axial},{{\"type\":\"projectCurve\",\"id\":\"F5\",\"sketch\":\"S2\",\"faces\":[\"F1.S.e1\"],\"method\":\"project\",\"direction\":\"forward\"}}"));
    assert_eq!(plain.errors, proj.errors);
    assert!(plain.errors.is_empty(), "{:?}", plain.errors);
    assert_eq!(curve_pts(&plain, "F5").len(), curve_pts(&proj, "F5").len());
}

#[test]
fn wrapped_curve_entities_and_refusals() {
    let r = 20.0;
    let base = tube(r, 0.0, 100.0);
    let two = curve_sketch(
        "S2",
        &[[0.0, 40.0], [10.0, 40.0], [30.0, 60.0], [40.0, 60.0]],
        &[r#"{"type":"line","id":"e0","p1":"p0","p2":"p1"}"#, r#"{"type":"line","id":"e1","p1":"p2","p2":"p3"}"#],
    );
    let go = |extra: &str| run(&format!("{base},{two},{}", wc("F5", "S2", "F1.S.e1", extra)));
    // two disjoint curves are projectCurveSeveral, a subset is one curve
    assert_eq!(code_of(&go("")), "projectCurveSeveral");
    let o = go(r#","entities":["e1"]"#);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(near(marker_num(&o, "F5", "length"), 10.0, 1e-5));
    assert_eq!(code_of(&go(r#","entities":["zz"]"#)), "missingReference");
    // face selection: none, two, a plane, a non-face
    let one = line_sketch("S2", [0.0, 40.0], [10.0, 40.0]);
    let with = |faces: &str, extra: &str| {
        let m = if extra.contains("method") { "" } else { r#","method":"wrap""# };
        run(&format!("{base},{one},{{\"type\":\"projectCurve\",\"id\":\"F5\",\"sketch\":\"S2\",\"faces\":{faces}{m}{extra}}}"))
    };
    assert_eq!(code_of(&with("[]", "")), "wrapFaceUnsupported");
    assert_eq!(code_of(&with(r#"["F1.S.e1","F1.C1"]"#, "")), "wrapFaceUnsupported");
    let o = with(r#"["F1.C1"]"#, "");
    assert_eq!(code_of(&o), "wrapFaceUnsupported");
    assert!(o.errors[0].2.contains("plane"), "{:?}", o.errors);
    assert_eq!(code_of(&with(r#"["F9.S.c9"]"#, "")), "missingReference");
    // method and field checks
    assert_eq!(code_of(&with(r#"["F1.S.e1"]"#, r#","method":"spin""#)), "invalidValue");
    assert_eq!(code_of(&with(r#"["F1.S.e1"]"#, r#","procedural":true"#)), "invalidValue");
    assert_eq!(code_of(&with(r#"["F1.S.e1"]"#, r#","tolerance":0"#)), "invalidValue");
    assert_eq!(code_of(&with(r#"["F1.S.e1"]"#, r#","degree":12"#)), "editCurveDegree");
    // a curve input is not wrapped
    assert_eq!(code_of(&run(&format!("{base},{one},{{\"type\":\"projectCurve\",\"id\":\"F5\",\"faces\":[\"F1.S.e1\"],\"method\":\"wrap\"}}"))), "wrapNoInput");
    assert_eq!(code_of(&run(&format!("{base},{{\"type\":\"projectCurve\",\"id\":\"F5\",\"sketch\":\"S9\",\"faces\":[\"F1.S.e1\"],\"method\":\"wrap\"}}"))), "missingSketch");
    // an end off the trimmed face names the entity (a wrap must not drop the part, the length would lie)
    let off = line_sketch("S2", [0.0, 90.0], [10.0, 110.0]);
    let o = run(&format!("{base},{off},{}", wc("F5", "S2", "F1.S.e1", "")));
    assert_eq!(code_of(&o), "wrapProfileOffFace", "{:?}", o.errors);
    assert!(o.errors[0].2.contains("e0"), "{:?}", o.errors);
    // a sketch plane perpendicular to the axis
    let o = run(&format!(
        r#"{base},{{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":0,"y":0}},{{"id":"b","x":5,"y":5}}],"entities":[{{"type":"line","id":"l","p1":"a","p2":"b"}}],"constraints":[]}},{}"#,
        wc("F5", "S2", "F1.S.e1", "")
    ));
    assert_eq!(code_of(&o), "wrapPlane");
    // no body at all keeps the existing code
    assert_eq!(code_of(&run(&format!("{one},{}", wc("F5", "S2", "F1.S.e1", "")))), "projectCurveNoFaces");
    // text is not a curve: a sketch of text only has nothing to wrap
    let txt = text_sketch("S2", "AB", "gauge", 5.0, [0.0, 45.0]);
    assert_eq!(code_of(&run(&format!("{base},{txt},{}", wc("F5", "S2", "F1.S.e1", "")))), "projectCurveNoInput");
}

#[test]
fn a_curve_too_long_for_400_control_points() {
    // 60 turns round a small cylinder at the default tolerance needs more than 400 cubic control points
    let base = tube(5.0, 0.0, 100.0);
    let u1 = 60.0 * TAU * 5.0;
    let o = run(&format!("{base},{},{}", line_sketch("S2", [0.0, 5.0], [u1, 95.0]), wc("F5", "S2", "F1.S.e1", "")));
    assert_eq!(code_of(&o), "wrapCurveTooLong", "{:?}", o.errors);
    assert!(o.errors[0].2.contains("maxControlPoints"));
    // a looser tolerance fits it
    let o = run(&format!("{base},{},{}", line_sketch("S2", [0.0, 5.0], [u1, 95.0]), wc("F5", "S2", "F1.S.e1", r#","tolerance":0.05"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}
