//! Rule engine (`checkProfile`) on small known parts: issues, values and persistent face names.

use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use ck_ops::profile::{profiles_reply, run, select_profile, RunOpts};
use ck_topo::body::Body;

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

fn build(features: &str) -> Body {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    ex.brep.body.expect("body")
}

/// Closed polygon sketch on `plane` with ids F<n> / p<i> / e<i>.
fn poly(id: &str, plane: &str, pts: &[(f64, f64)]) -> String {
    let n = pts.len();
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p.0, p.1)).collect();
    let lines: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{i}","p1":"p{i}","p2":"p{}"}}"#, (i + 1) % n)).collect();
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"{plane}"}},"points":[{}],"entities":[{}],"constraints":[]}}"#,
        points.join(","),
        lines.join(",")
    )
}
fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    poly(id, plane, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}
fn circle(id: &str, plane: &str, x: f64, y: f64, r: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"{plane}"}},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"e0","c":"c","r":{r}}}],"constraints":[]}}"#
    )
}
fn ext(id: &str, sk: &str, op: &str, depth: f64, extra: &str) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"{op}","sketch":"{sk}","end":"blind","depth":{depth}{extra}}}"#)
}

struct Found {
    rule: String,
    severity: String,
    faces: Vec<String>,
    edges: Vec<String>,
    value: f64,
    limit: f64,
    message: String,
}

fn check(body: &Body, profile: &str, rules: &str) -> Vec<Found> {
    let pj = if rules.is_empty() {
        Json::from(profile)
    } else {
        ck_json::parse(&format!(r#"{{"id":"t","name":"t","rules":[{rules}]}}"#)).unwrap()
    };
    let prof = select_profile(Some(&pj)).unwrap();
    let out = run(body, &prof, &RunOpts::from_json(None)).unwrap();
    let names: Vec<String> = body.face_ids().map(|f| body.face_id_str(f)).collect();
    out.get_arr("issues")
        .unwrap()
        .iter()
        .map(|i| {
            let strs = |k: &str| i.get_arr(k).unwrap().iter().map(|s| s.as_str().unwrap().to_string()).collect::<Vec<_>>();
            let f = Found {
                rule: i.get_str("rule").unwrap().into(),
                severity: i.get_str("severity").unwrap().into(),
                faces: strs("faces"),
                edges: strs("edges"),
                value: i.get_f64("value").unwrap_or(f64::NAN),
                limit: i.get_f64("limit").unwrap_or(f64::NAN),
                message: i.get_str("message").unwrap().into(),
            };
            for n in &f.faces {
                assert!(names.contains(n), "issue names unknown face {n}: {}", f.message);
            }
            f
        })
        .collect()
}

fn of<'a>(v: &'a [Found], rule: &str) -> Vec<&'a Found> {
    v.iter().filter(|f| f.rule == rule).collect()
}

#[test]
fn catalogue_lists_profiles_with_parameters() {
    let j = profiles_reply();
    let ps = j.get_arr("profiles").unwrap();
    let ids: Vec<&str> = ps.iter().map(|p| p.get_str("id").unwrap()).collect();
    assert_eq!(ids, ["fdm-print", "cnc-3axis", "injection"]);
    let fdm = &ps[0];
    let rules = fdm.get_arr("rules").unwrap();
    let wall = rules.iter().find(|r| r.get_str("id") == Some("minWall")).unwrap();
    let p = &wall.get_arr("params").unwrap()[0];
    assert_eq!(p.get_str("name"), Some("min"));
    assert_eq!(p.get_f64("value"), Some(1.2));
    assert_eq!(p.get_str("unit"), Some("mm"));
    assert!(j.get_arr("rules").unwrap().len() >= 11);
    // unknown profile / rule / parameter are errors that list the choices
    assert!(select_profile(Some(&Json::from("nope"))).unwrap_err().contains("fdm-print"));
    let bad = ck_json::parse(r#"{"rules":[{"id":"minWall","params":{"thick":1}}]}"#).unwrap();
    let prof = select_profile(Some(&bad)).unwrap();
    let body = build(&format!("{},{}", rect("F1", "front", 0.0, 0.0, 5.0, 5.0), ext("F2", "F1", "add", 5.0, "")));
    assert!(run(&body, &prof, &RunOpts::from_json(None)).unwrap_err().contains("unknown parameter"));
    let bad = ck_json::parse(r#"{"rules":[{"id":"zzz"}]}"#).unwrap();
    assert!(run(&body, &select_profile(Some(&bad)).unwrap(), &RunOpts::from_json(None)).unwrap_err().contains("unknown rule"));
}

#[test]
fn thin_walled_box_reports_the_half_millimetre_walls() {
    // 20x20x10 box (z 0..10) with a pocket from below leaving 0.5 mm side walls and a 0.5 mm roof
    let body = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 20.0, 20.0),
        ext("F2", "F1", "add", 10.0, ""),
        rect("F3", "front", 0.5, 0.5, 19.5, 19.5),
        ext("F4", "F3", "cut", 9.5, r#","flip":true"#)
    ));
    let v = check(&body, "fdm-print", "");
    let walls = of(&v, "minWall");
    assert!(walls.len() >= 5, "{}", walls.len());
    for w in &walls {
        assert!((w.value - 0.5).abs() < 0.01, "{}", w.message);
        assert_eq!(w.limit, 1.2);
        assert_eq!(w.severity, "error");
        assert_eq!(w.faces.len(), 2);
    }
    // the roof pair is the outer top face with the pocket ceiling
    assert!(walls.iter().any(|w| w.faces.iter().any(|f| f.contains(".C"))), "{:?}", walls.iter().map(|w| &w.faces).collect::<Vec<_>>());
    // thick enough: no issue
    let ok = check(&body, "", r#"{"id":"minWall","params":{"min":0.4}}"#);
    assert!(of(&ok, "minWall").is_empty());
    // it is also an undercut for a tool coming from +Z (roof underside)
    let cnc = check(&body, "cnc-3axis", "");
    assert!(!of(&cnc, "undercut").is_empty());
    assert!(of(&cnc, "undercut").iter().all(|u| u.severity == "error"));
}

#[test]
fn watertight_single_solid_is_clean() {
    let body = build(&format!("{},{}", rect("F1", "front", 0.0, 0.0, 20.0, 20.0), ext("F2", "F1", "add", 10.0, "")));
    let v = check(&body, "fdm-print", "");
    assert!(of(&v, "openShell").is_empty());
    assert!(of(&v, "bodyCount").is_empty());
    assert!(of(&v, "minWall").is_empty());
    assert!(of(&v, "overhang").is_empty());
    assert!(of(&v, "minFeature").is_empty());
    // two separate boxes
    let two = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 10.0, 10.0),
        ext("F2", "F1", "add", 10.0, ""),
        rect("F3", "front", 30.0, 0.0, 40.0, 10.0),
        ext("F4", "F3", "add", 10.0, "")
    ));
    let v = check(&two, "fdm-print", "");
    let bc = of(&v, "bodyCount");
    assert_eq!(bc.len(), 1);
    assert_eq!(bc[0].value, 2.0);
    assert_eq!(bc[0].faces.len(), 2);
}

#[test]
fn overhang_follows_the_chamfer_angle_and_ignores_the_plate() {
    // profile in XY (Y is up): the slanted underside runs (10,10) -> (20,20), 45 deg from vertical
    let prof = poly("F1", "front", &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (20.0, 20.0), (0.0, 20.0)]);
    let body = build(&format!("{},{}", prof, ext("F2", "F1", "add", 10.0, "")));
    // exactly at the 45 degree limit: allowed
    assert!(of(&check(&body, "fdm-print", ""), "overhang").is_empty());
    // a 30 degree limit flags the chamfer, with its measured angle, and not the bottom (build plate)
    let v = check(&body, "", r#"{"id":"overhang","params":{"angle":30}}"#);
    let o = of(&v, "overhang");
    assert_eq!(o.len(), 1, "{:?}", o.iter().map(|x| &x.message).collect::<Vec<_>>());
    assert!((o[0].value - 45.0).abs() < 0.5, "{}", o[0].value);
    assert_eq!(o[0].limit, 30.0);
    assert_eq!(o[0].faces.len(), 1);
    // building along -X instead makes other faces overhang; the build direction is a parameter
    let v = check(&body, "", r#"{"id":"overhang","params":{"direction":[-1,0,0],"angle":30}}"#);
    assert!(!of(&v, "overhang").is_empty());
    // a flat ceiling is a 90 degree overhang and a bridge
    let ceil = poly("F1", "front", &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (30.0, 10.0), (30.0, 20.0), (0.0, 20.0)]);
    let body = build(&format!("{},{}", ceil, ext("F2", "F1", "add", 30.0, "")));
    let v = check(&body, "fdm-print", "");
    let o = of(&v, "overhang");
    assert_eq!(o.len(), 1);
    assert!((o[0].value - 90.0).abs() < 0.01);
    let b = of(&v, "bridge");
    assert_eq!(b.len(), 1);
    assert!((b[0].value - 20.0).abs() < 0.1, "{}", b[0].value);
    assert_eq!(b[0].faces, o[0].faces);
    // 20 mm span is fine with a 25 mm limit
    assert!(of(&check(&body, "", r#"{"id":"bridge","params":{"maxSpan":25}}"#), "bridge").is_empty());
}

#[test]
fn plate_contact_warns_on_a_small_footprint() {
    let body = build(&format!("{},{}", rect("F1", "front", 0.0, 0.0, 2.0, 30.0), ext("F2", "F1", "add", 2.0, "")));
    // 2 x 2 mm footprint when standing on its short end (Y up): area 4 < 20
    let v = check(&body, "fdm-print", "");
    let p = of(&v, "plateContact");
    assert_eq!(p.len(), 1);
    assert!((p[0].value - 4.0).abs() < 1e-6, "{}", p[0].value);
    assert_eq!(p[0].severity, "warn");
    assert_eq!(p[0].faces.len(), 1);
}

#[test]
fn small_holes_pins_and_gaps_use_exact_radii() {
    let hole = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 20.0, 20.0),
        ext("F2", "F1", "add", 10.0, ""),
        circle("F3", "front", 10.0, 10.0, 0.25),
        ext("F4", "F3", "cut", 10.0, r#","flip":true"#)
    ));
    let v = check(&hole, "fdm-print", "");
    let m = of(&v, "minFeature");
    assert_eq!(m.len(), 1, "{:?}", m.iter().map(|x| &x.message).collect::<Vec<_>>());
    assert!((m[0].value - 0.5).abs() < 1e-9);
    assert_eq!(m[0].limit, 0.8);
    assert!(m[0].message.starts_with("Hole"), "{}", m[0].message);
    assert_eq!(m[0].faces.len(), 1);
    assert!(m[0].faces[0].starts_with("F4"), "{:?}", m[0].faces);
    // a 1 mm hole passes
    let ok = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 20.0, 20.0),
        ext("F2", "F1", "add", 10.0, ""),
        circle("F3", "front", 10.0, 10.0, 0.5),
        ext("F4", "F3", "cut", 10.0, r#","flip":true"#)
    ));
    assert!(of(&check(&ok, "fdm-print", ""), "minFeature").is_empty());
    // a 0.4 mm wide slot is a gap between two parallel walls
    let slot = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 20.0, 20.0),
        ext("F2", "F1", "add", 10.0, ""),
        rect("F3", "front", 5.0, 0.0, 5.4, 15.0),
        ext("F4", "F3", "cut", 10.0, r#","flip":true"#)
    ));
    let m = check(&slot, "fdm-print", "");
    let g = of(&m, "minFeature");
    assert_eq!(g.len(), 1, "{:?}", g.iter().map(|x| &x.message).collect::<Vec<_>>());
    assert!((g[0].value - 0.4).abs() < 1e-3, "{}", g[0].value);
    assert_eq!(g[0].faces.len(), 2);
}

#[test]
fn cnc_sharp_internal_corners_are_named_edges() {
    // box z -10..0 with a square pocket 20x20, 5 deep, open to +Z
    let body = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 40.0, 40.0),
        ext("F2", "F1", "add", 10.0, r#","flip":true"#),
        rect("F3", "front", 10.0, 10.0, 30.0, 30.0),
        ext("F4", "F3", "cut", 5.0, "")
    ));
    let v = check(&body, "cnc-3axis", "");
    let r = of(&v, "minInternalRadius");
    assert_eq!(r.len(), 4, "{:?}", r.iter().map(|x| &x.message).collect::<Vec<_>>());
    for x in &r {
        assert_eq!(x.value, 0.0);
        assert_eq!(x.limit, 1.0);
        assert_eq!(x.edges.len(), 1);
        assert!(x.edges[0].contains('|') && x.edges[0].contains('#'), "{}", x.edges[0]);
        assert_eq!(x.faces.len(), 2);
    }
    // nothing is hidden from the tool
    assert!(of(&v, "undercut").is_empty(), "{:?}", of(&v, "undercut").iter().map(|x| &x.message).collect::<Vec<_>>());
    // a plain box has no internal corners (calibrates the convex / concave test)
    let plain = build(&format!("{},{}", rect("F1", "front", 0.0, 0.0, 40.0, 40.0), ext("F2", "F1", "add", 10.0, "")));
    assert!(of(&check(&plain, "cnc-3axis", ""), "minInternalRadius").is_empty());
    // a zero cutter radius accepts sharp corners
    assert!(of(&check(&body, "", r#"{"id":"minInternalRadius","params":{"minRadius":0}}"#), "minInternalRadius").is_empty());
}

#[test]
fn draft_is_measured_against_the_pull_direction() {
    let build_with = |draft: f64| {
        build(&format!(
            "{},{}",
            rect("F1", "front", 0.0, 0.0, 20.0, 20.0),
            ext("F2", "F1", "add", 10.0, &format!(r#","draft":{draft}"#))
        ))
    };
    // straight walls: the four sides have no draft
    let v = check(&build_with(0.0), "injection", "");
    let d = of(&v, "draft");
    assert_eq!(d.len(), 4, "{:?}", d.iter().map(|x| &x.message).collect::<Vec<_>>());
    assert!(d.iter().all(|x| x.value.abs() < 1e-6 && x.limit == 1.0));
    // half a degree is still under the limit and is reported as such
    let v = check(&build_with(0.5), "injection", "");
    let d = of(&v, "draft");
    assert_eq!(d.len(), 4);
    assert!(d.iter().all(|x| (x.value - 0.5).abs() < 0.01), "{:?}", d.iter().map(|x| x.value).collect::<Vec<_>>());
    // 3 degrees passes
    assert!(of(&check(&build_with(3.0), "injection", ""), "draft").is_empty());
    // a drafted cup: tapered outside and inside is fine, a straight pocket in it is not
    let cup = |pocket_draft: f64| {
        build(&format!(
            "{},{},{},{}",
            circle("F1", "front", 0.0, 0.0, 20.0),
            ext("F2", "F1", "add", 30.0, r#","draft":3"#),
            circle("F3", "front", 0.0, 0.0, 14.0),
            ext("F4", "F3", "cut", 25.0, &format!(r#","flip":true,"draft":{pocket_draft}"#))
        ))
    };
    let v = check(&cup(3.0), "injection", "");
    assert!(of(&v, "draft").is_empty(), "{:?}", of(&v, "draft").iter().map(|x| (&x.faces, x.value)).collect::<Vec<_>>());
    let v = check(&cup(0.0), "injection", "");
    let d = of(&v, "draft");
    assert_eq!(d.len(), 1, "{:?}", d.iter().map(|x| &x.message).collect::<Vec<_>>());
    assert!(d[0].value.abs() < 1e-6);
    assert!(d[0].faces[0].starts_with("F4"));
    // pulling along X instead: the faces facing X now have no draft
    let v = check(&build_with(0.0), "", r#"{"id":"draft","params":{"direction":[1,0,0]}}"#);
    assert_eq!(of(&v, "draft").len(), 4);
}

#[test]
fn uniform_wall_flags_a_thick_boss() {
    // 2 mm plate with a 6 mm thick, 8 mm tall rib: the rib walls are 3x the 2 mm median
    let body = build(&format!(
        "{},{},{},{}",
        rect("F1", "front", 0.0, 0.0, 60.0, 60.0),
        ext("F2", "F1", "add", 2.0, ""),
        rect("F3", "front", 20.0, 10.0, 26.0, 50.0),
        ext("F4", "F3", "add", 8.0, "")
    ));
    let v = check(&body, "injection", "");
    let u = of(&v, "uniformWall");
    assert!(!u.is_empty());
    assert!(u.iter().all(|x| x.severity == "warn" && x.faces.len() <= 2));
    // a uniform plate is fine
    let plate = build(&format!("{},{}", rect("F1", "front", 0.0, 0.0, 60.0, 60.0), ext("F2", "F1", "add", 2.0, "")));
    assert!(of(&check(&plate, "injection", ""), "uniformWall").is_empty());
}
