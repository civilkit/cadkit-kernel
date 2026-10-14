//! Extended assembly mates (docs/spec-agent-interface.md, "Mate kinds") and
//! the degrees-of-freedom report: analytic checks, structured errors, no panics.

use ck_json::Json;
use ck_model::features::mate::Xf;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

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

fn boxp(w: f64, d: f64, h: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":{d}}},{{"id":"p4","x":0,"y":{d}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}]}}"#
    )
}
fn cylp(r: f64, h: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}]}}"#
    )
}
fn ins(id: &str, part: &str, extra: &str) -> String {
    format!(r#"{{"type":"insertPart","id":"{id}","name":"{id}","part":{part}{extra}}}"#)
}
fn mate(id: &str, kind: &str, a: &str, b: &str, extra: &str) -> String {
    format!(r#"{{"type":"mate","id":"{id}","kind":"{kind}","a":"{a}","b":"{b}"{extra}}}"#)
}

struct Run {
    errors: Vec<String>,
    report: Option<Json>,
    places: Vec<(String, [f64; 12])>,
    body: ck_topo::body::Body,
}

fn run(features: &[String]) -> Run {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let report = ex.brep.finalize_assembly(false);
    let places = ex
        .brep
        .asm
        .components
        .iter()
        .map(|c| {
            let mut a = [0.0; 12];
            a[..9].copy_from_slice(&c.place.r);
            a[9..].copy_from_slice(&c.place.t);
            (c.id.clone(), a)
        })
        .collect();
    Run { errors: r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect(), report, places, body: ex.brep.body.clone().unwrap() }
}

impl Run {
    fn place(&self, id: &str) -> [f64; 12] {
        self.places.iter().find(|p| p.0 == id).unwrap().1
    }
    fn dof(&self) -> (usize, usize, usize) {
        let d = self.report.as_ref().unwrap().get("dof").unwrap();
        (d.get_f64("total").unwrap() as usize, d.get_f64("translational").unwrap() as usize, d.get_f64("rotational").unwrap() as usize)
    }
    fn comp_dof(&self, id: &str) -> (usize, usize, usize) {
        let cs = self.report.as_ref().unwrap().get("components").unwrap().as_arr().unwrap();
        let c = cs.iter().find(|c| c.get_str("id") == Some(id)).unwrap();
        (c.get_f64("dof").unwrap() as usize, c.get_f64("translational").unwrap() as usize, c.get_f64("rotational").unwrap() as usize)
    }
    fn over(&self) -> bool {
        self.report.as_ref().unwrap().get_bool("overConstrained").unwrap()
    }
}

use ck_core::linalg::Vec3;
use ck_model::features::mate_kinds::{geo, G};
use std::f64::consts::PI;

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}
fn ang_y(p: &[f64; 12]) -> f64 {
    // rotation about +y
    p[2].atan2(p[0])
}
fn normal(r: &Run, id: &str) -> Vec3 {
    match geo(&r.body, id).unwrap() {
        G::Plane { n, .. } => n,
        G::Cyl { d, .. } => d,
        _ => panic!("no direction"),
    }
}
fn origin(r: &Run, id: &str) -> Vec3 {
    match geo(&r.body, id).unwrap() {
        G::Plane { o, .. } | G::Cyl { o, .. } => o,
        _ => panic!(),
    }
}

fn two_cyls(extra_first: &str, extra_second: &str) -> Vec<String> {
    let cyl = cylp(5.0, 20.0);
    vec![ins("F1", &cyl, &format!(r#","fixed":true{extra_first}"#)), ins("F2", &cyl, &format!(r#","dx":100,"dy":7,"dz":3{extra_second}"#))]
}
fn two_boxes() -> Vec<String> {
    let b = boxp(40.0, 20.0, 10.0);
    vec![ins("F1", &b, r#","fixed":true"#), ins("F2", &b, r#","dx":100,"dy":7,"dz":3"#)]
}

#[test]
fn hinge_leaves_one_rotation() {
    let mut f = two_cyls("", "");
    // concentric + axial stop: bottom cap of the mover on the top cap of the base
    f.push(mate("F3", "hinge", "F2.S.e1@F2.1", "F2.S.e1@F1.1", r#","a2":"F2.C0@F2.1","b2":"F2.C1@F1.1""#));
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let p = r.place("F2");
    assert!(near(p[9], 0.0, 1e-6) && near(p[11], 0.0, 1e-6), "{p:?}");
    assert!(near(p[10], 20.0, 1e-6), "stop at the cap: {p:?}");
    assert_eq!(r.dof(), (1, 0, 1));
    assert_eq!(r.comp_dof("F2"), (1, 0, 1));
    assert_eq!(r.comp_dof("F1"), (0, 0, 0));
    assert!(!r.over());
    // without the stop the part can also slide along the axis
    let mut g = two_cyls("", "");
    g.push(mate("F3", "hinge", "F2.S.e1@F2.1", "F2.S.e1@F1.1", ""));
    let r = run(&g);
    assert_eq!(r.dof(), (2, 1, 1));
}

#[test]
fn perpendicular_and_angle() {
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(normal(&r, "F2.C0@F2.1").dot(normal(&r, "F2.C1@F1.1")).abs() < 1e-8);
    assert_eq!(r.dof(), (5, 3, 2));
    for deg in [30.0f64, 120.0] {
        let mut f = two_boxes();
        f.push(mate("F3", "angle", "F2.C0@F2.1", "F2.C1@F1.1", &format!(r#","angle":{deg}"#)));
        let r = run(&f);
        assert!(r.errors.is_empty(), "{:?}", r.errors);
        let got = normal(&r, "F2.C0@F2.1").dot(normal(&r, "F2.C1@F1.1")).acos().to_degrees();
        assert!(near(got, deg, 1e-6), "{got} vs {deg}");
    }
    // flip takes the supplementary angle
    let mut f = two_boxes();
    f.push(mate("F3", "angle", "F2.C0@F2.1", "F2.C1@F1.1", r#","angle":30,"flip":true"#));
    let r = run(&f);
    let got = normal(&r, "F2.C0@F2.1").dot(normal(&r, "F2.C1@F1.1")).acos().to_degrees();
    assert!(near(got, 150.0, 1e-6), "{got}");
}

#[test]
fn tangent_cylinder_to_plane_and_cylinders() {
    let cyl = cylp(5.0, 20.0);
    let b = boxp(40.0, 20.0, 10.0);
    // cylinder lying on the top face of a block
    let f = vec![ins("F1", &b, r#","fixed":true"#), ins("F2", &cyl, r#","dx":10,"dy":50,"dz":3"#), mate("F3", "tangent", "F2.S.e1@F2.1", "F2.C1@F1.1", "")];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let axis = normal(&r, "F2.S.e1@F2.1");
    assert!(axis.y.abs() < 1e-8, "axis lies in the plane: {axis:?}");
    let o = origin(&r, "F2.S.e1@F2.1");
    assert!(near(o.y, 15.0, 1e-6), "centre one radius above the plane: {o:?}");
    // two parallel cylinders touching from outside: axes 10 apart
    let mut f = two_cyls("", "");
    f.push(mate("F3", "tangent", "F2.S.e1@F2.1", "F2.S.e1@F1.1", ""));
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let (a, b2) = (origin(&r, "F2.S.e1@F2.1"), origin(&r, "F2.S.e1@F1.1"));
    let d = (a - b2).reject(normal(&r, "F2.S.e1@F1.1")).len();
    assert!(near(d, 10.0, 1e-6), "axis distance {d}");
    // flip: internal contact has radius difference 0 -> axes coincide
    let mut f = two_cyls("", "");
    f.push(mate("F3", "tangent", "F2.S.e1@F2.1", "F2.S.e1@F1.1", r#","flip":true"#));
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let (a, b2) = (origin(&r, "F2.S.e1@F2.1"), origin(&r, "F2.S.e1@F1.1"));
    assert!((a - b2).reject(normal(&r, "F2.S.e1@F1.1")).len() < 1e-6);
}

#[test]
fn lock_follows_its_target() {
    let b = boxp(40.0, 20.0, 10.0);
    let f = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &b, r#","dx":100,"dy":7,"dz":3"#),
        ins("F3", &b, r#","dx":300,"dy":7,"dz":3,"axis":{"kind":"axis","axis":"x"},"angle":30"#),
        mate("F4", "lock", "F2.C0@F3.1", "F2.C0@F2.1", ""),
        mate("F5", "coincident", "F2.C0@F2.1", "F2.C1@F1.1", ""),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    // relative placement of F3 w.r.t. F2 is what it was when the lock was made
    let (p2, p3) = (r.place("F2"), r.place("F3"));
    let x2 = Xf { r: p2[..9].try_into().unwrap(), t: p2[9..].try_into().unwrap() };
    let x3 = Xf { r: p3[..9].try_into().unwrap(), t: p3[9..].try_into().unwrap() };
    let rel = ck_model::features::mate_kinds::inv(&x2).compose(&x3);
    assert!(near(rel.t[0], 200.0, 1e-6) && near(rel.t[1], 0.0, 1e-6) && near(rel.t[2], 0.0, 1e-6), "{rel:?}");
    let c = (30.0f64).to_radians().cos();
    assert!(near(rel.r[4], c, 1e-8) && near(rel.r[0], 1.0, 1e-8), "{:?}", rel.r);
    // F3 is rigid with F2, which still slides in the contact plane and turns about its normal
    assert_eq!(r.comp_dof("F3"), (3, 2, 1));
    assert_eq!(r.dof(), (3, 2, 1));
}

#[test]
fn symmetric_about_a_plane() {
    let b = boxp(40.0, 20.0, 10.0);
    let f = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F3", &b, r#","fixed":true,"dy":40"#),
        ins("F2", &b, r#","dx":100,"dy":-90,"dz":3"#),
        mate("F4", "symmetric", "F2.C1@F2.1", "F2.C0@F3.1", r#","c":"F2.C0@F1.1""#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    // the mirror image of F3's bottom face (y = 40) about y = 0 is y = -40
    let o = origin(&r, "F2.C1@F2.1");
    assert!(near(o.y, -40.0, 1e-6), "{o:?}");
    assert!(near(normal(&r, "F2.C1@F2.1").y, 1.0, 1e-8));
}

#[test]
fn width_centres_a_tab() {
    let b = boxp(40.0, 20.0, 10.0);
    let tab = boxp(40.0, 20.0, 6.0);
    // slot faces: the two caps of the block (y = 0 and y = 10); tab is 6 thick
    let f = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &tab, r#","dx":0,"dy":50,"dz":0"#),
        mate("F3", "width", "F2.C0@F2.1", "F2.C0@F1.1", r#","a2":"F2.C1@F2.1","b2":"F2.C1@F1.1""#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let mid = (origin(&r, "F2.C0@F2.1").y + origin(&r, "F2.C1@F2.1").y) * 0.5;
    assert!(near(mid, 5.0, 1e-6), "{mid}");
    assert_eq!(r.dof(), (3, 2, 1));
}

#[test]
fn slot_pin_between_walls() {
    let b = boxp(40.0, 20.0, 10.0);
    let cyl = cylp(2.0, 10.0);
    let f = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &cyl, r#","dx":5,"dy":3,"dz":4"#),
        mate("F3", "slot", "F2.S.e1@F2.1", "F2.S.e4@F1.1", r#","b2":"F2.S.e2@F1.1""#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let o = origin(&r, "F2.S.e1@F2.1");
    assert!(near(o.x, 20.0, 1e-6), "pin centred between the walls: {o:?}");
    assert_eq!(r.dof(), (4, 2, 2));
}

#[test]
fn gear_ratio_two_to_one() {
    let cyl = cylp(5.0, 20.0);
    let f = vec![
        ins("F1", &cyl, r#","fixed":true"#),
        ins("F4", &cyl, r#","fixed":true,"dx":30"#),
        ins("F2", &cyl, r#","dx":3,"dy":5,"dz":0"#),
        ins("F3", &cyl, r#","dx":33,"dy":2,"dz":0"#),
        mate("F5", "hinge", "F2.S.e1@F2.1", "F2.S.e1@F1.1", ""),
        mate("F6", "hinge", "F2.S.e1@F3.1", "F2.S.e1@F4.1", ""),
        mate("F7", "gear", "F2.S.e1@F2.1", "F2.S.e1@F3.1", r#","ratio":2,"angle":40"#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let (t2, t3) = (ang_y(&r.place("F2")).to_degrees(), ang_y(&r.place("F3")).to_degrees());
    assert!(near(t2, 40.0, 1e-6), "driver turned {t2}");
    assert!(near(t3, -20.0, 1e-6), "follower turned {t3}");
    // only the two axial slides remain
    assert_eq!(r.dof(), (2, 2, 0));
    // without a drive the pair is merely coupled: one more freedom
    let mut g = f.clone();
    g[6] = mate("F7", "gear", "F2.S.e1@F2.1", "F2.S.e1@F3.1", r#","ratio":2"#);
    let r = run(&g);
    assert_eq!(r.dof().0, 3);
    // same-direction (internal) coupling
    let mut g = f.clone();
    g[6] = mate("F7", "gear", "F2.S.e1@F2.1", "F2.S.e1@F3.1", r#","ratio":2,"angle":40,"flip":true"#);
    let r = run(&g);
    assert!(near(ang_y(&r.place("F3")).to_degrees(), 20.0, 1e-6));
}

#[test]
fn rack_and_pinion() {
    let cyl = cylp(5.0, 20.0);
    let b = boxp(40.0, 20.0, 10.0);
    let f = vec![
        ins("F1", &cyl, r#","fixed":true"#),
        ins("F2", &cyl, r#","dx":0,"dy":0,"dz":0"#),
        ins("F3", &b, r#","dx":-20,"dy":0,"dz":-30"#),
        mate("F4", "hinge", "F2.S.e1@F2.1", "F2.S.e1@F1.1", ""),
        mate("F5", "rackPinion", "F2.S.e1@F2.1", "F2.S.e2@F3.1", r#","radius":5,"angle":90,"dir":[1,0,0]"#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(near(ang_y(&r.place("F2")).to_degrees(), 90.0, 1e-6));
    // rack travels radius * angle along dir
    assert!(near(r.place("F3")[9], -20.0 + 5.0 * PI / 2.0, 1e-6), "{:?}", r.place("F3"));
}

#[test]
fn screw_couples_turn_and_advance() {
    let cyl = cylp(5.0, 20.0);
    let mk = |extra: &str| {
        vec![
            ins("F1", &cyl, r#","fixed":true"#),
            ins("F2", &cyl, r#","dx":50,"dy":0,"dz":9"#),
            mate("F3", "screw", "F2.S.e1@F1.1", "F2.S.e1@F2.1", extra),
        ]
    };
    // a = screw (fixed), b = nut; 3 turns of pitch 2 advance the nut by 6
    let r = run(&mk(r#","pitch":2,"angle":1080"#));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let p = r.place("F2");
    assert!(near(p[10], 6.0, 1e-6) && near(p[9], 0.0, 1e-6) && near(p[11], 0.0, 1e-6), "{p:?}");
    assert_eq!(r.dof().0, 0);
    let r = run(&mk(r#","pitch":2,"angle":90"#));
    let p = r.place("F2");
    assert!(near(p[10], 0.5, 1e-6) && near(ang_y(&p).to_degrees(), 90.0, 1e-6), "{p:?}");
    // left hand reverses the advance
    let r = run(&mk(r#","pitch":2,"angle":90,"flip":true"#));
    assert!(near(r.place("F2")[10], -0.5, 1e-6));
    // no drive: the helical freedom stays
    let r = run(&mk(r#","pitch":2"#));
    assert_eq!(r.dof().0, 1);
}

#[test]
fn limits_clamp_distance_and_angle() {
    let b = boxp(40.0, 20.0, 10.0);
    let f = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &b, r#","dx":0,"dy":50,"dz":0"#),
        mate("F3", "limits", "F2.C0@F2.1", "F2.C1@F1.1", r#","min":0,"max":15"#),
    ];
    let r = run(&f);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(near(r.place("F2")[10], 25.0, 1e-6), "{:?}", r.place("F2"));
    // inside the range nothing moves and no freedom is consumed
    let mut g = f.clone();
    g[2] = mate("F3", "limits", "F2.C0@F2.1", "F2.C1@F1.1", r#","min":0,"max":80"#);
    let r = run(&g);
    assert!(near(r.place("F2")[10], 50.0, 1e-9));
    assert_eq!(r.dof().0, 6);
    // angle limit: tilted part pulled back to at most 20 degrees
    let g = vec![
        ins("F1", &b, r#","fixed":true"#),
        ins("F2", &b, r#","dx":0,"dy":50,"dz":0,"axis":{"kind":"axis","axis":"x"},"angle":60"#),
        mate("F3", "limits", "F2.C1@F2.1", "F2.C1@F1.1", r#","of":"angle","min":0,"max":20"#),
    ];
    let r = run(&g);
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let a = normal(&r, "F2.C1@F2.1").dot(normal(&r, "F2.C1@F1.1")).acos().to_degrees();
    assert!(near(a, 20.0, 1e-6), "{a}");
}

#[test]
fn conflicts_and_redundancy_are_named() {
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    f.push(mate("F4", "angle", "F2.C0@F2.1", "F2.C1@F1.1", r#","angle":30"#));
    let r = run(&f);
    assert_eq!(r.errors.len(), 1, "{:?}", r.errors);
    assert!(r.errors[0].starts_with("F4:mateConflict"), "{:?}", r.errors);
    assert!(r.errors[0].contains("\"F4\"") && r.errors[0].contains("\"F3\""), "{:?}", r.errors);
    // the rejected mate left the part where F3 put it
    assert!(normal(&r, "F2.C0@F2.1").dot(normal(&r, "F2.C1@F1.1")).abs() < 1e-8);
    assert!(r.over());
    let c = r.report.as_ref().unwrap().get("conflicts").unwrap().as_arr().unwrap();
    assert_eq!(c.len(), 1);
    assert_eq!(c[0].get_str("mate"), Some("F4"));
    // a duplicate mate is redundant, not an error
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    f.push(mate("F4", "perpendicular", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    let r = run(&f);
    assert!(r.errors.is_empty());
    assert!(r.over());
    let red = r.report.as_ref().unwrap().get("redundant").unwrap().as_arr().unwrap();
    assert_eq!(red.len(), 1);
    assert_eq!(red[0].as_str(), Some("F4"));
    assert_eq!(r.dof().0, 5);
}

#[test]
fn structured_errors_never_panic() {
    // unsupported geometry
    let mut f = two_boxes();
    f.push(mate("F3", "hinge", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    let r = run(&f);
    assert!(r.errors[0].starts_with("F3:mateGeometry"), "{:?}", r.errors);
    // missing reference
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "nope", ""));
    assert!(run(&f).errors[0].starts_with("F3:missingReference"));
    // both fixed
    let b = boxp(40.0, 20.0, 10.0);
    let f = vec![ins("F1", &b, r#","fixed":true"#), ins("F2", &b, r#","fixed":true"#), mate("F3", "lock", "F2.C0@F2.1", "F2.C1@F1.1", "")];
    assert!(run(&f).errors[0].starts_with("F3:mateFixed"));
    // same body
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "F2.C1@F2.1", ""));
    assert!(run(&f).errors[0].starts_with("F3:mateSameBody"));
    // symmetric without a mirror face; width without second faces; bad gear ratio
    let mut f = two_boxes();
    f.push(mate("F3", "symmetric", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    assert!(run(&f).errors[0].starts_with("F3:missingReference"));
    let mut f = two_boxes();
    f.push(mate("F3", "width", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    assert!(run(&f).errors[0].starts_with("F3:missingReference"));
    let mut f = two_cyls("", "");
    f.push(mate("F3", "gear", "F2.S.e1@F2.1", "F2.S.e1@F1.1", r#","ratio":0"#));
    assert!(run(&f).errors[0].starts_with("F3:mateGeometry"));
    // limits without bounds, unknown kind
    let mut f = two_boxes();
    f.push(mate("F3", "limits", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    assert!(run(&f).errors[0].starts_with("F3:mateGeometry"));
    let mut f = two_boxes();
    f.push(mate("F3", "cam", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    assert!(run(&f).errors[0].starts_with("F3:mateGeometry"));
}

#[test]
fn solve_is_deterministic() {
    let mut f = two_boxes();
    f.push(mate("F3", "perpendicular", "F2.C0@F2.1", "F2.C1@F1.1", ""));
    f.push(mate("F4", "distance", "F2.C0@F2.1", "F2.C1@F1.1", r#","distance":4"#));
    let (a, b) = (run(&f), run(&f));
    assert_eq!(a.places, b.places);
    assert_eq!(a.report.as_ref().map(ck_json::to_string), b.report.as_ref().map(ck_json::to_string));
}

/// Every kind against every pair of face types and several option sets: no
/// panic, errors are structured, a clean regen yields a DOF report that is
/// consistent (total never above 6 for the one floating component) and the
/// second run is identical.
#[test]
fn every_kind_on_every_face_pair_is_safe() {
    let cyl = cylp(5.0, 20.0);
    let bx = boxp(40.0, 20.0, 10.0);
    let faces = ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e2"];
    let kinds = [
        "coincident", "concentric", "distance", "angle", "parallel", "perpendicular", "tangent", "lock", "symmetric", "width", "hinge", "slot", "screw", "gear",
        "rackPinion", "limits", "cam", "",
    ];
    let opts = [
        "",
        r#","flip":true"#,
        r#","angle":45,"ratio":3,"pitch":1.5,"radius":4,"min":1,"max":9,"dir":[0,0,1],"distance":2"#,
        r#","angle":"=1e9","of":"angle","min":-5"#,
        r#","a2":"F2.C1@F2.1","b2":"F2.C1@F1.1","c":"F2.C0@F1.1""#,
    ];
    let mut ran = 0;
    for kind in kinds {
        for fa in faces {
            for fb in faces {
                for (oi, o) in opts.iter().enumerate() {
                    let round = fa.ends_with("e1") || fb.ends_with("e1");
                    let part = if round { &cyl } else { &bx };
                    let f = vec![
                        ins("F1", part, r#","fixed":true"#),
                        ins("F2", part, r#","dx":100,"dy":7,"dz":3,"axis":{"kind":"axis","axis":"x"},"angle":17"#),
                        mate("F3", kind, &format!("{fa}@F2.1"), &format!("{fb}@F1.1"), o),
                    ];
                    let r = run(&f);
                    let again = run(&f);
                    assert_eq!(r.places, again.places, "{kind} {fa} {fb} {oi}");
                    assert_eq!(r.errors, again.errors);
                    for e in &r.errors {
                        assert!(e.starts_with("F3:"), "{e}");
                    }
                    if r.report.is_some() {
                        assert!(r.dof().0 <= 6, "{kind} {fa} {fb} {oi}: {:?}", r.dof());
                    }
                    ran += 1;
                }
            }
        }
    }
    assert!(ran > 1000);
}
