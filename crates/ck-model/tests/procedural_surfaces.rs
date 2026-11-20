//! The procedural surface features (features/proc_surface.rs): `sweepSurface` (a sheet on `Surface::Sweep`) and
//! `blendSurface` (a sheet on `Surface::Blend`). Areas and volumes against closed forms, validation, tessellation,
//! STEP export, the measure request, the body codec, the refusals with their codes, regen determinism and the
//! document round trip.
use ck_core::linalg::Vec3;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use ck_topo::body::Body;
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
    r: regen::RegenResult,
    brep: Brep,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { r, brep: ex.brep }
}

impl Out {
    fn code(&self) -> String {
        assert_eq!(self.r.errors.len(), 1, "one error expected: {:?} {:?}", self.r.errors, self.r.markers);
        self.r.errors[0].error.code.clone()
    }
    fn err_param(&self, k: &str) -> Json {
        self.r.errors[0].error.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()).unwrap_or(Json::Null)
    }
    fn has_hint(&self) -> bool {
        matches!(self.err_param("hint"), Json::Str(s) if !s.is_empty())
    }
    fn ok(&self) -> &Self {
        assert!(self.r.errors.is_empty(), "no error expected: {:?}", self.r.errors);
        self
    }
    fn marker(&self, id: &str) -> Json {
        self.r.markers.iter().find(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) == Some(id)).expect("marker").as_arr().unwrap()[1].clone()
    }
    fn num(&self, id: &str, k: &str) -> f64 {
        self.marker(id).get(k).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("marker field {k}"))
    }
    fn sheet(&self, id: &str) -> &Body {
        &self.brep.surfs.iter().find(|(i, _)| i == id).expect("sheet").1
    }
}

fn point(id: &str, x: f64, y: f64) -> String {
    format!(r#"{{"id":"{id}","x":{x},"y":{y}}}"#)
}
fn line(id: &str, a: &str, b: &str) -> String {
    format!(r#"{{"type":"line","id":"{id}","p1":"{a}","p2":"{b}"}}"#)
}
fn arc(id: &str, c: &str, a: &str, b: &str) -> String {
    format!(r#"{{"type":"arc","id":"{id}","c":"{c}","p1":"{a}","p2":"{b}"}}"#)
}
fn circle(id: &str, c: &str, r: f64) -> String {
    format!(r#"{{"type":"circle","id":"{id}","c":"{c}","r":{r}}}"#)
}
fn sketch(id: &str, plane: &str, points: &[String], entities: &[String]) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), entities.join(","))
}
/// A sketch on the plane through `o` with axes `x`, `y` (plane feature `P<id>`).
fn on_frame(id: &str, o: [f64; 3], x: [f64; 3], y: [f64; 3], points: &[String], entities: &[String]) -> String {
    format!(
        r#"{{"type":"plane","id":"P{id}","mode":"frame","origin":{o:?},"xDir":{x:?},"yDir":{y:?}}},{}"#,
        sketch(id, &format!(r#"{{"kind":"feature","id":"P{id}"}}"#), points, entities)
    )
}
const FRONT: &str = r#"{"kind":"std","id":"front"}"#;
fn doc(features: &[&str]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}
fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// 40 x 20 x 10 box on the front plane: top F2.C1 (z = 10), side F2.S.e1 (y = 0), side F2.S.e2 (x = 40).
fn box_features() -> String {
    let p = [point("p1", 0.0, 0.0), point("p2", 40.0, 0.0), point("p3", 40.0, 20.0), point("p4", 0.0, 20.0)];
    let e = [line("e1", "p1", "p2"), line("e2", "p2", "p3"), line("e3", "p3", "p4"), line("e4", "p4", "p1")];
    format!(r#"{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#, sketch("F1", FRONT, &p, &e))
}

/// S1: circle r = 2 about the origin on the front plane; S2: the line (0,0,0) -> (0,0,25).
fn tube_inputs() -> String {
    let prof = sketch("S1", FRONT, &[point("c", 0.0, 0.0)], &[circle("e1", "c", 2.0)]);
    let path = on_frame("S2", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], &[point("a", 0.0, 0.0), point("b", 25.0, 0.0)], &[line("e1", "a", "b")]);
    format!("{prof},{path}")
}

/// S3: circle r = 1.5 about (6, 0, 0) square to y; S4: the arc of radius 6 about the origin from (6,0,0) through
/// `angle` (a full circle for 360) in the front plane.
fn torus_inputs(angle: f64) -> String {
    let prof = on_frame("S3", [6.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], &[point("c", 0.0, 0.0)], &[circle("e1", "c", 1.5)]);
    let path = if angle >= 360.0 {
        sketch("S4", FRONT, &[point("c", 0.0, 0.0)], &[circle("e1", "c", 6.0)])
    } else {
        let a = angle.to_radians();
        sketch("S4", FRONT, &[point("c", 0.0, 0.0), point("a", 6.0, 0.0), point("b", 6.0 * a.cos(), 6.0 * a.sin())], &[arc("e1", "c", "a", "b")])
    };
    format!("{prof},{path}")
}

/// Validation: clean for a closed shell; an open sheet only has its free boundary (SHELL_OPEN, and the Euler count of
/// an open component, which the check reads as a closed one, and the signed volume it has no meaning for).
fn validate(b: &Body, closed: bool) {
    let issues: Vec<_> = ck_check::validate(b, 1e-6).into_iter().filter(|i| closed || !matches!(i.code, "SHELL_OPEN" | "EULER" | "VOLUME_NONPOSITIVE")).collect();
    assert!(issues.is_empty(), "{:?}", issues.iter().map(|i| (i.code, &i.entity, &i.detail)).collect::<Vec<_>>());
}

/// The mesh: every face meshed, and for a closed shell no mesh edge used by one triangle.
fn mesh(b: &Body, closed: bool) -> ck_tess::mesh::Mesh {
    let m = ck_tess::mesh::tessellate(b).mesh;
    assert!(m.failed.is_empty(), "failed faces {:?} {:?}", m.failed, m.failed_why);
    let nf = b.face_ids().count() as i32;
    for f in 0..nf {
        assert!(m.tri_face.contains(&f), "face {f} has no triangles");
    }
    if closed {
        let (open, faces) = ck_check::tessellation::open_mesh_edges(&m.positions, &m.tri_face, 1e-6);
        assert_eq!(open, 0, "open mesh edges on faces {faces:?}");
    }
    m
}

/// Area of the mesh triangles.
fn mesh_area(m: &ck_tess::mesh::Mesh) -> f64 {
    m.positions
        .chunks(9)
        .map(|t| {
            let p = |k: usize| Vec3::new(t[3 * k] as f64, t[3 * k + 1] as f64, t[3 * k + 2] as f64);
            (p(1) - p(0)).cross(p(2) - p(0)).len() * 0.5
        })
        .sum()
}

fn volume(b: &Body) -> f64 {
    let faces: Vec<_> = b.face_ids().collect();
    ck_topo::mass::signed_volume(b, &faces)
}

fn face_names(b: &Body) -> Vec<String> {
    b.face_ids().map(|f| b.face_id_str(f)).collect()
}

const TUBE: &str = r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","frame":"translate","caps":true}"#;
const TORUS: &str = r#"{"type":"sweepSurface","id":"W1","profile":"S3","path":"S4","caps":true}"#;
const BLEND: &str = r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":3}"#;

// ---- sweepSurface -------------------------------------------------------------------------------------------

#[test]
fn translated_circle_along_a_line_is_a_cylinder() {
    // area 2 pi r L, capped volume pi r^2 L
    let o = run(&doc(&[&tube_inputs(), TUBE]));
    o.ok();
    let b = o.sheet("W1");
    assert_eq!(face_names(b), ["W1.W", "W1.C0", "W1.C1"]);
    assert!(matches!(b.face(b.face_ids().next().unwrap()).surface, Surface::Sweep(_)));
    assert!(close(o.num("W1", "area"), 2.0 * PI * 2.0 * 25.0, 1e-8), "{}", o.num("W1", "area"));
    assert!(close(o.num("W1", "volume"), PI * 4.0 * 25.0, 1e-8));
    assert!(close(volume(b), PI * 4.0 * 25.0, 1e-8));
    assert!(o.num("W1", "deviation") <= 0.001);
    assert_eq!(o.marker("W1").get_str("frame"), Some("translate"));
    // the path and the profile are the exact rail and end curves; closed across the profile only
    assert_eq!(ck_json::to_string(o.marker("W1").get("closed").unwrap()), "[false,true]");
    validate(b, true);
    let m = mesh(b, true);
    assert!(close(mesh_area(&m), 100.0 * PI + 8.0 * PI, 0.01 * 108.0 * PI), "{}", mesh_area(&m));
    // without caps: the open tube
    let o = run(&doc(&[&tube_inputs(), &TUBE.replace(r#","caps":true"#, "")]));
    o.ok();
    let b = o.sheet("W1");
    assert_eq!(face_names(b), ["W1.W"]);
    validate(b, false);
    mesh(b, false);
}

#[test]
fn circle_along_an_arc_is_a_torus_patch() {
    // a quarter turn about a circle of radius R = 6, tube r = 1.5: area 2 pi r R theta, volume pi r^2 R theta
    let th = PI / 2.0;
    let o = run(&doc(&[&torus_inputs(90.0), TORUS]));
    o.ok();
    assert!(close(o.num("W1", "area"), 2.0 * PI * 1.5 * 6.0 * th, 1e-8), "{}", o.num("W1", "area"));
    assert!(close(o.num("W1", "volume"), PI * 2.25 * 6.0 * th, 1e-8));
    assert!(close(o.num("W1", "pathLength"), 6.0 * th, 1e-12));
    // the up direction defaults to the normal of the path plane
    let up = o.marker("W1").get("up").unwrap().clone();
    assert!(close(up.as_arr().unwrap()[2].as_f64().unwrap().abs(), 1.0, 1e-12));
    let b = o.sheet("W1");
    validate(b, true);
    // every point of the swept face is on the torus
    let s = &b.face(b.face_ids().next().unwrap()).surface;
    let ((u0, u1), (v0, v1)) = s.domain();
    for i in 0..=6 {
        for j in 0..=6 {
            let p = s.eval(u0 + (u1 - u0) * i as f64 / 6.0, v0 + (v1 - v0) * j as f64 / 6.0);
            let rho = (p.x * p.x + p.y * p.y).sqrt();
            assert!(close(((rho - 6.0).powi(2) + p.z * p.z).sqrt(), 1.5, 1e-9), "{p:?}");
        }
    }
    let m = mesh(b, true);
    let want = 2.0 * PI * 1.5 * 6.0 * th + 2.0 * PI * 2.25;
    assert!(close(mesh_area(&m), want, 0.01 * want));
}

#[test]
fn circle_round_a_circle_is_a_closed_torus() {
    // closed in both directions: a seam each way, no caps needed; area 4 pi^2 R r, volume 2 pi^2 R r^2
    let o = run(&doc(&[&torus_inputs(360.0), r#"{"type":"sweepSurface","id":"W1","profile":"S3","path":"S4"}"#]));
    o.ok();
    let b = o.sheet("W1");
    assert_eq!(face_names(b), ["W1.W"]);
    assert_eq!(ck_json::to_string(o.marker("W1").get("closed").unwrap()), "[true,true]");
    assert!(close(o.num("W1", "area"), 4.0 * PI * PI * 9.0, 1e-7), "{}", o.num("W1", "area"));
    assert!(close(volume(b), 2.0 * PI * PI * 6.0 * 2.25, 1e-7), "{}", volume(b));
    validate(b, true);
    mesh(b, true);
}

#[test]
fn open_profile_sweeps_an_open_sheet() {
    // a quarter arc of radius 2 translated 25 along z: area (pi) * 25
    let prof = sketch("S1", FRONT, &[point("c", 0.0, 0.0), point("a", 2.0, 0.0), point("b", 0.0, 2.0)], &[arc("e1", "c", "a", "b")]);
    let path = on_frame("S2", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], &[point("a", 0.0, 0.0), point("b", 25.0, 0.0)], &[line("e1", "a", "b")]);
    let o = run(&doc(&[&prof, &path, r#"{"type":"sweepSurface","id":"W1","profile":{"kind":"sketchCurve","sketch":"S1","entity":"e1"},"path":[{"kind":"sketchCurve","sketch":"S2","entity":"e1"}],"frame":"translate"}"#]));
    o.ok();
    assert!(close(o.num("W1", "area"), 25.0 * PI, 1e-9));
    let b = o.sheet("W1");
    assert_eq!(b.edge_ids().count(), 4);
    validate(b, false);
    mesh(b, false);
    // the tangent frame along a straight path is the same sheet (up: a world axis across the line)
    let o = run(&doc(&[&prof, &path, r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2"}"#]));
    o.ok();
    assert!(close(o.num("W1", "area"), 25.0 * PI, 1e-8));
    validate(o.sheet("W1"), false);
}

#[test]
fn path_can_be_a_procedural_curve() {
    // the path given as an offset curve (an offset of the line by 0: use a composite) and as a curve reference
    for path in [r#""C1""#, r#"{"kind":"curve","id":"C1"}"#] {
        let o = run(&doc(&[&tube_inputs(), r#"{"type":"compositeCurve","id":"C1","sketch":"S2"}"#, &TUBE.replace(r#""path":"S2""#, &format!(r#""path":{path}"#))]));
        o.ok();
        assert!(close(o.num("W1", "area"), 100.0 * PI, 1e-8));
    }
    // along an offset of the arc (radius 6 -> 5): the torus patch about the smaller circle, a profile of radius 1.5
    // centred on the offset's start (5, 0, 0)
    let prof = on_frame("S3", [5.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], &[point("c", 0.0, 0.0)], &[circle("e1", "c", 1.5)]);
    let a = (PI / 2.0).cos() * 6.0;
    let arc_s = sketch("S4", FRONT, &[point("c", 0.0, 0.0), point("a", 6.0, 0.0), point("b", a, 6.0)], &[arc("e1", "c", "a", "b")]);
    let o = run(&doc(&[&prof, &arc_s, r#"{"type":"offsetCurve","id":"O1","sketch":"S4","distance":-1}"#, r#"{"type":"sweepSurface","id":"W1","profile":"S3","path":"O1"}"#]));
    o.ok();
    assert!(close(o.num("W1", "area"), 2.0 * PI * 1.5 * 5.0 * PI / 2.0, 1e-6), "{}", o.num("W1", "area"));
}

#[test]
fn sweep_refusals() {
    let tube = |f: &str| run(&doc(&[&tube_inputs(), f]));
    // no path
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1"}"#);
    assert_eq!(o.code(), "sweepSurfaceNoInput");
    assert_eq!(o.err_param("field").as_str(), Some("path"));
    assert!(o.has_hint());
    // unknown sketch, bad frame, bad up, bad tolerance
    assert_eq!(tube(r#"{"type":"sweepSurface","id":"W1","profile":"NOPE","path":"S2"}"#).code(), "missingSketch");
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","frame":"spin"}"#);
    assert_eq!((o.code(), o.err_param("field").as_str().map(str::to_string)), ("invalidValue".into(), Some("frame".into())));
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","up":[0,0,0]}"#);
    assert_eq!((o.code(), o.err_param("field").as_str().map(str::to_string)), ("invalidValue".into(), Some("up".into())));
    assert_eq!(tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","tolerance":-1}"#).code(), "invalidValue");
    // up along the path (the line runs along z)
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","up":[0,0,1]}"#);
    assert_eq!(o.code(), "sweepSurfaceFrame");
    assert!(o.has_hint() && o.err_param("at").as_arr().is_some());
    // the line swept along itself has no area
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S2","path":"S2","frame":"translate"}"#);
    assert_eq!(o.code(), "sweepSurfaceDegenerate");
    assert!(o.has_hint());
    // a tube of radius 7 round a circle of radius 6 folds over the axis
    let fat = on_frame("S3", [6.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], &[point("c", 0.0, 0.0)], &[circle("e1", "c", 7.0)]);
    let path = torus_inputs(90.0);
    let path = &path[path.find(r#"{"type":"sketch","id":"S4""#).unwrap()..];
    let o = run(&doc(&[&fat, path, r#"{"type":"sweepSurface","id":"W1","profile":"S3","path":"S4"}"#]));
    assert_eq!(o.code(), "sweepSurfaceFold");
    assert!(close(o.err_param("radius").as_f64().unwrap(), 6.0, 1e-9));
    assert!(o.has_hint());
    // caps on an open profile
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S2","path":"S1","frame":"translate","caps":true}"#);
    assert_eq!(o.code(), "sweepSurfaceCaps");
    // a gap in a listed chain
    let o = tube(r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":[{"kind":"sketchCurve","sketch":"S2","entity":"e1"},{"kind":"sketchCurve","sketch":"S1","entity":"e1"}]}"#);
    assert_eq!(o.code(), "sweepSurfaceGap");
    // a reference that does not resolve
    assert_eq!(tube(r#"{"type":"sweepSurface","id":"W1","profile":{"kind":"sketchCurve","sketch":"S1","entity":"e9"},"path":"S2"}"#).code(), "missingReference");
}

// ---- blendSurface -------------------------------------------------------------------------------------------

#[test]
fn ball_between_perpendicular_planes_is_a_quarter_cylinder() {
    // the round of the box's top front edge: radius 3 over the 40 mm edge, area pi r L / 2
    let o = run(&doc(&[&box_features(), BLEND]));
    o.ok();
    assert!(close(o.num("B1", "area"), PI * 3.0 * 40.0 / 2.0, 1e-8), "{}", o.num("B1", "area"));
    assert_eq!(ck_json::to_string(o.marker("B1").get("sides").unwrap()), "[-1,-1]");
    assert_eq!(o.marker("B1").get_str("span"), Some("edge"));
    let b = o.sheet("B1");
    assert_eq!(face_names(b), ["B1.R"]);
    let f = b.face_ids().next().unwrap();
    assert!(matches!(b.face(f).surface, Surface::Blend(_)));
    // every point is 3 from the axis y = 3, z = 7; the contact edges lie on the top (y = 3) and the side (z = 7)
    let s = &b.face(f).surface;
    for i in 0..=8 {
        for j in 0..=8 {
            let p = s.eval(40.0 * i as f64 / 8.0, j as f64 / 8.0);
            assert!(close(((p.y - 3.0).powi(2) + (p.z - 7.0).powi(2)).sqrt(), 3.0, 1e-9), "{p:?}");
        }
    }
    let mids: Vec<Vec3> = b.edge_ids().map(|e| b.edge_mid(e)).collect();
    assert!(mids.iter().any(|m| close(m.z, 10.0, 1e-9) && close(m.y, 3.0, 1e-9)), "{mids:?}");
    assert!(mids.iter().any(|m| close(m.y, 0.0, 1e-9) && close(m.z, 7.0, 1e-9)), "{mids:?}");
    // the end arcs are circles of radius 3
    let circles = b.edge_ids().filter(|&e| matches!(b.edge(e).curve, ck_geom::curve::Curve::Circle { r, .. } if close(r, 3.0, 1e-9))).count();
    assert_eq!(circles, 2);
    // outward like the box faces it rounds: away from the axis
    let n = b.outward_normal(f, 20.0, 0.5);
    let p = s.eval(20.0, 0.5);
    assert!(n.dot(p - Vec3::new(20.0, 3.0, 7.0)) > 0.0);
    validate(b, false);
    let m = mesh(b, false);
    assert!(close(mesh_area(&m), 60.0 * PI, 0.01 * 60.0 * PI));
}

#[test]
fn blend_radius_law_and_points_span() {
    // radius 2 -> 4 along the edge: the end arcs are quarter circles of the end radii
    let o = run(&doc(&[&box_features(), &BLEND.replace(r#""radius":3"#, r#""radius":[2,"=2*2"]"#)]));
    o.ok();
    let b = o.sheet("B1");
    let mut rs: Vec<f64> = b.edge_ids().filter_map(|e| match b.edge(e).curve { ck_geom::curve::Curve::Circle { r, .. } => Some(r), _ => None }).collect();
    rs.sort_by(f64::total_cmp);
    assert!(rs.len() == 2 && close(rs[0], 2.0, 1e-9) && close(rs[1], 4.0, 1e-9), "{rs:?}");
    validate(b, false);
    mesh(b, false);
    // the span from (10, 0, 10) to (30, 0, 10): a quarter cylinder 20 long
    let o = run(&doc(&[&box_features(), &BLEND.replace(r#""radius":3"#, r#""radius":3,"from":[10,0,10],"to":[30,0,10]"#)]));
    o.ok();
    assert!(close(o.num("B1", "area"), PI * 3.0 * 20.0 / 2.0, 1e-8));
    assert_eq!(o.marker("B1").get_str("span"), Some("points"));
    // a concave corner: the ball outside both faces of an L (the top of the box and a wall standing on it)
    let wall = r#"{"type":"surfacePlanar","id":"P9","sketch":"S9"}"#;
    let ws = on_frame("S9", [0.0, 20.0, 10.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], &[point("a", 0.0, 0.0), point("b", 40.0, 0.0), point("c", 40.0, 10.0), point("d", 0.0, 10.0)], &[line("e1", "a", "b"), line("e2", "b", "c"), line("e3", "c", "d"), line("e4", "d", "a")]);
    let o = run(&doc(&[&box_features(), &ws, wall, r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","P9.P"],"radius":2,"from":[0,20,10],"to":[40,20,10]}"#]));
    o.ok();
    assert!(close(o.num("B1", "area"), PI * 2.0 * 40.0 / 2.0, 1e-8), "{}", o.num("B1", "area"));
    let b = o.sheet("B1");
    let s = &b.face(b.face_ids().next().unwrap()).surface;
    let p = s.eval(20.0, 0.5);
    // the ball centre (20, 18, 12) is above the top and in front of the wall
    assert!(close(p.dist(Vec3::new(20.0, 18.0, 12.0)), 2.0, 1e-9), "{p:?}");
    validate(b, false);
}

#[test]
fn blend_refusals() {
    let bx = |f: &str| run(&doc(&[&box_features(), f]));
    assert_eq!(bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1"],"radius":3}"#).code(), "invalidValue");
    assert_eq!(bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.C9"],"radius":3}"#).code(), "missingReference");
    for r in [r#""#, r#","radius":-1"#, r#","radius":[1,2,3]"#, r#","radius":[1,0]"#] {
        let o = bx(&format!(r#"{{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"]{r}}}"#));
        assert_eq!((o.code(), o.err_param("field").as_str().map(str::to_string)), ("invalidValue".into(), Some("radius".into())), "{r}");
    }
    let o = bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":3,"sides":[2,1]}"#);
    assert_eq!(o.err_param("field").as_str(), Some("sides"));
    // top and bottom share no edge
    let o = bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.C0"],"radius":3}"#);
    assert_eq!(o.code(), "blendSurfaceSpan");
    assert!(o.has_hint());
    assert_eq!(bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":3,"from":[0,0,10]}"#).code(), "blendSurfaceSpan");
    // a ball of radius 3 between faces 10 apart touches never; nor one outside both faces of the convex edge
    let o = bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.C0"],"radius":3,"from":[0,10,5],"to":[40,10,5]}"#);
    assert_eq!(o.code(), "blendSurfaceNoContact");
    assert!(o.has_hint() && o.err_param("at").as_arr().is_some());
    assert_eq!(bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":3,"sides":[1,1]}"#).code(), "blendSurfaceNoContact");
    // the side face is 10 high: a radius of 12 runs off it, 10 is the largest that fits
    let o = bx(r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":12}"#);
    assert_eq!(o.code(), "blendSurfaceRadius");
    assert_eq!(o.err_param("radius").as_f64(), Some(12.0));
    assert!(close(o.err_param("maxRadius").as_f64().unwrap(), 10.0, 1e-4), "{:?}", o.err_param("maxRadius"));
    assert!(o.has_hint());
    // two faces in one plane: no side faces the other
    let sq = |id: &str, x: f64| on_frame(id, [x, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], &[point("a", 0.0, 0.0), point("b", 5.0, 0.0), point("c", 5.0, 5.0), point("d", 0.0, 5.0)], &[line("e1", "a", "b"), line("e2", "b", "c"), line("e3", "c", "d"), line("e4", "d", "a")]);
    let o = run(&doc(&[&sq("S1", 0.0), &sq("S2", 10.0), r#"{"type":"surfacePlanar","id":"Q1","sketch":"S1"}"#, r#"{"type":"surfacePlanar","id":"Q2","sketch":"S2"}"#, r#"{"type":"blendSurface","id":"B1","faces":["Q1.P","Q2.P"],"radius":1,"from":[5,0,0],"to":[5,5,0]}"#]));
    assert_eq!(o.code(), "blendSurfaceSides");
}

// ---- downstream: export, measure, codec ---------------------------------------------------------------------

#[test]
fn sheets_export_measure_and_encode() {
    for (src, id, area) in [(doc(&[&torus_inputs(90.0), TORUS]), "W1", 9.0 * PI * PI), (doc(&[&box_features(), BLEND]), "B1", 60.0 * PI)] {
        let o = run(&src);
        o.ok();
        let b = o.sheet(id);
        let main = b.face_id_str(b.face_ids().next().unwrap());
        // STEP: the procedural face is written as its B-spline, and reads back with the same area
        let t = ck_io::step::write::write_step(b, id, &Default::default());
        assert!(t.contains("B_SPLINE_SURFACE"), "{id}");
        assert!(!ck_io::step::write::has_non_finite_token(&t));
        let back = ck_io::step::read::read_step(&t).expect("reads back");
        let bf = back.body.face_ids().next().expect("a face");
        let ba = ck_topo::mass::face_integral::<1>(&back.body, bf, 2, &|_, n| [n.len()])[0].abs();
        assert!(close(ba, area, 1e-3 * area), "{id}: {ba} vs {area}");
        // the measure request on the face
        let q = ck_json::parse(&format!(r#"{{"refs":[{{"kind":"face","id":"{main}"}}]}}"#)).unwrap();
        let r = ck_ops::measure::reply(b, &q);
        let a = r.get("items").and_then(|i| i.as_arr()).and_then(|i| i[0].get("area")).and_then(|v| v.as_f64()).expect("area");
        assert!(close(a, area, 1e-8 * area), "{id}: {a}");
        // the body codec keeps the definition
        let d = ck_topo::codec::decode(&ck_topo::codec::encode(b)).unwrap();
        assert_eq!(&d.face(d.face_ids().next().unwrap()).surface, &b.face(b.face_ids().next().unwrap()).surface);
        assert!(close(o.num(id, "area"), area, 1e-8 * area));
    }
}

// ---- registration, determinism, round trip ------------------------------------------------------------------

#[test]
fn registered_with_their_empty_payload_errors() {
    for t in ["sweepSurface", "blendSurface"] {
        assert!(regen::KNOWN_TYPES.contains(&t));
        let want = regen::EMPTY_FEATURE_ERRORS.iter().find(|(x, _)| *x == t).expect("listed").1;
        assert_eq!(run(&doc(&[&format!(r#"{{"type":"{t}","id":"F20"}}"#)])).code(), want, "{t}");
    }
}

#[test]
fn misspelt_field_warns() {
    let o = run(&doc(&[&tube_inputs(), r#"{"type":"sweepSurface","id":"W1","profile":"S1","path":"S2","frmae":"translate"}"#]));
    o.ok();
    let w = o.r.warnings.iter().find(|w| w.feature == "W1").expect("warning");
    assert_eq!(w.error.code, "unknownField");
}

fn full_doc() -> String {
    doc(&[&box_features(), &torus_inputs(90.0), TORUS, r#"{"type":"blendSurface","id":"B1","faces":["F2.C1","F2.S.e1"],"radius":"=1+2","tolerance":0.0005}"#])
}

#[test]
fn regen_is_deterministic_and_round_trips() {
    let d = full_doc();
    let (a, b) = (run(&d), run(&d));
    a.ok();
    assert_eq!(ck_json::to_string(&Json::Arr(a.r.markers.clone())), ck_json::to_string(&Json::Arr(b.r.markers.clone())));
    assert_eq!(a.brep.surfs, b.brep.surfs);
    // the document as written, read again and regenerated
    let part = Part::from_json(&ck_json::parse(&d).unwrap()).unwrap();
    let text = ck_json::to_string(&part.to_json());
    assert_eq!(text, ck_json::to_string(&ck_json::parse(&d).unwrap()));
    let c = run(&text);
    c.ok();
    assert_eq!(a.brep.surfs, c.brep.surfs);
    // the regen cache snapshot keeps the sheets
    let snap = a.brep.save(None);
    let mut l = Brep::new();
    l.load(&snap);
    assert_eq!(l.surfs, a.brep.surfs);
    assert_eq!(l.surfs.len(), 2);
}

#[test]
fn unreachable_tolerance_is_refused() {
    // a bound far below what a bicubic fit of the definition reaches
    let o = run(&doc(&[&torus_inputs(90.0), &TORUS.replace(r#""caps":true"#, r#""tolerance":1e-13"#)]));
    assert_eq!(o.code(), "sweepSurfaceFailed");
    assert_eq!(o.err_param("tolerance").as_f64(), Some(1e-13));
    assert!(o.has_hint());
    let o = run(&doc(&[&box_features(), &BLEND.replace(r#""radius":3"#, r#""radius":[2,4],"tolerance":1e-13"#)]));
    assert_eq!(o.code(), "blendSurfaceFailed");
}
