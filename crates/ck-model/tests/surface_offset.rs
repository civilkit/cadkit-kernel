//! `surfaceOffset` (Offset Surface): faces of a solid or a surface body offset by a distance into a new surface body.
//!
//! Fixtures: a 40 x 20 x 10 box (front plane: x 0..40, y 0..20, z 0..10), a cylinder R 10 / height 10, a sphere R 10.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    surfs: Vec<(String, ck_topo::body::Body)>,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn full(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn body(&self) -> &ck_topo::body::Body {
        self.body.as_ref().unwrap()
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.body()).volume
    }
    /// Volume of the lump with this name.
    fn named(&self, name: &str) -> f64 {
        let b = self.body();
        let l = b.lump_ids().find(|&l| b.lump(l).name.as_deref() == Some(name)).unwrap_or_else(|| panic!("no body {name}: {:?}", self.names()));
        ck_topo::mass::mass_props(&ck_ops::interference::extract_lump(b, l)).volume
    }
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body();
        b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone(), surfs: ex.brep.surfs.clone() }
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},
              {"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
  {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
const CYL: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],
  "entities":[{"type":"circle","id":"e1","c":"c","r":10}],"constraints":[]},
  {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

fn surf(r: &Run) -> &ck_topo::body::Body {
    r.surfs.iter().find(|s| s.0 == "F3").map(|s| &s.1).expect("surface F3")
}

fn bounds(b: &ck_topo::body::Body) -> (ck_geom::Vec3, ck_geom::Vec3) {
    let (mut lo, mut hi) = (ck_geom::Vec3::new(1e30, 1e30, 1e30), ck_geom::Vec3::new(-1e30, -1e30, -1e30));
    for v in b.vertex_ids() {
        let p = b.vertex(v).p;
        lo = ck_geom::Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = ck_geom::Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo, hi)
}

#[test]
fn a_planar_face_offsets_along_its_outward_normal() {
    let top = run(&format!(r#"{BOX},{{"type":"surfaceOffset","id":"F3","faces":["F2.C1"],"distance":2}}"#));
    assert!(top.errors().is_empty(), "{:?}", top.full());
    let s = surf(&top);
    assert_eq!(s.face_ids().count(), 1);
    let (lo, hi) = bounds(s);
    // F2.C1 is the cap at z = 10: it moves out to z = 12, same outline
    assert!((lo.x - 0.0).abs() < 1e-9 && (hi.x - 40.0).abs() < 1e-9 && (hi.y - 20.0).abs() < 1e-9, "{lo:?} {hi:?}");
    assert!((hi.z - lo.z).abs() < 1e-9);
    assert!((hi.z - 12.0).abs() < 1e-9, "{}", hi.z);
    // the solid is untouched
    assert!((top.volume() - 40.0 * 20.0 * 10.0).abs() < 1e-9);
    assert_eq!(top.body().face_ids().count(), 6);
    // negative distance and flip go the other way
    let neg = run(&format!(r#"{BOX},{{"type":"surfaceOffset","id":"F3","faces":["F2.C1"],"distance":-2}}"#));
    let flip = run(&format!(r#"{BOX},{{"type":"surfaceOffset","id":"F3","faces":["F2.C1"],"distance":2,"flip":true}}"#));
    assert!((bounds(surf(&neg)).1.z - bounds(surf(&flip)).1.z).abs() < 1e-9);
    assert!((bounds(surf(&neg)).1.z - 8.0).abs() < 1e-9);
}

#[test]
fn several_faces_meet_in_offset_edges() {
    let r = run(&format!(r#"{BOX},{{"type":"surfaceOffset","id":"F3","faces":["F2.C1","F2.S.e2"],"distance":3}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    let s = surf(&r);
    assert_eq!(s.face_ids().count(), 2);
    let names: Vec<String> = s.face_ids().map(|f| s.face_id_str(f)).collect();
    assert!(names.contains(&"F3.O1".to_string()) && names.contains(&"F3.O2".to_string()), "{names:?}");
    // the wall x = 40 moves to 43 and the top moves 3 out; the shared edge is at (43, y, top + 3)
    let (lo, hi) = bounds(s);
    assert!((hi.x - 43.0).abs() < 1e-9 && (hi.z - 13.0).abs() < 1e-9, "{lo:?} {hi:?}");
}

#[test]
fn a_cylinder_face_changes_radius_by_the_distance() {
    let r = run(&format!(r#"{CYL},{{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":2}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    let s = surf(&r);
    let f = s.face_ids().next().unwrap();
    match &s.face(f).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => assert!((r - 12.0).abs() < 1e-9, "{r}"),
        o => panic!("{o:?}"),
    }
    // inward: r = 6, and past the axis is refused with the numbers
    let inward = run(&format!(r#"{CYL},{{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":-4}}"#));
    match &surf(&inward).face(surf(&inward).face_ids().next().unwrap()).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => assert!((r - 6.0).abs() < 1e-9, "{r}"),
        o => panic!("{o:?}"),
    }
    let past = run(&format!(r#"{CYL},{{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":-10}}"#));
    assert_eq!(past.errors(), vec!["F3:offsetDegenerate"], "{:?}", past.full());
}

#[test]
fn a_sphere_offsets_and_a_surface_body_can_be_replaced() {
    let sph = r#"{"type":"sphere","id":"F2","op":"add","center":[0,0,0],"radius":10}"#;
    let r = run(&format!(r#"{sph},{{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":1.5}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    let s = surf(&r);
    match &s.face(s.face_ids().next().unwrap()).surface {
        ck_geom::surface::Surface::Sphere { r, .. } => assert!((r - 11.5).abs() < 1e-9, "{r}"),
        o => panic!("{o:?}"),
    }
    // offset the surface body again, dropping the first
    let again = run(&format!(r#"{sph},{{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":1.5}},{{"type":"surfaceOffset","id":"F4","surface":"F3","distance":1,"keep":false}}"#));
    assert!(again.errors().is_empty(), "{:?}", again.full());
    let ids: Vec<&str> = again.surfs.iter().map(|s| s.0.as_str()).collect();
    assert_eq!(ids, vec!["F4"]);
}

#[test]
fn refusals_have_stable_codes() {
    let t = |tail: &str| run(&format!("{BOX},{tail}")).errors();
    assert_eq!(t(r#"{"type":"surfaceOffset","id":"F3"}"#), vec!["F3:missingReference"]);
    assert_eq!(t(r#"{"type":"surfaceOffset","id":"F3","faces":["Nope"]}"#), vec!["F3:missingReference"]);
    assert_eq!(t(r#"{"type":"surfaceOffset","id":"F3","surface":"Nope"}"#), vec!["F3:missingReference"]);
    assert_eq!(t(r#"{"type":"surfaceOffset","id":"F3","faces":["F2.C1"],"keep":false}"#), vec!["F3:invalidValue"]);
    let r = run(r#"{"type":"surfaceOffset","id":"F1"}"#);
    assert_eq!(r.errors(), vec!["F1:missingReference"]);
}

#[test]
fn a_free_form_face_is_refused_not_approximated() {
    let r = run(&format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":0,"y":0}},{{"id":"b","x":30,"y":0}},{{"id":"c","x":15,"y":20}}],
        "entities":[{{"type":"spline","id":"e1","pts":["a","c","b"]}},{{"type":"line","id":"e2","p1":"b","p2":"a"}}],"constraints":[]}},
        {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
        {{"type":"surfaceOffset","id":"F3","faces":["F2.S.e1"],"distance":1}}"#
    ));
    assert_eq!(r.errors(), vec!["F3:offsetUnsupported"], "{:?}", r.full());
}
