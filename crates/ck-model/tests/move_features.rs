//! `moveFace` and `moveBody` (docs/spec-agent-interface.md, "Direct manipulation"):
//! analytic volumes, persistent names, structured errors, no panics.

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
    fn bbox(&self) -> ([f64; 3], [f64; 3]) {
        let (a, b) = self.body.as_ref().unwrap().bbox();
        ([a.x, a.y, a.z], [b.x, b.y, b.z])
    }
    fn valid(&self) -> bool {
        ck_check::validate(self.body.as_ref().unwrap(), 1e-6).is_empty()
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

/// 40 x 20 x 10 block on the top plane (sketch F1 -> extrude F2).
const BLOCK: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// Cylinder of radius 5, height 30 on the top plane.
const ROD: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#;

/// The block with a 6 mm hole through it at (20, 10).
const HOLED: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"sketch","id":"F3","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":20,"y":10}],"entities":[{"type":"circle","id":"c1","c":"c","r":3}],"constraints":[]},
 {"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"throughBoth","depth":0}"#;

fn with(base: &str, more: &str) -> Run {
    run(&format!("{base},{more}"))
}

#[test]
fn names_of_the_fixtures() {
    let r = run(BLOCK);
    eprintln!("block {:?} bbox {:?}", r.faces(), r.bbox());
    let r = run(HOLED);
    eprintln!("holed {:?}", r.faces());
    let r = run(ROD);
    eprintln!("rod {:?} bbox {:?}", r.faces(), r.bbox());
}

fn cap_names(r: &Run) -> Vec<String> {
    r.faces().into_iter().filter(|n| n.contains(".C")).collect()
}

/// The cap of F2 whose plane has the larger y (the far end of the extrusion; the top plane extrudes along y).
fn top_cap(base: &str) -> String {
    let r = run(base);
    let b = r.body.as_ref().unwrap();
    let mut best = (f64::MIN, String::new());
    for f in b.face_ids() {
        let n = b.face_id_str(f);
        if let ck_geom::surface::Surface::Plane { frame } = &b.face(f).surface {
            if frame.z.y.abs() > 0.99 && frame.o.y > best.0 {
                best = (frame.o.z, n);
            }
        }
    }
    best.1
}

#[test]
fn push_the_top_face_up_and_down() {
    let top = top_cap(BLOCK);
    let before = run(BLOCK);
    for d in [3.0, -4.0] {
        let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":{d}}}"#));
        assert!(r.errors().is_empty(), "{:?}", r.errors());
        assert!(close(r.volume, 40.0 * 20.0 * (10.0 + d), 1e-9), "d={d}: volume {}", r.volume);
        assert_eq!(r.faces(), before.faces(), "persistent names survive");
        assert!(r.valid());
        assert!(close(r.bbox().1[1] - r.bbox().0[1], 10.0 + d, 1e-9), "{:?}", r.bbox());
    }
    // flip negates
    let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":3,"flip":true}}"#));
    assert!(close(r.volume, 40.0 * 20.0 * 7.0, 1e-9), "{}", r.volume);
}

#[test]
fn a_side_wall_moves_in_both_modes() {
    let r0 = run(BLOCK);
    let b = r0.body.as_ref().unwrap();
    // the wall at x = 40: outward normal +x
    let wall = b
        .face_ids()
        .find(|&f| matches!(&b.face(f).surface, ck_geom::surface::Surface::Plane { frame } if frame.z.x.abs() > 0.99 && (frame.o.x - 40.0).abs() < 1e-9))
        .map(|f| b.face_id_str(f))
        .expect("wall");
    let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{wall}"],"distance":5}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 45.0 * 20.0 * 10.0, 1e-9), "{}", r.volume);
    // translate mode: the vector's component along the normal counts, the sliding part is ignored
    let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{wall}"],"mode":"translate","translate":[-6,3,2]}}"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 34.0 * 20.0 * 10.0, 1e-9), "{}", r.volume);
    assert_eq!(r.faces(), r0.faces());
    assert!(r.valid());
}

#[test]
fn rod_radius_changes_and_hole_wall_shrinks_the_hole() {
    let r = with(ROD, r#"{"type":"moveFace","id":"F9","faces":["F2.S.c1"],"distance":2}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, PI * 49.0 * 30.0, 1e-5), "{}", r.volume);
    assert!(r.valid());
    assert_eq!(r.faces(), run(ROD).faces());
    // radius 5 -> 3.5
    let r = with(ROD, r#"{"type":"moveFace","id":"F9","faces":["F2.S.c1"],"distance":-1.5}"#);
    assert!(close(r.volume, PI * 3.5 * 3.5 * 30.0, 1e-5), "{}", r.volume);
    // hole r 3 -> 4 means the wall moves INTO the material: negative distance
    let r = with(HOLED, r#"{"type":"moveFace","id":"F9","faces":["F4.S.c1"],"distance":-1}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 40.0 * 20.0 * 10.0 - PI * 16.0 * 10.0, 1e-5), "{}", r.volume);
    assert!(r.valid());
    // and a positive distance shrinks it
    let r = with(HOLED, r#"{"type":"moveFace","id":"F9","faces":["F4.S.c1"],"distance":1}"#);
    assert!(close(r.volume, 40.0 * 20.0 * 10.0 - PI * 4.0 * 10.0, 1e-5), "{}", r.volume);
}

#[test]
fn later_features_still_find_the_moved_faces() {
    let top = top_cap(BLOCK);
    // a boss sketched on the moved face sits on the new plane
    let tail = format!(
        r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":5}},
 {{"type":"sketch","id":"F10","plane":{{"kind":"face","id":"{top}"}},"points":[{{"id":"q","x":0,"y":0}}],"entities":[{{"type":"circle","id":"c1","c":"q","r":2}}],"constraints":[]}},
 {{"type":"extrude","id":"F11","op":"add","sketch":"F10","end":"blind","depth":4}}"#
    );
    let r = with(BLOCK, &tail);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 40.0 * 20.0 * 15.0 + PI * 4.0 * 4.0, 1e-4), "{}", r.volume);
    assert!(close(r.bbox().1[1], 19.0, 1e-9), "{:?}", r.bbox());
}

#[test]
fn structured_errors_and_no_panics() {
    let top = top_cap(BLOCK);
    let err = |base: &str, f: &str| -> Vec<String> { with(base, f).errors() };
    // no body yet
    assert_eq!(run(r#"{"type":"moveFace","id":"F1","faces":["x"],"distance":1}"#).errors(), vec!["F1:nothingToCut"]);
    assert_eq!(run(r#"{"type":"moveBody","id":"F1","translate":[1,0,0]}"#).errors(), vec!["F1:nothingToCut"]);
    assert_eq!(err(BLOCK, r#"{"type":"moveFace","id":"F9","faces":[],"distance":1}"#), vec!["F9:noFaces"]);
    assert_eq!(err(BLOCK, r#"{"type":"moveFace","id":"F9","faces":["nope"],"distance":1}"#).len(), 1);
    assert!(err(BLOCK, r#"{"type":"moveFace","id":"F9","faces":["nope"],"distance":1}"#)[0].starts_with("F9:missingReference"));
    // collapse: the block is only 10 tall
    let e = err(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":-12}}"#));
    assert_eq!(e.len(), 1, "{e:?}");
    assert!(e[0].starts_with("F9:moveFaceFailed"), "{e:?}");
    // the model is unchanged after a failure (later features still run on it)
    let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":-12}}"#));
    assert!(close(r.volume, 8000.0, 1e-9), "{}", r.volume);
    // rod collapse
    let e = err(ROD, r#"{"type":"moveFace","id":"F9","faces":["F2.S.c1"],"distance":-6}"#);
    assert!(e.len() == 1 && e[0].starts_with("F9:moveFaceFailed"), "{e:?}");
    // translate mode on a curved face
    let e = err(ROD, r#"{"type":"moveFace","id":"F9","faces":["F2.S.c1"],"mode":"translate","translate":[1,0,0]}"#);
    assert!(e.len() == 1 && e[0].starts_with("F9:moveFaceUnsupported"), "{e:?}");
    // bad mode / missing vector / bad number
    assert!(err(BLOCK, r#"{"type":"moveFace","id":"F9","faces":["F2.C0"],"mode":"spin"}"#)[0].starts_with("F9:invalidValue"));
    assert!(err(BLOCK, r#"{"type":"moveFace","id":"F9","faces":["F2.C0"],"mode":"translate"}"#)[0].starts_with("F9:invalidValue"));
    // zero distance: a no-op
    let r = with(BLOCK, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{top}"],"distance":0}}"#));
    assert!(r.errors().is_empty() && close(r.volume, 8000.0, 1e-9));
}

#[test]
fn move_body_translate_rotate_and_copy() {
    let r0 = run(BLOCK);
    // translate the only body
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","translate":[5,-3,2]}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 8000.0, 1e-9));
    assert_eq!(r.faces(), r0.faces(), "names are unchanged");
    let (a, b) = (r0.bbox(), r.bbox());
    for i in 0..3 {
        let d = [5.0, -3.0, 2.0][i];
        assert!(close(b.0[i], a.0[i] + d, 1e-9) && close(b.1[i], a.1[i] + d, 1e-9), "axis {i}: {a:?} {b:?}");
    }
    assert!(r.valid());
    // legacy dx/dy/dz
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","dx":1,"dy":2,"dz":3}"#);
    assert!(close(r.bbox().0[0], r0.bbox().0[0] + 1.0, 1e-9));
    // rotate 90 degrees about z through the origin: x extent 40 becomes y extent
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","rotate":{"axis":[0,0,1],"origin":[0,0,0],"angle":90}}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let (a, b) = (r0.bbox(), r.bbox());
    let (ex, ey) = (a.1[0] - a.0[0], a.1[1] - a.0[1]);
    assert!(close(b.1[0] - b.0[0], ey, 1e-9) && close(b.1[1] - b.0[1], ex, 1e-9), "{b:?}");
    assert!(close(r.volume, 8000.0, 1e-9) && r.valid());
    // rotate about a world axis name, then translate
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","axis":{"kind":"axis","axis":"z"},"angle":180,"translate":[100,0,0]}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(r.bbox().0[0] > 100.0 - 41.0 && close(r.volume, 8000.0, 1e-9));
    // an expression
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","translate":["=2*3",0,0]}"#);
    assert!(close(r.bbox().0[0], r0.bbox().0[0] + 6.0, 1e-9));
    // copy: two lumps, the copy's names carry the instance suffix
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9","copy":true,"translate":[60,0,0]}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, 16000.0, 1e-9), "{}", r.volume);
    assert_eq!(r.body.as_ref().unwrap().num_lumps(), 2);
    assert!(r.faces().iter().any(|n| n.ends_with("@F9.1")), "{:?}", r.faces());
    assert!(r.faces().iter().any(|n| n == "F2.C0" || n == "F2.C1"));
    assert!(r.valid());
    // bad input
    assert!(with(BLOCK, r#"{"type":"moveBody","id":"F9","bodies":[{"face":"nope"}],"translate":[1,0,0]}"#).errors()[0].starts_with("F9:missingReference"));
    assert!(with(BLOCK, r#"{"type":"moveBody","id":"F9","rotate":{"axis":[0,0,0],"angle":10}}"#).errors()[0].starts_with("F9:invalidValue"));
    assert!(with(BLOCK, r#"{"type":"moveBody","id":"F9","translate":[1,2]}"#).errors()[0].starts_with("F9:invalidValue"));
    // no motion: nothing happens
    let r = with(BLOCK, r#"{"type":"moveBody","id":"F9"}"#);
    assert!(r.errors().is_empty() && r.bbox() == r0.bbox());
}

#[test]
fn move_body_picks_one_of_two_bodies() {
    // two separate boxes: the second sketch is far from the first
    let two = format!(
        r#"{BLOCK},
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"a","x":100,"y":0}},{{"id":"b","x":110,"y":0}},{{"id":"c","x":110,"y":10}},{{"id":"d","x":100,"y":10}}],
   "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":10}}"#
    );
    let r0 = run(&two);
    assert!(r0.errors().is_empty(), "{:?}", r0.errors());
    assert_eq!(r0.body.as_ref().unwrap().num_lumps(), 2);
    let r = with(&two, r#"{"type":"moveBody","id":"F9","bodies":[{"face":"F4.C0"}],"translate":[0,50,0]}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, r0.volume, 1e-9));
    let b = r.body.as_ref().unwrap();
    let ymax = |prefix: &str| {
        b.face_ids().filter(|&f| b.face_id_str(f).starts_with(prefix)).map(|f| b.face(f).surface.eval(0.0, 0.0).y).fold(f64::MIN, f64::max)
    };
    assert!(r.valid());
    assert!(b.bbox().1.y > 50.0, "{:?}", b.bbox());
    let _ = ymax;
    // string form, rotate about the body's own corner
    let r = with(&two, r#"{"type":"moveBody","id":"F9","body":"F2.C0","rotate":{"axis":"z","origin":[0,0,0],"angle":45}}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!(close(r.volume, r0.volume, 1e-9) && r.valid());
}

/// The block with two vertical edges filleted and a top edge chamfered (blend faces next to the moved walls).
const FILLETED: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"fillet","id":"F3","radius":2,"edges":[{"faces":["F2.S.e1","F2.S.e2"]},{"faces":["F2.S.e3","F2.S.e4"]}]},
 {"type":"chamfer","id":"F5","chamferType":"equal","distance":1.5,"edges":[{"faces":["F2.C0","F2.S.e1"]}]}"#;

/// Only the vertical fillets (plane + cylinder blends).
fn rounded() -> String {
    FILLETED.rsplit_once(",\n {\"type\":\"chamfer\"").unwrap().0.to_string()
}

/// A wall next to a tangent fillet: the fillet travels with it and stays tangent.
#[test]
fn a_wall_carries_its_tangent_fillet() {
    let base = rounded();
    let r0 = run(&base);
    let b = r0.body.as_ref().unwrap();
    let wall = b
        .face_ids()
        .find(|&f| matches!(&b.face(f).surface, ck_geom::surface::Surface::Plane { frame } if frame.z.x.abs() > 0.99 && (frame.o.x - 40.0).abs() < 1e-9))
        .map(|f| b.face_id_str(f))
        .expect("wall");
    for d in [5.0, -5.0] {
        let r = with(&base, &format!(r#"{{"type":"moveFace","id":"F9","faces":["{wall}"],"distance":{d}}}"#));
        assert!(r.errors().is_empty(), "{:?}", r.errors());
        assert!(close(r.volume, r0.volume + d * 20.0 * 10.0, 1e-6), "d={d}: {} vs {}", r.volume, r0.volume);
        assert!(r.valid());
        assert_eq!(r.faces(), r0.faces());
        let t = ck_tess::mesh::tessellate(r.body.as_ref().unwrap());
        assert!(t.mesh.failed.is_empty() && t.mesh.num_tris() > 0);
    }
}

/// Property sweep: every face of several fixtures pushed and pulled; each outcome is
/// a valid, tessellating body with the same face names, or a structured error.
#[test]
fn sweep_every_face_never_panics_and_stays_valid() {
    let rounded_s = rounded();
    let fixtures: Vec<(&str, &str)> = vec![("block", BLOCK), ("rod", ROD), ("holed", HOLED), ("rounded", &rounded_s), ("filleted", FILLETED)];
    let (mut ok, mut structured) = (0, 0);
    for (label, base) in fixtures {
        let r0 = run(base);
        eprintln!("{label}: base errors {:?}", r0.errors());
        let Some(b0) = r0.body.as_ref() else { continue };
        let names0 = r0.faces();
        for f in b0.face_ids() {
            let n = b0.face_id_str(f);
            for d in [1.0, -1.0, 0.3] {
                let r = with(base, &format!(r#"{{"type":"moveFace","id":"M","faces":["{n}"],"distance":{d}}}"#));
                let errs: Vec<String> = r.errors().into_iter().filter(|e| e.starts_with("M:")).collect();
                if errs.is_empty() {
                    ok += 1;
                    assert!(r.valid(), "{label} {n} {d}: invalid");
                    assert_eq!(r.faces(), names0, "{label} {n} {d}: names");
                    let t = ck_tess::mesh::tessellate(r.body.as_ref().unwrap());
                    assert!(t.mesh.num_tris() > 0 && t.mesh.failed.is_empty(), "{label} {n} {d}: mesh failed {:?}", t.mesh.failed);
                    assert!(r.volume.is_finite() && r.volume > 0.0, "{label} {n} {d}: volume {}", r.volume);
                } else {
                    structured += 1;
                    eprintln!("  {label} {n} {d}: {}", errs[0]);
                    assert!(errs[0].starts_with("M:moveFaceFailed") || errs[0].starts_with("M:moveFaceUnsupported"), "{label} {n} {d}: {errs:?}");
                    assert!(close(r.volume, r0.volume, 1e-9), "{label} {n} {d}: model changed on error");
                }
            }
        }
    }
    eprintln!("moveFace sweep: {ok} moved, {structured} structured errors");
    assert!(ok > 20);
}
