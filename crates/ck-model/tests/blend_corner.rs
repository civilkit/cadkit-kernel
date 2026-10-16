//! Mixed vertices (two convex rim edges and a concave wall edge), unequal radii at a corner, and tangent-chain
//! propagation. Reference volumes: OCCT (tools/blend_reference.json, tools/blend_corpus.mjs).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
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

/// (errors, warnings, volume, validity issues) of the part made of `features` (JSON text, comma separated).
fn run_all(features: &str) -> (Vec<String>, Vec<String>, f64, usize) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = res.errors.iter().map(|e| e.error.code.clone()).collect();
    let warns: Vec<String> = res.warnings.iter().map(|e| e.error.code.clone()).collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, warns, ck_topo::mass::mass_props(b).volume, ck_topo::check::check(b, 1e-6).len())
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

fn extrude(id: &str, sketch: &str, op: &str, depth: f64) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"{op}","sketch":"{sketch}","end":"blind","depth":{depth}}}"#)
}

const TOP: &str = r#"{"kind":"std","id":"top"}"#;

fn pocket() -> String {
    format!(
        "{},{},{},{}",
        rect("F1", TOP, 0.0, 0.0, 40.0, 40.0),
        extrude("F2", "F1", "add", 20.0),
        rect("F3", r#"{"kind":"face","id":"F2.C1"}"#, 10.0, 10.0, 30.0, 30.0),
        extrude("F4", "F3", "cut", 8.0)
    )
}

const RIM1: &str = r#"{"faces":["F2.C1","F4.S.e4"],"at":[10,20,-20]}"#;
const RIM2: &str = r#"{"faces":["F2.C1","F4.S.e1"],"at":[20,20,-10]}"#;
const WALL: &str = r#"{"faces":["F4.S.e1","F4.S.e4"],"at":[10,16,-10]}"#;

#[test]
fn mixed_vertex_does_not_depend_on_the_selection_order() {
    // OCCT: 40 x 40 x 20 block with a 20 x 20 x 8 pocket, r = 2 on the two rim edges and the wall edge of one pocket corner
    let want = 28772.665456976367;
    let mut vols = vec![];
    for order in [[RIM1, RIM2, WALL], [WALL, RIM2, RIM1], [RIM2, WALL, RIM1]] {
        let f = format!(r#"{},{{"type":"fillet","id":"F5","radius":2,"edges":[{},{},{}]}}"#, pocket(), order[0], order[1], order[2]);
        let (errs, _, vol, issues) = run_all(&f);
        assert!(errs.is_empty(), "{errs:?}");
        assert_eq!(issues, 0);
        vols.push(vol);
    }
    for v in &vols {
        assert!((v - want).abs() < want * 1e-5, "{vols:?}");
        assert!((v - vols[0]).abs() < 1e-6, "{vols:?}");
    }
}

#[test]
fn unequal_radii_make_a_valid_solid_and_warn() {
    // 30 cube: top-front edge r 3 and top-right edge r 5, meeting at a corner (OCCT 26786.47)
    let cube = format!("{},{}", rect("F1", TOP, 0.0, 0.0, 30.0, 30.0), extrude("F2", "F1", "add", 30.0));
    let fillet = |r2: f64| {
        format!(
            r#"{cube},{{"type":"fillet","id":"F3","radius":3,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[15,30,0],"radius":3}},{{"faces":["F2.C1","F2.S.e2"],"at":[30,30,-15],"radius":{r2}}}]}}"#
        )
    };
    let (errs, warns, vol, issues) = run_all(&fillet(5.0));
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    assert!(warns.contains(&"cornerNotTangent".to_string()), "{warns:?}");
    assert!((vol - 26786.469785536647).abs() < 26786.0 * 2e-3, "{vol}");
    // equal radii: the ordinary builder, no warning
    let (errs, warns, _, _) = run_all(&fillet(3.0));
    assert!(errs.is_empty() && warns.is_empty(), "{errs:?} {warns:?}");
}

#[test]
fn mixed_vertex_with_a_wall_radius_and_a_rim_radius() {
    // OCCT: rims r = 2, wall edge r = 4 on the same pocket corner
    let want = 28794.004096462584;
    let pick = |e: &str, r: f64| e.replacen('}', &format!(r#","radius":{r}}}"#), 1);
    let f = format!(r#"{},{{"type":"fillet","id":"F5","radius":2,"edges":[{},{},{}]}}"#, pocket(), pick(RIM1, 2.0), pick(RIM2, 2.0), pick(WALL, 4.0));
    let (errs, warns, vol, issues) = run_all(&f);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    assert!(warns.is_empty(), "{warns:?}");
    assert!((vol - want).abs() < want * 1e-5, "{vol}");
}

#[test]
fn tangent_chain_propagation() {
    // 40 x 30 x 10 plate with rounded vertical edges (r 6); one top edge picked, `propagate` takes the whole outline
    let plate = format!(
        r#"{},{},{{"type":"fillet","id":"F3","radius":6,"edges":[{{"faces":["F2.S.e1","F2.S.e4"],"at":[0,5,0]}},{{"faces":["F2.S.e1","F2.S.e2"],"at":[40,5,0]}},{{"faces":["F2.S.e2","F2.S.e3"],"at":[40,5,-30]}},{{"faces":["F2.S.e3","F2.S.e4"],"at":[0,5,-30]}}]}}"#,
        rect("F1", TOP, 0.0, 0.0, 40.0, 30.0),
        extrude("F2", "F1", "add", 10.0)
    );
    let chamfer = format!(r#"{plate},{{"type":"chamfer","id":"F4","chamferType":"equal","distance":1.5,"propagate":true,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}}]}}"#);
    let (errs, _, vol, issues) = run_all(&chamfer);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(issues, 0);
    // OCCT propagates along the tangent outline by itself: 11548.596
    assert!((vol - 11548.596146204154).abs() < 11548.6 * 1e-4, "{vol}");
}
