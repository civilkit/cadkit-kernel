//! `op: "newBody"` and body names (docs/adr/0004-multiple-bodies-in-one-part.md, step 1; docs/spec-agent-interface.md,
//! "Separate bodies: `newBody` and body names"). Volumes are analytic: boxes with integer overlaps.

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
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn error_params(&self, code: &str) -> ck_json::Json {
        let e = self.result.errors.iter().find(|e| e.error.code == code).unwrap_or_else(|| panic!("no {code}: {:?}", self.errors()));
        ck_json::Json::Obj(e.error.params.clone())
    }
    fn volume(&self) -> f64 {
        self.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume)
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
    fn faces(&self) -> Vec<String> {
        let b = self.body.as_ref().unwrap();
        let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
        v.sort();
        v
    }
    /// Body name of every live lump, in lump order.
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body.as_ref().unwrap();
        b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
    }
    /// The `massProps {bodies:true}` entries.
    fn mass_bodies(&self) -> Vec<ck_json::Json> {
        let j = ck_topo::mass::mass_props_json(self.body.as_ref().unwrap(), true, None);
        j.get("bodies").and_then(|b| b.as_arr()).unwrap().to_vec()
    }
    /// Volume of the lump with this name.
    fn volume_of(&self, name: &str) -> f64 {
        self.mass_bodies().iter().find(|b| b.get_str("name") == Some(name)).and_then(|b| b.get("volume")).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("no lump named {name}: {:?}", self.names()))
    }
    fn clean(&self) {
        assert!(self.errors().is_empty(), "{:?}", self.errors());
        let b = self.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "invalid result");
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

fn rect_sketch(id: &str, x0: f64, x1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{x1},"y":0}},{{"id":"p3","x":{x1},"y":10}},{{"id":"p4","x":{x0},"y":10}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

/// A box [x0,x1] x [0,10] on the top plane, extruded 10: sketch `S<id>` + extrude `F<id>`; `op` and `extra`
/// (more extrude fields, starting with a comma) as given.
fn boxf(id: u32, x0: f64, x1: f64, op: &str, extra: &str) -> String {
    format!(
        r#"{},{{"type":"extrude","id":"F{id}","op":"{op}","sketch":"S{id}","end":"blind","depth":10{extra}}}"#,
        rect_sketch(&format!("S{id}"), x0, x1)
    )
}

/// Box A [0,10] and box B [5,15] (`newBody`, named "B"): volumes 1000 each, common part 500.
fn two_boxes() -> String {
    format!("{},{}", boxf(2, 0.0, 10.0, "add", ""), boxf(3, 5.0, 15.0, "newBody", r#","body":"B""#))
}

fn comb(op: &str, target: &str, tools: &str, extra: &str) -> String {
    format!(r#"{{"type":"combine","id":"C","op":"{op}","target":{target},"tools":[{tools}]{extra}}}"#)
}

const A: &str = r#"{"face":"F2.C1"}"#;
const B: &str = r#"{"body":"B"}"#;

#[test]
fn overlapping_boxes_stay_two_lumps() {
    let r = run(&two_boxes());
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert!((r.volume() - 2000.0).abs() < 1e-6, "{}", r.volume());
    let bodies = r.mass_bodies();
    assert_eq!(bodies.len(), 2);
    assert_eq!(r.names(), vec![None, Some("B".to_string())]);
    assert_eq!(bodies[0].get_str("name"), None);
    assert_eq!(bodies[1].get_str("name"), Some("B"));
    assert!((r.volume_of("B") - 1000.0).abs() < 1e-6);
    // the same feature with `add` would have merged them
    let merged = run(&format!("{},{}", boxf(2, 0.0, 10.0, "add", ""), boxf(3, 5.0, 15.0, "add", "")));
    assert_eq!(merged.lumps(), 1);
    assert!((merged.volume() - 1500.0).abs() < 1e-6);
}

#[test]
fn combine_union_intersect_cut_of_the_overlap() {
    for (op, vol) in [("union", 1500.0), ("intersect", 500.0), ("cut", 500.0)] {
        let r = run(&format!("{},{}", two_boxes(), comb(op, A, B, "")));
        r.clean();
        assert_eq!(r.lumps(), 1, "{op}");
        assert!((r.volume() - vol).abs() < 1e-6, "{op}: {}", r.volume());
    }
    // the target's name stays on the result; a `body` on the combine renames it
    let r = run(&format!("{},{}", two_boxes(), comb("cut", B, A, "")));
    r.clean();
    assert_eq!(r.names(), vec![Some("B".to_string())], "target name carries over");
    assert!((r.volume() - 500.0).abs() < 1e-6);
    let r = run(&format!("{},{}", two_boxes(), comb("union", B, A, r#","body":"Joined""#)));
    r.clean();
    assert_eq!(r.names(), vec![Some("Joined".to_string())]);
    assert!((r.volume() - 1500.0).abs() < 1e-6);
}

#[test]
fn new_body_into_an_empty_document_is_a_plain_solid() {
    let added = run(&boxf(2, 0.0, 10.0, "add", ""));
    let fresh = run(&boxf(2, 0.0, 10.0, "newBody", r#","body":"Only""#));
    added.clean();
    fresh.clean();
    assert_eq!(added.faces(), fresh.faces());
    assert!((added.volume() - fresh.volume()).abs() < 1e-9);
    assert_eq!(fresh.names(), vec![Some("Only".to_string())]);
    // the aliases read, `newBody` is what is written
    for alias in ["new", "separate"] {
        let r = run(&boxf(2, 0.0, 10.0, alias, ""));
        r.clean();
        assert_eq!(r.faces(), added.faces(), "{alias}");
    }
}

#[test]
fn new_body_extrude_thin_and_drafted() {
    let a = boxf(2, 0.0, 10.0, "add", "");
    // thin: a 2 mm wall inside the outline of A: 10x10 - 6x6 = 64, depth 10
    let thin = run(&format!("{a},{}", boxf(3, 0.0, 10.0, "newBody", r#","body":"Wall","thin":{"thickness":2,"side":"inside"}"#)));
    thin.clean();
    assert_eq!(thin.lumps(), 2);
    assert!((thin.volume() - (1000.0 + 640.0)).abs() < 1e-6, "{}", thin.volume());
    assert!((thin.volume_of("Wall") - 640.0).abs() < 1e-6);
    // drafted: a frustum, 10x10 base tapering over 10 mm (the draft direction is the extrude's own)
    let t = 5f64.to_radians().tan();
    let top = |s: f64| s * s;
    let frustum = |s0: f64, s1: f64| 10.0 / 3.0 * (top(s0) + top(s1) + s0 * s1);
    let drafted = run(&format!("{a},{}", boxf(3, 0.0, 10.0, "newBody", r#","body":"Cone","draft":5"#)));
    drafted.clean();
    assert_eq!(drafted.lumps(), 2);
    let v = drafted.volume_of("Cone");
    let (inward, outward) = (frustum(10.0, 10.0 - 20.0 * t), frustum(10.0, 10.0 + 20.0 * t));
    assert!((v - inward).abs() < 1e-6 || (v - outward).abs() < 1e-6, "{v} vs {inward} / {outward}");
    assert!((drafted.volume() - 1000.0 - v).abs() < 1e-6);
}

#[test]
fn new_body_revolve() {
    // a cylinder r 3 x 6 about the axis x = 5, overlapping A
    let rev = |op: &str| {
        format!(
            r#"{{"type":"sketch","id":"SR","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":5,"y":-3}},{{"id":"b","x":8,"y":-3}},{{"id":"c","x":8,"y":3}},{{"id":"d","x":5,"y":3}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}},{{"type":"line","id":"ax","p1":"a","p2":"d","construction":true}}],"constraints":[]}},{{"type":"revolve","id":"FR","op":"{op}","sketch":"SR","axis":"ax","angle":360,"body":"Pin"}}"#
        )
    };
    let a = boxf(2, 0.0, 10.0, "add", "");
    let r = run(&format!("{a},{}", rev("newBody")));
    r.clean();
    assert_eq!(r.lumps(), 2);
    let cyl = std::f64::consts::PI * 9.0 * 6.0;
    assert!((r.volume_of("Pin") - cyl).abs() < 1e-6 * cyl, "{}", r.volume_of("Pin"));
    assert!((r.volume() - 1000.0 - cyl).abs() < 1e-6 * cyl);
}

fn square(id: &str, plane: &str, h: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"a","x":-{h},"y":-{h}}},{{"id":"b","x":{h},"y":-{h}}},{{"id":"c","x":{h},"y":{h}}},{{"id":"d","x":-{h},"y":{h}}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}}"#
    )
}

#[test]
fn new_body_loft() {
    // a straight loft between a 10x10 and a 10x10 square 20 apart = a 10x10x20 prism, standing inside box A's footprint
    let plane = r#"{"type":"plane","id":"P","mode":"offset","base":{"kind":"std","id":"top"},"offset":20}"#;
    let a = boxf(2, -20.0, 20.0, "add", "");
    let l = format!(
        r#"{a},{},{plane},{},{{"type":"loft","id":"L","op":"newBody","body":"Tower","sketches":["S1","S2"]}}"#,
        square("S1", r#"{"kind":"std","id":"top"}"#, 5.0),
        square("S2", r#"{"kind":"feature","id":"P"}"#, 5.0)
    );
    let r = run(&l);
    r.clean();
    assert_eq!(r.lumps(), 2, "{:?}", r.names());
    assert!((r.volume_of("Tower") - 2000.0).abs() < 1e-6, "{}", r.volume_of("Tower"));
}

#[test]
fn new_body_sweep() {
    // a 4x4 square swept along a 20 mm line from the origin, next to / over box A
    let a = boxf(2, 0.0, 10.0, "add", "");
    let prof = square("SP", r#"{"kind":"std","id":"top"}"#, 2.0);
    let path = r#"{"type":"sketch","id":"P","plane":{"kind":"std","id":"front"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":0,"y":20}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}"#;
    let sw = r#"{"type":"sweep","id":"W","op":"newBody","body":"Rod","sketch":"SP","path":{"kind":"sketch","id":"P"}}"#;
    let doc = format!("{a},{prof},{path},{sw}");
    let r = run(&doc);
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert!((r.volume_of("Rod") - 320.0).abs() < 1e-6, "{}", r.volume_of("Rod"));
    assert!((r.volume() - 1320.0).abs() < 1e-6);
    // the rod crosses A: `add` would have merged them (volume less than the sum), newBody keeps both whole
    let merged = run(&doc.replace(r#""op":"newBody","body":"Rod","#, r#""op":"add","#));
    assert_eq!(merged.lumps(), 1);
    assert!(merged.volume() < 1320.0 - 1.0, "{}", merged.volume());
}

#[test]
fn a_cut_limited_to_one_body() {
    // A [0,10], B [5,15]; the tool T [3,8] crosses both. Cutting B only leaves A alone.
    let doc = format!("{},{},{}", two_boxes(), boxf(4, 3.0, 8.0, "newBody", r#","body":"T""#), comb("cut", B, r#"{"body":"T"}"#, ""));
    let r = run(&doc);
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert!((r.volume() - 1700.0).abs() < 1e-6, "{}", r.volume());
    assert!((r.volume_of("B") - 700.0).abs() < 1e-6);
    // A, the lump with the add feature, still has its 1000
    let first = &r.mass_bodies()[0];
    assert!((first.get("volume").and_then(|v| v.as_f64()).unwrap() - 1000.0).abs() < 1e-6);
    assert_eq!(first.get_str("name"), None);
}

fn mv(id: &str, extra: &str) -> String {
    format!(r#"{{"type":"moveBody","id":"{id}","translate":[0,30,0]{extra}}}"#)
}

#[test]
fn move_copy_names_the_copy_and_selects_by_name() {
    let a = boxf(2, 0.0, 10.0, "add", "");
    let r = run(&format!("{a},{}", mv("M", r#","copy":true,"body":"Case""#)));
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert_eq!(r.names(), vec![None, Some("Case".to_string())], "the original stays unnamed");
    assert!(r.faces().iter().any(|f| f.ends_with("@M.1")));
    // {body:"Case"} moves only the copy
    let r2 = run(&format!("{a},{},{}", mv("M", r#","copy":true,"body":"Case""#), r#"{"type":"moveBody","id":"N","bodies":[{"body":"Case"}],"translate":[0,0,40]}"#));
    r2.clean();
    let com = |r: &Run, i: usize| {
        let c = r.mass_bodies()[i].get("com").and_then(|c| c.as_arr()).unwrap().iter().map(|x| x.as_f64().unwrap()).collect::<Vec<_>>();
        c
    };
    let (c0a, c1a) = (com(&r, 0), com(&r, 1));
    let (c0b, c1b) = (com(&r2, 0), com(&r2, 1));
    assert!((0..3).all(|k| (c0a[k] - c0b[k]).abs() < 1e-9), "the original did not move: {c0a:?} {c0b:?}");
    let moved: f64 = (0..3).map(|k| (c1b[k] - c1a[k]).abs()).sum();
    assert!((moved - 40.0).abs() < 1e-6, "{c1a:?} -> {c1b:?}");
    // a second copy with the same name
    let dup = run(&format!("{a},{},{}", mv("M", r#","copy":true,"body":"Case""#), mv("M2", r#","copy":true,"body":"Case""#)));
    assert_eq!(dup.errors(), vec!["M2:bodyNameConflict"]);
    let p = dup.error_params("bodyNameConflict");
    assert_eq!(p.get_str("name"), Some("Case"));
    let fs: Vec<&str> = p.get("features").and_then(|f| f.as_arr()).unwrap().iter().filter_map(|x| x.as_str()).collect();
    assert_eq!(fs, vec!["M", "M2"]);
    assert_eq!(dup.lumps(), 2, "the failed feature leaves the body as it was");
    // an older document: a string `body` without `copy` is still a selection
    let legacy = run(&format!("{a},{}", mv("M", r#","body":"F2.C1""#)));
    legacy.clean();
    assert_eq!(legacy.lumps(), 1);
}

#[test]
fn names_survive_fillets_cuts_and_unrelated_edits() {
    // A [0,10] and B [20,30] (apart: a blend does not run on lumps that overlap, which is independent of names)
    let base = format!("{},{}", boxf(2, 0.0, 10.0, "add", ""), boxf(3, 20.0, 30.0, "newBody", r#","body":"B""#));
    let probe = run(&base);
    let b = probe.body.as_ref().unwrap();
    let edge = b
        .edge_ids()
        .find(|&e| {
            let n: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
            n.len() == 2 && n.contains(&"F3.C1".to_string()) && n.contains(&"F3.S.e1".to_string())
        })
        .expect("B edge");
    let ed = b.edge(edge);
    let (p, q) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
    let at = format!("[{},{},{}]", (p.x + q.x) / 2.0, (p.y + q.y) / 2.0, (p.z + q.z) / 2.0);
    let fillet = format!(r#"{{"type":"fillet","id":"FF","radius":1,"edges":[{{"faces":["F3.C1","F3.S.e1"],"at":{at}}}]}}"#);
    let r = run(&format!("{base},{fillet}"));
    r.clean();
    assert_eq!(r.names(), vec![None, Some("B".to_string())], "fillet");
    assert!(r.volume_of("B") < 1000.0);
    // a later cut into B: the name stays on the cut lump
    let cut = r#"{"type":"sketch","id":"SC","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":22,"y":2},{"id":"b","x":24,"y":2},{"id":"c","x":24,"y":4},{"id":"d","x":22,"y":4}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},{"type":"extrude","id":"FC","op":"cut","flip":true,"sketch":"SC","end":"blind","depth":3}"#;
    let r = run(&format!("{base},{fillet},{cut}"));
    r.clean();
    assert_eq!(r.names(), vec![None, Some("B".to_string())], "cut");
    assert!((r.volume_of("B") - (1000.0 - 12.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 10.0)).abs() < 1e-3, "{}", r.volume_of("B"));
    // a regen after an unrelated edit (box A made wider): same names, same B
    let wider = format!("{},{}", boxf(2, 0.0, 12.0, "add", ""), boxf(3, 20.0, 30.0, "newBody", r#","body":"B""#));
    let r2 = run(&wider);
    r2.clean();
    assert_eq!(r2.names(), vec![None, Some("B".to_string())]);
    assert!((r2.volume_of("B") - 1000.0).abs() < 1e-6);
    // names also follow the lump when it is not the last one: B first, A (add) second
    let r3 = run(&format!("{},{}", boxf(3, 20.0, 30.0, "newBody", r#","body":"B""#), boxf(2, 0.0, 10.0, "add", "")));
    r3.clean();
    assert_eq!(r3.names().iter().flatten().count(), 1);
}

#[test]
fn refusals() {
    let a = boxf(2, 0.0, 10.0, "add", "");
    // newBody on features that do not take it: invalidValue with a detail and a hint
    for f in [
        r#"{"type":"rib","id":"R","op":"newBody","sketch":"S2","thickness":2}"#,
        r#"{"type":"thread","id":"T","op":"newBody","face":"F2.C1","standard":"metric","size":"M5"}"#,
        r#"{"type":"hole","id":"H","op":"newBody","sketch":"S2","kind":"simple","diameter":3,"end":"through"}"#,
    ] {
        let r = run(&format!("{a},{f}"));
        let e = &r.result.errors;
        assert_eq!(e.len(), 1, "{f}: {:?}", r.errors());
        assert_eq!(e[0].error.code, "invalidValue", "{f}");
        let p = r.error_params("invalidValue");
        let detail = p.get_str("detail").unwrap();
        assert!(detail.starts_with("operation newBody is not supported by "), "{detail}");
        assert!(p.get_str("hint").is_some());
        assert!((r.volume() - 1000.0).abs() < 1e-6, "{f}: body untouched");
    }
    // a name on an extrude that is not newBody
    let r = run(&format!("{a},{}", boxf(3, 5.0, 15.0, "add", r#","body":"X""#)));
    assert_eq!(r.errors(), vec!["F3:invalidValue"]);
    // a selector that matches nothing: missingReference with the nearest names
    let r = run(&format!("{},{}", two_boxes(), comb("cut", A, r#"{"body":"Bee"}"#, "")));
    assert_eq!(r.errors(), vec!["C:missingReference"]);
    let p = r.error_params("missingReference");
    assert_eq!(p.get_str("name"), Some("Bee"));
    assert_eq!(p.get("nearest").and_then(|n| n.as_arr()).unwrap()[0].as_str(), Some("B"));
    assert_eq!(r.lumps(), 2);
    // two lumps with one name
    let r = run(&format!("{},{}", two_boxes(), boxf(4, 8.0, 12.0, "newBody", r#","body":"B""#)));
    assert_eq!(r.errors(), vec!["F4:bodyNameConflict"]);
    let p = r.error_params("bodyNameConflict");
    assert_eq!(p.get_str("name"), Some("B"));
    let fs: Vec<&str> = p.get("features").and_then(|f| f.as_arr()).unwrap().iter().filter_map(|x| x.as_str()).collect();
    assert_eq!(fs, vec!["F3", "F4"]);
    assert_eq!(r.lumps(), 2);
    // a name on a combine that another surviving lump has
    let r = run(&format!("{},{}", two_boxes(), comb("cut", B, A, r#","keepTools":true,"body":"B""#)));
    assert!(r.errors().is_empty(), "same lump keeps its name: {:?}", r.errors());
    let r = run(&format!("{},{},{}", two_boxes(), boxf(4, 20.0, 30.0, "newBody", r#","body":"Z""#), comb("cut", B, A, r#","body":"Z""#)));
    assert_eq!(r.errors(), vec!["C:bodyNameConflict"]);
}

#[test]
fn a_name_that_selects_several_lumps() {
    // a pattern row sharing an origin: two copies of A, both named "Row" by one moveBody
    let a = boxf(2, 0.0, 10.0, "add", "");
    let two_copies = format!(
        r#"{a},{{"type":"moveBody","id":"M","copy":true,"body":"Row","translate":[0,30,0]}}"#
    );
    // one name on two lumps: copies made by the same feature share the name (select all), a combine target must be one
    let both = run(&format!("{two_copies},{{\"type\":\"moveBody\",\"id\":\"M4\",\"bodies\":[{{\"face\":\"F2.C1\"}},{{\"body\":\"Row\"}}],\"copy\":true,\"body\":\"Row2\",\"translate\":[0,0,30]}}"));
    both.clean();
    let row2 = both.names().iter().filter(|n| n.as_deref() == Some("Row2")).count();
    assert_eq!(row2, 2, "{:?}", both.names());
    let amb = run(&format!("{two_copies},{{\"type\":\"moveBody\",\"id\":\"M4\",\"bodies\":[{{\"face\":\"F2.C1\"}},{{\"body\":\"Row\"}}],\"copy\":true,\"body\":\"Row2\",\"translate\":[0,0,30]}},{}", comb("union", r#"{"body":"Row2"}"#, A, "")));
    assert_eq!(amb.errors(), vec!["C:ambiguousBody"]);
    let p = amb.error_params("ambiguousBody");
    assert_eq!(p.get_str("name"), Some("Row2"));
    assert_eq!(p.get("candidates").and_then(|c| c.as_arr()).unwrap().len(), 2);
    // deleteBody takes the selector, too
    let del = run(&format!("{},{{\"type\":\"deleteBody\",\"id\":\"D\",\"mode\":\"delete\",\"bodies\":[{{\"body\":\"B\"}}]}}", two_boxes()));
    del.clean();
    assert_eq!(del.lumps(), 1);
    assert!((del.volume() - 1000.0).abs() < 1e-6);
}

#[test]
fn documents_without_new_body_or_body_regenerate_as_before() {
    let r = run(&format!("{},{}", boxf(2, 0.0, 10.0, "add", ""), boxf(3, 5.0, 15.0, "add", "")));
    r.clean();
    assert_eq!(r.names(), vec![None]);
    let j = r.mass_bodies();
    assert!(j.iter().all(|b| b.get("name").is_none()));
}

#[test]
fn move_copy_in_place_still_makes_the_named_copy() {
    // no translate and no rotation: the copy lies exactly on the original (two lumps, the copy named)
    let a = boxf(2, 0.0, 10.0, "add", "");
    let r = run(&format!(r#"{a},{{"type":"moveBody","id":"M","copy":true,"body":"Case"}}"#));
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert_eq!(r.names(), vec![None, Some("Case".to_string())]);
    assert!((r.volume_of("Case") - 1000.0).abs() < 1e-6);
}

#[test]
fn resolve_can_restrict_to_a_named_body() {
    // a copy in place lies on the original: every face exists twice, `body` picks the copy's
    let a = boxf(2, 0.0, 10.0, "add", "");
    let r = run(&format!(r#"{a},{{"type":"moveBody","id":"M","copy":true,"body":"Case"}}"#));
    r.clean();
    let q = |d: &str| ck_ops::agent::resolve(&r.body.as_ref().unwrap(), &ck_json::parse(&format!(r#"{{"description":{d}}}"#)).unwrap());
    let count = |j: ck_json::Json| j.get("results").and_then(|a| a.as_arr()).and_then(|a| a[0].get("count")).and_then(|c| c.as_f64()).unwrap();
    assert_eq!(count(q(r#"{"kind":"face"}"#)), 12.0);
    assert_eq!(count(q(r#"{"kind":"face","body":"Case"}"#)), 6.0);
    assert_eq!(count(q(r#"{"kind":"edge","curve":"line","body":"Case"}"#)), 12.0);
    assert_eq!(count(q(r#"{"kind":"face","body":"Nope"}"#)), 0.0);
}

#[test]
fn shell_acts_on_the_lump_of_its_face_not_on_a_coincident_copy() {
    // a copy in place: every face exists twice; shelling the copy must leave the original alone and not fail on the coincidence
    let a = boxf(2, 0.0, 10.0, "add", "");
    let one = run(&format!(r#"{a},{{"type":"shell","id":"S","faces":["F2.C1"],"thickness":1}}"#));
    one.clean();
    let r = run(&format!(r#"{a},{{"type":"moveBody","id":"M","copy":true,"body":"Case"}},{{"type":"shell","id":"S","faces":["F2.C1@M.1"],"thickness":1}}"#));
    r.clean();
    assert_eq!(r.lumps(), 2);
    assert!((r.volume_of("Case") - one.volume()).abs() < 1e-6, "{} vs {}", r.volume_of("Case"), one.volume());
    assert!((r.volume() - one.volume() - 1000.0).abs() < 1e-6, "the original is still the full box");
    assert_eq!(r.names().iter().filter(|n| n.as_deref() == Some("Case")).count(), 1);
}

/// A fillet on a copy that lies exactly on its original acts on the copy alone.
#[test]
fn fillet_acts_on_the_lump_of_its_edge_not_on_a_coincident_copy() {
    let r = run(&format!(
        r#"{},{{"type":"moveBody","id":"F4","bodies":[{{"body":"A"}}],"copy":true,"body":"C"}},
        {{"type":"fillet","id":"F5","radius":2,"edges":[{{"faces":["F2.C1@F4.1","F2.S.e1@F4.1"]}}]}}"#,
        boxf(2, 0.0, 10.0, "newBody", r#","body":"A""#)
    ));
    r.clean();
    assert!((r.volume_of("A") - 1000.0).abs() < 1e-6, "{:?}", r.names());
    let rounded = 1000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 4.0 * 10.0;
    assert!((r.volume_of("C") - rounded).abs() < 1e-4, "C is {}", r.volume_of("C"));
}
