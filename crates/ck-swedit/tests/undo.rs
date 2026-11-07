//! Histories that start from imported bodies (`undo.rs`): the file's solid with the later steps' material undone is
//! the starting body, and the later steps are read back on it as editable features.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::prefix::regen_part;
use ck_swedit::{rebuild_history, Outcome, Status, Tagged};
use ck_topo::body::{Body, FaceName};
use std::rc::Rc;

fn steps(o: &Outcome) -> Vec<String> {
    o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect()
}

/// The body of `src` (our features), tagged as the file would tag it: every face with the number of the feature
/// that made it, `kinds` giving the SolidWorks kind of a feature (default: extrude) and the source features' faces
/// standing for an imported body. The start body is the same solid with its faces named as an `importBody` `F1`
/// names them.
fn run(src: &str, kinds: &[(i64, &str, &str)]) -> Outcome {
    let fs = match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    };
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        // (a copy a mirror or pattern made carries that feature's tag, as SolidWorks tags it)
        let nm = &body.face(fid).name;
        let copy = nm.suffix.rsplit('@').next().filter(|_| nm.suffix.contains('@')).and_then(|s| s.strip_prefix('F')?.split('.').next()?.parse::<i64>().ok());
        if let Some(t) = copy.or_else(|| nm.feature[1..].parse::<i64>().ok()) {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    let sw: Vec<SwFeature> = fs
        .iter()
        .map(|f| {
            let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
            let cut = f.get_str("op") == Some("cut");
            let (kind, name) = match (kinds.iter().find(|k| k.0 == id), f.get_str("type").unwrap()) {
                (Some(k), _) => (k.1.to_string(), k.2.to_string()),
                (None, "sketch") => ("Sketch".into(), format!("Sketch{id}")),
                (None, "plane") => ("RefPlane".into(), format!("Plane{id}")),
                (None, "extrude") if cut => ("Extrusion".into(), format!("Cut-Extrude{id}")),
                (None, "extrude") => ("Boss-Extrude".into(), format!("Boss-Extrude{id}")),
                (None, t) => panic!("{t}"),
            };
            let dims = f.get_f64("depth").map(|d| vec![SwDim { name: "D1".into(), text: d.to_string(), value: Some(d) }]).unwrap_or_default();
            SwFeature { id, name, kind, dims }
        })
        .collect();
    let mut start: Body = body.clone();
    for (n, f) in start.faces.iter_mut().enumerate() {
        f.name = FaceName::other("F1", &format!("I1F{}", n + 1));
    }
    rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0, start: Some(Rc::new(start)) })
}

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

fn circle(id: &str, plane: &str, x: f64, y: f64, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":{r}}}],"constraints":[]}}"#)
}

const TOP: &str = r#"{"kind":"std","id":"top"}"#;

/// An imported block, then a boss on its top and a hole through both: the starting body is the file's solid with the
/// hole filled and the boss taken off, and both steps rebuild on it to the file's solid exactly.
#[test]
fn imported_block_then_boss_and_hole() {
    let top10 = r#"{"kind":"feature","id":"F3"}"#;
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
{{"type":"plane","id":"F3","mode":"offset","base":{TOP},"offset":10}},
{},{{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":6}},
{},{{"type":"extrude","id":"F7","op":"cut","sketch":"F6","end":"through","flip":true}}]"#,
        rect("F1", TOP, -30.0, -20.0, 30.0, 20.0),
        rect("F4", top10, -10.0, -8.0, 14.0, 8.0),
        circle("F6", TOP, 4.0, 0.0, 3.0),
    );
    let o = run(&src, &[(2, "Imported", "Imported1")]);
    assert!(o.exact(), "{:?}\n{:#?}", o.fallback, steps(&o));
    let names: Vec<&str> = o.steps.iter().filter(|s| s.status == Status::Rebuilt).map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Imported1", "Boss-Extrude5", "Cut-Extrude7"], "{:#?}", steps(&o));
    // the document: the file's solid, the two undo tools (the hole's first), then the steps themselves
    assert_eq!(o.features[0].get_str("type"), Some("importBody"));
    let undo: Vec<&str> = o.features.iter().filter_map(|f| f.get_str("name")).filter(|n| n.starts_with("Undo")).collect();
    assert_eq!(undo, ["Undo Cut-Extrude7", "Undo Boss-Extrude5"]);
}

/// A block read back as usual, a body joined by `Combine` (its faces carry the Combine's tag: nothing to read them
/// from), then a cut: the block stays editable, the Combine step is the file's solid with the cut undone, and the cut
/// rebuilds on it.
#[test]
fn combine_then_cut() {
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
{},{{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":25}},
{},{{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"through","flip":true}}]"#,
        rect("F1", TOP, -30.0, -20.0, 30.0, 20.0),
        circle("F3", TOP, -15.0, 0.0, 8.0),
        circle("F5", TOP, 15.0, 0.0, 4.0),
    );
    let o = run(&src, &[(4, "Combine", "Combine1")]);
    assert!(o.exact(), "{:?}\n{:#?}", o.fallback, steps(&o));
    let names: Vec<&str> = o.steps.iter().filter(|s| s.status == Status::Rebuilt).map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Boss-Extrude2", "Combine1", "Cut-Extrude6"], "{:#?}", steps(&o));
    assert!(o.features.iter().any(|f| f.get_str("type") == Some("importBody")));
}

/// End to end on a committed fixture: a cube with a hole whose cube is taken for an imported body. The part document
/// (with the file in its `blobs`) regenerates through the real `importBody` to exactly the plain import.
#[test]
fn fixture_imported_cube_then_hole() {
    let file = std::fs::read(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ck-sldprt/tests/fixtures/sw2022_cube_hole_5mm.sldprt")).unwrap();
    let mut imp = ck_sldprt::import_sldprt(&file).unwrap();
    // the first feature that made faces (the cube's extrude) becomes an imported body
    let made: Vec<i64> = imp.face_tags.iter().flat_map(|t| t.sw_ids.iter().map(|i| i.feature)).collect();
    let first = imp.meta.features.iter_mut().find(|f| ck_swedit::classify(f) != ck_swedit::Class::Skip && made.contains(&f.id)).unwrap();
    first.kind = "Imported".into();
    let o = rebuild_history(&Tagged::from_import(&imp));
    assert!(o.exact(), "{:?} {:#?}", o.fallback, steps(&o));
    assert!(o.steps.iter().filter(|s| s.status == Status::Rebuilt).count() >= 2, "{:#?}", steps(&o));
    let (b, errs) = regen_part(&o.part_json(Some(&file))).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let b = b.unwrap();
    let p = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "import", None).solid;
    let v = ck_swedit::fastverify::verify(&p, &b);
    assert!(v.exact, "rel {} area {}", v.rel, v.area_rel);
}

#[test]
fn sources_need_a_later_step() {
    let f = |id: i64, kind: &str| SwFeature { id, name: format!("{kind}{id}"), kind: kind.into(), dims: vec![] };
    let feats = [f(1, "Imported"), f(2, "Combine"), f(3, "Extrusion")];
    let tags = vec![vec![(1, 77)], vec![(2, 55)], vec![(3, 83)]];
    assert_eq!(ck_swedit::undo::sources(&feats, &tags).map(|s| s.len()), Some(2));
    assert_eq!(ck_swedit::undo::sources(&feats, &tags[..2]), None);
    // (a source run after steps read back as usual starts the body there)
    let feats = [f(1, "Extrusion"), f(2, "Combine"), f(3, "Extrusion")];
    assert_eq!(ck_swedit::undo::sources(&feats, &tags).map(|s| s.into_iter().collect::<Vec<_>>()), Some(vec![2]));
}

/// An imported block whose two long top edges a later fillet rounds, then a hole: the fillet's material is put back
/// on the file's solid (the corner wedge along each edge, added), so the fillet and the hole rebuild on the starting
/// body to the file's solid exactly.
#[test]
fn imported_block_then_fillet_and_hole() {
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
{{"type":"fillet","id":"F3","radius":3,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[0,-20,10]}},{{"faces":["F2.C1","F2.S.e3"],"at":[0,20,10]}}]}},
{},{{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"through","flip":true}}]"#,
        rect("F1", TOP, -30.0, -20.0, 30.0, 20.0),
        circle("F4", TOP, 4.0, 0.0, 3.0),
    );
    let o = run(&src, &[(2, "Imported", "Imported1"), (3, "Fillet", "Fillet3")]);
    assert!(o.exact(), "{:?}\n{:#?}", o.fallback, steps(&o));
    let names: Vec<&str> = o.steps.iter().filter(|s| s.status == Status::Rebuilt).map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Imported1", "Fillet3", "Cut-Extrude5"], "{:#?}", steps(&o));
    let undo: Vec<&str> = o.features.iter().filter_map(|f| f.get_str("name")).filter(|n| n.starts_with("Undo")).collect();
    assert_eq!(undo, ["Undo Cut-Extrude5", "Undo Fillet3", "Undo Fillet3"]);
    assert!(o.features.iter().any(|f| f.get_str("type") == Some("fillet")));
}

/// An imported block with a round boss on it, then a fillet of the concave edge at the boss's foot (a torus strip)
/// and a hole: the round's material is cut away by the revolve of the corner section, and both steps rebuild on the
/// starting body to the file's solid exactly.
#[test]
fn imported_boss_then_foot_round() {
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
{{"type":"plane","id":"F3","mode":"offset","base":{TOP},"offset":10}},
{},{{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":8}},
{{"type":"fillet","id":"F6","radius":2,"edges":[{{"faces":["F2.C1","F5.S.e1"],"at":[-2,0,10]}}]}},
{},{{"type":"extrude","id":"F8","op":"cut","sketch":"F7","end":"through","flip":true}}]"#,
        rect("F1", TOP, -30.0, -20.0, 30.0, 20.0),
        circle("F4", r#"{"kind":"feature","id":"F3"}"#, -10.0, 0.0, 8.0),
        circle("F7", TOP, 15.0, 0.0, 3.0),
    );
    let o = run(&src, &[(2, "Imported", "Imported1"), (5, "Combine", "Combine1"), (6, "Fillet", "Fillet6")]);
    assert!(o.exact(), "{:?}\n{:#?}", o.fallback, steps(&o));
    let names: Vec<&str> = o.steps.iter().filter(|s| s.status == Status::Rebuilt).map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Imported1", "Fillet6", "Cut-Extrude8"], "{:#?}", steps(&o));
    let tools: Vec<&str> = o.features.iter().filter(|f| f.get_str("name") == Some("Undo Fillet6")).filter_map(|f| f.get_str("type")).collect();
    assert_eq!(tools, ["revolve"]);
}

/// An imported block on one side of the right plane, a hole in it, then a mirror of the whole body in that plane:
/// the mirror's material is undone by cutting away the copy's side, so the hole and the mirror both rebuild on the
/// starting body (the hole once, its copy by the mirror) to the file's solid exactly.
#[test]
fn imported_block_then_hole_and_body_mirror() {
    let src = format!(
        r#"[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
{},{{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"through","flip":true}},
{{"type":"mirror","id":"F5","seeds":[],"plane":{{"kind":"std","id":"right"}},"body":true}}]"#,
        rect("F1", TOP, 0.0, -20.0, 30.0, 20.0),
        circle("F3", TOP, 15.0, 5.0, 3.0),
    );
    let o = run(&src, &[(2, "Imported", "Imported1"), (5, "Mirror", "Mirror5")]);
    assert!(o.exact(), "{:?}\n{:#?}", o.fallback, steps(&o));
    let names: Vec<&str> = o.steps.iter().filter(|s| s.status == Status::Rebuilt).map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Imported1", "Cut-Extrude4", "Mirror5"], "{:#?}", steps(&o));
    assert!(o.features.iter().any(|f| f.get_str("type") == Some("mirror") && f.get("body") == Some(&Json::Bool(true))));
    let undo: Vec<&str> = o.features.iter().filter_map(|f| f.get_str("name")).filter(|n| n.starts_with("Undo")).collect();
    assert_eq!(undo, ["Undo Mirror5", "Undo Cut-Extrude4"]);
}
