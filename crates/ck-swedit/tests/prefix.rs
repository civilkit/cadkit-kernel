//! Keeping the rebuilt prefix (WP4): a history that does not rebuild to the end keeps its leading steps that pass
//! their local checks and ends with the imported remainder (`importBody`, `replace`). Parts are made in-test and
//! tagged as a SolidWorks file tags them; the end-to-end case reads a committed `ck-sldprt` fixture.

use ck_json::Json;
use ck_recog::rebuild;
use ck_sldprt::meta::{SwDim, SwFeature};
use ck_swedit::prefix::{check_steps, regen_part, snapshots, Input};
use ck_swedit::{rebuild_history, Outcome, Status, Tagged};
use ck_topo::body::Body;

fn feats(src: &str) -> Vec<Json> {
    match ck_json::parse(src).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    }
}

fn sw_features(fs: &[Json]) -> Vec<SwFeature> {
    let mut out = vec![];
    for f in fs {
        let id: i64 = f.get_str("id").unwrap()[1..].parse().unwrap();
        let ty = f.get_str("type").unwrap();
        let cut = f.get_str("op") == Some("cut");
        let d1 = |k: &str| f.get_f64(k).map(|d| vec![SwDim { name: "D1".into(), text: d.to_string(), value: Some(d) }]).unwrap_or_default();
        let (kind, name, dims) = match ty {
            "sketch" => ("Sketch", format!("Sketch{id}"), vec![]),
            "extrude" if cut => ("Extrusion", format!("Cut-Extrude{id}"), d1("depth")),
            "extrude" => ("Boss-Extrude", format!("Boss-Extrude{id}"), d1("depth")),
            "fillet" => ("Fillet", format!("Fillet{id}"), d1("radius")),
            t => panic!("{t}"),
        };
        out.push(SwFeature { id, name, kind: kind.into(), dims });
    }
    out
}

/// The part made by `src`, its faces tagged with the feature that made them.
fn tagged(src: &str) -> (Body, Vec<Vec<(i64, i64)>>, Vec<SwFeature>) {
    let fs = feats(src);
    let (body, errs) = rebuild::rebuild(&fs).unwrap();
    assert!(errs.is_empty(), "original regen errors {errs:?}");
    let body = body.expect("body");
    let mut tags = vec![vec![]; body.faces.len()];
    for fid in body.face_ids() {
        if let Ok(t) = body.face(fid).name.feature[1..].parse::<i64>() {
            tags[fid.idx()] = vec![(t, 0)];
        }
    }
    (body, tags, sw_features(&fs))
}

fn steps(o: &Outcome) -> Vec<String> {
    o.steps.iter().map(|s| format!("{} {:?} {}", s.name, s.status, s.note)).collect()
}

fn types(fs: &[Json]) -> Vec<String> {
    fs.iter().map(|f| f.get_str("type").unwrap().to_string()).collect()
}

const BOX: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// Box, round boss on its top, rectangular pocket into the boss, through slot in the base.
fn four_steps() -> String {
    format!(
        r#"[{BOX},
        {{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"c","x":5,"y":3}}],"entities":[{{"id":"e1","type":"circle","c":"c","r":12}}],"constraints":[]}},
        {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":8}},
        {{"type":"sketch","id":"F5","plane":{{"kind":"face","id":"F4.C1"}},"points":[{{"id":"p1","x":2,"y":0}},{{"id":"p2","x":8,"y":0}},{{"id":"p3","x":8,"y":4}},{{"id":"p4","x":2,"y":4}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F6","op":"cut","sketch":"F5","end":"blind","depth":5}},
        {{"type":"sketch","id":"F7","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":-25,"y":-15}},{{"id":"p2","x":-15,"y":-15}},{{"id":"p3","x":-15,"y":15}},{{"id":"p4","x":-25,"y":15}}],"entities":[{{"id":"e1","type":"line","p1":"p1","p2":"p2"}},{{"id":"e2","type":"line","p1":"p2","p2":"p3"}},{{"id":"e3","type":"line","p1":"p3","p2":"p4"}},{{"id":"e4","type":"line","p1":"p4","p2":"p1"}}],"constraints":[]}},
        {{"type":"extrude","id":"F8","op":"cut","sketch":"F7","end":"through","flip":true}}]"#
    )
}

#[test]
fn unsupported_step_three_keeps_two_steps() {
    let (body, tags, mut sw) = tagged(&four_steps());
    // the pocket (step 3) is a feature type we do not read back
    let pocket = sw.iter_mut().find(|f| f.id == 6).unwrap();
    pocket.kind = "Sweep".into();
    pocket.name = "Sweep6".into();
    let o = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 });
    assert_eq!(o.prefix, 2, "steps {:#?}\nchecks {:?}", steps(&o), o.checks);
    assert!(!o.exact() && o.editable());
    assert_eq!(o.steps[2].status, Status::Unsupported);
    assert_eq!(o.checks.len(), 2);
    assert!(o.checks.iter().all(|c| c.ok()), "{:?}", o.checks);
    assert!(o.checks[0].faces.0 > 0, "the base's untouched faces are checked");
    let r = o.remainder.as_ref().unwrap();
    assert_eq!(r.from.1, "Sweep6");
    assert!(r.reason.contains("Sweep"), "{}", r.reason);
    // the remainder stands for the sweep and every step after it
    assert_eq!(r.steps.iter().map(|s| s.1.as_str()).collect::<Vec<_>>(), ["Sweep6", "Cut-Extrude8"]);
    let t = types(&o.features);
    assert_eq!(t.last().unwrap(), "importBody");
    assert_eq!(t.iter().filter(|x| *x == "extrude").count(), 2, "{t:?}");
    let last = o.features.last().unwrap();
    assert_eq!(last.get("replace"), Some(&Json::Bool(true)));
    assert_eq!(last.get_str("format"), Some("sldprt"));
    assert_eq!(o.ranges.len(), 2);
    assert_eq!(o.ranges[1].1, o.features.len() - 1);
    // the prefix alone regenerates without errors (the boss on the box)
    let (b, errs) = rebuild::rebuild(&o.features[..o.features.len() - 1]).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let v = ck_topo::mass::mass_props(&b.unwrap()).volume;
    let want = 60.0 * 40.0 * 10.0 + std::f64::consts::PI * 144.0 * 8.0;
    assert!((v - want).abs() < 1e-6 * want, "{v} want {want}");
    // the outcome's JSON carries the prefix, the remainder and the checks
    let j = o.json();
    assert_eq!(j.get_f64("prefix"), Some(2.0));
    assert_eq!(j.get("remainder").and_then(|r| r.get_str("mode")), Some("replace"));
}

#[test]
fn nothing_to_keep_stays_the_plain_solid() {
    let (body, tags, mut sw) = tagged(&four_steps());
    let base = sw.iter_mut().find(|f| f.id == 2).unwrap();
    base.kind = "Sweep".into();
    let o = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 });
    assert_eq!(o.prefix, 0);
    assert!(o.remainder.is_none() && !o.editable() && o.body.is_none());
    assert!(o.fallback.as_deref().unwrap().contains("Sweep"));
    assert!(!types(&o.features).contains(&"importBody".to_string()));
}

#[test]
fn a_complete_rebuild_has_no_remainder() {
    let (body, tags, sw) = tagged(&four_steps());
    let o = rebuild_history(&Tagged { body: &body, tags, features: &sw, unit_mm: 1.0 });
    assert!(o.exact(), "{:?}", o.fallback);
    assert_eq!(o.prefix, 0);
    assert!(o.remainder.is_none() && o.checks.is_empty());
    assert_eq!(o.ranges.len(), 4);
}

/// The read-back history of the four steps, its step ranges and the checks of `features` (an edited copy).
fn checks_of(edit: impl Fn(&mut Vec<Json>, &[(usize, usize)])) -> Vec<ck_swedit::prefix::Check> {
    let (body, tags, sw) = tagged(&four_steps());
    let o = rebuild_history(&Tagged { body: &body, tags: tags.clone(), features: &sw, unit_mm: 1.0 });
    assert!(o.exact());
    let mut fs = o.features.clone();
    edit(&mut fs, &o.ranges);
    let (snaps, errs) = snapshots(&fs, &o.ranges).unwrap();
    check_steps(&Input { body: &body, tags: &tags, features: &sw, steps: &o.steps, snaps: &snaps, errs: &errs })
}

fn set(f: &mut Json, key: &str, v: Json) {
    if let Json::Obj(kv) = f {
        kv.iter_mut().find(|(k, _)| k == key).unwrap().1 = v;
    }
}

#[test]
fn the_unedited_history_passes_every_check() {
    let c = checks_of(|_, _| {});
    assert_eq!(c.len(), 4);
    assert!(c.iter().all(|c| c.ok()), "{c:?}");
}

#[test]
fn a_wrong_depth_is_localised_to_its_step() {
    // the boss (step 2) read back 2 mm short: its top cap is not where the final body has it
    let c = checks_of(|fs, r| {
        let boss = fs[r[1].0..r[1].1].iter().position(|f| f.get_str("type") == Some("extrude")).unwrap() + r[1].0;
        set(&mut fs[boss], "depth", Json::Num(6.0));
    });
    assert_eq!(c.len(), 2, "{c:?}");
    assert!(c[0].ok());
    assert!(c[1].regen.is_none());
    let s = c[1].settled.as_deref().expect("the settled-face check fails");
    assert!(c[1].failure().unwrap().starts_with("settled faces"), "{s}");
}

#[test]
fn a_regen_error_is_localised_to_its_step() {
    // the pocket (step 3) names a sketch that does not exist: a regen error on that step only
    let c = checks_of(|fs, r| {
        let cut = fs[r[2].0..r[2].1].iter().position(|f| f.get_str("type") == Some("extrude")).unwrap() + r[2].0;
        set(&mut fs[cut], "sketch", Json::Str("F99".into()));
    });
    assert_eq!(c.len(), 3, "{c:?}");
    assert!(c[0].ok() && c[1].ok());
    assert!(c[2].regen.as_deref().unwrap().contains("missingSketch"), "{:?}", c[2]);
}

#[test]
fn a_cut_that_adds_material_fails_its_step() {
    // the through slot (step 4) made a boss: the step grows the body, and its walls are not the final ones
    let c = checks_of(|fs, r| {
        let cut = fs[r[3].0..r[3].1].iter().position(|f| f.get_str("type") == Some("extrude")).unwrap() + r[3].0;
        set(&mut fs[cut], "op", Json::Str("add".into()));
        set(&mut fs[cut], "end", Json::Str("blind".into()));
        if let Json::Obj(kv) = &mut fs[cut] {
            kv.push(("depth".into(), Json::Num(5.0)));
        }
    });
    assert_eq!(c.len(), 4, "{c:?}");
    assert!(c[..3].iter().all(|c| c.ok()));
    assert!(!c[3].ok());
}

/// End to end on a committed fixture: a cube with a hole whose last feature is made unsupported keeps the cube's extrude, and
/// the part document (with the file in its `blobs`) regenerates to exactly the plain import.
#[test]
fn fixture_prefix_regenerates_to_the_import() {
    let file = std::fs::read(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ck-sldprt/tests/fixtures/sw2022_cube_hole_5mm.sldprt")).unwrap();
    let mut imp = ck_sldprt::import_sldprt(&file).unwrap();
    let full = rebuild_history(&Tagged::from_import(&imp));
    assert!(full.exact(), "{:?} {:#?}", full.fallback, steps(&full));
    // the last feature that made faces (the hole) becomes a type we do not read back
    let made: Vec<i64> = imp.face_tags.iter().flat_map(|t| t.sw_ids.iter().map(|i| i.feature)).collect();
    let last = imp.meta.features.iter_mut().rev().find(|f| ck_swedit::classify(f) != ck_swedit::Class::Skip && made.contains(&f.id)).unwrap();
    last.kind = "Sweep".into();
    let o = rebuild_history(&Tagged::from_import(&imp));
    // every step before the unsupported one (a leading no-geometry step included), the extrude among them
    let u = o.steps.iter().position(|s| s.status == Status::Unsupported).unwrap();
    assert_eq!(o.prefix, u, "{:#?} {:?}", steps(&o), o.checks);
    assert_eq!(o.steps[..u].iter().filter(|s| s.status == Status::Rebuilt).count(), 1);
    assert_eq!(o.remainder.as_ref().unwrap().from.0, o.steps[u].sw_id);
    let part = o.part_json(Some(&file));
    let (b, errs) = regen_part(&part).unwrap();
    assert!(errs.is_empty(), "{errs:?}");
    let b = b.unwrap();
    let p = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "import", None).solid;
    let v = ck_swedit::fastverify::verify(&p, &b);
    assert!(v.exact, "rel {} area {}", v.rel, v.area_rel);
    assert_eq!(b.num_faces(), p.num_faces());
    // without the file the remainder names no blob, and the document says so on regeneration
    let (_, errs) = regen_part(&o.part_json(None)).unwrap();
    assert!(errs.iter().any(|e| e.contains("importBodyInvalid")), "{errs:?}");
}
