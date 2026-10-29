//! `importStep`: a STEP body held by the part (blob table or inline), read on regeneration.
//! The STEP texts are made by the kernel's own writer from small synthetic parts.
use ck_json::Json;
use ck_model::features::import_step::blob_key;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

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
    errors: Vec<String>,
}

fn run_doc(doc: &str) -> Out {
    let part = Part::parse(doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { body: ex.0.body.clone(), errors: r.errors.iter().map(|e| format!("{}:{} {}", e.feature, e.error.code, ck_json::to_string(&Json::Obj(e.error.params.clone())))).collect() }
}

fn run(features: &str) -> Out {
    run_doc(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#))
}

/// A part document whose `blobs` table holds `step`, with `features` after it.
fn doc_with_blob(step: &str, tail: &str) -> String {
    let key = blob_key(step);
    let step_json = ck_json::to_string(&Json::Str(step.to_string()));
    let sep = if tail.is_empty() { "" } else { "," };
    format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":{step_json}}},"features":[{{"type":"importStep","id":"F1","blob":"{key}"}}{sep}{tail}]}}"#)
}

fn volume(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn rect(sk: &str, x0: f64, x1: f64, hole: bool) -> String {
    let h = if hole {
        format!(r#",{{"id":"c","x":{},"y":0}}"#, (x0 + x1) / 2.0)
    } else {
        String::new()
    };
    let he = if hole { r#",{"type":"circle","id":"e5","c":"c","r":2}"# } else { "" };
    format!(
        r#"{{"type":"sketch","id":"{sk}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":{x0},"y":-5}},{{"id":"p2","x":{x1},"y":-5}},{{"id":"p3","x":{x1},"y":5}},{{"id":"p4","x":{x0},"y":5}}{h}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}{he}],"constraints":[]}}"#
    )
}

fn extrude(id: &str, sk: &str) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"add","sketch":"{sk}","end":"blind","depth":5}}"#)
}

fn step_text(b: &Body) -> String {
    ck_io::step::write_step(b, "synthetic", &ck_io::step::StepOpts { unix_ms: 0.0 })
}

/// A 20 x 10 x 5 block with a 4 mm hole through it (volume 1000 - 20 pi).
fn block_with_hole() -> (String, f64) {
    let o = run(&format!("{},{}", rect("S1", -10.0, 10.0, true), extrude("E1", "S1")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    let v = volume(&b);
    assert!((v - (1000.0 - 20.0 * std::f64::consts::PI)).abs() < 1e-3, "{v}");
    (step_text(&b), v)
}

fn face_ids(b: &Body) -> Vec<String> {
    let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    v.sort();
    v
}

#[test]
fn imports_the_solid_with_its_volume_and_named_faces() {
    let (step, want) = block_with_hole();
    let o = run_doc(&doc_with_blob(&step, ""));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.expect("a body");
    assert!((volume(&b) - want).abs() < 1e-6 * want, "{} vs {want}", volume(&b));
    assert_eq!(b.num_lumps(), 1);
    let ids = face_ids(&b);
    assert_eq!(ids.len(), 7, "{ids:?}");
    assert!(ids.iter().all(|i| i.starts_with("F1.I1F")), "{ids:?}");
    let mut dedup = ids.clone();
    dedup.dedup();
    assert_eq!(dedup.len(), ids.len());
}

#[test]
fn inline_step_text_works_like_a_blob() {
    let (step, want) = block_with_hole();
    let feat = format!(r#"{{"type":"importStep","id":"F1","step":{}}}"#, ck_json::to_string(&Json::Str(step)));
    let o = run(&feat);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!((volume(&o.body.unwrap()) - want).abs() < 1e-6 * want);
}

#[test]
fn file_units_are_applied() {
    let (step, want) = block_with_hole();
    let cm = step.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)");
    assert_ne!(cm, step, "the writer's length unit line changed");
    let o = run_doc(&doc_with_blob(&cm, ""));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let v = volume(&o.body.unwrap());
    assert!((v - want * 1000.0).abs() < 1e-6 * want * 1000.0, "{v}");
}

#[test]
fn two_solids_are_two_named_bodies() {
    let o = run(&format!("{},{},{},{}", rect("S1", 0.0, 10.0, false), extrude("E1", "S1"), rect("S2", 30.0, 40.0, false), extrude("E2", "S2")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let two = o.body.unwrap();
    assert_eq!(two.num_lumps(), 2);
    let step = step_text(&two);
    let key = blob_key(&step);
    let doc = format!(
        r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":{}}},"features":[{{"type":"importStep","id":"F1","blob":"{key}","body":"Pair"}}]}}"#,
        ck_json::to_string(&Json::Str(step))
    );
    let o = run_doc(&doc);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    assert_eq!(b.num_lumps(), 2);
    assert!((volume(&b) - 1000.0).abs() < 1e-6);
    let mut names: Vec<String> = b.lump_ids().filter_map(|l| b.lump(l).name.clone()).collect();
    names.sort();
    assert_eq!(names, vec!["Pair".to_string(), "Pair 2".to_string()]);
    let ids = face_ids(&b);
    assert!(ids.iter().any(|i| i.starts_with("F1.I1F")) && ids.iter().any(|i| i.starts_with("F1.I2F")), "{ids:?}");
}

#[test]
fn bad_data_is_a_structured_error() {
    let (step, _) = block_with_hole();
    let detail = |o: &Out| {
        assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
        assert!(o.errors[0].starts_with("F1:importStepInvalid "), "{:?}", o.errors);
        assert!(o.errors[0].contains("\"detail\""), "{:?}", o.errors);
        assert!(o.body.is_none());
    };
    // no data at all
    detail(&run(r#"{"type":"importStep","id":"F1"}"#));
    // not STEP
    detail(&run(r#"{"type":"importStep","id":"F1","step":"hello"}"#));
    detail(&run_doc(&doc_with_blob("hello", "")));
    // a key that names nothing, and a key that does not belong to the text
    detail(&run(r#"{"type":"importStep","id":"F1","blob":"00"}"#));
    let key = blob_key(&step);
    let wrong = format!(r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":{}}},"features":[{{"type":"importStep","id":"F1","blob":"{key}"}}]}}"#, ck_json::to_string(&Json::Str(step.replace("1.E-06", "2.E-06"))));
    detail(&run_doc(&wrong));
    // a header and nothing else: no solid
    detail(&run(r#"{"type":"importStep","id":"F1","step":"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"}"#));
    // cut at many places: an error or a body, never a panic
    let n = step.len();
    for k in (0..n).step_by((n / 40).max(1)) {
        let mut cut = k;
        while !step.is_char_boundary(cut) {
            cut -= 1;
        }
        let o = run_doc(&doc_with_blob(&step[..cut], ""));
        assert!(o.body.is_some() || o.errors.iter().any(|e| e.contains("importStepInvalid")), "cut {cut}: {:?}", o.errors);
    }
}

#[test]
fn a_later_feature_can_name_an_imported_face() {
    let (step, want) = block_with_hole();
    let first = run_doc(&doc_with_blob(&step, ""));
    let b = first.body.unwrap();
    let top = top_face(&b);
    assert!(top.starts_with("F1.I1F"), "{top}");
    // a small circular pocket sketched on that face: the same feature against the natively built block names it differently
    let pocket = |face: &str| {
        format!(
            r#"{{"type":"sketch","id":"S9","plane":{{"kind":"face","id":"{face}"}},"points":[{{"id":"q","x":6,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"q","r":1}}],"constraints":[]}},{{"type":"extrude","id":"E9","op":"cut","sketch":"S9","end":"blind","depth":2}}"#
        )
    };
    let native = run(&format!("{},{},{}", rect("S1", -10.0, 10.0, true), extrude("E1", "S1"), "{}".replace("{}", &pocket(&native_top()))));
    assert!(native.errors.is_empty(), "{:?}", native.errors);
    let imported = run_doc(&doc_with_blob(&step, &pocket(&top)));
    assert!(imported.errors.is_empty(), "{:?}", imported.errors);
    let (vn, vi) = (volume(&native.body.unwrap()), volume(&imported.body.unwrap()));
    assert!(vi < want - 1.0, "the pocket removed nothing: {vi} vs {want}");
    assert!((vn - vi).abs() < 1e-6 * want, "native {vn} imported {vi}");
}

/// The id of the +z face of the natively built block.
fn native_top() -> String {
    let o = run(&format!("{},{}", rect("S1", -10.0, 10.0, true), extrude("E1", "S1")));
    top_face(&o.body.unwrap())
}

/// The planar face facing +z that lies highest (the blocks are extruded along z from the front plane).
fn top_face(b: &Body) -> String {
    let z_of = |f| match &b.face(f).surface {
        ck_geom::surface::Surface::Plane { frame } => frame.o.dot(b.planar_normal(f)),
        _ => f64::NEG_INFINITY,
    };
    let f = b
        .face_ids()
        .filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }) && b.planar_normal(f).z > 0.99)
        .max_by(|&x, &y| z_of(x).total_cmp(&z_of(y)))
        .expect("a planar face facing +z");
    b.face_id_str(f)
}

#[test]
fn names_and_geometry_survive_save_and_load() {
    let (step, want) = block_with_hole();
    let doc = doc_with_blob(&step, "");
    let part = Part::parse(&doc).unwrap();
    // save: the document back to text (the blobs table is a top-level field the part carries through), load: parse again
    let saved = ck_json::to_string(&part.to_json());
    assert!(saved.contains(&blob_key(&step)));
    let a = run_doc(&doc);
    let b = run_doc(&saved);
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    let (ba, bb) = (a.body.unwrap(), b.body.unwrap());
    assert_eq!(face_ids(&ba), face_ids(&bb));
    assert!((volume(&bb) - want).abs() < 1e-6 * want);
}

#[test]
fn a_part_with_an_import_first_keeps_the_features_that_follow() {
    // the import then a second import beside it: the second adds its solid and keeps the first
    let (step, want) = block_with_hole();
    let key = blob_key(&step);
    let doc = format!(
        r#"{{"format":"ck-part","version":1,"blobs":{{"{key}":{}}},"features":[{{"type":"importStep","id":"F1","blob":"{key}"}},{{"type":"importStep","id":"F2","blob":"{key}"}}]}}"#,
        ck_json::to_string(&Json::Str(step))
    );
    let o = run_doc(&doc);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    assert_eq!(b.num_lumps(), 2);
    assert!((volume(&b) - 2.0 * want).abs() < 1e-6 * want);
    let ids = face_ids(&b);
    assert!(ids.iter().any(|i| i.starts_with("F1.")) && ids.iter().any(|i| i.starts_with("F2.")));
}

/// The same text with the sense of its `nth` face turned: how a converter's restatement can come out wrong.
fn with_a_face_turned(step: &str, nth: usize) -> String {
    let mut at = 0;
    for _ in 0..=nth {
        at += step[at..].find("ADVANCED_FACE(").expect("a face") + 1;
    }
    let end = at + step[at..].find(",.T.)").expect("a face with sense T");
    format!("{}{}{}", &step[..end], ",.F.)", &step[end + ",.T.)".len()..])
}

#[test]
fn a_face_turned_the_wrong_way_reads_wrong_as_it_is_and_right_with_heal() {
    // a plain block: 20 x 10 x 5 = 1000
    let o = run(&format!("{},{}", rect("S1", -10.0, 10.0, false), extrude("E1", "S1")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.unwrap();
    let want = volume(&b);
    assert!((want - 1000.0).abs() < 1e-6, "{want}");
    let step = step_text(&b);
    // a face through the origin adds nothing to a volume taken from the origin, so turn the first face that does
    let mut seen = 0;
    for nth in 0..6 {
        let turned = with_a_face_turned(&step, nth);
        assert_ne!(turned, step);
        // as it is: a solid of the wrong volume (the reader keeps the sense the text states)
        let plain = run_doc(&doc_with_blob(&turned, ""));
        assert!(plain.errors.is_empty(), "{:?}", plain.errors);
        let plain_volume = volume(&plain.body.expect("a body"));
        // healed, as the importStep message does: the solid of the source, whichever face was turned
        let key = blob_key(&turned);
        let healed_doc = doc_with_blob(&turned, "").replace(&format!(r#""blob":"{key}""#), &format!(r#""blob":"{key}","heal":true"#));
        assert!(healed_doc.contains(r#""heal":true"#));
        let healed = run_doc(&healed_doc);
        assert!(healed.errors.is_empty(), "face {nth}: {:?}", healed.errors);
        let healed = healed.body.expect("a body");
        assert!((volume(&healed) - want).abs() < 1e-6 * want, "face {nth}: {} vs {want}", volume(&healed));
        assert_eq!(healed.num_lumps(), 1);
        assert_eq!(face_ids(&healed).len(), 6);
        if (plain_volume - want).abs() > 0.05 * want {
            seen += 1;
        }
    }
    assert!(seen >= 4, "turning a face should change the volume as it is read: {seen} of 6 did");
}

#[test]
fn heal_on_a_sound_text_changes_nothing_and_off_is_the_default() {
    let (step, want) = block_with_hole();
    let key = blob_key(&step);
    let with = |flag: &str| doc_with_blob(&step, "").replace(&format!(r#""blob":"{key}""#), &format!(r#""blob":"{key}"{flag}"#));
    for flag in [r#","heal":true"#, r#","heal":false"#, ""] {
        let o = run_doc(&with(flag));
        assert!(o.errors.is_empty(), "{flag}: {:?}", o.errors);
        assert!((volume(&o.body.unwrap()) - want).abs() < 1e-6 * want, "{flag}");
    }
}

#[test]
fn heal_must_be_true_or_false() {
    let (step, _) = block_with_hole();
    let key = blob_key(&step);
    let doc = doc_with_blob(&step, "").replace(&format!(r#""blob":"{key}""#), &format!(r#""blob":"{key}","heal":"yes""#));
    let o = run_doc(&doc);
    assert!(o.body.is_none());
    assert!(o.errors.iter().any(|e| e.contains("importStepInvalid") && e.contains("heal")), "{:?}", o.errors);
}
