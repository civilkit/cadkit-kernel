//! `unknownField`: a misspelled feature field is reported as a warning (never an error), and no part of
//! the existing corpus gets one. The warning must not change anything else in the reply.
use ck_json::Json;
use ck_model::regen::{self, NoGeometry, RegenOpts};
use ck_model::Part;
use std::path::{Path, PathBuf};

fn reply(src: &str) -> Json {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    regen::regen(&part, &RegenOpts::default(), &mut NoGeometry).to_json()
}

fn warnings(r: &Json) -> Vec<(String, Json)> {
    r.get("warnings")
        .and_then(|w| w.as_arr())
        .map(|w| w.iter().map(|e| (e.as_arr().unwrap()[0].as_str().unwrap().to_string(), e.as_arr().unwrap()[1].clone())).collect())
        .unwrap_or_default()
}

fn part(features: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)
}

const SKETCH: &str = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[],"entities":[],"constraints":[]}"#;

#[test]
fn a_typo_gives_the_warning_with_a_suggestion() {
    let r = reply(&part(&format!(r#"{SKETCH},{{"type":"extrude","id":"F2","sketch":"S1","op":"add","end":"blind","depht":10}}"#)));
    let w = warnings(&r);
    assert_eq!(w.len(), 1, "{w:?}");
    let (id, e) = &w[0];
    assert_eq!(id, "F2");
    assert_eq!(e.get("code").and_then(|c| c.as_str()), Some("unknownField"));
    let p = e.get("params").unwrap();
    assert_eq!(p.get("fields").unwrap().as_arr().unwrap()[0].as_str(), Some("depht"));
    assert_eq!(p.get("suggest").unwrap().get("depht").and_then(|s| s.as_str()), Some("depth"));
}

#[test]
fn every_checked_type_warns_on_a_typo() {
    for (ty, bad) in [
        ("sketch", "pointz"),
        ("extrude", "dpeth"),
        ("revolve", "angel"),
        ("fillet", "raduis"),
        ("chamfer", "distnace"),
        ("hole", "diamter"),
    ] {
        let f = format!(r#"{{"type":"{ty}","id":"F9","{bad}":1}}"#);
        let w = warnings(&reply(&part(&f)));
        assert_eq!(w.len(), 1, "{ty}: {w:?}");
        assert_eq!(w[0].1.get("params").unwrap().get("fields").unwrap().as_arr().unwrap()[0].as_str(), Some(bad));
    }
}

#[test]
fn a_field_with_no_close_match_has_no_suggestion_and_several_are_listed_in_order() {
    let r = reply(&part(r#"{"type":"fillet","id":"F9","radius":1,"edges":[],"zzz":1,"colour":2}"#));
    let p = warnings(&r)[0].1.get("params").unwrap().clone();
    let fields: Vec<&str> = p.get("fields").unwrap().as_arr().unwrap().iter().map(|f| f.as_str().unwrap()).collect();
    assert_eq!(fields, ["zzz", "colour"]);
    assert!(p.get("suggest").is_none(), "{p:?}");
}

#[test]
fn unchecked_types_and_bookkeeping_fields_do_not_warn() {
    let r = reply(&part(
        r#"{"type":"extrude","id":"F2","num":3,"name":"Boss","suppressed":true,"group":"g","autoName":true,"visible":true,"userHidden":false,"tolerance":0,"autoFuzzy":true},
        {"type":"shell","id":"F3","anything":1,"faces":[]}"#,
    ));
    assert!(warnings(&r).is_empty(), "{:?}", warnings(&r));
    assert!(r.get("warnings").is_none());
}

#[test]
fn the_warning_changes_nothing_else_in_the_reply() {
    let typo = format!(r#"{SKETCH},{{"type":"extrude","id":"F2","sketch":"S1","depht":10}}"#);
    let plain = format!(r#"{SKETCH},{{"type":"extrude","id":"F2","sketch":"S1"}}"#);
    let (mut a, b) = (reply(&part(&typo)), reply(&part(&plain)));
    assert_eq!(warnings(&a).len(), 1);
    if let Json::Obj(m) = &mut a {
        m.retain(|(k, _)| k != "warnings");
    }
    assert_eq!(ck_json::to_string(&a), ck_json::to_string(&b));
}

#[test]
fn origin_is_bookkeeping_and_a_misspelling_of_a_real_field_still_warns() {
    let ok = reply(&part(&format!(
        r#"{},{{"type":"extrude","id":"F2","sketch":"S1","op":"add","end":"blind","depth":10,"origin":"script#1"}}"#,
        SKETCH.replace(r#""id":"S1""#, r#""id":"S1","origin":"script#1""#)
    )));
    assert!(warnings(&ok).is_empty(), "{:?}", warnings(&ok));
    let bad = reply(&part(&format!(r#"{SKETCH},{{"type":"extrude","id":"F2","sketch":"S1","origin":"script#1","depht":10}}"#)));
    let w = warnings(&bad);
    assert_eq!(w.len(), 1, "{w:?}");
    assert_eq!(w[0].1.get("params").unwrap().get("fields").unwrap().as_arr().unwrap().len(), 1);
}

#[test]
fn a_suppressed_feature_is_not_checked() {
    let r = reply(&part(r#"{"type":"extrude","id":"F2","suppressed":true,"depht":1}"#));
    assert!(warnings(&r).is_empty());
}

// ---- the existing corpus gives none ----

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    let mut ents: Vec<_> = rd.flatten().map(|e| e.path()).collect();
    ents.sort();
    for p in ents {
        if p.is_dir() {
            files(&p, out);
        } else if p.extension().map_or(false, |e| e == "json" || e == "ckpart") {
            out.push(p);
        }
    }
}

/// Every object in `j` that is a part document.
fn parts<'a>(j: &'a Json, out: &mut Vec<&'a Json>) {
    match j {
        Json::Obj(m) => {
            if j.get("format").and_then(|f| f.as_str()).is_some() && j.get("features").and_then(|f| f.as_arr()).is_some() {
                out.push(j);
            }
            for (_, v) in m {
                parts(v, out);
            }
        }
        Json::Arr(a) => a.iter().for_each(|v| parts(v, out)),
        _ => {}
    }
}

#[test]
fn no_part_of_the_corpus_gets_the_warning() {
    let here = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut fs = vec![];
    for d in ["tests/fixtures", "tests/golden", "../ck-io/tests/golden", "../../tools/cases", "../../tools/bench", "../../tools/fixtures", "../../tests-golden"] {
        files(&here.join(d), &mut fs);
    }
    let (mut n, mut bad) = (0, vec![]);
    for p in &fs {
        let Ok(src) = std::fs::read_to_string(p) else { continue };
        let Ok(j) = ck_json::parse(&src) else { continue };
        let mut ps = vec![];
        parts(&j, &mut ps);
        for doc in ps {
            let Ok(part) = Part::from_json(doc) else { continue };
            n += 1;
            let r = regen::regen(&part, &RegenOpts::default(), &mut NoGeometry).to_json();
            for (id, w) in warnings(&r) {
                if w.get("code").and_then(|c| c.as_str()) == Some("unknownField") {
                    bad.push(format!("{} {id}: {}", p.strip_prefix(here).unwrap_or(p).display(), ck_json::to_string(&w)));
                }
            }
        }
    }
    assert!(n > 50, "the corpus walk found only {n} parts");
    assert!(bad.is_empty(), "{} of {n} parts warn:\n{}", bad.len(), bad.iter().take(20).cloned().collect::<Vec<_>>().join("\n"));
}

#[test]
fn suggestions_are_for_close_names_only() {
    for (ty, bad, want) in [("extrude", "depht", Some("depth")), ("extrude", "Depth", Some("depth")), ("hole", "diamter", Some("diameter")), ("fillet", "raduis", Some("radius")), ("extrude", "dp", None), ("extrude", "qqqqq", None)] {
        let f = format!(r#"{{"type":"{ty}","id":"F9","{bad}":1}}"#);
        let w = warnings(&reply(&part(&f)));
        let got = w[0].1.get("params").unwrap().get("suggest").and_then(|s| s.get(bad)).and_then(|s| s.as_str().map(str::to_string));
        assert_eq!(got.as_deref(), want, "{bad}");
    }
}
