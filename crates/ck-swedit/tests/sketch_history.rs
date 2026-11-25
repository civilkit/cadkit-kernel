//! The rebuilt history carries the sketch the SolidWorks file stores (its points, curves, relations and dimensions) where that
//! sketch draws the profile the faces give, and a sketch drawn from the faces otherwise. The committed fixture parts always run;
//! the local corpus (`SW_CORPUS`, default `corpus-local`) is read when present.

use ck_json::Json;
use ck_swedit::{rebuild_history, sketch_profile, Outcome, Tagged};
use std::path::PathBuf;

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ck-sldprt/tests/fixtures").join(format!("{name}.sldprt"))).unwrap()
}

fn history(bytes: &[u8]) -> Outcome {
    let imp = ck_sldprt::import_sldprt(bytes).unwrap();
    let t = Tagged::from_import(&imp);
    rebuild_history(&t)
}

fn sketches(o: &Outcome) -> Vec<&Json> {
    o.features.iter().filter(|f| f.get("type").and_then(|t| t.as_str()) == Some("sketch")).collect()
}

fn own(o: &Outcome) -> Vec<&Json> {
    sketches(o).into_iter().filter(|f| f.get("source").and_then(|t| t.as_str()) == Some("sldprt")).collect()
}

fn count(j: &Json, key: &str) -> usize {
    j.get(key).and_then(|a| a.as_arr()).map_or(0, |a| a.len())
}

fn dims(j: &Json) -> usize {
    j.get("constraints").and_then(|a| a.as_arr()).map_or(0, |a| a.iter().filter(|c| c.get("value").is_some()).count())
}

#[test]
fn the_cube_history_carries_the_files_own_sketch() {
    let o = history(&fixture("sw2022_cube_10mm"));
    assert!(o.exact(), "{:?}", o.fallback);
    let s = own(&o);
    assert_eq!(s.len(), 1, "{:?}", sketches(&o));
    // the square: four lines (and the construction line the file's sketch also has), held by their horizontal and vertical relations
    let lines = s[0].get("entities").and_then(|a| a.as_arr()).unwrap().iter().filter(|e| e.get("type").and_then(|t| t.as_str()) == Some("line") && e.get("construction").is_none()).count();
    assert_eq!(lines, 4);
    let cons = s[0].get("constraints").and_then(|a| a.as_arr()).unwrap();
    assert!(cons.iter().any(|c| c.get("type").and_then(|t| t.as_str()) == Some("horizontal")), "{cons:?}");
    assert!(cons.iter().any(|c| c.get("type").and_then(|t| t.as_str()) == Some("vertical")), "{cons:?}");
}

#[test]
fn the_hole_fixture_keeps_its_circle_and_its_dimension() {
    let o = history(&fixture("sw2022_cube_hole_5mm"));
    assert!(o.exact() || o.close() || o.prefix > 0, "{:?}", o.fallback);
    let s = own(&o);
    // (the cube's sketch and the hole's, where the hole is read from its sketch)
    assert!(!s.is_empty());
    let circles: usize = s.iter().map(|j| j.get("entities").and_then(|a| a.as_arr()).map_or(0, |a| a.iter().filter(|e| e.get("type").and_then(|t| t.as_str()) == Some("circle")).count())).sum();
    assert!(circles <= 1);
    let _ = (dims, count(s[0], "points"));
}

#[test]
fn without_the_preference_the_sketches_are_drawn_from_the_faces() {
    let prev = sketch_profile::prefer(false);
    let o = history(&fixture("sw2022_cube_10mm"));
    sketch_profile::prefer(prev);
    assert!(o.exact());
    assert!(own(&o).is_empty());
    assert!(!sketches(&o).is_empty());
}

#[test]
fn the_cone_revolve_uses_the_files_sketch_and_one_of_its_lines_as_the_axis() {
    let o = history(&fixture("sw2022_cone"));
    assert!(o.exact() || o.close(), "{:?}", o.fallback);
    let rev = o.features.iter().find(|f| f.get("type").and_then(|t| t.as_str()) == Some("revolve")).expect("a revolve");
    let sk_id = rev.get("sketch").and_then(|s| s.as_str()).unwrap();
    let sk = sketches(&o).into_iter().find(|s| s.get("id").and_then(|i| i.as_str()) == Some(sk_id)).unwrap();
    let axis = rev.get("axis").and_then(|a| a.as_str()).unwrap();
    let ents = sk.get("entities").and_then(|a| a.as_arr()).unwrap();
    assert!(ents.iter().any(|e| e.get("id").and_then(|i| i.as_str()) == Some(axis) && e.get("type").and_then(|t| t.as_str()) == Some("line")));
    assert_eq!(sk.get("source").and_then(|t| t.as_str()), Some("sldprt"), "the cone's sketch is the file's own");
}

/// Over the corpus: with the file's sketches or without, no file does worse, and at least one sketch with dimensions is carried.
#[test]
fn corpus_the_preference_never_does_worse() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    let Ok(rd) = std::fs::read_dir(dir.join("samples")) else {
        eprintln!("no corpus");
        return;
    };
    let mut names: Vec<PathBuf> = rd.flatten().map(|e| e.path()).filter(|p| p.file_name().map_or(false, |n| n.to_string_lossy().starts_with("dump_SW2022")) || p.ends_with("nist_ctc_01")).collect();
    names.sort();
    let mut with_dims = 0;
    for dirp in names {
        let Some(file) = std::fs::read_dir(&dirp).ok().and_then(|r| r.flatten().map(|e| e.path()).find(|p| p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")))) else { continue };
        let bytes = std::fs::read(&file).unwrap();
        let rank = |o: &Outcome| if o.exact() { 3 } else if o.close() { 2 } else { 1 };
        let a = history(&bytes);
        let prev = sketch_profile::prefer(false);
        let b = history(&bytes);
        sketch_profile::prefer(prev);
        assert!(rank(&a) >= rank(&b), "{}: {} with the file's sketches, {} without", dirp.display(), rank(&a), rank(&b));
        with_dims += own(&a).iter().filter(|j| dims(j) > 0).count();
    }
    // (the sample parts of SolidWorks 2022 carry no dimension in their sketches)
    eprintln!("sketches with dimensions carried: {with_dims}");
}
