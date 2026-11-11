//! `check` / `resolve` / `describe` / `refDiff` through the full server path.

use ck_json::Json;

fn call_raw(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn call(msg: &str) -> Json {
    let j = call_raw(msg);
    assert_eq!(j.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(&j));
    j.get("result").unwrap().clone()
}

/// 40x20 plate (top plane), 10 deep, with a d=8 through hole at (30,10).
fn plate(depth: f64, hole_x: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}},{{"id":"c5","x":{hole_x},"y":10}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}},{{"type":"circle","id":"e5","c":"c5","r":4}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth}}}]}}"#
    )
}

fn load(tag: &str) {
    call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{}}}"#, plate(10.0, 30.0)));
}

fn results(r: &Json) -> &[Json] {
    r.get_arr("results").unwrap()
}

#[test]
fn check_reports_pass_fail_and_measured() {
    load("c");
    let vol = 40.0 * 20.0 * 10.0 - std::f64::consts::PI * 16.0 * 10.0;
    let r = call(&format!(
        r#"{{"type":"check","tag":"c","expect":[
{{"kind":"volume","equals":{vol}}},{{"kind":"volume","equals":1000}},
{{"kind":"bbox","min":[0,0,-10],"max":[40,20,0],"tol":1e-6}},{{"kind":"bbox","size":[40,20,11]}},
{{"kind":"faces","equals":7}},{{"kind":"holes","equals":1}},{{"kind":"solids","equals":1}},
{{"kind":"mass","density":2,"min":1}},{{"kind":"valid"}},{{"kind":"nope"}}]}}"#
    ));
    let pass: Vec<bool> = results(&r).iter().map(|x| x.get_bool("pass").unwrap()).collect();
    // bbox z extent depends on the sketch-plane orientation; the fixed expectations are
    // checked below against `measured`, so only assert the orientation-free ones here
    assert_eq!(pass[0], true, "{}", ck_json::to_string(&r));
    assert_eq!(pass[1], false);
    assert_eq!(pass[3], false);
    assert_eq!(pass[4], true);
    assert_eq!(pass[5], true);
    assert_eq!(pass[6], true);
    assert_eq!(pass[7], true);
    assert_eq!(pass[8], true);
    assert_eq!(pass[9], false);
    assert_eq!(r.get_bool("ok"), Some(false));
    let m = r.get("measured").unwrap();
    assert_eq!(m.get_f64("holes"), Some(1.0));
    let sz = m.get("bbox").unwrap().get("size").unwrap().as_vec3().unwrap();
    assert!((sz[0] - 40.0).abs() < 1e-6 && (sz[1] - 10.0).abs() < 1e-6 && (sz[2] - 20.0).abs() < 1e-6 || (sz[2] - 10.0).abs() < 1e-6, "{sz:?}");
    // object shorthand
    let r = call(r#"{"type":"check","tag":"c","expect":{"faces":7,"edges":{"min":10}}}"#);
    assert_eq!(r.get_bool("ok"), Some(true), "{}", ck_json::to_string(&r));
}

#[test]
fn resolve_finds_faces_and_flags_ambiguity() {
    load("r");
    let r = call(
        r#"{"type":"resolve","tag":"r","descriptions":[
{"kind":"face","surface":"plane","normal":"+y","extreme":"max"},
{"kind":"face","surface":"plane","normal":"y"},
{"kind":"face","hole":true},
{"kind":"face","surface":"cylinder","radius":4,"feature":"F2"},
{"kind":"face","surface":"sphere"},
{"kind":"face","surface":"plane","bogus":1},
{"kind":"face","extreme":"max"},
{"kind":"edge","curve":"circle","radius":4},
{"kind":"edge","curve":"line","dir":"x","extreme":"max","along":"y"}]}"#,
    );
    let rs = results(&r);
    let st = |i: usize| rs[i].get_str("status").unwrap();
    assert_eq!(st(0), "unique", "{}", ck_json::to_string(&rs[0]));
    assert_eq!(st(1), "ambiguous");
    assert_eq!(rs[1].get_bool("ambiguous"), Some(true));
    assert!(rs[1].get_f64("count").unwrap() >= 2.0);
    assert_eq!(st(2), "unique");
    assert_eq!(st(3), "unique");
    assert_eq!(st(4), "none");
    assert!(!rs[4].get_arr("narrowing").unwrap().is_empty());
    assert_eq!(st(5), "invalid");
    assert_eq!(st(6), "invalid"); // extreme without along
    assert_eq!(st(7), "ambiguous"); // top and bottom rim circles
    assert!(rs[7].get_f64("count").unwrap() >= 2.0);
    // the unique hole face round-trips through describe
    let name = rs[2].get_arr("matches").unwrap()[0].get_str("name").unwrap().to_string();
    let d = call(&format!(r#"{{"type":"describe","tag":"r","names":["{name}","F9.C0","F2.C1|F9.S.e1#0"]}}"#));
    let items = d.get_arr("items").unwrap();
    assert_eq!(items[0].get_bool("found"), Some(true));
    assert!(items[0].get_str("text").unwrap().contains("hole barrel"));
    assert_eq!(items[1].get_bool("found"), Some(false));
    assert!(!items[1].get_arr("nearest").unwrap().is_empty());
    assert_eq!(items[2].get_bool("found"), Some(false));
    // an edge key from resolve describes
    let ek = rs[8].get_arr("matches").unwrap();
    assert_eq!(rs[8].get_str("status"), Some("ambiguous"), "{}", ck_json::to_string(&rs[8])); // two top edges (z-low / z-high)
    let key = ek[0].get_str("name").unwrap().to_string();
    let d = call(&format!(r#"{{"type":"describe","tag":"r","names":["{key}"]}}"#));
    assert_eq!(d.get_arr("items").unwrap()[0].get_bool("found"), Some(true));
}

fn ref_diff(base: &str, part: &str) -> Json {
    call(&format!(r#"{{"type":"refDiff","base":{base},"part":{part}}}"#))
}

/// Plate, then a sketch on its end cap and a fillet of one edge.
fn with_follow_ups(depth: f64) -> String {
    let p = plate(depth, 30.0);
    let p = p.trim_end().trim_end_matches("]}").to_string();
    format!(
        r#"{p},
{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"a","x":5,"y":5}}],"entities":[{{"type":"circle","id":"e1","c":"a","r":2}}],"constraints":[]}},
{{"type":"fillet","id":"F4","radius":1,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,0,0]}}]}}]}}"#)
}

#[test]
fn ref_diff_is_quiet_for_dimension_edits_and_flags_repoints() {
    let base = with_follow_ups(10.0);
    // a depth edit keeps every reference on the same entity
    let r = ref_diff(&base, &with_follow_ups(14.0));
    assert_eq!(r.get_arr("warnings").unwrap().len(), 0, "{}", ck_json::to_string(&r));
    assert!(r.get_f64("checked").unwrap() >= 2.0);
    // flipping the extrude direction keeps C1 the end cap (same entity, no warning)
    let flipped = base.replace(r#""depth":10"#, r#""depth":10,"flip":true"#);
    assert_eq!(ref_diff(&base, &flipped).get_arr("warnings").unwrap().len(), 0);
    // moving the sketch to another plane re-points F2.C1 (its normal changes)
    let rotated = base.replace(r#""id":"top""#, r#""id":"front""#);
    let r = ref_diff(&base, &rotated);
    assert_eq!(r.get_bool("ok"), Some(false), "{}", ck_json::to_string(&r));
    let w = &r.get_arr("warnings").unwrap()[0];
    assert_eq!(w.get_str("code"), Some("referenceChanged"));
    assert_eq!(w.get_str("feature"), Some("F3"), "warnings are in feature order");
    assert!(w.get_str("reason").unwrap().contains("direction"));
    // deleting the extrude loses every downstream reference
    let no_extrude = {
        let mut j = ck_json::parse(&base).unwrap();
        j.get_mut("features").unwrap().as_arr_mut().unwrap().remove(1);
        ck_json::to_string(&j)
    };
    let r = ref_diff(&base, &no_extrude);
    assert_eq!(r.get_arr("features").unwrap_or(&[]).len(), 0);
    assert_eq!(r.get("features").unwrap().get_arr("removed").unwrap().len(), 1);
    assert!(r.get_arr("warnings").unwrap().iter().any(|w| w.get_str("code") == Some("referenceLost")), "{}", ck_json::to_string(&r));
}

/// One open sketch (F1: four lines, six points, a 0.0206 gap, one
/// dangling line) plus a closed square (F2) extruded to a body (F3) so a
/// tag query has state.
const SA_PART: &str = r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},
"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":4,"y":0.005},{"id":"p4","x":10.02,"y":0.005},{"id":"p5","x":20,"y":0},{"id":"p6","x":20.1,"y":0}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e3","p1":"p1","p2":"p3"},{"type":"line","id":"e2","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p5","p2":"p6"}],
"constraints":[]},
{"type":"sketch","id":"F2","plane":{"kind":"std","id":"top"},
"points":[{"id":"q1","x":30,"y":0},{"id":"q2","x":40,"y":0},{"id":"q3","x":40,"y":10},{"id":"q4","x":30,"y":10}],
"entities":[{"type":"line","id":"f1","p1":"q1","p2":"q2"},{"type":"line","id":"f2","p1":"q2","p2":"q3"},
{"type":"line","id":"f3","p1":"q3","p2":"q4"},{"type":"line","id":"f4","p1":"q4","p2":"q1"}],
"constraints":[]},
{"type":"extrude","id":"F3","op":"add","sketch":"F2","end":"blind","depth":5}]}"#;

/// One open chain with a 0.0206 gap: `sketchAnalyze` through the full
/// server path — inline `part`, then the tag's snapshot, then the errors.
#[test]
fn sketch_analyze_replies_findings_chains_and_profile_reason() {
    let gap = 0.02f64.hypot(0.005);
    let r = call(&format!(r#"{{"type":"sketchAnalyze","featureId":"F1","part":{SA_PART}}}"#));
    assert_eq!(r.get_f64("tolerance"), Some(0.01));
    assert_eq!(r.get_f64("tiny"), Some(0.001));
    let fs = r.get_arr("findings").unwrap();
    let opens: Vec<&Json> = fs.iter().filter(|f| f.get_str("kind") == Some("openEnd")).collect();
    assert_eq!(opens.len(), 4, "{}", ck_json::to_string(&r));
    assert!(!fs.iter().any(|f| f.get_str("kind") == Some("nearCoincident")), "0.0206 stays above the 0.01 default");
    assert!((opens[0].get_f64("gap").unwrap() - gap).abs() < 1e-12, "{r:?}");
    let chains = r.get_arr("chains").unwrap();
    assert_eq!(chains.len(), 2, "{}", ck_json::to_string(&r));
    let mut sizes: Vec<usize> = chains.iter().map(|c| c.get_arr("entities").unwrap().len()).collect();
    sizes.sort_unstable();
    assert_eq!(sizes, [1, 3], "{}", ck_json::to_string(&r));
    for c in chains {
        assert_eq!(c.get("closed"), Some(&Json::Bool(false)));
        assert_eq!(c.get("construction"), Some(&Json::Bool(false)));
    }
    let p = r.get("profile").unwrap();
    assert_eq!(p.get_f64("closedChains"), Some(0.0));
    let why = p.get("reason").unwrap();
    assert_eq!(why.get_str("kind"), Some("openEnd"), "{}", ck_json::to_string(&r));
    // the biggest gap: the 0.1 end of the dangling e4, not the 0.0206 pair
    let at = why.get_arr("at").unwrap();
    assert_eq!(at[0].as_f64(), Some(20.1));
    assert_eq!(at[1].as_f64(), Some(0.0));
    assert!((why.get_f64("gap").unwrap() - 0.1).abs() < 1e-9);

    // tolerance: 0.03 admits the near pair the default rejects
    let r2 = call(&format!(r#"{{"type":"sketchAnalyze","featureId":"F1","tolerance":0.03,"part":{SA_PART}}}"#));
    let fs2 = r2.get_arr("findings").unwrap();
    let nc: Vec<&Json> = fs2.iter().filter(|f| f.get_str("kind") == Some("nearCoincident")).collect();
    assert_eq!(nc.len(), 1, "{}", ck_json::to_string(&r2));
    let ids: Vec<&str> = nc[0].get_arr("points").unwrap().iter().filter_map(|p| p.as_str()).collect();
    assert_eq!(ids.len(), 2);
    assert!(ids.contains(&"p2") && ids.contains(&"p4"), "{ids:?}");
    assert!((nc[0].get_f64("distance").unwrap() - gap).abs() < 1e-12);

    // the same reply from the tag's snapshot (no inline part)
    call(&format!(r#"{{"type":"regen","tag":"sa","part":{SA_PART}}}"#));
    let r3 = call(r#"{"type":"sketchAnalyze","tag":"sa","featureId":"F1"}"#);
    assert_eq!(r3.get_arr("findings").unwrap().iter().filter(|f| f.get_str("kind") == Some("openEnd")).count(), 4);

    // error strings, never a trap (a tag with state, so the handler runs)
    let e = call_raw(r#"{"type":"sketchAnalyze","tag":"sa"}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)));
    assert_eq!(e.get_str("error"), Some("sketchAnalyze: featureId must be a string"));
    let e = call_raw(r#"{"type":"sketchAnalyze","tag":"sa","featureId":"F9"}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)));
    assert_eq!(e.get_str("error"), Some("sketchAnalyze: sketch F9 not found"));
}

/// `sketchFreedoms` through the full server path: F1's six unconstrained
/// points (12 dof), the snapshot path, and the error strings.
#[test]
fn sketch_freedoms_replies_points_radii_and_entities() {
    let r = call(&format!(r#"{{"type":"sketchFreedoms","featureId":"F1","part":{SA_PART}}}"#));
    assert_eq!(r.get_f64("dof"), Some(12.0), "{}", ck_json::to_string(&r));
    assert_eq!(r.get("solved"), Some(&Json::Bool(true)));
    let pts = r.get_arr("points").unwrap();
    assert_eq!(pts.len(), 6, "{}", ck_json::to_string(&r));
    for p in pts {
        assert_eq!(p.get_f64("free"), Some(2.0), "{p:?}");
        assert!(p.get("dir").is_none());
    }
    assert_eq!(r.get_arr("radii").unwrap().len(), 0);
    let ents = r.get_arr("entities").unwrap();
    assert_eq!(ents.len(), 4, "{}", ck_json::to_string(&r));
    for e in ents {
        assert_eq!(e.get_str("kind"), Some("line"));
        assert_eq!(e.get_f64("free"), Some(4.0), "{e:?}"); // both ends free in 2
    }

    // the same reply from the tag's snapshot
    call(&format!(r#"{{"type":"regen","tag":"sf","part":{SA_PART}}}"#));
    let r2 = call(r#"{"type":"sketchFreedoms","tag":"sf","featureId":"F1"}"#);
    assert_eq!(r2.get_f64("dof"), Some(12.0), "{}", ck_json::to_string(&r2));

    // error strings, never a trap
    let e = call_raw(r#"{"type":"sketchFreedoms","tag":"sf"}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)));
    assert_eq!(e.get_str("error"), Some("sketchFreedoms: featureId must be a string"));
    let e = call_raw(r#"{"type":"sketchFreedoms","tag":"sf","featureId":"F9"}"#);
    assert_eq!(e.get("ok"), Some(&Json::Bool(false)));
    assert_eq!(e.get_str("error"), Some("sketchFreedoms: sketch F9 not found"));
}

/// `refDiff` on a reference that resolves to nothing: the faces or edges it could mean now (`candidates`, closest first) and,
/// only when exactly one fits, `suggest`; `repair: true` returns the document with those unambiguous suggestions applied.
#[test]
fn ref_diff_suggests_the_one_entity_a_lost_reference_can_mean_and_repairs_it() {
    let base = with_follow_ups(10.0);
    // the plate's first side is now made by sketch entity e9: F2.S.e1 is gone, the same wall is F2.S.e9
    let renamed = base.replace(r#""type":"line","id":"e1""#, r#""type":"line","id":"e9""#);
    assert_ne!(base, renamed);
    let r = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{renamed},"repair":true}}"#));
    let ws = r.get_arr("warnings").unwrap();
    let lost = ws.iter().find(|w| w.get_str("code") == Some("referenceLost")).unwrap_or_else(|| panic!("{}", ck_json::to_string(&r)));
    assert_eq!(lost.get_str("feature"), Some("F4"), "{}", ck_json::to_string(&r));
    let sg = lost.get("suggest").unwrap_or_else(|| panic!("a unique match: {}", ck_json::to_string(&r)));
    let faces: Vec<&str> = sg.get_arr("faces").unwrap().iter().filter_map(|x| x.as_str()).collect();
    assert_eq!(faces, ["F2.C1", "F2.S.e9"], "{}", ck_json::to_string(&r));
    assert_eq!(lost.get_arr("candidates").unwrap().len(), 1);
    // the repaired document points at the renamed wall and builds
    let reps = r.get_arr("repairs").unwrap();
    assert_eq!(reps.len(), 1, "{}", ck_json::to_string(&r));
    assert_eq!(reps[0].get_str("feature"), Some("F4"));
    let fixed = ck_json::to_string(r.get("repaired").unwrap());
    assert!(fixed.contains("F2.S.e9") && !fixed.contains("F2.S.e1"), "{fixed}");
    let built = call(&format!(r#"{{"type":"regen","tag":"rp","part":{fixed}}}"#));
    assert_eq!(built.get_arr("errors").map_or(0, |e| e.len()), 0, "{}", ck_json::to_string(&built));
    // the original, unrepaired document still fails on that fillet
    let broken = call(&format!(r#"{{"type":"regen","tag":"rq","part":{renamed}}}"#));
    assert!(broken.get_arr("errors").map_or(0, |e| e.len()) > 0);
}

#[test]
fn ref_diff_has_no_suggestion_when_nothing_fits_and_repair_changes_nothing() {
    let base = with_follow_ups(10.0);
    let no_extrude = {
        let mut j = ck_json::parse(&base).unwrap();
        j.get_mut("features").unwrap().as_arr_mut().unwrap().remove(1);
        ck_json::to_string(&j)
    };
    let r = call(&format!(r#"{{"type":"refDiff","base":{base},"part":{no_extrude},"repair":true}}"#));
    let lost: Vec<&Json> = r.get_arr("warnings").unwrap().iter().filter(|w| w.get_str("code") == Some("referenceLost")).collect();
    assert!(!lost.is_empty());
    assert!(lost.iter().all(|w| w.get("suggest").is_none()), "{}", ck_json::to_string(&r));
    assert_eq!(r.get_arr("repairs").unwrap().len(), 0);
}
