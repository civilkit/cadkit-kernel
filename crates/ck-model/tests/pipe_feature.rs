//! The `pipe` feature (see features/solids.rs): a circular tube along a sketch path.
mod solids_common;
use solids_common::*;
use std::f64::consts::PI;

fn path(entities: &str, points: &str) -> String {
    format!(r#"{{"type":"sketch","id":"S","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{entities}],"constraints":[]}}"#)
}
fn pipe(extra: &str) -> String {
    format!(r#"{{"type":"pipe","id":"F2","op":"add","path":{{"kind":"sketch","id":"S"}}{}{extra}}}"#, if extra.is_empty() { "" } else { "," })
}
fn line_path() -> String {
    path(r#"{"type":"line","id":"l1","p1":"a","p2":"b"}"#, r#"{"id":"a","x":0,"y":0},{"id":"b","x":30,"y":0}"#)
}

#[test]
fn a_straight_solid_pipe_is_a_cylinder() {
    let r = run(&format!("{},{}", line_path(), pipe(r#""radius":5"#)));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.valid.is_empty(), "{:?}", r.valid);
    assert!(near(r.vol, PI * 25.0 * 30.0, 1e-6), "{}", r.vol);
    assert_eq!(r.faces.len(), 3, "{:?}", r.faces);
}

#[test]
fn a_wall_makes_it_a_hollow_tube() {
    let r = run(&format!("{},{}", line_path(), pipe(r#""radius":5,"wall":1"#)));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, PI * (25.0 - 16.0) * 30.0, 1e-6), "{}", r.vol);
    assert_eq!(r.faces.len(), 4, "{:?}", r.faces);
}

#[test]
fn a_bent_path_is_mitred_and_keeps_the_centre_line_volume() {
    let p = path(
        r#"{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"}"#,
        r#"{"id":"a","x":0,"y":0},{"id":"b","x":30,"y":0},{"id":"c","x":30,"y":20}"#,
    );
    let r = run(&format!("{p},{}", pipe(r#""radius":4"#)));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, PI * 16.0 * 50.0, 2e-3), "{}", r.vol);
}

#[test]
fn an_arc_path_is_a_torus_segment() {
    // a quarter turn of radius 20 about the sketch origin, starting on +x
    let p = path(r#"{"type":"arc","id":"a1","c":"o","p1":"a","p2":"b"}"#, r#"{"id":"o","x":0,"y":0},{"id":"a","x":20,"y":0},{"id":"b","x":0,"y":20}"#);
    let r = run(&format!("{p},{}", pipe(r#""radius":3"#)));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    assert!(near(r.vol, PI * 9.0 * (PI / 2.0 * 20.0), 2e-3), "{}", r.vol);
}

#[test]
fn cut_removes_the_tube_from_a_body() {
    let r = run(&format!("{BLOCK},{},{}", line_path(), pipe(r#""op":"cut","radius":5"#).replace(r#""op":"add","#, "")));
    assert!(r.errors.is_empty() && r.valid.is_empty(), "{:?} {:?}", r.errors, r.valid);
    // the path runs 0..30 along x through the middle of the 40 block, from the centre: half of it lies in the block
    assert!(near(r.vol, 64000.0 - PI * 25.0 * 20.0, 2e-3), "{}", r.vol);
}

#[test]
fn bad_input_is_refused_with_stable_codes() {
    let l = line_path();
    for extra in [r#""radius":0"#, r#""radius":-2"#, r#""radius":5,"wall":5"#, r#""radius":5,"wall":0"#, r#""radius":5,"wall":-1"#, ""] {
        let r = run(&format!("{l},{}", pipe(extra)));
        assert_eq!(r.codes, ["invalidValue"], "{extra}: {:?}", r.errors);
    }
    // no path, an unknown sketch
    let r = run(&format!("{l},{}", r#"{"type":"pipe","id":"F2","radius":5}"#));
    assert_eq!(r.codes, ["missingPath"], "{:?}", r.errors);
    let r = run(&format!("{l},{}", r#"{"type":"pipe","id":"F2","radius":5,"path":{"kind":"sketch","id":"nope"}}"#));
    assert_eq!(r.codes, ["missingPath"], "{:?}", r.errors);
    // a closed loop is not an open path
    let closed = path(
        r#"{"type":"line","id":"l1","p1":"a","p2":"b"},{"type":"line","id":"l2","p1":"b","p2":"c"},{"type":"line","id":"l3","p1":"c","p2":"a"}"#,
        r#"{"id":"a","x":0,"y":0},{"id":"b","x":30,"y":0},{"id":"c","x":30,"y":20}"#,
    );
    let r = run(&format!("{closed},{}", pipe(r#""radius":2"#)));
    assert_eq!(r.codes, ["sweepPathInvalid"], "{:?}", r.errors);
    let r = run(&format!("{l},{}", r#"{"type":"pipe","id":"F2","op":"cut","radius":5,"path":{"kind":"sketch","id":"S"}}"#));
    assert_eq!(r.codes, ["nothingToCut"], "{:?}", r.errors);
}
