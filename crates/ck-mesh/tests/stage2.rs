//! Stage 2: analytic refit. Our own solids with cylinders, cones and spheres are tessellated and converted back: the
//! result must be valid, recover the face types, keep the volume within the chord error and lie close to the original.
mod common;
use common::*;
use ck_mesh::{mesh_to_solid, Converted, Opts};

struct Case {
    c: Converted,
    chord: f64,
    v_true: f64,
}

fn convert(src: &str, chord_factor: f64) -> Case {
    let b = body_of(src);
    let v_true = ck_topo::mass::mass_props(&b).volume;
    let (lo, hi) = b.bbox();
    let mut tol = ck_tess::curvature::Tol::for_bbox(lo, hi);
    tol.chord *= chord_factor;
    let s = soup_tol(&b, &tol);
    let t = std::time::Instant::now();
    let c = match mesh_to_solid(&s, None, &Opts::default()) {
        Ok(c) => c,
        Err(r) => panic!("refused {}: {}", r.code, r.message),
    };
    let a = c.report.analytic.as_ref().unwrap();
    let d = c.report.deviation.as_ref().unwrap();
    eprintln!(
        "{} triangles -> {} faces {} in {:.0} ms; applied {} {:?}; declined {:?}; volume true {v_true:.4} mesh {:.4} body {:.4}; dev max {:.2e} rms {:.2e} (chord {:.2e})",
        s.len() / 9,
        c.body.num_faces(),
        kinds(&c.body),
        t.elapsed().as_secs_f64() * 1e3,
        a.applied,
        a.reason,
        a.declined,
        c.report.mesh_volume,
        c.report.body_volume,
        d.max,
        d.rms,
        tol.chord
    );
    Case { c, chord: tol.chord, v_true }
}

/// Valid, the face types, volume within `vol_rel` of the true solid, deviation within twice the chord heights.
fn check(k: &Case, want: &str, vol_rel: f64) {
    let issues = ck_check::validate(&k.c.body, 1e-6);
    assert!(issues.is_empty(), "validate: {:?}", &issues[..issues.len().min(5)]);
    assert_eq!(kinds(&k.c.body), want);
    let rel = (k.c.report.body_volume - k.v_true) / k.v_true;
    assert!(rel.abs() <= vol_rel, "volume {} vs {} rel {rel:e}", k.c.report.body_volume, k.v_true);
    let d = k.c.report.deviation.as_ref().unwrap();
    assert!(d.max <= 2.0 * (k.chord + d.chord), "deviation {} chord {} + {}", d.max, k.chord, d.chord);
}

const HOLE: &str = r#"{"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"k","x":5,"y":3}],"entities":[{"id":"pt1","type":"point","p":"k"}],"constraints":[]},
{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}"#;

#[test]
fn drilled_block() {
    let k = convert(&format!("{RECT},{HOLE}"), 1.0);
    check(&k, r#"{"cylinder":1,"plane":6}"#, 5e-3);
}

#[test]
fn cylinder_boss_on_plate() {
    let boss = r#"{"type":"sketch","id":"F3","plane":{"kind":"face","id":"F2.C1"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"id":"k1","type":"circle","c":"c","r":8}],"constraints":[]},
    {"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":12}"#;
    let k = convert(&format!("{RECT},{boss}"), 1.0);
    check(&k, r#"{"cylinder":1,"plane":7}"#, 5e-3);
}

#[test]
fn sphere() {
    let k = convert(r#"{"type":"sphere","id":"F1","op":"add","center":[0,0,0],"radius":10}"#, 1.0);
    check(&k, r#"{"sphere":1}"#, 5e-3);
}

#[test]
fn cone_frustum() {
    let k = convert(&revolved(&[(0.0, 0.0), (10.0, 0.0), (5.0, 20.0), (0.0, 20.0)]), 1.0);
    check(&k, r#"{"cone":1,"plane":2}"#, 5e-3);
}

#[test]
fn revolved_shaft() {
    let k = convert(&revolved(&[(0.0, 0.0), (10.0, 0.0), (10.0, 20.0), (6.0, 30.0), (6.0, 40.0), (0.0, 40.0)]), 1.0);
    check(&k, r#"{"cone":1,"cylinder":2,"plane":2}"#, 5e-3);
}

#[test]
fn filleted_box() {
    let f = r#"{"type":"fillet","id":"F5","radius":5,"edges":[{"faces":["F2.S.e1","F2.S.e2"]},{"faces":["F2.S.e2","F2.S.e3"]},{"faces":["F2.S.e3","F2.S.e4"]},{"faces":["F2.S.e4","F2.S.e1"]}]}"#;
    let k = convert(&format!("{RECT},{f}"), 1.0);
    check(&k, r#"{"cylinder":4,"plane":6}"#, 5e-3);
}

#[test]
fn pointed_cone() {
    // the apex is a pole of the cone face: its seam runs from the base circle to the tip
    let k = convert(&revolved(&[(0.0, 0.0), (10.0, 0.0), (0.0, 20.0)]), 1.0);
    check(&k, r#"{"cone":1,"plane":1}"#, 5e-3);
}

#[test]
fn capsule_end() {
    // a cylinder with a hemispherical end: sphere and cylinder meet tangentially on one circle
    let src = r#"{"type":"sketch","id":"S","plane":{"kind":"std","id":"front"},"points":[{"id":"p0","x":0,"y":0},{"id":"p1","x":10,"y":0},{"id":"p2","x":10,"y":20},{"id":"c","x":0,"y":20},{"id":"p3","x":0,"y":30},{"id":"a1","x":0,"y":-100},{"id":"a2","x":0,"y":100}],"entities":[{"id":"e0","type":"line","p1":"p0","p2":"p1"},{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"arc","c":"c","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p0"},{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}],"constraints":[]},{"type":"revolve","id":"R","op":"add","sketch":"S","axis":"ax","angle":360}"#;
    let k = convert(src, 1.0);
    check(&k, r#"{"cylinder":1,"plane":1,"sphere":1}"#, 5e-3);
}
