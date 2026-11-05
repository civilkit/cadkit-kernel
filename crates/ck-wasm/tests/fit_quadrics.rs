//! `fitQuadrics` through the full server path: a faceted cylinder comes in as STEP (a mesh import is the same kind of body: planar faces
//! on line edges), is refit as one cylinder face with circle edges, and keeps validating, measuring and exporting.

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_json::Json;
use ck_topo::body::{Body, FaceName};
use std::f64::consts::TAU;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

/// A prism of `n` sides, radius `r`, height `h`, as planar faces on shared line edges.
fn prism(n: usize, r: f64, h: f64) -> Body {
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let ring = |z: f64| -> Vec<Vec3> { (0..n).map(|i| { let t = TAU * i as f64 / n as f64; Vec3::new(r * t.cos(), r * t.sin(), z) }).collect() };
    let (lo, hi) = (ring(0.0), ring(h));
    let vlo: Vec<_> = lo.iter().map(|&p| b.add_vertex(p)).collect();
    let vhi: Vec<_> = hi.iter().map(|&p| b.add_vertex(p)).collect();
    let line = |b: &mut Body, a, c| { let (pa, pc) = (b.vertex(a).p, b.vertex(c).p); b.add_edge(Curve::line_through(pa, pc), 0.0, pa.dist(pc), a, c) };
    let e_lo: Vec<_> = (0..n).map(|i| line(&mut b, vlo[i], vlo[(i + 1) % n])).collect();
    let e_hi: Vec<_> = (0..n).map(|i| line(&mut b, vhi[i], vhi[(i + 1) % n])).collect();
    let e_rail: Vec<_> = (0..n).map(|i| line(&mut b, vlo[i], vhi[i])).collect();
    // caps
    let bottom = b.add_face(shell, Surface::Plane { frame: Frame { o: Vec3::ZERO, x: Vec3::X, y: -Vec3::Y, z: -Vec3::Z } }, true, FaceName::other("m", "F0"));
    let l = b.add_loop(bottom);
    for i in (0..n).rev() {
        b.add_coedge(l, e_lo[i], true, None);
    }
    let top = b.add_face(shell, Surface::Plane { frame: Frame { o: Vec3::new(0.0, 0.0, h), x: Vec3::X, y: Vec3::Y, z: Vec3::Z } }, true, FaceName::other("m", "F1"));
    let l = b.add_loop(top);
    for i in 0..n {
        b.add_coedge(l, e_hi[i], false, None);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let t = TAU * (i as f64 + 0.5) / n as f64;
        let nrm = Vec3::new(t.cos(), t.sin(), 0.0);
        let x = (lo[j] - lo[i]).normalize();
        let f = b.add_face(shell, Surface::Plane { frame: Frame { o: lo[i], x, y: Vec3::Z, z: nrm } }, true, FaceName::other("m", &format!("F{}", i + 2)));
        let l = b.add_loop(f);
        b.add_coedge(l, e_lo[i], false, None);
        b.add_coedge(l, e_rail[j], false, None);
        b.add_coedge(l, e_hi[i], true, None);
        b.add_coedge(l, e_rail[i], true, None);
    }
    b
}

fn import(tag: &str, b: &Body) -> Json {
    let text = ck_io::step::write::write_step(b, "facets", &Default::default());
    let msg = ck_json::to_string(&Json::Obj(vec![("type".into(), "importStep".into()), ("tag".into(), tag.into()), ("name".into(), "facets".into()), ("data".into(), text.into())]));
    call(&msg)
}

fn res(j: &Json) -> &Json {
    j.get("result").unwrap()
}

#[test]
fn a_faceted_cylinder_is_refit_and_stays_a_valid_exportable_solid() {
    let n = 48;
    let b = prism(n, 10.0, 20.0);
    let imp = import("m", &b);
    assert_eq!(res(&imp).get_arr("errors").unwrap().len(), 0, "{imp:?}");
    let before = call(r#"{"type":"massProps","tag":"m"}"#);
    let v0 = res(&before).get_f64("volume").unwrap();
    assert!(v0 > 6000.0 && v0 < std::f64::consts::PI * 100.0 * 20.0, "{v0}");

    let r = call(r#"{"type":"fitQuadrics","tag":"m","opts":{"tol":0.2}}"#);
    let rr = res(&r);
    assert_eq!(rr.get_bool("applied"), Some(true), "{r:?}");
    assert_eq!(rr.get("faces").unwrap().get_f64("before"), Some((n + 2) as f64));
    assert_eq!(rr.get("faces").unwrap().get_f64("after"), Some(3.0));
    let g = &rr.get_arr("groups").unwrap()[0];
    assert_eq!(g.get_str("kind"), Some("cylinder"));
    assert_eq!(g.get_f64("strips"), Some(n as f64));
    assert!((g.get_f64("radius").unwrap() - 10.0).abs() < 1e-6);
    // the tag now holds the cylinder
    let after = call(r#"{"type":"massProps","tag":"m"}"#);
    let v1 = res(&after).get_f64("volume").unwrap();
    assert!((v1 - std::f64::consts::PI * 100.0 * 20.0).abs() < 1e-4, "{v1}");
    let v = call(r#"{"type":"validate","tag":"m"}"#);
    assert_eq!(res(&v).get_bool("ok"), Some(true), "{v:?}");
    let step = call(r#"{"type":"step","tag":"m","name":"fitted"}"#);
    let text = res(&step).as_str().unwrap();
    assert!(text.contains("CYLINDRICAL_SURFACE") && text.contains("CIRCLE"), "the STEP writes a cylinder and circles");
}

#[test]
fn dry_run_reports_and_keeps_the_body_and_a_small_tolerance_declines() {
    let b = prism(16, 10.0, 20.0);
    import("d", &b);
    let faces = |tag: &str| -> f64 { let r = call(&format!(r#"{{"type":"inspect","tag":"{tag}"}}"#)); res(&r).get_f64("faces").unwrap_or(f64::NAN) };
    let _ = faces;
    // too coarse for 0.05 mm: declined with the reason, nothing applied
    let r = call(r#"{"type":"fitQuadrics","tag":"d","opts":{"tol":0.05}}"#);
    let rr = res(&r);
    assert_eq!(rr.get_bool("applied"), Some(false));
    assert_eq!(rr.get_str("reason"), Some("nothing"));
    assert!(rr.get_arr("declined").unwrap().iter().any(|d| d.get_str("reason") == Some("sag")), "{r:?}");
    // a dry run on a fit that would go through
    let r = call(r#"{"type":"fitQuadrics","tag":"d","opts":{"tol":0.5,"dryRun":true}}"#);
    let rr = res(&r);
    assert_eq!(rr.get_bool("applied"), Some(false));
    assert_eq!(rr.get_str("reason"), Some("dryRun"));
    assert_eq!(rr.get_arr("groups").unwrap().len(), 1);
    let v = call(r#"{"type":"massProps","tag":"d"}"#);
    assert!(res(&v).get_f64("volume").unwrap() < std::f64::consts::PI * 100.0 * 20.0 * 0.98, "the body is unchanged");
    // a volume bound the fit exceeds (16 sides change the volume by 2.6 %)
    let r = call(r#"{"type":"fitQuadrics","tag":"d","opts":{"tol":0.5,"volumeTol":0.01}}"#);
    assert_eq!(res(&r).get_str("reason"), Some("volumeChanged"));
    // and one it does not
    let r = call(r#"{"type":"fitQuadrics","tag":"d","opts":{"tol":0.5}}"#);
    assert_eq!(res(&r).get_bool("applied"), Some(true), "{r:?}");
}

#[test]
fn a_tag_without_a_body_answers_null() {
    let r = call(r#"{"type":"fitQuadrics","tag":"nothing-here"}"#);
    assert!(matches!(r.get("result"), Some(Json::Null) | None), "{r:?}");
}
