//! Test fixtures: our own solids (feature programs), their tessellation as a triangle soup, and small hand-made meshes.
#![allow(dead_code)]

use ck_geom::Vec3;
use ck_json::Json;
use ck_topo::body::Body;

pub fn body_of(features: &str) -> Body {
    let feats = match ck_json::parse(&format!("[{features}]")).unwrap() {
        Json::Arr(a) => a,
        _ => panic!("array expected"),
    };
    let (b, errs) = ck_recog::rebuild::rebuild(&feats).unwrap();
    assert!(errs.is_empty(), "regen errors {errs:?}");
    b.expect("body")
}

/// Rectangle 60 x 40 extruded 10 (F2; top face F2.C1).
pub const RECT: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p2","p2":"p3"},{"id":"e3","type":"line","p1":"p3","p2":"p4"},{"id":"e4","type":"line","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;

/// A closed polygon sketch (id `S`) on the front plane, extruded `depth` (id `X`).
pub fn prism(pts: &[(f64, f64)], depth: f64) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let e: Vec<String> = (0..pts.len()).map(|i| format!(r#"{{"id":"e{i}","type":"line","p1":"p{i}","p2":"p{}"}}"#, (i + 1) % pts.len())).collect();
    format!(
        r#"{{"type":"sketch","id":"S","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","id":"X","op":"add","sketch":"S","end":"blind","depth":{depth}}}"#,
        p.join(","),
        e.join(",")
    )
}

/// A closed polygon profile (x = radius, y = height) revolved a full turn about the sketch y axis.
pub fn revolved(pts: &[(f64, f64)]) -> String {
    let mut p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    p.push(r#"{"id":"a1","x":0,"y":-100}"#.into());
    p.push(r#"{"id":"a2","x":0,"y":100}"#.into());
    let mut e: Vec<String> = (0..pts.len()).map(|i| format!(r#"{{"id":"e{i}","type":"line","p1":"p{i}","p2":"p{}"}}"#, (i + 1) % pts.len())).collect();
    e.push(r#"{"id":"ax","type":"line","p1":"a1","p2":"a2","construction":true}"#.into());
    format!(
        r#"{{"type":"sketch","id":"S","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"revolve","id":"R","op":"add","sketch":"S","axis":"ax","angle":360}}"#,
        p.join(","),
        e.join(",")
    )
}

/// The body tessellated at chord `chord_factor` x the viewer's chord: a triangle soup (9 values per triangle, f32 precision).
pub fn soup(b: &Body, chord_factor: f64) -> Vec<f64> {
    let (lo, hi) = b.bbox();
    let mut tol = ck_tess::curvature::Tol::for_bbox(lo, hi);
    tol.chord *= chord_factor;
    let m = ck_tess::mesh::body_mesh(b, &tol);
    assert!(m.failed.is_empty(), "faces failed to mesh: {:?}", m.failed_why);
    m.positions.iter().map(|&x| x as f64).collect()
}

pub fn soup_tol(b: &Body, tol: &ck_tess::curvature::Tol) -> Vec<f64> {
    let m = ck_tess::mesh::body_mesh(b, tol);
    assert!(m.failed.is_empty(), "faces failed to mesh: {:?}", m.failed_why);
    m.positions.iter().map(|&x| x as f64).collect()
}

/// Indexed unit-ish cube `[0,s]^3` offset by `o`, outward.
pub fn cube(o: Vec3, s: f64) -> (Vec<f64>, Vec<u32>) {
    let mut p = vec![];
    for k in 0..8u32 {
        p.extend([o.x + s * (k & 1) as f64, o.y + s * ((k >> 1) & 1) as f64, o.z + s * ((k >> 2) & 1) as f64]);
    }
    // quads counter-clockwise from outside
    let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
    let mut ix = vec![];
    for q in quads {
        ix.extend([q[0], q[1], q[2], q[0], q[2], q[3]]);
    }
    (p, ix)
}

pub fn opts_faceted() -> ck_mesh::Opts {
    ck_mesh::Opts { analytic: false, ..Default::default() }
}

/// Valid body, one shell per component, volume equal to the mesh's within `rel`.
pub fn assert_solid(c: &ck_mesh::Converted, rel: f64) {
    let issues = ck_check::validate(&c.body, 1e-6);
    assert!(issues.is_empty(), "validate: {:?}", &issues[..issues.len().min(5)]);
    let r = &c.report;
    assert!(r.body_volume > 0.0, "volume {}", r.body_volume);
    assert!(r.volume_rel_err().abs() <= rel, "body {} mesh {} rel {:e}", r.body_volume, r.mesh_volume, r.volume_rel_err());
}

pub fn kinds(b: &Body) -> String {
    ck_json::to_string(&ck_mesh::request::face_types(b))
}
