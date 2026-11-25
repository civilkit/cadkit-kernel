//! Chamfers on free-form rims (issue 69). Bodies are built here from part JSON.
//! (1) A lofted taper: its top rim is a circle carried on a B-spline surface, split in two arcs. Equal, two-distance
//! and angle chamfers build, validate and change the volume by the section wedge times the rim length (Pappus); swapping
//! the distances, or the order of the two faces of a pick, swaps the legs consistently.
//! (2) An extruded closed spline block: a chamfer on the true free-form edge is refused with `blendUnsupported` and a
//! named detail, and the body stays valid. The variable chamfer refusal names its detail and hint as well.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

const R0: f64 = 34.695; // base radius
const R1: f64 = 14.5; // rim radius
const H: f64 = 38.45;

struct Out {
    body: Option<Body>,
    errors: Vec<(String, String, Vec<(String, String)>)>,
}

fn run(feats: &[String]) -> Out {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let errors = r
        .errors
        .iter()
        .map(|e| (e.feature.clone(), e.error.code.to_string(), e.error.params.iter().map(|(k, v)| (k.to_string(), format!("{v:?}"))).collect()))
        .collect();
    Out { body: br.body, errors }
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn valid(b: &Body) -> bool {
    ck_check::validate(b, 1e-6).is_empty()
}

fn taper() -> Vec<String> {
    vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":34.695}],"constraints":[]}"#.into(),
        r#"{"type":"plane","id":"F2","mode":"offset","base":{"kind":"std","id":"top"},"offset":38.45}"#.into(),
        r#"{"type":"sketch","id":"F3","plane":{"kind":"feature","id":"F2"},"points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":14.5}],"constraints":[]}"#.into(),
        r#"{"type":"loft","id":"F4","op":"add","sketches":["F1","F3"]}"#.into(),
    ]
}

/// A pick for every edge of face `cap`, listing the faces in the order (cap, other) or (other, cap).
fn picks(b: &Body, cap: &str, cap_first: bool) -> Vec<String> {
    let f = b.find_face(cap).unwrap();
    let mut out = vec![];
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !fs.contains(&f) {
            continue;
        }
        let ed = b.edge(e);
        let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let other = if fs[0] == f { fs[1] } else { fs[0] };
        let (a, c) = (b.face_id_str(f), b.face_id_str(other));
        let (x, y) = if cap_first { (a, c) } else { (c, a) };
        out.push(format!(r#"{{"faces":["{x}","{y}"],"at":[{:.9},{:.9},{:.9}]}}"#, p.x, p.y, p.z));
    }
    out
}

fn base_taper() -> (Vec<String>, f64, usize) {
    let o = run(&taper());
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b));
    let n = picks(b, "F4.C1", true).len();
    (taper(), vol(b), n)
}

/// Chamfer of the rim: returns the volume removed.
fn chamfer_rim(head: &str, cap_first: bool) -> f64 {
    let (mut feats, v0, n) = base_taper();
    assert_eq!(n, 2, "the rim is two arcs");
    let b = run(&feats).body.unwrap();
    let ps = picks(&b, "F4.C1", cap_first);
    feats.push(format!(r#"{{"type":"chamfer","id":"F5",{head},"edges":[{}]}}"#, ps.join(",")));
    let o = run(&feats);
    assert!(o.errors.is_empty(), "{head}: {:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(valid(b), "{head}: invalid");
    v0 - vol(b)
}

/// Section wedge of a chamfer with distance `d1` on the cap and `d2` on the cone, times the rim length (Pappus).
fn wedge(d1: f64, d2: f64) -> f64 {
    let l = (H * H + (R0 - R1) * (R0 - R1)).sqrt();
    let c = -(R0 - R1) / l; // cos of the interior angle at the rim
    let area = 0.5 * d1 * d2 * (1.0 - c * c).sqrt();
    // centroid radius: corner (R1, 0), cap leg towards the axis, cone leg down and outward
    let rc = R1 + (-d1 + d2 * (R0 - R1) / l) / 3.0;
    area * std::f64::consts::TAU * rc
}

fn close(got: f64, want: f64, what: &str) {
    assert!((got - want).abs() <= 0.005 * want, "{what}: removed {got}, wedge {want}");
}

#[test]
fn taper_equal_chamfer_is_the_wedge() {
    close(chamfer_rim(r#""chamferType":"equal","distance":1.45"#, true), wedge(1.45, 1.45), "equal");
}

#[test]
fn taper_two_distance_chamfer_is_the_wedge_and_swaps_with_the_faces() {
    let a = chamfer_rim(r#""chamferType":"distances","distance":1.45,"distance2":0.8"#, true);
    close(a, wedge(1.45, 0.8), "1.45 on the cap, 0.8 on the cone");
    // the other order of the faces of each pick: the first distance now follows the cone
    let b = chamfer_rim(r#""chamferType":"distances","distance":1.45,"distance2":0.8"#, false);
    close(b, wedge(0.8, 1.45), "faces listed cone first");
    // the same legs reached by swapping the distances
    let c = chamfer_rim(r#""chamferType":"distances","distance":0.8,"distance2":1.45"#, true);
    close(c, wedge(0.8, 1.45), "0.8 on the cap, 1.45 on the cone");
    assert!((b - c).abs() <= 1e-3 * c, "mirrored cases differ: {b} vs {c}");
    assert!((a - b).abs() > 0.005 * a, "the two leg assignments must differ");
}

#[test]
fn taper_angle_chamfer_is_the_wedge() {
    // the chamfer makes 30 degrees with the cap; the faces meet at theta (117.7 degrees), so the law of sines in the
    // section triangle gives the leg on the cone (a right angle would give d tan 30)
    let l = (H * H + (R0 - R1) * (R0 - R1)).sqrt();
    let theta = (-(R0 - R1) / l).acos();
    let d2 = 1.45 * 30f64.to_radians().sin() / (std::f64::consts::PI - theta - 30f64.to_radians()).sin();
    close(chamfer_rim(r#""chamferType":"angle","distance":1.45,"angle":30"#, true), wedge(1.45, d2), "angle");
}

// ---- true free-form edge: closed spline profile extruded

fn spline_block() -> Vec<String> {
    let n = 8;
    let pts: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / n as f64;
            ((20.0 + 3.0 * (3.0 * a).cos()) * a.cos(), (20.0 + 3.0 * (3.0 * a).cos()) * a.sin())
        })
        .collect();
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x:.9},"y":{y:.9}}}"#)).collect();
    let ids: Vec<String> = (0..n).map(|i| format!(r#""p{i}""#)).collect();
    vec![
        format!(
            r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{{"type":"spline","id":"e1","pts":[{}],"closed":true}}],"constraints":[]}}"#,
            ps.join(","),
            ids.join(",")
        ),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":15}"#.into(),
    ]
}

/// Chamfers the rim edge(s) of the cap `F2.C1` with `head`; returns the outcome and the volume before.
fn chamfer_spline(head: &str) -> (Out, f64) {
    let mut feats = spline_block();
    let b = run(&feats).body.unwrap();
    assert!(valid(&b));
    let v0 = vol(&b);
    let ps = picks(&b, "F2.C1", true);
    assert!(!ps.is_empty());
    feats.push(format!(r#"{{"type":"chamfer","id":"F3",{head},"edges":[{}]}}"#, ps[..1].join(",")));
    (run(&feats), v0)
}

fn refused(head: &str) -> Vec<(String, String)> {
    let (o, v0) = chamfer_spline(head);
    assert_eq!(o.errors.len(), 1, "{head}: {:?}", o.errors);
    let (feat, code, params) = &o.errors[0];
    assert_eq!((feat.as_str(), code.as_str()), ("F3", "blendUnsupported"), "{head}");
    let b = o.body.as_ref().expect("the body before the chamfer is kept");
    assert!(valid(b), "{head}: invalid");
    assert!((vol(b) - v0).abs() <= 1e-9 * v0, "{head}: volume changed");
    params.clone()
}

#[test]
fn spline_rim_chamfers_are_refused_with_a_detail() {
    for head in [r#""chamferType":"equal","distance":1"#, r#""chamferType":"distances","distance":1,"distance2":0.5"#, r#""chamferType":"angle","distance":1,"angle":30"#] {
        let p = refused(head);
        let detail = p.iter().find(|(k, _)| k == "detail").unwrap_or_else(|| panic!("{head}: no detail {p:?}"));
        assert!(detail.1.contains("chamfer") && detail.1.contains("spline"), "{head}: {detail:?}");
        assert!(p.iter().any(|(k, _)| k == "hint"), "{head}: no hint");
    }
}

#[test]
fn variable_chamfer_refusal_on_a_free_form_edge_has_a_detail_and_hint() {
    let p = refused(r#""chamferType":"variable","startDistance":1,"endDistance":2"#);
    let get = |k: &str| p.iter().find(|(n, _)| n == k).unwrap_or_else(|| panic!("no {k}: {p:?}")).1.clone();
    assert!(get("detail").contains("variable chamfer") && get("detail").contains("spline"), "{p:?}");
    let hint = get("hint");
    assert!(hint.contains("variable") && hint.contains("straight"), "{hint}");
    // the circular taper rim is refused the same way, naming the edge kind
    let (mut feats, _, _) = base_taper();
    let b = run(&feats).body.unwrap();
    feats.push(format!(r#"{{"type":"chamfer","id":"F5","chamferType":"variable","startDistance":1,"endDistance":2,"edges":[{}]}}"#, picks(&b, "F4.C1", true)[..1].join(",")));
    let o = run(&feats);
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].1, "blendUnsupported");
    assert!(o.errors[0].2.iter().any(|(k, _)| k == "detail"), "{:?}", o.errors[0]);
}
