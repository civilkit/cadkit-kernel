//! A round or chamfer on the rim of an extruded closed spline (six points, 20 mm blind): built in place on the profile's offset
//! curves (`blend/spline_rim.rs`), valid and with the volume the rim's section area and length give.
//! Expected removed volume (Pappus, the rim a simple closed curve of length L): round `(1 - pi/4) r^2 (L - 2 pi xr)` with the
//! centroid of the removed corner `xr = r (5/6 - pi/4) / (1 - pi/4)` inside the rim; chamfer `d^2/2 (L - 2 pi d/3)`.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::f64::consts::PI;

const PTS: [(f64, f64); 6] = [(30.0, 0.0), (18.0, 22.0), (-8.0, 26.0), (-30.0, 6.0), (-14.0, -22.0), (12.0, -20.0)];

fn sketch(kind: &str) -> String {
    let p: Vec<String> = PTS.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ids: Vec<String> = (0..PTS.len()).map(|i| format!(r#""p{i}""#)).collect();
    let deg = if kind == "cvspline" { r#","degree":3"# } else { "" };
    format!(
        r#"{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"{kind}","pts":[{}],"closed":true{deg}}}],"constraints":[]}}"#,
        p.join(","),
        ids.join(",")
    )
}

fn regen_doc(doc: &str) -> (Option<Body>, Vec<String>) {
    let part = Part::parse(doc).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

fn doc(kind: &str, extra: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":20,"id":"F2"}}{extra}]}}"#, sketch(kind))
}

fn base(kind: &str) -> Body {
    regen_doc(&doc(kind, "")).0.expect("base body")
}

/// Picks of the rim edges of `face`: all of them (chain on), or just one (chain off).
fn picks(b: &Body, face: &str, one: bool) -> Vec<String> {
    let f = b.find_face(face).unwrap();
    let mut out = vec![];
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !fs.contains(&f) {
            continue;
        }
        let p = b.vertex(b.edge(e).v0).p;
        out.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z));
    }
    if one {
        out.truncate(1);
    }
    out
}

fn rim_length(b: &Body, face: &str) -> f64 {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).map(|e| b.edge_length(e)).sum()
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn removed(chamfer: bool, s: f64, len: f64) -> f64 {
    if chamfer {
        s * s / 2.0 * (len - 2.0 * PI * s / 3.0)
    } else {
        let xr = s * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
        (1.0 - PI / 4.0) * s * s * (len - 2.0 * PI * xr)
    }
}

/// Builds `kind` of size `s` on the rims `faces`, returns the body.
fn build(sk: &str, chamfer: bool, s: f64, faces: &[&str], one: bool) -> Result<Body, Vec<String>> {
    let b = base(sk);
    let edges: Vec<String> = faces.iter().flat_map(|f| picks(&b, f, one)).collect();
    let feat = if chamfer {
        format!(r#","{}""#, "")
    } else {
        String::new()
    };
    let _ = feat;
    let f = if chamfer {
        format!(r#",{{"id":"F3","type":"chamfer","distance":{s},"edges":[{}]}}"#, edges.join(","))
    } else {
        format!(r#",{{"id":"F3","type":"fillet","radius":{s},"edges":[{}]}}"#, edges.join(","))
    };
    let (out, errs) = regen_doc(&doc(sk, &f));
    match out {
        Some(o) if errs.is_empty() => Ok(o),
        _ => Err(errs),
    }
}

fn check(name: &str, sk: &str, chamfer: bool, s: f64, faces: &[&str], one: bool) {
    let b0 = base(sk);
    let out = build(sk, chamfer, s, faces, one).unwrap_or_else(|e| panic!("{name}: refused: {e:?}"));
    let issues = ck_check::validate(&out, 1e-6);
    assert!(issues.is_empty(), "{name}: invalid {:?}", issues.iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>());
    let want: f64 = faces.iter().map(|f| removed(chamfer, s, rim_length(&b0, f))).sum();
    let got = vol(&b0) - vol(&out);
    // both rims at once consume the same wall: the sections do not overlap while 2 s < the wall height
    assert!((got - want).abs() <= 2e-3 * want + 1e-3, "{name}: removed {got:.4}, expected {want:.4}");
    let nb = out.face_ids().filter(|&f| out.face_id_str(f).contains(".B")).count();
    assert!(nb >= 2 * faces.len().max(1), "{name}: {nb} blend faces");
}

#[test]
fn top_rim_every_radius_chain_on_and_off() {
    for r in [0.5, 1.0, 2.0, 3.0, 5.0] {
        check(&format!("top r{r} chain on"), "spline", false, r, &["F2.C1"], false);
        check(&format!("top r{r} chain off"), "spline", false, r, &["F2.C1"], true);
    }
}

#[test]
fn chamfer_on_the_rim() {
    for d in [0.5, 1.0, 2.0, 3.0] {
        check(&format!("chamfer {d} chain on"), "spline", true, d, &["F2.C1"], false);
        check(&format!("chamfer {d} chain off"), "spline", true, d, &["F2.C1"], true);
    }
}

#[test]
fn bottom_rim_and_both_rims() {
    for r in [1.0, 2.0, 5.0] {
        check(&format!("bottom r{r}"), "spline", false, r, &["F2.C0"], false);
        check(&format!("bottom r{r} chain off"), "spline", false, r, &["F2.C0"], true);
        check(&format!("both r{r}"), "spline", false, r, &["F2.C1", "F2.C0"], false);
    }
    check("bottom chamfer 2", "spline", true, 2.0, &["F2.C0"], false);
    check("both chamfer 2", "spline", true, 2.0, &["F2.C1", "F2.C0"], false);
}

#[test]
fn control_vertex_spline() {
    for r in [0.5, 2.0, 3.0] {
        check(&format!("cvspline top r{r}"), "cvspline", false, r, &["F2.C1"], false);
    }
    check("cvspline top chain off", "cvspline", false, 2.0, &["F2.C1"], true);
    check("cvspline bottom", "cvspline", false, 2.0, &["F2.C0"], false);
    check("cvspline chamfer", "cvspline", true, 1.5, &["F2.C1"], false);
}

#[test]
fn too_large_is_a_named_fast_refusal_never_a_broken_solid() {
    let t = std::time::Instant::now();
    // wider than the wall is tall
    for (r, rims) in [(12.0, &["F2.C1"][..]), (11.0, &["F2.C1", "F2.C0"][..])] {
        match build("spline", false, r, rims, false) {
            Err(errs) => { eprintln!("r{r}: {errs:?}"); assert!(errs.iter().any(|e| e.contains("blendUnsupported") || e.contains("blendFailed") || e.contains("blendTooLarge")), "r{r}: {errs:?}"); }
            Ok(b) => assert!(ck_check::validate(&b, 1e-6).is_empty(), "r{r}: built but invalid"),
        }
    }
    // tighter than the profile's tightest bend (the offset turns inside out)
    match build("spline", false, 19.0, &["F2.C1"], false) {
        Err(errs) => { eprintln!("r19: {errs:?}"); assert!(!errs.is_empty()) }
        Ok(b) => assert!(ck_check::validate(&b, 1e-6).is_empty()),
    }
    assert!(t.elapsed().as_secs_f64() < 30.0, "refusals took {:?}", t.elapsed());
}

/// The display mesh of the rounded solid: every face meshes, and the mesh encloses the volume the solid has.
#[test]
fn rounded_rim_meshes_closed() {
    for (chamfer, s) in [(false, 2.0), (true, 2.0)] {
        let b = build("spline", chamfer, s, &["F2.C1", "F2.C0"], false).unwrap();
        let m = ck_tess::mesh::tessellate(&b).mesh;
        assert!(m.failed.is_empty(), "failed faces {:?} {:?}", m.failed, m.failed_why);
        let p = &m.positions;
        let mut v = 0.0f64;
        for t in 0..m.num_tris() {
            let q = |k: usize| [p[9 * t + k] as f64, p[9 * t + k + 1] as f64, p[9 * t + k + 2] as f64];
            let (a, b2, c) = (q(0), q(3), q(6));
            v += (a[0] * (b2[1] * c[2] - b2[2] * c[1]) - a[1] * (b2[0] * c[2] - b2[2] * c[0]) + a[2] * (b2[0] * c[1] - b2[1] * c[0])) / 6.0;
        }
        let want = vol(&b);
        assert!((v - want).abs() < 2e-3 * want, "chamfer {chamfer}: mesh volume {v:.2} against {want:.2}");
    }
}

/// The same profile on the other standard planes and extruded the other way (reversed, mid-plane): the rim's outward normal, the wall
/// direction and the cap orientation all change sign, the round must still build with the same volume.
#[test]
fn other_planes_and_directions() {
    for plane in ["front", "top", "right"] {
        for end in [r#""end":"blind","depth":20"#, r#""end":"blind","depth":20,"flip":true"#, r#""end":"mid","depth":20"#] {
            let sk = sketch("spline").replace(r#""id":"front""#, &format!(r#""id":"{plane}""#));
            let d0 = format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{{"type":"extrude","op":"add","sketch":"F1",{end},"id":"F2"}}]}}"#);
            let (b0, e0) = regen_doc(&d0);
            let b0 = b0.unwrap();
            assert!(e0.is_empty(), "{plane} {end}: {e0:?}");
            let mut removed_sum = 0.0;
            let mut want_sum = 0.0;
            let caps: Vec<String> = b0.face_ids().map(|f| b0.face_id_str(f)).filter(|n| n.starts_with("F2.C")).collect();
            assert_eq!(caps.len(), 2);
            let mut edges = vec![];
            for c in &caps {
                edges.extend(picks(&b0, c, false));
                want_sum += removed(false, 2.0, rim_length(&b0, c));
            }
            let d1 = format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{{"type":"extrude","op":"add","sketch":"F1",{end},"id":"F2"}},{{"id":"F3","type":"fillet","radius":2,"edges":[{}]}}]}}"#, edges.join(","));
            let (b1, e1) = regen_doc(&d1);
            assert!(e1.is_empty(), "{plane} {end}: {e1:?}");
            let b1 = b1.unwrap();
            assert!(ck_check::validate(&b1, 1e-6).is_empty(), "{plane} {end}: invalid");
            removed_sum += vol(&b0) - vol(&b1);
            assert!((removed_sum - want_sum).abs() <= 2e-3 * want_sum, "{plane} {end}: removed {removed_sum}, expected {want_sum}");
        }
    }
}

/// A spline hole in a plate: the ring is an inner loop of the cap, the round grows the hole's mouth (the offset goes away from the
/// hole): removed volume `(1 - pi/4) r^2 (L + 2 pi xr)`.
#[test]
fn spline_hole_in_a_plate() {
    let ps: Vec<String> = PTS.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ids: Vec<String> = (0..PTS.len()).map(|i| format!(r#""p{i}""#)).collect();
    let sk = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":-50,"y":-40}},{{"id":"b","x":50,"y":-40}},{{"id":"c","x":50,"y":40}},{{"id":"d","x":-50,"y":40}},{}],"entities":[{{"id":"l1","type":"line","p1":"a","p2":"b"}},{{"id":"l2","type":"line","p1":"b","p2":"c"}},{{"id":"l3","type":"line","p1":"c","p2":"d"}},{{"id":"l4","type":"line","p1":"d","p2":"a"}},{{"id":"e1","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#,
        ps.join(","),
        ids.join(",")
    );
    let base = format!(r#"{{"format":"ck-part","version":1,"features":[{sk},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":20,"id":"F2"}}]}}"#);
    let (b0, e0) = regen_doc(&base);
    let b0 = b0.unwrap();
    assert!(e0.is_empty(), "{e0:?}");
    // the rim edges of the cap that border the hole's wall
    let cap = b0.find_face("F2.C1").unwrap();
    let mut edges = vec![];
    let mut len = 0.0;
    for e in b0.edge_ids() {
        let fs = b0.edge_faces(e);
        if fs.len() == 2 && fs.contains(&cap) && fs.iter().any(|&f| b0.face_id_str(f).contains(".e1")) {
            let p = b0.vertex(b0.edge(e).v0).p;
            edges.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b0.face_id_str(fs[0]), b0.face_id_str(fs[1]), p.x, p.y, p.z));
            len += b0.edge_length(e);
        }
    }
    assert!(!edges.is_empty());
    let r = 2.0;
    let d1 = format!(r#"{},{{"id":"F3","type":"fillet","radius":{r},"edges":[{}]}}]}}"#, &base[..base.len() - 2], edges[..1].join(","));
    let (b1, e1) = regen_doc(&d1);
    assert!(e1.is_empty(), "{e1:?}");
    let b1 = b1.unwrap();
    assert!(ck_check::validate(&b1, 1e-6).is_empty());
    let xr = r * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
    let want = (1.0 - PI / 4.0) * r * r * (len + 2.0 * PI * xr);
    let got = vol(&b0) - vol(&b1);
    assert!((got - want).abs() <= 2e-3 * want, "removed {got}, expected {want}");
}
