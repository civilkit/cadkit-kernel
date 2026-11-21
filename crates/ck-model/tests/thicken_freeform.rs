//! Thicken after Freeform: a planar surface rebuilt as a NURBS skin by a Freeform point move, then thickened.
#[path = "robust_common/mod.rs"]
mod common;

fn sketch(circle: bool) -> String {
    if circle {
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":20}],"constraints":[]}"#.into()
    } else {
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
          "points":[{"id":"p1","x":-20,"y":-20},{"id":"p2","x":20,"y":-20},{"id":"p3","x":20,"y":20},{"id":"p4","x":-20,"y":20}],
          "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},
                      {"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#.into()
    }
}

fn edit(i: usize, j: usize, dz: f64) -> String {
    format!(r#"{{"points":[[{i},{j}]],"of":[5,5],"frame":"world","dx":0,"dy":0,"dz":{dz},"proportional":1}}"#)
}

fn doc(circle: bool, edits: &[String], thicken: &str) -> String {
    let mut feats = vec![sketch(circle), r#"{"type":"surfacePlanar","id":"F2","sketch":"F1"}"#.to_string()];
    let mut n = 3;
    for e in edits {
        feats.push(format!(r#"{{"type":"freeform","id":"F{n}","faces":["F2.P"],"edits":[{e}]}}"#));
        n += 1;
    }
    feats.push(format!(r#"{{"type":"thicken","id":"F{n}",{thicken}}}"#));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","))
}

fn run(src: &str) -> Vec<(String, String, String)> {
    common::regen_codes(src)
}

fn area(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).area
}

/// Run `f` on a worker thread with a time limit (a hang fails the one test, not the whole file).
fn bounded<T: Send + 'static>(label: &str, secs: u64, f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::Builder::new().stack_size(64 << 20).spawn(move || drop(tx.send(f()))).unwrap();
    rx.recv_timeout(std::time::Duration::from_secs(secs)).unwrap_or_else(|_| panic!("{label}: no result within {secs}s"))
}

/// Thicken after the given freeform edits: no error, a valid body, volume about skin area x thickness.
fn check(label: &str, circle: bool, edits: &[String], thicken: &str, t: f64) {
    let src = doc(circle, edits, thicken);
    let t0 = std::time::Instant::now();
    let s2 = src.clone();
    let errs = bounded(label, 400, move || run(&s2));
    eprintln!("{label}: regen {:.1}s errors {errs:?}", t0.elapsed().as_secs_f64());
    assert!(errs.is_empty(), "{label}: {errs:?}");
    let (issues, vol) = bounded(label, 400, move || common::regen_validate(&src));
    eprintln!("{label}: total {:.1}s issues {issues:?} volume {vol:?}", t0.elapsed().as_secs_f64());
    assert!(issues.is_empty(), "{label}: {issues:?}");
    let flat = if circle { std::f64::consts::PI * 400.0 } else { 1600.0 };
    let v = vol.unwrap();
    let want = flat * t;
    // the bowed skin is larger than the flat face by up to about 40 % (60 % with two edits)
    assert!(v > 0.95 * want && v < 1.6 * want, "{label}: volume {v} vs flat {want}");
}

macro_rules! case {
    ($name:ident, $circle:expr, $edits:expr, $fields:expr, $t:expr) => {
        #[test]
        fn $name() {
            check(stringify!($name), $circle, &$edits, $fields, $t);
        }
    };
}

case!(rect_ref_face, false, [edit(2, 2, 20.0)], r#""surface":"F2.P","thickness":2,"side":"front""#, 2.0);
case!(rect_ref_feature, false, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"front""#, 2.0);
case!(rect_ref_freeform, false, [edit(2, 2, 20.0)], r#""surface":"F3","thickness":2,"side":"front""#, 2.0);
case!(rect_back, false, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"back""#, 2.0);
case!(rect_both, false, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"both""#, 2.0);
case!(rect_two_freeforms, false, [edit(2, 2, 20.0), edit(1, 2, 6.0)], r#""surface":"F2","thickness":2,"side":"front""#, 2.0);
case!(circle_ref_face, true, [edit(2, 2, 20.0)], r#""surface":"F2.P","thickness":2,"side":"front""#, 2.0);
case!(circle_ref_feature, true, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"front""#, 2.0);
case!(circle_back, true, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"back""#, 2.0);
case!(circle_both, true, [edit(2, 2, 20.0)], r#""surface":"F2","thickness":2,"side":"both""#, 2.0);
case!(circle_two_freeforms, true, [edit(2, 2, 20.0), edit(1, 2, 6.0)], r#""surface":"F2","thickness":2,"side":"front""#, 2.0);

/// A negative thickness is refused with a plain invalidValue (the oracle's rule), never a missing reference.
#[test]
fn negative_thickness_is_refused() {
    for circle in [false, true] {
        let src = doc(circle, &[edit(2, 2, 20.0)], r#""surface":"F2","thickness":-2,"side":"front""#);
        let errs = run(&src);
        assert_eq!(errs.len(), 1, "{errs:?}");
        assert_eq!(errs[0].1, "invalidValue", "{errs:?}");
    }
}

#[test]
fn a_skin_that_cannot_be_thickened_says_why() {
    // thicker than the tightest curvature radius of the bow: refused with a reason, never "no longer exists"
    let src = doc(false, &[edit(2, 2, 20.0)], r#""surface":"F2","thickness":500,"side":"front""#);
    for (_, code, msg) in bounded("thick 500", 120, move || run(&src)) {
        assert_ne!(code, "missingReference", "{msg}");
    }
}

/// Mirror of surface bodies: a planar surface, a surface extrude and a freeform skin each give a second surface body.
#[test]
fn mirror_of_surface_bodies() {
    let mir = |seed: &str| format!(r#"{{"type":"mirror","id":"M1","seeds":["{seed}"],"plane":{{"kind":"std","id":"right"}}}}"#);
    let ext = r#"{"type":"surfaceExtrude","id":"F2","sketch":"F1","end":"blind","depth":10}"#;
    let planar = r#"{"type":"surfacePlanar","id":"F2","sketch":"F1"}"#;
    let ff = r#"{"type":"freeform","id":"F3","faces":["F2.P"],"edits":[{"points":[[2,2]],"of":[5,5],"frame":"world","dx":0,"dy":0,"dz":20,"proportional":1}]}"#;
    // the circle sits at x = 30 so the image is a separate body on the other side of the right plane
    let sk = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":30,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":10}],"constraints":[]}"#;
    for (label, mid, seed) in [("planar", vec![planar], "F2"), ("extrude", vec![ext], "F2"), ("freeform", vec![planar, ff], "F3"), ("freeform by face", vec![planar, ff], "F2.P")] {
        let mut feats = vec![sk.to_string()];
        feats.extend(mid.iter().map(|s| s.to_string()));
        feats.push(mir(seed));
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
        let errs = run(&src);
        assert!(errs.is_empty(), "{label}: {errs:?}");
        let surfs = common::regen_surfs(&src);
        assert_eq!(surfs.len(), 2, "{label}: {:?}", surfs.iter().map(|s| &s.0).collect::<Vec<_>>());
        let (a, b) = (area(&surfs[0].1), area(&surfs[1].1));
        assert!((a - b).abs() < 1e-6 * a, "{label}: areas {a} {b}");
        let (lo0, hi0) = surfs[0].1.bbox();
        let (lo1, hi1) = surfs[1].1.bbox();
        assert!((lo0.x + hi1.x).abs() < 1e-6 && (hi0.x + lo1.x).abs() < 1e-6, "{label}: not a reflection in x: {lo0:?} {hi0:?} {lo1:?} {hi1:?}");
    }
}

/// The part as the app wrote it (circle on the front plane, planar surface, one freeform point move, thicken of the default surface F2).
#[test]
fn app_repro_fixture() {
    let src = include_str!("fixtures/thicken_freeform_app.json");
    assert_eq!(run(src), vec![]);
    let (issues, vol) = common::regen_validate(src);
    assert!(issues.is_empty(), "{issues:?}");
    assert!(vol.unwrap() > 2500.0);
}
