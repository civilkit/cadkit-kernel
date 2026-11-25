//! Partial-edge fillets and chamfers (`startOffset`, `endOffset`) and the explicit corner `setback` of a fillet, through the
//! feature wire. Every volume is a closed form worked out by hand:
//!
//! * block 40 x 10 x 20 (`F2`), top-front edge `F2.C1 | F2.S.e1`, 40 long: a round of radius r removes (1 - pi/4) r^2 per length,
//!   a chamfer of leg d removes d^2/2 per length, so the part left by a blend that runs `L - start - end` along the edge is
//!   `8000 - (L - start - end) * section`;
//! * a convex corner of three equal rounds with the rolling-ball sphere patch removes r^3 (1 - pi/6) in the cube of side r at
//!   the vertex (the cube less the octant of the ball, pi r^3 / 6) plus (1 - pi/4) r^2 per length of each edge beyond the cube.
//!   A setback of at most r (the natural setback) is that sphere corner.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::f64::consts::PI;

const SQ: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":W,"y":0},{"id":"p3","x":W,"y":D},{"id":"p4","x":0,"y":D}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#;

fn block(w: f64, d: f64, h: f64) -> Vec<String> {
    vec![
        SQ.replace("W", &w.to_string()).replace("D", &d.to_string()),
        format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#),
    ]
}

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
    warnings: Vec<String>,
    err_params: Vec<(String, String)>,
}

fn run(feats: Vec<String>) -> Out {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let f = |v: &Vec<regen::RegenError>| v.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    let err_params = r.errors.iter().map(|e| (e.error.code.to_string(), format!("{:?}", e.error.params))).collect();
    Out { body: br.body, errors: f(&r.errors), warnings: f(&r.warnings), err_params }
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn top_front(extra: &str, kind: &str, size: &str, at_x: f64) -> Vec<String> {
    let mut f = block(40.0, 10.0, 20.0);
    f.push(format!(r#"{{"type":"{kind}","id":"F3",{size},{extra}"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[{at_x},0,20]}}]}}"#));
    f
}

fn valid_volume(o: &Out, want: f64) {
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
    let v = vol(b);
    assert!((v - want).abs() <= 1e-9 * want, "volume {v} vs closed form {want}");
}

#[test]
fn partial_fillet_volume_is_the_active_length_times_the_section() {
    let sec = 4.0 - PI; // r = 2: (1 - pi/4) r^2
    for (extra, active) in [(r#""startOffset":5,"endOffset":10,"#, 25.0), (r#""startOffset":10,"#, 30.0), (r#""endOffset":7.5,"#, 32.5)] {
        let o = run(top_front(extra, "fillet", r#""radius":2"#, 20.0));
        valid_volume(&o, 8000.0 - active * sec);
        assert!(o.warnings.is_empty(), "{:?}", o.warnings);
    }
}

#[test]
fn partial_chamfer_volume_is_the_active_length_times_the_section() {
    for (extra, active) in [(r#""startOffset":5,"endOffset":15,"#, 20.0), (r#""endOffset":4,"#, 36.0)] {
        let o = run(top_front(extra, "chamfer", r#""chamferType":"equal","distance":2"#, 20.0));
        valid_volume(&o, 8000.0 - active * 2.0);
    }
}

/// `startOffset` counts from the end of the edge nearer the pick, `endOffset` from the other end (as `radii: [near, far]`),
/// so the same feature means the same thing whichever way the edge happens to run in the model.
#[test]
fn start_is_the_end_nearer_the_pick() {
    // blend face B0 spans x in [start, 40] when the pick is near x = 0 and [0, 40 - start] when it is near x = 40
    for (kind, size) in [("fillet", r#""radius":2"#), ("chamfer", r#""chamferType":"equal","distance":2"#)] {
        let near_zero = run(top_front(r#""startOffset":6,"#, kind, size, 3.0));
        let near_forty = run(top_front(r#""startOffset":6,"#, kind, size, 37.0));
        for (o, lo, hi) in [(&near_zero, 6.0, 40.0), (&near_forty, 0.0, 34.0)] {
            assert!(o.errors.is_empty(), "{:?}", o.errors);
            let b = o.body.as_ref().unwrap();
            let f = b.find_face("F3.B0").expect("blend face F3.B0");
            let (a, z) = b.bbox_of_faces(&[f]);
            assert!((a.x - lo).abs() < 1e-6 && (z.x - hi).abs() < 1e-6, "{kind}: B0 x range {} .. {} vs {lo} .. {hi}", a.x, z.x);
        }
        assert!((vol(near_zero.body.as_ref().unwrap()) - vol(near_forty.body.as_ref().unwrap())).abs() < 1e-9);
    }
}

#[test]
fn offsets_may_be_expressions() {
    let o = run(top_front(r#""startOffset":"=2+3","endOffset":"=10","#, "fillet", r#""radius":2"#, 20.0));
    valid_volume(&o, 8000.0 - 25.0 * (4.0 - PI));
}

#[test]
fn bad_offsets_are_refused_with_the_field_and_a_hint() {
    let cases = [
        (r#""startOffset":-1,"#, "startOffset"),
        (r#""endOffset":-0.5,"#, "endOffset"),
        (r#""startOffset":25,"endOffset":15,"#, "endOffset"), // together they use up the whole 40 mm edge
        (r#""startOffset":40,"#, "startOffset"),
    ];
    for (extra, field) in cases {
        for (kind, size) in [("fillet", r#""radius":2"#), ("chamfer", r#""chamferType":"equal","distance":2"#)] {
            let o = run(top_front(extra, kind, size, 20.0));
            assert_eq!(o.err_params.len(), 1, "{kind} {extra}: {:?} {:?}", o.errors, o.warnings);
            let (code, params) = &o.err_params[0];
            assert_eq!(code, "invalidValue", "{kind} {extra}: {params}");
            assert!(params.contains(&format!("\"field\", Str(\"{field}\")")) || params.contains(field), "{kind} {extra}: {params}");
            assert!(params.contains("hint"), "{kind} {extra}: {params}");
            assert!(params.contains("detail"), "{kind} {extra}: {params}");
        }
    }
}

#[test]
fn offsets_with_propagate_are_refused() {
    let o = run(top_front(r#""startOffset":5,"propagate":true,"#, "fillet", r#""radius":2"#, 20.0));
    assert!(o.errors.iter().any(|e| e.contains("invalidValue") && e.contains("propagate")), "{:?}", o.errors);
}

// ---- the corner setback of a fillet

fn corner(setback: Option<&str>, r: f64) -> Vec<String> {
    let sb = setback.map(|s| format!(r#""setback":{s},"#)).unwrap_or_default();
    let mut f = block(40.0, 10.0, 20.0);
    // the vertex (40, 10, 20): its three edges are the top-back (x 0..40), the top-right (y 0..10) and the right-back vertical (z 0..20)
    f.push(format!(
        r#"{{"type":"fillet","id":"F3","radius":{r},{sb}"edges":[{{"faces":["F2.C1","F2.S.e3"],"at":[20,10,20]}},{{"faces":["F2.C1","F2.S.e2"],"at":[40,5,20]}},{{"faces":["F2.S.e2","F2.S.e3"],"at":[40,10,10]}}]}}"#
    ));
    f
}

fn sphere_corner_volume(r: f64) -> f64 {
    let removed = r * r * r * (1.0 - PI / 6.0) + ((40.0 - r) + (10.0 - r) + (20.0 - r)) * (1.0 - PI / 4.0) * r * r;
    8000.0 - removed + 0.0 * 1.0 // block 40 x 10 x 20 = 8000
}

#[test]
fn a_setback_at_the_natural_distance_is_the_rolling_ball_corner() {
    let r = 2.0;
    let default = run(corner(None, r));
    valid_volume(&default, sphere_corner_volume(r));
    // the natural setback of three equal rounds on a right-angle corner is r: asking for it, or less, changes nothing
    let at_r = run(corner(Some("2"), r));
    valid_volume(&at_r, sphere_corner_volume(r));
    assert!(at_r.warnings.is_empty(), "{:?}", at_r.warnings);
}

#[test]
fn a_smaller_setback_than_the_natural_one_is_raised_with_a_warning() {
    let o = run(corner(Some("0.5"), 2.0));
    valid_volume(&o, sphere_corner_volume(2.0));
    assert!(o.warnings.iter().any(|w| w.contains("blendSetbackAdjusted") && w.contains("applied")), "{:?}", o.warnings);
}

#[test]
fn a_larger_setback_builds_a_valid_patch_and_takes_more_material_at_the_corner() {
    let r = 2.0;
    let sphere = sphere_corner_volume(r);
    let mut prev = sphere;
    let mut area_prev = 0.0;
    for s in [3.0, 4.0, 5.0] {
        let o = run(corner(Some(&s.to_string()), r));
        assert!(o.errors.is_empty(), "setback {s}: {:?}", o.errors);
        let b = o.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "setback {s}");
        let v = vol(b);
        // bounds that are closed forms: the three rounds run from the setback plane out (so they remove
        // (1 - pi/4) r^2 per length of each edge beyond s), and everything the corner takes lies in the cube of side s
        let strips = ((40.0 - s) + (10.0 - s) + (20.0 - s)) * (1.0 - PI / 4.0) * r * r;
        let lower = 8000.0 - strips - s * s * s;
        let upper = 8000.0 - strips;
        assert!(v > lower && v < upper, "setback {s}: volume {v} outside the closed-form bracket ({lower}, {upper})");
        // the patch is the face V0 and it grows with the setback
        let v0 = b.find_face("F3.V0").expect("corner patch F3.V0");
        let area = ck_topo::mass::faces_moments(b, &[v0], ck_geom::Vec3::ZERO).area.abs();
        assert!(area > area_prev, "patch area {area} should grow with the setback ({area_prev})");
        area_prev = area;
        assert!(v < prev + 1e-9 || s == 3.0, "volume should not grow with the setback: {v} vs {prev}");
        prev = v;
    }
}

#[test]
fn a_setback_beyond_the_shortest_edge_is_moved_to_the_limit_with_a_warning() {
    let o = run(corner(Some("9"), 2.0)); // the 10 mm edge allows 0.45 * 10 = 4.5
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let w = o.warnings.iter().find(|w| w.contains("blendSetbackAdjusted")).unwrap_or_else(|| panic!("{:?}", o.warnings));
    assert!(w.contains("4.5") && w.contains("requested"), "{w}");
    // the same part as asking for 4.5
    let at_max = run(corner(Some("4.5"), 2.0));
    assert!((vol(o.body.as_ref().unwrap()) - vol(at_max.body.as_ref().unwrap())).abs() < 1e-9);
}

#[test]
fn a_setback_with_no_corner_to_apply_to_warns() {
    let mut f = block(40.0, 10.0, 20.0);
    f.push(r#"{"type":"fillet","id":"F3","radius":2,"setback":3,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[20,0,20]}]}"#.to_string());
    let o = run(f);
    valid_volume(&o, 8000.0 - 40.0 * (4.0 - PI));
    assert!(o.warnings.iter().any(|w| w.contains("blendSetbackUnused")), "{:?}", o.warnings);
}
