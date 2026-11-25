use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
    pub warnings: Vec<regen::RegenError>,
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors, warnings: r.warnings }
}


// Fusion-style loft end conditions (`conditions` on the loft feature): directional angle / weight on the first and last profile.

fn rrect(id: &str, plane: &str, hx: f64, hy: f64, r: f64) -> String {
    let (x, y) = (hx - r, hy - r);
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p5","x":-{x},"y":{hy}}},{{"id":"p6","x":-{hx},"y":{y}}},{{"id":"p7","x":-{x},"y":{y}}},{{"id":"p8","x":{hx},"y":{y}}},{{"id":"p9","x":{x},"y":{hy}}},{{"id":"p10","x":{x},"y":{y}}},{{"id":"p11","x":{x},"y":-{hy}}},{{"id":"p12","x":{hx},"y":-{y}}},{{"id":"p13","x":{x},"y":-{y}}},{{"id":"p14","x":-{x},"y":-{hy}}},{{"id":"p15","x":-{hx},"y":-{y}}},{{"id":"p16","x":-{x},"y":-{y}}}],"entities":[{{"id":"e1","type":"line","p1":"p14","p2":"p11"}},{{"id":"e2","type":"line","p1":"p12","p2":"p8"}},{{"id":"e3","type":"line","p1":"p9","p2":"p5"}},{{"id":"e4","type":"line","p1":"p6","p2":"p15"}},{{"type":"arc","c":"p7","p1":"p5","p2":"p6","id":"e5"}},{{"type":"arc","c":"p10","p1":"p8","p2":"p9","id":"e6"}},{{"type":"arc","c":"p13","p1":"p11","p2":"p12","id":"e7"}},{{"type":"arc","c":"p16","p1":"p15","p2":"p14","id":"e8"}}],"constraints":[]}}"#
    )
}

fn circ(id: &str, plane: &str, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"O","x":0,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"O","r":{r}}}],"constraints":[]}}"#)
}

/// `which`: the profile ids lofted (F1 base rrect, F5 rrect at 114, F6 circle at 152, F7 circle at 241); `extra`: more loft fields.
fn bottle(which: &[&str], extra: &str) -> String {
    let mut f = vec![];
    for (id, z) in [("F2", 114.0), ("F3", 152.0), ("F4", 241.0)] {
        f.push(format!(r#"{{"type":"plane","id":"{id}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{z}}}"#));
    }
    let front = r#"{"kind":"std","id":"front"}"#;
    f.push(rrect("F1", front, 31.5, 38.0, 7.0));
    f.push(rrect("F5", r#"{"kind":"feature","id":"F2"}"#, 38.0, 47.5, 12.0));
    f.push(circ("F6", r#"{"kind":"feature","id":"F3"}"#, 20.0));
    f.push(circ("F7", r#"{"kind":"feature","id":"F4"}"#, 15.0));
    let sk: Vec<String> = which.iter().map(|s| format!("\"{s}\"")).collect();
    f.push(format!(r#"{{"type":"loft","id":"F8","op":"add","sketches":[{}]{extra}}}"#, sk.join(",")));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

/// Mesh section at height `z`: (min, max) distance of the cut points from the z axis, and the cut point count.
fn ring(o: &Out, z: f64) -> (f64, f64, usize) {
    let b = o.brep.body.as_ref().unwrap();
    let m = ck_tess::mesh::tessellate(b).mesh;
    let p = |i: usize| {
        let k = 3 * i;
        (m.positions[k] as f64, m.positions[k + 1] as f64, m.positions[k + 2] as f64)
    };
    let (mut lo, mut hi, mut n) = (f64::INFINITY, 0.0f64, 0);
    for t in (0..m.positions.len() / 3).collect::<Vec<_>>().chunks(3) {
        for (a, c) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (pa, pc) = (p(a), p(c));
            if (pa.2 - z) * (pc.2 - z) < 0.0 {
                let s = (z - pa.2) / (pc.2 - pa.2);
                let (x, y) = (pa.0 + s * (pc.0 - pa.0), pa.1 + s * (pc.1 - pa.1));
                let r = (x * x + y * y).sqrt();
                lo = lo.min(r);
                hi = hi.max(r);
                n += 1;
            }
        }
    }
    (lo, hi, n)
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}

fn square(id: &str, plane: &str, h: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"a","x":-{h},"y":-{h}}},{{"id":"b","x":{h},"y":-{h}}},{{"id":"c","x":{h},"y":{h}}},{{"id":"d","x":-{h},"y":{h}}}],"entities":[{{"id":"e1","type":"line","p1":"a","p2":"b"}},{{"id":"e2","type":"line","p1":"b","p2":"c"}},{{"id":"e3","type":"line","p1":"c","p2":"d"}},{{"id":"e4","type":"line","p1":"d","p2":"a"}}],"constraints":[]}}"#
    )
}

/// Circles of the radii `rs` at z = 0, 30, 60 ...; `extra`: more loft fields. Profile ids S0, S1, ...
fn stack(rs: &[f64], extra: &str) -> String {
    let mut f = vec![];
    let front = r#"{"kind":"std","id":"front"}"#;
    f.push(circ("S0", front, rs[0]));
    let mut ids = vec!["\"S0\"".to_string()];
    for (i, r) in rs.iter().enumerate().skip(1) {
        f.push(format!(r#"{{"type":"plane","id":"P{i}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{}}}"#, 30.0 * i as f64));
        f.push(circ(&format!("S{i}"), &format!(r#"{{"kind":"feature","id":"P{i}"}}"#), *r));
        ids.push(format!("\"S{i}\""));
    }
    f.push(format!(r#"{{"type":"loft","id":"L","op":"add","sketches":[{}]{extra}}}"#, ids.join(",")));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

fn two(r0: f64, r1: f64, extra: &str) -> Out {
    run_part(&stack(&[r0, r1], extra))
}

fn vol(o: &Out) -> f64 {
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    ck_topo::mass::mass_props(o.brep.body.as_ref().expect("a body")).volume
}

fn code(o: &Out) -> String {
    o.errors.first().map(|e| e.error.code.to_string()).unwrap_or_default()
}

fn cond(a: &str, b: &str) -> String {
    format!(r#","conditions":[{a},{b}]"#)
}

fn dir(angle: f64, weight: f64) -> String {
    format!(r#"{{"type":"directional","angle":{angle},"weight":{weight}}}"#)
}

#[test]
fn directional_90_weight_1_is_the_legacy_normal() {
    let legacy = vol(&two(10.0, 20.0, r#","startNormal":1,"endNormal":1"#));
    let c = vol(&two(10.0, 20.0, &cond(&dir(90.0, 1.0), &dir(90.0, 1.0))));
    assert!((legacy - c).abs() < 1e-6 * legacy, "{legacy} vs {c}");
    // one end only, with a tension
    let legacy = vol(&two(10.0, 20.0, r#","startNormal":2"#));
    let c = vol(&two(10.0, 20.0, &cond(&dir(90.0, 2.0), "null")));
    assert!((legacy - c).abs() < 1e-6 * legacy, "{legacy} vs {c}");
    // the angle defaults to 90 and the weight to 1
    let d = vol(&two(10.0, 20.0, &cond(r#"{"type":"directional"}"#, r#"{"type":"directional"}"#)));
    let l = vol(&two(10.0, 20.0, r#","startNormal":true,"endNormal":true"#));
    assert!((d - l).abs() < 1e-6 * l, "{d} vs {l}");
}

#[test]
fn conditions_win_over_the_legacy_end_flags() {
    let only = vol(&two(10.0, 20.0, &cond(&dir(90.0, 1.0), "null")));
    let both = vol(&two(10.0, 20.0, &format!(r#","startNormal":3{}"#, cond(&dir(90.0, 1.0), "null"))));
    assert!((only - both).abs() < 1e-9 * only, "{only} vs {both}");
    // a free entry leaves the legacy flag in force
    let legacy = vol(&two(10.0, 20.0, r#","startNormal":3"#));
    let free = vol(&two(10.0, 20.0, &format!(r#","startNormal":3{}"#, cond(r#"{"type":"free"}"#, "null"))));
    assert!((legacy - free).abs() < 1e-9 * legacy, "{legacy} vs {free}");
}

#[test]
fn weights_grow_the_bulge_monotonically() {
    // frustum r 10 to r 20, height 30, perpendicular ends: the heavier the tension, the longer the rails run along the axis, then
    // the wider part is reached late and fast, and the bulge past the straight frustum grows with the weight
    let straight = vol(&two(10.0, 20.0, ""));
    let mut prev = straight;
    for w in [0.5, 1.0, 2.0] {
        let v = vol(&two(10.0, 20.0, &cond(&dir(90.0, w), &dir(90.0, w))));
        eprintln!("WEIGHT {w}: volume {v} (straight {straight})");
        assert!(v > prev + 1.0, "weight {w}: {v} not above {prev}");
        prev = v;
    }
}

#[test]
fn angle_below_90_leans_outward_and_above_90_inward() {
    // a cylinder r 10, height 30: the straight loft is the cylinder; leaning out flares (bulges) it, leaning in necks it
    let cyl = std::f64::consts::PI * 100.0 * 30.0;
    let v = |a: f64| vol(&two(10.0, 10.0, &cond(&dir(a, 1.0), &dir(a, 1.0))));
    let (v60, v75, v90, v105, v120) = (v(60.0), v(75.0), v(90.0), v(105.0), v(120.0));
    eprintln!("LEAN volumes 60 {v60} 75 {v75} 90 {v90} 105 {v105} 120 {v120} cylinder {cyl}");
    assert!((v90 - cyl).abs() < 1e-3 * cyl, "angle 90 is the straight cylinder: {v90}");
    assert!(v60 > v75 && v75 > cyl * 1.001, "below 90 the solid flares out: {v60} {v75}");
    assert!(v120 < v105 && v105 < cyl * 0.999, "above 90 it necks in: {v120} {v105}");
}

#[test]
fn free_conditions_are_no_conditions() {
    let none = vol(&two(10.0, 20.0, ""));
    for c in [cond("null", "null"), cond(r#"{"type":"free"}"#, r#"{"type":"free"}"#), r#","conditions":[]"#.to_string(), r#","conditions":null"#.to_string()] {
        let v = vol(&two(10.0, 20.0, &c));
        assert!((v - none).abs() < 1e-9 * none, "{c}: {v} vs {none}");
    }
}

#[test]
fn tangent_and_curvature_parse_and_are_refused_as_tangent_edges() {
    for t in ["tangent", "curvature"] {
        for c in [format!(r#"{{"type":"{t}"}}"#), format!(r#"{{"type":"{t}","weight":2,"face":{{"kind":"face","id":"x"}}}}"#)] {
            let o = two(10.0, 20.0, &cond(&c, "null"));
            assert_eq!(code(&o), "loftTangentEdges", "{c}: {:?}", o.errors);
        }
    }
}

#[test]
fn an_interior_condition_is_refused() {
    let o = run_part(&stack(&[10.0, 15.0, 10.0], &format!(r#","conditions":[null,{},null]"#, dir(90.0, 1.0))));
    assert_eq!(code(&o), "loftConditionInterior", "{:?}", o.errors);
    let e = &o.errors[0].error;
    assert!(e.params.iter().any(|(k, _)| k == "detail") && e.params.iter().any(|(k, _)| k == "hint"));
    // a free interior entry and end conditions are fine with three profiles
    let o = run_part(&stack(&[10.0, 15.0, 10.0], &format!(r#","conditions":[{},{{"type":"free"}},{}]"#, dir(90.0, 1.0), dir(90.0, 1.0))));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}

#[test]
fn malformed_conditions_are_refused() {
    for (c, why) in [
        (r#","conditions":[null]"#.to_string(), "length"),
        (r#","conditions":[null,null,null]"#.to_string(), "length"),
        (cond(&dir(0.0, 1.0), "null"), "angle 0"),
        (cond(&dir(180.0, 1.0), "null"), "angle 180"),
        (cond(&dir(90.0, 0.0), "null"), "weight 0"),
        (cond(&dir(90.0, -1.0), "null"), "negative weight"),
        (cond(r#"{"type":"bogus"}"#, "null"), "type"),
        (r#","conditions":{"a":1}"#.to_string(), "not an array"),
    ] {
        let o = two(10.0, 20.0, &c);
        assert_eq!(code(&o), "loftConditionInvalid", "{why}: {:?}", o.errors);
        let e = &o.errors[0].error;
        assert!(e.params.iter().any(|(k, _)| k == "detail") && e.params.iter().any(|(k, _)| k == "hint"), "{why}");
    }
}

#[test]
fn a_leaning_end_needs_a_smooth_outline() {
    let front = r#"{"kind":"std","id":"front"}"#;
    let src = |extra: &str| {
        let f = [
            square("S0", front, 10.0),
            r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":30}"#.to_string(),
            square("S1", r#"{"kind":"feature","id":"P1"}"#, 15.0),
            format!(r#"{{"type":"loft","id":"L","op":"add","sketches":["S0","S1"]{extra}}}"#),
        ];
        format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
    };
    let o = run_part(&src(&cond(&dir(60.0, 1.0), "null")));
    assert_eq!(code(&o), "loftConditionCorner", "{:?}", o.errors);
    // perpendicular ends need no outward direction: they build on a square
    let o = run_part(&src(&cond(&dir(90.0, 1.0), &dir(90.0, 1.0))));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}

#[test]
fn a_loop_making_condition_is_reported() {
    let o = two(10.0, 10.0, &cond(&dir(20.0, 6.0), &dir(20.0, 6.0)));
    let warned = o.warnings.iter().any(|w| w.error.code == "loftOvershoot");
    let refused = !o.errors.is_empty();
    assert!(warned || refused, "a steep, heavy lean must be flagged: {:?} {:?}", o.warnings, o.errors);
}

#[test]
fn the_bottle_with_directional_ends_builds_valid() {
    let all = ["F1", "F5", "F6", "F7"];
    let o = run_part(&bottle(&all, &format!(r#","railSmoothing":"auto"{}"#, r#","conditions":[{"type":"directional","angle":90,"weight":1},null,null,{"type":"directional","angle":90,"weight":1}]"#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("bottle conditions", &o);
    assert!(vol(&o) > 0.0);
}

#[test]
fn guides_and_centre_lines_refuse_conditions() {
    // a centre line sketch (a straight vertical line in the right plane) with a condition: refused by name
    let front = r#"{"kind":"std","id":"front"}"#;
    let f = [
        circ("S0", front, 10.0),
        r#"{"type":"plane","id":"P1","mode":"offset","base":{"kind":"std","id":"front"},"offset":30}"#.to_string(),
        circ("S1", r#"{"kind":"feature","id":"P1"}"#, 15.0),
        r#"{"type":"sketch","id":"CL","plane":{"kind":"std","id":"right"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":0,"y":30}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"}],"constraints":[]}"#.to_string(),
        format!(r#"{{"type":"loft","id":"L","op":"add","sketches":["S0","S1"],"centreLine":{{"sketch":"CL"}}{}}}"#, cond(&dir(60.0, 1.0), "null")),
    ];
    let o = run_part(&format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(",")));
    assert_eq!(code(&o), "loftConditionCombo", "{:?}", o.errors);
}
