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


// The bottle neck: four profiles (rounded rectangles 63x76 R7 / 76x95 R12, circles r20 / r15) lofted in one smooth feature
// overshot the profiles' boxes by 9..15 mm and tore a section, with a clean validate().

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

fn half(which: &[&str]) -> (f64, f64) {
    let mut h = (0.0f64, 0.0f64);
    for w in which {
        let (x, y) = match *w {
            "F1" => (31.5, 38.0),
            "F5" => (38.0, 47.5),
            "F6" => (20.0, 20.0),
            _ => (15.0, 15.0),
        };
        h = (h.0.max(x), h.1.max(y));
    }
    h
}

/// The loft's mesh box (the wire `box`) and volume.
fn measure(o: &Out) -> ((f64, f64, f64), f64) {
    let b = o.brep.body.as_ref().expect("a body");
    let (lo, hi) = ck_tess::mesh::tessellate(b).mesh.bbox().expect("a mesh");
    let ext = (lo.x.abs().max(hi.x.abs()), lo.y.abs().max(hi.y.abs()), lo.z.abs().max(hi.z.abs()));
    (ext, ck_topo::mass::mass_props(b).volume)
}

/// Every prefix of the bottle's profile list, with the given `railSmoothing`: the solid stays inside the union of the profile boxes.
fn check_boxes(rails: &str) {
    for which in [vec!["F6", "F7"], vec!["F5", "F6"], vec!["F1", "F5"], vec!["F5", "F6", "F7"], vec!["F1", "F5", "F6"], vec!["F1", "F5", "F6", "F7"]] {
        let o = run_part(&bottle(&which, &format!(r#","railSmoothing":"{rails}""#)));
        assert!(o.errors.is_empty(), "{which:?}: {:?}", o.errors);
        let (ext, vol) = measure(&o);
        let (hx, hy) = half(&which);
        eprintln!("BOTTLE {rails} {which:?}: ext {ext:?} vol {vol:.1}");
        // the loft runs along world z; x and y are the sketch's axes
        let tol = 0.5;
        assert!(ext.0 <= hx + tol && ext.1 <= hy + tol, "{rails} {which:?}: overshoot, lateral half extents ({}, {}) vs profiles ({hx}, {hy})", ext.0, ext.1);
        assert_valid_solid("bottle", &o);
        assert!(vol > 0.0);
        assert!(o.warnings.iter().all(|w| w.error.code != "loftOvershoot"), "{rails} {which:?}: {:?}", o.warnings);
    }
}

#[test]
fn bottle_profiles_loft_stays_inside_the_profile_boxes_with_auto_rails() {
    check_boxes("auto");
}

#[test]
fn bottle_profiles_loft_stays_inside_the_profile_boxes_with_local_rails() {
    check_boxes("local");
}

/// The default rails are the natural spline of the oracle probes (pinned elsewhere): they still bulge 9 to 12 mm past the widest
/// middle profile, but never silently: the feature carries a `loftOvershoot` warning with the distance and what to change.
#[test]
fn bottle_default_rails_overshoot_is_warned() {
    for which in [vec!["F1", "F5", "F6"], vec!["F1", "F5", "F6", "F7"]] {
        let o = run_part(&bottle(&which, ""));
        assert!(o.errors.is_empty(), "{:?}", o.errors);
        let (ext, _) = measure(&o);
        assert!(ext.1 > 47.5 + 5.0 && ext.0 > 38.0 + 5.0, "{which:?}: the natural rails are meant to bulge here: {ext:?}");
        let w: Vec<_> = o.warnings.iter().filter(|w| w.error.code == "loftOvershoot").collect();
        assert_eq!(w.len(), 1, "{which:?}: {:?}", o.warnings);
        let over = w[0].error.params.iter().find(|(k, _)| k == "overshoot").and_then(|(_, v)| v.as_f64()).unwrap();
        assert!((over - (ext.1 - 47.5).max(ext.0 - 38.0)).abs() < 0.5, "reported {over}, box excess {ext:?}");
        assert!(w[0].error.params.iter().any(|(k, _)| k == "hint"));
    }
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

/// The neck of the four-profile bottle is a round section, not a blob: the rect section no longer bleeds into it.
#[test]
fn bottle_neck_sections_are_round() {
    let o = run_part(&bottle(&["F1", "F5", "F6", "F7"], r#","railSmoothing":"auto""#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    for z in [175.0, 203.6, 230.0] {
        let (lo, hi, n) = ring(&o, z);
        let expect = 20.0 + (15.0 - 20.0) * (z - 152.0) / (241.0 - 152.0);
        assert!(n > 100 && (lo - expect).abs() < 2.0 && (hi - expect).abs() < 2.0, "z {z}: radius {lo}..{hi}, a frustum has {expect}");
    }
    assert!(o.warnings.iter().all(|w| w.error.code != "loftOvershoot"), "{:?}", o.warnings);
}

/// The window's own connectors from the repro (fractions of each profile's length), and 0 / null entries: all parse, all stay inside
/// the section boxes (`[null,0.25,null,0.5]`, which twists the circles half a turn against the rectangle, is in `loft_guides_bottle`).
/// `null` is automatic; `0` is the fraction 0 (a fixed start point), as in the connector field of the editor.
#[test]
fn bottle_with_connectors_stays_inside_the_boxes() {
    let all = ["F1", "F5", "F6", "F7"];
    for conn in ["[0.842,0.831,0.5,0.5]", "[0,0,0,0]", "[null,null,null,null]", "[0.95,0.95]"] {
        let o = run_part(&bottle(&all, &format!(r#","connectors":{conn},"railSmoothing":"auto""#)));
        assert!(o.errors.is_empty(), "{conn}: {:?}", o.errors);
        let (ext, _) = measure(&o);
        assert!(ext.0 <= 38.5 && ext.1 <= 48.0, "{conn}: {ext:?}");
        assert_valid_solid(conn, &o);
    }
}

/// A loft that the local rails cannot fix (end tension keeps the natural rails) still never overshoots silently: a `loftOvershoot`
/// warning names how far. The loft itself builds.
#[test]
fn overshoot_that_stays_is_warned() {
    let o = run_part(&bottle(&["F1", "F5", "F6"], r#","startNormal":8"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let (ext, _) = measure(&o);
    let w: Vec<_> = o.warnings.iter().filter(|w| w.error.code == "loftOvershoot").collect();
    eprintln!("TENSION ext {ext:?} warnings {}", w.len());
    assert!(ext.1 > 47.5 + 3.0, "this case is meant to overshoot: {ext:?}");
    assert_eq!(w.len(), 1, "{:?}", o.warnings);
    assert!(w[0].error.params.iter().any(|(k, _)| k == "overshoot") && w[0].error.params.iter().any(|(k, _)| k == "hint"));
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}
