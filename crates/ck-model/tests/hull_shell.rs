//! Open-top hollow hulls: shell of lofted free-form bodies (spline stations, blunt or point ends,
//! plain cone). The offset of a B-spline surface that collapses to a pole needs the limit normal,
//! and the offset edge curves of the lofted faces need an adaptive fit.
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

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn station_plane(id: &str, x: f64) -> String {
    format!(r#"{{"type":"plane","id":"{id}","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":{x}}}"#)
}

/// A closed 10-point hull section (half-beam `w`, depth `d`, deck height `h`) on station plane `pl`.
fn hull_section(id: &str, pl: &str, w: f64, d: f64, h: f64) -> String {
    let p = [(0.0, h), (0.55 * w, h + 0.4), (w, h - 1.0), (0.97 * w, -0.3 * d), (0.6 * w, -0.8 * d), (0.0, -d), (-0.6 * w, -0.8 * d), (-0.97 * w, -0.3 * d), (-w, h - 1.0), (-0.55 * w, h + 0.4)];
    let pts: Vec<String> = p.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let ids: Vec<String> = (1..=p.len()).map(|i| format!("\"p{i}\"")).collect();
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"feature","id":"{pl}"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{{"id":"e1","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#,
        pts.join(","),
        ids.join(",")
    )
}

fn point_section(id: &str, pl: &str, v: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"feature","id":"{pl}"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{{"id":"p1","x":0,"y":{v}}}],"entities":[{{"id":"e1","type":"point","p":"p1"}}],"constraints":[]}}"#
    )
}

/// Hull from stations `[x, w, d, h]`, optionally ended by a point at `x_tip`; then the deck cut
/// above y = 1.5 (face F92.C0) and a shell of that face.
fn hull(stations: &[[f64; 4]], tip: Option<f64>, thick: Option<f64>) -> String {
    let mut f: Vec<String> = vec![];
    let mut sk: Vec<String> = vec![];
    for (i, s) in stations.iter().enumerate() {
        f.push(station_plane(&format!("P{i}"), s[0]));
        f.push(hull_section(&format!("S{i}"), &format!("P{i}"), s[1], s[2], s[3]));
        sk.push(format!("\"S{i}\""));
    }
    if let Some(x) = tip {
        f.push(station_plane("PT", x));
        f.push(point_section("ST", "PT", 2.0));
        sk.push("\"ST\"".into());
    }
    f.push(format!(r#"{{"type":"loft","id":"F50","op":"add","sketches":[{}]}}"#, sk.join(",")));
    f.push(r#"{"type":"plane","id":"F90","mode":"offset","base":{"kind":"std","id":"top"},"offset":1.5}"#.into());
    f.push(r#"{"type":"sketch","id":"F91","plane":{"kind":"feature","id":"F90"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"a","x":-90,"y":-40},{"id":"b","x":90,"y":-40},{"id":"c","x":90,"y":40},{"id":"d","x":-90,"y":40}],"entities":[{"id":"e1","type":"line","p1":"a","p2":"b"},{"id":"e2","type":"line","p1":"b","p2":"c"},{"id":"e3","type":"line","p1":"c","p2":"d"},{"id":"e4","type":"line","p1":"d","p2":"a"}],"constraints":[]}"#.into());
    f.push(r#"{"type":"extrude","id":"F92","op":"cut","sketch":"F91","end":"blind","depth":30,"flip":true}"#.into());
    if let Some(t) = thick {
        f.push(format!(r#"{{"type":"shell","id":"F93","faces":["F92.C0"],"thickness":{t}}}"#));
    }
    f.join(",")
}

const BOAT: [[f64; 4]; 4] = [[-60.0, 6.0, 5.0, 3.0], [-35.0, 15.0, 9.0, 4.0], [0.0, 18.0, 12.0, 5.0], [30.0, 13.0, 9.0, 5.5]];
const BLUNT: [[f64; 4]; 5] = [[-60.0, 4.0, 4.0, 3.0], [-30.0, 14.0, 9.0, 4.0], [0.0, 18.0, 12.0, 5.0], [30.0, 14.0, 9.0, 4.0], [60.0, 2.0, 3.0, 7.0]];

fn check_shell(name: &str, stations: &[[f64; 4]], tip: Option<f64>, t: f64) -> (f64, f64) {
    let base = run(&hull(stations, tip, None));
    assert!(base.errors.is_empty(), "{name} base: {:?}", base.errors);
    let v0 = vol(&base);
    let o = run(&hull(stations, tip, Some(t)));
    assert!(o.errors.is_empty(), "{name} t={t}: {:?}", o.errors);
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name} t={t}: {iss:?}");
    let v = vol(&o);
    eprintln!("{name} t={t}: open hull {v0:.1} shelled {v:.1}");
    (v0, v)
}

/// Wall volumes of an independent construction (inner offset of the closed hull, cut at the deck
/// line, subtracted from the open hull), made from the same sections: 4383.0 at 1.0, 2683.5 at 0.6.
#[test]
fn blunt_hull_shell() {
    for (t, want) in [(1.0, 4383.0), (0.6, 2683.5)] {
        let (v0, v) = check_shell("blunt", &BLUNT, None, t);
        assert!(v0 > 20000.0);
        assert!((v - want).abs() < 0.04 * want, "t={t}: wall volume {v} against {want}");
    }
}

/// A hull ended by a point section: the shell is refused with a reason and a way out, both when
/// the apex stays in the body and when the deck cut leaves the rim a little way short of it.
#[test]
fn pointy_hull_shell_is_refused_with_a_hint() {
    let o = run(&hull(&BOAT, Some(62.0), Some(1.0)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].error.code, "offsetUnsupported");
    let detail = format!("{:?}", o.errors[0].error.params);
    assert!(detail.contains("apex") && detail.contains("closed section"), "{detail}");
    // the apex kept: shell the stern cap
    let mut f = hull(&BOAT, Some(62.0), None);
    let cut = f.find(r#",{"type":"plane","id":"F90""#).unwrap();
    f.truncate(cut);
    f.push_str(r#",{"type":"shell","id":"F93","faces":["F50.C0"],"thickness":1.0}"#);
    let o = run(&f);
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].error.code, "offsetUnsupported");
}

/// A cone made by lofting a circle to a point: its shell is refused (pole), with the hint.
#[test]
fn cone_loft_shell_is_refused_with_a_hint() {
    let circle = r#"{"type":"sketch","id":"S0","plane":{"kind":"feature","id":"P0"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":8}],"constraints":[]}"#;
    let f = [
        station_plane("P0", 0.0),
        circle.to_string(),
        station_plane("PT", 20.0),
        point_section("ST", "PT", 0.0),
        r#"{"type":"loft","id":"F50","op":"add","sketches":["S0","ST"]}"#.to_string(),
        r#"{"type":"shell","id":"F93","faces":["F50.C0"],"thickness":1.0}"#.to_string(),
    ]
    .join(",");
    let o = run(&f);
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].error.code, "offsetUnsupported");
}
