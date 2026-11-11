//! Sheet metal, stage 1 (`sheetRules`, `baseFlange`, `edgeFlange`, docs/sheet-metal.md). The expected numbers are the
//! flat sizes, bend lines and allowances of black-box runs of a reference sheet metal tool (rule: t = 2, k = 0.44, r = 2,
//! base 100 x 60) and of probes with other thickness, radius, angle, height reference and position; the folded solid is
//! checked against its own closed-form volume (plate + bend sectors + legs), validity and one lump.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

struct Out {
    body: Option<Body>,
    errors: Vec<(String, String)>,
    /// the wire entry of the last sheet feature
    last: Option<ck_json::Json>,
    wire: ck_json::Json,
    frame: Option<ck_core::linalg::Frame>,
}

fn run(features: &[String]) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let wire = r.to_json();
    let last = wire.get_arr("sheetMetal").and_then(|a| a.iter().rev().find(|e| e.get("flatPattern").is_some()).cloned());
    Out {
        body: ex.0.body.clone(),
        errors: r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect(),
        last,
        wire,
        frame: r.sketches.first().map(|s| s.frame),
    }
}

fn rule(t: f64, k: f64, r: f64, extra: &str) -> String {
    format!(r#"{{"type":"sheetRules","id":"R1","name":"R","thickness":{t},"kFactor":{k},"bendRadius":{r}{extra}}}"#)
}

fn sketch(w: f64, h: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":{h}}},{{"id":"p4","x":0,"y":{h}}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

const BASE: &str = r#"{"type":"baseFlange","id":"B1","sketch":"S1"}"#;

/// Edges of the 100 x 60 base by name: a point on the top face (z = 2) in the sketch plane, as world coordinates.
fn pick(o: &Out, name: &str) -> String {
    let f = o.frame.expect("sketch frame");
    let (x, y) = match name {
        "bottom" => (50.0, 0.0),
        "top" => (50.0, 60.0),
        "left" => (0.0, 30.0),
        "right" => (100.0, 30.0),
        _ => panic!(),
    };
    let p = f.o + f.x * x + f.y * y + f.z * 2.0;
    format!("[{},{},{}]", p.x, p.y, p.z)
}

/// Run rule + 100x60 base + flange features; `edges` are names, formatted into the feature with `{}` for the pick list.
fn plate(t: f64, k: f64, r: f64, extra_rule: &str, flanges: &[(&str, &[&str])]) -> Out {
    let pre = run(&[sketch(100.0, 60.0)]);
    let mut fs = vec![rule(t, k, r, extra_rule), sketch(100.0, 60.0), BASE.to_string()];
    for (n, (fields, edges)) in flanges.iter().enumerate() {
        let list: Vec<String> = edges.iter().map(|e| pick(&pre, e)).collect();
        fs.push(format!(r#"{{"type":"edgeFlange","id":"E{}","edges":[{}],{}}}"#, n + 1, list.join(","), fields));
    }
    run(&fs)
}

fn flat(o: &Out) -> ck_json::Json {
    o.last.as_ref().expect("a sheet entry").get("flatPattern").expect("flat pattern").clone()
}

fn size(o: &Out) -> (f64, f64) {
    let s = flat(o);
    let a = s.get_arr("flatSize").unwrap();
    (a[0].as_f64().unwrap(), a[1].as_f64().unwrap())
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6, "{what}: {a} vs {b}");
}

fn bend(o: &Out, k: usize) -> ck_json::Json {
    flat(o).get_arr("bends").unwrap()[k].clone()
}

fn ok(o: &Out) {
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}

fn valid_one_lump(o: &Out) -> f64 {
    let b = o.body.as_ref().expect("a body");
    let bad = ck_check::validate(b, 1e-6);
    assert!(bad.is_empty(), "{:?}", bad.iter().map(|i| i.code).collect::<Vec<_>>());
    assert_eq!(b.num_lumps(), 1);
    ck_topo::mass::mass_props(b).volume
}

#[test]
fn s1_two_flanges_flat_pattern() {
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":20,"angle":90"#, &["bottom", "top"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 100.0, "width");
    close(h, 93.04778684233861, "length");
    let (b1, b2) = (bend(&o, 0), bend(&o, 1));
    close(b1.get("start").unwrap().as_arr().unwrap()[1].as_f64().unwrap(), 1.7380532894153489, "bend 1 line");
    close(b2.get("start").unwrap().as_arr().unwrap()[1].as_f64().unwrap(), 58.26194671058465, "bend 2 line");
    for b in [&b1, &b2] {
        close(b.get_f64("allowance").unwrap(), 4.523893421169302, "allowance");
        close(b.get_f64("length").unwrap(), 100.0, "bend length");
        close(b.get_f64("angleDeg").unwrap(), 90.0, "angle");
        assert_eq!(b.get_str("direction"), Some("up"));
    }
    // outline: a 100 x 93.05 rectangle
    let fp = flat(&o);
    let l = &fp.get_arr("outline").unwrap()[0];
    assert_eq!(l.as_arr().unwrap().len(), 4);
    // the solid: plate 100 x 52 x 2, two bends of 100 wide, two legs 16 long
    let (t, r) = (2.0f64, 2.0f64);
    let want = 100.0 * 52.0 * t + 2.0 * (std::f64::consts::FRAC_PI_2 / 2.0) * ((r + t).powi(2) - r * r) * 100.0 + 2.0 * 16.0 * t * 100.0;
    let v = valid_one_lump(&o);
    assert!((v - want).abs() < 1e-6 * want, "{v} vs {want}");
    // the flanges rise 20 mm from the outer face
    let (lo, hi) = ck_model::features::body_box(o.body.as_ref().unwrap());
    close(hi.z - lo.z, 20.0, "height of the part");
}

#[test]
fn s4_four_flanges_corner_notch() {
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":15"#, &["bottom", "top", "left", "right"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 123.04778684233861, "width");
    close(h, 83.04778684233861, "length");
    let fp = flat(&o);
    let pts: Vec<Vec<f64>> = fp.get_arr("outline").unwrap()[0].as_arr().unwrap().iter().map(|p| p.as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect()).collect();
    assert_eq!(pts.len(), 28);
    let has = |x: f64, y: f64| pts.iter().any(|p| (p[0] - x).abs() < 1e-6 && (p[1] - y).abs() < 1e-6);
    for (x, y) in [(96.0, 4.0), (100.523893421169, 3.414213562373), (111.523893421169, 56.585786437627), (3.414213562373, 71.523893421169), (-11.523893421169, 3.414213562373), (4.0, -0.523893421169)] {
        assert!(has(x, y), "missing outline point ({x}, {y})");
    }
    for (k, len) in [92.0, 92.0, 52.0, 52.0].iter().enumerate() {
        close(bend(&o, k).get_f64("length").unwrap(), *len, "bend length");
    }
    let zones = fp.get_arr("bendZones").unwrap();
    assert_eq!(zones.len(), 4);
    // plate 92 x 52, bends (92 + 92 + 52 + 52 wide), legs trimmed by the gap: 2 + 2/sqrt 2 from the outer corner
    let (t, r) = (2.0f64, 2.0f64);
    let leg = 15.0 - 4.0;
    let g = 2.0 + 2.0 / 2f64.sqrt();
    let sect = std::f64::consts::FRAC_PI_4 * ((r + t).powi(2) - r * r);
    let want = 92.0 * 52.0 * t + sect * (92.0 + 92.0 + 52.0 + 52.0) + leg * t * ((100.0 - 2.0 * g) * 2.0 + (60.0 - 2.0 * g) * 2.0);
    let v = valid_one_lump(&o);
    assert!((v - want).abs() < 1e-6 * want, "{v} vs {want}");
}

#[test]
fn probes_thickness_radius_k() {
    // t 2, r 3: bends stop 5 from the corners
    let o = plate(2.0, 0.44, 3.0, "", &[(r#""height":15"#, &["bottom", "top", "left", "right"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 122.18937949592839, "r3 width");
    close(h, 82.18937949592839, "r3 length");
    close(bend(&o, 0).get_f64("allowance").unwrap(), 6.0946897479642, "r3 allowance");
    // t 1, r 1
    let o = plate(1.0, 0.44, 1.0, "", &[(r#""height":15"#, &["bottom", "top", "left", "right"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 126.5238934211693, "t1 width");
    close(h, 86.5238934211693, "t1 length");
    // t 1.5, k 0.33, r 4
    let o = plate(1.5, 0.33, 4.0, "", &[(r#""height":20"#, &["bottom", "top", "left", "right"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 132.1214589778861, "t1.5 width");
    close(h, 92.1214589778861, "t1.5 length");
    close(bend(&o, 0).get_f64("allowance").unwrap(), (4.0 + 0.33 * 1.5) * std::f64::consts::FRAC_PI_2, "t1.5 allowance");
}

fn bottom(fields: &str) -> Out {
    plate(2.0, 0.44, 2.0, "", &[(fields, &["bottom"])])
}

fn flat_len(fields: &str) -> f64 {
    let o = bottom(fields);
    ok(&o);
    size(&o).1
}

fn outline(o: &Out) -> Vec<Vec<f64>> {
    flat(o).get_arr("outline").unwrap()[0].as_arr().unwrap().iter().map(|p| p.as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect()).collect()
}

#[test]
fn probes_angle_height_from_position() {
    // flat lengths of one flange on the 60 mm edge, height 20 (reference probes)
    let cases: [(&str, f64); 16] = [
        (r#""height":20,"angle":45"#, 78.94823821209228),
        (r#""height":20,"angle":120"#, 72.17545143461689),
        (r#""height":20,"heightFrom":"inner""#, 78.5238934211693),
        (r#""height":20,"heightFrom":"inner","angle":45"#, 79.77666533683846),
        (r#""height":20,"heightFrom":"inner","angle":120"#, 75.63955304975465),
        (r#""height":20,"heightFrom":"tangent""#, 76.5238934211693),
        (r#""height":20,"heightFrom":"tangent","angle":45"#, 78.94823821209228),
        (r#""height":20,"heightFrom":"tangent","angle":120"#, 75.1036546648924),
        (r#""height":20,"heightFrom":"tangent","angle":150"#, 68.6116191386155),
        (r#""height":20,"position":"inside""#, 76.5238934211693),
        (r#""height":20,"position":"outside""#, 80.5238934211693),
        (r#""height":20,"position":"middle""#, 78.5238934211693),
        (r#""height":20,"flip":true"#, 76.5238934211693),
        (r#""height":4"#, 60.5238934211693),
        (r#""height":4.001"#, 60.5248934211693),
        (r#""height":20,"angle":"90 deg","radius":"2 mm""#, 76.5238934211693),
    ];
    for (fields, want) in cases {
        close(flat_len(fields), want, fields);
    }
}

#[test]
fn probes_angle_details() {
    let o = bottom(r#""height":20,"angle":45"#);
    ok(&o);
    let b = bend(&o, 0);
    close(b.get_f64("allowance").unwrap(), 2.261946710584651, "45 allowance");
    close(b.get_f64("includedAngleDeg").unwrap(), 135.0, "included");
    close(b.get("start").unwrap().as_arr().unwrap()[1].as_f64().unwrap(), 0.5258808937076744, "45 bend line");
    let z: Vec<f64> = flat(&o).get_arr("bendZones").unwrap()[0].as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    close(z[2], -0.6050924615846511, "zone low");
    close(z[3], 1.656854249, "zone high");
    valid_one_lump(&o);
    let o = bottom(r#""height":20,"angle":120"#);
    let b = bend(&o, 0);
    close(b.get_f64("allowance").unwrap(), 6.031858, "120 allowance");
    close(b.get("start").unwrap().as_arr().unwrap()[1].as_f64().unwrap(), 3.912274282553799, "120 bend line");
    valid_one_lump(&o);
    valid_one_lump(&bottom(r#""height":20,"position":"outside""#));
    valid_one_lump(&bottom(r#""height":20,"heightFrom":"inner","angle":120"#));
}

#[test]
fn flip_bends_the_other_way() {
    let o = bottom(r#""height":20,"flip":true"#);
    ok(&o);
    assert_eq!(bend(&o, 0).get_str("direction"), Some("down"));
    let (lo, hi) = ck_model::features::body_box(o.body.as_ref().unwrap());
    // the flange goes towards the face the base started on (below it)
    assert!(lo.z < -17.0 && hi.z < 2.0 + 1e-6, "{lo:?} {hi:?}");
    valid_one_lump(&o);
}

#[test]
fn radius_three_positions() {
    for (pos, want) in [("inside", 76.0946897479642), ("middle", 78.5946897479642), ("outside", 81.0946897479642)] {
        let o = plate(2.0, 0.44, 3.0, "", &[(&format!(r#""height":20,"position":"{pos}""#), &["bottom"])]);
        ok(&o);
        close(size(&o).1, want, pos);
    }
}

#[test]
fn height_must_reach_the_end_of_the_bend() {
    let o = bottom(r#""height":3"#);
    assert_eq!(o.errors, vec![("E1".to_string(), "flangeHeightTooLow".to_string())]);
    assert!(o.body.is_some());
}

#[test]
fn sequential_flanges_trim_only_the_bend() {
    // a flange added next to an existing one: its bend zone and its own leg stop at the neighbour's bend; the first leg stays whole
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":15"#, &["bottom"]), (r#""height":10"#, &["left"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 106.5238934211693, "width");
    close(h, 71.5238934211693, "length");
    let l = outline(&o);
    assert_eq!(l.len(), 8);
    let has = |x: f64, y: f64| l.iter().any(|p| (p[0] - x).abs() < 1e-6 && (p[1] - y).abs() < 1e-6);
    assert!(has(0.0, -11.523893421169302) && has(0.0, -0.523893421169302) && has(4.0, -0.523893421169302) && has(-6.523893421169302, 4.0));
    close(bend(&o, 0).get_f64("length").unwrap(), 96.0, "first bend trimmed");
    valid_one_lump(&o);
    // another angle on the second: the first bend stops at the second one's setback
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":15"#, &["bottom"]), (r#""height":10,"angle":45"#, &["left"])]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 108.94823821209226, "45 width");
    close(h, 71.5238934211693, "45 length");
    valid_one_lump(&o);
}

#[test]
fn gap_sets_the_leg_spacing() {
    let o = plate(2.0, 0.44, 2.0, ", \"gap\": 1", &[(r#""height":15"#, &["bottom", "left"])]);
    ok(&o);
    assert!(outline(&o).iter().any(|p| (p[0] - 2.707106781186548).abs() < 1e-6));
    // the other relief fields do not move the notch corner
    let o = plate(2.0, 0.44, 2.0, ", \"cornerRelief\": 3, \"reliefWidth\": 5, \"reliefDepth\": 4", &[(r#""height":15"#, &["bottom", "left"])]);
    ok(&o);
    assert!(outline(&o).iter().any(|p| (p[0] - 3.414213562373095).abs() < 1e-6));
}

#[test]
fn rule_defaults_and_expressions() {
    let fs = vec![
        r#"{"type":"sheetRules","id":"R1","name":"A","thickness":3}"#.to_string(),
        r#"{"type":"sheetRules","id":"R2","name":"B","thickness":"1.5 mm","bendRadius":"=Thickness*2","cornerRelief":"10","gap":"0.5 mm","cornerReliefShape":"round"}"#.to_string(),
    ];
    let o = run(&fs);
    ok(&o);
    let all = o.wire.get_arr("sheetMetal").unwrap();
    let r = all[0].get("rule").unwrap();
    for (k, v) in [("thickness", 3.0), ("bendRadius", 3.0), ("kFactor", 0.44), ("reliefWidth", 3.0), ("reliefDepth", 1.5), ("cornerRelief", 12.0), ("gap", 3.0), ("hemGap", 0.01)] {
        close(r.get_f64(k).unwrap(), v, k);
    }
    assert_eq!(r.get_str("cornerReliefShape"), Some("notch"));
    let b = all[1].get("rule").unwrap();
    close(b.get_f64("bendRadius").unwrap(), 3.0, "expression");
    close(b.get_f64("cornerRelief").unwrap(), 10.0, "plain string");
    close(b.get_f64("gap").unwrap(), 0.5, "mm string");
    assert_eq!(b.get_str("cornerReliefShape"), Some("round"));
    // no rule at all: 2.5 mm stock
    let o = run(&[sketch(100.0, 60.0), BASE.to_string()]);
    ok(&o);
    close(flat(&o).get_arr("flatSize").unwrap()[2].as_f64().unwrap(), 2.5, "default thickness");
    close(valid_one_lump(&o), 100.0 * 60.0 * 2.5, "plate volume");
}

#[test]
fn errors() {
    let o = run(&[r#"{"type":"edgeFlange","id":"E1","edges":[[0,0,0]],"height":10}"#.to_string()]);
    assert_eq!(o.errors, vec![("E1".to_string(), "nothingToCut".to_string())]);
    let o = run(&[r#"{"type":"baseFlange","id":"B1","sketch":"nope"}"#.to_string()]);
    assert_eq!(o.errors, vec![("B1".to_string(), "missingSketch".to_string())]);
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":20,"position":"sideways""#, &["bottom"])]);
    assert_eq!(o.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":20,"angle":200"#, &["bottom"])]);
    assert_eq!(o.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
    let o = plate(2.0, 0.44, 2.0, "", &[(r#""height":20"#, &["bottom"]), (r#""height":20"#, &["bottom"])]);
    assert_eq!(o.errors, vec![("E2".to_string(), "sheetMetalUnsupported".to_string())]);
}
