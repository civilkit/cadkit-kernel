//! Sheet metal bend table (`sheetRules.bendTable`, docs/sheet-metal.md, "Bend table"): the k-factor chosen by inside radius
//! over thickness and by bend angle, linear interpolation between rows, a named refusal outside the table, and the flat
//! pattern, the bend table export, unfold and refold using the k of each bend. Expected numbers are hand-computed from
//! `BA = (r + k t) a`, setback `S = (r + t) tan(a / 2)` and the flat length of a plate with two flanges
//! `(60 - S1 - S2) + BA1 + BA2 + (H - S1) + (H - S2)`.
//! Without a table nothing changes (the existing `sheet_*` tests pin that; here it is checked once more against a table of one k).
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

fn valid_one_lump(o: &Out) -> f64 {
    let b = o.body.as_ref().expect("a body");
    let bad = ck_check::validate(b, 1e-6);
    assert!(bad.is_empty(), "{:?}", bad.iter().map(|i| i.code).collect::<Vec<_>>());
    assert_eq!(b.num_lumps(), 1);
    ck_topo::mass::mass_props(b).volume
}


use ck_model::features::sheet::{BendTable, TableRow};
use ck_model::features::sheet_export::{bend_table_csv, bend_table_json, flat_pattern_from_json};

fn flat(o: &Out) -> ck_json::Json {
    o.last.as_ref().expect("a sheet entry").get("flatPattern").expect("flat pattern").clone()
}

fn flat_len(o: &Out) -> f64 {
    flat(o).get_arr("flatSize").unwrap()[1].as_f64().unwrap()
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6, "{what}: {a} vs {b}");
}

fn ok(o: &Out) {
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}

fn bends(o: &Out) -> Vec<ck_json::Json> {
    flat(o).get_arr("bends").unwrap().to_vec()
}

/// The bend row of the given angle.
fn by_angle(o: &Out, deg: f64) -> ck_json::Json {
    bends(o).into_iter().find(|b| (b.get_f64("angleDeg").unwrap() - deg).abs() < 1e-9).unwrap_or_else(|| panic!("no bend of {deg} degrees"))
}

const T: f64 = 2.0;
const R: f64 = 2.0;

fn ba(k: f64, deg: f64) -> f64 {
    (R + k * T) * deg.to_radians()
}
fn setback(deg: f64) -> f64 {
    (R + T) * (deg.to_radians() / 2.0).tan()
}

/// Flat length of the 100 x 60 plate with a flange of height `h` on the bottom (angle `a1`, k `k1`) and one on the top (`a2`, `k2`).
fn want_len(h: f64, a1: f64, k1: f64, a2: f64, k2: f64) -> f64 {
    let (s1, s2) = (setback(a1), setback(a2));
    (60.0 - s1 - s2) + ba(k1, a1) + ba(k2, a2) + (h - s1) + (h - s2)
}

/// Rows by angle for the radius ratio 1 (R / T): 45 degrees k 0.30, 90 degrees k 0.50, 120 degrees k 0.40.
const TABLE_ANGLES: &str = r#","bendTable":[{"radiusRatio":1,"angle":45,"k":0.30},{"radiusRatio":1,"angle":90,"k":0.50},{"radiusRatio":1,"angle":120,"k":0.40}]"#;

fn two_flanges(extra: &str, a1: f64, a2: f64) -> Out {
    let f1 = format!(r#""height":20,"angle":{a1}"#);
    let f2 = format!(r#""height":20,"angle":{a2}"#);
    plate(T, 0.44, R, extra, &[(f1.as_str(), &["bottom"]), (f2.as_str(), &["top"])])
}

#[test]
fn two_flanges_of_different_angles_use_the_k_of_each_bend() {
    let o = two_flanges(TABLE_ANGLES, 90.0, 45.0);
    ok(&o);
    // k 0.50 at 90 degrees, 0.30 at 45 degrees: the flat length is the hand-computed sum, to 1e-6
    close(flat_len(&o), want_len(20.0, 90.0, 0.50, 45.0, 0.30), "flat length");
    close(by_angle(&o, 90.0).get_f64("allowance").unwrap(), ba(0.50, 90.0), "allowance 90");
    close(by_angle(&o, 45.0).get_f64("allowance").unwrap(), ba(0.30, 45.0), "allowance 45");
    close(by_angle(&o, 90.0).get_f64("kFactor").unwrap(), 0.50, "k 90");
    close(by_angle(&o, 45.0).get_f64("kFactor").unwrap(), 0.30, "k 45");
    // and it differs from the constant k of the rule (0.44)
    let c = two_flanges("", 90.0, 45.0);
    ok(&c);
    close(flat_len(&c), want_len(20.0, 90.0, 0.44, 45.0, 0.44), "constant k flat length");
    assert!((flat_len(&o) - flat_len(&c)).abs() > 1e-3);
    // the folded solid is still one valid lump
    let b = o.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty());
    assert_eq!(b.num_lumps(), 1);
}

#[test]
fn the_bend_table_export_lists_the_k_of_each_bend() {
    let o = two_flanges(TABLE_ANGLES, 90.0, 45.0);
    ok(&o);
    let fp = flat_pattern_from_json(&flat(&o)).unwrap();
    assert!(fp.bend_table);
    let csv = bend_table_csv(&fp, "Body1");
    let lines: Vec<&str> = csv.trim_end().split("\r\n").collect();
    assert_eq!(lines.len(), 3);
    let cols = |l: &str| l.split(',').map(str::to_string).collect::<Vec<_>>();
    let head = cols(lines[0]);
    let (kc, ac) = (head.iter().position(|c| c == "k_factor").unwrap(), head.iter().position(|c| c == "angle_deg").unwrap());
    let mut got: Vec<(f64, f64)> = lines[1..].iter().map(|l| { let c = cols(l); (c[ac].parse().unwrap(), c[kc].parse().unwrap()) }).collect();
    got.sort_by(|a, b| a.0.total_cmp(&b.0));
    close(got[0].0, 45.0, "csv angle");
    close(got[0].1, 0.30, "csv k at 45");
    close(got[1].0, 90.0, "csv angle");
    close(got[1].1, 0.50, "csv k at 90");
    let j = bend_table_json(&fp, "Body1");
    let mut ks: Vec<f64> = j.get_arr("bends").unwrap().iter().map(|b| b.get_f64("k_factor").unwrap()).collect();
    ks.sort_by(|a, b| a.total_cmp(b));
    close(ks[0], 0.30, "json k");
    close(ks[1], 0.50, "json k");
    // without a table the JSON bends keep their old keys (no per-bend k)
    let c = two_flanges("", 90.0, 45.0);
    let fpc = flat_pattern_from_json(&flat(&c)).unwrap();
    assert!(!fpc.bend_table);
    assert!(bend_table_json(&fpc, "Body1").get_arr("bends").unwrap().iter().all(|b| b.get("k_factor").is_none()));
    assert!(bends(&c).iter().all(|b| b.get("kFactor").is_none()));
    let csvc = bend_table_csv(&fpc, "Body1");
    assert!(csvc.lines().skip(1).all(|l| l.contains(",0.440000000,")));
}

#[test]
fn k_is_interpolated_between_rows() {
    // by angle, inside one radius ratio: 60 degrees is 15 of the 45 degrees between 45 (0.30) and 90 (0.50)
    let k60 = 0.30 + 0.20 * (15.0 / 45.0);
    let o = plate(T, 0.44, R, TABLE_ANGLES, &[(r#""height":20,"angle":60"#, &["bottom"])]);
    ok(&o);
    close(by_angle(&o, 60.0).get_f64("kFactor").unwrap(), k60, "k at 60");
    close(by_angle(&o, 60.0).get_f64("allowance").unwrap(), ba(k60, 60.0), "allowance at 60");
    // by radius ratio alone (rows without an angle): ratio 1 lies a third of the way from 0.5 (k 0.30) to 2 (k 0.50)
    let t = r#","bendTable":[{"radiusRatio":0.5,"k":0.30},{"radiusRatio":2,"k":0.50}]"#;
    let o = plate(T, 0.44, R, t, &[(r#""height":20,"angle":90"#, &["bottom"])]);
    ok(&o);
    let k = 0.30 + 0.20 * (0.5 / 1.5);
    close(by_angle(&o, 90.0).get_f64("kFactor").unwrap(), k, "k by ratio");
    close(by_angle(&o, 90.0).get_f64("allowance").unwrap(), ba(k, 90.0), "allowance by ratio");
    // both: rows at ratios 0.5 and 2 with their own angles; ratio 1 at 90 degrees is interpolated between the two ratios' k at 90
    let t = r#","bendTable":[{"radiusRatio":0.5,"angle":45,"k":0.30},{"radiusRatio":0.5,"angle":135,"k":0.40},{"radiusRatio":2,"angle":45,"k":0.40},{"radiusRatio":2,"angle":135,"k":0.50}]"#;
    let o = plate(T, 0.44, R, t, &[(r#""height":20,"angle":90"#, &["bottom"])]);
    ok(&o);
    let (k_lo, k_hi) = (0.35, 0.45);
    close(by_angle(&o, 90.0).get_f64("kFactor").unwrap(), k_lo + (k_hi - k_lo) * (0.5 / 1.5), "k by ratio and angle");
    // the table itself: exact rows, and no extrapolation at the ends
    let tb = BendTable::new(vec![TableRow { ratio: 1.0, angle: None, k: 0.3 }, TableRow { ratio: 3.0, angle: None, k: 0.5 }]).unwrap();
    close(tb.k_at(1.0, 10.0).unwrap(), 0.3, "row 1");
    close(tb.k_at(3.0, 170.0).unwrap(), 0.5, "row 2");
    close(tb.k_at(2.0, 10.0).unwrap(), 0.4, "middle");
    assert!(tb.k_at(0.99, 90.0).is_err() && tb.k_at(3.01, 90.0).is_err());
}

#[test]
fn a_bend_outside_the_table_is_refused_with_a_name() {
    let refused = |o: &Out| {
        assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
        assert_eq!(o.errors[0].1, "bendOutsideTable");
        let s = ck_json::to_string(&o.wire);
        assert!(s.contains("bendOutsideTable") && s.contains("detail") && s.contains("hint"), "{s}");
    };
    // the angle 150 is above the rows (45 to 120)
    refused(&plate(T, 0.44, R, TABLE_ANGLES, &[(r#""height":30,"angle":150"#, &["bottom"])]));
    // the angle 30 is below them
    refused(&plate(T, 0.44, R, TABLE_ANGLES, &[(r#""height":20,"angle":30"#, &["bottom"])]));
    // the radius: ratio 5 (radius 10 at 2 mm) is above the only ratio of the table
    refused(&plate(T, 0.44, 10.0, TABLE_ANGLES, &[(r#""height":30,"angle":90"#, &["bottom"])]));
    // a chained bend outside the table is refused too
    refused(&plate(T, 0.44, R, TABLE_ANGLES, &[(r#""height":30,"angle":90,"then":[{"angle":30,"height":12}]"#, &["bottom"])]));
    // a table that cannot be read is an invalid value, named
    let o = plate(T, 0.44, R, r#","bendTable":[{"radiusRatio":1,"angle":45,"k":0.3},{"radiusRatio":2,"k":0.4}]"#, &[]);
    assert_eq!(o.errors[0].1, "invalidValue");
}

#[test]
fn without_a_table_nothing_changes_and_a_constant_table_is_the_same() {
    let plain = two_flanges("", 90.0, 90.0);
    ok(&plain);
    close(flat_len(&plain), 93.04778684233861, "the known flat length");
    assert!(flat(&plain).get("bends").unwrap().to_string().find("kFactor").is_none());
    // a table that gives 0.44 everywhere is the rule's constant
    let t = r#","bendTable":[{"radiusRatio":0.1,"angle":1,"k":0.44},{"radiusRatio":0.1,"angle":180,"k":0.44},{"radiusRatio":50,"angle":1,"k":0.44},{"radiusRatio":50,"angle":180,"k":0.44}]"#;
    let c = two_flanges(t, 90.0, 90.0);
    ok(&c);
    close(flat_len(&c), 93.04778684233861, "constant table flat length");
}

#[test]
fn unfold_and_refold_use_the_k_of_each_bend() {
    let fl = |extra: &[&str]| {
        let pre = run(&[sketch(100.0, 60.0)]);
        let mut fs = vec![rule(T, 0.44, R, TABLE_ANGLES), sketch(100.0, 60.0), BASE.to_string()];
        fs.push(format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{}],"height":20,"angle":90}}"#, pick(&pre, "bottom")));
        fs.push(format!(r#"{{"type":"edgeFlange","id":"E2","edges":[{}],"height":20,"angle":45}}"#, pick(&pre, "top")));
        for e in extra {
            fs.push(e.to_string());
        }
        run(&fs)
    };
    let folded = fl(&[]);
    ok(&folded);
    let v0 = valid_one_lump(&folded);
    let len = flat_len(&folded);
    close(len, want_len(20.0, 90.0, 0.50, 45.0, 0.30), "folded flat length");
    let u = fl(&[r#"{"type":"unfold","id":"U1"}"#]);
    ok(&u);
    // the flat sheet is the flat pattern extruded by the thickness
    close(valid_one_lump(&u), 100.0 * len * T, "unfolded volume");
    close(flat_len(&u), len, "unfolded flat length");
    let r = fl(&[r#"{"type":"unfold","id":"U1"}"#, r#"{"type":"refold","id":"U2"}"#]);
    ok(&r);
    let v1 = valid_one_lump(&r);
    assert!((v1 - v0).abs() < 1e-6 * v0, "refold volume {v1} vs {v0}");
    close(flat_len(&r), len, "refolded flat length");
}

#[test]
fn a_fold_uses_the_table_too() {
    let line = r#"{"type":"sketch","id":"S2","plane":{"kind":"std","id":"front"},"points":[{"id":"q1","x":40,"y":-5},{"id":"q2","x":40,"y":70}],"entities":[{"type":"line","id":"f1","p1":"q1","p2":"q2"}],"constraints":[]}"#;
    let fold = r#"{"type":"fold","id":"F1","line":{"sketch":"S2","curve":"f1"},"angle":60}"#;
    let o = run(&[rule(T, 0.44, R, TABLE_ANGLES), sketch(100.0, 60.0), BASE.to_string(), line.to_string(), fold.to_string()]);
    ok(&o);
    let k60 = 0.30 + 0.20 * (15.0 / 45.0);
    close(by_angle(&o, 60.0).get_f64("allowance").unwrap(), ba(k60, 60.0), "fold allowance");
    close(by_angle(&o, 60.0).get_f64("kFactor").unwrap(), k60, "fold k");
    // outside the table
    let fold = r#"{"type":"fold","id":"F1","line":{"sketch":"S2","curve":"f1"},"angle":20}"#;
    let o = run(&[rule(T, 0.44, R, TABLE_ANGLES), sketch(100.0, 60.0), BASE.to_string(), line.to_string(), fold.to_string()]);
    assert_eq!(o.errors.last().map(|e| e.1.as_str()), Some("bendOutsideTable"), "{:?}", o.errors);
}
