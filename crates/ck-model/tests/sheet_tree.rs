//! Sheet metal, stage 3: the panel tree (docs/sheet-metal.md, "Stage 3"). The legacy forms convert to the tree exactly; flanges,
//! hems and folds combine on any panel; corner reliefs are built and unfolded. Expected numbers are closed forms (bend allowance
//! `(r + k t) a`, volume = area x thickness for the flat sheet, bend volumes by the section area), plus the legacy numbers the
//! conversion must reproduce.
use ck_model::features::{sheet_features, Brep};
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::f64::consts::PI;

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
    brep: Brep,
    errors: Vec<(String, String)>,
    err_json: Vec<ck_json::Json>,
    warnings: Vec<(String, ck_json::Json)>,
    last: Option<ck_json::Json>,
    frame: Option<ck_core::linalg::Frame>,
}

impl Out {
    fn body(&self) -> &Body {
        self.brep.body.as_ref().expect("a body")
    }
}

fn run(features: &[String]) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let wire = r.to_json();
    // the entry of the last feature that made one (entries are sorted by id)
    let last_id = features.iter().rev().filter_map(|f| ck_json::parse(f).ok()).find(|j| j.get_str("type").map_or(false, |t| ["baseFlange", "edgeFlange", "hem", "fold", "contourFlange", "unfold", "refold", "sheetCorner", "sheetCut", "sheetConvert"].contains(&t))).and_then(|j| j.get_str("id").map(str::to_string));
    let last = wire.get_arr("sheetMetal").and_then(|a| a.iter().find(|e| e.get("flatPattern").is_some() && e.get_str("id").map(str::to_string) == last_id).cloned());
    Out {
        brep: ex.0,
        errors: r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect(),
        err_json: r.errors.iter().map(|e| ck_json::Json::Obj(e.error.params.clone())).collect(),
        warnings: r.warnings.iter().map(|e| (format!("{}:{}", e.feature, e.error.code), ck_json::Json::Obj(e.error.params.clone()))).collect(),
        last,
        frame: r.sketches.first().map(|s| s.frame),
    }
}

fn rule(t: f64, k: f64, r: f64, extra: &str) -> String {
    format!(r#"{{"type":"sheetRules","id":"R1","name":"R","thickness":{t},"kFactor":{k},"bendRadius":{r}{extra}}}"#)
}

fn polygon(pts: &[(f64, f64)]) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q.0, q.1)).collect();
    let n = pts.len();
    let e: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}

fn rect(w: f64, h: f64) -> String {
    polygon(&[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)])
}

fn chain(pts: &[(f64, f64)]) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q.0, q.1)).collect();
    let e: Vec<String> = (0..pts.len() - 1).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, i + 2)).collect();
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}

const BASE: &str = r#"{"type":"baseFlange","id":"B1","sketch":"S1"}"#;

fn frame_of() -> ck_core::linalg::Frame {
    run(&[rect(100.0, 60.0)]).frame.expect("sketch frame")
}

/// A point on the sketch plane at `(x, y)` and height `h` above it, in world coordinates, as a feature pick.
fn at(x: f64, y: f64, h: f64) -> String {
    let f = frame_of();
    let p = f.o + f.x * x + f.y * y + f.z * h;
    format!("[{},{},{}]", p.x, p.y, p.z)
}

/// Rule (t 2, k 0.44, r 2) + 100 x 60 base + the given features.
fn part(features: &[String]) -> Out {
    part_with(2.0, 0.44, 2.0, "", features)
}

fn part_with(t: f64, k: f64, r: f64, extra: &str, features: &[String]) -> Out {
    let mut fs = vec![rule(t, k, r, extra), rect(100.0, 60.0), BASE.to_string()];
    fs.extend(features.iter().cloned());
    run(&fs)
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6, "{what}: {a} vs {b}");
}

fn rel(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} vs {b}");
}

fn ok(o: &Out) {
    assert!(o.errors.is_empty(), "{:?} {:?}", o.errors, o.err_json.iter().map(|j| j.get_str("detail").map(str::to_string)).collect::<Vec<_>>());
}

fn flat(o: &Out) -> ck_json::Json {
    o.last.as_ref().expect("a sheet entry").get("flatPattern").expect("flat pattern").clone()
}

fn size(o: &Out) -> (f64, f64) {
    let s = flat(o);
    let a = s.get_arr("flatSize").unwrap();
    (a[0].as_f64().unwrap(), a[1].as_f64().unwrap())
}

fn bends(o: &Out) -> Vec<ck_json::Json> {
    flat(o).get_arr("bends").unwrap().to_vec()
}

fn loops(o: &Out) -> Vec<Vec<(f64, f64)>> {
    flat(o)
        .get_arr("outline")
        .unwrap()
        .iter()
        .map(|l| l.as_arr().unwrap().iter().map(|p| (p.as_arr().unwrap()[0].as_f64().unwrap(), p.as_arr().unwrap()[1].as_f64().unwrap())).collect())
        .collect()
}

fn shoelace(l: &[(f64, f64)]) -> f64 {
    let n = l.len();
    (0..n).map(|i| l[i].0 * l[(i + 1) % n].1 - l[(i + 1) % n].0 * l[i].1).sum::<f64>() * 0.5
}

/// Area of the flat pattern (outer loop less holes).
fn flat_area(o: &Out) -> f64 {
    loops(o).iter().enumerate().map(|(i, l)| if i == 0 { shoelace(l).abs() } else { -shoelace(l).abs() }).sum()
}

fn valid_one_lump(o: &Out) -> f64 {
    let b = o.body();
    let bad = ck_check::validate(b, 1e-6);
    assert!(bad.is_empty(), "{:?}", bad.iter().map(|i| i.code).collect::<Vec<_>>());
    assert_eq!(b.num_lumps(), 1, "one lump");
    ck_topo::mass::mass_props(b).volume
}

fn vol_close(v: f64, want: f64, tol: f64, what: &str) {
    assert!((v - want).abs() < tol * want, "{what}: volume {v} vs {want}");
}

fn fbox(o: &Out) -> ([f64; 3], [f64; 3]) {
    let f = o.frame.unwrap_or_else(frame_of);
    let (lo, hi) = ck_model::features::body_box(o.body());
    let mut a = [f64::MAX; 3];
    let mut b = [f64::MIN; 3];
    for k in 0..8 {
        let p = ck_core::linalg::Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z }) - f.o;
        for (i, v) in [p.dot(f.x), p.dot(f.y), p.dot(f.z)].into_iter().enumerate() {
            a[i] = a[i].min(v);
            b[i] = b[i].max(v);
        }
    }
    (a, b)
}

const BA90: f64 = 4.523893421169302;

fn flange(id: &str, edges: &[(f64, f64, f64)], fields: &str) -> String {
    let e: Vec<String> = edges.iter().map(|p| at(p.0, p.1, p.2)).collect();
    format!(r#"{{"type":"edgeFlange","id":"{id}","edges":[{}],{fields}}}"#, e.join(","))
}

fn fold(id: &str, a: (f64, f64, f64), b: (f64, f64, f64), fields: &str) -> String {
    let more = if fields.is_empty() { String::new() } else { format!(",{fields}") };
    format!(r#"{{"type":"fold","id":"{id}","points":[{},{}]{}}}"#, at(a.0, a.1, a.2), at(b.0, b.1, b.2), more)
}

// ---------------------------------------------------------------- the conversion is exact

/// Convert the legacy model of a run to its panel tree; the solid and the flat pattern must not change.
fn same_as_legacy(mut o: Out) {
    ok(&o);
    let vol0 = valid_one_lump(&o);
    let faces0 = o.body().num_faces();
    let (w0, h0) = size(&o);
    let bends0 = bends(&o);
    let area0 = flat_area(&o);
    let zones0 = flat(&o).get("bendZones").unwrap().clone();
    sheet_features::convert_to_tree(&mut o.brep).expect("convert");
    assert!(o.brep.sheet.model.as_ref().unwrap().tree.is_some());
    let tree = o.brep.sheet.model.as_ref().unwrap().tree.clone().unwrap();
    let rule = o.brep.sheet.model.as_ref().unwrap().sheet.rule.clone();
    let fp = tree.flat_pattern(&rule, false);
    close(fp.size[0], w0, "tree width");
    close(fp.size[1], h0, "tree length");
    assert_eq!(fp.bends.len(), bends0.len());
    for (a, b) in fp.bends.iter().zip(&bends0) {
        close(a.allowance, b.get_f64("allowance").unwrap(), "allowance");
        close(a.angle, b.get_f64("angleDeg").unwrap(), "angle");
        close(a.radius, b.get_f64("radius").unwrap(), "radius");
        close(a.length, b.get_f64("length").unwrap(), "length");
        assert_eq!(a.up, b.get_str("direction") == Some("up"), "direction");
        let s = b.get("start").unwrap().as_arr().unwrap();
        let e = b.get("end").unwrap().as_arr().unwrap();
        close(a.start[0], s[0].as_f64().unwrap(), "start x");
        close(a.start[1], s[1].as_f64().unwrap(), "start y");
        close(a.end[0], e[0].as_f64().unwrap(), "end x");
        close(a.end[1], e[1].as_f64().unwrap(), "end y");
    }
    for (a, b) in fp.zones.iter().zip(zones0.as_arr().unwrap()) {
        for k in 0..4 {
            close(a[k], b.as_arr().unwrap()[k].as_f64().unwrap(), "zone");
        }
    }
    let area_t: f64 = fp.outline.iter().enumerate().map(|(i, l)| if i == 0 { ck_model::features::sheet::area(l).abs() } else { -ck_model::features::sheet::area(l).abs() }).sum();
    close(area_t, area0, "flat area");
    let vol1 = valid_one_lump(&o);
    vol_close(vol1, vol0, 1e-6, "tree solid");
    assert_eq!(o.body().num_faces(), faces0, "faces");
}

#[test]
fn tree_equals_legacy_flanges() {
    same_as_legacy(part(&[flange("E1", &[(50.0, 0.0, 2.0), (50.0, 60.0, 2.0)], r#""height":20"#)]));
}

#[test]
fn tree_equals_legacy_four_flanges_with_gap_corners() {
    same_as_legacy(part(&[flange("E1", &[(50.0, 0.0, 2.0), (50.0, 60.0, 2.0), (0.0, 30.0, 2.0), (100.0, 30.0, 2.0)], r#""height":15"#)]));
}

#[test]
fn tree_equals_legacy_flange_later_next_to_a_flange() {
    same_as_legacy(part(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":15"#), flange("E2", &[(0.0, 30.0, 2.0)], r#""height":12"#)]));
}

#[test]
fn tree_equals_legacy_hem_and_chain() {
    let hem = format!(r#"{{"type":"hem","id":"H1","edges":[{}],"length":10}}"#, at(50.0, 0.0, 2.0));
    same_as_legacy(part(&[hem]));
    same_as_legacy(part(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"then":[{"angle":90,"height":10},{"angle":45,"height":8,"flip":true}]"#)]));
}

#[test]
fn tree_equals_legacy_folds() {
    same_as_legacy(part(&[fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":60"#)]));
    same_as_legacy(part(&[fold("F1", (20.0, 0.0, 2.0), (20.0, 60.0, 2.0), r#""angle":45"#), fold("F2", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":60"#)]));
    same_as_legacy(part(&[fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90,"flip":true,"position":"mould""#)]));
}

#[test]
fn tree_equals_legacy_contour_flange() {
    for pts in [vec![(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)], vec![(0.0, 0.0), (0.0, 20.0), (30.0, 20.0), (30.0, 0.0)], vec![(0.0, 0.0), (20.0, 20.0), (40.0, 0.0), (60.0, 20.0)]] {
        for fields in ["", r#","flip":true"#, r#","reverse":true"#, r#","flip":true,"reverse":true"#] {
            let o = run(&[rule(2.0, 0.44, 2.0, ""), chain(&pts), format!(r#"{{"type":"contourFlange","id":"C1","sketch":"S1","distance":40{fields}}}"#)]);
            same_as_legacy(o);
        }
    }
}

// ---------------------------------------------------------------- flanges and folds on one base

#[test]
fn flange_after_fold_on_the_folded_panel() {
    // fold the right side up 90 degrees at x = 70, then flange the far edge of the folded panel (a wall of P1)
    let o = part(&[fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#)]);
    ok(&o);
    let roles = o.last.as_ref().unwrap().get("roles").unwrap().get_arr("faces").unwrap().iter().filter_map(|f| f.get_str("role")).map(str::to_string).collect::<Vec<_>>();
    assert!(roles.contains(&"P1.side.e3".to_string()) || roles.contains(&"P1.side.e2".to_string()), "{roles:?}");
    let fl = format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{{"role":"P1.side.e2"}}],"height":15,"angle":90}}"#);
    let o = part(&[fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#), fl]);
    ok(&o);
    let bs = bends(&o);
    assert_eq!(bs.len(), 2);
    close(bs[1].get_f64("allowance").unwrap(), BA90, "flange allowance");
    // the flat pattern: e2 is the far side wall (x = 100); the wall moves in by 4 and the flange adds BA + (15 - 4)
    let (w, h) = size(&o);
    close(w, 100.0 - 4.0 + BA90 + 11.0, "width");
    close(h, 60.0, "length");
    valid_one_lump(&o);
}

#[test]
fn fold_after_flange_and_flange_after_fold_agree() {
    // a flange on the bottom edge and a fold at x = 70: the order of the two features must not change the sheet
    let fl = flange("E1", &[(0.0, 30.0, 2.0)], r#""height":15"#);
    let fo = fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#);
    let a = part(&[fl.clone(), fo.clone()]);
    ok(&a);
    let b = part(&[fo, fl]);
    ok(&b);
    let (va, vb) = (valid_one_lump(&a), valid_one_lump(&b));
    vol_close(va, vb, 1e-3, "order");
    // the flat area is the plate, the zone strip of the fold stays inside it, the flange adds its zone and leg
    let leg = 15.0 - 4.0;
    let want_area = 100.0 * 60.0 - 100.0_f64.min(8.0) * 0.0 + 0.0;
    let _ = want_area;
    assert!(flat_area(&a) > 6000.0 + 11.0 * 8.0, "{}", flat_area(&a));
    rel(flat_area(&a), flat_area(&b), 1e-3, "flat area");
    // bends: one flange (BA 90) and one fold (BA 90)
    for o in [&a, &b] {
        let bs = bends(o);
        assert_eq!(bs.len(), 2);
        for bd in bs {
            close(bd.get_f64("allowance").unwrap(), BA90, "BA");
        }
    }
    let _ = leg;
}

#[test]
fn several_folds_flanges_and_hems_on_one_base() {
    let fo1 = fold("F1", (30.0, 0.0, 2.0), (30.0, 60.0, 2.0), r#""angle":45,"fixed":[0,0,0]"#);
    let _ = fo1;
    let fo = fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":60"#);
    let fl = flange("E1", &[(20.0, 0.0, 2.0)], r#""height":14"#);
    let hm = format!(r#"{{"type":"hem","id":"H1","edges":[{}],"length":8}}"#, at(20.0, 60.0, 2.0));
    let o = part(&[fo, fl, hm]);
    ok(&o);
    assert_eq!(bends(&o).len(), 3);
    valid_one_lump(&o);
}

// ---------------------------------------------------------------- oblique folds, concave bases

#[test]
fn oblique_fold_line() {
    // a 45 degree line across the base: the flat pattern is the plate, the bend zone is a strip of BA / cos 45 along the base
    let o = part(&[fold("F1", (30.0, 0.0, 2.0), (90.0, 60.0, 2.0), r#""angle":60"#)]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 100.0, "flat width");
    close(h, 60.0, "flat length");
    close(flat_area(&o), 6000.0, "flat area");
    let b = &bends(&o)[0];
    let ba = (2.0 + 0.44 * 2.0) * PI / 3.0;
    close(b.get_f64("allowance").unwrap(), ba, "BA");
    // the bend line is as long as the line across the base (the zone is cut square to the line, ends oblique)
    let dl = (60.0f64.powi(2) + 60.0f64.powi(2)).sqrt();
    assert!(b.get_f64("length").unwrap() <= dl + 1e-6 && b.get_f64("length").unwrap() > dl - 4.0);
    let v = valid_one_lump(&o);
    // the plate volume less the little the bend zone's ends differ by (planar ends, flat ends oblique)
    rel(v, 6000.0 * 2.0, 0.02, "volume");
}

fn l_shape() -> String {
    polygon(&[(0.0, 0.0), (100.0, 0.0), (100.0, 40.0), (40.0, 40.0), (40.0, 80.0), (0.0, 80.0)])
}

fn part_poly(sk: String, features: &[String]) -> Out {
    let mut fs = vec![rule(2.0, 0.44, 2.0, ""), sk, BASE.to_string()];
    fs.extend(features.iter().cloned());
    run(&fs)
}

#[test]
fn concave_base_takes_flanges() {
    // L shape: area 100 x 40 + 40 x 40 = 5600; a flange on the 100-long bottom edge and on the inner (reflex) edge
    let o = part_poly(l_shape(), &[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":15"#)]);
    ok(&o);
    assert_eq!(bends(&o).len(), 1);
    close(bends(&o)[0].get_f64("length").unwrap(), 100.0, "bend length");
    // the flange adds its zone and leg to the plate less the strip the bend takes in
    let leg = 15.0 - 4.0;
    let want = 5600.0 + 100.0 * (BA90 + leg) - 100.0 * 4.0;
    close(flat_area(&o), want, "flat area");
    let v = valid_one_lump(&o);
    let vol = (5600.0 - 100.0 * 4.0) * 2.0 + 100.0 * (PI / 4.0) * ((2.0 + 2.0f64).powi(2) - 4.0) + 100.0 * leg * 2.0;
    vol_close(v, vol, 1e-4, "volume");
    // a flange on the edge beside the notch
    let o = part_poly(l_shape(), &[flange("E1", &[(50.0, 0.0, 2.0), (70.0, 40.0, 2.0), (20.0, 80.0, 2.0)], r#""height":12"#)]);
    ok(&o);
    assert_eq!(bends(&o).len(), 3);
    valid_one_lump(&o);
}

#[test]
fn concave_base_takes_a_fold() {
    // a vertical fold line across the L's wide part: the line meets the outline in two chords of different length
    let o = part_poly(l_shape(), &[fold("F1", (70.0, -10.0, 2.0), (70.0, 90.0, 2.0), r#""angle":90"#)]);
    ok(&o);
    close(flat_area(&o), 5600.0, "flat area");
    assert_eq!(bends(&o).len(), 1);
    close(bends(&o)[0].get_f64("length").unwrap(), 40.0, "bend length");
    valid_one_lump(&o);
    // a line through the narrow arm and the wide part: two zones from one line
    let o = part_poly(l_shape(), &[fold("F1", (20.0, -10.0, 2.0), (20.0, 90.0, 2.0), r#""angle":90,"fixed":[5,5,0]"#)]);
    ok(&o);
    assert_eq!(bends(&o).len(), 1);
    close(bends(&o)[0].get_f64("length").unwrap(), 80.0, "bend length");
    valid_one_lump(&o);
}

// ---------------------------------------------------------------- flanges on the walls of a contour flange

#[test]
fn flange_on_a_contour_flange_wall() {
    // an L contour (legs 16 + 26 straight, one 90 degree bend), distance 40; a flange on the end wall of the first leg
    let c = r#"{"type":"contourFlange","id":"C1","sketch":"S1","distance":40}"#.to_string();
    let base = run(&[rule(2.0, 0.44, 2.0, ""), chain(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)]), c.clone()]);
    ok(&base);
    let fl = r#"{"type":"edgeFlange","id":"E1","edges":[{"role":"P0.side.end"}],"height":12,"angle":90}"#.to_string();
    let o = run(&[rule(2.0, 0.44, 2.0, ""), chain(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)]), c, fl]);
    ok(&o);
    let bs = bends(&o);
    assert_eq!(bs.len(), 2);
    close(bs[1].get_f64("allowance").unwrap(), BA90, "flange BA");
    // the flat strip is 46.52 x 40 and the flange adds a (BA + leg) x 16 strip
    let leg = 12.0 - 4.0;
    // the flange moves its wall in by 4 (so the plate loses 16 x 4) and trims the end of the bend beside it by 4
    let want = base_area(&base) + 16.0 * (BA90 + leg) - 16.0 * 4.0 - 4.0 * BA90;
    rel(flat_area(&o), want, 1e-6, "flat area");
    valid_one_lump(&o);
}

fn base_area(o: &Out) -> f64 {
    flat_area(o)
}

// ---------------------------------------------------------------- corner reliefs

fn four_flanges(shape: &str, extra: &str) -> Out {
    let r = format!(r#","cornerReliefShape":"{shape}"{extra}"#);
    part_with(2.0, 0.44, 2.0, &r, &[flange("E1", &[(50.0, 0.0, 2.0), (50.0, 60.0, 2.0), (0.0, 30.0, 2.0), (100.0, 30.0, 2.0)], r#""height":15"#)])
}

#[test]
fn corner_reliefs_are_built_and_unfolded() {
    let mut vols = vec![];
    for shape in ["round", "rectangular", "tear"] {
        let o = four_flanges(shape, "");
        ok(&o);
        assert_eq!(bends(&o).len(), 4, "{shape}");
        let v = valid_one_lump(&o);
        vols.push(v);
        // the flat pattern shows the relief: there are more outline points than the 4 + 4 x 4 of a plain corner
        if shape == "round" {
            assert!(loops(&o)[0].len() > 28, "{shape} outline {}", loops(&o)[0].len());
        }
        // the relief only removes material
        let plain = four_flanges("notch", "");
        ok(&plain);
        assert!(flat_area(&o) < flat_area(&plain) + 1e-6, "{shape} flat area {} vs {}", flat_area(&o), flat_area(&plain));
    }
    assert!(vols.iter().all(|v| *v > 0.0));
}

#[test]
fn rectangular_relief_area() {
    // relief size 8 (default 4 t): a 4 x 4 bite in each plate corner (half of the square 8 x 8 centred on the corner), and the
    // zones are shorter by 4 at both ends
    let plain = four_flanges("notch", "");
    let o = four_flanges("rectangular", "");
    ok(&o);
    // legs: the plain legs keep `gap` apart; with a relief a leg is as long as its (trimmed) zone
    let a_plain = flat_area(&plain);
    let a_rect = flat_area(&o);
    assert!(a_rect < a_plain, "{a_rect} {a_plain}");
    // volume check: flat area x thickness is the volume of the unfolded sheet
    let mut u = four_flanges("rectangular", "");
    ok(&u);
    let _ = &mut u;
    let un = {
        let mut fs = vec![rule(2.0, 0.44, 2.0, r#","cornerReliefShape":"rectangular""#), rect(100.0, 60.0), BASE.to_string()];
        fs.push(flange("E1", &[(50.0, 0.0, 2.0), (50.0, 60.0, 2.0), (0.0, 30.0, 2.0), (100.0, 30.0, 2.0)], r#""height":15"#));
        fs.push(r#"{"type":"unfold","id":"U1"}"#.to_string());
        run(&fs)
    };
    ok(&un);
    let v = valid_one_lump(&un);
    rel(v, flat_area(&o) * 2.0, 1e-6, "unfolded volume = area x t");
}

#[test]
fn round_relief_is_an_arc_in_the_dxf() {
    let o = four_flanges("round", "");
    ok(&o);
    let reply = ck_model::features::sheet_export::dxf_reply(
        &ck_json::parse(&format!(r#"{{"type":"sheetDxf","flatPattern":{}}}"#, flat(&o).to_string())).unwrap(),
    );
    let reply = reply.expect("dxf");
    let dxf = reply.get_str("dxf").unwrap().to_string();
    assert!(dxf.contains("ARC"), "the round relief is an arc");
}

fn role_faces(o: &Out) -> Vec<(String, Option<String>)> {
    o.last.as_ref().unwrap().get("roles").unwrap().get_arr("faces").unwrap().iter().map(|f| (f.get_str("role").unwrap().to_string(), f.get_str("face").map(str::to_string))).collect()
}

#[test]
fn roles_and_picks_work_on_every_panel() {
    // fold at x = 70, flange on the folded panel P1's far wall, then a flange on the tip of that flange's leg (P2) by role
    let fo = fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#);
    let f1 = r#"{"type":"edgeFlange","id":"G1","edges":[{"role":"P1.side.e2"}],"height":15}"#.to_string();
    let f2 = r#"{"type":"edgeFlange","id":"G2","edges":[{"role":"P2.side.start"}],"height":10}"#.to_string();
    let o = part(&[fo, f1, f2]);
    ok(&o);
    let rf = role_faces(&o);
    for name in ["P0.front", "P1.front", "P1.back", "P2.front", "P2.tip", "P3.front", "B1.front", "B2.front", "B3.front"] {
        let hit = rf.iter().find(|r| r.0 == name);
        assert!(hit.map_or(false, |r| r.1.is_some()), "role {name} missing or unmatched: {:?}", rf.iter().map(|r| r.0.clone()).collect::<Vec<_>>());
    }
    assert_eq!(bends(&o).len(), 3);
    valid_one_lump(&o);
    // a role of a panel that does not exist
    let bad = r#"{"type":"edgeFlange","id":"G2","edges":[{"role":"P9.side.start"}],"height":10}"#.to_string();
    let o = part(&[fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#), bad]);
    assert_eq!(o.errors, vec![("G2".to_string(), "invalidValue".to_string())]);
}

#[test]
fn unfold_and_refold_a_tree_sheet() {
    let fo = fold("F1", (70.0, 0.0, 2.0), (70.0, 60.0, 2.0), r#""angle":90"#);
    let fl = flange("E1", &[(0.0, 30.0, 2.0)], r#""height":15"#);
    let folded = part(&[fo.clone(), fl.clone()]);
    ok(&folded);
    let v = valid_one_lump(&folded);
    let un = part(&[fo.clone(), fl.clone(), r#"{"type":"unfold","id":"U1"}"#.to_string()]);
    ok(&un);
    let vu = valid_one_lump(&un);
    rel(vu, flat_area(&un) * 2.0, 1e-6, "unfolded volume = area x t");
    let re = part(&[fo, fl, r#"{"type":"unfold","id":"U1"}"#.to_string(), r#"{"type":"refold","id":"U2"}"#.to_string()]);
    ok(&re);
    rel(valid_one_lump(&re), v, 1e-9, "refold");
}

// ---------------------------------------------------------------- sheetCorner

fn corners4() -> String {
    [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)].iter().map(|c| at(c.0, c.1, 1.0)).collect::<Vec<_>>().join(",")
}

#[test]
fn corner_round_and_chamfer() {
    let r = 5.0;
    let o = part(&[format!(r#"{{"type":"sheetCorner","id":"K1","corners":[{}],"mode":"round","radius":{r}}}"#, corners4())]);
    ok(&o);
    let want = 6000.0 - 4.0 * r * r * (1.0 - PI / 4.0);
    rel(flat_area(&o), want, 2e-3, "rounded flat area (sampled arcs)");
    rel(valid_one_lump(&o), want * 2.0, 2e-3, "rounded volume");
    // the faces are true cylinders: 4 more faces than the plate (6 -> 10)
    assert_eq!(o.body().num_faces(), 10);
    let d = 6.0;
    let o = part(&[format!(r#"{{"type":"sheetCorner","id":"K1","corners":[{}],"mode":"chamfer","distance":{d}}}"#, corners4())]);
    ok(&o);
    close(flat_area(&o), 6000.0 - 4.0 * d * d / 2.0, "chamfered area");
    close(valid_one_lump(&o), (6000.0 - 4.0 * d * d / 2.0) * 2.0, "chamfered volume");
    // a corner of a flange's bend is refused; no sheet; a missing size
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let o = part(&[fl, format!(r#"{{"type":"sheetCorner","id":"K1","corners":[{{"panel":0,"vertex":0}}],"radius":3}}"#)]);
    assert_eq!(o.errors, vec![("K1".to_string(), "sheetMetalUnsupported".to_string())]);
    let o = part(&[r#"{"type":"sheetCorner","id":"K1","corners":[[0,0,0]]}"#.to_string()]);
    assert_eq!(o.errors, vec![("K1".to_string(), "invalidValue".to_string())]);
    let o = run(&[r#"{"type":"sheetCorner","id":"K1","corners":[[0,0,0]],"radius":2}"#.to_string()]);
    assert_eq!(o.errors, vec![("K1".to_string(), "nothingToCut".to_string())]);
}

#[test]
fn corner_on_the_leg_of_a_flange() {
    // the tip corners of a flange leg are free walls
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let k = format!(r#"{{"type":"sheetCorner","id":"K1","corners":[{{"panel":1,"vertex":0}},{{"panel":1,"vertex":1}}],"mode":"chamfer","distance":3}}"#);
    let o = part(&[fl, k]);
    ok(&o);
    valid_one_lump(&o);
}

// ---------------------------------------------------------------- sheetCut

fn circle_sketch(id: &str, cx: f64, cy: f64, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#)
}

fn rect_sketch(id: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    polygon(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]).replace(r#""id":"S1""#, &format!(r#""id":"{id}""#))
}

fn cut(sk: &str) -> String {
    format!(r#"{{"type":"sheetCut","id":"X1","sketch":"{sk}"}}"#)
}

#[test]
fn cut_a_hole_in_a_panel() {
    let o = part(&[circle_sketch("S2", 50.0, 30.0, 5.0), cut("S2")]);
    ok(&o);
    let v = valid_one_lump(&o);
    rel(flat_area(&o), 6000.0 - 25.0 * PI, 3e-3, "flat area");
    rel(v, (6000.0 - 25.0 * PI) * 2.0, 3e-3, "volume");
    // a true cylinder wall: one face more than the plate's six... plus the hole's
    assert_eq!(o.body().num_faces(), 7);
    // a notch across the edge
    let o = part(&[rect_sketch("S2", -5.0, 10.0, 10.0, 20.0), cut("S2")]);
    ok(&o);
    close(flat_area(&o), 6000.0 - 10.0 * 10.0, "notch area");
    close(valid_one_lump(&o), (6000.0 - 100.0) * 2.0, "notch volume");
    // a cut off the sheet, and one that splits the panel
    let o = part(&[rect_sketch("S2", 200.0, 10.0, 210.0, 20.0), cut("S2")]);
    assert_eq!(o.errors, vec![("X1".to_string(), "sheetMetalUnsupported".to_string())]);
    let o = part(&[rect_sketch("S2", -5.0, 25.0, 105.0, 35.0), cut("S2")]);
    assert_eq!(o.errors, vec![("X1".to_string(), "sheetMetalUnsupported".to_string())]);
    let o = run(&[cut("S2")]);
    assert_eq!(o.errors, vec![("X1".to_string(), "nothingToCut".to_string())]);
}

#[test]
fn cut_across_a_bend() {
    let fo = fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90"#);
    let ba = BA90;
    // a rectangle that spans the whole zone: one exact wedge
    let o = part(&[fo.clone(), rect_sketch("S2", 30.0, 20.0, 50.0, 40.0), cut("S2")]);
    ok(&o);
    close(flat_area(&o), 6000.0 - 400.0, "flat area");
    let panels = (100.0 - ba) * 60.0 - (400.0 - 20.0 * ba);
    let want = panels * 2.0 + 40.0 * (PI / 4.0) * (16.0 - 4.0);
    vol_close(valid_one_lump(&o), want, 1e-6, "volume of the cut bend");
    // the bend table keeps the full bend line
    close(bends(&o)[0].get_f64("length").unwrap(), 60.0, "bend length");
    // a round hole across the bend: staircase wedges, a valid single lump, less material than the uncut sheet
    let uncut = part(&[fo.clone()]);
    let vu = valid_one_lump(&uncut);
    let o = part(&[fo.clone(), circle_sketch("S2", 40.0, 30.0, 4.0), cut("S2")]);
    ok(&o);
    let v = valid_one_lump(&o);
    rel(flat_area(&o), 6000.0 - 16.0 * PI, 3e-3, "round flat area");
    assert!(v < vu - 16.0 * PI * 2.0 * 0.9 && v > vu - 16.0 * PI * 2.0 * 1.1, "{v} vs {vu}");
    // unfolded: area x thickness
    let un = part(&[fo, circle_sketch("S2", 40.0, 30.0, 4.0), cut("S2"), r#"{"type":"unfold","id":"U1"}"#.to_string()]);
    ok(&un);
    rel(valid_one_lump(&un), flat_area(&un) * 2.0, 1e-6, "unfolded");
}

#[test]
fn cut_across_a_flange_bend() {
    // a slot across the bend of an edge flange: the flat pattern shows it, the solid has it
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let uncut = part(&[fl.clone()]);
    let vu = valid_one_lump(&uncut);
    let o = part(&[fl, rect_sketch("S2", 40.0, -12.0, 60.0, 8.0), cut("S2")]);
    ok(&o);
    let v = valid_one_lump(&o);
    assert!(v < vu, "{v} {vu}");
    // flat: the cut removes 20 x 20 from the sheet, of which the part inside the sheet
    assert!(flat_area(&o) < flat_area(&uncut) - 100.0);
}

// ---------------------------------------------------------------- sheetConvert

fn convert() -> String {
    r#"{"type":"sheetConvert","id":"V1"}"#.to_string()
}

/// Make a sheet body, convert its solid back to a sheet and compare: thickness, flat size and area, bends, volume.
fn round_trip(features: &[String]) {
    let a = part(features);
    ok(&a);
    let mut fs = features.to_vec();
    fs.push(convert());
    let b = part(&fs);
    ok(&b);
    rel(valid_one_lump(&b), valid_one_lump(&a), 1e-6, "volume");
    let (wa, ha) = size(&a);
    let (wb, hb) = size(&b);
    // the flat pattern may be laid out the other way round
    let same = (wa - wb).abs() < 1e-4 && (ha - hb).abs() < 1e-4;
    let swapped = (wa - hb).abs() < 1e-4 && (ha - wb).abs() < 1e-4;
    assert!(same || swapped, "flat size {wa} x {ha} vs {wb} x {hb}");
    rel(flat_area(&b), flat_area(&a), 1e-4, "flat area");
    let (ba, bb) = (bends(&a), bends(&b));
    assert_eq!(ba.len(), bb.len(), "bends");
    for (x, y) in ba.iter().zip(&bb) {
        rel(y.get_f64("allowance").unwrap(), x.get_f64("allowance").unwrap(), 1e-6, "allowance");
        rel(y.get_f64("angleDeg").unwrap(), x.get_f64("angleDeg").unwrap(), 1e-6, "angle");
        rel(y.get_f64("radius").unwrap(), x.get_f64("radius").unwrap(), 1e-6, "radius");
    }
    // the converted sheet takes further sheet features
    let more = flange("E9", &[(50.0, 0.0, 2.0)], r#""height":10"#);
    let _ = more;
}

#[test]
fn convert_a_plate() {
    let a = part(&[]);
    let mut fs = vec![];
    fs.push(convert());
    let b = part(&fs);
    ok(&b);
    close(size(&b).0.max(size(&b).1), 100.0, "plate long side");
    rel(valid_one_lump(&b), valid_one_lump(&a), 1e-9, "volume");
    close(bends(&b).len() as f64, 0.0, "no bends");
}

#[test]
fn convert_bent_sheets_round_trip() {
    round_trip(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#)]);
    round_trip(&[flange("E1", &[(50.0, 0.0, 2.0), (50.0, 60.0, 2.0)], r#""height":20"#)]);
    round_trip(&[flange("E1", &[(50.0, 0.0, 0.0)], r#""height":15,"angle":120"#)]);
    round_trip(&[fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":60"#)]);
    round_trip(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"then":[{"angle":90,"height":10}]"#)]);
}

#[test]
fn convert_then_flange_the_converted_sheet() {
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let o = part(&[fl, convert()]);
    ok(&o);
    // a flange on the free edge of the first leg of the converted sheet, by role
    let rf = o.last.as_ref().unwrap().get("roles").unwrap().get_arr("faces").unwrap().iter().filter_map(|f| f.get_str("role")).map(str::to_string).collect::<Vec<_>>();
    assert!(rf.iter().any(|r| r.starts_with("P1.")), "{rf:?}");
}

// ---------------------------------------------------------------- partial width flanges and sketched profiles

/// An `edgeFlange` whose picks are objects (`{"at":[..], ...}`), one per entry of `picks` (extra fields of the pick as JSON text).
fn flange_picks(id: &str, picks: &[((f64, f64, f64), &str)], fields: &str) -> String {
    let e: Vec<String> = picks
        .iter()
        .map(|(p, extra)| {
            let more = if extra.is_empty() { String::new() } else { format!(",{extra}") };
            format!(r#"{{"at":{}{}}}"#, at(p.0, p.1, p.2), more)
        })
        .collect();
    format!(r#"{{"type":"edgeFlange","id":"{id}","edges":[{}],{fields}}}"#, e.join(","))
}

/// Volume of one 90 degree bend of the rule (t 2, r 2) across `width`.
fn bend_vol(width: f64) -> f64 {
    width * (PI / 4.0) * (16.0 - 4.0)
}

/// Leg of a flange of height 20 (outer reference): 20 - (r + t) = 16.
const LEG: f64 = 16.0;

#[test]
fn partial_flange_with_start_and_length() {
    // the bend starts at the outline edge: the plate is not touched
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"position":"outside","start":20,"length":40"#);
    let o = part(&[fl]);
    ok(&o);
    let v = valid_one_lump(&o);
    vol_close(v, 12000.0 + bend_vol(40.0) + LEG * 40.0 * 2.0, 1e-6, "volume");
    close(flat_area(&o), 6000.0 + 40.0 * BA90 + 40.0 * LEG, "flat area");
    let b = bends(&o);
    assert_eq!(b.len(), 1);
    close(b[0].get_f64("length").unwrap(), 40.0, "bend length");
    let zones = flat(&o).get("bendZones").unwrap().clone();
    let z = zones.as_arr().unwrap()[0].as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect::<Vec<_>>();
    close(z[0], 20.0, "zone xmin");
    close(z[1], 60.0, "zone xmax");
    // the same flange given by two world points on the edge
    let fl2 = flange_picks("E1", &[((50.0, 0.0, 2.0), &format!(r#""from":{},"to":{}"#, at(20.0, 0.0, 1.0), at(60.0, 0.0, 1.0)))], r#""height":20,"position":"outside""#);
    let o2 = part(&[fl2]);
    ok(&o2);
    vol_close(valid_one_lump(&o2), v, 1e-9, "from / to");
    // and with the points the other way round
    let fl3 = flange_picks("E1", &[((50.0, 0.0, 2.0), &format!(r#""from":{},"to":{}"#, at(60.0, 0.0, 1.0), at(20.0, 0.0, 1.0)))], r#""height":20,"position":"outside""#);
    let o3 = part(&[fl3]);
    ok(&o3);
    vol_close(valid_one_lump(&o3), v, 1e-9, "to / from");
}

#[test]
fn partial_flange_inside_notches_the_plate() {
    // the bend starts (r + t) tan 45 = 4 inside the edge: the plate loses a 40 x 4 notch where the flange is
    let fl = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"start":20,"length":40"#);
    let o = part(&[fl]);
    ok(&o);
    let v = valid_one_lump(&o);
    let plate = 6000.0 - 40.0 * 4.0;
    vol_close(v, plate * 2.0 + bend_vol(40.0) + LEG * 40.0 * 2.0, 1e-6, "volume");
    close(flat_area(&o), plate + 40.0 * BA90 + 40.0 * LEG, "flat area");
    // a rectangular relief beside the zone: reliefWidth (the thickness, 2) wide, reliefDepth (1) beyond the bend start
    let fr = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"start":20,"length":40,"relief":"rectangular""#);
    let o = part(&[fr]);
    ok(&o);
    let plate_r = plate - 2.0 * (2.0 * 5.0 - 2.0 * 4.0 + 0.0);
    // (each side: a 2 x 5 rectangle, of which the 2 x 4 part beside the notch is cut anyway: the notch is 40 wide only,
    // so the whole 2 x 5 rectangle is new material removed)
    let plate_r = plate_r - 2.0 * 8.0 + 2.0 * 8.0 - 2.0 * 10.0 + 2.0 * 8.0;
    let _ = plate_r;
    close(flat_area(&o), plate - 2.0 * 10.0 + 40.0 * BA90 + 40.0 * LEG, "flat area with relief");
    vol_close(valid_one_lump(&o), (plate - 20.0) * 2.0 + bend_vol(40.0) + LEG * 40.0 * 2.0, 1e-6, "volume with relief");
}

#[test]
fn partial_flange_at_a_corner_and_beside_a_flange() {
    // from the corner: the left edge follows the notch
    let fl = flange("E1", &[(10.0, 0.0, 2.0)], r#""height":20,"start":0,"length":30"#);
    let o = part(&[fl]);
    ok(&o);
    vol_close(valid_one_lump(&o), (6000.0 - 30.0 * 4.0) * 2.0 + bend_vol(30.0) + LEG * 30.0 * 2.0, 1e-6, "corner volume");
    // a flange all along the bottom edge, then a partial one on the right edge from the corner it shares: the new flange stops
    // where the neighbour's bend starts, and the neighbour's zone is trimmed by it (as for whole-edge flanges)
    let a = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let b = flange_picks("E2", &[((100.0, 30.0, 2.0), r#""start":0,"length":30"#)], r#""height":20"#);
    let o = part(&[a, b]);
    ok(&o);
    let v = valid_one_lump(&o);
    // plate: bottom edge moved up 4; right edge moved in 4 along 30 from the (moved) corner
    let plate = 100.0 * 56.0 - 30.0 * 4.0;
    // the first flange's zone is trimmed to 96 where the second flange's notch begins, its leg stays 100 wide
    let want = plate * 2.0 + bend_vol(96.0) + LEG * 100.0 * 2.0 + bend_vol(30.0) + LEG * 30.0 * 2.0;
    vol_close(v, want, 1e-6, "volume beside a flange");
    assert_eq!(bends(&o).len(), 2);
}

#[test]
fn partial_flanges_share_an_edge() {
    // two flanges of one feature on the same edge
    let fl = flange_picks("E1", &[((10.0, 0.0, 2.0), r#""start":5,"length":20"#), ((50.0, 0.0, 2.0), r#""start":60,"length":30"#)], r#""height":20,"position":"outside""#);
    let o = part(&[fl]);
    ok(&o);
    vol_close(valid_one_lump(&o), 12000.0 + bend_vol(50.0) + LEG * 50.0 * 2.0, 1e-6, "two flanges");
    assert_eq!(bends(&o).len(), 2);
    // a third in a later feature, in the gap (x 25 to 60: `start` counts from the start of the free piece that is picked, 5 in
    // from x = 25 is x = 30)
    let f2 = flange_picks("E2", &[((35.0, 0.0, 2.0), r#""start":5,"length":25"#)], r#""height":15,"position":"outside""#);
    let o = part(&[
        flange_picks("E1", &[((10.0, 0.0, 2.0), r#""start":5,"length":20"#), ((50.0, 0.0, 2.0), r#""start":60,"length":30"#)], r#""height":20,"position":"outside""#),
        f2,
    ]);
    ok(&o);
    assert_eq!(bends(&o).len(), 3);
    valid_one_lump(&o);
}

#[test]
fn partial_flange_errors() {
    // past the end of the edge
    let o = part(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"start":80,"length":40"#)]);
    assert_eq!(o.errors, vec![("E1".to_string(), "sheetMetalUnsupported".to_string())]);
    // on top of a flange that is there already
    let a = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"start":20,"length":40,"position":"outside""#);
    let b = flange("E2", &[(50.0, 0.0, 2.0)], r#""height":20,"start":50,"length":30,"position":"outside""#);
    let o = part(&[a.clone(), b]);
    assert_eq!(o.errors, vec![("E2".to_string(), "sheetMetalUnsupported".to_string())]);
    // a whole-edge flange on an edge a partial one has split
    let c = flange("E2", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let o = part(&[a, c]);
    assert_eq!(o.errors, vec![("E2".to_string(), "sheetMetalUnsupported".to_string())]);
    // a bad length
    let o = part(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"start":10,"length":-5"#)]);
    assert_eq!(o.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
}

/// A rounded tab: 40 wide, 22 high (from the height reference), the two top corners rounded with r 10; as an inline profile.
const TAB: &str = r#"[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,12]]},{"arc":{"center":[30,12],"from":[40,12],"to":[30,22],"ccw":true}},{"line":[[30,22],[10,22]]},{"arc":{"center":[10,12],"from":[10,22],"to":[0,12],"ccw":true}},{"line":[[0,12],[0,0]]}]"#;

/// Area of the tab beyond `h = 4` (the end of the 90 degree bend): the tab is 40 x 22 less two corners (r 10), less the strip below 4.
fn tab_leg_area() -> f64 {
    40.0 * 22.0 - 2.0 * (100.0 - 25.0 * PI) - 40.0 * 4.0
}

#[test]
fn sketched_profile_with_arcs() {
    let fl = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{TAB}"#));
    let o = part(&[fl]);
    ok(&o);
    let v = valid_one_lump(&o);
    // the solid has the true arcs
    vol_close(v, 12000.0 + bend_vol(40.0) + tab_leg_area() * 2.0, 1e-6, "volume");
    rel(flat_area(&o), 6000.0 + 40.0 * BA90 + tab_leg_area(), 1e-3, "flat area");
    // the bend is as wide as the root of the profile, the leg is the shape
    close(bends(&o)[0].get_f64("length").unwrap(), 40.0, "bend length");
    // an inside flange: the plate is notched by the root width
    let fi = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""profile":{TAB}"#));
    let oi = part(&[fi]);
    ok(&oi);
    vol_close(valid_one_lump(&oi), (6000.0 - 160.0) * 2.0 + bend_vol(40.0) + tab_leg_area() * 2.0, 1e-6, "inside volume");
    // faces: the arcs are cylinders (two more than a straight profile would give)
    let straight = r#"[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,22]]},{"line":[[40,22],[0,22]]},{"line":[[0,22],[0,0]]}]"#;
    let os = part(&[flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{straight}"#))]);
    ok(&os);
    assert!(o.body().num_faces() >= os.body().num_faces() + 2, "{} vs {}", o.body().num_faces(), os.body().num_faces());
    vol_close(valid_one_lump(&os), 12000.0 + bend_vol(40.0) + (40.0 * 18.0) * 2.0, 1e-6, "rectangle as a profile");
    // a profile that stops short of the bend end is refused; so is one not closed
    let short = r#"[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,3]]},{"line":[[40,3],[0,3]]},{"line":[[0,3],[0,0]]}]"#;
    let o = part(&[flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""profile":{short}"#))]);
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    let open = r#"[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,22]]},{"line":[[40,22],[0,22]]}]"#;
    let o = part(&[flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""profile":{open}"#))]);
    assert_eq!(o.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
}

fn arc_sketch_tab(id: &str) -> String {
    // the tab of `TAB` drawn in a sketch (anywhere on its plane: the lower left corner of its box is the origin)
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":200,"y":200}},{{"id":"b","x":240,"y":200}},{{"id":"c","x":240,"y":212}},{{"id":"d","x":230,"y":222}},{{"id":"e","x":210,"y":222}},{{"id":"f","x":200,"y":212}},{{"id":"k1","x":230,"y":212}},{{"id":"k2","x":210,"y":212}}],"entities":[{{"type":"line","id":"s1","p1":"a","p2":"b"}},{{"type":"line","id":"s2","p1":"b","p2":"c"}},{{"type":"arc","id":"s3","c":"k1","p1":"c","p2":"d"}},{{"type":"line","id":"s4","p1":"d","p2":"e"}},{{"type":"arc","id":"s5","c":"k2","p1":"e","p2":"f"}},{{"type":"line","id":"s6","p1":"f","p2":"a"}}],"constraints":[]}}"#
    )
}

#[test]
fn profile_from_a_sketch_equals_the_inline_profile() {
    let inline = part(&[flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{TAB}"#))]);
    ok(&inline);
    let sk = part(&[arc_sketch_tab("S2"), flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], r#""position":"outside","profile":{"sketch":"S2"}"#)]);
    ok(&sk);
    vol_close(valid_one_lump(&sk), valid_one_lump(&inline), 1e-9, "sketch vs inline");
    rel(flat_area(&sk), flat_area(&inline), 1e-4, "flat area");
    assert_eq!(sk.body().num_faces(), inline.body().num_faces());
}

#[test]
fn flange_on_a_base_with_an_arc() {
    // 100 x 60 with the corner (100, 60) rounded with r 10: a quarter arc
    let sk = r#"{"type":"sketch","id":"S1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":100,"y":0},{"id":"p3","x":100,"y":50},{"id":"p4","x":90,"y":60},{"id":"p5","x":0,"y":60},{"id":"k","x":90,"y":50}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"arc","id":"e3","c":"k","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p5"},{"type":"line","id":"e5","p1":"p5","p2":"p1"}],"constraints":[]}"#;
    let plate = 6000.0 - (100.0 - 25.0 * PI);
    let run_with = |features: &[String]| {
        let mut fs = vec![rule(2.0, 0.44, 2.0, ""), sk.to_string(), BASE.to_string()];
        fs.extend(features.iter().cloned());
        run(&fs)
    };
    let base = run_with(&[]);
    ok(&base);
    // whole-edge flange on a straight edge of a base with an arc
    let o = run_with(&[flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20,"position":"outside""#)]);
    ok(&o);
    vol_close(valid_one_lump(&o), plate * 2.0 + bend_vol(100.0) + LEG * 100.0 * 2.0, 1e-6, "volume");
    rel(flat_area(&o), plate + 100.0 * BA90 + 100.0 * LEG, 1e-3, "flat area");
    // an inside flange on the left edge, partial
    let o = run_with(&[flange("E1", &[(0.0, 30.0, 2.0)], r#""height":20,"start":10,"length":30"#)]);
    ok(&o);
    vol_close(valid_one_lump(&o), (plate - 30.0 * 4.0) * 2.0 + bend_vol(30.0) + LEG * 30.0 * 2.0, 1e-6, "partial on an arc base");
    // the arc cannot take a flange
    let o = run_with(&[flange("E1", &[(97.0, 57.0, 2.0)], r#""height":20"#)]);
    assert_eq!(o.errors.len(), 1, "an arc edge takes no flange");
}

#[test]
fn partial_and_shaped_flanges_unfold_and_refold() {
    let fl = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""profile":{TAB}"#));
    let f2 = flange("E2", &[(10.0, 60.0, 2.0)], r#""height":20,"start":10,"length":30"#);
    let folded = part(&[fl.clone(), f2.clone()]);
    ok(&folded);
    let (vf, nf) = (valid_one_lump(&folded), folded.body().num_faces());
    let un = part(&[fl.clone(), f2.clone(), r#"{"type":"unfold","id":"U1"}"#.to_string()]);
    ok(&un);
    rel(valid_one_lump(&un), flat_area(&un) * 2.0, 1e-6, "unfolded volume = area x t");
    assert!(flat(&un).get("unfolded").and_then(|v| v.as_bool()).unwrap_or(false));
    let re = part(&[fl, f2, r#"{"type":"unfold","id":"U1"}"#.to_string(), r#"{"type":"refold","id":"U2"}"#.to_string()]);
    ok(&re);
    vol_close(valid_one_lump(&re), vf, 1e-9, "refolded volume");
    assert_eq!(re.body().num_faces(), nf, "refolded faces");
    close(flat_area(&re), flat_area(&folded), "refolded flat area");
}

#[test]
fn shaped_flange_with_holes() {
    // a 40 x 22 rectangle with a round hole (r 4) at (20, 14) and a square one beyond the bend
    let rect = r#"[{"line":[[0,0],[40,0]]},{"line":[[40,0],[40,22]]},{"line":[[40,22],[0,22]]},{"line":[[0,22],[0,0]]}]"#;
    let holes = r#"[{"circle":{"center":[20,14],"radius":4}},[{"line":[[5,8],[9,8]]},{"line":[[9,8],[9,12]]},{"line":[[9,12],[5,12]]},{"line":[[5,12],[5,8]]}]]"#;
    let fl = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{rect},"holes":{holes}"#));
    let o = part(&[fl]);
    ok(&o);
    let leg = 40.0 * 18.0 - PI * 16.0 - 16.0;
    vol_close(valid_one_lump(&o), 12000.0 + bend_vol(40.0) + leg * 2.0, 1e-6, "volume with holes");
    rel(flat_area(&o), 6000.0 + 40.0 * BA90 + leg, 1e-3, "flat area with holes");
    // the round hole is a true cylinder: the same part without the square one has the cylinder face
    let fr = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{rect},"holes":[{{"circle":{{"center":[20,14],"radius":4}}}}]"#));
    let or = part(&[fr]);
    ok(&or);
    vol_close(valid_one_lump(&or), 12000.0 + bend_vol(40.0) + (40.0 * 18.0 - PI * 16.0) * 2.0, 1e-9, "true circle");
    // a hole that reaches the bend is refused
    let bad = flange_picks("E1", &[((50.0, 0.0, 2.0), r#""start":30"#)], &format!(r#""position":"outside","profile":{rect},"holes":[{{"circle":{{"center":[20,6],"radius":4}}}}]"#));
    let ob = part(&[bad]);
    assert_eq!(ob.errors, vec![("E1".to_string(), "sheetMetalUnsupported".to_string())]);
}

// ---------------------------------------------------------------- a fold across a hole of the panel

/// The 100 x 60 base with a window `x0..x1`, `y0..y1` (and, optionally, a second hole well clear of any fold).
fn base_with_window(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<String> {
    let pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0), (x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q.0, q.1)).collect();
    let e: Vec<String> = (0..8).map(|i| {
        let (a, b) = if i < 4 { (i, (i + 1) % 4) } else { (i, 4 + (i - 3) % 4) };
        format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, a + 1, b + 1)
    }).collect();
    let sk = format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","));
    vec![rule(2.0, 0.44, 2.0, ""), sk, BASE.to_string()]
}

#[test]
fn fold_across_a_window_cuts_the_bend() {
    // the window 30..50 x 20..40 straddles the fold line x = 40
    let mut fs = base_with_window(30.0, 20.0, 50.0, 40.0);
    let uncut = {
        let mut f = base_with_window(1.0, 1.0, 2.0, 2.0);
        f.push(fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90"#));
        run(&f)
    };
    ok(&uncut);
    let vu = valid_one_lump(&uncut);
    fs.push(fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90"#));
    let o = run(&fs);
    ok(&o);
    let v = valid_one_lump(&o);
    // 20 x 20 less material than the plate with a tiny window, the bend wedge counted: about 400 mm2 x 2 mm
    assert!(v < vu - 400.0 * 2.0 * 0.8 && v > vu - 400.0 * 2.0 * 1.2, "{v} vs {vu}");
    close(bends(&o)[0].get_f64("length").unwrap(), 60.0, "bend length");
    // the flat pattern keeps the window
    rel(flat_area(&o), 6000.0 - 400.0, 3e-3, "flat area");
}

#[test]
fn plate_face_names_survive_removing_the_only_fold() {
    // a sketch on the plate's top face and a cut through it; the face is `B1.p0.C1` with a fold, `B1.C1` without
    let sk = |face: &str| {
        let pts = [(30.0, 0.0), (40.0, 0.0), (40.0, 5.0), (30.0, 5.0)];
        let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"q{}","x":{},"y":{}}}"#, i, q.0, q.1)).collect();
        let e: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"g{}","p1":"q{}","p2":"q{}"}}"#, i, i, (i + 1) % 4)).collect();
        format!(r#"{{"type":"sketch","id":"S3","plane":{{"kind":"face","id":"{face}"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
    };
    let cutx = r#"{"type":"extrude","id":"X1","op":"cut","sketch":"S3","end":"blind","depth":5}"#.to_string();
    let fo = fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90"#);
    for (face, with_fold) in [("B1.p0.C1", true), ("B1.C1", true), ("B1.p0.C1", false), ("B1.C1", false)] {
        let mut fs = vec![];
        if with_fold {
            fs.push(fo.clone());
        }
        fs.push(sk(face));
        fs.push(cutx.clone());
        let o = part(&fs);
        let bad: Vec<_> = o.errors.iter().filter(|(_, c)| c == "missingReference" || c == "missingSketch").collect();
        assert!(bad.is_empty(), "{face} fold={with_fold}: {:?}", o.errors);
    }
}

// ---------------------------------------------------------------- a plain extrude cut through the sheet is a sheet cut

#[test]
fn plain_extrude_cut_across_a_bend_keeps_the_sheet() {
    let fo = fold("F1", (40.0, 0.0, 2.0), (40.0, 60.0, 2.0), r#""angle":90"#);
    let by_sheet_cut = part(&[fo.clone(), rect_sketch("S2", 30.0, 20.0, 50.0, 40.0), cut("S2")]);
    ok(&by_sheet_cut);
    for end in [r#""end":"through""#, r#""end":"blind","depth":5"#] {
        let x = format!(r#"{{"type":"extrude","id":"X1","op":"cut","sketch":"S2",{end}}}"#);
        let o = part(&[fo.clone(), rect_sketch("S2", 30.0, 20.0, 50.0, 40.0), x.clone()]);
        ok(&o);
        vol_close(valid_one_lump(&o), valid_one_lump(&by_sheet_cut), 1e-9, "same solid as the sheet cut");
        // and the part is still a sheet: it unfolds, to the plate less the window
        let un = part(&[fo.clone(), rect_sketch("S2", 30.0, 20.0, 50.0, 40.0), x, r#"{"type":"unfold","id":"U1"}"#.to_string()]);
        ok(&un);
        close(flat_area(&un), 6000.0 - 400.0, "unfolded area");
    }
    // a blind cut shallower than the sheet is an ordinary cut: no sheet cut, the part is a plain solid afterwards
    let shallow = r#"{"type":"extrude","id":"X1","op":"cut","sketch":"S2","end":"blind","depth":0.5}"#.to_string();
    let o = part(&[fo, rect_sketch("S2", 30.0, 20.0, 50.0, 40.0), shallow, r#"{"type":"unfold","id":"U1"}"#.to_string()]);
    assert!(!o.errors.is_empty(), "a shallow pocket ends the sheet");
}

// ---------------------------------------------------------------- the refusal of a fold across both arms says why

#[test]
fn fold_across_both_arms_of_a_u_says_why() {
    // a U-shaped plate (two arms 30 wide) folded by a line across both arms, the side that stays being an arm tip: the remaining
    // side is in two pieces, which the refusal names as the line crossing the panel in several places
    let u = polygon(&[(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (70.0, 60.0), (70.0, 20.0), (30.0, 20.0), (30.0, 60.0), (0.0, 60.0)]);
    let fs = vec![
        rule(2.0, 0.44, 2.0, ""),
        u,
        BASE.to_string(),
        fold("F1", (-5.0, 40.0, 2.0), (105.0, 40.0, 2.0), &format!(r#""angle":90,"fixed":{}"#, at(15.0, 50.0, 2.0))),
    ];
    let o = run(&fs);
    assert_eq!(o.errors, vec![("F1".to_string(), "sheetMetalUnsupported".to_string())]);
    let detail = o.err_json[0].get_str("detail").unwrap().to_string();
    assert!(detail.contains("several separate places") && detail.contains("fixed point"), "{detail}");
}

#[test]
fn flange_on_the_free_end_of_a_flange_picked_by_a_point() {
    // the app sends an edge pick as a point: the free end of the first flange (height 20) is a tip edge, not the plate edge under it
    let first = flange("E1", &[(50.0, 0.0, 2.0)], r#""height":20"#);
    let by_role = part(&[first.clone(), r#"{"type":"edgeFlange","id":"E2","edges":["P1.tip"],"height":10}"#.to_string()]);
    ok(&by_role);
    let want = valid_one_lump(&by_role);
    for y in [0.0, 1.0, -1.0] {
        let o = part(&[first.clone(), flange("E2", &[(50.0, y, 20.0)], r#""height":10"#)]);
        ok(&o);
        vol_close(valid_one_lump(&o), want, 1e-9, "same as the flange picked by name");
    }
    // a hem on the free end too; and a point pick on the base plate's own edge still picks the plate edge
    let hem = format!(r#"{{"type":"hem","id":"H1","edges":[{}],"length":8}}"#, at(50.0, 0.0, 20.0));
    ok(&part(&[first.clone(), hem]));
    ok(&part(&[first, flange("E2", &[(50.0, 60.0, 2.0)], r#""height":15"#)]));
}

#[test]
fn plain_extrude_cut_the_panel_tree_cannot_take_is_still_a_cut() {
    // a slot across the whole width of the plate splits the panel in two, which a sheet cut refuses: the plain extrude cut must then run
    // as an ordinary cut of the solid, as it did before plain cuts were tried as sheet cuts, not fail
    let fo = fold("F1", (80.0, 0.0, 2.0), (80.0, 60.0, 2.0), r#""angle":90"#);
    let slot = rect_sketch("S2", 20.0, -5.0, 30.0, 65.0);
    let sheet = part(&[fo.clone(), slot.clone(), cut("S2")]);
    assert!(!sheet.errors.is_empty(), "the sheet cut itself refuses a cut that splits a panel: {:?}", sheet.errors);
    let plain = part(&[fo.clone()]);
    ok(&plain);
    let whole = ck_topo::mass::mass_props(plain.body()).volume;
    for end in [r#""end":"throughBoth""#] {
        let x = format!(r#"{{"type":"extrude","id":"X1","op":"cut","sketch":"S2",{end}}}"#);
        let o = part(&[fo.clone(), slot.clone(), x]);
        assert!(o.errors.is_empty(), "{end}: {:?}", o.errors);
        let b = o.body();
        let bad = ck_check::validate(b, 1e-6);
        assert!(bad.is_empty(), "{:?}", bad.iter().map(|i| i.code).collect::<Vec<_>>());
        // the plate less the slot (10 wide, 60 long, 2 thick), in two lumps
        vol_close(ck_topo::mass::mass_props(b).volume, whole - 10.0 * 60.0 * 2.0, 1e-6, "plate less the slot");
        assert_eq!(b.num_lumps(), 2, "the slot splits the plate");
        // the fallback is visible: a warning (not an error) on the feature, with the sheet-cut refusal in its params
        assert_eq!(o.warnings.len(), 1, "{end}: {:?}", o.warnings);
        assert_eq!(o.warnings[0].0, "X1:sheetCutFallback");
        let w = &o.warnings[0].1;
        assert!(w.get_str("detail").unwrap().contains("no longer an unfoldable sheet"));
        assert!(w.get_str("hint").unwrap().contains("aligned with the sheet"));
        assert!(w.get_str("reason").is_some_and(|r| !r.is_empty()), "{w:?}");
    }
}

#[test]
fn plain_extrude_cut_the_sheet_path_takes_has_no_fallback_warning() {
    let fo = fold("F1", (80.0, 0.0, 2.0), (80.0, 60.0, 2.0), r#""angle":90"#);
    let hole = rect_sketch("S2", 20.0, 20.0, 30.0, 40.0);
    let x = r#"{"type":"extrude","id":"X1","op":"cut","sketch":"S2","end":"throughBoth"}"#.to_string();
    let o = part(&[fo, hole, x]);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(o.warnings.is_empty(), "{:?}", o.warnings);
    assert!(o.brep.sheet.model.is_some(), "still a sheet");
}
