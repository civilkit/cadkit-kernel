//! Sheet metal, stage 2 (`hem`, `fold`, `contourFlange`, chained bends of `edgeFlange`, `unfold` / `refold`, roles;
//! docs/sheet-metal.md). The expected numbers are the flat sizes, bend lines, zones and bounding boxes of black-box runs of a
//! reference sheet metal tool (rule: t = 2, k = 0.44, r = 2, base 100 x 60 unless noted); the folded solids are also checked
//! against their closed-form volume, validity and one lump.
use ck_model::features::Brep;
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
    body: Option<Body>,
    errors: Vec<(String, String)>,
    err_json: Vec<ck_json::Json>,
    last: Option<ck_json::Json>,
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
        err_json: r.errors.iter().map(|e| ck_json::Json::Obj(e.error.params.clone())).collect(),
        last,
        frame: r.sketches.first().map(|s| s.frame),
    }
}

fn rule(t: f64, k: f64, r: f64, extra: &str) -> String {
    format!(r#"{{"type":"sheetRules","id":"R1","name":"R","thickness":{t},"kFactor":{k},"bendRadius":{r}{extra}}}"#)
}

/// A closed rectangle sketch, the corners listed from `start` (0..3) so that the entity ids `e1..e4` follow the walls
/// differently.
fn sketch_from(w: f64, h: f64, start: usize) -> String {
    let c = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)];
    let pts: Vec<String> = (0..4).map(|i| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, c[(i + start) % 4].0, c[(i + start) % 4].1)).collect();
    let ents: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % 4 + 1)).collect();
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

fn sketch(w: f64, h: f64) -> String {
    sketch_from(w, h, 0)
}

/// An open chain of lines through `pts` (ids `e1..`), the sketch `S1`.
fn chain(pts: &[(f64, f64)]) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, q)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, q.0, q.1)).collect();
    let e: Vec<String> = (0..pts.len() - 1).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, i + 2)).collect();
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}

const BASE: &str = r#"{"type":"baseFlange","id":"B1","sketch":"S1"}"#;

fn frame_of() -> ck_core::linalg::Frame {
    run(&[sketch(100.0, 60.0)]).frame.expect("sketch frame")
}

/// A point on the sketch plane at `(x, y)` and height `h` above it, in world coordinates, as a feature pick.
fn at(x: f64, y: f64, h: f64) -> String {
    let f = frame_of();
    let p = f.o + f.x * x + f.y * y + f.z * h;
    format!("[{},{},{}]", p.x, p.y, p.z)
}

/// Rule + 100 x 60 base + the given features.
fn part(t: f64, k: f64, r: f64, extra_rule: &str, features: &[String]) -> Out {
    let mut fs = vec![rule(t, k, r, extra_rule), sketch(100.0, 60.0), BASE.to_string()];
    fs.extend(features.iter().cloned());
    run(&fs)
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6, "{what}: {a} vs {b}");
}

fn flat(o: &Out) -> ck_json::Json {
    o.last.as_ref().expect("a sheet entry").get("flatPattern").expect("flat pattern").clone()
}

fn size(o: &Out) -> (f64, f64) {
    let s = flat(o);
    let a = s.get_arr("flatSize").unwrap();
    (a[0].as_f64().unwrap(), a[1].as_f64().unwrap())
}

fn bend(o: &Out, k: usize) -> ck_json::Json {
    flat(o).get_arr("bends").unwrap()[k].clone()
}

fn zone(o: &Out, k: usize) -> Vec<f64> {
    flat(o).get_arr("bendZones").unwrap()[k].as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect()
}

fn pt(j: &ck_json::Json, key: &str) -> (f64, f64) {
    let a = j.get(key).unwrap().as_arr().unwrap();
    (a[0].as_f64().unwrap(), a[1].as_f64().unwrap())
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

fn vol_close(v: f64, want: f64, what: &str) {
    assert!((v - want).abs() < 1e-6 * want, "{what}: volume {v} vs {want}");
}

/// Extent of the body along the sketch frame axes: `(lo, hi)` per axis x, y, z (z: along the sketch normal).
fn fbox(o: &Out) -> ([f64; 3], [f64; 3]) {
    let f = o.frame.unwrap_or_else(frame_of);
    let (lo, hi) = ck_model::features::body_box(o.body.as_ref().unwrap());
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

// ---------------------------------------------------------------- hem

fn hem(fields: &str, edge: (f64, f64, f64)) -> String {
    format!(r#"{{"type":"hem","id":"H1","edges":[{}],{}}}"#, at(edge.0, edge.1, edge.2), fields)
}

#[test]
fn s2_hem_flat_pattern() {
    let o = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 2.0))]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 100.0, "width");
    close(h, 70.78601746169491, "length");
    let b = bend(&o, 0);
    close(b.get_f64("radius").unwrap(), 0.01, "radius");
    close(b.get_f64("allowance").unwrap(), 2.796017461694916, "allowance");
    close(b.get_f64("angleDeg").unwrap(), 180.0, "angle");
    close(b.get_f64("includedAngleDeg").unwrap(), 0.0, "included");
    assert_eq!(b.get_str("direction"), Some("down"));
    close(pt(&b, "start").1, -1.398008730847458, "bend line");
    let z = zone(&o, 0);
    close(z[2], -2.796017461694916, "zone low");
    close(z[3], 0.0, "zone high");
    // the solid: plate + a half ring of 100 wide + a leg of length - (r + t) laid back over the plate (bbox of the reference)
    let (r, t) = (0.01f64, 2.0f64);
    let want = 100.0 * 60.0 * t + (PI / 2.0) * ((r + t).powi(2) - r * r) * 100.0 + (10.0 - (r + t)) * t * 100.0;
    vol_close(valid_one_lump(&o), want, "hem");
    // curls under the plate (pick on the top face): z from -2.02 (the leg) to 2, y down to -(r + t)
    let (lo, hi) = fbox(&o);
    close(lo[2], -(2.0 * r + t), "hem leg below the plate");
    close(hi[2], t, "plate top");
    close(lo[1], -(r + t), "hem outside");
}

#[test]
fn hem_rule_from_probes() {
    // length reaches the outer face of the hem: leg = length - (gap + t); flat = 60 + allowance + leg
    for (len, want) in [(10.0, 70.78601746169491), (5.0, 65.78601746169491), (2.01, 62.796017461694916), (20.0, 80.78601746169491), (50.0, 110.78601746169491)] {
        let o = part(2.0, 0.44, 2.0, "", &[hem(&format!(r#""length":{len}"#), (50.0, 0.0, 2.0))]);
        ok(&o);
        close(size(&o).1, want, &format!("length {len}"));
    }
    // thickness 3
    let o = part(3.0, 0.44, 3.0, "", &[hem(r#""length":12"#, (50.0, 0.0, 3.0))]);
    ok(&o);
    close(size(&o).1, 73.16831822927443, "t3");
    close(bend(&o, 0).get_f64("allowance").unwrap(), 4.178318229274425, "t3 allowance");
    // k-factor 0.3
    let o = part(2.0, 0.3, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 2.0))]);
    close(size(&o).1, 69.90637151868978, "k0.3");
    // the gap sets the inside radius
    let o = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10,"gap":1"#, (50.0, 0.0, 2.0))]);
    ok(&o);
    close(size(&o).1, 72.90619418874881, "gap 1");
    close(bend(&o, 0).get_f64("radius").unwrap(), 1.0, "gap radius");
    valid_one_lump(&o);
    // the rule's hem gap is the default
    let o = part(2.0, 0.44, 2.0, r#","hemGap":0.5"#, &[hem(r#""length":10"#, (50.0, 0.0, 2.0))]);
    close(size(&o).1, 71.83539786195391, "hem gap 0.5");
    // minimum: length < gap + t is refused
    let o = part(2.0, 0.44, 2.0, "", &[hem(r#""length":2"#, (50.0, 0.0, 2.0))]);
    assert_eq!(o.errors, vec![("H1".to_string(), "flangeHeightTooLow".to_string())]);
}

#[test]
fn hem_side_and_flip() {
    let top = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 2.0))]);
    assert_eq!(bend(&top, 0).get_str("direction"), Some("down"));
    let bottom = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 0.0))]);
    assert_eq!(bend(&bottom, 0).get_str("direction"), Some("up"));
    let (lo, hi) = fbox(&bottom);
    bclose(hi[2], 4.02, "hem over the plate");
    bclose(lo[1], -2.01, "hem outside");
    vol_close(valid_one_lump(&bottom), 14232.601716115003, "hem volume");
    close(lo[2], 0.0, "plate bottom");
    let flip = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10,"flip":true"#, (50.0, 0.0, 2.0))]);
    assert_eq!(bend(&flip, 0).get_str("direction"), Some("up"));
    valid_one_lump(&bottom);
    valid_one_lump(&flip);
}

#[test]
fn two_hems_and_neighbours() {
    let two = format!(r#"{{"type":"hem","id":"H1","edges":[{},{}],"length":10}}"#, at(50.0, 0.0, 2.0), at(50.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[two]);
    ok(&o);
    close(size(&o).1, 81.57203492338982, "two hems");
    let want = 100.0 * 60.0 * 2.0 + 2.0 * ((PI / 2.0) * ((2.01f64).powi(2) - 0.01 * 0.01) * 100.0 + (10.0 - 2.01) * 2.0 * 100.0);
    vol_close(valid_one_lump(&o), want, "two hems");
    // adjacent hems collide: refused (the reference refuses them too)
    let adj = format!(r#"{{"type":"hem","id":"H1","edges":[{},{}],"length":10}}"#, at(50.0, 0.0, 2.0), at(0.0, 30.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[adj]);
    assert_eq!(o.errors, vec![("H1".to_string(), "sheetMetalUnsupported".to_string())]);
    // a hem on an edge next to an edge flange too
    let fl = format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{}],"height":15}}"#, at(50.0, 0.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[fl, hem(r#""length":10"#, (0.0, 30.0, 2.0))]);
    assert_eq!(o.errors, vec![("H1".to_string(), "sheetMetalUnsupported".to_string())]);
}

// ---------------------------------------------------------------- chained bends

fn flange(fields: &str, edge: (f64, f64, f64)) -> String {
    format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{}],{}}}"#, at(edge.0, edge.1, edge.2), fields)
}

fn chained(fields: &str) -> Out {
    part(2.0, 0.44, 2.0, "", &[flange(fields, (50.0, 0.0, 2.0))])
}

#[test]
fn then_chain_flat_pattern() {
    // two bends: the second flange is 10 high from its own outer sharp, the first leg stops where the next bend starts
    let o = chained(r#""height":20,"then":[{"height":10}]"#);
    ok(&o);
    close(size(&o).1, 83.04778684233861, "length");
    close(bend(&o, 0).get("start").unwrap().as_arr().unwrap()[1].as_f64().unwrap(), 1.7380532894153489, "bend 1 line");
    close(pt(&bend(&o, 1), "start").1, -14.785840131584651, "bend 2 line");
    close(pt(&bend(&o, 1), "end").0, 100.0, "bend 2 length");
    let (z0, z1) = (zone(&o, 0), zone(&o, 1));
    close(z0[2], -0.5238934211693023, "zone 1 low");
    close(z0[3], 4.0, "zone 1 high");
    close(z1[2], -17.0477868421693, "zone 2 low");
    close(z1[3], -12.523893421, "zone 2 high");
    assert_eq!(bend(&o, 1).get_str("direction"), Some("up"));
    // volume (the reference body measures 16684.95559302563): plate 100 x 56, two quarter rings of 100, legs of 12 and 6
    let ring = (PI / 4.0) * (16.0 - 4.0) * 100.0;
    let want = 100.0 * 56.0 * 2.0 + 2.0 * ring + (12.0 + 6.0) * 2.0 * 100.0;
    vol_close(valid_one_lump(&o), want, "two bends");
    vol_close(want, 16684.95559302563, "reference volume");
    let (lo, hi) = fbox(&o);
    // U channel: the second leg lies back over the plate at the flange height
    close(hi[2] - lo[2], 20.0, "height");
}

#[test]
fn then_chain_probes() {
    let cases: [(&str, f64, f64); 9] = [
        (r#""height":20,"then":[{"angle":45,"height":10}]"#, 85.47213163309227, -17.12898588258465),
        (r#""height":20,"angle":45,"then":[{"height":10}]"#, 85.47213163326157, -19.4721316331693),
        (r#""height":20,"then":[{"angle":120,"height":10}]"#, 78.69934485561689, -15.627548085892402),
        (r#""height":20,"radius":3,"then":[{"height":10}]"#, 82.1893794959642, -17.1893794959642),
        (r#""height":20,"position":"outside","then":[{"height":10}]"#, 91.0477868423386, -25.0477868421693),
        (r#""height":20,"position":"middle","then":[{"height":10}]"#, 87.04778684233861, -21.0477868421693),
        (r#""height":20,"heightFrom":"inner","then":[{"height":10}]"#, 87.04778684233861, -19.0477868421693),
        (r#""height":20,"heightFrom":"inner","angle":45,"then":[{"height":10}]"#, 88.3005587581693, -20.300558758169302),
        (r#""height":20,"heightFrom":"tangent","then":[{"angle":120,"height":10}]"#, 81.6275480858924, -15.627548085892402),
    ];
    for (fields, len, z2lo) in cases {
        let o = chained(fields);
        ok(&o);
        close(size(&o).1, len, fields);
        close(zone(&o, 1)[2], z2lo, fields);
        valid_one_lump(&o);
    }
    // the first bend with inner reference and 45 then a 90: the inner sharp reach is r tan(a/2), the next bend starts b = setback before
    let o = chained(r#""height":20,"heightFrom":"inner","then":[{"angle":45,"height":10}]"#);
    close(size(&o).1, 88.30055875783846, "inner then 45");
    close(zone(&o, 1)[2], -19.12898588258465, "inner then 45 zone");
}

#[test]
fn then_chain_directions_and_limits() {
    // flip is relative to the first bend, not to the one before
    let dirs = |th: &str| -> Vec<String> {
        let o = chained(&format!(r#""height":20,"then":{th}"#));
        ok(&o);
        (0..flat(&o).get_arr("bends").unwrap().len()).map(|k| bend(&o, k).get_str("direction").unwrap().to_string()).collect()
    };
    assert_eq!(dirs(r#"[{"height":10},{"height":8,"flip":true}]"#), ["up", "up", "down"]);
    assert_eq!(dirs(r#"[{"height":10,"flip":true},{"height":8}]"#), ["up", "down", "up"]);
    assert_eq!(dirs(r#"[{"height":10,"flip":true},{"height":8,"flip":true}]"#), ["up", "down", "down"]);
    let o = chained(r#""height":20,"then":[{"height":10,"flip":true}]"#);
    close(size(&o).1, 83.04778684233861, "flipped second bend, same flat");
    close(valid_one_lump(&o), 16684.955592465238, "S bend volume");
    let (lo, hi) = fbox(&o);
    close(lo[1], -8.0, "S bend reaches out 8");
    close(hi[2], 20.0, "S bend height");
    // heights that do not reach their bend
    let o = chained(r#""height":20,"then":[{"height":3}]"#);
    assert_eq!(o.errors, vec![("E1".to_string(), "flangeHeightTooLow".to_string())]);
    close(o.err_json[0].get_f64("minHeight").unwrap(), 4.0, "minHeight");
    let o = chained(r#""height":20,"then":[{"height":4}]"#);
    ok(&o);
    close(size(&o).1, 77.04778684233861, "second leg of length 0");
    // at most 16 further bends
    let many: Vec<String> = (0..17).map(|_| r#"{"height":10}"#.to_string()).collect();
    let o = chained(&format!(r#""height":20,"then":[{}]"#, many.join(",")));
    assert_eq!(o.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
    // a chain of 3 bends is a valid solid (bends of 45 degrees spiral outwards)
    let o = chained(r#""height":20,"angle":45,"then":[{"angle":45,"height":12},{"angle":45,"height":10}]"#);
    ok(&o);
    vol_close(valid_one_lump(&o), 19825.49159542183, "three bends");
    let (lo, hi) = fbox(&o);
    bclose(lo[1], -14.142136, "three bends y");
    bclose(hi[2], 33.213203, "three bends z");
    assert_eq!(flat(&o).get_arr("bends").unwrap().len(), 3);
}

#[test]
fn then_chain_on_two_edges() {
    let two = format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{},{}],"height":20,"then":[{{"height":10}}]}}"#, at(50.0, 0.0, 2.0), at(50.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[two]);
    ok(&o);
    close(size(&o).1, 106.09557368467722, "two chains");
    assert_eq!(flat(&o).get_arr("bends").unwrap().len(), 4);
    close(zone(&o, 3)[3], 77.0477868421693, "last zone");
    valid_one_lump(&o);
}

// ---------------------------------------------------------------- contour flange

fn contour_part(pts: &[(f64, f64)], t: f64, k: f64, r: f64, fields: &str) -> Out {
    run(&[rule(t, k, r, ""), chain(pts), format!(r#"{{"type":"contourFlange","id":"C1","sketch":"S1","distance":40{fields}}}"#)])
}

#[test]
fn contour_l_profile() {
    let o = contour_part(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)], 2.0, 0.44, 2.0, "");
    ok(&o);
    let (w, h) = size(&o);
    close(w, 46.523893421169305, "flat length");
    close(h, 40.0, "flat width");
    let b = bend(&o, 0);
    assert_eq!(b.get_str("direction"), Some("up"));
    close(pt(&b, "start").0, 18.261946710584652, "bend line");
    close(pt(&b, "start").1, 0.0, "y0");
    close(pt(&b, "end").1, 40.0, "y1");
    close(b.get_f64("allowance").unwrap(), BA90, "allowance");
    let z = zone(&o, 0);
    close(z[0], 16.0, "zone x0");
    close(z[1], 20.5238934211693, "zone x1");
    // the chain is the outer face: bbox x 0..30, y 0..20, z 0..40
    let (lo, hi) = fbox(&o);
    for (a, b) in [(lo[0], 0.0), (hi[0], 30.0), (lo[1], 0.0), (hi[1], 20.0), (lo[2], 0.0), (hi[2], 40.0)] {
        close(a, b, "bbox");
    }
    // volume: legs 16 and 26 at 2 thick, one quarter ring, all 40 wide
    let want = 40.0 * (2.0 * (16.0 + 26.0) + (PI / 4.0) * (16.0 - 4.0));
    vol_close(valid_one_lump(&o), want, "L");
}

#[test]
fn contour_probes() {
    // (chain, t, k, r, flat length, bend directions)
    let cases: [(&[(f64, f64)], f64, f64, f64, f64, &[&str]); 5] = [
        (&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0), (30.0, 40.0)], 2.0, 0.44, 2.0, 67.04778684233861, &["up", "down"]),
        (&[(0.0, 0.0), (30.0, 0.0), (50.0, 20.0)], 2.0, 0.44, 2.0, 58.88936370855417, &["down"]),
        (&[(0.0, 0.0), (30.0, 0.0), (10.0, 20.0)], 2.0, 0.44, 2.0, 55.413257129723476, &["down"]),
        (&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)], 3.0, 0.3, 1.5, 44.76991118430775, &["up"]),
        (&[(0.0, 0.0), (30.0, 0.0)], 2.0, 0.44, 2.0, 30.0, &[]),
    ];
    for (pts, t, k, r, len, dirs) in cases {
        let o = contour_part(pts, t, k, r, "");
        ok(&o);
        close(size(&o).0, len, &format!("{pts:?}"));
        let got: Vec<String> = (0..flat(&o).get_arr("bends").unwrap().len()).map(|i| bend(&o, i).get_str("direction").unwrap().to_string()).collect();
        assert_eq!(got, dirs);
        valid_one_lump(&o);
    }
    // zones of the Z profile and of the acute one
    let o = contour_part(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0), (30.0, 40.0)], 2.0, 0.44, 2.0, "");
    close(pt(&bend(&o, 1), "start").0, 46.78584013175396, "second bend line");
    close(zone(&o, 1)[0], 44.523893421169305, "second zone");
    let o = contour_part(&[(0.0, 0.0), (30.0, 0.0), (10.0, 20.0)], 2.0, 0.44, 2.0, "");
    close(bend(&o, 0).get_f64("allowance").unwrap(), 6.785840131753953, "135 allowance");
    close(zone(&o, 0)[0], 25.171572875253812, "135 zone");
    let (lo, hi) = fbox(&o);
    close(lo[1], -2.0, "material below the chain");
    close(hi[1], 21.414213562373092, "obtuse top");
    bclose(hi[0], 29.171572875253812, "obtuse extent");
    let _ = lo;
}

#[test]
fn contour_flip_and_reverse() {
    let l = [(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)];
    // flip: material on the other side of the chain, the chain is the inner face of the bend
    let o = contour_part(&l, 2.0, 0.44, 2.0, r#","flip":true"#);
    ok(&o);
    close(size(&o).0, 50.523893421169305, "flip length");
    assert_eq!(bend(&o, 0).get_str("direction"), Some("down"));
    close(pt(&bend(&o, 0), "start").0, 20.261946710584652, "flip bend line");
    close(pt(&bend(&o, 0), "start").1, -40.0, "flip y");
    let (lo, hi) = fbox(&o);
    for (a, b) in [(lo[0], -2.0), (hi[0], 30.0), (lo[1], 0.0), (hi[1], 22.0), (lo[2], 0.0), (hi[2], 40.0)] {
        close(a, b, "flip bbox");
    }
    valid_one_lump(&o);
    // reverse: extruded against the sketch normal
    let o = contour_part(&l, 2.0, 0.44, 2.0, r#","reverse":true"#);
    ok(&o);
    close(size(&o).0, 46.523893421169305, "reverse length");
    close(pt(&bend(&o, 0), "start").1, -40.0, "reverse y");
    let (lo, hi) = fbox(&o);
    close(lo[2], -40.0, "reverse z lo");
    close(hi[2], 0.0, "reverse z hi");
    valid_one_lump(&o);
    // both
    let o = contour_part(&l, 2.0, 0.44, 2.0, r#","flip":true,"reverse":true"#);
    ok(&o);
    close(pt(&bend(&o, 0), "start").1, 0.0, "both y");
    close(pt(&bend(&o, 0), "end").1, 40.0, "both y1");
    let (lo, hi) = fbox(&o);
    close(lo[2], -40.0, "both z lo");
    close(hi[2], 0.0, "both z hi");
}

#[test]
fn contour_errors() {
    // a segment shorter than its bend
    let o = contour_part(&[(0.0, 0.0), (0.0, 3.0), (30.0, 3.0)], 2.0, 0.44, 2.0, "");
    assert_eq!(o.errors, vec![("C1".to_string(), "sheetMetalUnsupported".to_string())]);
    let o = contour_part(&[(0.0, 0.0), (0.0, 20.0), (3.0, 20.0), (3.0, 40.0)], 2.0, 0.44, 2.0, "");
    assert_eq!(o.errors, vec![("C1".to_string(), "sheetMetalUnsupported".to_string())]);
    // a closed loop is no chain
    let o = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), r#"{"type":"contourFlange","id":"C1","sketch":"S1","distance":40}"#.to_string()]);
    assert_eq!(o.errors, vec![("C1".to_string(), "invalidValue".to_string())]);
    // no distance
    let o = run(&[rule(2.0, 0.44, 2.0, ""), chain(&[(0.0, 0.0), (0.0, 20.0)]), r#"{"type":"contourFlange","id":"C1","sketch":"S1"}"#.to_string()]);
    assert_eq!(o.errors, vec![("C1".to_string(), "invalidValue".to_string())]);
    // an edge flange on a contour flange (stage 3): the pick must name a wall of a panel
    let o = run(&[
        rule(2.0, 0.44, 2.0, ""),
        chain(&[(0.0, 0.0), (0.0, 20.0)]),
        r#"{"type":"contourFlange","id":"C1","sketch":"S1","distance":40}"#.to_string(),
        r#"{"type":"hem","id":"H1","edges":[{"role":"P3.side.start"}],"length":10}"#.to_string(),
    ]);
    assert_eq!(o.errors, vec![("H1".to_string(), "invalidValue".to_string())]);
}

// ---------------------------------------------------------------- fold

fn fold_f(fields: &str, x: f64) -> String {
    let more = if fields.is_empty() { String::new() } else { format!(",{fields}") };
    format!(r#"{{"type":"fold","id":"F1","points":[{},{}]{}}}"#, at(x, 0.0, 2.0), at(x, 60.0, 2.0), more)
}

/// Bounding boxes of bodies with bends are grids over the curved faces (about 1e-3 on the arcs).
fn bclose(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-3, "{what}: {a} vs {b}");
}

/// Closed-form volume of the 100 x 60 base with folds at the given (allowance, angle) on full-width lines.
fn fold_volume(t: f64, r: f64, folds: &[(f64, f64)]) -> f64 {
    let mut v = 100.0 * 60.0 * t;
    for (ba, a) in folds {
        v += -ba * 60.0 * t + (a / 2.0) * ((r + t).powi(2) - r * r) * 60.0;
    }
    v
}

#[test]
fn s3_fold_flat_pattern() {
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 40.0)]);
    ok(&o);
    let (w, h) = size(&o);
    close(w, 100.0, "width");
    close(h, 60.0, "length");
    let b = bend(&o, 0);
    close(b.get_f64("allowance").unwrap(), 3.015928947446201, "allowance");
    close(b.get_f64("angleDeg").unwrap(), 60.0, "angle");
    close(b.get_f64("includedAngleDeg").unwrap(), 120.0, "included");
    assert_eq!(b.get_str("direction"), Some("up"));
    close(pt(&b, "start").0, 40.0, "bend line x");
    close(pt(&b, "start").1, 0.0, "start y");
    close(pt(&b, "end").1, 60.0, "end y");
    close(b.get_f64("length").unwrap(), 60.0, "bend length");
    let z = zone(&o, 0);
    for (a, e) in z.iter().zip([38.4920355265538, 41.507964474, 0.0, 60.0]) {
        close(*a, e, "zone");
    }
    assert_eq!(flat(&o).get_arr("outline").unwrap()[0].as_arr().unwrap().len(), 4);
    vol_close(valid_one_lump(&o), fold_volume(2.0, 2.0, &[(3.015928947446201, PI / 3.0)]), "fold");
}

#[test]
fn fold_bounding_boxes_and_volumes_match_reference() {
    // (fold x, angle, flip, bbox min x, bbox max x, max z, min z)
    let cases: [(f64, &str, bool, f64, f64, f64, f64); 5] = [
        (40.0, "60", false, 18.797845, 100.0, 36.335081, 0.0),
        (40.0, "60", true, 18.797845, 100.0, 2.0, -34.335081),
        (70.0, "60", false, 0.0, 86.202155, 27.674827, 0.0),
        (40.0, "90", false, 38.261947, 100.0, 41.738053, 0.0),
        (40.0, "120", false, 39.015929, 100.0, 38.029145, 0.0),
    ];
    for (x, ang, flip, x0, x1, zmax, zmin) in cases {
        let o = part(2.0, 0.44, 2.0, "", &[fold_f(&format!(r#""angle":{ang},"flip":{flip}"#), x)]);
        ok(&o);
        let (lo, hi) = fbox(&o);
        let what = format!("fold {x} {ang} {flip}");
        bclose(lo[0], x0, &what);
        bclose(hi[0], x1, &what);
        close(lo[1], 0.0, &what);
        close(hi[1], 60.0, &what);
        bclose(hi[2], zmax, &what);
        bclose(lo[2], zmin, &what);
        let a: f64 = ang.parse().unwrap();
        let ba = (2.0 + 0.44 * 2.0) * a.to_radians();
        vol_close(valid_one_lump(&o), fold_volume(2.0, 2.0, &[(ba, a.to_radians())]), &what);
    }
    // two folds: 45 degrees at x = 20, 60 degrees at x = 70
    let f1 = fold_f(r#""angle":45"#, 20.0);
    let f2 = fold_f(r#""angle":60"#, 70.0).replace("F1", "F2");
    let o = part(2.0, 0.44, 2.0, "", &[f1, f2]);
    ok(&o);
    let (lo, hi) = fbox(&o);
    bclose(lo[0], 4.96013, "two folds x0");
    bclose(hi[0], 86.202155, "two folds x1");
    bclose(hi[2], 27.674827, "two folds z");
    vol_close(valid_one_lump(&o), 12026.389378520671, "two folds volume");
}

#[test]
fn fold_positions() {
    // zone [z0, z1] measured to the moving side from the line; fixed side = the larger side (x > 40 here)
    let cases: [(&str, &str, [f64; 2], f64); 8] = [
        ("centerline", "60", [38.4920355265538, 41.507964474], 40.0),
        ("start", "60", [36.9840710525538, 40.0], 38.4920355262769),
        ("end", "60", [40.0, 43.015928947], 41.5079644732769),
        ("mould", "60", [39.2934721295538, 42.309401077], 40.8014366032769),
        ("centerline", "90", [37.73805328941535, 42.26194671058465], 40.0),
        ("start", "90", [35.476106578830695, 40.0], 37.73805328941535),
        ("end", "90", [40.0, 44.52389342116931], 42.26194671058465),
        ("mould", "90", [39.476106578830695, 44.0], 41.73805328941535),
    ];
    for (pos, ang, z, line) in cases {
        let o = part(2.0, 0.44, 2.0, "", &[fold_f(&format!(r#""angle":{ang},"position":"{pos}""#), 40.0)]);
        ok(&o);
        let zz = zone(&o, 0);
        close(zz[0], z[0], &format!("{pos} {ang} zone lo"));
        close(zz[1], z[1], &format!("{pos} {ang} zone hi"));
        close(pt(&bend(&o, 0), "start").0, line, &format!("{pos} {ang} line"));
        let a: f64 = ang.parse().unwrap();
        let ba = (2.0 + 0.44 * 2.0) * a.to_radians();
        vol_close(valid_one_lump(&o), fold_volume(2.0, 2.0, &[(ba, a.to_radians())]), &format!("{pos} {ang} volume"));
    }
    // the fixed side sets which way the zone extends: fixed on the left, 'start' extends to the right
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(&format!(r#""angle":60,"position":"start","fixed":{}"#, at(10.0, 30.0, 2.0)), 40.0)]);
    ok(&o);
    let zz = zone(&o, 0);
    close(zz[0], 40.0, "fixed left, start: zone lo");
    close(zz[1], 43.015928947, "fixed left, start: zone hi");
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(&format!(r#""angle":60,"position":"mould","fixed":{}"#, at(10.0, 30.0, 2.0)), 40.0)]);
    close(zone(&o, 0)[0], 37.690598923, "fixed left, mould");
    close(pt(&bend(&o, 0), "start").0, 39.1985633967231, "fixed left, mould line");
    // the larger side stays: a fold at x = 70 moves the right side
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 70.0)]);
    ok(&o);
    close(zone(&o, 0)[0], 68.492035526, "fold at 70");
    let (lo, hi) = fbox(&o);
    close(lo[0], 0.0, "left side stays");
    close(lo[2], 0.0, "left side on the plane");
    assert!(hi[2] > 20.0);
}

#[test]
fn fold_angle_radius_flip() {
    let d = |fields: &str| -> (f64, f64, String, f64) {
        let o = part(2.0, 0.44, 2.0, "", &[fold_f(fields, 40.0)]);
        ok(&o);
        valid_one_lump(&o);
        let b = bend(&o, 0);
        (b.get_f64("allowance").unwrap(), zone(&o, 0)[0], b.get_str("direction").unwrap().to_string(), zone(&o, 0)[1])
    };
    assert_eq!(d(""), (BA90, 37.73805328941535, "up".to_string(), 42.26194671058465));
    let (ba, lo, _, hi) = d(r#""angle":120"#);
    close(ba, 6.031857894892402, "120 allowance");
    close(lo, 36.984071052107595, "120 lo");
    close(hi, 43.015928947, "120 hi");
    let (ba, lo, _, _) = d(r#""angle":10"#);
    close(ba, 0.5026548245743669, "10 allowance");
    close(lo, 39.74867258771282, "10 lo");
    let (ba, lo, _, hi) = d(r#""angle":60,"radius":5"#);
    close(ba, 6.157521601035994, "r5 allowance");
    close(lo, 36.921239199964006, "r5 lo");
    close(hi, 43.078760801, "r5 hi");
    let (_, _, dir, _) = d(r#""angle":60,"flip":true"#);
    assert_eq!(dir, "down");
    // a fold down turns the moving side below the sketch plane
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60,"flip":true"#, 40.0)]);
    let (lo, hi) = fbox(&o);
    assert!(lo[2] < -30.0, "{lo:?}");
    close(hi[2], 2.0, "fixed side top");
    // k-factor
    let o = part(2.0, 0.3, 2.0, "", &[fold_f(r#""angle":60"#, 40.0)]);
    close(bend(&o, 0).get_f64("allowance").unwrap(), 2.722713633111154, "k0.3");
    // angle limits
    for a in ["0", "180", "-60"] {
        let o = part(2.0, 0.44, 2.0, "", &[fold_f(&format!(r#""angle":{a}"#), 40.0)]);
        assert_eq!(o.errors, vec![("F1".to_string(), "invalidValue".to_string())], "angle {a}");
    }
}

#[test]
fn fold_line_extends_across_the_base() {
    // a line that runs past the base, or stops short of its edges, folds the whole width
    for (y0, y1) in [(-10.0, 70.0), (0.0, 70.0), (5.0, 55.0), (10.0, 50.0)] {
        let f = format!(r#"{{"type":"fold","id":"F1","points":[{},{}],"angle":60}}"#, at(40.0, y0, 2.0), at(40.0, y1, 2.0));
        let o = part(2.0, 0.44, 2.0, "", &[f]);
        ok(&o);
        close(bend(&o, 0).get_f64("length").unwrap(), 60.0, "bend length");
        valid_one_lump(&o);
    }
}

#[test]
fn two_folds_chain_of_panels() {
    // fold 1 at x = 20 (45 degrees) and fold 2 at x = 70 (60 degrees): the larger side of each stays
    let f1 = r#"{"type":"fold","id":"F1","points":[POINT1,POINT2],"angle":45}"#.replace("POINT1", &at(20.0, 0.0, 2.0)).replace("POINT2", &at(20.0, 60.0, 2.0));
    let f2 = r#"{"type":"fold","id":"F2","points":[POINT1,POINT2],"angle":60}"#.replace("POINT1", &at(70.0, 0.0, 2.0)).replace("POINT2", &at(70.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[f1.clone(), f2.clone()]);
    ok(&o);
    assert_eq!(flat(&o).get_arr("bends").unwrap().len(), 2);
    close(bend(&o, 0).get_f64("allowance").unwrap(), 2.261946710584651, "bend 1 allowance");
    close(pt(&bend(&o, 0), "start").0, 20.0, "bend 1 line");
    close(pt(&bend(&o, 1), "start").0, 70.0, "bend 2 line");
    close(zone(&o, 0)[0], 18.869026644415346, "zone 1");
    close(zone(&o, 1)[1], 71.5079644734462, "zone 2");
    vol_close(valid_one_lump(&o), fold_volume(2.0, 2.0, &[(2.261946710584651, PI / 4.0), (3.015928947446201, PI / 3.0)]), "two folds");
    // close together: 40 (45 degrees) and 45 (60 degrees), zones do not meet
    let g1 = r#"{"type":"fold","id":"F1","points":[POINT1,POINT2],"angle":45}"#.replace("POINT1", &at(40.0, 0.0, 2.0)).replace("POINT2", &at(40.0, 60.0, 2.0));
    let g2 = r#"{"type":"fold","id":"F2","points":[POINT1,POINT2],"angle":60}"#.replace("POINT1", &at(45.0, 0.0, 2.0)).replace("POINT2", &at(45.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[g1.clone(), g2]);
    ok(&o);
    close(zone(&o, 0)[0], 38.86902664441535, "near zone 1");
    close(zone(&o, 1)[0], 43.4920355265538, "near zone 2");
    valid_one_lump(&o);
    // zones that overlap are refused
    let g3 = r#"{"type":"fold","id":"F2","points":[POINT1,POINT2],"angle":60}"#.replace("POINT1", &at(41.0, 0.0, 2.0)).replace("POINT2", &at(41.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[g1, g3]);
    assert_eq!(o.errors, vec![("F2".to_string(), "sheetMetalUnsupported".to_string())]);
}

#[test]
fn fold_follows_a_sketch_line() {
    // the fold line is a line of another sketch, followed when the sketch changes
    let line_sketch = |x: f64| {
        format!(
            r#"{{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":{x},"y":-5}},{{"id":"q2","x":{x},"y":70}}],"entities":[{{"type":"line","id":"f1","p1":"q1","p2":"q2"}}],"constraints":[]}}"#
        )
    };
    let fold = r#"{"type":"fold","id":"F1","line":{"sketch":"S2","curve":"f1"},"angle":60}"#.to_string();
    let a = part(2.0, 0.44, 2.0, "", &[line_sketch(40.0), fold.clone()]);
    ok(&a);
    close(pt(&bend(&a, 0), "start").0, 40.0, "line at 40");
    let b = part(2.0, 0.44, 2.0, "", &[line_sketch(55.0), fold]);
    ok(&b);
    close(pt(&b_bend(&b), "start").0, 55.0, "line moved to 55");
    valid_one_lump(&b);
}

fn b_bend(o: &Out) -> ck_json::Json {
    bend(o, 0)
}

#[test]
fn fold_limits() {
    // oblique fold lines are folds of the panel tree (stage 3): the bend zone ends obliquely, the flat pattern keeps its size
    let diag = format!(r#"{{"type":"fold","id":"F1","points":[{},{}],"angle":60}}"#, at(20.0, 0.0, 2.0), at(60.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[diag]);
    ok(&o);
    close(size(&o).0, 100.0, "oblique fold keeps the flat size");
    // a fold line off the base
    let off = format!(r#"{{"type":"fold","id":"F1","points":[{},{}],"angle":60}}"#, at(140.0, 0.0, 2.0), at(140.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[off]);
    assert!(!o.errors.is_empty());
    // flanges and folds mix since stage 3 (a flange on the bottom edge is crossed by a fold at x = 40: refused; one on the
    // left wall is not)
    let fl = flange(r#""height":15"#, (50.0, 0.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[fl.clone(), fold_f(r#""angle":60"#, 40.0)]);
    assert_eq!(o.errors, vec![("F1".to_string(), "sheetMetalUnsupported".to_string())]);
    let left = flange(r#""height":15"#, (0.0, 30.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[left.clone(), fold_f(r#""angle":60"#, 40.0)]);
    ok(&o);
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 40.0), left]);
    ok(&o);
    // no sheet
    let o = run(&[fold_f("", 40.0)]);
    assert_eq!(o.errors, vec![("F1".to_string(), "nothingToCut".to_string())]);
}

// ---------------------------------------------------------------- unfold / refold

fn simple(t: &str) -> String {
    format!(r#"{{"type":"{t}","id":"U1"}}"#)
}

#[test]
fn unfold_is_the_flat_sheet_and_refold_restores() {
    let e1 = flange(r#""height":20"#, (50.0, 0.0, 2.0));
    let folded = part(2.0, 0.44, 2.0, "", &[e1.clone()]);
    ok(&folded);
    let v0 = valid_one_lump(&folded);
    let faces0 = folded.body.as_ref().unwrap().num_faces();
    let u = part(2.0, 0.44, 2.0, "", &[e1.clone(), simple("unfold")]);
    ok(&u);
    // the flat sheet: the flat pattern extruded by the thickness (reference bbox: x 0..100, y -16.5239..60, z 0..2, 6 faces)
    let (lo, hi) = fbox(&u);
    for (a, b) in [(lo[0], 0.0), (hi[0], 100.0), (lo[1], -16.523893421169305), (hi[1], 60.0), (lo[2], 0.0), (hi[2], 2.0)] {
        close(a, b, "unfolded bbox");
    }
    assert_eq!(u.body.as_ref().unwrap().num_faces(), 6);
    vol_close(valid_one_lump(&u), 100.0 * 76.52389342116931 * 2.0, "flat volume");
    assert_eq!(u.last.as_ref().unwrap().get("flatPattern").unwrap().get("unfolded").and_then(|v| v.as_bool()), Some(true));
    // refold restores the folded solid exactly
    let r = part(2.0, 0.44, 2.0, "", &[e1.clone(), simple("unfold"), r#"{"type":"refold","id":"U2"}"#.to_string()]);
    ok(&r);
    vol_close(valid_one_lump(&r), v0, "refold volume");
    assert_eq!(r.body.as_ref().unwrap().num_faces(), faces0);
    let (a, b) = (fbox(&r), fbox(&folded));
    for k in 0..3 {
        close(a.0[k], b.0[k], "refold lo");
        close(a.1[k], b.1[k], "refold hi");
    }
    assert_eq!(r.last.as_ref().unwrap().get("flatPattern").unwrap().get("unfolded").and_then(|v| v.as_bool()), Some(false));
    // flat pattern numbers are those of the folded sheet
    close(size(&r).1, 76.52389342116931, "refold flat length");
}

#[test]
fn unfold_every_kind_of_sheet() {
    // fold: a flat 100 x 60 x 2 plate
    let u = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 40.0), simple("unfold")]);
    ok(&u);
    vol_close(valid_one_lump(&u), 100.0 * 60.0 * 2.0, "unfolded fold");
    let (lo, hi) = fbox(&u);
    close(hi[2] - lo[2], 2.0, "flat");
    let r = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 40.0), simple("unfold"), r#"{"type":"refold","id":"U2"}"#.to_string()]);
    ok(&r);
    vol_close(valid_one_lump(&r), fold_volume(2.0, 2.0, &[(3.015928947446201, PI / 3.0)]), "refolded fold");
    // hem: flat reference bbox y -10.786017...
    let u = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 2.0)), simple("unfold")]);
    ok(&u);
    let (lo, hi) = fbox(&u);
    close(lo[1], -10.786017461694916, "hem flat y");
    close(hi[2], 2.0, "hem flat z");
    assert_eq!(u.body.as_ref().unwrap().num_faces(), 6);
    // contour
    let c = |extra: &str| {
        run(&[
            rule(2.0, 0.44, 2.0, ""),
            chain(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)]),
            r#"{"type":"contourFlange","id":"C1","sketch":"S1","distance":40}"#.to_string(),
            simple("unfold"),
        ]
        .into_iter()
        .chain((!extra.is_empty()).then(|| extra.to_string()))
        .collect::<Vec<_>>())
    };
    let u = c("");
    ok(&u);
    vol_close(valid_one_lump(&u), 46.523893421169305 * 40.0 * 2.0, "unfolded contour");
    let r = c(r#"{"type":"refold","id":"U2"}"#);
    ok(&r);
    vol_close(valid_one_lump(&r), 40.0 * (2.0 * 42.0 + (PI / 4.0) * 12.0), "refolded contour");
    // chained bends
    let ch = flange(r#""height":20,"then":[{"height":10}]"#, (50.0, 0.0, 2.0));
    let u = part(2.0, 0.44, 2.0, "", &[ch.clone(), simple("unfold")]);
    ok(&u);
    vol_close(valid_one_lump(&u), 100.0 * 83.04778684233861 * 2.0, "unfolded chain");
}

#[test]
fn unfold_errors() {
    let e1 = flange(r#""height":20"#, (50.0, 0.0, 2.0));
    // no sheet
    let o = run(&[simple("unfold")]);
    assert_eq!(o.errors, vec![("U1".to_string(), "nothingToCut".to_string())]);
    // refold without unfold
    let o = part(2.0, 0.44, 2.0, "", &[e1.clone(), simple("refold")]);
    assert_eq!(o.errors, vec![("U1".to_string(), "sheetMetalUnsupported".to_string())]);
    // unfold twice
    let o = part(2.0, 0.44, 2.0, "", &[e1.clone(), simple("unfold"), r#"{"type":"unfold","id":"U2"}"#.to_string()]);
    assert_eq!(o.errors, vec![("U2".to_string(), "sheetMetalUnsupported".to_string())]);
    // flanging an unfolded sheet needs a refold first
    let o = part(2.0, 0.44, 2.0, "", &[e1.clone(), simple("unfold"), format!(r#"{{"type":"edgeFlange","id":"E2","edges":[{}],"height":20}}"#, at(50.0, 60.0, 0.0))]);
    assert_eq!(o.errors, vec![("E2".to_string(), "sheetMetalUnsupported".to_string())]);
    // a plain base unfolds to itself
    let o = part(2.0, 0.44, 2.0, "", &[simple("unfold")]);
    ok(&o);
    vol_close(valid_one_lump(&o), 100.0 * 60.0 * 2.0, "base only");
}

// ---------------------------------------------------------------- roles

fn role_face(o: &Out, role: &str) -> Option<String> {
    let roles = o.last.as_ref().unwrap().get("roles").unwrap();
    roles.get_arr("faces").unwrap().iter().find(|f| f.get_str("role") == Some(role)).and_then(|f| f.get_str("face").map(str::to_string))
}

fn role_names(o: &Out) -> Vec<String> {
    o.last.as_ref().unwrap().get("roles").unwrap().get_arr("faces").unwrap().iter().map(|f| f.get_str("role").unwrap().to_string()).collect()
}

#[test]
fn role_names_of_a_flanged_plate() {
    let fl = format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{},{}],"height":20}}"#, at(50.0, 0.0, 2.0), at(50.0, 60.0, 2.0));
    let o = part(2.0, 0.44, 2.0, "", &[fl]);
    ok(&o);
    let names = role_names(&o);
    for want in [
        "P0.front", "P0.back", "P0.side.e2", "P0.side.e4", "B1.front", "B1.back", "B1.side.start", "B1.side.end", "P1.front", "P1.back", "P1.tip", "P1.side.start", "P1.side.end",
        "B2.front", "B2.back", "P2.front", "P2.tip",
    ] {
        assert!(names.iter().any(|n| n == want), "missing role {want} in {names:?}");
    }
    // the walls of the flanged edges are bends, not walls
    assert!(!names.iter().any(|n| n == "P0.side.e1" || n == "P0.side.e3"));
    // every named face exists in the body, each under a different id
    let body = o.body.as_ref().unwrap();
    let mut seen = vec![];
    for n in &names {
        let f = role_face(&o, n).unwrap_or_else(|| panic!("role {n} has no face"));
        assert!(body.find_face(&f).is_some(), "{n} -> {f}");
        if !n.contains("side.start") && !n.contains("side.end") {
            assert!(!seen.contains(&f), "{n} shares face {f}");
        }
        seen.push(f);
    }
    // front/back: P0.front is the sketch plane, the leg's front continues it
    let f = o.frame.unwrap();
    let face = |r: &str| body.find_face(&role_face(&o, r).unwrap()).unwrap();
    let nrm = |r: &str| match &body.face(face(r)).surface {
        ck_geom::surface::Surface::Plane { frame } => frame.z,
        _ => panic!("{r} is not planar"),
    };
    assert!(nrm("P0.front").dot(f.z).abs() > 1.0 - 1e-9);
    assert!(nrm("P1.tip").dot(f.z).abs() > 1.0 - 1e-9, "the leg tip is parallel to the plate");
    match &body.face(face("B1.front")).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => close(*r, 4.0, "outer bend face"),
        _ => panic!("bend face is not a cylinder"),
    }
    match &body.face(face("B1.back")).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => close(*r, 2.0, "inner bend face"),
        _ => panic!("bend face is not a cylinder"),
    }
    // edges: seams of named faces
    let edges = o.last.as_ref().unwrap().get("roles").unwrap().get_arr("edges").unwrap().clone();
    let has = |a: &str, b: &str| edges.iter().any(|e| e.get_str("role") == Some(&format!("{a}|{b}")));
    assert!(has("P0.front", "B1.front") && has("B1.front", "P1.front") && has("P0.back", "B1.back"), "{:?}", edges.iter().map(|e| e.get_str("role").unwrap()).collect::<Vec<_>>());
}

#[test]
fn role_names_of_hem_fold_contour_and_chain() {
    let o = part(2.0, 0.44, 2.0, "", &[hem(r#""length":10"#, (50.0, 0.0, 2.0))]);
    let names = role_names(&o);
    for w in ["P0.front", "B1.front", "P1.front", "P1.tip"] {
        assert!(names.iter().any(|n| n == w), "hem: missing {w}");
        assert!(role_face(&o, w).is_some(), "hem: {w} unresolved");
    }
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60"#, 40.0)]);
    let names = role_names(&o);
    for w in ["P0.front", "P0.back", "P1.front", "P1.back", "B1.front", "B1.back", "P0.side.e2", "P1.side.e4"] {
        assert!(names.iter().any(|n| n == w), "fold: missing {w} in {names:?}");
        assert!(role_face(&o, w).is_some(), "fold: {w} unresolved");
    }
    let o = contour_part(&[(0.0, 0.0), (0.0, 20.0), (30.0, 20.0)], 2.0, 0.44, 2.0, "");
    for w in ["P0.front", "P0.back", "P1.front", "P1.back", "B1.front", "B1.back", "P0.side.start", "P1.side.end"] {
        assert!(role_face(&o, w).is_some(), "contour: {w} unresolved");
    }
    let o = chained(r#""height":20,"then":[{"height":10}]"#);
    for w in ["P1.front", "B1.front", "B2.front", "B2.back", "P2.front", "P2.tip"] {
        assert!(role_face(&o, w).is_some(), "chain: {w} unresolved");
    }
    // the front of the chain's second leg is the same sheet face as the first leg's: A is the outer face of both bends
    let body = o.body.as_ref().unwrap();
    let c = |r: &str| match &body.face(body.find_face(&role_face(&o, r).unwrap()).unwrap()).surface {
        ck_geom::surface::Surface::Cylinder { r, .. } => *r,
        _ => 0.0,
    };
    close(c("B1.front"), 4.0, "first bend front");
    close(c("B2.front"), 4.0, "second bend front (same sense)");
    close(c("B2.back"), 2.0, "second bend back");
}

#[test]
fn flat_unfolded_flag_and_bend_roles_in_the_table() {
    let o = part(2.0, 0.44, 2.0, "", &[flange(r#""height":20,"then":[{"height":10}]"#, (50.0, 0.0, 2.0))]);
    assert_eq!(bend(&o, 0).get_str("role"), Some("B1"));
    assert_eq!(bend(&o, 1).get_str("role"), Some("B2"));
}

#[test]
fn picks_by_role_stay_on_their_wall() {
    // the same four walls with ids e1..e4, but the sketch starts at another corner: the point of e1 moves, a role follows the curve
    for start in 0..4 {
        let fs = vec![
            rule(2.0, 0.44, 2.0, ""),
            sketch_from(100.0, 60.0, start),
            BASE.to_string(),
            r#"{"type":"edgeFlange","id":"E1","edges":[{"role":"P0.side.e1"}],"height":20}"#.to_string(),
        ];
        let o = run(&fs);
        ok(&o);
        // e1 runs from corner `start` to the next one: its length is the length of the flange
        let c = [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)];
        let (a, b) = (c[start % 4], c[(start + 1) % 4]);
        let wall = ((b.0 - a.0) as f64).hypot(b.1 - a.1);
        close(bend(&o, 0).get_f64("length").unwrap(), wall, "flange along the wall of e1");
        valid_one_lump(&o);
        // the bend line lies on the wall of e1, 1.738 inside the outline
        let (s, e) = (pt(&bend(&o, 0), "start"), pt(&bend(&o, 0), "end"));
        let mid = ((s.0 + e.0) / 2.0, (s.1 + e.1) / 2.0);
        let wall_mid = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
        let d = (mid.0 - wall_mid.0).hypot(mid.1 - wall_mid.1);
        close(d, 1.7380532894153489, "bend line offset from the wall of e1");
    }
    // edge roles and shorthands: back|side, curve, a plain string
    let mk = |pick: &str| {
        let o = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), BASE.to_string(), format!(r#"{{"type":"edgeFlange","id":"E1","edges":[{pick}],"height":20}}"#)]);
        ok(&o);
        o
    };
    let up = mk(r#"{"role":"P0.back|P0.side.e1"}"#);
    assert_eq!(bend(&up, 0).get_str("direction"), Some("up"));
    let down = mk(r#"{"role":"P0.front|P0.side.e1"}"#);
    assert_eq!(bend(&down, 0).get_str("direction"), Some("down"));
    assert_eq!(bend(&mk(r#"{"curve":"e3"}"#), 0).get_str("direction"), Some("up"));
    assert_eq!(bend(&mk(r#""P0.side.e3""#), 0).get_str("direction"), Some("up"));
    // hem by role
    let h = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), BASE.to_string(), r#"{"type":"hem","id":"H1","edges":[{"role":"P0.side.e1"}],"length":10}"#.to_string()]);
    ok(&h);
    close(size(&h).1, 70.78601746169491, "hem by role");
    // unknown curves and other panels
    let bad = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), BASE.to_string(), r#"{"type":"edgeFlange","id":"E1","edges":[{"role":"P0.side.zz"}],"height":20}"#.to_string()]);
    assert_eq!(bad.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
    let bad = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), BASE.to_string(), r#"{"type":"edgeFlange","id":"E1","edges":[{"role":"P3.side.e1"}],"height":20}"#.to_string()]);
    assert_eq!(bad.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
    let bad = run(&[rule(2.0, 0.44, 2.0, ""), sketch(100.0, 60.0), BASE.to_string(), r#"{"type":"edgeFlange","id":"E1","edges":[{"role":"nonsense"}],"height":20}"#.to_string()]);
    assert_eq!(bad.errors, vec![("E1".to_string(), "invalidValue".to_string())]);
}

#[test]
fn fixed_side_by_role() {
    // the wall e4 (x = 0) is on the left: fixing it makes the left side stay and the zone of `start` extend to the right
    let o = part(2.0, 0.44, 2.0, "", &[fold_f(r#""angle":60,"position":"start","fixed":"P0.side.e4""#, 40.0)]);
    ok(&o);
    close(zone(&o, 0)[0], 40.0, "start, fixed left");
}
