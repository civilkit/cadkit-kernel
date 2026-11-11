//! `mirror` of a part that is a sheet (docs/sheet-metal.md, "Mirror"): the part stays a sheet. A mirror with `replace` is the
//! reflected panel tree (same flat pattern, same bends, any plane, a face of the sheet included); without it the image is joined
//! to the sheet when the two meet along a bend-free straight edge of the base plate and the joined tree unfolds as a whole;
//! every other plane is refused with the reason.
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
    brep: Brep,
    errors: Vec<(String, String)>,
    err_json: Vec<ck_json::Json>,
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
    let last_id = features.iter().rev().filter_map(|f| ck_json::parse(f).ok()).find(|j| j.get_str("type").map_or(false, |t| ["baseFlange", "edgeFlange", "hem", "fold", "contourFlange", "unfold", "refold", "sheetCorner", "sheetCut", "sheetConvert", "mirror"].contains(&t))).and_then(|j| j.get_str("id").map(str::to_string));
    let last = wire.get_arr("sheetMetal").and_then(|a| a.iter().find(|e| e.get("flatPattern").is_some() && e.get_str("id").map(str::to_string) == last_id).cloned());
    Out {
        brep: ex.0,
        errors: r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect(),
        err_json: r.errors.iter().map(|e| ck_json::Json::Obj(e.error.params.clone())).collect(),
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

fn role_face(o: &Out, role: &str) -> String {
    role_faces(o).into_iter().find(|r| r.0 == role).and_then(|r| r.1).unwrap_or_else(|| panic!("no face for role {role}"))
}

fn role_faces(o: &Out) -> Vec<(String, Option<String>)> {
    o.last.as_ref().unwrap().get("roles").unwrap().get_arr("faces").unwrap().iter().map(|f| (f.get_str("role").unwrap().to_string(), f.get_str("face").map(str::to_string))).collect()
}

fn mirror(id: &str, plane: &str, replace: bool) -> String {
    format!(r#"{{"type":"mirror","id":"{id}","body":true,"plane":{plane}{}}}"#, if replace { r#","replace":true"# } else { "" })
}

fn face_plane(face: &str) -> String {
    format!(r#"{{"kind":"face","id":"{face}"}}"#)
}

fn std_plane(id: &str) -> String {
    format!(r#"{{"kind":"std","id":"{id}"}}"#)
}

fn circle_sketch(id: &str, cx: f64, cy: f64, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#)
}

fn cut(sk: &str) -> String {
    format!(r#"{{"type":"sheetCut","id":"X1","sketch":"{sk}"}}"#)
}

fn unfold() -> String {
    r#"{"type":"unfold","id":"U9"}"#.to_string()
}

/// An L-bracket: the 100 x 60 plate with a 15 high flange on the edge x = 0.
fn l_bracket() -> Vec<String> {
    vec![flange("E1", &[(0.0, 30.0, 2.0)], r#""height":15"#)]
}

fn with(mut a: Vec<String>, b: String) -> Vec<String> {
    a.push(b);
    a
}

fn detail(o: &Out) -> String {
    o.err_json.iter().filter_map(|j| j.get_str("detail").map(str::to_string)).collect::<Vec<_>>().join(" | ")
}

#[test]
fn replace_mirror_of_an_l_bracket_is_the_same_sheet_the_other_way_round() {
    let base = part(&l_bracket());
    ok(&base);
    let (v0, (w0, h0), a0) = (valid_one_lump(&base), size(&base), flat_area(&base));
    let end = role_face(&base, "P0.side.e2");
    // across the end face of the plate, across the three standard planes and across the front face of the plate
    for (name, plane) in [("end face", face_plane(&end)), ("front", std_plane("front")), ("right", std_plane("right")), ("top", std_plane("top")), ("plate front face", face_plane(&role_face(&base, "P0.front")))] {
        let o = part(&with(l_bracket(), mirror("M1", &plane, true)));
        ok(&o);
        assert!(o.brep.sheet.model.as_ref().unwrap().tree.is_some(), "{name}: still a sheet");
        close(valid_one_lump(&o), v0, &format!("{name}: volume"));
        let (w, h) = size(&o);
        close(w, w0, &format!("{name}: flat width"));
        close(h, h0, &format!("{name}: flat height"));
        close(flat_area(&o), a0, &format!("{name}: flat area"));
        assert_eq!(bends(&o).len(), 1, "{name}: one bend");
        close(bends(&o)[0].get("allowance").unwrap().as_f64().unwrap(), BA90, &format!("{name}: bend allowance"));
        // and it unfolds and folds back
        let un = part(&[l_bracket(), vec![mirror("M1", &plane, true), unfold()]].concat());
        ok(&un);
        let (wu, hu) = size(&un);
        close(wu, w0, &format!("{name}: unfolded width"));
        close(hu, h0, &format!("{name}: unfolded height"));
        rel(valid_one_lump(&un), flat_area(&un) * 2.0, 1e-6, &format!("{name}: unfolded volume = area x t"));
        let re = part(&[l_bracket(), vec![mirror("M1", &plane, true), unfold(), r#"{"type":"refold","id":"U10"}"#.to_string()]].concat());
        ok(&re);
        rel(valid_one_lump(&re), v0, 1e-9, &format!("{name}: refold"));
    }
}

#[test]
fn replace_mirror_puts_the_part_on_the_other_side_of_the_plane() {
    let base = part(&l_bracket());
    let end = role_face(&base, "P0.side.e2");
    let (a, b) = fbox(&base);
    let o = part(&with(l_bracket(), mirror("M1", &face_plane(&end), true)));
    ok(&o);
    let (a2, b2) = fbox(&o);
    // across x = 100: the plate runs from 100 to 200, the flange stands at the far end; z and y are unchanged
    close(a2[0], 200.0 - b[0], "x min");
    close(b2[0], 200.0 - a[0], "x max");
    close(a2[1], a[1], "y min");
    close(b2[2], b[2], "z max");
    // a flange can still be added to the mirrored sheet: on the edge at x = 100 of the image plate (its own end)
    let more = part(&with(with(l_bracket(), mirror("M1", &face_plane(&end), true)), flange("E2", &[(100.0, 30.0, 2.0)], r#""height":10"#)));
    ok(&more);
    assert_eq!(bends(&more).len(), 2);
    valid_one_lump(&more);
}

#[test]
fn joined_mirror_of_an_l_bracket_is_one_channel_that_unfolds_as_a_whole() {
    let base = part(&l_bracket());
    ok(&base);
    let (v0, (w0, h0)) = (valid_one_lump(&base), size(&base));
    let end = role_face(&base, "P0.side.e2");
    let o = part(&with(l_bracket(), mirror("M1", &face_plane(&end), false)));
    ok(&o);
    assert!(o.brep.sheet.model.as_ref().unwrap().tree.is_some(), "still a sheet");
    rel(valid_one_lump(&o), 2.0 * v0, 1e-9, "joined volume");
    let (w, h) = size(&o);
    close(w, 2.0 * w0, "flat width is twice the half");
    close(h, h0, "flat height");
    assert_eq!(bends(&o).len(), 2, "one bend each side");
    assert_eq!(loops(&o).len(), 1, "one outline");
    let tree = o.brep.sheet.model.as_ref().unwrap().tree.as_ref().unwrap();
    assert_eq!(tree.panels.len(), 3, "the plates are one panel, a flange hangs on each side");
    assert_eq!(tree.panels.iter().filter(|p| p.via.is_none()).count(), 1);
    // the seam is not a corner: the merged plate is a plain 200 x 60 rectangle
    assert_eq!(tree.panels[0].poly.len(), 4);
    // unfolds as a whole
    let un = part(&with(with(l_bracket(), mirror("M1", &face_plane(&end), false)), unfold()));
    ok(&un);
    let (wu, hu) = size(&un);
    close(wu, w, "unfolded width");
    close(hu, h, "unfolded height");
    rel(valid_one_lump(&un), flat_area(&un) * 2.0, 1e-6, "unfolded volume = area x t");
    // a feature after the join still works on the joined sheet (a cut across the seam)
    let oc = part(&[l_bracket(), vec![mirror("M1", &face_plane(&end), false), circle_sketch("S2", 100.0, 30.0, 5.0), cut("S2")]].concat());
    ok(&oc);
    assert!(valid_one_lump(&oc) < valid_one_lump(&o));
}

#[test]
fn joined_mirror_keeps_holes_of_both_halves() {
    let pre = [l_bracket(), vec![circle_sketch("S2", 60.0, 30.0, 4.0), cut("S2")]].concat();
    let base = part(&pre);
    ok(&base);
    let end = role_face(&base, "P0.side.e2");
    let o = part(&with(pre, mirror("M1", &face_plane(&end), false)));
    ok(&o);
    rel(valid_one_lump(&o), 2.0 * valid_one_lump(&base), 1e-9, "joined volume");
    assert_eq!(loops(&o).len(), 3, "outline and two holes");
}

#[test]
fn a_mirror_that_cannot_be_one_sheet_is_refused_with_the_reason() {
    let base = part(&l_bracket());
    ok(&base);
    let v0 = valid_one_lump(&base);
    // across a face of the plate: the halves would touch over the whole face
    let face = role_face(&base, "P0.front");
    let o = part(&with(l_bracket(), mirror("M1", &face_plane(&face), false)));
    assert_eq!(o.errors, vec![("M1".to_string(), "sheetMetalUnsupported".to_string())]);
    assert!(detail(&o).contains("face of the sheet"), "{}", detail(&o));
    // the part is still the sheet it was
    close(valid_one_lump(&o), v0, "unchanged");
    // across the outer face of the flange: the flange and its image would fuse to a double wall
    let wall = role_face(&base, "P1.back");
    let o = part(&with(l_bracket(), mirror("M1", &face_plane(&wall), false)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].1, "sheetMetalUnsupported");
    // the same planes are fine with `replace`
    for plane in [face_plane(&face), face_plane(&wall)] {
        let o = part(&with(l_bracket(), mirror("M1", &plane, true)));
        ok(&o);
        close(valid_one_lump(&o), v0, "replace keeps the volume");
    }
}

#[test]
fn a_plane_above_the_sheet_or_across_it_is_refused() {
    // a plate with a flange at each end: both ends carry a bend, so no end of the plate is a free seam
    let two = [flange("E1", &[(0.0, 30.0, 2.0)], r#""height":15"#), flange("E2", &[(100.0, 30.0, 2.0)], r#""height":15"#)];
    let base = part(&two);
    ok(&base);
    // the tip of the second flange is parallel to the plate, above it: the image lies over the sheet, not along an edge
    let tip = role_face(&base, "P2.tip");
    let o = part(&with(two.to_vec(), mirror("M1", &face_plane(&tip), false)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].1, "sheetMetalUnsupported");
    assert!(detail(&o).contains("parallel"), "{}", detail(&o));
    // the end face of the plate that carries a flange is not a free seam either
    let end = role_face(&base, "P1.back");
    let o = part(&with(two.to_vec(), mirror("M1", &face_plane(&end), false)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert_eq!(o.errors[0].1, "sheetMetalUnsupported");
    assert!(detail(&o).contains("cannot be joined"), "{}", detail(&o));
}

#[test]
fn a_flange_that_runs_to_the_seam_makes_the_halves_fuse_along_more_than_a_seam() {
    // the flange on the long edge y = 0 runs to the end x = 100: the two flanges would touch end to end, so the halves meet in
    // more than the end of the plate and the tree cannot say it is one sheet
    let one = [flange("E1", &[(50.0, 0.0, 2.0)], r#""height":15"#)];
    let base = part(&one);
    ok(&base);
    let end = role_face(&base, "P0.side.e2");
    let o = part(&with(one.to_vec(), mirror("M1", &face_plane(&end), false)));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    assert!(detail(&o).contains("more than"), "{}", detail(&o));
    // as an image on its own (replace) it is fine
    let o = part(&with(one.to_vec(), mirror("M1", &face_plane(&end), true)));
    ok(&o);
    close(valid_one_lump(&o), valid_one_lump(&base), "replace");
}

#[test]
fn replace_on_a_plain_solid_moves_the_body_to_the_other_side() {
    let fs = [rect(10.0, 10.0), r#"{"type":"extrude","id":"F2","sketch":"S1","end":"blind","depth":5}"#.to_string()];
    let plain = run(&fs);
    ok(&plain);
    let v = ck_topo::mass::mass_props(plain.body()).volume;
    let both = run(&[fs.to_vec(), vec![mirror("M1", &std_plane("right"), false)]].concat());
    ok(&both);
    let swapped = run(&[fs.to_vec(), vec![mirror("M1", &std_plane("right"), true)]].concat());
    ok(&swapped);
    close(ck_topo::mass::mass_props(swapped.body()).volume, v, "replace keeps one copy");
    assert!(ck_topo::mass::mass_props(both.body()).volume > v - 1e-9);
    let (lo, hi) = ck_model::features::body_box(swapped.body());
    let (lo0, hi0) = ck_model::features::body_box(plain.body());
    close(lo.x, -hi0.x, "x min");
    close(hi.x, -lo0.x, "x max");
}
