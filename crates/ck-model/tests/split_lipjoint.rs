//! `split` (a plane cuts a body into two) and `lipJoint` (a centred tongue on one half, a groove wider and deeper by the
//! clearance on the other), and the `inset` / `clearance` fields of `lip`.
//!
//! Fixture: a 40 x 20 x 10 box (on the front plane: x 0..40, y 0..20, z 0..10) shelled open at the top with 2 mm walls, cut at mid height (z = 5).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }
    fn full(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn body(&self) -> &ck_topo::body::Body {
        self.body.as_ref().unwrap()
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.body()).volume
    }
    /// Volume of the lump with this name.
    fn named(&self, name: &str) -> f64 {
        let b = self.body();
        let l = b.lump_ids().find(|&l| b.lump(l).name.as_deref() == Some(name)).unwrap_or_else(|| panic!("no body {name}: {:?}", self.names()));
        ck_topo::mass::mass_props(&ck_ops::interference::extract_lump(b, l)).volume
    }
    fn names(&self) -> Vec<Option<String>> {
        let b = self.body();
        b.lump_ids().map(|l| b.lump(l).name.clone()).collect()
    }
    fn lumps(&self) -> usize {
        self.body.as_ref().map_or(0, |b| ck_topo::region::components(b).len())
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { result, body: ex.brep.body.clone() }
}

const SKETCH: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},
  "points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},
              {"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]}"#;
const EXTRUDE: &str = r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
const SHELL: &str = r#"{"type":"shell","id":"F3","faces":["F2.C1"],"thickness":2}"#;

fn case(tail: &str) -> Run {
    run(&format!("{SKETCH},{EXTRUDE},{SHELL}{}", if tail.is_empty() { String::new() } else { format!(",{tail}") }))
}

fn joint(extra: &str) -> String {
    format!(r#"{{"type":"lipJoint","id":"F4","plane":{{"kind":"std","id":"front"}},"offset":5,"height":1.5,"width":1,"clearance":0.2,"names":["Base","Lid"]{extra}}}"#)
}

/// Area of the band between two inward offsets `d1 < d2` of the 40 x 20 outline.
fn band(d1: f64, d2: f64) -> f64 {
    (40.0 - 2.0 * d1) * (20.0 - 2.0 * d1) - (40.0 - 2.0 * d2) * (20.0 - 2.0 * d2)
}

fn valid(r: &Run) {
    let issues = ck_check::validate(r.body(), 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
}

#[test]
fn split_cuts_the_shelled_box_into_two_named_bodies() {
    let whole = case("");
    assert!(whole.errors().is_empty(), "{:?}", whole.errors());
    let v0 = whole.volume();
    let r = case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":5,"names":["Base","Lid"]}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.lumps(), 2);
    valid(&r);
    // the wall ring above and the floor + wall cup below: the volumes add up to the shelled box
    assert!((r.named("Base") + r.named("Lid") - v0).abs() < 1e-6, "{} {} {v0}", r.named("Base"), r.named("Lid"));
    // the lid is the 5 mm high wall ring of 2 mm: 5 * band(0, 2)
    assert!((r.named("Lid") - 5.0 * band(0.0, 2.0)).abs() < 1e-6, "{}", r.named("Lid"));
}

#[test]
fn split_keep_and_default_names() {
    let r = case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":5,"keep":"above"}"#);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.lumps(), 1);
    assert!((r.volume() - 5.0 * band(0.0, 2.0)).abs() < 1e-6);
    // flip: the normal reversed, so "below" is the upper part
    let f = case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":-5,"flip":true,"names":["Up","Down"]}"#);
    assert!(f.errors().is_empty(), "{:?}", f.errors());
    assert!((f.named("Up") - 5.0 * band(0.0, 2.0)).abs() < 1e-6, "{}", f.named("Up"));
}

#[test]
fn split_errors() {
    assert_eq!(case(r#"{"type":"split","id":"F4","offset":5}"#).errors(), vec!["F4:invalidValue"]);
    // a plane that misses the body
    assert_eq!(case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":50}"#).errors(), vec!["F4:cutNoEffect"]);
    let none = run(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"}}"#);
    assert_eq!(none.errors(), vec!["F4:nothingToCut"]);
}

#[test]
fn lip_joint_tongue_fits_the_groove_with_the_clearance() {
    let whole = case("");
    let v0 = whole.volume();
    let r = case(&joint(""));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.lumps(), 2);
    valid(&r);
    // wall 2, width 1, centred: inset 0.5. tongue: height 1.5 on band(0.5, 1.5)
    let tongue = 1.5 * band(0.5, 1.5);
    // groove: 0.2 wider on each side (band 0.3..1.7) and 0.2 deeper
    let groove = 1.7 * band(0.3, 1.7);
    let (base0, lid0) = (5.0 * band(0.0, 2.0), 5.0 * band(0.0, 2.0));
    let _ = lid0;
    let whole_split = case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":5,"names":["Base","Lid"]}"#);
    let (base, lid) = (whole_split.named("Base"), whole_split.named("Lid"));
    assert!((r.named("Base") - (base + tongue)).abs() < 1e-6, "tongue {} vs {}", r.named("Base") - base, tongue);
    assert!((r.named("Lid") - (lid - groove)).abs() < 1e-6, "groove {} vs {}", lid - r.named("Lid"), groove);
    assert!((r.volume() - (v0 + tongue - groove)).abs() < 1e-6, "{} {}", r.volume(), v0 + tongue - groove);
    let _ = base0;
    // the void left around the tongue is groove - tongue; it is what the clearance makes
    let clearance_void = groove - tongue;
    assert!((clearance_void - ((1.7 * band(0.3, 1.7)) - (1.5 * band(0.5, 1.5)))).abs() < 1e-9);
    // the halves do not interfere
    let ifr = ck_ops::interference::reply(r.body());
    let pairs = ifr.get("pairs").and_then(|p| p.as_arr()).map_or(0, |p| p.len());
    assert_eq!(pairs, 0, "{ifr:?}");
    // the tongue stands 1.5 above the cut plane (z = 6.5), the groove floor lies 1.7 above it (z = 6.7)
    let b = r.body();
    let heights: Vec<f64> = b
        .face_ids()
        .filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }) && b.planar_normal(f).z.abs() > 0.99)
        .filter_map(|f| b.face_edges(f).first().map(|&e| b.vertex(b.edge(e).v0).p.z))
        .collect();
    assert!(heights.iter().any(|z| (z - 6.5).abs() < 1e-6), "{heights:?}");
    assert!(heights.iter().any(|z| (z - 6.7).abs() < 1e-6), "{heights:?}");
}

#[test]
fn lip_joint_tongue_on_the_upper_half_and_width_ratio() {
    let r = case(&joint(r#","tongue":"above""#).replace(r#""width":1,"#, r#""widthRatio":0.5,"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    valid(&r);
    let base = 5.0 * band(0.0, 2.0);
    let whole = case(r#"{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":5,"names":["Base","Lid"]}"#);
    let (b0, l0) = (whole.named("Base"), whole.named("Lid"));
    assert!((l0 - base).abs() < 1e-6);
    assert!((r.named("Lid") - (l0 + 1.5 * band(0.5, 1.5))).abs() < 1e-6);
    assert!((r.named("Base") - (b0 - 1.7 * band(0.3, 1.7))).abs() < 1e-6);
}

#[test]
fn lip_joint_errors() {
    // clearance larger than the inset
    let c = case(&joint(r#","inset":0.1,"clearance":0.5"#));
    assert_eq!(c.errors(), vec!["F4:invalidValue"]);
    // the tongue does not fit the wall
    let w = case(&joint("").replace(r#""width":1,"#, r#""width":1.9,"#));
    assert_eq!(w.errors(), vec!["F4:invalidValue"]);
    let h = case(&joint("").replace(r#""height":1.5"#, r#""height":0"#));
    assert_eq!(h.errors(), vec!["F4:invalidValue"]);
    // a solid body has no inner wall to measure: width and inset in mm are needed
    let solid = run(&format!("{SKETCH},{EXTRUDE},{}", joint("").replace(r#""width":1,"#, "")));
    assert_eq!(solid.errors(), vec!["F4:invalidValue"]);
}

#[test]
fn lip_inset_and_clearance_fields() {
    // a tongue on the rim of the shelled box, 0.5 mm in from the outer wall, without any split
    let rim = r#"{"type":"lip","id":"F4","kind":"lip","faces":["F2.C1"],"edges":[
      {"faces":["F2.C1","F2.S.e1~1"],"at":[20,0,10]},{"faces":["F2.C1","F2.S.e2~1"],"at":[40,10,10]},
      {"faces":["F2.C1","F2.S.e3~1"],"at":[20,20,10]},{"faces":["F2.C1","F2.S.e4~1"],"at":[0,10,10]}],"width":1,"height":1.5,"inset":0.5}"#;
    let whole = case("");
    let r = case(rim);
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    valid(&r);
    assert!((r.volume() - whole.volume() - 1.5 * band(0.5, 1.5)).abs() < 1e-6, "{}", r.volume() - whole.volume());
    // default (no inset): unchanged flush band
    let flush = case(&rim.replace(r#","inset":0.5"#, ""));
    assert!(flush.errors().is_empty(), "{:?}", flush.errors());
    assert!((flush.volume() - whole.volume() - 1.5 * band(0.0, 1.0)).abs() < 1e-6);
    // the groove of the same size with a clearance: 0.2 wider each side, 0.2 deeper
    let groove = rim.replace(r#""kind":"lip""#, r#""kind":"groove""#).replace(r#""inset":0.5"#, r#""inset":0.5,"clearance":0.2"#);
    let g = case(&groove);
    assert!(g.errors().is_empty(), "{:?}", g.errors());
    assert!((whole.volume() - g.volume() - 1.7 * band(0.3, 1.7)).abs() < 1e-6, "{}", whole.volume() - g.volume());
}

#[test]
fn closed_case_two_cups_with_a_joint() {
    // the hollow box (an empty open-face list closes the cavity), cut at mid height: a lower and an upper cup
    let closed = |tail: &str| run(&format!("{SKETCH},{EXTRUDE},{{\"type\":\"shell\",\"id\":\"F3\",\"faces\":[],\"thickness\":2}}{tail}"));
    let whole = closed("");
    assert!(whole.errors().is_empty(), "{:?}", whole.errors());
    let v0 = 40.0 * 20.0 * 10.0 - 36.0 * 16.0 * 6.0;
    assert!((whole.volume() - v0).abs() < 1e-6, "{}", whole.volume());
    let r = closed(&format!(",{}", joint("")));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    assert_eq!(r.lumps(), 2);
    valid(&r);
    let (tongue, groove) = (1.5 * band(0.5, 1.5), 1.7 * band(0.3, 1.7));
    assert!((r.volume() - (v0 + tongue - groove)).abs() < 1e-6, "{} vs {}", r.volume(), v0 + tongue - groove);
    // the two cups are the halves of a hollow case: each one is a closed-bottom cup, the lid's top is 2 mm thick
    let split = closed(r#",{"type":"split","id":"F4","plane":{"kind":"std","id":"front"},"offset":5,"names":["Base","Lid"]}"#);
    assert!((split.named("Base") - split.named("Lid")).abs() < 1e-6);
    assert!((r.named("Base") - (split.named("Base") + tongue)).abs() < 1e-6);
    assert!((r.named("Lid") - (split.named("Lid") - groove)).abs() < 1e-6);
}

// ---- names of the two halves (both halves share one origin feature: the box that was cut) ----

fn split_named(extra: &str) -> String {
    format!(r#"{{"type":"split","id":"F4","plane":{{"kind":"std","id":"front"}},"offset":5{extra}}}"#)
}

fn sorted_names(r: &Run) -> Vec<String> {
    let mut v: Vec<String> = r.names().into_iter().map(|n| n.unwrap_or_else(|| "-".into())).collect();
    v.sort();
    v
}

/// A cut box through the left wall: `z` is its centre height (the lid's wall ring is z 5..10, the base's z 0..5).
fn wall_cut(id: &str, z: f64, target: &str) -> String {
    format!(r#"{{"type":"box","id":"{id}","op":"cut","center":[0,10,{z}],"size":[6,6,2]{target}}}"#)
}

#[test]
fn halves_carry_the_given_names_after_split_and_lip_joint() {
    for f in [split_named(r#","names":["Base","Lid"]"#), joint("")] {
        let r = case(&f);
        assert!(r.errors().is_empty(), "{:?}", r.full());
        assert_eq!(sorted_names(&r), vec!["Base", "Lid"], "{f}");
        let g = case(&f.replace("Base", "Bottom").replace("Lid", "Top"));
        assert_eq!(sorted_names(&g), vec!["Bottom", "Top"]);
    }
}

#[test]
fn halves_keep_their_names_through_later_features() {
    for f in [split_named(r#","names":["Base","Lid"]"#), joint("")] {
        let whole = case(&f);
        let (base, lid) = (whole.named("Base"), whole.named("Lid"));
        // a cut through the base wall, one through the lid wall, a fillet-free hole in the floor
        let a = case(&format!("{f},{}", wall_cut("F5", 2.5, "")));
        assert!(a.errors().is_empty(), "{:?}", a.full());
        assert_eq!(sorted_names(&a), vec!["Base", "Lid"], "{f}");
        assert!(a.named("Base") < base - 1e-6 && (a.named("Lid") - lid).abs() < 1e-6);
        let b = case(&format!("{f},{}", wall_cut("F5", 7.5, "")));
        assert!(b.errors().is_empty(), "{:?}", b.full());
        assert_eq!(sorted_names(&b), vec!["Base", "Lid"], "{f}");
        assert!(b.named("Lid") < lid - 1e-6 && (b.named("Base") - base).abs() < 1e-6, "{f}");
        // two features in a row
        let c = case(&format!("{f},{},{}", wall_cut("F5", 2.5, ""), wall_cut("F6", 7.5, "")));
        assert_eq!(sorted_names(&c), vec!["Base", "Lid"], "{f}");
    }
}

#[test]
fn a_cut_aimed_at_one_half_by_name() {
    for f in [split_named(r#","names":["Base","Lid"]"#), joint("")] {
        let whole = case(&f);
        let (base, lid) = (whole.named("Base"), whole.named("Lid"));
        let r = case(&format!("{f},{}", wall_cut("F5", 7.5, r#","target":{"body":"Lid"}"#)));
        assert!(r.errors().is_empty(), "{f}: {:?}", r.full());
        assert_eq!(sorted_names(&r), vec!["Base", "Lid"], "{f}");
        assert!(r.named("Lid") < lid - 1e-6 && (r.named("Base") - base).abs() < 1e-6, "{f}");
    }
}

#[test]
fn halves_default_names_without_names() {
    // no `names`: the two halves are unnamed bodies (the kit calls them Case 1 and Case 2 itself)
    let r = case(&split_named(""));
    assert!(r.errors().is_empty(), "{:?}", r.full());
    assert_eq!(r.lumps(), 2);
    println!("default names: {:?}", r.names());
}


#[test]
fn names_survive_a_save_and_reload() {
    // the names live in the feature (`names`), so a part written out and read back names the halves again
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{SKETCH},{EXTRUDE},{SHELL},{},{}]}}"#, joint(""), wall_cut("F5", 2.5, ""));
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let again = Part::from_json(&ck_json::parse(&part.to_json().to_string()).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&again, &RegenOpts::default(), &mut ex);
    let r = Run { result, body: ex.brep.body.clone() };
    assert!(r.errors().is_empty(), "{:?}", r.full());
    assert_eq!(sorted_names(&r), vec!["Base", "Lid"]);
}

/// The same notch as a sketch and an extrude cut (what the kit writes): `depth` up from the bottom plane (a cut goes against the sketch normal unless `flip`).
fn notch(id: &str, depth: f64, target: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}s","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":-1,"y":9}},{{"id":"b","x":3,"y":9}},{{"id":"c","x":3,"y":16}},{{"id":"d","x":-1,"y":16}}],
"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}},
{{"type":"extrude","id":"{id}","op":"cut","sketch":"{id}s","end":"blind","depth":{depth},"flip":true{target}}}"#
    )
}

#[test]
fn extrude_cut_notches_keep_the_halves_apart_and_named() {
    for f in [split_named(r#","names":["Base","Lid"]"#), joint("")] {
        let whole = case(&f);
        let (base, lid) = (whole.named("Base"), whole.named("Lid"));
        // below the cut plane: only the base is met
        let a = case(&format!("{f},{}", notch("F5", 4.0, "")));
        assert!(a.errors().is_empty(), "{:?}", a.full());
        assert_eq!(sorted_names(&a), vec!["Base", "Lid"], "{f}");
        assert!(a.named("Base") < base - 1e-6 && (a.named("Lid") - lid).abs() < 1e-6, "{f}");
        // through everything, aimed at the lid
        let b = case(&format!("{f},{}", notch("F5", 10.0, r#","target":{"body":"Lid"}"#)));
        assert!(b.errors().is_empty(), "{:?}", b.full());
        assert_eq!(sorted_names(&b), vec!["Base", "Lid"], "{f}");
        assert!(b.named("Lid") < lid - 1e-6 && (b.named("Base") - base).abs() < 1e-6, "{f}");
        // a cut of the base first, then one aimed at the lid
        let c = case(&format!("{f},{},{}", notch("F5", 4.0, ""), notch("F7", 10.0, r#","target":{"body":"Lid"}"#)));
        assert!(c.errors().is_empty(), "{:?}", c.full());
        assert_eq!(sorted_names(&c), vec!["Base", "Lid"], "{f}");
        assert_eq!(c.lumps(), 2, "{f}");
        assert!(c.named("Lid") < lid - 1e-6 && c.named("Base") < base - 1e-6, "{f}");
    }
}
